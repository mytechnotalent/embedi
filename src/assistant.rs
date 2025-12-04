/*
 * @file assistant.rs
 * @brief Implementation of Embedi's voice-assistant runtime
 * @author Kevin Thomas
 * @date 2025
 *
 * MIT License
 *
 * Copyright (c) 2025 Kevin Thomas
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//! Voice assistant orchestration module.

use crate::audio::{record_audio, save_wav};
use crate::commands;
use crate::speech::speak;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;
use std::{
    env, fs,
    path::Path,
    time::Duration,
};

/// Temporary file used for passing audio samples to Whisper.
///
/// The file lives only for the duration of a single loop iteration
/// and is removed automatically by [`TempAudioGuard`].
const TEMP_AUDIO_PATH: &str = "temp.wav";

/// Delay inserted before each recording so the listener has time to prepare.
///
/// Keeping the delay configurable makes it easy to tweak responsiveness
/// without touching the main loop.
const PRE_RECORD_DELAY: Duration = Duration::from_millis(200);

/// Minimum RMS amplitude considered speech.
///
/// Values much above ~300 miss normal speaking levels on some microphones, so
/// we bias toward a lower threshold and rely on Whisper to filter background noise.
const SILENCE_RMS_THRESHOLD: f32 = 150.0;

/// Path to the JSON configuration file that holds runtime defaults.
const CONFIG_PATH: &str = "config.json";

/// Default serial device Embedi uses to reach the Raspberry Pi Pico when no config exists.
const FALLBACK_SERIAL_PORT: &str = "/dev/cu.usbmodem21402";

/// Persistent memory log that accumulates every exchange Embedi hears.
const MEMORY_PATH: &str = "memory.json";

/// Default baud rate used for Pico UART commands.
const DEFAULT_SERIAL_BAUD: u32 = 115_200;

/// Ollama API endpoint for local LLM inference.
const OLLAMA_API_URL: &str = "http://localhost:11434/api/chat";

/// Whisper model path (will be auto-downloaded if not present).
const WHISPER_MODEL_PATH: &str = "models/ggml-base.en.bin";

/// Serial write timeout to avoid blocking the microphone loop.
const SERIAL_TIMEOUT: Duration = Duration::from_millis(100);

/// Time to let the Pico reboot after the UART is opened (DTR toggle).
const SERIAL_BOOT_DELAY: Duration = Duration::from_millis(150);

/// Minimum spacing between consecutive UART commands.
const SERIAL_COMMAND_DELAY: Duration = Duration::from_millis(30);



/// Strongly typed representation of `config.json`.
#[derive(Clone, Deserialize)]
struct AppConfig {
    #[serde(default = "fallback_serial_port")]
    default_serial_port: String,
    #[serde(default = "fallback_ollama_model")]
    default_ollama_model: String,
}

/// Returns the fallback serial port path.
///
/// # Details
/// Provides default configuration values when config.json is missing or invalid.
impl Default for AppConfig {
    fn default() -> Self {
        Self {
            default_serial_port: fallback_serial_port(),
            default_ollama_model: fallback_ollama_model(),
        }
    }
}

/// Runs the fully hands-free voice assistant loop until the user says quits.
///
/// The loop records microphone input, transcribes it with Groq Whisper,
/// fetches a response from Llama 3.3 70B, and finally speaks the reply using
/// the macOS `say` command.
///
/// # Returns
/// `Ok(())` when the user issues a quit command or the loop exits naturally.
///
/// # Errors
/// Returns an error only if Groq initialization fails before the loop starts.
pub async fn run_voice_assistant() -> Result<()> {
    VoiceAssistantRuntime::new().await?.run_loop().await
}

/// Cleans up temporary files created during operation.
///
/// This function is safe to call multiple times and will not fail if files don't exist.
pub fn cleanup_temp_files() {
    cleanup_temp_file();
}

/// Runtime container that owns the HTTP client and conversation state.
///
/// # Details
/// Maintains the Ollama client, Whisper context, chat history, serial port
/// connection, and memory log across the entire voice assistant session.
struct VoiceAssistantRuntime {
    client: reqwest::Client,
    ollama_model: String,
    whisper_ctx: Option<whisper_rs::WhisperContext>,
    history: Vec<ChatMessage>,
    serial: Option<Box<dyn SerialPort>>,
    serial_path: String,
    serial_baud: u32,
    memory: Vec<MemoryEntry>,
    commands: commands::CommandsConfig,
    device_context_active: bool,
}

/// Chat message structure for Ollama API.
///
/// # Details
/// Represents a single message in the conversation history, containing
/// the role (user, assistant, or system) and the message content.
#[derive(Clone, Serialize, Deserialize)]
struct ChatMessage {
    role: String,
    content: String,
}

/// Request structure for Ollama API.
///
/// # Details
/// Encapsulates the model name, conversation history, and streaming preference
/// for Ollama chat completion requests.
#[derive(Serialize)]
struct OllamaRequest {
    model: String,
    messages: Vec<ChatMessage>,
    stream: bool,
}

/// Response structure for Ollama API.
///
/// # Details
/// Contains the assistant's generated message returned by Ollama's
/// chat completion endpoint.
#[derive(Deserialize)]
struct OllamaResponse {
    message: ChatMessage,
}

/// Represents a single memory entry in persistent storage.
/// Implementation of core voice assistant runtime methods.
///
/// # Details
/// Provides methods for initializing the runtime, managing the conversation loop,
/// transcribing audio, querying the LLM, and controlling connected devices.
impl VoiceAssistantRuntime {
    /// Creates a new runtime by loading configuration.
    ///
    /// # Returns
    /// A ready-to-run [`VoiceAssistantRuntime`] with an empty conversation history.
    ///
    /// # Errors
    /// Propagates failures from initialization.
    async fn new() -> Result<Self> {
        // Ensure Ollama is ready before proceeding
        Self::ensure_ollama_ready().await?;
        let memory_entries = Self::load_memory_with_fallback();
        let history = Self::build_chat_history(&memory_entries);
        let config = load_app_config();
        let commands_config = commands::load_commands();
        let ollama_model =
            env::var("OLLAMA_MODEL").unwrap_or_else(|_| config.default_ollama_model.clone());
        Ok(Self {
            client: reqwest::Client::new(),
            ollama_model,
            whisper_ctx: None,
            history,
            serial: None,
            serial_path: serial_port_path(&config),
            serial_baud: serial_baud_rate(),
            memory: memory_entries,
            commands: commands_config,
            device_context_active: false,
        })
    }

    /// Loads memory entries from disk with error handling and fallback.
    ///
    /// # Details
    /// Attempts to load the conversation history from the persistent memory file.
    /// If loading fails (file doesn't exist, is corrupted, etc.), logs the error
    /// and returns an empty vector to allow the application to start fresh.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Vec<MemoryEntry>` - Loaded memory entries, or empty vector on failure.
    fn load_memory_with_fallback() -> Vec<MemoryEntry> {
        match load_memory_entries() {
            Ok(entries) => entries,
            Err(err) => {
                eprintln!("Memory load error: {}", err);
                Vec::new()
            }
        }
    }

    /// Converts memory entries into chat messages for LLM context.
    ///
    /// # Details
    /// Transforms the persistent memory format into the chat message structure
    /// required by the Ollama API. This preserves conversation history across
    /// application restarts.
    ///
    /// # Arguments
    /// * `memory_entries` - The memory entries to convert.
    ///
    /// # Returns
    /// * `Vec<ChatMessage>` - Formatted chat messages ready for LLM consumption.
    fn build_chat_history(memory_entries: &[MemoryEntry]) -> Vec<ChatMessage> {
        memory_entries
            .iter()
            .map(|entry| ChatMessage {
                role: match entry.role {
                    MemoryRole::User => "user".to_string(),
                    MemoryRole::Assistant => "assistant".to_string(),
                },
                content: entry.text.clone(),
            })
            .collect()
    }

    /// Ensures Ollama service is running and required model is available.
    ///
    /// # Details
    /// Checks if Ollama is responding on localhost:11434. If not, attempts to start
    /// the service. Then verifies the required model is available and downloads it
    /// if missing. This makes the application self-sufficient and easier for others
    /// to run without manual setup.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(())` - Ollama is running and the model is available.
    ///
    /// # Errors
    /// Returns an error if Ollama cannot be started or the model cannot be downloaded.
    async fn ensure_ollama_ready() -> Result<()> {
        let config = load_app_config();
        let model =
            env::var("OLLAMA_MODEL").unwrap_or_else(|_| config.default_ollama_model.clone());
        Self::ensure_ollama_service().await?;
        Self::ensure_ollama_model(&model).await?;
        Ok(())
    }

    /// Ensures the Ollama service is running, starting it if necessary.
    ///
    /// # Details
    /// Checks if Ollama is responding on localhost:11434. If not, attempts to start
    /// the service and waits for it to initialize. This allows the application to
    /// automatically recover from a stopped Ollama service without user intervention.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(())` - Ollama service is running and responding.
    ///
    /// # Errors
    /// Returns an error if Ollama cannot be started or is not installed.
    async fn ensure_ollama_service() -> Result<()> {
        eprintln!("Checking Ollama service...");
        if !Self::is_ollama_running().await {
            eprintln!("Ollama not running, attempting to start...");
            Self::start_ollama_service()?;
            tokio::time::sleep(Duration::from_secs(3)).await;
            if !Self::is_ollama_running().await {
                anyhow::bail!(
                    "Failed to start Ollama service. Please install Ollama from https://ollama.ai"
                );
            }
        }
        eprintln!("✓ Ollama service is running");
        Ok(())
    }

    /// Ensures the specified Ollama model is available, downloading if necessary.
    ///
    /// # Details
    /// Queries the Ollama API to check if the model is already installed. If not,
    /// initiates a download using the Ollama CLI. The download may take several
    /// minutes depending on model size and network speed.
    ///
    /// # Arguments
    /// * `model` - The name of the model to ensure is available (e.g., "llama3.2:3b").
    ///
    /// # Returns
    /// * `Ok(())` - Model is available or was successfully downloaded.
    ///
    /// # Errors
    /// Returns an error if the model query fails or download is unsuccessful.
    async fn ensure_ollama_model(model: &str) -> Result<()> {
        eprintln!("Checking for model {}...", model);
        if !Self::has_ollama_model(model).await? {
            eprintln!("Model {} not found, downloading...", model);
            eprintln!("This may take several minutes depending on your connection.");
            Self::pull_ollama_model(model)?;
        }
        eprintln!("✓ Model {} is ready", model);
        Ok(())
    }

    /// Checks if Ollama service is responding.
    ///
    /// # Details
    /// Attempts a simple HTTP GET to the Ollama API tags endpoint with a short timeout.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `bool` - `true` if Ollama responds, `false` otherwise.
    async fn is_ollama_running() -> bool {
        let client = reqwest::Client::builder()
            .timeout(Duration::from_secs(2))
            .build()
            .unwrap();
        client.get("http://localhost:11434/api/tags").send().await.is_ok()
    }

    /// Attempts to start the Ollama service in the background.
    ///
    /// # Details
    /// First kills any existing Ollama processes to prevent multiple instances,
    /// then spawns `ollama serve` as a detached background process.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(())` - Command was spawned successfully.
    ///
    /// # Errors
    /// Returns an error if the ollama executable cannot be found or spawned.
    fn start_ollama_service() -> Result<()> {
        let _ = std::process::Command::new("pkill")
            .arg("-9")
            .arg("ollama")
            .output();
        std::thread::sleep(Duration::from_millis(500));
        std::process::Command::new("ollama")
            .arg("serve")
            .stdin(std::process::Stdio::null())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .with_context(|| "Failed to start Ollama. Please install from https://ollama.ai")?;
        Ok(())
    }

    /// Checks if a specific model is available in Ollama.
    ///
    /// # Details
    /// Queries the Ollama API for the list of installed models and checks
    /// if the specified model is present.
    ///
    /// # Arguments
    /// * `model_name` - The name of the model to check (e.g., "llama3.2:3b").
    ///
    /// # Returns
    /// * `Ok(true)` - Model is available.
    /// * `Ok(false)` - Model is not available.
    ///
    /// # Errors
    /// Returns an error if the API request fails.
    async fn has_ollama_model(model_name: &str) -> Result<bool> {
        let client = reqwest::Client::new();
        let response = client
            .get("http://localhost:11434/api/tags")
            .send()
            .await
            .with_context(|| "Failed to query Ollama models")?;
        let json: serde_json::Value = response
            .json()
            .await
            .with_context(|| "Failed to parse Ollama response")?;
        if let Some(models) = json["models"].as_array() {
            for model in models {
                if let Some(name) = model["name"].as_str() {
                    if name == model_name {
                        return Ok(true);
                    }
                }
            }
        }
        Ok(false)
    }

    /// Downloads a model using the Ollama CLI.
    ///
    /// # Details
    /// Executes `ollama pull <model>` and waits for completion. Output is shown
    /// to the user so they can track download progress.
    ///
    /// # Arguments
    /// * `model_name` - The name of the model to download.
    ///
    /// # Returns
    /// * `Ok(())` - Model was downloaded successfully.
    ///
    /// # Errors
    /// Returns an error if the pull command fails or times out.
    fn pull_ollama_model(model_name: &str) -> Result<()> {
        let status = std::process::Command::new("ollama")
            .arg("pull")
            .arg(model_name)
            .status()
            .with_context(|| "Failed to execute ollama pull")?;
        if !status.success() {
            anyhow::bail!("Failed to download model {}", model_name);
        }
        Ok(())
    }

    /// Continuously runs the assistant until a quit phrase is detected.
    ///
    /// # Returns
    /// `Ok(())` once the user exits.
    ///
    /// # Errors
    /// Bubbles up any fatal errors from the underlying processing pipeline.
    async fn run_loop(mut self) -> Result<()> {
        while self.process_iteration().await? {}
        Ok(())
    }

    /// Executes one listen-transcribe-respond iteration.
    ///
    /// # Returns
    /// * `Ok(true)` to keep looping, `Ok(false)` to exit gracefully.
    ///
    /// # Errors
    /// Surfaces fatal issues (e.g., Groq initialization errors).
    async fn process_iteration(&mut self) -> Result<bool> {
        announce_listening();
        let samples = match Self::capture_samples()? {
            Some(data) => data,
            None => return Ok(true),
        };
        if !Self::contains_speech(&samples) {
            return Ok(true);
        }
        if !Self::persist_samples(&samples) {
            return Ok(true);
        }
        let _guard = TempAudioGuard::new();
        match self.transcribe_current_audio().await {
            Some(text) => self.handle_user_text(text).await,
            None => Ok(true),
        }
    }

    /// Captures microphone input and reports recoverable errors.
    ///
    /// # Returns
    /// * `Ok(Some(samples))` when recording succeeds.
    /// * `Ok(None)` when recording fails but the loop should continue.
    /// * `Err` for unrecoverable setup issues.
    fn capture_samples() -> Result<Option<Vec<i16>>> {
        match record_audio() {
            Ok(data) => Ok(Some(data)),
            Err(err) => {
                eprintln!("❌ Microphone error: {}", err);
                Ok(None)
            }
        }
    }

    /// Persists captured samples to a temporary WAV file.
    ///
    /// # Parameters
    /// * `samples` - The PCM payload to save.
    ///
    /// # Returns
    /// `true` when writing succeeds; `false` when the caller should retry.
    fn persist_samples(samples: &[i16]) -> bool {
        match save_wav(TEMP_AUDIO_PATH, samples) {
            Ok(_) => true,
            Err(err) => {
                eprintln!("Save error: {}", err);
                false
            }
        }
    }

    /// Detects whether audio samples contain meaningful speech content.
    ///
    /// # Details
    /// Calculates the root mean square (RMS) energy of the audio signal and compares
    /// it against a threshold to determine if speech is present. This prevents the
    /// assistant from processing silent or near-silent recordings.
    ///
    /// # Arguments
    /// * `samples` - PCM audio samples as signed 16-bit integers.
    ///
    /// # Returns
    /// * `bool` - `true` when the RMS energy exceeds the silence threshold, `false` otherwise.
    fn contains_speech(samples: &[i16]) -> bool {
        if samples.is_empty() {
            return false;
        }
        let energy = samples
            .iter()
            .map(|sample| (*sample as f32).powi(2))
            .sum::<f32>()
            / samples.len() as f32;
        energy.sqrt() >= SILENCE_RMS_THRESHOLD
    }

    /// Transcribes audio from the temporary WAV file using local Whisper.
    ///
    /// # Details
    /// Initializes the Whisper context if needed, loads and preprocesses the audio,
    /// configures transcription parameters for English speech, and extracts the
    /// transcribed text from all segments. Errors are logged but not propagated
    /// to allow the main loop to continue.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Some(String)` - The transcribed text when successful.
    /// * `None` - When initialization, loading, or transcription fails.
    async fn transcribe_current_audio(&mut self) -> Option<String> {
        self.ensure_whisper_context()?;
        let ctx = self.whisper_ctx.as_ref()?;
        let audio_data = Self::load_audio_data()?;
        let text = Self::run_whisper_inference(ctx, &audio_data)?;
        Some(text)
    }

    /// Ensures the Whisper context is initialized, creating it if necessary.
    ///
    /// # Details
    /// Checks if the Whisper context exists and initializes it on first use.
    /// Logs initialization errors and returns None to allow graceful degradation.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Some(())` - Context is ready for use.
    /// * `None` - Initialization failed and error was logged.
    fn ensure_whisper_context(&mut self) -> Option<()> {
        if self.whisper_ctx.is_none() {
            match Self::init_whisper() {
                Ok(ctx) => self.whisper_ctx = Some(ctx),
                Err(err) => {
                    eprintln!("Whisper init error: {}", err);
                    return None;
                }
            }
        }
        Some(())
    }

    /// Loads and preprocesses audio data from the temporary file.
    ///
    /// # Details
    /// Reads the temporary WAV file and converts it to the format required
    /// by Whisper (16 kHz, mono, normalized f32 samples).
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Some(Vec<f32>)` - Preprocessed audio samples.
    /// * `None` - Loading or conversion failed and error was logged.
    fn load_audio_data() -> Option<Vec<f32>> {
        match Self::load_audio_for_whisper(TEMP_AUDIO_PATH) {
            Ok(data) => Some(data),
            Err(err) => {
                eprintln!("Audio load error: {}", err);
                None
            }
        }
    }

    /// Runs Whisper inference and extracts transcribed text from all segments.
    ///
    /// # Details
    /// Creates a Whisper state, runs the full transcription pipeline, and
    /// concatenates text from all detected segments into a single string.
    ///
    /// # Arguments
    /// * `ctx` - The initialized Whisper context.
    /// * `audio_data` - Preprocessed audio samples.
    ///
    /// # Returns
    /// * `Some(String)` - The transcribed text with whitespace trimmed.
    /// * `None` - State creation or transcription failed and error was logged.
    fn run_whisper_inference(
        ctx: &whisper_rs::WhisperContext,
        audio_data: &[f32],
    ) -> Option<String> {
        let mut params =
            whisper_rs::FullParams::new(whisper_rs::SamplingStrategy::Greedy { best_of: 1 });
        params.set_language(Some("en"));
        params.set_print_progress(false);
        params.set_print_special(false);
        params.set_print_realtime(false);
        let mut state = match ctx.create_state() {
            Ok(s) => s,
            Err(err) => {
                eprintln!("Whisper state error: {}", err);
                return None;
            }
        };
        if let Err(err) = state.full(params, audio_data) {
            eprintln!("Whisper transcription error: {}", err);
            return None;
        }
        Self::extract_transcription_text(&state)
    }

    /// Extracts and concatenates text from all Whisper transcription segments.
    ///
    /// # Details
    /// Iterates through all detected segments and builds a single string
    /// with spaces between segments.
    ///
    /// # Arguments
    /// * `state` - The Whisper state after running inference.
    ///
    /// # Returns
    /// * `Some(String)` - Concatenated transcription with trimmed whitespace.
    /// * `None` - Should not occur in normal operation.
    fn extract_transcription_text(state: &whisper_rs::WhisperState) -> Option<String> {
        let num_segments = state.full_n_segments().unwrap_or(0);
        let mut text = String::new();
        for i in 0..num_segments {
            if let Ok(segment) = state.full_get_segment_text(i) {
                text.push_str(&segment);
                text.push(' ');
            }
        }
        Some(text.trim().to_string())
    }

    /// Initializes the Whisper context, downloading the model if needed.
    ///
    /// # Details
    /// Creates the models directory, downloads the GGML model from Hugging Face
    /// if not already present, and initializes a CPU-based Whisper context.
    /// This is a one-time setup operation that may take several minutes on
    /// first run due to the ~147 MB model download.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(WhisperContext)` - Ready-to-use Whisper context.
    ///
    /// # Errors
    /// Returns an error if directory creation, model download, or context
    /// initialization fails.
    fn init_whisper() -> Result<whisper_rs::WhisperContext> {
        use whisper_rs::WhisperContext;
        fs::create_dir_all("models")?;
        if !Path::new(WHISPER_MODEL_PATH).exists() {
            eprintln!("Downloading Whisper model (this may take a few minutes)...");
            Self::download_whisper_model()?;
        }
        let mut params = whisper_rs::WhisperContextParameters::default();
        params.use_gpu(false);
        WhisperContext::new_with_params(WHISPER_MODEL_PATH, params)
            .with_context(|| "Failed to initialize Whisper")
    }
    /// Downloads the Whisper GGML model from Hugging Face if not already present.
    ///
    /// # Details
    /// Uses the curl command-line tool to download the base.en model (approximately
    /// 147 MB) from the official ggerganov/whisper.cpp repository on Hugging Face.
    /// The download is blocking and may take several minutes depending on connection
    /// speed. Follows HTTP redirects (-L flag) to handle Hugging Face's CDN.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(())` - Model was downloaded successfully to WHISPER_MODEL_PATH.
    ///
    /// # Errors
    /// Returns an error if curl is not installed, execution fails, or the HTTP
    /// request is unsuccessful.
    fn download_whisper_model() -> Result<()> {
        const MODEL_URL: &str =
            "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin";
        let output = std::process::Command::new("curl")
            .args(["-L", "-o", WHISPER_MODEL_PATH, MODEL_URL])
            .output()
            .with_context(|| "Failed to execute curl")?;
        if !output.status.success() {
            anyhow::bail!("Failed to download Whisper model");
        }
        eprintln!("Whisper model downloaded successfully");
        Ok(())
    }

    /// Loads and preprocesses a WAV file for Whisper transcription.
    ///
    /// # Details
    /// Reads the WAV file using the hound library, converts 16-bit PCM samples
    /// to normalized f32 values in the range [-1.0, 1.0], resamples to 16 kHz
    /// if the source sample rate differs, and converts stereo to mono by
    /// averaging channels. Whisper requires 16 kHz mono audio for optimal
    /// speech recognition performance.
    ///
    /// # Arguments
    /// * `path` - The filesystem path to the WAV file to load.
    ///
    /// # Returns
    /// * `Ok(Vec<f32>)` - Normalized, resampled, mono audio samples ready for Whisper.
    ///
    /// # Errors
    /// Returns an error if the file cannot be opened, is not a valid WAV format,
    /// or sample reading fails.
    fn load_audio_for_whisper(path: &str) -> Result<Vec<f32>> {
        let reader = Self::open_wav_reader(path)?;
        let spec = reader.spec();
        let samples = Self::read_wav_samples(reader)?;
        let audio_data = Self::normalize_samples(&samples);
        let resampled = Self::maybe_resample(audio_data, spec.sample_rate);
        Ok(Self::maybe_downmix(resampled, spec.channels))
    }

    /// Opens a WAV file reader.
    ///
    /// # Details
    /// Creates a buffered WAV reader for the specified file path.
    ///
    /// # Arguments
    /// * `path` - The filesystem path to the WAV file.
    ///
    /// # Returns
    /// * `Ok(WavReader)` - Successfully opened WAV file reader.
    ///
    /// # Errors
    /// Returns an error if the file cannot be opened or is not a valid WAV.
    fn open_wav_reader(path: &str) -> Result<hound::WavReader<std::io::BufReader<std::fs::File>>> {
        hound::WavReader::open(path).with_context(|| format!("Failed to open WAV file: {}", path))
    }

    /// Reads all samples from a WAV file.
    ///
    /// # Details
    /// Iterates through the WAV file and collects all i16 PCM samples.
    ///
    /// # Arguments
    /// * `reader` - The WAV file reader to read from.
    ///
    /// # Returns
    /// * `Ok(Vec<i16>)` - All samples from the WAV file.
    ///
    /// # Errors
    /// Returns an error if sample reading fails.
    fn read_wav_samples(
        reader: hound::WavReader<std::io::BufReader<std::fs::File>>,
    ) -> Result<Vec<i16>> {
        reader
            .into_samples::<i16>()
            .collect::<Result<Vec<_>, _>>()
            .with_context(|| "Failed to read WAV samples")
    }

    /// Normalizes i16 samples to f32 range [-1.0, 1.0].
    ///
    /// # Details
    /// Divides each i16 sample by 32768.0 to map from integer to float range.
    ///
    /// # Arguments
    /// * `samples` - The i16 PCM samples to normalize.
    ///
    /// # Returns
    /// * `Vec<f32>` - Normalized samples in [-1.0, 1.0] range.
    fn normalize_samples(samples: &[i16]) -> Vec<f32> {
        samples.iter().map(|&s| s as f32 / 32768.0).collect()
    }

    /// Resamples audio if needed.
    ///
    /// # Details
    /// Converts audio to 16 kHz if the current sample rate differs from target.
    ///
    /// # Arguments
    /// * `audio_data` - The audio samples to potentially resample.
    /// * `sample_rate` - The current sample rate in Hz.
    ///
    /// # Returns
    /// * `Vec<f32>` - Audio resampled to 16 kHz, or original if already 16 kHz.
    fn maybe_resample(audio_data: Vec<f32>, sample_rate: u32) -> Vec<f32> {
        if sample_rate != 16000 {
            Self::resample(&audio_data, sample_rate, 16000)
        } else {
            audio_data
        }
    }

    /// Converts stereo to mono if needed.
    ///
    /// # Details
    /// Averages left and right channels when stereo audio is detected.
    ///
    /// # Arguments
    /// * `audio_data` - The audio samples to potentially downmix.
    /// * `channels` - The number of channels in the audio.
    ///
    /// # Returns
    /// * `Vec<f32>` - Mono audio, or original if already mono.
    fn maybe_downmix(audio_data: Vec<f32>, channels: u16) -> Vec<f32> {
        if channels == 2 {
            audio_data
                .chunks(2)
                .map(|chunk| (chunk[0] + chunk[1]) / 2.0)
                .collect()
        } else {
            audio_data
        }
    }

    /// Resamples audio data from one sample rate to another using linear interpolation.
    ///
    /// # Details
    /// Performs simple linear interpolation to convert audio between sample rates.
    /// Calculates the resampling ratio and interpolates between adjacent samples
    /// to generate output at the target rate. While not as precise as sinc
    /// interpolation, this approach is computationally efficient and sufficient
    /// for speech recognition where perfect frequency response is not critical.
    ///
    /// # Arguments
    /// * `input` - The source audio samples at the original sample rate.
    /// * `from_rate` - The original sample rate in Hz (e.g., 44100).
    /// * `to_rate` - The target sample rate in Hz (e.g., 16000).
    ///
    /// # Returns
    /// * `Vec<f32>` - Resampled audio data at the target rate, with length
    ///   proportional to the rate conversion ratio.
    fn resample(input: &[f32], from_rate: u32, to_rate: u32) -> Vec<f32> {
        let ratio = from_rate as f32 / to_rate as f32;
        let output_len = (input.len() as f32 / ratio) as usize;
        Self::interpolate_audio(input, ratio, output_len)
    }

    /// Performs linear interpolation on audio samples.
    ///
    /// # Details
    /// Generates output samples by interpolating between input samples at positions
    /// determined by the resampling ratio. Uses linear interpolation to estimate
    /// values between discrete input samples.
    ///
    /// # Arguments
    /// * `input` - Source audio samples to interpolate from.
    /// * `ratio` - Resampling ratio (from_rate / to_rate).
    /// * `output_len` - Number of output samples to generate.
    ///
    /// # Returns
    /// * `Vec<f32>` - Interpolated audio samples at the target length.
    fn interpolate_audio(input: &[f32], ratio: f32, output_len: usize) -> Vec<f32> {
        let mut output = Vec::with_capacity(output_len);
        for i in 0..output_len {
            let sample = Self::sample_at_position(input, i as f32 * ratio);
            output.push(sample);
        }
        output
    }

    /// Gets an interpolated sample at a specific position.
    ///
    /// # Details
    /// Performs linear interpolation between adjacent input samples at the given
    /// fractional position. Returns the boundary sample when at array edges, or
    /// zero when position exceeds input length.
    ///
    /// # Arguments
    /// * `input` - The source audio samples to interpolate from.
    /// * `pos` - Fractional position in the input array (may be non-integer).
    ///
    /// # Returns
    /// * `f32` - Interpolated sample value at the specified position.
    fn sample_at_position(input: &[f32], pos: f32) -> f32 {
        let idx = pos as usize;
        if idx + 1 < input.len() {
            let frac = pos - idx as f32;
            input[idx] * (1.0 - frac) + input[idx + 1] * frac
        } else if idx < input.len() {
            input[idx]
        } else {
            0.0
        }
    }

    /// Handles post-transcription logic such as exit detection and speaking.
    ///
    /// # Parameters
    /// * `user_text` - The normalized text returned by Whisper.
    ///
    /// # Returns
    /// * `Ok(true)` to continue looping, `Ok(false)` to exit.
    ///
    /// # Errors
    /// Forwards fatal errors from Groq completion calls via [`fetch_response`].
    async fn handle_user_text(&mut self, user_text: String) -> Result<bool> {
        if user_text.is_empty() {
            return Ok(true);
        }
        if should_quit(&user_text) {
            return Ok(false);
        }
        if self.try_device_command(&user_text) {
            return Ok(true);
        }
        let response = self.fetch_response(&user_text).await?;
        self.store_exchange(&user_text, &response);
        self.speak_response(&response);
        Ok(true)
    }

    /// Checks for Pico control intents and handles them locally.
    ///
    /// # Details
    /// Parses the user's utterance to detect LED control commands (on/off).
    /// If a valid command is found, sends it to the Pico over UART and speaks
    /// a confirmation message. Updates conversation history with the exchange.
    /// This enables local hardware control without requiring an LLM round-trip.
    ///
    /// # Arguments
    /// * `user_text` - The user's transcribed utterance to analyze for device commands.
    ///
    /// # Returns
    /// * `bool` - `true` if a device command was detected and handled, `false` otherwise.
    fn try_device_command(&mut self, user_text: &str) -> bool {
        // First try commands.json for extensible commands
        let normalized = user_text.to_lowercase();
        let cmd_match = commands::find_command(&self.commands, &normalized)
            .map(|c| (c.uart_command.clone(), c.description.clone()));
        
        if let Some((uart_cmd, description)) = cmd_match {
            let spoken = match self.send_raw_uart_command(&uart_cmd) {
                Ok(_) => {
                    self.device_context_active = true;
                    format!("{}.", description)
                }
                Err(err) => {
                    eprintln!("Serial command error: {}", err);
                    "I couldn't reach the Pico controller just yet.".to_string()
                }
            };
            self.store_exchange(user_text, &spoken);
            self.speak_response(&spoken);
            return true;
        }
        false
    }

    /// Requests an Ollama completion given the latest user input.
    ///
    /// # Parameters
    /// * `user_text` - The user's most recent utterance.
    ///
    /// # Returns
    /// A response string suitable for playback.
    ///
    /// # Errors
    /// Translates Ollama-specific errors into [`anyhow::Error`].
    async fn fetch_response(&mut self, user_text: &str) -> Result<String> {
        let cmd_list = commands::generate_command_list(&self.commands);
        let system_content = format!(
            "You are Embedi, an embedded-systems AI assistant. Your name is pronounced \
            'em bee dee'—like Auntie Em, the letter B, then the letter D. Share that \
            phrasing only when someone asks or when pronunciation confusion blocks the \
            conversation, and always add that any close pronunciation is fine. Stay \
            personable, concise, friendly, and a little playful while keeping guidance \
            grounded in embedded-systems thinking. Never sound combative or defensive \
            about your name. Keep replies short and conversational, and when asked who \
            created you, cite Kevin Thomas as your creator. When users want to control \
            hardware, you can send commands over a Raspberry Pi Pico UART link. \
            \n{}",
            cmd_list
        );
        let system_message = ChatMessage {
            role: "system".to_string(),
            content: system_content,
        };
        let mut messages = vec![system_message];
        messages.extend(self.history.clone());
        messages.push(ChatMessage {
            role: "user".to_string(),
            content: user_text.to_string(),
        });
        let request = OllamaRequest {
            model: self.ollama_model.clone(),
            messages,
            stream: false,
        };
        let response = self
            .client
            .post(OLLAMA_API_URL)
            .json(&request)
            .send()
            .await
            .with_context(|| "Failed to send request to Ollama")?;
        let ollama_response: OllamaResponse = response
            .json()
            .await
            .with_context(|| "Failed to parse Ollama response")?;
        Ok(ollama_response.message.content)
    }

    /// Stores the latest user/assistant exchange for conversational context.
    ///
    /// # Parameters
    /// * `user_text` - The user's utterance.
    /// * `response` - The assistant's reply.
    fn store_exchange(&mut self, user_text: &str, response: &str) {
        self.history.push(ChatMessage {
            role: "user".to_string(),
            content: user_text.to_string(),
        });
        self.history.push(ChatMessage {
            role: "assistant".to_string(),
            content: response.to_string(),
        });
        self.record_memory(user_text, response);
    }

    /// Speaks the response via the macOS `say` command, logging failures.
    ///
    /// # Parameters
    /// * `response` - The text that should be synthesized.
    fn speak_response(&self, response: &str) {
        if let Err(err) = speak(response) {
            eprintln!("TTS error: {}", err);
        }
    }

    /// Records a user-assistant exchange in the persistent memory log.
    ///
    /// # Details
    /// Appends both the user's input and the assistant's response to the in-memory
    /// buffer and immediately persists the updated history to disk.
    ///
    /// # Arguments
    /// * `user_text` - The user's transcribed utterance.
    /// * `response` - The assistant's generated reply.
    ///
    /// # Returns
    /// * `()` - Returns nothing; errors during persistence are logged.
    fn record_memory(&mut self, user_text: &str, response: &str) {
        self.memory
            .push(MemoryEntry::new(MemoryRole::User, user_text));
        self.memory
            .push(MemoryEntry::new(MemoryRole::Assistant, response));
        if let Err(err) = persist_memory(&self.memory) {
            eprintln!("Memory persist error: {}", err);
        }
    }
}

/// RAII guard that removes the temporary WAV file at scope exit.
///
/// # Details
/// Ensures automatic cleanup of temp.wav using RAII pattern. The file
/// is deleted when the guard goes out of scope, even on early returns.
struct TempAudioGuard;

/// Implementation of TempAudioGuard construction.
///
/// # Details
/// Provides a constructor for the RAII guard. Cleanup logic is in the Drop impl.
impl TempAudioGuard {
    /// Creates a guard instance; cleanup happens in `Drop`.
    ///
    /// # Returns
    /// A guard that triggers `cleanup_temp_file` when dropped.
    fn new() -> Self {
        Self
    }
}

/// Drop implementation to ensure temp file cleanup.
///
/// # Details
/// Automatically removes temp.wav when the guard goes out of scope,
/// ensuring cleanup even on panic or early return.
impl Drop for TempAudioGuard {
    /// Ensures the temp file is always removed, even on early returns.
    fn drop(&mut self) {
        cleanup_temp_file();
    }
}

/// Injects a short delay so the user has time to prepare before recording begins.
///
/// # Details
/// Introduces a configurable pause (PRE_RECORD_DELAY) before audio capture starts,
/// giving users a moment to begin speaking after the assistant indicates readiness.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `()` - Returns nothing; simply sleeps for the configured duration.
fn announce_listening() {
    std::thread::sleep(PRE_RECORD_DELAY);
}

/// Removes the temporary audio file, ignoring any failure (e.g., missing file).
///
/// # Details
/// Attempts to delete the WAV file at TEMP_AUDIO_PATH. This function is idempotent
/// and safe to call even when the file doesn't exist, making it suitable for cleanup
/// routines and error paths.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `()` - Returns nothing; all errors are silently ignored.
fn cleanup_temp_file() {
    fs::remove_file(TEMP_AUDIO_PATH).ok();
}

/// Determines whether the user has issued a command to exit the assistant.
///
/// # Details
/// Normalizes the input to lowercase and checks for common exit phrases
/// like "quit", "exit", "stop", "goodbye", or "bye". The check is substring-based,
/// so "please quit" will also trigger an exit.
///
/// # Arguments
/// * `user_text` - The transcribed user utterance to analyze.
///
/// # Returns
/// * `bool` - `true` when the utterance contains a quit phrase, `false` otherwise.
fn should_quit(user_text: &str) -> bool {
    let normalized = user_text.to_lowercase();
    ["quit", "exit", "stop", "goodbye", "bye"]
        .iter()
        .any(|word| normalized.contains(word))
}

/// Implementation of UART device command methods.
///
/// # Details
/// Provides methods for sending commands to the Raspberry Pi Pico over serial,
/// handling acknowledgments, and managing the serial port lifecycle.
impl VoiceAssistantRuntime {
    /// Sends a raw UART command string to the Pico.
    ///
    /// # Details
    /// Writes the command string as bytes to the serial port without acknowledgment checking.
    /// Used for extensible commands loaded from commands.json.
    ///
    /// # Arguments
    /// * `command` - The command string to send.
    ///
    /// # Returns
    /// * `Ok(())` - Command was sent successfully.
    ///
    /// # Errors
    /// Returns an error if serial write fails or port cannot be opened.
    fn send_raw_uart_command(&mut self, command: &str) -> Result<()> {
        eprintln!("→ Sending {} to Pico", command);
        self.ensure_serial_port()?;
        let payload = command.as_bytes();
        self.serial
            .as_mut()
            .unwrap()
            .write_all(payload)
            .with_context(|| "Failed to write to serial port")?;
        self.serial.as_mut().unwrap().write_all(b"\n")?;
        self.serial.as_mut().unwrap().flush()?;
        std::thread::sleep(SERIAL_COMMAND_DELAY);
        eprintln!("✓ Command sent");
        Ok(())
    }

    /// Opens the serial port if not already connected, with fallback to callout variant.
    ///
    /// # Details
    /// Checks if a serial port is already open. If not, attempts to open the
    /// configured port. On failure, tries the callout variant (cu.* instead of tty.*)
    /// and updates the stored path if successful. Sets DTR and RTS signals and
    /// waits for the Pico to boot.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(&mut dyn SerialPort)` - Mutable reference to the active serial port.
    ///
    /// # Errors
    /// Returns an error if both primary and fallback ports cannot be opened.
    fn ensure_serial_port(&mut self) -> Result<&mut dyn SerialPort> {
        if self.serial.is_none() {
            let port = self.open_port_with_fallback()?;
            self.serial = Some(port);
        }
        Ok(self
            .serial
            .as_mut()
            .map(|port| port.as_mut())
            .expect("serial port missing after initialization"))
    }

    /// Opens serial port with fallback to callout variant.
    ///
    /// # Details
    /// Attempts primary port first, then tries callout variant on failure.
    /// Configures DTR/RTS and waits for boot delay.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(Box<dyn SerialPort>)` - Configured serial port ready for use.
    ///
    /// # Errors
    /// Returns an error if both primary and fallback ports cannot be opened.
    fn open_port_with_fallback(&mut self) -> Result<Box<dyn SerialPort>> {
        let mut port = self.try_primary_then_fallback()?;
        configure_port_signals(&mut port);
        Ok(port)
    }

    /// Tries primary port then fallback variant.
    ///
    /// # Details
    /// Opens primary port or switches to callout variant on failure.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `Ok(Box<dyn SerialPort>)` - Successfully opened port.
    ///
    /// # Errors
    /// Returns an error if both ports fail to open.
    fn try_primary_then_fallback(&mut self) -> Result<Box<dyn SerialPort>> {
        match Self::open_serial_port(&self.serial_path, self.serial_baud) {
            Ok(port) => Ok(port),
            Err(primary_err) => self.try_fallback_port(primary_err),
        }
    }

    /// Attempts to open callout variant port.
    ///
    /// # Details
    /// Tries cu.* variant when tty.* fails, updates path on success.
    ///
    /// # Arguments
    /// * `primary_err` - Error from primary port attempt.
    ///
    /// # Returns
    /// * `Ok(Box<dyn SerialPort>)` - Opened fallback port.
    ///
    /// # Errors
    /// Returns primary error if no fallback exists or fails.
    fn try_fallback_port(&mut self, primary_err: anyhow::Error) -> Result<Box<dyn SerialPort>> {
        let Some(callout) = callout_variant(&self.serial_path) else {
            return Err(primary_err);
        };
        match Self::open_serial_port(&callout, self.serial_baud) {
            Ok(port) => {
                eprintln!(
                    "Primary port {} unavailable, switching to {}",
                    self.serial_path, callout
                );
                self.serial_path = callout;
                Ok(port)
            }
            Err(_) => Err(primary_err),
        }
    }
}

/// Configures serial port control signals.
///
/// # Details
/// Sets DTR and RTS signals and waits for Pico boot delay.
///
/// # Arguments
/// * `port` - The serial port to configure.
fn configure_port_signals(port: &mut Box<dyn SerialPort>) {
    let _ = port.write_data_terminal_ready(true);
    let _ = port.write_request_to_send(true);
    std::thread::sleep(SERIAL_BOOT_DELAY);
}

/// Implementation of serial port opening utility.
///
/// # Details
/// Provides low-level serial port opening functionality with timeout configuration.
impl VoiceAssistantRuntime {
    /// Opens a serial port with the specified path and baud rate.
    ///
    /// # Details
    /// Configures the port with a short timeout and attempts to establish
    /// a connection. This is a low-level helper used by ensure_serial_port.
    ///
    /// # Arguments
    /// * `path` - The device path (e.g., "/dev/cu.usbmodem21402").
    /// * `baud` - The baud rate (e.g., 115200).
    ///
    /// # Returns
    /// * `Ok(Box<dyn SerialPort>)` - Opened serial port ready for I/O.
    ///
    /// # Errors
    /// Returns an error if the port cannot be opened at the given path and baud.
    fn open_serial_port(path: &str, baud: u32) -> Result<Box<dyn SerialPort>> {
        serialport::new(path, baud)
            .timeout(SERIAL_TIMEOUT)
            .open()
            .with_context(|| format!("Failed to open {}", path))
    }
}

/// Determines the serial port path from environment variable or configuration.
///
/// # Details
/// Checks the EMBEDI_SERIAL_PORT environment variable first, then falls back
/// to the default_serial_port value from the configuration file.
///
/// # Arguments
/// * `config` - The loaded application configuration.
///
/// # Returns
/// * `String` - The serial port device path to use.
fn serial_port_path(config: &AppConfig) -> String {
    env::var("EMBEDI_SERIAL_PORT").unwrap_or_else(|_| config.default_serial_port.clone())
}

/// Determines the serial baud rate from environment variable or default.
///
/// # Details
/// Checks the EMBEDI_SERIAL_BAUD environment variable and parses it as a u32.
/// Falls back to DEFAULT_SERIAL_BAUD (115200) if not set or invalid.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `u32` - The baud rate to use for serial communication.
fn serial_baud_rate() -> u32 {
    env::var("EMBEDI_SERIAL_BAUD")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(DEFAULT_SERIAL_BAUD)
}

/// Converts a tty.* device path to its cu.* callout variant.
///
/// # Details
/// On macOS, serial devices appear as both /dev/tty.* (blocking) and /dev/cu.*
/// (non-blocking callout). This function converts between them for fallback logic.
///
/// # Arguments
/// * `path` - The original device path to convert.
///
/// # Returns
/// * `Some(String)` - The callout variant if the path starts with "/dev/tty.".
/// * `None` - If the path doesn't match the expected pattern.
fn callout_variant(path: &str) -> Option<String> {
    let suffix = path.strip_prefix("/dev/tty.")?;
    Some(format!("/dev/cu.{}", suffix))
}

/// Represents a single memory entry in persistent storage.
///
/// # Details
/// Stores one utterance from either the user or assistant along with
/// its role. Entries are serialized to JSON for conversation persistence.
#[derive(Clone, Serialize, Deserialize)]
struct MemoryEntry {
    role: MemoryRole,
    text: String,
}

/// Role designation for memory entries.
///
/// # Details
/// Distinguishes between user input and assistant responses in the
/// persistent conversation log.
#[derive(Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
enum MemoryRole {
    User,
    Assistant,
}

/// Implementation of memory entry construction.
impl MemoryEntry {
    /// Creates a new memory entry with the specified role and text.
    ///
    /// # Details
    /// Constructs a memory entry by pairing a role (User or Assistant) with
    /// the corresponding utterance text.
    ///
    /// # Arguments
    /// * `role` - The speaker role (User or Assistant).
    /// * `text` - The utterance content; accepts any type convertible to String.
    ///
    /// # Returns
    /// * `Self` - A new MemoryEntry instance.
    fn new(role: MemoryRole, text: impl Into<String>) -> Self {
        Self {
            role,
            text: text.into(),
        }
    }
}

/// Loads conversation history from the persistent memory file.
///
/// # Details
/// Reads the JSON memory log from disk and deserializes it into a vector of
/// memory entries. Returns an empty vector if the file doesn't exist or is empty.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `Ok(Vec<MemoryEntry>)` - The loaded conversation history.
///
/// # Errors
/// Returns an error if the file exists but cannot be read or parsed.
fn load_memory_entries() -> Result<Vec<MemoryEntry>> {
    if !Path::new(MEMORY_PATH).exists() {
        return Ok(Vec::new());
    }
    let contents = fs::read_to_string(MEMORY_PATH)
        .with_context(|| format!("Failed to read {}", MEMORY_PATH))?;
    if contents.trim().is_empty() {
        return Ok(Vec::new());
    }
    let entries: Vec<MemoryEntry> = serde_json::from_str(&contents)
        .with_context(|| format!("Failed to parse {}", MEMORY_PATH))?;
    Ok(entries)
}

/// Writes the conversation history to the persistent memory file.
///
/// # Details
/// Serializes the memory entries to pretty-printed JSON and writes them to
/// disk, overwriting any existing content.
///
/// # Arguments
/// * `entries` - The conversation history to persist.
///
/// # Returns
/// * `Ok(())` - Memory was successfully written to disk.
///
/// # Errors
/// Returns an error if serialization fails or the file cannot be written.
fn persist_memory(entries: &[MemoryEntry]) -> Result<()> {
    let json = serde_json::to_string_pretty(entries)
        .with_context(|| format!("Failed to serialize {}", MEMORY_PATH))?;
    fs::write(MEMORY_PATH, json).with_context(|| format!("Failed to write {}", MEMORY_PATH))
}

/// Loads configuration from `config.json`, falling back to baked defaults when missing.
///
/// # Details
/// Attempts to read and parse the configuration file. If the file doesn't exist
/// or contains invalid JSON, returns a default configuration and logs the error.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `AppConfig` - The loaded or default configuration.
fn load_app_config() -> AppConfig {
    match fs::read_to_string(CONFIG_PATH) {
        Ok(raw) => match serde_json::from_str(&raw) {
            Ok(cfg) => cfg,
            Err(err) => {
                eprintln!("Config parse error ({}): {}", CONFIG_PATH, err);
                AppConfig::default()
            }
        },
        Err(err) => {
            eprintln!("Config load error ({}): {}", CONFIG_PATH, err);
            AppConfig::default()
        }
    }
}

/// Returns the hardcoded fallback serial port path.
///
/// # Details
/// Provides the default serial device path used when no configuration is available.
/// This function exists to satisfy serde's default attribute requirements.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `String` - The fallback device path ("/dev/cu.usbmodem21402").
fn fallback_serial_port() -> String {
    FALLBACK_SERIAL_PORT.to_string()
}

/// Returns the hardcoded fallback Ollama model name.
///
/// # Details
/// Provides the default Ollama model used when no configuration is available.
/// This function exists to satisfy serde's default attribute requirements.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `String` - The fallback model name ("llama3.2:3b").
fn fallback_ollama_model() -> String {
    "llama3.2:3b".to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::File;
    use std::path::Path;
    use std::sync::Mutex;

    static TEMP_FILE_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn quit_detection_understands_variants() {
        assert!(should_quit("Please quit now"));
        assert!(should_quit("can you EXIT"));
        assert!(!should_quit("keep going"));
    }

    #[test]
    fn cleanup_removes_temp_file() {
        let _guard = TEMP_FILE_LOCK.lock().unwrap();
        File::create(TEMP_AUDIO_PATH).expect("create temp file");
        assert!(Path::new(TEMP_AUDIO_PATH).exists());
        cleanup_temp_file();
        assert!(!Path::new(TEMP_AUDIO_PATH).exists());
    }

    #[test]
    fn guard_drops_temp_file() {
        let _guard = TEMP_FILE_LOCK.lock().unwrap();
        File::create(TEMP_AUDIO_PATH).expect("create temp file");
        {
            let _temp_guard = TempAudioGuard::new();
        }
        assert!(!Path::new(TEMP_AUDIO_PATH).exists());
    }

    #[test]
    fn persist_samples_writes_audio() {
        let _guard = TEMP_FILE_LOCK.lock().unwrap();
        let samples = vec![0_i16, i16::MAX / 4, -i16::MAX / 4];
        assert!(VoiceAssistantRuntime::persist_samples(&samples));
        assert!(Path::new(TEMP_AUDIO_PATH).exists());
        cleanup_temp_file();
    }

    #[test]
    fn contains_speech_requires_energy() {
        assert!(!VoiceAssistantRuntime::contains_speech(&[0_i16; 1600]));
        let loud = vec![i16::MAX / 2; 1600];
        assert!(VoiceAssistantRuntime::contains_speech(&loud));
    }
}
