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

use std::{
    env, fs,
    io::ErrorKind,
    path::Path,
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;

use crate::audio::{record_audio, save_wav};
use crate::speech::speak;

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

/// Default Ollama model to use.
const DEFAULT_OLLAMA_MODEL: &str = "llama3.2:3b";

/// Whisper model path (will be auto-downloaded if not present).
const WHISPER_MODEL_PATH: &str = "models/ggml-base.en.bin";

/// Serial write timeout to avoid blocking the microphone loop.
const SERIAL_TIMEOUT: Duration = Duration::from_millis(100);

/// Time to let the Pico reboot after the UART is opened (DTR toggle).
const SERIAL_BOOT_DELAY: Duration = Duration::from_millis(150);

/// Minimum spacing between consecutive UART commands.
const SERIAL_COMMAND_DELAY: Duration = Duration::from_millis(30);

/// Bytes to pull from the UART echo buffer per read.
const SERIAL_READ_CHUNK: usize = 256;

/// How long to wait for a Pico acknowledgment before retrying.
const SERIAL_ACK_TIMEOUT: Duration = Duration::from_millis(500);

/// Sleep between acknowledgment polling iterations.
const SERIAL_ACK_SLEEP: Duration = Duration::from_millis(15);

/// Tokens that explicitly mention the Pico LED hardware.
const LED_TARGET_TOKENS: &[&str] = &["led", "light", "lights", "gpio", "pin"];

/// Pronoun tokens that we allow once an LED command has been executed.
const LED_PRONOUN_TOKENS: &[&str] = &["it", "thing", "that", "this", "one"];

/// Lowercase tokens that count as an acknowledgement for LED ON commands.
const LED_ON_ACK_TOKENS: &[&str] = &["led on", "led: on"];

/// Lowercase tokens that count as an acknowledgement for LED OFF commands.
const LED_OFF_ACK_TOKENS: &[&str] = &["led off", "led: off"];

/// Strongly typed representation of `config.json`.
#[derive(Clone, Deserialize)]
struct AppConfig {
    #[serde(default = "fallback_serial_port")]
    default_serial_port: String,
}

/// Returns the fallback serial port path.
impl Default for AppConfig {
    fn default() -> Self {
        Self {
            default_serial_port: fallback_serial_port(),
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
    VoiceAssistantRuntime::new()?.run_loop().await
}

/// Cleans up temporary files created during operation.
///
/// This function is safe to call multiple times and will not fail if files don't exist.
pub fn cleanup_temp_files() {
    cleanup_temp_file();
}

/// Runtime container that owns the HTTP client and conversation state.
struct VoiceAssistantRuntime {
    client: reqwest::Client,
    ollama_model: String,
    whisper_ctx: Option<whisper_rs::WhisperContext>,
    history: Vec<ChatMessage>,
    serial: Option<Box<dyn SerialPort>>,
    serial_path: String,
    serial_baud: u32,
    memory: Vec<MemoryEntry>,
    led_context_active: bool,
}

/// Chat message structure for Ollama API.
#[derive(Clone, Serialize, Deserialize)]
struct ChatMessage {
    role: String,
    content: String,
}

/// Request structure for Ollama API.
#[derive(Serialize)]
struct OllamaRequest {
    model: String,
    messages: Vec<ChatMessage>,
    stream: bool,
}

/// Response structure for Ollama API.
#[derive(Deserialize)]
struct OllamaResponse {
    message: ChatMessage,
}

/// Represents a single memory entry in persistent storage.
impl VoiceAssistantRuntime {
    /// Creates a new runtime by loading configuration.
    ///
    /// # Returns
    /// A ready-to-run [`VoiceAssistantRuntime`] with an empty conversation history.
    ///
    /// # Errors
    /// Propagates failures from initialization.
    fn new() -> Result<Self> {
        let memory_entries = match load_memory_entries() {
            Ok(entries) => entries,
            Err(err) => {
                eprintln!("Memory load error: {}", err);
                Vec::new()
            }
        };
        let mut history = Vec::new();
        for entry in &memory_entries {
            history.push(ChatMessage {
                role: match entry.role {
                    MemoryRole::User => "user".to_string(),
                    MemoryRole::Assistant => "assistant".to_string(),
                },
                content: entry.text.clone(),
            });
        }
        let config = load_app_config();
        let ollama_model =
            env::var("OLLAMA_MODEL").unwrap_or_else(|_| DEFAULT_OLLAMA_MODEL.to_string());
        Ok(Self {
            client: reqwest::Client::new(),
            ollama_model,
            whisper_ctx: None,
            history,
            serial: None,
            serial_path: serial_port_path(&config),
            serial_baud: serial_baud_rate(),
            memory: memory_entries,
            led_context_active: false,
        })
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

        // Create models directory if it doesn't exist
        fs::create_dir_all("models")?;

        // Download model if not present
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

        // Use curl to download instead of blocking reqwest
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
        let reader = hound::WavReader::open(path)
            .with_context(|| format!("Failed to open WAV file: {}", path))?;

        let spec = reader.spec();
        let samples: Vec<i16> = reader
            .into_samples::<i16>()
            .collect::<Result<Vec<_>, _>>()
            .with_context(|| "Failed to read WAV samples")?;

        // Convert to f32 and normalize
        let mut audio_data: Vec<f32> = samples.iter().map(|&s| s as f32 / 32768.0).collect();

        // Resample to 16kHz if needed (Whisper expects 16kHz)
        if spec.sample_rate != 16000 {
            audio_data = Self::resample(&audio_data, spec.sample_rate, 16000);
        }

        // Convert stereo to mono if needed
        if spec.channels == 2 {
            audio_data = audio_data
                .chunks(2)
                .map(|chunk| (chunk[0] + chunk[1]) / 2.0)
                .collect();
        }

        Ok(audio_data)
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
        // Simple linear interpolation resampling
        let ratio = from_rate as f32 / to_rate as f32;
        let output_len = (input.len() as f32 / ratio) as usize;
        let mut output = Vec::with_capacity(output_len);

        for i in 0..output_len {
            let pos = i as f32 * ratio;
            let idx = pos as usize;
            if idx + 1 < input.len() {
                let frac = pos - idx as f32;
                let sample = input[idx] * (1.0 - frac) + input[idx + 1] * frac;
                output.push(sample);
            } else if idx < input.len() {
                output.push(input[idx]);
            }
        }

        output
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
    fn try_device_command(&mut self, user_text: &str) -> bool {
        let allow_pronoun_reference = self.led_context_active;
        let Some(command) = DeviceCommand::from_text(user_text, allow_pronoun_reference) else {
            return false;
        };
        let spoken = match self.send_device_command(command) {
            Ok(_) => {
                self.led_context_active = true;
                command.success_message().to_string()
            }
            Err(err) => {
                eprintln!("Serial command error: {}", err);
                "I couldn't reach the Pico controller just yet.".to_string()
            }
        };
        self.store_exchange(user_text, &spoken);
        self.speak_response(&spoken);
        true
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
        let system_message = ChatMessage {
            role: "system".to_string(),
            content: "You are Embedi, an embedded-systems AI assistant. Your name is pronounced \
                'em bee dee'—like Auntie Em, the letter B, then the letter D. Share that \
                phrasing only when someone asks or when pronunciation confusion blocks the \
                conversation, and always add that any close pronunciation is fine. Stay \
                personable, concise, friendly, and a little playful while keeping guidance \
                grounded in embedded-systems thinking. Never sound combative or defensive \
                about your name. Keep replies short and conversational, and when asked who \
                created you, cite Kevin Thomas as your creator. When users want to toggle \
                hardware, explain that you can speak over a Raspberry Pi Pico UART link to \
                flip its LED on or off and confirm what action you just triggered."
                .to_string(),
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
struct TempAudioGuard;

impl TempAudioGuard {
    /// Creates a guard instance; cleanup happens in `Drop`.
    ///
    /// # Returns
    /// A guard that triggers `cleanup_temp_file` when dropped.
    fn new() -> Self {
        Self
    }
}

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

/// Commands that can be executed locally without a round-trip to the LLM.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DeviceCommand {
    LedOn,
    LedOff,
}

impl DeviceCommand {
    /// Parses user text to extract LED control commands.
    ///
    /// # Details
    /// Tokenizes the input, checks for LED-related keywords ("led", "light", etc.),
    /// and detects intent words like "on", "enable", "off", or "disable". When
    /// `allow_pronoun_reference` is true, pronouns like "it" are also accepted as
    /// valid LED targets.
    ///
    /// # Arguments
    /// * `text` - The user's utterance to parse.
    /// * `allow_pronoun_reference` - Whether to allow pronouns as LED references.
    ///
    /// # Returns
    /// * `Option<Self>` - `Some(DeviceCommand)` if a valid LED command is detected,
    ///   `None` otherwise.
    fn from_text(text: &str, allow_pronoun_reference: bool) -> Option<Self> {
        let normalized = text.to_lowercase();
        let tokens: Vec<&str> = normalized
            .split(|c: char| !c.is_ascii_alphanumeric())
            .filter(|token| !token.is_empty())
            .collect();
        let mentions_target = tokens.iter().any(|token| LED_TARGET_TOKENS.contains(token))
            || (allow_pronoun_reference
                && tokens
                    .iter()
                    .any(|token| LED_PRONOUN_TOKENS.contains(token)));
        if !mentions_target {
            return None;
        }
        let wants_on = tokens.iter().any(|token| {
            matches!(
                *token,
                "on" | "enable" | "start" | "activate" | "high" | "set"
            )
        });
        let wants_off = tokens.iter().any(|token| {
            matches!(
                *token,
                "off" | "disable" | "stop" | "deactivate" | "low" | "clear"
            )
        });
        match (wants_on, wants_off) {
            (true, false) => Some(Self::LedOn),
            (false, true) => Some(Self::LedOff),
            _ => None,
        }
    }

    /// Returns the UART payload bytes for this command.
    ///
    /// # Details
    /// Provides the exact byte sequence to transmit over the serial port
    /// for the corresponding LED command.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `&'static [u8]` - The command bytes ("ON" or "OFF").
    fn serial_bytes(self) -> &'static [u8] {
        match self {
            Self::LedOn => b"ON",
            Self::LedOff => b"OFF",
        }
    }

    /// Returns the spoken confirmation message for this command.
    ///
    /// # Details
    /// Provides the text that the assistant will speak when the command
    /// is successfully sent to the Pico.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `&'static str` - The confirmation message to be spoken.
    fn success_message(self) -> &'static str {
        match self {
            Self::LedOn => "Turning the Pico LED on.",
            Self::LedOff => "Turning the Pico LED off.",
        }
    }

    /// Returns a human-readable label for this command.
    ///
    /// # Details
    /// Provides a string representation used for logging and debugging,
    /// such as "LED ON" or "LED OFF".
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `&'static str` - The command label for display purposes.
    fn label(self) -> &'static str {
        match self {
            Self::LedOn => "LED ON",
            Self::LedOff => "LED OFF",
        }
    }

    /// Returns the expected acknowledgment tokens from the Pico.
    ///
    /// # Details
    /// Provides the list of lowercase strings to look for in the UART response
    /// to confirm the command was received and executed by the microcontroller.
    ///
    /// # Arguments
    /// None.
    ///
    /// # Returns
    /// * `&'static [&'static str]` - Array of valid acknowledgment strings.
    fn ack_tokens(self) -> &'static [&'static str] {
        match self {
            Self::LedOn => LED_ON_ACK_TOKENS,
            Self::LedOff => LED_OFF_ACK_TOKENS,
        }
    }
}

impl VoiceAssistantRuntime {
    /// Sends a command to the Pico over UART with acknowledgment and retry logic.
    ///
    /// # Details
    /// Attempts to write the command payload to the serial port and waits for
    /// acknowledgment from the Pico. Retries once if the first attempt fails or
    /// times out. The serial port is reopened between retries.
    ///
    /// # Arguments
    /// * `command` - The LED command to execute on the Pico.
    ///
    /// # Returns
    /// * `Ok(())` - Command was acknowledged by the Pico.
    ///
    /// # Errors
    /// Returns an error if both attempts fail to receive acknowledgment.
    fn send_device_command(&mut self, command: DeviceCommand) -> Result<()> {
        let payload = command.serial_bytes();
        let mut last_err = None;
        for attempt in 0..2 {
            let path = self.serial_path.clone();
            match self.write_uart_payload(payload) {
                Ok(_) => {
                    eprintln!(
                        "[UART send] {} -> {} (attempt {})",
                        path,
                        command.label(),
                        attempt + 1
                    );
                    if self.await_command_ack(command)? {
                        return Ok(());
                    } else {
                        eprintln!(
                            "UART ack missing for {} on {}, retrying",
                            command.label(),
                            path
                        );
                        self.serial = None;
                    }
                }
                Err(err) => {
                    eprintln!("UART write error on {}: {}", path, err);
                    last_err = Some(err);
                    self.serial = None;
                }
            }
        }
        if let Some(err) = last_err {
            Err(err)
        } else {
            anyhow::bail!(
                "UART did not acknowledge command {} even after retries",
                command.label()
            )
        }
    }

    /// Writes a command payload to the UART with proper framing.
    ///
    /// # Details
    /// Ensures the serial port is open, drains any pending input, writes the
    /// payload bytes followed by a newline, and flushes the output buffer.
    /// Includes a delay after transmission to prevent command overlap.
    ///
    /// # Arguments
    /// * `payload` - The command bytes to transmit.
    ///
    /// # Returns
    /// * `Ok(())` - Payload was written and flushed successfully.
    ///
    /// # Errors
    /// Returns an error if the serial port cannot be opened or write operations fail.
    fn write_uart_payload(&mut self, payload: &[u8]) -> Result<()> {
        let path = self.serial_path.clone();
        let port = self.ensure_serial_port()?;
        // Clear stale data before sending new command
        drain_serial_input(port);
        port.write_all(payload)
            .with_context(|| format!("Failed to write to {}", path))?;
        port.write_all(b"\n")
            .with_context(|| format!("Failed to send newline to {}", path))?;
        port.flush()
            .with_context(|| format!("Failed to flush {}", path))?;
        std::thread::sleep(SERIAL_COMMAND_DELAY);
        Ok(())
    }

    /// Waits for acknowledgment from the Pico after sending a command.
    ///
    /// # Details
    /// Polls the serial port for incoming data until the acknowledgment timeout
    /// expires. Accumulates received text and checks for command-specific
    /// acknowledgment tokens (e.g., "LED ON", "LED OFF").
    ///
    /// # Arguments
    /// * `command` - The command for which acknowledgment is expected.
    ///
    /// # Returns
    /// * `Ok(true)` - Valid acknowledgment received.
    /// * `Ok(false)` - Timeout expired without acknowledgment.
    ///
    /// # Errors
    /// Returns an error only for unexpected I/O failures (not timeout).
    fn await_command_ack(&mut self, command: DeviceCommand) -> Result<bool> {
        let deadline = Instant::now() + SERIAL_ACK_TIMEOUT;
        let port = self.ensure_serial_port()?;
        let mut scratch = [0_u8; SERIAL_READ_CHUNK];
        let mut transcript = String::new();
        while Instant::now() < deadline {
            match port.read(&mut scratch) {
                Ok(0) => {}
                Ok(n) => {
                    let raw = String::from_utf8_lossy(&scratch[..n]).replace('\r', "");
                    if !raw.trim().is_empty() {
                        eprintln!("[UART ack] {}", raw.trim());
                    }
                    transcript.push_str(&raw.to_lowercase());
                    if command
                        .ack_tokens()
                        .iter()
                        .any(|token| transcript.contains(token))
                    {
                        return Ok(true);
                    }
                }
                Err(err) => match err.kind() {
                    ErrorKind::WouldBlock | ErrorKind::TimedOut => {}
                    _ => {
                        eprintln!("UART ack error: {}", err);
                        return Ok(false);
                    }
                },
            }
            std::thread::sleep(SERIAL_ACK_SLEEP);
        }
        Ok(false)
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
            let mut port = match Self::open_serial_port(&self.serial_path, self.serial_baud) {
                Ok(port) => port,
                Err(primary_err) => {
                    if let Some(callout) = callout_variant(&self.serial_path) {
                        match Self::open_serial_port(&callout, self.serial_baud) {
                            Ok(port) => {
                                eprintln!(
                                    "Primary port {} unavailable, switching to {}",
                                    self.serial_path, callout
                                );
                                self.serial_path = callout;
                                port
                            }
                            Err(_) => return Err(primary_err),
                        }
                    } else {
                        return Err(primary_err);
                    }
                }
            };
            let _ = port.write_data_terminal_ready(true);
            let _ = port.write_request_to_send(true);
            std::thread::sleep(SERIAL_BOOT_DELAY);
            self.serial = Some(port);
        }
        Ok(self
            .serial
            .as_mut()
            .map(|port| port.as_mut())
            .expect("serial port missing after initialization"))
    }
}

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

/// Drains any pending data from the serial port input buffer.
///
/// # Details
/// Repeatedly reads from the port until no more data is available or a timeout
/// occurs. This prevents stale acknowledgments from previous commands from
/// interfering with the current operation.
///
/// # Arguments
/// * `port` - Mutable reference to the serial port to drain.
///
/// # Returns
/// * `()` - Returns nothing; errors are logged but not propagated.
fn drain_serial_input(port: &mut dyn SerialPort) {
    let mut buffer = [0_u8; SERIAL_READ_CHUNK];
    loop {
        match port.read(&mut buffer) {
            Ok(0) => break,
            Ok(_) => continue,
            Err(err) => match err.kind() {
                ErrorKind::WouldBlock | ErrorKind::TimedOut => break,
                _ => {
                    eprintln!("UART drain error: {}", err);
                    break;
                }
            },
        }
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

#[derive(Clone, Serialize, Deserialize)]
struct MemoryEntry {
    role: MemoryRole,
    text: String,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
enum MemoryRole {
    User,
    Assistant,
}

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

    #[test]
    fn device_command_classification() {
        assert_eq!(
            DeviceCommand::from_text("please turn the LED on", false),
            Some(DeviceCommand::LedOn)
        );
        assert_eq!(
            DeviceCommand::from_text("could you disable the light", false),
            Some(DeviceCommand::LedOff)
        );
        assert_eq!(DeviceCommand::from_text("tell me a joke", false), None);
        assert_eq!(
            DeviceCommand::from_text("can you turn it on", false),
            None,
            "pronoun commands require LED context"
        );
        assert_eq!(
            DeviceCommand::from_text("turn it off", true),
            Some(DeviceCommand::LedOff)
        );
    }
}
