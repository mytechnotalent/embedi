//! Voice assistant orchestration module.

use std::fs;
use std::time::Duration;

use anyhow::Result;
use rig::completion::{Chat, Prompt};
use rig::message::Message;
use rig::prelude::*;
use rig::providers::groq;
use rig::transcription::TranscriptionModel;

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
const PRE_RECORD_DELAY: Duration = Duration::from_millis(500);

/// Minimum RMS amplitude considered speech.
///
/// Values much above ~300 miss normal speaking levels on some microphones, so
/// we bias toward a lower threshold and rely on Whisper to filter background noise.
const SILENCE_RMS_THRESHOLD: f32 = 150.0;

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

/// Runtime container that owns the Groq client and conversation state.
struct VoiceAssistantRuntime {
    client: groq::Client,
    history: Vec<Message>,
}

impl VoiceAssistantRuntime {
    /// Creates a new runtime by loading Groq credentials from the environment.
    ///
    /// # Returns
    /// A ready-to-run [`VoiceAssistantRuntime`] with an empty conversation history.
    ///
    /// # Errors
    /// Propagates failures from [`groq::Client::from_env`].
    fn new() -> Result<Self> {
        Ok(Self {
            client: groq::Client::from_env(),
            history: Vec::new(),
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

    /// Returns `true` when the recording contains meaningful speech.
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

    /// Sends the temporary WAV file to Groq Whisper for transcription.
    ///
    /// # Returns
    /// * `Some(text)` when Whisper succeeds.
    /// * `None` when the error has already been logged and the loop should continue.
    async fn transcribe_current_audio(&self) -> Option<String> {
        match self
            .client
            .transcription_model(groq::WHISPER_LARGE_V3_TURBO)
            .transcription_request()
            .load_file(TEMP_AUDIO_PATH)
            .send()
            .await
        {
            Ok(data) => {
                let text = data.text.trim().to_string();
                Some(text)
            }
            Err(err) => {
                eprintln!("Transcription error: {}", err);
                None
            }
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
        let response = self.fetch_response(&user_text).await?;
        self.store_exchange(&user_text, &response);
        self.speak_response(&response);
        Ok(true)
    }

    /// Requests a Groq completion given the latest user input.
    ///
    /// # Parameters
    /// * `user_text` - The user's most recent utterance.
    ///
    /// # Returns
    /// A response string suitable for playback.
    ///
    /// # Errors
    /// Translates groq-specific prompt errors into [`anyhow::Error`].
    async fn fetch_response(&self, user_text: &str) -> Result<String> {
        let agent = self
            .client
            .agent("llama-3.3-70b-versatile")
            .preamble(
                "You are Embedi, an embedded-systems AI assistant. Your name is pronounced \n
                    'em bee dee'—like Auntie Em, the letter B, then the letter D. Share that \n
                    phrasing only when someone asks or when pronunciation confusion blocks the \n
                    conversation, and always add that any close pronunciation is fine. Stay \n
                    personable, concise, friendly, and a little playful while keeping guidance \n
                    grounded in embedded-systems thinking. Never sound combative or defensive \n
                    about your name. Keep replies short and conversational, and when asked who \n
                    created you, cite Kevin Thomas as your creator.",
            )
            .build();
        let pending = if self.history.is_empty() {
            agent.prompt(user_text).await
        } else {
            agent.chat(user_text, self.history.clone()).await
        };
        pending.map_err(|err| anyhow::anyhow!(err))
    }

    /// Stores the latest user/assistant exchange for conversational context.
    ///
    /// # Parameters
    /// * `user_text` - The user's utterance.
    /// * `response` - The assistant's reply.
    fn store_exchange(&mut self, user_text: &str, response: &str) {
        self.history.push(Message::user(user_text));
        self.history.push(Message::assistant(response));
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
/// # Returns
/// Nothing. The function sleeps briefly to give the user time to prepare.
fn announce_listening() {
    std::thread::sleep(PRE_RECORD_DELAY);
}

/// Removes the temporary audio file, ignoring any failure (e.g., missing file).
///
/// # Returns
/// Nothing; failures are intentionally suppressed.
fn cleanup_temp_file() {
    fs::remove_file(TEMP_AUDIO_PATH).ok();
}

/// Returns true if the user wants to stop the assistant.
///
/// # Parameters
/// * `user_text` - The most recent user utterance.
///
/// # Returns
/// `true` when the utterance matches a quit phrase, otherwise `false`.
fn should_quit(user_text: &str) -> bool {
    let normalized = user_text.to_lowercase();
    ["quit", "exit", "stop", "goodbye", "bye"]
        .iter()
        .any(|word| normalized.contains(word))
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
