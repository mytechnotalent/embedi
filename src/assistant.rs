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
use rig::completion::{Chat, Prompt};
use rig::message::Message;
use rig::prelude::*;
use rig::providers::groq;
use rig::transcription::TranscriptionModel;
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
const PRE_RECORD_DELAY: Duration = Duration::from_millis(500);

/// Minimum RMS amplitude considered speech.
///
/// Values much above ~300 miss normal speaking levels on some microphones, so
/// we bias toward a lower threshold and rely on Whisper to filter background noise.
const SILENCE_RMS_THRESHOLD: f32 = 150.0;

/// Default serial device Embedi uses to reach the Raspberry Pi Pico.
const DEFAULT_SERIAL_PORT: &str = "/dev/cu.usbmodem1402";

/// Persistent memory log that accumulates every exchange Embedi hears.
const MEMORY_PATH: &str = "memory.json";

/// Default baud rate used for Pico UART commands.
const DEFAULT_SERIAL_BAUD: u32 = 115_200;

/// Serial write timeout to avoid blocking the microphone loop.
const SERIAL_TIMEOUT: Duration = Duration::from_millis(100);

/// Time to let the Pico reboot after the UART is opened (DTR toggle).
const SERIAL_BOOT_DELAY: Duration = Duration::from_millis(250);

/// Minimum spacing between consecutive UART commands.
const SERIAL_COMMAND_DELAY: Duration = Duration::from_millis(75);

/// Bytes to pull from the UART echo buffer per read.
const SERIAL_READ_CHUNK: usize = 256;

/// How long to wait for a Pico acknowledgment before retrying.
const SERIAL_ACK_TIMEOUT: Duration = Duration::from_millis(300);

/// Sleep between acknowledgment polling iterations.
const SERIAL_ACK_SLEEP: Duration = Duration::from_millis(15);

/// Tokens that explicitly mention the Pico LED hardware.
const LED_TARGET_TOKENS: &[&str] = &["led", "light", "lights", "gpio", "pin"];

/// Pronoun tokens that we allow once an LED command has been executed.
const LED_PRONOUN_TOKENS: &[&str] = &["it", "thing", "that", "this", "one"];

/// Lowercase tokens that count as an acknowledgement for LED ON commands.
const LED_ON_ACK_TOKENS: &[&str] = &["led on", "led: on", "on"];

/// Lowercase tokens that count as an acknowledgement for LED OFF commands.
const LED_OFF_ACK_TOKENS: &[&str] = &["led off", "led: off", "off"];

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
    serial: Option<Box<dyn SerialPort>>,
    serial_path: String,
    serial_baud: u32,
    memory: Vec<MemoryEntry>,
    led_context_active: bool,
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
        let memory_entries = match load_memory_entries() {
            Ok(entries) => entries,
            Err(err) => {
                eprintln!("Memory load error: {}", err);
                Vec::new()
            }
        };
        let mut history = Vec::new();
        for entry in &memory_entries {
            if let Some(message) = entry.to_message() {
                history.push(message);
            }
        }
        Ok(Self {
            client: groq::Client::from_env(),
            history,
            serial: None,
            serial_path: serial_port_path(),
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
                created you, cite Kevin Thomas as your creator. When users want to toggle \n
                hardware, explain that you can speak over a Raspberry Pi Pico UART link to \n
                flip its LED on or off and confirm what action you just triggered.",
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

/// Commands that can be executed locally without a round-trip to the LLM.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DeviceCommand {
    LedOn,
    LedOff,
}

impl DeviceCommand {
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

    fn serial_bytes(self) -> &'static [u8] {
        match self {
            Self::LedOn => b"ON",
            Self::LedOff => b"OFF",
        }
    }

    fn success_message(self) -> &'static str {
        match self {
            Self::LedOn => "Turning the Pico LED on.",
            Self::LedOff => "Turning the Pico LED off.",
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::LedOn => "LED ON",
            Self::LedOff => "LED OFF",
        }
    }

    fn ack_tokens(self) -> &'static [&'static str] {
        match self {
            Self::LedOn => LED_ON_ACK_TOKENS,
            Self::LedOff => LED_OFF_ACK_TOKENS,
        }
    }
}

impl VoiceAssistantRuntime {
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

    fn write_uart_payload(&mut self, payload: &[u8]) -> Result<()> {
        let path = self.serial_path.clone();
        let port = self.ensure_serial_port()?;
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
    fn open_serial_port(path: &str, baud: u32) -> Result<Box<dyn SerialPort>> {
        serialport::new(path, baud)
            .timeout(SERIAL_TIMEOUT)
            .open()
            .with_context(|| format!("Failed to open {}", path))
    }
}

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

fn serial_port_path() -> String {
    env::var("EMBEDI_SERIAL_PORT").unwrap_or_else(|_| DEFAULT_SERIAL_PORT.to_string())
}

fn serial_baud_rate() -> u32 {
    env::var("EMBEDI_SERIAL_BAUD")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(DEFAULT_SERIAL_BAUD)
}

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
    fn new(role: MemoryRole, text: impl Into<String>) -> Self {
        Self {
            role,
            text: text.into(),
        }
    }

    fn to_message(&self) -> Option<Message> {
        match self.role {
            MemoryRole::User => Some(Message::user(self.text.clone())),
            MemoryRole::Assistant => Some(Message::assistant(self.text.clone())),
        }
    }
}

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

fn persist_memory(entries: &[MemoryEntry]) -> Result<()> {
    let json = serde_json::to_string_pretty(entries)
        .with_context(|| format!("Failed to serialize {}", MEMORY_PATH))?;
    fs::write(MEMORY_PATH, json).with_context(|| format!("Failed to write {}", MEMORY_PATH))
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
