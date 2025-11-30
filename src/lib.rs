//! Rig Speech Example - A voice-interactive AI assistant.
//!
//! This library provides a fully hands-free AI assistant that uses:
//! - Groq's Llama 3.3 70B for conversational AI
//! - Groq's Whisper Large v3 Turbo for speech transcription
//! - macOS `say` command for text-to-speech
//!
//! # Example
//! ```no_run
//! use anyhow::Result;
//! use embedi::assistant;
//!
//! #[tokio::main]
//! async fn main() -> Result<()> {
//!     dotenv::dotenv().ok();
//!     assistant::run_voice_assistant().await
//! }
//! ```

pub mod assistant;
pub mod audio;
pub mod speech;
