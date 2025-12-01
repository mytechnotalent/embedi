/*
 * @file lib.rs
 * @brief Embedi library root
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
pub mod commands;
pub mod speech;
