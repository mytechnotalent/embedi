/*
 * @file speech.rs
 * @brief Text-to-speech utilities for Embedi
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

//! Text-to-speech functionality module.

use anyhow::Result;
use std::sync::Mutex;

/// Speaks the given text using macOS `say` command.
///
/// # Parameters
/// * `text` - The utterance to synthesize.
///
/// # Returns
/// `Ok(())` when the `say` command completes successfully.
///
/// # Errors
/// Returns an error if the `say` command fails to spawn or exits unexpectedly.
pub fn speak(text: &str) -> Result<()> {
    if text.trim().is_empty() {
        anyhow::bail!("Cannot speak empty text");
    }
    run_say(text)?;
    Ok(())
}

fn run_say(text: &str) -> Result<()> {
    if cfg!(test) {
        if *FORCE_ERROR.lock().unwrap() {
            anyhow::bail!("Forced failure for testing");
        }
        return Ok(());
    }

    std::process::Command::new("say").arg(text).output()?;
    Ok(())
}

#[cfg_attr(not(test), allow(dead_code))]
static FORCE_ERROR: Mutex<bool> = Mutex::new(false);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speak_succeeds_with_text() {
        assert!(speak("Hello test").is_ok());
    }

    #[test]
    fn speak_fails_when_forced() {
        *super::FORCE_ERROR.lock().unwrap() = true;
        let result = speak("failure case");
        *super::FORCE_ERROR.lock().unwrap() = false;
        assert!(result.is_err());
    }

    #[test]
    fn speak_rejects_empty_text() {
        assert!(speak("   ").is_err());
    }
}
