//! Text-to-speech functionality module.

use std::sync::Mutex;

use anyhow::Result;

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
