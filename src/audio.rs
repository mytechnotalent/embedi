/*
 * @file audio.rs
 * @brief Audio capture and WAV helpers for Embedi
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

//! Audio recording and playback functionality.
//!
//! This module handles microphone input recording using CPAL and WAV file operations.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Result;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, Stream, StreamConfig, StreamError};
use hound::{WavSpec, WavWriter};

/// Sample rate for audio recording (16kHz).
///
/// Value is expressed in Hertz and matches Whisper's preferred input rate.
const SAMPLE_RATE: u32 = 16000;

/// Number of audio channels (mono).
///
/// Mono input keeps bandwidth low while remaining compatible with Whisper.
const CHANNELS: u16 = 1;

/// Bits per sample for WAV encoding.
///
/// Using 16-bit samples keeps files compact but high-quality.
const BITS_PER_SAMPLE: u16 = 16;

/// Amount of time to capture audio for each request.
///
/// Recorded audio is trimmed or padded downstream by Whisper if needed.
const RECORD_DURATION: Duration = Duration::from_secs(5);

/// Records audio from the default input device for a fixed duration.
///
/// # Returns
/// A vector of 16-bit PCM audio samples.
///
/// # Errors
/// Returns an error if:
/// - No input device is available
/// - The audio stream cannot be created
/// - Recording fails
///
/// # Examples
/// ```no_run
/// use embedi::audio::record_audio;
///
/// let samples = record_audio().expect("Failed to record audio");
/// let _sample_count = samples.len();
/// ```
pub fn record_audio() -> Result<Vec<i16>> {
    let device = default_input_device()?;
    let config = input_config();
    let samples = shared_samples();
    let stream = build_input_stream(&device, &config, samples.clone())?;
    stream.play()?;
    std::thread::sleep(RECORD_DURATION);
    drop(stream);
    Ok(samples.lock().unwrap().clone())
}

/// Saves audio samples to a WAV file.
///
/// # Parameters
/// * `path` - Destination path for the generated WAV file.
/// * `samples` - Slice of signed 16-bit PCM audio frames to persist.
///
/// # Returns
/// `Ok(())` if the file was successfully written.
///
/// # Errors
/// Returns an error if the file cannot be created or written.
///
/// # Examples
/// ```no_run
/// use embedi::audio::{record_audio, save_wav};
///
/// let samples = record_audio().expect("Failed to record");
/// save_wav("output.wav", &samples).expect("Failed to save WAV");
/// ```
pub fn save_wav(path: &str, samples: &[i16]) -> Result<()> {
    let spec = WavSpec {
        channels: CHANNELS,
        sample_rate: SAMPLE_RATE,
        bits_per_sample: BITS_PER_SAMPLE,
        sample_format: hound::SampleFormat::Int,
    };

    let mut writer = WavWriter::create(path, spec)?;
    for &sample in samples {
        writer.write_sample(sample)?;
    }
    writer.finalize()?;
    Ok(())
}

/// Locates the system default input device.
///
/// # Returns
/// A CPAL [`Device`] ready for stream construction.
///
/// # Errors
/// Returns an error when the user has no available microphone.
fn default_input_device() -> Result<Device> {
    cpal::default_host()
        .default_input_device()
        .ok_or_else(|| anyhow::anyhow!("No input device"))
}

/// Builds the CPAL stream configuration used by the recorder.
///
/// The configuration uses mono audio, a 16 kHz sample rate, and a default buffer.
///
/// # Returns
/// A [`StreamConfig`] tailored to Whisper-friendly audio settings.
fn input_config() -> StreamConfig {
    StreamConfig {
        channels: CHANNELS,
        sample_rate: cpal::SampleRate(SAMPLE_RATE),
        buffer_size: cpal::BufferSize::Default,
    }
}

/// Creates the shared buffer that accumulates captured samples.
///
/// # Returns
/// A thread-safe vector suitable for use inside CPAL callbacks.
fn shared_samples() -> Arc<Mutex<Vec<i16>>> {
    Arc::new(Mutex::new(Vec::new()))
}

/// Builds and configures the CPAL input stream.
///
/// # Parameters
/// * `device` - The input device to capture from.
/// * `config` - The stream configuration (channels/rate/buffer).
/// * `samples` - Shared buffer that receives converted samples.
///
/// # Returns
/// A started-but-paused [`Stream`] the caller can `play`.
///
/// # Errors
/// Returns any stream-construction issues wrapped in [`anyhow::Error`].
fn build_input_stream(
    device: &Device,
    config: &StreamConfig,
    samples: Arc<Mutex<Vec<i16>>>,
) -> Result<Stream> {
    let shared = samples.clone();
    device
        .build_input_stream(
            config,
            move |data: &[f32], _: &_| push_samples(&shared, data),
            log_stream_error,
            None,
        )
        .map_err(|err| anyhow::anyhow!(err))
}

/// Converts floating-point frames into 16-bit PCM and appends them to the buffer.
///
/// # Parameters
/// * `buffer` - Shared sample accumulator.
/// * `data` - The latest interleaved floating-point frames from CPAL.
///
/// # Returns
/// Nothing; the buffer is mutated in place.
fn push_samples(buffer: &Arc<Mutex<Vec<i16>>>, data: &[f32]) {
    let mut guard = buffer.lock().unwrap();
    for &sample in data {
        guard.push((sample * i16::MAX as f32) as i16);
    }
}

/// Logs recoverable stream errors emitted by CPAL.
///
/// # Parameters
/// * `error` - The error instance to display.
///
/// # Returns
/// Nothing; the error is printed to stderr for observability.
fn log_stream_error(error: StreamError) {
    eprintln!("Audio stream error: {}", error);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::Path;

    #[test]
    fn input_config_matches_constants() {
        let config = input_config();
        assert_eq!(config.channels, CHANNELS);
        assert_eq!(config.sample_rate.0, SAMPLE_RATE);
    }

    #[test]
    fn shared_samples_starts_empty() {
        let samples = shared_samples();
        assert!(samples.lock().unwrap().is_empty());
    }

    #[test]
    fn push_samples_converts_floats() {
        let samples = shared_samples();
        push_samples(&samples, &[0.0, 0.5, -1.0]);
        let guard = samples.lock().unwrap();
        assert_eq!(guard.len(), 3);
        assert_eq!(guard[0], 0);
        assert!(guard[1] > 0);
        assert!(guard[2] < 0);
    }

    #[test]
    fn save_wav_writes_file() {
        let temp_path = std::env::temp_dir().join("rig_audio_test.wav");
        let temp_str = temp_path.to_string_lossy().to_string();
        let samples = vec![0_i16, i16::MAX / 2, -i16::MAX / 2];
        save_wav(&temp_str, &samples).expect("save wav");
        assert!(Path::new(&temp_str).exists());
        fs::remove_file(temp_path).ok();
    }
}
