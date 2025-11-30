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

use std::sync::{Arc, Mutex, Once};
use std::time::Duration;

use anyhow::Result;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, SampleFormat, Stream, StreamConfig, StreamError};
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
    let resolved = negotiate_input_config(&device)?;
    let samples = shared_samples();
    log_channel_notice(resolved.config.channels);
    log_sample_rate_notice(resolved.config.sample_rate.0);
    let stream = build_input_stream(&device, &resolved, samples.clone())?;
    stream.play()?;
    std::thread::sleep(RECORD_DURATION);
    drop(stream);
    let captured = samples.lock().unwrap().clone();
    Ok(resample_to_target_rate(
        &captured,
        resolved.config.sample_rate.0,
        SAMPLE_RATE,
    ))
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

/// Negotiates a stream configuration supported by the selected device.
///
/// Prefers 16 kHz mono audio and falls back to the device default when
/// those constraints are unavailable.
fn negotiate_input_config(device: &Device) -> Result<ResolvedStreamConfig> {
    if let Ok(configs) = device.supported_input_configs() {
        for range in configs {
            if range.channels() == CHANNELS
                && range.min_sample_rate().0 <= SAMPLE_RATE
                && SAMPLE_RATE <= range.max_sample_rate().0
            {
                let supported = range.with_sample_rate(cpal::SampleRate(SAMPLE_RATE));
                let mut config = supported.config();
                config.buffer_size = cpal::BufferSize::Default;
                return Ok(ResolvedStreamConfig {
                    config,
                    sample_format: supported.sample_format(),
                });
            }
        }
    }
    let supported = device
        .default_input_config()
        .map_err(|err| anyhow::anyhow!("Default input config error: {}", err))?;
    let mut config = supported.config();
    config.buffer_size = cpal::BufferSize::Default;
    Ok(ResolvedStreamConfig {
        config,
        sample_format: supported.sample_format(),
    })
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
    resolved: &ResolvedStreamConfig,
    samples: Arc<Mutex<Vec<i16>>>,
) -> Result<Stream> {
    let channel_count = resolved.config.channels.max(1);
    match resolved.sample_format {
        SampleFormat::F32 => {
            let shared = samples.clone();
            device.build_input_stream(
                &resolved.config,
                move |data: &[f32], _: &_| push_samples_from_f32(&shared, data, channel_count),
                log_stream_error,
                None,
            )
        }
        SampleFormat::I16 => {
            let shared = samples.clone();
            device.build_input_stream(
                &resolved.config,
                move |data: &[i16], _: &_| push_samples_from_i16(&shared, data, channel_count),
                log_stream_error,
                None,
            )
        }
        SampleFormat::U16 => {
            let shared = samples;
            device.build_input_stream(
                &resolved.config,
                move |data: &[u16], _: &_| push_samples_from_u16(&shared, data, channel_count),
                log_stream_error,
                None,
            )
        }
        other => {
            return Err(anyhow::anyhow!(
                "Unsupported microphone sample format {:?}; please report",
                other
            ));
        }
    }
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
fn push_samples_from_f32(buffer: &Arc<Mutex<Vec<i16>>>, data: &[f32], channels: u16) {
    push_normalized_samples(buffer, data, channels);
}

fn push_samples_from_i16(buffer: &Arc<Mutex<Vec<i16>>>, data: &[i16], channels: u16) {
    let normalized: Vec<f32> = data
        .iter()
        .map(|&sample| sample as f32 / i16::MAX as f32)
        .collect();
    push_normalized_samples(buffer, &normalized, channels);
}

fn push_samples_from_u16(buffer: &Arc<Mutex<Vec<i16>>>, data: &[u16], channels: u16) {
    let normalized: Vec<f32> = data
        .iter()
        .map(|&sample| (sample as f32 / u16::MAX as f32) * 2.0 - 1.0)
        .collect();
    push_normalized_samples(buffer, &normalized, channels);
}

fn push_normalized_samples(buffer: &Arc<Mutex<Vec<i16>>>, data: &[f32], channels: u16) {
    let ch = channels.max(1) as usize;
    let mut guard = buffer.lock().unwrap();
    for frame in data.chunks(ch) {
        let mono = frame.iter().copied().sum::<f32>() / frame.len().max(1) as f32;
        let clamped = (mono * i16::MAX as f32)
            .round()
            .clamp(i16::MIN as f32, i16::MAX as f32);
        guard.push(clamped as i16);
    }
}

fn resample_to_target_rate(samples: &[i16], from_rate: u32, to_rate: u32) -> Vec<i16> {
    if samples.len() < 2 || from_rate == to_rate {
        return samples.to_vec();
    }
    let ratio = to_rate as f32 / from_rate as f32;
    let new_len = ((samples.len() as f32) * ratio).round().max(1.0) as usize;
    let mut output = Vec::with_capacity(new_len);
    for n in 0..new_len {
        let position = n as f32 / ratio;
        let mut idx = position.floor() as usize;
        if idx >= samples.len() {
            idx = samples.len() - 1;
        }
        let next_idx = (idx + 1).min(samples.len() - 1);
        let frac = position - idx as f32;
        let s0 = samples[idx] as f32;
        let s1 = samples[next_idx] as f32;
        let interpolated = s0 + (s1 - s0) * frac;
        let clamped = interpolated.round().clamp(i16::MIN as f32, i16::MAX as f32);
        output.push(clamped as i16);
    }
    output
}

/// Memoized notice that we had to downmix channels.
static CHANNEL_NOTICE: Once = Once::new();
/// Memoized notice that we had to resample from the hardware's preferred rate.
static SAMPLE_RATE_NOTICE: Once = Once::new();

fn log_channel_notice(actual_channels: u16) {
    if actual_channels == CHANNELS {
        return;
    }
    CHANNEL_NOTICE.call_once(|| {
        eprintln!(
            "ℹ️ Microphone exposes {} channel input; downmixing to mono for Whisper.",
            actual_channels
        );
    });
}

fn log_sample_rate_notice(actual_rate: u32) {
    if actual_rate == SAMPLE_RATE {
        return;
    }
    SAMPLE_RATE_NOTICE.call_once(|| {
        eprintln!(
            "ℹ️ Microphone defaults to {} Hz; resampling to Whisper-friendly 16000 Hz.",
            actual_rate
        );
    });
}

#[derive(Clone)]
struct ResolvedStreamConfig {
    config: StreamConfig,
    sample_format: SampleFormat,
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
    fn shared_samples_starts_empty() {
        let samples = shared_samples();
        assert!(samples.lock().unwrap().is_empty());
    }

    #[test]
    fn push_samples_converts_floats() {
        let samples = shared_samples();
        push_samples_from_f32(&samples, &[0.0, 0.5, -1.0], 1);
        let guard = samples.lock().unwrap();
        assert_eq!(guard.len(), 3);
        assert_eq!(guard[0], 0);
        assert!(guard[1] > 0);
        assert!(guard[2] < 0);
    }

    #[test]
    fn push_samples_downmixes_channels() {
        let samples = shared_samples();
        push_samples_from_f32(&samples, &[1.0, -1.0], 2);
        let guard = samples.lock().unwrap();
        assert_eq!(guard.len(), 1);
        assert_eq!(guard[0], 0);
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

    #[test]
    fn resample_changes_length() {
        let source = vec![0_i16; 1600];
        let resampled = resample_to_target_rate(&source, 16000, 8000);
        assert_eq!(resampled.len(), 800);
    }
}
