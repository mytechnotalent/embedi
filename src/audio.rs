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

use anyhow::Result;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, SampleFormat, Stream, StreamConfig, StreamError};
use hound::{WavSpec, WavWriter};
use std::sync::{Arc, Mutex, Once};
use std::time::Duration;

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
/// # Details
/// Queries the system's default audio host for the default input device.
/// This is typically the microphone selected in system preferences.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `Ok(Device)` - A CPAL device ready for stream construction.
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
/// # Details
/// Prefers 16 kHz mono audio and falls back to the device default when
/// those constraints are unavailable.
///
/// # Arguments
/// * `device` - The input device to negotiate configuration with.
///
/// # Returns
/// * `Ok(ResolvedStreamConfig)` - A configuration compatible with the device.
///
/// # Errors
/// Returns an error if no suitable configuration can be found.
fn negotiate_input_config(device: &Device) -> Result<ResolvedStreamConfig> {
    if let Some(resolved) = find_matching_config(device) {
        return Ok(resolved);
    }
    fallback_to_default_config(device)
}

/// Searches for a device configuration matching our preferred constraints.
///
/// # Details
/// Iterates through supported configurations looking for 16 kHz mono audio.
///
/// # Arguments
/// * `device` - The input device to query.
///
/// # Returns
/// * `Some(ResolvedStreamConfig)` - A matching configuration if available.
/// * `None` - No configuration matches our constraints.
fn find_matching_config(device: &Device) -> Option<ResolvedStreamConfig> {
    let configs = device.supported_input_configs().ok()?;
    for range in configs {
        if is_config_acceptable(&range) {
            return Some(build_resolved_config(&range));
        }
    }
    None
}

/// Checks if a configuration range meets our audio requirements.
///
/// # Details
/// Verifies that the range supports mono audio and that 16 kHz falls
/// within the supported sample rate range.
///
/// # Arguments
/// * `range` - The configuration range to evaluate.
///
/// # Returns
/// * `bool` - `true` when the range supports 16 kHz mono audio.
fn is_config_acceptable(range: &cpal::SupportedStreamConfigRange) -> bool {
    range.channels() == CHANNELS
        && range.min_sample_rate().0 <= SAMPLE_RATE
        && SAMPLE_RATE <= range.max_sample_rate().0
}

/// Constructs a resolved config from a supported configuration range.
///
/// # Details
/// Sets the sample rate to 16 kHz and uses the default buffer size.
/// Preserves the sample format from the configuration range.
///
/// # Arguments
/// * `range` - The configuration range to use.
///
/// # Returns
/// * `ResolvedStreamConfig` - Configuration set to 16 kHz with default buffer.
fn build_resolved_config(range: &cpal::SupportedStreamConfigRange) -> ResolvedStreamConfig {
    let supported = range.with_sample_rate(cpal::SampleRate(SAMPLE_RATE));
    let mut config = supported.config();
    config.buffer_size = cpal::BufferSize::Default;
    ResolvedStreamConfig {
        config,
        sample_format: supported.sample_format(),
    }
}

/// Falls back to the device's default configuration.
///
/// # Details
/// Uses the device's native configuration when 16 kHz mono is unavailable.
/// Sets buffer size to default for optimal latency.
///
/// # Arguments
/// * `device` - The input device to query.
///
/// # Returns
/// * `Ok(ResolvedStreamConfig)` - The default configuration.
///
/// # Errors
/// Returns an error if the device has no default configuration.
fn fallback_to_default_config(device: &Device) -> Result<ResolvedStreamConfig> {
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
/// # Details
/// Wraps an empty vector in Arc<Mutex<>> for thread-safe access from
/// CPAL's audio callback thread.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `Arc<Mutex<Vec<i16>>>` - A thread-safe vector suitable for use inside CPAL callbacks.
fn shared_samples() -> Arc<Mutex<Vec<i16>>> {
    Arc::new(Mutex::new(Vec::new()))
}

/// Builds and configures the CPAL input stream.
///
/// # Details
/// Determines channel count from configuration and dispatches to the
/// appropriate stream builder based on sample format.
///
/// # Arguments
/// * `device` - The input device to capture from.
/// * `resolved` - The stream configuration (channels/rate/buffer).
/// * `samples` - Shared buffer that receives converted samples.
///
/// # Returns
/// * `Ok(Stream)` - A started-but-paused stream the caller can play.
///
/// # Errors
/// Returns any stream-construction issues wrapped in anyhow::Error.
fn build_input_stream(
    device: &Device,
    resolved: &ResolvedStreamConfig,
    samples: Arc<Mutex<Vec<i16>>>,
) -> Result<Stream> {
    let channel_count = resolved.config.channels.max(1);
    dispatch_stream_builder(device, resolved, samples, channel_count)
}

/// Dispatches to the appropriate stream builder based on sample format.
///
/// # Details
/// Routes to format-specific stream builders (f32, i16, u16) based on the
/// detected sample format. Unsupported formats return an error.
///
/// # Arguments
/// * `device` - The input device.
/// * `resolved` - The stream configuration.
/// * `samples` - Shared sample buffer.
/// * `channel_count` - Number of audio channels.
///
/// # Returns
/// * `Ok(Stream)` - Configured input stream.
///
/// # Errors
/// Returns an error for unsupported sample formats.
fn dispatch_stream_builder(
    device: &Device,
    resolved: &ResolvedStreamConfig,
    samples: Arc<Mutex<Vec<i16>>>,
    channel_count: u16,
) -> Result<Stream> {
    let result = match resolved.sample_format {
        SampleFormat::F32 => build_f32_stream(device, resolved, samples, channel_count),
        SampleFormat::I16 => build_i16_stream(device, resolved, samples, channel_count),
        SampleFormat::U16 => build_u16_stream(device, resolved, samples, channel_count),
        other => {
            return Err(anyhow::anyhow!(
                "Unsupported microphone sample format {:?}; please report",
                other
            ));
        }
    };
    result.map_err(|err| anyhow::anyhow!(err))
}

/// Builds an input stream for f32 sample format.
///
/// # Details
/// Creates a CPAL input stream configured to receive floating-point samples.
/// Converts incoming f32 data to i16 via the push_samples_from_f32 callback.
///
/// # Arguments
/// * `device` - The input device to capture from.
/// * `resolved` - The stream configuration (channels/rate/buffer).
/// * `samples` - Shared buffer to accumulate converted samples.
/// * `channel_count` - Number of audio channels for downmixing.
///
/// # Returns
/// * `Ok(Stream)` - Configured input stream ready for playback.
///
/// # Errors
/// Returns `BuildStreamError` if stream construction fails.
fn build_f32_stream(
    device: &Device,
    resolved: &ResolvedStreamConfig,
    samples: Arc<Mutex<Vec<i16>>>,
    channel_count: u16,
) -> Result<Stream, cpal::BuildStreamError> {
    device.build_input_stream(
        &resolved.config,
        move |data: &[f32], _: &_| push_samples_from_f32(&samples, data, channel_count),
        log_stream_error,
        None,
    )
}

/// Builds an input stream for i16 sample format.
///
/// # Details
/// Creates a CPAL input stream configured to receive signed 16-bit PCM samples.
/// Normalizes incoming i16 data through push_samples_from_i16 callback.
///
/// # Arguments
/// * `device` - The input device to capture from.
/// * `resolved` - The stream configuration (channels/rate/buffer).
/// * `samples` - Shared buffer to accumulate converted samples.
/// * `channel_count` - Number of audio channels for downmixing.
///
/// # Returns
/// * `Ok(Stream)` - Configured input stream ready for playback.
///
/// # Errors
/// Returns `BuildStreamError` if stream construction fails.
fn build_i16_stream(
    device: &Device,
    resolved: &ResolvedStreamConfig,
    samples: Arc<Mutex<Vec<i16>>>,
    channel_count: u16,
) -> Result<Stream, cpal::BuildStreamError> {
    device.build_input_stream(
        &resolved.config,
        move |data: &[i16], _: &_| push_samples_from_i16(&samples, data, channel_count),
        log_stream_error,
        None,
    )
}

/// Builds an input stream for u16 sample format.
///
/// # Details
/// Creates a CPAL input stream configured to receive unsigned 16-bit PCM samples.
/// Converts incoming u16 data to signed range via push_samples_from_u16 callback.
///
/// # Arguments
/// * `device` - The input device to capture from.
/// * `resolved` - The stream configuration (channels/rate/buffer).
/// * `samples` - Shared buffer to accumulate converted samples.
/// * `channel_count` - Number of audio channels for downmixing.
///
/// # Returns
/// * `Ok(Stream)` - Configured input stream ready for playback.
///
/// # Errors
/// Returns `BuildStreamError` if stream construction fails.
fn build_u16_stream(
    device: &Device,
    resolved: &ResolvedStreamConfig,
    samples: Arc<Mutex<Vec<i16>>>,
    channel_count: u16,
) -> Result<Stream, cpal::BuildStreamError> {
    device.build_input_stream(
        &resolved.config,
        move |data: &[u16], _: &_| push_samples_from_u16(&samples, data, channel_count),
        log_stream_error,
        None,
    )
}

/// Converts floating-point frames into 16-bit PCM and appends them to the buffer.
///
/// # Details
/// Delegates to push_normalized_samples since f32 data is already normalized.
/// This is the callback used for devices with f32 sample format.
///
/// # Arguments
/// * `buffer` - Shared sample accumulator.
/// * `data` - The latest interleaved floating-point frames from CPAL.
/// * `channels` - Number of audio channels.
///
/// # Returns
/// * `()` - Nothing; the buffer is mutated in place.
fn push_samples_from_f32(buffer: &Arc<Mutex<Vec<i16>>>, data: &[f32], channels: u16) {
    push_normalized_samples(buffer, data, channels);
}

/// Converts signed 16-bit PCM frames and appends them to the buffer.
///
/// # Details
/// Normalizes i16 samples to the [-1.0, 1.0] range before processing.
/// This is the callback used for devices with i16 sample format.
///
/// # Arguments
/// * `buffer` - Shared sample accumulator.
/// * `data` - The latest interleaved i16 frames from CPAL.
/// * `channels` - Number of audio channels.
///
/// # Returns
/// * `()` - Nothing; the buffer is mutated in place.
fn push_samples_from_i16(buffer: &Arc<Mutex<Vec<i16>>>, data: &[i16], channels: u16) {
    let normalized: Vec<f32> = data
        .iter()
        .map(|&sample| sample as f32 / i16::MAX as f32)
        .collect();
    push_normalized_samples(buffer, &normalized, channels);
}

/// Converts unsigned 16-bit PCM frames and appends them to the buffer.
///
/// # Details
/// Normalizes u16 samples from [0, 65535] range to [-1.0, 1.0] before processing.
/// This is the callback used for devices with u16 sample format.
///
/// # Arguments
/// * `buffer` - Shared sample accumulator.
/// * `data` - The latest interleaved u16 frames from CPAL.
/// * `channels` - Number of audio channels.
///
/// # Returns
/// * `()` - Nothing; the buffer is mutated in place.
fn push_samples_from_u16(buffer: &Arc<Mutex<Vec<i16>>>, data: &[u16], channels: u16) {
    let normalized: Vec<f32> = data
        .iter()
        .map(|&sample| (sample as f32 / u16::MAX as f32) * 2.0 - 1.0)
        .collect();
    push_normalized_samples(buffer, &normalized, channels);
}

/// Processes normalized samples by downmixing to mono and converting to i16.
///
/// # Details
/// Averages channels to mono, scales to i16 range, clamps to prevent overflow,
/// and appends to the shared buffer. Handles multi-channel input gracefully.
///
/// # Arguments
/// * `buffer` - Shared sample accumulator.
/// * `data` - Normalized floating-point samples in [-1.0, 1.0] range.
/// * `channels` - Number of audio channels.
///
/// # Returns
/// * `()` - Nothing; the buffer is mutated in place.
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

/// Resamples audio from one sample rate to another using linear interpolation.
///
/// # Details
/// Returns a copy if sample rates match or input is too short. Otherwise,
/// calculates the resampling ratio and interpolates samples to the target rate.
///
/// # Arguments
/// * `samples` - Source audio samples at the original sample rate.
/// * `from_rate` - Original sample rate in Hz.
/// * `to_rate` - Target sample rate in Hz.
///
/// # Returns
/// * `Vec<i16>` - Resampled audio at the target rate.
fn resample_to_target_rate(samples: &[i16], from_rate: u32, to_rate: u32) -> Vec<i16> {
    if samples.len() < 2 || from_rate == to_rate {
        return samples.to_vec();
    }
    let ratio = to_rate as f32 / from_rate as f32;
    let new_len = calculate_output_length(samples.len(), ratio);
    interpolate_samples(samples, ratio, new_len)
}

/// Calculates the output length after resampling.
///
/// # Details
/// Multiplies input length by the resampling ratio, rounds to nearest integer,
/// and ensures at least one sample in the output.
///
/// # Arguments
/// * `input_len` - Number of input samples.
/// * `ratio` - Resampling ratio (to_rate / from_rate).
///
/// # Returns
/// * `usize` - Number of output samples after resampling.
fn calculate_output_length(input_len: usize, ratio: f32) -> usize {
    ((input_len as f32) * ratio).round().max(1.0) as usize
}

/// Interpolates samples using linear interpolation.
///
/// # Details
/// Generates output samples by interpolating between input samples at positions
/// determined by the resampling ratio. Pre-allocates output vector for efficiency.
///
/// # Arguments
/// * `samples` - Source audio samples to interpolate from.
/// * `ratio` - Resampling ratio (to_rate / from_rate).
/// * `output_len` - Number of output samples to generate.
///
/// # Returns
/// * `Vec<i16>` - Interpolated audio samples.
fn interpolate_samples(samples: &[i16], ratio: f32, output_len: usize) -> Vec<i16> {
    let mut output = Vec::with_capacity(output_len);
    for n in 0..output_len {
        let sample = interpolate_at_position(samples, n as f32 / ratio);
        output.push(sample);
    }
    output
}

/// Performs linear interpolation at a specific position.
///
/// # Details
/// Calculates bounded indices, performs linear interpolation between adjacent
/// samples, and clamps the result to i16 range to prevent overflow.
///
/// # Arguments
/// * `samples` - Source audio samples.
/// * `position` - Fractional position in the input array.
///
/// # Returns
/// * `i16` - Interpolated sample value clamped to valid range.
fn interpolate_at_position(samples: &[i16], position: f32) -> i16 {
    let idx = bound_index(position.floor() as usize, samples.len());
    let next_idx = (idx + 1).min(samples.len() - 1);
    let frac = position - idx as f32;
    let interpolated = lerp(samples[idx] as f32, samples[next_idx] as f32, frac);
    interpolated.round().clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

/// Ensures an index stays within bounds.
///
/// # Details
/// Clamps the index to the last valid position when it exceeds array length,
/// preventing out-of-bounds access during interpolation.
///
/// # Arguments
/// * `idx` - The index to bound.
/// * `len` - The array length.
///
/// # Returns
/// * `usize` - The bounded index, guaranteed to be < len.
fn bound_index(idx: usize, len: usize) -> usize {
    if idx >= len { len - 1 } else { idx }
}

/// Linear interpolation between two values.
///
/// # Details
/// Computes a + (b - a) * t, which smoothly blends between a (when t=0)
/// and b (when t=1). Used for audio sample interpolation.
///
/// # Arguments
/// * `a` - Starting value.
/// * `b` - Ending value.
/// * `t` - Interpolation factor in [0.0, 1.0].
///
/// # Returns
/// * `f32` - Interpolated value between a and b.
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Memoized notice that we had to downmix channels.
static CHANNEL_NOTICE: Once = Once::new();
/// Memoized notice that we had to resample from the hardware's preferred rate.
static SAMPLE_RATE_NOTICE: Once = Once::new();

/// Logs a one-time notice when channel downmixing occurs.
///
/// # Details
/// Uses Once to ensure the message is printed only on first mismatch.
/// Informs the user when multi-channel audio is being converted to mono.
///
/// # Arguments
/// * `actual_channels` - Number of channels reported by the device.
///
/// # Returns
/// * `()` - Nothing; prints to stderr if downmixing is needed.
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

/// Logs a one-time notice when sample rate conversion occurs.
///
/// # Details
/// Uses Once to ensure the message is printed only on first mismatch.
/// Informs the user when audio is being resampled to 16 kHz for Whisper.
///
/// # Arguments
/// * `actual_rate` - Sample rate in Hz reported by the device.
///
/// # Returns
/// * `()` - Nothing; prints to stderr if resampling is needed.
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

/// Resolved stream configuration pairing config with sample format.
///
/// # Details
/// Bundles the CPAL stream configuration with the detected sample format
/// so downstream code can dispatch to the appropriate callback handler.
#[derive(Clone)]
struct ResolvedStreamConfig {
    config: StreamConfig,
    sample_format: SampleFormat,
}

/// Logs recoverable stream errors emitted by CPAL.
///
/// # Details
/// This callback is invoked by CPAL when non-fatal stream errors occur.
/// Errors are printed to stderr for debugging and observability.
///
/// # Arguments
/// * `error` - The error instance to display.
///
/// # Returns
/// * `()` - Nothing; the error is printed to stderr for observability.
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
