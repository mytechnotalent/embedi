/*
 * @file detector.rs
 * @brief Keyword detection engine
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

//! FILE: detector.rs
//!
//! DESCRIPTION:
//! RP2350 Keyword Detection Engine.
//!
//! BRIEF:
//! Implements sliding window keyword detection for ON/OFF commands.
//! Uses 3-byte buffer to detect ASCII keywords in byte stream.
//!
//! AUTHOR: Kevin Thomas
//! CREATION DATE: November 30, 2025
//! UPDATE DATE: November 30, 2025

/// Detection result enumeration.
///
/// # Details
/// Represents the result of keyword detection.
/// Used to signal ON or OFF command detection.
///
/// # Variants
/// * `On` - "ON" keyword detected
/// * `Off` - "OFF" keyword detected
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Detection {
    On,
    Off,
}

/// Keyword detector with sliding window.
///
/// # Details
/// Maintains 3-byte sliding window buffer for keyword detection.
/// Detects "ON" and "OFF" keywords in incoming byte stream.
///
/// # Fields
/// * `buf` - 3-byte sliding window buffer
#[derive(Default)]
pub struct Detector {
    buf: [u8; 3],
}

/// Public methods for Detector
impl Detector {
    /// Creates new detector instance.
    ///
    /// # Details
    /// Initializes detector with zeroed buffer using Default trait.
    /// Ready to start detecting keywords immediately.
    ///
    /// # Returns
    /// * `Self` - New Detector instance with empty buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Tracks incoming byte and checks for keyword match.
    ///
    /// # Details
    /// Shifts byte into sliding window buffer and checks for ON/OFF patterns.
    /// Returns Some(Detection) if keyword detected, None otherwise.
    ///
    /// # Arguments
    /// * `byte` - Incoming byte to track
    ///
    /// # Returns
    /// * `Option<Detection>` - Some(Detection::On/Off) if matched, None otherwise
    pub fn track(&mut self, byte: u8) -> Option<Detection> {
        self.shift(byte);
        if self.is_on() {
            Some(Detection::On)
        } else if self.is_off() {
            Some(Detection::Off)
        } else {
            None
        }
    }

    /// Shifts byte into sliding window buffer.
    ///
    /// # Details
    /// Moves buffer contents left and adds new byte at end.
    /// Implements FIFO behavior for keyword detection.
    ///
    /// # Arguments
    /// * `byte` - New byte to shift into buffer
    fn shift(&mut self, byte: u8) {
        self.buf[0] = self.buf[1];
        self.buf[1] = self.buf[2];
        self.buf[2] = byte;
    }

    /// Checks if buffer contains "ON" keyword.
    ///
    /// # Details
    /// Detects "ON" pattern in positions 1-2 of sliding window.
    /// Allows any character before "ON" in the buffer.
    ///
    /// # Returns
    /// * `bool` - true if "ON" detected, false otherwise
    fn is_on(&self) -> bool {
        self.buf[1] == b'O' && self.buf[2] == b'N'
    }

    /// Checks if buffer contains "OFF" keyword.
    ///
    /// # Details
    /// Detects "OFF" pattern across full 3-byte buffer.
    /// Requires exact match of complete keyword.
    ///
    /// # Returns
    /// * `bool` - true if "OFF" detected, false otherwise
    fn is_off(&self) -> bool {
        self.buf == [b'O', b'F', b'F']
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_detector() {
        let d = Detector::new();
        assert_eq!(d.buf, [0; 3]);
    }

    #[test]
    fn test_track_on() {
        let mut d = Detector::new();
        assert_eq!(d.track(b'O'), None);
        assert_eq!(d.track(b'N'), Some(Detection::On));
    }

    #[test]
    fn test_track_off() {
        let mut d = Detector::new();
        assert_eq!(d.track(b'O'), None);
        assert_eq!(d.track(b'F'), None);
        assert_eq!(d.track(b'F'), Some(Detection::Off));
    }

    #[test]
    fn test_track_no_match() {
        let mut d = Detector::new();
        assert_eq!(d.track(b'A'), None);
        assert_eq!(d.track(b'B'), None);
    }
}
