/*
 * @file process.rs
 * @brief Byte processing pipeline
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

//! FILE: process.rs
//!
//! DESCRIPTION:
//! RP2350 Byte Processing Pipeline.
//!
//! BRIEF:
//! Processes incoming UART bytes through validation and detection.
//! Filters non-printable characters and feeds valid bytes to detector.
//!
//! AUTHOR: Kevin Thomas
//! CREATION DATE: November 30, 2025
//! UPDATE DATE: November 30, 2025

use crate::detector::Detector;
use crate::filter::is_printable;

/// Processes single byte through filter and detector.
///
/// # Details
/// Validates byte is printable ASCII, then tracks in detector.
/// Returns None for non-printable bytes or non-matching keywords.
///
/// # Arguments
/// * `byte` - Incoming byte to process
/// * `detector` - Mutable reference to keyword detector
///
/// # Returns
/// * `Option<Detection>` - Some(Detection) if keyword detected, None otherwise
pub fn process_byte(byte: u8, detector: &mut Detector) -> Option<crate::detector::Detection> {
    if !is_printable(byte) {
        return None;
    }
    detector.track(byte)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detector::Detection;

    #[test]
    fn test_process_printable_on() {
        let mut detector = Detector::new();
        assert_eq!(process_byte(b'O', &mut detector), None);
        assert_eq!(process_byte(b'N', &mut detector), Some(Detection::On));
    }

    #[test]
    fn test_process_printable_off() {
        let mut detector = Detector::new();
        assert_eq!(process_byte(b'O', &mut detector), None);
        assert_eq!(process_byte(b'F', &mut detector), None);
        assert_eq!(process_byte(b'F', &mut detector), Some(Detection::Off));
    }

    #[test]
    fn test_process_non_printable() {
        let mut detector = Detector::new();
        assert_eq!(process_byte(0, &mut detector), None);
        assert_eq!(process_byte(127, &mut detector), None);
    }

    #[test]
    fn test_process_no_match() {
        let mut detector = Detector::new();
        assert_eq!(process_byte(b'A', &mut detector), None);
        assert_eq!(process_byte(b'B', &mut detector), None);
    }
}
