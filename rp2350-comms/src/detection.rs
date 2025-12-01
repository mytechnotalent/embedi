/*
 * @file detection.rs
 * @brief Detection result handler
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

//! FILE: detection.rs
//!
//! DESCRIPTION:
//! RP2350 Detection Result Handler.
//!
//! BRIEF:
//! Converts Detection enum to boolean LED state.
//! Handles ON/OFF detection results for GPIO control.
//!
//! AUTHOR: Kevin Thomas
//! CREATION DATE: November 30, 2025
//! UPDATE DATE: November 30, 2025

use crate::detector::Detection;

/// Handles detection result and converts to boolean.
///
/// # Details
/// Converts Detection enum variant to boolean value for LED control.
/// Returns true for Detection::On, false for Detection::Off.
///
/// # Arguments
/// * `det` - Detection enum variant (On or Off)
///
/// # Returns
/// * `bool` - true if On, false if Off
pub fn handle_detection(det: Detection) -> bool {
    matches!(det, Detection::On)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_handle_detection_on() {
        assert!(handle_detection(Detection::On));
    }

    #[test]
    fn test_handle_detection_off() {
        assert!(!handle_detection(Detection::Off));
    }
}
