/*
 * @file filter.rs
 * @brief ASCII character validation
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

//! FILE: filter.rs
//!
//! DESCRIPTION:
//! RP2350 ASCII Character Validation.
//!
//! BRIEF:
//! Validates incoming bytes as printable ASCII characters.
//! Filters control characters and non-printable bytes.
//!
//! AUTHOR: Kevin Thomas
//! CREATION DATE: November 30, 2025
//! UPDATE DATE: November 30, 2025

/// Minimum printable ASCII value (space character).
const PRINTABLE_MIN: u8 = 32;

/// Maximum printable ASCII value (tilde character).
const PRINTABLE_MAX: u8 = 126;

/// Checks if byte is printable ASCII character.
///
/// # Details
/// Validates byte falls within printable ASCII range (32-126).
/// Rejects control characters and extended ASCII.
///
/// # Arguments
/// * `byte` - Byte to validate
///
/// # Returns
/// * `bool` - true if printable, false otherwise
pub fn is_printable(byte: u8) -> bool {
    (PRINTABLE_MIN..=PRINTABLE_MAX).contains(&byte)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_printable_space() {
        assert!(is_printable(b' '));
    }

    #[test]
    fn test_printable_letter() {
        assert!(is_printable(b'A'));
    }

    #[test]
    fn test_printable_tilde() {
        assert!(is_printable(b'~'));
    }

    #[test]
    fn test_non_printable_null() {
        assert!(!is_printable(0));
    }

    #[test]
    fn test_non_printable_del() {
        assert!(!is_printable(127));
    }
}
