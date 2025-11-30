/*
 * @file config.rs
 * @brief Compile-time constants for the UART command example
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

pub const XTAL_FREQ_HZ: u32 = 12_000_000;
pub const RX_BUF_SIZE: usize = 128;
pub const UART_BAUD: u32 = 115_200;
pub const READY_MSG: &[u8] = b"\r\n[MCU] Ready for ON/OFF commands\r\n";
pub const MSG_ON: &[u8] = b"\r\n[MCU] Detected ON, GPIO16=1\r\n";
pub const MSG_OFF: &[u8] = b"\r\n[MCU] Detected OFF, GPIO16=0\r\n";
pub const PRINTABLE_RANGE: core::ops::RangeInclusive<u8> = 32..=126;
