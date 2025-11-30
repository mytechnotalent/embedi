/*
 * @file app.rs
 * @brief Application loop and UART command detection logic
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

use crate::config::{MSG_OFF, MSG_ON, PRINTABLE_RANGE, READY_MSG};
use crate::hardware::{AppUart, Runtime};
use crate::mailbox;
use core::hint::spin_loop;
use defmt::{info, warn};
use embedded_hal::digital::OutputPin;

/// Sends the ready banner so the host knows the MCU is alive.
///
/// # Parameters
/// * `uart` - The UART used to transmit the message.
pub fn send_ready(uart: &mut AppUart) {
    uart.write_full_blocking(READY_MSG);
}

/// Runs the infinite UART command processing loop.
///
/// # Parameters
/// * `runtime` - Aggregated peripherals required by the loop.
///
/// # Returns
/// `!` because bare-metal firmware never exits.
pub fn run(runtime: &mut Runtime) -> ! {
    let mut detector = Detector::new();
    loop {
        process_bytes(runtime, &mut detector);
        report_overflow();
        spin_loop();
    }
}

/// Pulls bytes from the shared FIFO and dispatches detections.
///
/// # Parameters
/// * `runtime` - Provides UART and LED access.
/// * `detector` - Tracks recent characters for keyword detection.
fn process_bytes(runtime: &mut Runtime, detector: &mut Detector) {
    while let Some(byte) = mailbox::pop_byte() {
        echo_byte(&mut runtime.uart, byte);
        if PRINTABLE_RANGE.contains(&byte)
            && let Some(hit) = detector.track(byte)
        {
            act_on_detection(runtime, hit);
        }
    }
}

/// Mirrors a received byte back over UART.
///
/// # Parameters
/// * `uart` - UART handle used for transmission.
/// * `byte` - The byte that should be echoed.
fn echo_byte(uart: &mut AppUart, byte: u8) {
    uart.write_full_blocking(&[byte]);
}

/// Applies side effects for detected "ON"/"OFF" phrases.
///
/// # Parameters
/// * `runtime` - Provides LED and UART access.
/// * `detection` - Indicates which keyword fired.
fn act_on_detection(runtime: &mut Runtime, detection: Detection) {
    match detection {
        Detection::On => {
            runtime.led.set_high().ok();
            runtime.uart.write_full_blocking(MSG_ON);
            info!("Received ON - setting GPIO16 HIGH");
        }
        Detection::Off => {
            runtime.led.set_low().ok();
            runtime.uart.write_full_blocking(MSG_OFF);
            info!("Received OFF - setting GPIO16 LOW");
        }
    }
}

/// Emits a log warning if the RX FIFO overflowed.
fn report_overflow() {
    if mailbox::take_overflow() {
        warn!("RX buffer was full, dropped incoming bytes");
    }
}

/// Sliding window tracker that detects "ON"/"OFF" sequences.
struct Detector {
    buf: [u8; 3],
}

/// Implements the finite-state logic that powers the detector.
impl Detector {
    /// Creates a fresh detector with an empty comparison buffer.
    fn new() -> Self {
        Self { buf: [0; 3] }
    }

    /// Feeds one byte into the rolling window and reports detections.
    ///
    /// # Parameters
    /// * `byte` - The newly received character.
    ///
    /// # Returns
    /// `Some` detection when "ON"/"OFF" was observed, otherwise `None`.
    fn track(&mut self, byte: u8) -> Option<Detection> {
        self.shift(byte);
        if self.buf[1] == b'O' && self.buf[2] == b'N' {
            return Some(Detection::On);
        }
        if self.buf == [b'O', b'F', b'F'] {
            return Some(Detection::Off);
        }
        None
    }

    /// Advances the sliding window by one byte.
    ///
    /// # Parameters
    /// * `byte` - The latest character to append.
    fn shift(&mut self, byte: u8) {
        self.buf[0] = self.buf[1];
        self.buf[1] = self.buf[2];
        self.buf[2] = byte;
    }
}

/// Keywords recognized by the detector.
enum Detection {
    On,
    Off,
}
