/*
 * @file globals.rs
 * @brief Global ring buffer state shared between IRQ and main loop
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

use crate::ring_buffer::RingBuffer;
use core::cell::{Cell, RefCell};
use critical_section::Mutex;

static RX_BUFFER: Mutex<RefCell<RingBuffer>> = Mutex::new(RefCell::new(RingBuffer::new()));
static RX_OVERFLOWED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

/// Attempts to enqueue a byte into the shared ring buffer.
///
/// # Parameters
/// * `byte` - The UART byte to push.
///
/// # Returns
/// `true` when the byte was stored; `false` if the buffer was full.
pub fn push_byte(byte: u8) -> bool {
    critical_section::with(|cs| RX_BUFFER.borrow_ref_mut(cs).push(byte))
}

/// Pops one byte from the shared ring buffer if available.
///
/// # Returns
/// `Some` byte when data exists, otherwise `None`.
pub fn pop_byte() -> Option<u8> {
    critical_section::with(|cs| RX_BUFFER.borrow_ref_mut(cs).pop())
}

/// Flags that the RX buffer overflowed inside the interrupt context.
pub fn mark_overflow() {
    critical_section::with(|cs| RX_OVERFLOWED.borrow(cs).set(true));
}

/// Returns and clears the overflow flag.
///
/// # Returns
/// `true` if an overflow occurred since the last call.
pub fn take_overflow() -> bool {
    critical_section::with(|cs| RX_OVERFLOWED.borrow(cs).replace(false))
}
