/*
 * @file mailbox.rs
 * @brief Shared RX mailbox between the UART IRQ and main loop
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

use crate::config::RX_BUF_SIZE;
use core::cell::{Cell, RefCell};
use critical_section::Mutex;
use heapless::Deque;

type RxQueue = Deque<u8, RX_BUF_SIZE>;

static RX_QUEUE: Mutex<RefCell<RxQueue>> = Mutex::new(RefCell::new(Deque::new()));
static RX_OVERFLOWED: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

/// Attempts to enqueue a byte from the UART interrupt context.
///
/// # Parameters
/// * `byte` - The received UART byte to store.
///
/// # Returns
/// `true` when the byte was stored, `false` if the queue was full.
pub fn push_byte(byte: u8) -> bool {
    critical_section::with(|cs| RX_QUEUE.borrow_ref_mut(cs).push_back(byte).is_ok())
}

/// Pops the next byte for the foreground loop if one exists.
///
/// # Returns
/// `Some` byte when data is available, otherwise `None`.
pub fn pop_byte() -> Option<u8> {
    critical_section::with(|cs| RX_QUEUE.borrow_ref_mut(cs).pop_front())
}

/// Marks that the interrupt handler observed a queue overflow.
pub fn mark_overflow() {
    critical_section::with(|cs| RX_OVERFLOWED.borrow(cs).set(true));
}

/// Returns and clears the overflow flag so the app can log it once.
///
/// # Returns
/// `true` if an overflow occurred since the last call.
pub fn take_overflow() -> bool {
    critical_section::with(|cs| RX_OVERFLOWED.borrow(cs).replace(false))
}
