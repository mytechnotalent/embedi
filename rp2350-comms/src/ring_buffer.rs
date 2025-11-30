/*
 * @file ring_buffer.rs
 * @brief Fixed-size byte ring buffer implementation
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

/// Fixed-size FIFO used between the interrupt and main loop.
pub struct RingBuffer {
    buf: [u8; RX_BUF_SIZE],
    head: usize,
    tail: usize,
}

/// Methods for the ring buffer.
impl RingBuffer {
    /// Creates an empty ring buffer.
    pub const fn new() -> Self {
        Self {
            buf: [0; RX_BUF_SIZE],
            head: 0,
            tail: 0,
        }
    }

    /// Attempts to push one byte into the buffer.
    ///
    /// # Parameters
    /// * `byte` - The byte to enqueue.
    ///
    /// # Returns
    /// `true` if the byte was written; `false` if the buffer was full.
    pub fn push(&mut self, byte: u8) -> bool {
        let next = Self::advance(self.head);
        if next == self.tail {
            false
        } else {
            self.buf[self.head] = byte;
            self.head = next;
            true
        }
    }

    /// Pops the oldest byte from the buffer, if any.
    ///
    /// # Returns
    /// `Some` byte when data exists, otherwise `None`.
    pub fn pop(&mut self) -> Option<u8> {
        if self.is_empty() {
            None
        } else {
            let byte = self.buf[self.tail];
            self.tail = Self::advance(self.tail);
            Some(byte)
        }
    }

    /// Calculates the next index modulo the buffer size.
    ///
    /// # Parameters
    /// * `idx` - Current index to advance.
    ///
    /// # Returns
    /// Next index, wrapping to zero at the end.
    const fn advance(idx: usize) -> usize {
        let next = idx + 1;
        if next == RX_BUF_SIZE { 0 } else { next }
    }

    /// Returns `true` when no bytes are stored in the buffer.
    fn is_empty(&self) -> bool {
        self.head == self.tail
    }
}
