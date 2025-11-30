/*
 * @file irq.rs
 * @brief UART interrupt handler and NVIC helpers
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

use crate::globals;
use crate::hal;
use crate::hal::pac::interrupt;

/// UART0 interrupt handler that drains the RX FIFO into the software buffer.
///
/// # Safety
/// Must be registered exactly once by the startup code.
#[interrupt]
unsafe fn UART0_IRQ() {
    let uart = uart_regs();
    drain_fifo(uart);
    clear_interrupts(uart);
}

/// Enables the UART0 interrupt at the NVIC level when supported.
pub fn enable_uart_irq() {
    unmask_uart_irq();
}

/// Unmasks the UART0 interrupt in the NVIC on ARM targets.
#[cfg(target_arch = "arm")]
fn unmask_uart_irq() {
    unsafe {
        cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::UART0_IRQ);
    }
}

/// Moves pending RX bytes into the global ring buffer.
///
/// # Parameters
/// * `uart` - Pointer to the UART register block.
fn drain_fifo(uart: &hal::pac::uart0::RegisterBlock) {
    while uart.uartfr().read().rxfe().bit_is_clear() {
        let byte = uart.uartdr().read().data().bits();
        if !globals::push_byte(byte) {
            globals::mark_overflow();
        }
    }
}

/// Clears the interrupt status bits that triggered this handler.
///
/// # Parameters
/// * `uart` - Pointer to the UART register block.
fn clear_interrupts(uart: &hal::pac::uart0::RegisterBlock) {
    uart.uarticr().write(|w| {
        w.rxic().bit(true);
        w.rtic().bit(true);
        w
    });
}

/// Returns a shared reference to the UART0 register block.
///
/// # Safety
/// Accessing peripheral registers is inherently unsafe; this helper confines the
/// raw-pointer dereference to a single location.
fn uart_regs() -> &'static hal::pac::uart0::RegisterBlock {
    unsafe { &*hal::pac::UART0::ptr() }
}
