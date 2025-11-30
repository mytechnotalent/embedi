/*
 * @file main.rs
 * @brief UART command example entry point
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

#![no_std]
#![no_main]

mod app;
mod bootinfo;
mod config;
mod hardware;
mod irq;
mod mailbox;

use defmt::*;
use defmt_rtt as _;
use hal::entry;
#[cfg(target_arch = "arm")]
use panic_probe as _;
#[cfg(rp2350)]
pub use rp235x_hal as hal;

/// Boots the UART command example and never returns.
///
/// # Returns
/// `!` because bare-metal firmware does not exit to a caller.
#[entry]
fn main() -> ! {
    info!("UART command example start");
    let mut runtime = hardware::build_runtime();
    irq::enable_uart_irq();
    app::send_ready(&mut runtime.uart);
    app::run(&mut runtime);
}

// Boot ROM metadata lives in bootinfo.rs; keep module referenced so it links in.
