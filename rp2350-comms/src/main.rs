/*
 * @file main.rs
 * @brief Microcontroller entry point
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

//! FILE: main.rs
//!
//! DESCRIPTION:
//! RP2350 Embedded Rust Embassy UART Control Application.
//!
//! BRIEF:
//! Main application entry point for RP2350 UART-controlled LED using Embassy.
//! Reads UART input and controls GPIO16 LED based on ON/OFF keywords.
//!
//! AUTHOR: Kevin Thomas
//! CREATION DATE: November 30, 2025
//! UPDATE DATE: November 30, 2025

#![no_std]
#![no_main]

mod config;
mod detection;
mod detector;
mod filter;
mod process;

use config::UART_BAUD;
use detection::handle_detection;
use detector::Detector;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{Config, InterruptHandler, Uart};
use panic_halt as _;
use process::process_byte;

// Binds UART0 interrupt to handler.
bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
});

/// Main application entry point.
///
/// # Details
/// Initializes UART and LED, then runs infinite loop reading UART.
/// When "ON" is detected, LED turns on. When "OFF" is detected, LED turns off.
///
/// # Arguments
/// * `_spawner` - Embassy task spawner (unused)
///
/// # Returns
/// Never returns (infinite loop)
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_16, Level::Low);

    let mut config = Config::default();
    config.baudrate = UART_BAUD;
    let mut uart = Uart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH0, p.DMA_CH1, config,
    );

    let _ = uart.write(b"READY\r\n").await;
    run_loop(&mut uart, &mut led).await;
}

/// Runs main event loop
async fn run_loop(
    uart: &mut Uart<'static, embassy_rp::uart::Async>,
    led: &mut Output<'static>,
) -> ! {
    let mut detector = Detector::new();
    let mut buf = [0u8; 1];
    loop {
        if uart.read(&mut buf).await.is_ok() {
            if let Some(det) = process_byte(buf[0], &mut detector) {
                if handle_detection(det) {
                    led.set_high();
                    let _ = uart.write(b"LED ON\r\n").await;
                } else {
                    led.set_low();
                    let _ = uart.write(b"LED OFF\r\n").await;
                }
            }
        }
    }
}
