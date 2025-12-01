/*
 * @file hardware.rs
 * @brief Hardware initialization and control
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

//! Hardware initialization and control module.
//!
//! Manages RP2350 peripheral initialization, UART communication, and GPIO control.

use crate::config::UART_BAUD;
use crate::detection::handle_detection;
use crate::detector::Detector;
use crate::process::process_byte;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::{Config, InterruptHandler, Uart};
use embassy_rp::Peri;

// Binds UART0 interrupt to handler.
bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<UART0>;
});

/// Runs the main application logic.
///
/// # Details
/// Initializes RP2350 peripherals, configures UART and LED GPIO,
/// sends ready signal, and enters the main event loop.
///
/// # Arguments
/// * `_spawner` - Embassy task spawner (reserved for future async tasks).
///
/// # Returns
/// * `()` - Never returns (infinite loop).
pub async fn run_application(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut uart = create_uart(p.UART0, p.PIN_0, p.PIN_1, p.DMA_CH0, p.DMA_CH1);
    let mut led = Output::new(p.PIN_16, Level::Low);
    send_ready_signal(&mut uart).await;
    run_event_loop(&mut uart, &mut led).await;
}

/// Creates UART peripheral with configured baud rate.
///
/// # Details
/// Configures UART instance on UART0 with TX on GPIO0, RX on GPIO1,
/// using DMA channels 0 and 1 for efficient data transfer.
///
/// # Arguments
/// * `uart0` - UART0 peripheral.
/// * `tx_pin` - TX GPIO pin (PIN_0).
/// * `rx_pin` - RX GPIO pin (PIN_1).
/// * `dma_ch0` - DMA channel 0 for TX.
/// * `dma_ch1` - DMA channel 1 for RX.
///
/// # Returns
/// * `Uart<'static, Async>` - Configured UART peripheral ready for I/O.
fn create_uart(
    uart0: Peri<'static, embassy_rp::peripherals::UART0>,
    tx_pin: Peri<'static, embassy_rp::peripherals::PIN_0>,
    rx_pin: Peri<'static, embassy_rp::peripherals::PIN_1>,
    dma_ch0: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
    dma_ch1: Peri<'static, embassy_rp::peripherals::DMA_CH1>,
) -> Uart<'static, embassy_rp::uart::Async> {
    let mut config = Config::default();
    config.baudrate = UART_BAUD;
    Uart::new(uart0, tx_pin, rx_pin, Irqs, dma_ch0, dma_ch1, config)
}

/// Sends ready signal over UART.
///
/// # Details
/// Transmits "READY\r\n" to indicate the microcontroller is initialized
/// and waiting for commands. Errors are ignored.
///
/// # Arguments
/// * `uart` - Mutable reference to UART peripheral.
///
/// # Returns
/// * `()` - Returns after sending (or attempting to send) the message.
async fn send_ready_signal(uart: &mut Uart<'static, embassy_rp::uart::Async>) {
    let _ = uart.write(b"READY\r\n").await;
}

/// Runs the main event loop.
///
/// # Details
/// Continuously reads UART bytes, processes them through the keyword detector,
/// and controls the LED based on detected commands. Sends acknowledgment
/// messages over UART for each successful detection.
///
/// # Arguments
/// * `uart` - Mutable reference to UART peripheral.
/// * `led` - Mutable reference to LED GPIO output.
///
/// # Returns
/// * `!` - Never returns (infinite loop).
async fn run_event_loop(
    uart: &mut Uart<'static, embassy_rp::uart::Async>,
    led: &mut Output<'static>,
) -> ! {
    let mut detector = Detector::new();
    let mut buf = [0u8; 1];
    loop {
        read_and_process(uart, led, &mut detector, &mut buf).await;
    }
}

/// Reads one byte and processes detection.
///
/// # Details
/// Reads a single byte from UART, processes it through the keyword detector,
/// and if a command is detected, updates LED state and sends acknowledgment.
///
/// # Arguments
/// * `uart` - UART peripheral for I/O.
/// * `led` - LED output to control.
/// * `detector` - Keyword detector state machine.
/// * `buf` - Single-byte read buffer.
///
/// # Returns
/// * `()` - Returns after processing the byte.
async fn read_and_process(
    uart: &mut Uart<'static, embassy_rp::uart::Async>,
    led: &mut Output<'static>,
    detector: &mut Detector,
    buf: &mut [u8; 1],
) {
    if uart.read(buf).await.is_ok() {
        if let Some(det) = process_byte(buf[0], detector) {
            handle_led_control(uart, led, det).await;
        }
    }
}

/// Handles LED control and UART acknowledgment.
///
/// # Details
/// Updates LED state based on detection result (ON/OFF) and sends
/// confirmation message over UART for user feedback.
///
/// # Arguments
/// * `uart` - UART peripheral for acknowledgment message.
/// * `led` - LED output to control.
/// * `det` - Detection result indicating ON or OFF command.
///
/// # Returns
/// * `()` - Returns after updating LED and sending acknowledgment.
async fn handle_led_control(
    uart: &mut Uart<'static, embassy_rp::uart::Async>,
    led: &mut Output<'static>,
    det: crate::detector::Detection,
) {
    if handle_detection(det) {
        led.set_high();
        let _ = uart.write(b"LED ON\r\n").await;
    } else {
        led.set_low();
        let _ = uart.write(b"LED OFF\r\n").await;
    }
}
