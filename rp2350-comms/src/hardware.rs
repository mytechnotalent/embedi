/*
 * @file hardware.rs
 * @brief Hardware bring-up helpers for the UART command example
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

use crate::config;
use crate::hal::{self, Clock};
use crate::types::{AppUart, AppUartPins, LedPin};
use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use hal::gpio::Pins;
use hal::uart::{DataBits, FifoWatermark, StopBits, UartConfig, UartPeripheral};

/// Aggregates the peripherals needed by the application loop.
pub struct Runtime {
    pub uart: AppUart,
    pub led: LedPin,
}

/// Consumes the PAC and constructs the runtime peripherals.
///
/// # Returns
/// A fully initialized [`Runtime`] with UART, LED, clocks, and watchdog configured.
pub fn build_runtime() -> Runtime {
    let mut pac = take_peripherals();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = init_system_clocks(
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    );
    let pins = init_pins(pac.SIO, pac.IO_BANK0, pac.PADS_BANK0, &mut pac.RESETS);
    let mut led = pins.gpio16.into_push_pull_output();
    led.set_low().ok();
    let uart_pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );
    let uart = configure_uart(pac.UART0, &mut pac.RESETS, &clocks, uart_pins);
    Runtime { uart, led }
}

/// Takes ownership of the RP235x PAC peripherals.
///
/// # Returns
/// The singleton PAC peripheral set.
fn take_peripherals() -> hal::pac::Peripherals {
    hal::pac::Peripherals::take().unwrap()
}

/// Sets up the system clocks using rp-hal helpers.
///
/// # Parameters
/// * `xosc` - Crystal oscillator peripheral.
/// * `clocks` - CLOCKS block handle.
/// * `pll_sys` - System PLL peripheral.
/// * `pll_usb` - USB PLL peripheral.
/// * `resets` - Reset controller used during bring-up.
/// * `watchdog` - Watchdog used by the clock init routine.
///
/// # Returns
/// A configured [`hal::clocks::ClocksManager`].
fn init_system_clocks(
    xosc: hal::pac::XOSC,
    clocks: hal::pac::CLOCKS,
    pll_sys: hal::pac::PLL_SYS,
    pll_usb: hal::pac::PLL_USB,
    resets: &mut hal::pac::RESETS,
    watchdog: &mut hal::Watchdog,
) -> hal::clocks::ClocksManager {
    hal::clocks::init_clocks_and_plls(
        config::XTAL_FREQ_HZ,
        xosc,
        clocks,
        pll_sys,
        pll_usb,
        resets,
        watchdog,
    )
    .unwrap()
}

/// Configures the SIO block and splits the GPIO pins.
///
/// # Parameters
/// * `sio_device` - SIO peripheral consumed to build the pin bank.
/// * `io` - IO bank peripheral.
/// * `pads` - Pad control peripheral.
/// * `resets` - Reset controller required by `Pins::new`.
///
/// # Returns
/// Fully configured [`Pins`] structure.
fn init_pins(
    sio_device: hal::pac::SIO,
    io: hal::pac::IO_BANK0,
    pads: hal::pac::PADS_BANK0,
    resets: &mut hal::pac::RESETS,
) -> Pins {
    let sio = hal::Sio::new(sio_device);
    Pins::new(io, pads, sio.gpio_bank0, resets)
}

/// Enables UART0 with FIFO interrupts configured for reception.
///
/// # Parameters
/// * `uart` - Raw UART0 peripheral.
/// * `resets` - Reset controller for enabling the block.
/// * `clocks` - Clock manager that yields the peripheral clock rate.
/// * `pins` - UART TX/RX pin pair already switched into UART function.
///
/// # Returns
/// A fully configured [`AppUart`].
fn configure_uart(
    uart: hal::pac::UART0,
    resets: &mut hal::pac::RESETS,
    clocks: &hal::clocks::ClocksManager,
    pins: AppUartPins,
) -> AppUart {
    let mut uart = enable_uart_device(uart, resets, clocks, pins);
    uart.set_fifos(true);
    uart.set_rx_watermark(FifoWatermark::Bytes4);
    uart.enable_rx_interrupt();
    uart
}

/// Helper that wraps `UartPeripheral::enable` with the desired configuration.
///
/// # Parameters
/// * `uart` - Raw UART peripheral to wrap.
/// * `resets` - Reset controller reference.
/// * `clocks` - Clock manager for baud calculations.
/// * `pins` - UART TX/RX pin pair.
///
/// # Returns
/// Enabled UART handle with the requested baud rate.
fn enable_uart_device(
    uart: hal::pac::UART0,
    resets: &mut hal::pac::RESETS,
    clocks: &hal::clocks::ClocksManager,
    pins: AppUartPins,
) -> AppUart {
    UartPeripheral::new(uart, pins, resets)
        .enable(
            UartConfig::new(config::UART_BAUD.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap()
}
