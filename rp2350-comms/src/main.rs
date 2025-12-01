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

//! RP2350 embedded application entry point.
//!
//! This module contains only the main entry point which initializes
//! the hardware and delegates to the runtime module.

#![no_std]
#![no_main]

mod config;
mod detection;
mod detector;
mod filter;
mod hardware;
mod process;

use embassy_executor::Spawner;
use hardware::run_application;
use panic_halt as _;

/// Main application entry point.
///
/// # Details
/// Initializes Embassy runtime and spawns the main application task.
/// Hardware initialization and event loop are delegated to runtime module.
///
/// # Arguments
/// * `spawner` - Embassy task spawner for async execution.
///
/// # Returns
/// * `()` - Never returns (infinite loop in runtime).
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    run_application(spawner).await;
}
