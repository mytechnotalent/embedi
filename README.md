![image](https://github.com/mytechnotalent/embedi/blob/main/Embedi.png?raw=true)

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# Embedi

Embedi — hands‑free open‑source embedded AI: speech capture, local Whisper transcription, Ollama LLM chat, macOS TTS, and UART control for Pico robotics.

<br>

# Features
- Always-on voice loop with automatic microphone capture and temporary file cleanup.
- Local Whisper transcription (ggml-base.en) plus Ollama LLM responses with persistent conversational memory.
- Friendly "Em-bee-dee" embedded-systems persona optimized for hardware control and technical guidance.
- macOS `say` playback for instant spoken responses.
- Direct UART hardware control bypass for LED commands without LLM round-trip.

<br>

# Prerequisites
- macOS with working microphone and `say` command (available by default).
- Rust 1.78+ (or latest stable) and Cargo.
- Ollama installed and running (`brew install ollama` or download from https://ollama.ai).
- Default model: `llama3.3:70b` (auto-downloaded on first run, ~40GB).

<br>

# Setup
```bash
# clone and enter the project
git clone https://github.com/mytechnotalent/embedi.git
cd embedi

# optional: configure serial port and model in config.json
# default values work for most setups
```

<br>

# Run
```bash
# release build for optimal performance
cargo run --release
```
The application auto-starts Ollama if needed, downloads the model on first run, records 5-second audio clips, transcribes locally with Whisper, and responds via Ollama. Say "quit", "stop", or "goodbye" to exit.

<br>

# Raspberry Pi Pico 2 LED Control
- Flash your Pico with the `rp2350-comms` firmware (see below).
- Connect the Pico USB to your Mac; note the device path (typically `/dev/cu.usbmodem1402`).
- Optional: override defaults via environment variables:

```bash
export EMBEDI_SERIAL_PORT=/dev/cu.usbmodem1402
export EMBEDI_SERIAL_BAUD=115200
```

When Embedi hears phrases like "turn the LED on" or "turn the LED off", it sends the command over UART immediately and speaks confirmation without waiting for the LLM.

<br>

# `rp2350-comms` Firmware
The `rp2350-comms/` directory contains bare-metal firmware for Raspberry Pi Pico 2 (RP2350) that listens for UART commands and controls an LED on GPIO16.

## Hardware Requirements
- Raspberry Pi Pico 2 (RP2350)
- Pico Debug Probe (for SWD flashing)
- LED and 100Ω resistor connected to GPIO16

## Wiring
1. Connect Pico Debug Probe to Mac via USB
2. Wire Debug Probe to Pico 2: SWDIO, SWCLK, GND
3. Keep Pico USB connected for power and UART communication
4. Connect LED: Anode → GPIO16, Cathode → 100Ω resistor → GND

## Building & Flashing
```bash
cd rp2350-comms

# one-time setup
rustup target add thumbv8m.main-none-eabihf

# build and flash
make flash

# or build only
make build

# run tests
make test
```

The firmware listens on UART0 (115200 baud) and responds to:
- `ON` — Turns LED on, responds with "LED ON"
- `OFF` — Turns LED off, responds with "LED OFF"

<br>

# Testing
```bash
# test main application
cargo test

# test firmware
cd rp2350-comms && make test
```

<br>

# Project Structure
```
embedi/
├── src/
│   ├── main.rs           # Application entry point
│   ├── assistant.rs      # Voice loop, Ollama orchestration, UART control
│   ├── audio.rs          # Microphone capture and WAV handling
│   └── speech.rs         # macOS TTS wrapper
├── rp2350-comms/
│   ├── src/
│   │   ├── main.rs       # Firmware entry point
│   │   ├── hardware.rs   # UART and GPIO initialization
│   │   ├── detector.rs   # Keyword detection engine
│   │   ├── detection.rs  # Detection result handler
│   │   ├── filter.rs     # ASCII validation
│   │   ├── process.rs    # Byte processing pipeline
│   │   └── config.rs     # Configuration constants
│   ├── Makefile          # Build and flash automation
│   └── Cargo.toml        # Firmware dependencies
├── config.json           # Runtime configuration
├── memory.json           # Persistent conversation history
└── Cargo.toml            # Application dependencies
```

<br>

# Configuration
Edit `config.json` to customize:
```json
{
    "default_serial_port": "/dev/cu.usbmodem1402",
    "default_ollama_model": "llama3.3:70b",
    "_comment_default": "llama3.2:3b - Fast and efficient 3B parameter model, ideal for resource-constrained systems",
    "_comment_powerful": "For more powerful inference, use: qwen2.5:32b or llama3.3:70b"
}
```

Override via environment variables:
```bash
export EMBEDI_SERIAL_PORT=/dev/cu.usbmodem1402
export EMBEDI_SERIAL_BAUD=115200
export OLLAMA_MODEL=llama3.3:70b
```

<br>

# License
[MIT](https://github.com/mytechnotalent/embedi/blob/main/LICENSE)
