![image](https://github.com/mytechnotalent/embedi/blob/main/Embedi.png?raw=true)

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# Embedi

Embedi — hands‑free open‑source embedded AI: speech capture, Groq Whisper transcription, Llama 3.3 chat, macOS TTS, UART control for Pico robotics.

<br>

# Features
- Always-on loop that captures short microphone samples and cleans up temp audio automatically.
- Groq Whisper V3 Turbo transcription plus Llama 3.3 70B responses with conversational memory.
- Friendly "Em-bee-dee" embedded-systems persona.
- macOS `say` playback for responses so you get spoken output immediately.

<br>

# Prerequisites
- macOS with a working microphone and the `say` command (available by default).
- Rust 1.78+ (or the latest stable release) and Cargo.
- Groq API credentials (`GROQ_API_KEY`) set in your environment or a `.env` file.

<br>

# Setup
```bash
# clone and enter the project
git clone <repo-url>
cd embedi

# copy env template and add your key
cp .env.example .env

# export your Groq key (or place it in .env)
export GROQ_API_KEY=sk_your_key
```

<br>

# Run
```bash
# release build for lower latency
cargo run --release
```
The binary prompts you every few seconds, records ~5 seconds of audio, and responds once Whisper returns text. Say "quit", "stop", or "goodbye" to exit.

<br>

# Raspberry Pi Pico LED Control
- Flash your Pico with the UART listener shown above (or any firmware that reacts to the strings `ON` and `OFF`).
- Plug the board into your Mac; note the device path (e.g., `/dev/tty.usbmodem21302`).
- Optional: override the defaults via environment variables before launching Embedi:

```bash
export EMBEDI_SERIAL_PORT=/dev/tty.usbmodem21302
export EMBEDI_SERIAL_BAUD=115200
```

When Embedi hears phrases like "turn the LED on" or "disable the Pico light," it sends the matching command over that UART without waiting for the LLM round-trip, and you'll hear a spoken confirmation.

<br>

# `rp2350-comms/` Firmware
- Lives under `rp2350-comms/` and provides the bare-metal UART/LED helper that Embedi talks to.
- Hardware: a Raspberry Pi Pico 2 (RP2350), a Pico Debug Probe for SWD flashing, and a single LED plus 100 Ω resistor in series on GPIO16 (anode to GP16, cathode through the resistor to GND).
- Wiring & flash steps:
	1. Plug the Pico Debug Probe into your Mac, wire SWDIO/SWCLK/GND to the Pico 2 header, and keep the Pico's USB cable connected for power and UART back to macOS.
	2. Drop the LED/resistor pair between GPIO16 and GND so the firmware has something to toggle when ON/OFF strings arrive.
	3. `cd rp2350-comms`
	4. (One time) `rustup target add thumbv8m.main-none-eabihf`
	5. `cargo run --release` — the bundled `.cargo/config.toml` calls `picotool` by default; set `PICOTOOL_PATH` or swap the runner for `probe-rs run --chip RP2350` if you prefer flashing over the debug probe.
- Once flashed, return to the repo root, run `cargo run --release` for Embedi, and it will drive the Pico over the default `/dev/tty.usbmodem21302` / `115200` UART (override via the env vars above if your port differs).

<br>

# Testing
```bash
cargo test
```
Unit tests cover audio helpers, temp-file cleanup, and the silence gate.

<br>

# Project Structure
- `src/audio.rs` – Microphone capture and WAV persistence.
- `src/assistant.rs` – Main loop, Groq orchestration, and persona guidance.
- `src/speech.rs` – Text-to-speech wrapper around macOS `say`.
- `Cargo.toml` – Dependencies (Rig, Groq client, CPAL, etc.).

Feel free to tailor the persona or thresholds in `src/assistant.rs` to suit your deployment environment.

<br>

# License
[MIT](https://github.com/mytechnotalent/embedi/blob/main/LICENSE)
