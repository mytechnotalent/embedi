![image](https://github.com/mytechnotalent/embedi/blob/main/Embedi.png?raw=true)

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# Embedi

Hands-free open-source embedded-systems AI assistant named Embedi that records speech, transcribes it with Groq Whisper, chats with Llama 3.3, and replies through macOS text-to-speech.

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
