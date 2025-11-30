//! Binary entry point that wires environment bootstrap and launches the
//! hands-free Groq-powered voice assistant loop.

use anyhow::Result;

use embedi::assistant;

#[tokio::main]
/// Bootstraps environment variables and launches the asynchronous voice
/// assistant loop.
async fn main() -> Result<()> {
    dotenv::dotenv().ok();
    assistant::run_voice_assistant().await
}
