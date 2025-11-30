//! AI agent and transcription functionality.
//!
//! This module handles interaction with Groq's API for chat and speech transcription.

use anyhow::Result;
use rig::completion::{Chat, Prompt};
use rig::providers::groq;
use rig::transcription::TranscriptionModel;

/// Configuration and state for the AI assistant.
pub struct Assistant {
    /// The chat agent powered by Llama 3.3 70B.
    agent: groq::completion::CompletionModel,

    /// The speech transcription model (Whisper Large v3 Turbo).
    whisper: groq::transcription::TranscriptionModel,

    /// Conversation history for maintaining context.
    pub conversation_history: Vec<rig::message::Message>,
}

impl Assistant {
    /// Creates a new AI assistant instance.
    ///
    /// # Returns
    /// A configured Assistant with Groq agent and Whisper model.
    ///
    /// # Errors
    /// Returns an error if the Groq client cannot be initialized from environment.
    ///
    /// # Examples
    /// ```no_run
    /// use rig_speech_example::ai::Assistant;
    ///
    /// let assistant = Assistant::new().expect("Failed to create assistant");
    /// ```
    pub fn new() -> Result<Self> {
        let groq_client = groq::Client::from_env();

        let agent = groq_client
            .agent("llama-3.3-70b-versatile")
            .preamble("You are a helpful AI assistant. Keep responses concise and conversational. If the user says 'quit', 'exit', or 'goodbye', acknowledge it briefly.")
            .build();

        let whisper = groq_client.transcription_model(groq::WHISPER_LARGE_V3_TURBO);

        Ok(Self {
            agent,
            whisper,
            conversation_history: vec![],
        })
    }

    /// Transcribes audio from a WAV file.
    ///
    /// # Arguments
    /// * `audio_path` - Path to the WAV file to transcribe
    ///
    /// # Returns
    /// The transcribed text as a String.
    ///
    /// # Errors
    /// Returns an error if transcription fails.
    pub async fn transcribe(&self, audio_path: &str) -> Result<String> {
        let transcription = self
            .whisper
            .transcription_request()
            .load_file(audio_path)
            .send()
            .await?;

        Ok(transcription.text.trim().to_string())
    }

    /// Gets a response from the AI agent.
    ///
    /// # Arguments
    /// * `user_input` - The user's text input
    ///
    /// # Returns
    /// The AI's response as a String.
    ///
    /// # Errors
    /// Returns an error if the API request fails.
    pub async fn get_response(&self, user_input: &str) -> Result<String> {
        if self.conversation_history.is_empty() {
            self.agent.prompt(user_input).await
        } else {
            self.agent
                .chat(user_input, self.conversation_history.clone())
                .await
        }
    }

    /// Adds a user message to the conversation history.
    ///
    /// # Arguments
    /// * `text` - The user's message text
    pub fn add_user_message(&mut self, text: &str) {
        self.conversation_history
            .push(rig::message::Message::user(text));
    }

    /// Adds an assistant message to the conversation history.
    ///
    /// # Arguments
    /// * `text` - The assistant's response text
    pub fn add_assistant_message(&mut self, text: &str) {
        self.conversation_history
            .push(rig::message::Message::assistant(text));
    }

    /// Checks if the user wants to quit the conversation.
    ///
    /// # Arguments
    /// * `text` - The user's input text
    ///
    /// # Returns
    /// true if the text contains quit/exit keywords.
    pub fn is_quit_command(text: &str) -> bool {
        let lower = text.to_lowercase();
        lower.contains("quit")
            || lower.contains("exit")
            || lower.contains("goodbye")
            || lower.contains("bye")
    }
}
