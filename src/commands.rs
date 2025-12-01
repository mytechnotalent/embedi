/*
 * @file commands.rs
 * @brief Device command configuration and matching logic
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

//! Device command configuration and matching system for UART control.

use anyhow::{Context, Result};
use serde::Deserialize;
use std::fs;

/// Path to the commands configuration file.
const COMMANDS_FILE: &str = "commands.json";

/// Represents a single device command configuration.
///
/// # Details
/// Maps natural language keywords to UART commands that can be sent
/// to connected devices like the Raspberry Pi Pico.
#[derive(Clone, Debug, Deserialize)]
pub struct CommandConfig {
    /// Natural language keywords that trigger this command.
    pub keywords: Vec<String>,
    /// UART command string to send to the device.
    pub uart_command: String,
    /// Human-readable description of what the command does.
    pub description: String,
}

/// Container for all configured device commands.
///
/// # Details
/// Loaded from commands.json and provides methods for matching
/// user utterances to specific device commands.
#[derive(Clone, Debug, Deserialize)]
pub struct CommandsConfig {
    /// List of all available device commands.
    pub commands: Vec<CommandConfig>,
}

/// Loads commands configuration from commands.json.
///
/// # Details
/// Reads and parses the commands.json file from the current directory.
/// Returns a default configuration if the file doesn't exist or cannot be parsed.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `CommandsConfig` - Loaded or default command configuration.
pub fn load_commands() -> CommandsConfig {
    load_commands_from_file().unwrap_or_else(|err| {
        eprintln!(
            "Warning: Failed to load {}: {}. Using default commands.",
            COMMANDS_FILE, err
        );
        default_commands()
    })
}

/// Loads commands from the JSON file.
///
/// # Details
/// Attempts to read and parse commands.json. Returns an error
/// if the file is missing or contains invalid JSON.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `Ok(CommandsConfig)` - Successfully parsed configuration.
///
/// # Errors
/// Returns an error if file cannot be read or parsed.
fn load_commands_from_file() -> Result<CommandsConfig> {
    let content = fs::read_to_string(COMMANDS_FILE)
        .with_context(|| format!("Failed to read {}", COMMANDS_FILE))?;
    serde_json::from_str(&content)
        .with_context(|| format!("Failed to parse {}", COMMANDS_FILE))
}

/// Provides default commands when commands.json is unavailable.
///
/// # Details
/// Returns a hardcoded configuration with LED ON/OFF commands
/// to ensure basic functionality even without a config file.
///
/// # Arguments
/// None.
///
/// # Returns
/// * `CommandsConfig` - Default LED command configuration.
fn default_commands() -> CommandsConfig {
    CommandsConfig {
        commands: vec![
            CommandConfig {
                keywords: vec![
                    "turn on".to_string(),
                    "on".to_string(),
                    "light on".to_string(),
                    "led on".to_string(),
                ],
                uart_command: "ON".to_string(),
                description: "Turn LED on".to_string(),
            },
            CommandConfig {
                keywords: vec![
                    "turn off".to_string(),
                    "off".to_string(),
                    "light off".to_string(),
                    "led off".to_string(),
                ],
                uart_command: "OFF".to_string(),
                description: "Turn LED off".to_string(),
            },
        ],
    }
}

/// Finds a matching command for the given text.
///
/// # Details
/// Searches through all configured commands to find one whose keywords
/// match the input text. Returns the first matching command found.
///
/// # Arguments
/// * `commands` - The command configuration to search through.
/// * `text` - The user's utterance in lowercase.
///
/// # Returns
/// * `Some(&CommandConfig)` - The first matching command.
/// * `None` - No matching command found.
pub fn find_command<'a>(
    commands: &'a CommandsConfig,
    text: &str,
) -> Option<&'a CommandConfig> {
    commands.commands.iter().find(|cmd| {
        cmd.keywords
            .iter()
            .any(|keyword| text.contains(keyword.as_str()))
    })
}

/// Generates system prompt text listing all available commands.
///
/// # Details
/// Creates a formatted string describing all commands that the LLM
/// can use, which gets injected into the system prompt.
///
/// # Arguments
/// * `commands` - The command configuration.
///
/// # Returns
/// * `String` - Formatted command list for the system prompt.
pub fn generate_command_list(commands: &CommandsConfig) -> String {
    let mut result = String::from("Available device commands:\n");
    for cmd in &commands.commands {
        result.push_str(&format!("- {}: {}\n", cmd.uart_command, cmd.description));
    }
    result
}
