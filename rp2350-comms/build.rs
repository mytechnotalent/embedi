/*
 * @file build.rs
 * @brief Cargo build script for memory layout
 * @author Kevin Thomas
 * @date 2025
 *
 * MIT License
 *
 * Copyright (c) 2025 Kevin Thomas
 */

//! Cargo build script that configures linker memory layout.
//!
//! # Details
//! Copies memory.x linker script to OUT_DIR and configures link search path.
//! Required for embedded ARM builds to define memory regions.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

/// Build script entry point.
///
/// # Details
/// Copies memory.x to build output directory and sets linker search path.
/// Triggers rebuild when memory.x changes.
fn main() {
    let out_dir = get_out_dir();
    copy_memory_layout(&out_dir);
    configure_linker(&out_dir);
    set_rebuild_trigger();
}

/// Gets the cargo OUT_DIR path.
///
/// # Details
/// Retrieves the build output directory from environment.
///
/// # Returns
/// * `PathBuf` - Path to OUT_DIR
fn get_out_dir() -> PathBuf {
    PathBuf::from(env::var_os("OUT_DIR").unwrap())
}

/// Copies memory.x linker script to output directory.
///
/// # Details
/// Creates memory.x in OUT_DIR with embedded content.
///
/// # Arguments
/// * `out` - Path to output directory
fn copy_memory_layout(out: &PathBuf) {
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
}

/// Configures linker search path.
///
/// # Details
/// Tells linker to search OUT_DIR for memory.x.
///
/// # Arguments
/// * `out` - Path to output directory
fn configure_linker(out: &PathBuf) {
    println!("cargo:rustc-link-search={}", out.display());
}

/// Sets rebuild trigger for memory.x changes.
///
/// # Details
/// Informs cargo to rebuild when memory.x is modified.
fn set_rebuild_trigger() {
    println!("cargo:rerun-if-changed=memory.x");
}