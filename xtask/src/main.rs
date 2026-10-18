//! `cargo run -p xtask [SCHEMA] [OUT]`
//!
//! Rewrites the record types (`src/generated.rs`) from the schema
//! (`schema/trajectory.schema.json`).

use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args_os().skip(1);
    let schema = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(xtask::schema_path);
    let out = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(xtask::generated_path);
    let code = xtask::generate(&std::fs::read_to_string(&schema)?)?;
    std::fs::write(&out, code)?;
    eprintln!("wrote {} from {}", out.display(), schema.display());
    Ok(())
}
