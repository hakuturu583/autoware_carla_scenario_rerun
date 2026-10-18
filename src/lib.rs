//! Convert autoware_carla_scenario's trajectory logs (`*.trajectory.jsonl`)
//! into Rerun recordings (`.rrd`).
//!
//! * [`parse`] reads a log (memory-mapped, chunks in parallel, ticks straight
//!   into columns);
//! * [`map`] reads the run's Lanelet2 map, once per process;
//! * [`rrd`] writes the recording with Rerun's columnar API;
//! * [`convert()`] / [`convert_dir()`] put them together, one file or a whole
//!   output directory across all cores.
//!
//! The record types are generated from the log's JSON Schema
//! (`schema/trajectory.schema.json`) into [`generated`] by the `xtask`
//! crate: `cargo run -p xtask` regenerates them.

// Exactly what `cargo run -p xtask` writes (xtask/tests/generated.rs checks
// it): rustfmt keeps out.
#[allow(clippy::all, clippy::pedantic)]
#[rustfmt::skip]
pub mod generated;

pub mod convert;
pub mod layout;
pub mod map;
pub mod parse;
pub mod rrd;
pub mod scene;

#[cfg(feature = "python")]
mod python;

pub use convert::{
    convert, convert_dir, default_output, find_inputs, ConvertOptions, Error, Stats, LOG_TARGET,
};
pub use parse::{JsonParser, ParseOptions};

/// The Rerun SDK version recordings are written with.
pub const RERUN_VERSION: &str = "0.38.1";
