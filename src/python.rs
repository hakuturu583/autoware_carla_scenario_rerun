//! The Python module `autoware_carla_scenario_rerun._native`.
//!
//! Every conversion runs with the GIL released (`Python::detach`), so a
//! caller's other threads -- the next case of a sweep, say -- keep running.
//! Warnings go through the `log` crate to Python's `logging` (pyo3-log),
//! under the logger `autoware_carla_scenario_rerun`.

use std::path::PathBuf;

use pyo3::create_exception;
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

use crate::convert as core;
use crate::convert::{ConvertOptions, Error, Stats};
use crate::parse::JsonParser;

create_exception!(
    _native,
    ConversionError,
    PyRuntimeError,
    "A trajectory log could not be converted."
);
create_exception!(
    _native,
    MalformedLineError,
    ConversionError,
    "A line of the trajectory log is not a record of the schema (`.line` says which)."
);
create_exception!(
    _native,
    UnknownRecordTypeError,
    ConversionError,
    "strict=True and the log has a record type the schema does not define."
);

fn to_pyerr(py: Python<'_>, error: &Error) -> PyErr {
    let message = error.to_string();
    let err = match error {
        Error::Parse { .. } => MalformedLineError::new_err(message),
        Error::UnknownType { .. } => UnknownRecordTypeError::new_err(message),
        _ => ConversionError::new_err(message),
    };
    let value = err.value(py);
    let _ = value.setattr("path", error.path().to_string_lossy());
    match error {
        Error::Parse { line, .. } => {
            let _ = value.setattr("line", *line);
        }
        Error::UnknownType {
            line, record_type, ..
        } => {
            let _ = value.setattr("line", *line);
            let _ = value.setattr("record_type", record_type.clone());
        }
        _ => {}
    }
    err
}

fn stats_dict<'py>(py: Python<'py>, s: &Stats) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("ok", true)?;
    d.set_item("error", py.None())?;
    d.set_item("input", s.input.to_string_lossy())?;
    d.set_item("output", s.output.to_string_lossy())?;
    d.set_item("schema_version", s.schema_version)?;
    d.set_item("bytes", s.bytes)?;
    d.set_item("lines", s.lines)?;
    d.set_item("ticks", s.ticks)?;
    d.set_item("actors", s.actors)?;
    d.set_item("actor_rows", s.actor_rows)?;
    d.set_item("traffic_lights", s.traffic_lights)?;
    d.set_item("markers", s.markers)?;
    d.set_item("conditions", s.conditions)?;
    d.set_item("has_result", s.has_result)?;
    d.set_item("map", s.map.as_ref().map(|p| p.to_string_lossy()))?;
    d.set_item("map_cached", s.map_cached)?;
    d.set_item("unknown_types", s.unknown_types.clone())?;
    d.set_item("unknown_first_lines", s.unknown_first_lines.clone())?;
    d.set_item("warnings", s.warnings.clone())?;
    d.set_item("rrd_bytes", s.rrd_bytes)?;
    d.set_item("parse_seconds", s.parse_seconds)?;
    d.set_item("map_seconds", s.map_seconds)?;
    d.set_item("write_seconds", s.write_seconds)?;
    d.set_item("seconds", s.seconds)?;
    Ok(d)
}

fn parser(name: &str) -> PyResult<JsonParser> {
    match name {
        "serde_json" => Ok(JsonParser::SerdeJson),
        "simd_json" => Ok(JsonParser::SimdJson),
        other => Err(pyo3::exceptions::PyValueError::new_err(format!(
            "parser must be 'serde_json' or 'simd_json', not {other:?}"
        ))),
    }
}

/// Converts one trajectory log; see the stub for the parameters.
#[pyfunction]
#[pyo3(signature = (jsonl, out_rrd=None, *, result_json=None, map_dir=None, map_path=None, no_map=false, strict=false, parser="serde_json"))]
#[allow(clippy::too_many_arguments)]
fn convert<'py>(
    py: Python<'py>,
    jsonl: PathBuf,
    out_rrd: Option<PathBuf>,
    result_json: Option<PathBuf>,
    map_dir: Option<PathBuf>,
    map_path: Option<PathBuf>,
    no_map: bool,
    strict: bool,
    parser: &str,
) -> PyResult<Bound<'py, PyDict>> {
    let options = ConvertOptions {
        result_json,
        map_dir,
        map_path,
        no_map,
        strict,
        parser: self::parser(parser)?,
        chunk_bytes: 0,
    };
    let out = out_rrd.unwrap_or_else(|| core::default_output(&jsonl));
    let result = py.detach(|| core::convert(&jsonl, &out, &options));
    match result {
        Ok(stats) => stats_dict(py, &stats),
        Err(e) => Err(to_pyerr(py, &e)),
    }
}

/// Converts every trajectory log under a directory; see the stub.
#[pyfunction]
#[pyo3(signature = (output_dir, *, jobs=None, out_dir=None, map_dir=None, no_map=false, strict=false, parser="serde_json"))]
#[allow(clippy::too_many_arguments)]
fn convert_dir<'py>(
    py: Python<'py>,
    output_dir: PathBuf,
    jobs: Option<usize>,
    out_dir: Option<PathBuf>,
    map_dir: Option<PathBuf>,
    no_map: bool,
    strict: bool,
    parser: &str,
) -> PyResult<Bound<'py, PyList>> {
    let options = ConvertOptions {
        map_dir,
        no_map,
        strict,
        parser: self::parser(parser)?,
        ..Default::default()
    };
    let results = py
        .detach(|| core::convert_dir(&output_dir, out_dir.as_deref(), &options, jobs))
        .map_err(|e| to_pyerr(py, &e))?;
    let list = PyList::empty(py);
    for result in results {
        match result {
            Ok(stats) => list.append(stats_dict(py, &stats)?)?,
            Err(e) => {
                let d = PyDict::new(py);
                d.set_item("ok", false)?;
                d.set_item("error", e.to_string())?;
                d.set_item("error_type", to_pyerr(py, &e).get_type(py).name()?)?;
                d.set_item("input", e.path().to_string_lossy())?;
                list.append(d)?;
            }
        }
    }
    Ok(list)
}

/// Forgets the Lanelet2 maps read so far in this process.
#[pyfunction]
fn clear_map_cache() {
    crate::map::clear_cache();
}

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Rust `log` records -> Python `logging`, warnings and up only: below
    // that a record never leaves Rust (the Rerun SDK logs a lot at debug
    // level). Nothing is cached, so a level or handler set later (pytest's
    // caplog, an application's logging config) applies; at a warning or two
    // per file that costs nothing. A second import (another interpreter)
    // finds a logger installed already: keep that one.
    let _ = pyo3_log::Logger::new(m.py(), pyo3_log::Caching::Nothing)
        .map(|logger| logger.filter(log::LevelFilter::Warn).install());
    m.add_function(wrap_pyfunction!(convert, m)?)?;
    m.add_function(wrap_pyfunction!(convert_dir, m)?)?;
    m.add_function(wrap_pyfunction!(clear_map_cache, m)?)?;
    let py = m.py();
    m.add("ConversionError", py.get_type::<ConversionError>())?;
    m.add("MalformedLineError", py.get_type::<MalformedLineError>())?;
    m.add(
        "UnknownRecordTypeError",
        py.get_type::<UnknownRecordTypeError>(),
    )?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add("RERUN_VERSION", crate::RERUN_VERSION)?;
    m.add("SCHEMA_VERSION", crate::generated::SCHEMA_VERSION)?;
    m.add(
        "RECORD_TYPES",
        crate::generated::RECORD_TYPES
            .iter()
            .map(|(t, _)| *t)
            .collect::<Vec<_>>(),
    )?;
    m.add("LOGGER_NAME", crate::LOG_TARGET)?;
    Ok(())
}
