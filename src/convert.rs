//! Converting one file, or a whole output directory.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use rayon::prelude::*;
use serde::Deserialize;

use crate::generated as g;
use crate::map::{self, Projection};
use crate::parse::{self, JsonParser, LineErrorKind, ParseOptions, Parsed, Record};
use crate::rrd;

/// The `log` target (and Python logger name) warnings go to.
pub const LOG_TARGET: &str = "autoware_carla_scenario_rerun";
/// The suffix of the files converted.
pub const TRAJECTORY_SUFFIX: &str = ".trajectory.jsonl";

/// Why a conversion failed.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{}: {source}", path.display())]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// A line that is not a record of the schema.
    #[error("{}:{line}: {message}", path.display())]
    Parse {
        path: PathBuf,
        line: u64,
        message: String,
    },
    /// A record type the schema does not know, in strict mode.
    #[error("{}:{line}: unknown record type `{record_type}` (strict mode)", path.display())]
    UnknownType {
        path: PathBuf,
        line: u64,
        record_type: String,
    },
    #[error("{}: {message}", path.display())]
    Result { path: PathBuf, message: String },
    #[error("{}: {message}", path.display())]
    Map { path: PathBuf, message: String },
    #[error("{}: {message}", path.display())]
    Rerun { path: PathBuf, message: String },
    /// A field the schema does not define, in strict mode.
    #[error("{}:{line}: unknown field `{field}` (strict mode)", path.display())]
    UnknownField {
        path: PathBuf,
        line: u64,
        field: String,
    },
    /// The conversion panicked (a bug): reported, so a batch goes on.
    #[error("{}: the conversion panicked: {message}", path.display())]
    Panic { path: PathBuf, message: String },
}

impl Error {
    /// The file the error is about.
    pub fn path(&self) -> &Path {
        match self {
            Error::Io { path, .. }
            | Error::Parse { path, .. }
            | Error::UnknownType { path, .. }
            | Error::Result { path, .. }
            | Error::Map { path, .. }
            | Error::Rerun { path, .. }
            | Error::UnknownField { path, .. }
            | Error::Panic { path, .. } => path,
        }
    }
}

/// How a file is converted.
#[derive(Clone, Debug, Default)]
pub struct ConvertOptions {
    /// The run's `_result.json`, for a file without a `result` record; found
    /// beside the file (`<stem>_result.json`) when unset.
    pub result_json: Option<PathBuf>,
    /// A directory to look the Lanelet2 map up in by the CARLA map's name
    /// ([`map::find_in_dir`]), when the file's `map` record names no file
    /// that exists.
    pub map_dir: Option<PathBuf>,
    /// The Lanelet2 map to draw, overriding the file's.
    pub map_path: Option<PathBuf>,
    /// Draw no map.
    pub no_map: bool,
    /// Fail on a record type or a field the schema does not define.
    pub strict: bool,
    /// Parsing.
    pub parser: JsonParser,
    /// The parse chunk size, bytes (0: the default).
    pub chunk_bytes: usize,
}

/// What a conversion did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Stats {
    pub input: PathBuf,
    pub output: PathBuf,
    /// The format version of the file (1 when its `world` record says none).
    pub schema_version: u64,
    pub bytes: u64,
    pub lines: u64,
    pub ticks: u64,
    /// Distinct vehicles and walkers.
    pub actors: u64,
    /// Actor rows over all ticks.
    pub actor_rows: u64,
    pub traffic_lights: u64,
    pub markers: u64,
    pub conditions: u64,
    pub has_result: bool,
    /// The Lanelet2 map drawn, if any.
    pub map: Option<PathBuf>,
    /// Whether the map came from the per-process cache.
    pub map_cached: bool,
    /// Skipped record types: lines per type.
    pub unknown_types: BTreeMap<String, u64>,
    /// The first line of each skipped type.
    pub unknown_first_lines: BTreeMap<String, u64>,
    /// Ignored fields the schema does not define (`record.field`): lines per field.
    pub unknown_fields: BTreeMap<String, u64>,
    /// The first line of each ignored field.
    pub unknown_field_first_lines: BTreeMap<String, u64>,
    /// Numbers written as `null` (NaN), taken as NaN.
    pub null_numbers: u64,
    /// Warnings (also logged, and written under `diagnostics/` in the file).
    pub warnings: Vec<String>,
    pub rrd_bytes: u64,
    pub parse_seconds: f64,
    pub map_seconds: f64,
    pub write_seconds: f64,
    pub seconds: f64,
}

/// `X.trajectory.jsonl` -> `X.rrd` (any other name: its extension replaced).
pub fn default_output(input: &Path) -> PathBuf {
    let name = input
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    match name.strip_suffix(TRAJECTORY_SUFFIX) {
        Some(stem) => input.with_file_name(format!("{stem}.rrd")),
        None => input.with_extension("rrd"),
    }
}

/// The run's stem: `X` of `X.trajectory.jsonl`.
pub fn stem(input: &Path) -> String {
    let name = input
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    name.strip_suffix(TRAJECTORY_SUFFIX)
        .map(str::to_string)
        .unwrap_or_else(|| {
            input
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default()
        })
}

#[derive(Deserialize)]
struct ResultJson {
    passed: bool,
    #[serde(default)]
    message: String,
    #[serde(default)]
    elapsed_seconds: f64,
    #[serde(default)]
    condition_statuses: Vec<StatusJson>,
}

#[derive(Deserialize)]
struct StatusJson {
    #[serde(default)]
    label: String,
    #[serde(default)]
    role: String,
    #[serde(default)]
    condition_type: String,
    #[serde(default)]
    satisfied: bool,
    #[serde(default)]
    message: String,
}

/// Reads a `_result.json` as the `result` record it corresponds to.
pub fn read_result_json(path: &Path) -> Result<g::ResultRecord, Error> {
    let text = std::fs::read(path).map_err(|source| Error::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let r: ResultJson = serde_json::from_slice(&text).map_err(|e| Error::Result {
        path: path.to_path_buf(),
        message: e.to_string(),
    })?;
    Ok(g::ResultRecord {
        extra: Default::default(),
        type_: serde_json::Value::String("result".into()),
        passed: r.passed,
        message: r.message,
        elapsed_seconds: r.elapsed_seconds,
        conditions: r
            .condition_statuses
            .into_iter()
            .map(|c| g::ConditionStatus {
                label: c.label,
                role: c.role,
                condition_type: c.condition_type,
                satisfied: c.satisfied,
                message: c.message,
                extra: Default::default(),
            })
            .collect(),
    })
}

/// Reads and parses a file (mapped into memory).
pub fn read(input: &Path, options: &ParseOptions) -> Result<Parsed, Error> {
    let file = std::fs::File::open(input).map_err(|source| Error::Io {
        path: input.to_path_buf(),
        source,
    })?;
    let len = file
        .metadata()
        .map_err(|source| Error::Io {
            path: input.to_path_buf(),
            source,
        })?
        .len();
    let parsed = if len == 0 {
        parse::parse_bytes(&[], options)
    } else {
        // SAFETY: the file is only read. One rewritten while it is converted
        // can fault the process, which is why output files are written once
        // and only converted after.
        let bytes = unsafe { memmap2::Mmap::map(&file) }.map_err(|source| Error::Io {
            path: input.to_path_buf(),
            source,
        })?;
        parse::parse_bytes(&bytes, options)
    };
    parsed.map_err(|e| match e.kind {
        LineErrorKind::Invalid(message) => Error::Parse {
            path: input.to_path_buf(),
            line: e.line,
            message,
        },
        LineErrorKind::UnknownType(record_type) => Error::UnknownType {
            path: input.to_path_buf(),
            line: e.line,
            record_type,
        },
        LineErrorKind::UnknownField(field) => Error::UnknownField {
            path: input.to_path_buf(),
            line: e.line,
            field,
        },
    })
}

fn carla_map_name(world: Option<&g::WorldRecord>, map: Option<&g::MapRecord>) -> Option<String> {
    map.map(|m| m.name.clone())
        .or_else(|| world.map(|w| w.map.rsplit('/').next().unwrap_or(&w.map).to_string()))
}

struct MapChoice {
    path: PathBuf,
    sha256: Option<String>,
    projection: Projection,
    offset: Option<[f64; 3]>,
}

/// Which Lanelet2 map to draw, and how to place it.
///
/// The file: `map_path`, else the one the `map` record names (when it is on
/// this machine), else one found under `map_dir`. The run's projection and
/// offsets apply only when the file is the run's own -- its sha256 is the
/// one the `map` record has; any other map (a version-1 log, another copy,
/// an override) is read with its own `map_projector_info.yaml` and placed at
/// its origin, with a warning when it is not the map the run recorded.
fn choose_map(
    parsed: &Parsed,
    options: &ConvertOptions,
    warnings: &mut Vec<(String, String)>,
) -> Option<MapChoice> {
    if options.no_map {
        return None;
    }
    let mut world = None;
    let mut record = None;
    for l in &parsed.records {
        match &l.record {
            Record::World(w) => world = world.or(Some(w)),
            Record::Map(m) => record = record.or(Some(&**m)),
            _ => {}
        }
    }
    let name = carla_map_name(world, record);
    let recorded = record.and_then(|m| m.lanelet2.as_ref());
    let path = if let Some(path) = &options.map_path {
        path.clone()
    } else if let Some(file) = recorded.filter(|f| Path::new(&f.path).is_file()) {
        PathBuf::from(&file.path)
    } else if let (Some(dir), Some(name)) = (&options.map_dir, &name) {
        match map::find_in_dir(dir, name) {
            Some(path) => path,
            None => {
                warnings.push((
                    "map".into(),
                    format!(
                        "no Lanelet2 map of {name} under {}: no map drawn",
                        dir.display()
                    ),
                ));
                return None;
            }
        }
    } else {
        if let Some(file) = recorded {
            warnings.push((
                "map".into(),
                format!(
                    "the run's Lanelet2 map {} is not on this machine (give map_dir): no map drawn",
                    file.path
                ),
            ));
        }
        return None;
    };
    // Only a recorded map has a hash to check; hashing is per file, once.
    let sha256 = match recorded {
        Some(_) => match map::file_sha256(&path) {
            Ok(hash) => Some(hash),
            Err(message) => {
                warnings.push(("map".into(), format!("{message}: no map drawn")));
                return None;
            }
        },
        None => None,
    };
    let own = recorded.is_some_and(|f| sha256.as_deref() == Some(&*f.sha256));
    if let (Some(file), false) = (recorded, own) {
        warnings.push((
            "map".into(),
            format!(
                "{} is not the Lanelet2 map the run recorded ({}, sha256 {}): drawn with its own projection, at its origin",
                path.display(),
                file.path,
                *file.sha256
            ),
        ));
    }
    let record = record.filter(|_| own);
    let info = record
        .and_then(|m| m.projector_info.as_ref())
        .map(map::ProjectorInfo::from_json);
    let opendrive = record
        .and_then(|m| m.opendrive.as_ref())
        .map(|f| PathBuf::from(&f.path))
        .filter(|p| p.is_file());
    let projection = map::resolve_projection(
        &path,
        record.and_then(|m| m.projector_type.as_deref()),
        info,
        opendrive.as_deref(),
    );
    let offset = record.and_then(|m| {
        m.mgrs_offset
            .map(|(ox, oy)| [ox, oy, m.z_offset.unwrap_or(0.0)])
    });
    Some(MapChoice {
        path,
        sha256,
        projection,
        offset,
    })
}

/// Converts `input` to the recording `output`.
///
/// The recording is written to a temporary file beside `output` and renamed
/// over it once complete, so a failed conversion leaves no partial `.rrd`.
pub fn convert(input: &Path, output: &Path, options: &ConvertOptions) -> Result<Stats, Error> {
    let start = Instant::now();
    let parse_options = ParseOptions {
        parser: options.parser,
        strict: options.strict,
        chunk_bytes: if options.chunk_bytes == 0 {
            ParseOptions::default().chunk_bytes
        } else {
            options.chunk_bytes
        },
    };
    let parsed = read(input, &parse_options)?;
    let parse_seconds = start.elapsed().as_secs_f64();
    let mut stats = Stats {
        input: input.to_path_buf(),
        output: output.to_path_buf(),
        schema_version: 1,
        bytes: std::fs::metadata(input).map(|m| m.len()).unwrap_or(0),
        lines: parsed.lines,
        ticks: parsed.ticks.len() as u64,
        actor_rows: parsed.ticks.actor_id.len() as u64,
        parse_seconds,
        ..Default::default()
    };
    let mut warnings: Vec<(String, String)> = Vec::new();
    let file = input.display();
    for (ty, u) in &parsed.unknown {
        let text = format!(
            "skipped {} line(s) of unknown record type `{ty}` (first at line {}): this converter reads format version {} of the trajectory log",
            u.count,
            u.first_line,
            g::SCHEMA_VERSION
        );
        stats.unknown_types.insert(ty.clone(), u.count);
        stats.unknown_first_lines.insert(ty.clone(), u.first_line);
        warnings.push((format!("unknown_types/{ty}"), text));
    }
    for (field, u) in &parsed.unknown_fields {
        let text = format!(
            "ignored unknown field `{field}` on {} line(s) (first at line {}): this converter reads format version {} of the trajectory log",
            u.count,
            u.first_line,
            g::SCHEMA_VERSION
        );
        stats.unknown_fields.insert(field.clone(), u.count);
        stats
            .unknown_field_first_lines
            .insert(field.clone(), u.first_line);
        warnings.push((format!("unknown_fields/{field}"), text));
    }
    if let Some(n) = parsed.nulls {
        stats.null_numbers = n.count;
        warnings.push((
            "null_numbers".into(),
            format!(
                "{} number(s) were null (NaN when the log was written), first at line {}: drawn as NaN",
                n.count, n.first_line
            ),
        ));
    }
    if parsed.lines == 0 {
        warnings.push(("empty".into(), "the trajectory log is empty".into()));
    }
    let mut actors = std::collections::HashSet::new();
    let mut result = None;
    for l in &parsed.records {
        match &l.record {
            Record::World(w) => stats.schema_version = w.schema_version.map_or(1, |v| v.get()),
            Record::Actor(a) => {
                actors.insert(a.id.0);
            }
            Record::TrafficLight(_) => stats.traffic_lights += 1,
            Record::Markers(_) => stats.markers += 1,
            Record::Condition(_) => stats.conditions += 1,
            Record::Result(_) => stats.has_result = true,
            _ => {}
        }
    }
    stats.actors = actors.len() as u64;
    if !stats.has_result {
        let candidate = options
            .result_json
            .clone()
            .unwrap_or_else(|| input.with_file_name(format!("{}_result.json", stem(input))));
        if options.result_json.is_some() || candidate.is_file() {
            result = Some(read_result_json(&candidate)?);
            stats.has_result = true;
        }
    }

    let map_start = Instant::now();
    let choice = choose_map(&parsed, options, &mut warnings);
    // A map that cannot be read is a warning: the run is still worth seeing.
    let loaded = choice.as_ref().and_then(|c| {
        map::load(&c.path, &c.projection, c.sha256.as_deref())
            .map_err(|message| warnings.push(("map".into(), format!("{message}: no map drawn"))))
            .ok()
    });
    stats.map_seconds = map_start.elapsed().as_secs_f64();
    let layer = match (&choice, &loaded) {
        (Some(c), Some(l)) => {
            stats.map = Some(c.path.clone());
            stats.map_cached = l.cached;
            let offset = c.offset.unwrap_or_else(|| {
                let (lat, lon, alt) = c.projection.origin;
                let o = c.projection.forward(lat, lon, alt);
                [o[0], o[1], 0.0]
            });
            Some(rrd::MapLayer {
                geometry: &l.geometry,
                offset,
            })
        }
        _ => None,
    };

    for (_, text) in &warnings {
        log::warn!(target: LOG_TARGET, "{file}: {text}");
    }
    stats.warnings = warnings.iter().map(|(_, t)| t.clone()).collect();

    let write_start = Instant::now();
    if let Some(dir) = output.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir).map_err(|source| Error::Io {
            path: dir.to_path_buf(),
            source,
        })?;
    }
    let recording_id = stem(input);
    let partial = partial_path(output);
    let written = rrd::write(
        &partial,
        &parsed,
        &rrd::Inputs {
            recording_id: &recording_id,
            result: result.as_ref(),
            map: layer,
            warnings: &warnings,
        },
    )
    .map_err(|message| Error::Rerun {
        path: output.to_path_buf(),
        message,
    })
    .and_then(|_| {
        std::fs::rename(&partial, output).map_err(|source| Error::Io {
            path: output.to_path_buf(),
            source,
        })
    });
    if let Err(e) = written {
        let _ = std::fs::remove_file(&partial);
        return Err(e);
    }
    stats.write_seconds = write_start.elapsed().as_secs_f64();
    stats.rrd_bytes = std::fs::metadata(output).map(|m| m.len()).unwrap_or(0);
    stats.seconds = start.elapsed().as_secs_f64();
    Ok(stats)
}

/// The temporary file a recording is written to before it is renamed to
/// `output`: hidden, beside it (the same file system), unique per process
/// and conversion.
fn partial_path(output: &Path) -> PathBuf {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let name = output
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    output.with_file_name(format!(".{name}.{}-{n}.partial", std::process::id()))
}

/// Every `*.trajectory.jsonl` under `dir` (recursively), sorted.
pub fn find_inputs(dir: &Path) -> Result<Vec<PathBuf>, Error> {
    let mut found = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        let entries = std::fs::read_dir(&d).map_err(|source| Error::Io {
            path: d.clone(),
            source,
        })?;
        for entry in entries.flatten() {
            let path = entry.path();
            let kind = entry.file_type().ok();
            if kind.is_some_and(|k| k.is_dir()) {
                stack.push(path);
            } else if path.to_string_lossy().ends_with(TRAJECTORY_SUFFIX) {
                found.push(path);
            }
        }
    }
    found.sort();
    Ok(found)
}

/// One file of [`convert_dir`]: its log, and how its conversion went.
#[derive(Debug)]
pub struct DirResult {
    /// The trajectory log.
    pub input: PathBuf,
    pub result: Result<Stats, Error>,
}

/// Converts every trajectory log under `dir`, each to its [`default_output`]
/// (under `out_dir` instead when given, keeping the layout), on `jobs`
/// threads (all cores when `None`). Each file's `_result.json` is found
/// beside it. One result per file, in [`find_inputs`] order; a file that
/// fails -- or panics -- is reported and the others still convert.
pub fn convert_dir(
    dir: &Path,
    out_dir: Option<&Path>,
    options: &ConvertOptions,
    jobs: Option<usize>,
) -> Result<Vec<DirResult>, Error> {
    let inputs = find_inputs(dir)?;
    let options = ConvertOptions {
        result_json: None,
        ..options.clone()
    };
    let run = || {
        inputs
            .par_iter()
            .map(|input| {
                let mut output = default_output(input);
                if let Some(out_dir) = out_dir {
                    let rel = output.strip_prefix(dir).unwrap_or(&output).to_path_buf();
                    output = out_dir.join(rel);
                }
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    convert(input, &output, &options)
                }))
                .unwrap_or_else(|panic| {
                    let message = panic
                        .downcast_ref::<&str>()
                        .map(|s| s.to_string())
                        .or_else(|| panic.downcast_ref::<String>().cloned())
                        .unwrap_or_else(|| "a panic".to_string());
                    Err(Error::Panic {
                        path: input.clone(),
                        message,
                    })
                });
                DirResult {
                    input: input.clone(),
                    result,
                }
            })
            .collect()
    };
    match jobs {
        Some(n) => {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(n.max(1))
                .build()
                .map_err(|e| Error::Io {
                    path: dir.to_path_buf(),
                    source: std::io::Error::other(e),
                })?;
            Ok(pool.install(run))
        }
        None => Ok(run()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn output_names() {
        assert_eq!(
            default_output(Path::new("/o/Case-1.trajectory.jsonl")),
            Path::new("/o/Case-1.rrd")
        );
        assert_eq!(default_output(Path::new("x.jsonl")), Path::new("x.rrd"));
        assert_eq!(stem(Path::new("/o/Case-1.trajectory.jsonl")), "Case-1");
    }

    #[test]
    fn a_failed_write_leaves_no_file() {
        let dir = tempfile::tempdir().unwrap();
        let input = dir.path().join("x.trajectory.jsonl");
        std::fs::write(&input, "{\"type\":\"world\",\"map\":\"M\",\"episode\":1}\n").unwrap();
        // The output is a directory: the rename fails after the write.
        let output = dir.path().join("x.rrd");
        std::fs::create_dir(&output).unwrap();
        std::fs::write(output.join("keep"), "").unwrap();
        assert!(convert(&input, &output, &ConvertOptions::default()).is_err());
        let left: Vec<_> = std::fs::read_dir(dir.path())
            .unwrap()
            .flatten()
            .map(|e| e.file_name())
            .collect();
        assert_eq!(left.len(), 2, "{left:?}");
    }
}
