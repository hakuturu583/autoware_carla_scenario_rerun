//! Reading a `*.trajectory.jsonl` file: lines in parallel, ticks into columns.
//!
//! The file is split into chunks at newlines and the chunks are parsed in
//! parallel (rayon). Each line is told apart by its `type` -- read straight
//! off the line when the writer put it first, as the recorder does -- and:
//!
//! * a `tick` (nearly every line of a file) goes through [`TickSeed`], which
//!   appends it to the columnar [`Ticks`] buffers without building a record:
//!   no per-tick allocation, only the buffers growing;
//! * every other record type the schema knows is deserialized into its
//!   generated type ([`crate::generated`]);
//! * a type the schema does not know is skipped and counted (or, with
//!   [`ParseOptions::strict`], an error), so a newer writer's file still
//!   converts.
//!
//! The elements of a tick (an actor's state, a control, a light state) are
//! deserialized as their generated types too, which are fixed-size tuples and
//! enums and allocate nothing; only the containers around them are walked by
//! hand. A schema change to them is a compile error here, not a silent
//! mismatch.

use std::collections::BTreeMap;
use std::fmt;
use std::marker::PhantomData;

use rayon::prelude::*;
use serde::de::{self, DeserializeSeed, IgnoredAny, MapAccess, SeqAccess, Visitor};
use serde::Deserialize;

use crate::generated as g;

/// The record types this crate converts: every `type` of the schema.
/// `tests/schema.rs` checks it against [`g::RECORD_TYPES`].
pub const HANDLED_TYPES: &[&str] = &[
    "world",
    "map",
    "traffic_light",
    "warmup",
    "actor",
    "actor_gone",
    "tick",
    "markers",
    "condition",
    "result",
];

/// Which JSON parser reads the lines.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum JsonParser {
    /// `serde_json`, reading the mapped file in place.
    #[default]
    SerdeJson,
    /// `simd-json`, on a per-thread copy of each line (it parses in place).
    SimdJson,
}

/// How a file is parsed.
#[derive(Clone, Debug)]
pub struct ParseOptions {
    /// The JSON parser.
    pub parser: JsonParser,
    /// Fail on a record type the schema does not know, instead of skipping it.
    pub strict: bool,
    /// The size a chunk of the file parsed by one task aims at, bytes.
    pub chunk_bytes: usize,
}

impl Default for ParseOptions {
    fn default() -> Self {
        Self {
            parser: JsonParser::default(),
            strict: false,
            chunk_bytes: 1 << 20,
        }
    }
}

/// A light's state, as the schema names them.
pub type LightState = g::TrafficLightState;

/// An ego's control on a tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Control {
    /// Index of the tick in [`Ticks`].
    pub tick: u32,
    /// The ego's actor id.
    pub id: u32,
    pub throttle: f64,
    pub steer: f64,
    pub brake: f64,
    pub gear: i64,
}

/// A traffic light that changed state on a tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LightChange {
    /// Index of the tick in [`Ticks`].
    pub tick: u32,
    /// The light's actor id.
    pub id: u32,
    pub state: LightState,
}

/// Every tick of a file, as columns.
///
/// Positions, rotations and velocities are as written: CARLA's left-handed
/// world frame, metres, degrees (roll, pitch, yaw) and m/s.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Ticks {
    /// The CARLA frame of each tick.
    pub frame: Vec<i64>,
    /// The simulated time of each tick, seconds.
    pub t: Vec<f64>,
    /// Where each tick's actors start in the actor columns; tick `i` has the
    /// rows `actor_start[i]..actor_start[i + 1]` (the last one up to the end).
    pub actor_start: Vec<u32>,
    /// The actor columns: one row per actor and tick.
    pub actor_id: Vec<u32>,
    pub position: Vec<[f64; 3]>,
    pub rotation: Vec<[f64; 3]>,
    pub velocity: Vec<[f64; 3]>,
    /// The egos' controls.
    pub controls: Vec<Control>,
    /// The traffic lights' state changes.
    pub lights: Vec<LightChange>,
}

impl Ticks {
    /// The number of ticks.
    pub fn len(&self) -> usize {
        self.frame.len()
    }

    /// Whether there is no tick.
    pub fn is_empty(&self) -> bool {
        self.frame.is_empty()
    }

    /// The rows of tick `i` in the actor columns.
    pub fn actors(&self, i: usize) -> std::ops::Range<usize> {
        let start = self.actor_start[i] as usize;
        let end = self
            .actor_start
            .get(i + 1)
            .map_or(self.actor_id.len(), |&e| e as usize);
        start..end
    }

    fn append(&mut self, mut other: Ticks) {
        let ticks = self.len() as u32;
        let rows = self.actor_id.len() as u32;
        self.frame.append(&mut other.frame);
        self.t.append(&mut other.t);
        self.actor_start
            .extend(other.actor_start.iter().map(|s| s + rows));
        self.actor_id.append(&mut other.actor_id);
        self.position.append(&mut other.position);
        self.rotation.append(&mut other.rotation);
        self.velocity.append(&mut other.velocity);
        self.controls.extend(other.controls.iter().map(|c| Control {
            tick: c.tick + ticks,
            ..*c
        }));
        self.lights.extend(other.lights.iter().map(|l| LightChange {
            tick: l.tick + ticks,
            ..*l
        }));
    }
}

/// A record other than a tick, as its generated type.
#[derive(Clone, Debug, PartialEq)]
pub enum Record {
    World(g::WorldRecord),
    Map(g::MapRecord),
    TrafficLight(g::TrafficLightRecord),
    Warmup(g::WarmupRecord),
    Actor(g::ActorRecord),
    ActorGone(g::ActorGoneRecord),
    Markers(g::MarkersRecord),
    Condition(g::ConditionRecord),
    Result(g::ResultRecord),
}

/// A record and the line it was on.
#[derive(Clone, Debug, PartialEq)]
pub struct Located {
    /// 1-based line number.
    pub line: u64,
    pub record: Record,
}

/// The lines of a record type the schema does not know.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Unknown {
    /// How many lines had it.
    pub count: u64,
    /// The first of them, 1-based.
    pub first_line: u64,
}

/// A parsed file.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Parsed {
    pub ticks: Ticks,
    /// Every other record, in file order.
    pub records: Vec<Located>,
    /// The skipped record types, by name.
    pub unknown: BTreeMap<String, Unknown>,
    /// Lines in the file (blank ones included).
    pub lines: u64,
}

/// What went wrong on a line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LineErrorKind {
    /// Not JSON, or not a record of the schema.
    Invalid(String),
    /// A record type the schema does not know, in strict mode.
    UnknownType(String),
}

/// A line that could not be read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LineError {
    /// 1-based line number.
    pub line: u64,
    pub kind: LineErrorKind,
}

impl fmt::Display for LineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.kind {
            LineErrorKind::Invalid(message) => write!(f, "line {}: {message}", self.line),
            LineErrorKind::UnknownType(ty) => write!(
                f,
                "line {}: unknown record type `{ty}` (strict mode refuses record types the schema does not define)",
                self.line
            ),
        }
    }
}

impl std::error::Error for LineError {}

#[derive(Default)]
struct ChunkOut {
    ticks: Ticks,
    records: Vec<Located>,
    unknown: BTreeMap<String, Unknown>,
    lines: u64,
}

/// Splits `bytes` into chunks of about `target` bytes that end at a newline.
fn split_chunks(bytes: &[u8], target: usize) -> Vec<&[u8]> {
    let target = target.max(1);
    let mut chunks = Vec::with_capacity(bytes.len() / target + 1);
    let mut start = 0;
    while start < bytes.len() {
        let mut end = (start + target).min(bytes.len());
        if end < bytes.len() {
            end = match memchr::memchr(b'\n', &bytes[end..]) {
                Some(i) => end + i + 1,
                None => bytes.len(),
            };
        }
        chunks.push(&bytes[start..end]);
        start = end;
    }
    chunks
}

/// Parses a whole file's bytes.
pub fn parse_bytes(bytes: &[u8], options: &ParseOptions) -> Result<Parsed, LineError> {
    let chunks = split_chunks(bytes, options.chunk_bytes);
    let results: Vec<Result<ChunkOut, LineError>> = if chunks.len() > 1 {
        chunks
            .par_iter()
            .map(|chunk| parse_chunk(chunk, options))
            .collect()
    } else {
        chunks
            .iter()
            .map(|chunk| parse_chunk(chunk, options))
            .collect()
    };
    let mut parsed = Parsed::default();
    for result in results {
        let base = parsed.lines;
        let chunk = result.map_err(|e| LineError {
            line: e.line + base,
            kind: e.kind,
        })?;
        parsed.ticks.append(chunk.ticks);
        parsed
            .records
            .extend(chunk.records.into_iter().map(|r| Located {
                line: r.line + base,
                record: r.record,
            }));
        for (ty, u) in chunk.unknown {
            parsed
                .unknown
                .entry(ty)
                .and_modify(|seen| seen.count += u.count)
                .or_insert(Unknown {
                    count: u.count,
                    first_line: u.first_line + base,
                });
        }
        parsed.lines += chunk.lines;
    }
    Ok(parsed)
}

/// The `type` of a line, read off its start when the writer put it first.
fn sniff_type(line: &[u8]) -> Option<&[u8]> {
    const PREFIX: &[u8] = br#"{"type":""#;
    let rest = line.strip_prefix(PREFIX)?;
    let end = memchr::memchr(b'"', rest)?;
    let ty = &rest[..end];
    (!ty.contains(&b'\\')).then_some(ty)
}

#[derive(Deserialize)]
struct TypeProbe<'a> {
    #[serde(rename = "type", borrow)]
    ty: std::borrow::Cow<'a, str>,
}

/// The `type` of a line wherever it is; a line without one is invalid.
fn probe_type(line: &[u8]) -> Result<String, String> {
    serde_json::from_slice::<TypeProbe>(line)
        .map(|p| p.ty.into_owned())
        .map_err(|e| format!("not a trajectory record: {}", tidy(&e.to_string())))
}

/// Drops serde_json's "at line 1": a line is parsed on its own.
fn tidy(message: &str) -> String {
    message.replace(" at line 1 column ", " at column ")
}

/// Per-thread state for the parsers.
struct Scratch {
    copy: Vec<u8>,
    buffers: simd_json::Buffers,
}

fn deserialize<V, S>(
    parser: JsonParser,
    line: &[u8],
    scratch: &mut Scratch,
    seed: S,
) -> Result<V, String>
where
    S: for<'de> DeserializeSeed<'de, Value = V>,
{
    match parser {
        JsonParser::SerdeJson => {
            let mut de = serde_json::Deserializer::from_slice(line);
            let value = seed
                .deserialize(&mut de)
                .and_then(|v| de.end().map(|()| v))
                .map_err(|e| tidy(&e.to_string()))?;
            Ok(value)
        }
        JsonParser::SimdJson => {
            scratch.copy.clear();
            scratch.copy.extend_from_slice(line);
            let mut de = simd_json::Deserializer::from_slice_with_buffers(
                &mut scratch.copy,
                &mut scratch.buffers,
            )
            .map_err(|e| e.to_string())?;
            seed.deserialize(&mut de).map_err(|e| e.to_string())
        }
    }
}

fn record<T>(parser: JsonParser, line: &[u8], scratch: &mut Scratch) -> Result<T, String>
where
    T: for<'de> Deserialize<'de>,
{
    deserialize(parser, line, scratch, PhantomData::<T>)
}

fn parse_chunk(chunk: &[u8], options: &ParseOptions) -> Result<ChunkOut, LineError> {
    let mut out = ChunkOut::default();
    let mut scratch = Scratch {
        copy: Vec::new(),
        buffers: simd_json::Buffers::default(),
    };
    let parser = options.parser;
    let mut start = 0;
    let mut line_no = 0u64;
    while start < chunk.len() {
        let end = memchr::memchr(b'\n', &chunk[start..]).map_or(chunk.len(), |i| start + i);
        let mut line = &chunk[start..end];
        start = end + 1;
        line_no += 1;
        if let Some(stripped) = line.strip_suffix(b"\r") {
            line = stripped;
        }
        if line.iter().all(u8::is_ascii_whitespace) {
            continue;
        }
        let fail = |message: String| LineError {
            line: line_no,
            kind: LineErrorKind::Invalid(message),
        };
        let probed;
        let ty: &[u8] = match sniff_type(line) {
            Some(ty) => ty,
            None => {
                probed = probe_type(line).map_err(fail)?;
                probed.as_bytes()
            }
        };
        let rec = match ty {
            b"tick" => {
                let tick = out.ticks.len() as u32;
                deserialize(
                    parser,
                    line,
                    &mut scratch,
                    TickSeed {
                        out: &mut out.ticks,
                        tick,
                    },
                )
                .map_err(|m| fail(format!("tick: {m}")))?;
                continue;
            }
            b"world" => record(parser, line, &mut scratch).map(Record::World),
            b"map" => record(parser, line, &mut scratch).map(Record::Map),
            b"traffic_light" => record(parser, line, &mut scratch).map(Record::TrafficLight),
            b"warmup" => record(parser, line, &mut scratch).map(Record::Warmup),
            b"actor" => record(parser, line, &mut scratch).map(Record::Actor),
            b"actor_gone" => record(parser, line, &mut scratch).map(Record::ActorGone),
            b"markers" => record(parser, line, &mut scratch).map(Record::Markers),
            b"condition" => record(parser, line, &mut scratch).map(Record::Condition),
            b"result" => record(parser, line, &mut scratch).map(Record::Result),
            other => {
                let name = String::from_utf8_lossy(other).into_owned();
                if options.strict {
                    return Err(LineError {
                        line: line_no,
                        kind: LineErrorKind::UnknownType(name),
                    });
                }
                out.unknown
                    .entry(name)
                    .and_modify(|u| u.count += 1)
                    .or_insert(Unknown {
                        count: 1,
                        first_line: line_no,
                    });
                continue;
            }
        };
        let record = rec.map_err(|m| fail(format!("{}: {m}", String::from_utf8_lossy(ty))))?;
        out.records.push(Located {
            line: line_no,
            record,
        });
    }
    out.lines = line_no;
    Ok(out)
}

// ---------------------------------------------------------------------------
// The tick fast path
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
#[serde(field_identifier, rename_all = "lowercase")]
enum TickField {
    Type,
    Frame,
    T,
    Actors,
    Control,
    Lights,
    #[serde(other)]
    Other,
}

/// Appends one tick record to [`Ticks`].
pub(crate) struct TickSeed<'a> {
    pub(crate) out: &'a mut Ticks,
    pub(crate) tick: u32,
}

impl<'de> DeserializeSeed<'de> for TickSeed<'_> {
    type Value = ();
    fn deserialize<D: de::Deserializer<'de>>(self, deserializer: D) -> Result<(), D::Error> {
        deserializer.deserialize_map(self)
    }
}

impl<'de> Visitor<'de> for TickSeed<'_> {
    type Value = ();

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a tick record")
    }

    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<(), A::Error> {
        let out = self.out;
        let start = out.actor_id.len() as u32;
        let (mut frame, mut t, mut actors, mut control) = (None, None, false, false);
        while let Some(key) = map.next_key::<TickField>()? {
            match key {
                TickField::Frame => frame = Some(map.next_value::<g::SafeUInt>()?.0),
                TickField::T => t = Some(map.next_value::<f64>()?),
                TickField::Actors => {
                    map.next_value_seed(ActorsSeed(out))?;
                    actors = true;
                }
                TickField::Control => {
                    map.next_value_seed(ControlSeed {
                        out: &mut out.controls,
                        tick: self.tick,
                    })?;
                    control = true;
                }
                TickField::Lights => map.next_value_seed(LightsSeed {
                    out: &mut out.lights,
                    tick: self.tick,
                })?,
                TickField::Type | TickField::Other => {
                    map.next_value::<IgnoredAny>()?;
                }
            }
        }
        let frame = frame.ok_or_else(|| de::Error::missing_field("frame"))?;
        let t = t.ok_or_else(|| de::Error::missing_field("t"))?;
        if !actors {
            return Err(de::Error::missing_field("actors"));
        }
        if !control {
            return Err(de::Error::missing_field("control"));
        }
        out.frame.push(frame);
        out.t.push(t);
        out.actor_start.push(start);
        Ok(())
    }
}

struct ActorsSeed<'a>(&'a mut Ticks);

impl<'de> DeserializeSeed<'de> for ActorsSeed<'_> {
    type Value = ();
    fn deserialize<D: de::Deserializer<'de>>(self, deserializer: D) -> Result<(), D::Error> {
        deserializer.deserialize_seq(self)
    }
}

impl<'de> Visitor<'de> for ActorsSeed<'_> {
    type Value = ();

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a list of actor states")
    }

    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<(), A::Error> {
        let out = self.0;
        while let Some(g::ActorState((id, x, y, z, roll, pitch, yaw, vx, vy, vz))) =
            seq.next_element::<g::ActorState>()?
        {
            out.actor_id.push(id.0);
            out.position.push([x, y, z]);
            out.rotation.push([roll, pitch, yaw]);
            out.velocity.push([vx, vy, vz]);
        }
        Ok(())
    }
}

/// An actor id written as an object key ("78"), read without allocating.
struct IdKey;

impl<'de> DeserializeSeed<'de> for IdKey {
    type Value = u32;
    fn deserialize<D: de::Deserializer<'de>>(self, deserializer: D) -> Result<u32, D::Error> {
        deserializer.deserialize_str(self)
    }
}

impl Visitor<'_> for IdKey {
    type Value = u32;

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("an actor id as a decimal string")
    }

    fn visit_str<E: de::Error>(self, v: &str) -> Result<u32, E> {
        if v.is_empty() || !v.bytes().all(|b| b.is_ascii_digit()) {
            return Err(E::invalid_value(de::Unexpected::Str(v), &self));
        }
        v.parse()
            .map_err(|_| E::invalid_value(de::Unexpected::Str(v), &self))
    }
}

struct ControlSeed<'a> {
    out: &'a mut Vec<Control>,
    tick: u32,
}

impl<'de> DeserializeSeed<'de> for ControlSeed<'_> {
    type Value = ();
    fn deserialize<D: de::Deserializer<'de>>(self, deserializer: D) -> Result<(), D::Error> {
        deserializer.deserialize_map(self)
    }
}

impl<'de> Visitor<'de> for ControlSeed<'_> {
    type Value = ();

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a map of actor id to vehicle control")
    }

    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<(), A::Error> {
        while let Some(id) = map.next_key_seed(IdKey)? {
            let g::VehicleControl((throttle, steer, brake, gear)) = map.next_value()?;
            self.out.push(Control {
                tick: self.tick,
                id,
                throttle,
                steer,
                brake,
                gear,
            });
        }
        Ok(())
    }
}

struct LightsSeed<'a> {
    out: &'a mut Vec<LightChange>,
    tick: u32,
}

impl<'de> DeserializeSeed<'de> for LightsSeed<'_> {
    type Value = ();
    fn deserialize<D: de::Deserializer<'de>>(self, deserializer: D) -> Result<(), D::Error> {
        deserializer.deserialize_map(self)
    }
}

impl<'de> Visitor<'de> for LightsSeed<'_> {
    type Value = ();

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a map of traffic light id to state")
    }

    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<(), A::Error> {
        while let Some(id) = map.next_key_seed(IdKey)? {
            let state: g::TrafficLightState = map.next_value()?;
            self.out.push(LightChange {
                tick: self.tick,
                id,
                state,
            });
        }
        Ok(())
    }
}

/// Converts a tick deserialized as the generated [`g::TickRecord`] into the
/// columns, the way [`TickSeed`] fills them: the reference the fast path is
/// tested and benchmarked against.
pub fn append_generated_tick(out: &mut Ticks, record: &g::TickRecord) {
    let tick = out.len() as u32;
    out.frame.push(record.frame.0);
    out.t.push(record.t);
    out.actor_start.push(out.actor_id.len() as u32);
    for g::ActorState((id, x, y, z, roll, pitch, yaw, vx, vy, vz)) in &record.actors {
        out.actor_id.push(id.0);
        out.position.push([*x, *y, *z]);
        out.rotation.push([*roll, *pitch, *yaw]);
        out.velocity.push([*vx, *vy, *vz]);
    }
    let mut controls: Vec<_> = record.control.iter().collect();
    controls.sort_by_key(|(k, _)| k.parse::<u32>().unwrap_or(u32::MAX));
    for (key, g::VehicleControl((throttle, steer, brake, gear))) in controls {
        out.controls.push(Control {
            tick,
            id: key.parse().unwrap_or(u32::MAX),
            throttle: *throttle,
            steer: *steer,
            brake: *brake,
            gear: *gear,
        });
    }
    let mut lights: Vec<_> = record.lights.iter().collect();
    lights.sort_by_key(|(k, _)| k.parse::<u32>().unwrap_or(u32::MAX));
    for (key, state) in lights {
        out.lights.push(LightChange {
            tick,
            id: key.parse().unwrap_or(u32::MAX),
            state: *state,
        });
    }
}

/// Parses the ticks of `bytes` the slow way, through the generated
/// [`g::TickRecord`] (allocating a record per tick): for benchmarks and tests.
pub fn parse_ticks_generated(bytes: &[u8]) -> Result<Ticks, String> {
    let mut ticks = Ticks::default();
    for line in bytes.split(|b| *b == b'\n') {
        if sniff_type(line) == Some(b"tick") {
            let record: g::TickRecord = serde_json::from_slice(line).map_err(|e| e.to_string())?;
            append_generated_tick(&mut ticks, &record);
        }
    }
    Ok(ticks)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICK: &str = r#"{"type":"tick","frame":991,"t":0.0,"actors":[[78,106.064,43.979,0.059,0.287,0.082,-89.609,0.0,0.0,0.0]],"control":{"78":[0.0,0.0,1.0,0]},"lights":{"77":"Red","76":"Green"}}"#;

    fn parse(text: &str) -> Result<Parsed, LineError> {
        parse_bytes(text.as_bytes(), &ParseOptions::default())
    }

    #[test]
    fn a_tick_lands_in_the_columns() {
        for parser in [JsonParser::SerdeJson, JsonParser::SimdJson] {
            let options = ParseOptions {
                parser,
                ..Default::default()
            };
            let parsed = parse_bytes(TICK.as_bytes(), &options).unwrap();
            let t = &parsed.ticks;
            assert_eq!(t.frame, vec![991]);
            assert_eq!(t.actor_id, vec![78]);
            assert_eq!(t.position, vec![[106.064, 43.979, 0.059]]);
            assert_eq!(t.rotation, vec![[0.287, 0.082, -89.609]]);
            assert_eq!(t.controls.len(), 1);
            assert_eq!(t.controls[0].brake, 1.0);
            assert_eq!(t.lights.len(), 2);
            assert_eq!(parsed.lines, 1);
        }
    }

    #[test]
    fn fast_path_matches_generated_types() {
        let fast = parse(TICK).unwrap().ticks;
        let slow = parse_ticks_generated(TICK.as_bytes()).unwrap();
        // The generated path sorts map keys; the fast one keeps file order.
        let mut fast_sorted = fast.clone();
        fast_sorted.lights.sort_by_key(|l| l.id);
        assert_eq!(fast_sorted, slow);
    }

    #[test]
    fn type_need_not_come_first() {
        let line =
            r#"{"frame":1,"type":"tick","t":0.5,"actors":[[1,0,0,0,0,0,0,0,0,0]],"control":{}}"#;
        let parsed = parse(line).unwrap();
        assert_eq!(parsed.ticks.t, vec![0.5]);
    }

    #[test]
    fn unknown_types_are_counted() {
        let text = format!("{TICK}\n{{\"type\":\"future\",\"x\":1}}\n\n{{\"type\":\"future\"}}\n{{\"type\":\"other\"}}\n");
        let parsed = parse(&text).unwrap();
        assert_eq!(
            parsed.unknown["future"],
            Unknown {
                count: 2,
                first_line: 2
            }
        );
        assert_eq!(
            parsed.unknown["other"],
            Unknown {
                count: 1,
                first_line: 5
            }
        );
        assert_eq!(parsed.lines, 5);
    }

    #[test]
    fn strict_refuses_unknown_types() {
        let text = format!("{TICK}\n{{\"type\":\"future\"}}\n");
        let options = ParseOptions {
            strict: true,
            ..Default::default()
        };
        let err = parse_bytes(text.as_bytes(), &options).unwrap_err();
        assert_eq!(err.line, 2);
        assert_eq!(err.kind, LineErrorKind::UnknownType("future".into()));
    }

    #[test]
    fn malformed_lines_say_where() {
        let text = format!("{TICK}\n{TICK}\n{{\"type\":\"tick\",\"frame\":\"x\"}}\n");
        let err = parse(&text).unwrap_err();
        assert_eq!(err.line, 3);
        assert!(
            err.to_string().starts_with("line 3: tick: invalid type"),
            "{err}"
        );
        let err = parse("not json\n").unwrap_err();
        assert_eq!(err.line, 1);
        let err = parse(&format!(
            "{TICK}\n{{\"type\":\"tick\",\"frame\":1,\"t\":0,\"control\":{{}}}}"
        ))
        .unwrap_err();
        assert_eq!(err.line, 2);
        assert!(err.to_string().contains("missing field `actors`"), "{err}");
    }

    #[test]
    fn chunks_keep_line_numbers() {
        let mut text = String::new();
        for i in 0..1000 {
            text.push_str(&TICK.replace("991", &i.to_string()));
            text.push('\n');
        }
        text.push_str("{\"type\":\"tick\"\n");
        let options = ParseOptions {
            chunk_bytes: 4096,
            ..Default::default()
        };
        let err = parse_bytes(text.as_bytes(), &options).unwrap_err();
        assert_eq!(err.line, 1001);
        let ok = parse_bytes(
            text.rsplit_once("{\"type\":\"tick\"\n")
                .unwrap()
                .0
                .as_bytes(),
            &options,
        )
        .unwrap();
        assert_eq!(ok.ticks.frame, (0..1000).collect::<Vec<i64>>());
        assert_eq!(ok.ticks.actor_start[999], 999);
        assert!(ok
            .ticks
            .controls
            .iter()
            .enumerate()
            .all(|(i, c)| c.tick as usize == i));
    }

    #[test]
    fn empty_input_is_empty() {
        let parsed = parse("").unwrap();
        assert!(parsed.ticks.is_empty());
        assert_eq!(parsed.lines, 0);
    }
}
