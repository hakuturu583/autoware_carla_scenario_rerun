//! Writing a parsed trajectory log as a Rerun recording.
//!
//! Everything temporal goes through `send_columns`, one call per entity and
//! component group, on two timelines: `frame` (the CARLA frame, a sequence)
//! and `sim_time` (the scenario clock, seconds). What does not change is
//! logged static, once. The layout follows ACS's in-process recorder:
//!
//! | entity | what |
//! |---|---|
//! | `/` | `ViewCoordinates` right-handed, z up (static) |
//! | `info` | the run: map, episode, format version, warm-up (static markdown) |
//! | `world/actors` | every vehicle and walker, a `Boxes3D` batch per tick; sizes, labels (`id type_id role`) and colours when the set of actors changes |
//! | `world/actors/velocity` | their velocities, `Arrows3D` from the box centres |
//! | `world/actors/events` | spawned / gone (at the `actor_gone` record's frame when there is one), `TextLog` |
//! | `world/ego/pose` | the ego's position and yaw (`Transform3D`): the chase view follows it |
//! | `world/ego/driven_path` | the ego's path (static `LineStrips3D`, a point every 0.5 m) |
//! | `world/ego/plan`, `world/ego/plan/points` | the ego's debug markers (`markers` records), held until the next |
//! | `ego/speed_mps`, `ego/control` | the ego's speed, and throttle/steer/brake (`Scalars`) |
//! | `world/traffic_lights` | the light heads, wireframe boxes (static) |
//! | `world/traffic_lights/bulbs` | three bulbs per head (red, yellow, green), the one the state names lit |
//! | `world/traffic_lights/tags` | each light's Lanelet2 id (else `carla <id>`) and state, written above it in the light's colour |
//! | `world/traffic_lights/stops` | their stop-line waypoints (static `Points3D`) |
//! | `world/traffic_lights/states` | the state changes, `TextLog` |
//! | `conditions/<role>/<index>` | 1 while a pass/fail condition is satisfied, 0 while not (`condition` records, carried forward) |
//! | `scenario/progress` | running, then the verdict (`StateChange`, for the state timeline view) |
//! | `scenario/actions/<label>` | each action's lifecycle (standby, start, running, end, complete), from the `action` records (`StateChange`) |
//! | `scenario/conditions/<role>/<label>` | satisfied / not satisfied, from the `condition` records (`StateChange`) |
//! | `scenario/traffic_lights/<id>` | each light's state over time (`StateChange`) |
//! | `result`, `result/log`, `result/passed` | the verdict |
//! | `map/lanelet2/lane_boundaries`, `map/lanelet2/stop_lines` | the Lanelet2 map (static) |
//! | `diagnostics/...` | what the conversion skipped or could not find, `TextLog` WARN (static) |

use std::collections::{BTreeMap, HashMap};
use std::path::Path;

use rerun::{
    Arrows3D, Boxes3D, Color, LineStrips3D, Points3D, Quaternion, RecordingStream,
    RecordingStreamBuilder, Scalars, SeriesLines, StateChange, StateConfiguration, TextDocument,
    TextLog, TimeColumn, Transform3D, ViewCoordinates,
};

use crate::generated as g;
use crate::map::{BoundaryKind, MapGeometry};
use crate::parse::{LightState, Parsed, Record, Ticks};
use crate::scene::{box_center, point32, quaternion};

/// The application id recordings are made under (ACS's in-process recorder's).
pub const APPLICATION_ID: &str = "autoware_carla_scenario";
/// How far the ego moves between two points of its driven path, metres.
pub const DRIVEN_PATH_STEP_M: f64 = 0.5;
/// `role_name` values taken for the ego.
pub const EGO_ROLES: [&str; 2] = ["ego", "hero"];

const EGO_COLOR: [u8; 3] = [255, 64, 64];
const VEHICLE_COLOR: [u8; 3] = [64, 160, 255];
const WALKER_COLOR: [u8; 3] = [255, 200, 0];
const DRIVEN_PATH_COLOR: [u8; 3] = [255, 128, 0];
const STOP_LINE_COLOR: [u8; 3] = [255, 255, 255];
const HOUSING_COLOR: [u8; 3] = [150, 150, 150];
/// The bulbs off: red, yellow, green, dimmed.
const BULB_DIM: [[u8; 3]; 3] = [[70, 0, 0], [70, 56, 0], [0, 62, 0]];
/// How far above a light's highest head its id and state are written, metres.
const LIGHT_TAG_HEIGHT_M: f64 = 0.8;
const RUNNING_COLOR: [u8; 3] = [64, 160, 255];
const PASSED_COLOR: [u8; 3] = [0, 200, 80];
const FAILED_COLOR: [u8; 3] = [230, 50, 50];
const START_COLOR: [u8; 3] = [255, 200, 0];
const END_COLOR: [u8; 3] = [255, 140, 0];
const UNSATISFIED_COLOR: [u8; 3] = [90, 90, 90];

fn rgb(c: [u8; 3]) -> Color {
    Color::from_rgb(c[0], c[1], c[2])
}

fn light_color(state: LightState) -> Color {
    rgb(match state {
        LightState::Red => [255, 0, 0],
        LightState::Yellow => [255, 200, 0],
        LightState::Green => [0, 220, 0],
        LightState::Off => [60, 60, 60],
        LightState::Unknown => [128, 128, 128],
    })
}

fn boundary_style(kind: BoundaryKind) -> ([u8; 3], f32) {
    match kind {
        BoundaryKind::Solid => ([220, 220, 220], 0.06),
        BoundaryKind::Dashed => ([160, 160, 160], 0.05),
        BoundaryKind::Curb => ([120, 120, 140], 0.08),
        BoundaryKind::Virtual => ([70, 70, 70], 0.03),
    }
}

/// The map to draw, placed in the recording's frame by `offset`
/// (`[ox, oy, z_offset]`, see [`crate::map`]).
pub struct MapLayer<'a> {
    pub geometry: &'a MapGeometry,
    pub offset: [f64; 3],
}

/// What the recording is made from, besides the parsed file.
pub struct Inputs<'a> {
    /// The recording id: the run's file stem.
    pub recording_id: &'a str,
    /// The verdict, when the file carries none (`_result.json`).
    pub result: Option<&'a g::ResultRecord>,
    pub map: Option<MapLayer<'a>>,
    /// Problems to show under `diagnostics` (as `(entity name, text)`).
    pub warnings: &'a [(String, String)],
}

/// What was written.
#[derive(Clone, Debug, Default)]
pub struct Written {
    pub entities: usize,
}

type RrResult<T> = Result<T, String>;

fn err(e: impl std::fmt::Display) -> String {
    format!("rerun: {e}")
}

struct Writer {
    rec: RecordingStream,
    entities: usize,
}

impl Writer {
    fn columns<I>(&mut self, path: &str, frames: &[i64], times: &[f64], columns: I) -> RrResult<()>
    where
        I: IntoIterator<Item = rerun::SerializedComponentColumn>,
    {
        debug_assert_eq!(frames.len(), times.len());
        if frames.is_empty() {
            return Ok(());
        }
        self.entities += 1;
        self.rec
            .send_columns(
                path,
                [
                    TimeColumn::new_sequence("frame", frames.iter().copied()),
                    TimeColumn::new_duration_secs("sim_time", times.iter().copied()),
                ],
                columns,
            )
            .map_err(err)
    }

    fn fixed(&mut self, path: &str, archetype: &impl rerun::AsComponents) -> RrResult<()> {
        self.entities += 1;
        self.rec.log_static(path, archetype).map_err(err)
    }
}

/// Writes `parsed` to the recording file `out`.
pub fn write(out: &Path, parsed: &Parsed, inputs: &Inputs<'_>) -> RrResult<Written> {
    let rec = RecordingStreamBuilder::new(APPLICATION_ID)
        .recording_id(inputs.recording_id)
        .save(out)
        .map_err(err)?;
    let mut w = Writer { rec, entities: 0 };
    w.fixed("/", &ViewCoordinates::RIGHT_HAND_Z_UP())?;

    let mut actors: HashMap<u32, &g::ActorRecord> = HashMap::new();
    let mut lights: Vec<&g::TrafficLightRecord> = Vec::new();
    let mut markers: Vec<&g::MarkersRecord> = Vec::new();
    let mut gone: Vec<&g::ActorGoneRecord> = Vec::new();
    let mut conditions: BTreeMap<(String, u64), Vec<&g::ConditionRecord>> = BTreeMap::new();
    let mut actions: BTreeMap<(String, u64), Vec<&g::ActionRecord>> = BTreeMap::new();
    let mut world = None;
    let mut map_record = None;
    let mut warmup = None;
    let mut result = None;
    for located in &parsed.records {
        match &located.record {
            Record::World(r) => world = world.or(Some(r)),
            Record::Map(r) => map_record = map_record.or(Some(&**r)),
            Record::Warmup(r) => warmup = warmup.or(Some(r)),
            Record::TrafficLight(r) => lights.push(r),
            Record::Actor(r) => {
                actors.entry(r.id.0).or_insert(r);
            }
            Record::ActorGone(r) => gone.push(r),
            Record::Markers(r) => markers.push(r),
            Record::Condition(r) => {
                let role = match r.role {
                    g::ConditionRecordRole::Pass => "pass",
                    g::ConditionRecordRole::Fail => "fail",
                };
                conditions
                    .entry((role.to_string(), r.index))
                    .or_default()
                    .push(r);
            }
            Record::Action(r) => {
                actions
                    .entry((r.phase.to_string(), r.index))
                    .or_default()
                    .push(r);
            }
            Record::Result(r) => result = Some(r),
        }
    }
    let result = result.or(inputs.result);

    write_info(&mut w, parsed, world, map_record, warmup)?;
    let ticks = &parsed.ticks;
    let ego = ego_id(parsed, &actors);
    crate::layout::send(&w.rec, APPLICATION_ID, inputs.recording_id, ego.is_some())?;
    write_actors(&mut w, ticks, &actors, &gone)?;
    if let Some(ego) = ego {
        write_ego(&mut w, ticks, ego)?;
    }
    let names = light_names(&lights, inputs.map.as_ref());
    write_lights(&mut w, ticks, &lights, &names)?;
    write_markers(&mut w, &markers)?;
    write_conditions(&mut w, ticks, &conditions)?;
    if let Some(result) = result {
        write_result(&mut w, ticks, result)?;
    }
    write_progress(&mut w, ticks, result)?;
    write_condition_states(&mut w, &conditions)?;
    write_action_states(&mut w, &actions)?;
    write_light_states(&mut w, ticks, &names)?;
    if let Some(map) = &inputs.map {
        write_map(&mut w, map)?;
    }
    for (name, text) in inputs.warnings {
        w.fixed(
            &format!("diagnostics/{name}"),
            &TextLog::new(text.as_str()).with_level(rerun::TextLogLevel::WARN),
        )?;
    }
    w.rec.flush_blocking().map_err(err)?;
    Ok(Written {
        entities: w.entities,
    })
}

/// The first actor whose role is an ego's.
fn ego_id(parsed: &Parsed, actors: &HashMap<u32, &g::ActorRecord>) -> Option<u32> {
    parsed
        .records
        .iter()
        .find_map(|l| match &l.record {
            Record::Actor(a) if EGO_ROLES.contains(&a.role.to_lowercase().as_str()) => Some(a.id.0),
            _ => None,
        })
        .filter(|id| actors.contains_key(id))
}

fn write_info(
    w: &mut Writer,
    parsed: &Parsed,
    world: Option<&g::WorldRecord>,
    map: Option<&g::MapRecord>,
    warmup: Option<&g::WarmupRecord>,
) -> RrResult<()> {
    let mut md = String::from("# Run\n\n");
    if let Some(world) = world {
        md.push_str(&format!(
            "- CARLA map: `{}`\n- episode: {}\n- format version: {}\n",
            world.map,
            world.episode.0,
            world.schema_version.map_or(1, |v| v.get())
        ));
    }
    if let Some(map) = map {
        md.push_str(&format!(
            "- map source: {}\n",
            map.source.as_deref().unwrap_or("-")
        ));
        if let Some(l2) = &map.lanelet2 {
            md.push_str(&format!(
                "- Lanelet2: `{}` (sha256 {})\n",
                l2.path, *l2.sha256
            ));
        }
        if let Some(od) = &map.opendrive {
            md.push_str(&format!("- OpenDRIVE: `{}`\n", od.path));
        }
    }
    if let Some(warmup) = warmup {
        md.push_str(&format!(
            "- warm-up: {} ticks, frames {:?}..{}\n",
            warmup.ticks,
            warmup.start_frame.as_ref().map(|f| f.0),
            warmup.end_frame.0
        ));
    }
    md.push_str(&format!(
        "- ticks: {}\n- lines: {}\n",
        parsed.ticks.len(),
        parsed.lines
    ));
    w.fixed(
        "info",
        &TextDocument::new(md).with_media_type(rerun::MediaType::markdown()),
    )
}

fn actor_color(a: Option<&g::ActorRecord>) -> Color {
    rgb(match a {
        Some(a) if EGO_ROLES.contains(&a.role.to_lowercase().as_str()) => EGO_COLOR,
        Some(a) if a.type_id.starts_with("walker.") => WALKER_COLOR,
        _ => VEHICLE_COLOR,
    })
}

fn actor_label(id: u32, a: Option<&g::ActorRecord>) -> String {
    match a {
        Some(a) if a.role.is_empty() => format!("{id} {}", *a.type_id),
        Some(a) => format!("{id} {} {}", *a.type_id, a.role),
        None => format!("{id}"),
    }
}

/// The actor columns of one row of `world/actors`: the tick's rows, or (for
/// an `actor_gone` between written ticks) the previous tick's, less the gone.
struct ActorRows {
    frames: Vec<i64>,
    times: Vec<f64>,
    lengths: Vec<usize>,
    centers: Vec<[f32; 3]>,
    quats: Vec<Quaternion>,
    vectors: Vec<[f32; 3]>,
    change_frames: Vec<i64>,
    change_times: Vec<f64>,
    change_lengths: Vec<usize>,
    half_sizes: Vec<[f32; 3]>,
    labels: Vec<String>,
    colors: Vec<Color>,
    ev_frames: Vec<i64>,
    ev_times: Vec<f64>,
    ev_texts: Vec<String>,
    previous: Vec<u32>,
}

impl ActorRows {
    fn row(
        &mut self,
        ticks: &Ticks,
        actors: &HashMap<u32, &g::ActorRecord>,
        (frame, t): (i64, f64),
        rows: impl Iterator<Item = usize>,
        recorded_gone: &std::collections::HashSet<u32>,
    ) {
        let mut ids = Vec::new();
        for r in rows {
            let id = ticks.actor_id[r];
            let [roll, pitch, yaw] = ticks.rotation[r];
            let q = quaternion(roll, pitch, yaw);
            let offset = actors.get(&id).map_or([0.0; 3], |a| a.center.0.into());
            let c = box_center(ticks.position[r], q, offset);
            self.centers.push([c[0] as f32, c[1] as f32, c[2] as f32]);
            self.quats.push(Quaternion::from_xyzw([
                q[0] as f32,
                q[1] as f32,
                q[2] as f32,
                q[3] as f32,
            ]));
            self.vectors.push(point32(ticks.velocity[r]));
            ids.push(id);
        }
        self.frames.push(frame);
        self.times.push(t);
        self.lengths.push(ids.len());
        if ids == self.previous {
            return;
        }
        self.change_frames.push(frame);
        self.change_times.push(t);
        self.change_lengths.push(ids.len());
        for id in &ids {
            let a = actors.get(id).copied();
            let e = a.map_or([0.5; 3], |a| a.extent.0.into());
            self.half_sizes
                .push([e[0] as f32, e[1] as f32, e[2] as f32]);
            self.labels.push(actor_label(*id, a));
            self.colors.push(actor_color(a));
        }
        // Spawned and gone, from who is on the road; an `actor_gone` record
        // says when itself.
        let spawned: Vec<u32> = ids
            .iter()
            .filter(|id| !self.previous.contains(id))
            .copied()
            .collect();
        for id in spawned {
            self.event(
                frame,
                t,
                format!("spawned {}", actor_label(id, actors.get(&id).copied())),
            );
        }
        let gone: Vec<u32> = self
            .previous
            .iter()
            .filter(|id| !ids.contains(id) && !recorded_gone.contains(id))
            .copied()
            .collect();
        for id in gone {
            self.event(
                frame,
                t,
                format!("gone {}", actor_label(id, actors.get(&id).copied())),
            );
        }
        self.previous = ids;
    }

    fn event(&mut self, frame: i64, t: f64, text: String) {
        self.ev_frames.push(frame);
        self.ev_times.push(t);
        self.ev_texts.push(text);
    }
}

fn write_actors(
    w: &mut Writer,
    ticks: &Ticks,
    actors: &HashMap<u32, &g::ActorRecord>,
    gone: &[&g::ActorGoneRecord],
) -> RrResult<()> {
    let n = ticks.actor_id.len();
    if n == 0 {
        return Ok(());
    }
    let mut a = ActorRows {
        frames: Vec::with_capacity(ticks.len()),
        times: Vec::with_capacity(ticks.len()),
        lengths: Vec::with_capacity(ticks.len()),
        centers: Vec::with_capacity(n),
        quats: Vec::with_capacity(n),
        vectors: Vec::with_capacity(n),
        change_frames: Vec::new(),
        change_times: Vec::new(),
        change_lengths: Vec::new(),
        half_sizes: Vec::new(),
        labels: Vec::new(),
        colors: Vec::new(),
        ev_frames: Vec::new(),
        ev_times: Vec::new(),
        ev_texts: Vec::new(),
        previous: Vec::new(),
    };
    let recorded_gone: std::collections::HashSet<u32> = gone.iter().map(|g| g.id.0).collect();
    let mut gone = gone.to_vec();
    gone.sort_by_key(|g| g.frame.0);
    let mut next_gone = 0;
    for i in 0..ticks.len() {
        let frame = ticks.frame[i];
        // Gone on a tick the log left out (no vehicle or walker in it, or
        // before this one): the box goes then, not at the next written tick.
        while next_gone < gone.len() && gone[next_gone].frame.0 <= frame {
            let record = gone[next_gone];
            next_gone += 1;
            let label = actor_label(record.id.0, actors.get(&record.id.0).copied());
            a.event(record.frame.0, record.t, format!("gone {label}"));
            if record.frame.0 < frame && i > 0 && a.previous.contains(&record.id.0) {
                let left: Vec<usize> = ticks
                    .actors(i - 1)
                    .filter(|r| {
                        let id = ticks.actor_id[*r];
                        !gone[..next_gone]
                            .iter()
                            .any(|g| g.id.0 == id && g.frame.0 <= record.frame.0)
                    })
                    .collect();
                a.row(
                    ticks,
                    actors,
                    (record.frame.0, record.t),
                    left.into_iter(),
                    &recorded_gone,
                );
            }
        }
        a.row(
            ticks,
            actors,
            (frame, ticks.t[i]),
            ticks.actors(i),
            &recorded_gone,
        );
    }
    for record in &gone[next_gone..] {
        let label = actor_label(record.id.0, actors.get(&record.id.0).copied());
        a.event(record.frame.0, record.t, format!("gone {label}"));
    }
    let boxes = Boxes3D::update_fields()
        .with_centers(a.centers.iter().copied())
        .with_quaternions(a.quats)
        .columns(a.lengths.iter().copied())
        .map_err(err)?;
    w.columns("world/actors", &a.frames, &a.times, boxes)?;
    let looks = Boxes3D::update_fields()
        .with_half_sizes(a.half_sizes)
        .with_labels(a.labels)
        .with_colors(a.colors)
        .columns(a.change_lengths.iter().copied())
        .map_err(err)?;
    w.columns("world/actors", &a.change_frames, &a.change_times, looks)?;
    let arrows = Arrows3D::update_fields()
        .with_origins(a.centers)
        .with_vectors(a.vectors)
        .columns(a.lengths.iter().copied())
        .map_err(err)?;
    w.columns("world/actors/velocity", &a.frames, &a.times, arrows)?;
    let events = TextLog::update_fields()
        .with_many_text(a.ev_texts)
        .columns_of_unit_batches()
        .map_err(err)?;
    w.columns("world/actors/events", &a.ev_frames, &a.ev_times, events)
}

fn write_ego(w: &mut Writer, ticks: &Ticks, ego: u32) -> RrResult<()> {
    let (mut frames, mut times, mut speeds) = (Vec::new(), Vec::new(), Vec::new());
    let (mut translations, mut rotations) = (Vec::new(), Vec::new());
    let mut path: Vec<[f64; 3]> = Vec::new();
    for i in 0..ticks.len() {
        for r in ticks.actors(i) {
            if ticks.actor_id[r] != ego {
                continue;
            }
            let [vx, vy, vz] = ticks.velocity[r];
            frames.push(ticks.frame[i]);
            times.push(ticks.t[i]);
            speeds.push((vx * vx + vy * vy + vz * vz).sqrt());
            let p = crate::scene::point(ticks.position[r]);
            // The yaw alone: a chase eye that rolls and pitches with the car
            // shakes.
            let q = quaternion(0.0, 0.0, ticks.rotation[r][2]);
            translations.push([p[0] as f32, p[1] as f32, p[2] as f32]);
            rotations.push(Quaternion::from_xyzw([
                q[0] as f32,
                q[1] as f32,
                q[2] as f32,
                q[3] as f32,
            ]));
            let far = path.last().is_none_or(|last| {
                let d = [p[0] - last[0], p[1] - last[1], p[2] - last[2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() >= DRIVEN_PATH_STEP_M
            });
            if far {
                path.push(p);
            }
        }
    }
    let pose = Transform3D::update_fields()
        .with_many_translation(translations)
        .with_many_quaternion(rotations)
        .columns_of_unit_batches()
        .map_err(err)?;
    w.columns(crate::layout::EGO_POSE, &frames, &times, pose)?;
    w.columns(
        "ego/speed_mps",
        &frames,
        &times,
        Scalars::new(speeds)
            .columns_of_unit_batches()
            .map_err(err)?,
    )?;
    if path.len() >= 2 {
        let strip: Vec<[f32; 3]> = path
            .iter()
            .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
            .collect();
        w.fixed(
            "world/ego/driven_path",
            &LineStrips3D::new([strip])
                .with_colors([rgb(DRIVEN_PATH_COLOR)])
                .with_radii([0.08]),
        )?;
    }
    let mine: Vec<_> = ticks.controls.iter().filter(|c| c.id == ego).collect();
    if !mine.is_empty() {
        w.fixed(
            "ego/control",
            &SeriesLines::new().with_names(["throttle", "steer", "brake"]),
        )?;
        let frames: Vec<i64> = mine.iter().map(|c| ticks.frame[c.tick as usize]).collect();
        let times: Vec<f64> = mine.iter().map(|c| ticks.t[c.tick as usize]).collect();
        let values: Vec<f64> = mine
            .iter()
            .flat_map(|c| [c.throttle, c.steer, c.brake])
            .collect();
        let columns = Scalars::new(values)
            .columns(std::iter::repeat_n(3, mine.len()))
            .map_err(err)?;
        w.columns("ego/control", &frames, &times, columns)?;
    }
    Ok(())
}

/// How far a Lanelet2 light or stop line may be from a CARLA light's heads
/// or stop waypoints (in plan) to be taken for it, metres.
pub const LIGHT_MATCH_M: f64 = 3.0;

/// Distance in plan from `p` to the polyline `line`.
fn plan_distance(p: [f64; 3], line: &[[f64; 3]]) -> f64 {
    let d = |a: [f64; 3], b: [f64; 3]| {
        let (vx, vy) = (b[0] - a[0], b[1] - a[1]);
        let len2 = vx * vx + vy * vy;
        let t = if len2 > 0.0 {
            (((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (p[0] - a[0] - t * vx).hypot(p[1] - a[1] - t * vy)
    };
    match line {
        [] => f64::INFINITY,
        [a] => d(*a, *a),
        _ => line
            .windows(2)
            .map(|w| d(w[0], w[1]))
            .fold(f64::INFINITY, f64::min),
    }
}

/// What each CARLA light is called: the ids of the Lanelet2 traffic-light
/// regulatory elements that are its (`1234` or `1234,1240`), else
/// `carla <id>`.
///
/// An element is a light's when one of its lights is within
/// [`LIGHT_MATCH_M`] of one of the light's heads, or one of its stop lines of
/// one of the light's stop waypoints; of several lights, the nearest's.
fn light_names(
    lights: &[&g::TrafficLightRecord],
    map: Option<&MapLayer<'_>>,
) -> HashMap<u32, String> {
    let mut ids: HashMap<u32, Vec<i64>> = HashMap::new();
    if let Some(map) = map {
        let [ox, oy, oz] = map.offset;
        let place = |line: &Vec<[f64; 3]>| -> Vec<[f64; 3]> {
            line.iter()
                .map(|p| [p[0] - ox, p[1] - oy, p[2] - oz])
                .collect()
        };
        // Each light's id, heads and stop waypoints, in the recording's frame.
        type Points = Vec<[f64; 3]>;
        let carla: Vec<(u32, Points, Points)> = lights
            .iter()
            .map(|l| {
                let heads = l
                    .boxes
                    .iter()
                    .map(|b| crate::scene::point([b.0 .0, b.0 .1, b.0 .2]))
                    .collect();
                let stops = l
                    .stops
                    .iter()
                    .map(|s| crate::scene::point([s.0 .0, s.0 .1, s.0 .2]))
                    .collect();
                (l.id.0, heads, stops)
            })
            .collect();
        for element in &map.geometry.traffic_lights {
            let bulbs: Vec<Vec<[f64; 3]>> = element.lights.iter().map(place).collect();
            let stop_lines: Vec<Vec<[f64; 3]>> = element.stop_lines.iter().map(place).collect();
            let nearest = |points: &[[f64; 3]], lines: &[Vec<[f64; 3]>]| {
                points
                    .iter()
                    .flat_map(|p| lines.iter().map(|l| plan_distance(*p, l)))
                    .fold(f64::INFINITY, f64::min)
            };
            let best = carla
                .iter()
                .map(|(id, heads, stops)| {
                    let d = nearest(heads, &bulbs).min(nearest(stops, &stop_lines));
                    (d, *id)
                })
                .filter(|(d, _)| *d <= LIGHT_MATCH_M)
                .min_by(|a, b| a.0.total_cmp(&b.0));
            if let Some((_, id)) = best {
                ids.entry(id).or_default().push(element.id);
            }
        }
    }
    lights
        .iter()
        .map(|l| {
            let id = l.id.0;
            let name = match ids.get(&id) {
                Some(found) => found
                    .iter()
                    .map(|e| e.to_string())
                    .collect::<Vec<_>>()
                    .join(","),
                None => format!("carla {id}"),
            };
            (id, name)
        })
        .collect()
}

/// A light's name, for a light the log did not describe too.
fn light_name(names: &HashMap<u32, String>, id: u32) -> String {
    names
        .get(&id)
        .cloned()
        .unwrap_or_else(|| format!("carla {id}"))
}

/// A light head's three bulbs, top to bottom: red, yellow, green; the one
/// the state names lit, the others dim.
fn bulb_colors(state: LightState) -> [Color; 3] {
    let dim = [rgb(BULB_DIM[0]), rgb(BULB_DIM[1]), rgb(BULB_DIM[2])];
    let lit = |i: usize| {
        let mut c = dim;
        c[i] = light_color(state);
        c
    };
    match state {
        LightState::Red => lit(0),
        LightState::Yellow => lit(1),
        LightState::Green => lit(2),
        LightState::Off => dim,
        LightState::Unknown => [light_color(state); 3],
    }
}

fn write_lights(
    w: &mut Writer,
    ticks: &Ticks,
    lights: &[&g::TrafficLightRecord],
    names: &HashMap<u32, String>,
) -> RrResult<()> {
    if lights.is_empty() {
        return Ok(());
    }
    let (mut centers, mut half_sizes, mut quats) = (Vec::new(), Vec::new(), Vec::new());
    let (mut bulbs, mut bulb_radii) = (Vec::new(), Vec::new());
    let (mut tags, mut stops, mut stop_labels) = (Vec::new(), Vec::new(), Vec::new());
    let mut heads: Vec<(u32, usize)> = Vec::new();
    for light in lights {
        heads.push((light.id.0, light.boxes.len()));
        let mut top = light.location.0 .2;
        for b in &light.boxes {
            let (x, y, z, ex, ey, ez, yaw) = b.0;
            centers.push(point32([x, y, z]));
            half_sizes.push([ex as f32, ey as f32, ez as f32]);
            let q = quaternion(0.0, 0.0, yaw);
            quats.push(Quaternion::from_xyzw([
                q[0] as f32,
                q[1] as f32,
                q[2] as f32,
                q[3] as f32,
            ]));
            // Three bulbs stacked in the head, a third of its height apart.
            let radius = (ex.min(ey).min(ez / 3.0) * 0.9) as f32;
            for k in [1.0, 0.0, -1.0] {
                bulbs.push(point32([x, y, z + k * ez * 2.0 / 3.0]));
                bulb_radii.push(radius);
            }
            top = top.max(z + ez);
        }
        let (x, y, _) = light.location.0;
        tags.push(point32([x, y, top + LIGHT_TAG_HEIGHT_M]));
        for s in &light.stops {
            let (x, y, z, _yaw) = s.0;
            stops.push(point32([x, y, z + 0.05]));
            stop_labels.push(names[&light.id.0].clone());
        }
    }
    if !centers.is_empty() {
        w.fixed(
            "world/traffic_lights",
            &Boxes3D::from_centers_and_half_sizes(centers, half_sizes)
                .with_quaternions(quats)
                .with_colors([rgb(HOUSING_COLOR)])
                .with_fill_mode(rerun::FillMode::MajorWireframe),
        )?;
        // Static but for the colours, which are the lights' states: a static
        // component would hide every temporal one.
        w.fixed(
            "world/traffic_lights/bulbs",
            &Points3D::new(bulbs).with_radii(bulb_radii),
        )?;
    }
    // Where each light's id and state is written; the text and its colour
    // (the light's) change with the state.
    w.fixed(
        "world/traffic_lights/tags",
        &Points3D::new(tags)
            .with_radii([0.05])
            .with_show_labels(true),
    )?;
    if !stops.is_empty() {
        w.fixed(
            "world/traffic_lights/stops",
            &Points3D::new(stops)
                .with_colors([rgb(STOP_LINE_COLOR)])
                .with_radii([0.25])
                .with_labels(stop_labels),
        )?;
    }
    let total: usize = heads.iter().map(|(_, n)| n).sum();
    let mut state: HashMap<u32, LightState> = HashMap::new();
    let (mut frames, mut times, mut colors, mut texts) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut labels, mut label_colors) = (Vec::new(), Vec::new());
    let mut k = 0;
    while k < ticks.lights.len() {
        let tick = ticks.lights[k].tick;
        let mut changed = Vec::new();
        while k < ticks.lights.len() && ticks.lights[k].tick == tick {
            let c = ticks.lights[k];
            state.insert(c.id, c.state);
            changed.push(format!("{}={:?}", light_name(names, c.id), c.state));
            k += 1;
        }
        frames.push(ticks.frame[tick as usize]);
        times.push(ticks.t[tick as usize]);
        for (id, n) in &heads {
            let s = state.get(id).copied().unwrap_or(LightState::Unknown);
            for _ in 0..*n {
                colors.extend(bulb_colors(s));
            }
            labels.push(format!("{} {s}", names[id]));
            label_colors.push(light_color(s));
        }
        texts.push(changed.join(", "));
    }
    if total > 0 {
        let columns = Points3D::update_fields()
            .with_colors(colors)
            .columns(std::iter::repeat_n(3 * total, frames.len()))
            .map_err(err)?;
        w.columns("world/traffic_lights/bulbs", &frames, &times, columns)?;
    }
    let columns = Points3D::update_fields()
        .with_labels(labels)
        .with_colors(label_colors)
        .columns(std::iter::repeat_n(heads.len(), frames.len()))
        .map_err(err)?;
    w.columns("world/traffic_lights/tags", &frames, &times, columns)?;
    let columns = TextLog::update_fields()
        .with_many_text(texts)
        .columns_of_unit_batches()
        .map_err(err)?;
    w.columns("world/traffic_lights/states", &frames, &times, columns)
}

fn write_markers(w: &mut Writer, markers: &[&g::MarkersRecord]) -> RrResult<()> {
    if markers.is_empty() {
        return Ok(());
    }
    let frames: Vec<i64> = markers.iter().map(|m| m.frame.0).collect();
    let times: Vec<f64> = markers.iter().map(|m| m.t).collect();
    let (mut strips, mut colors, mut radii, mut lengths) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut points, mut point_colors, mut point_radii, mut point_lengths) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for m in markers {
        lengths.push(m.polylines.len());
        let mut dots = 0;
        for line in &m.polylines {
            let strip: Vec<[f32; 3]> = line.points.iter().map(|p| point32(p.0.into())).collect();
            let (r, g_, b) = line.color.0;
            colors.push(Color::from_rgb(r, g_, b));
            radii.push((line.thickness / 2.0) as f32);
            if line.point_size > 0.0 {
                dots += strip.len();
                point_colors.extend(std::iter::repeat_n(Color::from_rgb(r, g_, b), strip.len()));
                point_radii.extend(std::iter::repeat_n(
                    (line.point_size / 2.0) as f32,
                    strip.len(),
                ));
                points.extend(strip.iter().copied());
            }
            strips.push(strip);
        }
        point_lengths.push(dots);
    }
    // An empty row clears what the previous one drew: markers hold until the
    // next record, and a record without polylines shows none.
    let columns = LineStrips3D::update_fields()
        .with_strips(strips)
        .with_colors(colors)
        .with_radii(radii)
        .columns(lengths)
        .map_err(err)?;
    w.columns("world/ego/plan", &frames, &times, columns)?;
    let columns = Points3D::update_fields()
        .with_positions(points)
        .with_colors(point_colors)
        .with_radii(point_radii)
        .columns(point_lengths)
        .map_err(err)?;
    w.columns("world/ego/plan/points", &frames, &times, columns)
}

fn write_conditions(
    w: &mut Writer,
    ticks: &Ticks,
    conditions: &BTreeMap<(String, u64), Vec<&g::ConditionRecord>>,
) -> RrResult<()> {
    for ((role, index), records) in conditions {
        let path = format!("conditions/{role}/{index}");
        let label = &records[0].label;
        w.fixed(
            &path,
            &SeriesLines::new().with_names([format!("{role}[{index}] {label}")]),
        )?;
        // The records say when the state changed; the series holds it on
        // every tick after, as the run checked it.
        let (mut frames, mut times, mut values) = (Vec::new(), Vec::new(), Vec::new());
        let mut next = 0;
        let mut current: Option<f64> = None;
        let push = |frame: i64,
                    t: f64,
                    v: f64,
                    frames: &mut Vec<i64>,
                    times: &mut Vec<f64>,
                    values: &mut Vec<f64>| {
            if frames.last() == Some(&frame) {
                *values.last_mut().unwrap() = v;
            } else {
                frames.push(frame);
                times.push(t);
                values.push(v);
            }
        };
        for i in 0..ticks.len() {
            while next < records.len() && records[next].frame.0 <= ticks.frame[i] {
                let r = records[next];
                let v = f64::from(u8::from(r.satisfied));
                push(r.frame.0, r.t, v, &mut frames, &mut times, &mut values);
                current = Some(v);
                next += 1;
            }
            if let Some(v) = current {
                push(
                    ticks.frame[i],
                    ticks.t[i],
                    v,
                    &mut frames,
                    &mut times,
                    &mut values,
                );
            }
        }
        for r in &records[next..] {
            let v = f64::from(u8::from(r.satisfied));
            push(r.frame.0, r.t, v, &mut frames, &mut times, &mut values);
        }
        let columns = Scalars::new(values)
            .columns_of_unit_batches()
            .map_err(err)?;
        w.columns(&path, &frames, &times, columns)?;
    }
    Ok(())
}

fn write_result(w: &mut Writer, ticks: &Ticks, result: &g::ResultRecord) -> RrResult<()> {
    let verdict = if result.passed { "PASSED" } else { "FAILED" };
    let mut md = format!(
        "# {verdict}\n\n{}\n\nElapsed: {:.2} s (simulated)\n\n| role | condition | satisfied | message |\n|---|---|---|---|\n",
        result.message, result.elapsed_seconds
    );
    for c in &result.conditions {
        md.push_str(&format!(
            "| {} | {} | {} | {} |\n",
            c.role,
            c.label,
            c.satisfied,
            c.message.replace('|', "\\|")
        ));
    }
    w.fixed(
        "result",
        &TextDocument::new(md).with_media_type(rerun::MediaType::markdown()),
    )?;
    let (frame, t) = match (ticks.frame.last(), ticks.t.last()) {
        (Some(f), Some(t)) => (*f, *t),
        _ => (0, result.elapsed_seconds),
    };
    let level = if result.passed {
        rerun::TextLogLevel::INFO
    } else {
        rerun::TextLogLevel::WARN
    };
    let columns = TextLog::update_fields()
        .with_text(format!("{verdict}: {}", result.message))
        .with_level(level)
        .columns_of_unit_batches()
        .map_err(err)?;
    w.columns("result/log", &[frame], &[t], columns)?;
    let columns = Scalars::new([f64::from(u8::from(result.passed))])
        .columns_of_unit_batches()
        .map_err(err)?;
    w.columns("result/passed", &[frame], &[t], columns)
}

/// Logs `states` on `path` as a `StateChange` per row, with how the state
/// timeline view shows each value (`(value, label, colour)`, static).
fn write_states(
    w: &mut Writer,
    path: &str,
    styles: &[(&str, String, [u8; 3])],
    frames: &[i64],
    times: &[f64],
    states: Vec<String>,
) -> RrResult<()> {
    if frames.is_empty() {
        return Ok(());
    }
    w.fixed(
        path,
        &StateConfiguration::new()
            .with_values(styles.iter().map(|(v, _, _)| *v))
            .with_labels(styles.iter().map(|(_, l, _)| l.as_str()))
            .with_colors(styles.iter().map(|(_, _, c)| rgb(*c))),
    )?;
    let columns = StateChange::update_fields()
        .with_state(states)
        .columns_of_unit_batches()
        .map_err(err)?;
    w.columns(path, frames, times, columns)
}

/// The run as a whole: running from its first tick, then its verdict.
fn write_progress(w: &mut Writer, ticks: &Ticks, result: Option<&g::ResultRecord>) -> RrResult<()> {
    let (Some(&first), Some(&t0)) = (ticks.frame.first(), ticks.t.first()) else {
        return Ok(());
    };
    let (mut frames, mut times, mut states) = (vec![first], vec![t0], vec!["running".to_string()]);
    if let Some(result) = result {
        let verdict = if result.passed { "passed" } else { "failed" };
        // At the last tick: the run's verdict is what its end decided.
        let (frame, t) = (*ticks.frame.last().unwrap(), *ticks.t.last().unwrap());
        if frame == first {
            states[0] = verdict.to_string();
        } else {
            frames.push(frame);
            times.push(t);
            states.push(verdict.to_string());
        }
    }
    write_states(
        w,
        "scenario/progress",
        &[
            ("running", "running".into(), RUNNING_COLOR),
            ("passed", "PASSED".into(), PASSED_COLOR),
            ("failed", "FAILED".into(), FAILED_COLOR),
        ],
        &frames,
        &times,
        states,
    )
}

/// Each action's lifecycle state, OpenSCENARIO's: standby, the tick it
/// started on, running, the tick it ended on, complete.
fn write_action_states(
    w: &mut Writer,
    actions: &BTreeMap<(String, u64), Vec<&g::ActionRecord>>,
) -> RrResult<()> {
    let names: Vec<Option<String>> = actions
        .values()
        .map(|records| condition_lane(&records[0].label))
        .collect();
    for (((phase, index), records), name) in actions.iter().zip(&names) {
        // Two actions with the same label: their place tells them apart.
        let shared = names.iter().filter(|n| *n == name).count() > 1;
        let lane = match name {
            Some(name) if shared => format!("{name}_{phase}_{index}"),
            Some(name) => name.clone(),
            None => format!("{phase}_{index}"),
        };
        let (mut frames, mut times, mut states) = (Vec::new(), Vec::new(), Vec::new());
        for r in records {
            let state = r.state.to_string();
            if frames.last() == Some(&r.frame.0) {
                *states.last_mut().unwrap() = state;
            } else {
                frames.push(r.frame.0);
                times.push(r.t);
                states.push(state);
            }
        }
        write_states(
            w,
            &format!("scenario/actions/{lane}"),
            &[
                ("standbyState", "standby".into(), UNSATISFIED_COLOR),
                ("startTransition", "start".into(), START_COLOR),
                ("runningState", "running".into(), RUNNING_COLOR),
                ("endTransition", "end".into(), END_COLOR),
                ("completeState", "complete".into(), PASSED_COLOR),
            ],
            &frames,
            &times,
            states,
        )?;
    }
    Ok(())
}

/// A condition's lane name: its label (the view names a lane by its entity
/// path), with what a path part would have to escape -- spaces, brackets --
/// as `_`: `ego stopped AND green` -> `ego_stopped_AND_green`. `None` for a
/// label with nothing left.
fn condition_lane(label: &str) -> Option<String> {
    let mut name = String::new();
    let mut gap = false;
    for c in label.chars() {
        if c.is_alphanumeric() || matches!(c, '-' | '.') {
            if gap && !name.is_empty() {
                name.push('_');
            }
            name.push(c);
            gap = false;
        } else {
            gap = true;
        }
    }
    (!name.is_empty()).then_some(name)
}

/// Each pass/fail condition's state as the run checked it: a pass condition
/// satisfied is green, a fail condition satisfied red.
fn write_condition_states(
    w: &mut Writer,
    conditions: &BTreeMap<(String, u64), Vec<&g::ConditionRecord>>,
) -> RrResult<()> {
    let names: Vec<Option<String>> = conditions
        .values()
        .map(|records| condition_lane(&records[0].label))
        .collect();
    for (((role, index), records), name) in conditions.iter().zip(&names) {
        // Two conditions of a role with the same label: the index tells them apart.
        let shared = conditions
            .keys()
            .zip(&names)
            .filter(|((r, _), n)| r == role && *n == name)
            .count()
            > 1;
        let lane = match name {
            Some(name) if shared => format!("{name}_{index}"),
            Some(name) => name.clone(),
            None => index.to_string(),
        };
        let satisfied = if role == "fail" {
            FAILED_COLOR
        } else {
            PASSED_COLOR
        };
        let (mut frames, mut times, mut states) = (Vec::new(), Vec::new(), Vec::new());
        for r in records {
            let state = if r.satisfied {
                "satisfied"
            } else {
                "not_satisfied"
            };
            // Two records on a frame: the later is the state the tick ended in.
            if frames.last() == Some(&r.frame.0) {
                *states.last_mut().unwrap() = state.to_string();
            } else {
                frames.push(r.frame.0);
                times.push(r.t);
                states.push(state.to_string());
            }
        }
        write_states(
            w,
            &format!("scenario/conditions/{role}/{lane}"),
            &[
                ("satisfied", "satisfied".into(), satisfied),
                ("not_satisfied", "not satisfied".into(), UNSATISFIED_COLOR),
            ],
            &frames,
            &times,
            states,
        )?;
    }
    Ok(())
}

/// Each traffic light's state, from the ticks that change one.
fn write_light_states(w: &mut Writer, ticks: &Ticks, names: &HashMap<u32, String>) -> RrResult<()> {
    type Lane = (Vec<i64>, Vec<f64>, Vec<String>);
    let mut lanes: BTreeMap<u32, Lane> = BTreeMap::new();
    for c in &ticks.lights {
        let (frames, times, states) = lanes.entry(c.id).or_default();
        frames.push(ticks.frame[c.tick as usize]);
        times.push(ticks.t[c.tick as usize]);
        states.push(c.state.to_string());
    }
    let state_names: Vec<(LightState, String)> = [
        LightState::Red,
        LightState::Yellow,
        LightState::Green,
        LightState::Off,
        LightState::Unknown,
    ]
    .into_iter()
    .map(|s| (s, s.to_string()))
    .collect();
    let styles: Vec<(&str, String, [u8; 3])> = state_names
        .iter()
        .map(|(s, name)| {
            let [r, g_, b, _] = light_color(*s).to_array();
            (name.as_str(), name.clone(), [r, g_, b])
        })
        .collect();
    for (id, (frames, times, states)) in lanes {
        write_states(
            w,
            &format!(
                "scenario/traffic_lights/{}",
                light_name(names, id).replace([',', ' '], "_")
            ),
            &styles,
            &frames,
            &times,
            states,
        )?;
    }
    Ok(())
}

fn write_map(w: &mut Writer, map: &MapLayer<'_>) -> RrResult<()> {
    let [ox, oy, oz] = map.offset;
    let place = |p: &[f64; 3]| [(p[0] - ox) as f32, (p[1] - oy) as f32, (p[2] - oz) as f32];
    let g = map.geometry;
    if !g.lane_boundaries.is_empty() {
        let strips: Vec<Vec<[f32; 3]>> = g
            .lane_boundaries
            .iter()
            .map(|b| b.points.iter().map(place).collect())
            .collect();
        let (colors, radii): (Vec<Color>, Vec<f32>) = g
            .lane_boundaries
            .iter()
            .map(|b| {
                let (c, r) = boundary_style(b.kind);
                (rgb(c), r)
            })
            .unzip();
        w.fixed(
            "map/lanelet2/lane_boundaries",
            &LineStrips3D::new(strips)
                .with_colors(colors)
                .with_radii(radii),
        )?;
    }
    if !g.stop_lines.is_empty() {
        let strips: Vec<Vec<[f32; 3]>> = g
            .stop_lines
            .iter()
            .map(|s| s.iter().map(place).collect())
            .collect();
        w.fixed(
            "map/lanelet2/stop_lines",
            &LineStrips3D::new(strips)
                .with_colors([rgb(STOP_LINE_COLOR)])
                .with_radii([0.15]),
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{condition_lane, light_names, MapLayer};
    use crate::generated as g;
    use crate::map::{MapGeometry, TrafficLightElement};

    fn light(id: u32, head: [f64; 3], stop: [f64; 3]) -> g::TrafficLightRecord {
        serde_json::from_value(serde_json::json!({
            "type": "traffic_light", "id": id, "opendrive_id": "1", "location": head,
            "boxes": [[head[0], head[1], head[2], 0.2, 0.2, 0.6, 90.0]],
            "stops": [[stop[0], stop[1], stop[2], 0.0]],
        }))
        .unwrap()
    }

    #[test]
    fn lights_are_named_by_their_lanelet2_elements() {
        // CARLA's y is flipped in the recording, and the map is placed by
        // its offset: a map point (x, y) lands at (x - 10, y - 20).
        let a = light(77, [30.0, 5.0, 5.0], [25.0, 3.0, 0.0]);
        let b = light(78, [-40.0, 0.0, 5.0], [-35.0, 0.0, 0.0]);
        let c = light(79, [100.0, 100.0, 5.0], [95.0, 100.0, 0.0]);
        let element =
            |id, lights: Vec<Vec<[f64; 3]>>, stops: Vec<Vec<[f64; 3]>>| TrafficLightElement {
                id,
                lights,
                stop_lines: stops,
            };
        let geometry = MapGeometry {
            traffic_lights: vec![
                // By its light, 0.5 m from 77's head.
                element(
                    1001,
                    vec![vec![[40.5, 14.0, 6.0], [40.5, 16.0, 6.0]]],
                    vec![],
                ),
                // By its stop line, across 77's stop waypoint.
                element(
                    1002,
                    vec![],
                    vec![vec![[35.0, 15.0, 1.0], [35.0, 19.0, 1.0]]],
                ),
                // Nearer 78 than 77.
                element(1003, vec![vec![[-29.0, 20.0, 6.0]]], vec![]),
                // Nobody's: 50 m from every light.
                element(1004, vec![vec![[90.0, 15.0, 6.0]]], vec![]),
            ],
            ..MapGeometry::default()
        };
        let map = MapLayer {
            geometry: &geometry,
            offset: [10.0, 20.0, 1.0],
        };
        let names = light_names(&[&a, &b, &c], Some(&map));
        assert_eq!(names[&77], "1001,1002");
        assert_eq!(names[&78], "1003");
        assert_eq!(names[&79], "carla 79");
        assert_eq!(light_names(&[&a], None)[&77], "carla 77");
    }

    #[test]
    fn condition_lanes_are_named_by_their_labels() {
        assert_eq!(
            condition_lane("reached_goal").as_deref(),
            Some("reached_goal")
        );
        assert_eq!(
            condition_lane("  ego stopped (in zone) AND green ").as_deref(),
            Some("ego_stopped_in_zone_AND_green")
        );
        assert_eq!(condition_lane(" () "), None);
    }
}
