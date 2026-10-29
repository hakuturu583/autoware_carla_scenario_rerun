# autoware_carla_scenario_rerun

Convert [autoware_carla_scenario](https://github.com/autowarefoundation/autoware_carla_scenario)
(ACS) trajectory logs -- the `{case}.trajectory.jsonl` every run writes -- into
[Rerun](https://rerun.io) recordings (`.rrd`), after the run instead of inside it.

The core is Rust (the `rerun` crate's columnar API, serde, rayon); the Python
package is a thin [pyo3](https://pyo3.rs) wrapper, one abi3 wheel per platform
for every CPython 3.10+.

```python
from autoware_carla_scenario_rerun import convert, convert_dir

stats = convert("outputs/CutIn-3.trajectory.jsonl", "CutIn-3.rrd")
results = convert_dir("outputs/2026-10-18/08-48-12", jobs=8, map_dir="/home/me/autoware_data/maps")
```

```bash
pip install autoware-carla-scenario-rerun
pip install 'autoware-carla-scenario-rerun[viewer]'   # + the Rerun viewer of the matching version
rerun outputs/CutIn-3.rrd
```

A whole 24-case sweep converts in under 30 ms (see [Performance](#performance)).

## API

| | |
|---|---|
| `convert(jsonl, out_rrd=None, *, result_json=None, map_dir=None, map_path=None, no_map=False, strict=False, parser="serde_json") -> Stats` | One log. `out_rrd` defaults to `X.rrd` beside `X.trajectory.jsonl`; `result_json` to `X_result.json` beside it (read only for a log without a `result` record). |
| `convert_dir(output_dir, *, jobs=None, out_dir=None, map_dir=None, no_map=False, strict=False, parser="serde_json") -> list[Stats \| Failure]` | Every `*.trajectory.jsonl` under a directory (recursively), `jobs` at a time (all cores by default), into `out_dir` (same layout) or beside the logs. A failing log -- or one whose conversion panics -- does not stop the others: its entry has `ok: False`, `error`, `error_type`, the log as `input` and the file at fault (the log, its `_result.json`, the recording) as `error_path`. |
| `ConversionError` (`.input`, `.path`) > `MalformedLineError` (`.line`), `UnknownRecordTypeError` (`.line`, `.record_type`), `UnknownFieldError` (`.line`, `.field`) | What `convert` raises. Plain Python exceptions: they pickle, attributes included, so a conversion in a `ProcessPoolExecutor` worker raises the same error in the parent. |
| `clear_map_cache()` | Forget the Lanelet2 maps read so far. |
| `RERUN_VERSION`, `SCHEMA_VERSION`, `RECORD_TYPES`, `LOGGER_NAME` | |

Both functions release the GIL for all of their work, so a sweep can hand the
previous case to a worker thread and go on with the next. Type stubs ship in
the wheel (`_native.pyi`, `py.typed`); `Stats` is a `TypedDict`: `input`,
`output`, `schema_version`, `bytes`, `lines`, `ticks`, `actors`, `actor_rows`,
`traffic_lights`, `markers`, `plans`, `conditions`, `actions`, `events`, `has_result`, `map`, `map_cached`,
`unknown_types` (`{type: lines}`), `unknown_first_lines`, `unknown_fields`
(`{record.field: lines}`), `unknown_field_first_lines`, `null_numbers`, `warnings`,
`rrd_bytes`, and the seconds spent parsing, reading the map and writing.

The Rust library has the same entry points
(`autoware_carla_scenario_rerun::{convert, convert_dir, ConvertOptions}`).

### A newer log: unknown record types and fields

A newer ACS may write what the vendored schema does not define. The log still
converts, and never silently:

- a record type it does not know is skipped (if the line is JSON at all);
- a field it does not know -- on any record, nested ones too
  (`map.lanelet2.size`), on a tick, or past the end of an actor row or a
  control (`tick.actors[10..]`) -- is ignored.

Each is reported once per type (or `record.field`) and file, with the number
of lines and the first one:

- a `logging` warning (logger `autoware_carla_scenario_rerun`; in Rust,
  `log::warn!` with that target);
- `stats["unknown_types"]` / `stats["unknown_first_lines"]`, and
  `stats["unknown_fields"]` / `stats["unknown_field_first_lines"]`;
- a `WARN` `TextLog` under `diagnostics/unknown_types/<type>` or
  `diagnostics/unknown_fields/<record.field>` in the recording, so the viewer
  shows it too.

`strict=True` raises `UnknownRecordTypeError` / `UnknownFieldError` instead.
A number of an actor row or a control written as `null` (orjson's NaN) is
read as NaN and counted (`stats["null_numbers"]`, a warning). Other problems
-- a map that is not on this machine or cannot be read, an empty log -- are
warnings the same three ways; a map that cannot be read is retried by the
next conversion. A recording is written to a temporary file and renamed into
place, so a conversion that fails leaves no partial `.rrd`.

## What a recording holds

Timelines `frame` (the CARLA frame, a sequence) and `sim_time` (the scenario
clock, the log's `t`). Coordinates are CARLA's world frame with `y` flipped
(right-handed, `z` up), as in ACS's in-process recorder.

| Entity | |
|---|---|
| `info` | the run: CARLA map, episode, format version, map files, warm-up (markdown) |
| `world/actors` | every vehicle and walker, one `Boxes3D` batch per tick (centre and rotation each tick; sizes, `id type_id role` labels and colours whenever the set of actors changes) |
| `world/actors/velocity` | their velocities (`Arrows3D` from the box centres) |
| `world/actors/events` | spawned / gone (`TextLog`; gone at the `actor_gone` record's frame, the box too, even when that tick was not written) |
| `world/ego/pose` | the ego's position and yaw (`Transform3D`), which the 3D view follows |
| `world/ego/driven_path` | the ego's path, a point every 0.5 m (static) |
| `world/ego/plan`, `world/ego/plan/points` | the ego's debug markers (a planner's trajectory: `markers` records), held until the next record |
| `world/ego/plan/previous` | a driver policy's previous plan (`plan` records), grey, beside the current one: how far each new plan moved from the last |
| `plan_profile/speed` | the current plan's speed against its time ahead (green) and the previous plan's (grey), `LineStrips2D` |
| `plan/inference_seconds`, `plan/debug/<name>` | how long the policy took to plan, and each diagnostic it surfaced with its plans (`Scalars`) |
| `scenario/turn_indicator` | the turn indicators the policy commanded (`off`, `left`, `right`), when a command changes them |
| `ego/speed_mps`, `ego/control` | speed, and throttle / steer / brake (`Scalars`) |
| `world/traffic_lights` | the light heads, as wireframe boxes |
| `world/traffic_lights/bulbs` | three bulbs per head, red / yellow / green top to bottom: the one the light's state names lit, the others dim |
| `world/traffic_lights/tags` | each light's Lanelet2 id and state (`1234 Red`; `carla 77 Red` for a light the map has no element for), written above it in the light's colour |
| `world/traffic_lights/stops`, `world/traffic_lights/states` | stop-line waypoints; state changes (`TextLog`) |
| `conditions/<pass\|fail>/<index>` | 1 while the condition is satisfied, on every tick from its first check |
| `scenario/progress` | the run in the state timeline view (`StateChange`): `running` from the first tick, then `PASSED` / `FAILED` at the last |
| `scenario/actions/<label>` | each action's lifecycle (`action` records): standby, start, running, end, complete -- OpenSCENARIO's states, the start and end held for the tick they happen on; the lane named by its label (two actions with one label get `_<phase>_<index>`, and a name still taken `_2`, `_3`, ...) |
| `scenario/conditions/<pass\|fail>/<label>` | each condition `satisfied` / `not satisfied`, its lane named by its label (`pass/reached_goal`; what a path would have to escape becomes `_`, two conditions with one label get `_<index>`, and a name still taken `_2`, `_3`, ...); a pass condition satisfied is green, a fail condition red |
| `scenario/traffic_lights/<name>` | each light's state (`Red`, `Yellow`, `Green`, ...) over time, the lane named as the tag (`1234`, `carla_77`) |
| `scenario/events` | what else happened, in order (`TextLog`): an adversity NADE sampled, a routed entity reaching a timed waypoint (and how late), a collision between road users other than the ego, a walker fallen (WARN, as a collision), an entity spawned or despawned |
| `result`, `result/log`, `result/passed` | the verdict (the `result` record, or `_result.json` for a version-1 log) |
| `map/lanelet2/lane_boundaries`, `map/lanelet2/stop_lines` | the Lanelet2 map (static) |
| `diagnostics/...` | what the conversion skipped or could not find |

### The layout

A recording carries its blueprint, active when it opens, with three views:

- **Ego** (3D, the top three fifths): the scene from behind the ego,
  following it -- the view's origin is `world/ego/pose`, the eye 14 m behind
  and 6 m above it (still free to orbit);
- **Conditions** (state timeline): `scenario/conditions/**` -- which
  condition held when;
- **Actions** (state timeline): `scenario/actions/**` -- when each action
  was triggered, ran and finished (and, with plans, `scenario/turn_indicator`).

A run with a driver policy's plans (`plan` records) gets a second column, a
third of the width: **Plan speed** (2D, `plan_profile/speed`: the current
plan's speed profile over the previous one's) above **Plan** (time series,
`plan/**`: inference time and diagnostics).

Everything else -- speed, control, the verdict, the lights, the logs -- is
in the recording, to be added from the blueprint panel, which starts open
(the selection and time panels start collapsed, the time panel on
`sim_time`). Without an ego, the 3D view shows the world from above.

### The map

A light is named by the Lanelet2 traffic-light regulatory element(s) that
are its, when the map has them: an element (`type=regulatory_element`,
`subtype=traffic_light`) is a CARLA light's when one of its `refers` lights
is within 3 m (in plan) of one of the light's heads, or one of its
`ref_line` stop lines of one of the light's stop waypoints -- of several
lights, the nearest's. A light with two elements is `1234,1240`; one with
none keeps CARLA's actor id (`carla 77`), as the log has no other.

The file drawn is `map_path` when given, else the Lanelet2 file the log's
`map` record (version 2) names when it is on this machine, else
`<name>/lanelet2_map.osm` under `map_dir` (one level deeper too: the layout
of ACS's map root `~/autoware_data/maps`).

The run's own projection (`projector_type`, `projector_info`, its OpenDRIVE)
and offsets (`mgrs_offset`, `z_offset`) apply only when that file *is* the
run's map: its SHA-256 is the one the `map` record has. Any other file -- an
override, another copy found under `map_dir`, a version-1 log's -- is read
with its own `map_projector_info.yaml` and placed at its origin, with a
warning when the log recorded a different map.

The OSM is read with quick-xml and projected the way ACS reads it
(`coordinate/projection.py`): with the projector `map_projector_info.yaml`
names (`Local` -> Lanelet2's `UtmProjector` at the origin, `MGRS` ->
Autoware's `MGRSProjector`, `TransverseMercator`, `LocalCartesian`), `MGRS`
without one; anchored at the descriptor's origin, else the OpenDRIVE
`geoReference` beside the map, else (0, 0). A point lands at
`(x_map - ox, y_map - oy, z_map - z_offset)`; without the run's offsets, the
origin's projection is the offset (what ACS computes) and `z_offset` is 0. The projections (Krüger's series to n^6) match simple-lanelet2's projectors
to 1e-6 m. A map is parsed once per process, keyed by its sha256 from the log
(else its path, size and mtime): `convert_dir` over a sweep reads it once.

## Formats it reads

The log's JSON Schema (draft 2020-12) is vendored as
`schema/trajectory.schema.json`; `schema/SOURCE` records the ACS revision it
came from. Both format versions convert:

- **version 1** (ACS 14.x and earlier): `world`, `traffic_light`, `actor`, `tick`;
- **version 2** (`world.schema_version: 2`): adds `map`, `warmup`,
  `actor_gone`, `markers`, `condition`, `action`, `result`. (`action` -- each
  action's lifecycle -- comes from
  [ACS#293](https://github.com/autowarefoundation/autoware_carla_scenario/pull/293),
  merged into the version-2 branch, ACS#287.) Later within version 2:
  `adversity`, `waypoint_arrival`, `collision`, `walker_fallen`, `spawn`,
  `despawn` (drawn as `scenario/events`), and `plan` (a driver policy's plan,
  timed: drawn as above). `schema/SOURCE` names the ACS
  commit the vendored schema is from.

### Updating the schema

The Rust record types (`src/generated.rs`) are generated from the schema with
[typify](https://github.com/oxidecomputer/typify) by the `xtask` crate; they
are never edited by hand. Every closed object of the schema is opened on the
way (`additionalProperties: false` -> `{}`), so the generated structs keep an
undeclared field in `extra` instead of refusing the line, and `xtask`
generates an `UnknownFields` impl per struct that reports them by path.

```bash
tools/update_schema.sh ~/workspace/autoware_carla_scenario   # copy, record schema/SOURCE, regenerate
cargo test --workspace
```

`xtask/tests/generated.rs` fails while `src/generated.rs` is not what the
schema generates, and `tests/schema.rs` fails while a record type of the
schema is not handled (`HANDLED_TYPES` in `parse.rs`, then drawing it in
`rrd.rs`). typify reads draft-07, so `xtask` first rewrites the 2020-12
spellings (`$defs`, `prefixItems`) into their draft-07 equivalents.

## Design

- **Reading.** The log is memory-mapped and split into ~1 MiB chunks at
  newlines, parsed in parallel with rayon; each chunk counts its lines, so an
  error names the file's line. A line's `type` is read straight off its start
  (the recorder writes it first; anywhere works). Ticks -- nearly every line
  -- go through a serde `DeserializeSeed` that appends straight into columnar
  buffers (frame, t; one row per actor: id, position, rotation, velocity;
  controls; light changes): no record, map or string is built per tick (a
  field name is copied only for a field the schema does not define). Light
  states and actor ids are the *generated* types; the numbers of a row are
  read one by one so that a `null` is NaN rather than an error. A property
  test checks the fast path against the generated `TickRecord` for random
  ticks, chunk sizes and both parsers. The other records deserialize into
  their generated types. A UTF-8 BOM and whitespace inside the
  `{"type": ...` prefix are accepted; a line of an unknown type must still be
  JSON.
- **Writing.** `rerun::RecordingStreamBuilder::save` writes the file;
  everything temporal goes through `send_columns`, one call per entity (the
  whole run's actor boxes are one chunk, partitioned by tick). What never
  changes is logged static, once.
- **Batches.** `convert_dir` runs the files on a rayon pool (`jobs` threads);
  a file's own chunks share it.
- **JSON parser.** serde_json reading the mapped file in place (the default),
  or simd-json on a per-thread copy of each line (`parser="simd_json"`).
  simd-json wins on long logs, serde_json on the short logs of a typical
  sweep (see below); both give the same recording. (serde_json's default float
  parsing is not correctly rounded past ~15 significant digits; the recorder
  writes 3 decimals, where both are exact.)

## Performance

On a 32-thread machine with a CARLA sweep running alongside (load ~20), Rust
1.99.1, release build:

| | |
|---|---|
| 24-case sweep (866 KB, 5,319 lines, 4,911 ticks), `convert_dir` from Python, all cores, fresh process (Town10HD map parse included) | **17-28 ms** |
| the same, warm (map cached) | 5-10 ms: ~110 MB/s, ~700 k lines/s |
| the same, 1 thread | 57 ms: 2.4 ms per file, **0.012 ms per tick** |
| ACS's in-process Python recorder, for comparison | 0.68-0.96 ms per tick |
| `.rrd` size | 172 KB per case with the Town10HD map (71 KB without); the in-process recorder's: 600-760 KB |

Criterion (`cargo bench`):

| benchmark | time | throughput |
|---|---|---|
| parse a real log (334 ticks, 54 KB): serde_json fast path | 115 us | 452 MiB/s |
| ... simd-json fast path | 141 us | 369 MiB/s |
| ... generated `TickRecord`s | 197 us | 263 MiB/s |
| parse 20k ticks x 12 actors (15.7 MB), serde_json: chunks in parallel / 1 thread | 13.9 ms / 57.6 ms | 1.08 GiB/s / 267 MiB/s |
| ... simd-json: in parallel / 1 thread | 11.1 ms / 52.5 ms | 1.35 GiB/s / 292 MiB/s |
| ... generated `TickRecord`s, 1 thread | 54.3 ms | 283 MiB/s |
| Town10HD Lanelet2 map (2.4 MB OSM): parse and project | 10.8 ms | 209 MiB/s |
| convert the real log, no map | 2.24 ms | |
| convert the 20k-tick log, no map | 38.3 ms | 400 MiB/s |

Against the in-process recorder's own recording of the same run
(`tools/compare_inloop.py`, PedestrianDartOutScenario-5, 126 ticks): the same
rows on `sim_time`; box rotations within 7e-6, speeds and controls within
7e-4 (the log's 3-decimal rounding), the driven path within 0.7 mm, the
Lanelet2 map identical (279 boundaries, 10,283 points). Box centres differ by
up to 4 cm: this converter turns the box offset with the full rotation, the
in-process recorder with the yaw alone.

## Rerun version

Recordings are written with the `rerun` crate **0.38.1** (pinned exactly).
Open them with a Rerun viewer **0.38.x** (`pip install 'rerun-sdk>=0.38.1,<0.39'`,
or the `viewer` extra). A newer viewer usually reads older recordings
(`rerun rrd migrate` upgrades them); an older viewer does not read newer ones.

## Development

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets --features python -- -D warnings
cargo test --workspace
cargo bench                               # ACSR_BENCH_MAP=<lanelet2_map.osm> times a real map
cargo run --release --example convert_dir -- DIR --map-dir D --jobs N --repeat 3

uv venv && uvx maturin develop --uv --release
uv pip install pytest jsonschema 'rerun-sdk[catalog]>=0.38.1,<0.39'
.venv/bin/python -m pytest tests/python
```

The Python tests read every recording back with Rerun's dataframe API (a local
`rerun.server.Server` and its catalog): entity paths, rows per timeline, actor
counts and values against the log; and `rerun rrd verify` loads them.

Installing from a git checkout (no wheel) builds from source and needs a Rust
toolchain:

```bash
pip install "autoware-carla-scenario-rerun @ git+https://github.com/hakuturu583/autoware_carla_scenario_rerun"
```

## Releasing

As in [typesafe_carla](https://github.com/hakuturu583/typesafe_carla): a PR
merged to `main` with exactly one `release:patch` / `release:minor` /
`release:major` label bumps the version (`tools/bump_version.py`: Cargo.toml
and Cargo.lock), tags it `X.Y.Z` and publishes the manylinux_2_28 abi3 wheels
(x86_64, aarch64) and the sdist to PyPI (`.github/workflows/release.yml`). A
pushed `X.Y.Z` tag publishes that version. A manual run (`workflow_dispatch`)
is a dry run: it tests, builds and checks the sdist and both wheels, and
uploads nothing. CI builds the same manylinux wheels on every PR.

The first release, 0.1.0 (the version in Cargo.toml):

1. merge this repository's first PR (no release label);
2. run the Release workflow by hand on `main` and check that the sdist and
   both manylinux wheels build and pass their checks;
3. push the tag `0.1.0` on that commit: the run publishes to PyPI, installs
   the release back from PyPI on both architectures, and publishes the
   GitHub release.

Publishing uses PyPI Trusted Publishing, no tokens: the project's publisher is

| | |
|---|---|
| PyPI project name | `autoware-carla-scenario-rerun` |
| Owner | `hakuturu583` |
| Repository name | `autoware_carla_scenario_rerun` |
| Workflow name | `release.yml` |
| Environment name | `pypi` |

## License

Apache-2.0
