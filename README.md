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
| `convert_dir(output_dir, *, jobs=None, out_dir=None, map_dir=None, no_map=False, strict=False, parser="serde_json") -> list[Stats \| Failure]` | Every `*.trajectory.jsonl` under a directory (recursively), `jobs` at a time (all cores by default), into `out_dir` (same layout) or beside the logs. A failing log does not stop the others: its entry has `ok: False`, `error` and `error_type`. |
| `ConversionError` > `MalformedLineError` (`.line`), `UnknownRecordTypeError` (`.line`, `.record_type`) | What `convert` raises. |
| `clear_map_cache()` | Forget the Lanelet2 maps read so far. |
| `RERUN_VERSION`, `SCHEMA_VERSION`, `RECORD_TYPES`, `LOGGER_NAME` | |

Both functions release the GIL for all of their work, so a sweep can hand the
previous case to a worker thread and go on with the next. Type stubs ship in
the wheel (`_native.pyi`, `py.typed`); `Stats` is a `TypedDict`: `input`,
`output`, `schema_version`, `bytes`, `lines`, `ticks`, `actors`, `actor_rows`,
`traffic_lights`, `markers`, `conditions`, `has_result`, `map`, `map_cached`,
`unknown_types` (`{type: lines}`), `unknown_first_lines`, `warnings`,
`rrd_bytes`, and the seconds spent parsing, reading the map and writing.

The Rust library has the same entry points
(`autoware_carla_scenario_rerun::{convert, convert_dir, ConvertOptions}`).

### Unknown record types

A record type the vendored schema does not define (a newer ACS) is skipped, so
the rest of the log still converts -- never silently:

- one `logging` warning per type and file, with the count and the first line
  (logger `autoware_carla_scenario_rerun`; in Rust, `log::warn!` with that
  target);
- `stats["unknown_types"]` and `stats["unknown_first_lines"]`;
- a `WARN` `TextLog` under `diagnostics/unknown_types/<type>` in the
  recording, so the viewer shows it too.

`strict=True` raises `UnknownRecordTypeError` instead. Other problems -- a map
that is not on this machine, an empty log -- are warnings the same three ways.

## What a recording holds

Timelines `frame` (the CARLA frame, a sequence) and `sim_time` (the scenario
clock, the log's `t`). Coordinates are CARLA's world frame with `y` flipped
(right-handed, `z` up), as in ACS's in-process recorder.

| Entity | |
|---|---|
| `info` | the run: CARLA map, episode, format version, map files, warm-up (markdown) |
| `world/actors` | every vehicle and walker, one `Boxes3D` batch per tick (centre and rotation each tick; sizes, `id type_id role` labels and colours whenever the set of actors changes) |
| `world/actors/velocity` | their velocities (`Arrows3D` from the box centres) |
| `world/actors/events` | spawned / gone (`TextLog`) |
| `world/ego/driven_path` | the ego's path, a point every 0.5 m (static) |
| `world/ego/plan`, `world/ego/plan/points` | the ego's debug markers (a planner's trajectory: `markers` records), held until the next record |
| `ego/speed_mps`, `ego/control` | speed, and throttle / steer / brake (`Scalars`) |
| `world/traffic_lights` | the light heads as boxes, coloured by their state over time |
| `world/traffic_lights/stops`, `world/traffic_lights/states` | stop-line waypoints; state changes (`TextLog`) |
| `conditions/<pass\|fail>/<index>` | 1 while the condition is satisfied, on every tick from its first check |
| `result`, `result/log`, `result/passed` | the verdict (the `result` record, or `_result.json` for a version-1 log) |
| `map/lanelet2/lane_boundaries`, `map/lanelet2/stop_lines` | the Lanelet2 map (static) |
| `diagnostics/...` | what the conversion skipped or could not find |

### The map

The map is drawn from the log's `map` record (version 2): its Lanelet2 file,
`projector_info`, `mgrs_offset` and `z_offset`. Without one (version 1), or
when the file is not on this machine, `map_dir` is searched for
`<name>/lanelet2_map.osm` (one level deeper too: the layout of ACS's map root
`~/autoware_data/maps`); `map_path` forces a file.

The OSM is read with quick-xml and projected the way ACS reads it
(`coordinate/projection.py`): with the projector `map_projector_info.yaml`
names (`Local` -> Lanelet2's `UtmProjector` at the origin, `MGRS` ->
Autoware's `MGRSProjector`, `TransverseMercator`, `LocalCartesian`), `MGRS`
without one; anchored at the descriptor's origin, else the OpenDRIVE
`geoReference` beside the map, else (0, 0). A point lands at
`(x_map - ox, y_map - oy, z_map - z_offset)`; a version-1 log has no offsets,
so the origin's projection is the offset (what ACS computes) and `z_offset` is
0. The projections (Krüger's series to n^6) match simple-lanelet2's projectors
to 1e-6 m. A map is parsed once per process, keyed by its sha256 from the log
(else its path, size and mtime): `convert_dir` over a sweep reads it once.

## Formats it reads

The log's JSON Schema (draft 2020-12) is vendored as
`schema/trajectory.schema.json`; `schema/SOURCE` records the ACS revision it
came from. Both format versions convert:

- **version 1** (ACS 14.x and earlier): `world`, `traffic_light`, `actor`, `tick`;
- **version 2** (`world.schema_version: 2`): adds `map`, `warmup`,
  `actor_gone`, `markers`, `condition`, `result`.

### Updating the schema

The Rust record types (`src/generated.rs`) are generated from the schema with
[typify](https://github.com/oxidecomputer/typify) by the `xtask` crate; they
are never edited by hand.

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
  controls; light changes): no record, map or string is built per tick. Their
  elements are the *generated* types (`ActorState`, `VehicleControl`,
  `TrafficLightState`: tuples and enums, which allocate nothing), so a schema
  change to them is a compile error, and a property test checks the fast path
  against the generated `TickRecord` for random ticks, chunk sizes and both
  parsers. The other records deserialize into their generated types.
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
pushed `X.Y.Z` tag publishes that version -- the first release, 0.1.0, is cut
this way -- and a manual run is a dry run (with `publish: true`, to TestPyPI).

Publishing uses PyPI Trusted Publishing, no tokens. It needs a (pending)
publisher on PyPI (and on TestPyPI for dry runs) with

| | |
|---|---|
| PyPI project name | `autoware-carla-scenario-rerun` |
| Owner | `hakuturu583` |
| Repository name | `autoware_carla_scenario_rerun` |
| Workflow name | `release.yml` |
| Environment name | `pypi` (TestPyPI: `testpypi`) |

## License

Apache-2.0
