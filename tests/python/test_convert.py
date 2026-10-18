"""Converting trajectory logs, and reading the recordings back with Rerun's
dataframe API."""

from __future__ import annotations

import concurrent.futures
import json
import logging
import multiprocessing
import pickle
import math
import shutil
import threading
import time
from pathlib import Path

import numpy as np
import pytest

import autoware_carla_scenario_rerun as acsr
from conftest import BIG_EPISODE, SCHEMA, TEST_MAP, V1_LOG, v2_records, write_log


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def cells(frame, column: str) -> list:
    return [c for c in frame[column] if c is not None]


# ---------------------------------------------------------------------------
# The schema and the generated types
# ---------------------------------------------------------------------------


def test_record_types_are_the_schemas() -> None:
    schema = json.loads(SCHEMA.read_text())
    consts = [schema["$defs"][ref["$ref"].rsplit("/", 1)[1]]["properties"]["type"]["const"] for ref in schema["oneOf"]]
    assert acsr.RECORD_TYPES == consts
    assert acsr.SCHEMA_VERSION == schema["x-schema-version"]
    assert acsr.RERUN_VERSION.startswith("0.38.")


def test_fixtures_follow_the_schema(v2_log: Path) -> None:
    jsonschema = pytest.importorskip("jsonschema")
    validator = jsonschema.Draft202012Validator(json.loads(SCHEMA.read_text()))
    for path in (V1_LOG, v2_log):
        for record in jsonl(path):
            validator.validate(record)


# ---------------------------------------------------------------------------
# A real version-1 log
# ---------------------------------------------------------------------------


def test_v1_log(v1_log: Path, tmp_path: Path, read_rrd, rrd_columns) -> None:
    out = tmp_path / "v1.rrd"
    stats = acsr.convert(v1_log, out, map_path=TEST_MAP)
    records = jsonl(v1_log)
    ticks = [r for r in records if r["type"] == "tick"]
    assert stats["ok"] and stats["error"] is None
    assert stats["schema_version"] == 1
    assert stats["ticks"] == len(ticks) == 334
    assert stats["lines"] == len(records)
    assert stats["actors"] == 1
    assert stats["traffic_lights"] == 15
    # No result record in version 1: _result.json beside the log is read.
    assert stats["has_result"]
    assert stats["unknown_types"] == {}
    assert out.stat().st_size == stats["rrd_bytes"] > 0

    columns = rrd_columns(out)
    for entity in (
        "/world/actors:Boxes3D:centers",
        "/world/actors/velocity:Arrows3D:vectors",
        "/ego/speed_mps:Scalars:scalars",
        "/ego/control:Scalars:scalars",
        "/world/traffic_lights:Boxes3D:centers",
        "/world/traffic_lights/bulbs:Points3D:positions",
        "/world/traffic_lights/tags:Points3D:labels",
        "/world/traffic_lights/stops:Points3D:positions",
        "/result:TextDocument:text",
        "/result/passed:Scalars:scalars",
        "/map/lanelet2/lane_boundaries:LineStrips3D:strips",
        "/map/lanelet2/stop_lines:LineStrips3D:strips",
        "/info:TextDocument:text",
    ):
        assert entity in columns, entity

    df = read_rrd(out, ["/world/actors", "/world/actors/velocity", "/ego/**"])
    # One row per tick on the frame timeline, at the tick's frame and time.
    assert df["frame"].tolist() == [r["frame"] for r in ticks]
    sim = df["sim_time"].dt.total_seconds().to_numpy()
    assert np.allclose(sim, [r["t"] for r in ticks], atol=1e-6)
    centers = df["/world/actors:Boxes3D:centers"].tolist()
    assert [len(c) for c in centers] == [len(r["actors"]) for r in ticks]
    for row, tick in zip(df["/world/actors/velocity:Arrows3D:vectors"], ticks):
        (_, *_, vx, vy, vz) = tick["actors"][0]
        assert np.allclose(row[0], [vx, -vy, vz], atol=1e-4)
    speeds = [c[0] for c in df["/ego/speed_mps:Scalars:scalars"]]
    expected = [math.hypot(*r["actors"][0][7:10]) for r in ticks]
    assert np.allclose(speeds, expected, atol=1e-4)
    controls = df["/ego/control:Scalars:scalars"].tolist()
    assert np.allclose(controls[5], ticks[5]["control"]["78"][:3])
    # The box: its centre is the origin plus the turned offset, y flipped.
    x, y, z = ticks[0]["actors"][0][1:4]
    actor = next(r for r in records if r["type"] == "actor")
    cx, cy, cz = centers[0][0]
    assert abs(cx - x) < 0.05 and abs(cy + y) < 0.05 and abs(cz - (z + actor["center"][2])) < 0.05
    labels = cells(df, "/world/actors:Boxes3D:labels")
    assert labels == [["78 vehicle.lincoln.mkz Ego"]]

    # The heads as wireframe boxes; three bulbs each, the one the state
    # names lit; each light's id and state above it.
    light_records = [r for r in records if r["type"] == "traffic_light"]
    heads = sum(len(r["boxes"]) for r in light_records)
    housings = read_rrd(out, "/world/traffic_lights", index=None)
    assert len(housings["/world/traffic_lights:Boxes3D:half_sizes"][0]) == heads
    lights = read_rrd(out, ["/world/traffic_lights/bulbs", "/world/traffic_lights/tags"])
    colors = cells(lights, "/world/traffic_lights/bulbs:Points3D:colors")
    changes = [r for r in ticks if r.get("lights")]
    assert len(colors) == len(changes)
    assert all(len(c) == 3 * heads for c in colors)
    red, dim_yellow, dim_green, green = 0xFF0000FF, 0x463800FF, 0x003E00FF, 0x00DC00FF
    assert colors[0].tolist()[:3] == [red, dim_yellow, dim_green]  # every light red
    assert colors[-1].tolist()[-3:] == [0x460000FF, dim_yellow, green]
    tags = cells(lights, "/world/traffic_lights/tags:Points3D:labels")
    # No light of the test map is near these (Town10HD's): named by CARLA's ids.
    assert list(tags[0]) == [f"carla {r['id']} Red" for r in light_records]
    assert list(tags[-1]) == [f"carla {r['id']} Green" for r in light_records]
    tag_colors = cells(lights, "/world/traffic_lights/tags:Points3D:colors")
    assert set(tag_colors[0].tolist()) == {red} and set(tag_colors[-1].tolist()) == {green}

    # Each light's state for the state timeline view; the run fails at its end.
    scenario = read_rrd(out, "/scenario/**")
    lanes = {c.split(":")[0] for c in scenario.columns if c.endswith(":StateChange:state")}
    ids = {r["id"] for r in records if r["type"] == "traffic_light"}
    assert lanes == {"/scenario/progress"} | {f"/scenario/traffic_lights/carla_{i}" for i in ids}
    progress = cells(scenario, "/scenario/progress:StateChange:state")
    assert [p[0] for p in progress] == ["running", "failed"]


# ---------------------------------------------------------------------------
# Version 2: every record type
# ---------------------------------------------------------------------------


def test_v2_log(v2_log: Path, tmp_path: Path, read_rrd) -> None:
    out = tmp_path / "v2.rrd"
    stats = acsr.convert(v2_log, out)
    assert stats["schema_version"] == 2
    assert stats["ticks"] == 40
    assert stats["actors"] == 3
    assert stats["markers"] == 2
    assert stats["conditions"] == 3
    assert stats["has_result"]
    # The map record names the file, and its offsets place it.
    assert stats["map"] == str(TEST_MAP)
    assert stats["warnings"] == []

    df = read_rrd(out, ["/world/actors", "/world/actors/events"])
    counts = [len(c) for c in df["/world/actors:Boxes3D:centers"]]
    assert counts == [2] * 10 + [3] * 10 + [2] * 20
    # Sizes and labels go again whenever who is on the road changes.
    labels = cells(df, "/world/actors:Boxes3D:labels")
    assert [list(lbl) for lbl in labels] == [
        ["1 vehicle.lincoln.mkz ego", "2 vehicle.tesla.model3"],
        ["1 vehicle.lincoln.mkz ego", "2 vehicle.tesla.model3", "3 walker.pedestrian.0001 pedestrian"],
        ["1 vehicle.lincoln.mkz ego", "3 walker.pedestrian.0001 pedestrian"],
    ]
    events = [list(e) for e in cells(df, "/world/actors/events:TextLog:text")]
    assert ["gone 2 vehicle.tesla.model3"] in events

    plan = read_rrd(out, ["/world/ego/plan", "/world/ego/plan/points"])
    strips = plan["/world/ego/plan:LineStrips3D:strips"].tolist()
    assert len(strips[0]) == 2 and len(strips[1]) == 0  # set at frame 105, cleared at 115
    assert plan["frame"].tolist() == [105, 115]
    assert np.allclose(strips[0][0][2], [20.0, -3.0, 0.2])
    points = plan["/world/ego/plan/points:Points3D:positions"].tolist()
    assert len(points[0]) == 2 and len(points[1]) == 0

    cond = read_rrd(out, "/conditions/pass/0")
    values = [c[0] for c in cond["/conditions/pass/0:Scalars:scalars"]]
    # Carried forward on every tick: 0 until frame 130, 1 from then on.
    assert len(values) == 40
    assert values[:30] == [0.0] * 30 and values[30:] == [1.0] * 10

    result = read_rrd(out, ["/result/log", "/result/passed"])
    assert cells(result, "/result/passed:Scalars:scalars")[0][0] == 1.0

    # The state timeline view: the run, its conditions and its lights.
    scenario = read_rrd(out, "/scenario/**")

    def states(entity: str) -> list[tuple[int, str]]:
        column = f"/scenario/{entity}:StateChange:state"
        return [(f, c[0]) for f, c in zip(scenario["frame"], scenario[column]) if c is not None]
    assert states("progress") == [(100, "running"), (139, "passed")]
    # A condition's lane is named by its label.
    assert states("conditions/pass/reached_goal") == [(100, "not_satisfied"), (130, "satisfied")]
    assert states("conditions/fail/collision") == [(100, "not_satisfied")]
    # Light 9 is the map's traffic-light element 30: named by its Lanelet2 id.
    assert states("traffic_lights/30") == [(100, "Red"), (125, "Green")]
    tags = read_rrd(out, "/world/traffic_lights/tags")
    assert [list(t) for t in cells(tags, "/world/traffic_lights/tags:Points3D:labels")] == [["30 Red"], ["30 Green"]]
    style = read_rrd(out, "/scenario/conditions/pass/reached_goal", index=None)
    labels = style["/scenario/conditions/pass/reached_goal:StateConfiguration:labels"][0]
    assert list(labels) == ["satisfied", "not satisfied"]
    assert "PASSED" in cells(result, "/result/log:TextLog:text")[0][0]

    # The ego's pose, which the chase view follows: its position and its yaw.
    pose = read_rrd(out, "/world/ego/pose")
    translations = pose["/world/ego/pose:Transform3D:translation"].tolist()
    assert len(translations) == 40 and np.allclose(translations[3][0], [3.0, -2.0, 0.1])
    (x, y, z, w) = pose["/world/ego/pose:Transform3D:quaternion"][3][0]
    assert abs(x) < 1e-6 and abs(y) < 1e-6
    assert math.isclose(math.degrees(2 * math.atan2(z, w)), -13.0, abs_tol=1e-3)  # yaw 13, y flipped

    path = read_rrd(out, "/world/ego/driven_path", index=None)
    strip = path["/world/ego/driven_path:LineStrips3D:strips"][0][0]
    # The ego drives 1 m a tick: a point every tick, y flipped.
    assert len(strip) == 40 and np.allclose(strip[3], [3.0, -2.0, 0.1])

    info = read_rrd(out, "/info", index=None)
    assert str(BIG_EPISODE) in info["/info:TextDocument:text"][0][0]

    lanes = read_rrd(out, "/map/lanelet2/**", index=None)
    bounds = lanes["/map/lanelet2/lane_boundaries:LineStrips3D:strips"][0]
    assert len(bounds) == 3  # ways 11, 10, 12: each once
    # Node 1 (lat 0, lon 0) is the map origin: x_map - ox, y_map - oy, z - z_offset.
    first = bounds[1][0]
    assert np.allclose(first, [-10.0, -20.0, -1.0], atol=1e-4)
    assert len(lanes["/map/lanelet2/stop_lines:LineStrips3D:strips"][0]) == 1


def test_simd_json_gives_the_same_recording(v2_log: Path, tmp_path: Path, read_rrd) -> None:
    a = acsr.convert(v2_log, tmp_path / "a.rrd", parser="serde_json")
    b = acsr.convert(v2_log, tmp_path / "b.rrd", parser="simd_json")
    for key in ("ticks", "actors", "actor_rows", "markers", "conditions", "lines"):
        assert a[key] == b[key]
    da = read_rrd(tmp_path / "a.rrd", "/world/**")
    db = read_rrd(tmp_path / "b.rrd", "/world/**")
    assert da.drop(columns="rerun_segment_id").to_json(date_format="iso") == db.drop(
        columns="rerun_segment_id"
    ).to_json(date_format="iso")


# ---------------------------------------------------------------------------
# Unknown record types, malformed lines, empty files
# ---------------------------------------------------------------------------


def test_unknown_types_warn_once_per_type(tmp_path: Path, caplog, read_rrd) -> None:
    extra = ['{"type":"telemetry","x":1}'] * 3 + ['{"type":"future_thing"}']
    log = write_log(tmp_path / "Case-7.trajectory.jsonl", v2_records(ticks=5), extra)
    first_line = len(v2_records(ticks=5)) + 1
    with caplog.at_level(logging.WARNING, logger=acsr.LOGGER_NAME):
        stats = acsr.convert(log, tmp_path / "Case-7.rrd")
    assert stats["unknown_types"] == {"telemetry": 3, "future_thing": 1}
    assert stats["unknown_first_lines"] == {"telemetry": first_line, "future_thing": first_line + 3}
    warnings = [r for r in caplog.records if r.name == acsr.LOGGER_NAME]
    assert len(warnings) == 2, [r.getMessage() for r in warnings]
    by_type = {("telemetry" if "`telemetry`" in r.getMessage() else "future_thing"): r for r in warnings}
    assert all(r.levelno == logging.WARNING for r in warnings)
    message = by_type["telemetry"].getMessage()
    assert "Case-7.trajectory.jsonl" in message and "3 line(s)" in message and f"line {first_line}" in message
    # ... and in the recording, for whoever opens it in the viewer.
    diag = read_rrd(tmp_path / "Case-7.rrd", "/diagnostics/**", index=None)
    text = diag["/diagnostics/unknown_types/telemetry:TextLog:text"][0][0]
    assert "telemetry" in text and "3 line(s)" in text
    assert diag["/diagnostics/unknown_types/telemetry:TextLog:level"][0][0] == "WARN"


def test_strict_refuses_unknown_types(tmp_path: Path) -> None:
    log = write_log(tmp_path / "x.trajectory.jsonl", v2_records(ticks=3), ['{"type":"telemetry"}'])
    with pytest.raises(acsr.UnknownRecordTypeError) as info:
        acsr.convert(log, tmp_path / "x.rrd", strict=True)
    assert info.value.line == len(v2_records(ticks=3)) + 1
    assert info.value.record_type == "telemetry"
    assert isinstance(info.value, acsr.ConversionError)
    assert not (tmp_path / "x.rrd").exists()


@pytest.mark.parametrize(
    ("bad", "expect"),
    [
        ("{not json", "line"),
        ('{"type":"tick","frame":"x","t":0,"actors":[],"control":{}}', "invalid type"),
        ('{"type":"tick","frame":1,"t":0,"control":{}}', "missing field `actors`"),
        ('{"type":"actor","id":1}', "missing field"),
        ('{"frame":1}', "not a trajectory record"),
    ],
)
def test_malformed_lines_say_which(tmp_path: Path, bad: str, expect: str) -> None:
    records = v2_records(ticks=3)
    log = write_log(tmp_path / "bad.trajectory.jsonl", records, [bad])
    with pytest.raises(acsr.MalformedLineError) as info:
        acsr.convert(log, tmp_path / "bad.rrd")
    line = len(records) + 1
    assert info.value.line == line
    assert f":{line}: " in str(info.value) and expect in str(info.value)


def test_empty_file(tmp_path: Path, read_rrd) -> None:
    log = tmp_path / "Empty.trajectory.jsonl"
    log.write_text("")
    stats = acsr.convert(log)
    assert stats["output"] == str(tmp_path / "Empty.rrd")
    assert stats["ticks"] == 0 and stats["lines"] == 0
    assert stats["warnings"] == ["the trajectory log is empty"]
    diag = read_rrd(tmp_path / "Empty.rrd", "/diagnostics/**", index=None)
    assert "empty" in diag["/diagnostics/empty:TextLog:text"][0][0]


def test_missing_file_is_a_conversion_error(tmp_path: Path) -> None:
    with pytest.raises(acsr.ConversionError):
        acsr.convert(tmp_path / "none.trajectory.jsonl", tmp_path / "none.rrd")


def test_missing_map_is_a_warning(tmp_path: Path) -> None:
    records = v2_records(ticks=3)
    records[1]["lanelet2"]["path"] = "/nowhere/lanelet2_map.osm"
    log = write_log(tmp_path / "m.trajectory.jsonl", records)
    stats = acsr.convert(log, tmp_path / "m.rrd")
    assert stats["map"] is None
    assert any("not on this machine" in w for w in stats["warnings"])
    # map_dir finds it by the CARLA map's name.
    stats = acsr.convert(log, tmp_path / "m.rrd", map_dir=TEST_MAP.parent.parent)
    assert stats["map"] == str(TEST_MAP)


# ---------------------------------------------------------------------------
# A whole output directory
# ---------------------------------------------------------------------------


def test_convert_dir(tmp_path: Path, read_rrd) -> None:
    acsr.clear_map_cache()
    runs = tmp_path / "outputs" / "2026-10-18" / "08-48-12"
    for i in range(6):
        write_log(runs / f"Case-{i}.trajectory.jsonl", v2_records(ticks=10 + i))
    shutil.copy(V1_LOG, runs / V1_LOG.name)
    write_log(runs / "nested" / "Broken.trajectory.jsonl", v2_records(ticks=2), ["{oops"])
    results = acsr.convert_dir(tmp_path / "outputs", jobs=3, out_dir=tmp_path / "rrd", map_dir=TEST_MAP.parent.parent)
    assert len(results) == 8
    failed = [r for r in results if not r["ok"]]
    assert len(failed) == 1 and failed[0]["error_type"] == "MalformedLineError"
    assert failed[0]["input"].endswith("Broken.trajectory.jsonl")
    ok = [r for r in results if r["ok"]]
    assert sorted(r["ticks"] for r in ok if r["schema_version"] == 2) == list(range(10, 16))
    for r in ok:
        assert Path(r["output"]).is_file()
        assert Path(r["output"]).is_relative_to(tmp_path / "rrd" / "2026-10-18" / "08-48-12")
    # One map for all of them, read once. (The real log's Town10HD_Opt is not
    # under map_dir: a warning, and no map.)
    with_map = [r for r in ok if r["map"]]
    assert len(with_map) == 6
    town10 = next(r for r in ok if r["schema_version"] == 1)
    assert any("no Lanelet2 map of Town10HD_Opt" in w for w in town10["warnings"])
    assert sum(not r["map_cached"] for r in with_map) == 1


# ---------------------------------------------------------------------------
# The GIL
# ---------------------------------------------------------------------------


def test_conversion_releases_the_gil(tmp_path: Path) -> None:
    records = v2_records(ticks=3)
    head, tail = records[:4], records[-1]
    lines = []
    for i in range(60_000):
        actors = [[a, i * 0.1, a * 3.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0] for a in range(1, 13)]
        lines.append(json.dumps({"type": "tick", "frame": 100 + i, "t": round(i * 0.05, 3), "actors": actors,
                                 "control": {"1": [0.1, 0.0, 0.0, 1]}}))
    log = write_log(tmp_path / "Big.trajectory.jsonl", [*head, tail], lines)

    counter = 0
    done = threading.Event()

    def count() -> None:
        nonlocal counter
        while not done.is_set():
            counter += 1

    worker = threading.Thread(target=count)
    worker.start()
    try:
        time.sleep(0.2)
        # How fast the thread counts with the main thread idle.
        rate = counter / 0.2
        before = counter
        start = time.perf_counter()
        acsr.convert(log, tmp_path / "Big.rrd", no_map=True)
        elapsed = time.perf_counter() - start
        during = counter - before
    finally:
        done.set()
        worker.join()
    # Holding the GIL for the call would stop the counter for all of it (it
    # could only run up to the call, within one switch interval, 5 ms).
    assert elapsed > 0.05, elapsed
    assert during > 0.2 * rate * elapsed, (during, rate, elapsed)


def test_rerun_verifies_the_recordings(v1_log: Path, v2_log: Path, tmp_path: Path) -> None:
    """`rerun rrd verify` (the viewer's own loader) reads both back cleanly."""
    import subprocess
    import sys

    outs = [tmp_path / "v1.rrd", tmp_path / "v2.rrd"]
    acsr.convert(v1_log, outs[0], map_path=TEST_MAP)
    acsr.convert(v2_log, outs[1])
    rerun = Path(sys.executable).with_name("rerun")
    if not rerun.exists():
        pytest.skip("no rerun executable beside the interpreter")
    done = subprocess.run([str(rerun), "rrd", "verify", *map(str, outs)], capture_output=True, text=True)
    assert done.returncode == 0, done.stdout + done.stderr


def test_recordings_open_with_their_layout(v2_log: Path, tmp_path: Path) -> None:
    """The blueprint is in the file, active: a chase view on the ego and the
    conditions' state timeline; the rest is left to the blueprint panel."""
    import subprocess
    import sys

    out = tmp_path / "v2.rrd"
    acsr.convert(v2_log, out)
    rerun = Path(sys.executable).with_name("rerun")
    if not rerun.exists():
        pytest.skip("no rerun executable beside the interpreter")
    done = subprocess.run([str(rerun), "rrd", "print", "--verbose", str(out)], capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    text = done.stdout
    assert f'StoreId(Blueprint, "autoware_carla_scenario", "{v2_log.name.split(".")[0]}.blueprint"), make_active: true' in text
    assert text.count("ViewBlueprint:class_identifier") == 2
    for expected in ("/world/ego/pose@Transform3D", "EyeControls3D:position", "EyeControls3D:look_target"):
        assert expected in text, expected


# ---------------------------------------------------------------------------
# Forward compatibility: fields the schema does not define, nulls
# ---------------------------------------------------------------------------


def test_unknown_fields_warn_and_convert(tmp_path: Path, caplog, read_rrd) -> None:
    records = v2_records(ticks=6)
    records[0]["weather"] = "rain"  # world
    records[1]["lanelet2"]["mtime"] = 3  # map -> FileRef
    for r in records:
        if r["type"] == "tick":
            r["gnss"] = [1, 2]
    log = write_log(tmp_path / "New.trajectory.jsonl", records)
    ticks_from = next(i for i, r in enumerate(records) if r["type"] == "tick") + 1
    with caplog.at_level(logging.WARNING, logger=acsr.LOGGER_NAME):
        stats = acsr.convert(log, tmp_path / "New.rrd")
    assert stats["ticks"] == 6
    assert stats["unknown_fields"] == {"map.lanelet2.mtime": 1, "tick.gnss": 6, "world.weather": 1}
    assert stats["unknown_field_first_lines"] == {"map.lanelet2.mtime": 2, "tick.gnss": ticks_from, "world.weather": 1}
    messages = [r.getMessage() for r in caplog.records if r.name == acsr.LOGGER_NAME]
    assert sum("`tick.gnss` on 6 line(s)" in m for m in messages) == 1
    diag = read_rrd(tmp_path / "New.rrd", "/diagnostics/**", index=None)
    assert "world.weather" in diag["/diagnostics/unknown_fields/world.weather:TextLog:text"][0][0]
    with pytest.raises(acsr.UnknownFieldError) as info:
        acsr.convert(log, tmp_path / "New.rrd", strict=True)
    assert (info.value.line, info.value.field) == (1, "world.weather")


def test_null_numbers_are_nan(tmp_path: Path, read_rrd) -> None:
    records = v2_records(ticks=4)
    tick = next(r for r in records if r["type"] == "tick" and r["frame"] == 102)
    tick["actors"][0][7] = None  # vx, as orjson writes NaN
    tick["actors"][0][1] = None
    log = write_log(tmp_path / "Nan.trajectory.jsonl", records)
    stats = acsr.convert(log, tmp_path / "Nan.rrd")
    assert stats["null_numbers"] == 2 and stats["ticks"] == 4
    assert any("null" in w for w in stats["warnings"])
    df = read_rrd(tmp_path / "Nan.rrd", "/world/actors/velocity")
    row = df[df["frame"] == 102]["/world/actors/velocity:Arrows3D:vectors"].iloc[0]
    assert math.isnan(row[0][0])


# ---------------------------------------------------------------------------
# Which map, and how it is placed
# ---------------------------------------------------------------------------


def test_another_map_is_placed_by_itself(tmp_path: Path, read_rrd) -> None:
    other = tmp_path / "maps" / "TestTown" / "lanelet2_map.osm"
    other.parent.mkdir(parents=True)
    other.write_text(TEST_MAP.read_text().replace("<!--", "<!-- another copy;"))
    (other.parent / "map_projector_info.yaml").write_text("projector_type: Local\n")
    log = write_log(tmp_path / "o.trajectory.jsonl", v2_records(ticks=3))
    for kwargs in ({"map_path": other}, {}):
        if not kwargs:
            # The recorded file is gone: map_dir finds the other copy.
            records = v2_records(ticks=3)
            records[1]["lanelet2"]["path"] = "/nowhere/lanelet2_map.osm"
            log = write_log(tmp_path / "o.trajectory.jsonl", records)
            kwargs = {"map_dir": tmp_path / "maps"}
        stats = acsr.convert(log, tmp_path / "o.rrd", **kwargs)
        assert stats["map"] == str(other)
        assert any("is not the Lanelet2 map the run recorded" in w for w in stats["warnings"]), stats["warnings"]
        lanes = read_rrd(tmp_path / "o.rrd", "/map/lanelet2/**", index=None)
        # Not shifted by the run's mgrs_offset/z_offset (10, 20, 1): its own origin.
        first = lanes["/map/lanelet2/lane_boundaries:LineStrips3D:strips"][0][1][0]
        assert np.allclose(first, [0.0, 0.0, 0.0], atol=1e-4)


def test_an_unreadable_map_is_a_warning(tmp_path: Path) -> None:
    broken = tmp_path / "broken" / "TestTown" / "lanelet2_map.osm"
    broken.parent.mkdir(parents=True)
    broken.write_text("<osm><node id='1' lat='0' lon='0'></way></osm>")
    log = write_log(tmp_path / "b.trajectory.jsonl", v2_records(ticks=3))
    stats = acsr.convert(log, tmp_path / "b.rrd", map_path=broken)
    assert stats["map"] is None and stats["ticks"] == 3
    assert any("no map drawn" in w for w in stats["warnings"])
    # Not remembered: once the file is fixed, the next conversion draws it.
    shutil.copy(TEST_MAP, broken)
    assert acsr.convert(log, tmp_path / "b.rrd", map_path=broken)["map"] == str(broken)


# ---------------------------------------------------------------------------
# Errors: what they say, that they pickle, and that nothing half-written stays
# ---------------------------------------------------------------------------


def test_errors_pickle_with_their_attributes(tmp_path: Path) -> None:
    log = write_log(tmp_path / "bad.trajectory.jsonl", v2_records(ticks=2), ["{oops"])
    with pytest.raises(acsr.MalformedLineError) as info:
        acsr.convert(log, tmp_path / "bad.rrd")
    again = pickle.loads(pickle.dumps(info.value))
    assert type(again) is acsr.MalformedLineError
    assert (again.line, again.input, again.path, str(again)) == (
        info.value.line, str(log), str(log), str(info.value))
    # Raised in a worker process, it arrives whole.
    context = multiprocessing.get_context("spawn")
    with concurrent.futures.ProcessPoolExecutor(1, mp_context=context) as pool:
        with pytest.raises(acsr.MalformedLineError) as remote:
            pool.submit(acsr.convert, str(log), str(tmp_path / "bad.rrd")).result()
        assert remote.value.line == info.value.line
        good = write_log(tmp_path / "good.trajectory.jsonl", v2_records(ticks=2))
        stats = pool.submit(acsr.convert, str(good), str(tmp_path / "good.rrd")).result()
        assert stats["ticks"] == 2
    log2 = write_log(tmp_path / "s.trajectory.jsonl", v2_records(ticks=2), ['{"type":"x"}'])
    with pytest.raises(acsr.UnknownRecordTypeError) as info2:
        acsr.convert(log2, strict=True)
    again = pickle.loads(pickle.dumps(info2.value))
    assert (again.record_type, again.line) == ("x", info2.value.line)


def test_failures_name_the_log_and_the_file_at_fault(tmp_path: Path) -> None:
    runs = tmp_path / "runs"
    log = write_log(runs / "R-1.trajectory.jsonl", v2_records(ticks=2)[:4] + v2_records(ticks=2)[4:-1])
    (runs / "R-1_result.json").write_text("{not json")
    results = acsr.convert_dir(runs, no_map=True)
    (failure,) = results
    assert not failure["ok"] and failure["error_type"] == "ConversionError"
    assert failure["input"] == str(log)
    assert failure["error_path"] == str(runs / "R-1_result.json")
    with pytest.raises(acsr.ConversionError) as info:
        acsr.convert(log, no_map=True)
    assert info.value.input == str(log) and info.value.path == str(runs / "R-1_result.json")


def test_no_partial_recording_is_left(tmp_path: Path) -> None:
    out = tmp_path / "x.rrd"
    out.mkdir()
    (out / "keep").write_text("")
    log = write_log(tmp_path / "x.trajectory.jsonl", v2_records(ticks=2))
    with pytest.raises(acsr.ConversionError):
        acsr.convert(log, out)
    assert sorted(p.name for p in tmp_path.iterdir()) == ["x.rrd", "x.trajectory.jsonl"]


# ---------------------------------------------------------------------------
# actor_gone between written ticks
# ---------------------------------------------------------------------------


def test_actor_gone_takes_effect_at_its_own_frame(tmp_path: Path, read_rrd) -> None:
    records = v2_records(ticks=30)
    # The actor goes on frame 119, a tick the log left out (frame 119 has no
    # tick record): its box goes then, not at frame 120.
    for r in records:
        if r["type"] == "actor_gone":
            r["frame"], r["t"] = 119, 0.95
    records = [r for r in records if not (r["type"] == "tick" and r["frame"] == 119)]
    log = write_log(tmp_path / "g.trajectory.jsonl", records)
    acsr.convert(log, tmp_path / "g.rrd", no_map=True)
    df = read_rrd(tmp_path / "g.rrd", ["/world/actors", "/world/actors/events"])
    at = df.set_index("frame")
    assert len(at.loc[118, "/world/actors:Boxes3D:centers"]) == 3
    assert len(at.loc[119, "/world/actors:Boxes3D:centers"]) == 2
    assert list(at.loc[119, "/world/actors/events:TextLog:text"]) == ["gone 2 vehicle.tesla.model3"]
    events = [e for c in df["/world/actors/events:TextLog:text"] if c is not None for e in c]
    assert events.count("gone 2 vehicle.tesla.model3") == 1
