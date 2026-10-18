"""Converting trajectory logs, and reading the recordings back with Rerun's
dataframe API."""

from __future__ import annotations

import json
import logging
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
        "/world/traffic_lights:Boxes3D:colors",
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

    lights = read_rrd(out, "/world/traffic_lights")
    heads = sum(len(r["boxes"]) for r in records if r["type"] == "traffic_light")
    colors = cells(lights, "/world/traffic_lights:Boxes3D:colors")
    changes = [r for r in ticks if r.get("lights")]
    assert len(colors) == len(changes)
    assert all(len(c) == heads for c in colors)
    red = 0xFF0000FF
    assert set(colors[0].tolist()) == {red}


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
    assert "PASSED" in cells(result, "/result/log:TextLog:text")[0][0]

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
