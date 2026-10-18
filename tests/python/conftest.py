"""Fixtures: trajectory logs of both format versions, and reading recordings
back with Rerun's dataframe API."""

from __future__ import annotations

import json
import shutil
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "tests" / "data"
SCHEMA = ROOT / "schema" / "trajectory.schema.json"
#: A real version-1 log (an ACS 14.4 traffic-light sweep case) and its result.
V1_LOG = DATA / "v1" / "TrafficLightComplianceScenario-1.trajectory.jsonl"
V1_RESULT = DATA / "v1" / "TrafficLightComplianceScenario-1_result.json"
TEST_MAP = DATA / "maps" / "TestTown" / "lanelet2_map.osm"
#: An episode id past 2^53: version 2 says it is a full uint64.
BIG_EPISODE = 18_446_744_073_709_551_000


def v2_records(map_path: Path = TEST_MAP, ticks: int = 40) -> list[dict[str, Any]]:
    """A version-2 log: every record type, an actor spawned and one gone,
    markers set and cleared, a condition that flips, a verdict."""
    records: list[dict[str, Any]] = [
        {"type": "world", "schema_version": 2, "map": "Carla/Maps/TestTown", "episode": BIG_EPISODE},
        {
            "type": "map",
            "name": "TestTown",
            "source": "directory:tests/data/maps",
            "lanelet2": {"path": str(map_path), "sha256": "0" * 64, "bytes": map_path.stat().st_size},
            "opendrive": None,
            "projector_info": {"projector_type": "Local"},
            "projector_type": None,
            "mgrs_offset": [10.0, 20.0],
            "z_offset": 1.0,
        },
        {
            "type": "traffic_light",
            "id": 9,
            "opendrive_id": "42",
            "location": [30.0, 5.0, 0.0],
            "boxes": [[30.0, 5.0, 5.0, 0.2, 0.2, 0.6, 90.0], [30.0, 5.5, 3.0, 0.2, 0.5, 1.0, 90.0]],
            "stops": [[25.0, 3.0, 0.0, 0.0]],
        },
        {"type": "warmup", "ticks": 40, "start_frame": 60, "end_frame": 100},
    ]
    for i in range(ticks):
        frame, t = 100 + i, round(0.05 * i, 3)
        if i == 0:
            records.append({"type": "actor", "id": 1, "type_id": "vehicle.lincoln.mkz", "role": "ego",
                            "extent": [2.4, 1.0, 0.8], "center": [0.1, 0.0, 0.7]})
            records.append({"type": "actor", "id": 2, "type_id": "vehicle.tesla.model3", "role": "",
                            "extent": [2.3, 1.0, 0.7], "center": [0.0, 0.0, 0.7]})
        if i == 10:
            records.append({"type": "actor", "id": 3, "type_id": "walker.pedestrian.0001", "role": "pedestrian",
                            "extent": [0.3, 0.3, 0.9], "center": [0.0, 0.0, 0.9]})
        if i == 20:
            records.append({"type": "actor_gone", "id": 2, "frame": frame, "t": t})
        actors = [[1, 1.0 * i, 2.0, 0.1, 0.0, 0.5, 10.0 + i, 20.0, -1.5 + 0.1 * i, 0.0]]
        if i < 20:
            actors.append([2, 50.0 - i, 6.0, 0.1, 0.0, 0.0, 180.0, -20.0, 0.0, 0.0])
        if i >= 10:
            actors.append([3, 20.0, 10.0 - 0.1 * i, 0.0, 0.0, 0.0, -90.0, 0.0, -2.0, 0.0])
        tick: dict[str, Any] = {
            "type": "tick", "frame": frame, "t": t, "actors": actors,
            "control": {"1": [round(0.01 * i, 3), round(-0.02 * i, 3), 0.0 if i < 30 else 0.5, 1]},
        }
        if i == 0:
            tick["lights"] = {"9": "Red"}
        if i == 25:
            tick["lights"] = {"9": "Green"}
        records.append(tick)
        if i == 5:
            records.append({"type": "markers", "frame": frame, "t": t, "actor": 1, "polylines": [
                {"points": [[0.0, 2.0, 0.2], [10.0, 2.0, 0.2], [20.0, 3.0, 0.2]], "color": [0, 255, 0],
                 "thickness": 0.2, "point_size": 0.0},
                {"points": [[0.0, 0.0, 0.2], [5.0, 0.0, 0.2]], "color": [255, 0, 255], "thickness": 0.1,
                 "point_size": 0.3},
            ]})
        if i == 15:
            records.append({"type": "markers", "frame": frame, "t": t, "actor": 1, "polylines": []})
        if i == 0:
            records.append({"type": "condition", "frame": frame, "t": t, "role": "pass", "index": 0,
                            "label": "reached_goal", "satisfied": False})
            records.append({"type": "condition", "frame": frame, "t": t, "role": "fail", "index": 0,
                            "label": "collision", "satisfied": False})
        if i == 30:
            records.append({"type": "condition", "frame": frame, "t": t, "role": "pass", "index": 0,
                            "label": "reached_goal", "satisfied": True})
    records.append({
        "type": "result", "passed": True, "message": "reached the goal", "elapsed_seconds": round(0.05 * (ticks - 1), 3),
        "conditions": [
            {"label": "pass[0](reached_goal)", "role": "pass", "condition_type": "ReachPositionCondition",
             "satisfied": True, "message": "reached"},
            {"label": "fail[0](collision)", "role": "fail", "condition_type": "CollisionCondition",
             "satisfied": False, "message": ""},
        ],
    })
    return records


def write_log(path: Path, records: list[dict[str, Any]], extra_lines: list[str] = ()) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as out:
        for record in records:
            out.write(json.dumps(record, separators=(",", ":")) + "\n")
        for line in extra_lines:
            out.write(line + "\n")
    return path


@pytest.fixture
def v2_log(tmp_path: Path) -> Path:
    return write_log(tmp_path / "CutIn-3.trajectory.jsonl", v2_records())


@pytest.fixture
def v1_log(tmp_path: Path) -> Path:
    shutil.copy(V1_LOG, tmp_path / V1_LOG.name)
    shutil.copy(V1_RESULT, tmp_path / V1_RESULT.name)
    return tmp_path / V1_LOG.name


@pytest.fixture(scope="session")
def read_rrd() -> Iterator[Callable[..., Any]]:
    """``read_rrd(path, contents, index="frame")``: the recording as a pandas
    frame, through Rerun's dataframe API (a local server and its catalog)."""
    import rerun as rr

    def read(path: Path, contents: list[str] | str = "/**", index: str | None = "frame") -> Any:
        with rr.server.Server(host="127.0.0.1", datasets={"d": [str(path)]}) as server:
            dataset = server.client().get_dataset("d")
            view = dataset.filter_contents(contents if isinstance(contents, list) else [contents])
            return view.reader(index=index).to_pandas()

    yield read


@pytest.fixture(scope="session")
def rrd_columns() -> Callable[[Path], dict[str, Any]]:
    """``rrd_columns(path)``: every column of the recording's schema, by name."""
    import rerun as rr

    def columns(path: Path) -> dict[str, Any]:
        with rr.server.Server(host="127.0.0.1", datasets={"d": [str(path)]}) as server:
            schema = server.client().get_dataset("d").schema()
            return {str(c.name): c for c in schema}

    return columns
