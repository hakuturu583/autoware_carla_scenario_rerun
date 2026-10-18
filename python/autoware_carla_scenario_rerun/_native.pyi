"""Type stubs of the Rust extension module."""

from os import PathLike
from typing import Literal, TypedDict

_Path = str | PathLike[str]
_Parser = Literal["serde_json", "simd_json"]

__version__: str
#: The Rerun SDK version recordings are written with; open them with a viewer
#: of the same minor version.
RERUN_VERSION: str
#: The trajectory log format version this build reads (older ones too).
SCHEMA_VERSION: int
#: The record types of that version; any other is skipped (or refused with
#: ``strict=True``).
RECORD_TYPES: list[str]
#: The ``logging`` logger warnings go to.
LOGGER_NAME: str

class Stats(TypedDict):
    ok: Literal[True]
    error: None
    input: str
    output: str
    #: The file's format version (1 when its ``world`` record states none).
    schema_version: int
    bytes: int
    lines: int
    ticks: int
    #: Distinct vehicles and walkers.
    actors: int
    #: Actor rows over all ticks.
    actor_rows: int
    traffic_lights: int
    markers: int
    conditions: int
    has_result: bool
    #: The Lanelet2 map drawn, if any.
    map: str | None
    #: Whether the map came from this process's cache (read by an earlier
    #: conversion).
    map_cached: bool
    #: Skipped record types: lines per type.
    unknown_types: dict[str, int]
    #: The first line of each skipped type.
    unknown_first_lines: dict[str, int]
    warnings: list[str]
    rrd_bytes: int
    parse_seconds: float
    map_seconds: float
    write_seconds: float
    seconds: float

class Failure(TypedDict):
    ok: Literal[False]
    error: str
    #: ``ConversionError``, ``MalformedLineError`` or ``UnknownRecordTypeError``.
    error_type: str
    input: str

class ConversionError(RuntimeError):
    """A trajectory log could not be converted."""

    path: str

class MalformedLineError(ConversionError):
    """A line is not a record of the schema."""

    #: 1-based.
    line: int

class UnknownRecordTypeError(ConversionError):
    """``strict=True`` and the log has a record type the schema does not define."""

    line: int
    record_type: str

def convert(
    jsonl: _Path,
    out_rrd: _Path | None = None,
    *,
    result_json: _Path | None = None,
    map_dir: _Path | None = None,
    map_path: _Path | None = None,
    no_map: bool = False,
    strict: bool = False,
    parser: _Parser = "serde_json",
) -> Stats:
    """Convert one ``*.trajectory.jsonl`` into a Rerun recording.

    Args:
        jsonl: The trajectory log.
        out_rrd: The recording to write; ``X.rrd`` beside ``X.trajectory.jsonl``
            when not given.
        result_json: The run's ``_result.json``, for a log without a
            ``result`` record (format version 1); ``X_result.json`` beside the
            log is used when not given.
        map_dir: Where to look the Lanelet2 map up by the CARLA map's name
            (``<map_dir>/<name>/lanelet2_map.osm``, or one level deeper), when
            the log's ``map`` record names no file on this machine.
        map_path: The Lanelet2 map to draw instead of the log's.
        no_map: Draw no map.
        strict: Raise :class:`UnknownRecordTypeError` on a record type the
            schema does not define, instead of skipping it with a warning.
        parser: The JSON parser.

    Raises:
        MalformedLineError: A line is not a record of the schema.
        UnknownRecordTypeError: ``strict`` and an unknown record type.
        ConversionError: The log, the result or the map could not be read, or
            the recording could not be written.
    """

def convert_dir(
    output_dir: _Path,
    *,
    jobs: int | None = None,
    out_dir: _Path | None = None,
    map_dir: _Path | None = None,
    no_map: bool = False,
    strict: bool = False,
    parser: _Parser = "serde_json",
) -> list[Stats | Failure]:
    """Convert every ``*.trajectory.jsonl`` under ``output_dir`` (recursively).

    Each log is converted as :func:`convert` would with its defaults (its
    ``_result.json`` found beside it), ``jobs`` at a time (all cores when
    ``None``); a Lanelet2 map is read once for all of them. A log that fails
    does not stop the others: its entry has ``ok`` false and the error.

    Args:
        out_dir: Write the recordings here, keeping the directory layout,
            instead of beside the logs.
    """

def clear_map_cache() -> None:
    """Forget the Lanelet2 maps this process has read."""
