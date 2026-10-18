"""Convert autoware_carla_scenario trajectory logs into Rerun recordings.

>>> from autoware_carla_scenario_rerun import convert, convert_dir
>>> stats = convert("outputs/Case-1.trajectory.jsonl", "Case-1.rrd")
>>> results = convert_dir("outputs/2026-10-18/08-48-12", jobs=8)

The work runs in Rust with the GIL released, so another thread (the next case
of a sweep) keeps running while a log converts. Warnings -- record types this
version does not know, a map it cannot find -- go to the ``logging`` logger
``autoware_carla_scenario_rerun``, into the returned stats, and into the
recording under ``diagnostics/``.
"""

from ._native import (
    LOGGER_NAME,
    RECORD_TYPES,
    RERUN_VERSION,
    SCHEMA_VERSION,
    ConversionError,
    MalformedLineError,
    UnknownRecordTypeError,
    __version__,
    clear_map_cache,
    convert,
    convert_dir,
)

__all__ = [
    "LOGGER_NAME",
    "RECORD_TYPES",
    "RERUN_VERSION",
    "SCHEMA_VERSION",
    "ConversionError",
    "MalformedLineError",
    "UnknownRecordTypeError",
    "__version__",
    "clear_map_cache",
    "convert",
    "convert_dir",
]
