"""Compare a converted recording with ACS's in-process recorder's.

    python tools/compare_inloop.py CONVERTED.rrd INLOOP.rrd

The in-process recorder (ACS's ``rerun_recorder.py``) logs on a ``tick``
timeline; the converter logs on ``frame``. Both log ``sim_time``, so rows are
matched on it. Prints, per shared entity component, how many rows each side
has and the largest difference between matched values.
"""

from __future__ import annotations

import sys

import numpy as np
import rerun as rr


def table(path: str, contents: list[str]):
    with rr.server.Server(host="127.0.0.1", datasets={"d": [path]}) as server:
        dataset = server.client().get_dataset("d")
        return dataset.filter_contents(contents).reader(index="sim_time").to_pandas()


def flat(cell) -> np.ndarray | None:
    if cell is None:
        return None
    try:
        return np.asarray([np.asarray(v, dtype=float) for v in cell], dtype=float)
    except (TypeError, ValueError):
        return None


def strips(cell) -> list[np.ndarray]:
    return [np.asarray([np.asarray(p, dtype=float) for p in strip]).reshape(-1, 3) for strip in cell]


def compare_strips(a, b) -> str:
    """Strips as sets: their count, points, and the farthest any point of one
    side is from the nearest point of the other."""
    sa, sb = strips(a), strips(b)
    pa = np.concatenate(sa) if sa else np.zeros((0, 3))
    pb = np.concatenate(sb) if sb else np.zeros((0, 3))
    if not len(pa) or not len(pb):
        return f"strips {len(sa)} vs {len(sb)}"

    def farthest(x: np.ndarray, y: np.ndarray) -> float:
        worst = 0.0
        for chunk in np.array_split(x, max(1, len(x) // 2000)):
            d = np.min(np.linalg.norm(chunk[:, None, :2] - y[None, :, :2], axis=2), axis=1)
            worst = max(worst, float(d.max()))
        return worst

    return (
        f"strips {len(sa)} vs {len(sb)}, points {len(pa)} vs {len(pb)}, "
        f"farthest point from the other side {farthest(pa, pb):.4g} / {farthest(pb, pa):.4g} m"
    )


def main(converted: str, inloop: str) -> int:
    contents = ["/world/**", "/ego/**", "/map/**", "/result/**", "/conditions/**"]
    ours, theirs = table(converted, contents), table(inloop, contents)
    shared = sorted(set(ours.columns) & set(theirs.columns) - {"rerun_segment_id", "sim_time"})
    only_ours = sorted(set(ours.columns) - set(theirs.columns))
    only_theirs = sorted(set(theirs.columns) - set(ours.columns))
    print(f"rows: converted {len(ours)}, in-loop {len(theirs)}")
    print("only in converted:", only_ours)
    print("only in in-loop:", only_theirs)
    # The log rounds t to the millisecond; the in-process recorder logs the
    # clock unrounded.
    import pandas as pd

    merged = pd.merge_asof(
        ours.sort_values("sim_time"),
        theirs.sort_values("sim_time"),
        on="sim_time",
        suffixes=("_c", "_i"),
        tolerance=pd.Timedelta(milliseconds=1),
        direction="nearest",
    )
    print(f"rows matched on sim_time: {len(merged)}")
    worst = 0.0
    for col in shared:
        c, i = merged[f"{col}_c"], merged[f"{col}_i"]
        if col.endswith(":strips"):
            last = [(a, b) for a, b in zip(c, i) if a is not None and b is not None]
            if last:
                print(f"{col:<55} {compare_strips(*last[-1])} (last row)")
            continue
        both = [(a, b) for a, b in zip(c, i) if a is not None and b is not None]
        diffs, shape_mismatch = [], 0
        for a, b in both:
            fa, fb = flat(a), flat(b)
            if fa is None or fb is None:
                continue
            if fa.shape != fb.shape:
                shape_mismatch += 1
                continue
            diffs.append(float(np.max(np.abs(fa - fb))) if fa.size else 0.0)
        d = max(diffs) if diffs else float("nan")
        if diffs and "map" not in col:
            worst = max(worst, d)
        print(f"{col:<55} rows {sum(x is not None for x in ours[col]):>5} vs {sum(x is not None for x in theirs[col]):>5}"
              f"  compared {len(diffs):>5}  shape mismatches {shape_mismatch:>4}  max |diff| {d:.4g}")
    print(f"largest difference outside the map: {worst:.4g}")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:3]))
