"""tools/bump_version.py moves Cargo.toml and Cargo.lock together."""

from __future__ import annotations

import importlib.util
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("bump_version", ROOT / "tools" / "bump_version.py")
bump = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bump)


def test_bumps_both_files(tmp_path: Path) -> None:
    shutil.copy(ROOT / "Cargo.toml", tmp_path / "Cargo.toml")
    shutil.copy(ROOT / "Cargo.lock", tmp_path / "Cargo.lock")
    current = bump.current_version(tmp_path)
    major, minor, _ = (int(p) for p in current.split("."))
    bump.main(["minor"], root=tmp_path)
    assert bump.current_version(tmp_path) == f"{major}.{minor + 1}.0"
    # Nothing else moved: the xtask crate and dependencies keep theirs.
    before = (ROOT / "Cargo.toml").read_text().splitlines()
    after = (tmp_path / "Cargo.toml").read_text().splitlines()
    assert sum(a != b for a, b in zip(before, after)) == 1


@pytest.mark.parametrize(("current", "spec", "expected"), [
    ("0.1.0", "patch", "0.1.1"), ("0.1.9", "minor", "0.2.0"), ("1.2.3", "major", "2.0.0"), ("0.1.0", "3.4.5", "3.4.5"),
])
def test_next_version(current: str, spec: str, expected: str) -> None:
    assert bump.next_version(current, spec) == expected


def test_rejects_garbage() -> None:
    with pytest.raises(SystemExit):
        bump.next_version("0.1.0", "huge")
