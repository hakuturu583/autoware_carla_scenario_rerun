#!/usr/bin/env python3
"""Bumps the release version in every place that has to agree on it.

The version lives in `Cargo.toml` (`[package] version`, which maturin hands
PyPI and the module's `__version__`) and in `Cargo.lock`'s entry for the
crate; the release workflow refuses to go on unless the tag agrees with
`Cargo.toml`. The auto-release job calls this rather than editing by hand.

    tools/bump_version.py patch          # 0.1.0 -> 0.1.1
    tools/bump_version.py minor          # 0.1.0 -> 0.2.0
    tools/bump_version.py major          # 0.1.0 -> 1.0.0
    tools/bump_version.py 1.4.2          # set exactly
    tools/bump_version.py --current      # print the version, change nothing

The single line printed to stdout is the new version, which the workflow reads
back to form the tag.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CRATE = "autoware_carla_scenario_rerun"
_PACKAGE_VERSION = re.compile(r'(?ms)^(\[package\]\n(?:(?!^\[).)*?^version = )"([^"]+)"')
_LOCK_VERSION = re.compile(rf'(?m)^(name = "{CRATE}"\nversion = )"([^"]+)"')


def current_version(root: Path = ROOT) -> str:
    cargo = (root / "Cargo.toml").read_text()
    match = _PACKAGE_VERSION.search(cargo)
    if match is None:
        raise SystemExit("could not find [package] version in Cargo.toml")
    version = match.group(2)
    lock = _LOCK_VERSION.search((root / "Cargo.lock").read_text())
    if lock is None:
        raise SystemExit(f"could not find {CRATE} in Cargo.lock")
    if lock.group(2) != version:
        raise SystemExit(f"Cargo.toml says {version} and Cargo.lock {lock.group(2)}")
    return version


def next_version(current: str, spec: str) -> str:
    if spec not in {"major", "minor", "patch"}:
        if not re.fullmatch(r"\d+\.\d+\.\d+", spec):
            raise SystemExit(f"expected major|minor|patch or an X.Y.Z version, got {spec!r}")
        return spec
    if not re.fullmatch(r"\d+\.\d+\.\d+", current):
        raise SystemExit(f"cannot bump {current!r}: not an X.Y.Z version; give one explicitly")
    major, minor, patch = (int(part) for part in current.split("."))
    if spec == "major":
        return f"{major + 1}.0.0"
    if spec == "minor":
        return f"{major}.{minor + 1}.0"
    return f"{major}.{minor}.{patch + 1}"


def set_version(version: str, root: Path = ROOT) -> None:
    for name, pattern in (("Cargo.toml", _PACKAGE_VERSION), ("Cargo.lock", _LOCK_VERSION)):
        path = root / name
        text, count = pattern.subn(rf'\g<1>"{version}"', path.read_text(), count=1)
        if count != 1:
            raise SystemExit(f"failed to rewrite the version in {name}")
        path.write_text(text)


def main(argv: list[str] | None = None, root: Path = ROOT) -> None:
    argv = sys.argv[1:] if argv is None else argv
    if argv == ["--current"]:
        print(current_version(root))
        return
    if len(argv) != 1:
        raise SystemExit(__doc__)
    version = next_version(current_version(root), argv[0])
    set_version(version, root)
    print(version)


if __name__ == "__main__":
    main()
