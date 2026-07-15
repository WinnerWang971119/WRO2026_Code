#!/usr/bin/env python3
"""Compile the tick benchmark with its firmware source directories."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_FQBN = "arduino:renesas_uno:unor4wifi"
SOURCE_DOMAINS = ("drive", "perception", "nav", "mission", "manip", "util")


def parse_args() -> argparse.Namespace:
    """Parse command-line overrides for the compiler and board."""
    parser = argparse.ArgumentParser(
        description="Compile inoTest/tick_bench with the WRO2026 sources."
    )
    parser.add_argument(
        "--cli",
        default="arduino-cli",
        help="arduino-cli executable or path (default: arduino-cli)",
    )
    parser.add_argument(
        "--fqbn",
        default=DEFAULT_FQBN,
        help=f"Arduino board FQBN (default: {DEFAULT_FQBN})",
    )
    return parser.parse_args()


def main() -> int:
    """Run arduino-cli and return its exit status."""
    args = parse_args()
    cli = shutil.which(args.cli)
    if cli is None:
        print(
            f"error: Arduino CLI executable not found: {args.cli!r}\n"
            "Install arduino-cli or pass its path with --cli.",
            file=sys.stderr,
        )
        return 127

    repo_root = Path(__file__).resolve().parents[2]
    sketch = repo_root / "inoTest" / "tick_bench"
    firmware = repo_root / "WRO2026"

    command = [cli, "compile", "--fqbn", args.fqbn, str(sketch), "--library", str(firmware)]
    for domain in SOURCE_DOMAINS:
        command.extend(("--library", str(firmware / "src" / domain)))

    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
