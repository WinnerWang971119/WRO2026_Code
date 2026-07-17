#!/usr/bin/env python3
"""Build, upload, and monitor the lifted wheel-match test.

    uv run inoTest/wheel_match/build.py

The build/upload/monitor driver lives in ``inoTest/build_common.py`` and is shared with the
other diagnostic sketches; this wrapper only names the sketch folder and the source domains.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

SOURCE_DOMAINS = ("drive", "perception", "nav", "mission", "manip", "util")


if __name__ == "__main__":
    raise SystemExit(main("wheel_match", SOURCE_DOMAINS))
