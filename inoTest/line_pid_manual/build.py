#!/usr/bin/env python3
"""Build, upload, and monitor the live manual LineFollower P+D tuner.

    uv run inoTest/line_pid_manual/build.py

The build/upload/monitor driver lives in ``inoTest/build_common.py`` and is shared with the
other diagnostic sketches; this wrapper only names the sketch folder and the source domains
it pulls in.
"""

from __future__ import annotations

import sys
from pathlib import Path

# The wrappers sit one level below `build_common.py`, and `uv run` puts THIS file's
# directory on sys.path — not its parent. Add the parent so the shared driver imports
# regardless of the current working directory.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

# Same module graph as the auto-tuner: drives `Drive.setSpeed` and reads `WROLineArray`, and
# `Drive` transitively pulls in the rest. Mirror the proven `tick_bench` domain set.
SOURCE_DOMAINS = ("drive", "perception", "nav", "mission", "manip", "util")


if __name__ == "__main__":
    raise SystemExit(main("line_pid_manual", SOURCE_DOMAINS))
