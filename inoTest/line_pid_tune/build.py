#!/usr/bin/env python3
"""Build, upload, and monitor the LineFollower P+D auto-tuner sketch.

    uv run inoTest/line_pid_tune/build.py

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

# The sketch drives `Drive.setSpeed` and reads `WROLineArray`. `Drive` transitively pulls in
# `nav` (YawSource/Localization/routes) and `drive`'s own LineFollower TU, which in turn reach
# `mission`/`manip`/`util`. Mirror the proven `tick_bench` domain set so the whole module graph
# links cleanly.
SOURCE_DOMAINS = ("drive", "perception", "nav", "mission", "manip", "util")


if __name__ == "__main__":
    raise SystemExit(main("line_pid_tune", SOURCE_DOMAINS))
