#!/usr/bin/env python3
"""Build, upload, and monitor the passive junction-classifier monitor.

    uv run inoTest/junction_monitor/build.py

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

# The sketch touches WROLineArray only, and `perception` is self-contained (its headers pull
# nothing beyond Arduino / MatrixMiniR4 / Config.h). Keeping the list minimal avoids dragging
# Drive + the rest of the module graph into a sketch that must never command a motor.
SOURCE_DOMAINS = ("perception",)


if __name__ == "__main__":
    raise SystemExit(main("junction_monitor", SOURCE_DOMAINS))
