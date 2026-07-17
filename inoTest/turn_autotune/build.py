#!/usr/bin/env python3
"""Build, upload, and monitor the on-robot Twiddle PD-gain auto-tuner.

    uv run inoTest/turn_autotune/build.py

The build/upload/monitor driver lives in ``inoTest/build_common.py`` and is shared with the
other diagnostic sketches; this wrapper only names the sketch folder and the source domains
``cooperativeTick()`` ticks (the same six ``inoTest/turn_probe`` and ``inoTest/turn_tune`` use,
since this sketch's ``cooperativeTick()`` ticks the identical module set).
"""

from __future__ import annotations

import sys
from pathlib import Path

# The wrappers sit one level below `build_common.py`, and `uv run` puts THIS file's directory on
# sys.path — not its parent. Add the parent so the shared driver imports regardless of the
# current working directory.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

# cooperativeTick() ticks every module in WRO2026.ino::cooperativeTick(), so it needs all six
# domains — identical to the other inoTest sketches' SOURCE_DOMAINS.
SOURCE_DOMAINS = ("drive", "perception", "nav", "mission", "manip", "util")


if __name__ == "__main__":
    raise SystemExit(main("turn_autotune", SOURCE_DOMAINS))
