#!/usr/bin/env python3
"""Build, upload, and monitor the live HUE12 hue-band tuning read-out.

    uv run tools/vision_read/build.py                 # compile, upload, open the monitor
    uv run tools/vision_read/build.py --build-only    # compile only (no board needed)

Same wiring as tools/vision_bench/build.py: reuses the shared inoTest driver and reaches the
firmware headers (config/ConfigVision.h, src/perception/VisionFrame.h) through a raw -I root
rather than --library (see the sketch header / vision_bench for why).
"""

from __future__ import annotations

import sys
from pathlib import Path

# The sketch sits under tools/, the shared driver under inoTest/. `uv run` puts THIS file's
# directory on sys.path, not the repo root, so add inoTest/ explicitly.
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "inoTest"))

from build_common import main  # noqa: E402

SKETCH_DIR = REPO_ROOT / "tools" / "vision_read"
INCLUDES = (str(REPO_ROOT / "WRO2026"),)


if __name__ == "__main__":
    raise SystemExit(
        main(
            "vision_read",
            sketch_dir=SKETCH_DIR,
            includes=INCLUDES,
            location="tools/vision_read",
        )
    )
