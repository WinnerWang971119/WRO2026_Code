#!/usr/bin/env python3
"""Build, upload, and monitor the HUE12 vision bring-up bench.

    uv run tools/vision_bench/build.py                 # compile, upload, open the monitor
    uv run tools/vision_bench/build.py --build-only    # compile only (no board needed)
    uv run tools/vision_bench/build.py --port /dev/cu.usbmodemXXXX   # skip auto-detect

Reuses the shared inoTest driver (``inoTest/build_common.py``) for board detection, upload,
re-enumeration, and the serial monitor. Unlike the inoTest sketches, vision_bench lives under
``tools/`` and reaches its firmware headers by path-qualified include
(``config/ConfigVision.h``, ``src/perception/VisionFrame.h``), so it uses a raw ``-I`` root
instead of ``--library`` — arduino-cli's ``--library`` include scan does not resolve
path-qualified includes (see the sketch header for the full explanation).
"""

from __future__ import annotations

import sys
from pathlib import Path

# vision_bench sits under tools/, while the shared driver lives in inoTest/. `uv run` puts THIS
# file's directory on sys.path, not the repo root, so add inoTest/ explicitly.
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "inoTest"))

from build_common import main  # noqa: E402

# The sketch folder (its basename must match the .ino per Arduino IDE) and the ONE root include
# that reaches both config/ConfigVision.h and src/perception/VisionFrame.h.
SKETCH_DIR = REPO_ROOT / "tools" / "vision_bench"
INCLUDES = (str(REPO_ROOT / "WRO2026"),)


if __name__ == "__main__":
    raise SystemExit(
        main(
            "vision_bench",
            sketch_dir=SKETCH_DIR,
            includes=INCLUDES,
            location="tools/vision_bench",
        )
    )
