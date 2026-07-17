#!/usr/bin/env python3
"""Build, upload, and monitor the per-motor closed-loop setSpeed test.

    uv run inoTest/speed_test/build.py

Shares the build/upload/monitor driver in ``inoTest/build_common.py``. Pulls in only
``EventTrace`` (util domain) on top of the firmware root (``Config.h``).
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

SOURCE_DOMAINS = ("util",)


if __name__ == "__main__":
    raise SystemExit(main("speed_test", SOURCE_DOMAINS))
