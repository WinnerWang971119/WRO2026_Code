#!/usr/bin/env python3
"""Build, upload, and monitor the spare-motor match tester.

    uv run inoTest/motor_match/build.py

Shares the build/upload/monitor driver in ``inoTest/build_common.py``. Pulls in no
``src/`` domain -- only the firmware root (``Config.h``) plus the bundled ``EEPROM``
library.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

SOURCE_DOMAINS: tuple[str, ...] = ()


if __name__ == "__main__":
    raise SystemExit(main("motor_match", SOURCE_DOMAINS))
