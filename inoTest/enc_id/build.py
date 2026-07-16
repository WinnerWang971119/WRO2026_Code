#!/usr/bin/env python3
"""Build, upload, and monitor the hand-spin encoder identify test.

    uv run inoTest/enc_id/build.py

The build/upload/monitor driver lives in ``inoTest/build_common.py``. This sketch only pulls in
``EventTrace`` (util domain) on top of the firmware root (``Config.h``), so it needs just that
one ``src`` domain.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

SOURCE_DOMAINS = ("util",)


if __name__ == "__main__":
    raise SystemExit(main("enc_id", SOURCE_DOMAINS))
