#!/usr/bin/env python3
"""Build, upload, and monitor the encoder ground-truth diagnostic.

    uv run inoTest/enc_check/build.py

The build/upload/monitor driver lives in ``inoTest/build_common.py`` and is shared with the
other diagnostic sketches; this wrapper only names the sketch folder and the source domains
``cooperativeTick()`` ticks (the same six the other inoTest sketches use).
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

SOURCE_DOMAINS = ("drive", "perception", "nav", "mission", "manip", "util")


if __name__ == "__main__":
    raise SystemExit(main("enc_check", SOURCE_DOMAINS))
