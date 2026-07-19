#!/usr/bin/env python3
"""Build, upload, and monitor the belt rotate-authority probe.

    uv run inoTest/belt_rotate_authority/build.py

The build/upload/monitor driver lives in ``inoTest/build_common.py`` and is shared with
the other diagnostic sketches; this wrapper only names the sketch folder. The probe uses
the raw ``MiniR4`` motor facade plus ``Config.h`` only, so it needs no ``src/<domain>``
libraries.
"""

from __future__ import annotations

import sys
from pathlib import Path

# The wrappers sit one level below `build_common.py`, and `uv run` puts THIS file's
# directory on sys.path — not its parent. Add the parent so the shared driver imports
# regardless of the current working directory.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from build_common import main  # noqa: E402

# Facade-only probe: no WRO2026/src/<domain> modules are compiled in.
SOURCE_DOMAINS: tuple[str, ...] = ()


if __name__ == "__main__":
    raise SystemExit(main("belt_rotate_authority", SOURCE_DOMAINS))
