"""Allows ``python -m tools.reorg ...`` to drive the CLI directly."""

from __future__ import annotations

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
