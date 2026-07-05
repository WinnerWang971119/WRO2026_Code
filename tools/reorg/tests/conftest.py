"""Shared fixtures for the tools.reorg test suite.

`repo_root` / `sketch_root` / `test_root` point at the REAL repository tree
(read-only for these tests -- nothing under `tools/reorg/tests/` ever calls
`rewriter.apply_diffs` against them). Tests that need to exercise an actual
write use a `tmp_path`-backed synthetic tree instead.
"""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture(scope="session")
def repo_root() -> Path:
    # tools/reorg/tests/conftest.py -> tests -> reorg -> tools -> <repo root>
    return Path(__file__).resolve().parents[3]


@pytest.fixture(scope="session")
def sketch_root(repo_root: Path) -> Path:
    return repo_root / "WRO2026"


@pytest.fixture(scope="session")
def test_root(sketch_root: Path) -> Path:
    return sketch_root / "test"
