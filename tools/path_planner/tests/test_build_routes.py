"""Tests for tools.path_planner.build_routes (CLI orchestrator).

Drives the pure-function ``_run`` helper rather than the argparse layer
because argparse is a thin shell that argparse's own tests already
cover. Each test case mirrors a row from the T8 spec in
``docs/plans/2026-05-05-phase2-pr1-path-planner-python.md``.

Covered:

* Default-paths run produces all three artefacts.
* ``no_viz=True`` produces routes.{h,cpp} but no map.png.
* Invalid field_data exits 2 and does NOT write routes.h.
* Two consecutive runs are byte-identical (AC2 idempotency).
* ``verbose=True`` writes per-step progress to stderr.
* Disconnected goal warns but still exits 0.
"""

from __future__ import annotations

import copy
import warnings
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner import build_routes
from tools.path_planner import field_data as real_field_data

# Mirrors the cloning helper from sibling test modules. Listed here
# rather than imported so the test file stays self-contained.
_FIELD_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "PROJECTIONS",
    "GOAL_APPROACH",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
)


def _clone_field_data() -> SimpleNamespace:
    """Return a deep-copied SimpleNamespace mirror of the real field_data.

    Mutating the returned namespace (or any nested container) does NOT
    affect the live ``tools.path_planner.field_data`` module.
    """
    snapshot = {
        attr: copy.deepcopy(getattr(real_field_data, attr))
        for attr in _FIELD_ATTRS
    }
    return SimpleNamespace(**snapshot)


# ---------------------------------------------------------------------------
# 1. Default-paths run produces all three artefacts.
# ---------------------------------------------------------------------------


def test_run_with_defaults_returns_zero(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Calling ``_run()`` with redirected defaults writes all three files.

    We monkeypatch the three module-level default paths so the run
    lands inside ``tmp_path`` rather than the real ``WRO2026/`` and
    ``tools/path_planner/output/`` directories. That keeps the test
    hermetic and verifies the function honours its module-level
    defaults (i.e. the lookup happens at call time, not import time).
    """
    header_target = tmp_path / "routes.h"
    source_target = tmp_path / "routes.cpp"
    map_target = tmp_path / "map.png"

    monkeypatch.setattr(
        "tools.path_planner.build_routes.DEFAULT_HEADER_PATH", header_target
    )
    monkeypatch.setattr(
        "tools.path_planner.build_routes.DEFAULT_SOURCE_PATH", source_target
    )
    monkeypatch.setattr(
        "tools.path_planner.build_routes.DEFAULT_MAP_PNG", map_target
    )

    rc = build_routes._run()

    assert rc == 0
    assert header_target.exists() and header_target.stat().st_size > 0
    assert source_target.exists() and source_target.stat().st_size > 0
    assert map_target.exists() and map_target.stat().st_size > 0


# ---------------------------------------------------------------------------
# 2. no_viz=True writes routes.{h,cpp} but skips map.png.
# ---------------------------------------------------------------------------


def test_run_with_no_viz(tmp_path: Path) -> None:
    """``no_viz=True`` skips the visualizer entirely.

    The map output path is left at its module default; the test then
    asserts ``map.png`` was not created in the project's default
    output dir during this run by checking it was never written under
    ``tmp_path``.
    """
    out_dir = tmp_path / "wro2026"
    map_target = tmp_path / "map.png"  # would be written if viz ran

    rc = build_routes._run(out_dir=out_dir, map_out=map_target, no_viz=True)

    assert rc == 0
    assert (out_dir / "routes.h").exists()
    assert (out_dir / "routes.cpp").exists()
    assert not map_target.exists(), (
        "no_viz=True should suppress visualization output entirely"
    )


# ---------------------------------------------------------------------------
# 3. Invalid field_data returns 2 and does NOT write routes.h.
# ---------------------------------------------------------------------------


def test_run_with_invalid_field_data_returns_two(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Validator failure short-circuits before any file is written.

    Builds a malformed namespace (LINE_SEGMENTS references a node that
    does not exist) and confirms exit code 2 plus the absence of any
    output file. The validator must catch this before ``emit_routes``
    runs.
    """
    bad = _clone_field_data()
    bad.LINE_SEGMENTS = [("A", "ZZZ")]  # ZZZ is not a known node

    monkeypatch.setattr(
        "tools.path_planner.build_routes.field_data", bad
    )

    out_dir = tmp_path / "wro2026"
    rc = build_routes._run(out_dir=out_dir, no_viz=True)

    assert rc == 2
    assert not (out_dir / "routes.h").exists()
    assert not (out_dir / "routes.cpp").exists()


# ---------------------------------------------------------------------------
# 4. Idempotency: two consecutive runs yield byte-identical headers.
# ---------------------------------------------------------------------------


def test_run_idempotent(tmp_path: Path) -> None:
    """AC2: two consecutive runs against unchanged input produce equal bytes.

    Captures the bytes of both files after the first run, runs again to
    the same paths, and compares. Any non-determinism (hash-randomised
    dict iteration, datetime stamps, locale-dependent float formatting)
    would surface as a byte mismatch on the second run.
    """
    out_dir = tmp_path / "out"

    rc1 = build_routes._run(out_dir=out_dir, no_viz=True)
    assert rc1 == 0
    header_bytes_first = (out_dir / "routes.h").read_bytes()
    source_bytes_first = (out_dir / "routes.cpp").read_bytes()

    rc2 = build_routes._run(out_dir=out_dir, no_viz=True)
    assert rc2 == 0
    header_bytes_second = (out_dir / "routes.h").read_bytes()
    source_bytes_second = (out_dir / "routes.cpp").read_bytes()

    assert header_bytes_first == header_bytes_second
    assert source_bytes_first == source_bytes_second


# ---------------------------------------------------------------------------
# 5. Verbose mode writes per-step progress to stderr.
# ---------------------------------------------------------------------------


def test_run_verbose_logs_to_stderr(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """``verbose=True`` writes each pipeline step to stderr.

    We assert on substrings rather than exact log-line text so the
    messages can evolve without breaking this test, but the four
    pipeline phase keywords must be present.
    """
    out_dir = tmp_path / "out"
    rc = build_routes._run(out_dir=out_dir, no_viz=True, verbose=True)
    assert rc == 0

    captured = capsys.readouterr()
    err = captured.err

    assert "validating" in err
    assert "building graph" in err
    assert "computing" in err
    assert "emitting" in err


# ---------------------------------------------------------------------------
# 6. Disconnected goal warns but still exits 0.
# ---------------------------------------------------------------------------


def test_run_disconnected_goal_warns_but_succeeds(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """TC14-style: a wall that cuts off ``G`` warns but does not fail.

    Mirrors the disconnect-goal fixture from ``test_graph.py`` /
    ``test_header_writer.py``: place a wall along the ``_proj_G -> G``
    blind linestring so build_graph emits a UserWarning, then confirm
    ``_run`` translates that into a ``WARN:`` line on stderr while
    still completing successfully (exit 0; routes.h / routes.cpp are
    still written, with ``0xFF`` cells in NEXT_HOP).
    """
    fd = _clone_field_data()
    # Box centred on the blind linestring's y=30 row, well inside the
    # robot-radius-inflated band.
    fd.OBSTACLES = [(70.0, 25.0, 80.0, 35.0, "wall")]

    monkeypatch.setattr(
        "tools.path_planner.build_routes.field_data", fd
    )

    out_dir = tmp_path / "out"

    # Suppress the test runner's own warning capture so warnings.warn()
    # inside graph_builder still routes through catch_warnings inside
    # _run (pytest's default warning filter would otherwise turn
    # UserWarning into an error or hide it before _run sees it).
    with warnings.catch_warnings():
        warnings.simplefilter("default")
        rc = build_routes._run(out_dir=out_dir, no_viz=True)

    captured = capsys.readouterr()

    assert rc == 0, f"disconnected goal should warn, not fail; stderr={captured.err!r}"
    assert (out_dir / "routes.h").exists()
    assert (out_dir / "routes.cpp").exists()
    assert "WARN:" in captured.err
