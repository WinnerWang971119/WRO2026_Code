"""End-to-end CTPS serializer -> loader -> renderer seam test (T11, AC13 / NC15).

This is the only test that proves the REAL C++ ``ctps_dump`` serializer output
and the Python consumption side (``trace_loader`` + ``track_builder``) actually
agree. Every other CTPS test exercises one side in isolation: the C++ self-test
substring-asserts the serialized JSON keys, and the Python loader/renderer tests
run against hand-authored Data-Model-A fixtures. Because the loader whitelist
(T6) and the serializer (T1) were authored in parallel against the same written
contract — never against each other's code — a silent divergence between the two
would slip past both of those isolated suites. This test is the backstop: it
builds and runs ``ctps_dump --fixture``, pipes its stdout through the real
``trace_loader.load`` + ``track_builder.build``, and asserts that

* the new ``mode`` / ``heading`` / ``reverse`` fields and the ``lineFollowDist``
  method survive the round trip through the real serializer (AC13), and
* a ``wallStop`` command serializes its ``cm`` so the trace replays without the
  ``float(None)`` crash the AC7 regression guards against.

Field-data note (load-bearing): the ``--fixture`` mission navigates
``goto("Start")`` / ``goto("final")``, which are nodes of the C++
``routes_fixture`` — NOT of the live :mod:`tools.path_planner.field_data`, which
carries only ``Start``. Replaying against the live field would raise ``KeyError``
on ``goto("final")`` before the drive commands are reached. So this test builds
the trace against :func:`field_fixture.build_field_data` — the single-sourced
Python mirror of ``routes_fixture`` — whose ``NODES`` contain both ``Start`` and
``final`` (plus the full ``field_data`` surface ``track_builder.build`` needs).

Toolchain gating (see [[gotcha-ci-gated-test-broad-pytest-collection]]): this
test hard-fails under CI when the ``ctps_dump`` binary is absent (the dedicated
``ctps-e2e`` CI job MUST build it first, so a missing binary there is a real
bug, never a silent green). A LOCAL run with no binary skips with a build hint.
Because of that hard-fail-under-CI behavior the BROAD ``path-planner`` pytest job
``--ignore``s this file (the binary is not built there) and the dedicated
``ctps-e2e`` job invokes it by explicit path with ``CTPS_DUMP_EXE`` pointed at
the binary it just built.

Build it locally with::

    cmake -S WRO2026/test -B build/ctps
    cmake --build build/ctps --target ctps_dump
"""

from __future__ import annotations

import os
import subprocess
import sys
import warnings
from pathlib import Path
from typing import List, Optional

import pytest

# ---------------------------------------------------------------------------
# Cross-tree import shim (pinned). This test lives under
# tools/path_planner/tests/ but must import ``field_fixture`` from
# WRO2026/test/fixtures/ — the Python mirror of the C++ ``routes_fixture`` the
# ``ctps_dump --fixture`` mission navigates. Insert the repo root (so the
# ``tools.path_planner.*`` imports below resolve under any rootdir) AND the
# fixtures dir (so the sibling ``field_fixture`` module is importable), both
# BEFORE importing field_fixture — mirroring test_pathtracker_parity.py.
#
# File layout: tools/path_planner/tests/test_ctps_end_to_end.py
#   parents[0] = .../tools/path_planner/tests
#   parents[1] = .../tools/path_planner
#   parents[2] = .../tools
#   parents[3] = repo (worktree) root
# ---------------------------------------------------------------------------
_REPO_ROOT: Path = Path(__file__).resolve().parents[3]
_FIXTURES_DIR: Path = _REPO_ROOT / "WRO2026" / "test" / "fixtures"
for _p in (str(_REPO_ROOT), str(_FIXTURES_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from field_fixture import build_field_data  # noqa: E402  (cross-tree fixture module)
except ImportError as exc:  # pragma: no cover - defensive import guard
    raise ImportError(
        "ctps end-to-end test could not import 'field_fixture' from "
        f"{_FIXTURES_DIR}. It is the Python mirror of the C++ routes_fixture the "
        "ctps_dump --fixture mission navigates; without it the fixture gotos "
        f"(Start/final) cannot be resolved for the replay. Original error: {exc}"
    ) from exc

from tools.path_planner.ctps import track_builder, trace_loader  # noqa: E402


# ---------------------------------------------------------------------------
# Dump-binary discovery (mirrors ctps/__main__.py::_find_dump_exe + the parity
# test's PARITY_DUMP_EXE override).
# ---------------------------------------------------------------------------


def _dump_exe_name() -> str:
    """Return the platform-specific ``ctps_dump`` executable file name.

    :return: ``"ctps_dump.exe"`` on Windows, ``"ctps_dump"`` elsewhere.
    """
    return "ctps_dump.exe" if sys.platform.startswith("win") else "ctps_dump"


def _find_dump_exe() -> Optional[Path]:
    """Locate the built ``ctps_dump`` binary, generator-agnostic.

    An explicit ``CTPS_DUMP_EXE`` env var wins (the dedicated CI job points it at
    the binary it just built). Otherwise the ``build/ctps`` tree is searched:
    single-config generators (Ninja, Unix Makefiles) drop the binary at the build
    root, while multi-config generators (Visual Studio) nest it under ``Debug/``
    or ``Release/``. The newest existing match is returned so a stale binary from
    a previous generator never shadows a fresh build.

    :return: the most-recently-modified matching executable, or ``None`` when no
        build output exists yet.
    """
    override = os.environ.get("CTPS_DUMP_EXE")
    if override:
        candidate = Path(override)
        return candidate if candidate.is_file() else None

    name = _dump_exe_name()
    build_dir = _REPO_ROOT / "build" / "ctps"
    candidates: List[Path] = [
        build_dir / name,
        build_dir / "Debug" / name,
        build_dir / "Release" / name,
    ]
    existing = [c for c in candidates if c.is_file()]
    if not existing:
        return None
    return max(existing, key=lambda p: p.stat().st_mtime)


# ---------------------------------------------------------------------------
# Module-scoped fixtures: run the dump once, load once, mirror the fixture field.
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def dump_stdout() -> str:
    """Build/locate + run ``ctps_dump --fixture`` once; return its JSON stdout.

    Locating the binary is the only place that skips: a LOCAL run with no binary
    skips with a build hint, but in CI (``CI`` env var set) the dedicated
    ``ctps-e2e`` job must have built it first, so absence is a hard failure — the
    anti-vacuous guard that stops this CI-gated test from silently passing
    without the resource it exists to exercise.

    :return: the captured stdout — a Data-Model-A command-stream JSON document.
    """
    exe = _find_dump_exe()
    if exe is None:
        message = (
            "ctps_dump binary not found. Build it with:\n"
            "  cmake -S WRO2026/test -B build/ctps\n"
            "  cmake --build build/ctps --target ctps_dump\n"
            "(or point CTPS_DUMP_EXE at an existing build)."
        )
        if os.environ.get("CI"):
            # The dedicated ctps-e2e CI job builds it first — absence is a bug.
            pytest.fail(message)
        pytest.skip(message)

    completed = subprocess.run(
        [str(exe), "--fixture"],
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or "(no stderr)"
        pytest.fail(
            f"ctps_dump --fixture exited with code {completed.returncode}:\n{stderr}"
        )
    return completed.stdout


@pytest.fixture(scope="module")
def trace(dump_stdout: str) -> trace_loader.Trace:
    """Parse the real serializer stdout through ``trace_loader.load`` (step 3).

    A parse failure here means the real serializer emitted JSON the loader
    rejects — a genuine seam break — so it surfaces as a test error rather than
    being swallowed.
    """
    return trace_loader.load(dump_stdout)


@pytest.fixture(scope="module")
def fixture_field_data():
    """The Python mirror of the C++ ``routes_fixture`` (has ``Start`` + ``final``)."""
    return build_field_data()


# ---------------------------------------------------------------------------
# AC13 — the new drive fields survive the REAL serializer -> loader round trip.
# ---------------------------------------------------------------------------


def test_new_drive_fields_survive_serializer(trace: trace_loader.Trace) -> None:
    """The mode / heading / reverse / lineFollowDist payload survives the seam.

    Asserts on the PRESENCE of each category rather than exact values, so the
    test is robust to how the T4 self-test segment authors the moves — only that
    the real serializer emitted, and the loader parsed, at least one of each.
    """
    drive_cmds = [c for c in trace.commands if c.op == "drive"]
    assert drive_cmds, (
        "the ctps_dump --fixture stream carried no drive commands — the seam "
        "test would be vacuous (did the fixture mission drain?)."
    )

    gyro = [c for c in drive_cmds if c.mode == "gyro" and c.heading is not None]
    assert gyro, (
        "no GYRO drive command with an explicit heading survived the real "
        "serializer -> trace_loader round trip — mode/heading were dropped "
        "somewhere in the recorder -> JSON -> loader seam (AC2/AC13)."
    )

    line_follow = [c for c in drive_cmds if c.method == "lineFollowDist"]
    assert line_follow, (
        "no lineFollowDist drive command survived — the method string was "
        "dropped by the serializer or is missing from the loader whitelist "
        "(AC5/AC13)."
    )

    reversed_cmds = [c for c in drive_cmds if c.reverse is True]
    assert reversed_cmds, (
        "no drive command with reverse=true survived the real serializer -> "
        "loader round trip (AC6/AC13)."
    )


def test_wallstop_serializes_cm(trace: trace_loader.Trace) -> None:
    """Every ``wallStop`` command carries its ``cm`` (AC7 regression, via AC13).

    A ``wallStop`` that serialized without ``cm`` would reach ``float(None)`` and
    crash ``track_builder`` on replay. This asserts the real serializer emits
    ``cm`` for every ``wallStop`` it produces (a universal over whatever the
    fixture happens to author).
    """
    wall_stops = [
        c for c in trace.commands if c.op == "drive" and c.method == "wallStop"
    ]
    for cmd in wall_stops:
        assert cmd.cm is not None, (
            f"wallStop at seq={cmd.seq} serialized without cm — the AC7 "
            "regression (float(None) crash on replay) would reappear."
        )


def test_trace_replays_without_crashing(
    trace: trace_loader.Trace, fixture_field_data
) -> None:
    """The real dump replays through ``track_builder.build`` without crashing.

    This is the core AC13 assertion and the end-to-end wallStop-``float(None)``
    regression guard: feeding real serializer output through the renderer must
    NOT raise, and must yield a non-empty track. Building against
    :func:`field_fixture.build_field_data` (not the live field) is required — a
    ``KeyError`` here would mean the wrong field_data, whose ``NODES`` lack the
    fixture mission's ``final`` node.

    The benign ``build_graph`` orphan-node ``UserWarning`` (the fixture field
    carries intentionally-disconnected nodes) is suppressed so it does not
    clutter the run; any real exception from ``build`` still propagates and fails
    the test.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        track = track_builder.build(trace, fixture_field_data)

    assert track.keyframes, "the replayed track has no keyframes."
    assert track.events, "the replayed track has no events."
