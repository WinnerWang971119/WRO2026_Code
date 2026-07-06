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

T7 addendum (issue #55 — CTPS pivot wheel-arc): this module also carries TC10,
a **structural** (no JS execution harness) check that the pivot-wheel-arc
overlay landed correctly on the renderer + emitter side: ``renderer.js``
registers the ``"wheel-arc"`` render mode and its ``TRACK.pivots`` ``t0``/``t1``
span guard (source-regex, both on the asset file and on the fully-assembled
emitted page, since ``renderer.js`` is inlined into every page regardless of
whether a given track carries pivots), and a track that DOES carry a
``pivots`` segment produces a self-contained page that embeds that segment
verbatim in ``TRACK`` and inlines the same wheel-arc overlay machinery, with a
``PLAYER_META.render_mode`` the emitted ``renderer.js`` registers (so boot
validation passes). The pivots-> ``"wheel-arc"`` render_mode *selection* itself
is owned by :func:`tools.path_planner.ctps.player_html.render` (task T3, per the
plan's Contracts) and is asserted by the ``test_ctps_player_html`` suite (TC8),
not re-asserted here. These tests need neither
the ``ctps_dump`` binary nor a browser/Node runtime — they exercise the
Python emitter directly, so they always run (no ``CI`` / build-binary gating).
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import warnings
from pathlib import Path
from typing import Any, Dict, List, Optional

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

from tools.path_planner.ctps import player_html, track_builder, trace_loader  # noqa: E402


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


# ---------------------------------------------------------------------------
# TC10 (issue #55, T7) — renderer.js registers "wheel-arc" + its span guard,
# and a pivot-carrying track selects that render mode end to end.
#
# These tests do NOT need the ctps_dump binary: they exercise renderer.js as
# text (no JS execution harness exists — the interpreter here is Python, not
# Node/a browser) and the real Python emitter (`player_html.render`), which is
# the actual "end" this module's other tests read the C++ "end" from. They
# always run, unlike the dump-gated fixtures above.
# ---------------------------------------------------------------------------


#: A hand-built Data Model B track dict carrying one ``pivots`` segment (the
#: T2 schema — see the plan's Data Model). Shaped like the existing
#: ``ctps_lrs_track.json`` fixture (poses + events only, no command stream) to
#: prove `player_html.render` is source-agnostic even with the new optional
#: key: a plain dict, never built via `Track`/`track_builder`, is enough to
#: drive the `render_mode` selection this test exercises. The geometry here is
#: illustrative only (T5 owns the geometry-correctness tests against the real
#: `track_builder` output) — this dict exists purely to carry a well-formed
#: ``pivots`` entry through the emitter.
_PIVOT_TRACK_DICT: Dict[str, Any] = {
    "schema_version": 1,
    "field": {
        "nodes": {"U": [0.0, 0.0], "V": [60.0, 0.0], "W": [69.0, -69.0]},
        "obstacles": [],
        "bounds": [0.0, -100.0, 100.0, 100.0],
    },
    "duration_s": 5.0,
    "keyframes": [
        {"t": 0.0, "x": 0.0, "y": 0.0, "h": 0.0},
        {"t": 2.0, "x": 60.0, "y": 0.0, "h": 0.0},
        {"t": 2.5, "x": 65.6, "y": -4.5, "h": -45.0},
        {"t": 3.0, "x": 69.0, "y": -9.0, "h": -90.0},
        {"t": 5.0, "x": 69.0, "y": -69.0, "h": -90.0},
    ],
    "events": [
        {"t": 0.0, "type": "print", "label": "pivot replay start"},
        {"t": 2.0, "type": "goto", "label": "U->V"},
        {"t": 3.0, "type": "drive", "label": "turnDeg(-90.0)"},
    ],
    "pivots": [
        {
            "t0": 2.0,
            "t1": 3.0,
            "cx": 60.0,
            "cy": -9.0,
            "theta_deg": -90.0,
            "wheel": "right",
        },
    ],
}

#: Same non-greedy-to-``</script>`` capture idiom as
#: ``test_ctps_player_html.py``'s ``_TRACK_RE`` / ``_META_RE`` (each inline
#: constant lives in its own ``<script>``, so this terminates at the intended
#: assignment alone).
_TC10_META_RE = re.compile(r"const PLAYER_META = (?P<json>.*?);\s*</script>", re.DOTALL)
_TC10_TRACK_RE = re.compile(r"const TRACK = (?P<json>.*?);\s*</script>", re.DOTALL)


def test_tc10_renderer_registers_wheel_arc_mode() -> None:
    """TC10: ``renderer.js`` registers the ``"wheel-arc"`` render mode (AC1/AC5).

    Source-regex only (structural — no JS execution harness exists): asserts
    a ``CTPS.renderModes.register({ id: "wheel-arc" ... })`` call is present,
    so boot-time validation (``boot.js``) never logs the "unregistered render
    mode" error once a page emits ``render_mode: "wheel-arc"``.
    """
    js = (player_html.PLAYER_JS_PATH.parent / "renderer.js").read_text(encoding="utf-8")
    assert re.search(r'CTPS\.renderModes\.register\(\s*\{\s*id:\s*"wheel-arc"', js), (
        'renderer.js must register a render mode via '
        'CTPS.renderModes.register({ id: "wheel-arc", ... }); none was found.'
    )


def test_tc10_renderer_pivot_span_guard_present() -> None:
    """TC10: the ``TRACK.pivots`` ``t0``/``t1`` span guard exists (AC1/AC5).

    Source-regex checks for the span-lookup shape described in Contracts
    (owner T4): a read of ``track.pivots``, a comparison bracketing the
    playhead time between a segment's ``t0`` and ``t1``, and a ``return
    null`` degrade path — so the "unknown / malformed pivots" Error Handling
    rule (never throw; skip the overlay) is asserted present, not merely
    assumed.
    """
    js = (player_html.PLAYER_JS_PATH.parent / "renderer.js").read_text(encoding="utf-8")

    assert re.search(r"\btrack\.pivots\b", js), (
        "renderer.js must read track.pivots to locate the active pivot span."
    )
    assert re.search(r"\bt0\b", js) and re.search(r"\bt1\b", js), (
        "renderer.js must reference the pivots segment's t0/t1 span bounds."
    )
    assert re.search(r"t\s*>=\s*\w+\.t0\s*&&\s*t\s*<=\s*\w+\.t1", js), (
        "renderer.js must guard the active pivot span with a "
        "t0 <= t <= t1 comparison against the playhead time."
    )
    assert re.search(r"return\s+null", js), (
        "the pivot span lookup must degrade to null (skip the overlay) "
        "rather than throw on a missing/malformed segment."
    )


def test_tc10_wheel_arc_survives_html_inlining(tmp_path) -> None:
    """TC10: the wheel-arc registration + span guard survive into the page.

    ``renderer.js`` is inlined into EVERY emitted page verbatim (own
    ``<script>``, no additional escaping beyond the JSON constants) regardless
    of whether the rendered track carries pivots — the T3 blocker rationale
    depends on exactly this: the mode must already be registered before any
    page could ever set ``render_mode: "wheel-arc"``. This renders a
    pivot-free track (the existing LRS fixture) and re-runs the same
    source-regex checks against the fully assembled HTML, proving the pattern
    is not merely true on disk but survives the emitter's asset-inlining
    pipeline unchanged.
    """
    track_dict = json.loads(
        (Path(__file__).resolve().parent / "fixtures" / "ctps_lrs_track.json").read_text(
            encoding="utf-8"
        )
    )
    out_path = tmp_path / "no_pivots.html"
    player_html.render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert re.search(r'CTPS\.renderModes\.register\(\s*\{\s*id:\s*"wheel-arc"', html), (
        "the emitted page must embed the wheel-arc render-mode registration."
    )
    assert re.search(r"\btrack\.pivots\b", html), (
        "the emitted page must embed the track.pivots span lookup."
    )
    assert re.search(r"t\s*>=\s*\w+\.t0\s*&&\s*t\s*<=\s*\w+\.t1", html), (
        "the emitted page must embed the t0 <= t <= t1 span guard."
    )
    assert re.search(r"return\s+null", html), (
        "the emitted page must embed the degrade-on-missing-segment path."
    )


def test_tc10_pivot_track_page_is_self_contained_with_overlay(tmp_path) -> None:
    """TC10: a pivot-carrying track emits a self-contained page with the overlay.

    T7's structural end (AC1/AC5), proven on a track whose dict has a non-empty
    ``pivots`` list:

    * the ``pivots`` segment rides along verbatim in the inline ``TRACK`` so the
      ``wheel-arc`` mode can locate the active span,
    * the ``wheel-arc`` registration and the ``t0``/``t1`` span guard (the same
      overlay machinery asserted against ``renderer.js`` on disk above) survive
      into THIS pivot page's fully assembled HTML,
    * ``PLAYER_META.render_mode`` is a mode the emitted ``renderer.js`` actually
      registers, so boot validation (``boot.js``) never logs an "unregistered
      render mode" error, and
    * the page stays offline/self-contained (TC5) with the new key present.

    The pivots->``"wheel-arc"`` render_mode *selection* is owned by
    :func:`tools.path_planner.ctps.player_html.render` (task T3, per the plan's
    Contracts) and is asserted by the ``test_ctps_player_html`` suite (TC8). It
    is deliberately NOT re-asserted here, so this e2e structural test stays green
    independent of whether that emitter change has landed in the tree yet — the
    boot-validation invariant below still holds for the current ``"box"`` default
    and for ``"wheel-arc"`` once T3 lands.
    """
    out_path = tmp_path / "pivot.html"
    returned = player_html.render(_PIVOT_TRACK_DICT, out_path)
    assert returned == out_path

    html = out_path.read_text(encoding="utf-8")

    # The pivots segment rides along verbatim in the embedded TRACK.
    track_match = _TC10_TRACK_RE.search(html)
    assert track_match is not None, "could not locate the embedded const TRACK assignment"
    embedded = json.loads(track_match.group("json").replace("<\\/", "</"))
    assert embedded.get("pivots") == _PIVOT_TRACK_DICT["pivots"], (
        "the embedded TRACK must carry the pivots segment unchanged so the "
        "wheel-arc overlay can locate the active pivot span."
    )

    # The wheel-arc registration + span guard survive into THIS pivot page.
    assert re.search(r'CTPS\.renderModes\.register\(\s*\{\s*id:\s*"wheel-arc"', html), (
        "the emitted pivot page must embed the wheel-arc render-mode registration."
    )
    assert re.search(r"\btrack\.pivots\b", html), (
        "the emitted pivot page must embed the track.pivots span lookup."
    )
    assert re.search(r"t\s*>=\s*\w+\.t0\s*&&\s*t\s*<=\s*\w+\.t1", html), (
        "the emitted pivot page must embed the t0 <= t <= t1 span guard."
    )
    assert re.search(r"return\s+null", html), (
        "the emitted pivot page must embed the degrade-on-missing-segment path."
    )

    # PLAYER_META.render_mode is present and is a mode the page registers, so
    # boot validation passes (AC5's "registered so boot validation passes"). This
    # is the T3-independent invariant: true for the current "box" default and for
    # "wheel-arc" once the render_mode selection lands, so it never falsely fails.
    meta_match = _TC10_META_RE.search(html)
    assert meta_match is not None, "could not locate the embedded const PLAYER_META assignment"
    meta = json.loads(meta_match.group("json").replace("<\\/", "</"))
    render_mode = meta.get("render_mode")
    assert isinstance(render_mode, str) and render_mode, (
        f"PLAYER_META.render_mode must be a non-empty string, got {render_mode!r}."
    )
    assert re.search(
        r'CTPS\.renderModes\.register\(\s*\{\s*id:\s*"%s"' % re.escape(render_mode),
        html,
    ), (
        f"PLAYER_META.render_mode {render_mode!r} is not registered in the "
        "emitted renderer.js; boot validation would log an unregistered-mode error."
    )

    # Still fully self-contained/offline (TC5) with the pivots key present.
    assert "http://" not in html
    assert "https://" not in html
    assert "src=" not in html
    assert "href=" not in html
