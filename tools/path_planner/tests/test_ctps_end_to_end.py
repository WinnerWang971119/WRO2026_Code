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

Field-data note: ``ctps_dump`` links the PRODUCTION ``routes.cpp``, so the
``--fixture`` mission navigates ``goto("Start")`` / ``goto("MYU")`` — both live
:mod:`tools.path_planner.field_data` nodes. The replay therefore runs against the
live field directly (the earlier ``routes_fixture`` Python mirror is no longer
needed here). If a future field edit removes ``MYU``, both the C++ self-test's
``goto MYU`` assertion and this replay would fault — the fixture mission is
intentionally coupled to real node names.

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

import hashlib
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
# Repo-root import shim (pinned). This test lives under
# tools/path_planner/tests/ but imports ``tools.path_planner.*`` — insert the
# repo root so those resolve under any pytest rootdir (mirroring
# test_pathtracker_parity.py). ``ctps_dump`` links the PRODUCTION ``routes.cpp``,
# so both the default and ``--fixture`` missions navigate production node names;
# the replay therefore runs against the live ``field_data`` (no routes_fixture
# mirror needed).
#
# File layout: tools/path_planner/tests/test_ctps_end_to_end.py
#   parents[0] = .../tools/path_planner/tests
#   parents[1] = .../tools/path_planner
#   parents[2] = .../tools
#   parents[3] = repo (worktree) root
# ---------------------------------------------------------------------------
_REPO_ROOT: Path = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tools.path_planner import field_data  # noqa: E402  (live production field)
from tools.path_planner.ctps import player_html, track_builder, trace_loader  # noqa: E402
from tools.path_planner.tests.fixtures import toy_field_data  # noqa: E402


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
def live_field_data():
    """The live production field — ``ctps_dump`` links the production ``routes.cpp``,
    so the ``--fixture`` mission navigates production nodes (``Start`` + ``MYU``)."""
    return field_data


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
    trace: trace_loader.Trace, live_field_data
) -> None:
    """The real dump replays through ``track_builder.build`` without crashing.

    This is the core AC13 assertion and the end-to-end wallStop-``float(None)``
    regression guard: feeding real serializer output through the renderer must
    NOT raise, and must yield a non-empty track. The replay runs against the live
    production ``field_data`` — ``ctps_dump`` links the production ``routes.cpp``,
    so the ``--fixture`` mission's gotos (``Start`` + ``i5``) resolve there.

    Any benign ``build_graph`` orphan-node ``UserWarning`` is suppressed so it
    does not clutter the run; any real exception from ``build`` still propagates
    and fails the test.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        track = track_builder.build(trace, live_field_data)

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


# ---------------------------------------------------------------------------
# T14 (issue #130) — full-seam end to end: dump-JSON shape -> real loader ->
# real track_builder -> real player_html, exercising T8's new drive fields,
# a declare_position op, and a raw-drive-then-goto return maneuver together.
#
# The live ``ctps_dump --fixture`` mission (asserted above) does not yet
# author any of `power`/`preset`/`start_speed`/`end_speed`/`profiled`/`brake`,
# a `declare_position` op, or an off-node raw drive with enough offset to
# trigger a return-to-node maneuver — so a hand-authored Data-Model-A trace
# fixture (``ctps_full_seam_trace.json``, shaped exactly like the OTHER
# committed ``ctps_*.json`` fixtures this suite's sibling files already use,
# e.g. ``ctps_gyro_heading.json``) is the vehicle here, run through the SAME
# real ``trace_loader.load`` -> ``track_builder.build`` -> ``player_html.render``
# chain the dump-based tests above exercise. This needs neither the
# ``ctps_dump`` binary nor a browser/Node runtime, so it always runs (no CI
# gating) — keeping the "fixture-driven, CI-portable" property the plan asks
# for, without introducing a hard host-build dependency into a test CI runs
# without a compiler.
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def full_seam_trace() -> trace_loader.Trace:
    """Load the T14 full-seam fixture through the REAL ``trace_loader.load``."""
    return trace_loader.load(_FIXTURES_DIR / "ctps_full_seam_trace.json")


@pytest.fixture(scope="module")
def full_seam_track(full_seam_trace: trace_loader.Trace):
    """Build the T14 fixture through the REAL ``track_builder.build`` (T10/T11).

    Runs against the stable toy field (the fixture's ``S``/``B`` node names are
    toy-field nodes, mirroring every other hand-authored ``ctps_*.json`` fixture
    in this suite) rather than the live production field the dump-based tests
    above use — the two node universes are unrelated and this fixture's node
    names are not guaranteed to exist in the live field.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return track_builder.build(full_seam_trace, toy_field_data)


_FIXTURES_DIR: Path = Path(__file__).resolve().parent / "fixtures"

#: sha256 of the CURRENT (this-feature-unchanged) ``assets/player.js`` /
#: ``assets/renderer.js`` — see :func:`test_t14_player_and_renderer_js_byte_unchanged`
#: (AC10). Hashed over LF-normalized bytes so the pin is identical on Windows
#: (CRLF checkout), macOS, and Linux/CI; a future edit to either file's actual
#: content must fail this test loudly rather than silently pass.
_PLAYER_JS_SHA256: str = "0bbace38c37f5f82fe0b752de6087b02cf5aaea7ad80800f7957ca10cfbaf997"
# Re-frozen for issue #165 (axle->body render offset): _drawActors + hitTest now
# anchor the robot marker to the body pose via CTPS_OBJ.axleToBodyPose.
_RENDERER_JS_SHA256: str = "8b6f72abdbe2b510837ef796b88a2d8d3ac2e41576907a743a0c212a4157c912"


def test_t14_new_drive_fields_survive_loader(full_seam_trace: trace_loader.Trace) -> None:
    """The T8 per-command speed knobs parse into ``Command`` via the REAL loader.

    Asserts both drive commands in the fixture retain the exact fields their
    JSON carries: ``power``/``profiled``/``brake`` on the first, and
    ``preset``/``start_speed``/``end_speed``/``brake`` on the second — proving
    the additive ``drive`` fields (Contracts, T8/T9) survive
    ``trace_loader.load`` unmodified.
    """
    drive_cmds = [c for c in full_seam_trace.commands if c.op == "drive"]
    assert len(drive_cmds) == 2, (
        "expected exactly two drive commands in ctps_full_seam_trace.json"
    )
    first, second = drive_cmds

    assert first.power == 90
    assert first.profiled is True
    assert first.brake is True
    assert first.preset is None
    assert first.start_speed is None
    assert first.end_speed is None

    assert second.preset == "fast"
    assert second.start_speed == 60
    assert second.end_speed == 60
    assert second.brake is False
    assert second.power is None


def test_t14_declare_position_op_survives_loader(full_seam_trace: trace_loader.Trace) -> None:
    """The ``declare_position`` op (T8/T9) parses with its target node."""
    declares = [c for c in full_seam_trace.commands if c.op == "declare_position"]
    assert len(declares) == 1, "expected exactly one declare_position command"
    assert declares[0].node == "B"


def test_t14_track_has_densified_keyframes(full_seam_track) -> None:
    """T10 densification: far more than one keyframe per primitive boundary.

    The fixture carries 2 raw drives + 2 gotos + 1 declare_position — a flat,
    pre-#130 (one-keyframe-per-primitive-boundary) builder would emit on the
    order of a handful of keyframes. The profile-driven densification (T10)
    replaces each geometry step with many profile-timed sub-keyframes, so the
    actual count is asserted to be at least an order of magnitude larger, and
    the run has a strictly positive, monotonically non-decreasing duration.
    """
    keyframes = full_seam_track.keyframes
    assert len(keyframes) > 40, (
        f"expected densified keyframes (>40), got {len(keyframes)} — the "
        "profile-driven sub-sampling (T10) may have regressed to flat timing."
    )
    times = [kf["t"] for kf in keyframes]
    assert all(b >= a for a, b in zip(times, times[1:])), (
        "keyframe timestamps must be non-decreasing (AC10)."
    )
    assert full_seam_track.duration_s == pytest.approx(times[-1])
    assert full_seam_track.duration_s > 0.0


def test_t14_localization_lane_events_present(full_seam_track) -> None:
    """T11: the raw-drive-then-goto return and the declare both mark the lane.

    The two small raw drives at the top of the fixture push the chassis just
    past ``RETURN_TRIGGER_THRESHOLD_CM`` and well under
    ``RETURN_DISTANCE_CAP_CM`` (sim/runtime.py), so the following ``goto B``'s
    ``step_through`` yields a physical return-to-node prologue — rendered and
    marked exactly once. The later ``declare_position`` op marks the lane a
    second time, distinct from (never conflated with) the return marker. No
    residual ``drift_warn`` survives anywhere: the return re-anchors the first
    excursion, and ``declare_position`` clears the offset without moving the
    chassis (T11 Decisions).
    """
    events = full_seam_track.events
    types = [ev["type"] for ev in events]

    return_events = [ev for ev in events if ev["type"] == "return_to_node"]
    assert len(return_events) == 1, f"expected exactly one return_to_node marker, got {types}"
    assert "S" in return_events[0]["label"]

    declare_events = [ev for ev in events if ev["type"] == "declare_position"]
    assert len(declare_events) == 1, f"expected exactly one declare_position marker, got {types}"
    assert "B" in declare_events[0]["label"]

    assert "drift_warn" not in types, (
        f"no residual drift indicator should survive a return-then-declare "
        f"trace, got events: {types}"
    )

    # Ordering: both drives narrate first, then the return marker precedes its
    # own goto span, then the declare sits between the two gotos.
    goto_events = [ev for ev in events if ev["type"] == "goto"]
    assert len(goto_events) == 2
    assert return_events[0]["t"] <= goto_events[0]["t_end"]
    assert goto_events[0]["t_end"] <= declare_events[0]["t"] <= goto_events[1]["t"]


def test_t14_rendered_player_embeds_full_seam_track(tmp_path) -> None:
    """End to end: the REAL ``player_html.render`` embeds the full-seam track.

    Loads the fixture (real loader), builds it (real track_builder), renders it
    (real player_html) to a temp file, and re-parses the embedded ``const
    TRACK`` out of the emitted HTML — proving the densified keyframes and the
    localization-lane events (return-to-node + declare_position) round-trip
    byte-for-byte through the self-contained page the same way TC7's
    boundary-pose invariant is proven at the track level. Also confirms the
    lane manifest still carries the ``"drift"`` id (T12 keeps the id, relabels
    only the dock display text in ``timeline.js``) and the page stays
    self-contained (TC5).
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        trace = trace_loader.load(_FIXTURES_DIR / "ctps_full_seam_trace.json")
        track = track_builder.build(trace, toy_field_data)

    out_path = tmp_path / "full_seam.html"
    returned = player_html.render(track, out_path, filename="ctps_full_seam_trace.json")
    assert returned == out_path

    html = out_path.read_text(encoding="utf-8")

    track_match = _TC10_TRACK_RE.search(html)
    assert track_match is not None, "could not locate the embedded const TRACK assignment"
    embedded = json.loads(track_match.group("json").replace("<\\/", "</"))

    assert embedded["keyframes"] == track.keyframes
    assert embedded["events"] == track.events
    assert len(embedded["keyframes"]) > 40

    embedded_types = {ev["type"] for ev in embedded["events"]}
    assert {"return_to_node", "declare_position"} <= embedded_types

    meta_match = _TC10_META_RE.search(html)
    assert meta_match is not None, "could not locate the embedded const PLAYER_META assignment"
    meta = json.loads(meta_match.group("json").replace("<\\/", "</"))
    assert "drift" in meta["lanes"], (
        "PLAYER_META must still manifest the localization lane under its "
        "original id 'drift' (T12 relabels only the dock display text)."
    )

    # Self-contained / offline (TC5).
    assert "http://" not in html
    assert "https://" not in html
    assert "src=" not in html
    assert "href=" not in html


def test_t14_player_and_renderer_js_byte_unchanged() -> None:
    """Pin ``player.js`` / ``renderer.js`` so any edit forces a reviewed decision.

    ``player.js`` (the seek + lerp engine) has stayed byte-for-byte identical
    since the issue-#130 motion-profile feature — dense keyframes need zero
    player changes. ``renderer.js`` was frozen the same way, then issue #156
    deliberately extended it with the parity-safe lateral hug-side render and
    the pin was re-frozen against that #156 content. Both files are legitimate
    co-owners of this freeze now (#130 for player.js, #130 + #156 for
    renderer.js). This hashes the CURRENT on-disk assets and pins them so ANY
    future edit fails this test loudly, forcing an explicit, reviewed pin update.
    """
    assets_dir = player_html.PLAYER_JS_PATH.parent
    # Normalize CRLF -> LF before hashing: the assets are stored LF in git but a
    # Windows checkout materializes them as CRLF, so a raw-byte pin would only
    # match on one line-ending convention. Normalizing keeps the pin stable
    # across Windows / macOS / Linux while still catching any real content edit.
    player_js = (assets_dir / "player.js").read_bytes().replace(b"\r\n", b"\n")
    renderer_js = (assets_dir / "renderer.js").read_bytes().replace(b"\r\n", b"\n")

    player_hash = hashlib.sha256(player_js).hexdigest()
    renderer_hash = hashlib.sha256(renderer_js).hexdigest()

    # Pinned against the CURRENT (unchanged-by-this-feature) files at authoring
    # time. A future edit to either asset changes its hash and must fail this
    # test, forcing an explicit, reviewed decision to update the pin.
    assert player_hash == _PLAYER_JS_SHA256, (
        "player.js changed — the CTPS motion-profile feature (issue #130) "
        "must NOT touch the seek/lerp engine (AC10)."
    )
    assert renderer_hash == _RENDERER_JS_SHA256, (
        "renderer.js changed — this file is pinned (it carries the #130 render "
        "engine and the #156 lateral-offset render). If the edit is intentional, "
        "update _RENDERER_JS_SHA256 in a reviewed change."
    )
