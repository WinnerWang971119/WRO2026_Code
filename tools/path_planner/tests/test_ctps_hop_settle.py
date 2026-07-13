"""CTPS hop-settle-wait render tests (ADR-061 plan, T7 — render half of the tether).

Asserts the render-layer settle dwell (``track_builder.py``'s
``append_dwell_keyframe(hop_settle_s)`` calls, T6) fires EXACTLY on the same
**Settle-fire matrix** the firmware host ctest (T4) asserts — see
``docs/plans/2026-07-12-hop-settle-wait.md``'s Contracts section. A "dwell
keyframe" is a same-pose keyframe (``x``/``y``/``h`` unchanged from its
predecessor) whose timestamp jumps by ``hop_settle_ms / 1000`` seconds.

Covered cases (cite the plan's Testing Strategy)
-------------------------------------------------

* **TC9** — dwell keyframes appear at every matrix seam: a braked hop
  (dwell-1), after a cold pre-edge turn (dwell-2 turn->move, INCLUDING the
  very first hop's cold turn and a pivot-arc cold turn), and at goal arrival.
  The total render duration increases by exactly ``HOP_SETTLE_MS`` per seam
  (counted against a 0-dwell render of the same route).
* **TC10** — NO dwell keyframe at a LINE/projection pass-through node, an
  ADR-059 blind-chain INTERMEDIATE hop, or anywhere inside a wall-return
  interlude (the RESET-node arrival itself, and the
  ``WALL_STOP``/``REANCHOR``/``WALL_DEPART`` sequence). The FIRST cold turn
  on the normal hop AFTER the interlude completes DOES settle (it is an
  ordinary cold pre-edge turn on a normal hop, not part of the interlude).
* **TC11** — ``HOP_SETTLE_MS == 0`` (injected DIRECTLY on a perturbed
  :class:`~tools.path_planner.ctps.profile.ProfileConfig`, never through
  ``config_parse``/``load_profile_config`` — those read the shipped source
  default 100) renders ZERO dwell keyframes: byte-identical to a preserved
  pre-feature baseline shape (same keyframe count / duration as the
  same route rendered with the dwell fully stripped).
* **PROJECTION_FOLLOW stop-at-B** (matrix row "``PROJECTION_FOLLOW``,
  stop-at-B") — a ``goto`` that BRAKES at a projection node (the projection
  node IS the goto's destination, so the line/proj-pass-through gate declines
  by construction — ``projection_node == goal_name``) DOES settle. TC10 above
  already covers the pass-THROUGH twin of this row (``ProjL``, an
  intermediate); this closes the braked-STOP half the render tether left
  unpinned.
* **Return-prologue turn/drive → no dwell, then the first post-return cold
  turn → dwell fires** (matrix rows "Return-prologue pre-edge turn" and
  "First cold turn AFTER ... ``RETURN_TO_NODE`` completes") — a raw
  off-node ``drive`` command (routed through
  ``sim.operator_move``/``operator_advance_keyframe``, issue #130 T11) grows
  an operator offset past ``RETURN_TRIGGER_THRESHOLD_CM`` BEFORE a ``goto``;
  the sim's ``step_through`` then yields the return-to-node prologue (its own
  pre-edge turn + dead-reckon drive) FIRST, immediately followed by the
  planned walk's own first-hop cold turn. Asserts the prologue's turn/drive
  span carries NO dwell (the render suppresses via ``_RETURN_DRIVE_TAGS``,
  same mechanism TC10's wall-return case exercises) and that dwell-2 DOES
  fire on the first normal hop's cold turn once the prologue completes.

All routes replay the SAME feature-complete ``field_fixture`` map the
firmware host-test tether (T4) links (``WRO2026/test/fixtures/field_fixture.py``,
single-sourced with ``routes_fixture``) — the strongest form of the "paired
eligibility tether" the plan asks for: both sides assert against the same
Settle-fire matrix over the same topology.
"""

from __future__ import annotations

import dataclasses
import sys
import warnings
from pathlib import Path
from typing import Any, Dict, List, Tuple

import pytest

from tools.path_planner.ctps import track_builder
from tools.path_planner.ctps.profile import ProfileConfig, default_config
from tools.path_planner.ctps.trace_loader import Command, Trace, load

# ---------------------------------------------------------------------------
# field_fixture (the feature-complete parity map) lives in WRO2026/test/fixtures/,
# not on the default import path — mirrors test_ctps_run_grouper.py's shim (the
# SAME map the ADR-050 parity sweep and the T4 firmware host-test tether use).
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIELD_FIXTURE_DIR = _REPO_ROOT / "WRO2026" / "test" / "fixtures"
if str(_FIELD_FIXTURE_DIR) not in sys.path:
    sys.path.insert(0, str(_FIELD_FIXTURE_DIR))

import field_fixture  # noqa: E402  (cross-tree fixture, single-sourced with routes_fixture)

#: Position / heading tolerance for byte-exact pose comparisons (cm / deg).
_POSE_TOL: float = 1e-9

#: The shipped firmware default (``HOP_SETTLE_MS_DEFAULT`` in ConfigNav.h),
#: seconds. Every ``build_field_data()`` route below settles at this value
#: (the module-level ``default_config()`` cache reads the real header).
_HOP_SETTLE_S: float = default_config().hop_settle_ms / 1000.0

#: Guard: this whole test module is vacuous if the shipped default is 0 (the
#: feature disabled at the source). Every TC9/TC10 dwell-count assertion below
#: assumes a strictly positive dwell.
assert _HOP_SETTLE_S > 0.0, (
    "Config::HOP_SETTLE_MS_DEFAULT is 0 in the shipped ConfigNav.h — the "
    "TC9/TC10 dwell assertions below would pass vacuously."
)

_FIELD_DATA = field_fixture.build_field_data()


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _goto_trace(*targets: str, field_data=_FIELD_DATA) -> Trace:
    """Build a Data-Model-A :class:`Trace` of successive ``goto`` commands.

    :param targets: node names, dispatched as ``goto(target)`` in order.
    :param field_data: the field whose ``START_NODE`` / ``START_HEADING_DEG``
        seed the trace (defaults to the feature-complete ``field_fixture``).
    :return: the loaded :class:`Trace`.
    """
    return load(
        {
            "schema_version": 1,
            "start_node": field_data.START_NODE,
            "start_heading_deg": field_data.START_HEADING_DEG,
            "build_error": False,
            "commands": [
                {"seq": i, "op": "goto", "target": target}
                for i, target in enumerate(targets)
            ],
        }
    )


def _build_quiet(trace: Trace, field_data=_FIELD_DATA):
    """Run :func:`track_builder.build`, swallowing the benign orphan-node warning.

    ``field_fixture`` carries no orphan node, but this stays defensive/consistent
    with the sibling CTPS render-test suites (``test_ctps_run_grouper.py``).
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return track_builder.build(trace, field_data)


def _find_dwells(keyframes: List[Dict[str, Any]]) -> List[Tuple[float, float, float, float]]:
    """Return every dwell span in ``keyframes`` as ``(t0, t1, x, y)``.

    A dwell span is a same-pose (x/y/h byte-unchanged) consecutive keyframe
    pair whose ``dt`` matches :data:`_HOP_SETTLE_S` (within a loose relative
    tolerance — the render clock accumulates float error over a long route,
    but a dwell span's OWN dt is always exactly ``hop_settle_s`` per
    ``append_dwell_keyframe``, so a tight absolute tolerance on dt alone is
    safe and specific: it cannot mistake a same-pose FLOOR-CRAWL sub-frame
    for a dwell, since the T-shape profile never holds a sub-frame open for
    a full 100 ms at a fixed pose outside an actual dwell).

    :param keyframes: a :class:`~tools.path_planner.ctps.track_builder.Track`'s
        ``keyframes`` list.
    :return: one ``(t0, t1, x, y)`` tuple per dwell span found, in order.
    """
    dwells: List[Tuple[float, float, float, float]] = []
    for a, b in zip(keyframes, keyframes[1:]):
        same_pose = (
            abs(b["x"] - a["x"]) <= _POSE_TOL
            and abs(b["y"] - a["y"]) <= _POSE_TOL
            and abs(b["h"] - a["h"]) <= _POSE_TOL
        )
        if not same_pose:
            continue
        dt = b["t"] - a["t"]
        if abs(dt - _HOP_SETTLE_S) <= 1e-6:
            dwells.append((a["t"], b["t"], a["x"], a["y"]))
    return dwells


def _xy_close(a: Tuple[float, float], b: Tuple[float, float], tol: float = _POSE_TOL) -> bool:
    """Return ``True`` iff ``a`` and ``b`` match within ``tol`` on both axes."""
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def _xy_in(needle: Tuple[float, float], haystack, tol: float = _POSE_TOL) -> bool:
    """Return ``True`` iff ``needle`` matches some entry of ``haystack`` within ``tol``."""
    return any(_xy_close(needle, candidate, tol) for candidate in haystack)


def _zero_dwell_config() -> ProfileConfig:
    """Return a :class:`ProfileConfig` identical to the shipped default except ``hop_settle_ms=0``.

    Built via :func:`dataclasses.replace` off the REAL parsed default (never a
    hand-built ``ProfileConfig`` with guessed field values) so every OTHER
    knob (cruise speeds, ramp constants, ...) stays exactly what the shipped
    firmware headers specify — only the settle dwell is perturbed.
    """
    return dataclasses.replace(default_config(), hop_settle_ms=0)


def _build_with_zero_dwell(trace: Trace, field_data, monkeypatch: pytest.MonkeyPatch):
    """Build ``trace`` with a DIRECTLY-injected ``hop_settle_ms=0`` config (TC11).

    Patches the ``default_config`` NAME inside ``track_builder``'s own module
    namespace (every call site in ``track_builder.py`` — including the
    ``hop_settle_s`` seed at the top of ``build()`` — resolves through that
    bare name, imported at module load time via ``from ...profile import
    default_config``), never ``tools.path_planner.ctps.profile.default_config``
    itself (that would also perturb any OTHER module still holding the real
    default). This is the plan's mandated injection seam: "NOT via
    config_parse/load_profile_config, which reads the shipped source default
    100" — ``load_profile_config()`` is never called here at all; the
    perturbed config is built directly off :func:`_zero_dwell_config`.
    """
    monkeypatch.setattr(track_builder, "default_config", _zero_dwell_config)
    return _build_quiet(trace, field_data)


# ===========================================================================
# TC9 — dwell keyframes at every matrix seam; duration grows by HOP_SETTLE_MS
# per seam
# ===========================================================================


def test_tc9_dwell_fires_at_every_matrix_seam_first_hop_through_pivot_goal():
    """TC9: one ``goto`` chain covering braked hops, cold turns, and goal arrival.

    ``Start -> Corner`` (the feature-complete ``field_fixture``) walks:

    * a COLD pre-edge turn on the very FIRST hop (``Start``'s spawn heading 0
      vs. the ``Start->Inter1`` edge heading ~23.96 deg) -> dwell-2,
    * a braked ``stop_at_line`` arrival at ``Inter1`` -> dwell-1,
    * a cold pre-edge turn AT ``Inter1`` before the pass-through run -> dwell-2,
    * a collinear LINE pass-through run ``Inter1 -> ProjL -> Inter2 -> Inter3
      -> Inter4`` (NO dwell anywhere inside — TC10 territory, asserted
      separately) ending in a braked ``line_overshoot`` at ``Inter4`` ->
      dwell-1,
    * a single-wheel PIVOT turn ``Inter4 -> Corner`` (a cold turn on a normal
      hop, same as any other) -> dwell-2,
    * the braked goal arrival at ``Corner`` -> dwell-1.

    Six matrix seams total. Asserts each dwell keyframe sits at the expected
    pose, and that the total render duration is longer than a 0-dwell render
    of the identical route by exactly ``6 * HOP_SETTLE_MS`` (ms->s).

    NOTE: the ``Inter4`` braked arrival's dwell does NOT sit at ``Inter4``'s
    own field-table coordinate — a pivot DEPARTURE that lands via a SENSOR
    primitive (here, ``line_overshoot``) dead-reckons the departing trim PAST
    its true landing along the departure-edge ray (ADR-044 D1;
    ``track_builder.py``'s ``_SENSOR_LANDING_TAGS`` correction, applied to the
    NEXT pivot's arc-start, not this arrival). So this test pins the dwell
    COUNT and the two unambiguous (non-pivot-adjacent) poses (``Inter1``,
    ``Corner``) rather than hardcoding every intermediate coordinate.
    """
    trace = _goto_trace("Corner")
    track = _build_quiet(trace)

    dwells = _find_dwells(track.keyframes)
    assert len(dwells) == 6, (
        f"expected exactly 6 settle dwells (first-hop turn, Inter1 arrival, "
        f"Inter1 turn->move, Inter4 arrival, pivot turn->move, goal arrival), "
        f"got {len(dwells)}: {dwells}"
    )

    # Every dwell span's OWN dt is exactly HOP_SETTLE_S (append_dwell_keyframe
    # never rounds/truncates).
    for t0, t1, _x, _y in dwells:
        assert t1 - t0 == pytest.approx(_HOP_SETTLE_S, abs=1e-9)

    # The dwell-1 (braked arrival) seam at Inter1, and the goal-arrival
    # dwell-1 at Corner, both sit exactly on their field-table coordinates
    # (neither is pivot-adjacent).
    inter1_xy = tuple(float(c) for c in _FIELD_DATA.NODES["Inter1"])
    corner_xy = tuple(float(c) for c in _FIELD_DATA.NODES["Corner"])
    dwell_xy = [(x, y) for _t0, _t1, x, y in dwells]
    assert _xy_in(inter1_xy, dwell_xy), f"no dwell at Inter1 {inter1_xy}: {dwell_xy}"
    assert _xy_in(corner_xy, dwell_xy), f"no dwell at goal Corner {corner_xy}: {dwell_xy}"
    # Ordering: Corner (the goal-arrival dwell) is strictly the LAST dwell.
    assert dwells[-1][2:4] == pytest.approx(corner_xy, abs=_POSE_TOL)

    # Duration delta vs a 0-dwell render of the SAME route == 6 seams worth.
    # Rebuild with a locally-scoped default_config swap (module-attribute
    # patch + restore, not `monkeypatch` — this helper has no fixture arg).
    orig = track_builder.default_config
    track_builder.default_config = _zero_dwell_config
    try:
        zero_track = _build_quiet(trace)
    finally:
        track_builder.default_config = orig

    delta = track.duration_s - zero_track.duration_s
    assert delta == pytest.approx(6 * _HOP_SETTLE_S, abs=1e-6), (
        f"duration delta {delta} != 6 seams * {_HOP_SETTLE_S}s "
        f"(with={track.duration_s}, without={zero_track.duration_s})"
    )
    # Every dwell-free keyframe/pose is otherwise IDENTICAL: stripping the
    # dwells leaves the same number of "real" (non-dwell) keyframes.
    assert len(track.keyframes) - len(dwells) == len(zero_track.keyframes)


def test_tc9_first_hop_cold_turn_settles_even_with_no_preceding_braked_arrival():
    """TC9 / AC5 / TC5: the FIRST hop's cold turn settles (dwell-2), never dwell-1.

    A fresh goal's first plan has no braked arrival to wrap (dwell-1 is
    structurally absent — there is nothing to settle FROM), but the first
    hop's own cold pre-edge turn is an ordinary turn->move seam and DOES
    settle (dwell-2) — the matrix row "First-hop turn->move dwell (dwell-2,
    after the first cold pivot)". A single ``goto("Inter1")`` isolates this:
    exactly ONE dwell (the first-hop turn) plus the goal-arrival dwell at
    Inter1 itself (a braked stop_at_line) — two, not more.
    """
    trace = _goto_trace("Inter1")
    track = _build_quiet(trace)

    dwells = _find_dwells(track.keyframes)
    assert len(dwells) == 2, f"expected 2 dwells (first-hop turn, goal arrival), got {dwells}"

    start_xy = tuple(float(c) for c in _FIELD_DATA.NODES["Start"])
    inter1_xy = tuple(float(c) for c in _FIELD_DATA.NODES["Inter1"])
    dwell_xy = [(x, y) for _t0, _t1, x, y in dwells]
    assert dwell_xy[0] == pytest.approx(start_xy, abs=_POSE_TOL), (
        "the first dwell must be the first-hop cold turn->move, at Start's pose"
    )
    assert dwell_xy[1] == pytest.approx(inter1_xy, abs=_POSE_TOL), (
        "the second dwell must be the goal-arrival dwell, at Inter1's pose"
    )
    # Ordering: the turn dwell strictly precedes the goal-arrival dwell.
    assert dwells[0][0] < dwells[1][0]


# ===========================================================================
# TC10 — no dwell at pass-throughs, blind-chain intermediates, or anywhere in
# the wall-return interlude; the hop AFTER the interlude DOES settle.
# ===========================================================================


def test_tc10_line_pass_through_run_gets_no_dwell():
    """TC10: a collinear LINE pass-through chain has NO dwell at ANY intermediate.

    ``Inter1 -> Inter2`` walks the pass-through triple ``Inter1 -> ProjL ->
    Inter2`` (ProjL is a PROJECTION pass-through node peeked by the LINE
    junction gate). Only the braked arrival at the FINAL node (Inter2, the
    goto's goal) settles; ProjL gets no dwell at all.
    """
    trace = _goto_trace("Inter1", "Inter2")
    track = _build_quiet(trace)

    dwells = _find_dwells(track.keyframes)
    dwell_xy = [(x, y) for _t0, _t1, x, y in dwells]

    proj_xy = tuple(float(c) for c in _FIELD_DATA.NODES["ProjL"])
    assert not _xy_in(proj_xy, dwell_xy), (
        f"a pass-through node (ProjL) must get NO settle dwell, found dwells at {dwell_xy}"
    )
    # Sanity: the projection node's pose does appear among the (non-dwell)
    # keyframes — the run was actually rendered through it, not skipped.
    all_xy = [(round(k["x"], 6), round(k["y"], 6)) for k in track.keyframes]
    assert any(
        abs(x - proj_xy[0]) <= 1e-6 and abs(y - proj_xy[1]) <= 1e-6 for x, y in all_xy
    ), "ProjL's pose never appears in the rendered keyframes"


def test_tc10_blind_chain_intermediates_get_no_dwell():
    """TC10 / ADR-059: a blind-chain run settles ONLY on its start turn + final hop.

    ``final -> ChainD`` walks the collinear BLIND chain ``final -> ChainA ->
    ChainB -> ChainC -> ChainD`` (all BLIND edges, ADR-059-eligible). No
    intermediate chain node (``ChainA`` / ``ChainB`` / ``ChainC``) gets a
    dwell — only the cold turn->move at the chain's start (``final``) and the
    braked FINAL hop's arrival at ``ChainD`` do.
    """
    trace = _goto_trace("final", "ChainD")
    track = _build_quiet(trace)

    # Isolate the final->ChainD goto span so an unrelated dwell from the
    # Start->final prefix route cannot be mistaken for a chain-intermediate.
    goto_events = [ev for ev in track.events if ev["type"] == "goto"]
    chain_span = next(ev for ev in goto_events if ev["label"] == "final->ChainD")
    span_dwells = [
        (x, y)
        for t0, _t1, x, y in _find_dwells(track.keyframes)
        if chain_span["t"] - 1e-9 <= t0 <= chain_span["t_end"] + 1e-9
    ]

    chain_a_xy = tuple(float(c) for c in _FIELD_DATA.NODES["ChainA"])
    chain_b_xy = tuple(float(c) for c in _FIELD_DATA.NODES["ChainB"])
    chain_c_xy = tuple(float(c) for c in _FIELD_DATA.NODES["ChainC"])
    for name, xy in (("ChainA", chain_a_xy), ("ChainB", chain_b_xy), ("ChainC", chain_c_xy)):
        assert not _xy_in(xy, span_dwells), (
            f"a blind-chain INTERMEDIATE hop ({name}) must get NO settle dwell, "
            f"found dwells at {span_dwells}"
        )

    # Exactly two dwells within the chain's own goto span: the cold turn at
    # `final` before the chain starts, and the braked final-hop arrival at
    # `ChainD`.
    assert len(span_dwells) == 2, (
        f"expected exactly 2 dwells in the final->ChainD span (start turn + "
        f"final braked hop), got {span_dwells}"
    )
    final_xy = tuple(float(c) for c in _FIELD_DATA.NODES["final"])
    chain_d_xy = tuple(float(c) for c in _FIELD_DATA.NODES["ChainD"])
    assert _xy_in(final_xy, span_dwells)
    assert _xy_in(chain_d_xy, span_dwells)


def test_tc10_wall_return_interlude_gets_no_dwell_but_the_next_cold_turn_does():
    """TC10 + matrix row "first cold turn after WALL_DEPART": the interlude is dwell-free.

    ``position1 -> Wall1 -> Inter3`` walks INTO the RESET node ``Wall1``
    (``position1``'s own cold turn settles normally — a normal-hop turn, not
    part of the interlude), then the wall-zero interlude itself
    (``WALL_STOP -> REANCHOR -> WALL_DEPART``), then the FIRST cold turn of
    the following normal hop (``Wall1 -> Inter3``).

    Per the Settle-fire matrix:

    * arrival AT the RESET node (``Wall1``) -> no dwell (the wall interlude
      owns that boundary instead);
    * nothing inside ``WALL_STOP`` / ``REANCHOR`` / ``WALL_DEPART`` dwells;
    * the FIRST cold turn AFTER ``WALL_DEPART`` completes (heading into
      ``Inter3``) DOES settle — the interlude is over, so the following hop
      is an ordinary normal hop.
    """
    trace = _goto_trace("position1", "Wall1", "Inter3")
    track = _build_quiet(trace)

    wall1_xy = tuple(float(c) for c in _FIELD_DATA.NODES["Wall1"])
    position1_xy = tuple(float(c) for c in _FIELD_DATA.NODES["position1"])

    # Nothing dwells strictly inside the position1->Wall1 goto span EXCEPT the
    # ordinary cold-turn dwell at position1 itself (an ordinary normal-hop
    # turn before the wall-bound edge — not part of the interlude, which only
    # starts once the chassis reaches Wall1). In particular, the ARRIVAL at
    # Wall1 (the RESET node) inside THIS span gets no dwell — the wall
    # interlude owns that boundary instead.
    goto_events = [ev for ev in track.events if ev["type"] == "goto"]
    wall_span = next(ev for ev in goto_events if ev["label"] == "position1->Wall1")
    wall_span_dwells = [
        (x, y)
        for t0, _t1, x, y in _find_dwells(track.keyframes)
        if wall_span["t"] - 1e-9 <= t0 <= wall_span["t_end"] + 1e-9
    ]
    assert len(wall_span_dwells) == 1 and _xy_close(wall_span_dwells[0], position1_xy), (
        f"the position1->Wall1 span must dwell ONLY at position1's own cold "
        f"turn (interlude + Wall1 arrival must stay dwell-free), got "
        f"{wall_span_dwells}"
    )
    assert not _xy_in(wall1_xy, wall_span_dwells), (
        "the RESET-node arrival (Wall1) must get NO settle dwell within its "
        "own goto span (the wall interlude owns that boundary)"
    )

    # The NEXT goto (Wall1 -> Inter3) DOES settle on its first cold turn — the
    # interlude is over, so this is an ordinary normal-hop turn->move dwell,
    # parked at the post-WALL_DEPART pose (which coincides with Wall1's node
    # coordinate — the interlude re-anchors back onto the node before
    # departing).
    next_span = next(ev for ev in goto_events if ev["label"] == "Wall1->Inter3")
    next_span_dwells = [
        (x, y)
        for t0, _t1, x, y in _find_dwells(track.keyframes)
        if next_span["t"] - 1e-9 <= t0 <= next_span["t_end"] + 1e-9
    ]
    assert _xy_in(wall1_xy, next_span_dwells), (
        f"the first cold turn AFTER WALL_DEPART completes must settle (at "
        f"Wall1's pose, the post-depart parked pose), got {next_span_dwells}"
    )


# ===========================================================================
# Settle-fire matrix row: `PROJECTION_FOLLOW`, stop-at-B -> dwell fires
# (render half of the tether gap this file closes; TC10 above already covers
# the pass-THROUGH twin of this row via `ProjL`)
# ===========================================================================


def test_projection_follow_stop_at_b_gets_a_dwell():
    """Settle-fire matrix: a BRAKED arrival AT a projection node settles.

    ``Corner -> ProjB`` walks the BLIND auto-seek hop into ``ProjB`` (a
    PROJECTION node reached by a BLIND edge, per ``field_fixture``'s
    ``Corner, ProjB, Zed`` sensor-regime pivot triple). Because ``ProjB`` is
    THIS goto's own destination, the line/projection pass-through gate
    declines by construction (``_should_projection_pass_through`` returns
    ``False`` whenever ``projection_node == goal_name`` —
    ``sim/runtime.py``'s ``projection_node == goal_name`` guard, mirroring
    the firmware's non-final ``_nextHop == _destinationNode`` check): the
    arrival is a normal (non-pass) ``PROJECTION_FOLLOW`` stop, dispatched as
    ``ACT_STOP_AT_LINE`` (the BLIND->projection auto-seek tag), which sits in
    the braked :data:`~tools.path_planner.ctps.track_builder._STOP_GATED_TAGS`
    family — the exact matrix row "``PROJECTION_FOLLOW``, stop-at-B ->
    ``!_projPassThrough`` -> braked -> **yes**".

    This is the STOP half of the pair TC10's ``test_tc10_line_pass_through_run_gets_no_dwell``
    already pins the PASS-THROUGH half of (``ProjL``, an interior node on a
    ``goto`` that continues past it) — together the two tests pin both rows
    of the projection-node matrix pair.
    """
    trace = _goto_trace("Corner", "ProjB")
    track = _build_quiet(trace)

    goto_events = [ev for ev in track.events if ev["type"] == "goto"]
    proj_span = next(ev for ev in goto_events if ev["label"] == "Corner->ProjB")
    span_dwells = [
        (x, y)
        for t0, _t1, x, y in _find_dwells(track.keyframes)
        if proj_span["t"] - 1e-9 <= t0 <= proj_span["t_end"] + 1e-9
    ]

    projb_xy = tuple(float(c) for c in _FIELD_DATA.NODES["ProjB"])
    assert _xy_in(projb_xy, span_dwells), (
        f"a braked PROJECTION_FOLLOW stop-at-B arrival (ProjB, the goto's own "
        f"destination) must get a settle dwell, found dwells at {span_dwells}"
    )


# ===========================================================================
# Settle-fire matrix rows: return-prologue turn/drive -> NO dwell; the first
# cold turn on the following NORMAL hop -> dwell fires
# ===========================================================================


def test_return_prologue_gets_no_dwell_but_the_first_post_return_cold_turn_does():
    """Settle-fire matrix: the return-to-node prologue settles neither its own
    turn nor its own drive; the first cold turn of the NEXT normal hop does.

    Constructs a RAW off-node ``drive`` (a ``driveDist`` command, routed
    through ``PathTrackerSim.operator_move`` via ``track_builder``'s
    ``operator_advance_keyframe`` — issue #130 T11) BEFORE a ``goto``, so the
    sim accumulates an operator offset the goto's ``step_through`` must
    correct via the return-to-node prologue before it can plan the walk.

    ``Start``'s spawn heading is 0 deg (``field_fixture.START_HEADING_DEG``).
    A 15 cm forward ``driveDist`` (no ``goto`` yet) pushes the chassis to
    offset ``(15, 0)`` off ``Start`` — comfortably between
    ``RETURN_TRIGGER_THRESHOLD_CM`` (3 cm) and ``RETURN_DISTANCE_CAP_CM``
    (50 cm), so the return gate resolves to a genuine (in-trust) RETURN, not
    NONE or an OFF_MAP fault. The following ``goto("Inter1")`` then makes
    ``sim.step_through`` yield, in order: the return prologue's pre-edge turn
    (facing the inverse-offset heading, ``atan2(-0, -15) == 180 deg`` — a
    turn from 0 to 180, well past ``NAV_TURN_TOLERANCE_DEG``) and its
    dead-reckon drive (``Start`` is a BLIND anchor, so the return dead-reckons
    via ``ACT_RETURN_FORWARD`` rather than line-seeking), THEN the planned
    walk's own first (and only) hop ``Start -> Inter1``, whose cold pre-edge
    turn (180 deg -> ~23.96 deg, the ``Start->Inter1`` edge heading) is an
    ordinary normal-hop turn, NOT part of the interlude.

    Per the Settle-fire matrix:

    * the return prologue's own pre-edge turn and its
      ``ACT_RETURN_FORWARD`` drive settle NEITHER (render-side suppression:
      ``track_builder``'s ``_RETURN_DRIVE_TAGS`` gate on both the deferred
      dwell-2 turn->move seam and the dwell-1 braked-arrival seam — the same
      mechanism ``test_tc10_wall_return_interlude_gets_no_dwell_but_the_next_cold_turn_does``
      exercises for the WALL_DEPART case);
    * the first cold turn of the following NORMAL hop (``Start -> Inter1``,
      now that the return is complete) DOES settle (dwell-2 turn->move) —
      the interlude is over, so this is an ordinary cold pre-edge turn;
    * the braked arrival at ``Inter1`` (a forced-seek ``stop_at_line`` BLIND
      edge, per the ``_EDGES`` override) ALSO settles (dwell-1, the ordinary
      goal-arrival dwell) — asserted here only as a sanity check that the
      goto completed normally past the prologue.
    """
    start_xy = tuple(float(c) for c in _FIELD_DATA.NODES["Start"])
    inter1_xy = tuple(float(c) for c in _FIELD_DATA.NODES["Inter1"])

    trace = load(
        {
            "schema_version": 1,
            "start_node": _FIELD_DATA.START_NODE,
            "start_heading_deg": _FIELD_DATA.START_HEADING_DEG,
            "build_error": False,
            "commands": [
                {"seq": 0, "op": "drive", "method": "driveDist", "cm": 15.0},
                {"seq": 1, "op": "goto", "target": "Inter1"},
            ],
        }
    )
    track = _build_quiet(trace)

    # The return prologue is rendered as `drive`/`return_to_node` narration
    # BEFORE the `goto` span in the trace, but its geometry is emitted INSIDE
    # the goto's `step_through` call — so it plays out inside the `goto`
    # event's [t, t_end] span, ahead of the planned walk's own hop(s). Confirm
    # the marker fired (the offset genuinely triggered a physical return, not
    # the sub-threshold no-op path) before isolating spans by time.
    return_events = [ev for ev in track.events if ev["type"] == "return_to_node"]
    assert len(return_events) == 1, (
        f"expected exactly one return_to_node marker (a 15 cm offset must "
        f"trigger a genuine return), got {return_events}"
    )
    return_t = return_events[0]["t"]

    goto_events = [ev for ev in track.events if ev["type"] == "goto"]
    goto_span = next(ev for ev in goto_events if ev["label"] == "Start->Inter1")

    all_dwells = _find_dwells(track.keyframes)
    # Every dwell inside this goto's span, ordered by t0.
    span_dwells = sorted(
        (t0, t1, x, y)
        for t0, t1, x, y in all_dwells
        if goto_span["t"] - 1e-9 <= t0 <= goto_span["t_end"] + 1e-9
    )

    # No dwell keyframe sits BEFORE the first normal-hop cold turn completes:
    # the return prologue's own turn + ACT_RETURN_FORWARD drive render first
    # (starting at/after `return_t`) and must be dwell-free. Concretely: the
    # ONLY dwells in this goto are the first-hop turn->move dwell (parked at
    # Start's pose, the return-corrected anchor) and the goal-arrival dwell
    # at Inter1 — never one parked mid-prologue.
    assert len(span_dwells) == 2, (
        f"expected exactly 2 dwells in the Start->Inter1 span (the "
        f"post-return first-hop cold turn + the goal arrival at Inter1), "
        f"got {span_dwells}"
    )
    dwell_xy = [(x, y) for _t0, _t1, x, y in span_dwells]
    assert dwell_xy[0] == pytest.approx(start_xy, abs=_POSE_TOL), (
        "the first dwell in the goto span must be the post-return first-hop "
        "cold turn->move, parked back at Start's (return-corrected) pose — "
        "not inside the return prologue itself"
    )
    assert dwell_xy[1] == pytest.approx(inter1_xy, abs=_POSE_TOL), (
        "the second dwell must be the ordinary goal-arrival dwell at Inter1"
    )
    # The prologue is rendered before the walk: its marker/keyframes precede
    # the first (and only) dwell this goto carries.
    assert return_t <= span_dwells[0][0], (
        f"the return_to_node marker (t={return_t}) must precede the "
        f"post-return cold-turn dwell (t={span_dwells[0][0]}) — the "
        f"prologue renders before the planned walk"
    )


# ===========================================================================
# TC11 — HOP_SETTLE_MS == 0 (directly injected) renders ZERO dwell keyframes
# ===========================================================================


def test_tc11_zero_hop_settle_ms_renders_no_dwell_keyframes(monkeypatch: pytest.MonkeyPatch):
    """TC11: a directly-injected ``hop_settle_ms=0`` config renders zero dwells.

    Re-renders the SAME rich route as the TC9 comprehensive test
    (``Start -> Corner``, six matrix seams at the shipped default) with
    ``hop_settle_ms`` forced to 0 via :func:`_build_with_zero_dwell` — the
    plan's mandated injection seam (a perturbed ``ProfileConfig``, never
    ``config_parse``/``load_profile_config``, which read the shipped source
    default 100 and would defeat the whole point of this test). Asserts:

    * the render carries NO dwell keyframes at all (the same detector TC9
      uses, :func:`_find_dwells`, finds nothing), and
    * the render is byte-identical (keyframe-for-keyframe, event-for-event)
      to an equivalent 0-dwell render built the same way — i.e. this IS the
      preserved pre-feature baseline shape, not merely "fewer keyframes".
    """
    trace = _goto_trace("Corner")

    # The shipped-default render of the SAME route (the REAL default_config(),
    # taken BEFORE the monkeypatch below applies) — used afterward to prove
    # the 0-dwell render is not accidentally identical to the enabled one.
    default_track = _build_quiet(trace)

    zero_track = _build_with_zero_dwell(trace, _FIELD_DATA, monkeypatch)
    assert _find_dwells(zero_track.keyframes) == [], (
        "HOP_SETTLE_MS=0 must render ZERO dwell keyframes"
    )

    # Byte-identical (up to the JSON-serialisable dict) to a second independent
    # 0-dwell render — the disabled path is deterministic, matching the
    # "0 fully disables, byte-identical to pre-feature" plan Decision.
    second_zero_track = _build_with_zero_dwell(trace, _FIELD_DATA, monkeypatch)
    assert zero_track.to_dict() == second_zero_track.to_dict()

    assert len(default_track.keyframes) > len(zero_track.keyframes)
    assert default_track.duration_s > zero_track.duration_s


def test_tc11_zero_hop_settle_ms_matches_pre_feature_keyframe_count():
    """TC11: the 0-dwell render's keyframe count equals the default render's, minus the dwells.

    Cross-checks :func:`_build_with_zero_dwell` (TC11's injection seam) against
    the ordinary shipped-default render by simple subtraction: stripping every
    dwell keyframe found in the default render must land EXACTLY on the
    0-dwell render's keyframe count — proving the dwell insertion is the ONLY
    difference between the two builds (never a geometry change).
    """
    trace = _goto_trace("Corner")

    default_track = _build_quiet(trace)
    dwell_count = len(_find_dwells(default_track.keyframes))
    assert dwell_count > 0, "the default-config render must carry at least one dwell"

    orig = track_builder.default_config
    track_builder.default_config = _zero_dwell_config
    try:
        zero_track = _build_quiet(trace)
    finally:
        track_builder.default_config = orig

    assert len(default_track.keyframes) - dwell_count == len(zero_track.keyframes)
    assert _find_dwells(zero_track.keyframes) == []
