"""CTPS collinear run-grouper render tests (ADR-059 / collinear-run plan, T11).

These close the Python/CTPS coverage gaps the plan's Testing Strategy calls
TC2, TC11, and TC12 — the *render* half of the collinear-run continuous-motion
feature. Every test drives the parity-locked sim through
:func:`tools.path_planner.ctps.track_builder.build` (no C++ compiler in the
path — geometry comes entirely from the Python sim) and asserts the grouper's
whole-run trapezoid, its transparency to zero-motion records, its malformed
fallback, and the non-chain byte-identity regression.

Covered cases
-------------

* **TC2 / AC1** — a collinear LINE-junction chain (the ``collinear_4node``
  fixture, wired in via its new CTPS render factory) renders as ONE trapezoid:
  a single ramp-up, cruise held THROUGH the intermediate nodes (velocity ==
  cruise, never the floor), ZERO intra-run stop keyframes, one ramp-down/stop.
  The TC7 boundary-pose invariant is asserted too: each node boundary pose
  appears byte-exact among the keyframes, in order.
* **AC2 (render)** — a collinear PROJECTION chain (the feature-complete
  ``field_fixture`` line-proj-line ``Inter1 -> ProjL -> Inter2`` triple)
  renders with NO per-node dip across the projection node ``ProjL`` — the
  render half of the T2 firmware cruise-hold fix.
* **TC11 (interleave)** — an ``ACT_HEADING_RESET`` (zero-motion) interleaved
  mid collinear run keeps the run grouped (one trapezoid, no spurious
  ``UserWarning``).
* **TC11 (malformed fallback)** — (a) a pass tag with NO following braked stop
  and (b) a blind run MISSING its recorded per-hop speeds each REJECT the group
  and fall back to per-primitive rendering WITH a ``UserWarning``.
* **TC7 (render) / review C1** — a LINE->BLIND SPANNING collinear run (a line
  pass-through feeding straight into a recorded blind chain) is a VALID run,
  not a malformed one: the leading LINE segments legitimately carry no chain
  speeds, and the whole run renders as ONE trapezoid with cruise held across
  the line->blind handoff (no ``UserWarning``, no mid-run dip to the floor).
* **TC12 / AC12** — a non-chain single hop AND an existing (pre-feature) trace
  render byte-identical whether or not the chain/grouper logic is present: the
  grouper reduces a single-segment run to the same one-ramp/stop shape the
  pre-grouper per-primitive path produced, so a hop-count-1 route is unchanged.

Malformed / interleave cases are injected by spying ``step_through`` (the same
seam ``test_ctps_track_builder`` uses for the direction-tag forwarding tests):
the sim's real advance tags never emit an unbraked GROUPED run to a goto
boundary, nor a blind run with a dropped speed suffix, so a hand-crafted
primitive stream is the only way to exercise those defensive branches. The spy
mutates the SHARED chassis so each yielded snapshot pose advances honestly.
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
import warnings
from pathlib import Path

import pytest

from tools.path_planner.ctps.config import SCALE_CM_S_PER_PWM
from tools.path_planner.ctps.profile import default_config
from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import Command, Trace, load
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import BlindDecayParams, compute_all_pairs_next_hop
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import (
    ACT_BLIND_PASS,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_HEADING_RESET,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    ACT_LINE_REVERSE,
    ACT_TURN_REL,
    NAV_TURN_TOLERANCE_DEG,
    Direction,
    PathTrackerSim,
)
from tools.path_planner.tests.fixtures import collinear_4node, toy_field_data

# ---------------------------------------------------------------------------
# field_fixture (the feature-complete parity map) lives in WRO2026/test/fixtures/,
# not on the default import path. Insert that dir so the collinear PROJECTION
# chain (Inter1 -> ProjL -> Inter2) can be rendered through the SAME map the
# ADR-050 parity sweep drives — mirroring test_pathtracker_parity.py's shim.
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIELD_FIXTURE_DIR = _REPO_ROOT / "WRO2026" / "test" / "fixtures"
if str(_FIELD_FIXTURE_DIR) not in sys.path:
    sys.path.insert(0, str(_FIELD_FIXTURE_DIR))

import field_fixture  # noqa: E402  (cross-tree fixture, single-sourced with routes_fixture)

#: Position / heading tolerance for byte-exact pose comparisons (cm / deg).
_POSE_TOL: float = 1e-6

#: Velocity match tolerance (cm/s) for the cruise-held-through-a-node assertions.
_VEL_TOL: float = 1e-6


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


def _build_quiet(trace: Trace, field_data):
    """Run :func:`build`, treating a stray fixture ``UserWarning`` as an error.

    Unlike the toy field, neither ``collinear_4node`` nor ``field_fixture`` has an
    orphan node, so ``build_graph`` never emits the benign orphan warning. This
    test therefore keeps ``UserWarning`` an ERROR by default: a warning here is a
    genuinely malformed run (the exact failure the grouper must NOT hit on a clean
    chain), so it should fail the test loudly rather than be swallowed.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("error", UserWarning)
        return build(trace, field_data)


def _inbound_velocities(keyframes):
    """Return the inbound velocity (cm/s) for each keyframe after the first.

    ``velocity[i]`` is the straight-line distance from ``keyframes[i]`` to
    ``keyframes[i + 1]`` divided by their timestamp delta — the average speed the
    player renders across that lerp span. A zero-``dt`` (instantaneous-brake)
    span reports 0.
    """
    vels = []
    for a, b in zip(keyframes, keyframes[1:]):
        dt = b["t"] - a["t"]
        dist = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        vels.append(dist / dt if dt > 1e-12 else 0.0)
    return vels


def _last_index_at(keyframes, x_cm, y_cm):
    """Return the index of the LAST keyframe sitting byte-exact at ``(x, y)``."""
    idxs = [
        i
        for i, k in enumerate(keyframes)
        if abs(k["x"] - x_cm) <= _POSE_TOL and abs(k["y"] - y_cm) <= _POSE_TOL
    ]
    assert idxs, f"no keyframe at ({x_cm}, {y_cm})"
    return idxs[-1]


def _first_index_at(keyframes, x_cm, y_cm):
    """Return the index of the FIRST keyframe sitting byte-exact at ``(x, y)``.

    A braked arrival at a settle-eligible node (Config::HOP_SETTLE_MS > 0,
    ADR-061) is followed by a same-pose ``append_dwell_keyframe`` dwell
    keyframe, so ``_last_index_at`` on that node now returns the DWELL
    keyframe rather than the genuine braked ARRIVAL keyframe. Callers that
    need the arrival boundary itself (e.g. to slice OUT the terminal-brake
    ramp from an "interior velocities" check) must anchor on the FIRST
    keyframe at the node, which is unaffected by any trailing dwell.
    """
    idxs = [
        i
        for i, k in enumerate(keyframes)
        if abs(k["x"] - x_cm) <= _POSE_TOL and abs(k["y"] - y_cm) <= _POSE_TOL
    ]
    assert idxs, f"no keyframe at ({x_cm}, {y_cm})"
    return idxs[0]


def _velocity_into(keyframes, vels, x_cm, y_cm):
    """Return the inbound velocity of the keyframe landing byte-exact at ``(x, y)``."""
    for i, k in enumerate(keyframes[1:], start=1):
        if abs(k["x"] - x_cm) <= _POSE_TOL and abs(k["y"] - y_cm) <= _POSE_TOL:
            return vels[i - 1]
    raise AssertionError(f"no keyframe lands at ({x_cm}, {y_cm})")


def _zero_motion_dwells(keyframes):
    """Return indices ``i`` where ``keyframes[i] -> [i+1]`` is a same-pose,
    time-advancing dwell keyframe (an ``append_dwell_keyframe`` settle / manip
    hold: identical pose, clock advanced). One per genuine braked-arrival settle.
    """
    out = []
    for i, (a, b) in enumerate(zip(keyframes, keyframes[1:])):
        same_pose = (
            abs(b["x"] - a["x"]) <= _POSE_TOL
            and abs(b["y"] - a["y"]) <= _POSE_TOL
            and abs(b["h"] - a["h"]) <= _POSE_TOL
        )
        if same_pose and (b["t"] - a["t"]) > 1e-9:
            out.append(i)
    return out


def _make_step_through_spy(sequence):
    """Return a ``step_through`` replacement that replays a scripted primitive stream.

    ``sequence`` is a list of ``(primitive_tuple, advance_cm)`` pairs. For each,
    the spy advances the SHARED sim chassis forward by ``advance_cm`` (0 for a
    zero-motion record — the snapshot pose then equals the run tail) and yields
    ``(primitive_tuple, chassis)`` — exactly the ``(primitive, snapshot)`` shape
    :func:`build` consumes, with the snapshot being the live
    :class:`~tools.path_planner.sim.chassis.VirtualChassis`, so the rendered pose
    advances honestly. The localization node is never mutated, so the pivot /
    return-prologue detection stays inert (``prev_node`` remains ``None``,
    ``find_pivot_turn`` misses) — which is exactly what these grouper-only cases
    want.
    """

    def _spy(self, goal_name, *, route_direction=Direction.AUTO, arrival_direction=Direction.AUTO):
        for primitive, advance_cm in sequence:
            if advance_cm:
                self._chassis.forward(advance_cm)
            yield (primitive, self._chassis)

    return _spy


def _track_hash(track) -> str:
    """Return a stable sha256 of ``track.to_dict()`` (sorted keys).

    Matches ``test_ctps_track_builder._track_hash`` — deterministic for identical
    float values, sufficient for identical-build-vs-identical-build byte-identity
    (never cross-platform text diffing).
    """
    payload = json.dumps(track.to_dict(), sort_keys=True).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


# ===========================================================================
# TC2 / AC1 — collinear LINE-junction chain renders as ONE trapezoid
# ===========================================================================


def test_tc2_collinear_line_chain_renders_one_trapezoid():
    """TC2 / AC1: the ``collinear_4node`` LINE chain renders as ONE trapezoid.

    Wires the ``collinear_4node`` fixture (its new CTPS render factory) through
    the run-grouper: a ``goto("A")`` re-anchors onto the line at ``A``, then
    ``goto("D")`` drives the whole A->B->C->D collinear chain. The A->D run must
    render as a SINGLE trapezoid:

    * a single ramp-up at the run start (rising velocity),
    * cruise HELD through the intermediate junctions ``B`` and ``C`` — the
      inbound velocity at each equals the follow cruise, NEVER the deadband
      floor (no per-node dip),
    * NO intra-run stop keyframe (no interior keyframe at rest / zero velocity),
    * a single ramp-down/stop into the goal ``D``.

    TC7 invariant: each node boundary pose (``B`` / ``C`` / ``D``) appears
    byte-exact among the keyframes, in order — only timing/velocity between the
    pose boundaries changed, not the poses themselves.
    """
    field_data = collinear_4node.make_ctps_field_data()
    trace = load(
        {
            "schema_version": 1,
            "start_node": field_data.START_NODE,
            "start_heading_deg": 0.0,
            "build_error": False,
            "commands": [
                {"seq": 0, "op": "goto", "target": "A"},
                {"seq": 1, "op": "goto", "target": "D"},
            ],
        }
    )
    track = _build_quiet(trace, field_data)

    cfg = default_config()
    cruise_speed = SCALE_CM_S_PER_PWM * cfg.line_cruise
    floor_speed = SCALE_CM_S_PER_PWM * cfg.pwm_min
    # Sanity: the two reference speeds must be distinct, else a "held cruise"
    # assertion could pass trivially against the floor.
    assert cruise_speed > floor_speed + 1.0

    kfs = track.keyframes
    b_xy = collinear_4node.NODES["B"]
    c_xy = collinear_4node.NODES["C"]
    d_xy = collinear_4node.NODES["D"]

    # The A->D run begins at the LAST keyframe sitting on node A (the goto("A")
    # arrival) — the grouper opens the run on the first motion primitive after it.
    a_run_start = _last_index_at(kfs, *collinear_4node.NODES["A"])
    run = kfs[a_run_start:]
    assert len(run) >= 2

    # TC7: B, C, D boundary poses appear byte-exact, in order, within the run.
    b_idx = _last_index_at(run, *b_xy)
    c_idx = _last_index_at(run, *c_xy)
    d_idx = _last_index_at(run, *d_xy)
    assert 0 < b_idx < c_idx < d_idx, (
        f"boundary poses out of order: B={b_idx}, C={c_idx}, D={d_idx}"
    )
    # D is the run's terminal keyframe — the LAST keyframe at D, which (at the
    # shipped Config::HOP_SETTLE_MS > 0 default, ADR-061) is the goal-arrival
    # settle dwell keyframe appended right after the braked arrival, not the
    # arrival itself.
    assert d_idx == len(run) - 1
    # The genuine braked ARRIVAL boundary — the FIRST keyframe at D — is
    # distinct from ``d_idx`` whenever a trailing dwell keyframe was appended.
    # Used below to exclude the terminal-brake ramp (and any trailing dwell)
    # from the "no intra-run stop keyframe" interior check.
    d_arrival_idx = _first_index_at(run, *d_xy)

    vels = _inbound_velocities(run)

    # Cruise held THROUGH the intermediate junctions B and C (no dip to floor).
    assert _velocity_into(run, vels, *b_xy) == pytest.approx(cruise_speed, abs=_VEL_TOL)
    assert _velocity_into(run, vels, *c_xy) == pytest.approx(cruise_speed, abs=_VEL_TOL)

    # ONE trapezoid: a contiguous cruise plateau spanning both B and C, with no
    # intra-run dip. Every keyframe from just before B through just after C sits
    # at cruise (the plateau is unbroken across the two intermediate nodes).
    plateau = [i for i, v in enumerate(vels) if abs(v - cruise_speed) <= _VEL_TOL]
    assert plateau, "no cruise plateau found — the run never reached cruise"
    plateau_lo, plateau_hi = plateau[0], plateau[-1]
    # The plateau is contiguous (a single hold, not two separated by a dip).
    assert plateau == list(range(plateau_lo, plateau_hi + 1)), (
        "cruise plateau is broken by an intra-run dip"
    )
    # Both B and C land inside the single plateau span.
    assert plateau_lo < b_idx <= plateau_hi + 1
    assert plateau_lo < c_idx <= plateau_hi + 1

    # Single ramp-up (before the plateau) then single ramp-down (after it):
    # velocities strictly rise into the plateau and strictly fall out of it. No
    # interior segment returns to rest between the ramps.
    ramp_up = vels[:plateau_lo]
    ramp_down = vels[plateau_hi + 1 : d_idx]  # up to (not incl.) the terminal brake
    assert all(b >= a for a, b in zip(ramp_up, ramp_up[1:])), "ramp-up not monotonic"
    assert all(b <= a for a, b in zip(ramp_down, ramp_down[1:])), "ramp-down not monotonic"

    # ZERO intra-run stop keyframes: no interior keyframe (between the run start
    # and the terminal D arrival) sits at rest — a stop keyframe would be a
    # zero-length (zero-dt) span at the pre-terminal position, i.e. an interior
    # velocity of 0. Sliced off the ARRIVAL boundary (``d_arrival_idx``, not
    # ``d_idx``) so this excludes the terminal-brake ramp AND any trailing
    # settle dwell span alike — neither is an "intra-run" dip.
    interior = vels[: d_arrival_idx - 1]  # exclude the last span (the terminal brake)
    assert all(v > _VEL_TOL for v in interior), (
        "an intra-run stop keyframe (interior zero velocity) was emitted"
    )


# ===========================================================================
# Regression — a SUB-TOLERANCE alignment turn mid-run keeps the run grouped
# ===========================================================================


def test_subtolerance_alignment_turn_keeps_collinear_run_grouped():
    """A near-collinear pass-through's tiny alignment turn must NOT split the run.

    Real field nodes are never perfectly axis-aligned, so the sim emits a small
    ``turn_relative`` (a few hundredths of a degree) to align onto each next
    edge's exact heading at a pass-through — a turn it already deemed a
    pass-through (a ``line_pass``, NOT a braked stop) because the delta is below
    ``NAV_TURN_TOLERANCE_DEG``. The CTPS run-grouper must gate its run-break on
    the SAME tolerance; a hard ``1e-9`` threshold instead treated that alignment
    turn as a real turn and split the whole-run trapezoid into two ramps with a
    dip at the pass-through node (the ``C -> CG2`` two-ramp render on the real
    field). The perfectly-collinear ``collinear_4node`` fixture cannot surface
    this — the ``_skewed`` variant nudges node ``C`` off-axis so ``B`` and ``C``
    emit genuine sub-tolerance alignment turns.
    """
    field_data = collinear_4node.make_ctps_field_data_skewed()

    # Guard: the skewed fixture genuinely emits a NON-ZERO, sub-tolerance
    # ``turn_relative`` at an intermediate node — otherwise this test would pass
    # trivially (a perfectly-collinear chain has no turn to be transparent to)
    # and stop guarding the regression.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph = build_graph(field_data)
        next_hop = compute_all_pairs_next_hop(
            graph, BlindDecayParams.from_field_data(field_data)
        )
    chassis = VirtualChassis.from_field_data(field_data)
    sim = PathTrackerSim(field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    align_turns = []
    for target in ("A", "D"):
        for primitive, _snap in sim.step_through(target):
            if primitive[0] == ACT_TURN_REL:
                align_turns.append(abs(float(primitive[1])))
    assert any(0.0 < t < NAV_TURN_TOLERANCE_DEG for t in align_turns), (
        "skewed fixture emitted no sub-tolerance alignment turn — the regression "
        f"is not exercised (turns seen: {align_turns})"
    )

    trace = load(
        {
            "schema_version": 1,
            "start_node": field_data.START_NODE,
            "start_heading_deg": 0.0,
            "build_error": False,
            "commands": [
                {"seq": 0, "op": "goto", "target": "A"},
                {"seq": 1, "op": "goto", "target": "D"},
            ],
        }
    )
    track = _build_quiet(trace, field_data)

    cfg = default_config()
    cruise_speed = SCALE_CM_S_PER_PWM * cfg.line_cruise
    floor_speed = SCALE_CM_S_PER_PWM * cfg.pwm_min
    assert cruise_speed > floor_speed + 1.0

    kfs = track.keyframes
    b_xy = field_data.NODES["B"]
    c_xy = field_data.NODES["C"]
    d_xy = field_data.NODES["D"]

    a_run_start = _last_index_at(kfs, *field_data.NODES["A"])
    run = kfs[a_run_start:]
    vels = _inbound_velocities(run)

    # The anti-regression: cruise HELD through BOTH intermediate pass-throughs
    # (never dipping toward the deadband floor, which is what a run-split would
    # do — it re-seeds the second ramp from the start-speed floor).
    tol = 1e-3
    assert _velocity_into(run, vels, *b_xy) == pytest.approx(cruise_speed, abs=tol)
    assert _velocity_into(run, vels, *c_xy) == pytest.approx(cruise_speed, abs=tol)

    # ONE trapezoid: a single CONTIGUOUS cruise plateau spanning B and C, with no
    # intra-run dip between them.
    plateau = [i for i, v in enumerate(vels) if abs(v - cruise_speed) <= tol]
    assert plateau, "no cruise plateau — the run never reached cruise"
    assert plateau == list(range(plateau[0], plateau[-1] + 1)), (
        "cruise plateau broken by an intra-run dip — the alignment turn split the run"
    )
    b_idx = _last_index_at(run, *b_xy)
    c_idx = _last_index_at(run, *c_xy)
    assert plateau[0] < b_idx <= plateau[-1] + 1
    assert plateau[0] < c_idx <= plateau[-1] + 1

    # No interior stop keyframe up to the terminal brake into D. Anchored on
    # the ARRIVAL boundary (the FIRST keyframe at D), not the last — at the
    # shipped Config::HOP_SETTLE_MS > 0 default (ADR-061) the goal arrival
    # appends a trailing same-pose settle dwell keyframe, so "the last
    # keyframe at D" no longer identifies the arrival. The terminal-brake
    # ramp legitimately dips to/below the floor right before that arrival —
    # this must stay excluded, exactly as before the dwell existed.
    d_arrival_idx = _first_index_at(run, *d_xy)
    interior = vels[: d_arrival_idx - 1]
    assert all(v > floor_speed for v in interior), (
        "an intra-run dip toward the floor was emitted — the run split at a "
        "sub-tolerance alignment turn"
    )


# ===========================================================================
# AC2 (render) — collinear PROJECTION chain shows NO per-node dip
# ===========================================================================


def test_ac2_collinear_projection_chain_no_per_node_dip():
    """AC2 (render): the line-proj-line chain holds cruise across the projection node.

    Renders the feature-complete ``field_fixture`` collinear line-projection-line
    triple ``Inter1 -> ProjL -> Inter2`` (the SAME map the ADR-050 parity sweep
    drives). A ``goto("Inter1")`` seats the chassis on the line at ``Inter1``,
    then ``goto("Inter2")`` drives the pass-through across the projection node
    ``ProjL``. The render must hold cruise across ``ProjL`` — the inbound
    velocity at ``ProjL`` equals the follow cruise, NOT the deadband floor — so
    there is no per-node dip (the render half of the T2 firmware cruise-hold
    fix). A zero-motion ``ACT_HEADING_RESET`` fires at ``ProjL`` in this stream;
    the run must stay grouped through it (no spurious ``UserWarning`` — enforced
    by ``_build_quiet``).
    """
    field_data = field_fixture.build_field_data()
    trace = load(
        {
            "schema_version": 1,
            "start_node": field_data.START_NODE,
            "start_heading_deg": field_data.START_HEADING_DEG,
            "build_error": False,
            "commands": [
                {"seq": 0, "op": "goto", "target": "Inter1"},
                {"seq": 1, "op": "goto", "target": "Inter2"},
            ],
        }
    )
    track = _build_quiet(trace, field_data)

    cfg = default_config()
    cruise_speed = SCALE_CM_S_PER_PWM * cfg.line_cruise
    floor_speed = SCALE_CM_S_PER_PWM * cfg.pwm_min
    assert cruise_speed > floor_speed + 1.0

    kfs = track.keyframes
    proj_xy = tuple(float(c) for c in field_data.NODES["ProjL"])
    inter2_xy = tuple(float(c) for c in field_data.NODES["Inter2"])

    # The Inter1 -> Inter2 run begins at the LAST keyframe on Inter1.
    inter1_xy = tuple(float(c) for c in field_data.NODES["Inter1"])
    run_start = _last_index_at(kfs, *inter1_xy)
    run = kfs[run_start:]
    vels = _inbound_velocities(run)

    # Cruise held THROUGH the projection node ProjL — no dip to the floor.
    proj_vel = _velocity_into(run, vels, *proj_xy)
    assert proj_vel == pytest.approx(cruise_speed, abs=_VEL_TOL), (
        f"projection node ProjL dipped: v={proj_vel} (cruise={cruise_speed}, "
        f"floor={floor_speed})"
    )
    # And it is strictly above the floor (a dip would sit at the floor).
    assert proj_vel > floor_speed + 1.0

    # TC7: ProjL and Inter2 boundary poses appear byte-exact, in order.
    proj_idx = _last_index_at(run, *proj_xy)
    inter2_idx = _last_index_at(run, *inter2_xy)
    assert 0 < proj_idx < inter2_idx


# ===========================================================================
# TC11 (interleave) — a zero-motion record mid-run keeps the run grouped
# ===========================================================================


def test_tc11_heading_reset_interleave_keeps_run_grouped(monkeypatch):
    """TC11: an ``ACT_HEADING_RESET`` interleaved mid-run stays grouped, no warning.

    Scripts a collinear run — a LINE pass, a ZERO-MOTION ``ACT_HEADING_RESET``
    (its snapshot pose equals the run tail), a second LINE pass, then a braked
    ``ACT_LINE_OVERSHOOT`` stop — through the ``step_through`` spy. The
    heading-reset record is TRANSPARENT: it neither contributes distance nor
    breaks the run, so the whole thing renders as ONE grouped trapezoid with a
    single cruise plateau across both pass boundaries, and NO ``UserWarning``
    fires (a spurious per-primitive fallback would).
    """
    cfg = default_config()
    cruise_speed = SCALE_CM_S_PER_PWM * cfg.line_cruise
    floor_speed = SCALE_CM_S_PER_PWM * cfg.pwm_min

    offset = 12.02
    sequence = [
        ((ACT_LINE_PASS, offset), 40.0),                       # pass (motion)
        ((ACT_HEADING_RESET, "ProjX", "proj", 0.0), 0.0),      # zero-motion record
        ((ACT_LINE_PASS, offset), 40.0),                       # pass (motion)
        ((ACT_LINE_OVERSHOOT, offset), offset),                # braked stop (ends run)
    ]
    monkeypatch.setattr(
        PathTrackerSim, "step_through", _make_step_through_spy(sequence)
    )

    trace = Trace(
        schema_version=1,
        start_node=toy_field_data.START_NODE,
        start_heading_deg=toy_field_data.START_HEADING_DEG,
        build_error=False,
        commands=[Command(seq=0, op="goto", target="B")],
    )

    with warnings.catch_warnings():
        # Any UserWarning here is a spurious malformed-run rejection — fail loud.
        warnings.simplefilter("error", UserWarning)
        track = build(trace, toy_field_data)

    kfs = track.keyframes
    vels = _inbound_velocities(kfs)
    # ONE trapezoid: a single contiguous cruise plateau (the two passes are one
    # held run, not two separated by the heading-reset). The floor only appears
    # at the terminal ramp-down.
    plateau = [i for i, v in enumerate(vels) if abs(v - cruise_speed) <= _VEL_TOL]
    assert plateau, "no cruise plateau — the interleaved run did not group"
    assert plateau == list(range(plateau[0], plateau[-1] + 1)), (
        "the cruise plateau is broken — the heading-reset split the run"
    )
    # No interior stop keyframe (the run is unbroken up to the terminal brake).
    interior = vels[:-1]
    assert all(v > _VEL_TOL for v in interior)


# ===========================================================================
# REAR line hop — the reverse follow leg is a mid-run contributor (ADR-061 #122
# follow-up). Regression for the "stop in the middle of the hop" bug.
# ===========================================================================


def test_rear_line_hop_reverse_leg_is_mid_run_no_mid_edge_stop(monkeypatch):
    """A REAR line hop's ``ACT_LINE_REVERSE`` leg groups like ``ACT_FOLLOW_INT``.

    The sim emits a REAR line hop symmetrically to a forward one: a follow leg
    that lands one axle-offset SHORT of the node (``ACT_LINE_REVERSE``, the REAR
    mirror of ``ACT_FOLLOW_INT``), then a SEPARATE ``ACT_LINE_OVERSHOOT`` that
    rolls onto the node. Both follow legs must be mid-run contributors, so the
    reverse leg + overshoot render as ONE trapezoid — cruise held THROUGH the
    reverse leg's short-landing point, a single terminal ramp-down/stop, and
    exactly ONE settle dwell (the goal arrival after the overshoot).

    Before the fix, ``ACT_LINE_REVERSE`` was absent from
    ``_MID_RUN_CONTRIB_TAGS``, so the grouper treated the reverse leg as a braked
    arrival: the run split at its short-landing point into two ramps (an interior
    stop), and ADR-061's hop-settle fired a spurious mid-edge dwell there — the
    ~100 ms "stop in the middle of the hop" on ``CG2 -> CG1``.
    """
    cfg = default_config()
    assert cfg.hop_settle_ms > 0, "settle must be ON for this regression to bite"
    cruise_speed = SCALE_CM_S_PER_PWM * cfg.line_cruise

    offset = 12.02  # rear-board axle offset (the overshoot roll distance)
    reverse_leg_cm = 40.0
    sequence = [
        # Reverse follow leg — lands SHORT of the node (mid-run, must not brake).
        ((ACT_LINE_REVERSE, "B", "line"), reverse_leg_cm),
        # Overshoot rolls onto the node (braked goal arrival — one settle dwell).
        ((ACT_LINE_OVERSHOOT, offset), offset),
    ]
    monkeypatch.setattr(
        PathTrackerSim, "step_through", _make_step_through_spy(sequence)
    )

    trace = Trace(
        schema_version=1,
        start_node=toy_field_data.START_NODE,
        start_heading_deg=toy_field_data.START_HEADING_DEG,
        build_error=False,
        commands=[Command(seq=0, op="goto", target="B")],
    )

    with warnings.catch_warnings():
        warnings.simplefilter("error", UserWarning)
        track = build(trace, toy_field_data)

    kfs = track.keyframes
    seed = kfs[0]
    vels = _inbound_velocities(kfs)

    # ONE trapezoid: a contiguous cruise plateau, not two ramps split at the
    # reverse leg's short-landing point.
    plateau = [i for i, v in enumerate(vels) if abs(v - cruise_speed) <= _VEL_TOL]
    assert plateau, "no cruise plateau — the reverse hop did not group into one run"
    assert plateau == list(range(plateau[0], plateau[-1] + 1)), (
        "the cruise plateau is broken — ACT_LINE_REVERSE split the run"
    )

    # Exactly ONE settle dwell — the goal arrival after the overshoot — and it
    # sits at the FINAL (goal) pose, never at the reverse leg's short-landing
    # point (seed + reverse_leg_cm), which would be the spurious mid-edge stop.
    dwells = _zero_motion_dwells(kfs)
    assert len(dwells) == 1, (
        f"expected 1 goal-arrival settle dwell, got {len(dwells)} "
        "(a mid-edge dwell means the reverse leg was mis-classified as braked)"
    )
    dwell_kf = kfs[dwells[0]]
    total_advance = math.hypot(dwell_kf["x"] - seed["x"], dwell_kf["y"] - seed["y"])
    assert total_advance == pytest.approx(reverse_leg_cm + offset, abs=1e-6), (
        "the settle dwell is not at the goal pose — it fired mid-edge"
    )


# ===========================================================================
# TC11 (malformed) — pass-without-stop and blind-missing-speeds fall back + warn
# ===========================================================================


def test_tc11_pass_without_braked_stop_falls_back_with_warning(monkeypatch):
    """TC11 (malformed a): a grouped pass run with no braked stop warns + falls back.

    Scripts two LINE passes (each with motion) and NO following braked stop, so
    the run survives GROUPED to the goto-end strict flush — a genuinely malformed
    trace (a pass tag with no following braked stop). The grouper must REJECT the
    group and fall back to per-primitive rendering WITH a ``UserWarning`` (assert
    the warning fires and both segments still render — nothing is dropped).
    """
    sequence = [
        ((ACT_LINE_PASS, 12.02), 20.0),
        ((ACT_LINE_PASS, 12.02), 20.0),
    ]
    monkeypatch.setattr(
        PathTrackerSim, "step_through", _make_step_through_spy(sequence)
    )

    trace = Trace(
        schema_version=1,
        start_node=toy_field_data.START_NODE,
        start_heading_deg=toy_field_data.START_HEADING_DEG,
        build_error=False,
        commands=[Command(seq=0, op="goto", target="B")],
    )

    with pytest.warns(UserWarning, match="pass tag with no following braked stop"):
        track = build(trace, toy_field_data)

    # Fallback still renders both passes (nothing dropped): the chassis advanced
    # the full 40 cm across the two per-primitive segments.
    seed = track.keyframes[0]
    moved = track.keyframes[-1]
    advance = math.hypot(moved["x"] - seed["x"], moved["y"] - seed["y"])
    assert advance == pytest.approx(40.0, abs=1e-6)


def test_tc11_blind_run_missing_speeds_falls_back_with_warning(monkeypatch):
    """TC11 (malformed b): a blind run missing a hop's recorded speeds warns + falls back.

    Scripts a CHAINED blind mid-hop (carrying a recorded ``(start, end,
    chain=True)`` suffix) + its ``ACT_BLIND_PASS`` boundary, then a PLAIN
    ``ACT_FORWARD`` braked hop with NO recorded suffix. The run is marked a blind
    run (the first segment carries recorded speeds) but a segment is missing its
    recorded ``(start, end)`` — the grouper must NOT fabricate a cruise. It
    REJECTS the group and falls back to per-primitive rendering WITH a
    ``UserWarning``.
    """
    sequence = [
        ((ACT_FORWARD, 20.0, 50, 85, True), 20.0),   # chained mid hop (has speeds)
        ((ACT_BLIND_PASS, "X"), 0.0),                # zero-motion boundary (transparent)
        ((ACT_FORWARD, 20.0), 20.0),                 # plain braked hop (NO recorded speeds)
    ]
    monkeypatch.setattr(
        PathTrackerSim, "step_through", _make_step_through_spy(sequence)
    )

    trace = Trace(
        schema_version=1,
        start_node=toy_field_data.START_NODE,
        start_heading_deg=toy_field_data.START_HEADING_DEG,
        build_error=False,
        commands=[Command(seq=0, op="goto", target="B")],
    )

    with pytest.warns(UserWarning, match="blind run missing recorded per-hop speeds"):
        track = build(trace, toy_field_data)

    # Fallback still renders both moving hops (the zero-motion pass contributes
    # nothing): the chassis advanced the full 40 cm.
    seed = track.keyframes[0]
    moved = track.keyframes[-1]
    advance = math.hypot(moved["x"] - seed["x"], moved["y"] - seed["y"])
    assert advance == pytest.approx(40.0, abs=1e-6)


def test_tc11_zero_motion_record_is_not_malformed(monkeypatch):
    """TC11: a run whose ONLY interruption is a zero-motion record is NOT malformed.

    Guards the transparency contract from the other side: a heading-reset mid-run
    (with a proper braked terminal stop) must render WITHOUT any warning — a
    zero-motion record is transparent, never a malformed trigger.
    """
    sequence = [
        ((ACT_LINE_PASS, 12.02), 30.0),
        ((ACT_HEADING_RESET, "ProjX", "proj", 0.0), 0.0),
        ((ACT_LINE_OVERSHOOT, 12.02), 12.02),
    ]
    monkeypatch.setattr(
        PathTrackerSim, "step_through", _make_step_through_spy(sequence)
    )

    trace = Trace(
        schema_version=1,
        start_node=toy_field_data.START_NODE,
        start_heading_deg=toy_field_data.START_HEADING_DEG,
        build_error=False,
        commands=[Command(seq=0, op="goto", target="B")],
    )

    with warnings.catch_warnings():
        warnings.simplefilter("error", UserWarning)
        build(trace, toy_field_data)  # must not raise


# ===========================================================================
# TC7 (render) / review C1 — LINE->BLIND spanning run renders as ONE trapezoid
# ===========================================================================


def test_tc7_line_to_blind_spanning_run_renders_one_trapezoid(monkeypatch):
    """TC7 (render) / review C1: a LINE->BLIND spanning run is ONE trapezoid, no warning.

    Replays the REAL primitive stream the parity sim emits for the
    ``field_fixture`` TC7 topology ``LpIn -> LpJoin -> LpBlindA -> LpBlindB``
    (a LINE pass-through ``LpIn -> LpJoin`` feeding straight into the 2-hop
    blind chain ``LpJoin -> LpBlindA -> LpBlindB``), via the ``step_through``
    spy. The leading LINE segments legitimately carry NO recorded chain speeds
    (line-follow speed comes from the LineFollower preset, not per-hop
    ``ChainSpeedPolicy``), so the grouper must treat this as a VALID spanning
    run — the plan's Decision "a collinear run MAY span LINE/projection ->
    BLIND" — never a malformed one:

    * NO ``UserWarning`` (the broken grouper rejected the run as "blind run
      missing recorded per-hop speeds" and fell back per-primitive, re-seeding
      each speed-less segment from the start floor — the exact mid-run dip
      this feature exists to remove, on its flagship route shape).
    * ONE trapezoid: a single contiguous cruise plateau; velocity INTO and OUT
      OF the LINE->BLIND handoff node both equal cruise (continuous across the
      handoff — the ADR-023 cruise carry: the firmware passes through at
      cruise and hands the captured cruise into the blind chain,
      ``PathTracker.cpp:2674-2690``).
    * one monotonic ramp-down into the final node only; no interior stop.
    * TC7 pose pins: every node-boundary pose appears byte-exact, in order.
    """
    # The captured field_fixture stream (LpIn seat, goto LpBlindB): a LINE
    # follow leg + pass-through roll (NO chain speeds), then the recorded blind
    # chain — mid hop (85 -> 85, chain), zero-motion blind_pass boundary,
    # braked final hop (85 -> 0, chain).
    sequence = [
        ((ACT_FOLLOW_INT, None), 12.98),            # LINE follow leg (no speeds)
        ((ACT_LINE_PASS, 12.02), 12.02),            # pass roll onto the handoff node
        ((ACT_FORWARD, 20.0, 85, 85, True), 20.0),  # chained blind mid hop
        ((ACT_BLIND_PASS, "LpBlindA"), 0.0),        # zero-motion boundary (transparent)
        ((ACT_FORWARD, 20.0, 85, 0, True), 20.0),   # chained braked final hop
    ]
    monkeypatch.setattr(
        PathTrackerSim, "step_through", _make_step_through_spy(sequence)
    )

    field_data = collinear_4node.make_ctps_field_data()
    trace = load(
        {
            "schema_version": 1,
            "start_node": field_data.START_NODE,
            "start_heading_deg": field_data.START_HEADING_DEG,
            "build_error": False,
            "commands": [{"seq": 0, "op": "goto", "target": "D"}],
        }
    )
    # NO UserWarning: a spanning run is valid, never a malformed fallback.
    track = _build_quiet(trace, field_data)

    # Node-boundary poses along the spy's straight +x advance from the spawn.
    kfs = track.keyframes
    x0, y0 = kfs[0]["x"], kfs[0]["y"]
    follow_xy = (x0 + 12.98, y0)    # follow leg end (sensor short of the node)
    handoff_xy = (x0 + 25.0, y0)    # LINE -> BLIND handoff node (LpJoin)
    blind_mid_xy = (x0 + 45.0, y0)  # blind mid node (LpBlindA)
    goal_xy = (x0 + 65.0, y0)       # braked goal (LpBlindB)

    # The recorded chain cruise (85) is the run peak — after a pass-through it
    # IS the captured line cruise (the same physical cruise, ADR-023).
    cruise_speed = SCALE_CM_S_PER_PWM * 85
    floor_speed = SCALE_CM_S_PER_PWM * default_config().pwm_min
    assert cruise_speed > floor_speed + 1.0

    vels = _inbound_velocities(kfs)

    # Velocity continuous across the LINE -> BLIND handoff: cruise held both
    # INTO and OUT OF the handoff node — no dip to the floor at the boundary.
    handoff_idx = _last_index_at(kfs, *handoff_xy)
    assert vels[handoff_idx - 1] == pytest.approx(cruise_speed, abs=_VEL_TOL), (
        f"dip INTO the line->blind handoff: v={vels[handoff_idx - 1]} "
        f"(cruise={cruise_speed}, floor={floor_speed})"
    )
    assert vels[handoff_idx] == pytest.approx(cruise_speed, abs=_VEL_TOL), (
        f"dip OUT OF the line->blind handoff: v={vels[handoff_idx]}"
    )
    # And held through the blind mid node too.
    assert _velocity_into(kfs, vels, *blind_mid_xy) == pytest.approx(
        cruise_speed, abs=_VEL_TOL
    )

    # ONE trapezoid: a single contiguous cruise plateau — no intra-run dip
    # anywhere (the per-primitive fallback would break it at each speed-less
    # segment boundary by re-seeding from the start floor).
    plateau = [i for i, v in enumerate(vels) if abs(v - cruise_speed) <= _VEL_TOL]
    assert plateau, "no cruise plateau — the spanning run never reached cruise"
    assert plateau == list(range(plateau[0], plateau[-1] + 1)), (
        "cruise plateau broken — the run dipped mid-way (per-node dip lie)"
    )

    # TC7 pose pins: each boundary pose appears byte-exact, in order; the goal
    # is the terminal keyframe.
    follow_idx = _last_index_at(kfs, *follow_xy)
    blind_mid_idx = _last_index_at(kfs, *blind_mid_xy)
    goal_idx = _last_index_at(kfs, *goal_xy)
    assert 0 < follow_idx < handoff_idx < blind_mid_idx < goal_idx
    assert goal_idx == len(kfs) - 1

    # The handoff and blind-mid boundaries sit INSIDE the single plateau span.
    assert plateau[0] < handoff_idx <= plateau[-1] + 1
    assert plateau[0] < blind_mid_idx <= plateau[-1] + 1

    # One ramp-down only, into the final node: monotonic non-increasing after
    # the plateau; no interior stop keyframe before the terminal brake.
    ramp_down = vels[plateau[-1] + 1 : goal_idx]
    assert all(b <= a for a, b in zip(ramp_down, ramp_down[1:])), (
        "ramp-down not monotonic"
    )
    interior = vels[: goal_idx - 1]
    assert all(v > _VEL_TOL for v in interior), (
        "an intra-run stop keyframe (interior zero velocity) was emitted"
    )


# ===========================================================================
# TC12 / AC12 — non-chain single hop + an existing trace render byte-identical
# ===========================================================================


def test_tc12_non_chain_single_hop_renders_stable_hash():
    """TC12 / AC12: a non-chain single hop renders to a stable, grouper-invariant track.

    ``goto("A")`` from the blind spawn ``S`` is a SINGLE blind hop (hop count 1,
    NOT a chain). The chain/grouper logic applies only to ``hopCount >= 2``
    eligible chains, so this renders through the ordinary per-primitive path —
    the grouper reduces a single-segment run to the same one-ramp/stop shape.
    Rendering it twice yields a byte-identical track (deterministic build), and
    the render is warning-free (a single hop is never a grouped/malformed run).
    """
    field_data = collinear_4node.make_ctps_field_data()
    trace = load(
        {
            "schema_version": 1,
            "start_node": field_data.START_NODE,
            "start_heading_deg": 0.0,
            "build_error": False,
            "commands": [{"seq": 0, "op": "goto", "target": "A"}],
        }
    )
    first = _build_quiet(trace, field_data)
    second = _build_quiet(trace, field_data)
    assert _track_hash(first) == _track_hash(second)

    # A single hop lands on the node cleanly: exactly one goto span event, no
    # drift_warn, and the final pose is the node A.
    assert [ev for ev in first.events if ev["type"] == "drift_warn"] == []
    a_xy = collinear_4node.NODES["A"]
    last = first.keyframes[-1]
    assert last["x"] == pytest.approx(a_xy[0], abs=_POSE_TOL)
    assert last["y"] == pytest.approx(a_xy[1], abs=_POSE_TOL)


def test_tc12_existing_trace_renders_byte_identical():
    """TC12 / AC12: an existing (pre-feature) trace renders byte-identical across builds.

    ``ctps_two_goto.json`` is a pre-feature trace over the toy field whose route
    takes only single-hop / turn moves (no eligible collinear chain), so the
    run-grouper leaves it exactly as the pre-grouper builder produced it.
    Rendering it twice yields identical sha256 digests (a stable, deterministic
    track — the regression baseline for the grouper being additive on
    non-chain routes). The toy field's orphan P1 node makes ``build`` emit the
    benign orphan warning, so this build suppresses UserWarning rather than
    treating it as an error.
    """
    trace = load(Path(__file__).resolve().parent / "fixtures" / "ctps_two_goto.json")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        first = build(trace, toy_field_data)
        second = build(trace, toy_field_data)
    assert _track_hash(first) == _track_hash(second)
