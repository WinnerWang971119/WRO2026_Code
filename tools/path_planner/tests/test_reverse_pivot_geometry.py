"""World-frame geometry proof for reverse (REAR-traversal) single-wheel pivots.

TC15 / AC11 of the reverse-pivots plan
(``docs/plans/2026-07-19-issue54-reverse-pivots.md``). Cross-surface parity
(``test_pathtracker_parity.py``, ``test_reverse_pivot_policy_parity.py``) only
proves firmware / sim / CTPS AGREE with each other -- they could agree while all
three are wrong. This module proves the pivot arc geometry is *right* from FIRST
PRINCIPLES, independent of any single surface's implementation:

* **arc-end axle pose** = ``v + delta * u_out(travel)`` -- the rigid rotation
  about the planted wheel ends the axle ``delta`` PAST the pivot node ``v`` along
  the OUTGOING TRAVEL ray, NOT on ``v`` (the departing dead-reckon trim then nets
  the chassis to ``w``; on a sensor-terminated departure the destination line
  re-anchors it -- ADR-054). A test asserting "arc-end on ``v``" would codify
  wrong geometry.
* **planted-wheel world side** = the INSIDE of the turn: ``v - delta *
  u_in(travel)`` offset ``track/2`` toward ``sign(theta) * 90 deg`` off the
  inbound TRAVEL heading. This is direction-INVARIANT: a forward pivot and its
  reverse counterpart plant the wheel at the SAME world point (the L<->R wheel
  flip and the ``+180`` body-heading offset cancel -- the Data / geometry model
  in ADR-073), so the two coincide in the world frame.

Everything is derived from the fixture's node coordinates + the parsed chassis
track (``compute_pivot_compensation`` with ``parse_track_cm_from_config``) and
compared against what ``track_builder`` actually renders (the pivot arc keyframes
+ the ``pivots[]`` segment centre). ``delta = (T/2) * tan(|theta|/2)`` is never
hard-coded.

The AUTO-REAR fixture triple is asserted to actually FIRE the AUTO REAR rung
(``taken_pivot(...).direction == REAR``): an axis-aligned fixture would tie
FRONT under ``BIDIRECTIONAL_AUTO`` (a false green), so the fixture uses a >90 deg
approach delta and the test proves the rung fired.
"""

from __future__ import annotations

import warnings
from math import atan2, ceil, cos, degrees, hypot, radians, sin

import pytest

from tools.path_planner.compiler import normalize_turn
from tools.path_planner.ctps.config import ARC_STEP_DEG
from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import Command, Trace
from tools.path_planner.planner import (
    compute_pivot_compensation,
    parse_track_cm_from_config,
)
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.chassis import VirtualChassis
from tools.path_planner.sim.runtime import Direction, PathTrackerSim
from tools.path_planner.tests.fixtures import pivot_field_data

#: Closed-form geometry tolerance (cm). The sim / track_builder integrate the arc
#: in float64, so the first-principles closed form matches to a few ULPs.
_GEOM_TOL_CM: float = 1e-6

_TRACK_CM: float = parse_track_cm_from_config()


# ---------------------------------------------------------------------------
# Fixtures + track-build helpers
# ---------------------------------------------------------------------------


def _goto_trace(field_data, target: str) -> Trace:
    """Build a synthetic single-``goto`` Trace targeting ``target`` (see CTPS TC).

    :func:`build` always seeds the chassis from ``field_data``, so
    ``start_node`` / ``start_heading_deg`` here are informational only.
    """
    return Trace(
        schema_version=1,
        start_node=field_data.START_NODE,
        start_heading_deg=field_data.START_HEADING_DEG,
        build_error=False,
        commands=[Command(seq=0, op="goto", target=target)],
    )


def _build_quiet(trace, field_data):
    """Run :func:`build`, silencing the benign small-fixture orphan-node warning."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return build(trace, field_data)


def _matched_pivot_field(direction: str):
    """Return a BLIND->BLIND 90 deg pivot ``U -> V -> W`` pinned to ``direction``.

    A minimal matched pair built inline (via the T9 module's shared
    :func:`pivot_field_data._make_field`, NOT touching that file) so the forward
    and reverse variants share IDENTICAL geometry -- the only difference is the
    ``direction:"front"`` / ``direction:"rear"`` pin on both pivot legs. Same
    nodes -> same world-frame arc-end + planted-wheel point, so the forward and
    reverse renders must coincide in the world frame.

    ``U(0,0) -> V(0,60)`` heads 90 deg; ``V(0,60) -> W(60,60)`` heads 0 deg, so
    the turn at ``V`` is ``theta = wrap(0 - 90) = -90`` deg (a right turn,
    ``delta = 9`` cm at ``TRACK_MM = 180``). Both legs dead-reckon (BLIND), so no
    sensor overshoot budget is involved -- this pair isolates the pivot ARC
    geometry.

    :param direction: ``"front"`` or ``"rear"`` -- pinned on both pivot legs.
    """
    return pivot_field_data._make_field(
        nodes={"U": (0.0, 0.0), "V": (0.0, 60.0), "W": (60.0, 60.0)},
        node_types={"U": "blind", "V": "blind", "W": "blind"},
        line_segments=[],
        blind_edges=[],
        pivot_turns=[("U", "V", "W")],
        start_node="U",
        start_heading_deg=90.0,
        edges=[
            {"type": "BLIND", "u": "U", "v": "V", "direction": direction},
            {"type": "BLIND", "u": "V", "v": "W", "direction": direction},
        ],
    )


def _arc_end_keyframe(track, segment):
    """Return the arc-END keyframe for a ``pivots[]`` segment.

    Locates the arc via the segment's ``[t0, t1]`` bracket (the approach leg
    densifies, so the arc is not at a fixed index -- the same idiom the CTPS
    track-builder tests use): the arc keyframes are the ``N =
    ceil(|theta| / ARC_STEP_DEG)`` samples with ``t > t0``, and ``arc[-1]`` is
    the arc-end pose (``(cx_end, cy_end)`` -- the axle after the rigid rotation,
    BEFORE the departure leg densifies).
    """
    n = ceil(abs(segment["theta_deg"]) / ARC_STEP_DEG)
    arc_start_idx = next(
        i for i, kf in enumerate(track.keyframes) if kf["t"] > segment["t0"]
    )
    arc = track.keyframes[arc_start_idx : arc_start_idx + n]
    assert len(arc) == n
    return arc[-1]


def _closed_form_geometry(field, u: str, v: str, w: str):
    """Return the FIRST-PRINCIPLES pivot geometry for the turn ``(u, v, w)``.

    Derives everything from the fixture's node coordinates + the parsed chassis
    track -- never from ``track_builder`` -- so comparing the rendered arc against
    it is a genuine cross-check, not a tautology:

    * ``theta`` = ``wrap(h_out - h_in)`` (the TRAVEL-frame body rotation);
    * ``delta`` = ``(track/2) * tan(|theta|/2)`` via
      :func:`compute_pivot_compensation`;
    * ``arc_end`` = ``v + delta * u_out(travel)`` (delta PAST ``v`` along the
      outgoing travel ray);
    * ``wheel`` = ``(v - delta * u_in) + (track/2) * perp`` where ``perp`` points
      ``sign(theta) * 90 deg`` off the INBOUND travel heading -- the INSIDE of the
      turn, direction-invariant (forward + reverse coincide here).

    :return: ``(theta_deg, delta_cm, (arc_end_x, arc_end_y), (wheel_x, wheel_y))``.
    """
    ux, uy = field.NODES[u]
    vx, vy = field.NODES[v]
    wx, wy = field.NODES[w]
    h_in = degrees(atan2(vy - uy, vx - ux))
    h_out = degrees(atan2(wy - vy, wx - vx))
    theta_deg = normalize_turn(h_out - h_in)
    delta = compute_pivot_compensation(theta_deg, _TRACK_CM)

    arc_end = (
        vx + delta * cos(radians(h_out)),
        vy + delta * sin(radians(h_out)),
    )

    radius = _TRACK_CM / 2.0
    c0 = (vx - delta * cos(radians(h_in)), vy - delta * sin(radians(h_in)))
    perp = radians(h_in + (1.0 if theta_deg > 0 else -1.0) * 90.0)
    wheel = (c0[0] + radius * cos(perp), c0[1] + radius * sin(perp))
    return theta_deg, delta, arc_end, wheel


def _walk_taken_pivot(field, target: str, u: str, v: str, w: str):
    """Drive an INDEPENDENT sim to ``target`` and return ``taken_pivot(u, v, w)``.

    Fresh sim (not the one ``build`` uses internally) so the taken-pivot read is a
    genuine cross-check on the direction the walk DERIVED (force-SAME), not a
    value plucked from the track builder.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph, next_hop = pivot_field_data.build_graph_and_next_hop(field)
    sim = PathTrackerSim(field, graph, next_hop, VirtualChassis.from_field_data(field))
    sim.localization.zero_heading_from_current()
    for _prim, _snap in sim.step_through(target):
        pass
    return sim.taken_pivot(u, v, w)


def _assert_arc_end_and_wheel(field, u, v, w, target, expected_direction):
    """Build the track, then assert the arc-end + planted wheel match closed form.

    Returns ``(segment, arc_end_keyframe)`` for callers that additionally assert
    forward/reverse coincidence.
    """
    track = _build_quiet(_goto_trace(field, target), field)
    assert len(track.pivots) == 1
    segment = track.pivots[0]
    assert segment["direction"] == expected_direction

    theta_deg, delta, exp_arc_end, exp_wheel = _closed_form_geometry(field, u, v, w)

    # The rendered body rotation is the TRAVEL-frame theta (the +180 body offsets
    # cancel), identical for forward and reverse traversal.
    assert segment["theta_deg"] == pytest.approx(theta_deg, abs=_GEOM_TOL_CM)

    # Arc-end axle = v + delta * u_out(travel): delta PAST v, NOT on v.
    arc_end = _arc_end_keyframe(track, segment)
    assert arc_end["x"] == pytest.approx(exp_arc_end[0], abs=_GEOM_TOL_CM)
    assert arc_end["y"] == pytest.approx(exp_arc_end[1], abs=_GEOM_TOL_CM)

    vx, vy = field.NODES[v]
    dist_from_v = hypot(arc_end["x"] - vx, arc_end["y"] - vy)
    assert dist_from_v == pytest.approx(delta, abs=_GEOM_TOL_CM)
    # The arc-end is genuinely OFF the node -- not a degenerate "lands on v".
    assert delta > 1.0
    assert dist_from_v > 1.0

    # Planted wheel on the correct WORLD side (the inside of the turn).
    assert segment["cx"] == pytest.approx(exp_wheel[0], abs=_GEOM_TOL_CM)
    assert segment["cy"] == pytest.approx(exp_wheel[1], abs=_GEOM_TOL_CM)

    return segment, arc_end


# ---------------------------------------------------------------------------
# TC15 / AC11 -- arc-end pose + planted-wheel world side, forward AND reverse
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "direction, expected_tag",
    [(Direction.FRONT, "front"), (Direction.REAR, "rear")],
    ids=["forward", "reverse"],
)
def test_pivot_arc_end_is_delta_past_node_along_travel_ray(direction, expected_tag):
    """AC11: the pivot arc-end axle = ``v + delta * u_out(travel)`` (delta PAST
    ``v``), and the planted wheel is on the correct world side -- forward AND
    reverse, from first principles.

    The matched ``U -> V -> W`` pivot is driven once FRONT-pinned and once
    REAR-pinned. In BOTH renders the arc-end lands ``delta`` past ``v`` along the
    outgoing TRAVEL ray (never on ``v``), and the ``pivots[]`` segment centre --
    the planted wheel -- sits on the inside of the turn exactly where the
    closed-form geometry puts it. The reverse render reaches the SAME world
    points as the forward one despite the L<->R wheel flip + the ``+180`` body
    heading (they cancel -- proven directly by the coincidence test below).
    """
    field = _matched_pivot_field(expected_tag)

    # Cross-check: the walk actually DERIVED this direction (force-SAME).
    taken = _walk_taken_pivot(field, "W", "U", "V", "W")
    assert taken is not None
    assert taken.direction == direction

    _assert_arc_end_and_wheel(field, "U", "V", "W", "W", expected_tag)


def test_pivot_forward_and_reverse_coincide_in_world_frame():
    """AC11: a forward pivot and its reverse counterpart plant the wheel + end the
    arc at the SAME world points -- the flip + the +180 body heading cancel.

    The two matched fields share IDENTICAL geometry, differing only in the
    ``front`` / ``rear`` pin. The reverse pivot flips the planted wheel side
    (``left`` <-> ``right`` on the ``pivots[]`` segment) AND faces the body 180 deg
    opposite; those two effects cancel in the world frame, so the rendered
    planted-wheel centre and the arc-end axle pose coincide to ~1e-6 across the
    two directions. This is the load-bearing claim of the Data / geometry model:
    the reverse correction is a pure body-frame relabel, not a different world
    trajectory.
    """
    front_field = _matched_pivot_field("front")
    rear_field = _matched_pivot_field("rear")

    seg_f, end_f = _assert_arc_end_and_wheel(
        front_field, "U", "V", "W", "W", "front"
    )
    seg_r, end_r = _assert_arc_end_and_wheel(
        rear_field, "U", "V", "W", "W", "rear"
    )

    # The planted-wheel SIDE flips L<->R between forward and reverse...
    assert {seg_f["wheel"], seg_r["wheel"]} == {"left", "right"}

    # ...yet the planted-wheel WORLD point coincides (the +180 body heading
    # cancels the flip).
    assert hypot(seg_f["cx"] - seg_r["cx"], seg_f["cy"] - seg_r["cy"]) < _GEOM_TOL_CM

    # ...and the arc-end axle world pose coincides too.
    assert hypot(end_f["x"] - end_r["x"], end_f["y"] - end_r["y"]) < _GEOM_TOL_CM


# ---------------------------------------------------------------------------
# TC15 / AC11 -- the AUTO-REAR rung genuinely FIRES (false-green trap)
# ---------------------------------------------------------------------------


def test_auto_rear_pivot_rung_fires_and_arc_end_matches_closed_form():
    """AC11 (critical): the AUTO-REAR fixture RESOLVES the approach REAR under
    ``BIDIRECTIONAL_AUTO`` (the rung fires), and its rendered arc still obeys the
    closed-form geometry.

    ``make_auto_rear_approach_pivot_field`` carries NO authored direction pin: the
    ``U -> V`` approach must resolve REAR purely from a >90 deg, non-axis-aligned
    entry heading. An axis-aligned fixture would tie FRONT (a false green), so the
    fired rung is ASSERTED here (``taken_pivot(...).direction == REAR``), not
    assumed. The sharp 120 deg pivot then renders a single coherent arc whose
    arc-end axle = ``v + delta * u_out(travel)`` and whose planted wheel sits on
    the inside of the turn -- the same first-principles checks the forward
    fixtures pass, now on an AUTO-DERIVED reverse pivot.
    """
    field = pivot_field_data.make_auto_rear_approach_pivot_field()

    # The AUTO REAR rung FIRED (proof, not assumption).
    taken = _walk_taken_pivot(field, "W", "U", "V", "W")
    assert taken is not None
    assert taken.direction == Direction.REAR

    # A genuinely sharp (>90 deg) pivot -- the regime that makes the AUTO
    # departure resolve OPPOSITE the approach without force-SAME.
    theta_deg, _delta, _end, _wheel = _closed_form_geometry(field, "U", "V", "W")
    assert abs(theta_deg) > 90.0

    _assert_arc_end_and_wheel(field, "U", "V", "W", "W", "rear")
