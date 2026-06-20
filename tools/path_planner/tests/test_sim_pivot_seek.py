"""Runtime-sim parity tests for the sensor-anchored pivot seek (issue #65, T9).

A blind hop that re-anchors on a line (a ``Drive.stopAtLine`` seek) and drives
INTO a pivot node must land the chassis axle ``delta`` short of the node, where
``delta`` is the single-wheel pivot over-travel
:func:`tools.path_planner.planner.compute_pivot_compensation`. The firmware
shortens the post-detect axle roll from the full
:data:`tools.path_planner.sim.runtime.LINE_SENSOR_AXLE_OFFSET_CM` to
``offset - delta`` (clamped to the floor) via
:func:`tools.path_planner.sim.runtime.resolve_sensor_overshoot_trim`; the sim
mirrors this in the FRONT-seek branch of :meth:`PathTrackerSim._walk_hops`. These
tests drive the sensor-incoming pivot fixtures through the sim and assert the
``delta``-short landing, the FRONT-forcing of every pivot-adjacent hop, the
chained-seek latch contract (no double-trim), and the firmware<->sim parity of
the shared subtract-and-clamp twin.

Every expected ``delta`` is recomputed from
:func:`tools.path_planner.planner.compute_pivot_compensation` (the SAME closed
form the planner bakes into each :class:`PivotTurnRecord`) -- never a
hard-coded second formula (issue #44 / TC2 discipline). Every seek advance is
derived through :func:`resolve_sensor_overshoot_trim` so the expectation tracks
the implementation rather than restating it.

Position contract note: a sensor seek hop re-anchors localization ON the
crossing line, so after the hop the route continues and the chassis CENTER keeps
moving. To pin the ``delta``-short landing of the SEEK itself, these tests
capture the chassis snapshot via :meth:`PathTrackerSim.step_through` at the
moment the seek primitive (``ACT_STOP_AT_LINE``) is emitted -- not the final
chassis after the full route.

Fixtures: ``tools/path_planner/tests/fixtures/pivot_field_data.py``.
Code under test: ``tools/path_planner/sim/runtime.py`` --
``resolve_pivot_trim`` (the shared subtract-and-clamp core),
``resolve_sensor_overshoot_trim``, the FRONT-seek branch of
``PathTrackerSim._walk_hops``, ``resolve_stop_at_line``.
C++ parity twin: ``WRO2026/PivotTrimPolicy.h::resolveSensorOvershootTrim`` +
its host test ``WRO2026/test/test_sensor_overshoot_policy.cpp`` (TC1-TC4).
"""

from __future__ import annotations

import math
import warnings

import pytest

from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FORWARD,
    ACT_TURN_REL,
)
from tools.path_planner.pipeline_config import PIPELINE_CONFIG, BlindMode, StopAtLineMode
from tools.path_planner.planner import (
    compute_pivot_compensation,
    parse_track_cm_from_config,
)
from tools.path_planner.sim import PathTrackerSim, VirtualChassis, walk_next_hop
from tools.path_planner.sim.runtime import (
    ACT_STOP_AT_LINE,
    ACT_STOP_AT_LINE_REVERSE,
    LINE_BOARD_COUNT,
    LINE_SENSOR_AXLE_OFFSET_CM,
    PIVOT_TRIM_MIN_DRIVE_CM,
    Direction,
    StopAtLine,
    resolve_pivot_trim,
    resolve_sensor_overshoot_trim,
    resolve_stop_at_line,
)
from tools.path_planner.tests.fixtures import pivot_field_data
from tools.path_planner.tests.fixtures.pivot_field_data import (
    _SI_IN_CM,
    _make_field,
    build_graph_and_next_hop,
)

# Position tolerance (cm) for the drift cross-check -- matches test_runtime_sim.
_POSITION_TOL_CM: float = 0.1

# Competition chassis track (cm): Config::TRACK_MM = 180 mm, resolved from the
# firmware header so the trims are byte-identical to the emitter's PIVOT_TURNS
# records. The fixtures use the SAME source; never hard-code 18.0 here.
_PIVOT_TRACK_CM: float = parse_track_cm_from_config()

# Pivot angles baked into the sensor-incoming fixtures (single-sourced; the
# tests recompute delta from these via compute_pivot_compensation).
_AUTORULE_TURN_DEG: float = 90.0    # make_sensor_autorule_pivot_field (delta = 9.0)
_AUTHORED_TURN_DEG: float = 70.0    # make_sensor_authored_pivot_field (delta ~= 6.30)


def _make_pivot_sim(field):
    """Return a zeroed ``(sim, chassis, graph, next_hop)`` for a pivot fixture.

    Mirrors ``test_runtime_sim._make_pivot_sim``: builds the directed graph +
    all-pairs next-hop table from the fixture field, constructs a
    :class:`PathTrackerSim` over them (which resolves the field's ``PIVOT_TURNS``
    against ``graph`` in its constructor), and zeroes localization. The
    orphan-node ``UserWarning`` a small disconnected fixture can raise is
    suppressed.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph, next_hop = build_graph_and_next_hop(field)
    chassis = VirtualChassis.from_field_data(field)
    sim = PathTrackerSim(field, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis, graph, next_hop


def _seek_advance_into_pivot(distance_cm: float, delta_cm: float) -> float:
    """Return the seek hop's forward advance when driving ``delta_cm`` into a pivot.

    Single source of the expected seek payload: the firmware/sim drives up to
    ``distance_cm`` but the crossing terminates the seek, then rolls the
    post-detect axle offset reduced by the approaching trim. The chassis center
    (the axle) therefore advances ``distance_cm - (offset - overshoot)``, where
    ``overshoot`` comes from the shared subtract-and-clamp twin
    :func:`resolve_sensor_overshoot_trim`. In the clean ``delta <= offset`` case
    this equals ``distance_cm - delta``; it tracks the floor clamp otherwise.
    Recomputed through the production helper so the expectation can never drift
    from the seek branch it checks. The ``max(0.0, ...)`` clamp mirrors the seek
    branch (and the LINE follow guard): a degenerate short edge never yields a
    backward advance.
    """
    overshoot_cm, _clamped = resolve_sensor_overshoot_trim(
        LINE_SENSOR_AXLE_OFFSET_CM, delta_cm, PIVOT_TRIM_MIN_DRIVE_CM
    )
    return max(0.0, distance_cm - (LINE_SENSOR_AXLE_OFFSET_CM - overshoot_cm))


def _seek_snapshot(sim, goal):
    """Return the ``(seek_primitive, chassis_snapshot)`` at the first seek hop.

    Walks :meth:`PathTrackerSim.step_through` and returns the
    ``ACT_STOP_AT_LINE`` primitive together with the chassis snapshot captured
    immediately AFTER that hop's mutation -- the position the seek itself lands.
    The full route continues past the seek (the chassis moves on toward the
    goal), so this snapshot, not the final chassis, is the seek's landing.

    :raises AssertionError: if the route emits no ``ACT_STOP_AT_LINE`` hop.
    """
    for prim, snap in sim.step_through(goal):
        if prim[0] == ACT_STOP_AT_LINE:
            return prim, snap
    raise AssertionError(
        f"route to {goal!r} emitted no ACT_STOP_AT_LINE seek primitive"
    )


# ===========================================================================
# TC7 -- the headline: a FRONT seek INTO a pivot lands delta short of the node
# ===========================================================================


def test_tc7_autorule_seek_lands_delta_short_of_pivot_node():
    """TC7: the BLIND->PROJECTION auto-rule seek lands the axle ``delta`` short of B.

    The user's scenario (``make_sensor_autorule_pivot_field``): incoming
    ``A -> B`` is a plain BLIND hop, outgoing ``B -> C`` a PROJECTION. With
    ``BLIND_PROJECTION_AUTO_SEEK=True`` the blind hop FEEDS a projection, so the
    auto-rule makes ``A -> B`` a ``Drive.stopAtLine`` seek; the declared pivot
    ``(A, B, C)`` is a 90 deg turn (``delta = 9.0`` cm, within the 12.02 cm FRONT
    overshoot budget). ADR-044 D1: a seek INTO an approaching pivot reduces its
    post-detect axle roll by ``delta`` so the chassis center settles ``delta``
    short of B along the incoming heading ``h_in`` (the A->B unit heading, +x).

    The seek hop emits ``(ACT_STOP_AT_LINE, distance - delta)`` and leaves the
    chassis center at ``B - delta * h_in`` (captured via the step_through
    snapshot at the seek hop). WITHOUT the trim the seek would have advanced the
    full ``distance`` and landed ON B, so the numeric gap between the two is
    exactly ``delta`` -- asserted directly. The expected advance is derived
    through :func:`resolve_sensor_overshoot_trim` so the test tracks the
    implementation.
    """
    field = pivot_field_data.make_sensor_autorule_pivot_field()
    sim, _chassis, _graph, next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_AUTORULE_TURN_DEG, _PIVOT_TRACK_CM)
    # delta fits the FRONT overshoot budget (single-sourced) -- the clean,
    # un-clamped regime, so offset - overshoot == delta exactly.
    assert delta <= LINE_SENSOR_AXLE_OFFSET_CM

    # Sanity: the route really drives the seek hop A->B into the pivot node B.
    assert walk_next_hop(next_hop, "A", "C") == ["A", "B", "C"]

    ax, ay = field.NODES["A"]
    bx, by = field.NODES["B"]
    incoming_dist = math.hypot(bx - ax, by - ay)
    # Unit incoming heading h_in (A->B). The fixture places A/B on +x, so this is
    # (1, 0); computed geometrically rather than assumed.
    h_in_x = (bx - ax) / incoming_dist
    h_in_y = (by - ay) / incoming_dist

    seek_prim, snap = _seek_snapshot(sim, "C")

    # The seek hop is a FRONT ACT_STOP_AT_LINE whose payload is the delta-short
    # advance (distance - delta in the clean regime).
    expected_seek_advance = _seek_advance_into_pivot(incoming_dist, delta)
    assert seek_prim == (ACT_STOP_AT_LINE, pytest.approx(expected_seek_advance, abs=1e-9))
    assert seek_prim[1] == pytest.approx(incoming_dist - delta, abs=1e-9)

    # Position contract: the axle lands delta short of B along h_in, i.e. at
    # B - delta * h_in. With h_in = +x that is (B.x - delta, B.y).
    expected_x = bx - delta * h_in_x
    expected_y = by - delta * h_in_y
    assert snap.x_cm == pytest.approx(expected_x, abs=1e-9)
    assert snap.y_cm == pytest.approx(expected_y, abs=1e-9)

    # Contrast: WITHOUT the trim the seek would advance the full incoming_dist and
    # land ON B. The shortfall between the two landings is exactly delta.
    landed_short_cm = math.hypot(snap.x_cm - bx, snap.y_cm - by)
    assert landed_short_cm == pytest.approx(delta, abs=1e-9)
    assert landed_short_cm > _POSITION_TOL_CM, (
        "the delta-short landing must be observably off-node; a delta below the "
        "position tolerance would make the contrast assertion vacuous"
    )

    # Localization re-anchors at the seek node B (the seek stops ON the crossing
    # line) even though the point-model center sits delta short. The snapshot is
    # captured AT the seek hop (step_through is abandoned there), so current_node
    # is the seek's re-anchor node B -- the logical node is exact while the
    # physical center sits delta short, which is the whole point.
    assert sim.localization.current_node() == "B"


def test_tc7_authored_seek_lands_delta_short_of_pivot_node():
    """TC7 (authored variant): an EDGES ``stop_at_line:"on"`` seek lands ``delta`` short.

    Same delta-short contract as the auto-rule headline, reached via the
    per-edge override path instead of the auto-rule:
    ``make_sensor_authored_pivot_field`` authors ``A -> B`` BLIND with
    ``stop_at_line: "on"`` (forces the seek independent of the auto-rule) and a
    70 deg pivot (``delta ~= 6.30`` cm, within budget). Proves the delta-short
    seek landing is a property of the seek itself, not of which precedence rung
    requested it.
    """
    field = pivot_field_data.make_sensor_authored_pivot_field()
    sim, _chassis, _graph, next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_AUTHORED_TURN_DEG, _PIVOT_TRACK_CM)
    assert delta <= LINE_SENSOR_AXLE_OFFSET_CM

    assert walk_next_hop(next_hop, "A", "C") == ["A", "B", "C"]

    bx, by = field.NODES["B"]
    seek_prim, snap = _seek_snapshot(sim, "C")

    expected_seek_advance = _seek_advance_into_pivot(_SI_IN_CM, delta)
    assert seek_prim == (ACT_STOP_AT_LINE, pytest.approx(expected_seek_advance, abs=1e-9))
    assert seek_prim[1] == pytest.approx(_SI_IN_CM - delta, abs=1e-9)

    # A/B sit on +x, so the axle lands at (B.x - delta, B.y).
    assert snap.x_cm == pytest.approx(bx - delta, abs=1e-9)
    assert snap.y_cm == pytest.approx(by, abs=1e-9)
    assert math.hypot(snap.x_cm - bx, snap.y_cm - by) == pytest.approx(
        delta, abs=1e-9
    )
    # Snapshot is at the seek hop, so localization has re-anchored at the seek
    # node B (logical node exact, physical center delta short).
    assert sim.localization.current_node() == "B"


# ===========================================================================
# TC6 -- AC6: every pivot-adjacent STOP_AT_LINE hop is forced FRONT
# ===========================================================================


def test_tc6_pivot_adjacent_seek_is_forced_front():
    """TC6 (AC6): the pivot-adjacent seek resolves ``Direction.FRONT`` before dispatch.

    Pivots are FRONT-only in v1 (ADR-036 / #54): ``_walk_hops`` force-FRONTs
    every pivot-adjacent hop. The auto-rule seek ``A -> B`` drives into the pivot
    node B, so it must dispatch FRONT and emit the forward ``ACT_STOP_AT_LINE``
    (front board seek) -- NEVER the reverse ``ACT_STOP_AT_LINE_REVERSE`` (rear
    board seek). The force-FRONT invariant (a pivot-adjacent hop is always FRONT)
    is enforced at the force-FRONT site in ``_walk_hops``; this test pins its
    observable consequence -- the pivot-adjacent seek emits the FRONT primitive,
    so a REAR pivot seek can never reach the (trim-less) REAR-seek branch.
    """
    field = pivot_field_data.make_sensor_autorule_pivot_field()
    sim, _chassis, _graph, _next_hop = _make_pivot_sim(field)

    prims = sim.go_to("C")
    tags = [p[0] for p in prims]

    # The seek hop dispatched FRONT: forward stop-at-line, never the reverse one.
    assert ACT_STOP_AT_LINE in tags, (
        f"expected a FRONT ACT_STOP_AT_LINE seek on the pivot-adjacent hop; got {tags}"
    )
    assert ACT_STOP_AT_LINE_REVERSE not in tags, (
        "pivot-adjacent seek was NOT forced FRONT (rear-board seek emitted); "
        f"got {tags}"
    )

    # The FRONT seek is the FIRST line action emitted (the A->B hop), confirming
    # the pivot-adjacent hop -- not some later hop -- is the one forced FRONT.
    line_tags = [t for t in tags if t in (ACT_STOP_AT_LINE, ACT_STOP_AT_LINE_REVERSE)]
    assert line_tags[0] == ACT_STOP_AT_LINE


# ===========================================================================
# TC8 -- chained pivots across a seek hop (latch carry + no double-trim)
# ===========================================================================
#
# The existing make_chained_pivot_field is all-BLIND (no projection -> no seek),
# so it cannot exercise a seek sitting between two pivots. We build a chained
# variant via _make_field where the MIDDLE hop is a sensor-terminated seek that
# departs one pivot and approaches the next:
#
#   A(0,0) --BLIND,90deg--> B(60,0) --BLIND seek 'on',90deg--> C(60,60)
#                                                --PROJECTION,90deg--> D(-40,60)
#
# Pivots (A,B,C) at B and (B,C,D) at C, each a 90 deg turn (delta = 9 cm).
# The middle hop B->C is a STOP_AT_LINE seek: it DEPARTS pivot B and APPROACHES
# pivot C. Because a seek is sensor-terminated, the departing over-travel of B is
# self-corrected on the crossing line and is NOT subtracted from the seek
# advance -- only the approaching trim of C reduces the post-detect roll. So the
# seek nets length - delta (NOT length - 2*delta): the no-double-trim contract.


_TC8_OUT_CM: float = 100.0
_TC8_NODES = {
    "A": (0.0, 0.0),
    "B": (60.0, 0.0),
    "C": (60.0, 60.0),
    "D": (60.0 - _TC8_OUT_CM, 60.0),
}


def _make_chained_seek_field():
    """Return a chained-pivot field with a sensor-terminated seek as the middle hop.

    ``A -> B`` BLIND (dead-reckon approach into pivot B), ``B -> C`` BLIND
    authored ``stop_at_line: "on"`` (a seek that departs B and approaches C),
    ``C -> D`` PROJECTION (dead-reckon departure from pivot C). Pivots
    ``(A, B, C)`` and ``(B, C, D)`` are each a 90 deg turn. PASSES ``validate``
    by construction (the two trims fit every edge; the seek hop carries only the
    approaching trim).
    """
    return _make_field(
        nodes=_TC8_NODES,
        line_segments=[],
        blind_edges=[("A", "B")],
        pivot_turns=[("A", "B", "C"), ("B", "C", "D")],
        start_node="A",
        start_heading_deg=0.0,    # facing along A->B (+x)
        edges=[
            {
                "type": "BLIND",
                "u": "B",
                "v": "C",
                "distance_cm": 60.0,
                "heading_deg": 90.0,
                "stop_at_line": "on",    # force the middle hop to seek
            },
            {
                "type": "PROJECTION",
                "u": "C",
                "v": "D",
                "distance_cm": _TC8_OUT_CM,
                "heading_deg": 180.0,
            },
        ],
    )


def test_tc8_chained_seek_no_double_trim_and_lands_on_goal():
    """TC8: a seek between two pivots carries one trim, and both pivots land clean.

    On the route ``A -> B -> C -> D`` the ``_pending_pivot -> _departing_pivot``
    latch resolves the pivot at B (approached by the A->B dead-reckon, departed
    by the B->C seek) and the pivot at C (approached by the B->C seek, departed
    by the C->D projection). The middle hop B->C is a STOP_AT_LINE seek:

    * The dead-reckon approach ``A -> B`` is trimmed to ``length - delta``
      (the ``_approaching_pivot`` of B).
    * The seek ``B -> C`` nets ``length - delta`` -- ONLY the approaching trim of
      C. The departing trim of B is self-corrected on the seek's crossing line
      and is NOT subtracted (no double-trim): a plain dead-reckon middle hop
      would net ``length - 2*delta``, so the seek payload being ``length - delta``
      is the load-bearing distinction.
    * The projection departure ``C -> D`` is trimmed to ``length - delta`` (the
      ``_departing_pivot`` of C).

    The point-model chassis lands ``delta`` short on each replayed leg; adding
    each pivot's physical translation -- pivot B's outbound over-travel ABSORBED
    by the seek (only its inbound leg replays), pivot C contributing both legs --
    recovers node ``D`` with ZERO residual.
    """
    field = _make_chained_seek_field()
    sim, chassis, _graph, next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_AUTORULE_TURN_DEG, _PIVOT_TRACK_CM)

    assert walk_next_hop(next_hop, "A", "D") == ["A", "B", "C", "D"]

    prims = sim.go_to("D")

    # The seek advance into pivot C (delta-short of the full 60 cm).
    seek_advance = _seek_advance_into_pivot(60.0, delta)

    # Per-turn dispatch contract: approach trimmed, seek single-trimmed, departure
    # trimmed -- never the double-trim a plain dead-reckon middle hop would take.
    assert prims == [
        (ACT_FORWARD, pytest.approx(60.0 - delta, abs=1e-9)),       # A->B approach @ B
        (ACT_TURN_REL, pytest.approx(_AUTORULE_TURN_DEG, abs=1e-9)),
        (ACT_STOP_AT_LINE, pytest.approx(seek_advance, abs=1e-9)),  # B->C seek: ONLY approach @ C
        (ACT_TURN_REL, pytest.approx(_AUTORULE_TURN_DEG, abs=1e-9)),
        (ACT_FOLLOW_DIST, pytest.approx(_TC8_OUT_CM - delta, abs=1e-9)),  # C->D depart @ C
    ]

    # No-double-trim, stated as a direct numeric guard: the seek nets length-delta,
    # strictly MORE than the length-2*delta a dead-reckon middle hop would net.
    assert seek_advance == pytest.approx(60.0 - delta, abs=1e-9)
    assert seek_advance > (60.0 - 2.0 * delta) + _POSITION_TOL_CM

    # Both pivots land clean: add each pivot's physical translation. Pivot B's
    # OUTBOUND over-travel is absorbed by the B->C seek re-anchoring on the line,
    # so only its inbound leg (A->B, heading 0) replays; pivot C contributes both
    # its inbound (B->C, heading 90) and outbound (C->D, heading 180) legs.
    px, py = chassis.x_cm, chassis.y_cm
    for heading_deg in (0.0,):              # pivot B: inbound only (seek absorbs outbound)
        px += delta * math.cos(math.radians(heading_deg))
        py += delta * math.sin(math.radians(heading_deg))
    for heading_deg in (90.0, 180.0):       # pivot C: inbound + outbound
        px += delta * math.cos(math.radians(heading_deg))
        py += delta * math.sin(math.radians(heading_deg))
    dx, dy = field.NODES["D"]
    assert math.hypot(px - dx, py - dy) < _POSITION_TOL_CM
    assert sim.localization.current_node() == "D"


def test_tc8_chained_seek_latch_carry_is_byte_identical_step_through():
    """TC8 (latch parity): go_to and step_through agree across the chained-seek path.

    The ``_pending_pivot -> _departing_pivot`` snapshot-before-overwrite latch is
    the single site threading the two pivots through the seek hop. Two
    independent sims drive the route; the emitted primitive streams must be
    byte-identical, proving the latch carry is deterministic across the seek
    (not order-dependent on which entry point consumes ``_walk_hops``).
    """
    field_go = _make_chained_seek_field()
    field_step = _make_chained_seek_field()
    sim_go, _cg, _gg, _ng = _make_pivot_sim(field_go)
    sim_step, _cs, _gs, _ns = _make_pivot_sim(field_step)

    go_prims = sim_go.go_to("D")
    step_prims = [prim for prim, _snap in sim_step.step_through("D")]

    # Genuinely multi-hop with two chained pivots latched across the seek.
    assert len(go_prims) >= 5
    assert go_prims == step_prims
    assert sim_go.localization.current_node() == "D"
    assert sim_step.localization.current_node() == "D"


# ===========================================================================
# TC9 -- a pure call-site setStopAtLine(ON) into a pivot gives the SAME landing
# ===========================================================================
#
# The offline sim's public walk API (go_to / step_through) carries no per-hop
# call-site stop-at-line verb -- it always passes StopAtLine.AUTO, matching the
# firmware's per-edge AUTO default. We therefore assert the call-site ON path at
# the resolver + seek-formula level: resolve_stop_at_line(StopAtLine.ON, ...)
# requests the seek with NO auto-rule and NO edge override, and the seek-branch
# advance formula then produces the SAME delta-short landing as TC7. This is the
# documented equivalence point -- the resolver is the single gate the seek
# branch consults, so a matching (seek, advance) pair proves call-site ON would
# drive the identical chassis geometry the auto-rule drives in TC7.


def test_tc9_call_site_on_requests_seek_with_no_rule_or_override():
    """TC9: call-site ``StopAtLine.ON`` forces the seek on a plain BLIND edge.

    ``resolve_stop_at_line`` precedence rung 1: a per-hop ``StopAtLine.ON`` beats
    every lower rung. With NO per-edge override (``StopAtLineMode.DEFAULT``), a
    non-STOP_AT_LINE edge mode (``PIPELINE_CONFIG.blind``), and the auto-rule
    INACTIVE, the resolver still requests the seek purely on the call flag -- and
    the AUTO control with the same inputs does NOT seek, so the ON flag is
    load-bearing.
    """
    edge_mode = PIPELINE_CONFIG.blind
    # The control input must be a non-seeking edge so the ON flag is the only
    # thing that can request the seek (guards a vacuous pass).
    assert edge_mode != BlindMode.STOP_AT_LINE

    seek_on, motion_on = resolve_stop_at_line(
        StopAtLine.ON,
        StopAtLineMode.DEFAULT,
        edge_mode,
        PIPELINE_CONFIG.blind,
        Direction.FRONT,
        LINE_BOARD_COUNT,
        False,    # auto_rule_active -- explicitly OFF
    )
    assert seek_on is True
    # The motion bundle is concrete (never STOP_AT_LINE / DEFAULT).
    assert motion_on not in (BlindMode.STOP_AT_LINE, BlindMode.DEFAULT)

    # Control: the SAME inputs under AUTO (no rule, no override) do NOT seek.
    seek_auto, _ = resolve_stop_at_line(
        StopAtLine.AUTO,
        StopAtLineMode.DEFAULT,
        edge_mode,
        PIPELINE_CONFIG.blind,
        Direction.FRONT,
        LINE_BOARD_COUNT,
        False,
    )
    assert seek_auto is False


def test_tc9_call_site_on_seek_into_pivot_matches_tc7_landing():
    """TC9: a call-site ON seek into a pivot yields the SAME delta-short advance as TC7.

    A call-site ``StopAtLine.ON`` seek into a pivot drives the identical FRONT
    seek branch the auto-rule drives in TC7. Since the sim's public API has no
    per-hop call flag, the equivalence is asserted at the seek-formula level:
    the seek-branch advance ``distance - (offset - overshoot)`` -- recomputed
    through the production :func:`resolve_sensor_overshoot_trim` -- equals
    ``distance - delta`` for a 90 deg pivot within budget, byte-identical to the
    advance TC7's auto-rule seek emits. The seek mechanism is direction- and
    request-source agnostic once the resolver returns ``seek=True``; TC7 already
    proves the auto-rule path drives this exact advance end-to-end.
    """
    delta = compute_pivot_compensation(_AUTORULE_TURN_DEG, _PIVOT_TRACK_CM)
    assert delta <= LINE_SENSOR_AXLE_OFFSET_CM

    # Confirm the call-site ON path reaches the FRONT seek branch.
    seek, _motion = resolve_stop_at_line(
        StopAtLine.ON,
        StopAtLineMode.DEFAULT,
        PIPELINE_CONFIG.blind,
        PIPELINE_CONFIG.blind,
        Direction.FRONT,
        LINE_BOARD_COUNT,
        False,
    )
    assert seek is True

    # The seek-branch advance for the same 60 cm incoming edge as TC7's fixture.
    call_site_advance = _seek_advance_into_pivot(_SI_IN_CM, delta)
    # Identical to the auto-rule TC7 advance (both derive from the same helper).
    autorule_advance = _seek_advance_into_pivot(_SI_IN_CM, delta)
    assert call_site_advance == pytest.approx(autorule_advance, abs=1e-12)
    # And both equal distance - delta (the clean, un-clamped 90 deg regime).
    assert call_site_advance == pytest.approx(_SI_IN_CM - delta, abs=1e-9)


# ===========================================================================
# TC13 -- firmware <-> sim parity grid for resolve_sensor_overshoot_trim
# ===========================================================================
#
# resolve_sensor_overshoot_trim is the Python twin of the C++
# PivotTrimPolicy.h::resolveSensorOvershootTrim. The C++ is not callable from
# Python, so parity is enforced against (a) the GOLDEN outputs the C++ host test
# test_sensor_overshoot_policy.cpp asserts (TC1-TC4) and (b) a broader grid of
# (offset, trim, floor) triples checked against the closed form the C++ computes,
# max(offset - trim, floor) with clamped = (offset - trim) < floor (strict <).


# The four golden rows from WRO2026/test/test_sensor_overshoot_policy.cpp
# (offset, trim, floor) -> (expected_overshoot, expected_clamped). The C++ host
# test is the normative cross-language source; these MUST stay in lockstep.
_CPP_GOLDEN_ROWS = [
    # TC1: normal trim (delta < offset) -> offset - delta, not clamped.
    ((12.02, 9.0, 0.5), (3.02, False)),
    # TC2: over-trim (delta > offset) -> floor, clamped.
    ((12.02, 15.588, 0.5), (0.5, True)),
    # TC3: no trim (delta == 0) -> offset, not clamped.
    ((12.02, 0.0, 0.5), (12.02, False)),
    # TC4: strict-< boundary (delta == offset - floor => effective == floor) ->
    # floor, NOT clamped (clamp fires only when effective is STRICTLY < floor).
    ((12.02, 12.02 - 0.5, 0.5), (0.5, False)),
]


@pytest.mark.parametrize(
    "inputs, expected",
    _CPP_GOLDEN_ROWS,
    ids=["TC1_normal", "TC2_over_trim", "TC3_no_trim", "TC4_strict_boundary"],
)
def test_tc13_sensor_overshoot_trim_matches_cpp_golden(inputs, expected):
    """TC13: resolve_sensor_overshoot_trim matches the C++ host-test golden rows.

    The Python twin must return the SAME ``(overshoot_cm, clamped)`` the C++
    ``resolveSensorOvershootTrim`` asserts in
    ``WRO2026/test/test_sensor_overshoot_policy.cpp`` (TC1-TC4). A divergence here
    means the offline sim would command a different post-detect axle roll than
    the robot, breaking the SENSOR-path parity the seek branch relies on.
    """
    offset, trim, floor = inputs
    expected_overshoot, expected_clamped = expected

    overshoot_cm, clamped = resolve_sensor_overshoot_trim(offset, trim, floor)

    assert overshoot_cm == pytest.approx(expected_overshoot, abs=1e-4)
    assert clamped is expected_clamped


@pytest.mark.parametrize(
    "offset, trim, floor",
    [
        # Clean regime: trim < offset - floor -> offset - trim, not clamped.
        (12.02, 0.0, 0.5),
        (12.02, 3.0, 0.5),
        (12.02, 9.0, 0.5),
        (7.68, 0.0, 0.5),       # REAR axle offset, no trim.
        (7.68, 3.0, 0.5),       # REAR axle offset, mid trim.
        # Strict-< clamp boundary: effective exactly == floor -> floor, NOT clamped.
        (12.02, 12.02 - 0.5, 0.5),
        (7.68, 7.68 - 0.5, 0.5),
        (10.0, 9.5, 0.5),
        # Over-trim: effective < floor -> floor, clamped.
        (12.02, 12.0, 0.5),
        (12.02, 15.588, 0.5),
        (7.68, 8.0, 0.5),
        (5.0, 100.0, 0.5),
        # A different floor value to exercise the parameter (not hard-coded 0.5).
        (10.0, 4.0, 1.0),       # effective 6.0 > floor 1.0 -> not clamped.
        (10.0, 9.5, 1.0),       # effective 0.5 < floor 1.0 -> clamped.
        (10.0, 9.0, 1.0),       # effective 1.0 == floor 1.0 -> NOT clamped (strict <).
    ],
)
def test_tc13_sensor_overshoot_trim_matches_cpp_closed_form(offset, trim, floor):
    """TC13 (broad grid): the twin matches the C++ closed form across (offset, trim, floor).

    The C++ ``resolveSensorOvershootTrim`` computes
    ``effective = axleOffsetCm - approachingTrimCm``,
    ``overshootCm = max(effective, minDriveCm)``, and ``clamped = effective <
    minDriveCm`` (strict ``<`` -- the floor-equal boundary is NOT clamped). This
    grid asserts the Python twin reproduces that exact closed form -- including
    the strict-``<`` clamp boundary and a non-default floor -- so the two stay in
    lockstep beyond just the four hand-picked golden rows.
    """
    overshoot_cm, clamped = resolve_sensor_overshoot_trim(offset, trim, floor)

    effective = offset - trim
    expected_overshoot = max(effective, floor)
    expected_clamped = effective < floor

    assert overshoot_cm == pytest.approx(expected_overshoot, abs=1e-9)
    assert clamped is expected_clamped


def test_tc13_sensor_overshoot_trim_no_trim_is_full_offset():
    """TC13 (AC2 identity): with no approaching pivot the overshoot is the full offset.

    A non-pivot sensor hop passes ``approaching_trim_cm == 0.0`` and must roll the
    FULL axle offset (no reduction, no clamp), so every non-pivot hop nets exactly
    its edge length -- the invariant that keeps drift == 0 on non-pivot routes.
    """
    for offset in (LINE_SENSOR_AXLE_OFFSET_CM, 7.68, 5.0):
        overshoot_cm, clamped = resolve_sensor_overshoot_trim(
            offset, 0.0, PIVOT_TRIM_MIN_DRIVE_CM
        )
        assert overshoot_cm == pytest.approx(offset, abs=1e-9)
        assert clamped is False


# ===========================================================================
# Shared resolve_pivot_trim core (finding #5) -- parity with the C++ twin
# ===========================================================================


@pytest.mark.parametrize(
    "raw, departing, approaching, floor",
    [
        (60.0, 0.0, 0.0, 0.5),              # no trim -> raw, not clamped
        (60.0, 9.0, 9.0, 0.5),              # chained: 60 - 9 - 9 = 42, not clamped
        (12.02, 0.0, 9.0, 0.5),             # sensor-wrapper case: 12.02 - 9 = 3.02
        (12.02, 0.0, 12.02 - 0.5, 0.5),     # strict-< boundary -> floor, NOT clamped
        (12.02, 0.0, 12.0, 0.5),            # over-trim -> floor, clamped
        (10.0, 6.0, 6.0, 0.5),              # 10 - 12 = -2 -> floor, clamped
        (10.0, 4.0, 5.0, 1.0),              # 10 - 9 = 1.0 == floor -> NOT clamped (strict <)
    ],
)
def test_resolve_pivot_trim_matches_cpp_closed_form(raw, departing, approaching, floor):
    """The shared core matches the C++ ``resolvePivotTrim`` closed form (finding #5).

    ``resolve_pivot_trim`` is the Python twin of
    ``PivotTrimPolicy.h::resolvePivotTrim`` that BOTH
    :meth:`PathTrackerSim._pivot_trim_distance` and
    :func:`resolve_sensor_overshoot_trim` route through. Assert it reproduces the
    C++ closed form across ``(raw, departing, approaching, floor)``:
    ``effective = raw - departing - approaching``,
    ``drive = max(effective, floor)``, ``clamped = effective < floor`` (strict
    ``<`` -- the floor-equal boundary is NOT clamped).
    """
    drive_cm, effective_cm, clamped = resolve_pivot_trim(
        raw, departing, approaching, floor
    )
    expected_effective = raw - departing - approaching
    assert effective_cm == pytest.approx(expected_effective, abs=1e-9)
    assert drive_cm == pytest.approx(max(expected_effective, floor), abs=1e-9)
    assert clamped is (expected_effective < floor)


def test_resolve_sensor_overshoot_trim_delegates_to_core():
    """The sensor wrapper is ``resolve_pivot_trim`` with departing fixed at 0 (finding #5).

    Proves the SENSOR path and the DISTANCE path physically share one core (the
    C++ ``resolveSensorOvershootTrim`` / ``resolvePivotTrim`` split), so they can
    never drift: for every input the wrapper returns exactly the core's
    ``(drive_cm, clamped)`` with the departing slot hard-zeroed.
    """
    for offset, trim, floor in [
        (12.02, 0.0, 0.5),
        (12.02, 9.0, 0.5),
        (12.02, 15.588, 0.5),
        (7.68, 3.0, 0.5),
    ]:
        overshoot_cm, clamped = resolve_sensor_overshoot_trim(offset, trim, floor)
        drive_cm, _effective_cm, core_clamped = resolve_pivot_trim(
            offset, 0.0, trim, floor
        )
        assert overshoot_cm == pytest.approx(drive_cm, abs=1e-12)
        assert clamped is core_clamped


# ===========================================================================
# Degenerate short sensor edge (finding #3) -- the seek never drives backward
# ===========================================================================


def _make_short_seek_into_pivot_field():
    """Return a field whose A->B seek edge is SHORTER than the approaching delta.

    ``A(0,0) --BLIND,stop_at_line "on",5 cm--> B(5,0) --BLIND,90 deg--> C(5,100)``.
    The pivot ``(A, B, C)`` is a 90 deg turn (``delta = 9`` cm), and ``delta`` is
    within the 12.02 cm FRONT overshoot budget the validator gates on -- yet the
    incoming seek edge is only 5 cm. The validator bounds ``delta`` against the
    overshoot budget, NOT the edge length (the edge-length floor is gated on the
    "distance" regime), so this degenerate geometry validates clean and reaches
    the sim. Real edges are never this short; this exists only to drive the seek
    branch's ``max(0.0, ...)`` clamp.
    """
    return _make_field(
        nodes={"A": (0.0, 0.0), "B": (5.0, 0.0), "C": (5.0, 100.0)},
        line_segments=[],
        blind_edges=[("B", "C")],
        pivot_turns=[("A", "B", "C")],
        start_node="A",
        start_heading_deg=0.0,    # facing along A->B (+x)
        edges=[
            {
                "type": "BLIND",
                "u": "A",
                "v": "B",
                "distance_cm": 5.0,
                "heading_deg": 0.0,
                "stop_at_line": "on",    # force the short hop to seek
            }
        ],
    )


def test_short_sensor_edge_seek_never_drives_backward():
    """Finding #3: a seek edge shorter than delta clamps the advance to 0, never backward.

    The seek branch's net-displacement model ``distance - (offset - overshoot)``
    would go negative here (5 - (12.02 - 3.02) = -4), modeling a backward drive the
    firmware -- sensor-terminated, it physically finds the line -- never makes. The
    ``max(0.0, ...)`` clamp (mirroring the LINE follow's ``follow_cm = max(0.0,
    ...)`` guard) pins the advance at 0 instead.
    """
    field = _make_short_seek_into_pivot_field()
    sim, _chassis, _graph, next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_AUTORULE_TURN_DEG, _PIVOT_TRACK_CM)
    # delta is within the FRONT budget (so it validates), but the edge is shorter.
    assert delta <= LINE_SENSOR_AXLE_OFFSET_CM
    assert 5.0 < delta

    assert walk_next_hop(next_hop, "A", "C") == ["A", "B", "C"]

    seek_prim, _snap = _seek_snapshot(sim, "C")
    # Clamped to 0 -- the unclamped model would be negative (5 - 9 = -4).
    assert seek_prim[1] == pytest.approx(_seek_advance_into_pivot(5.0, delta), abs=1e-9)
    assert seek_prim[1] == pytest.approx(0.0, abs=1e-9)
    assert seek_prim[1] >= 0.0
