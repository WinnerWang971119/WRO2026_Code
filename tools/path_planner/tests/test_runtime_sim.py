"""End-to-end tests for the PathTracker runtime simulator (PR3).

PR3 (post T9) emission contract: every EDGE_LINE traversal into a junction
(line) node emits the canonical ``(ACT_FOLLOW_INT, None)`` followed by a
runtime-only ``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)`` step that
mirrors the C++ LINE_OVERSHOOT FSM transition. A LINE hop into a ``projection``
node (issue #64) emits ``(ACT_FOLLOW_DIST, distance_cm)`` and advances
Localization via ``on_dead_reckoned_advance`` (no overshoot -- the encoder gate
lands the chassis on the node). EDGE_BLIND traversals still emit
``(ACT_FORWARD, distance_cm)``.

These tests cover the runtime-sim equivalence proof (sim primitive
sequence == compile_path primitive sequence interleaved with the
sim-only LINE_OVERSHOOT steps), per-edge mode override resolution, and
the LINE_OVERSHOOT + PROJECTION_FOLLOW position contracts.
"""

from __future__ import annotations

import copy
import csv
import math
import pathlib
import random
import warnings
from types import SimpleNamespace
from typing import List

import networkx as nx
import pytest

from tools.path_planner.tests.fixtures import toy_field_data
from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
    Primitive,
    compile_path,
)
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    build_graph,
)
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
    DirectionMode,
    SpeedTier,
    StopAtLineMode,
)
from tools.path_planner.planner import (
    BlindDecayParams,
    compute_all_pairs_next_hop,
    compute_pivot_compensation,
    parse_track_cm_from_config,
)
from tools.path_planner.sim import (
    LocalizationSim,
    PathTrackerSim,
    UnreachableGoalError,
    VirtualChassis,
    walk_next_hop,
)
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.runtime import (
    ACT_BLIND_PASS,
    ACT_BLIND_REVERSE,
    ACT_FOLLOW_DIST_PASS,
    ACT_HEADING_RESET,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    ACT_LINE_REVERSE,
    ACT_PROJECTION_REVERSE,
    ACT_REANCHOR,
    ACT_RETURN_FORWARD,
    ACT_RETURN_STOP_AT_LINE,
    ACT_STOP_AT_LINE,
    ACT_STOP_AT_LINE_REVERSE,
    ACT_WALL_DEPART,
    ACT_WALL_STOP,
    Direction,
    DispatchPrimitive,
    EdgeDispatchQuery,
    LINE_HEADING_RESET_ON_JUNCTION,
    LINE_HEADING_RESET_ON_PROJECTION,
    LINE_SENSOR_AXLE_OFFSET_CM,
    LINE_SENSOR_AXLE_OFFSET_REAR_CM,
    OFF_MAP_GATE,
    OFFSET_ROTATION_DEADBAND_DEG,
    OFFSET_TRANSLATION_DEADBAND_CM,
    Offset2D,
    OffMapFaultError,
    OffMapGate,
    RETURN_DISTANCE_CAP_CM,
    RETURN_TRIGGER_THRESHOLD_CM,
    ReanchorKind,
    ReturnAction,
    StopAtLine,
    WALL_STOP_BACK_OFF_CM,
    resolve_blind_mode,
    resolve_direction,
    resolve_edge_dispatch,
    resolve_return_action,
    resolve_speed,
    resolve_stop_at_line,
)
from tools.path_planner.sim.parity import (
    ParityCase,
    ParitySchemaError,
    TraceRecord,
    integrate_pose,
    normalize_sim_stream,
)
from tools.path_planner.tests.fixtures import bidirectional_field_data
from tools.path_planner.tests.fixtures import collinear_4node
from tools.path_planner.tests.fixtures import pivot_field_data

_POSITION_TOL_CM: float = 0.1

# The set of field-data attributes the test clone needs. ``EDGES`` is
# intentionally OMITTED so the cloned field stays on the pure
# LINE_SEGMENTS / BLIND_EDGES graph; the demo EDGES entries pull in a
# pivot tag on C->E that the planner refuses to compensate when its
# destination has multiple out-edges. The dedicated TC10 fixture below
# rebuilds a minimal synthetic field containing the PROJECTION edge it
# needs to exercise without inheriting that pivot.
_FIELD_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    # Blind-decay knobs (replace the old COST_BLIND_MULTIPLIER; PR3 / T22).
    "BLIND_DECAY_ENABLED",
    "BLIND_DECAY_K",
    "BLIND_DECAY_EXPONENT",
    "BLIND_DECAY_D_REF",
    "START_NODE",
    "START_HEADING_DEG",
    "FIELD_MM",
    "MAP_PNG",
)


def _clone_field_data() -> SimpleNamespace:
    """Return a SimpleNamespace mirror of the toy field MINUS the EDGES list.

    Stripping EDGES keeps the demo PROJECTION edges out of the baseline test
    graph. New tests that need to exercise PROJECTION or per-edge overrides
    build their own minimal fixtures rather than mutating the shared toy field.

    ``PIVOT_TURNS`` is added empty: the toy field declares no pivots, but the sim
    constructor resolves ``field_data.PIVOT_TURNS`` against its graph on
    construction (ADR-036), so the attribute must exist. The pivot-bearing
    fixtures live in :mod:`tools.path_planner.tests.fixtures.pivot_field_data`.
    """
    snapshot = {attr: copy.deepcopy(getattr(toy_field_data, attr)) for attr in _FIELD_ATTRS}
    snapshot["PIVOT_TURNS"] = []
    return SimpleNamespace(**snapshot)


@pytest.fixture(scope="module")
def toy_field():
    """Cached clone of the toy field with EDGES stripped (see _clone_field_data)."""
    return _clone_field_data()


@pytest.fixture
def toy_graph_and_table(toy_field):
    # The clone strips EDGES, which orphans node P1 (P1 is only reached
    # via the demo PROJECTION edge in EDGES). build_graph warns on every
    # orphan node by design; suppress it here because the orphan is an
    # intentional consequence of the fixture's narrow scope.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph = build_graph(toy_field)
        next_hop = compute_all_pairs_next_hop(
            graph, BlindDecayParams.from_field_data(toy_field)
        )
    return graph, next_hop


@pytest.fixture
def fresh_sim(toy_field, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field)
    sim = PathTrackerSim(toy_field, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim


def _interleave_overshoot(primitives: List[Primitive]) -> List[Primitive]:
    """Insert the per-hop ``(ACT_LINE_OVERSHOOT, offset)`` after each LINE primitive.

    The runtime sim (post-T9) emits a LINE_OVERSHOOT step after every
    EDGE_LINE traversal so the wheel axle (not the line sensor) lands on
    the node. ``compile_path`` deliberately does NOT emit this step (the
    C++ runtime owns it as an FSM state), so to compare the sim's output
    against ``compile_path``'s output we must re-add the LINE_OVERSHOOT
    steps in the same positions the sim emits them.

    The overshoot advance is the axle offset of the BOARD that LED the hop,
    so the helper picks the offset per primitive (mirroring the sim, which
    reads ``LINE_SENSOR_AXLE_OFFSET_REAR_CM`` for a REAR hop and
    ``LINE_SENSOR_AXLE_OFFSET_CM`` for a FRONT hop):

    * FRONT LINE hop -> ``ACT_FOLLOW_INT`` -> overshoot ``LINE_SENSOR_AXLE_OFFSET_CM``.
    * REAR LINE hop  -> ``ACT_LINE_REVERSE`` -> overshoot ``LINE_SENSOR_AXLE_OFFSET_REAR_CM``.

    Post-T12, EDGE_LINE traversals on REAR hops emit ``ACT_LINE_REVERSE``
    instead of ``ACT_FOLLOW_INT`` but still trigger the LINE_OVERSHOOT
    FSM transition; the helper therefore matches either primitive name and
    emits that hop's own board offset (ADR-035 reconciliation: FRONT 12.02
    cm, REAR 7.68 cm -- previously both stale 5.0, which masked the
    per-board distinction and only worked while FRONT == REAR).

    Parity limitation — pass-through fixtures only:

    The helper unconditionally inserts ``ACT_LINE_OVERSHOOT``. The sim,
    however, emits ``ACT_LINE_PASS`` at intermediate intersections where
    the body-heading delta is below ``NAV_TURN_TOLERANCE_DEG``
    (ADR-023 collinear pass-through). ``compile_path`` is pass-through
    unaware — it never emits ``ACT_LINE_PASS`` — so any fixture that
    triggers pass-through in the sim cannot use this helper for parity
    comparison. TC18 fixtures are constructed to avoid pass-through by
    construction (90° turns or 2-node paths). If a future TC18-style
    fixture introduces collinear intermediate hops, the parity assertion
    will fail with a confusing diff; the fix is fixture redesign, not a
    helper change. See the assertion below for the loud-failure mode.
    """
    assert ACT_LINE_PASS not in (p[0] for p in primitives), (
        "_interleave_overshoot received a primitive sequence containing "
        "ACT_LINE_PASS. compile_path never emits this primitive, so the "
        "input could only be a sim-emitted sequence — but the helper is "
        "designed for compile_path output. The fixture cannot be used "
        "for compile_path↔sim parity until it is redesigned to avoid "
        "pass-through (e.g. add a turn at every intermediate node, or "
        "drop intermediate nodes entirely)."
    )
    out: List[Primitive] = []
    for prim in primitives:
        out.append(prim)
        if prim[0] == ACT_FOLLOW_INT:
            # FRONT line hop: front board led, so roll the FRONT axle offset.
            out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM))
        elif prim[0] == ACT_LINE_REVERSE:
            # REAR line hop: rear board led, so roll the REAR axle offset.
            out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_REAR_CM))
    return out


def _line_count(primitives: List[Primitive]) -> int:
    """Return the number of ``(ACT_FOLLOW_INT, None)`` primitives in @p primitives."""
    return sum(1 for action, _ in primitives if action == ACT_FOLLOW_INT)


# -- baseline equivalence + reachability (post-T9 rebaselined) ---------------


def test_goto_B_matches_compile_path(toy_field, fresh_sim, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    primitives = fresh_sim.go_to("B")
    # Walk from the toy field's START node (issue #64: a blind ``S`` spawn, not
    # ``A``) so the compiled expectation tracks the same hops the sim drives.
    walked = walk_next_hop(next_hop, toy_field.START_NODE, "B")
    expected = compile_path(walked, toy_field.START_HEADING_DEG, graph, toy_field)
    assert primitives == _interleave_overshoot(expected)
    bx, by = toy_field.NODES["B"]
    # Position contract: the LINE_OVERSHOOT step rolls only the final
    # sensor-to-axle offset, so the wheel axle lands ON the node (net advance
    # == edge length), matching the firmware re-anchor
    # (PathTracker.cpp LINE_FOLLOW->LINE_OVERSHOOT). The chassis ends at the
    # node coordinates with no residual offset.
    assert math.hypot(fresh_sim.chassis.x_cm - bx, fresh_sim.chassis.y_cm - by) < _POSITION_TOL_CM
    assert fresh_sim.localization.current_node() == "B"


def test_goto_C_via_path(toy_field, fresh_sim, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    primitives = fresh_sim.go_to("C")
    walked = walk_next_hop(next_hop, toy_field.START_NODE, "C")
    expected = compile_path(walked, toy_field.START_HEADING_DEG, graph, toy_field)
    assert primitives == _interleave_overshoot(expected)
    assert fresh_sim.localization.current_node() == "C"


def test_chained_goto_lands_at_G(toy_field, fresh_sim):
    fresh_sim.go_to("B")
    fresh_sim.go_to("G")
    assert fresh_sim.localization.current_node() == "G"


def test_goto_self_is_noop(fresh_sim):
    initial_x = fresh_sim.chassis.x_cm
    initial_y = fresh_sim.chassis.y_cm
    initial_heading = fresh_sim.chassis.heading_deg
    primitives = fresh_sim.go_to(fresh_sim.localization.current_node())
    assert primitives == []
    assert fresh_sim.chassis.x_cm == initial_x
    assert fresh_sim.chassis.y_cm == initial_y
    assert fresh_sim.chassis.heading_deg == initial_heading


def test_unreachable_goal_raises():
    """Removing the blind edge makes G unreachable from A."""
    fd = _clone_field_data()
    fd.BLIND_EDGES = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph = build_graph(fd)
        next_hop = compute_all_pairs_next_hop(
            graph, BlindDecayParams.from_field_data(fd)
        )
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    with pytest.raises(UnreachableGoalError) as excinfo:
        sim.go_to("G")
    err = excinfo.value
    assert err.from_node == fd.START_NODE
    assert err.goal_node == "G"


def test_property_random_walks(toy_field, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    rng = random.Random(0xA1B2C3)
    reachable_targets = [
        n for n in toy_field.NODES
        if next_hop[toy_field.START_NODE][n] is not None
    ]

    for _ in range(30):
        chassis = VirtualChassis.from_field_data(toy_field)
        sim = PathTrackerSim(toy_field, graph, next_hop, chassis)
        sim.localization.zero_heading_from_current()
        n_goals = rng.randint(1, 3)
        all_emitted: List[Primitive] = []
        all_expected: List[Primitive] = []

        for _ in range(n_goals):
            goal = rng.choice(reachable_targets)
            current_before = sim.localization.current_node()
            if next_hop[current_before][goal] is None and current_before != goal:
                continue
            heading_before = chassis.heading_deg
            walked = walk_next_hop(next_hop, current_before, goal)
            expected = _interleave_overshoot(
                compile_path(walked, heading_before, graph, toy_field)
            )
            emitted = sim.go_to(goal)
            assert emitted == expected
            all_emitted.extend(emitted)
            all_expected.extend(expected)
        assert all_emitted == all_expected


def test_goto_unknown_node_raises(fresh_sim):
    with pytest.raises(KeyError):
        fresh_sim.go_to("ZZZ")


def test_localization_starts_unzeroed(toy_field):
    loc = LocalizationSim(toy_field)
    assert loc.is_zeroed() is False
    assert loc.current_node() == toy_field.START_NODE


def test_declare_position_sets_node_clears_offset_preserves_heading(toy_field):
    """T7: a valid ``declare_position`` changes the node, clears the offset,
    and preserves whatever heading calibration (``is_zeroed``) was in force --
    mirrors ``LocalizationClass::declarePosition`` leaving ``_yawOffsetDeg``
    untouched (Localization.cpp:190-201)."""
    loc = LocalizationSim(toy_field)
    loc.zero_heading_from_current()
    assert loc.is_zeroed() is True

    # Grow a non-zero offset via the same accumulate path other re-anchor
    # tests use, so the assertion actually exercises the clear.
    loc.set_nav_active(False)
    loc.accumulate_operator_move(10.0, 0.0)
    assert loc.get_offset_from_node().magnitude_cm() > 0.0

    loc.declare_position("B")

    assert loc.current_node() == "B"
    offset = loc.get_offset_from_node()
    assert offset.x == 0.0
    assert offset.y == 0.0
    assert offset.theta_deg == 0.0
    # Heading calibration is a separate concern from the offset -- declarePosition
    # must not touch it either way (verified true here; the negative-path test
    # below re-confirms it is untouched when a node is unknown too).
    assert loc.is_zeroed() is True


def test_declare_position_unknown_node_warns_and_is_noop(toy_field):
    """T7 / TC17 negative-path seed: an unknown node name warns (UserWarning,
    mirroring the loader's strict-field warn-and-skip pattern -- see
    ``ctps/trace_loader.py``) and leaves current_node + offset + heading
    calibration completely unchanged -- not a hard crash."""
    loc = LocalizationSim(toy_field)
    loc.zero_heading_from_current()
    loc.set_nav_active(False)
    loc.accumulate_operator_move(10.0, 0.0)

    node_before = loc.current_node()
    offset_before = loc.get_offset_from_node()

    with pytest.warns(UserWarning, match="unknown node"):
        loc.declare_position("ZZZ")

    assert loc.current_node() == node_before
    offset_after = loc.get_offset_from_node()
    assert offset_after.x == offset_before.x
    assert offset_after.y == offset_before.y
    assert offset_after.theta_deg == offset_before.theta_deg
    assert loc.is_zeroed() is True


def test_goto_before_zero_raises(toy_field, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field)
    sim = PathTrackerSim(toy_field, graph, next_hop, chassis)
    with pytest.raises(RuntimeError) as excinfo:
        sim.go_to("C")
    assert "before" in str(excinfo.value).lower()


def test_every_reachable_goal_lands_at_node(toy_field, toy_graph_and_table):
    """Every reachable goal lands the chassis ON that goal node.

    The LINE_OVERSHOOT step rolls only the final sensor-to-axle offset, so the
    wheel axle ends on the node (net advance == edge length), matching the
    firmware re-anchor. With no raw drive injected between hops the chassis
    therefore coincides with the goal node within ``_POSITION_TOL_CM`` AND
    Localization.current_node() advances to the goal.
    """
    graph, next_hop = toy_graph_and_table
    targets = [
        n for n in toy_field.NODES
        if next_hop[toy_field.START_NODE][n] is not None
        and n != toy_field.START_NODE
    ]
    for goal in targets:
        chassis = VirtualChassis.from_field_data(toy_field)
        sim = PathTrackerSim(toy_field, graph, next_hop, chassis)
        sim.localization.zero_heading_from_current()
        sim.go_to(goal)
        assert sim.localization.current_node() == goal
        gx, gy = toy_field.NODES[goal]
        assert math.hypot(chassis.x_cm - gx, chassis.y_cm - gy) < _POSITION_TOL_CM


def test_walk_next_hop_self(toy_graph_and_table):
    _, next_hop = toy_graph_and_table
    assert walk_next_hop(next_hop, "A", "A") == ["A"]


# ---------------------------------------------------------------------------
# Test-fixture builders for TC9 / TC10 (synthetic minimal graphs)
# ---------------------------------------------------------------------------


def _minimal_field(nodes: dict, start_node: str, start_heading_deg: float):
    """Return a SimpleNamespace usable in place of field_data for the sim.

    The returned object exposes the attribute surface the sim consumes
    (``NODES`` + ``START_NODE`` + ``START_HEADING_DEG`` + an empty
    ``PIVOT_TURNS``). :class:`PathTrackerSim` resolves ``field_data.PIVOT_TURNS``
    against its graph on construction (ADR-036), so the attribute must exist even
    on a pivot-free synthetic field; these minimal graphs declare no pivots.
    """
    return SimpleNamespace(
        NODES=nodes,
        START_NODE=start_node,
        START_HEADING_DEG=start_heading_deg,
        PIVOT_TURNS=[],
    )


def _single_edge_graph(
    u: str,
    v: str,
    edge_type: str,
    distance_cm: float,
    heading_deg: float,
    *,
    u_type: str = "line",
    v_type: str = "line",
) -> nx.DiGraph:
    """Build a 2-node DiGraph with a single directed edge.

    Skips the validator + builder path entirely so the test fixture stays
    independent of field_data schema rules. Adds the reverse edge so
    Dijkstra's all-pairs sweep doesn't trip over a dead-end target.

    ``u_type`` / ``v_type`` (issue #64) set each node's ``type`` attribute so the
    sim's LOCAL ``is_projection_node`` read resolves correctly -- a LINE edge into
    a ``v_type="projection"`` node is DISTANCE-terminated (FOLLOW_DIST).
    """
    g = nx.DiGraph()
    g.add_node(u, type=u_type)
    g.add_node(v, type=v_type)
    g.add_edge(u, v, type=edge_type, distance_cm=float(distance_cm), heading_deg=float(heading_deg))
    g.add_edge(
        v, u,
        type=edge_type,
        distance_cm=float(distance_cm),
        heading_deg=(float(heading_deg) + 180.0) % 360.0,
    )
    return g


def _next_hop_two_node(u: str, v: str) -> dict:
    """Hand-rolled next-hop table for a 2-node graph (avoids planner pivot path)."""
    return {
        u: {u: u, v: v},
        v: {u: u, v: v},
    }


# ---------------------------------------------------------------------------
# TC9 — runtime-sim LINE_OVERSHOOT
# ---------------------------------------------------------------------------


def test_sim_line_edge_emits_overshoot_step():
    """PathTrackerSim.go_to emits an ACT_LINE_OVERSHOOT step after each EDGE_LINE.

    A 30 cm LINE edge A->B heading 0 must produce, in order:
        (ACT_FOLLOW_INT, None)
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)
    and leave the chassis x at 30 cm — the wheel axle lands ON node B (the
    overshoot rolls only the final offset; net advance == the 30 cm edge,
    heading 0 so motion is along +x).
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (30.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, 30.0, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM),
    ]
    assert chassis.x_cm == pytest.approx(30.0, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_sim_line_edge_step_through_yields_overshoot_step():
    """PathTrackerSim.step_through yields the LINE_OVERSHOOT step after FOLLOW_INT."""
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (30.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, 30.0, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    steps = list(sim.step_through("B"))
    prims = [prim for prim, _snap in steps]

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM),
    ]
    assert sim.localization.current_node() == "B"


# ---------------------------------------------------------------------------
# TC10 — runtime-sim PROJECTION_FOLLOW
# ---------------------------------------------------------------------------


def test_sim_line_into_projection_node_advances_position_and_localization():
    """PathTrackerSim handles a LINE hop into a projection node via follow_line_to_distance + on_dead_reckoned_advance.

    Issue #64: a LINE edge into a ``projection`` node is DISTANCE-terminated. A
    12 cm LINE hop A->B (B a projection node) heading 0 must produce exactly
    ``[(ACT_FOLLOW_DIST, 12.0)]`` (no LINE_OVERSHOOT step — the encoder gate
    already lands the chassis on the node), advance the chassis x by exactly
    12 cm, and advance Localization.current_node() to ``B``.
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (12.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, 12.0, 0.0, v_type="projection")
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FOLLOW_DIST, 12.0)]
    assert chassis.x_cm == pytest.approx(12.0, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_sim_line_into_projection_node_step_through_emits_follow_dist_only():
    """step_through over a LINE hop into a projection node yields only the FOLLOW_DIST step (no overshoot)."""
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (12.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, 12.0, 0.0, v_type="projection")
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    steps = list(sim.step_through("B"))
    prims = [prim for prim, _snap in steps]

    assert prims == [(ACT_FOLLOW_DIST, 12.0)]
    assert sim.localization.current_node() == "B"


def test_sim_blind_edge_emits_forward_no_overshoot():
    """EDGE_BLIND traversal emits ACT_FORWARD and no LINE_OVERSHOOT step."""
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (20.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_BLIND, 20.0, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FORWARD, 20.0)]
    assert chassis.x_cm == pytest.approx(20.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


# ---------------------------------------------------------------------------
# TC11 — per-edge override resolution (SpeedTier / BlindMode / TurnMode)
# (issue #64 retired ProjectionMode along with the projection EDGE type; ADR-064
# renamed the LINE/BLIND-shared speed axis from LineMode -> SpeedTier, keyed on
# the ``speed`` edge attribute, and resolve_blind_mode now reads ``mode``.)
# ---------------------------------------------------------------------------


def test_resolve_speed_default_falls_back_to_pipeline_config():
    """resolve_speed: missing/DEFAULT override -> PIPELINE_CONFIG.speed, concrete -> as-is."""
    # Edge with no speed attr -> fallback.
    edge_missing: dict = {}
    assert resolve_speed(edge_missing) == PIPELINE_CONFIG.speed

    # Edge with DEFAULT sentinel -> fallback.
    edge_default = {"speed": SpeedTier.DEFAULT}
    assert resolve_speed(edge_default) == PIPELINE_CONFIG.speed

    # Edge with concrete value -> returned as-is.
    edge_fast = {"speed": SpeedTier.FAST}
    assert resolve_speed(edge_fast) == SpeedTier.FAST

    # Raw int value -> coerced to enum, returned as-is.
    edge_raw_int = {"speed": int(SpeedTier.SLOW)}
    assert resolve_speed(edge_raw_int) == SpeedTier.SLOW


def test_resolve_blind_mode_default_falls_back_to_pipeline_config():
    """resolve_blind_mode: missing/DEFAULT override -> PIPELINE_CONFIG.blind, concrete -> as-is."""
    edge_missing: dict = {}
    assert resolve_blind_mode(edge_missing) == PIPELINE_CONFIG.blind

    edge_default = {"mode": BlindMode.DEFAULT}
    assert resolve_blind_mode(edge_default) == PIPELINE_CONFIG.blind

    edge_gyro = {"mode": BlindMode.GYRO_TSHAPE}
    assert resolve_blind_mode(edge_gyro) == BlindMode.GYRO_TSHAPE

    edge_raw = {"mode": BlindMode.RAW_FLAT}
    assert resolve_blind_mode(edge_raw) == BlindMode.RAW_FLAT


def test_sim_passthrough_resolve_helpers_match_module_level(toy_field, toy_graph_and_table):
    """PathTrackerSim.resolve_*_mode instance methods mirror the module-level helpers.

    ADR-036 removed the per-edge ``pre_turn_mode`` override (and its
    ``resolve_pre_turn_mode`` helper): the pivot wheel is now resolved per turn
    from the ``PivotTurnRecord`` table, not an edge field. Issue #64 removed the
    per-edge ``resolve_projection_mode`` (projection is a node type now). Only the
    surviving per-edge mode passthroughs (speed / blind) are checked here.
    """
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field)
    sim = PathTrackerSim(toy_field, graph, next_hop, chassis)

    edge_line_fast = {"speed": SpeedTier.FAST}
    edge_blind_gyro = {"mode": BlindMode.GYRO_TSHAPE}

    assert sim.resolve_speed(edge_line_fast) == SpeedTier.FAST
    assert sim.resolve_blind_mode(edge_blind_gyro) == BlindMode.GYRO_TSHAPE


# ---------------------------------------------------------------------------
# Symmetric trapezoidal profile — FSM-level coverage of the C++ ramp-down
# behaviour (see DECISIONS.md "LineFollower distance-gated stops use a
# symmetric trapezoidal profile" and `WRO2026/LineFollower.cpp`).
#
# The runtime sim collapses each primitive into a single atomic geometric
# advance — it does NOT model PWM trajectories — so these tests verify the
# FSM contract (state-transitions + final position) rather than per-tick
# cruise samples. PWM-level behaviour is verified on hardware; the sim's
# job is to prove that the dispatched primitive sequence still matches the
# C++ runtime after the LineFollower ramp-down work.
# ---------------------------------------------------------------------------


def test_line_overshoot_no_rampup_only_rampdown():
    """LINE_OVERSHOOT advances exactly LINE_SENSOR_AXLE_OFFSET_CM with no cruise-→-0 step.

    The C++ runtime re-arms ``LineFollower.setEnabled(true).setTShapeRampUp(false)
    .setDistanceTarget(LINE_SENSOR_AXLE_OFFSET_CM)`` at intersection-detect.
    The re-arm guard in ``_applyEnableBuilder`` skips the ramp-up reseed
    (so cruise does not drop from preset back to 0) and the ramp-down
    fitted to the 5 cm hop carries the chassis from cruise PWM down to
    ``Config::PWM_MIN_MAGNITUDE`` before the gate's active brake fires.
    The sim does not model PWM, so the equivalent FSM-level contract is
    that the LINE_OVERSHOOT step rolls the final ``LINE_SENSOR_AXLE_OFFSET_CM``
    so the wheel axle lands ON the node (net advance == the edge length, not
    past it) and Localization advances to the next node.
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (30.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, 30.0, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    # FSM contract: FOLLOW_INT then a separate LINE_OVERSHOOT step
    # carrying the LINE_SENSOR_AXLE_OFFSET_CM payload — never collapsed,
    # never elided, never re-emitting an ACT_TURN_REL between them.
    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM),
    ]
    # Position contract: net advance == the 30 cm edge; the axle lands ON node B
    # (25 cm FOLLOW_INT + 5 cm LINE_OVERSHOOT along +x = 30 cm, not past it).
    assert chassis.x_cm == pytest.approx(30.0, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_projection_full_trapezoid():
    """A long LINE-into-projection hop runs ramp-up + plateau + ramp-down and lands on the node.

    Mirrors the hardware contract for a long LINE hop into a projection node
    (issue #64, DISTANCE-terminated): the C++ ``LineFollower`` ramps up from 0 to
    cruise, holds the plateau, then ramps down to ``Config::PWM_MIN_MAGNITUDE``
    before the distance gate actively brakes the chassis. The sim collapses all
    three phases into a single atomic ``follow_line_to_distance(100.0)`` advance,
    so the FSM-level invariants are: a single ``ACT_FOLLOW_DIST`` primitive, no
    ``ACT_LINE_OVERSHOOT`` (the encoder gate already lands the chassis on the
    node), and a chassis position exactly at the edge's terminus.
    """
    distance_cm = 100.0
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (distance_cm, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, distance_cm, 0.0, v_type="projection")
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FOLLOW_DIST, distance_cm)]
    # No overshoot step on a projection arrival — encoder gate lands the chassis on the node.
    assert not any(action == ACT_LINE_OVERSHOOT for action, _ in prims)
    assert chassis.x_cm == pytest.approx(distance_cm, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_projection_short_truncated_triangle():
    """A LINE-into-projection hop shorter than 2*TSHAPE_RAMP_MIN_CM collapses to a half-clamped triangle.

    With both ramp-up and ramp-down active on a very short LINE-into-projection
    hop (issue #64), the C++ ``LineFollower`` clamps each ramp to half the
    distance target so the two zones cannot overlap — producing a triangular
    profile with no plateau. The sim still collapses the whole thing to a single
    atomic ``follow_line_to_distance(cm)`` advance, so the FSM-level invariants
    are unchanged: a single ``ACT_FOLLOW_DIST`` primitive, no
    ``ACT_LINE_OVERSHOOT``, and chassis position exactly at the terminus.
    The picked distance ``3.0`` cm is below ``2 * TSHAPE_RAMP_MIN_CM`` =
    4.0 cm (using the C++ ``Config::TSHAPE_RAMP_MIN_CM = 2.0`` cm), so it
    is the regime where the half-clamp kicks in.
    """
    distance_cm = 3.0
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (distance_cm, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, distance_cm, 0.0, v_type="projection")
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FOLLOW_DIST, distance_cm)]
    assert not any(action == ACT_LINE_OVERSHOOT for action, _ in prims)
    assert chassis.x_cm == pytest.approx(distance_cm, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


# ===========================================================================
# Bidirectional line-following sim end-to-end tests (T14)
#
# Covers: AC1-AC9 (both clauses), AC11, AC12, AC14, TC17, TC18.
#
# Fixtures: tools/path_planner/tests/fixtures/bidirectional_field_data.py
# Code under test: tools/path_planner/sim/runtime.py -- resolve_direction,
#                  PathTrackerSim.go_to, Direction, BIDIRECTIONAL_AUTO_ENABLED,
#                  LINE_BOARD_COUNT.
# ===========================================================================


# ---------------------------------------------------------------------------
# Shared sim factory
# ---------------------------------------------------------------------------


def _make_bidir_sim(fd):
    """Return a (sim, chassis) pair from a bidirectional fixture namespace.

    The fixture namespace carries build_graph() and build_next_hop()
    factory attributes mirroring the collinear_4node pattern.

    The bidirectional fixtures predate the per-turn pivot model and expose no
    ``PIVOT_TURNS`` attribute, but :class:`PathTrackerSim` resolves it on
    construction (ADR-036). These fields declare no pivots, so default it empty
    here rather than editing the shared bidirectional fixture module.
    """
    if not hasattr(fd, "PIVOT_TURNS"):
        fd.PIVOT_TURNS = []
    graph = fd.build_graph()
    next_hop = fd.build_next_hop()
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis


# ---------------------------------------------------------------------------
# TC1 -- resolve_direction heading boundary: 89deg / 90deg / 91deg
#
# Verifies AC1/AC2: threshold is strictly greater-than so ties (== 90deg)
# stay FRONT.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "heading_delta_deg, expected_direction",
    [
        (89.0, Direction.FRONT),   # abs(89) < 90  -> FRONT
        (90.0, Direction.FRONT),   # abs(90) == 90 -> FRONT  (strictly >)
        (91.0, Direction.REAR),    # abs(91) > 90  -> REAR
    ],
    ids=["TC1_89deg_FRONT", "TC1_90deg_FRONT", "TC1_91deg_REAR"],
)
def test_bidir_tc1_resolve_direction_boundary(heading_delta_deg, expected_direction):
    """TC1: Auto algorithm boundary at BIDIRECTIONAL_TURN_THRESHOLD_DEG = 90deg.

    At exactly 90deg the condition abs(delta) > threshold is False -> FRONT.
    At 91deg the condition is True -> REAR.  Verifies AC1/AC2.
    """
    result = resolve_direction(
        edge_dir_override=DirectionMode.DEFAULT,
        pipeline_default=DirectionMode.DEFAULT,
        builder_arrival=Direction.AUTO,
        builder_route=Direction.AUTO,
        current_heading=0.0,
        edge_heading=heading_delta_deg,
        is_last_hop=False,
        bidirectional_enabled=True,
        line_board_count=2,
    )
    assert result == expected_direction


# ---------------------------------------------------------------------------
# TC2 -- heading delta 179.5deg -> REAR (near-antiparallel)
# ---------------------------------------------------------------------------


def test_bidir_tc2_near_antiparallel_resolves_rear():
    """TC2: A 179.5deg heading delta exceeds the 90deg threshold -> REAR.

    Verifies AC1: near-antiparallel edges are served by the rear sensor.
    """
    result = resolve_direction(
        edge_dir_override=DirectionMode.DEFAULT,
        pipeline_default=DirectionMode.DEFAULT,
        builder_arrival=Direction.AUTO,
        builder_route=Direction.AUTO,
        current_heading=0.0,
        edge_heading=179.5,
        is_last_hop=False,
        bidirectional_enabled=True,
        line_board_count=2,
    )
    assert result == Direction.REAR


# ---------------------------------------------------------------------------
# TC3 -- BIDIRECTIONAL_AUTO_ENABLED kill-switch disables auto REAR (AC9 clause a)
# ---------------------------------------------------------------------------


def test_bidir_tc3_kill_switch_disables_auto_rear(monkeypatch):
    """TC3: When BIDIRECTIONAL_AUTO_ENABLED is monkeypatched to False, a 175deg delta
    still resolves FRONT.  The auto step checks bidirectional_enabled and
    short-circuits to FRONT before evaluating the heading delta.
    Verifies AC9 clause a.
    """
    monkeypatch.setattr(_runtime, "BIDIRECTIONAL_AUTO_ENABLED", False)

    result = resolve_direction(
        edge_dir_override=DirectionMode.DEFAULT,
        pipeline_default=DirectionMode.DEFAULT,
        builder_arrival=Direction.AUTO,
        builder_route=Direction.AUTO,
        current_heading=0.0,
        edge_heading=175.0,
        is_last_hop=False,
        bidirectional_enabled=False,
        line_board_count=2,
    )
    assert result == Direction.FRONT


# ---------------------------------------------------------------------------
# TC3b -- kill-switch off + explicit route_direction=REAR still wins (AC9b)
# ---------------------------------------------------------------------------


def test_bidir_tc3b_kill_switch_off_explicit_rear_still_rear(monkeypatch):
    """TC3b: kill-switch=False but builder override=REAR -> REAR.

    AC9 clause b: setRouteDirection(REAR) bypasses the kill-switch because
    step 4 (builder route) runs before step 5 (auto kill-switch check).
    """
    monkeypatch.setattr(_runtime, "BIDIRECTIONAL_AUTO_ENABLED", False)

    result = resolve_direction(
        edge_dir_override=DirectionMode.DEFAULT,
        pipeline_default=DirectionMode.DEFAULT,
        builder_arrival=Direction.AUTO,
        builder_route=Direction.REAR,
        current_heading=0.0,
        edge_heading=0.0,
        is_last_hop=False,
        bidirectional_enabled=False,
        line_board_count=2,
    )
    assert result == Direction.REAR


# ---------------------------------------------------------------------------
# TC4 -- LINE_BOARD_COUNT == 1 prevents auto REAR (AC3)
# ---------------------------------------------------------------------------


def test_bidir_tc4_one_board_chassis_cannot_auto_rear(monkeypatch):
    """TC4: With LINE_BOARD_COUNT monkeypatched to 1, a 175deg delta resolves FRONT.

    The auto step checks line_board_count < 2 and short-circuits to FRONT
    when the rear board is absent.  Verifies AC3.
    """
    monkeypatch.setattr(_runtime, "LINE_BOARD_COUNT", 1)

    result = resolve_direction(
        edge_dir_override=DirectionMode.DEFAULT,
        pipeline_default=DirectionMode.DEFAULT,
        builder_arrival=Direction.AUTO,
        builder_route=Direction.AUTO,
        current_heading=0.0,
        edge_heading=175.0,
        is_last_hop=False,
        bidirectional_enabled=True,
        line_board_count=1,
    )
    assert result == Direction.FRONT


# ---------------------------------------------------------------------------
# TC5 -- setArrivalDirection(REAR) forces only the last hop REAR (AC6/AC7)
# ---------------------------------------------------------------------------


def test_bidir_tc5_arrival_direction_rear_only_last_hop():
    """TC5: arrival_direction=REAR forces only the C->D (is_last_hop=True) hop REAR.

    On the mixed-turn fixture (edge headings 0deg / 135deg / 270deg):
      A->B: auto, delta 0deg -> FRONT.
      B->C: auto, delta 135deg > 90deg -> REAR (body becomes 315deg).
      C->D: auto would resolve FRONT (delta -45deg from body 315deg);
            is_last_hop=True and arrival=REAR overrides -> forced REAR.

    Two ACT_LINE_REVERSE (B->C, C->D) and one ACT_FOLLOW_INT (A->B).
    The fixture's last-hop auto-FRONT makes this assertion load-bearing on
    the arrival_direction parameter -- without the override it would emit
    only one ACT_LINE_REVERSE.  Verifies AC6/AC7.
    """
    fd = bidirectional_field_data.make_mixed_turn_field()
    sim, chassis = _make_bidir_sim(fd)

    prims = sim.go_to("D", arrival_direction=Direction.REAR)
    act_tags = [p[0] for p in prims]

    assert ACT_FOLLOW_INT in act_tags
    assert act_tags.count(ACT_LINE_REVERSE) == 2
    assert sim.localization.current_node() == "D"


def test_bidir_tc5_baseline_no_arrival_override_last_hop_is_front():
    """TC5 negative control: without arrival_direction, the last hop auto-FRONTs.

    Uses the same fixture as TC5 but lets every hop auto-resolve.  Proves
    that TC5's arrival_direction=REAR is load-bearing: without it the
    C->D hop emits ACT_FOLLOW_INT (not ACT_LINE_REVERSE), so the second
    ACT_LINE_REVERSE in TC5 can only come from the override.
    """
    fd = bidirectional_field_data.make_mixed_turn_field()
    sim, _chassis = _make_bidir_sim(fd)

    prims = sim.go_to("D")
    act_tags = [p[0] for p in prims]

    assert act_tags.count(ACT_LINE_REVERSE) == 1
    assert act_tags.count(ACT_FOLLOW_INT) == 2
    assert sim.localization.current_node() == "D"


# ---------------------------------------------------------------------------
# TC6 -- setRouteDirection(REAR) forces every hop REAR (AC4/AC5)
# ---------------------------------------------------------------------------


def test_bidir_tc6_route_direction_rear_all_hops():
    """TC6: route_direction=REAR forces ALL three hops on the path to use REAR.

    Every hop emits ACT_LINE_REVERSE; no ACT_FOLLOW_INT is emitted anywhere.
    Verifies AC4/AC5.
    """
    fd = bidirectional_field_data.make_mixed_turn_field()
    sim, chassis = _make_bidir_sim(fd)

    prims = sim.go_to("D", route_direction=Direction.REAR)
    act_tags = [p[0] for p in prims]

    assert act_tags.count(ACT_LINE_REVERSE) == 3
    assert ACT_FOLLOW_INT not in act_tags
    assert sim.localization.current_node() == "D"


# ---------------------------------------------------------------------------
# TC7 -- per-edge direction_override wins over builder route direction (AC8)
# ---------------------------------------------------------------------------


def test_bidir_tc7_per_edge_override_wins_over_builder():
    """TC7: Edge A->B has direction_override=FORCE_REVERSE; builder says FRONT.

    Step 1 of the resolution algorithm is binding and supersedes the builder
    preference.  A->B emits ACT_LINE_REVERSE; B->C (no override, builder
    FRONT) emits ACT_FOLLOW_INT.  The REAR hop must precede the FRONT hop in
    emission order.  Verifies AC8.
    """
    fd = bidirectional_field_data.make_rear_edge_field()
    sim, chassis = _make_bidir_sim(fd)

    prims = sim.go_to("C", route_direction=Direction.FRONT)
    act_tags = [p[0] for p in prims]

    assert act_tags.count(ACT_LINE_REVERSE) == 1
    assert act_tags.count(ACT_FOLLOW_INT) == 1

    # The REAR hop (A->B) must precede the FRONT hop (B->C).
    line_tags = [t for t in act_tags if t in (ACT_LINE_REVERSE, ACT_FOLLOW_INT)]
    assert line_tags[0] == ACT_LINE_REVERSE

    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC10 -- BLIND edge resolved REAR emits ACT_BLIND_REVERSE (AC12)
# ---------------------------------------------------------------------------


def test_bidir_tc10_blind_edge_rear_emits_blind_reverse():
    """TC10: A BLIND edge traversed with REAR direction emits ACT_BLIND_REVERSE.

    The primitive tuple shape is:
        (ACT_BLIND_REVERSE, target_node, distance_cm, blind_mode)

    Chassis advances via VirtualChassis.reverse().  Per-edge FORCE_REVERSE
    override used for determinism.  Verifies AC12.
    """
    distance_cm = 40.0
    nodes = {"A": (0.0, 0.0), "B": (distance_cm, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge(
        "A", "B",
        type=EDGE_BLIND,
        distance_cm=distance_cm,
        heading_deg=0.0,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    g.add_edge(
        "B", "A",
        type=EDGE_BLIND,
        distance_cm=distance_cm,
        heading_deg=180.0,
    )
    next_hop = {"A": {"A": "A", "B": "B"}, "B": {"A": "A", "B": "B"}}

    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")
    act_tags = [p[0] for p in prims]

    assert ACT_BLIND_REVERSE in act_tags

    blind_rev_prims = [p for p in prims if p[0] == ACT_BLIND_REVERSE]
    assert len(blind_rev_prims) == 1
    _, target_node, emitted_dist, emitted_mode = blind_rev_prims[0]
    assert target_node == "B"
    assert emitted_dist == pytest.approx(distance_cm, abs=1e-9)
    assert emitted_mode == BlindMode.SYNC_TSHAPE   # PIPELINE_CONFIG.blind default

    assert sim.localization.current_node() == "B"

    # REAR body heading = normalize(0+180) = 180deg.
    # reverse(40) moves opposite to 180deg = toward +x. chassis.x == 40cm.
    assert chassis.x_cm == pytest.approx(distance_cm, abs=_POSITION_TOL_CM)


# ---------------------------------------------------------------------------
# TC11 -- PROJECTION edge resolved REAR emits ACT_PROJECTION_REVERSE (AC11)
# ---------------------------------------------------------------------------


def test_bidir_tc11_line_into_projection_rear_emits_projection_reverse():
    """TC11: A REAR LINE hop into a projection node emits ACT_PROJECTION_REVERSE.

    Issue #64: a REAR LINE edge into a ``projection`` node mirrors the FORWARD
    LINE->projection ``ACT_FOLLOW_DIST`` -- the projection node has no junction
    marker, so the reverse follow is DISTANCE-terminated. Tuple shape:
    (ACT_PROJECTION_REVERSE, target_node, distance_cm, line_mode). The mode slot
    carries the resolved ``SpeedTier`` (``PIPELINE_CONFIG.speed`` = NORMAL), since
    a LINE->projection follow takes its motion profile from the LINE edge's own
    resolved speed (``ProjectionMode`` deleted). Per-edge FORCE_REVERSE override
    used for determinism.  Verifies AC11.
    """
    distance_cm = 25.0
    nodes = {"A": (0.0, 0.0), "B": (distance_cm, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    g = nx.DiGraph()
    g.add_node("A", type="line")
    g.add_node("B", type="projection")
    g.add_edge(
        "A", "B",
        type=EDGE_LINE,
        distance_cm=distance_cm,
        heading_deg=0.0,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    g.add_edge(
        "B", "A",
        type=EDGE_LINE,
        distance_cm=distance_cm,
        heading_deg=180.0,
    )
    next_hop = {"A": {"A": "A", "B": "B"}, "B": {"A": "A", "B": "B"}}

    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")
    act_tags = [p[0] for p in prims]

    assert ACT_PROJECTION_REVERSE in act_tags

    proj_rev_prims = [p for p in prims if p[0] == ACT_PROJECTION_REVERSE]
    assert len(proj_rev_prims) == 1
    _, target_node, emitted_dist, emitted_mode = proj_rev_prims[0]
    assert target_node == "B"
    assert emitted_dist == pytest.approx(distance_cm, abs=1e-9)
    assert emitted_mode == SpeedTier.NORMAL   # PIPELINE_CONFIG.speed default (LINE->proj motion)

    assert sim.localization.current_node() == "B"


# ---------------------------------------------------------------------------
# TC17 -- bare go_to (no modifier) defaults to AUTO direction resolution (AC14)
# ---------------------------------------------------------------------------


def test_bidir_tc17_bare_goto_defaults_to_auto():
    """TC17: A bare go_to call (no modifier) runs the AUTO algorithm.

    On the collinear fixture (all hops heading 0deg, start heading 0deg) the
    auto algorithm sees delta=0deg -> FRONT for every hop.  No ACT_LINE_REVERSE
    is emitted; three ACT_FOLLOW_INT primitives are emitted (A->B->C->D).

    Verifies AC14: go_to(goal) with no modifiers is equivalent to
    go_to(goal, route_direction=AUTO, arrival_direction=AUTO).
    """
    fd = bidirectional_field_data.make_collinear_field()
    sim, chassis = _make_bidir_sim(fd)

    prims = sim.go_to("D")
    act_tags = [p[0] for p in prims]

    assert ACT_LINE_REVERSE not in act_tags
    assert act_tags.count(ACT_FOLLOW_INT) == 3
    assert sim.localization.current_node() == "D"


# ---------------------------------------------------------------------------
# TC18 -- compiler <-> sim primitive parity
# ---------------------------------------------------------------------------


def test_bidir_tc18_compiler_sim_parity_front_only():
    """TC18 (FRONT-only path): compile_path and PathTrackerSim emit identical primitives.

    Uses a 2-hop path A->B->C with a 90deg turn at B (non-collinear) so the
    sim emits ACT_LINE_OVERSHOOT (not ACT_LINE_PASS) at the intermediate node,
    matching the _interleave_overshoot expectation.

    The compiler emits ACT_FOLLOW_INT for each EDGE_LINE hop; the sim emits
    the same plus ACT_LINE_OVERSHOOT after each ACT_FOLLOW_INT.
    Verifies TC18 cross-module parity for the FRONT-only case.
    """
    # A(0,0) --heading 0deg--> B(50,0) --heading 90deg--> C(50,50)
    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0), "C": (50.0, 50.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y))
    edges_raw = [
        ("A", "B", EDGE_LINE, 50.0, 0.0),
        ("B", "C", EDGE_LINE, 50.0, 90.0),
    ]
    for u, v, etype, dist, hdg in edges_raw:
        g.add_edge(u, v, type=etype, distance_cm=dist, heading_deg=hdg)
        g.add_edge(v, u, type=etype, distance_cm=dist, heading_deg=(hdg + 180.0) % 360.0)
    next_hop = {
        "A": {"A": "A", "B": "B", "C": "B"},
        "B": {"A": "A", "B": "B", "C": "C"},
        "C": {"A": "B", "B": "B", "C": "C"},
    }

    walked = walk_next_hop(next_hop, "A", "C")
    compiler_prims = compile_path(walked, fd.START_HEADING_DEG, g, fd)

    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    sim_prims = sim.go_to("C")

    assert sim_prims == _interleave_overshoot(compiler_prims)
    assert sim.localization.current_node() == "C"


def test_bidir_tc18_compiler_sim_parity_rear_hop():
    """TC18 (REAR hop): compile_path and PathTrackerSim emit identical primitives
    for a single REAR hop (no intermediate nodes so pass-through cannot interfere).

    T12 is already landed: compile_path emits ACT_LINE_REVERSE for REAR hops.
    A 2-node graph A->B with direction_override=FORCE_REVERSE gives a minimal
    fixture where compiler and sim must produce the same sequence:
        (ACT_TURN_REL, 180.0)                    -- turn to face 180deg (body heading for REAR)
        (ACT_LINE_REVERSE, 'B', SpeedTier.NORMAL) -- REAR traverse
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_REAR_CM)  -- sim-only overshoot step
                                                 -- (7.68 cm: a REAR hop rolls the REAR
                                                 --  board's own axle offset, ADR-035)

    Verifies the TC18 contract for the REAR case.
    """
    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge(
        "A", "B",
        type=EDGE_LINE,
        distance_cm=50.0,
        heading_deg=0.0,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    g.add_edge(
        "B", "A",
        type=EDGE_LINE,
        distance_cm=50.0,
        heading_deg=180.0,
    )
    next_hop = {"A": {"A": "A", "B": "B"}, "B": {"A": "A", "B": "B"}}

    walked = walk_next_hop(next_hop, "A", "B")
    compiler_prims = compile_path(walked, fd.START_HEADING_DEG, g, fd)

    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    sim_prims = sim.go_to("B")

    # compile_path emits (ACT_TURN_REL, 180.0) + (ACT_LINE_REVERSE, 'B', mode).
    # _interleave_overshoot adds ACT_LINE_OVERSHOOT after ACT_LINE_REVERSE.
    assert sim_prims == _interleave_overshoot(compiler_prims)
    assert sim.localization.current_node() == "B"


# ===========================================================================
# ADR-023 + bidirectional composition tests (T15)
#
# Covers AC13: pass-through fires correctly (or is correctly suppressed) when
# one or more hops on the route are traversed in the REAR direction.  The
# key invariant is that _should_pass_through peeks at resolved BODY headings,
# not raw forward-edge headings -- so a REAR->FRONT transition across
# anti-parallel forward edges still produces a zero body-heading delta and
# ACT_LINE_PASS fires (TC9c).
#
# Test IDs:
#   TC9  -- two consecutive REAR hops, collinear fwd edges -> ACT_LINE_PASS
#   TC9b -- two consecutive REAR hops, non-collinear fwd edges -> no pass-through
#   TC9c -- REAR->FRONT transition, anti-parallel fwd edges -> body delta 0 -> ACT_LINE_PASS
# ===========================================================================


def _make_three_hop_sim(
    fwd_headings: tuple,
    start_heading_deg: float,
    edge_overrides: dict = None,
) -> tuple:
    """Build a minimal (sim, graph, next_hop) triple for a 3-hop A->B->C topology.

    :param fwd_headings: ``(hdg_ab, hdg_bc)`` -- forward heading (degrees) of
        each directed edge.
    :param start_heading_deg: chassis heading at ``A`` after zeroing.
    :param edge_overrides: optional dict mapping ``(u, v)`` pairs to extra
        graph-edge attributes (e.g. ``{('A','B'): {'direction_override': 2}}``).
        Only applied to the **forward** edge; reverse edges never inherit
        directional overrides.
    :returns: ``(sim, chassis, graph, next_hop)`` ready for ``go_to`` calls.
    """
    if edge_overrides is None:
        edge_overrides = {}

    hdg_ab, hdg_bc = fwd_headings

    # Place nodes geometrically along the given headings.
    import math as _math
    ax, ay = 0.0, 0.0
    bx = ax + _math.cos(_math.radians(hdg_ab)) * 50.0
    by = ay + _math.sin(_math.radians(hdg_ab)) * 50.0
    cx = bx + _math.cos(_math.radians(hdg_bc)) * 50.0
    cy = by + _math.sin(_math.radians(hdg_bc)) * 50.0

    nodes = {"A": (ax, ay), "B": (bx, by), "C": (cx, cy)}
    fd = SimpleNamespace(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=start_heading_deg,
        PIVOT_TURNS=[],
    )

    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y))

    raw_edges = [("A", "B", hdg_ab), ("B", "C", hdg_bc)]
    for u, v, hdg in raw_edges:
        fwd_attrs = dict(type=EDGE_LINE, distance_cm=50.0, heading_deg=float(hdg))
        fwd_attrs.update(edge_overrides.get((u, v), {}))
        rev_attrs = dict(
            type=EDGE_LINE,
            distance_cm=50.0,
            heading_deg=(float(hdg) + 180.0) % 360.0,
        )
        g.add_edge(u, v, **fwd_attrs)
        g.add_edge(v, u, **rev_attrs)

    next_hop = {
        "A": {"A": "A", "B": "B", "C": "B"},
        "B": {"A": "A", "B": "B", "C": "C"},
        "C": {"A": "B", "B": "B", "C": "C"},
    }

    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis, g, next_hop


# ---------------------------------------------------------------------------
# TC9 -- two consecutive REAR hops, collinear forward edges -> ACT_LINE_PASS
#
# Topology: A->B->C, forward headings both 0deg.
# route_direction=REAR forces both hops REAR.
# Body heading for REAR on 0deg edge = 180deg.
# Inbound body heading at B = 180deg.
# Peek: B->C forward heading 0deg, forced REAR -> body heading 180deg.
# Delta = |180 - 180| = 0deg <= NAV_TURN_TOLERANCE_DEG -> ACT_LINE_PASS at B.
# No ACT_TURN_REL between the two REAR hops (chassis stays at 180deg).
# ---------------------------------------------------------------------------


def test_bidir_tc9_two_rear_hops_collinear_emits_line_pass():
    """TC9: Two consecutive REAR hops on collinear (0deg, 0deg) forward edges emit
    ACT_LINE_PASS at the intermediate intersection B, and ACT_LINE_OVERSHOOT only
    at the destination C (destination guard always emits OVERSHOOT, not PASS).

    The body heading after the first REAR hop is 180deg; the next-hop body heading
    (B->C REAR) is also 180deg; delta = 0deg -> _should_pass_through returns True
    at B.  B->C is the last hop (C is the destination) so pass-through is NOT
    re-applied at C -- the destination guard forces ACT_LINE_OVERSHOOT there.
    No ACT_TURN_REL is emitted between the two REAR hops because the chassis
    heading does not change.

    Verifies AC13: pass-through composes correctly with REAR direction.
    """
    sim, chassis, _, _ = _make_three_hop_sim(
        fwd_headings=(0.0, 0.0),
        start_heading_deg=0.0,
    )

    prims = sim.go_to("C", route_direction=Direction.REAR)
    act_tags = [p[0] for p in prims]

    # ACT_LINE_PASS must appear exactly once (at B, the intermediate node).
    assert ACT_LINE_PASS in act_tags, (
        f"Expected ACT_LINE_PASS at B for two collinear REAR hops; got {act_tags}"
    )
    assert act_tags.count(ACT_LINE_PASS) == 1, (
        f"Expected exactly one ACT_LINE_PASS (at B only); got {act_tags}"
    )

    # ACT_LINE_OVERSHOOT appears exactly once at C (destination guard).
    assert act_tags.count(ACT_LINE_OVERSHOOT) == 1, (
        f"Expected exactly one ACT_LINE_OVERSHOOT (at destination C); got {act_tags}"
    )

    # No inter-hop ACT_TURN_REL: chassis stays at body heading 180deg the whole way.
    # The initial turn from start heading (0deg) to first body heading (180deg) IS
    # emitted (it is NOT between hops); strip it and check the slice between the
    # two LINE_REVERSE primitives.
    line_rev_indices = [i for i, (tag, *_) in enumerate(prims) if tag == ACT_LINE_REVERSE]
    assert len(line_rev_indices) == 2, (
        f"Expected two ACT_LINE_REVERSE primitives for two forced-REAR hops; got {act_tags}"
    )
    # Primitives between the two ACT_LINE_REVERSE are: ACT_LINE_PASS only (no turn).
    between = prims[line_rev_indices[0] + 1 : line_rev_indices[1]]
    between_tags = [p[0] for p in between]
    assert ACT_TURN_REL not in between_tags, (
        f"Unexpected ACT_TURN_REL between the two REAR hops; slice={between}"
    )
    assert between_tags == [ACT_LINE_PASS], (
        f"Expected only [ACT_LINE_PASS] between the two REAR hops; got {between_tags}"
    )

    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC9b -- two consecutive REAR hops, non-collinear -> no pass-through
#
# Topology: A->B->C, forward headings 0deg then 90deg.
# route_direction=REAR forces both hops REAR.
# Body heading after A->B REAR on 0deg edge = 180deg.
# Peek: B->C forward heading 90deg, forced REAR -> body heading 270deg.
# Delta = |270 - 180| = 90deg > NAV_TURN_TOLERANCE_DEG -> ACT_LINE_OVERSHOOT at B.
# A pre-hop turn IS emitted before the second REAR hop.
# ---------------------------------------------------------------------------


def test_bidir_tc9b_two_rear_hops_non_collinear_no_line_pass():
    """TC9b: Two consecutive REAR hops on non-collinear (0deg, 90deg) forward edges do
    NOT emit ACT_LINE_PASS at B.

    Body heading after A->B REAR = 180deg.  Next-hop peek (B->C REAR) resolves to
    body heading 270deg.  Delta = 90deg > NAV_TURN_TOLERANCE_DEG -> pass-through
    is blocked; ACT_LINE_OVERSHOOT fires and a pre-hop ACT_TURN_REL is emitted
    before the second REAR hop.

    Verifies AC13 (negative path): non-collinear REAR hops correctly suppress
    pass-through and still emit the turn primitive.
    """
    sim, chassis, _, _ = _make_three_hop_sim(
        fwd_headings=(0.0, 90.0),
        start_heading_deg=0.0,
    )

    prims = sim.go_to("C", route_direction=Direction.REAR)
    act_tags = [p[0] for p in prims]

    # ACT_LINE_PASS must NOT appear when the body-heading delta exceeds tolerance.
    assert ACT_LINE_PASS not in act_tags, (
        f"Unexpected ACT_LINE_PASS for non-collinear REAR hops; got {act_tags}"
    )

    # ACT_LINE_OVERSHOOT must appear at B (pass-through blocked).
    assert ACT_LINE_OVERSHOOT in act_tags, (
        f"Expected ACT_LINE_OVERSHOOT at B when pass-through is blocked; got {act_tags}"
    )

    # A turn primitive must appear between the two REAR hops (body heading changes
    # from 180deg to 270deg to align for the second hop).
    line_rev_indices = [i for i, (tag, *_) in enumerate(prims) if tag == ACT_LINE_REVERSE]
    assert len(line_rev_indices) == 2, (
        f"Expected two ACT_LINE_REVERSE primitives for two forced-REAR hops; got {act_tags}"
    )
    between = prims[line_rev_indices[0] + 1 : line_rev_indices[1]]
    between_tags = [p[0] for p in between]
    assert ACT_TURN_REL in between_tags, (
        f"Expected ACT_TURN_REL between non-collinear REAR hops; slice={between}"
    )

    assert sim.localization.current_node() == "C"


# ---------------------------------------------------------------------------
# TC9c -- mixed REAR->FRONT transition on anti-parallel forward edges -> ACT_LINE_PASS
#
# Topology: A->B->C.
#   A->B: forward heading 0deg, forced REAR via direction_override.
#         Body heading = (0 + 180) % 360 = 180deg.
#   B->C: forward heading 180deg, no override.
#         Auto-resolution: chassis heading after A->B = 180deg,
#         edge_heading = 180deg, delta = wrap(180 - 180) = 0deg <= 90deg -> FRONT.
#         Body heading = 180deg (FRONT on 180deg forward edge).
# Body-heading delta at B = |180 - 180| = 0deg -> ACT_LINE_PASS fires.
#
# Critical invariant: _should_pass_through uses resolved BODY headings, not raw
# forward-edge headings. The forward-edge delta would be |180 - 0| = 180deg
# (which would BLOCK pass-through). The body-heading delta is 0deg (which fires
# pass-through). TC9c breaks if the peek reverts to raw forward-edge comparison.
# ---------------------------------------------------------------------------


def test_bidir_tc9c_rear_to_front_direction_flip_suppresses_pass_through():
    """TC9c: REAR->FRONT transition on anti-parallel forward edges (0deg, 180deg) —
    body-heading peek correctly identifies the geometry as collinear, but the
    direction-flip guard (PR #34 review, F6) suppresses pass-through because
    firing it would force LineFollower.setReverse(true->false) mid-flight, instantly
    inverting cruise sign and jolting the chassis from +x motion to -x motion
    in one tick without a controlled stop.

    Body-heading geometry of the fixture is unchanged from the original TC9c:
        A->B forced REAR (body heading = 180deg);
        after A->B REAR, chassis heading = 180deg;
        B->C forward heading = 180deg, auto-resolves FRONT (delta = 0deg < 90deg);
        body delta at B = |180 - 180| = 0deg <= NAV_TURN_TOLERANCE_DEG.

    The peek therefore correctly identifies the body-heading collinearity (which
    is what the original TC9c proved). The new direction-flip guard then notes
    that next_dir (FRONT) != inbound _resolvedDirection (REAR) and blocks
    pass-through, falling through to ACT_LINE_OVERSHOOT + ACT_TURN_REL so the
    LineFollower has a clean re-init window before the outbound hop.

    Without F6: ACT_LINE_PASS at B, motors flip cruise sign in one tick.
    With F6:    ACT_LINE_OVERSHOOT at B + a 180deg turn before the FRONT hop.

    The body-heading peek itself is still load-bearing — it correctly classifies
    the geometry as collinear. The fact that we don't ACT on that classification
    when direction differs is the new safety guard.

    Verifies AC13 (mixed-direction clause of ADR-023) AND the F6 safety guard.
    """
    sim, chassis, _, _ = _make_three_hop_sim(
        fwd_headings=(0.0, 180.0),
        start_heading_deg=0.0,
        edge_overrides={
            ("A", "B"): {"direction_override": int(DirectionMode.FORCE_REVERSE)},
        },
    )
    # B->C has forward heading 180deg (no override -- auto will resolve FRONT because
    # after the A->B REAR hop the chassis heading is 180deg, so delta = 0deg <= 90deg).
    # We do NOT pass route_direction=REAR here; only A->B is forced REAR via the
    # per-edge override. B->C uses auto resolution.

    prims = sim.go_to("C")
    act_tags = [p[0] for p in prims]

    # ACT_LINE_PASS must NOT fire anywhere — direction flips between hops, and
    # the F6 guard suppresses pass-through across direction changes.
    assert ACT_LINE_PASS not in act_tags, (
        f"Expected NO ACT_LINE_PASS (F6 direction-flip guard should suppress); "
        f"got {act_tags}. If you see ACT_LINE_PASS, the guard regressed or the "
        f"fixture geometry no longer triggers a direction flip."
    )

    # ACT_LINE_REVERSE fires once for A->B REAR; ACT_FOLLOW_INT fires once for B->C FRONT.
    assert act_tags.count(ACT_LINE_REVERSE) == 1, (
        f"Expected one ACT_LINE_REVERSE (A->B REAR); got {act_tags}"
    )
    assert act_tags.count(ACT_FOLLOW_INT) == 1, (
        f"Expected one ACT_FOLLOW_INT (B->C FRONT); got {act_tags}"
    )

    # OVERSHOOT fires at BOTH B (post-REAR, suppressed pass-through) AND C
    # (destination guard always emits OVERSHOOT). Two OVERSHOOT total.
    assert act_tags.count(ACT_LINE_OVERSHOOT) == 2, (
        f"Expected two ACT_LINE_OVERSHOOT (suppressed pass-through at B + "
        f"destination guard at C); got {act_tags}"
    )

    assert sim.localization.current_node() == "C"


# ===========================================================================
# STOP_AT_LINE blind edges (TC5 / TC6)
#
# A BLIND edge whose effective BlindMode is STOP_AT_LINE drives blind toward
# the expected crossing and stops on the physical perpendicular line. FRONT
# traversal re-anchors localization via on_intersection (the "better
# localization") instead of dead-reckoning; the chassis ends exactly on the
# node, so a goal-position cross-check passes. REAR traversal emits the
# blind-reverse primitive carrying BlindMode.STOP_AT_LINE.
#
# Code under test: tools/path_planner/sim/runtime.py -- PathTrackerSim.go_to
# EDGE_BLIND branch (FRONT STOP_AT_LINE re-anchor; REAR blind-reverse).
# Fixtures: synthetic single-edge graphs marked mode=
# BlindMode.STOP_AT_LINE (the schema's per-edge override slot).
# ===========================================================================


class _RecordingLocalizationSim(LocalizationSim):
    """LocalizationSim that records every advance call for assertion.

    Each entry is a ``(method_tag, advance_to)`` pair, ``method_tag`` being
    ``"on_intersection"`` or ``"on_dead_reckoned_advance"``. Lets a test
    prove a STOP_AT_LINE FRONT hop re-anchors via the intersection path
    rather than the dead-reckon path -- the difference the feature exists to
    make.
    """

    def __init__(self, field_data) -> None:
        super().__init__(field_data)
        self.advance_calls: List[tuple] = []

    def on_intersection(self, advance_to: str) -> None:
        self.advance_calls.append(("on_intersection", advance_to))
        super().on_intersection(advance_to)

    def on_dead_reckoned_advance(self, advance_to: str) -> None:
        self.advance_calls.append(("on_dead_reckoned_advance", advance_to))
        super().on_dead_reckoned_advance(advance_to)


def _stop_at_line_blind_sim(
    distance_cm: float,
    *,
    force_reverse: bool = False,
):
    """Build a sim over a single STOP_AT_LINE BLIND edge A->B heading 0 deg.

    The forward (A, B) edge carries ``mode=BlindMode.STOP_AT_LINE``
    (and, when @p force_reverse is set, ``direction_override=FORCE_REVERSE``
    so the hop resolves REAR). The sim's localization is swapped for a
    :class:`_RecordingLocalizationSim` so the test can assert which advance
    path fired.

    :param distance_cm: forward edge length in centimetres.
    :param force_reverse: tag the forward edge FORCE_REVERSE when True.
    :returns: ``(sim, chassis)`` with localization already zeroed.
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (distance_cm, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    fwd_attrs = dict(
        type=EDGE_BLIND,
        distance_cm=float(distance_cm),
        heading_deg=0.0,
        mode=BlindMode.STOP_AT_LINE,
    )
    if force_reverse:
        fwd_attrs["direction_override"] = int(DirectionMode.FORCE_REVERSE)
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge("A", "B", **fwd_attrs)
    g.add_edge(
        "B", "A",
        type=EDGE_BLIND,
        distance_cm=float(distance_cm),
        heading_deg=180.0,
        mode=BlindMode.STOP_AT_LINE,
    )
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim._localization = _RecordingLocalizationSim(fd)
    sim.localization.zero_heading_from_current()
    return sim, chassis


def test_sim_stop_at_line_blind_edge_reanchors_via_on_intersection():
    """TC5: a FRONT STOP_AT_LINE blind edge emits ACT_STOP_AT_LINE and re-anchors.

    A 40 cm BLIND edge A->B heading 0 deg with blind_mode=STOP_AT_LINE must
    produce exactly ``[(ACT_STOP_AT_LINE, 40.0)]`` (no ACT_LINE_OVERSHOOT --
    it is a blind hop, not a line hop), advance Localization via
    on_intersection (NOT on_dead_reckoned_advance), and land the chassis on
    the node. The on_intersection re-anchor is the "better localization" the
    feature delivers; the goal-position cross-check (x == 40 cm, y == 0) is
    the LINE-arrival equivalence the plan calls for.
    """
    sim, chassis = _stop_at_line_blind_sim(40.0)

    prims = sim.go_to("B")

    assert prims == [(ACT_STOP_AT_LINE, pytest.approx(40.0, abs=1e-9))]
    # Re-anchor via on_intersection, never the dead-reckon path.
    assert sim.localization.advance_calls == [("on_intersection", "B")]
    # Chassis ends on the node -- goal-position cross-check.
    assert chassis.x_cm == pytest.approx(40.0, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_sim_stop_at_line_blind_edge_step_through_emits_stop_at_line():
    """TC5 (step_through): the generator variant yields a single ACT_STOP_AT_LINE step."""
    sim, _chassis = _stop_at_line_blind_sim(40.0)

    steps = list(sim.step_through("B"))
    prims = [prim for prim, _snap in steps]

    assert prims == [(ACT_STOP_AT_LINE, pytest.approx(40.0, abs=1e-9))]
    assert sim.localization.advance_calls == [("on_intersection", "B")]
    assert sim.localization.current_node() == "B"


def test_sim_stop_at_line_only_overrides_blind_edges():
    """A plain (default) blind edge still dead-reckons -- STOP_AT_LINE does not leak.

    Guards the mode boundary on the sim side: a BLIND edge without the
    override emits ACT_FORWARD and advances via on_dead_reckoned_advance, so
    the STOP_AT_LINE re-anchor path is gated strictly on the override.
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (20.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_BLIND, 20.0, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim._localization = _RecordingLocalizationSim(fd)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FORWARD, pytest.approx(20.0, abs=1e-9))]
    assert sim.localization.advance_calls == [("on_dead_reckoned_advance", "B")]
    assert sim.localization.current_node() == "B"


def test_sim_stop_at_line_blind_edge_rear_2board_seeks_via_stop_at_line_reverse():
    """A 2-board REAR STOP_AT_LINE blind edge seeks via ACT_STOP_AT_LINE_REVERSE.

    On the default 2-board chassis (module ``LINE_BOARD_COUNT == 2``), a
    forced-REAR traversal of the STOP_AT_LINE BLIND edge has a rear board
    facing the travel direction, so ``resolve_stop_at_line`` returns
    ``seek=True``. The hop turns 180 deg (so the rear sensor leads), emits
    ``(ACT_STOP_AT_LINE_REVERSE, 'B', 40.0, BlindMode.STOP_AT_LINE)``, and
    re-anchors via ``on_intersection`` (the rear board detects the physical
    crossing) -- mirroring the firmware reverse seek + ``onIntersection``
    (AC1). The chassis ends on the node.
    """
    sim, chassis = _stop_at_line_blind_sim(40.0, force_reverse=True)

    prims = sim.go_to("B")
    act_tags = [p[0] for p in prims]

    assert ACT_STOP_AT_LINE_REVERSE in act_tags
    assert ACT_BLIND_REVERSE not in act_tags
    stop_rev = [p for p in prims if p[0] == ACT_STOP_AT_LINE_REVERSE]
    assert len(stop_rev) == 1
    _, target_node, emitted_dist, emitted_mode = stop_rev[0]
    assert target_node == "B"
    assert emitted_dist == pytest.approx(40.0, abs=1e-9)
    assert emitted_mode == BlindMode.STOP_AT_LINE
    # 180 deg turn precedes the reverse hop so the rear sensor leads.
    assert (ACT_TURN_REL, pytest.approx(180.0, abs=1e-9)) in prims
    # Chassis ends on the node (REAR body heading 180 deg; reverse(40) -> +x).
    assert chassis.x_cm == pytest.approx(40.0, abs=_POSITION_TOL_CM)
    # Rear-board seek re-anchors on the physical crossing, not dead-reckoning.
    assert sim.localization.advance_calls == [("on_intersection", "B")]
    assert sim.localization.current_node() == "B"


def test_sim_stop_at_line_blind_edge_rear_1board_degrades_to_blind_reverse(monkeypatch):
    """A 1-board forced-REAR STOP_AT_LINE blind edge degrades to ACT_BLIND_REVERSE.

    With ``LINE_BOARD_COUNT == 1`` there is no rear board to seek with, so
    ``resolve_stop_at_line`` returns ``seek=False`` for the forced-REAR hop.
    It degrades to a reverse dead-reckon: turn 180 deg, emit
    ``(ACT_BLIND_REVERSE, 'B', 40.0, BlindMode.STOP_AT_LINE)``, and advance via
    ``on_dead_reckoned_advance`` -- no seek, no fault (AC2).
    """
    monkeypatch.setattr(_runtime, "LINE_BOARD_COUNT", 1)
    sim, chassis = _stop_at_line_blind_sim(40.0, force_reverse=True)

    prims = sim.go_to("B")
    act_tags = [p[0] for p in prims]

    assert ACT_BLIND_REVERSE in act_tags
    assert ACT_STOP_AT_LINE_REVERSE not in act_tags
    blind_rev = [p for p in prims if p[0] == ACT_BLIND_REVERSE]
    assert len(blind_rev) == 1
    _, target_node, emitted_dist, emitted_mode = blind_rev[0]
    assert target_node == "B"
    assert emitted_dist == pytest.approx(40.0, abs=1e-9)
    assert emitted_mode == BlindMode.STOP_AT_LINE
    # 180 deg turn precedes the reverse hop so the rear sensor leads.
    assert (ACT_TURN_REL, pytest.approx(180.0, abs=1e-9)) in prims
    # Chassis ends on the node (REAR body heading 180 deg; reverse(40) -> +x).
    assert chassis.x_cm == pytest.approx(40.0, abs=_POSITION_TOL_CM)
    # 1-board degrade dead-reckons; no rear-board seek re-anchor.
    assert sim.localization.advance_calls == [("on_dead_reckoned_advance", "B")]
    assert sim.localization.current_node() == "B"


# ===========================================================================
# TC13 -- resolver-vs-fixture parity (Python side)
#
# Verifies AC4 (#6): the Python ``resolve_stop_at_line`` in runtime.py is a
# faithful transcription of the canonical truth table frozen at
# ``tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv`` (T0).
# Every row in the fixture is driven through the resolver and the result is
# compared to the expected columns. A row-count assertion guards against a
# truncated or empty CSV vacuously passing.
#
# Column names are used for lookup (not positional index) so a future column
# reorder cannot silently desync the test from the fixture.
# ===========================================================================

_FIXTURE_PATH = (
    pathlib.Path(__file__).parent / "fixtures" / "stop_at_line_truth_table.csv"
)

# 936 rows = base 864 (3 callFlag × 3 edgeOverride × 4 edgeMode
#            × 3 pipelineBlind × 2 direction × 2 lineBoardCount × 2 autoRuleActive)
#            + supplemental 72 (pipelineBlind=STOP_AT_LINE, edgeMode pinned).
# The generator (tools/path_planner/tests/fixtures/gen_stop_at_line_truth_table.py)
# is the single source of truth; this constant is a guard, not a magic number.
_EXPECTED_ROW_COUNT = 936

_CALL_FLAG_MAP = {
    "AUTO": StopAtLine.AUTO,
    "ON":   StopAtLine.ON,
    "OFF":  StopAtLine.OFF,
}

_EDGE_OVERRIDE_MAP = {
    "DEFAULT": StopAtLineMode.DEFAULT,
    "ON":      StopAtLineMode.ON,
    "OFF":     StopAtLineMode.OFF,
}

_BLIND_MODE_MAP = {
    "SYNC_TSHAPE":  BlindMode.SYNC_TSHAPE,
    "GYRO_TSHAPE":  BlindMode.GYRO_TSHAPE,
    "RAW_FLAT":     BlindMode.RAW_FLAT,
    "STOP_AT_LINE": BlindMode.STOP_AT_LINE,
}

_DIRECTION_MAP = {
    "FRONT": Direction.FRONT,
    "REAR":  Direction.REAR,
}


def test_tc13_resolve_stop_at_line_matches_fixture():
    """TC13 (Python): resolve_stop_at_line agrees with the T0 truth table for every row.

    Reads ``stop_at_line_truth_table.csv`` by column name (DictReader, so column
    order is irrelevant) and calls ``resolve_stop_at_line`` for each of the 936
    input combinations, asserting the returned ``(seek, motion_mode)``
    matches the fixture's ``expect_seek`` / ``expect_motionMode`` columns.

    Satisfies AC4 (#6): the Python transcription cannot silently drift from
    the canonical single-source truth table.
    """
    assert _FIXTURE_PATH.exists(), (
        f"Truth-table fixture not found at {_FIXTURE_PATH}. "
        "Run T0 to generate it before running TC13."
    )

    rows = []
    with _FIXTURE_PATH.open(newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(row)

    assert len(rows) == _EXPECTED_ROW_COUNT, (
        f"Expected {_EXPECTED_ROW_COUNT} data rows in the truth-table fixture "
        f"but found {len(rows)}. A truncated or empty CSV would produce "
        "vacuous passing without this guard."
    )

    for row in rows:
        call_flag        = _CALL_FLAG_MAP[row["callFlag"]]
        edge_override    = _EDGE_OVERRIDE_MAP[row["edgeOverride"]]
        edge_mode        = _BLIND_MODE_MAP[row["edgeMode"]]
        pipeline_blind   = _BLIND_MODE_MAP[row["pipelineBlind"]]
        direction        = _DIRECTION_MAP[row["direction"]]
        line_board_count = int(row["lineBoardCount"])
        auto_rule_active = row["autoRuleActive"].strip().lower() == "true"
        expect_seek   = row["expect_seek"].strip().lower() == "true"
        expect_motion = _BLIND_MODE_MAP[row["expect_motionMode"]]

        actual_seek, actual_motion = resolve_stop_at_line(
            call_flag,
            edge_override,
            edge_mode,
            pipeline_blind,
            direction,
            line_board_count,
            auto_rule_active,
        )

        assert (actual_seek, actual_motion) == (
            expect_seek,
            expect_motion,
        ), (
            f"resolve_stop_at_line mismatch for row "
            f"callFlag={row['callFlag']} edgeOverride={row['edgeOverride']} "
            f"edgeMode={row['edgeMode']} pipelineBlind={row['pipelineBlind']} "
            f"direction={row['direction']} lineBoardCount={row['lineBoardCount']} "
            f"autoRuleActive={row['autoRuleActive']}: "
            f"got (seek={actual_seek}, motionMode={actual_motion!r}) "
            f"but fixture expects (seek={expect_seek}, motionMode={expect_motion!r})"
        )


# ===========================================================================
# TC1 -- resolveEdgeDispatch resolver-vs-fixture parity (Python side)
#
# Verifies AC9 (#108): the Python ``resolve_edge_dispatch`` in runtime.py is a
# faithful transcription of the canonical edge-dispatch truth table frozen at
# ``tools/path_planner/tests/fixtures/edge_dispatch_truth_table.csv``. Every row
# is driven through the resolver and all SEVEN decision fields are compared to the
# fixture's ``expect_*`` columns -- enums and bools exactly, the commanded /
# overshoot floats within a tiny tolerance (the C++ resolver computes in float32
# and the CSV carries fixed 6-decimal text, so they are compared numerically,
# never as strings). A row-count assertion guards against a truncated or empty
# CSV vacuously passing.
#
# This is the Python half of the standalone cross-language CSV parity test for
# ``resolveEdgeDispatch`` (the C++ half is
# ``WRO2026/test/test_edge_dispatch_truth_table.cpp``), mirroring the #115
# ``resolveStopAtLine`` pattern (TC13 above). The generator
# ``tools/path_planner/tests/fixtures/gen_edge_dispatch_truth_table.py`` is an
# INDEPENDENT transcription -- it never calls either resolver -- so a drift in
# the table OR in either resolver surfaces as a hard failure, not a quiet
# tautological pass. Column names (not positional index) drive the lookup, so a
# future column reorder cannot silently desync the test from the fixture.
# ===========================================================================

_EDGE_DISPATCH_FIXTURE_PATH = (
    pathlib.Path(__file__).parent / "fixtures" / "edge_dispatch_truth_table.csv"
)

# 864 rows = LINE block 432 (2 destNodeType × 2 direction × 3 rawDistanceCm
#            × 3 lineMode × 2 projPassThrough × 2 departingTrimCm
#            × 3 approachingTrimCm)
#          + BLIND block 432 (2 direction × 2 seek × 3 motionMode
#            × 3 rawDistanceCm × 2 departingTrimCm × 3 approachingTrimCm
#            × 2 axleOffsetCm).
# The generator (tools/path_planner/tests/fixtures/gen_edge_dispatch_truth_table.py)
# is the single source of truth; this constant is a guard against a truncated or
# empty CSV vacuously passing, not a magic number.
_EDGE_DISPATCH_EXPECTED_ROW_COUNT = 864

# Float comparison tolerance (the plan's DIST_TOL): commanded-distance and
# overshoot are continuous, so they are compared numerically within 1e-4 cm.
_EDGE_DISPATCH_DIST_TOL = 1e-4

_EDGE_TYPE_MAP = {
    "LINE":  EDGE_LINE,
    "BLIND": EDGE_BLIND,
}

_LINE_MODE_MAP = {
    "NORMAL": SpeedTier.NORMAL,
    "SLOW":   SpeedTier.SLOW,
    "FAST":   SpeedTier.FAST,
}

# Drive-mode column spans the DEFAULT sentinel (line primitives) plus the three
# concrete blind bundles (STOP_AT_LINE / DEFAULT never appear as a resolved
# motion bundle, so they are intentionally absent -- a stray token aborts).
_DRIVE_MODE_MAP = {
    "DEFAULT":     BlindMode.DEFAULT,
    "SYNC_TSHAPE": BlindMode.SYNC_TSHAPE,
    "GYRO_TSHAPE": BlindMode.GYRO_TSHAPE,
    "RAW_FLAT":    BlindMode.RAW_FLAT,
}

_PRIMITIVE_MAP = {
    "LINE_FOLLOW_INTERSECTION": DispatchPrimitive.LINE_FOLLOW_INTERSECTION,
    "PROJECTION_FOLLOW":        DispatchPrimitive.PROJECTION_FOLLOW,
    "STOP_AT_LINE_SEEK":        DispatchPrimitive.STOP_AT_LINE_SEEK,
    "BLIND_DEAD_RECKON":        DispatchPrimitive.BLIND_DEAD_RECKON,
}

_REANCHOR_MAP = {
    "ON_INTERSECTION":          ReanchorKind.ON_INTERSECTION,
    "ON_DEAD_RECKONED_ADVANCE": ReanchorKind.ON_DEAD_RECKONED_ADVANCE,
}


def _edge_dispatch_bool(token: str) -> bool:
    """Parse a strict ``true`` / ``false`` CSV token, raising on anything else.

    Deliberately stricter than TC13's ``.lower() == "true"`` (which silently
    coerces a malformed cell to False): a bad token raises so fixture drift
    fails loudly, matching the C++ harness's ``toBool``.
    """
    if token == "true":
        return True
    if token == "false":
        return False
    raise ValueError(
        f"expected 'true' or 'false' edge-dispatch bool token, got {token!r}"
    )


def test_tc1_resolve_edge_dispatch_matches_fixture():
    """TC1 (Python): resolve_edge_dispatch agrees with the truth table for every row.

    Reads ``edge_dispatch_truth_table.csv`` by column name (DictReader, so column
    order is irrelevant) and calls ``resolve_edge_dispatch`` for each of the 864
    input combinations, asserting every one of the seven decision fields matches
    the fixture's ``expect_*`` columns -- ``primitive`` / ``drive_mode`` /
    ``reanchor`` enums and the three bools exactly, ``commanded_distance_cm`` /
    ``overshoot_cm`` within ``_EDGE_DISPATCH_DIST_TOL``.

    Satisfies AC9 / TC1 (#108): the Python transcription cannot silently drift
    from the canonical single-source truth table, and (with the C++ mirror) the
    two resolvers cannot drift from each other.
    """
    assert _EDGE_DISPATCH_FIXTURE_PATH.exists(), (
        f"Edge-dispatch truth-table fixture not found at "
        f"{_EDGE_DISPATCH_FIXTURE_PATH}. Run gen_edge_dispatch_truth_table.py "
        "to generate it before running TC1."
    )

    rows = []
    with _EDGE_DISPATCH_FIXTURE_PATH.open(newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(row)

    assert len(rows) == _EDGE_DISPATCH_EXPECTED_ROW_COUNT, (
        f"Expected {_EDGE_DISPATCH_EXPECTED_ROW_COUNT} data rows in the "
        f"edge-dispatch truth-table fixture but found {len(rows)}. A truncated "
        "or empty CSV would produce vacuous passing without this guard."
    )

    for row in rows:
        query = EdgeDispatchQuery(
            edge_type=_EDGE_TYPE_MAP[row["edgeType"]],
            dest_is_projection=(row["destNodeType"] == "PROJECTION"),
            direction=_DIRECTION_MAP[row["direction"]],
            raw_distance_cm=float(row["rawDistanceCm"]),
            line_mode=_LINE_MODE_MAP[row["lineMode"]],
            stop_at_line=(
                _edge_dispatch_bool(row["seek"]),
                _DRIVE_MODE_MAP[row["motionMode"]],
            ),
            proj_pass_through=_edge_dispatch_bool(row["projPassThrough"]),
            departing_trim_cm=float(row["departingTrimCm"]),
            approaching_trim_cm=float(row["approachingTrimCm"]),
            axle_offset_cm=float(row["axleOffsetCm"]),
            min_drive_cm=float(row["minDriveCm"]),
        )

        decision = resolve_edge_dispatch(query)

        expect_primitive = _PRIMITIVE_MAP[row["expect_primitive"]]
        expect_reverse = _edge_dispatch_bool(row["expect_reverse"])
        expect_drive_mode = _DRIVE_MODE_MAP[row["expect_driveMode"]]
        expect_commanded = float(row["expect_commandedDistanceCm"])
        expect_overshoot = float(row["expect_overshootCm"])
        expect_reanchor = _REANCHOR_MAP[row["expect_reanchor"]]
        expect_brake = _edge_dispatch_bool(row["expect_brakeOnGate"])

        ctx = (
            f"edgeType={row['edgeType']} destNodeType={row['destNodeType']} "
            f"direction={row['direction']} rawDistanceCm={row['rawDistanceCm']} "
            f"lineMode={row['lineMode']} seek={row['seek']} "
            f"motionMode={row['motionMode']} projPassThrough={row['projPassThrough']} "
            f"departingTrimCm={row['departingTrimCm']} "
            f"approachingTrimCm={row['approachingTrimCm']} "
            f"axleOffsetCm={row['axleOffsetCm']} minDriveCm={row['minDriveCm']}"
        )

        assert decision.primitive == expect_primitive, (
            f"primitive mismatch for {ctx}: got {decision.primitive!r}, "
            f"expected {expect_primitive!r}"
        )
        assert decision.reverse == expect_reverse, (
            f"reverse mismatch for {ctx}: got {decision.reverse}, "
            f"expected {expect_reverse}"
        )
        assert decision.drive_mode == expect_drive_mode, (
            f"drive_mode mismatch for {ctx}: got {decision.drive_mode!r}, "
            f"expected {expect_drive_mode!r}"
        )
        assert abs(decision.commanded_distance_cm - expect_commanded) < _EDGE_DISPATCH_DIST_TOL, (
            f"commanded_distance_cm mismatch for {ctx}: got "
            f"{decision.commanded_distance_cm}, expected {expect_commanded}"
        )
        assert abs(decision.overshoot_cm - expect_overshoot) < _EDGE_DISPATCH_DIST_TOL, (
            f"overshoot_cm mismatch for {ctx}: got {decision.overshoot_cm}, "
            f"expected {expect_overshoot}"
        )
        assert decision.reanchor == expect_reanchor, (
            f"reanchor mismatch for {ctx}: got {decision.reanchor!r}, "
            f"expected {expect_reanchor!r}"
        )
        assert decision.brake_on_gate == expect_brake, (
            f"brake_on_gate mismatch for {ctx}: got {decision.brake_on_gate}, "
            f"expected {expect_brake}"
        )


# ===========================================================================
# TC7 -- _resolve_override / resolve_speed accept bare string mode names
#
# Finding #41: IntEnum(str) raises ValueError for 'NORMAL'; only IntEnum['NORMAL']
# works for name-based lookup.  After the fix _resolve_override must accept
# string tags via enum_cls[name], matching header_writer._override_enum_value.
# ===========================================================================


def test_resolve_speed_accepts_bare_string_name():
    """TC7: an edge with speed='NORMAL' (bare string) resolves without ValueError.

    Before finding #41 was fixed, ``_resolve_override`` would call
    ``SpeedTier('NORMAL')`` which raises ValueError because SpeedTier is an
    IntEnum and only accepts integer values for value-based lookup.
    The fix adds a str branch that uses ``SpeedTier['NORMAL']`` (name lookup)
    to match the handling in header_writer._override_enum_value.

    Asserts:
    - ``resolve_speed({'speed': 'NORMAL'})`` returns ``SpeedTier.NORMAL``
      without raising.
    - A string 'DEFAULT' still collapses to the pipeline default (same as the enum
      sentinel), so the DEFAULT fallback path works for strings too.
    - An unknown string name falls back to the pipeline default rather than crashing.
    """
    # Known concrete string name -> correct enum member, no ValueError.
    edge_str_normal = {"speed": "NORMAL"}
    result = resolve_speed(edge_str_normal)
    assert result == SpeedTier.NORMAL, (
        f"Expected SpeedTier.NORMAL for speed='NORMAL'; got {result!r}"
    )

    # 'DEFAULT' string -> falls back to pipeline default (same as enum DEFAULT).
    edge_str_default = {"speed": "DEFAULT"}
    assert resolve_speed(edge_str_default) == PIPELINE_CONFIG.speed

    # Unknown string -> falls back to pipeline default without raising.
    edge_str_unknown = {"speed": "NONEXISTENT_MODE"}
    assert resolve_speed(edge_str_unknown) == PIPELINE_CONFIG.speed


def test_resolve_speed_string_name_end_to_end_via_sim():
    """TC7 (end-to-end): go_to over an EDGE_LINE with speed='NORMAL' completes.

    Builds a 2-node graph where the A->B edge carries ``speed='NORMAL'``
    as a bare string (the form graph_builder forwards verbatim when field_data
    uses string tags and validation is skipped, as in test fixtures).  Confirms
    the sim's go_to path does not crash and emits the standard LINE primitives.
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (30.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge(
        "A", "B",
        type=EDGE_LINE,
        distance_cm=30.0,
        heading_deg=0.0,
        speed="NORMAL",
    )
    g.add_edge(
        "B", "A",
        type=EDGE_LINE,
        distance_cm=30.0,
        heading_deg=180.0,
    )
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    # Must not raise ValueError; must emit the standard FOLLOW_INT + OVERSHOOT pair.
    prims = sim.go_to("B")

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM),
    ]
    assert sim.localization.current_node() == "B"


# ===========================================================================
# Per-turn single-wheel pivot compensation (ADR-036, issue #44) -- sim parity
# (TC16 - TC21)
#
# These replace the per-edge ADR-035 pivot tests. A pivot is now a property of a
# 3-node turn ``(u, v, w)`` declared in ``field_data.PIVOT_TURNS`` -- it fires
# only when the route takes that turn. The sim resolves each fixture's pivots
# from ``field_data.PIVOT_TURNS`` against its own graph
# (``build_pivot_turns(self._graph, pivot_turns=...)``) and applies the
# departing + approaching trims via the ``_pending_pivot`` latch in
# ``_walk_hops`` -- a byte-for-byte mirror of ``PathTracker._onPlanNextHop``.
#
# Geometry note (load-bearing, carried over from the ADR-035 tests):
# :class:`VirtualChassis` models a turn as a pure in-place rotation (zero
# translation), so for a PIVOT hop the trimmed drives leave the point-model
# chassis ``delta`` short on the incoming leg AND ``delta`` short on the outgoing
# leg. The firmware closes that residual with the pivot maneuver's OWN physical
# translation (``delta`` along the inbound heading to reach ``v``, then ``delta``
# along the outbound heading of over-travel past ``v``), which the point-sim does
# not replay. So the drift-is-zero acceptance adds that documented physical pivot
# translation back to the sim's point position -- once per pivot the route took
# -- and checks the axle lands on the node within ``_POSITION_TOL_CM``. The
# load-bearing firmware-parity contract -- the adjacent drives trimmed by
# ``delta`` -- is asserted DIRECTLY on the emitted primitive payloads. We never
# re-introduce a per-junction offset into the sim and never relax
# ``_POSITION_TOL_CM``.
#
# Every expected ``delta`` is sourced from
# ``compute_pivot_compensation(theta, track_cm)`` -- the SAME closed form the
# planner bakes into each ``PivotTurnRecord`` -- never a second hard-coded
# formula (issue #44, TC2 discipline).
#
# Fixtures: tools/path_planner/tests/fixtures/pivot_field_data.py
# Code under test: tools/path_planner/sim/runtime.py -- PathTrackerSim
#   ._walk_hops (per-turn latch/carry) / ._pivot_trim_distance / .go_to /
#   .step_through.
# ===========================================================================

# Competition chassis track (cm): Config::TRACK_MM = 180 mm. The fixtures and the
# sim both resolve this from the firmware header (parse_track_cm_from_config), so
# the trims are byte-identical to the emitter's PIVOT_TURNS records.
_PIVOT_TRACK_CM: float = parse_track_cm_from_config()

# Turn magnitudes baked into the pivot fixtures (see pivot_field_data.py).
_PIVOT_TURN_90_DEG: float = 90.0     # dead-reckon + chain fixtures (delta = 9.0 cm)
_PIVOT_TURN_70_DEG: float = 70.0     # LINE-incoming fixture (delta ~= 6.30 cm)
_PIVOT_TURN_120_DEG: float = 120.0   # auto-REAR fixture (delta ~= 15.59 cm)


def _make_pivot_sim(field):
    """Return a zeroed ``(sim, chassis, graph, next_hop)`` for a pivot fixture.

    Builds the directed graph + all-pairs next-hop table from the fixture field
    and constructs a :class:`PathTrackerSim` over them. The sim resolves the
    field's ``PIVOT_TURNS`` against ``graph`` in its constructor, so the trims it
    applies match the geometry the validator checked. The orphan-node
    UserWarning a disconnected pivot cluster can raise is suppressed (the
    fixtures are deliberately small single-route fields).
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        graph, next_hop = pivot_field_data.build_graph_and_next_hop(field)
    chassis = VirtualChassis.from_field_data(field)
    sim = PathTrackerSim(field, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis, graph, next_hop


def _pivot_corrected_position(chassis, delta, pivot_legs):
    """Return the sim point-position plus each pivot's physical translation.

    The point-model :class:`VirtualChassis` rotates in place, so every pivot hop
    the route took leaves it ``delta`` short along that pivot's inbound heading
    and ``delta`` short along its outbound heading. The firmware's pivot maneuver
    physically translates the axle by exactly that. Add it back -- once per pivot
    the route traversed -- to recover the axle's true landing for the drift
    cross-check.

    :param chassis: the sim chassis after the route completed.
    :param delta: the per-pivot over-travel (all fixtures use one turn magnitude
        per route, so a single ``delta`` covers every pivot on the route).
    :param pivot_legs: list of ``(inbound_heading_deg, outbound_heading_deg)``
        pairs, one per pivot node the route turned through, in route order.
    :return: the drift-corrected ``(x_cm, y_cm)`` axle position.
    """
    px, py = chassis.x_cm, chassis.y_cm
    for hdg_in_deg, hdg_out_deg in pivot_legs:
        px += delta * math.cos(math.radians(hdg_in_deg))
        px += delta * math.cos(math.radians(hdg_out_deg))
        py += delta * math.sin(math.radians(hdg_in_deg))
        py += delta * math.sin(math.radians(hdg_out_deg))
    return px, py


def test_pivot_fixtures_are_legal_fields():
    """Every pivot fixture passes build_graph + validate (the legality gate).

    The validator now enforces the per-turn pivot regime / angle / budget rules
    (T5), so a fixture that passes ``validate`` is a legal field by
    construction. The orphan-node UserWarning a small disconnected fixture may
    raise is suppressed; ``validate`` itself must not raise.
    """
    from tools.path_planner.validator import validate

    fields = pivot_field_data.all_fields()
    assert len(fields) == 6   # guard against an empty sweep vacuously passing
    for field in fields:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            validate(field)               # must not raise
            build_graph(field)            # must build cleanly


# ---------------------------------------------------------------------------
# TC16 -- dead-reckon pivot reached and left by dead-reckon: both drives trimmed
# ---------------------------------------------------------------------------


def test_pivot_tc16_dead_reckon_incoming_and_outgoing_both_trimmed():
    """TC16: a BLIND->BLIND pivot trims BOTH the approach and the departure; drift ~ 0.

    On the route ``U -> V -> W`` the declared pivot ``(U, V, W)`` (90 deg right
    turn) is reached by dead-reckon (the BLIND approach ``U -> V``) and left by
    dead-reckon (the BLIND departure ``V -> W``). The latch in ``_walk_hops``:

    * trims the approach drive to ``L_uv - delta`` (``_approaching_pivot``),
    * carries ``delta`` one hop, and
    * trims the departure drive to ``L_vw - delta`` (``_departing_pivot``).

    Both emitted ``ACT_FORWARD`` payloads carry the trimmed distances (the
    load-bearing per-turn dispatch contract). The point-sim lands ``delta`` short
    on each leg; adding the pivot maneuver's physical translation recovers node
    ``W`` with ZERO residual within ``_POSITION_TOL_CM``.
    """
    field = pivot_field_data.make_dead_reckon_pivot_field()
    sim, chassis, _graph, _next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_PIVOT_TURN_90_DEG, _PIVOT_TRACK_CM)

    prims = sim.go_to("W")

    # Both adjacent dead-reckon drives are trimmed by delta (the contract).
    assert prims == [
        (ACT_FORWARD, pytest.approx(60.0 - delta, abs=1e-9)),    # U->V approach trim
        (ACT_TURN_REL, pytest.approx(-_PIVOT_TURN_90_DEG, abs=1e-9)),
        (ACT_FORWARD, pytest.approx(60.0 - delta, abs=1e-9)),    # V->W departure trim
    ]

    # Drift ~ 0: the point-sim is delta short on each leg of the single pivot at
    # V (inbound U->V heading 90 deg, outbound V->W heading 0 deg); the pivot's
    # own physical translation closes it, landing the axle on W.
    px, py = _pivot_corrected_position(chassis, delta, [(90.0, 0.0)])
    wx, wy = field.NODES["W"]
    assert math.hypot(px - wx, py - wy) < _POSITION_TOL_CM
    assert sim.localization.current_node() == "W"


# ---------------------------------------------------------------------------
# TC17 -- a route through the pivot node that does NOT take the declared turn
# ---------------------------------------------------------------------------


def test_pivot_tc17_non_pivot_route_through_pivot_node_drives_full_distance():
    """TC17: routing through pivot node V WITHOUT taking ``(U, V, W)`` drives FULL distance.

    The same field declares the pivot ``(U, V, W)``, but node ``V`` has a SECOND
    continuation ``V -> Y`` that is collinear with ``U -> V`` (turn 0 deg) and is
    NOT a declared pivot. Routing ``U -> Y`` passes through the pivot node ``V``
    without taking the ``(U, V, W)`` turn, so the lookahead ``find_pivot_turn(U,
    V, Y)`` misses on the approach hop and ``find_pivot_turn(V, Y, None)`` misses
    on the departure: NO trim is applied and both BLIND hops drive their FULL
    60 cm. This proves the per-turn pivot "fires only when the route takes that
    turn" guarantee at the sim level.
    """
    field = pivot_field_data.make_dead_reckon_pivot_field()
    sim, chassis, _graph, next_hop = _make_pivot_sim(field)

    # Sanity: the route really does pass through the pivot node V.
    assert walk_next_hop(next_hop, "U", "Y") == ["U", "V", "Y"]

    prims = sim.go_to("Y")

    # Full distance on BOTH hops -- no delta subtracted on a non-pivot route, and
    # the collinear V->Y continuation needs no ACT_TURN_REL. ADR-059: U->V->Y is a
    # collinear BLIND run of 2 hops, so the sim chains it into ONE continuous
    # motion -- hop 0 carries the resolved (start=TSHAPE_START_SPEED, end=cruise)
    # + chain=True, a blind_pass marks the non-braking node V, and the final hop
    # brakes (end 0). The DISTANCES stay the full 60 cm (the TC17 no-trim intent);
    # only the velocity endpoints + the pass marker are added by the chain.
    assert prims == [
        (ACT_FORWARD, pytest.approx(60.0, abs=1e-9),
         _runtime.TSHAPE_START_SPEED, _runtime.SPEED_NORMAL, True),
        (ACT_BLIND_PASS, "V"),
        (ACT_FORWARD, pytest.approx(60.0, abs=1e-9),
         _runtime.SPEED_NORMAL, 0, True),
    ]
    yx, yy = field.NODES["Y"]
    assert math.hypot(chassis.x_cm - yx, chassis.y_cm - yy) < _POSITION_TOL_CM
    assert sim.localization.current_node() == "Y"


# ---------------------------------------------------------------------------
# TC18 -- LINE-incoming pivot: overshoot reduced by delta + outgoing trimmed
# ---------------------------------------------------------------------------


def test_pivot_tc18_line_incoming_overshoot_reduced_and_outgoing_trimmed():
    """TC18: a LINE-incoming pivot reduces the overshoot advance by delta and trims the outgoing hop.

    Issue #64 (T12 fixture migration): the validator now rejects an on-line START
    node, so ``make_line_incoming_pivot_field`` prepends a blind spawn ``S0`` on a
    BLIND edge into ``L0``. The route to ``L2`` is therefore ``S0 -> L0 -> L1 ->
    L2``, and the FIRST emitted primitive is the leading ``S0 -> L0`` blind hop
    ``(ACT_FORWARD, 60.0)`` -- NOT the pivot's LINE-incoming ``ACT_FOLLOW_INT``.
    The pivot ``(L0, L1, L2)`` is unchanged: it arrives over the LINE edge
    ``L0 -> L1`` (re-anchors on the junction line, so it cannot stop short on its
    approach) and leaves over the BLIND edge ``L1 -> L2``. With a 70 deg turn at
    track 18 cm, ``delta ~= 6.30`` cm, within the FRONT overshoot budget
    (12.02 cm). The latch therefore:

    * reduces the LINE_OVERSHOOT advance to ``LINE_SENSOR_AXLE_OFFSET_CM - delta``
      (the axle settles delta short of L1 -- the ``_approaching_pivot`` read on
      the LINE-incoming hop), and
    * carries the same ``delta`` to the outgoing ``L1 -> L2`` BLIND hop, trimming
      it to ``L_vw - delta`` (the next hop's ``_departing_pivot``).

    Both adjacent corrections are asserted on the emitted primitive payloads.
    """
    field = pivot_field_data.make_line_incoming_pivot_field()
    sim, chassis, _graph, _next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_PIVOT_TURN_70_DEG, _PIVOT_TRACK_CM)
    # Within the FRONT overshoot budget (single-sourced) -- the validator accepts
    # this; a 90 deg LINE-incoming pivot (delta = 9 < 12.02) would also fit, but
    # 70 deg keeps the reduced overshoot comfortably positive.
    assert delta <= LINE_SENSOR_AXLE_OFFSET_CM

    prims = sim.go_to("L2")

    # Leading S0 -> L0 blind hop (60 cm, no pivot, heading 0 so no turn), then the
    # LINE-incoming pivot: FOLLOW_INT then a reduced LINE_OVERSHOOT (offset -
    # delta), the turn, and the outgoing BLIND hop trimmed by delta.
    assert prims == [
        (ACT_FORWARD, pytest.approx(60.0, abs=1e-9)),
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, pytest.approx(LINE_SENSOR_AXLE_OFFSET_CM - delta, abs=1e-9)),
        (ACT_TURN_REL, pytest.approx(_PIVOT_TURN_70_DEG, abs=1e-9)),
        (ACT_FORWARD, pytest.approx(40.0 - delta, abs=1e-9)),
    ]
    assert sim.localization.current_node() == "L2"


def test_pivot_tc18_non_pivot_line_traversal_drives_full_overshoot():
    """TC18 (control): the reverse LINE traversal of L1 (non-pivot) rolls the FULL overshoot.

    The pivot ``(L0, L1, L2)`` is keyed on the forward turn only. Driving the
    reverse LINE edge ``L1 -> L0`` -- a non-pivot traversal through the same line
    node L1 -- rolls the FULL ``LINE_SENSOR_AXLE_OFFSET_CM`` overshoot with no
    delta reduction, proving a non-pivot route through the same line node is
    unaffected.

    Issue #64: the validator rejects an on-line START, so we cannot start at the
    line node ``L1`` directly. A blind spawn ``Z`` is added on a BLIND edge into
    ``L1``, placed collinear with the ``L1 -> L0`` line (so ``Z -> L1`` heads the
    same -x direction, no turn) -- the route ``Z -> L1 -> L0`` then drives the
    reverse non-pivot LINE traversal as the second hop after the leading blind hop.
    """
    field = pivot_field_data.make_line_incoming_pivot_field()
    # Add a blind spawn Z collinear with L1->L0 (both heading 180 / -x), on a
    # BLIND edge into L1, and start there facing -x.
    field.NODES = {**field.NODES, "Z": (120.0, 0.0)}
    field.NODE_TYPES = {**field.NODE_TYPES, "Z": "blind"}
    field.BLIND_EDGES = [*field.BLIND_EDGES, ("Z", "L1")]
    field.START_NODE = "Z"
    field.START_HEADING_DEG = 180.0
    sim, _chassis, _graph, _next_hop = _make_pivot_sim(field)

    prims = sim.go_to("L0")

    # Leading Z -> L1 blind hop (60 cm, heading 180 = collinear so no turn), then
    # the reverse non-pivot LINE traversal L1 -> L0 with the FULL overshoot.
    assert prims == [
        (ACT_FORWARD, pytest.approx(60.0, abs=1e-9)),
        (ACT_FOLLOW_INT, None),
        (ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM),
    ]
    assert sim.localization.current_node() == "L0"


# ---------------------------------------------------------------------------
# TC19 -- chained fitting pivots u->v->w->x land with ~0 residual
# ---------------------------------------------------------------------------


def test_pivot_tc19_chained_fitting_pivots_land_with_zero_residual():
    """TC19: the fitting chain ``U -> V -> W -> X`` (pivots at V and W) lands with ~0 residual.

    Two declared pivots ``(U, V, W)`` and ``(V, W, X)`` (each a 90 deg right
    turn, delta = 9 cm) chain on the shared dead-reckoned edge ``V -> W``: the
    snapshot-before-overwrite latch makes the departing V-pivot trim and the
    approaching W-pivot trim two DISTINCT resolved records, so the shared edge
    nets ``length - delta_V - delta_W`` = ``60 - 9 - 9 = 42`` cm. The outer hops
    each net ``60 - 9 = 51`` cm. Adding each pivot's physical translation lands
    the axle on ``X`` with ZERO residual.
    """
    field = pivot_field_data.make_chained_pivot_field()
    sim, chassis, _graph, _next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_PIVOT_TURN_90_DEG, _PIVOT_TRACK_CM)

    prims = sim.go_to("X")

    # U->V trimmed by delta; the SHARED V->W trimmed by delta on BOTH sides
    # (length - 2*delta); W->X trimmed by delta.
    assert prims == [
        (ACT_FORWARD, pytest.approx(60.0 - delta, abs=1e-9)),         # U->V (approach @ V)
        (ACT_TURN_REL, pytest.approx(-_PIVOT_TURN_90_DEG, abs=1e-9)),
        (ACT_FORWARD, pytest.approx(60.0 - 2.0 * delta, abs=1e-9)),   # V->W (depart V + approach W)
        (ACT_TURN_REL, pytest.approx(-_PIVOT_TURN_90_DEG, abs=1e-9)),
        (ACT_FORWARD, pytest.approx(60.0 - delta, abs=1e-9)),         # W->X (depart @ W)
    ]

    # Drift ~ 0: add each pivot's physical translation. Pivot @ V: inbound U->V
    # 90 deg, outbound V->W 0 deg. Pivot @ W: inbound V->W 0 deg, outbound W->X
    # 270 deg.
    px, py = _pivot_corrected_position(chassis, delta, [(90.0, 0.0), (0.0, 270.0)])
    xx, xy = field.NODES["X"]
    assert math.hypot(px - xx, py - xy) < _POSITION_TOL_CM
    assert sim.localization.current_node() == "X"


# ---------------------------------------------------------------------------
# TC20 -- a pivot hop the auto heuristic would resolve REAR is forced FRONT
# ---------------------------------------------------------------------------


def test_pivot_tc20_auto_rear_pivot_hop_is_forced_front_and_fires():
    """TC20: an obtuse pivot whose departure hop auto-resolves REAR is forced FRONT.

    On the route ``P -> Q -> R`` the pivot ``(P, Q, R)`` is an obtuse 120 deg
    turn. After the FRONT approach ``P -> Q`` the chassis heading is 0 deg and the
    departure edge ``Q -> R`` heads 120 deg, so the bidirectional auto heuristic
    (|delta| = 120 deg > the 90 deg threshold, 2-board chassis) would resolve the
    departure REAR. The FRONT-only pivot override in ``_walk_hops`` must flip it
    back to FRONT and STILL fire the pivot trim:

    * the departure ``Q -> R`` emits a FRONT ``ACT_FORWARD`` (trimmed by delta),
      NEVER ``ACT_BLIND_REVERSE`` -- no silent downgrade to a counter-rotation;
    * the approach ``P -> Q`` is also trimmed (it drives into the pivot node).

    This is the load-bearing FRONT-only sim case. The skip+trace backstop stays a
    should-be-impossible path.
    """
    field = pivot_field_data.make_auto_rear_pivot_field()
    sim, chassis, graph, _next_hop = _make_pivot_sim(field)
    delta = compute_pivot_compensation(_PIVOT_TURN_120_DEG, _PIVOT_TRACK_CM)

    # Control: the SAME departure edge, resolved WITHOUT a pivot, auto-resolves
    # REAR -- so the force-FRONT below is genuinely load-bearing.
    qr_heading = float(graph.edges["Q", "R"]["heading_deg"])
    auto_dir = resolve_direction(
        edge_dir_override=DirectionMode.DEFAULT,
        pipeline_default=DirectionMode.DEFAULT,
        builder_arrival=Direction.AUTO,
        builder_route=Direction.AUTO,
        current_heading=0.0,            # chassis heading after the FRONT approach P->Q
        edge_heading=qr_heading,
        is_last_hop=True,
        bidirectional_enabled=True,
        line_board_count=2,
    )
    assert auto_dir == Direction.REAR, (
        "fixture geometry no longer makes Q->R auto-resolve REAR; the force-FRONT "
        "assertion below would be vacuous"
    )

    prims = sim.go_to("R")
    act_tags = [p[0] for p in prims]

    # Force-FRONT: the departure emits FRONT ACT_FORWARD, never ACT_BLIND_REVERSE.
    assert ACT_BLIND_REVERSE not in act_tags, (
        f"pivot departure hop was NOT forced FRONT (silent REAR downgrade); got {act_tags}"
    )

    # Both legs trimmed by delta; the obtuse pivot turn is emitted as ACT_TURN_REL.
    assert prims == [
        (ACT_FORWARD, pytest.approx(40.0 - delta, abs=1e-9)),    # P->Q approach trim (FRONT)
        (ACT_TURN_REL, pytest.approx(_PIVOT_TURN_120_DEG, abs=1e-9)),
        (ACT_FORWARD, pytest.approx(40.0 - delta, abs=1e-9)),    # Q->R departure trim (forced FRONT)
    ]
    assert sim.localization.current_node() == "R"


# ---------------------------------------------------------------------------
# TC21 -- go_to and step_through emit identical primitives via the shared helper
# ---------------------------------------------------------------------------
#
# Both public entry points walk NEXT_HOP through the single private per-hop
# producer ``_walk_hops``; ``go_to`` collects the flat primitive list while
# ``step_through`` pairs each primitive with a chassis snapshot. This pins the
# equivalence the per-turn pivot dispatch relies on: the latch/carry threading
# lands in one site, so the two streams must stay byte-identical -- including
# across the pivot path.


def _step_through_primitives(sim, goal, **kwargs):
    """Return only the primitive halves of ``sim.step_through(goal)``'s output."""
    return [primitive for primitive, _snapshot in sim.step_through(goal, **kwargs)]


def test_tc21_goto_step_through_emit_identical_primitives_multi_hop(
    toy_field, toy_graph_and_table
):
    """TC21: go_to and step_through emit byte-identical primitives on a multi-hop route.

    The toy route ``A -> B -> G`` spans a LINE hop (with its LINE_OVERSHOOT
    step), a heading turn, and a BLIND hop -- enough edge variety that any
    divergence between the two entry points' per-hop dispatch would surface.
    Two independent sims drive the same route so the walks do not share mutable
    chassis / localization state; the primitive streams must match exactly.
    """
    graph, next_hop = toy_graph_and_table

    chassis_a = VirtualChassis.from_field_data(toy_field)
    sim_a = PathTrackerSim(toy_field, graph, next_hop, chassis_a)
    sim_a.localization.zero_heading_from_current()

    chassis_b = VirtualChassis.from_field_data(toy_field)
    sim_b = PathTrackerSim(toy_field, graph, next_hop, chassis_b)
    sim_b.localization.zero_heading_from_current()

    go_to_prims = sim_a.go_to("G")
    step_prims = _step_through_primitives(sim_b, "G")

    # The route is genuinely multi-hop (guards against a 0/1-hop fixture
    # silently passing a trivial equality).
    assert len(go_to_prims) >= 3
    assert go_to_prims == step_prims
    assert sim_a.localization.current_node() == "G"
    assert sim_b.localization.current_node() == "G"


def test_tc21_goto_step_through_identical_on_pivot_chain():
    """TC21: parity holds across the per-turn pivot latch/carry path.

    The chained fixture ``U -> V -> W -> X`` exercises the exact ``_pending_pivot``
    snapshot-before-overwrite latch + the departing/approaching trim folding on
    the shared edge -- the single site the per-turn pivot dispatch lives in. Two
    independent sims drive the route; the primitive streams must be byte-identical
    even with two chained pivots active.
    """
    field_go = pivot_field_data.make_chained_pivot_field()
    field_step = pivot_field_data.make_chained_pivot_field()
    sim_go, _chassis_go, _g_go, _nh_go = _make_pivot_sim(field_go)
    sim_step, _chassis_step, _g_step, _nh_step = _make_pivot_sim(field_step)

    go_to_prims = sim_go.go_to("X")
    step_prims = _step_through_primitives(sim_step, "X")

    # Multi-hop with the pivot latch active on the shared edge (guards a trivial
    # equality).
    assert len(go_to_prims) >= 5
    assert go_to_prims == step_prims
    assert sim_go.localization.current_node() == "X"
    assert sim_step.localization.current_node() == "X"


# ===========================================================================
# Operator offset + return-to-node sim mirror (issue #36, PR1b -- T11)
#
# Covers: AC17 pillar 1, TC18a parity. Mirrors the two PR1b firmware
# behaviors -- (1) operator-offset accumulation while not navigating
# (WRO2026/Localization.cpp _integrateOffset), and (2) the return-to-node
# prologue that drives the chassis back onto its anchored node before a go_to
# plans (WRO2026/PathTracker.cpp _evaluateReturnGate / _dispatchReturnMove,
# WRO2026/ReturnToNodePolicy.h resolveReturnAction).
#
# Code under test: tools/path_planner/sim/runtime.py --
#   LocalizationSim (offset state + accumulate_operator_move + clear_offset +
#   set_nav_active + node_type), PathTrackerSim.operator_move /
#   _dispatch_return_move / go_to prologue, resolve_return_action,
#   Offset2D, ReturnAction, OffMapGate, OffMapFaultError, and the
#   ACT_RETURN_STOP_AT_LINE / ACT_RETURN_FORWARD primitive tags.
# ===========================================================================


def _typed_two_node_field(
    node_type_b: str,
    *,
    distance_cm: float = 50.0,
    start_heading_deg: float = 0.0,
):
    """Return a (field, graph, next_hop) triple for a 2-node A->B LINE edge.

    Node ``A`` is always LINE (the start); ``B`` carries @p node_type_b so a
    return-to-node test can anchor the chassis on either a LINE-backed or a
    BLIND-only node and exercise the matching re-anchor path. The edge is
    EDGE_LINE in both cases -- the node *kind* (not the edge kind) selects the
    return dispatch, mirroring ``FieldMap::getNodeType`` in
    ``PathTracker::_dispatchReturnMove``.
    """
    nodes = {"A": (0.0, 0.0), "B": (distance_cm, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        NODE_TYPES={"A": "line", "B": node_type_b},
        START_NODE="A",
        START_HEADING_DEG=start_heading_deg,
        PIVOT_TURNS=[],
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, distance_cm, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    return fd, graph, next_hop


def _make_typed_sim(fd, graph, next_hop):
    """Construct and zero a PathTrackerSim over the given fixture triple."""
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis


# ---------------------------------------------------------------------------
# resolve_return_action parity -- NONE / RETURN / OFF_MAP_* boundaries
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "mag_cm, gate, expected",
    [
        # Case 1: mag <= trigger -> NONE (trigger boundary CLOSED at 3.0).
        (0.0, OffMapGate.FAULT, ReturnAction.NONE),
        (RETURN_TRIGGER_THRESHOLD_CM, OffMapGate.FAULT, ReturnAction.NONE),
        # Case 2: trigger < mag <= cap -> RETURN (cap boundary CLOSED at 50.0).
        (RETURN_TRIGGER_THRESHOLD_CM + 1e-6, OffMapGate.FAULT, ReturnAction.RETURN),
        (10.0, OffMapGate.FAULT, ReturnAction.RETURN),
        (RETURN_DISTANCE_CAP_CM, OffMapGate.FAULT, ReturnAction.RETURN),
        # Case 3/4: mag > cap -> off-map gate (cap boundary OPEN above 50.0).
        (RETURN_DISTANCE_CAP_CM + 1e-6, OffMapGate.FAULT, ReturnAction.OFF_MAP_FAULT),
        (100.0, OffMapGate.FAULT, ReturnAction.OFF_MAP_FAULT),
        (100.0, OffMapGate.BEST_EFFORT, ReturnAction.OFF_MAP_PROCEED),
    ],
    ids=[
        "zero_NONE",
        "at_trigger_NONE",
        "just_over_trigger_RETURN",
        "mid_RETURN",
        "at_cap_RETURN",
        "just_over_cap_FAULT",
        "over_cap_FAULT",
        "over_cap_BEST_EFFORT_PROCEED",
    ],
)
def test_resolve_return_action_boundaries(mag_cm, gate, expected):
    """resolve_return_action matches the C++ resolveReturnAction four-case rule.

    Boundary semantics (mirror of ``WRO2026/ReturnToNodePolicy.h``): the
    trigger test is ``<=`` (CLOSED -- mag == trigger is NONE) and the cap test
    is ``>`` (OPEN -- mag == cap is still RETURN, only mag strictly above the
    cap fires the off-map gate). The cap test runs BEFORE deciding RETURN so an
    over-cap offset can never fall into a plain return.
    """
    action = resolve_return_action(
        mag_cm,
        RETURN_TRIGGER_THRESHOLD_CM,
        RETURN_DISTANCE_CAP_CM,
        gate,
    )
    assert action == expected


def test_resolve_return_action_constants_mirror_config():
    """The hand-mirrored Config constants carry the firmware's ConfigLocalization.h values.

    Guards the "MUST mirror Config::X" contract: a divergence here means the
    sim's gate would fire at a different offset than the robot's, breaking
    sim/firmware parity. Values per WRO2026/ConfigLocalization.h.
    """
    assert RETURN_TRIGGER_THRESHOLD_CM == pytest.approx(3.0)
    assert RETURN_DISTANCE_CAP_CM == pytest.approx(50.0)
    assert OFFSET_TRANSLATION_DEADBAND_CM == pytest.approx(0.2)
    assert OFFSET_ROTATION_DEADBAND_DEG == pytest.approx(0.5)
    # Off-map gate default is FAULT (the safer firmware default).
    assert OFF_MAP_GATE == OffMapGate.FAULT


# ---------------------------------------------------------------------------
# Offset accumulation (AC1 / AC2 / AC2b / AC3 mirror) -- pillar 1
# ---------------------------------------------------------------------------


def test_operator_move_while_idle_grows_offset_by_magnitude():
    """A raw operator move while nav-inactive grows the offset ~by the magnitude.

    Mirror of AC1: after a raw ``Drive.driveDist(d)`` while PathTracker is idle,
    ``getOffsetFromNode()`` reports a 2D offset whose magnitude ~= d and the
    current node is unchanged. Here a forward push of 15 cm along +x leaves an
    offset of (15, 0, 0) and the node still 'A'.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, chassis = _make_typed_sim(fd, graph, next_hop)

    sim.operator_move(15.0)

    off = sim.localization.get_offset_from_node()
    assert off.x == pytest.approx(15.0, abs=1e-9)
    assert off.y == pytest.approx(0.0, abs=1e-9)
    assert off.magnitude_cm() == pytest.approx(15.0, abs=1e-9)
    # Node id is unchanged -- the operator move does not advance graph position.
    assert sim.localization.current_node() == "A"
    # The chassis actually moved (the sim models the hand-drive, not just the
    # bookkeeping).
    assert chassis.x_cm == pytest.approx(15.0, abs=1e-9)


def test_operator_move_offset_accumulates_across_moves():
    """Successive raw operator moves accumulate into one offset vector.

    Two forward pushes (10 cm then 8 cm along +x) sum to an 18 cm offset --
    the integrator carries state across moves (mirror of the firmware
    accumulating per-tick deltas into the same ``_offset``).
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    sim.operator_move(10.0)
    sim.operator_move(8.0)

    off = sim.localization.get_offset_from_node()
    assert off.magnitude_cm() == pytest.approx(18.0, abs=1e-9)


def test_operator_turn_updates_theta_not_translation():
    """A raw operator turn updates theta_deg via the rotation channel; translation ~0.

    Mirror of AC2b: a raw ``Drive.turnDeg(theta)`` while idle updates the
    offset's ``theta_deg`` (gated on the IMU-yaw deadband) even though net
    translation is ~0. A pure 45deg turn leaves x/y at 0 and theta_deg at 45.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    sim.operator_move(0.0, turn_deg=45.0)

    off = sim.localization.get_offset_from_node()
    assert off.x == pytest.approx(0.0, abs=1e-9)
    assert off.y == pytest.approx(0.0, abs=1e-9)
    assert off.theta_deg == pytest.approx(45.0, abs=1e-9)


def test_operator_move_below_deadband_drops_translation():
    """A sub-deadband translation is dropped (encoder-noise gate); theta still drops too.

    Mirror of AC2 (no phantom drift): a move shorter than
    OFFSET_TRANSLATION_DEADBAND_CM (0.2 cm) does not grow the translation
    channel, and a turn smaller than OFFSET_ROTATION_DEADBAND_DEG (0.5 deg)
    does not grow the rotation channel. Both channels gate INDEPENDENTLY.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    sim.operator_move(
        OFFSET_TRANSLATION_DEADBAND_CM / 2.0,
        turn_deg=OFFSET_ROTATION_DEADBAND_DEG / 2.0,
    )

    off = sim.localization.get_offset_from_node()
    assert off.magnitude_cm() == pytest.approx(0.0, abs=1e-9)
    assert off.theta_deg == pytest.approx(0.0, abs=1e-9)


def test_operator_move_while_nav_active_accumulates_nothing():
    """A move issued while nav-active accumulates nothing (the gate is closed).

    Mirror of AC3 / the firmware gate ``(!_navActive) && _zeroed``: motion that
    PathTracker itself commands (nav-active True) never grows the operator
    offset. With the gate forced closed, even a 30 cm push leaves the offset at
    zero (the chassis still moves -- only the bookkeeping is gated).
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, chassis = _make_typed_sim(fd, graph, next_hop)

    sim.set_nav_active(True)
    sim.operator_move(30.0)

    off = sim.localization.get_offset_from_node()
    assert off.magnitude_cm() == pytest.approx(0.0, abs=1e-9)
    # The chassis moved even though the offset was gated.
    assert chassis.x_cm == pytest.approx(30.0, abs=1e-9)


def test_operator_move_before_zero_accumulates_nothing():
    """The offset integrator is gated on is_zeroed() (world frame must be bound).

    Mirror of the firmware gate's ``_zeroed`` clause: before
    ``zero_heading_from_current()`` the world frame is not bound, so a raw move
    accumulates no offset.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    # NOTE: deliberately NOT zeroed.

    sim.operator_move(20.0)

    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )


def test_node_arrival_clears_offset():
    """A node arrival re-anchors the offset to zero (AC3 arrival re-anchor).

    Mirror of ``LocalizationClass::onIntersection`` /
    ``onDeadReckonedAdvance`` -> ``clearOffset()``: arriving on a node zeroes
    the accumulated offset. Drive the chassis off-node (offset grows), then a
    clean go_to to B re-anchors at the destination with a zero offset.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    # A sub-trigger push so the subsequent go_to does NOT fire a return -- we
    # are isolating the node-arrival clear, not the return prologue.
    sim.operator_move(RETURN_TRIGGER_THRESHOLD_CM / 2.0)
    assert sim.localization.get_offset_from_node().magnitude_cm() > 0.0

    sim.go_to("B")

    assert sim.localization.current_node() == "B"
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )


# ---------------------------------------------------------------------------
# Return-to-node prologue -- NONE / RETURN (line + blind) / off-map gate
# ---------------------------------------------------------------------------


def test_goto_offset_below_trigger_is_byte_identical_noop(fresh_sim, toy_field):
    """With offset ~0 (below trigger), go_to emits NO return -- stream unchanged (AC5).

    A clean go_to from the anchored node and a go_to after a sub-trigger
    operator push must emit the byte-identical primitive stream: the
    sub-trigger offset resolves NONE, so the prologue inserts nothing.
    """
    graph, next_hop = (fresh_sim._graph, fresh_sim._next_hop)

    # Baseline: clean go_to from A to C.
    chassis_clean = VirtualChassis.from_field_data(toy_field)
    sim_clean = PathTrackerSim(toy_field, graph, next_hop, chassis_clean)
    sim_clean.localization.zero_heading_from_current()
    clean_prims = sim_clean.go_to("C")

    # Sub-trigger push, then the same go_to: must be identical (no return).
    chassis_push = VirtualChassis.from_field_data(toy_field)
    sim_push = PathTrackerSim(toy_field, graph, next_hop, chassis_push)
    sim_push.localization.zero_heading_from_current()
    sim_push.operator_move(RETURN_TRIGGER_THRESHOLD_CM - 0.5)
    pushed_prims = sim_push.go_to("C")

    assert not any(
        p[0] in (ACT_RETURN_STOP_AT_LINE, ACT_RETURN_FORWARD) for p in pushed_prims
    )
    assert pushed_prims == clean_prims


def test_goto_line_backed_return_emits_stop_at_line_and_lands_on_node():
    """A LINE-backed anchor returns via ACT_RETURN_STOP_AT_LINE, axle landing ON the node.

    Mirror of AC4 (line-backed): with the offset above the trigger, go_to first
    re-acquires the line (ACT_RETURN_STOP_AT_LINE) before any planned hop. The
    sim models the SAME stopAtLine overshoot/axle-roll the LINE hops use, so
    the chassis ends back ON the anchored node (drift~0) with the offset
    cleared.

    Setup: the chassis is pushed 20 cm BACKWARD along +x (offset (-20, 0)), so
    the inverse-return heading is 0deg -- the return drives forward 20 cm and
    lands the axle back on node A at the origin. The push is kept above the
    sensor-to-axle lead (LINE_SENSOR_AXLE_OFFSET_CM) so the front sensor can
    actually re-cross the node line -- a shorter LINE return dead-reckons (#14,
    see test_goto_short_line_backed_return_dead_reckons).
    """
    # Node A is line-backed (start). Push backward so the return heading is 0.
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, chassis = _make_typed_sim(fd, graph, next_hop)

    sim.operator_move(-20.0)    # offset (-20, 0); chassis now at x = -20
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        20.0, abs=1e-9
    )

    prims = sim.go_to("B")

    # The FIRST emitted primitive is the line-backed return seek.
    assert prims[0][0] == ACT_RETURN_STOP_AT_LINE
    tag, node, dist, heading = prims[0]
    assert node == "A"
    assert dist == pytest.approx(20.0, abs=1e-9)
    assert heading == pytest.approx(0.0, abs=1e-9)

    # Exactly one return primitive (the prologue fires once).
    assert sum(1 for p in prims if p[0] == ACT_RETURN_STOP_AT_LINE) == 1
    assert not any(p[0] == ACT_RETURN_FORWARD for p in prims)

    # The return cleared the offset and re-anchored on A before the walk.
    assert sim.localization.current_node() == "B"    # after the planned hop
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )
    # Final pose: the planned hop A->B (50 cm along +x) lands the axle on B.
    bx, by = fd.NODES["B"]
    assert math.hypot(chassis.x_cm - bx, chassis.y_cm - by) < _POSITION_TOL_CM


def test_line_return_models_stopatline_axle_roll():
    """The line return models the stopAtLine follow + axle-roll, not a snap-to-node.

    The gotcha: the return seek must model the SAME overshoot/axle-roll the LINE
    hops use (drive back toward the node, roll the axle ONTO it), or the parity
    is fake. Verify the chassis lands EXACTLY on the node after the return seek
    -- not at the line-sensor position (which would be
    LINE_SENSOR_AXLE_OFFSET_CM short of the node).

    Step through to capture the chassis snapshot at the return primitive (before
    any planned hop), so the assertion is on the return move alone.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    push_cm = 20.0    # well above the axle offset so the follow phase is positive
    sim.operator_move(-push_cm)    # offset (-20, 0); return heading 0deg

    steps = list(sim.step_through("B"))
    # The first step is the return seek; its snapshot is post-return-move.
    first_prim, first_snap = steps[0]
    assert first_prim[0] == ACT_RETURN_STOP_AT_LINE
    # Axle lands ON node A (origin), NOT LINE_SENSOR_AXLE_OFFSET_CM short of it.
    assert first_snap.x_cm == pytest.approx(0.0, abs=1e-9)
    assert first_snap.y_cm == pytest.approx(0.0, abs=1e-9)
    # The net return advance equals the full offset (follow short + roll the
    # axle offset): had the sim snapped to the sensor position the axle would
    # sit at x == -LINE_SENSOR_AXLE_OFFSET_CM.
    assert first_snap.x_cm != pytest.approx(-LINE_SENSOR_AXLE_OFFSET_CM)


def test_goto_blind_only_return_emits_turn_and_return_forward():
    """A BLIND-only anchor dead-reckons back via ACT_TURN_REL + ACT_RETURN_FORWARD.

    Mirror of AC4 (blind-only): the return turns to face the inverse-offset
    heading (ACT_TURN_REL, the turnToYaw) then dead-reckons forward
    (ACT_RETURN_FORWARD, the driveDist). It re-anchors via clear_offset (no
    node change -- the dead-reckon return IS the re-anchor) and lands the
    chassis back on the node within tolerance, then plans the path.

    Setup: anchor the chassis ON the blind node 'B' (a chained go_to to B),
    push it off-node, then go_to back to A so the blind-anchor return fires.
    """
    fd, graph, next_hop = _typed_two_node_field("blind")
    sim, chassis = _make_typed_sim(fd, graph, next_hop)

    # First reach the blind node B so it becomes the anchor.
    sim.go_to("B")
    assert sim.localization.current_node() == "B"
    bx, by = fd.NODES["B"]

    # Push the chassis 12 cm off-node along +x (offset (12, 0); return heading
    # 180deg -- the dead-reckon return turns to face -x then drives back).
    sim.operator_move(12.0)
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        12.0, abs=1e-9
    )

    prims = sim.go_to("A")

    # The return is a turn-to-face + dead-reckon forward, emitted before any
    # planned hop. Find them at the head of the stream.
    return_tags = [p[0] for p in prims if p[0] in (ACT_TURN_REL, ACT_RETURN_FORWARD)]
    assert ACT_RETURN_FORWARD in [p[0] for p in prims]
    rf = next(p for p in prims if p[0] == ACT_RETURN_FORWARD)
    _tag, rf_node, rf_dist = rf
    assert rf_node == "B"
    assert rf_dist == pytest.approx(12.0, abs=1e-9)
    # The ACT_TURN_REL aligning to the return heading precedes the forward.
    assert prims.index(rf) >= 1
    assert prims[prims.index(rf) - 1][0] == ACT_TURN_REL

    # Blind return uses clear_offset (no node change), so the anchor stays B
    # until the planned hop advances it; the offset is cleared.
    assert sim.localization.current_node() == "A"   # after the planned hop B->A

    # No line-backed return primitive on a blind anchor.
    assert not any(p[0] == ACT_RETURN_STOP_AT_LINE for p in prims)


def test_blind_return_lands_back_on_node_within_tolerance():
    """The blind dead-reckon return lands the chassis back on the anchored node.

    Capture the chassis at the ACT_RETURN_FORWARD step (before the planned hop)
    and assert it is back on the blind node within tolerance -- the dead-reckon
    return drives the exact inverse offset.
    """
    fd, graph, next_hop = _typed_two_node_field("blind")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)
    sim.go_to("B")
    bx, by = fd.NODES["B"]

    # Push diagonally off-node so x AND y are both non-trivial.
    sim.operator_move(0.0, turn_deg=30.0)    # face 30deg
    sim.operator_move(15.0)                  # 15 cm along 30deg

    steps = list(sim.step_through("A"))
    # Find the return-forward step and its snapshot.
    rf_step = next(
        (prim, snap) for prim, snap in steps if prim[0] == ACT_RETURN_FORWARD
    )
    _prim, snap = rf_step
    assert math.hypot(snap.x_cm - bx, snap.y_cm - by) < _POSITION_TOL_CM


def test_line_return_off_axis_emits_turn_before_seek():
    """An off-axis LINE return turns to the inverse heading FIRST, then seeks the line.

    Regression for issue #36 #1: the firmware LINE return now dispatches the pre-edge
    ``turnToYaw`` through the SHARED turn machinery before the ``stopAtLine`` seek (it
    previously seeked without turning, arcing forward away from the node on a body-axis
    push). The sim mirrors that -- a FORWARD push off the body axis makes the inverse-return
    heading ~180deg from the facing, so the return emits ``ACT_TURN_REL`` (the turnToYaw)
    BEFORE ``ACT_RETURN_STOP_AT_LINE``. The straight-back regression tests never exercised
    this (return heading == facing -> delta 0 -> no turn).
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    # Forward push 20 cm along +x (chassis faces 0deg): offset (20, 0), inverse-return
    # heading 180deg -> a real pre-edge turn is required before the seek. The push stays
    # above the sensor-to-axle lead so this exercises the line seek (a shorter LINE return
    # dead-reckons, #14) while still requiring the off-axis pre-turn.
    sim.operator_move(20.0)
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        20.0, abs=1e-9
    )

    prims = sim.go_to("B")

    # The return's pre-edge turn fires FIRST (prims[0]), then the line re-acquire seek.
    assert prims[0][0] == ACT_TURN_REL
    assert prims[0][1] == pytest.approx(180.0, abs=1e-9)
    seek = next(p for p in prims if p[0] == ACT_RETURN_STOP_AT_LINE)
    assert prims.index(seek) == 1
    _tag, node, dist, heading = seek
    assert node == "A"
    assert dist == pytest.approx(20.0, abs=1e-9)
    assert heading == pytest.approx(180.0, abs=1e-9)
    # The return re-anchored on A and cleared the offset before the planned hop.
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )


def test_goto_short_line_backed_return_dead_reckons():
    """A LINE return shorter than the sensor-to-axle lead dead-reckons, not stopAtLine (#14).

    The front line sensor leads the wheel axle by LINE_SENSOR_AXLE_OFFSET_CM, so a return whose
    distance is below that lead starts the sensor already PAST the node line -- a forward
    stopAtLine seek could never re-cross it and would fault LINE_NOT_FOUND on hardware. The fix
    routes such short LINE returns through the dead-reckon path (ACT_RETURN_FORWARD +
    clear_offset), exactly like a blind anchor. This is the COMMON small hand-correction case
    (offset between RETURN_TRIGGER_THRESHOLD_CM and the axle lead).

    Setup: node A is line-backed; push 8 cm BACKWARD along +x (offset (-8, 0), 3 < 8 < 12.02), so
    the return heading is 0deg (== facing -> no pre-turn) and the return dead-reckons forward 8 cm
    back onto A.
    """
    push_cm = 8.0
    assert RETURN_TRIGGER_THRESHOLD_CM < push_cm < LINE_SENSOR_AXLE_OFFSET_CM

    fd, graph, next_hop = _typed_two_node_field("line")
    sim, chassis = _make_typed_sim(fd, graph, next_hop)

    sim.operator_move(-push_cm)    # offset (-8, 0); return heading 0deg == facing
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        push_cm, abs=1e-9
    )

    prims = sim.go_to("B")

    # Straight-back return (heading == facing) -> no pre-turn; the head primitive is the
    # dead-reckon forward, NOT a line seek.
    assert prims[0][0] == ACT_RETURN_FORWARD
    _tag, node, dist = prims[0]
    assert node == "A"
    assert dist == pytest.approx(push_cm, abs=1e-9)
    # A short LINE return never emits the (would-fault) line seek.
    assert not any(p[0] == ACT_RETURN_STOP_AT_LINE for p in prims)
    # It re-anchored on A (clear_offset, node unchanged), cleared the offset, then walked to B.
    assert sim.localization.current_node() == "B"
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )
    bx, by = fd.NODES["B"]
    assert math.hypot(chassis.x_cm - bx, chassis.y_cm - by) < _POSITION_TOL_CM


def test_line_return_seek_threshold_is_the_axle_offset():
    """The line-seek vs dead-reckon boundary for a LINE return is exactly the axle lead (#14).

    A LINE return at exactly LINE_SENSOR_AXLE_OFFSET_CM re-acquires via the seek (the sensor
    reaches the node line); a hair below it dead-reckons (the sensor cannot re-cross). Locks the
    ``>=`` threshold so a future off-by-one (e.g. ``>``) is caught. Both pushes are straight back
    (return heading == facing) so the head primitive is the dispatch itself, not a pre-turn.
    """
    eps = 1e-3

    # Just BELOW the lead -> dead-reckon (ACT_RETURN_FORWARD), no line seek.
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)
    sim.operator_move(-(LINE_SENSOR_AXLE_OFFSET_CM - eps))
    prims_below = sim.go_to("B")
    assert prims_below[0][0] == ACT_RETURN_FORWARD
    assert not any(p[0] == ACT_RETURN_STOP_AT_LINE for p in prims_below)

    # Exactly AT the lead -> line seek (ACT_RETURN_STOP_AT_LINE).
    fd2, graph2, next_hop2 = _typed_two_node_field("line")
    sim2, _chassis2 = _make_typed_sim(fd2, graph2, next_hop2)
    sim2.operator_move(-LINE_SENSOR_AXLE_OFFSET_CM)
    prims_at = sim2.go_to("B")
    assert prims_at[0][0] == ACT_RETURN_STOP_AT_LINE
    assert not any(p[0] == ACT_RETURN_FORWARD for p in prims_at)


def test_blind_return_turn_snapshot_is_post_turn_not_post_forward():
    """step_through pairs the BLIND return's ACT_TURN_REL with the POST-TURN pose (#7).

    The return dispatch is a generator that yields each primitive AFTER its own mutation, so
    the ``ACT_TURN_REL`` snapshot is the chassis after the in-place turn but BEFORE the
    forward drive -- not the final post-forward pose. A consumer animating step_through must
    not see the chassis jump to the return destination on the turn step.
    """
    fd, graph, next_hop = _typed_two_node_field("blind")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)
    # Anchor on blind node B, then push it off-node so a blind return fires on go_to("A").
    sim.go_to("B")
    bx, by = fd.NODES["B"]
    sim.operator_move(12.0)    # offset (12, 0); return heading 180deg; chassis at (bx+12, by)

    steps = list(sim.step_through("A"))
    turn_idx = next(i for i, (p, _s) in enumerate(steps) if p[0] == ACT_TURN_REL)
    fwd_idx = next(i for i, (p, _s) in enumerate(steps) if p[0] == ACT_RETURN_FORWARD)
    assert turn_idx < fwd_idx
    _tp, turn_snap = steps[turn_idx]
    _fp, fwd_snap = steps[fwd_idx]
    # Post-turn snapshot: the chassis has only ROTATED (still at the pushed-off pose); it has
    # NOT yet driven back. The forward snapshot IS back on the node.
    assert math.hypot(turn_snap.x_cm - (bx + 12.0), turn_snap.y_cm - by) < 1e-6
    assert math.hypot(fwd_snap.x_cm - bx, fwd_snap.y_cm - by) < _POSITION_TOL_CM


def test_operator_move_subdeadband_carries_across_calls():
    """Sub-deadband moves accumulate across calls until they cross the floor, then flush (#5).

    Mirror of the firmware leaving the encoder baseline UNADVANCED below the deadband: a slow
    hand-drive delivered as many sub-deadband ``operator_move`` calls must NOT be dropped per
    call -- the residual carries and flushes (in full) once it crosses
    OFFSET_TRANSLATION_DEADBAND_CM, so the sim's offset tracks the firmware's instead of
    staying at ~0. The single-call drop test (a sub-deadband move read once) still holds.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    step = OFFSET_TRANSLATION_DEADBAND_CM * 0.4    # 0.08 cm: each call is sub-deadband
    # Two sub-deadband calls stay below the 0.2 cm floor -> offset still ~0 (residual carried).
    sim.operator_move(step)
    sim.operator_move(step)
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )

    # The third call crosses the floor (0.24 >= 0.2) -> flush the WHOLE accumulated residual.
    sim.operator_move(step)
    off = sim.localization.get_offset_from_node()
    assert off.x == pytest.approx(3 * step, abs=1e-9)    # along START_HEADING 0deg
    assert off.magnitude_cm() == pytest.approx(3 * step, abs=1e-9)

    # Residual reset after the flush: a further sub-deadband move is again carried (no growth).
    sim.operator_move(step)
    assert sim.localization.get_offset_from_node().x == pytest.approx(3 * step, abs=1e-9)


def test_partial_node_types_fixture_raises():
    """A fixture authoring NODE_TYPES but omitting a graph node raises (bench-fidelity, #9).

    ``node_type()`` defaults an absent node to ``"line"``, diverging from
    ``FieldMap::getNodeType`` (which returns the authored type for every valid node). A
    fixture missing an entry would silently route that node through the LINE re-acquire while
    the firmware dead-reckons -- a false-green parity. The sim now refuses to construct such a
    fixture. A fixture with NO NODE_TYPES at all stays exempt (the minimal synthetic graphs).
    """
    fd, graph, next_hop = _typed_two_node_field("blind")
    # Drop B's authored type, leaving NODE_TYPES non-empty but incomplete.
    fd.NODE_TYPES = {"A": "line"}
    chassis = VirtualChassis.from_field_data(fd)
    with pytest.raises(ValueError, match="NODE_TYPES is missing"):
        PathTrackerSim(fd, graph, next_hop, chassis)


def test_goto_over_cap_fault_gate_raises_and_moves_nothing():
    """An over-cap offset under the FAULT gate raises OffMapFaultError; no chassis motion.

    Mirror of AC6 (FAULT): when the offset exceeds RETURN_DISTANCE_CAP_CM and
    OFF_MAP_GATE is FAULT, go_to refuses to plan -- it raises OffMapFaultError
    and the chassis is not moved (the offset is left intact). Default gate is
    FAULT, so no patch is needed.
    """
    fd, graph, next_hop = _typed_two_node_field("line")
    sim, chassis = _make_typed_sim(fd, graph, next_hop)

    over_cap = RETURN_DISTANCE_CAP_CM + 5.0
    sim.operator_move(-over_cap)    # push backward beyond the cap
    chassis_x_before = chassis.x_cm
    node_before = sim.localization.current_node()

    with pytest.raises(OffMapFaultError) as excinfo:
        sim.go_to("B")

    err = excinfo.value
    assert err.node == "A"
    assert err.goal_node == "B"
    assert err.offset_mag_cm == pytest.approx(over_cap, abs=1e-9)
    # No motion, no node advance, offset untouched.
    assert chassis.x_cm == pytest.approx(chassis_x_before, abs=1e-9)
    assert sim.localization.current_node() == node_before
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        over_cap, abs=1e-9
    )


def test_goto_over_cap_best_effort_gate_proceeds_capped(monkeypatch):
    """An over-cap offset under BEST_EFFORT returns anyway, clamped to the cap (AC6).

    Mirror of AC6 (BEST_EFFORT): with OFF_MAP_GATE patched to BEST_EFFORT, an
    over-cap offset does NOT fault -- it dispatches the return with the drive
    distance CLAMPED to RETURN_DISTANCE_CAP_CM (the over-cap vector is
    untrustworthy, so the firmware bounds the seek), then plans the goal.
    """
    monkeypatch.setattr(_runtime, "OFF_MAP_GATE", OffMapGate.BEST_EFFORT)

    fd, graph, next_hop = _typed_two_node_field("line")
    sim, _chassis = _make_typed_sim(fd, graph, next_hop)

    over_cap = RETURN_DISTANCE_CAP_CM + 12.0
    sim.operator_move(-over_cap)    # backward beyond cap; return heading 0deg

    prims = sim.go_to("B")

    # A return seek fired (no fault) and its distance is clamped to the cap.
    return_prims = [p for p in prims if p[0] == ACT_RETURN_STOP_AT_LINE]
    assert len(return_prims) == 1
    _tag, node, dist, _heading = return_prims[0]
    assert node == "A"
    assert dist == pytest.approx(RETURN_DISTANCE_CAP_CM, abs=1e-9)
    # The goal still planned after the capped return.
    assert sim.localization.current_node() == "B"


# ---------------------------------------------------------------------------
# TC18a parity -- post-return planned tail matches a clean go_to (AC17 pillar 1)
# ---------------------------------------------------------------------------


def test_tc18a_post_return_tail_matches_clean_goto():
    """TC18a: the planned tail after a return is byte-identical to a clean go_to.

    The return erases the operator excursion, so the planned-path primitive
    stream (everything after the return primitive) and the final arrival pose
    must match a clean go_to from the anchored node with drift~0.

    Heading parity: the line return faces the inverse-offset heading. A pure
    BACKWARD push along the start heading gives an inverse-return heading equal
    to the start heading, so the post-return chassis faces exactly as a clean
    start does -- making the first planned ACT_TURN_REL identical between the
    two runs.
    """
    # A->B->C with a 90deg turn at B (so the sim emits ACT_LINE_OVERSHOOT, not
    # ACT_LINE_PASS, at the intermediate node -- matching the clean baseline).
    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0), "C": (50.0, 50.0)}
    node_types = {"A": "line", "B": "line", "C": "line"}

    def _build():
        fd = SimpleNamespace(
            NODES=nodes,
            NODE_TYPES=node_types,
            START_NODE="A",
            START_HEADING_DEG=0.0,
            PIVOT_TURNS=[],
        )
        g = nx.DiGraph()
        for name, (x, y) in nodes.items():
            g.add_node(name, x_cm=float(x), y_cm=float(y))
        for u, v, hdg in (("A", "B", 0.0), ("B", "C", 90.0)):
            g.add_edge(u, v, type=EDGE_LINE, distance_cm=50.0, heading_deg=hdg)
            g.add_edge(
                v, u, type=EDGE_LINE, distance_cm=50.0, heading_deg=(hdg + 180.0) % 360.0
            )
        next_hop = {
            "A": {"A": "A", "B": "B", "C": "B"},
            "B": {"A": "A", "B": "B", "C": "C"},
            "C": {"A": "B", "B": "B", "C": "C"},
        }
        return fd, g, next_hop

    # Clean baseline.
    fd_clean, g_clean, nh_clean = _build()
    chassis_clean = VirtualChassis.from_field_data(fd_clean)
    sim_clean = PathTrackerSim(fd_clean, g_clean, nh_clean, chassis_clean)
    sim_clean.localization.zero_heading_from_current()
    clean_prims = sim_clean.go_to("C")

    # Post-return run: push BACKWARD along +x (offset (-d, 0); return heading
    # 0deg == start heading), then go_to C. The push stays above the sensor-to-axle
    # lead so the LINE return re-acquires via the seek (a shorter return dead-reckons, #14).
    fd_ret, g_ret, nh_ret = _build()
    chassis_ret = VirtualChassis.from_field_data(fd_ret)
    sim_ret = PathTrackerSim(fd_ret, g_ret, nh_ret, chassis_ret)
    sim_ret.localization.zero_heading_from_current()
    sim_ret.operator_move(-20.0)
    ret_prims = sim_ret.go_to("C")

    # The head primitive is the return seek; the tail is the planned path.
    assert ret_prims[0][0] == ACT_RETURN_STOP_AT_LINE
    planned_tail = ret_prims[1:]
    assert planned_tail == clean_prims

    # Final arrival pose parity: both end on C with drift~0 (do NOT loosen the
    # existing position tolerance).
    cx, cy = nodes["C"]
    assert math.hypot(chassis_ret.x_cm - cx, chassis_ret.y_cm - cy) < _POSITION_TOL_CM
    assert math.hypot(chassis_clean.x_cm - cx, chassis_clean.y_cm - cy) < _POSITION_TOL_CM
    assert sim_ret.localization.current_node() == "C"


def test_tc18a_clean_goto_after_return_nets_zero_drift_on_toy_fixture(toy_field, toy_graph_and_table):
    """TC18a (toy fixture): a return then a clean go_to lands on the goal, drift~0.

    On the shared toy fixture, first drive to the line node ``A`` (issue #64: the
    toy START is now the blind spawn ``S``, so the chassis must re-anchor on a
    line node before the line-backed return can be exercised). Then push the
    chassis off ``A`` and issue go_to(C): the prologue returns to ``A``
    (line-backed) via the seek then walks A->...->C. The chassis lands on C within
    _POSITION_TOL_CM and the node advances to C -- proving the return erased the
    excursion end-to-end on a multi-hop route.
    """
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field)
    sim = PathTrackerSim(toy_field, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    # Re-anchor on the line node A first (the blind START S is not line-backed).
    sim.go_to("A")
    assert sim.localization.current_node() == "A"

    sim.operator_move(-20.0)    # above the axle lead (line seek), within cap; heading 0deg
    prims = sim.go_to("C")

    assert prims[0][0] == ACT_RETURN_STOP_AT_LINE
    assert sim.localization.current_node() == "C"
    cx, cy = toy_field.NODES["C"]
    assert math.hypot(chassis.x_cm - cx, chassis.y_cm - cy) < _POSITION_TOL_CM
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )


def test_tc14_return_to_projection_node_dead_reckons_no_line_seek():
    """TC14 (issue #64): a RETURN_TO_NODE to a projection node DEAD-RECKONS, no junction-seek.

    A projection node sits on a line but at NO junction, so there is no junction
    marker to re-acquire -- ``is_line_backed`` returns False for it (it tests
    ``== "line"``), and the return prologue MUST dead-reckon (``ACT_RETURN_FORWARD``)
    rather than line-seek (``ACT_RETURN_STOP_AT_LINE``). No projection special-case
    branch is added; the default ``!= "line"`` path handles it.

    Topology: a blind spawn ``S`` feeds a projection node ``P`` over a BLIND edge
    (the auto-seek re-anchors the chassis ON ``P``). Standing on ``P``, push the
    chassis off by more than the axle lead, then issue a go_to back toward ``S``:
    the return prologue, finding the current node ``P`` NOT line-backed,
    dead-reckons.
    """
    nodes = {"S": (0.0, 0.0), "P": (60.0, 0.0)}
    fd = _minimal_field(nodes, start_node="S", start_heading_deg=0.0)
    # The return prologue reads node types off field_data.NODE_TYPES (the
    # LocalizationSim mirror of FieldMap::getNodeType), so author them here -- the
    # bench-fidelity guard requires every graph node be covered once NODE_TYPES
    # is present.
    fd.NODE_TYPES = {"S": "blind", "P": "projection"}
    g = nx.DiGraph()
    g.add_node("S", type="blind")
    g.add_node("P", type="projection")
    g.add_edge("S", "P", type=EDGE_BLIND, distance_cm=60.0, heading_deg=0.0)
    g.add_edge("P", "S", type=EDGE_BLIND, distance_cm=60.0, heading_deg=180.0)
    next_hop = {"S": {"S": "S", "P": "P"}, "P": {"S": "S", "P": "P"}}

    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    # Drive S -> P: the BLIND->projection auto-seek re-anchors the chassis ON P.
    sim.go_to("P")
    assert sim.localization.current_node() == "P"
    assert sim.localization.is_line_backed("P") is False

    # Push the chassis off P by more than the FRONT axle lead (so a line-backed
    # node WOULD line-seek), then return toward S.
    sim.operator_move(-20.0)
    prims = sim.go_to("S")

    # The return DEAD-RECKONS (no junction at a projection node) -- the head
    # primitive is ACT_RETURN_FORWARD, never ACT_RETURN_STOP_AT_LINE.
    return_tags = [
        p[0] for p in prims if p[0] in (ACT_RETURN_FORWARD, ACT_RETURN_STOP_AT_LINE)
    ]
    assert return_tags, "expected a return primitive in the prologue"
    assert return_tags[0] == ACT_RETURN_FORWARD
    assert ACT_RETURN_STOP_AT_LINE not in [p[0] for p in prims]
    assert sim.localization.current_node() == "S"


# ===========================================================================
# TC18b — wall-zero re-anchor at a reset node (PR2, AC17 pillar 2)
#
# Covers: the WALL_STOP -> REANCHOR interlude the sim runs the moment a hop
# re-anchors localization onto a node typed "reset". Mirrors the firmware
# interlude (PathTracker::_maybeEnterReset -> _dispatchWallStop -> REANCHOR
# -> Localization.reanchorAtWall). Asserts primitive-sequence parity (the
# interlude fires at the right point), pose parity / re-zero (the believed
# on-axis coordinate snaps to wall_coord, heading to facing, any pre-existing
# drift CORRECTED), and both the intermediate-hop and final-destination cases.
#
# Code under test: tools/path_planner/sim/runtime.py --
#   PathTrackerSim._dispatch_wall_reset, LocalizationSim.is_reset_node /
#   reset_info / reanchor_at_wall, ACT_WALL_STOP, ACT_REANCHOR,
#   WALL_STOP_BACK_OFF_CM.
#
# Fixtures are hand-rolled deterministic graphs (no field_data.json /
# validator / builder), mirroring the TC9 / TC10 / TC10-bidir pattern so the
# pose + primitive assertions are exact. wall_coord stands a fixed clearance
# off the reset node (>= WALL_PIVOT_CLEARANCE_CM, the validator floor); the
# WALL_DEPART phase returns the believed on-axis coordinate TO the node, so a
# clean inbound hop nets the node and the re-zero corrects ONLY an injected
# drift -- never re-introducing a per-junction offset.
# ===========================================================================


def _reset_chain_field(
    *,
    inbound_distance_cm: float = 30.0,
    reset_axis: str = "x",
    reset_wall_coord: float = 10.0,
    reset_facing: float = 0.0,
):
    """Build an A -> R -> C blind chain with R typed 'reset'.

    Geometry is collinear along +x: A(0,0), R(30,0), C(60,0). All three edges
    are BLIND (dead-reckoned), so the chassis advances each edge's
    ``distance_cm`` exactly -- the test controls the inbound A->R distance to
    inject (or not) an arrival drift relative to R's stored coordinate.

    The wall sits on the -x side at ``wall_coord=10`` (R is at x=30, so the
    node-to-wall standoff is 20 cm -- above the validator's
    ``WALL_PIVOT_CLEARANCE_CM`` floor, and consistent with the WALL_DEPART return
    math). With the wall on the -x side, ``facing=0`` (front +x) points AWAY from
    it, collinear with the inbound and the R->C hop headings, so the interlude
    introduces no extra pre-edge turn. After the interlude the believed on-axis
    coordinate returns to R's stored x (30), correcting any injected inbound
    drift TO the node -- never re-introducing a per-junction offset.

    :param inbound_distance_cm: the A->R edge length. Set ABOVE 30 to land the
        chassis past R's stored x (an injected dead-reckon overshoot the wall
        re-zero must correct); leave at 30 for a clean (drift-free) arrival.
    :param reset_axis: ``"x"`` or ``"y"`` -- the axis R re-zeroes against.
    :param reset_wall_coord: the wall coordinate R presses against on
        ``reset_axis`` (default -x side of R, 20 cm standoff).
    :param reset_facing: the squared heading R adopts (deg).
    :return: ``(field_data, graph, next_hop)`` ready for ``PathTrackerSim``.
    """
    nodes = {"A": (0.0, 0.0), "R": (30.0, 0.0), "C": (60.0, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        NODE_TYPES={"A": "blind", "R": "reset", "C": "blind"},
        RESET_NODES={
            "R": {
                "axis": reset_axis,
                "wall_coord": reset_wall_coord,
                "facing": reset_facing,
            }
        },
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y))
    # A->R carries the (possibly drift-injecting) inbound distance; R->C is a
    # clean 30 cm hop. Reverse edges keep Dijkstra's all-pairs sweep happy.
    g.add_edge("A", "R", type=EDGE_BLIND, distance_cm=float(inbound_distance_cm), heading_deg=0.0)
    g.add_edge("R", "A", type=EDGE_BLIND, distance_cm=float(inbound_distance_cm), heading_deg=180.0)
    g.add_edge("R", "C", type=EDGE_BLIND, distance_cm=30.0, heading_deg=0.0)
    g.add_edge("C", "R", type=EDGE_BLIND, distance_cm=30.0, heading_deg=180.0)
    next_hop = {
        "A": {"A": "A", "R": "R", "C": "R"},
        "R": {"A": "A", "R": "R", "C": "C"},
        "C": {"A": "R", "R": "R", "C": "C"},
    }
    return fd, g, next_hop


def _make_reset_sim(fd, g, next_hop):
    """Return a zeroed PathTrackerSim + chassis from a reset-chain fixture."""
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis


def test_tc18b_reset_node_final_destination_emits_interlude_and_rezeroes():
    """A go_to ENDING on a reset node emits FORWARD then the 3-phase interlude.

    Clean inbound (A->R = 30 cm, no drift): the chassis lands on R's stored x,
    then the interlude presses the -x wall (x=10), re-anchors to the axle, and
    departs forward back onto the node. The believed x ends EXACTLY on R's NODE
    coordinate (x=30, drift 0) facing the wall normal.
    """
    fd, g, next_hop = _reset_chain_field()
    sim, chassis = _make_reset_sim(fd, g, next_hop)

    prims = sim.go_to("R")

    # Primitive-sequence parity: the inbound blind hop, then the three-phase
    # interlude in order, and nothing after (R is the destination).
    assert prims == [
        (ACT_FORWARD, 30.0),
        (ACT_WALL_STOP, "R", 0, 10.0),
        (ACT_REANCHOR, "R", 0, 10.0),
        (ACT_WALL_DEPART, "R", 0, 30.0),
    ]

    # Pose parity / re-zero: the WALL_DEPART phase returns the believed x to R's
    # NODE coordinate (30), heading squared to facing, off-axis untouched. Do
    # NOT loosen _POSITION_TOL_CM.
    assert chassis.x_cm == pytest.approx(30.0, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert chassis.heading_deg == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "R"
    # The wall re-anchor clears the operator-offset frame, exactly like a node
    # arrival -- no residual phantom offset.
    assert sim.localization.get_offset_from_node().magnitude_cm() == pytest.approx(
        0.0, abs=1e-9
    )


def test_tc18b_reset_node_corrects_injected_arrival_drift():
    """An over-long inbound hop drifts the chassis past R; the re-zero CORRECTS it.

    A->R is 31 cm but R's true x is 30, so the dead-reckon lands the chassis at
    x=31 -- 1 cm of injected arrival drift. The interlude presses the -x wall,
    re-anchors to the axle, then departs forward onto the node, snapping the
    believed x back to R's NODE coordinate 30 and ERASING the drift (the whole
    point of wall-zero). Without the interlude the chassis would sit at x=31.
    """
    fd, g, next_hop = _reset_chain_field(inbound_distance_cm=31.0)
    sim, chassis = _make_reset_sim(fd, g, next_hop)

    # Sanity: the inbound dead-reckon really does overshoot R's stored x.
    rx, _ry = fd.NODES["R"]
    assert rx == pytest.approx(30.0)

    prims = sim.go_to("R")

    assert prims == [
        (ACT_FORWARD, 31.0),
        (ACT_WALL_STOP, "R", 0, 10.0),
        (ACT_REANCHOR, "R", 0, 10.0),
        (ACT_WALL_DEPART, "R", 0, 30.0),
    ]
    # The +1 cm drift is corrected to exactly R's NODE coordinate 30 (NOT 31):
    # WALL_DEPART returns the believed pose to the node, drift on the axis == 0.
    assert chassis.x_cm == pytest.approx(30.0, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert chassis.heading_deg == pytest.approx(0.0, abs=1e-9)
    # Back-off is NOT visible in the believed pose: the node coordinate, not
    # the axle/back-off intermediate.
    assert chassis.x_cm != pytest.approx(30.0 - WALL_STOP_BACK_OFF_CM, abs=1e-9)
    assert sim.localization.current_node() == "R"


def test_tc18b_reset_node_intermediate_hop_runs_interlude_then_continues():
    """A reset node MID-route runs the interlude, then routing continues to C.

    go_to(C) routes A -> R -> C with R typed 'reset'. The interlude fires after
    the A->R hop (intermediate), then the R->C hop continues. Both the inbound
    drift correction AND the axle-on-node contract for the onward hop hold: with
    wall_coord == R's stored x, R->C nets its 30 cm edge and lands ON C.
    """
    fd, g, next_hop = _reset_chain_field(inbound_distance_cm=31.0)
    sim, chassis = _make_reset_sim(fd, g, next_hop)

    prims = sim.go_to("C")

    # Interlude (all three phases) is wedged between the two blind hops, in order.
    assert prims == [
        (ACT_FORWARD, 31.0),
        (ACT_WALL_STOP, "R", 0, 10.0),
        (ACT_REANCHOR, "R", 0, 10.0),
        (ACT_WALL_DEPART, "R", 0, 30.0),
        (ACT_FORWARD, 30.0),
    ]

    # After the re-zero corrected the +1 cm drift at R (believed x=30), the
    # onward 30 cm R->C hop lands the axle ON C (x=60), drift~0 -- the axle-on-
    # node contract still holds through a reset node. Do NOT loosen the tol.
    cx, cy = fd.NODES["C"]
    assert math.hypot(chassis.x_cm - cx, chassis.y_cm - cy) < _POSITION_TOL_CM
    assert sim.localization.current_node() == "C"


def test_tc18b_reset_node_y_axis_squares_heading_and_rezeroes_y():
    """A reset node on the y-axis re-zeroes y and squares heading to facing.

    Exercises the axis == "y" branch (axis_idx 1) and a non-zero facing. The
    chassis arrives at R via a +x blind hop, then the interlude squares to
    facing=90deg (front +y, away from the -y wall), presses the wall, and departs
    back onto the node -- the believed y returns to R's NODE coordinate while x is
    left untouched.
    """
    fd, g, next_hop = _reset_chain_field(
        reset_axis="y", reset_wall_coord=-20.0, reset_facing=90.0
    )
    sim, chassis = _make_reset_sim(fd, g, next_hop)

    prims = sim.go_to("R")

    assert prims == [
        (ACT_FORWARD, 30.0),
        (ACT_WALL_STOP, "R", 1, -20.0),
        (ACT_REANCHOR, "R", 1, -20.0),
        (ACT_WALL_DEPART, "R", 1, 0.0),
    ]
    # y returned to R's NODE coordinate (0) after WALL_DEPART; x (off-axis)
    # untouched at R's arrival x; heading squared to the wall normal.
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert chassis.x_cm == pytest.approx(30.0, abs=1e-9)
    assert chassis.heading_deg == pytest.approx(90.0, abs=1e-9)
    assert sim.localization.current_node() == "R"


def test_tc18b_step_through_yields_interlude_primitives():
    """step_through yields the WALL_STOP + REANCHOR steps, each with a snapshot.

    The generator entry point must surface the interlude identically to go_to
    (same emission shape), paired with the post-mutation chassis snapshot.
    """
    fd, g, next_hop = _reset_chain_field()
    sim, chassis = _make_reset_sim(fd, g, next_hop)

    steps = list(sim.step_through("R"))
    emitted = [prim for prim, _snap in steps]

    assert emitted == [
        (ACT_FORWARD, 30.0),
        (ACT_WALL_STOP, "R", 0, 10.0),
        (ACT_REANCHOR, "R", 0, 10.0),
        (ACT_WALL_DEPART, "R", 0, 30.0),
    ]
    # The snapshot paired with the final WALL_DEPART step reflects the pose
    # returned to R's NODE coordinate.
    _depart_prim, depart_snap = steps[-1]
    assert depart_snap.x_cm == pytest.approx(30.0, abs=1e-9)
    assert depart_snap.heading_deg == pytest.approx(0.0, abs=1e-9)


def test_tc18b_non_reset_node_emits_no_interlude():
    """A non-reset destination emits NO wall-stop/reanchor primitives (AC5).

    Negative control / regression guard: routing to a plain BLIND node must not
    emit the interlude. Proves the interlude is gated strictly on the arrived
    node's type.
    """
    nodes = {"A": (0.0, 0.0), "B": (30.0, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        NODE_TYPES={"A": "blind", "B": "blind"},
        RESET_NODES={},
        START_NODE="A",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    g = _single_edge_graph("A", "B", EDGE_BLIND, 30.0, 0.0)
    nh = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, nh, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FORWARD, 30.0)]
    assert not any(
        p[0] in (ACT_WALL_STOP, ACT_REANCHOR, ACT_WALL_DEPART) for p in prims
    )
    assert chassis.x_cm == pytest.approx(30.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


# ===========================================================================
# TC18c — decay-weighted routing parity (PR3, AC17 pillar 3)
#
# Covers: the sim walks the SAME decay-weighted NEXT_HOP the firmware does.
# The sim builds its NEXT_HOP via the planner's
# ``compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(...))``
# (the SAME call demo.py and every other sim test uses, migrated by T22) --
# never a stale flat-multiplier path or a sim-only hardcoded table. So the
# routing the sim follows IS decay-weighted by construction; this row proves
# that end-to-end: the sim's NEXT_HOP-walked node sequence matches the
# planner's, the decay penalty actually FLIPS the chosen route at the field's
# default k vs k=0 / lowered k, and the chassis lands ON the decay-chosen goal
# with drift~0 (the VirtualChassis tracks the wheel axle and lands on each
# node -- the position tolerance is NOT loosened to absorb drift).
#
# Code under test: tools/path_planner/sim/runtime.py -- PathTrackerSim
# walking a decay-weighted NextHopTable; tools/path_planner/planner.py --
# BlindDecayParams / compute_all_pairs_next_hop.
#
# Fixture: a deep-blind-shortcut-vs-reset-detour diamond (the same idea the
# planner-level TC16 / AC15 tests use), but with edge headings + distances
# derived from the node coordinates so the chassis geometry is self-consistent
# and a clean go_to lands axle-on-node. All seven edges are exactly 50 cm:
#
#     SHORTCUT (raw 150):  S --50-- A1 --50-- A2 --50-- Gl
#     DETOUR   (raw 200):  S --50-- B1 --50-- R --50-- B2 --50-- Gl
#
# When R is a "reset" anchor (d(R)=0) the detour's blind hops have low-d tails
# and are barely penalised, while the shortcut's interior node A2 sits two
# blind hops (d=100) from the only other anchor S, so the shortcut is penalised
# hard. Hand-computed weighted costs (exponent 2, d_ref 50) match the planner
# fixture: shortcut = 150 + 250*k, detour = 200 + 100*k, so the detour wins iff
# k > 1/3. Default k=0.5 -> detour (threads R); lowered k=0.05 -> shortcut.
# ===========================================================================


# Decay knobs for the TC18c diamond. Mirror the planner-level AC15 fixture
# (tools/path_planner/tests/test_planner.py): exponent / d_ref match the toy
# field, only k is varied to flip the route. k=0.5 is decisively "default"
# (> 1/3 crossover) and k=0.05 decisively "lowered".
_TC18C_D_REF: float = 50.0
_TC18C_EXPONENT: float = 2.0
_TC18C_K_DEFAULT: float = 0.5
_TC18C_K_LOWERED: float = 0.05

# Flat-bottomed trapezoid detour: every detour segment is exactly 50 cm
# (S->B1->R->B2->Gl), the same length the planner fixture assigns by hand, and
# the shortcut S->A1->A2->Gl is collinear on +x at 3 x 50 = 150 cm. B1/R/B2 sit
# at y = -50*sin(60deg) so each consecutive pair is exactly 50 cm apart -- the
# geometry is self-consistent, so a sim go_to lands the chassis ON Gl (drift~0)
# regardless of which branch decay selects.
_TC18C_DETOUR_Y: float = -50.0 * math.sin(math.radians(60.0))  # = -43.30127...
_TC18C_NODES = {
    "S":  (0.0, 0.0),
    "A1": (50.0, 0.0),
    "A2": (100.0, 0.0),
    "Gl": (150.0, 0.0),
    "B1": (25.0, _TC18C_DETOUR_Y),
    "R":  (75.0, _TC18C_DETOUR_Y),
    "B2": (125.0, _TC18C_DETOUR_Y),
}
_TC18C_EDGES = (
    ("S", "A1"), ("A1", "A2"), ("A2", "Gl"),          # shortcut: raw 150
    ("S", "B1"), ("B1", "R"), ("R", "B2"), ("B2", "Gl"),  # detour: raw 200
)


def _tc18c_decay_params(k: float) -> BlindDecayParams:
    """Return the TC18c diamond's decay params at gain ``k``."""
    return BlindDecayParams(
        enabled=True, k=k, exponent=_TC18C_EXPONENT, d_ref=_TC18C_D_REF
    )


def _tc18c_diamond(*, reset_node: bool):
    """Build the shortcut-vs-detour diamond with geometry-consistent edges.

    Every edge's ``heading_deg`` and ``distance_cm`` are derived from the node
    coordinates (mirroring ``graph_builder.build_graph``'s geometry), so the
    :class:`VirtualChassis` tracks real geometry and a clean ``go_to`` lands the
    wheel axle ON each visited node -- the arrival-parity contract is not
    achievable with the planner fixture's hand-stamped ``heading_deg=0`` edges,
    which is why this fixture is rebuilt here rather than imported.

    All seven edges are exactly 50 cm (verified by the geometry sanity assert in
    the tests). The shortcut S->A1->A2->Gl is raw 150; the detour
    S->B1->R->B2->Gl is raw 200.

    :param reset_node: classify the mid-detour node ``R`` as ``"reset"`` (an
        anchor with ``d(R)=0``, so the detour's blind hops are barely penalised)
        when True, or a plain ``"blind"`` node (the negative control) when
        False. ``S`` is always a ``"line"`` anchor; every other node is blind.
    :return: a ``(field_data, graph)`` pair. The NEXT_HOP table is built per-k
        by the caller via :func:`compute_all_pairs_next_hop` so one graph can be
        routed at several gains.
    """
    node_types = {name: "blind" for name in _TC18C_NODES}
    node_types["S"] = "line"
    node_types["R"] = "reset" if reset_node else "blind"

    def _heading(u: str, v: str) -> float:
        (x0, y0), (x1, y1) = _TC18C_NODES[u], _TC18C_NODES[v]
        return math.degrees(math.atan2(y1 - y0, x1 - x0)) % 360.0

    def _distance(u: str, v: str) -> float:
        (x0, y0), (x1, y1) = _TC18C_NODES[u], _TC18C_NODES[v]
        return math.hypot(x1 - x0, y1 - y0)

    g = nx.DiGraph()
    for name, (x, y) in _TC18C_NODES.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y), type=node_types[name])
    for u, v in _TC18C_EDGES:
        g.add_edge(u, v, type=EDGE_BLIND, distance_cm=_distance(u, v), heading_deg=_heading(u, v))
        g.add_edge(v, u, type=EDGE_BLIND, distance_cm=_distance(u, v), heading_deg=_heading(v, u))

    # The reset metadata re-zeroes R's y-axis to its own true coordinate (the
    # wall IS at the node), so a clean inbound hop is corrected by a no-op and
    # the chassis stays on the node geometry -- exactly the TC18b convention.
    reset_nodes = {}
    if reset_node:
        reset_nodes["R"] = {
            "axis": "y",
            "wall_coord": _TC18C_DETOUR_Y,
            "facing": _heading("B1", "R"),
        }

    fd = SimpleNamespace(
        NODES=dict(_TC18C_NODES),
        NODE_TYPES=node_types,
        RESET_NODES=reset_nodes,
        # The decay knobs live ON the field_data so the sim builds its NEXT_HOP
        # the same way demo.py does: BlindDecayParams.from_field_data(fd). The
        # per-test routing uses explicit params to flip k, but exposing the
        # knobs here proves from_field_data wires through unchanged.
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=_TC18C_K_DEFAULT,
        BLIND_DECAY_EXPONENT=_TC18C_EXPONENT,
        BLIND_DECAY_D_REF=_TC18C_D_REF,
        START_NODE="S",
        START_HEADING_DEG=0.0,
        PIVOT_TURNS=[],
    )
    return fd, g


def _tc18c_sim(fd, g, next_hop):
    """Return a zeroed PathTrackerSim + chassis over the diamond fixture."""
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis


def _path_has_reset(path, fd) -> bool:
    """True iff any node on ``path`` is typed ``"reset"`` in ``fd.NODE_TYPES``."""
    return any(fd.NODE_TYPES[n] == "reset" for n in path)


def test_tc18c_diamond_all_edges_are_50cm():
    """Geometry sanity: every diamond edge is exactly 50 cm (cost-math anchor).

    The hand-computed crossover (shortcut = 150 + 250k, detour = 200 + 100k,
    flip at k = 1/3) only holds if every edge is 50 cm. This guards the fixture
    geometry so a future coordinate edit that breaks the 50 cm invariant fails
    HERE with a clear message rather than silently shifting the crossover and
    making the routing-flip tests below pass/fail for the wrong reason.
    """
    _fd, g = _tc18c_diamond(reset_node=True)
    for u, v in _TC18C_EDGES:
        assert g.edges[u, v]["distance_cm"] == pytest.approx(50.0, abs=1e-9), (
            f"edge {u}->{v} is not 50 cm; the TC18c cost crossover assumes it is"
        )
    # Shortcut raw total 150, detour raw total 200 (the lengths the planner
    # fixture stamps by hand).
    shortcut = sum(g.edges[u, v]["distance_cm"] for u, v in (("S", "A1"), ("A1", "A2"), ("A2", "Gl")))
    detour = sum(
        g.edges[u, v]["distance_cm"]
        for u, v in (("S", "B1"), ("B1", "R"), ("R", "B2"), ("B2", "Gl"))
    )
    assert shortcut == pytest.approx(150.0, abs=1e-9)
    assert detour == pytest.approx(200.0, abs=1e-9)


def test_tc18c_sim_next_hop_built_from_field_decay_params():
    """The sim's NEXT_HOP is the planner's decay-weighted table, byte-for-byte.

    Builds the sim's NEXT_HOP exactly as demo.py does --
    ``compute_all_pairs_next_hop(g, BlindDecayParams.from_field_data(fd))`` --
    and confirms it is the same object the sim routes on AND that it equals an
    independently-computed decay-weighted table at the field's default k. This
    is the "sim mirrors the robot, not a parallel sim-only config" contract:
    the routing the sim follows IS the decay model, sourced from the field's own
    knobs, never a stale flat multiplier or a hardcoded sim table.
    """
    fd, g = _tc18c_diamond(reset_node=True)
    # The from_field_data path (what demo.py / every sim test uses).
    field_params = BlindDecayParams.from_field_data(fd)
    assert field_params == _tc18c_decay_params(_TC18C_K_DEFAULT)
    next_hop = compute_all_pairs_next_hop(g, field_params)

    sim, _chassis = _tc18c_sim(fd, g, next_hop)
    # The sim routes on exactly this table (no internal rebuild / no flat path).
    assert sim._next_hop is next_hop    # pylint: disable=protected-access
    # And the table genuinely carries the decay route at the default k.
    assert walk_next_hop(next_hop, "S", "Gl") == ["S", "B1", "R", "B2", "Gl"]


def test_tc18c_default_k_sim_walks_planner_decay_route_through_reset():
    """At default k the sim follows the planner's decay route through the reset.

    The decay penalty makes the longer reset-threading detour cheaper than the
    deep-blind shortcut (k=0.5 > 1/3 crossover). The sim, routing on the
    decay-weighted NEXT_HOP, must walk the SAME node sequence the planner's
    compute_all_pairs_next_hop produces -- and that sequence must thread R.
    """
    fd, g = _tc18c_diamond(reset_node=True)
    next_hop = compute_all_pairs_next_hop(g, _tc18c_decay_params(_TC18C_K_DEFAULT))

    # Planner's decay-walked path == the sim's NEXT_HOP-walked path (parity).
    planner_path = walk_next_hop(next_hop, "S", "Gl")
    sim, chassis = _tc18c_sim(fd, g, next_hop)
    sim_path = walk_next_hop(sim._next_hop, "S", "Gl")    # pylint: disable=protected-access
    assert sim_path == planner_path

    # Decay actually chose the detour through the reset node.
    assert _path_has_reset(planner_path, fd), (
        f"expected the decay detour through R; got {planner_path}"
    )
    assert "R" in planner_path
    assert "A2" not in planner_path    # specifically NOT the shortcut

    # The sim drives that route and lands the wheel axle ON Gl (drift~0). The
    # reset node fires its WALL_STOP -> REANCHOR interlude mid-route. Do NOT
    # loosen _POSITION_TOL_CM to absorb drift -- the geometry is exact.
    prims = sim.go_to("Gl")
    assert sim.localization.current_node() == "Gl"
    gx, gy = fd.NODES["Gl"]
    assert math.hypot(chassis.x_cm - gx, chassis.y_cm - gy) < _POSITION_TOL_CM

    # The interlude fired exactly once, at R (pillar-2 behavior still holds
    # through a decay-chosen route).
    assert sum(1 for p in prims if p[0] == ACT_WALL_STOP) == 1
    assert sum(1 for p in prims if p[0] == ACT_REANCHOR) == 1


def test_tc18c_lowered_k_sim_walks_planner_shortcut_without_reset():
    """At lowered k the route flips to the shortcut and the sim mirrors it.

    The same fixture at k=0.05 (< 1/3 crossover): the tiny penalty no longer
    overcomes the 50 cm raw advantage, so both planner and sim take the
    reset-free shortcut S->A1->A2->Gl. Pairing this with the default-k test
    proves the decay knob actually INFLUENCED the route (the chosen path differs
    between the two k values) AND that the sim matches the planner in both.
    """
    fd, g = _tc18c_diamond(reset_node=True)
    next_hop = compute_all_pairs_next_hop(g, _tc18c_decay_params(_TC18C_K_LOWERED))

    planner_path = walk_next_hop(next_hop, "S", "Gl")
    sim, chassis = _tc18c_sim(fd, g, next_hop)
    sim_path = walk_next_hop(sim._next_hop, "S", "Gl")    # pylint: disable=protected-access
    assert sim_path == planner_path

    # Decay chose the shortcut; no reset is threaded.
    assert not _path_has_reset(planner_path, fd), (
        f"expected the reset-free shortcut; got {planner_path}"
    )
    assert "R" not in planner_path
    assert "A1" in planner_path and "A2" in planner_path

    # The sim drives the shortcut and lands axle-on-Gl with no interlude.
    prims = sim.go_to("Gl")
    assert sim.localization.current_node() == "Gl"
    gx, gy = fd.NODES["Gl"]
    assert math.hypot(chassis.x_cm - gx, chassis.y_cm - gy) < _POSITION_TOL_CM
    assert not any(p[0] in (ACT_WALL_STOP, ACT_REANCHOR) for p in prims)


def test_tc18c_decay_flips_the_route_between_default_and_lowered_k():
    """The default-k and lowered-k routes DIFFER -- decay is load-bearing.

    A single assertion that the chosen route is k-dependent: at the field's
    default k the sim threads the reset, at lowered k it does not. Without the
    decay model both would be the plain-length shortcut, so this difference is
    the proof that the sim's routing is genuinely decay-weighted (not a flat
    multiplier or a hardcoded table). Both routes are independently checked
    against the planner in the two tests above.
    """
    fd, g = _tc18c_diamond(reset_node=True)
    default_hop = compute_all_pairs_next_hop(g, _tc18c_decay_params(_TC18C_K_DEFAULT))
    lowered_hop = compute_all_pairs_next_hop(g, _tc18c_decay_params(_TC18C_K_LOWERED))

    default_path = walk_next_hop(default_hop, "S", "Gl")
    lowered_path = walk_next_hop(lowered_hop, "S", "Gl")

    assert default_path != lowered_path
    assert _path_has_reset(default_path, fd)
    assert not _path_has_reset(lowered_path, fd)


def test_tc18c_negative_control_no_reset_anchor_sim_takes_shortcut():
    """Negative control: with R relabeled blind, default k routes the shortcut.

    The same diamond geometry but the mid-detour node is a plain ``"blind"``
    node (not an anchor). The detour is then penalised even harder, so at the
    default k both planner and sim pick the shortcut and the chosen path threads
    no reset (there is none). This prevents the positive test from
    false-positiving on "it routed through R" -- the reset-threading there is
    driven by R's anchor status, not by the topology / raw lengths.
    """
    fd, g = _tc18c_diamond(reset_node=False)
    next_hop = compute_all_pairs_next_hop(g, _tc18c_decay_params(_TC18C_K_DEFAULT))

    planner_path = walk_next_hop(next_hop, "S", "Gl")
    sim, chassis = _tc18c_sim(fd, g, next_hop)
    sim_path = walk_next_hop(sim._next_hop, "S", "Gl")    # pylint: disable=protected-access
    assert sim_path == planner_path

    # No node is reset-typed, and the shortcut wins on weight alone.
    assert not _path_has_reset(planner_path, fd), f"no node is reset-typed; got {planner_path}"
    assert "A1" in planner_path and "A2" in planner_path

    prims = sim.go_to("Gl")
    assert sim.localization.current_node() == "Gl"
    gx, gy = fd.NODES["Gl"]
    assert math.hypot(chassis.x_cm - gx, chassis.y_cm - gy) < _POSITION_TOL_CM
    assert not any(p[0] in (ACT_WALL_STOP, ACT_REANCHOR) for p in prims)


def test_tc18c_k_zero_sim_matches_planner_plain_length_route():
    """At k=0 the decay model collapses to plain length and the sim mirrors it.

    The AC14b regression anchor at the sim level: with k=0 every blind edge
    costs its raw length, so the planner picks the shorter shortcut and the sim
    follows the same plain-length route -- proving the decay model is a strict
    generalisation of plain length and the sim agrees at the degenerate gain.
    """
    fd, g = _tc18c_diamond(reset_node=True)
    next_hop = compute_all_pairs_next_hop(g, _tc18c_decay_params(0.0))

    planner_path = walk_next_hop(next_hop, "S", "Gl")
    sim, chassis = _tc18c_sim(fd, g, next_hop)
    sim_path = walk_next_hop(sim._next_hop, "S", "Gl")    # pylint: disable=protected-access
    assert sim_path == planner_path

    # k=0 -> plain length -> the shorter shortcut (raw 150 < 200), no reset.
    assert planner_path == ["S", "A1", "A2", "Gl"]
    assert not _path_has_reset(planner_path, fd)

    sim.go_to("Gl")
    assert sim.localization.current_node() == "Gl"
    gx, gy = fd.NODES["Gl"]
    assert math.hypot(chassis.x_cm - gx, chassis.y_cm - gy) < _POSITION_TOL_CM


# ===========================================================================
# ADR-052 line-arrival heading reset -- TC15 sim<->firmware ACCEPT-path parity
# (T8, docs/plans/2026-07-03-line-arrival-heading-reset.md). T7 wired the sim
# mirror -- this module's ``ACT_HEADING_RESET`` emission inside ``_walk_hops``,
# gated by the ``LINE_HEADING_RESET_ON_PROJECTION`` / ``_ON_JUNCTION`` knobs --
# and the parity three-place contract (``sim/parity.py``'s
# ``TRACE_RECORD_FIELDS["heading_reset"]`` entry, the ``normalize_sim_stream``
# tag arm, and the no-op ``integrate_pose`` branch). These tests cover the
# sim's ACCEPT-path decision only: the sim has no IMU/drift model and never
# strict-rejects (AC12), so the reject path is host-C++-only
# (``WRO2026/test/test_reanchor_heading.cpp``, T3) and out of scope here.
# ===========================================================================


def _make_projection_pass_through_sim():
    """``Inter1`` --LINE--> ``ProjL`` [projection] --LINE--> ``Inter2``, collinear.

    The node names echo the shared ``WRO2026/test/fixtures/field_fixture.py``
    collinear-projection anchor the plan's Testing Strategy cites (its own
    ``Inter1-ProjL-Inter2`` triple on y=40), but the fixture itself is built
    inline -- the same ad-hoc minimal-field style
    ``test_pass_through_projection.py`` already uses (T7 adapted that exact
    file for the ``ACT_HEADING_RESET`` tag) -- rather than pulling in the
    cross-tree C++ fixture module the #116 parity harness links. No direct
    ``Inter1``->``Inter2`` edge exists, so the route is forced through
    ``ProjL`` (mirroring ``test_pass_through_projection.py``'s TC11 layout).

    :return: a fresh, zeroed ``(sim, field_data)`` pair seated on ``Inter1``.
    """
    nodes = {
        "Inter1": (60.0, 40.0),
        "ProjL": (100.0, 40.0),
        "Inter2": (140.0, 40.0),
    }
    graph = nx.DiGraph()
    for name, (x, y) in nodes.items():
        graph.add_node(
            name,
            x_cm=float(x),
            y_cm=float(y),
            type="projection" if name == "ProjL" else "line",
        )
    for u, v, dist, hdg in (
        ("Inter1", "ProjL", 40.0, 0.0),
        ("ProjL", "Inter2", 40.0, 0.0),
    ):
        graph.add_edge(u, v, type=EDGE_LINE, distance_cm=dist, heading_deg=hdg)
        graph.add_edge(
            v, u, type=EDGE_LINE, distance_cm=dist, heading_deg=(hdg + 180.0) % 360.0
        )
    next_hop = {
        "Inter1": {"Inter1": "Inter1", "ProjL": "ProjL", "Inter2": "ProjL"},
        "ProjL": {"Inter1": "Inter1", "ProjL": "ProjL", "Inter2": "Inter2"},
        "Inter2": {"Inter1": "ProjL", "ProjL": "ProjL", "Inter2": "Inter2"},
    }
    fd = _minimal_field(nodes, "Inter1", 0.0)
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, fd


def test_adr052_tc15a_projection_pass_through_fires_heading_reset_and_matches_record():
    """TC15 / AC12: a collinear PROJECTION pass-through fires ``ACT_HEADING_RESET``
    immediately after the pass-through arrival, bound to the incoming edge's
    body-corrected heading -- and a fresh sim's normalized stream carries the
    matching ``heading_reset`` :class:`TraceRecord`.
    """
    sim, _fd = _make_projection_pass_through_sim()
    prims = sim.go_to("Inter2")

    # (1) The distance-gated pass-through actually fired at ProjL (not a stop) --
    #     a test that silently bypassed ProjL would be vacuous.
    assert prims[0] == (ACT_FOLLOW_DIST_PASS, 40.0)
    # (2) The heading reset fired at ProjL, "proj" kind, bound to the
    #     Inter1->ProjL FRONT body heading (0.0 deg).
    reset_prim = (ACT_HEADING_RESET, "ProjL", "proj", 0.0)
    assert reset_prim in prims
    # (3) It fires immediately after the pass-through arrival -- before the
    #     onward ProjL->Inter2 hop is planned.
    assert prims[prims.index(reset_prim) - 1] == (ACT_FOLLOW_DIST_PASS, 40.0)

    # (4) A FRESH sim (normalize_sim_stream drives it to completion, so it is
    #     single-use per case) produces a matching heading_reset TraceRecord.
    sim2, _fd2 = _make_projection_pass_through_sim()
    case = ParityCase(case_id="Inter1__Inter2__clean", start="Inter1", goal="Inter2")
    records = normalize_sim_stream(sim2, case)
    heading_reset_records = [r for r in records if r.kind == "heading_reset"]
    assert len(heading_reset_records) == 1
    assert heading_reset_records[0].fields == {
        "node": "ProjL",
        "pass_kind": "proj",
        "heading_deg": 0.0,
    }


def test_adr052_tc15b_junction_pass_through_silent_by_default():
    """TC15 / AC8: a collinear LINE (physical junction) pass-through does NOT
    fire ``ACT_HEADING_RESET`` while ``LINE_HEADING_RESET_ON_JUNCTION`` sits at
    its firmware-matching default (``False``) -- even though the pass-through
    itself (``ACT_LINE_PASS``) still fires at both intermediate nodes.
    """
    fd = collinear_4node.make_field_data()
    graph = collinear_4node.build_graph()
    next_hop = collinear_4node.build_next_hop()
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("D")
    act_tags = [p[0] for p in prims]

    # Sanity: the junction pass-through DID fire at both B and C -- a test that
    # never reached a pass-through node would be vacuous.
    assert act_tags.count(ACT_LINE_PASS) == 2
    assert ACT_HEADING_RESET not in act_tags


def test_adr052_tc15c_junction_pass_through_fires_when_knob_enabled(monkeypatch):
    """TC15 / AC8: with ``LINE_HEADING_RESET_ON_JUNCTION`` monkeypatched ``True``
    (mirroring how TC8/TC9 patch ``PATH_PASS_THROUGH_ENABLED`` on ``_runtime``),
    the SAME collinear LINE junction pass-through fires ``ACT_HEADING_RESET`` at
    both B and C -- proving the knob, not the pass-through eligibility, gates
    the junction-kind emission.
    """
    monkeypatch.setattr(_runtime, "LINE_HEADING_RESET_ON_JUNCTION", True)

    fd = collinear_4node.make_field_data()
    graph = collinear_4node.build_graph()
    next_hop = collinear_4node.build_next_hop()
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("D")

    assert (ACT_HEADING_RESET, "B", "junction", 0.0) in prims
    assert (ACT_HEADING_RESET, "C", "junction", 0.0) in prims
    # D is the goal (LINE_OVERSHOOT, never a pass-through) -- no reset there.
    assert not any(p[0] == ACT_HEADING_RESET and p[1] == "D" for p in prims)


def test_adr052_tc15d_projection_knob_off_suppresses_heading_reset(monkeypatch):
    """TC15 / AC7+knob: monkeypatching ``LINE_HEADING_RESET_ON_PROJECTION`` to
    ``False`` suppresses the reset on the SAME projection pass-through that
    fires it by default (TC15a) -- proving the knob, not pass-through
    eligibility, gates the emission (mirrors the sim exactly as the firmware's
    knob gates ``_maybeHeadingReset``).
    """
    monkeypatch.setattr(_runtime, "LINE_HEADING_RESET_ON_PROJECTION", False)

    sim, _fd = _make_projection_pass_through_sim()
    prims = sim.go_to("Inter2")
    act_tags = [p[0] for p in prims]

    # The pass-through itself is unaffected by the knob -- only the reset is gated.
    assert prims[0] == (ACT_FOLLOW_DIST_PASS, 40.0)
    assert ACT_HEADING_RESET not in act_tags


def test_adr052_tc15e_projection_goal_not_pass_through_no_heading_reset():
    """TC15 / AC10 (structural): when the projection node IS the goal, the
    not-final guard makes it a plain DISTANCE stop -- never a pass-through --
    so no ``ACT_HEADING_RESET`` fires even though the knob defaults ON and the
    geometry would otherwise qualify (mirrors
    ``test_pass_through_projection.py``'s TC12 fixture).

    Reuses :func:`_make_projection_pass_through_sim` (the SAME collinear
    ``Inter1 --LINE--> ProjL[projection] --LINE--> Inter2`` triple TC15a fires
    the reset on) so this AC10 regression cannot drift onto a stale topology --
    only the goal differs (``ProjL`` itself, not ``Inter2``), which is exactly
    the not-final guard under test.
    """
    sim, _fd = _make_projection_pass_through_sim()

    prims = sim.go_to("ProjL")

    # 40.0 == the helper's Inter1->ProjL edge distance (immaterial to the
    # assertion's intent: the hop is a plain DISTANCE stop, not a pass-through).
    assert prims == [(ACT_FOLLOW_DIST, 40.0)]
    assert ACT_HEADING_RESET not in [p[0] for p in prims]
    assert sim.localization.current_node() == "ProjL"


def test_adr052_tc15f_ordinary_toy_route_no_heading_reset(fresh_sim):
    """TC15 / AC10 (structural, regression): an ordinary toy-field route with no
    collinear pass-through anywhere never emits ``ACT_HEADING_RESET``, under
    the default (projection ON / junction OFF) knob settings.
    """
    prims = fresh_sim.go_to("B")
    assert ACT_HEADING_RESET not in [p[0] for p in prims]


def test_adr052_tc15g_heading_reset_record_is_noop_on_pose():
    """TC15 / AC12: applying a ``heading_reset`` record through
    :func:`integrate_pose` leaves the virtual chassis pose UNCHANGED -- the
    reset is belief-only (the sim's heading is already ground truth, so there
    is no drift for it to correct), unlike ``wall_reanchor``'s real coordinate
    correction.
    """
    field = SimpleNamespace(
        NODES={"ProjL": (100.0, 40.0)},
        START_NODE="ProjL",
        START_HEADING_DEG=0.0,
    )
    record = TraceRecord(
        "heading_reset",
        {"node": "ProjL", "pass_kind": "proj", "heading_deg": 37.5},
    )

    poses = integrate_pose(
        [record],
        field,
        node_sequence=[],
        start_node="ProjL",
        start_heading_deg=12.3,
    )

    assert len(poses) == 1
    pose = poses[0]
    assert pose.x == pytest.approx(100.0, abs=1e-9)
    assert pose.y == pytest.approx(40.0, abs=1e-9)
    assert pose.heading == pytest.approx(12.3, abs=1e-9)


def test_adr052_tc15h_normalize_sim_stream_unknown_act_tag_loud_fails():
    """TC15 (three-place parity contract): :func:`normalize_sim_stream` loud-fails
    with :class:`ParitySchemaError` on an ACT tag it does not recognize --
    documenting that a sim-side tag added without a matching normalizer arm (or
    a genuine schema desync) is a hard failure, never a silent skip
    (``parity.py``'s ``else: raise`` guard, ~line 940).
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (30.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_LINE, 30.0, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    class _FakeSnapshot:
        heading_deg = 0.0

    def _bogus_stream(*_args, **_kwargs):
        yield (("__adr052_test_bogus_tag__", "A", "junction", 0.0), _FakeSnapshot())

    # Instance-level shadow (a plain function attribute, not a bound method) of
    # step_through -- normalize_sim_stream drives off whatever stream this
    # returns, so this is the narrowest way to inject an unrecognized tag
    # without hand-building a full FSM desync.
    sim.step_through = _bogus_stream

    case = ParityCase(case_id="A__B__clean", start="A", goal="B")
    with pytest.raises(ParitySchemaError, match="unknown sim ACT tag"):
        normalize_sim_stream(sim, case)


# ===========================================================================
# W1 / TC5 -- STOP_AT_LINE fold resolution-equivalence (ADR-064, issues #97/#99)
#
# The ADR-064 fold removes the per-edge ``STOP_AT_LINE`` BLIND mode and lowers a
# legacy ``STOP_AT_LINE`` edge to the canonical
# ``mode: SYNC_TSHAPE + stop_at_line: on`` form. TC5 pins that the two forms
# resolve to the SAME ``(seek, motion_mode)`` through ``resolve_stop_at_line``
# across all FOUR degrade triggers -- with ``motion_mode`` pinned SYNC_TSHAPE --
# so the fold cannot silently change the seek / dead-reckon decision. This is a
# RESOLUTION-EQUIVALENCE assertion (``{seek, motionMode}`` equal), NOT a
# bit-compare against a pre-fold golden. Degrade semantics per
# ``WRO2026/src/nav/StopAtLinePolicy.h`` and the plan's Decisions bullet on the
# fold.
#
# ``pipeline_blind`` is SYNC_TSHAPE -- today's ``PIPELINE_CONFIG.blind`` default,
# the case the plan documents as behaviour-unchanged: the pre-fold degraded seek
# dead-reckoned under ``PIPELINE_CONFIG.blind`` (StopAtLinePolicy.h:182-183), and
# the explicit ``SYNC_TSHAPE`` lowering pins that degrade to SYNC to match the
# non-degraded seek mode a bare ``STOP_AT_LINE`` edge already runs
# (StopAtLinePolicy.h:173-174).
# ===========================================================================

# Each row names one degrade trigger. ``legacy_override`` / ``lowered_override``
# are the per-edge ``stop_at_line_override`` each form carries: the legacy
# ``STOP_AT_LINE`` edge carries no seek override of its own (DEFAULT) and seeks
# via the legacy force-on tag; the lowered form bakes ``stop_at_line: on``
# (StopAtLineMode.ON).
#
# Trigger 4 (``edge_override_off``) is the one the lowered case CANNOT express as
# a ``STOP_AT_LINE`` + override-OFF pair: the canonical lowering pins the
# override to ON, so a lowered edge can never simultaneously carry OFF. What IS
# expressible is a plain non-seeking SYNC edge (``mode: SYNC_TSHAPE +
# stop_at_line: off``), so BOTH forms carry override OFF on that row and must
# still agree -- pinning that "author a non-seek SYNC edge two ways" resolves
# identically (the ADR-048 ``STOP_AT_LINE + off`` contradiction is now simply
# inexpressible).
_TC5_FOLD_TRIGGERS = [
    # (id, call_flag, direction, line_board_count,
    #  legacy_override, lowered_override, expect_seek)
    ("normal_seek", StopAtLine.AUTO, Direction.FRONT, 2,
     StopAtLineMode.DEFAULT, StopAtLineMode.ON, True),
    ("rear_1board", StopAtLine.AUTO, Direction.REAR, 1,
     StopAtLineMode.DEFAULT, StopAtLineMode.ON, False),
    ("call_off", StopAtLine.OFF, Direction.FRONT, 2,
     StopAtLineMode.DEFAULT, StopAtLineMode.ON, False),
    ("edge_override_off", StopAtLine.AUTO, Direction.FRONT, 2,
     StopAtLineMode.OFF, StopAtLineMode.OFF, False),
]


@pytest.mark.parametrize(
    "trigger",
    _TC5_FOLD_TRIGGERS,
    ids=[row[0] for row in _TC5_FOLD_TRIGGERS],
)
def test_tc5_stop_at_line_fold_resolution_equivalence(trigger):
    """TC5: legacy STOP_AT_LINE == lowered SYNC_TSHAPE+ON resolve identically.

    For each degrade trigger the legacy per-edge ``BlindMode.STOP_AT_LINE`` edge
    and its lowered ``mode: SYNC_TSHAPE + stop_at_line: on`` twin must produce the
    SAME ``(seek, motion_mode)`` from ``resolve_stop_at_line``, and that
    ``motion_mode`` must be ``SYNC_TSHAPE`` (the fold's SYNC pin). ``expect_seek``
    is asserted too so the equivalence is non-vacuous -- a regression that flipped
    the seek decision or degraded to a different motion bundle fails here.
    """
    (
        _id,
        call_flag,
        direction,
        board_count,
        legacy_override,
        lowered_override,
        expect_seek,
    ) = trigger
    pipeline_blind = BlindMode.SYNC_TSHAPE

    legacy = resolve_stop_at_line(
        call_flag,
        legacy_override,
        BlindMode.STOP_AT_LINE,   # the legacy per-edge STOP_AT_LINE tag
        pipeline_blind,
        direction,
        board_count,
        False,                    # auto_rule_active
    )
    lowered = resolve_stop_at_line(
        call_flag,
        lowered_override,
        BlindMode.SYNC_TSHAPE,    # the lowered `mode: SYNC_TSHAPE`
        pipeline_blind,
        direction,
        board_count,
        False,                    # auto_rule_active
    )

    # Resolution-equivalence: both forms agree on the whole {seek, motion_mode}.
    assert legacy == lowered, (
        f"fold divergence on trigger {_id!r}: legacy={legacy!r} lowered={lowered!r}"
    )
    # ... and both land on the expected seek with motion pinned SYNC_TSHAPE.
    assert legacy == (expect_seek, BlindMode.SYNC_TSHAPE)


# ===========================================================================
# W2 -- sim tier-branch coverage (ADR-064, the sim legs of TC10 + TC14)
#
# Drives PathTrackerSim over synthetic collinear BLIND fields to pin three
# per-edge-speed behaviours in the offline mirror so it cannot silently drift
# from the firmware:
#   (a) a chain breaks at a resolved-tier boundary, but SAME-tier hops still
#       chain across a SYNC<->GYRO mode boundary (TC10 / AC7);
#   (b) a FAST-line -> collinear SLOW-blind pass-through clamps the blind entry
#       seed to <= the SLOW cruise (TC14 / AC12);
#   (c) a BLIND edge cruises at its resolved per-tier PWM (SLOW=60 / FAST=90)
#       via resolve_speed -> _speed_tier_to_pwm (TC1 / AC1).
# ===========================================================================


def _linear_next_hop(names) -> dict:
    """Dense next-hop table for a linear chain ``names[0] -> ... -> names[-1]``.

    Every source routes one step toward the target along the linear order (the
    successor toward a later target, the predecessor toward an earlier one), so
    the table covers both forward and return goals without invoking the planner.
    """
    idx = {name: i for i, name in enumerate(names)}
    table: dict = {}
    for src in names:
        row: dict = {}
        for tgt in names:
            if src == tgt:
                row[tgt] = src
            elif idx[tgt] > idx[src]:
                row[tgt] = names[idx[src] + 1]
            else:
                row[tgt] = names[idx[src] - 1]
        table[src] = row
    return table


def _collinear_blind_chain_sim(hop_specs, *, distance_cm: float = 40.0):
    """Build a sim over a collinear (heading 0) BLIND chain with per-hop overrides.

    ``hop_specs`` is one dict per hop; each may carry ``"speed"`` (a
    :class:`SpeedTier`) and/or ``"mode"`` (a :class:`BlindMode`) that become the
    corresponding graph-edge override attributes on both directions. Nodes are
    laid out ``A, B, C, ...`` along ``+x`` spaced ``distance_cm`` apart so the
    geometry matches the 0 deg heading, and every node is typed ``"blind"`` so no
    projection auto-seek fires. ``distance_cm`` defaults to 40 cm -- well above
    ``TSHAPE_MIN_MOVE_CM`` (chain-eligible) and long enough for the ramp to reach
    cruise.

    :returns: ``(sim, chassis)`` with localization already zeroed at ``A``.
    """
    names = [chr(ord("A") + i) for i in range(len(hop_specs) + 1)]
    nodes = {name: (float(i) * distance_cm, 0.0) for i, name in enumerate(names)}
    fd = _minimal_field(nodes=nodes, start_node=names[0], start_heading_deg=0.0)

    g = nx.DiGraph()
    for name in names:
        g.add_node(name, type="blind")
    for i, spec in enumerate(hop_specs):
        u, v = names[i], names[i + 1]
        fwd = dict(type=EDGE_BLIND, distance_cm=float(distance_cm), heading_deg=0.0)
        rev = dict(type=EDGE_BLIND, distance_cm=float(distance_cm), heading_deg=180.0)
        for key in ("speed", "mode"):
            if key in spec:
                fwd[key] = spec[key]
                rev[key] = spec[key]
        g.add_edge(u, v, **fwd)
        g.add_edge(v, u, **rev)

    next_hop = _linear_next_hop(names)
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim, chassis


def test_w2a_blind_chain_breaks_at_tier_boundary():
    """W2(a): two collinear BLIND hops of DIFFERENT resolved tiers do NOT chain.

    ``A ->(NORMAL) B ->(SLOW) C``: the two hops are collinear + chain-eligible in
    every respect except their resolved :class:`SpeedTier`, so the chain must end
    at the tier boundary (a single velocity run cannot straddle two cruises).
    Each hop therefore dispatches as its own braked, PRISTINE ``(ACT_FORWARD,
    dist)`` move -- no ``ACT_BLIND_PASS`` boundary marker and no velocity suffix.

    Non-vacuous: if the SPEED-only chain restriction regressed, the walk-ahead
    would fold B into hop 0's run and emit ``(ACT_BLIND_PASS, "B")`` plus 5-tuple
    velocity suffixes (driving the SLOW hop at the NORMAL cruise) -- which these
    assertions reject. The same-tier control below proves the two hops WOULD
    otherwise chain.
    """
    sim, _chassis = _collinear_blind_chain_sim(
        [{"speed": SpeedTier.NORMAL}, {"speed": SpeedTier.SLOW}]
    )

    prims = sim.go_to("C")
    tags = [p[0] for p in prims]

    # The chain broke at the tier boundary: node B was braked at, not passed.
    assert ACT_BLIND_PASS not in tags
    forwards = [p for p in prims if p[0] == ACT_FORWARD]
    assert len(forwards) == 2
    # Both hops are the pristine 2-tuple (no chained velocity suffix).
    assert all(len(p) == 2 for p in forwards), forwards
    assert sim.localization.current_node() == "C"


def test_w2a_same_tier_chains_across_mode_boundary():
    """W2(a) control: two SAME-tier BLIND hops chain across a SYNC<->GYRO boundary.

    ``A ->(NORMAL, SYNC_TSHAPE) B ->(NORMAL, GYRO_TSHAPE) C``: same resolved tier,
    different drive mode. The blind-chain rule restricts on SPEED tier ONLY, so
    the mode boundary must NOT break the run -- the two hops fold into one
    continuous motion with an ``(ACT_BLIND_PASS, "B")`` marker at the non-braking
    node and a chained velocity suffix on each hop.

    Non-vacuous: a (hypothetical) mode-equality restriction would split this into
    two braked hops with no ``ACT_BLIND_PASS`` -- which these assertions reject
    (AC7: same-tier hops still chain, mode mixing unchanged).
    """
    sim, _chassis = _collinear_blind_chain_sim(
        [
            {"speed": SpeedTier.NORMAL, "mode": BlindMode.SYNC_TSHAPE},
            {"speed": SpeedTier.NORMAL, "mode": BlindMode.GYRO_TSHAPE},
        ]
    )

    prims = sim.go_to("C")
    tags = [p[0] for p in prims]

    # The mode boundary did not break the chain: B was passed without braking.
    assert tags.count(ACT_BLIND_PASS) == 1
    assert (ACT_BLIND_PASS, "B") in prims
    # Both hops carry the chained 5-tuple velocity suffix (chain flag True).
    forwards = [p for p in prims if p[0] == ACT_FORWARD]
    assert len(forwards) == 2
    assert all(len(p) == 5 and p[4] is True for p in forwards), forwards
    assert sim.localization.current_node() == "C"


def test_w2b_passthrough_entry_seed_clamped_to_slow_cruise():
    """W2(b): a FAST-line -> collinear SLOW-blind pass-through clamps entry <= 60.

    ``A ->LINE(FAST) B ->BLIND(SLOW) C`` collinear: the LINE pass-through at the
    junction B keeps the follower live, so the B->C blind hop is
    PASS-THROUGH-SEEDED. Its entry seed is clamped to the hop's own resolved
    cruise, so a fast inbound speed cannot enter the SLOW blind hop above the SLOW
    cruise (60).

    Documented sim limitation (pre-existing, not ADR-064): the captured cruise is
    modeled as ``SPEED_NORMAL`` (70), not the true FAST preset -- so this pins the
    CLAMP DIRECTION (entry <= SLOW cruise), not the exact captured magnitude. The
    clamp pulls the modeled 70 down to the SLOW cruise (60).

    Non-vacuous: without the clamp the seed would be the un-clamped ``SPEED_NORMAL``
    (70), which is both ``> 60`` and ``>= SPEED_NORMAL`` -- both assertions below
    reject that.
    """
    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0), "C": (100.0, 0.0)}
    fd = _minimal_field(nodes=nodes, start_node="A", start_heading_deg=0.0)

    g = nx.DiGraph()
    g.add_node("A", type="line")
    g.add_node("B", type="line")
    g.add_node("C", type="blind")
    # A->B LINE, FAST. (The sim models the captured pass-through cruise as
    # SPEED_NORMAL regardless of the LINE tier, so FAST here is the authored
    # scenario, not the seed magnitude -- see the docstring.)
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=50.0, heading_deg=0.0,
               speed=SpeedTier.FAST)
    g.add_edge("B", "A", type=EDGE_LINE, distance_cm=50.0, heading_deg=180.0,
               speed=SpeedTier.FAST)
    # B->C BLIND, SLOW -> cruise 60.
    g.add_edge("B", "C", type=EDGE_BLIND, distance_cm=50.0, heading_deg=0.0,
               speed=SpeedTier.SLOW)
    g.add_edge("C", "B", type=EDGE_BLIND, distance_cm=50.0, heading_deg=180.0,
               speed=SpeedTier.SLOW)

    next_hop = _linear_next_hop(["A", "B", "C"])
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("C")

    # The B->C blind hop is a lone pass-through-seeded hop (C is the goal), so it
    # rides the resolved (start_pwm, end_pwm, chain) suffix, chain False.
    seeded = [p for p in prims if p[0] == ACT_FORWARD and len(p) == 5]
    assert len(seeded) == 1, prims
    start_pwm = seeded[0][2]

    slow_cruise = _runtime._speed_tier_to_pwm(SpeedTier.SLOW)   # 60
    # The clamp caps the entry seed at the SLOW cruise ...
    assert start_pwm <= slow_cruise
    # ... and demonstrably engaged: it pulled the seed below the un-clamped
    # SPEED_NORMAL the sim models the captured cruise as.
    assert start_pwm < _runtime.SPEED_NORMAL
    assert sim.localization.current_node() == "C"


def test_w2c_per_tier_blind_cruise_resolves_to_preset():
    """W2(c): resolve_speed -> _speed_tier_to_pwm maps SLOW->60 and FAST->90.

    The direct pin on the sim's per-edge blind cruise resolution -- the exact
    composition the dispatch (``cruise_mag = _speed_tier_to_pwm(resolve_speed(
    edge))``) and the chain walk-ahead both use. ADR-064 blind speed reuses the
    line presets (SLOW/NORMAL/FAST = 60/70/90).
    """
    assert _runtime._speed_tier_to_pwm(resolve_speed({"speed": SpeedTier.SLOW})) == 60
    assert _runtime._speed_tier_to_pwm(resolve_speed({"speed": SpeedTier.NORMAL})) == 70
    assert _runtime._speed_tier_to_pwm(resolve_speed({"speed": SpeedTier.FAST})) == 90
    # Tie the literals to the module's Config mirrors so a retune moves together.
    assert _runtime.SPEED_SLOW == 60
    assert _runtime.SPEED_FAST == 90


@pytest.mark.parametrize(
    "tier, expected_cruise",
    [
        (SpeedTier.SLOW, 60),
        (SpeedTier.FAST, 90),
    ],
    ids=["SLOW_cruise_60", "FAST_cruise_90"],
)
def test_w2c_per_tier_blind_cruise_drives_chain_at_preset(tier, expected_cruise):
    """W2(c): a same-tier BLIND chain cruises at its resolved per-tier PWM.

    A 3-hop collinear chain of one tier is folded into a single continuous run;
    the peak commanded PWM magnitude across the emitted chain suffix is the run
    cruise, which must equal the tier's resolved preset (SLOW=60, FAST=90) -- NOT
    the hardcoded NORMAL (70) blind speed used before ADR-064.

    Non-vacuous: if per-edge blind speed regressed to a constant NORMAL, both the
    SLOW and FAST chains would peak at 70 -- failing the 60 / 90 assertion.
    """
    sim, _chassis = _collinear_blind_chain_sim(
        [{"speed": tier}, {"speed": tier}, {"speed": tier}]
    )

    prims = sim.go_to("D")

    forwards = [p for p in prims if p[0] == ACT_FORWARD]
    assert len(forwards) == 3
    assert all(len(p) == 5 for p in forwards), forwards
    # Peak commanded magnitude across every hop's (start_pwm, end_pwm) == cruise.
    pwm_mags = [abs(value) for p in forwards for value in (p[2], p[3])]
    assert max(pwm_mags) == expected_cruise
    assert expected_cruise == _runtime._speed_tier_to_pwm(tier)
    assert sim.localization.current_node() == "D"
