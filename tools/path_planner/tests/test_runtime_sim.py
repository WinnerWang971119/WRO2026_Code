"""End-to-end tests for the PathTracker runtime simulator (PR3).

PR3 (post T9) emission contract: every EDGE_LINE traversal emits the
canonical ``(ACT_FOLLOW_INT, None)`` followed by a runtime-only
``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)`` step that mirrors
the C++ LINE_OVERSHOOT FSM transition. EDGE_PROJECTION traversals emit
``(ACT_FOLLOW_DIST, distance_cm)`` and advance Localization via
``on_dead_reckoned_advance``. EDGE_BLIND traversals still emit
``(ACT_FORWARD, distance_cm)``.

These tests cover the runtime-sim equivalence proof (sim primitive
sequence == compile_path primitive sequence interleaved with the
sim-only LINE_OVERSHOOT steps), per-edge mode override resolution, and
the LINE_OVERSHOOT + PROJECTION_FOLLOW position contracts.
"""

from __future__ import annotations

import copy
import math
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
    EDGE_PROJECTION,
    build_graph,
)
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
    DirectionMode,
    LineMode,
    ProjectionMode,
    TurnMode,
)
from tools.path_planner.planner import compute_all_pairs_next_hop
from tools.path_planner.sim import (
    LocalizationSim,
    PathTrackerSim,
    UnreachableGoalError,
    VirtualChassis,
    walk_next_hop,
)
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.runtime import (
    ACT_BLIND_REVERSE,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    ACT_LINE_REVERSE,
    ACT_PROJECTION_REVERSE,
    ACT_STOP_AT_LINE,
    ACT_STOP_AT_LINE_REVERSE,
    Direction,
    LINE_SENSOR_AXLE_OFFSET_CM,
    resolve_blind_mode,
    resolve_direction,
    resolve_line_mode,
    resolve_pre_turn_mode,
    resolve_projection_mode,
)
from tools.path_planner.tests.fixtures import bidirectional_field_data

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
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
    "FIELD_MM",
    "MAP_PNG",
)


def _clone_field_data() -> SimpleNamespace:
    """Return a SimpleNamespace mirror of the toy field MINUS the EDGES list.

    Stripping EDGES keeps the demo PROJECTION + pivot fixtures out of the
    baseline test graph so ``apply_pivot_compensations`` does not abort on
    the C->E pivot landing in a 4-out-degree node. New tests that need to
    exercise PROJECTION or per-edge overrides build their own minimal
    fixtures rather than mutating the shared toy field.
    """
    snapshot = {attr: copy.deepcopy(getattr(toy_field_data, attr)) for attr in _FIELD_ATTRS}
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
        next_hop = compute_all_pairs_next_hop(graph, toy_field.COST_BLIND_MULTIPLIER)
    return graph, next_hop


@pytest.fixture
def fresh_sim(toy_field, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field)
    sim = PathTrackerSim(toy_field, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim


def _interleave_overshoot(primitives: List[Primitive]) -> List[Primitive]:
    """Insert ``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)`` after each LINE primitive.

    The runtime sim (post-T9) emits a LINE_OVERSHOOT step after every
    EDGE_LINE traversal so the wheel axle (not the line sensor) lands on
    the node. ``compile_path`` deliberately does NOT emit this step (the
    C++ runtime owns it as an FSM state), so to compare the sim's output
    against ``compile_path``'s output we must re-add the LINE_OVERSHOOT
    steps in the same positions the sim emits them.

    Post-T12, EDGE_LINE traversals on REAR hops emit ``ACT_LINE_REVERSE``
    instead of ``ACT_FOLLOW_INT`` but still trigger the LINE_OVERSHOOT
    FSM transition; the helper therefore matches either primitive name.

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
        if prim[0] in (ACT_FOLLOW_INT, ACT_LINE_REVERSE):
            out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM))
    return out


def _line_count(primitives: List[Primitive]) -> int:
    """Return the number of ``(ACT_FOLLOW_INT, None)`` primitives in @p primitives."""
    return sum(1 for action, _ in primitives if action == ACT_FOLLOW_INT)


# -- baseline equivalence + reachability (post-T9 rebaselined) ---------------


def test_goto_B_matches_compile_path(toy_field, fresh_sim, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    primitives = fresh_sim.go_to("B")
    walked = walk_next_hop(next_hop, "A", "B")
    expected = compile_path(walked, toy_field.START_HEADING_DEG, graph, toy_field)
    assert primitives == _interleave_overshoot(expected)
    bx, by = toy_field.NODES["B"]
    # Position contract: each LINE edge over-advances by LINE_SENSOR_AXLE_OFFSET_CM.
    n_line = _line_count(expected)
    extra = n_line * LINE_SENSOR_AXLE_OFFSET_CM
    expected_x = bx + extra  # path A->B is heading 0, so the overshoot is along +x.
    assert math.hypot(fresh_sim.chassis.x_cm - expected_x, fresh_sim.chassis.y_cm - by) < _POSITION_TOL_CM
    assert fresh_sim.localization.current_node() == "B"


def test_goto_C_via_path(toy_field, fresh_sim, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    primitives = fresh_sim.go_to("C")
    walked = walk_next_hop(next_hop, "A", "C")
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
        next_hop = compute_all_pairs_next_hop(graph, fd.COST_BLIND_MULTIPLIER)
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


def test_goto_before_zero_raises(toy_field, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field)
    sim = PathTrackerSim(toy_field, graph, next_hop, chassis)
    with pytest.raises(RuntimeError) as excinfo:
        sim.go_to("C")
    assert "before" in str(excinfo.value).lower()


def test_every_reachable_goal_lands_at_node(toy_field, toy_graph_and_table):
    """Every reachable goal updates Localization to that goal.

    Position tolerance is relaxed (vs the pre-T9 sub-mm bound) because the
    LINE_OVERSHOOT step deliberately over-advances the chassis by
    ``LINE_SENSOR_AXLE_OFFSET_CM`` past every intersection -- the sim's
    chassis carries the cumulative overshoot, so a node-level position
    equality check is no longer the right invariant. The contract is
    instead: Localization.current_node() advances to the goal.
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


def test_walk_next_hop_self(toy_graph_and_table):
    _, next_hop = toy_graph_and_table
    assert walk_next_hop(next_hop, "A", "A") == ["A"]


# ---------------------------------------------------------------------------
# Test-fixture builders for TC9 / TC10 (synthetic minimal graphs)
# ---------------------------------------------------------------------------


def _minimal_field(nodes: dict, start_node: str, start_heading_deg: float):
    """Return a SimpleNamespace usable in place of field_data for the sim.

    The returned object exposes the attribute surface the sim consumes
    (``NODES`` + ``START_NODE`` + ``START_HEADING_DEG``). We only need
    this thin shape because :class:`PathTrackerSim` doesn't reach into the
    other field-data attributes once the graph + next-hop table are built.
    """
    return SimpleNamespace(
        NODES=nodes,
        START_NODE=start_node,
        START_HEADING_DEG=start_heading_deg,
    )


def _single_edge_graph(
    u: str,
    v: str,
    edge_type: str,
    distance_cm: float,
    heading_deg: float,
) -> nx.DiGraph:
    """Build a 2-node DiGraph with a single directed edge.

    Skips the validator + builder path entirely so the test fixture stays
    independent of field_data schema rules. Adds the reverse edge so
    Dijkstra's all-pairs sweep doesn't trip over a dead-end target.
    """
    g = nx.DiGraph()
    g.add_node(u)
    g.add_node(v)
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
    and advance the chassis x by 30 + LINE_SENSOR_AXLE_OFFSET_CM cm (heading 0,
    so the advance lies entirely along +x).
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
    assert chassis.x_cm == pytest.approx(30.0 + LINE_SENSOR_AXLE_OFFSET_CM, abs=1e-9)
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


def test_sim_projection_edge_advances_position_and_localization():
    """PathTrackerSim handles EDGE_PROJECTION via follow_line_to_distance + on_dead_reckoned_advance.

    A 12 cm PROJECTION edge A->B heading 0 must produce exactly
    ``[(ACT_FOLLOW_DIST, 12.0)]`` (no LINE_OVERSHOOT step — the encoder
    gate already lands the chassis on the node), advance the chassis x by
    exactly 12 cm, and advance Localization.current_node() to ``B``.
    """
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (12.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_PROJECTION, 12.0, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FOLLOW_DIST, 12.0)]
    assert chassis.x_cm == pytest.approx(12.0, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_sim_projection_edge_step_through_emits_follow_dist_only():
    """step_through over a PROJECTION edge yields only the FOLLOW_DIST step (no overshoot)."""
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (12.0, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_PROJECTION, 12.0, 0.0)
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
# TC11 — per-edge override resolution (LineMode / BlindMode / ProjectionMode / TurnMode)
# ---------------------------------------------------------------------------


def test_resolve_line_mode_default_falls_back_to_pipeline_config():
    """resolve_line_mode: missing/DEFAULT override -> PIPELINE_CONFIG.line, concrete -> as-is."""
    # Edge with no primitive_mode attr -> fallback.
    edge_missing: dict = {}
    assert resolve_line_mode(edge_missing) == PIPELINE_CONFIG.line

    # Edge with DEFAULT sentinel -> fallback.
    edge_default = {"primitive_mode": LineMode.DEFAULT}
    assert resolve_line_mode(edge_default) == PIPELINE_CONFIG.line

    # Edge with concrete value -> returned as-is.
    edge_fast = {"primitive_mode": LineMode.FAST}
    assert resolve_line_mode(edge_fast) == LineMode.FAST

    # Raw int value -> coerced to enum, returned as-is.
    edge_raw_int = {"primitive_mode": int(LineMode.SLOW)}
    assert resolve_line_mode(edge_raw_int) == LineMode.SLOW


def test_resolve_blind_mode_default_falls_back_to_pipeline_config():
    """resolve_blind_mode: missing/DEFAULT override -> PIPELINE_CONFIG.blind, concrete -> as-is."""
    edge_missing: dict = {}
    assert resolve_blind_mode(edge_missing) == PIPELINE_CONFIG.blind

    edge_default = {"primitive_mode": BlindMode.DEFAULT}
    assert resolve_blind_mode(edge_default) == PIPELINE_CONFIG.blind

    edge_gyro = {"primitive_mode": BlindMode.GYRO_TSHAPE}
    assert resolve_blind_mode(edge_gyro) == BlindMode.GYRO_TSHAPE

    edge_raw = {"primitive_mode": BlindMode.RAW_FLAT}
    assert resolve_blind_mode(edge_raw) == BlindMode.RAW_FLAT


def test_resolve_projection_mode_default_falls_back_to_pipeline_config():
    """resolve_projection_mode: missing/DEFAULT override -> PIPELINE_CONFIG.projection, concrete -> as-is."""
    edge_missing: dict = {}
    assert resolve_projection_mode(edge_missing) == PIPELINE_CONFIG.projection

    edge_default = {"primitive_mode": ProjectionMode.DEFAULT}
    assert resolve_projection_mode(edge_default) == PIPELINE_CONFIG.projection

    edge_slow = {"primitive_mode": ProjectionMode.SLOW}
    assert resolve_projection_mode(edge_slow) == ProjectionMode.SLOW

    edge_fast = {"primitive_mode": ProjectionMode.FAST}
    assert resolve_projection_mode(edge_fast) == ProjectionMode.FAST


def test_resolve_pre_turn_mode_default_falls_back_to_pipeline_config():
    """resolve_pre_turn_mode: missing/DEFAULT override -> PIPELINE_CONFIG.turn, concrete -> as-is.

    Note that resolve_pre_turn_mode looks up the 'pre_turn_mode' attr (not
    'primitive_mode'), since pre-turn mode and per-primitive mode are
    independent overrides on the same edge.
    """
    edge_missing: dict = {}
    assert resolve_pre_turn_mode(edge_missing) == PIPELINE_CONFIG.turn

    edge_default = {"pre_turn_mode": TurnMode.DEFAULT}
    assert resolve_pre_turn_mode(edge_default) == PIPELINE_CONFIG.turn

    edge_pivot_left = {"pre_turn_mode": TurnMode.PIVOT_LEFT}
    assert resolve_pre_turn_mode(edge_pivot_left) == TurnMode.PIVOT_LEFT

    edge_pivot_right = {"pre_turn_mode": TurnMode.PIVOT_RIGHT}
    assert resolve_pre_turn_mode(edge_pivot_right) == TurnMode.PIVOT_RIGHT


def test_resolve_line_mode_ignores_pre_turn_mode_attr():
    """An edge that ONLY sets pre_turn_mode must not influence line-mode resolution."""
    edge_only_pre_turn = {"pre_turn_mode": TurnMode.PIVOT_LEFT}
    assert resolve_line_mode(edge_only_pre_turn) == PIPELINE_CONFIG.line


def test_resolve_pre_turn_mode_ignores_primitive_mode_attr():
    """An edge that ONLY sets primitive_mode must not influence pre-turn-mode resolution."""
    edge_only_primitive = {"primitive_mode": LineMode.FAST}
    assert resolve_pre_turn_mode(edge_only_primitive) == PIPELINE_CONFIG.turn


def test_sim_passthrough_resolve_helpers_match_module_level(toy_field, toy_graph_and_table):
    """PathTrackerSim.resolve_*_mode instance methods mirror the module-level helpers."""
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field)
    sim = PathTrackerSim(toy_field, graph, next_hop, chassis)

    edge_line_fast = {"primitive_mode": LineMode.FAST}
    edge_blind_gyro = {"primitive_mode": BlindMode.GYRO_TSHAPE}
    edge_proj_slow = {"primitive_mode": ProjectionMode.SLOW}
    edge_turn_pivot = {"pre_turn_mode": TurnMode.PIVOT_RIGHT}

    assert sim.resolve_line_mode(edge_line_fast) == LineMode.FAST
    assert sim.resolve_blind_mode(edge_blind_gyro) == BlindMode.GYRO_TSHAPE
    assert sim.resolve_projection_mode(edge_proj_slow) == ProjectionMode.SLOW
    assert sim.resolve_pre_turn_mode(edge_turn_pivot) == TurnMode.PIVOT_RIGHT


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
    that the LINE_OVERSHOOT step advances the chassis by exactly
    ``LINE_SENSOR_AXLE_OFFSET_CM`` past the intersection marker and
    Localization advances to the next node.
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
    # Position contract: 30 cm FOLLOW_INT + 5 cm LINE_OVERSHOOT along +x.
    assert chassis.x_cm == pytest.approx(30.0 + LINE_SENSOR_AXLE_OFFSET_CM, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_projection_full_trapezoid():
    """A long PROJECTION edge runs ramp-up + plateau + ramp-down and lands on the node.

    Mirrors the hardware contract for a long PROJECTION edge: the C++
    ``LineFollower`` ramps up from 0 to cruise, holds the plateau, then
    ramps down to ``Config::PWM_MIN_MAGNITUDE`` before the distance gate
    actively brakes the chassis. The sim collapses all three phases into
    a single atomic ``follow_line_to_distance(100.0)`` advance, so the
    FSM-level invariants are: a single ``ACT_FOLLOW_DIST`` primitive, no
    ``ACT_LINE_OVERSHOOT`` (the encoder gate already lands the chassis on
    the node), and a chassis position exactly at the edge's terminus.
    """
    distance_cm = 100.0
    fd = _minimal_field(
        nodes={"A": (0.0, 0.0), "B": (distance_cm, 0.0)},
        start_node="A",
        start_heading_deg=0.0,
    )
    graph = _single_edge_graph("A", "B", EDGE_PROJECTION, distance_cm, 0.0)
    next_hop = _next_hop_two_node("A", "B")
    chassis = VirtualChassis.from_field_data(fd)
    sim = PathTrackerSim(fd, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()

    prims = sim.go_to("B")

    assert prims == [(ACT_FOLLOW_DIST, distance_cm)]
    # No overshoot step on PROJECTION — encoder gate lands the chassis on the node.
    assert not any(action == ACT_LINE_OVERSHOOT for action, _ in prims)
    assert chassis.x_cm == pytest.approx(distance_cm, abs=1e-9)
    assert chassis.y_cm == pytest.approx(0.0, abs=1e-9)
    assert sim.localization.current_node() == "B"


def test_projection_short_truncated_triangle():
    """A PROJECTION edge shorter than 2*TSHAPE_RAMP_MIN_CM collapses to a half-clamped triangle.

    With both ramp-up and ramp-down active on a very short edge, the C++
    ``LineFollower`` clamps each ramp to half the distance target so the
    two zones cannot overlap — producing a triangular profile with no
    plateau. The sim still collapses the whole thing to a single atomic
    ``follow_line_to_distance(cm)`` advance, so the FSM-level invariants
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
    graph = _single_edge_graph("A", "B", EDGE_PROJECTION, distance_cm, 0.0)
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
    """
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


def test_bidir_tc11_projection_edge_rear_emits_projection_reverse():
    """TC11: A PROJECTION edge traversed with REAR direction emits ACT_PROJECTION_REVERSE.

    Tuple shape: (ACT_PROJECTION_REVERSE, target_node, distance_cm, proj_mode).
    Per-edge FORCE_REVERSE override used for determinism.  Verifies AC11.
    """
    distance_cm = 25.0
    nodes = {"A": (0.0, 0.0), "B": (distance_cm, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=0.0,
    )
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge(
        "A", "B",
        type=EDGE_PROJECTION,
        distance_cm=distance_cm,
        heading_deg=0.0,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    g.add_edge(
        "B", "A",
        type=EDGE_PROJECTION,
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
    assert emitted_mode == ProjectionMode.NORMAL   # PIPELINE_CONFIG.projection default

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
        (ACT_TURN_REL, 180.0)                   -- turn to face 180deg (body heading for REAR)
        (ACT_LINE_REVERSE, 'B', LineMode.NORMAL) -- REAR traverse
        (ACT_LINE_OVERSHOOT, 5.0)                -- sim-only overshoot step

    Verifies the TC18 contract for the REAR case.
    """
    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0)}
    fd = SimpleNamespace(
        NODES=nodes,
        START_NODE="A",
        START_HEADING_DEG=0.0,
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
# Fixtures: synthetic single-edge graphs marked primitive_mode=
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

    The forward (A, B) edge carries ``primitive_mode=BlindMode.STOP_AT_LINE``
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
        primitive_mode=BlindMode.STOP_AT_LINE,
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
        primitive_mode=BlindMode.STOP_AT_LINE,
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
