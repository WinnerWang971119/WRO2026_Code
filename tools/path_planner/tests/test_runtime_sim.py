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

from tools.path_planner import field_data as toy_field_data
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
from tools.path_planner.sim.runtime import (
    ACT_LINE_OVERSHOOT,
    LINE_SENSOR_AXLE_OFFSET_CM,
    resolve_blind_mode,
    resolve_line_mode,
    resolve_pre_turn_mode,
    resolve_projection_mode,
)

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
    """Insert ``(ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM)`` after each FOLLOW_INT.

    The runtime sim (post-T9) emits a LINE_OVERSHOOT step after every
    EDGE_LINE traversal so the wheel axle (not the line sensor) lands on
    the node. ``compile_path`` deliberately does NOT emit this step (the
    C++ runtime owns it as an FSM state), so to compare the sim's output
    against ``compile_path``'s output we must re-add the LINE_OVERSHOOT
    steps in the same positions the sim emits them.
    """
    out: List[Primitive] = []
    for prim in primitives:
        out.append(prim)
        if prim[0] == ACT_FOLLOW_INT:
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
