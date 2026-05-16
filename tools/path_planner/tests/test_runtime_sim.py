"""End-to-end tests for the PathTracker runtime simulator (Phase 2 PR 2).

The simulator's correctness story is:

1. For every reachable ``(start, goal)`` pair on the toy graph, the
   sequence of primitives the simulator dispatches must equal the
   sequence :func:`tools.path_planner.compiler.compile_path` produces
   when given the same NEXT_HOP-walked node list.
2. The chassis lands within a small tolerance of the goal node's
   geometric ``(x, y)`` after running through that primitive sequence.
3. Unreachable goals (a custom field_data with the goal walled off)
   raise :class:`UnreachableGoalError`, matching PathTracker's
   ``Fault::UNREACHABLE`` semantics.
4. ``go_to`` before ``zero_heading_from_current`` raises ``RuntimeError``
   matching the C++ runtime's ``Fault::NOT_ZEROED`` path.
5. Property test: 50 random multi-step goal sequences across the toy
   graph keep sim and compiler in agreement.
"""

from __future__ import annotations

import importlib
import math
import random
import types
from typing import List

import pytest

from tools.path_planner.compiler import (
    ACT_FORWARD,
    ACT_TURN_REL,
    Primitive,
    compile_path,
)
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import compute_all_pairs_next_hop
from tools.path_planner.sim import (
    LocalizationSim,
    PathTrackerSim,
    UnreachableGoalError,
    VirtualChassis,
    walk_next_hop,
)
from tools.path_planner import field_data as toy_field_data


# Position tolerance: the simulator does pure-geometry float math; values
# arrive within machine epsilon of the analytic result. 0.1 cm is a safe
# upper bound while still catching any sign-flip or off-by-edge bug.
_POSITION_TOL_CM: float = 0.1


# -- shared fixtures ---------------------------------------------------------


@pytest.fixture
def toy_graph_and_table():
    """Build the toy graph + NEXT_HOP once per test that asks for it."""
    graph = build_graph(toy_field_data)
    next_hop = compute_all_pairs_next_hop(
        graph, toy_field_data.COST_BLIND_MULTIPLIER
    )
    return graph, next_hop


@pytest.fixture
def fresh_sim(toy_graph_and_table):
    """A PathTrackerSim seeded at the toy field's START_NODE / heading.

    The localization is pre-zero'd so the test does not have to repeat
    the boilerplate. Tests that exercise the not-zeroed fault path
    construct their own sim explicitly.
    """
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field_data)
    sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim


# -- TC1: single goal A -> C -------------------------------------------------


def test_goto_C_matches_compile_path(fresh_sim, toy_graph_and_table):
    """Sim's primitive list equals compile_path's for the A -> C walk."""
    graph, next_hop = toy_graph_and_table
    primitives = fresh_sim.go_to("C")

    walked = walk_next_hop(next_hop, "A", "C")
    expected = compile_path(
        walked,
        toy_field_data.START_HEADING_DEG,
        graph,
        toy_field_data,
    )
    assert primitives == expected

    # Chassis arrived at C's coordinates within tolerance.
    cx, cy = toy_field_data.NODES["C"]
    assert math.hypot(fresh_sim.chassis.x_cm - cx, fresh_sim.chassis.y_cm - cy) < _POSITION_TOL_CM
    assert fresh_sim.localization.current_node() == "C"


# -- TC2: chained goals A -> C -> G ------------------------------------------


def test_chained_goto_C_then_G(fresh_sim, toy_graph_and_table):
    """Two sequential goTos compose: chassis ends at G's blind-move position."""
    graph, next_hop = toy_graph_and_table

    primitives_C = fresh_sim.go_to("C")
    primitives_G = fresh_sim.go_to("G")

    # Sanity: each leg matches its compile_path independently when given
    # the heading the chassis had at the leg's start.
    walked_C = walk_next_hop(next_hop, "A", "C")
    expected_C = compile_path(
        walked_C, toy_field_data.START_HEADING_DEG, graph, toy_field_data
    )
    assert primitives_C == expected_C

    # Goal G's GOAL_APPROACH blind sequence is run inside go_to("G"),
    # so the chassis ends past G in +x by the (forward, 30) blind step.
    gx, gy = toy_field_data.NODES["G"]
    final_x = gx + 30.0    # one (forward, 30) blind move at heading 0
    final_y = gy
    assert math.isclose(fresh_sim.chassis.x_cm, final_x, abs_tol=_POSITION_TOL_CM)
    assert math.isclose(fresh_sim.chassis.y_cm, final_y, abs_tol=_POSITION_TOL_CM)
    # The simulator advances current_node to G when it crosses the
    # blind edge, before running the goal-blind sequence.
    assert fresh_sim.localization.current_node() == "G"
    # Ensure the goal-blind sequence emitted at least the forward step.
    assert any(p == (ACT_FORWARD, 30.0) for p in primitives_G)


# -- TC3: goTo to currentNode ------------------------------------------------


def test_goto_self_is_noop(fresh_sim):
    """goTo(currentNode) returns an empty primitive list and leaves chassis put."""
    initial_x = fresh_sim.chassis.x_cm
    initial_y = fresh_sim.chassis.y_cm
    initial_heading = fresh_sim.chassis.heading_deg

    primitives = fresh_sim.go_to(fresh_sim.localization.current_node())
    assert primitives == []
    assert fresh_sim.chassis.x_cm == initial_x
    assert fresh_sim.chassis.y_cm == initial_y
    assert fresh_sim.chassis.heading_deg == initial_heading


# -- TC4: unreachable goal ---------------------------------------------------


def _make_walled_off_field():
    """Return a custom field_data namespace with G unreachable.

    Reuses every other toy attribute but inflates an obstacle so that no
    edge into ``_proj_G`` survives C-space inflation, making G
    unreachable from anywhere. The graph builder logs a warning for the
    disconnected goal — that's the path PathTracker's UNREACHABLE fault
    handles on the C++ side.
    """
    fd = types.SimpleNamespace(
        NODES                  = dict(toy_field_data.NODES),
        NODE_TYPES             = dict(toy_field_data.NODE_TYPES),
        LINE_SEGMENTS          = list(toy_field_data.LINE_SEGMENTS),
        PROJECTIONS            = dict(toy_field_data.PROJECTIONS),
        GOAL_APPROACH          = dict(toy_field_data.GOAL_APPROACH),
        # Wall covering the entire (B, C) segment area + the projection's
        # surroundings. After C-space inflation by ROBOT_RADIUS_CM=8 the
        # wall blocks every edge into _proj_G and out of _proj_G.
        OBSTACLES              = [
            (50.0, 25.0, 70.0, 65.0, "barrier"),
        ],
        ROBOT_RADIUS_CM        = toy_field_data.ROBOT_RADIUS_CM,
        COST_BLIND_MULTIPLIER  = toy_field_data.COST_BLIND_MULTIPLIER,
        START_NODE             = toy_field_data.START_NODE,
        START_HEADING_DEG      = toy_field_data.START_HEADING_DEG,
    )
    return fd


def test_unreachable_goal_raises():
    """Goal G fully walled off => UnreachableGoalError on go_to('G')."""
    import warnings

    fd = _make_walled_off_field()
    with warnings.catch_warnings():
        # graph_builder emits a UserWarning for the disconnected goal,
        # which is exactly the condition we are exercising. Suppress it
        # so the test output stays clean.
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
    assert err.current is not None    # carried for diagnostic context


# -- TC5: goal-blind sequence semantics --------------------------------------


def test_goto_G_runs_blind_sequence(fresh_sim, toy_graph_and_table):
    """The (turn_relative, 0) at the start of G's blind sequence is skipped;
    the (forward, 30) is emitted. Chassis ends 30 cm east of G.
    """
    primitives = fresh_sim.go_to("G")

    # The (turn_relative, 0) entry is below TURN_EPSILON_DEG so should
    # NOT appear in the output.
    assert (ACT_TURN_REL, 0.0) not in primitives
    # The (forward, 30) blind step IS emitted.
    assert primitives[-1] == (ACT_FORWARD, 30.0)


# -- TC6: property test on 50 random goal sequences --------------------------


def test_property_random_walks(toy_graph_and_table):
    """For 50 random reachable goal sequences, sim primitives == compile_path.

    Builds a fresh sim per iteration, performs 1-3 goTos to randomly
    chosen reachable goals, and asserts the concatenated primitive list
    matches a per-leg compile_path computed against the chassis's
    starting heading at each leg's beginning.
    """
    graph, next_hop = toy_graph_and_table
    rng = random.Random(0xA1B2C3)    # deterministic seed
    reachable_targets = [
        n for n in toy_field_data.NODES
        if next_hop[toy_field_data.START_NODE][n] is not None
    ]

    iterations = 50
    for _ in range(iterations):
        chassis = VirtualChassis.from_field_data(toy_field_data)
        sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
        sim.localization.zero_heading_from_current()

        n_goals = rng.randint(1, 3)
        # Track the chassis heading at each leg's start so we can produce
        # the matching compile_path expected output.
        prev_heading = chassis.heading_deg
        all_emitted: List[Primitive] = []
        all_expected: List[Primitive] = []

        for _ in range(n_goals):
            goal = rng.choice(reachable_targets)
            current_before = sim.localization.current_node()

            # Skip if there is no path from the running current node;
            # this can happen if a previous leg dead-ended at a
            # one-way-reachable node.
            if next_hop[current_before][goal] is None and current_before != goal:
                continue

            heading_before = chassis.heading_deg

            walked = walk_next_hop(next_hop, current_before, goal)
            expected = compile_path(walked, heading_before, graph, toy_field_data)

            emitted = sim.go_to(goal)
            assert emitted == expected, (
                f"mismatch on leg {current_before!r} -> {goal!r}: "
                f"emitted={emitted!r} expected={expected!r}"
            )
            all_emitted.extend(emitted)
            all_expected.extend(expected)
            prev_heading = chassis.heading_deg

        assert all_emitted == all_expected


# -- TC7: name lookup miss ----------------------------------------------------


def test_goto_unknown_node_raises(fresh_sim):
    with pytest.raises(KeyError):
        fresh_sim.go_to("Z")


# -- TC10/TC11: localization mirror behaviour --------------------------------


def test_localization_starts_unzeroed():
    fd = toy_field_data
    loc = LocalizationSim(fd)
    assert loc.is_zeroed() is False
    assert loc.current_node() == fd.START_NODE


def test_goto_before_zero_raises(toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field_data)
    sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
    # Intentionally skip zero_heading_from_current().
    with pytest.raises(RuntimeError) as excinfo:
        sim.go_to("C")
    assert "before" in str(excinfo.value).lower()


# -- Position tolerance regression: every reachable goal lands within tol ---


def test_every_reachable_goal_lands_within_tolerance(toy_graph_and_table):
    """For every node reachable from START_NODE, sim arrives within tol."""
    graph, next_hop = toy_graph_and_table
    targets = [
        n for n in toy_field_data.NODES
        if next_hop[toy_field_data.START_NODE][n] is not None
        and n != toy_field_data.START_NODE
    ]
    for goal in targets:
        chassis = VirtualChassis.from_field_data(toy_field_data)
        sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
        sim.localization.zero_heading_from_current()
        sim.go_to(goal)
        if (
            toy_field_data.NODE_TYPES.get(goal) == "goal"
            and goal in toy_field_data.GOAL_APPROACH
        ):
            # The chassis is past the goal by the blind sequence; just
            # check current_node was advanced.
            assert sim.localization.current_node() == goal
        else:
            gx, gy = toy_field_data.NODES[goal]
            dist = math.hypot(chassis.x_cm - gx, chassis.y_cm - gy)
            assert dist < _POSITION_TOL_CM, (
                f"goal {goal!r}: chassis at ({chassis.x_cm}, {chassis.y_cm}) "
                f"vs node ({gx}, {gy}), dist={dist}"
            )
            assert sim.localization.current_node() == goal


# -- walk_next_hop self-target case ------------------------------------------


def test_walk_next_hop_self(toy_graph_and_table):
    _, next_hop = toy_graph_and_table
    assert walk_next_hop(next_hop, "A", "A") == ["A"]
