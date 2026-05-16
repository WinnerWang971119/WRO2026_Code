"""End-to-end tests for the PathTracker runtime simulator (PR3).

PR3 simplification: only LINE and BLIND edges; no projection / goal-approach
blind sequences.
"""

from __future__ import annotations

import copy
import math
import random
import warnings
from types import SimpleNamespace
from typing import List

import pytest

from tools.path_planner import field_data as toy_field_data
from tools.path_planner.compiler import Primitive, compile_path
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.planner import compute_all_pairs_next_hop
from tools.path_planner.sim import (
    LocalizationSim,
    PathTrackerSim,
    UnreachableGoalError,
    VirtualChassis,
    walk_next_hop,
)

_POSITION_TOL_CM: float = 0.1

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
    snapshot = {attr: copy.deepcopy(getattr(toy_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


@pytest.fixture
def toy_graph_and_table():
    graph = build_graph(toy_field_data)
    next_hop = compute_all_pairs_next_hop(graph, toy_field_data.COST_BLIND_MULTIPLIER)
    return graph, next_hop


@pytest.fixture
def fresh_sim(toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field_data)
    sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    return sim


def test_goto_B_matches_compile_path(fresh_sim, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    primitives = fresh_sim.go_to("B")
    walked = walk_next_hop(next_hop, "A", "B")
    expected = compile_path(walked, toy_field_data.START_HEADING_DEG, graph, toy_field_data)
    assert primitives == expected
    bx, by = toy_field_data.NODES["B"]
    assert math.hypot(fresh_sim.chassis.x_cm - bx, fresh_sim.chassis.y_cm - by) < _POSITION_TOL_CM
    assert fresh_sim.localization.current_node() == "B"


def test_goto_C_via_path(fresh_sim, toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    primitives = fresh_sim.go_to("C")
    walked = walk_next_hop(next_hop, "A", "C")
    expected = compile_path(walked, toy_field_data.START_HEADING_DEG, graph, toy_field_data)
    assert primitives == expected
    cx, cy = toy_field_data.NODES["C"]
    assert math.hypot(fresh_sim.chassis.x_cm - cx, fresh_sim.chassis.y_cm - cy) < _POSITION_TOL_CM
    assert fresh_sim.localization.current_node() == "C"


def test_chained_goto_lands_at_G(fresh_sim):
    fresh_sim.go_to("B")
    fresh_sim.go_to("G")
    gx, gy = toy_field_data.NODES["G"]
    assert math.hypot(fresh_sim.chassis.x_cm - gx, fresh_sim.chassis.y_cm - gy) < _POSITION_TOL_CM
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


def test_property_random_walks(toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    rng = random.Random(0xA1B2C3)
    reachable_targets = [
        n for n in toy_field_data.NODES
        if next_hop[toy_field_data.START_NODE][n] is not None
    ]

    for _ in range(30):
        chassis = VirtualChassis.from_field_data(toy_field_data)
        sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
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
            expected = compile_path(walked, heading_before, graph, toy_field_data)
            emitted = sim.go_to(goal)
            assert emitted == expected
            all_emitted.extend(emitted)
            all_expected.extend(expected)
        assert all_emitted == all_expected


def test_goto_unknown_node_raises(fresh_sim):
    with pytest.raises(KeyError):
        fresh_sim.go_to("ZZZ")


def test_localization_starts_unzeroed():
    loc = LocalizationSim(toy_field_data)
    assert loc.is_zeroed() is False
    assert loc.current_node() == toy_field_data.START_NODE


def test_goto_before_zero_raises(toy_graph_and_table):
    graph, next_hop = toy_graph_and_table
    chassis = VirtualChassis.from_field_data(toy_field_data)
    sim = PathTrackerSim(toy_field_data, graph, next_hop, chassis)
    with pytest.raises(RuntimeError) as excinfo:
        sim.go_to("C")
    assert "before" in str(excinfo.value).lower()


def test_every_reachable_goal_lands_within_tolerance(toy_graph_and_table):
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
        gx, gy = toy_field_data.NODES[goal]
        dist = math.hypot(chassis.x_cm - gx, chassis.y_cm - gy)
        assert dist < _POSITION_TOL_CM
        assert sim.localization.current_node() == goal


def test_walk_next_hop_self(toy_graph_and_table):
    _, next_hop = toy_graph_and_table
    assert walk_next_hop(next_hop, "A", "A") == ["A"]
