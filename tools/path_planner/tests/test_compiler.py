"""Tests for tools.path_planner.compiler (PR3 schema).

Covers ``normalize_turn``, ``final_heading_after``, and ``compile_path``
on the LINE-only / BLIND-only edge types. The PR2 goal-approach blind
sequence has been removed; tests for it live in the git history.
"""

from __future__ import annotations

import math
import random
from types import SimpleNamespace
from typing import Dict, List, Tuple

import networkx as nx
import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.compiler import (
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
    Primitive,
    compile_path,
    final_heading_after,
    normalize_turn,
)
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, build_graph


@pytest.fixture(scope="session")
def toy_graph() -> nx.DiGraph:
    return build_graph(real_field_data)


# -- normalize_turn ----------------------------------------------------------


def test_normalize_zero() -> None:
    assert normalize_turn(0.0) == pytest.approx(0.0, abs=1e-12)


def test_normalize_positive_half_rotation() -> None:
    assert normalize_turn(180.0) == pytest.approx(180.0, abs=1e-12)


def test_normalize_negative_half_rotation() -> None:
    assert normalize_turn(-180.0) == pytest.approx(180.0, abs=1e-12)


def test_normalize_overshoot_positive() -> None:
    assert normalize_turn(190.0) == pytest.approx(-170.0, abs=1e-12)


def test_normalize_overshoot_negative() -> None:
    assert normalize_turn(-190.0) == pytest.approx(170.0, abs=1e-12)


def test_normalize_full_rotation() -> None:
    assert normalize_turn(360.0) == pytest.approx(0.0, abs=1e-12)


def test_normalize_multi_rotation() -> None:
    assert normalize_turn(720.0 + 45.0) == pytest.approx(45.0, abs=1e-12)
    assert normalize_turn(-720.0 - 45.0) == pytest.approx(-45.0, abs=1e-12)


# -- compile_path: trivial -----------------------------------------------------


def test_empty_path_returns_empty(toy_graph: nx.DiGraph) -> None:
    assert compile_path([], 0.0, toy_graph, real_field_data) == []


def test_single_node_returns_empty(toy_graph: nx.DiGraph) -> None:
    assert compile_path(["A"], 0.0, toy_graph, real_field_data) == []


# -- compile_path: traversal ---------------------------------------------------


def test_simple_line_traversal(toy_graph: nx.DiGraph) -> None:
    """A=(10,10) -> B=(200,10) is a LINE edge heading 0 deg (East)."""
    out = compile_path(["A", "B"], 0.0, toy_graph, real_field_data)
    assert out == [(ACT_FOLLOW_INT, None)]


def test_traversal_with_turn(toy_graph: nx.DiGraph) -> None:
    """A -> D is North; from 0 deg requires a +90 turn."""
    out = compile_path(["A", "D"], 0.0, toy_graph, real_field_data)
    assert out == [(ACT_TURN_REL, 90.0), (ACT_FOLLOW_INT, None)]


def test_zero_turn_skipped(toy_graph: nx.DiGraph) -> None:
    out = compile_path(["A", "B"], 0.0, toy_graph, real_field_data)
    assert all(p[0] != ACT_TURN_REL for p in out)


def test_blind_edge_emits_forward(toy_graph: nx.DiGraph) -> None:
    """B -> G is the toy field's only blind edge."""
    out = compile_path(["B", "G"], 0.0, toy_graph, real_field_data)
    # B=(200,10), G=(250,80). Heading 90 - atan2(70, 50) ~= 54.46 deg.
    assert toy_graph.edges["B", "G"]["type"] == EDGE_BLIND
    expected_dist = math.hypot(250.0 - 200.0, 80.0 - 10.0)
    assert out[-1] == (ACT_FORWARD, pytest.approx(expected_dist, abs=1e-9))
    # Initial turn was emitted since start heading 0 != edge heading.
    assert out[0][0] == ACT_TURN_REL


def test_unknown_edge_raises_key_error(toy_graph: nx.DiGraph) -> None:
    # A -> G has no direct edge (B is the only access to G).
    assert not toy_graph.has_edge("A", "G")
    with pytest.raises(KeyError):
        compile_path(["A", "G"], 0.0, toy_graph, real_field_data)


def test_invalid_start_heading_raises(toy_graph: nx.DiGraph) -> None:
    with pytest.raises(ValueError):
        compile_path(["A", "B"], -1.0, toy_graph, real_field_data)
    with pytest.raises(ValueError):
        compile_path(["A", "B"], 360.0, toy_graph, real_field_data)


def test_final_heading_after_simple(toy_graph: nx.DiGraph) -> None:
    assert final_heading_after(["A", "D"], 0.0, toy_graph) == pytest.approx(
        90.0, abs=1e-12
    )


def test_final_heading_after_short_paths(toy_graph: nx.DiGraph) -> None:
    assert final_heading_after([], 47.0, toy_graph) == pytest.approx(47.0, abs=1e-12)
    assert final_heading_after(["A"], 47.0, toy_graph) == pytest.approx(47.0, abs=1e-12)


# -- Property: random graphs ---------------------------------------------------


def _heading_deg(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx)) % 360.0


def _build_random_graph(rng: random.Random) -> Tuple[nx.DiGraph, Dict[str, Tuple[float, float]]]:
    n = rng.randint(3, 8)
    positions: Dict[str, Tuple[float, float]] = {}
    used: set = set()
    for i in range(n):
        while True:
            x = round(rng.uniform(0.0, 100.0), 4)
            y = round(rng.uniform(0.0, 100.0), 4)
            if (x, y) not in used:
                used.add((x, y))
                positions[f"N{i}"] = (x, y)
                break

    g = nx.DiGraph()
    for name, (x, y) in positions.items():
        g.add_node(name, x_cm=x, y_cm=y, type="line")
    names = list(positions.keys())

    def _add_directed_pair(u: str, v: str) -> None:
        if u == v or g.has_edge(u, v):
            return
        pu, pv = positions[u], positions[v]
        h = _heading_deg(pu, pv)
        d = math.hypot(pv[0] - pu[0], pv[1] - pu[1])
        g.add_edge(u, v, type=EDGE_LINE, distance_cm=d, heading_deg=h)
        if not g.has_edge(v, u):
            g.add_edge(v, u, type=EDGE_LINE, distance_cm=d, heading_deg=(h + 180.0) % 360.0)

    rng.shuffle(names)
    for u, v in zip(names[:-1], names[1:]):
        _add_directed_pair(u, v)
    extra = rng.randint(0, n)
    for _ in range(extra):
        u = rng.choice(names)
        v = rng.choice(names)
        _add_directed_pair(u, v)
    return g, positions


def _walk_expected_headings(
    path: List[str],
    start_heading: float,
    graph: nx.DiGraph,
) -> List[float]:
    headings: List[float] = []
    current = start_heading
    for u, v in zip(path[:-1], path[1:]):
        current = graph.edges[u, v]["heading_deg"]
        headings.append(current)
    return headings


def _replay_primitives(primitives: List[Primitive], start_heading: float) -> List[float]:
    motion_actions = {ACT_FOLLOW_INT, ACT_FORWARD}
    headings: List[float] = []
    current = start_heading
    for action, value in primitives:
        if action == ACT_TURN_REL:
            assert value is not None
            current = (current + value) % 360.0
        elif action in motion_actions:
            headings.append(current)
        else:
            raise AssertionError(f"unexpected primitive {action!r}")
    return headings


def test_property_random_paths() -> None:
    rng = random.Random(42)
    fake_field = SimpleNamespace()

    runs = 0
    attempts = 0
    while runs < 100 and attempts < 1000:
        attempts += 1
        graph, _ = _build_random_graph(rng)
        nodes = list(graph.nodes())
        if len(nodes) < 2:
            continue
        source = rng.choice(nodes)
        try:
            lengths = nx.single_source_dijkstra_path_length(graph, source, weight="distance_cm")
        except nx.NetworkXNoPath:
            continue
        candidates = [t for t in lengths if t != source]
        if not candidates:
            continue
        target = rng.choice(candidates)
        path = nx.shortest_path(graph, source, target, weight="distance_cm")
        if len(path) < 2:
            continue

        start_heading = round(rng.uniform(0.0, 359.999), 4)
        primitives = compile_path(path, start_heading, graph, fake_field)
        expected = _walk_expected_headings(path, start_heading, graph)
        replayed = _replay_primitives(primitives, start_heading)

        assert len(replayed) == len(expected) == len(path) - 1
        for got, want in zip(replayed, expected):
            delta = normalize_turn(got - want)
            assert abs(delta) == pytest.approx(0.0, abs=1e-6)
        runs += 1

    assert runs == 100
