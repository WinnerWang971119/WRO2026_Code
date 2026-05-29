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

from tools.path_planner.tests.fixtures import toy_field_data as real_field_data
from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
    Primitive,
    compile_path,
    final_heading_after,
    normalize_turn,
)
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    EDGE_PROJECTION,
    build_graph,
)
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.runtime import (
    ACT_BLIND_REVERSE,
    ACT_LINE_REVERSE,
    ACT_PROJECTION_REVERSE,
    Direction,
    resolve_direction,
)


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


# -- compile_path: PROJECTION (TC4) -------------------------------------------


def test_compile_projection_edge_emits_follow_line_to_distance() -> None:
    """A PROJECTION edge in the graph emits ``(ACT_FOLLOW_DIST, distance_cm)``.

    Heading delta is zero so no ACT_TURN_REL is emitted; the compiler
    must produce exactly one primitive: the encoder-distance-gated
    line-follow that EDGE_PROJECTION encodes.
    """
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge("A", "B", type=EDGE_PROJECTION, distance_cm=12.0, heading_deg=0.0)

    prims = compile_path(["A", "B"], 0.0, g, None)

    # No turn (heading delta 0), one follow-distance primitive.
    assert prims == [(ACT_FOLLOW_DIST, 12.0)]


def test_compile_projection_edge_with_turn_emits_turn_then_follow_dist() -> None:
    """A PROJECTION edge with a non-zero heading delta emits TURN_REL then FOLLOW_DIST."""
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge("A", "B", type=EDGE_PROJECTION, distance_cm=20.0, heading_deg=90.0)

    prims = compile_path(["A", "B"], 0.0, g, None)

    assert prims == [(ACT_TURN_REL, 90.0), (ACT_FOLLOW_DIST, 20.0)]


def test_compile_mixed_path_interleaves_primitives_correctly() -> None:
    """A LINE -> turn -> PROJECTION -> BLIND path emits primitives in the right order.

    Builds a four-node path with one of each edge type and a heading
    change between every consecutive pair, then asserts the full
    primitive sequence interleaves TURN_REL + the per-type motion
    primitive in the order the compiler walks the edges.

    Graph:
        A --LINE 30 cm  heading 0    --> B
        B --PROJ 15 cm  heading 90   --> C
        C --BLIND 25 cm heading 180  --> D

    Starting heading 0 deg:
        edge A->B: delta 0  -> no turn, FOLLOW_INT (None)
        edge B->C: delta 90 -> TURN_REL 90,  FOLLOW_DIST 15.0
        edge C->D: delta 90 -> TURN_REL 90,  FORWARD 25.0
    """
    g = nx.DiGraph()
    for n in ("A", "B", "C", "D"):
        g.add_node(n)
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=30.0, heading_deg=0.0)
    g.add_edge("B", "C", type=EDGE_PROJECTION, distance_cm=15.0, heading_deg=90.0)
    g.add_edge("C", "D", type=EDGE_BLIND, distance_cm=25.0, heading_deg=180.0)

    prims = compile_path(["A", "B", "C", "D"], 0.0, g, None)

    assert prims == [
        (ACT_FOLLOW_INT, None),
        (ACT_TURN_REL, 90.0),
        (ACT_FOLLOW_DIST, 15.0),
        (ACT_TURN_REL, 90.0),
        (ACT_FORWARD, 25.0),
    ]


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
    """Return the expected chassis body heading at each motion primitive.

    Calls :func:`tools.path_planner.sim.runtime.resolve_direction` for every
    hop with the same inputs ``compile_path`` would use — so the helper
    honours the full 5-step precedence (per-edge override → pipeline default
    → builder arrival → builder route → auto), not just the auto branch.
    A previous version inlined only the auto branch; any random graph that
    later seeded a per-edge ``direction_override`` would cause non-deterministic
    parity failures because the mirror silently ignored Step 1.

    Module-level constants ``BIDIRECTIONAL_ENABLED`` / ``LINE_BOARD_COUNT``
    are read off the live ``_runtime`` module at call time so a
    ``monkeypatch.setattr(_runtime, ...)`` propagates to this helper the
    same way it propagates to ``compile_path``'s lazy reads.
    """
    headings: List[float] = []
    current = start_heading
    last_idx = len(path) - 1
    for i, (u, v) in enumerate(zip(path[:-1], path[1:])):
        edge = graph.edges[u, v]
        edge_heading = float(edge["heading_deg"])
        direction = resolve_direction(
            edge_dir_override=_runtime._read_edge_dir_override(edge),
            pipeline_default=_runtime._pipeline_config.PIPELINE_CONFIG.direction,
            builder_arrival=Direction.AUTO,
            builder_route=Direction.AUTO,
            current_heading=current,
            edge_heading=edge_heading,
            is_last_hop=((i + 1) == last_idx),
            bidirectional_enabled=_runtime.BIDIRECTIONAL_ENABLED,
            line_board_count=_runtime.LINE_BOARD_COUNT,
        )
        if direction == Direction.REAR:
            body_target = (edge_heading + 180.0) % 360.0
        else:
            body_target = edge_heading % 360.0
        # Only advance the running heading when the turn delta would
        # actually be emitted (mirror compile_path's epsilon guard).
        body_delta = normalize_turn(body_target - current)
        if abs(body_delta) > 1e-6:
            current = body_target
        headings.append(current)
    return headings


def _replay_primitives(primitives: List[Primitive], start_heading: float) -> List[float]:
    """Replay a primitive sequence and return the body heading at each motion step.

    Handles both forward primitives (``ACT_FOLLOW_INT`` / ``ACT_FORWARD``
    / ``ACT_FOLLOW_DIST``) and reverse primitives
    (``ACT_LINE_REVERSE`` / ``ACT_BLIND_REVERSE`` /
    ``ACT_PROJECTION_REVERSE``) introduced by T12.
    """
    motion_actions = {
        ACT_FOLLOW_INT,
        ACT_FORWARD,
        ACT_FOLLOW_DIST,
        ACT_LINE_REVERSE,
        ACT_BLIND_REVERSE,
        ACT_PROJECTION_REVERSE,
    }
    headings: List[float] = []
    current = start_heading
    for prim in primitives:
        action = prim[0]
        if action == ACT_TURN_REL:
            value = prim[1]
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
