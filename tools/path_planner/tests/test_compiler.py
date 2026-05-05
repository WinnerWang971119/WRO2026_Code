"""Tests for tools.path_planner.compiler.

Covers TC9 (property-based heading propagation: 200 random small graphs)
and TC10 (goal-arrival blind sequence with auto-corrected heading) from
the PR 1 plan, plus a battery of unit checks on ``normalize_turn`` and
structural guardrails on ``compile_path``.

The toy-graph fixture is session-scoped; mutation tests deep-clone the
real ``field_data`` via the same ``_clone_field_data()`` helper used in
sibling tests so the live module never sees a side-effect.
"""

from __future__ import annotations

import copy
import math
import random
from types import SimpleNamespace
from typing import Dict, List, Tuple

import networkx as nx
import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.compiler import (
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_TURN_REL,
    ACT_TURN_TO,
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

# Mirrors the list used in test_graph.py / test_planner.py so cloning
# stays in lock-step if the field-data surface ever grows.
_FIELD_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "PROJECTIONS",
    "GOAL_APPROACH",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
)


def _clone_field_data() -> SimpleNamespace:
    """Return a deep-copied SimpleNamespace mirror of the real field_data."""
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


@pytest.fixture(scope="session")
def toy_graph() -> nx.DiGraph:
    """The validated toy graph used across the compile_path tests."""
    return build_graph(real_field_data)


@pytest.fixture(scope="session")
def toy_field() -> SimpleNamespace:
    """A read-only clone of the real field_data for use in compile tests."""
    return _clone_field_data()


# -- normalize_turn unit tests -----------------------------------------------


def test_normalize_zero() -> None:
    assert normalize_turn(0.0) == pytest.approx(0.0, abs=1e-12)


def test_normalize_positive_half_rotation() -> None:
    """+180 is the canonical representative of a half-rotation."""
    assert normalize_turn(180.0) == pytest.approx(180.0, abs=1e-12)


def test_normalize_negative_half_rotation() -> None:
    """-180 folds onto +180 because ``a % 360`` puts it in [0, 360)."""
    assert normalize_turn(-180.0) == pytest.approx(180.0, abs=1e-12)


def test_normalize_overshoot_positive() -> None:
    assert normalize_turn(190.0) == pytest.approx(-170.0, abs=1e-12)


def test_normalize_overshoot_negative() -> None:
    assert normalize_turn(-190.0) == pytest.approx(170.0, abs=1e-12)


def test_normalize_full_rotation() -> None:
    assert normalize_turn(360.0) == pytest.approx(0.0, abs=1e-12)


def test_normalize_multi_rotation_positive() -> None:
    """Multiple full turns reduce identically to a single delta."""
    assert normalize_turn(720.0 + 45.0) == pytest.approx(45.0, abs=1e-12)


def test_normalize_multi_rotation_negative() -> None:
    assert normalize_turn(-720.0 - 45.0) == pytest.approx(-45.0, abs=1e-12)


# -- compile_path: trivial paths ---------------------------------------------


def test_empty_path_returns_empty(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    assert compile_path([], 0.0, toy_graph, toy_field) == []


def test_single_node_no_goal_returns_empty(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """``A`` is a line node with no GOAL_APPROACH entry, so nothing emits."""
    assert compile_path(["A"], 0.0, toy_graph, toy_field) == []


def test_single_node_goal_emits_blind_only(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """A path of just ``[goal]`` emits only the blind sequence (with optional
    turn_to_heading) -- no path-walking primitives."""
    # start_heading matches approach_heading_deg, so no turn_to_heading.
    # The blind sequence is [(turn_relative, 0), (forward, 30)]; the zero
    # turn is skipped, leaving exactly one forward primitive.
    out = compile_path(["G"], 0.0, toy_graph, toy_field)
    assert out == [(ACT_FORWARD, 30.0)]


def test_single_node_goal_inserts_turn_to_heading(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """If the start heading does not match the approach heading, a single
    ``turn_to_heading`` precedes the blind sequence."""
    # approach_heading_deg for G is 0.0; start at 90 -> emit turn_to(0.0).
    out = compile_path(["G"], 90.0, toy_graph, toy_field)
    assert out == [(ACT_TURN_TO, 0.0), (ACT_FORWARD, 30.0)]


# -- compile_path: traversal --------------------------------------------------


def test_simple_line_traversal(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """A->B is a line edge heading 0 deg (East). Starting at 0 deg means no
    turn -- only the line-follow primitive should appear."""
    out = compile_path(["A", "B"], 0.0, toy_graph, toy_field)
    assert out == [(ACT_FOLLOW_INT, None)]


def test_traversal_with_turn(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """A->D heads 90 deg (North). Starting at 0 deg requires a +90 turn."""
    out = compile_path(["A", "D"], 0.0, toy_graph, toy_field)
    assert out == [(ACT_TURN_REL, 90.0), (ACT_FOLLOW_INT, None)]


def test_zero_turn_skipped(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """A zero-turn between path edges must NOT produce a no-op primitive."""
    out = compile_path(["A", "B"], 0.0, toy_graph, toy_field)
    # Specifically: no ``(ACT_TURN_REL, 0.0)`` anywhere in the output.
    assert all(p[0] != ACT_TURN_REL or abs(p[1] or 0.0) > 1e-12 for p in out)


def test_unknown_edge_raises_key_error(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """A and C have no direct edge in the toy graph; KeyError must surface."""
    assert not toy_graph.has_edge("A", "C")
    with pytest.raises(KeyError):
        compile_path(["A", "C"], 0.0, toy_graph, toy_field)


def test_invalid_start_heading_raises(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """``start_heading_deg`` outside [0, 360) raises ``ValueError``."""
    with pytest.raises(ValueError):
        compile_path(["A", "B"], -1.0, toy_graph, toy_field)
    with pytest.raises(ValueError):
        compile_path(["A", "B"], 360.0, toy_graph, toy_field)


def test_final_heading_after_simple(toy_graph: nx.DiGraph) -> None:
    """A->D is a single edge heading 90 deg; final heading is therefore 90."""
    assert final_heading_after(["A", "D"], 0.0, toy_graph) == pytest.approx(
        90.0, abs=1e-12
    )


def test_final_heading_after_short_paths(toy_graph: nx.DiGraph) -> None:
    """Paths of length <= 1 leave the start heading untouched."""
    assert final_heading_after([], 47.0, toy_graph) == pytest.approx(47.0, abs=1e-12)
    assert final_heading_after(["A"], 47.0, toy_graph) == pytest.approx(
        47.0, abs=1e-12
    )


# -- TC10: full goal path with blind sequence --------------------------------


def test_goal_path_with_blind_moves(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """TC10: A -> B -> _proj_G -> G full path emits the expected primitive
    sequence including the blind goal arrival.

    Reasoning:
      A->B is a line edge East (0 deg) starting from 0 deg -- no turn,
      one ``follow_line_to_intersection``.
      B->_proj_G is a projection edge North (90 deg) -- turn +90,
      ``follow_line_distance`` 30 cm.
      _proj_G->G is a blind edge East (0 deg) -- turn -90 (delta from
      90 deg back to 0 deg), ``forward`` 30 cm.
      Arrival at G with heading 0 deg matches approach_heading_deg=0 deg,
      so no ``turn_to_heading`` is inserted. Goal blind moves are
      ``[(turn_relative, 0), (forward, 30)]``: the zero turn is skipped,
      leaving one ``forward`` 30 cm.
    """
    expected: List[Primitive] = [
        (ACT_FOLLOW_INT, None),         # A -> B
        (ACT_TURN_REL, 90.0),            # face North for B -> _proj_G
        (ACT_FOLLOW_DIST, 30.0),         # B -> _proj_G
        (ACT_TURN_REL, -90.0),           # face East for _proj_G -> G
        (ACT_FORWARD, 30.0),             # _proj_G -> G blind
        (ACT_FORWARD, 30.0),             # GOAL_APPROACH blind move
    ]
    out = compile_path(
        ["A", "B", "_proj_G", "G"], 0.0, toy_graph, toy_field
    )
    assert len(out) == len(expected)
    for actual, want in zip(out, expected):
        assert actual[0] == want[0]
        if want[1] is None:
            assert actual[1] is None
        else:
            assert actual[1] == pytest.approx(want[1], abs=1e-9)


def test_goal_arrival_inserts_turn_to_heading_when_drifted(
    toy_graph: nx.DiGraph,
) -> None:
    """If the path leaves the chassis facing a heading other than
    ``approach_heading_deg``, a single ``turn_to_heading`` is inserted
    before the blind sequence runs.

    We mutate a clone so the live field_data stays untouched.
    """
    fd = _clone_field_data()
    # Force a 45-degree mismatch by setting the goal's required approach
    # heading to a value the path's terminal edge will not produce.
    fd.GOAL_APPROACH["G"] = copy.deepcopy(fd.GOAL_APPROACH["G"])
    fd.GOAL_APPROACH["G"]["approach_heading_deg"] = 45.0

    out = compile_path(
        ["A", "B", "_proj_G", "G"], 0.0, toy_graph, fd
    )

    # The terminal _proj_G -> G blind edge has heading 0 deg, so the
    # chassis arrives at G facing 0 deg -- a 45-degree mismatch.
    # Find the absolute correction; it should be the one and only
    # ACT_TURN_TO in the output and target 45.0.
    turn_tos = [p for p in out if p[0] == ACT_TURN_TO]
    assert len(turn_tos) == 1
    assert turn_tos[0][1] == pytest.approx(45.0, abs=1e-12)


# -- TC9: property-based heading propagation ---------------------------------


def _heading_deg(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Same atan2 math the graph builder uses; duplicated here so the
    property test is independent of any internal helper rename."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx)) % 360.0


def _build_random_graph(rng: random.Random) -> Tuple[nx.DiGraph, Dict[str, Tuple[float, float]]]:
    """Build a small random connected DiGraph with non-coincident nodes.

    * ``n`` nodes in [3, 8] at random positions in [0, 100]^2.
    * Names ``N0..N{n-1}``.
    * Connectivity is generated as a random spanning chain plus a few
      extra random edges so most pairs are reachable. Every edge carries
      ``type=EDGE_LINE`` plus the same ``distance_cm`` / ``heading_deg``
      attribute names the real graph_builder uses.
    * Node positions are coalesced so no two nodes share a coordinate
      (atan2(0, 0) is 0 but produces a degenerate edge - we steer clear).
    """
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
        # Reverse edge with the +180 mod 360 heading so paths are
        # bidirectional, matching graph_builder's convention.
        if not g.has_edge(v, u):
            g.add_edge(
                v,
                u,
                type=EDGE_LINE,
                distance_cm=d,
                heading_deg=(h + 180.0) % 360.0,
            )

    # Spanning chain so every node is connected to its neighbour by index.
    rng.shuffle(names)
    for u, v in zip(names[:-1], names[1:]):
        _add_directed_pair(u, v)

    # A handful of random extra edges to widen the path search space.
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
    """Independently compute the per-edge expected ``current_heading`` after
    each edge using the same ``edge['heading_deg']`` lookup, so we have a
    reference series to compare ``compile_path`` against."""
    headings: List[float] = []
    current = start_heading
    for u, v in zip(path[:-1], path[1:]):
        edge_h = graph.edges[u, v]["heading_deg"]
        current = edge_h
        headings.append(current)
    return headings


def _replay_primitives(
    primitives: List[Primitive], start_heading: float
) -> List[float]:
    """Walk the emitted primitives, applying every relative turn, and
    return the heading after each motion primitive (one entry per edge
    of the source path). Motion primitives keep the heading unchanged;
    relative turns rotate it modulo 360. ``ACT_TURN_TO`` is not expected
    in the property test (no goals)."""
    motion_actions = {ACT_FOLLOW_INT, ACT_FOLLOW_DIST, ACT_FORWARD}
    headings: List[float] = []
    current = start_heading
    for action, value in primitives:
        if action == ACT_TURN_REL:
            assert value is not None
            current = (current + value) % 360.0
        elif action in motion_actions:
            headings.append(current)
        else:
            raise AssertionError(
                f"unexpected primitive {action!r} in non-goal property test"
            )
    return headings


def test_property_random_paths() -> None:
    """TC9: 200 random small graphs, random reachable subpaths, replay the
    emitted primitives and confirm the heading at every edge boundary
    matches the corresponding ``edge['heading_deg']``."""
    rng = random.Random(42)
    fake_field = SimpleNamespace(
        NODE_TYPES={},  # no goals -> no blind sequence
        GOAL_APPROACH={},
    )

    runs = 0
    attempts = 0
    # Cap attempts so a string of bad coin-flips can't loop forever; the
    # plan asks for 200 successful runs.
    while runs < 200 and attempts < 2000:
        attempts += 1
        graph, _positions = _build_random_graph(rng)
        nodes = list(graph.nodes())
        if len(nodes) < 2:
            continue

        source = rng.choice(nodes)
        # Find some node that is actually reachable from ``source`` so we
        # always have a non-trivial path to compile.
        try:
            lengths = nx.single_source_dijkstra_path_length(
                graph, source, weight="distance_cm"
            )
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

        assert len(replayed) == len(expected) == len(path) - 1, (
            f"length mismatch on run {runs}: replayed={len(replayed)} "
            f"expected={len(expected)} path_len={len(path)}"
        )
        for i, (got, want) in enumerate(zip(replayed, expected)):
            # Compare on the unit circle to ignore the 0/360 wraparound.
            delta = normalize_turn(got - want)
            assert abs(delta) == pytest.approx(0.0, abs=1e-6), (
                f"run {runs} edge {i}: replayed heading {got} != expected "
                f"{want} (delta={delta})"
            )

        # Sanity: final_heading_after must agree with the walk.
        assert final_heading_after(path, start_heading, graph) == pytest.approx(
            expected[-1], abs=1e-9
        )

        runs += 1

    assert runs == 200, f"only achieved {runs} property runs in {attempts} attempts"


# -- structural guardrails ---------------------------------------------------


def test_projection_edge_emits_follow_distance(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """A path crossing only a projection edge emits ``follow_line_distance``
    with the edge's ``distance_cm``, not ``follow_line_to_intersection``."""
    # B -> _proj_G is a projection edge of length 30 cm (B at (60, 0),
    # _proj_G at (60, 30)). Start heading 90 deg matches the edge so no
    # turn is emitted.
    out = compile_path(["B", "_proj_G"], 90.0, toy_graph, toy_field)
    assert out == [(ACT_FOLLOW_DIST, 30.0)]
    # Independently confirm the edge type so a future graph_builder
    # change that re-tags this edge fails this test loudly.
    assert toy_graph.edges["B", "_proj_G"]["type"] == EDGE_PROJECTION


def test_blind_edge_emits_forward(
    toy_graph: nx.DiGraph,
    toy_field: SimpleNamespace,
) -> None:
    """The toy field's only blind edge is _proj_G -> G; the path
    [_proj_G, G] (no goal blind appendix avoidance) must emit a
    ``forward`` primitive of 30 cm before the GOAL_APPROACH sequence."""
    out = compile_path(["_proj_G", "G"], 0.0, toy_graph, toy_field)
    # Edge: forward 30 cm. Goal blind: turn 0 (skipped) + forward 30 cm.
    assert out == [(ACT_FORWARD, 30.0), (ACT_FORWARD, 30.0)]
    assert toy_graph.edges["_proj_G", "G"]["type"] == EDGE_BLIND
