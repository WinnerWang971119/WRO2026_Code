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
    ACT_STOP_AT_LINE,
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
from tools.path_planner.pipeline_config import BlindMode, DirectionMode
from tools.path_planner.sim import runtime as _runtime
from tools.path_planner.sim.runtime import (
    ACT_BLIND_REVERSE,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_REVERSE,
    ACT_PROJECTION_REVERSE,
    ACT_STOP_AT_LINE_REVERSE,
    LINE_SENSOR_AXLE_OFFSET_CM,
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


def test_final_heading_after_rear_final_hop_matches_compile_path() -> None:
    """TC8 (#44): on a REAR final hop, ``final_heading_after`` returns the
    chassis body heading (edge_heading + 180) and AGREES with ``compile_path``.

    A single forced-REAR BLIND hop (``direction_override=FORCE_REVERSE``)
    resolves REAR regardless of geometry, so the chassis turns to face the
    edge with its rear sensor: the body heading the robot is left on is
    ``(edge_heading + 180) % 360``, not the raw forward edge heading. The
    fix makes ``final_heading_after`` resolve direction the same way
    ``compile_path`` does, so the returned pose must equal both the
    ``+180`` body heading and the heading ``compile_path`` actually drives
    (recovered by replaying its primitive stream).
    """
    edge_heading = 90.0
    # Plain (non-STOP_AT_LINE) BLIND edge forced REAR: the REAR branch emits
    # ACT_BLIND_REVERSE, which ``_replay_primitives`` understands, keeping the
    # test focused on the resolved heading rather than the seek-mode split.
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge(
        "A", "B",
        type=EDGE_BLIND,
        distance_cm=40.0,
        heading_deg=edge_heading,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    g.add_edge(
        "B", "A",
        type=EDGE_BLIND,
        distance_cm=40.0,
        heading_deg=(edge_heading + 180.0) % 360.0,
    )

    start_heading = 0.0
    end = final_heading_after(["A", "B"], start_heading, g)

    # The REAR body heading is the forward edge heading rotated 180 deg.
    assert end == pytest.approx((edge_heading + 180.0) % 360.0, abs=1e-9)

    # ...and it matches the end heading ``compile_path`` leaves the chassis
    # on (recovered by replaying its emitted primitives).
    prims = compile_path(["A", "B"], start_heading, g, None)
    replayed = _replay_primitives(prims, start_heading)
    assert replayed[-1] == pytest.approx(end, abs=1e-9)


def test_final_heading_after_forward_hop_returns_edge_heading() -> None:
    """TC8 (forward case): a FRONT hop still returns the plain edge heading.

    A bare BLIND edge with no directional override resolves FRONT (the auto
    branch only flips to REAR past the turn threshold; a 90 deg edge from a
    0 deg start stays FRONT). The returned heading must be the raw forward
    edge heading and must agree with the heading ``compile_path`` drives.
    """
    edge_heading = 90.0
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge("A", "B", type=EDGE_BLIND, distance_cm=40.0, heading_deg=edge_heading)
    g.add_edge("B", "A", type=EDGE_BLIND, distance_cm=40.0, heading_deg=270.0)

    start_heading = 0.0
    end = final_heading_after(["A", "B"], start_heading, g)

    assert end == pytest.approx(edge_heading, abs=1e-9)

    prims = compile_path(["A", "B"], start_heading, g, None)
    replayed = _replay_primitives(prims, start_heading)
    assert replayed[-1] == pytest.approx(end, abs=1e-9)


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

    Module-level constants ``BIDIRECTIONAL_AUTO_ENABLED`` / ``LINE_BOARD_COUNT``
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
            bidirectional_enabled=_runtime.BIDIRECTIONAL_AUTO_ENABLED,
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


# -- compile_path: BLIND STOP_AT_LINE (TC1 / TC2 / TC12) ---------------------
#
# The stop-at-line work makes the FRONT blind branch mode-sensitive for the
# first time: a BLIND edge whose effective ``BlindMode`` is ``STOP_AT_LINE``
# compiles to ``(ACT_STOP_AT_LINE, distance_cm)`` instead of the plain
# ``(ACT_FORWARD, distance_cm)``; a REAR traversal of the same edge keeps the
# existing ``(ACT_BLIND_REVERSE, v, distance_cm, blind_mode)`` shape but now
# carries ``BlindMode.STOP_AT_LINE`` in the mode slot. The override is keyed
# on the ``primitive_mode`` graph attribute (a :class:`BlindMode` enum), the
# same slot ``resolve_blind_mode`` reads -- the ``field_data`` schema spells
# this per-edge as a ``primitive_mode`` override, so the tests author the
# graph edge directly the same way the override-resolution tests do.


def _stop_at_line_blind_graph(
    distance_cm: float,
    heading_deg: float,
    *,
    force_reverse: bool = False,
) -> nx.DiGraph:
    """Build a 2-node A->B graph with a STOP_AT_LINE BLIND edge.

    The forward (A, B) edge carries ``primitive_mode=BlindMode.STOP_AT_LINE``
    (and, when @p force_reverse is set, ``direction_override=FORCE_REVERSE``
    so the hop resolves REAR without depending on the auto heading delta).
    The reverse (B, A) edge mirrors the mode but never inherits the
    directional override, matching :func:`graph_builder._emit_directed_pair`.

    :param distance_cm: forward edge length in centimetres.
    :param heading_deg: forward edge heading in ``[0, 360)`` degrees.
    :param force_reverse: when True, tag the forward edge
        ``FORCE_REVERSE`` so :func:`compile_path` resolves it REAR.
    :return: a 2-node :class:`networkx.DiGraph` ready for ``compile_path``.
    """
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    fwd_attrs = dict(
        type=EDGE_BLIND,
        distance_cm=distance_cm,
        heading_deg=heading_deg,
        primitive_mode=BlindMode.STOP_AT_LINE,
    )
    if force_reverse:
        fwd_attrs["direction_override"] = int(DirectionMode.FORCE_REVERSE)
    g.add_edge("A", "B", **fwd_attrs)
    g.add_edge(
        "B", "A",
        type=EDGE_BLIND,
        distance_cm=distance_cm,
        heading_deg=(heading_deg + 180.0) % 360.0,
        primitive_mode=BlindMode.STOP_AT_LINE,
    )
    return g


def test_compile_stop_at_line_blind_edge_front_emits_stop_at_line() -> None:
    """TC1: a FRONT BLIND edge with blind_mode=STOP_AT_LINE emits ACT_STOP_AT_LINE.

    Start heading 0 deg matches the edge heading 0 deg, so no ACT_TURN_REL
    is emitted: the compiler produces exactly one primitive carrying the
    edge distance.
    """
    g = _stop_at_line_blind_graph(40.0, 0.0)

    prims = compile_path(["A", "B"], 0.0, g, None)

    assert prims == [(ACT_STOP_AT_LINE, pytest.approx(40.0, abs=1e-9))]


def test_compile_stop_at_line_blind_edge_front_with_turn() -> None:
    """TC1 (turn variant): a non-zero heading delta emits TURN_REL then STOP_AT_LINE."""
    g = _stop_at_line_blind_graph(40.0, 90.0)

    prims = compile_path(["A", "B"], 0.0, g, None)

    assert prims == [
        (ACT_TURN_REL, pytest.approx(90.0, abs=1e-9)),
        (ACT_STOP_AT_LINE, pytest.approx(40.0, abs=1e-9)),
    ]


def test_compile_stop_at_line_blind_edge_rear_2board_emits_stop_at_line_reverse() -> None:
    """A 2-board REAR STOP_AT_LINE blind edge seeks via ACT_STOP_AT_LINE_REVERSE.

    On the default 2-board chassis (module ``LINE_BOARD_COUNT == 2``), a
    forced-REAR STOP_AT_LINE edge has a rear board facing the travel
    direction, so ``resolve_stop_at_line`` returns ``seek=True``. The hop
    turns 180 deg (so the rear sensor leads) and emits the reverse seek
    tuple ``(ACT_STOP_AT_LINE_REVERSE, target_node, distance_cm, blind_mode)``
    whose mode slot carries the RAW edge mode ``BlindMode.STOP_AT_LINE``.
    Byte-identical to the REAR branch of ``PathTrackerSim.go_to`` (AC1).
    """
    g = _stop_at_line_blind_graph(40.0, 0.0, force_reverse=True)

    prims = compile_path(["A", "B"], 0.0, g, None)

    assert prims == [
        (ACT_TURN_REL, pytest.approx(180.0, abs=1e-9)),
        (
            ACT_STOP_AT_LINE_REVERSE,
            "B",
            pytest.approx(40.0, abs=1e-9),
            BlindMode.STOP_AT_LINE,
        ),
    ]


def test_compile_stop_at_line_blind_edge_rear_1board_degrades_to_blind_reverse(
    monkeypatch,
) -> None:
    """A 1-board forced-REAR STOP_AT_LINE blind edge degrades to ACT_BLIND_REVERSE.

    With ``LINE_BOARD_COUNT == 1`` there is no rear board to seek with, so
    ``resolve_stop_at_line`` returns ``seek=False`` for the forced-REAR hop.
    It degrades to a reverse dead-reckon reusing the existing blind-reverse
    tuple ``(ACT_BLIND_REVERSE, target_node, distance_cm, blind_mode)`` whose
    mode slot still carries the RAW edge mode ``BlindMode.STOP_AT_LINE``. The
    forced-REAR edge first turns 180 deg so the rear sensor faces along the
    edge. Byte-identical to the REAR branch of ``PathTrackerSim.go_to`` (AC2).
    """
    monkeypatch.setattr(_runtime, "LINE_BOARD_COUNT", 1)
    g = _stop_at_line_blind_graph(40.0, 0.0, force_reverse=True)

    prims = compile_path(["A", "B"], 0.0, g, None)

    assert prims == [
        (ACT_TURN_REL, pytest.approx(180.0, abs=1e-9)),
        (ACT_BLIND_REVERSE, "B", pytest.approx(40.0, abs=1e-9), BlindMode.STOP_AT_LINE),
    ]


def test_compile_stop_at_line_only_on_stop_at_line_mode() -> None:
    """A plain (SYNC_TSHAPE-default) BLIND edge still emits ACT_FORWARD.

    Guards the mode-sensitivity boundary: the STOP_AT_LINE action must NOT
    leak onto blind edges that do not carry the override. A bare BLIND edge
    (no ``primitive_mode``) inherits ``PIPELINE_CONFIG.blind`` and keeps the
    dead-reckoned forward hop.
    """
    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_edge("A", "B", type=EDGE_BLIND, distance_cm=40.0, heading_deg=0.0)
    g.add_edge("B", "A", type=EDGE_BLIND, distance_cm=40.0, heading_deg=180.0)

    prims = compile_path(["A", "B"], 0.0, g, None)

    assert prims == [(ACT_FORWARD, pytest.approx(40.0, abs=1e-9))]


def _interleave_overshoot(primitives: List[Primitive]) -> List[Primitive]:
    """Re-add the sim-only ``ACT_LINE_OVERSHOOT`` step after each LINE primitive.

    ``PathTrackerSim.go_to`` emits an ``ACT_LINE_OVERSHOOT`` step after every
    EDGE_LINE traversal (the runtime owns that FSM transition; the compiler
    does not). To compare ``compile_path`` output against the sim's output we
    re-insert the overshoot step in the same positions. STOP_AT_LINE blind
    edges add no overshoot step (they are blind hops, not line hops), so the
    helper is a no-op across them -- which is exactly what proves byte-parity
    on the blind action itself. Mirrors ``_interleave_overshoot`` in
    ``test_runtime_sim.py``.
    """
    out: List[Primitive] = []
    for prim in primitives:
        out.append(prim)
        if prim[0] in (ACT_FOLLOW_INT, ACT_LINE_REVERSE):
            out.append((ACT_LINE_OVERSHOOT, LINE_SENSOR_AXLE_OFFSET_CM))
    return out


def test_stop_at_line_compiler_sim_parity_rear() -> None:
    """TC12 (REAR): compiler and sim emit byte-identical streams over a REAR STOP_AT_LINE hop.

    Closes the coverage gap left by the FORWARD parity test (TC12) and the
    REAR emission unit tests (T7).  A two-hop path
    ``A --LINE(heading 0)--> B --BLIND(STOP_AT_LINE, FORCE_REVERSE)--> C``
    forces the BLIND hop REAR via ``direction_override=FORCE_REVERSE`` (the
    same technique T7's emission tests use), so the result is deterministic
    regardless of auto-direction geometry.

    On the default 2-board chassis (``LINE_BOARD_COUNT == 2``) the resolver
    returns ``seek=True`` for the REAR hop, so the compiler emits
    ``(ACT_STOP_AT_LINE_REVERSE, "C", 50.0, BlindMode.STOP_AT_LINE)`` and the
    sim re-anchors via ``on_intersection``.  The LINE hop produces a 180-degree
    body-heading change at B (forward heading 0 → REAR body heading 180), which
    exceeds ``NAV_TURN_TOLERANCE_DEG``, so the sim emits ``ACT_LINE_OVERSHOOT``
    (not ``ACT_LINE_PASS``) — satisfying the ``_interleave_overshoot`` contract.

    Assertions (AC1):
    - ``ACT_STOP_AT_LINE_REVERSE`` with the correct 4-tuple appears in both
      the compiler stream and the sim stream.
    - The two streams are byte-identical once the sim-only ``ACT_LINE_OVERSHOOT``
      steps are re-interleaved into the compiler output.
    - The sim's final localization node is the destination ``"C"``.
    """
    from tools.path_planner.sim import PathTrackerSim, VirtualChassis, walk_next_hop

    # Three nodes in a line; A→B is the LINE hop, B→C is the BLIND REAR hop.
    # All on heading 0 so the start heading 0 requires no initial turn, and the
    # REAR body heading for B→C is 180 — a clean 180-degree turn that is
    # guaranteed to break pass-through at B.
    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0), "C": (100.0, 0.0)}
    fake_field = SimpleNamespace(
        NODES=nodes, START_NODE="A", START_HEADING_DEG=0.0
    )
    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y))
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=50.0, heading_deg=0.0)
    g.add_edge("B", "A", type=EDGE_LINE, distance_cm=50.0, heading_deg=180.0)
    # BLIND edge with STOP_AT_LINE mode and FORCE_REVERSE so the resolver sees
    # direction=REAR; on a 2-board chassis this returns seek=True.
    g.add_edge(
        "B", "C",
        type=EDGE_BLIND,
        distance_cm=50.0,
        heading_deg=0.0,
        primitive_mode=BlindMode.STOP_AT_LINE,
        direction_override=int(DirectionMode.FORCE_REVERSE),
    )
    g.add_edge(
        "C", "B",
        type=EDGE_BLIND,
        distance_cm=50.0,
        heading_deg=180.0,
        primitive_mode=BlindMode.STOP_AT_LINE,
    )
    next_hop = {
        "A": {"A": "A", "B": "B", "C": "B"},
        "B": {"A": "A", "B": "B", "C": "C"},
        "C": {"A": "B", "B": "B", "C": "C"},
    }

    walked = walk_next_hop(next_hop, "A", "C")
    compiler_prims = compile_path(walked, fake_field.START_HEADING_DEG, g, fake_field)

    chassis = VirtualChassis.from_field_data(fake_field)
    sim = PathTrackerSim(fake_field, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    sim_prims = sim.go_to("C")

    # The STOP_AT_LINE_REVERSE action must appear in both streams with the
    # correct 4-tuple shape (action, target_node, distance_cm, blind_mode).
    assert (
        ACT_STOP_AT_LINE_REVERSE, "C", pytest.approx(50.0, abs=1e-9), BlindMode.STOP_AT_LINE
    ) in compiler_prims
    assert (
        ACT_STOP_AT_LINE_REVERSE, "C", pytest.approx(50.0, abs=1e-9), BlindMode.STOP_AT_LINE
    ) in sim_prims
    # Byte-identical once the sim-only overshoot steps are re-interleaved.
    assert sim_prims == _interleave_overshoot(compiler_prims)
    assert sim.localization.current_node() == "C"


def test_stop_at_line_compiler_sim_parity() -> None:
    """TC12: compiler and sim emit byte-identical streams over a STOP_AT_LINE hop.

    Re-establishes the compiler<->sim parity invariant for the now
    mode-sensitive FRONT blind branch. A two-hop path
    ``A --LINE--> B --BLIND(STOP_AT_LINE)--> C`` with a 90 deg turn at B
    (so the sim emits ACT_LINE_OVERSHOOT, not ACT_LINE_PASS, at the
    intermediate node) must produce identical action streams once the
    sim-only LINE_OVERSHOOT steps are re-interleaved into the compiler
    output. A divergence here silently breaks the sim-mirrors-robot
    guarantee on stop-at-line edges.
    """
    from tools.path_planner.sim import PathTrackerSim, VirtualChassis, walk_next_hop

    nodes = {"A": (0.0, 0.0), "B": (50.0, 0.0), "C": (50.0, 50.0)}
    fake_field = SimpleNamespace(
        NODES=nodes, START_NODE="A", START_HEADING_DEG=0.0
    )
    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y))
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=50.0, heading_deg=0.0)
    g.add_edge("B", "A", type=EDGE_LINE, distance_cm=50.0, heading_deg=180.0)
    g.add_edge(
        "B", "C",
        type=EDGE_BLIND,
        distance_cm=50.0,
        heading_deg=90.0,
        primitive_mode=BlindMode.STOP_AT_LINE,
    )
    g.add_edge(
        "C", "B",
        type=EDGE_BLIND,
        distance_cm=50.0,
        heading_deg=270.0,
        primitive_mode=BlindMode.STOP_AT_LINE,
    )
    next_hop = {
        "A": {"A": "A", "B": "B", "C": "B"},
        "B": {"A": "A", "B": "B", "C": "C"},
        "C": {"A": "B", "B": "B", "C": "C"},
    }

    walked = walk_next_hop(next_hop, "A", "C")
    compiler_prims = compile_path(walked, fake_field.START_HEADING_DEG, g, fake_field)

    chassis = VirtualChassis.from_field_data(fake_field)
    sim = PathTrackerSim(fake_field, g, next_hop, chassis)
    sim.localization.zero_heading_from_current()
    sim_prims = sim.go_to("C")

    # The STOP_AT_LINE action must survive in both streams unchanged.
    assert (ACT_STOP_AT_LINE, pytest.approx(50.0, abs=1e-9)) in compiler_prims
    assert (ACT_STOP_AT_LINE, pytest.approx(50.0, abs=1e-9)) in sim_prims
    # Byte-identical once the sim-only overshoot steps are re-interleaved.
    assert sim_prims == _interleave_overshoot(compiler_prims)
    assert sim.localization.current_node() == "C"
