"""Tests for tools.path_planner.planner (PR3 schema)."""

from __future__ import annotations

import copy
import math
import warnings
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner.tests.fixtures import toy_field_data as real_field_data
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, build_graph
from tools.path_planner.planner import (
    compute_all_pairs_next_hop,
    compute_path_cost,
)

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
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


def _make_blind_vs_line_graph() -> nx.DiGraph:
    """Three-node graph for the line-vs-blind cost-tradeoff test.

    Layout::

        A --[BLIND, 10cm]--> B
        A --[LINE, 10cm]--> C --[LINE, 12cm]--> B
    """
    g = nx.DiGraph()
    for name, (x, y) in (("A", (0.0, 0.0)), ("B", (20.0, 0.0)), ("C", (10.0, 5.0))):
        g.add_node(name, x_cm=x, y_cm=y, type="line")
    g.add_edge("A", "B", type=EDGE_BLIND, distance_cm=10.0, heading_deg=0.0)
    g.add_edge("A", "C", type=EDGE_LINE, distance_cm=10.0, heading_deg=0.0)
    g.add_edge("C", "B", type=EDGE_LINE, distance_cm=12.0, heading_deg=0.0)
    return g


def test_self_loop_sentinel() -> None:
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, real_field_data.COST_BLIND_MULTIPLIER)
    for node in g.nodes():
        assert next_hop[node][node] == node


def test_dijkstra_prefers_line_over_blind() -> None:
    g = _make_blind_vs_line_graph()
    table_penalised = compute_all_pairs_next_hop(g, cost_blind_multiplier=5.0)
    assert table_penalised["A"]["B"] == "C"
    table_unpenalised = compute_all_pairs_next_hop(g, cost_blind_multiplier=1.0)
    assert table_unpenalised["A"]["B"] == "B"


def test_unreachable_pair_is_none() -> None:
    """Removing the blind edge to G leaves G unreachable from every other node."""
    fd = _clone_field_data()
    fd.BLIND_EDGES = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(g, fd.COST_BLIND_MULTIPLIER)
    for source in g.nodes():
        if source == "G":
            assert next_hop["G"]["G"] == "G"
            continue
        assert next_hop[source]["G"] is None


def test_next_hop_table_has_every_pair() -> None:
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, real_field_data.COST_BLIND_MULTIPLIER)
    nodes = set(g.nodes())
    assert set(next_hop.keys()) == nodes
    for source, row in next_hop.items():
        assert set(row.keys()) == nodes


def test_compute_path_cost_line_edge() -> None:
    g = build_graph(real_field_data)
    # A=(10,10), B=(200,10): straight line 190 cm.
    expected = 190.0
    cost = compute_path_cost(g, ["A", "B"], real_field_data.COST_BLIND_MULTIPLIER)
    assert cost == pytest.approx(expected, abs=1e-6)


def test_compute_path_cost_blind_edge_multiplier() -> None:
    g = build_graph(real_field_data)
    edge = g.edges["B", "G"]
    assert edge["type"] == EDGE_BLIND
    expected = edge["distance_cm"] * real_field_data.COST_BLIND_MULTIPLIER
    actual = compute_path_cost(g, ["B", "G"], real_field_data.COST_BLIND_MULTIPLIER)
    assert actual == pytest.approx(expected, abs=1e-6)


def test_compute_path_cost_zero_length_path() -> None:
    g = build_graph(real_field_data)
    assert compute_path_cost(g, ["A"], real_field_data.COST_BLIND_MULTIPLIER) == 0.0
    assert compute_path_cost(g, [], real_field_data.COST_BLIND_MULTIPLIER) == 0.0


def test_compute_path_cost_missing_edge_raises() -> None:
    g = build_graph(real_field_data)
    # A and G have no direct edge (G is reached via B).
    assert not g.has_edge("A", "G")
    with pytest.raises(KeyError):
        compute_path_cost(g, ["A", "G"], real_field_data.COST_BLIND_MULTIPLIER)


# -- pivot compensation tests (T13) ------------------------------------------


def test_pivot_compensation_zero_turn_no_displacement():
    """A 0 deg pivot compensates to no geometry change on the following edge."""
    from tools.path_planner.planner import compute_pivot_compensation

    adj_dist, adj_heading = compute_pivot_compensation(
        turn_deg=0.0,
        track_cm=15.0,
        next_edge_dist=42.0,
        next_edge_heading=90.0,
    )
    assert adj_dist == pytest.approx(42.0)
    assert adj_heading == pytest.approx(90.0)


def test_pivot_compensation_90deg_with_track_15cm_yields_75cm_reduction():
    """90 deg pivot, TRACK_CM=15 -> chord 10.6066cm, forward projection 7.5cm.

    chord_cm = 2 * (15/2) * sin(45 deg) = 15 * sqrt(2)/2 ~= 10.6066
    forward_component = chord_cm * sin(45 deg) = 10.6066 * sqrt(2)/2 = 7.5000
    adjusted_dist = 30 - 7.5 = 22.5
    """
    from tools.path_planner.planner import compute_pivot_compensation

    adj_dist, adj_heading = compute_pivot_compensation(
        turn_deg=90.0,
        track_cm=15.0,
        next_edge_dist=30.0,
        next_edge_heading=90.0,
    )
    assert adj_dist == pytest.approx(22.5, abs=1e-6)
    assert adj_heading == pytest.approx(90.0)


def test_pivot_compensation_neg_90deg_yields_same_magnitude():
    """Negative turn (right pivot) yields same magnitude as positive turn."""
    from tools.path_planner.planner import compute_pivot_compensation

    pos, _ = compute_pivot_compensation(+90.0, 15.0, 30.0, 0.0)
    neg, _ = compute_pivot_compensation(-90.0, 15.0, 30.0, 0.0)
    assert pos == pytest.approx(neg)


def test_pivot_compensation_clamps_to_nonneg_distance():
    """If the forward component exceeds next_edge_dist, adjusted_dist clamps to 0."""
    from tools.path_planner.planner import compute_pivot_compensation

    # 180 deg pivot with TRACK_CM=15: chord = 15, forward_component = 15 * 1 = 15
    # next_edge_dist = 5 -> adjusted should clamp to 0
    adj_dist, _ = compute_pivot_compensation(180.0, 15.0, 5.0, 0.0)
    assert adj_dist == pytest.approx(0.0)


def test_apply_pivot_compensations_skips_multi_successor():
    """A pivot landing with >1 forward out-edge is skipped, not raised.

    The out-degree precondition is now enforced by the validator (finding
    #36), so ``apply_pivot_compensations`` no longer raises on a mismatch:
    for direct / hand-built callers that bypass ``validate`` it skips the
    offending pivot edge, leaving every following edge uncompensated rather
    than mis-compensating an arbitrary branch. The back-edge to ``src`` is
    excluded from the count -- a bidirectional pair plus 2 real forward
    continuations is still a multi-successor landing.
    """
    from tools.path_planner.graph_builder import Pivot
    from tools.path_planner.planner import apply_pivot_compensations

    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_node("C")
    g.add_node("D")
    g.add_edge("A", "B", distance_cm=10.0, heading_deg=0.0, pivot=int(Pivot.LEFT))
    g.add_edge("B", "C", distance_cm=20.0, heading_deg=90.0)
    g.add_edge("B", "D", distance_cm=15.0, heading_deg=-90.0)

    # Must not raise; must not touch either successor's distance.
    apply_pivot_compensations(g, track_cm=15.0)
    assert g.edges["B", "C"]["distance_cm"] == pytest.approx(20.0)
    assert g.edges["B", "D"]["distance_cm"] == pytest.approx(15.0)


# -- Codex P1 regression: directional overrides must not mirror to reverse --


def test_emit_directed_pair_pivot_not_mirrored_to_reverse():
    """`pre_turn_mode` and `pivot` are direction-dependent and must stay forward-only.

    Codex P1 regression: mirroring these onto the reverse edge would steer
    the chassis with the wrong wheel pattern on backward traversal.
    """
    from tools.path_planner.graph_builder import EDGE_LINE, Pivot, _emit_directed_pair

    g = nx.DiGraph()
    g.add_node("U")
    g.add_node("V")
    _emit_directed_pair(
        g,
        "U",
        "V",
        pu=(0.0, 0.0),
        pv=(10.0, 0.0),
        edge_type=EDGE_LINE,
        options={
            "pivot": Pivot.LEFT,
            "pre_turn_mode": "PIVOT_LEFT",
            "primitive_mode": "NORMAL",
        },
    )
    forward = g.edges["U", "V"]
    reverse = g.edges["V", "U"]

    # Forward edge carries every override.
    assert forward.get("pivot") == int(Pivot.LEFT)
    assert forward.get("pre_turn_mode") == "PIVOT_LEFT"
    assert forward.get("primitive_mode") == "NORMAL"

    # Reverse edge must NOT inherit the directional overrides.
    assert "pivot" not in reverse
    assert "pre_turn_mode" not in reverse
    # `primitive_mode` is direction-symmetric (selects how to follow the
    # segment, not which way) so it stays mirrored.
    assert reverse.get("primitive_mode") == "NORMAL"


def test_direction_override_not_mirrored_to_reverse_edge():
    """TC15 / Codex P1 regression: EDGES `direction` is direction-dependent.

    An override on (A, B) with direction='rear' must produce
    direction_override = FORCE_REVERSE on the (A, B) directed edge ONLY.
    The reverse (B, A) edge must retain DEFAULT -- mirroring would silently
    flip the chassis on backward traversal of an unrelated hop.
    """
    fd = SimpleNamespace(
        NODES={"A": (0.0, 0.0), "B": (100.0, 0.0)},
        NODE_TYPES={"A": "line", "B": "line"},
        LINE_SEGMENTS=[("A", "B")],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=1.0,
        COST_BLIND_MULTIPLIER=1.0,
        EDGES=[{"type": EDGE_LINE, "u": "A", "v": "B", "distance_cm": 100.0,
                "heading_deg": 0.0, "direction": "rear"}],
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(200, 200),
        MAP_PNG="maps/map.png",
    )
    graph = build_graph(fd)
    fwd = graph.edges["A", "B"]
    rev = graph.edges["B", "A"]
    # Forward edge has the override (integer 2 = FORCE_REVERSE).
    assert fwd["direction_override"] == 2
    # Reverse edge must NOT have the override copied (absent or DEFAULT==0).
    assert rev.get("direction_override", 0) == 0


def test_apply_pivot_compensations_ignores_back_edge():
    """Bidirectional graph: pivot dst with back-edge + 1 forward edge must compensate.

    Codex P1 regression: counting all out-edges (including the back-edge
    to ``src``) makes pivot compensation unusable on the bidirectional
    graphs ``build_graph`` actually produces.
    """
    from tools.path_planner.graph_builder import Pivot
    from tools.path_planner.planner import apply_pivot_compensations

    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_node("C")
    # Forward pivot edge A->B, plus the bidirectional back-edge B->A.
    g.add_edge("A", "B", distance_cm=10.0, heading_deg=0.0, pivot=int(Pivot.LEFT))
    g.add_edge("B", "A", distance_cm=10.0, heading_deg=180.0)
    # Single real continuation off B (plus the back-edge above).
    g.add_edge("B", "C", distance_cm=30.0, heading_deg=90.0)
    g.add_edge("C", "B", distance_cm=30.0, heading_deg=270.0)

    apply_pivot_compensations(g, track_cm=15.0)

    # The B->C edge should have been shortened by the 90 deg pivot's
    # forward chord component (7.5 cm for TRACK_CM=15, |theta|=90 deg).
    assert g.edges["B", "C"]["distance_cm"] == pytest.approx(22.5, abs=1e-6)
    # The back-edge B->A must NOT have been touched.
    assert g.edges["B", "A"]["distance_cm"] == pytest.approx(10.0)


def test_compute_all_pairs_next_hop_compensates_input_graph_in_place():
    """TC9 / finding #42: pivot compensation reaches the SHARED graph.

    Decision B: ``compute_all_pairs_next_hop`` now applies pivot
    compensation to its input ``graph`` in place (not to a discarded
    weighted copy). The caller passes that same ``graph`` object to
    ``emit_routes`` immediately afterward, so the shortened distance must
    survive on the graph -- otherwise routes.cpp emits the un-shortened
    drive distance and the robot overshoots every pivot landing.

    Bidirectional layout mirroring
    :func:`test_apply_pivot_compensations_ignores_back_edge`: A->B is a
    90 deg LEFT pivot, B has exactly one forward continuation B->C
    (the back-edge B->A is excluded from the out-degree count). For
    TRACK_CM=15 and a 90 deg pivot the forward chord component is 7.5 cm,
    so B->C is shortened from 30.0 to 22.5 cm.
    """
    from tools.path_planner.graph_builder import Pivot

    g = nx.DiGraph()
    for name, (x, y) in (("A", (0.0, 0.0)), ("B", (10.0, 0.0)), ("C", (10.0, 30.0))):
        g.add_node(name, x_cm=x, y_cm=y, type="line")
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=10.0, heading_deg=0.0,
               pivot=int(Pivot.LEFT))
    g.add_edge("B", "A", type=EDGE_LINE, distance_cm=10.0, heading_deg=180.0)
    g.add_edge("B", "C", type=EDGE_LINE, distance_cm=30.0, heading_deg=90.0)
    g.add_edge("C", "B", type=EDGE_LINE, distance_cm=30.0, heading_deg=270.0)

    # Drive the FULL planner flow (not apply_pivot_compensations directly).
    compute_all_pairs_next_hop(g, cost_blind_multiplier=1.0)

    # The shared input graph now carries the compensated B->C distance.
    assert g.edges["B", "C"]["distance_cm"] == pytest.approx(22.5, abs=1e-6)
    # The pivot edge itself and the back-edge are untouched.
    assert g.edges["A", "B"]["distance_cm"] == pytest.approx(10.0)
    assert g.edges["B", "A"]["distance_cm"] == pytest.approx(10.0)
