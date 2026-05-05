"""Tests for tools.path_planner.planner.

Covers TC7 (Dijkstra prefers a longer line route over a shorter blind
shortcut when the blind multiplier penalises blind edges) and TC8
(self-loop sentinel) from the PR 1 plan, plus a handful of structural
guardrails: full table coverage, unreachable -> None, internal
consistency between next_hop walking and direct Dijkstra paths, and the
``compute_path_cost`` helper.
"""

from __future__ import annotations

import copy
import math
import warnings
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, build_graph
from tools.path_planner.planner import (
    compute_all_pairs_next_hop,
    compute_path_cost,
)

# Mirrors the list used in test_graph.py / test_field_data.py so cloning
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
    """Return a deep-copied SimpleNamespace mirror of the real field_data.

    Mutating the returned namespace (or any nested container) does not
    affect the live ``tools.path_planner.field_data`` module.
    """
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


def _make_blind_vs_line_graph() -> nx.DiGraph:
    """Hand-rolled 3-node graph for the line-vs-blind cost-tradeoff test.

    Layout (distances in cm):

        A --[BLIND, 10cm]--> B
        A --[LINE, 10cm]--> C --[LINE, 12cm]--> B

    With ``cost_blind_multiplier = 5.0`` the blind shortcut costs 50 and
    the line detour costs 22, so Dijkstra must pick A -> C -> B. With a
    multiplier of ``1.0`` the blind shortcut costs 10 and wins outright.
    """
    g = nx.DiGraph()
    for name, (x, y) in (("A", (0.0, 0.0)), ("B", (20.0, 0.0)), ("C", (10.0, 5.0))):
        g.add_node(name, x_cm=x, y_cm=y, type="line")
    # Blind shortcut, one-way like the real goal approaches.
    g.add_edge("A", "B", type=EDGE_BLIND, distance_cm=10.0, heading_deg=0.0)
    # Line detour A -> C -> B (line edges are bidirectional in the real
    # graph; we only need the forward direction for this test).
    g.add_edge("A", "C", type=EDGE_LINE, distance_cm=10.0, heading_deg=0.0)
    g.add_edge("C", "B", type=EDGE_LINE, distance_cm=12.0, heading_deg=0.0)
    return g


# -- TC-prefixed tests --------------------------------------------------------


def test_self_loop_sentinel() -> None:
    """TC8: ``next_hop[n][n] == n`` for every node in the toy graph."""
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, real_field_data.COST_BLIND_MULTIPLIER)
    for node in g.nodes():
        assert next_hop[node][node] == node, f"self-loop missing for '{node}'"


def test_dijkstra_prefers_line_over_blind() -> None:
    """TC7: a 5x blind multiplier flips Dijkstra from the blind shortcut to the line detour.

    Hand-rolled graph: blind A->B is 10cm, line A->C->B is 10+12=22cm.
    With multiplier 5.0 the blind edge effectively costs 50, so the
    line detour wins (next hop is ``C``). With multiplier 1.0 the blind
    edge costs 10, so the direct hop wins (next hop is ``B``).
    """
    g = _make_blind_vs_line_graph()

    table_penalised = compute_all_pairs_next_hop(g, cost_blind_multiplier=5.0)
    assert table_penalised["A"]["B"] == "C"

    table_unpenalised = compute_all_pairs_next_hop(g, cost_blind_multiplier=1.0)
    assert table_unpenalised["A"]["B"] == "B"


def test_unreachable_pair_is_none() -> None:
    """A wall covering the blind edge to ``G`` makes ``G`` unreachable from every other node.

    Reuses the disconnected-goal pattern from TC14 in test_graph.py:
    placing an obstacle that clips the blind linestring from ``_proj_G``
    to ``G`` drops the only incoming edge for ``G``. Every
    ``next_hop[*][G]`` for ``* != G`` must therefore be ``None``.
    """
    fd = _clone_field_data()
    fd.OBSTACLES = [(70.0, 25.0, 80.0, 35.0, "wall")]

    # The graph builder warns about both the dropped goal and the
    # now-isolated G node; we intentionally ignore both.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)

    next_hop = compute_all_pairs_next_hop(g, fd.COST_BLIND_MULTIPLIER)
    for source in g.nodes():
        if source == "G":
            # Self-loop sentinel still applies to G itself.
            assert next_hop["G"]["G"] == "G"
            continue
        assert next_hop[source]["G"] is None, (
            f"expected next_hop['{source}']['G'] to be None after isolating G"
        )


def test_next_hop_traverses_optimal_path() -> None:
    """Walking ``next_hop`` from A to G reconstructs the same path Dijkstra returns.

    This is the key consistency check: if the table is internally
    correct, repeatedly indexing ``next_hop[current][target]`` should
    enumerate the optimal path node-by-node and terminate at the target.
    """
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, real_field_data.COST_BLIND_MULTIPLIER)

    source, target = "A", "G"

    # Reference path direct from networkx (using the same weighted graph
    # the planner builds internally is overkill - we just need to know
    # that Dijkstra produces *some* optimal path that reaches G).
    weighted = nx.DiGraph()
    for node, attrs in g.nodes(data=True):
        weighted.add_node(node, **attrs)
    for u, v, data in g.edges(data=True):
        mul = real_field_data.COST_BLIND_MULTIPLIER if data["type"] == EDGE_BLIND else 1.0
        weighted.add_edge(u, v, cost=data["distance_cm"] * mul, **data)
    _lengths, paths = nx.single_source_dijkstra(weighted, source, weight="cost")
    reference_path = paths[target]

    # Walk the next-hop table from source to target and confirm equality.
    walked = [source]
    current = source
    # Hard upper bound on iterations to fail loud if the table cycles.
    for _ in range(len(g.nodes()) + 1):
        if current == target:
            break
        nxt = next_hop[current][target]
        assert nxt is not None, f"unexpected dead-end at '{current}' -> '{target}'"
        walked.append(nxt)
        current = nxt
    else:
        pytest.fail(f"next_hop walk did not reach '{target}' within node budget")

    assert walked == reference_path


def test_next_hop_table_has_every_pair() -> None:
    """For ``N`` nodes the table has ``N`` rows and each row has ``N`` entries."""
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, real_field_data.COST_BLIND_MULTIPLIER)

    nodes = set(g.nodes())
    assert set(next_hop.keys()) == nodes
    for source, row in next_hop.items():
        assert set(row.keys()) == nodes, (
            f"row for source '{source}' missing keys: "
            f"{nodes - set(row.keys())}"
        )


def test_compute_path_cost_smoke() -> None:
    """``compute_path_cost`` returns the expected float for a known short path.

    A -> E in the toy graph is a single LINE edge of length sqrt(30^2 + 30^2)
    = 30 * sqrt(2). The blind multiplier is irrelevant because the edge
    type is LINE.
    """
    g = build_graph(real_field_data)
    expected = 30.0 * math.sqrt(2.0)
    cost = compute_path_cost(g, ["A", "E"], real_field_data.COST_BLIND_MULTIPLIER)
    assert cost == pytest.approx(expected, abs=1e-6)

    # A blind path picks up the multiplier: _proj_G -> G is the toy
    # field's only blind edge, length 30cm, so cost is 30 * 5 = 150.
    blind_cost = compute_path_cost(
        g, ["_proj_G", "G"], real_field_data.COST_BLIND_MULTIPLIER
    )
    assert blind_cost == pytest.approx(
        30.0 * real_field_data.COST_BLIND_MULTIPLIER, abs=1e-6
    )


def test_compute_path_cost_zero_length_path() -> None:
    """Single-node and empty paths have cost ``0.0`` (no edges traversed)."""
    g = build_graph(real_field_data)
    assert compute_path_cost(g, ["A"], real_field_data.COST_BLIND_MULTIPLIER) == 0.0
    assert compute_path_cost(g, [], real_field_data.COST_BLIND_MULTIPLIER) == 0.0


def test_compute_path_cost_missing_edge_raises() -> None:
    """``compute_path_cost`` raises ``KeyError`` when a consecutive pair is not connected.

    ``B -> D`` is not a direct LINE_SEGMENTS entry in the toy fixture,
    so the graph has no ``B -> D`` edge.
    """
    g = build_graph(real_field_data)
    assert not g.has_edge("B", "D")
    with pytest.raises(KeyError):
        compute_path_cost(g, ["B", "D"], real_field_data.COST_BLIND_MULTIPLIER)


def test_blind_edge_cost_uses_multiplier() -> None:
    """The internal cost for the blind edge ``_proj_G -> G`` matches ``distance_cm * multiplier``.

    Verified end-to-end: pull the edge's raw ``distance_cm``, multiply
    by the configured multiplier, and confirm
    ``compute_path_cost(['_proj_G', 'G'], ...)`` agrees. This locks in
    that the same multiplier flows through both the planner's internal
    Dijkstra weighting and the public ``compute_path_cost`` helper.
    """
    g = build_graph(real_field_data)
    edge = g.edges["_proj_G", "G"]
    assert edge["type"] == EDGE_BLIND  # guard against a future refactor.

    expected = edge["distance_cm"] * real_field_data.COST_BLIND_MULTIPLIER
    actual = compute_path_cost(
        g, ["_proj_G", "G"], real_field_data.COST_BLIND_MULTIPLIER
    )
    assert actual == pytest.approx(expected, abs=1e-6)
