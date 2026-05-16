"""Tests for tools.path_planner.planner (PR3 schema)."""

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
