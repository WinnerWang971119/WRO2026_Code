"""Tests for tools.path_planner.graph_builder (PR3 schema).

PR3 graph: LINE_SEGMENTS and BLIND_EDGES, both bidirectional.
"""

from __future__ import annotations

import copy
import warnings
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    build_graph,
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


def test_build_graph_smoke() -> None:
    g = build_graph(real_field_data)
    assert isinstance(g, nx.DiGraph)
    assert g.number_of_nodes() == len(real_field_data.NODES)
    for name in real_field_data.NODES:
        assert name in g.nodes
    assert g.number_of_edges() > 0


def test_line_segments_are_bidirectional() -> None:
    g = build_graph(real_field_data)
    for u, v in real_field_data.LINE_SEGMENTS:
        assert g.has_edge(u, v)
        assert g.has_edge(v, u)
        assert g.edges[u, v]["type"] == EDGE_LINE
        assert g.edges[v, u]["type"] == EDGE_LINE


def test_blind_edges_are_bidirectional() -> None:
    g = build_graph(real_field_data)
    for u, v in real_field_data.BLIND_EDGES:
        assert g.has_edge(u, v)
        assert g.has_edge(v, u)
        assert g.edges[u, v]["type"] == EDGE_BLIND
        assert g.edges[v, u]["type"] == EDGE_BLIND


def test_headings_differ_by_180_on_reverse() -> None:
    g = build_graph(real_field_data)
    for u, v, data in g.edges(data=True):
        assert g.has_edge(v, u), f"missing reverse edge for {u}->{v}"
        forward = data["heading_deg"]
        reverse = g.edges[v, u]["heading_deg"]
        delta = (forward - reverse) % 360.0
        assert delta == pytest.approx(180.0, abs=1e-6)


def test_obstacle_inflation_drops_clipped_edge() -> None:
    fd = _clone_field_data()
    # Wall straddles the A-B segment midpoint (placeholder field puts
    # A=(10,10), B=(200,10)).
    fd.OBSTACLES = [(50.0, 5.0, 70.0, 15.0, "wall")]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    assert not g.has_edge("A", "B")
    assert not g.has_edge("B", "A")


def test_disconnected_node_warns() -> None:
    """A wall blocking every approach to a node emits a UserWarning."""
    fd = _clone_field_data()
    # Drop the only blind connection to G, and wall off every line node
    # near G. Easiest: remove the blind edge.
    fd.BLIND_EDGES = []
    with pytest.warns(UserWarning, match=r"Node 'G'"):
        build_graph(fd)


def test_heading_correctness_for_known_segment() -> None:
    g = build_graph(real_field_data)
    # A=(10, 10) -> B=(200, 10): East, heading 0.
    assert g.edges["A", "B"]["heading_deg"] == pytest.approx(0.0, abs=1e-6)
    # Reverse: West, heading 180.
    assert g.edges["B", "A"]["heading_deg"] == pytest.approx(180.0, abs=1e-6)
    # A=(10, 10) -> D=(10, 150): North, heading 90.
    assert g.edges["A", "D"]["heading_deg"] == pytest.approx(90.0, abs=1e-6)
