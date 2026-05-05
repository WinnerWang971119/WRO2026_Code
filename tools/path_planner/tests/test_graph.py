"""Tests for tools.path_planner.graph_builder.

Covers TC4 (segment split at projection), TC5 (forward/reverse heading
differs by 180 deg), TC6 (obstacle inflation drops clipped edges) and
TC14 (disconnected goal logs a warning) from the PR 1 plan, plus a few
extra structural guardrails (blind-edge directionality, edge-type tags,
heading correctness for known-direction segments).
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
    EDGE_PROJECTION,
    build_graph,
)

# Names of the public top-level attributes the validator inspects.
# Mirrors the list in test_field_data.py so cloning stays in lock-step.
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

    Mutating the returned namespace (or any nested container) does **not**
    affect the live ``tools.path_planner.field_data`` module.
    """
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


# -- TC-prefixed tests --------------------------------------------------------


def test_build_graph_smoke() -> None:
    """``build_graph`` returns a non-empty DiGraph with every NODES entry."""
    g = build_graph(real_field_data)
    assert isinstance(g, nx.DiGraph)
    assert g.number_of_nodes() == len(real_field_data.NODES)
    for name in real_field_data.NODES:
        assert name in g.nodes
    # Sanity: at least the bidirectional perimeter edges should survive
    # (the toy obstacle is interior and small).
    assert g.number_of_edges() > 0


def test_segment_split_at_projection() -> None:
    """TC4: segment B-C with projection ``_proj_G`` splits into B-_proj_G-C.

    The original direct edges B<->C must be absent; the four sub-edges
    around the projection must be present.
    """
    g = build_graph(real_field_data)

    # Sub-edges through the projection exist in both directions.
    assert g.has_edge("B", "_proj_G")
    assert g.has_edge("_proj_G", "B")
    assert g.has_edge("_proj_G", "C")
    assert g.has_edge("C", "_proj_G")

    # The original direct line edges are gone.
    assert not g.has_edge("B", "C")
    assert not g.has_edge("C", "B")


def test_bidirectional_headings_differ_by_180() -> None:
    """TC5: forward / reverse line edges differ in heading by 180 deg.

    Iterates every undirected pair that survived inflation. For each edge
    that has a reverse counterpart (i.e., not a one-way blind edge),
    confirm ``(forward + 180) mod 360 == reverse`` to within 1e-6.
    """
    g = build_graph(real_field_data)
    for u, v, data in g.edges(data=True):
        if data["type"] == EDGE_BLIND:
            # Blind edges are intentionally one-way.
            continue
        assert g.has_edge(v, u), f"missing reverse edge for {u}->{v}"
        forward = data["heading_deg"]
        reverse = g.edges[v, u]["heading_deg"]
        # Compute the wrapped delta and compare to 180 deg.
        delta = (forward - reverse) % 360.0
        assert delta == pytest.approx(180.0, abs=1e-6)


def test_obstacle_inflation_drops_clipped_edge() -> None:
    """TC6: an obstacle covering segment A-B's midpoint drops both A<->B edges.

    A wall at ``(20, -5, 40, 5)`` plus the default 8 cm robot radius
    inflates to a polygon that strictly intersects the y=0 line between
    ``A=(0,0)`` and ``B=(60,0)``, so both directed edges must be absent
    in the resulting graph.
    """
    fd = _clone_field_data()
    fd.OBSTACLES = [(20.0, -5.0, 40.0, 5.0, "wall")]

    # We don't care about ancillary "isolated node" warnings here.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)

    assert not g.has_edge("A", "B")
    assert not g.has_edge("B", "A")


def test_disconnected_goal_logs_warning() -> None:
    """TC14: walls between ``_proj_G`` and ``G`` produce a warning, not a crash.

    The wall covers the blind linestring from ``_proj_G=(60,30)`` to
    ``G=(90,30)``. After inflation the blind edge is dropped and
    ``build_graph`` emits a UserWarning naming the goal. The graph still
    builds; it just has no incoming edge for ``G``.
    """
    fd = _clone_field_data()
    # Box centred on y=30, well within the radius-inflated band.
    fd.OBSTACLES = [(70.0, 25.0, 80.0, 35.0, "wall")]

    with pytest.warns(UserWarning, match=r"Goal 'G'"):
        g = build_graph(fd)

    assert g.in_degree("G") == 0
    assert g.out_degree("G") == 0


# -- Extra structural guardrails ---------------------------------------------


def test_blind_edge_is_unidirectional() -> None:
    """The goal blind edge is one-way: ``_proj_G -> G`` only.

    Blind moves walk *off* the line; v1 has no machinery to walk back, so
    the reverse edge must not exist.
    """
    g = build_graph(real_field_data)
    assert g.has_edge("_proj_G", "G")
    assert not g.has_edge("G", "_proj_G")


def test_blind_edge_type() -> None:
    """The goal edge ``_proj_G -> G`` is tagged :data:`EDGE_BLIND`."""
    g = build_graph(real_field_data)
    assert g.edges["_proj_G", "G"]["type"] == EDGE_BLIND


def test_line_edge_type() -> None:
    """Line edges are LINE; edges incident to a projection are PROJECTION."""
    g = build_graph(real_field_data)
    # A->B has no projection on it, so it's a plain LINE edge.
    assert g.edges["A", "B"]["type"] == EDGE_LINE
    # B->_proj_G touches the projection node, so it's a PROJECTION edge.
    assert g.edges["B", "_proj_G"]["type"] == EDGE_PROJECTION


def test_heading_correctness_for_known_segment() -> None:
    """East/north edges have headings 0 deg / 90 deg respectively."""
    g = build_graph(real_field_data)
    # A=(0,0) -> B=(60,0): pointing East, heading == 0.
    assert g.edges["A", "B"]["heading_deg"] == pytest.approx(0.0, abs=1e-6)
    # Reverse: pointing West, heading == 180.
    assert g.edges["B", "A"]["heading_deg"] == pytest.approx(180.0, abs=1e-6)
    # A=(0,0) -> D=(0,60): pointing North, heading == 90.
    assert g.edges["A", "D"]["heading_deg"] == pytest.approx(90.0, abs=1e-6)
