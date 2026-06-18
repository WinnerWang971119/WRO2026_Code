"""Tests for tools.path_planner.graph_builder (PR3 schema).

PR3 graph: LINE_SEGMENTS and BLIND_EDGES, both bidirectional.
"""

from __future__ import annotations

import copy
import warnings
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner.tests.fixtures import toy_field_data as real_field_data
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    EDGE_PROJECTION,
    _reverse_heading,
    build_graph,
    directed_edge_heading,
)

_FIELD_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    "BLIND_DECAY_ENABLED",
    "BLIND_DECAY_K",
    "BLIND_DECAY_EXPONENT",
    "BLIND_DECAY_D_REF",
    "START_NODE",
    "START_HEADING_DEG",
    "FIELD_MM",
    "MAP_PNG",
)


def _clone_field_data() -> SimpleNamespace:
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    # EDGES is optional; clone it too when present so override tests can
    # mutate it without polluting other tests sharing real_field_data.
    if hasattr(real_field_data, "EDGES"):
        snapshot["EDGES"] = copy.deepcopy(real_field_data.EDGES)
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


# ---------------------------------------------------------------------------
# EDGES geometry override (graph_builder honors caller-supplied
# distance_cm / heading_deg instead of recomputing from node coords).
# Regression guard for Copilot PR #12 review comment #6.
# ---------------------------------------------------------------------------


def test_edges_distance_cm_override_is_honored() -> None:
    """An EDGES entry with explicit distance_cm wins over Euclidean geometry."""
    fd = _clone_field_data()
    # A=(10, 10) -> B=(200, 10): true Euclidean = 190.0 cm. Inject a
    # bidirectional rich-edge override claiming the strip is 99.0 cm.
    # Drop the LINE_SEGMENTS A-B pair so the EDGES entry is the sole
    # carrier of the A-B edge; otherwise the EDGES PROJECTION type would
    # conflict with the existing LINE pair (findings #38/#47) and the
    # validator would reject it before build_graph could emit the edge.
    fd.LINE_SEGMENTS = [e for e in fd.LINE_SEGMENTS if set(e) != {"A", "B"}]
    fd.EDGES = [
        {
            "type":         EDGE_PROJECTION,
            "u":            "A",
            "v":            "B",
            "distance_cm":  99.0,
            "heading_deg":  0.0,    # geometric heading also happens to be 0
        },
    ]
    # Replacing the cloned EDGES with a single entry leaves projection
    # node P1 unreferenced; the resulting "node has no incident edges"
    # warning is unrelated to the override path under test.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    assert g.edges["A", "B"]["distance_cm"] == pytest.approx(99.0, abs=1e-9)
    assert g.edges["B", "A"]["distance_cm"] == pytest.approx(99.0, abs=1e-9)


def test_edges_heading_deg_override_is_honored_with_reverse() -> None:
    """An EDGES entry with explicit heading_deg uses it for the forward edge
    and ``_reverse_heading(...)`` for the reverse edge."""
    fd = _clone_field_data()
    # A->B's geometric heading is 0.0 (due East). Inject an obviously wrong
    # 45.0 to prove the override path runs and the reverse is computed off
    # the override (45.0 + 180.0 == 225.0), not off the computed geometry.
    # Drop the LINE_SEGMENTS A-B pair so the EDGES entry is the sole carrier
    # of the A-B edge; otherwise the PROJECTION type conflicts with the
    # existing LINE pair (findings #38/#47) and the validator rejects it.
    fd.LINE_SEGMENTS = [e for e in fd.LINE_SEGMENTS if set(e) != {"A", "B"}]
    fd.EDGES = [
        {
            "type":         EDGE_PROJECTION,
            "u":            "A",
            "v":            "B",
            "distance_cm":  190.0,   # validator wants a positive number
            "heading_deg":  45.0,
        },
    ]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    assert g.edges["A", "B"]["heading_deg"] == pytest.approx(45.0, abs=1e-9)
    assert g.edges["B", "A"]["heading_deg"] == pytest.approx(
        _reverse_heading(45.0), abs=1e-9
    )
    assert g.edges["B", "A"]["heading_deg"] == pytest.approx(225.0, abs=1e-9)


def test_legacy_edges_without_options_fall_back_to_computed_geometry() -> None:
    """LINE_SEGMENTS / BLIND_EDGES entries (no options dict) still use the
    computed Euclidean distance + atan2 heading. Regression guard so the
    override path does not silently change legacy callers."""
    fd = _clone_field_data()
    # Drop EDGES to isolate the legacy path. LINE_SEGMENTS pair A-B remains.
    fd.EDGES = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    # A=(10, 10) -> B=(200, 10): Euclidean = 190.0 cm, heading = 0.0.
    assert g.edges["A", "B"]["distance_cm"] == pytest.approx(190.0, abs=1e-6)
    assert g.edges["A", "B"]["heading_deg"] == pytest.approx(0.0, abs=1e-6)
    assert g.edges["B", "A"]["distance_cm"] == pytest.approx(190.0, abs=1e-6)
    assert g.edges["B", "A"]["heading_deg"] == pytest.approx(180.0, abs=1e-6)


# ---------------------------------------------------------------------------
# directed_edge_heading helper (T3: per-turn pivot compensation)
# ---------------------------------------------------------------------------
#
# Node coordinates from the toy fixture:
#   A=(10,10)  B=(200,10)  C=(200,150)  D=(10,150)  E=(100,80)
#
# Expected headings (atan2(dy,dx) % 360, [0,360)):
#   A->B  dx=+190, dy=0    => 0.0 deg  (East)
#   B->A  dx=-190, dy=0    => 180.0 deg (West)
#   A->D  dx=0,   dy=+140  => 90.0 deg  (North)
#   D->A  dx=0,   dy=-140  => 270.0 deg (South)
#   B->C  dx=0,   dy=+140  => 90.0 deg  (North)
#   C->B  dx=0,   dy=-140  => 270.0 deg (South)


def test_directed_edge_heading_east_west() -> None:
    """A->B (due East) returns 0.0; reverse B->A returns 180.0."""
    g = build_graph(real_field_data)
    assert directed_edge_heading(g, "A", "B") == pytest.approx(0.0, abs=1e-6)
    assert directed_edge_heading(g, "B", "A") == pytest.approx(180.0, abs=1e-6)


def test_directed_edge_heading_north_south() -> None:
    """A->D (due North) returns 90.0; reverse D->A returns 270.0."""
    g = build_graph(real_field_data)
    assert directed_edge_heading(g, "A", "D") == pytest.approx(90.0, abs=1e-6)
    assert directed_edge_heading(g, "D", "A") == pytest.approx(270.0, abs=1e-6)


def test_directed_edge_heading_differs_from_reverse_by_180() -> None:
    """directed_edge_heading(g,u,v) and directed_edge_heading(g,v,u) differ by 180."""
    g = build_graph(real_field_data)
    for u, v in real_field_data.LINE_SEGMENTS:
        h_fwd = directed_edge_heading(g, u, v)
        h_rev = directed_edge_heading(g, v, u)
        assert (h_fwd - h_rev) % 360.0 == pytest.approx(180.0, abs=1e-6), (
            f"{u}->{v}: fwd={h_fwd}, rev={h_rev}"
        )


def test_directed_edge_heading_matches_edge_attribute() -> None:
    """directed_edge_heading reads the stored edge attribute, not recomputed geometry."""
    g = build_graph(real_field_data)
    for u, v, data in g.edges(data=True):
        assert directed_edge_heading(g, u, v) == pytest.approx(
            data["heading_deg"], abs=1e-9
        ), f"mismatch on {u}->{v}"


def test_directed_edge_heading_missing_edge_raises() -> None:
    """directed_edge_heading raises KeyError for a non-existent edge."""
    g = build_graph(real_field_data)
    with pytest.raises(KeyError):
        directed_edge_heading(g, "A", "G")  # no direct edge A->G in the toy fixture


def test_directed_edge_heading_respects_heading_override() -> None:
    """When an EDGES entry supplies an explicit heading_deg, directed_edge_heading
    returns the overridden value, not the atan2-computed geometry."""
    fd = _clone_field_data()
    # Drop the LINE_SEGMENTS A-B pair so the EDGES PROJECTION entry is sole carrier.
    fd.LINE_SEGMENTS = [e for e in fd.LINE_SEGMENTS if set(e) != {"A", "B"}]
    fd.EDGES = [
        {
            "type":        EDGE_PROJECTION,
            "u":           "A",
            "v":           "B",
            "distance_cm": 190.0,
            "heading_deg": 45.0,  # overridden to NE, not the real 0.0 East
        },
    ]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    assert directed_edge_heading(g, "A", "B") == pytest.approx(45.0, abs=1e-9)
    assert directed_edge_heading(g, "B", "A") == pytest.approx(225.0, abs=1e-9)
