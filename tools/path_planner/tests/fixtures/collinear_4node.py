"""Collinear 4-node LINE fixture for pass-through intersection tests.

Exposes a minimal straight-line chain A → B → C → D where all three
edges are EDGE_LINE at the same heading (0°, i.e. along +x), so every
intermediate node (B and C) is collinear within ``NAV_TURN_TOLERANCE_DEG``.

The fixture is intentionally hand-rolled (no validator / builder pipeline)
so it stays independent of field_data schema rules and is stable across
editor saves.

Public surface
--------------
``NODES``          : dict of node name → ``(x_cm, y_cm)`` tuple.
``EDGES``          : list of ``(u, v, type, distance_cm, heading_deg)`` tuples.
``START_NODE``     : the starting node name ("A").
``START_HEADING_DEG`` : the starting heading in degrees (0.0).
``build_graph()``  : returns an ``nx.DiGraph`` with the fixture's edges.
``build_next_hop()`` : returns a hand-rolled ``next_hop`` dict matching the
                       chain topology (no Dijkstra dependency).
``make_field_data()`` : returns a ``SimpleNamespace`` usable as a ``field_data``
                        argument to ``PathTrackerSim`` and ``VirtualChassis``.
"""

from __future__ import annotations

from types import SimpleNamespace

import networkx as nx

from tools.path_planner.graph_builder import EDGE_LINE

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

# A ──50cm──► B ──50cm──► C ──50cm──► D
# All headings 0° (along +x axis).

NODES: dict = {
    "A": (0.0, 0.0),
    "B": (50.0, 0.0),
    "C": (100.0, 0.0),
    "D": (150.0, 0.0),
}

# Each tuple: (u, v, edge_type, distance_cm, heading_deg)
EDGES: list = [
    ("A", "B", EDGE_LINE, 50.0, 0.0),
    ("B", "C", EDGE_LINE, 50.0, 0.0),
    ("C", "D", EDGE_LINE, 50.0, 0.0),
]

START_NODE: str = "A"
START_HEADING_DEG: float = 0.0


# ---------------------------------------------------------------------------
# Graph builder
# ---------------------------------------------------------------------------


def build_graph() -> nx.DiGraph:
    """Return a directed graph with both forward and reverse edges for every hop.

    Reverse edges are added so that Dijkstra / next-hop tables (if used)
    can always find return paths; they do not affect forward-only tests.
    """
    g = nx.DiGraph()
    for name, (x, y) in NODES.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y), type="line")
    for u, v, edge_type, dist, hdg in EDGES:
        g.add_edge(u, v, type=edge_type, distance_cm=float(dist), heading_deg=float(hdg))
        reverse_hdg = (float(hdg) + 180.0) % 360.0
        g.add_edge(v, u, type=edge_type, distance_cm=float(dist), heading_deg=reverse_hdg)
    return g


# ---------------------------------------------------------------------------
# Next-hop table
# ---------------------------------------------------------------------------


def build_next_hop() -> dict:
    """Return a hand-rolled ``next_hop`` dict for the A→B→C→D chain.

    The table covers all source/target combinations for the four nodes.
    Unreachable entries (where there is no path in either direction) are
    left absent from the inner dict; callers that need a ``None``-filled
    dense table should use
    :func:`tools.path_planner.planner.compute_all_pairs_next_hop`
    on :func:`build_graph` instead.

    For forward traversal (left to right) the first hop is always the
    immediate successor.  For backward traversal (right to left) the
    first hop is the immediate predecessor.  Same-node entries map to
    self (convention used by ``PathTrackerSim.go_to``).
    """
    nodes_ordered = ["A", "B", "C", "D"]
    # Build forward and backward adjacency from the edge list.
    fwd = {"A": "B", "B": "C", "C": "D"}   # next node in +x direction
    bwd = {"D": "C", "C": "B", "B": "A"}   # next node in -x direction

    table: dict = {}
    for src in nodes_ordered:
        row: dict = {}
        for tgt in nodes_ordered:
            if src == tgt:
                row[tgt] = src
                continue
            src_idx = nodes_ordered.index(src)
            tgt_idx = nodes_ordered.index(tgt)
            if tgt_idx > src_idx:
                # Walk forward: first hop is fwd[src].
                row[tgt] = fwd[src]
            else:
                # Walk backward: first hop is bwd[src].
                row[tgt] = bwd[src]
        table[src] = row
    return table


# ---------------------------------------------------------------------------
# Convenience factory
# ---------------------------------------------------------------------------


def make_field_data() -> SimpleNamespace:
    """Return a ``SimpleNamespace`` usable as a ``field_data`` argument.

    :class:`tools.path_planner.sim.runtime.PathTrackerSim` and
    :class:`tools.path_planner.sim.chassis.VirtualChassis` consume the
    ``NODES``, ``START_NODE``, and ``START_HEADING_DEG`` attributes from a
    field-data object — this factory exposes exactly that surface.
    """
    return SimpleNamespace(
        NODES=dict(NODES),
        START_NODE=START_NODE,
        START_HEADING_DEG=START_HEADING_DEG,
    )
