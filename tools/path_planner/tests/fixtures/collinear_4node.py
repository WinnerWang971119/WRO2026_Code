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
``make_ctps_field_data()`` : returns a FULL field-data surface (adds a blind
                        spawn ``S`` feeding ``A``, plus the ``OBSTACLES`` /
                        ``FIELD_MM`` / ``BLIND_DECAY_*`` / ``PIVOT_TURNS`` /
                        ``RESET_NODES`` attributes) so the SAME collinear
                        A-B-C-D LINE geometry can be replayed through the CTPS
                        :func:`tools.path_planner.ctps.track_builder.build`
                        run-grouper (ADR-059, TC2 / AC1). The minimal
                        :func:`make_field_data` surface above cannot feed
                        ``build`` (it lacks those attributes AND its
                        ``START_NODE`` is the on-line ``A``, which
                        ``build_graph``'s ``validate`` rejects); this factory
                        keeps ``make_field_data`` untouched so the existing
                        sim-level pass-through tests stay byte-identical.
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
        BLIND_PROJECTION_AUTO_SEEK=True,
    )


# ---------------------------------------------------------------------------
# CTPS render factory (full field surface — ADR-059 run-grouper, TC2 / AC1)
# ---------------------------------------------------------------------------

# Blind dead-reckon spawn feeding node A. Placed one nominal hop WEST of A
# (heading 0, +x) so a ``goto("A")`` re-anchors the chassis exactly onto the
# line at A before the collinear A-B-C-D run begins.
_CTPS_SPAWN_NODE: str = "S"
_CTPS_SPAWN_XY: tuple = (-40.0, 0.0)


def make_ctps_field_data() -> SimpleNamespace:
    """Return a FULL field-data surface for the CTPS run-grouper render (TC2 / AC1).

    :func:`tools.path_planner.ctps.track_builder.build` needs the complete
    field-data attribute surface (``OBSTACLES`` / ``FIELD_MM`` /
    ``ROBOT_RADIUS_CM`` / ``BLIND_DECAY_*`` / ``NODE_TYPES`` / ``LINE_SEGMENTS`` /
    ``BLIND_EDGES`` / ``PIVOT_TURNS`` / ``RESET_NODES``) and calls
    ``build_graph`` -> ``validate``, which REJECTS an on-line ``START_NODE``. The
    minimal :func:`make_field_data` above exposes neither, so it cannot render
    through ``build``.

    This factory wraps the identical collinear A-B-C-D LINE geometry with a
    blind dead-reckon spawn ``S`` one nominal hop WEST of ``A`` (a
    ``BLIND_EDGES`` ``S -> A`` hop, matching the toy field's blind start). A
    mission that ``goto("A")`` then ``goto("D")`` re-anchors onto the line at
    ``A`` and then drives the whole B/C pass-through chain as ONE grouped run —
    exactly the collinear LINE chain AC1 / TC2 pins.

    The three A-B / B-C / C-D edges become ``LINE_SEGMENTS`` (all collinear on
    ``y = 0`` at heading 0), so ``B`` and ``C`` are collinear pass-through
    junctions. No pivots, no reset nodes, no obstacles — a clean straight line.

    :return: a field-data namespace ready for ``build`` (and ``build_graph`` /
        ``validate``), leaving :func:`make_field_data` byte-unchanged.
    """
    nodes = dict(NODES)
    nodes[_CTPS_SPAWN_NODE] = _CTPS_SPAWN_XY

    node_types = {name: "line" for name in NODES}
    node_types[_CTPS_SPAWN_NODE] = "blind"

    # The three collinear chain edges become LINE segments (undirected pairs).
    line_segments = [(u, v) for (u, v, *_rest) in EDGES]

    return SimpleNamespace(
        FIELD_MM=(3000.0, 2000.0),
        MAP_PNG="maps/map.png",
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        BLIND_PROJECTION_AUTO_SEEK=True,
        START_NODE=_CTPS_SPAWN_NODE,
        START_HEADING_DEG=0.0,
        NODES=nodes,
        NODE_TYPES=node_types,
        LINE_SEGMENTS=line_segments,
        BLIND_EDGES=[(_CTPS_SPAWN_NODE, START_NODE)],
        OBSTACLES=[],
        PIVOT_TURNS=[],
        RESET_NODES={},
    )


# ---------------------------------------------------------------------------
# CTPS render factory — SKEWED intermediate node (sub-tolerance alignment turn)
# ---------------------------------------------------------------------------

#: Lateral nudge (cm) applied to the intermediate node ``C`` so ``B->C`` and
#: ``C->D`` acquire a slight slope off the ``y = 0`` axis. Over the 50 cm edges
#: this yields per-node heading-alignment turns of a few hundredths of a degree —
#: comfortably BELOW ``NAV_TURN_TOLERANCE_DEG`` (0.5°), so the sim still passes
#: through ``B`` and ``C`` (a ``line_pass``, NOT a braked stop) and merely emits a
#: tiny ``turn_relative`` to align onto each next edge's exact heading. Real
#: fields are never perfectly axis-aligned, so this mirrors what
#: ``field_data.json`` actually produces; the perfectly-collinear
#: :func:`make_ctps_field_data` above cannot surface it.
_CTPS_SKEW_CM: float = 0.05


def make_ctps_field_data_skewed() -> SimpleNamespace:
    """Like :func:`make_ctps_field_data`, but with node ``C`` nudged off-axis.

    The nudge (:data:`_CTPS_SKEW_CM`) makes the two intermediate pass-throughs
    ``B`` and ``C`` emit SUB-TOLERANCE heading-alignment ``turn_relative``
    records — the exact stream a real field produces at a near-collinear
    pass-through. It is the regression fixture for the CTPS run-grouper bug where
    a hard ``1e-9`` heading threshold treated such an alignment turn as a real
    turn and split the whole-run trapezoid into two ramps with a dip at the
    pass-through node. The grouper must instead gate run-breaks on
    ``NAV_TURN_TOLERANCE_DEG`` (matching the sim's own pass-through decision), so
    this skewed chain still renders as ONE trapezoid.

    :return: the :func:`make_ctps_field_data` surface with ``NODES["C"]`` moved
        laterally by :data:`_CTPS_SKEW_CM`.
    """
    field_data = make_ctps_field_data()
    nodes = dict(field_data.NODES)
    cx, cy = nodes["C"]
    nodes["C"] = (cx, cy + _CTPS_SKEW_CM)
    field_data.NODES = nodes
    return field_data
