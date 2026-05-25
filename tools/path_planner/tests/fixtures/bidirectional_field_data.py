"""Bidirectional line-following fixtures for sim end-to-end tests.

Exposes three purpose-built synthetic field layouts for the T14 test suite
(``test_runtime_sim.py``).  Every fixture is hand-rolled — no validator /
builder pipeline — so it is stable across schema changes and tests remain
independent of the live field data.

Each public factory returns a ``SimpleNamespace`` with the minimal attribute
surface consumed by :class:`tools.path_planner.sim.runtime.PathTrackerSim`
(``NODES``, ``START_NODE``, ``START_HEADING_DEG``) plus a ``build_graph()``
and ``build_next_hop()`` helper identical in shape to ``collinear_4node``.

Fixtures
--------
``make_collinear_field()``
    4-node straight line (A→B→C→D) at heading 0°.  Identical topology to
    ``collinear_4node`` but bundled here so TC9-class pass-through-with-REAR
    tests (T15) can share it without a cross-fixture import.

``make_rear_edge_field()``
    3-node chain A→B→C at heading 0° where the A→B edge carries a
    ``direction_override = DirectionMode.FORCE_REVERSE`` attribute.  Used by
    TC7 to verify that the per-edge override wins even when the builder says
    FRONT.

``make_mixed_turn_field()``
    4-node chain A→B→C→D with edge headings 0° → 135° → 270°.  The A→B hop
    is straight (auto FRONT); the B→C hop turns 135° (exceeds the
    ``BIDIRECTIONAL_TURN_THRESHOLD_DEG`` default of 90°, so auto resolves
    REAR — body heading becomes 315°); the C→D hop at 270° from body 315°
    has |delta| = 45° ≤ 90°, so auto resolves FRONT.  Used by TC5 / TC6 to
    exercise per-hop auto-resolution and forced-route overrides on a
    multi-hop path; the FRONT auto-resolution on the last hop makes TC5's
    ``arrival_direction=REAR`` override load-bearing (without it, the hop
    would auto-resolve FRONT).
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import networkx as nx

from tools.path_planner.graph_builder import EDGE_LINE
from tools.path_planner.pipeline_config import DirectionMode

# ---------------------------------------------------------------------------
# Internal graph-building helpers (mirror collinear_4node patterns exactly)
# ---------------------------------------------------------------------------


def _build_graph_from_edges(nodes: dict, edges: list) -> nx.DiGraph:
    """Return a ``DiGraph`` with forward + reverse edges from an edge list.

    :param nodes: dict of node-name → ``(x_cm, y_cm)``.
    :param edges: list of ``(u, v, type, distance_cm, heading_deg[, attrs])``
        tuples.  The optional sixth element is a dict of extra edge attributes
        (e.g. ``{"direction_override": ...}``).  Reverse edges never inherit
        directional overrides.
    """
    g = nx.DiGraph()
    for name, (x, y) in nodes.items():
        g.add_node(name, x_cm=float(x), y_cm=float(y), type="line")
    for entry in edges:
        u, v, edge_type, dist, hdg = entry[:5]
        extra: dict = entry[5] if len(entry) > 5 else {}
        fwd_attrs = dict(
            type=edge_type,
            distance_cm=float(dist),
            heading_deg=float(hdg),
            **extra,
        )
        rev_attrs = dict(
            type=edge_type,
            distance_cm=float(dist),
            heading_deg=(float(hdg) + 180.0) % 360.0,
            # Directional overrides are NOT propagated to the reverse edge.
        )
        g.add_edge(u, v, **fwd_attrs)
        g.add_edge(v, u, **rev_attrs)
    return g


def _build_chain_next_hop(nodes_ordered: list, fwd: dict, bwd: dict) -> dict:
    """Return a hand-rolled next-hop table for a linear node chain.

    :param nodes_ordered: list of node names in traversal order.
    :param fwd: dict mapping each node to its forward successor.
    :param bwd: dict mapping each node to its backward predecessor.
    """
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
                row[tgt] = fwd.get(src)
            else:
                row[tgt] = bwd.get(src)
        table[src] = row
    return table


# ---------------------------------------------------------------------------
# Fixture 1 — collinear 4-node (shared with TC9-class pass-through+REAR tests)
# ---------------------------------------------------------------------------
#
#   A ──50cm── B ──50cm── C ──50cm── D
#   heading 0° throughout (along +x axis)

_COLLINEAR_NODES: dict = {
    "A": (0.0, 0.0),
    "B": (50.0, 0.0),
    "C": (100.0, 0.0),
    "D": (150.0, 0.0),
}

_COLLINEAR_EDGES: list = [
    ("A", "B", EDGE_LINE, 50.0, 0.0),
    ("B", "C", EDGE_LINE, 50.0, 0.0),
    ("C", "D", EDGE_LINE, 50.0, 0.0),
]

_COLLINEAR_START_NODE: str = "A"
_COLLINEAR_START_HEADING: float = 0.0


def make_collinear_field() -> SimpleNamespace:
    """Return the simple 4-node collinear field (A→B→C→D, heading 0°).

    The ``SimpleNamespace`` carries convenience factory methods
    ``build_graph()`` and ``build_next_hop()`` as attributes so callers
    can wire a :class:`PathTrackerSim` without importing the private
    helpers.
    """
    ns = SimpleNamespace(
        NODES=dict(_COLLINEAR_NODES),
        START_NODE=_COLLINEAR_START_NODE,
        START_HEADING_DEG=_COLLINEAR_START_HEADING,
    )
    ns.build_graph = lambda: _build_graph_from_edges(_COLLINEAR_NODES, _COLLINEAR_EDGES)
    ns.build_next_hop = lambda: _build_chain_next_hop(
        ["A", "B", "C", "D"],
        {"A": "B", "B": "C", "C": "D"},
        {"D": "C", "C": "B", "B": "A"},
    )
    return ns


# ---------------------------------------------------------------------------
# Fixture 2 — rear-edge 3-node (A→B forced REAR, B→C forward)
# ---------------------------------------------------------------------------
#
#   A ──50cm── B ──50cm── C
#   A→B: heading 0°, direction_override = FORCE_REVERSE
#   B→C: heading 0°, no override (auto)
#
# The A→B edge carries ``direction_override = DirectionMode.FORCE_REVERSE``
# (integer 2, the wire encoding) as an explicit graph attribute.  This
# fixture targets TC7: "per-edge direction='rear' on (A,B), builder says
# FRONT → edge override wins; that hop is REAR despite operator preference".

_REAR_EDGE_NODES: dict = {
    "A": (0.0, 0.0),
    "B": (50.0, 0.0),
    "C": (100.0, 0.0),
}

_REAR_EDGE_EDGES: list = [
    # A→B forced REAR via direction_override (integer 2 = DirectionMode.FORCE_REVERSE).
    ("A", "B", EDGE_LINE, 50.0, 0.0, {"direction_override": int(DirectionMode.FORCE_REVERSE)}),
    # B→C no override (auto).
    ("B", "C", EDGE_LINE, 50.0, 0.0),
]

_REAR_EDGE_START_NODE: str = "A"
_REAR_EDGE_START_HEADING: float = 0.0


def make_rear_edge_field() -> SimpleNamespace:
    """Return the 3-node field where A→B is forced REAR by a per-edge override.

    The ``SimpleNamespace`` carries ``build_graph()`` and ``build_next_hop()``
    convenience factory attributes.
    """
    ns = SimpleNamespace(
        NODES=dict(_REAR_EDGE_NODES),
        START_NODE=_REAR_EDGE_START_NODE,
        START_HEADING_DEG=_REAR_EDGE_START_HEADING,
    )
    ns.build_graph = lambda: _build_graph_from_edges(_REAR_EDGE_NODES, _REAR_EDGE_EDGES)
    ns.build_next_hop = lambda: _build_chain_next_hop(
        ["A", "B", "C"],
        {"A": "B", "B": "C"},
        {"C": "B", "B": "A"},
    )
    return ns


# ---------------------------------------------------------------------------
# Fixture 3 — mixed-turn 4-node (FRONT / REAR / FRONT auto-resolution)
# ---------------------------------------------------------------------------
#
#   A ──50cm── B ──50cm── C ──50cm── D
#   Edge headings: 0°         135°       270°
#
# Auto-resolution per hop (start heading 0°):
#   A→B (edge 0°):   delta = 0°   → FRONT; body stays at 0°.
#   B→C (edge 135°): chassis at 0°, delta = 135° > 90° → REAR; body becomes 315°.
#   C→D (edge 270°): chassis at 315°, delta = wrap(270-315) = -45°, |delta|≤90° → FRONT.
#
# The FRONT auto-resolution on the last hop is intentional: it makes TC5's
# arrival_direction=REAR override actually change behavior (without it the
# hop would auto-resolve FRONT, so an ignored arrival_direction would let
# the assertion fail).
#
# TC5 tests setArrivalDirection(REAR) forces only the last hop REAR.
# TC6 tests setRouteDirection(REAR) forces ALL hops REAR.
#
# Node coordinates are placed geometrically along the given headings
# so x_cm/y_cm are consistent with the declared heading values.

_BC_HEADING = 135.0
_BC_DX = math.cos(math.radians(_BC_HEADING)) * 50.0
_BC_DY = math.sin(math.radians(_BC_HEADING)) * 50.0

_CD_HEADING = 270.0
_CD_DX = math.cos(math.radians(_CD_HEADING)) * 50.0
_CD_DY = math.sin(math.radians(_CD_HEADING)) * 50.0

_MIXED_TURN_NODES: dict = {
    "A": (0.0, 0.0),
    "B": (50.0, 0.0),
    "C": (50.0 + _BC_DX, 0.0 + _BC_DY),
    "D": (50.0 + _BC_DX + _CD_DX, 0.0 + _BC_DY + _CD_DY),
}

_MIXED_TURN_EDGES: list = [
    ("A", "B", EDGE_LINE, 50.0, 0.0),
    ("B", "C", EDGE_LINE, 50.0, _BC_HEADING),
    ("C", "D", EDGE_LINE, 50.0, _CD_HEADING),
]

_MIXED_TURN_START_NODE: str = "A"
_MIXED_TURN_START_HEADING: float = 0.0


def make_mixed_turn_field() -> SimpleNamespace:
    """Return a 4-node field with edge headings 0° / 135° / 270°.

    With ``BIDIRECTIONAL_TURN_THRESHOLD_DEG = 90°`` and a 2-board chassis
    (``LINE_BOARD_COUNT = 2``), the auto algorithm produces:

    * A→B: delta 0° from chassis 0° → FRONT; body stays at 0°.
    * B→C: delta 135° from chassis 0° → REAR; body becomes 315°.
    * C→D: delta -45° from chassis 315° → FRONT; body stays at 270°.

    The last-hop FRONT is intentional so that TC5's ``arrival_direction=REAR``
    override is load-bearing (without it the hop would auto-resolve FRONT
    and the REAR-primitive count would drop).

    The ``SimpleNamespace`` carries ``build_graph()`` and ``build_next_hop()``
    convenience factory attributes.
    """
    ns = SimpleNamespace(
        NODES=dict(_MIXED_TURN_NODES),
        START_NODE=_MIXED_TURN_START_NODE,
        START_HEADING_DEG=_MIXED_TURN_START_HEADING,
    )
    ns.build_graph = lambda: _build_graph_from_edges(_MIXED_TURN_NODES, _MIXED_TURN_EDGES)
    ns.build_next_hop = lambda: _build_chain_next_hop(
        ["A", "B", "C", "D"],
        {"A": "B", "B": "C", "C": "D"},
        {"D": "C", "C": "B", "B": "A"},
    )
    return ns
