"""All-pairs shortest-path planner for the WRO 2026 field graph.

This module consumes the directed graph emitted by
:mod:`tools.path_planner.graph_builder` and produces the next-hop routing
table that PR 2's runtime walks at every line node:

    NEXT_HOP[from_name][to_name] -> next_node_name | None

The runtime (PR 2) only ever needs to know "given that I am standing at
``from_name`` and want to end up at ``to_name``, which neighbour do I
step to *now*?". Storing the full path per pair would waste flash; the
next-hop table compresses an all-pairs solution into ``N x N`` strings.

Algorithm: repeated ``nx.single_source_dijkstra`` (one call per source).
The plan explicitly chose this over Floyd-Warshall because networkx
ships an optimised heap-based Dijkstra and the toy / real graphs both
have <50 nodes - the constant factor matters far less than readability.

Cost model
----------

Edge cost = ``edge['distance_cm'] * cost_multiplier`` where
``cost_multiplier`` is :data:`tools.path_planner.field_data.COST_BLIND_MULTIPLIER`
(passed in as an argument here for testability) when the edge type is
:data:`tools.path_planner.graph_builder.EDGE_BLIND`, and ``1.0`` otherwise.
Penalising blind edges keeps the planner on the line whenever a line
detour is reasonable - the line follower is far more reliable than
open-loop encoder moves.

The raw ``distance_cm`` attribute is left untouched on the input graph;
this module operates on a *copy* with an extra ``cost`` attribute, so
callers can re-use the original graph for visualisation or heading
propagation downstream (T6).

Sentinels
---------

* ``next_hop[source][source] == source`` (self-loop; TC8). The C++ side
  treats reaching one's own destination as a no-op.
* ``next_hop[source][target] is None`` when ``target`` is unreachable
  from ``source``. PR 1 keeps this as Python ``None`` for clarity; T7
  (``header_writer``) translates it to the byte ``0xFF`` when emitting
  ``routes.cpp``.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import networkx as nx

from tools.path_planner.graph_builder import EDGE_BLIND

# Public type alias: outer key = source node, inner key = target node,
# value = next node on the shortest path (or None if unreachable).
NextHopTable = Dict[str, Dict[str, Optional[str]]]


# -- internal helpers ---------------------------------------------------------


def _weighted_copy(graph: nx.DiGraph, cost_blind_multiplier: float) -> nx.DiGraph:
    """Return a copy of ``graph`` with a ``cost`` attribute on every edge.

    The copy preserves all original edge attributes (``type``,
    ``distance_cm``, ``heading_deg``) so downstream code that consumes
    the weighted graph still has the geometric information available.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Every edge
        must carry ``type`` and ``distance_cm``.
    :param cost_blind_multiplier: multiplier applied to the
        ``distance_cm`` of edges whose ``type`` equals
        :data:`tools.path_planner.graph_builder.EDGE_BLIND`. Must be
        positive; a multiplier of ``1.0`` disables the penalty.
    :return: new :class:`networkx.DiGraph` carrying the same nodes and
        edges as ``graph``, plus a ``cost`` attribute per edge.
    """
    weighted = nx.DiGraph()
    # Copy nodes with their attributes so node-only graphs (e.g. the
    # fully-isolated-G fixture in the unreachable test) still iterate.
    for node, attrs in graph.nodes(data=True):
        weighted.add_node(node, **attrs)

    for u, v, data in graph.edges(data=True):
        distance_cm = float(data["distance_cm"])
        edge_type = data["type"]
        multiplier = cost_blind_multiplier if edge_type == EDGE_BLIND else 1.0
        cost = distance_cm * multiplier
        # Re-emit every original attribute alongside the new cost field.
        weighted.add_edge(u, v, cost=cost, **data)

    return weighted


# -- public API ---------------------------------------------------------------


def compute_all_pairs_next_hop(
    graph: nx.DiGraph,
    cost_blind_multiplier: float,
) -> NextHopTable:
    """Return the all-pairs next-hop routing table for ``graph``.

    For every ordered pair ``(source, target)`` of nodes:

    * ``source == target`` -> ``next_hop[source][target] == source``
      (self-loop sentinel; TC8).
    * ``target`` unreachable from ``source`` -> ``None`` (T7 emits this
      as ``0xFF``).
    * Otherwise -> the second node on the shortest ``source -> target``
      path returned by :func:`networkx.single_source_dijkstra`. Walking
      the table from ``source`` by repeatedly indexing on the running
      next-hop reconstructs the optimal path.

    Edge cost = ``distance_cm * (cost_blind_multiplier if type == EDGE_BLIND
    else 1.0)``. The function does not mutate ``graph``.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Every edge
        must carry ``type`` and ``distance_cm``.
    :param cost_blind_multiplier: positive multiplier applied to
        :data:`tools.path_planner.graph_builder.EDGE_BLIND` edges.
    :return: nested dict ``{source: {target: next_hop | None}}``.
        Outer dict has every node as a key; each inner dict has every
        node as a key (no missing entries).
    """
    weighted = _weighted_copy(graph, cost_blind_multiplier)
    nodes = list(graph.nodes())

    next_hop: NextHopTable = {}
    for source in nodes:
        # Default every target to None so unreachable pairs are populated
        # without an explicit conditional below. The self-loop overrides
        # to the source itself.
        row: Dict[str, Optional[str]] = {target: None for target in nodes}
        row[source] = source

        # single_source_dijkstra returns (lengths, paths) where each
        # paths[target] is the list of node names from source to target,
        # inclusive on both ends. Source's own path is [source].
        _lengths, paths = nx.single_source_dijkstra(weighted, source, weight="cost")
        for target, path in paths.items():
            if target == source:
                # Already set to source above; explicit guard avoids
                # the IndexError that would come from path[1] when
                # path == [source] (length 1).
                continue
            # path[0] == source, path[-1] == target, len(path) >= 2 here.
            row[target] = path[1]

        next_hop[source] = row

    return next_hop


def compute_path_cost(
    graph: nx.DiGraph,
    path: List[str],
    cost_blind_multiplier: float,
) -> float:
    """Return the total cost of walking ``path`` through ``graph``.

    Uses the same edge-cost formula as
    :func:`compute_all_pairs_next_hop`, so tests can confirm a Dijkstra
    decision matches a hand-computed cost. A single-node path (or empty
    path) has cost ``0.0`` because no edges are traversed.

    :param graph: directed graph carrying ``type`` and ``distance_cm``
        on every edge.
    :param path: ordered list of node names. Consecutive pairs must be
        connected by a directed edge in ``graph``.
    :param cost_blind_multiplier: positive multiplier applied to
        :data:`tools.path_planner.graph_builder.EDGE_BLIND` edges.
    :return: summed edge cost as a float.
    :raises KeyError: if any consecutive ``(path[i], path[i+1])`` pair
        is not an edge in ``graph``.
    """
    if len(path) < 2:
        return 0.0

    total = 0.0
    for u, v in zip(path[:-1], path[1:]):
        # graph.edges[u, v] raises KeyError on a missing edge, which is
        # exactly the contract we promise.
        data = graph.edges[u, v]
        distance_cm = float(data["distance_cm"])
        multiplier = cost_blind_multiplier if data["type"] == EDGE_BLIND else 1.0
        total += distance_cm * multiplier

    return total
