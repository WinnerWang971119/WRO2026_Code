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

The weighted copy used by Dijkstra carries an extra ``cost`` attribute;
the underlying geometry (``distance_cm`` / ``heading_deg``) is taken
from the *input* graph AFTER pivot compensation has been applied to it
in place, so the cost the planner optimises and the geometry the
emitter consumes describe the same compensated edges.

Single-wheel pivot compensation
-------------------------------

Edges in the graph may carry a ``pivot`` attribute (the integer value of
a :class:`tools.path_planner.graph_builder.Pivot` member). When set to
``Pivot.LEFT`` or ``Pivot.RIGHT``, the turn entering that edge is
executed as a single-wheel pivot rather than a counter-rotation. The
chassis centre traces an arc instead of staying put, so it lands offset
from where a counter-rotation would have left it. The planner
compensates for that displacement by shortening the FOLLOWING edge's
``distance_cm`` (see :func:`compute_pivot_compensation`). The
compensation is applied to the input graph **in place** by
:func:`compute_all_pairs_next_hop`, so the shortened distance reaches
both the Dijkstra cost and the routes.cpp emitter that consumes the
same graph object downstream.

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

import math
from typing import Dict, List, Optional, Tuple

import networkx as nx

from tools.path_planner.graph_builder import EDGE_BLIND, Pivot

# Public type alias: outer key = source node, inner key = target node,
# value = next node on the shortest path (or None if unreachable).
NextHopTable = Dict[str, Dict[str, Optional[str]]]


# -- physical constants -------------------------------------------------------

# Chassis track width (wheel-to-wheel) in centimetres. This MUST mirror
# ``Config::TRACK_MM`` in ``WRO2026/Config.h`` (currently 150 mm = 15 cm).
# Kept as a Python-side mirror so :mod:`tools.path_planner.field_data` does
# not have to take a dependency on the C++ headers. Update both sides
# together when the real chassis is measured.
TRACK_CM: float = 15.0


# -- pivot-compensation helpers ----------------------------------------------


def compute_pivot_compensation(
    turn_deg: float,
    track_cm: float,
    next_edge_dist: float,
    next_edge_heading: float,
) -> Tuple[float, float]:
    """Adjust next-edge geometry to compensate for a single-wheel pivot turn.

    For a single-wheel pivot through ``turn_deg`` degrees around one wheel,
    the chassis centre moves along a chord of length
    ``2 * (track_cm / 2) * sin(|theta| / 2)`` (i.e. an arc of radius
    ``track_cm / 2``). The chord direction depends on pivot side and turn
    direction.

    For now we treat the chord as a translation along the bisector of the
    turn (a small-angle approximation that is exact for the chord
    *magnitude* and a reasonable estimate for the heading-projection at
    modest turn angles). The forward component of the chord -- the piece
    that overshoots the next node along the next-edge heading -- is
    ``chord_cm * sin(|theta| / 2)``, which we subtract from
    ``next_edge_dist``. ``next_edge_heading`` is unchanged: the planner
    emits absolute world headings, so the next edge's heading direction
    is independent of how the chassis arrived at its starting point.

    The math may need refining once we have real chassis measurements;
    keeping the helper pure (no graph dependency) makes it cheap to swap
    out later.

    :param turn_deg: signed turn magnitude (degrees, CCW positive). Only
        ``|turn_deg|`` enters the chord calculation, so the sign is
        accepted but unused -- callers may pass a signed delta directly.
    :param track_cm: chassis track width (wheel-to-wheel) in centimetres.
        Must be non-negative. ``0.0`` is allowed and yields zero
        compensation.
    :param next_edge_dist: planner's intended distance to the next node
        (cm). The compensated distance is clamped at ``0.0`` -- a turn so
        large it would erase the next edge entirely yields a zero-length
        edge rather than a negative one.
    :param next_edge_heading: planner's intended heading at the next
        edge (degrees, world frame). Returned unchanged.
    :return: ``(adjusted_distance_cm, adjusted_heading_deg)`` tuple.
        Currently the heading is returned verbatim; the distance is
        ``max(0.0, next_edge_dist - chord_cm * sin(|theta| / 2))``.
    """
    abs_theta_rad = math.radians(abs(turn_deg))
    half_theta = abs_theta_rad / 2.0
    chord_cm = 2.0 * (track_cm / 2.0) * math.sin(half_theta)
    # Forward component of the chord along the post-turn heading.
    # Geometrically this is the projection of the chord onto the
    # next-edge direction; for a chord oriented along the bisector of
    # the turn, that projection simplifies to chord_cm * sin(|theta| / 2).
    forward_component = chord_cm * math.sin(half_theta)
    adjusted_dist = max(0.0, next_edge_dist - forward_component)
    return (adjusted_dist, next_edge_heading)


def apply_pivot_compensations(graph: nx.DiGraph, track_cm: float) -> None:
    """Walk ``graph`` and shorten every edge that follows a pivot turn.

    For every edge ``(src, dst)`` in ``graph`` whose ``pivot`` attribute
    is set to :attr:`Pivot.LEFT` or :attr:`Pivot.RIGHT` (compared by
    integer value, since :func:`graph_builder._extract_directional_attrs`
    stores the int rather than the IntEnum), this function locates the
    *single* out-edge from ``dst`` and rewrites its ``distance_cm`` to
    the value returned by :func:`compute_pivot_compensation`. The
    ``heading_deg`` field is also overwritten with the (currently
    unchanged) value from the helper, so a future refinement of the
    heading math automatically flows through.

    The turn magnitude is computed as the heading delta between the
    pivot-tagged edge and the following edge (normalised to
    ``[-180, 180]`` degrees).

    .. note::

        This conservative implementation refuses to compensate when the
        landing node has anything other than exactly one forward
        out-edge. Out-edge choice is only known once Dijkstra has picked
        a path, but :func:`apply_pivot_compensations` runs *before*
        Dijkstra so it cannot know which successor will fire -- applying
        the adjustment to all successors would silently overcorrect every
        branch that is not the one we end up taking. The safe scope cut
        is to require pivot destinations to have exactly one forward
        out-edge so the choice is unambiguous. The field-data author can
        structure the graph accordingly. A future per-path pivot-aware
        planner could relax this restriction.

        For the production pipeline the validator enforces this
        precondition up front (finding #36): a pivot landing with the
        wrong out-degree fails as a :class:`FieldDataError` at validation
        time. This function therefore does NOT raise on a mismatch -- it
        skips the offending pivot edge (leaving its following edge
        uncompensated). The skip is a defensive guard for direct /
        hand-built callers that assemble a graph without running
        :func:`tools.path_planner.validator.validate`; it keeps
        ``forward_out_edges[0]`` safe from an IndexError and never
        mis-compensates a multi-successor landing by picking an arbitrary
        branch.

        The back-edge from ``dst`` to ``src`` (always present when the
        edge was emitted by :func:`build_graph`'s bidirectional pair) is
        excluded from the out-edge count and from the compensation
        target -- the chassis is never going to immediately reverse the
        pivot it just executed.

    :param graph: a directed graph whose following-edge ``distance_cm`` /
        ``heading_deg`` attributes are mutated **in place**. Under
        Decision B (finding #42) :func:`compute_all_pairs_next_hop` passes
        its own input graph here on purpose, so the compensated geometry
        reaches both the Dijkstra cost and the routes.cpp emitter. Callers
        that must preserve the original geometry should pass a copy.
    :param track_cm: chassis track width in centimetres. Forwarded to
        :func:`compute_pivot_compensation`.
    :return: ``None``. Pivot edges whose destination does not have
        exactly one forward out-edge are skipped (left uncompensated)
        rather than raising; the validator (finding #36) enforces the
        one-forward-out-edge precondition for the production pipeline.
    """
    pivot_values = {int(Pivot.LEFT), int(Pivot.RIGHT)}

    # Snapshot the edge list so in-place attribute mutation during
    # iteration cannot disturb the loop variable. We only ever mutate
    # the *following* edge, never the pivot-tagged edge itself, but
    # making this explicit removes any doubt.
    pivot_edges = [
        (src, dst, data)
        for src, dst, data in graph.edges(data=True)
        if int(data.get("pivot", Pivot.NONE)) in pivot_values
    ]

    for src, dst, pivot_edge_data in pivot_edges:
        # Exclude the back-edge to ``src``: build_graph emits every edge
        # bidirectionally, so dst->src is always present and is never a
        # legitimate next-edge candidate (the chassis would not pivot
        # only to immediately reverse). Counting it would make pivot
        # compensation impossible to apply on any realistic graph.
        forward_out_edges = [
            (s, d, data)
            for s, d, data in graph.out_edges(dst, data=True)
            if d != src
        ]
        if len(forward_out_edges) != 1:
            # The validator (finding #36) now enforces this precondition for
            # the production pipeline -- a pivot landing with anything other
            # than one forward out-edge fails as a clean FieldDataError at
            # validation time. This skip is defensive for direct / hand-built
            # callers that bypass `validate` (e.g. unit tests assembling a
            # raw DiGraph): it leaves the edge uncompensated rather than
            # raising, so `forward_out_edges[0]` below can never IndexError on
            # a 0-successor landing and a multi-successor landing is never
            # mis-compensated by picking an arbitrary branch.
            continue

        _next_src, _next_dst, next_edge_data = forward_out_edges[0]
        pre_heading = float(pivot_edge_data["heading_deg"])
        post_heading = float(next_edge_data["heading_deg"])
        # Normalise the turn delta into [-180, 180]. ((d + 180) % 360) - 180
        # is the standard trick; works for any IEEE-754 float input.
        turn_deg = ((post_heading - pre_heading + 180.0) % 360.0) - 180.0

        next_dist = float(next_edge_data["distance_cm"])
        adj_dist, adj_heading = compute_pivot_compensation(
            turn_deg=turn_deg,
            track_cm=track_cm,
            next_edge_dist=next_dist,
            next_edge_heading=post_heading,
        )
        next_edge_data["distance_cm"] = adj_dist
        next_edge_data["heading_deg"] = adj_heading


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
    else 1.0)``. Single-wheel-pivot compensation
    (:func:`apply_pivot_compensations`) is applied to the input ``graph``
    **in place** *before* the weighted copy is built, so the shortened
    ``distance_cm`` feeds both the Dijkstra cost computed here and the
    ``routes.cpp`` emitter that the caller drives with the same ``graph``
    object afterward (Decision B, finding #42).

    .. warning::

        This function **mutates** ``graph``: every edge that follows a
        single-wheel pivot has its ``distance_cm`` / ``heading_deg``
        shortened to the compensated value. This is intentional -- the
        caller (:func:`tools.path_planner.build_routes._run`) passes the
        same ``graph`` to :func:`tools.path_planner.header_writer.emit_routes`
        right after, so the compensated geometry must live on the shared
        object. Callers that need the raw geometry afterward should pass a
        copy. A single call applies the compensation exactly once.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Every edge
        must carry ``type`` and ``distance_cm``.
    :param cost_blind_multiplier: positive multiplier applied to
        :data:`tools.path_planner.graph_builder.EDGE_BLIND` edges.
    :return: nested dict ``{source: {target: next_hop | None}}``.
        Outer dict has every node as a key; each inner dict has every
        node as a key (no missing entries).
    :raises ValueError: when ``cost_blind_multiplier`` is not positive.
        A pivot-tagged edge whose destination does not have exactly one
        forward out-edge is no longer an error here -- the validator
        rejects it up front (finding #36), and
        :func:`apply_pivot_compensations` skips it defensively for callers
        that bypass validation.
    """
    if cost_blind_multiplier <= 0:
        raise ValueError(
            "cost_blind_multiplier must be positive, got "
            f"{cost_blind_multiplier}. Zero would make BLIND edges cost "
            "nothing, so the planner would over-prefer them; a negative "
            "multiplier would give them negative weight, which Dijkstra "
            "cannot handle. A positive multiplier keeps BLIND edges more "
            "expensive than equal-length LINE edges."
        )

    # Apply pivot compensation to the INPUT graph in place BEFORE building
    # the weighted copy. Decision B (finding #42): the caller passes this
    # same ``graph`` object to ``emit_routes`` right after, so shortening
    # the following edge's ``distance_cm`` here is what makes the
    # compensated geometry reach routes.cpp -- not just the Dijkstra cost.
    apply_pivot_compensations(graph, TRACK_CM)
    # Build the weighted copy FROM the now-compensated graph so every
    # ``cost`` is derived from the compensated ``distance_cm``. No separate
    # cost-refresh pass is needed: compensation already ran above.
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

    .. note::

        Unlike :func:`compute_all_pairs_next_hop`, this function reads
        ``distance_cm`` straight off the *input* graph and does not
        apply pivot compensation. If a caller needs the compensated
        cost, they should run :func:`apply_pivot_compensations` on a
        copy of the graph first and pass that copy in here. Today this
        function is only used by tests to cross-check Dijkstra against a
        hand-computed cost on graphs without pivot tags, so the
        divergence is intentional.

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
