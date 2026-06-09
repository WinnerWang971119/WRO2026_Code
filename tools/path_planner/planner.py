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
from the *input* graph. Pivot compensation no longer mutates that
geometry (see below), so the cost the planner optimises and the geometry
the emitter consumes describe the same un-shortened edges; the
correction rides along as a separate ``pivot_trim_cm`` scalar on the
pivot edge itself.

Single-wheel pivot compensation
-------------------------------

Edges in the graph may carry a ``pivot`` attribute (the integer value of
a :class:`tools.path_planner.graph_builder.Pivot` member). When set to
``Pivot.LEFT`` or ``Pivot.RIGHT``, the turn entering that edge is
executed as a single-wheel pivot rather than a counter-rotation. The
chassis centre swings forward through one wheel, so it lands offset from
where a counter-rotation would have left it: it over-travels along the
post-turn heading by ::

    delta = (track_cm / 2) * tan(|theta| / 2)

where ``theta`` is the interior turn angle (see
:func:`compute_pivot_compensation`). Rather than fold this trim into the
FOLLOWING edge's ``distance_cm`` -- which corrupts the geometry of any
non-pivot route that happens to share that edge cell (issue #44) --
:func:`apply_pivot_compensations` stores the scalar ``delta`` onto the
PIVOT-tagged edge itself as a ``pivot_trim_cm`` attribute. Both the
pivot edge's and the following edge's ``distance_cm`` / ``heading_deg``
are left byte-identical to the pre-compensation graph, so the Dijkstra
cost and every shared route read the original distances; only the
firmware consumer of ``pivot_trim_cm`` (wired up by the emitter in a
later task) applies the trim, and only when the pivot actually fires.

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
import re
from pathlib import Path
from typing import Dict, List, Optional

import networkx as nx

from tools.path_planner.graph_builder import EDGE_BLIND, Pivot

# Public type alias: outer key = source node, inner key = target node,
# value = next node on the shortest path (or None if unreachable).
NextHopTable = Dict[str, Dict[str, Optional[str]]]


# -- chassis track (single-sourced from the firmware header) ------------------

# The firmware owns the one true chassis track width as
# ``constexpr float TRACK_MM`` in ``WRO2026/Config.h``. We parse it out of
# that header at build time rather than mirroring a literal here, so the
# planner can never drift from the value the robot actually drives with
# (issue #44: the old hard-coded 15.0 cm literal was stale -- the header
# says 180 mm = 18 cm). The regex matches the ``constexpr float TRACK_MM``
# declaration and captures the numeric literal, tolerating an optional
# trailing ``f`` suffix and surrounding whitespace.
_TRACK_MM_PATTERN = re.compile(
    r"constexpr\s+float\s+TRACK_MM\s*=\s*([0-9]*\.?[0-9]+)\s*[fF]?\s*;"
)

# ``planner.py`` lives at ``<repo>/tools/path_planner/planner.py``; the
# firmware header lives at ``<repo>/WRO2026/Config.h``. Resolve it relative
# to this file (three parents up to the repo root) so the lookup is
# independent of the caller's current working directory.
_CONFIG_H_PATH = Path(__file__).resolve().parents[2] / "WRO2026" / "Config.h"


def parse_track_cm_from_config(config_path: Path = _CONFIG_H_PATH) -> float:
    """Parse ``Config::TRACK_MM`` from ``WRO2026/Config.h`` and return cm.

    The firmware header is the single source of truth for the chassis
    track width. This reads ``constexpr float TRACK_MM = <value>f;`` out
    of it and converts millimetres to centimetres (``/ 10``).

    Fails loudly: if the header is missing or does not contain a parseable
    ``TRACK_MM`` declaration this raises instead of returning a stale
    fallback, because a wrong track width silently mis-compensates every
    pivot (issue #44).

    :param config_path: path to ``Config.h``. Defaults to the firmware
        header resolved relative to this module's location (independent of
        the caller's working directory).
    :return: chassis track width in centimetres (``TRACK_MM / 10``).
    :raises OSError: when ``config_path`` cannot be read (missing,
        permission denied, or a directory).
    :raises ValueError: when no ``constexpr float TRACK_MM = ...;``
        declaration is found in the header.
    """
    try:
        text = config_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise OSError(
            f"Cannot parse chassis track width: firmware header at "
            f"{config_path} is unreadable ({exc}). Config::TRACK_MM is the "
            f"single source of truth for the planner's pivot compensation "
            f"(issue #44); there is no fallback literal."
        ) from exc

    match = _TRACK_MM_PATTERN.search(text)
    if match is None:
        raise ValueError(
            f"Cannot parse chassis track width: no "
            f"'constexpr float TRACK_MM = <value>;' declaration found in "
            f"{config_path}. The planner refuses to fall back to a stale "
            f"literal (issue #44) -- fix the header or the parser, do not "
            f"hard-code a track width."
        )

    track_mm = float(match.group(1))
    return track_mm / 10.0


# -- pivot-compensation helpers ----------------------------------------------


def compute_pivot_compensation(turn_deg: float, track_cm: float) -> float:
    """Return the forward over-travel ``delta`` of a single-wheel pivot.

    A single-wheel pivot through ``turn_deg`` degrees rotates the chassis
    about one wheel, so the drive-wheel midpoint swings forward through an
    arc of radius ``track_cm / 2``. The component of that swing along the
    POST-turn heading -- the amount the chassis over-travels past the
    landing node before the following edge begins -- has the closed form ::

        delta = (track_cm / 2) * tan(|theta| / 2)

    This corrects the previous chord-decomposition estimate
    (``2 * (track_cm / 2) * sin(|theta| / 2) * sin(|theta| / 2)``), which
    only happened to agree at ``theta == 90 deg`` (both yield
    ``0.5 * track_cm`` there) and diverged everywhere else -- e.g. at
    ``theta == 60 deg`` the tangent form gives ``~0.289 * track_cm`` and at
    ``theta == 120 deg`` ``~0.866 * track_cm``.

    The helper is pure (no graph dependency) and side-effect free: callers
    decide where to store the returned scalar.
    :func:`apply_pivot_compensations` writes it onto the pivot edge as
    ``pivot_trim_cm`` rather than folding it into any edge's
    ``distance_cm`` (issue #44).

    :param turn_deg: signed turn magnitude (degrees, CCW positive). Only
        ``|turn_deg|`` enters the formula, so the sign is accepted but
        unused -- callers may pass a signed heading delta directly.
    :param track_cm: chassis track width (wheel-to-wheel) in centimetres.
        Must be non-negative. ``0.0`` yields ``delta == 0.0``.
    :return: the forward over-travel ``delta`` in centimetres
        (always ``>= 0`` for ``track_cm >= 0`` and ``|theta| < 180``).
    :note: ``tan`` diverges as ``|theta| -> 180 deg`` (a full reversal on
        one wheel). The hard fail-fast for degenerate / near-180 turn
        angles is the validator's responsibility; this helper computes the
        closed-form value verbatim and does NOT silently clamp it to a
        wrong number.
    """
    half_theta_rad = math.radians(abs(turn_deg) / 2.0)
    delta = (track_cm / 2.0) * math.tan(half_theta_rad)
    return delta


def apply_pivot_compensations(graph: nx.DiGraph, track_cm: float) -> None:
    """Tag every single-wheel-pivot edge with its forward over-travel trim.

    For every edge ``(src, dst)`` in ``graph`` whose ``pivot`` attribute
    is set to :attr:`Pivot.LEFT` or :attr:`Pivot.RIGHT` (compared by
    integer value, since :func:`graph_builder._extract_directional_attrs`
    stores the int rather than the IntEnum), this function locates the
    *single* forward out-edge from ``dst`` (to read the post-turn
    heading), computes the over-travel scalar ::

        delta = (track_cm / 2) * tan(|theta| / 2)

    via :func:`compute_pivot_compensation`, and STORES it on the
    PIVOT-tagged edge itself as ``pivot_edge_data["pivot_trim_cm"] =
    delta``. It does **not** mutate the following edge -- neither its
    ``distance_cm`` nor its ``heading_deg`` -- nor the pivot edge's own
    distance/heading. Every edge's geometry stays byte-identical to the
    pre-compensation graph; only the new ``pivot_trim_cm`` key is added.

    This is the issue #44 fix: the previous version shortened the
    following edge's ``distance_cm`` in place, which corrupted the
    geometry of any non-pivot route that happened to share that edge
    cell. Storing the trim on the pivot edge keeps the correction local
    to the pivot maneuver -- it only fires when the pivot does.

    The turn magnitude ``theta`` is the heading delta between the
    pivot-tagged edge and its following edge, normalised to
    ``[-180, 180]`` degrees.

    .. note::

        Locating the following edge still requires the landing node to
        have exactly one forward out-edge. Out-edge choice is only known
        once Dijkstra has picked a path, but this function runs *before*
        Dijkstra, so a multi-successor landing has no single post-turn
        heading to measure the turn against. The safe scope cut is to
        require pivot destinations to have exactly one forward out-edge;
        the field-data author can structure the graph accordingly. A
        future per-path pivot-aware planner could relax this.

        For the production pipeline the validator enforces this
        precondition up front (finding #36): a pivot landing with the
        wrong out-degree fails as a :class:`FieldDataError` at validation
        time. This function therefore does NOT raise on a mismatch -- it
        skips the offending pivot edge (leaving it un-trimmed). The skip
        is a defensive guard for direct / hand-built callers that assemble
        a graph without running
        :func:`tools.path_planner.validator.validate`; it keeps
        ``forward_out_edges[0]`` safe from an IndexError and never
        measures the turn against an arbitrary branch.

        The back-edge from ``dst`` to ``src`` (always present when the
        edge was emitted by :func:`build_graph`'s bidirectional pair) is
        excluded from the out-edge count and from the heading lookup --
        the chassis is never going to immediately reverse the pivot it
        just executed.

        Storing the trim is idempotent per pivot edge: the trim now lives
        on the pivot edge itself, so re-running this function (or a second
        pass over the same graph) re-stores the same scalar and never
        compounds. The cross-pivot double-compensation that finding #94
        guarded against -- two pivots from non-adjacent sources landing on
        one shared following edge -- is structurally gone now that each
        pivot carries its own trim instead of subtracting from the shared
        edge; this function still skips a pivot edge that already carries
        ``pivot_trim_cm`` so the operation is a no-op on a second call.

    :param graph: a directed graph that gains a ``pivot_trim_cm`` scalar
        on each qualifying pivot edge **in place**. No ``distance_cm`` /
        ``heading_deg`` is mutated, so the same object is safe to hand to
        both the Dijkstra cost step and the routes.cpp emitter
        afterwards. :func:`compute_all_pairs_next_hop` relies on that.
    :param track_cm: chassis track width in centimetres. Forwarded to
        :func:`compute_pivot_compensation`.
    :return: ``None``. Pivot edges whose destination does not have
        exactly one forward out-edge are skipped (left un-trimmed) rather
        than raising; the validator (finding #36) enforces the
        one-forward-out-edge precondition for the production pipeline.
    """
    pivot_values = {int(Pivot.LEFT), int(Pivot.RIGHT)}

    # Snapshot the edge list so in-place attribute mutation during
    # iteration cannot disturb the loop variable. We only ever write to
    # the pivot-tagged edge itself (never the following edge), but making
    # this explicit removes any doubt.
    pivot_edges = [
        (src, dst, data)
        for src, dst, data in graph.edges(data=True)
        if int(data.get("pivot", Pivot.NONE)) in pivot_values
    ]

    for src, dst, pivot_edge_data in pivot_edges:
        # Idempotency guard: if this pivot edge already carries its trim,
        # re-storing it is a no-op, so skip the work. The trim now lives on
        # the pivot edge itself (not the shared following edge), so a second
        # pass -- or a second pivot landing on the same node -- can never
        # compound the correction; each pivot owns its own scalar. This guard
        # just keeps the function a clean no-op on a repeated call.
        if "pivot_trim_cm" in pivot_edge_data:
            continue

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
            # raw DiGraph): it leaves the pivot edge un-trimmed rather than
            # raising, so `forward_out_edges[0]` below can never IndexError on
            # a 0-successor landing and a multi-successor landing's turn is
            # never measured against an arbitrary branch.
            continue

        _next_src, _next_dst, next_edge_data = forward_out_edges[0]

        pre_heading = float(pivot_edge_data["heading_deg"])
        post_heading = float(next_edge_data["heading_deg"])
        # Normalise the turn delta into [-180, 180]. ((d + 180) % 360) - 180
        # is the standard trick; works for any IEEE-754 float input.
        turn_deg = ((post_heading - pre_heading + 180.0) % 360.0) - 180.0

        delta = compute_pivot_compensation(turn_deg=turn_deg, track_cm=track_cm)

        # Store the over-travel trim on the PIVOT edge itself -- do NOT
        # touch the following edge's distance/heading (issue #44). The
        # emitter (a later task) reads this scalar and applies the trim
        # only when the pivot actually fires.
        pivot_edge_data["pivot_trim_cm"] = delta


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
    track_cm: Optional[float] = None,
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
    **in place** *before* the weighted copy is built. As of the issue #44
    fix it no longer touches any edge's ``distance_cm`` / ``heading_deg``
    -- it only tags each pivot edge with a ``pivot_trim_cm`` scalar -- so
    the Dijkstra cost computed here and the ``routes.cpp`` emitter driven
    by the caller both read the original, un-shortened distances.

    .. note::

        This function still mutates ``graph``, but only by ADDING a
        ``pivot_trim_cm`` key to single-wheel-pivot edges. No existing
        geometry attribute is changed, so a caller that re-uses ``graph``
        afterward (e.g. :func:`tools.path_planner.build_routes._run`
        handing it to
        :func:`tools.path_planner.header_writer.emit_routes`) sees the
        same distances it passed in, plus the trim scalars. The store is
        idempotent: a second call re-stores the same scalars.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Every edge
        must carry ``type`` and ``distance_cm``.
    :param cost_blind_multiplier: positive multiplier applied to
        :data:`tools.path_planner.graph_builder.EDGE_BLIND` edges.
    :param track_cm: chassis track width in centimetres, forwarded to
        :func:`apply_pivot_compensations`. ``None`` (the default) parses
        ``Config::TRACK_MM`` from ``WRO2026/Config.h`` via
        :func:`parse_track_cm_from_config` -- the single source of truth
        (issue #44). :func:`tools.path_planner.build_routes._run` parses
        it once and passes it in explicitly; callers that only want the
        next-hop table (and have no pivot edges to compensate, or accept
        the header-derived default) may omit it.
    :return: nested dict ``{source: {target: next_hop | None}}``.
        Outer dict has every node as a key; each inner dict has every
        node as a key (no missing entries).
    :raises ValueError: when ``cost_blind_multiplier`` is not positive.
        A pivot-tagged edge whose destination does not have exactly one
        forward out-edge is no longer an error here -- the validator
        rejects it up front (finding #36), and
        :func:`apply_pivot_compensations` skips it defensively for callers
        that bypass validation.
    :raises FileNotFoundError: when ``track_cm`` is ``None`` and
        ``WRO2026/Config.h`` cannot be found (no stale-literal fallback).
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

    # Resolve the chassis track from the firmware header when the caller
    # did not pass one. There is no stale literal fallback (issue #44):
    # parse_track_cm_from_config raises if Config::TRACK_MM is missing.
    if track_cm is None:
        track_cm = parse_track_cm_from_config()

    # Tag pivot edges with their over-travel trim BEFORE building the
    # weighted copy. This adds a ``pivot_trim_cm`` scalar to each pivot
    # edge but leaves every ``distance_cm`` / ``heading_deg`` untouched,
    # so the trim never leaks into the Dijkstra cost or any shared route's
    # geometry (issue #44).
    apply_pivot_compensations(graph, track_cm)
    # Build the weighted copy from the graph. Geometry is unchanged by the
    # pivot tagging above, so every ``cost`` is derived from the original
    # ``distance_cm``; the ``pivot_trim_cm`` scalar rides along on the
    # pivot edges and is ignored by the cost model.
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
