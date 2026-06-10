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
from the *input* graph. Pivot compensation never mutates that geometry
(see below), so the cost the planner optimises and the geometry the
emitter consumes describe the same un-shortened edges.

Single-wheel pivot compensation (per-turn model)
------------------------------------------------

A single-wheel pivot is a property of a *turn* -- a 3-node triple
``(u, v, w)`` declared in :data:`tools.path_planner.field_data.PIVOT_TURNS`
-- not of an edge. The chassis arrives at ``v`` from ``u``, pivots about
one wheel, and leaves toward ``w``. Because the centre swings forward
through one wheel, the chassis lands offset from where a counter-rotation
would have left it: it over-travels along the post-turn heading by ::

    delta = (track_cm / 2) * tan(|theta| / 2)

where ``theta`` is the signed interior turn angle at ``v`` (see
:func:`compute_pivot_compensation`). :func:`build_pivot_turns` derives
``theta`` from the two edge headings, the wheel from ``sign(theta)``, and
the over-travel ``delta`` per triple, returning a sparse list of
:class:`PivotTurnRecord` for the header emitter to serialise into a small
``PIVOT_TURNS[]`` wire table. The graph itself is never mutated: every
``distance_cm`` / ``heading_deg`` is left byte-identical to the input, so
the Dijkstra cost and every shared route read the original distances. The
firmware applies a record's trim only when the route actually takes that
turn.

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
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import networkx as nx

from tools.path_planner.compiler import normalize_turn
from tools.path_planner.graph_builder import EDGE_BLIND, directed_edge_heading
from tools.path_planner.pipeline_config import TurnMode

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
    decide where to store the returned scalar. :func:`build_pivot_turns`
    bakes it into each :class:`PivotTurnRecord`'s ``trim_cm`` (the firmware
    subtracts it from the relevant drive when the route takes the turn)
    rather than folding it into any edge's ``distance_cm`` (issue #44).

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


@dataclass(frozen=True)
class PivotTurnRecord:
    """One resolved single-wheel pivot turn ``(u, v, w)`` for the wire table.

    A pivot is declared per turn in
    :data:`tools.path_planner.field_data.PIVOT_TURNS` as a node-name
    triple. :func:`build_pivot_turns` resolves each triple into one of
    these records: the geometry-derived pivot wheel and the forward
    over-travel ``trim_cm``. The header writer (T4) serialises a sorted
    list of these into the sparse ``PIVOT_TURNS[]`` array in
    ``routes.h`` / ``routes.cpp``; the firmware applies the trim only
    when the route actually takes the turn ``u -> v -> w``.

    Frozen (immutable, hashable) so a record can be safely sorted, kept
    in a set for duplicate detection, and compared byte-for-byte in
    golden tests.

    :ivar u: arrive-from node name.
    :ivar v: pivot-at node name (where the single-wheel turn happens).
    :ivar w: leave-to node name.
    :ivar wheel: the pivot wheel, :attr:`TurnMode.PIVOT_LEFT` or
        :attr:`TurnMode.PIVOT_RIGHT`, derived from ``sign(theta)``.
    :ivar trim_cm: forward over-travel ``delta`` in centimetres
        (``(track_cm / 2) * tan(|theta| / 2)``).
    """

    u: str
    v: str
    w: str
    wheel: TurnMode
    trim_cm: float


def build_pivot_turns(
    graph: nx.DiGraph,
    track_cm: Optional[float] = None,
) -> List[PivotTurnRecord]:
    """Resolve every authored pivot triple into a :class:`PivotTurnRecord`.

    A single-wheel pivot is a property of a *turn*, not an edge: the
    chassis arrives at ``v`` from ``u``, pivots about one wheel, and
    leaves toward ``w``. For each ``(u, v, w)`` triple in
    :data:`tools.path_planner.field_data.PIVOT_TURNS` this:

    1. measures the signed turn angle at ``v`` --
       ``theta = normalize(heading(v->w) - heading(u->v))`` into
       ``(-180, 180]`` -- using
       :func:`tools.path_planner.graph_builder.directed_edge_heading`
       (which honours field-data heading overrides) and the canonical
       :func:`tools.path_planner.compiler.normalize_turn`;
    2. computes the forward over-travel
       ``delta = (track_cm / 2) * tan(|theta| / 2)`` via
       :func:`compute_pivot_compensation` (reused, never re-derived); and
    3. derives the pivot wheel from ``sign(theta)``:
       ``theta > 0`` (CCW / left turn) -> :attr:`TurnMode.PIVOT_LEFT`
       (brake the left wheel, drive the right; the nose swings CCW),
       ``theta < 0`` -> :attr:`TurnMode.PIVOT_RIGHT`. ADR-036 derives this
       mapping from the wheel-byte semantics plus the forward-travel
       assumption -- authors never type a wheel.

    The graph is **never mutated**: every ``distance_cm`` / ``heading_deg``
    stays byte-identical to the input, so the same object is safe to hand
    to both the Dijkstra cost step and the ``routes.cpp`` emitter. Pivot
    derivation is decoupled from :func:`compute_all_pairs_next_hop`; the
    orchestrator (``build_routes``) calls this separately and threads the
    returned records into the header emitter.

    Edge existence for both legs is guaranteed by the validator
    (``_validate_pivot_turns``) running first, but this code guards
    defensively per existing style: a triple whose ``u -> v`` or
    ``v -> w`` is absent from ``graph`` is skipped (not derived) rather
    than raising a :class:`KeyError`, so a direct / hand-built caller that
    bypasses :func:`tools.path_planner.validator.validate` degrades
    cleanly instead of crashing.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Read-only.
    :param track_cm: chassis track width in centimetres, forwarded to
        :func:`compute_pivot_compensation`. ``None`` (the default) parses
        ``Config::TRACK_MM`` from ``WRO2026/Config.h`` via
        :func:`parse_track_cm_from_config` -- the single source of truth
        (issue #44). Bypass callers (e.g. CTPS) pass it explicitly so no
        new hard Config.h dependency is baked in.
    :return: list of :class:`PivotTurnRecord`, one per resolvable triple,
        in :data:`field_data.PIVOT_TURNS` authoring order. Empty when the
        field declares no pivots (the real field today).
    :raises FileNotFoundError: when ``track_cm`` is ``None`` and
        ``WRO2026/Config.h`` cannot be found (no stale-literal fallback).
    """
    # Deferred import (mirrors the validator's pattern): keeps the planner
    # module import lightweight and avoids loading field_data.json -- and
    # the editor loader it pulls in -- at every planner import. The library
    # layer (validator / compiler / graph_builder) all defer or parameterise
    # field_data the same way; only orchestrator / CLI modules import it
    # eagerly.
    from tools.path_planner import field_data

    # Resolve the chassis track from the firmware header when the caller
    # did not pass one. There is no stale literal fallback (issue #44):
    # parse_track_cm_from_config raises if Config::TRACK_MM is missing.
    if track_cm is None:
        track_cm = parse_track_cm_from_config()

    records: List[PivotTurnRecord] = []
    for u, v, w in field_data.PIVOT_TURNS:
        # Defensive edge-existence guard: the validator enforces both legs
        # exist up front, so this only fires for hand-built callers that
        # bypass `validate`. Skip the triple rather than letting
        # directed_edge_heading raise a KeyError.
        if not (graph.has_edge(u, v) and graph.has_edge(v, w)):
            continue

        # theta = the signed turn at v: how far the travel heading rotates
        # from the incoming edge (u -> v) to the outgoing edge (v -> w).
        # directed_edge_heading reads the stored heading_deg (honouring any
        # field_data override); normalize_turn is the one canonical
        # normalizer, folding the delta into (-180, 180].
        in_heading = directed_edge_heading(graph, u, v)
        out_heading = directed_edge_heading(graph, v, w)
        theta = normalize_turn(out_heading - in_heading)

        # delta = forward over-travel; reuse the closed-form helper, never
        # re-derive (T/2)*tan(theta/2) inline (issue #44, TC2).
        trim_cm = compute_pivot_compensation(turn_deg=theta, track_cm=track_cm)

        # Wheel from sign(theta): a CCW (left) turn pivots about the LEFT
        # wheel, a CW (right) turn about the RIGHT wheel (ADR-036). theta
        # is never exactly 0 here -- the validator's raised min-angle floor
        # rejects a degenerate ~0 deg "turn" -- so the >0 / else split is
        # unambiguous.
        wheel = TurnMode.PIVOT_LEFT if theta > 0 else TurnMode.PIVOT_RIGHT

        records.append(
            PivotTurnRecord(u=u, v=v, w=w, wheel=wheel, trim_cm=trim_cm)
        )

    return records


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
    else 1.0)``. This function is **pivot-agnostic**: single-wheel-pivot
    derivation is decoupled into :func:`build_pivot_turns`, which the
    orchestrator calls separately. Nothing here mutates ``graph``, so the
    Dijkstra cost computed here and the ``routes.cpp`` emitter driven by
    the caller both read the same original, un-shortened distances.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Every edge
        must carry ``type`` and ``distance_cm``. Read-only.
    :param cost_blind_multiplier: positive multiplier applied to
        :data:`tools.path_planner.graph_builder.EDGE_BLIND` edges.
    :param track_cm: accepted but unused -- retained only so the
        orchestrator's existing ``track_cm=`` call site keeps working
        until T16 drops it there. Pivot trim no longer lives in this
        function (it moved to :func:`build_pivot_turns`); nothing here
        reads the track width, so the value is ignored.
    :return: nested dict ``{source: {target: next_hop | None}}``.
        Outer dict has every node as a key; each inner dict has every
        node as a key (no missing entries).
    :raises ValueError: when ``cost_blind_multiplier`` is not positive.
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

    # ``track_cm`` is intentionally not consulted here any more: pivot
    # derivation moved out of the next-hop pass into build_pivot_turns
    # (the orchestrator calls it separately). The parameter stays in the
    # signature only so the current build_routes call site keeps working
    # until T16 removes it; deleting it here would break that call before
    # T16 lands.
    del track_cm

    # Build the weighted copy from the (unmutated) graph. Every ``cost`` is
    # derived from the original ``distance_cm``; no pivot geometry rides
    # along, so the cost model and the emitter see identical distances.
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

        This function reads ``distance_cm`` straight off the input graph.
        Pivot compensation never enters the cost: under the per-turn model
        the over-travel trim lives in the sparse :class:`PivotTurnRecord`
        table (:func:`build_pivot_turns`), not on any edge, and the
        firmware applies it at drive time -- it does not shorten the
        planned distance the Dijkstra cost optimises. This function is used
        by tests to cross-check Dijkstra against a hand-computed cost.

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
