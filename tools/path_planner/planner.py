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

Cost model (blind-distance decay)
---------------------------------

The cost of a :data:`tools.path_planner.graph_builder.EDGE_LINE` or
:data:`tools.path_planner.graph_builder.EDGE_PROJECTION` edge equals its
``distance_cm`` (factor ``1`` -- the chassis is tracking the line, the
most reliable mode). A :data:`tools.path_planner.graph_builder.EDGE_BLIND`
edge is penalised by a *convex, multiplicative* decay factor keyed on how
deep its tail sits in line-free, anchor-far blind territory::

    weight(blind u -> v) = distance_cm(u -> v)
                           * (1 + k * (d(u) / d_ref) ** exponent)

where (see :class:`BlindDecayParams`):

* ``k`` is a unitless penalty gain (``k == 0`` reduces the model to plain
  ``distance_cm`` -- a strict generalisation of the old flat multiplier);
* ``exponent`` (``>= 1``) makes the penalty convex in ``d``;
* ``d_ref`` is a length normaliser (cm) so ``k`` is map-portable;
* ``d(u)`` is the **blind-subgraph distance from the edge's TAIL node ``u``
  to the nearest ANCHOR**, an anchor being any node whose ``type`` is
  ``"line"`` OR ``"reset"``. It is computed once per table build by a
  multi-source search over the blind subgraph (only ``EDGE_BLIND`` edges,
  weighted by ``distance_cm``) seeded with ``d = 0`` at every line + reset
  node -- see :func:`compute_blind_distance_to_anchor`.

Rationale: a line intersection re-anchors localization (ADR-001/005), so a
route on the line backbone clears accumulated blind drift exactly like a
reset node does. Seeding the search from all line + reset nodes makes the
planner prefer that self-correcting backbone; reset nodes then matter
specifically in line-free blind pockets the line graph cannot reach. The
penalty is keyed on the edge *tail* (a static potential field), so it is
directionally naive by design -- the goal is to discourage deep blind
*chains*, not single bounded hops (ADR-041).

Unreachable fallback: a blind node with no anchor reachable in the blind
subgraph (``d == inf``) falls back to factor ``1`` (no decay signal -> no
penalty), keeping every Dijkstra weight finite. With dense line anchors
this is degenerate; an anchor-less blind island is already the
disconnected-node case the builder warns about.

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
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import networkx as nx

from tools.path_planner.compiler import normalize_turn
from tools.path_planner.config_parse import parse_config_float
from tools.path_planner.graph_builder import EDGE_BLIND, directed_edge_heading
from tools.path_planner.pipeline_config import TurnMode

# Public type alias: outer key = source node, inner key = target node,
# value = next node on the shortest path (or None if unreachable).
NextHopTable = Dict[str, Dict[str, Optional[str]]]

# Node ``type`` values that re-anchor localization and therefore seed the
# blind-distance decay search (see :func:`compute_blind_distance_to_anchor`).
# A line intersection re-anchors via ``onIntersection`` (ADR-001/005) exactly
# as a wall-zero reset node does, so both clear accumulated blind drift; the
# decay model treats them identically as anchors. ``"blind"`` nodes are NOT
# anchors -- they are the deep-blind territory the penalty discourages.
NODE_TYPE_LINE = "line"
NODE_TYPE_RESET = "reset"
_ANCHOR_NODE_TYPES = frozenset((NODE_TYPE_LINE, NODE_TYPE_RESET))


# -- blind-distance decay parameters ------------------------------------------


@dataclass(frozen=True)
class BlindDecayParams:
    """Immutable knobs for the blind-edge decay cost (replaces the old multiplier).

    A blind edge is penalised by ``1 + k * (d(u) / d_ref) ** exponent`` where
    ``d(u)`` is the blind-subgraph distance from the edge tail to its nearest
    line / reset anchor (see the module docstring's "Cost model" section).
    Frozen so a params object can be shared across calls and compared in tests
    without risk of mutation.

    The four fields mirror ``field_data.BLIND_DECAY_*`` (authored in
    ``field_data.json`` per ADR-022); :func:`from_field_data` adapts a
    field-data namespace into one of these so callers never spell the four
    attribute names out by hand.

    :ivar enabled: master switch. ``False`` short-circuits every blind edge to
        factor ``1`` (plain ``distance_cm``) and skips the d-map computation.
    :ivar k: unitless penalty gain ``>= 0``. ``k == 0`` also reduces every
        blind edge to factor ``1`` (the AC14b regression anchor) -- a strict
        generalisation of the old flat ``COST_BLIND_MULTIPLIER`` model.
    :ivar exponent: convexity exponent ``>= 1`` in ``(d / d_ref) ** exponent``.
        A sub-linear exponent would invert the penalty's intent (cheaper far
        from an anchor), so the planner rejects ``exponent < 1``.
    :ivar d_ref: length normaliser in centimetres, ``> 0``, so ``k`` is
        portable across maps. A non-positive ``d_ref`` divides by zero / flips
        the sign, so the planner rejects it.
    """

    enabled: bool
    k: float
    exponent: float
    d_ref: float

    @classmethod
    def from_field_data(cls, field_data) -> "BlindDecayParams":
        """Build a :class:`BlindDecayParams` from a field-data namespace.

        Reads the four ``BLIND_DECAY_*`` top-level scalars off ``field_data``
        (the JSON-bridge module or any object exposing the same attributes).
        The reads are strict (no ``getattr`` default): a fixture missing any
        of the four knobs fails loudly here rather than silently defaulting.

        :param field_data: object exposing ``BLIND_DECAY_ENABLED`` /
            ``BLIND_DECAY_K`` / ``BLIND_DECAY_EXPONENT`` / ``BLIND_DECAY_D_REF``.
        :return: the populated, frozen params object.
        :raises AttributeError: if any ``BLIND_DECAY_*`` attribute is absent.
        """
        return cls(
            enabled=bool(field_data.BLIND_DECAY_ENABLED),
            k=float(field_data.BLIND_DECAY_K),
            exponent=float(field_data.BLIND_DECAY_EXPONENT),
            d_ref=float(field_data.BLIND_DECAY_D_REF),
        )


def _validate_blind_decay_params(params: BlindDecayParams) -> None:
    """Reject a malformed :class:`BlindDecayParams` (mirrors the validator).

    The field-data validator (``validator._validate_blind_decay``) already
    enforces these domains on the authored ``field_data.json``, but a direct
    or bypass caller can construct a :class:`BlindDecayParams` by hand. This
    guards the planner so such a caller fails loudly rather than emitting a
    negative / NaN Dijkstra weight (which would corrupt the routing table or
    crash the heap). The rules match the validator one-for-one:

    * ``d_ref > 0`` (the normaliser; a non-positive value divides by zero);
    * ``k >= 0`` (a negative gain would make blind edges *cheaper*);
    * ``exponent >= 1`` (a sub-linear exponent inverts the penalty's intent).

    A disabled params object (``enabled is False``) short-circuits to factor
    ``1`` upstream and never reaches the weight formula, so it is exempt.

    :param params: the decay params to validate.
    :return: ``None`` on success.
    :raises ValueError: on the first out-of-domain field, with the offending
        value in the message.
    """
    if not params.enabled:
        return
    if params.d_ref <= 0:
        raise ValueError(
            "BlindDecayParams.d_ref must be positive (it normalises d so k is "
            f"map-portable; a non-positive d_ref divides by zero), got "
            f"{params.d_ref}."
        )
    if params.k < 0:
        raise ValueError(
            "BlindDecayParams.k must be non-negative (a negative gain would "
            f"make BLIND edges cheaper than equal-length LINE edges), got "
            f"{params.k}."
        )
    if params.exponent < 1:
        raise ValueError(
            "BlindDecayParams.exponent must be >= 1 (a sub-linear exponent "
            "would make far-from-anchor blind hops cheaper, inverting the "
            f"penalty), got {params.exponent}."
        )


# -- chassis track (single-sourced from the firmware header) ------------------

# The firmware owns the one true chassis track width as
# ``constexpr float TRACK_MM`` in ``WRO2026/config/ConfigDrive.h``. We parse it
# out of that header at build time rather than mirroring a literal here, so the
# planner can never drift from the value the robot actually drives with (issue
# #44: the old hard-coded 15.0 cm literal was stale -- the header says 180 mm =
# 18 cm). The parse is delegated to the shared, anchored
# ``config_parse.parse_config_float`` so the planner and validator read the
# header the same way.

# ``planner.py`` lives at ``<repo>/tools/path_planner/planner.py``; the
# firmware header lives at ``<repo>/WRO2026/config/ConfigDrive.h``. Resolve it
# relative to this file (three parents up to the repo root) so the lookup is
# independent of the caller's current working directory.
_CONFIG_H_PATH = (
    Path(__file__).resolve().parents[2] / "WRO2026" / "config" / "ConfigDrive.h"
)


def parse_track_cm_from_config(config_path: Path = _CONFIG_H_PATH) -> float:
    """Parse ``Config::TRACK_MM`` from ``WRO2026/config/ConfigDrive.h`` and return cm.

    The firmware header is the single source of truth for the chassis
    track width. This reads ``constexpr float TRACK_MM = <value>f;`` out
    of it and converts millimetres to centimetres (``/ 10``).

    Fails loudly: if the header is missing or does not contain a parseable
    ``TRACK_MM`` declaration this raises instead of returning a stale
    fallback, because a wrong track width silently mis-compensates every
    pivot (issue #44).

    :param config_path: path to ``config/ConfigDrive.h``. Defaults to the
        firmware header resolved relative to this module's location (independent
        of the caller's working directory).
    :return: chassis track width in centimetres (``TRACK_MM / 10``).
    :raises OSError: when ``config_path`` cannot be read (missing,
        permission denied, or a directory).
    :raises ValueError: when no ``constexpr float TRACK_MM = ...;``
        declaration is found in the header.
    """
    track_mm = parse_config_float("TRACK_MM", config_path)
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
    *,
    pivot_turns: Optional[List] = None,
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
        ``Config::TRACK_MM`` from ``WRO2026/config/ConfigDrive.h`` via
        :func:`parse_track_cm_from_config` -- the single source of truth
        (issue #44). Bypass callers (e.g. CTPS) pass it explicitly so no
        new hard ConfigDrive.h dependency is baked in.
    :param pivot_turns: the ``(u, v, w)`` triples to resolve. ``None`` (the
        default) reads the module-global
        :data:`tools.path_planner.field_data.PIVOT_TURNS` -- the existing
        behavior, byte-for-byte unchanged. A caller constructed against a
        *different* field (e.g. the sim, which is built with a toy/synthetic
        fixture) passes its own field's triples here so the real field's
        pivots are not resolved against the wrong graph.
    :return: list of :class:`PivotTurnRecord`, one per resolvable triple,
        in authoring order (``field_data.PIVOT_TURNS`` order for the
        default, or ``pivot_turns`` order when supplied). Empty when no
        pivots are declared (the real field today).
    :raises FileNotFoundError: when ``track_cm`` is ``None`` and
        ``WRO2026/config/ConfigDrive.h`` cannot be found (no stale-literal fallback).
    """
    # Choose the triples source. ``None`` reads the module-global
    # field_data.PIVOT_TURNS (the existing behavior); a caller built against a
    # different field passes its own triples so the real field's pivots are not
    # resolved against a foreign graph. The field_data import stays DEFERRED and
    # only fires on the None default -- mirroring the validator's pattern, it
    # keeps the planner import lightweight (no field_data.json / editor loader
    # at import time) and leaves every existing caller byte-for-byte unchanged.
    if pivot_turns is None:
        from tools.path_planner import field_data

        triples = field_data.PIVOT_TURNS
    else:
        triples = pivot_turns

    # Resolve the chassis track from the firmware header when the caller
    # did not pass one. There is no stale literal fallback (issue #44):
    # parse_track_cm_from_config raises if Config::TRACK_MM is missing.
    if track_cm is None:
        track_cm = parse_track_cm_from_config()

    records: List[PivotTurnRecord] = []
    for u, v, w in triples:
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

        # A collinear turn (theta == 0, including -0.0) is not a pivot: there is
        # no rotation to compensate and no wheel to pivot about, and emitting a
        # record would wrongly force a single-wheel maneuver on a straight-
        # through. The validator's raised min-angle floor rejects it in
        # production; guard here too so a direct/bypass caller that declares a
        # degenerate triple neither emits a spurious zero-trim record nor
        # silently maps -0.0 to PIVOT_RIGHT through the sign split below.
        if theta == 0.0:
            continue

        # delta = forward over-travel; reuse the closed-form helper, never
        # re-derive (T/2)*tan(theta/2) inline (issue #44, TC2).
        trim_cm = compute_pivot_compensation(turn_deg=theta, track_cm=track_cm)

        # Wheel from sign(theta): a CCW (left) turn pivots about the LEFT wheel,
        # a CW (right) turn about the RIGHT wheel (ADR-036). theta is non-zero
        # here -- the collinear guard above skipped theta == 0 -- so the
        # >0 / else split is unambiguous.
        wheel = TurnMode.PIVOT_LEFT if theta > 0 else TurnMode.PIVOT_RIGHT

        records.append(
            PivotTurnRecord(u=u, v=v, w=w, wheel=wheel, trim_cm=trim_cm)
        )

    return records


# -- public helper: blind-distance-to-anchor ---------------------------------


def compute_blind_distance_to_anchor(graph: nx.DiGraph) -> Dict[str, float]:
    """Return each node's blind-subgraph distance to its nearest anchor.

    An **anchor** is any node whose ``type`` is ``"line"`` OR ``"reset"`` (see
    :data:`_ANCHOR_NODE_TYPES`). The distance ``d(u)`` is measured over the
    **blind subgraph** -- the directed graph restricted to
    :data:`tools.path_planner.graph_builder.EDGE_BLIND` edges, weighted by each
    edge's ``distance_cm``. A single multi-source Dijkstra
    (:func:`networkx.multi_source_dijkstra_path_length`) seeded with ``d == 0``
    at every anchor node yields, for each node, the length of the shortest
    all-blind chain back to *some* anchor.

    Semantics:

    * an anchor node maps to ``0.0`` (it is its own nearest anchor, even when
      it has no blind edges -- isolated anchors are still seeds);
    * a blind-only node maps to the shortest blind-chain length to an anchor;
    * a blind-only node with **no** anchor reachable over blind edges maps to
      :data:`math.inf` (the unreachable fallback, which the weight formula
      treats as factor ``1`` -- no decay signal, no penalty).

    Every node in ``graph`` appears in the returned dict (``inf`` if not
    reached and not itself an anchor), so a caller can index any tail node
    without a ``KeyError``.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Nodes must carry
        a ``type`` attribute; blind edges must carry ``distance_cm``.
        Read-only.
    :return: ``{node_name: d}`` with ``d`` a finite float or ``math.inf``.
    """
    # 1. The anchors: every line / reset node seeds the search at d == 0.
    anchors = {
        node
        for node, attrs in graph.nodes(data=True)
        if attrs.get("type") in _ANCHOR_NODE_TYPES
    }

    # 2. The blind subgraph: only EDGE_BLIND edges, weighted by distance_cm.
    #    Every node is copied first so isolated anchors (no blind edges) and
    #    blind-only nodes alike are present as keys, mirroring the input graph.
    blind = nx.DiGraph()
    blind.add_nodes_from(graph.nodes())
    for u, v, data in graph.edges(data=True):
        if data["type"] == EDGE_BLIND:
            blind.add_edge(u, v, distance_cm=float(data["distance_cm"]))

    # 3. Default every node to inf (no anchor reachable), then overwrite the
    #    reachable ones. With no anchors at all every node stays inf and the
    #    weight formula falls back to factor 1 everywhere -- a degenerate but
    #    finite, never-disconnecting result.
    distances: Dict[str, float] = {node: math.inf for node in graph.nodes()}
    if not anchors:
        return distances

    reachable = nx.multi_source_dijkstra_path_length(
        blind, anchors, weight="distance_cm"
    )
    for node, dist in reachable.items():
        distances[node] = float(dist)

    return distances


# -- internal helpers ---------------------------------------------------------


def _blind_decay_factor(d: float, params: BlindDecayParams) -> float:
    """Return the multiplicative weight factor for a blind edge with tail-d ``d``.

    Implements ``1 + k * (d / d_ref) ** exponent`` with the disabled / zero-gain
    / unreachable short-circuits to factor ``1``:

    * ``params.enabled is False`` or ``params.k == 0`` -> ``1.0`` (plain length);
    * ``d`` is :data:`math.inf` (no anchor reachable) -> ``1.0`` (no decay signal);
    * otherwise the convex formula above.

    :param d: blind-subgraph distance from the edge tail to the nearest anchor.
    :param params: the validated decay params.
    :return: the weight factor (``>= 1.0`` for legal params).
    """
    if not params.enabled or params.k == 0.0:
        return 1.0
    if math.isinf(d):
        # No anchor reachable over blind edges: no decay signal, so no penalty.
        return 1.0
    return 1.0 + params.k * (d / params.d_ref) ** params.exponent


def _weighted_copy(graph: nx.DiGraph, params: BlindDecayParams) -> nx.DiGraph:
    """Return a copy of ``graph`` with a decay-weighted ``cost`` on every edge.

    Line / projection edges get ``cost == distance_cm`` (factor ``1``). Blind
    edges get ``cost == distance_cm * (1 + k * (d(u) / d_ref) ** exponent)``
    where ``d(u)`` is the tail node's blind-distance-to-anchor. The d-map is
    computed once via :func:`compute_blind_distance_to_anchor` and skipped
    entirely when the penalty is disabled / zero-gain (factor ``1`` for every
    blind edge, so no d-map is needed).

    The copy preserves all original edge attributes (``type``,
    ``distance_cm``, ``heading_deg``) so downstream code that consumes the
    weighted graph still has the geometric information available.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Every edge must
        carry ``type`` and ``distance_cm``; nodes must carry ``type``.
    :param params: the validated :class:`BlindDecayParams`.
    :return: new :class:`networkx.DiGraph` carrying the same nodes and edges as
        ``graph``, plus a ``cost`` attribute per edge.
    """
    # The d-map is only needed when the penalty actually applies. A disabled /
    # zero-gain params object collapses every blind factor to 1, so the (one)
    # multi-source Dijkstra over the blind subgraph is skipped.
    penalty_active = params.enabled and params.k != 0.0
    d_map = compute_blind_distance_to_anchor(graph) if penalty_active else {}

    weighted = nx.DiGraph()
    # Copy nodes with their attributes so node-only graphs (e.g. the
    # fully-isolated-G fixture in the unreachable test) still iterate.
    for node, attrs in graph.nodes(data=True):
        weighted.add_node(node, **attrs)

    for u, v, data in graph.edges(data=True):
        distance_cm = float(data["distance_cm"])
        if data["type"] == EDGE_BLIND:
            # Penalty keyed on the edge TAIL u (a static potential field). An
            # unreachable / inactive tail yields factor 1 (no penalty).
            factor = _blind_decay_factor(d_map.get(u, math.inf), params)
        else:
            factor = 1.0
        cost = distance_cm * factor
        # Re-emit every original attribute alongside the new cost field.
        weighted.add_edge(u, v, cost=cost, **data)

    return weighted


# -- public API ---------------------------------------------------------------


def compute_all_pairs_next_hop(
    graph: nx.DiGraph,
    params: BlindDecayParams,
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

    Edge cost follows the blind-distance decay model (see the module
    docstring): line / projection edges cost ``distance_cm``; blind edges
    cost ``distance_cm * (1 + k * (d(u) / d_ref) ** exponent)`` keyed on the
    tail node's blind-distance-to-anchor. This function is **pivot-agnostic**:
    single-wheel-pivot derivation is decoupled into :func:`build_pivot_turns`,
    which the orchestrator calls separately. Nothing here mutates ``graph``,
    so the Dijkstra cost computed here and the ``routes.cpp`` emitter driven
    by the caller both read the same original, un-shortened distances.

    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`. Every edge must
        carry ``type`` and ``distance_cm``; nodes must carry ``type``.
        Read-only.
    :param params: the :class:`BlindDecayParams` for the blind-edge penalty.
        ``enabled=False`` or ``k == 0`` reduces the model to plain length.
    :return: nested dict ``{source: {target: next_hop | None}}``.
        Outer dict has every node as a key; each inner dict has every
        node as a key (no missing entries).
    :raises ValueError: when ``params`` is out of domain (``d_ref <= 0``,
        ``k < 0``, or ``exponent < 1``) while enabled.
    """
    _validate_blind_decay_params(params)

    # Build the weighted copy from the (unmutated) graph. Every ``cost`` is
    # derived from the original ``distance_cm``; no pivot geometry rides
    # along, so the cost model and the emitter see identical distances.
    weighted = _weighted_copy(graph, params)

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
    params: BlindDecayParams,
) -> float:
    """Return the total cost of walking ``path`` through ``graph``.

    Uses the same blind-distance decay edge-cost formula as
    :func:`compute_all_pairs_next_hop`, so tests can confirm a Dijkstra
    decision matches a hand-computed cost. A single-node path (or empty
    path) has cost ``0.0`` because no edges are traversed.

    The blind-distance-to-anchor map is computed once over the whole graph
    (not just the path) so each blind edge on the path is priced by the same
    tail-``d`` the all-pairs table used. It is skipped when the penalty is
    disabled / zero-gain (every blind edge then costs plain ``distance_cm``).

    .. note::

        This function reads ``distance_cm`` straight off the input graph.
        Pivot compensation never enters the cost: under the per-turn model
        the over-travel trim lives in the sparse :class:`PivotTurnRecord`
        table (:func:`build_pivot_turns`), not on any edge, and the
        firmware applies it at drive time -- it does not shorten the
        planned distance the Dijkstra cost optimises. This function is used
        by tests to cross-check Dijkstra against a hand-computed cost.

    :param graph: directed graph carrying ``type`` and ``distance_cm``
        on every edge and ``type`` on every node.
    :param path: ordered list of node names. Consecutive pairs must be
        connected by a directed edge in ``graph``.
    :param params: the :class:`BlindDecayParams` for the blind-edge penalty.
    :return: summed edge cost as a float.
    :raises ValueError: when ``params`` is out of domain (``d_ref <= 0``,
        ``k < 0``, or ``exponent < 1``) while enabled.
    :raises KeyError: if any consecutive ``(path[i], path[i+1])`` pair
        is not an edge in ``graph``.
    """
    _validate_blind_decay_params(params)

    if len(path) < 2:
        return 0.0

    penalty_active = params.enabled and params.k != 0.0
    d_map = compute_blind_distance_to_anchor(graph) if penalty_active else {}

    total = 0.0
    for u, v in zip(path[:-1], path[1:]):
        # graph.edges[u, v] raises KeyError on a missing edge, which is
        # exactly the contract we promise.
        data = graph.edges[u, v]
        distance_cm = float(data["distance_cm"])
        if data["type"] == EDGE_BLIND:
            factor = _blind_decay_factor(d_map.get(u, math.inf), params)
        else:
            factor = 1.0
        total += distance_cm * factor

    return total
