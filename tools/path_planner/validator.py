"""Strict validator for the path planner's field-data input (PR3 schema).

Inspects the raw module attributes (``NODES``, ``NODE_TYPES``,
``LINE_SEGMENTS``, ``BLIND_EDGES``, ``OBSTACLES``, ``FIELD_MM``,
``MAP_PNG``, optional ``EDGES``, ...) and raises :class:`FieldDataError`
on the first malformed entry with a precise key path and a one-line
nudge toward the fix.

PR3 schema (vs PR1/PR2): drops ``PROJECTIONS`` and ``GOAL_APPROACH``,
adds required ``BLIND_EDGES`` and ``FIELD_MM`` / ``MAP_PNG``.

PR4 (T3/T6) adds an optional ``EDGES`` table for rich per-edge metadata
(EDGE_PROJECTION, per-edge mode overrides, single-wheel pivot tags).
Absence of ``EDGES`` is benign for backward compatibility.

The validator performs no I/O, no logging, and mutates no global state.

Imports of :mod:`tools.path_planner.graph_builder` and
:mod:`tools.path_planner.pipeline_config` are deferred inside the EDGES
validator to avoid a circular import (``graph_builder`` imports
:func:`validate` from this module).
"""

from __future__ import annotations

import math
from typing import Any, Iterable

_REQUIRED_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
    "FIELD_MM",
    "MAP_PNG",
)

_ALLOWED_NODE_TYPES = ("line", "blind")


class FieldDataError(Exception):
    """Raised by :func:`validate` when the field-data module is malformed.

    Carries a single human-readable message that includes the offending
    key path and a hint about the fix.
    """


def _is_number(value: Any) -> bool:
    """Return True for a finite ``int`` or ``float`` (but not ``bool``).

    NaN and infinities are rejected: they slip through every downstream
    numeric gate (``x_min >= x_max`` is False when an operand is NaN,
    ``<= 0`` is False for NaN, etc.) and corrupt the planner graph with no
    diagnostic. Requiring :func:`math.isfinite` here closes all gates at the
    root. ``math.isfinite`` accepts ordinary ``int`` values unchanged.
    """
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _segment_key(a: str, b: str) -> frozenset:
    """Order-independent identity for an undirected segment."""
    return frozenset((a, b))


def _authored_out_neighbors(field_data, node: str) -> set:
    """Return the directed out-neighbors of ``node`` in the AUTHORED graph.

    Mirrors the directed-edge emission of
    :func:`tools.path_planner.graph_builder.build_graph` exactly, so the
    count of forward out-edges this function reports matches what the
    planner would see -- with one deliberate exception noted below.

    ``build_graph`` emits every undirected pair bidirectionally:

    * each ``LINE_SEGMENTS`` pair ``(a, b)`` -> ``a->b`` and ``b->a``;
    * each ``BLIND_EDGES`` pair ``(a, b)`` -> ``a->b`` and ``b->a``;
    * each ``EDGES`` entry ``{u, v}`` -> ``u->v`` and ``v->u``.

    So ``node`` has an out-edge to every line/blind neighbor, an out-edge
    to ``w`` for every EDGES entry with ``u == node``, and an out-edge to
    ``u`` (the reverse leg) for every EDGES entry with ``v == node``.
    Out-neighbors are collected into a ``set`` because ``networkx``'s
    ``DiGraph.add_edge`` keeps at most one directed edge per ordered pair:
    a node named in two source lists still contributes a single out-edge.

    .. note::

        This counts the AUTHORED topology, BEFORE obstacle / C-space
        inflation clipping. The planner's old in-line check (now removed)
        counted the POST-clip graph instead. The divergence is
        intentional: a pivot landing should be unambiguous in the graph
        the author actually wrote, independent of which edges inflation
        happens to drop. Edges silently lost to obstacle clipping are a
        separate concern, warned per finding #37; this check must not
        depend on that clipping to decide a pivot is well-formed.

    :param field_data: the field-data module (or any object exposing
        ``LINE_SEGMENTS``, ``BLIND_EDGES``, and optionally ``EDGES``).
    :param node: the node whose out-neighbors to collect.
    :return: set of node names ``node`` has a directed out-edge to.
    """
    neighbors: set = set()

    for a, b in getattr(field_data, "LINE_SEGMENTS", ()):
        if a == node:
            neighbors.add(b)
        elif b == node:
            neighbors.add(a)

    for a, b in getattr(field_data, "BLIND_EDGES", ()):
        if a == node:
            neighbors.add(b)
        elif b == node:
            neighbors.add(a)

    known_nodes = getattr(field_data, "NODES", {})
    for entry in getattr(field_data, "EDGES", ()) or ():
        if not isinstance(entry, dict):
            continue  # malformed entry; the per-entry shape check reports it.
        u = entry.get("u")
        v = entry.get("v")
        # Only count a neighbor that is a real, known node name. A later
        # malformed entry (missing or non-string ``u``/``v``) would otherwise
        # contribute a ``None`` -- or a bogus value -- that inflates an
        # EARLIER pivot landing's out-degree and raises a misleading
        # "exactly one forward out-edge" error, masking the per-entry shape
        # check that actually pinpoints the malformed entry. ``in`` on the
        # NODES dict is False for ``None`` and any unknown key, so this gate
        # keeps the count faithful to the authored topology.
        if u == node and v in known_nodes:
            neighbors.add(v)
        elif v == node and u in known_nodes:
            neighbors.add(u)

    return neighbors


# -- pivot regime classification (issue #44, T2) -----------------------------

# Over-travel budget on a LINE / sensor-terminated INCOMING side. A LINE edge
# re-anchors the chassis on the junction line, so the incoming pivot
# correction is realised as a REDUCTION of the post-detect LINE_OVERSHOOT
# advance -- valid only while the over-travel ``delta`` it cancels stays
# within that advance. The advance is the line-sensor-to-axle offset, which
# differs front (``LINE_SENSOR_AXLE_OFFSET_CM = 12.02``) vs rear
# (``LINE_SENSOR_AXLE_OFFSET_REAR_CM = 7.68``). Which board leads a hop is a
# RUNTIME direction decision the offline validator cannot see, so the only
# value safe to guarantee is the MIN of the two: a ``delta`` that fits the
# rear budget fits the front budget too. Mirrors Config.h; not single-sourced
# because the offline pipeline never reads the firmware overshoot constants
# (only ``TRACK_MM``, via ``parse_track_cm_from_config``).
_LINE_OVERSHOOT_BUDGET_CM = 7.68

# Above this absolute turn angle the over-travel formula
# ``delta = (track/2) * tan(|theta| / 2)`` blows up: ``tan`` diverges toward a
# full one-wheel reversal at 180 deg. Reject at/above this threshold rather
# than emit an unboundedly large (garbage) trim. At 179 deg the multiplier is
# already ~115x the half-track, far beyond any realisable maneuver.
_PIVOT_MAX_TURN_DEG = 179.0

# Below this absolute turn angle there is effectively no turn, so tagging the
# edge as a pivot is meaningless (and ``delta`` rounds to ~0). A "pivot" that
# does not turn is an authoring mistake -- the author meant a different edge
# or forgot to set the geometry -- so it fails rather than silently no-ops.
_PIVOT_MIN_TURN_DEG = 1.0


def _normalized_turn_deg(incoming_heading: float, outgoing_heading: float) -> float:
    """Return the signed turn angle at a pivot landing, normalized to (-180, 180].

    The turn the chassis executes at the landing node is the change of
    travel heading from the INCOMING edge (``u -> v``) to the OUTGOING edge
    (``v -> w``). Both headings follow the graph_builder convention (degrees
    in ``[0, 360)``, CCW from +x). The delta is wrapped into ``(-180, 180]``
    via the standard ``((d + 180) % 360) - 180`` trick so a turn measures
    the same regardless of how the two absolute headings straddle 0/360.
    This matches the turn-angle derivation in
    :func:`tools.path_planner.planner.apply_pivot_compensations`.

    :param incoming_heading: heading of the pivot-tagged edge ``u -> v``.
    :param outgoing_heading: heading of the single forward edge ``v -> w``.
    :return: signed turn magnitude in ``(-180, 180]`` degrees.
    """
    return ((outgoing_heading - incoming_heading + 180.0) % 360.0) - 180.0


def _resolve_authored_directed_edge(field_data, a: str, b: str):
    """Resolve the authored directed edge ``a -> b`` to ``(type, distance_cm, heading_deg, primitive_mode)``.

    Mirrors :func:`tools.path_planner.graph_builder._emit_directed_pair`'s
    emission so the validator measures the same geometry -- and the same
    regime-bearing ``primitive_mode`` -- the planner will later see. The
    resolution order matches ``build_graph``'s edge-source precedence --
    LINE_SEGMENTS, then BLIND_EDGES, then EDGES -- where a later source's
    directed edge overwrites an earlier one's attributes (networkx
    ``add_edge`` replaces). An EDGES entry therefore wins:

    * a forward leg ``{u: a, v: b}`` contributes the entry's explicit
      ``type`` / ``distance_cm`` / ``heading_deg`` verbatim;
    * a reverse leg ``{u: b, v: a}`` contributes the entry's ``type`` and
      ``distance_cm`` with the heading reversed (``+180`` mod 360), exactly
      as ``_emit_directed_pair`` emits the back-edge.

    ``primitive_mode`` is a *bidirectional* edge attribute in graph_builder
    (:data:`tools.path_planner.graph_builder._BIDIRECTIONAL_OPTIONAL_KEYS`):
    it describes the segment itself, not the entry direction, so it is
    copied onto BOTH the forward and reverse directed edges. This resolver
    surfaces it for either leg so the outgoing-side regime classification
    can see a STOP_AT_LINE override authored on the winning edge in either
    sense. It is absent (``None``) for plain LINE_SEGMENTS / BLIND_EDGES
    geometry and for any EDGES entry that does not set it.

    Falling back to LINE_SEGMENTS / BLIND_EDGES, an undirected pair
    ``{a, b}`` yields the edge type (LINE / BLIND) with distance and heading
    computed from the node coordinates (Euclidean length, ``atan2`` heading
    of ``a -> b``) and no ``primitive_mode``.

    :param field_data: the field-data module (or namespace) exposing
        ``NODES``, ``LINE_SEGMENTS``, ``BLIND_EDGES``, and optionally
        ``EDGES``.
    :param a: source node of the directed edge.
    :param b: destination node of the directed edge.
    :return: ``(edge_type, distance_cm, heading_deg, primitive_mode)`` for
        ``a -> b`` (``primitive_mode`` is ``None`` when unset), or ``None``
        if no authored edge connects the ordered pair.
    """
    nodes = getattr(field_data, "NODES", {})

    edge_type = None
    distance_cm = None
    heading_deg = None
    primitive_mode = None

    # LINE_SEGMENTS / BLIND_EDGES are undirected; compute directed geometry
    # for a -> b from node coordinates. EDGE_LINE / EDGE_BLIND string tags are
    # taken from graph_builder so the regime matrix compares like for like.
    from tools.path_planner.graph_builder import (  # local: avoid circular import
        EDGE_BLIND,
        EDGE_LINE,
    )

    for seg_a, seg_b in getattr(field_data, "LINE_SEGMENTS", ()):
        if _segment_key(seg_a, seg_b) == _segment_key(a, b):
            pa = nodes[a]
            pb = nodes[b]
            edge_type = EDGE_LINE
            distance_cm = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            heading_deg = math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0])) % 360.0
            break

    for seg_a, seg_b in getattr(field_data, "BLIND_EDGES", ()):
        if _segment_key(seg_a, seg_b) == _segment_key(a, b):
            pa = nodes[a]
            pb = nodes[b]
            edge_type = EDGE_BLIND
            distance_cm = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
            heading_deg = math.degrees(math.atan2(pb[1] - pa[1], pb[0] - pa[0])) % 360.0
            break

    # EDGES wins (last writer in build_graph). Scan in order so a later entry
    # over the same ordered pair takes precedence, matching networkx replace.
    # ``primitive_mode`` is read with the SAME source-precedence/last-writer
    # semantics as type/distance/heading, on either leg -- it is a symmetric
    # (bidirectional) attribute, so a STOP_AT_LINE authored on the winning
    # edge is surfaced regardless of which sense ``a -> b`` traverses it.
    for entry in getattr(field_data, "EDGES", ()) or ():
        if not isinstance(entry, dict):
            continue  # malformed entry; the per-entry shape check reports it.
        eu = entry.get("u")
        ev = entry.get("v")
        if "type" not in entry or "distance_cm" not in entry or "heading_deg" not in entry:
            continue  # malformed entry; the per-entry shape check reports it.
        if eu == a and ev == b:
            edge_type = entry["type"]
            distance_cm = float(entry["distance_cm"])
            heading_deg = float(entry["heading_deg"])
            primitive_mode = entry.get("primitive_mode")
        elif eu == b and ev == a:
            edge_type = entry["type"]
            distance_cm = float(entry["distance_cm"])
            heading_deg = (float(entry["heading_deg"]) + 180.0) % 360.0
            primitive_mode = entry.get("primitive_mode")

    if edge_type is None:
        return None
    return edge_type, distance_cm, heading_deg, primitive_mode


def _validate_pivot_regimes(field_data) -> None:
    """Reject pivot edges whose two-sided over-travel geometry is unrealisable.

    Runs after the per-entry EDGES shape check and finding #36 (the
    single-forward-out-edge precondition), so by the time this pass touches a
    pivot edge ``u -> v`` it can rely on ``v`` having exactly one forward
    continuation ``v -> w``. For every pivot-tagged edge (``entry["pivot"]``
    is :attr:`Pivot.LEFT` / :attr:`Pivot.RIGHT`) this:

    1. derives the turn angle ``theta`` at ``v`` from the two edges' headings
       -- ``theta = normalize(heading(v->w) - heading(u->v))`` into
       ``(-180, 180]`` (see :func:`_normalized_turn_deg`);
    2. classifies each side's edge type into a *regime* -- SENSOR-terminated
       (LINE / a BLIND STOP_AT_LINE override; self-corrects on the physical
       line) vs DISTANCE-terminated (plain BLIND / PROJECTION; dead-reckoned,
       where the over-travel trim actually bites);
    3. computes the single-wheel over-travel
       ``delta = (track/2) * tan(|theta| / 2)`` (issue #44 closed form,
       :func:`tools.path_planner.planner.compute_pivot_compensation`) using
       the chassis track parsed from the firmware header; and
    4. fails fast with a precise :class:`FieldDataError` on any configuration
       the geometry cannot realise.

    Fail-fast conditions (each a distinct, precisely-worded error):

    * **Degenerate turn angle.** ``|theta|`` at/above
      :data:`_PIVOT_MAX_TURN_DEG` (a near-full one-wheel reversal, where
      ``tan`` diverges) or below :data:`_PIVOT_MIN_TURN_DEG` (no turn at all,
      so the pivot tag is meaningless).
    * **Trim exceeds a dead-reckoned drive.** On a DISTANCE-terminated side
      the trim is subtracted from that side's encoder drive, so ``delta``
      must stay strictly below the edge length -- otherwise the chassis would
      drive a negative distance. Checked on the incoming edge when ``u -> v``
      is dead-reckoned and on the outgoing edge when ``v -> w`` is.
    * **Trim exceeds the line overshoot budget.** When the INCOMING edge is
      LINE / sensor-terminated, the incoming correction is a reduction of the
      post-detect LINE_OVERSHOOT advance, valid only while
      ``delta <= `` :data:`_LINE_OVERSHOOT_BUDGET_CM` (the MIN of the front
      and rear sensor-to-axle offsets, since edge direction is a runtime
      choice the offline validator cannot see).
    * **Unsupported regime combo.** Either side's edge type fails to classify
      as sensor- or distance-terminated (an edge type outside the supported
      regime matrix), so the geometry is not modelled and the pivot is
      rejected rather than silently mis-compensated.

    The supported incoming x outgoing matrix (LINE / STOP_AT_LINE = sensor;
    BLIND-plain / PROJECTION = distance) is::

        incoming \\ outgoing | sensor                 | distance
        --------------------+------------------------+-----------------------
        LINE (re-anchor)    | overshoot-reduce in     | overshoot-reduce in
                            | out no-op               | + out trim
        BLIND/PROJECTION    | approach trim in        | approach trim in
                            | out no-op               | + out trim

    The cross-pivot turn-magnitude refinement (is the upcoming turn worth a
    pivot at all?) stays in :mod:`tools.path_planner.planner`; this pass owns
    the fail-fast on geometry the chassis cannot physically realise.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing the same top-level attributes).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``EDGES[i]`` key path and a
        one-line fix hint on the first unrealisable pivot.
    """
    edges = getattr(field_data, "EDGES", None)
    if not edges:
        return  # no authored edges -> no pivots to classify (inert on real field).

    # Deferred imports to avoid a circular import with graph_builder (which
    # imports `validate` from this module). `parse_track_cm_from_config` is the
    # single source of truth for the chassis track (issue #44): import it here
    # rather than re-reading Config.h or hard-coding a literal.
    from tools.path_planner.graph_builder import (
        EDGE_BLIND,
        EDGE_LINE,
        EDGE_PROJECTION,
        Pivot,
    )
    from tools.path_planner.pipeline_config import BlindMode
    from tools.path_planner.planner import parse_track_cm_from_config

    track_cm = parse_track_cm_from_config()

    # Regime classifier. SENSOR-terminated edges re-anchor on the physical
    # junction line (LINE always; a BLIND edge only when its per-edge
    # primitive_mode override is STOP_AT_LINE). DISTANCE-terminated edges are
    # dead-reckoned: plain BLIND and every PROJECTION. Returns "sensor",
    # "distance", or None (unclassifiable -> unsupported combo).
    def _regime(edge_type, primitive_mode) -> str | None:
        if edge_type == EDGE_LINE:
            return "sensor"
        if edge_type == EDGE_PROJECTION:
            return "distance"
        if edge_type == EDGE_BLIND:
            if isinstance(primitive_mode, BlindMode) and primitive_mode == BlindMode.STOP_AT_LINE:
                return "sensor"
            return "distance"
        return None

    for i, entry in enumerate(edges):
        if not isinstance(entry, dict):
            continue  # malformed entry; the per-entry shape check already reported it.
        pivot = entry.get("pivot")
        if not isinstance(pivot, Pivot) or pivot not in (Pivot.LEFT, Pivot.RIGHT):
            continue  # only LEFT / RIGHT pivots carry over-travel geometry.

        u = entry["u"]
        v = entry["v"]

        # Incoming edge = the pivot-tagged edge itself; its geometry is the
        # entry's explicit type / distance / heading.
        in_type = entry["type"]
        in_distance = float(entry["distance_cm"])
        in_heading = float(entry["heading_deg"])
        in_primitive = entry.get("primitive_mode")

        # Outgoing edge = the single forward out-edge from v (excluding the
        # back-edge to u). Finding #36 already guaranteed exactly one such w,
        # so this set has exactly one member by the time we get here.
        forward_out = _authored_out_neighbors(field_data, v) - {u}
        # Defensive: finding #36 guarantees |forward_out| == 1, but if a future
        # reordering ever ran this pass first, do not crash on a bad set -- let
        # finding #36 own the error message for that shape.
        if len(forward_out) != 1:
            continue
        w = next(iter(forward_out))

        resolved_out = _resolve_authored_directed_edge(field_data, v, w)
        if resolved_out is None:
            # The forward neighbor exists per _authored_out_neighbors but no
            # authored directed edge resolves v -> w. This should not happen
            # for a well-formed graph; skip rather than crash so a genuinely
            # malformed adjacency surfaces through the existing shape checks.
            continue
        out_type, out_distance, out_heading, out_primitive = resolved_out

        # 1. Degenerate turn angle -----------------------------------------
        theta = _normalized_turn_deg(in_heading, out_heading)
        abs_theta = abs(theta)
        if abs_theta >= _PIVOT_MAX_TURN_DEG or abs_theta < _PIVOT_MIN_TURN_DEG:
            raise FieldDataError(
                f"EDGES[{i}] pivot on edge ('{u}','{v}') has a degenerate turn "
                f"angle theta={theta:.2f} deg into continuation '{w}' (must be "
                f"in [{_PIVOT_MIN_TURN_DEG}, {_PIVOT_MAX_TURN_DEG}) deg). A "
                f"near-180 deg turn makes the single-wheel over-travel "
                f"(track/2)*tan(theta/2) diverge, and a ~0 deg turn is no pivot "
                f"at all -- fix the headings of ('{u}','{v}') / ('{v}','{w}') "
                f"or drop the pivot tag."
            )

        # 4. Unsupported incoming x outgoing combo -------------------------
        # Classify both sides symmetrically: a STOP_AT_LINE override makes a
        # BLIND edge sensor-terminated on EITHER side. The outgoing edge's
        # primitive_mode IS in scope (AC7) -- a STOP_AT_LINE continuation
        # self-corrects on the line and must take no outgoing trim, so it must
        # classify "sensor" and skip the delta >= out_distance check below.
        # _resolve_authored_directed_edge surfaces that override (it is a
        # bidirectional attribute in graph_builder) with the same
        # last-writer precedence it uses for type/distance/heading.
        in_regime = _regime(in_type, in_primitive)
        out_regime = _regime(out_type, out_primitive)
        if in_regime is None or out_regime is None:
            raise FieldDataError(
                f"EDGES[{i}] pivot on edge ('{u}','{v}') has an unsupported "
                f"regime combo: incoming type {in_type!r} x outgoing type "
                f"{out_type!r} (continuation '{w}'). The two-sided pivot "
                f"compensation only models SENSOR-terminated (LINE / BLIND "
                f"STOP_AT_LINE) and DISTANCE-terminated (BLIND / PROJECTION) "
                f"edges -- reclassify the offending edge or drop the pivot tag."
            )

        delta = (track_cm / 2.0) * math.tan(math.radians(abs_theta / 2.0))

        # 2. delta >= edge length on a DISTANCE-terminated side ------------
        # The trim is subtracted from a dead-reckoned drive, so it cannot
        # reach or exceed that drive's length (the chassis cannot travel a
        # non-positive distance). Check incoming and outgoing independently.
        if in_regime == "distance" and delta >= in_distance:
            raise FieldDataError(
                f"EDGES[{i}] pivot on edge ('{u}','{v}') over-travels "
                f"delta={delta:.3f} cm, which is >= the INCOMING dead-reckoned "
                f"edge length {in_distance:.3f} cm (type {in_type!r}). The "
                f"approach trim cannot meet or exceed the drive it shortens -- "
                f"lengthen ('{u}','{v}'), soften the turn, or drop the pivot tag."
            )
        if out_regime == "distance" and delta >= out_distance:
            raise FieldDataError(
                f"EDGES[{i}] pivot on edge ('{u}','{v}') over-travels "
                f"delta={delta:.3f} cm into continuation '{w}', which is >= the "
                f"OUTGOING dead-reckoned edge length {out_distance:.3f} cm "
                f"(type {out_type!r}). The out trim cannot meet or exceed the "
                f"drive it shortens -- lengthen ('{v}','{w}'), soften the turn, "
                f"or drop the pivot tag."
            )

        # 3. delta > line-incoming overshoot budget ------------------------
        # A LINE / sensor-terminated INCOMING edge realises its correction as
        # a reduction of the post-detect LINE_OVERSHOOT advance, valid only
        # while delta stays within the (rear) sensor-to-axle budget.
        if in_regime == "sensor" and delta > _LINE_OVERSHOOT_BUDGET_CM:
            raise FieldDataError(
                f"EDGES[{i}] pivot on edge ('{u}','{v}') over-travels "
                f"delta={delta:.3f} cm, which exceeds the LINE overshoot budget "
                f"{_LINE_OVERSHOOT_BUDGET_CM:.2f} cm (min of front 12.02 / rear "
                f"7.68 sensor-to-axle offset; edge direction is a runtime "
                f"choice). A sensor-terminated incoming edge can only absorb a "
                f"trim up to its overshoot advance -- soften the turn or drop "
                f"the pivot tag on ('{u}','{v}')."
            )


def _validate_edge_list(
    edges: Iterable,
    nodes: dict,
    label: str,
) -> None:
    """Validate that every entry is a 2-tuple of known node names."""
    for i, edge in enumerate(edges):
        if not isinstance(edge, tuple) or len(edge) != 2:
            raise FieldDataError(
                f"{label}[{i}] must be a 2-tuple of node names; got: {edge!r}"
            )
        a, b = edge
        if a == b:
            raise FieldDataError(
                f"{label}[{i}] is a self-loop ('{a}','{b}'); remove or fix"
            )
        if a not in nodes:
            raise FieldDataError(
                f"{label} contains unknown node: '{a}' "
                f"(entry {i}: ('{a}', '{b}')) -- add to NODES or fix"
            )
        if b not in nodes:
            raise FieldDataError(
                f"{label} contains unknown node: '{b}' "
                f"(entry {i}: ('{a}', '{b}')) -- add to NODES or fix"
            )


def _validate_edges_field(field_data) -> None:
    """Validate the optional ``field_data.EDGES`` rich-edge table.

    Runs only when ``field_data`` exposes an ``EDGES`` attribute. Each
    entry must be a dict carrying required keys (``type``, ``u``, ``v``,
    ``distance_cm``, ``heading_deg``) and may carry optional keys
    (``primitive_mode``, ``pre_turn_mode``, ``pivot``). Mode overrides
    are cross-checked against the edge ``type`` -- e.g. a
    :class:`LineMode` is illegal on an :data:`EDGE_BLIND` edge -- and
    the :class:`DEFAULT <pipeline_config.LineMode>` sentinel is rejected
    on any override (DEFAULT == "absent", so being explicit about it is
    a confusion).

    An anti-ambiguity cross-check rejects an entry whose unordered pair
    ``{u, v}`` already exists in ``LINE_SEGMENTS`` or ``BLIND_EDGES``
    with a *conflicting* type (:data:`EDGE_LINE` matches LINE_SEGMENTS,
    :data:`EDGE_BLIND` matches BLIND_EDGES, :data:`EDGE_PROJECTION`
    matches neither): such an entry would silently overwrite the earlier
    edge's type and geometry in :mod:`graph_builder`. A *matching*-type
    entry on an existing pair is a legitimate override layer and is
    accepted.

    The pivot out-degree check (finding #36) lives here: a pivot-tagged
    entry's destination ``v`` must have exactly one forward out-edge in
    the AUTHORED adjacency (excluding the back-edge to the pivot's source
    ``u``), otherwise pivot compensation in
    :mod:`tools.path_planner.planner` is ambiguous. The check counts the
    authored topology before obstacle / C-space inflation, so a bad pivot
    fails as a clean :class:`FieldDataError` at validation time rather
    than as an uncaught ``ValueError`` from the planner at build time.

    The cross-edge pivot-geometry refinement (does the upcoming turn
    delta exceed the minimum useful angle?) is still owned by
    :mod:`tools.path_planner.planner` (T5); this validator checks the
    per-entry shape and the pivot landing's out-degree.

    Imports of :class:`Pivot`, :class:`LineMode`, :class:`BlindMode`,
    :class:`ProjectionMode`, and :class:`TurnMode` are performed inside
    the function to avoid a circular import with
    :mod:`tools.path_planner.graph_builder`.

    :param field_data: the ``tools.path_planner.field_data`` module (or
        any object exposing the same top-level attributes).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``EDGES[i]`` key path and
        one-line fix hint on the first malformed entry.
    """
    edges = getattr(field_data, "EDGES", None)
    if edges is None:
        return  # backward-compatible: missing EDGES is fine.

    if not isinstance(edges, (list, tuple)):
        raise FieldDataError(
            "EDGES must be a list/tuple of dicts; "
            f"got: {type(edges).__name__} -- "
            "define EDGES = [ {...}, {...}, ... ] in field_data.py"
        )

    # Deferred imports to avoid circular import via graph_builder, which
    # itself imports `validate` from this module.
    from tools.path_planner.graph_builder import (
        EDGE_BLIND,
        EDGE_LINE,
        EDGE_PROJECTION,
        Pivot,
    )
    from tools.path_planner.pipeline_config import (
        BlindMode,
        LineMode,
        ProjectionMode,
        TurnMode,
    )

    legal_edge_types = (EDGE_LINE, EDGE_BLIND, EDGE_PROJECTION)
    mode_class_by_edge_type = {
        EDGE_LINE: (LineMode, "LineMode"),
        EDGE_BLIND: (BlindMode, "BlindMode"),
        EDGE_PROJECTION: (ProjectionMode, "ProjectionMode"),
    }
    nodes = getattr(field_data, "NODES", {})

    # Anti-ambiguity cross-check (findings #38/#47): an EDGES entry whose
    # unordered pair {u, v} already exists in LINE_SEGMENTS or BLIND_EDGES
    # with a *conflicting* type silently overwrites the earlier edge's type
    # and geometry in graph_builder (LINE then BLIND then EDGES; networkx
    # add_edge replaces attributes). EDGE_LINE matches LINE_SEGMENTS and
    # EDGE_BLIND matches BLIND_EDGES; EDGE_PROJECTION matches neither, so a
    # PROJECTION entry on a LINE/BLIND pair is a conflict. A *matching*-type
    # entry (e.g. EDGE_LINE on a LINE_SEGMENTS pair) is a legitimate override
    # layer (distance/heading/pivot/direction) and is accepted. This mirrors,
    # via EDGES, the LINE-vs-BLIND guard in :func:`validate`.
    line_segment_keys = {
        _segment_key(a, b)
        for a, b in getattr(field_data, "LINE_SEGMENTS", ())
    }
    blind_edge_keys = {
        _segment_key(a, b)
        for a, b in getattr(field_data, "BLIND_EDGES", ())
    }

    for i, entry in enumerate(edges):
        if not isinstance(entry, dict):
            raise FieldDataError(
                f"EDGES[{i}] must be a dict; got: {entry!r} -- "
                "each entry needs at least 'type', 'u', 'v', "
                "'distance_cm', 'heading_deg'"
            )

        # Required keys ----------------------------------------------------
        for required_key in ("type", "u", "v", "distance_cm", "heading_deg"):
            if required_key not in entry:
                raise FieldDataError(
                    f"EDGES[{i}] missing required key '{required_key}' -- "
                    "each entry needs 'type', 'u', 'v', 'distance_cm', "
                    "and 'heading_deg'"
                )

        edge_type = entry["type"]
        u = entry["u"]
        v = entry["v"]
        distance_cm = entry["distance_cm"]
        heading_deg = entry["heading_deg"]

        if edge_type not in legal_edge_types:
            raise FieldDataError(
                f"EDGES[{i}].type = {edge_type!r}; expected one of "
                f"{legal_edge_types} -- use EDGE_LINE, EDGE_BLIND, or "
                "EDGE_PROJECTION from graph_builder"
            )

        if u == v:
            raise FieldDataError(
                f"EDGES[{i}] is a self-loop ('{u}','{v}'); "
                "remove the entry or fix one of the endpoints"
            )

        if u not in nodes:
            raise FieldDataError(
                f"EDGES[{i}] references unknown node u='{u}' -- "
                "add it to NODES or fix the entry"
            )

        if v not in nodes:
            raise FieldDataError(
                f"EDGES[{i}] references unknown node v='{v}' -- "
                "add it to NODES or fix the entry"
            )

        # Conflicting-type cross-check (findings #38/#47). A matching-type
        # override is allowed; only a type that reclassifies an existing
        # LINE/BLIND pair (or a PROJECTION entry over either) is rejected.
        pair_key = _segment_key(u, v)
        if pair_key in line_segment_keys and edge_type != EDGE_LINE:
            raise FieldDataError(
                f"EDGES[{i}] ('{u}','{v}') is type {edge_type} but the pair "
                "is already a LINE_SEGMENTS entry -- this silently overwrites "
                "the line edge's type and geometry in graph_builder. Use "
                "EDGE_LINE to override the existing line edge, or remove the "
                "pair from LINE_SEGMENTS if it is not line-reachable"
            )
        if pair_key in blind_edge_keys and edge_type != EDGE_BLIND:
            raise FieldDataError(
                f"EDGES[{i}] ('{u}','{v}') is type {edge_type} but the pair "
                "is already a BLIND_EDGES entry -- this silently overwrites "
                "the blind edge's type and geometry in graph_builder. Use "
                "EDGE_BLIND to override the existing blind edge, or remove the "
                "pair from BLIND_EDGES if it is not blind-reachable"
            )

        if not _is_number(distance_cm) or distance_cm <= 0:
            raise FieldDataError(
                f"EDGES[{i}].distance_cm must be a positive number; "
                f"got: {distance_cm!r} -- use a value > 0 in centimetres"
            )

        if (
            not _is_number(heading_deg)
            or not (0.0 <= float(heading_deg) < 360.0)
        ):
            raise FieldDataError(
                f"EDGES[{i}].heading_deg must be a number in [0, 360); "
                f"got: {heading_deg!r} -- normalize via "
                "`heading % 360.0`"
            )

        # Optional: primitive_mode ----------------------------------------
        # Any concrete member of the edge type's mode enum is accepted --
        # for BLIND edges this includes BlindMode.STOP_AT_LINE. Only the
        # DEFAULT sentinel (== "absent") and values of the wrong enum type
        # (or no enum type at all) are rejected.
        if "primitive_mode" in entry:
            primitive_mode = entry["primitive_mode"]
            expected_cls, expected_name = mode_class_by_edge_type[edge_type]
            if not isinstance(primitive_mode, expected_cls):
                raise FieldDataError(
                    f"EDGES[{i}].primitive_mode = {primitive_mode!r} -- "
                    f"type {edge_type} requires a {expected_name} instance "
                    "(LineMode on LINE edges, BlindMode on BLIND edges, "
                    "ProjectionMode on PROJECTION edges)"
                )
            if primitive_mode == expected_cls.DEFAULT:
                raise FieldDataError(
                    f"EDGES[{i}].primitive_mode = {expected_name}.DEFAULT -- "
                    "DEFAULT means 'inherit PIPELINE_CONFIG', which is "
                    "what absence already encodes; omit the override or "
                    "pick a concrete mode"
                )

        # Optional: pre_turn_mode -----------------------------------------
        if "pre_turn_mode" in entry:
            pre_turn_mode = entry["pre_turn_mode"]
            if not isinstance(pre_turn_mode, TurnMode):
                raise FieldDataError(
                    f"EDGES[{i}].pre_turn_mode = {pre_turn_mode!r} -- "
                    "must be a TurnMode instance "
                    "(COUNTER_ROT, PIVOT_LEFT, or PIVOT_RIGHT)"
                )
            if int(pre_turn_mode) == 0:
                raise FieldDataError(
                    f"EDGES[{i}].pre_turn_mode = TurnMode.DEFAULT -- "
                    "DEFAULT means 'inherit PIPELINE_CONFIG.turn', which "
                    "is what absence already encodes; omit the override "
                    "or pick a concrete mode"
                )

        # Optional: direction --------------------------------------------
        # Per-directed-edge traversal override (asymmetric -- applies only
        # to the forward u -> v edge, never mirrored to the reverse).
        # Allowed values: "front" (FORCE_FORWARD), "rear" (FORCE_REVERSE),
        # or absent (DEFAULT / inherit PIPELINE_CONFIG.direction).
        if "direction" in entry:
            direction = entry["direction"]
            if direction not in ("front", "rear"):
                raise FieldDataError(
                    f"EDGES[{i}].direction must be 'front', 'rear', or "
                    f"absent; got {direction!r}"
                )

        # Optional: pivot -------------------------------------------------
        if "pivot" in entry:
            pivot = entry["pivot"]
            if not isinstance(pivot, Pivot):
                raise FieldDataError(
                    f"EDGES[{i}].pivot = {pivot!r} -- "
                    "must be a Pivot instance (Pivot.LEFT, Pivot.RIGHT, "
                    "or Pivot.NONE; though NONE is redundant since "
                    "absence already encodes it)"
                )
            # NONE is allowed but redundant -- do not reject it as
            # DEFAULT-sentinel, because Pivot.NONE is the meaningful
            # "counter-rotation" value, not a placeholder.
            #
            # Pivot out-degree check (finding #36). A single-wheel pivot
            # entering 'v' is compensated by shortening 'v's ONE forward
            # continuation; the planner cannot pick among several before
            # Dijkstra runs, so it requires 'v' to have exactly one forward
            # out-edge (excluding the back-edge to the pivot's source 'u',
            # which the chassis never immediately reverses). This used to be
            # a raw ValueError raised from planner.apply_pivot_compensations
            # at build time -- an uncaught traceback rather than a clean
            # data error. It now lives here so a bad pivot fails as a
            # FieldDataError at validation time (finding #36 decision B).
            #
            # The count is taken over the AUTHORED adjacency, BEFORE
            # obstacle / C-space inflation clipping (the planner counted the
            # post-clip graph). This is intentional: a pivot landing must be
            # unambiguous in the graph the author wrote, not contingent on
            # which edges inflation drops. Edges lost to clipping are warned
            # separately (finding #37).
            if pivot in (Pivot.LEFT, Pivot.RIGHT):
                forward_out = _authored_out_neighbors(field_data, v) - {u}
                if len(forward_out) != 1:
                    raise FieldDataError(
                        f"EDGES[{i}] pivot compensation requires destination "
                        f"node '{v}' (entered from '{u}') to have exactly one "
                        f"forward out-edge, got {len(forward_out)}. "
                        "Restructure the graph so pivot landings are "
                        f"unambiguous, or drop the pivot tag on edge "
                        f"('{u}', '{v}')."
                    )

    # Two-sided pivot over-travel geometry check (issue #44, T2). Runs after
    # the full per-entry loop above so every entry has cleared its shape check
    # and finding #36's single-forward-out-edge precondition first -- the
    # regime pass relies on that uniqueness to read the post-turn heading.
    _validate_pivot_regimes(field_data)


def validate(field_data) -> None:
    """Raise :class:`FieldDataError` on the first problem; return ``None`` on success.

    :param field_data: The ``tools.path_planner.field_data`` module (or any
        object exposing the same top-level attributes).
    :raises FieldDataError: with a precise key path and one-line fix hint.
    """
    # 1. Structural presence of every required attribute.
    for attr in _REQUIRED_ATTRS:
        if not hasattr(field_data, attr):
            raise FieldDataError(
                f"missing required attribute: {attr} -- "
                f"add `{attr} = ...` to field_data.py"
            )

    nodes = getattr(field_data, "NODES")
    node_types = getattr(field_data, "NODE_TYPES")
    line_segments = getattr(field_data, "LINE_SEGMENTS")
    blind_edges = getattr(field_data, "BLIND_EDGES")
    obstacles = getattr(field_data, "OBSTACLES")
    robot_radius = getattr(field_data, "ROBOT_RADIUS_CM")
    cost_blind_mul = getattr(field_data, "COST_BLIND_MULTIPLIER")
    start_node = getattr(field_data, "START_NODE")
    start_heading = getattr(field_data, "START_HEADING_DEG")
    field_mm = getattr(field_data, "FIELD_MM")
    map_png = getattr(field_data, "MAP_PNG")

    # 2. NODES coords.
    if not isinstance(nodes, dict):
        raise FieldDataError(
            "NODES must be a dict mapping node-name -> (x, y); "
            f"got: {type(nodes).__name__}"
        )
    for name, coord in nodes.items():
        if not isinstance(name, str) or not name:
            raise FieldDataError(
                f"NODES key {name!r} must be a non-empty string"
            )
        # The header writer emits each name verbatim into a C++ string
        # literal (`NodeInfo.name = "<name>"`). A double-quote, backslash, or
        # any non-printable/control char (newline, tab, ...) would produce a
        # malformed literal that fails the Arduino build on the auto-generated
        # routes.cpp -- far from the field-data root cause. Reject those here
        # so the error names the offending node at the right layer.
        if not name.isprintable() or '"' in name or "\\" in name:
            raise FieldDataError(
                f"NODES key {name!r} contains a disallowed character "
                "(double-quote, backslash, or non-printable/control char) -- "
                "node names are emitted into C++ string literals, so use only "
                "printable characters without quotes or backslashes"
            )
        if (
            not isinstance(coord, tuple)
            or len(coord) != 2
            or not _is_number(coord[0])
            or not _is_number(coord[1])
        ):
            raise FieldDataError(
                f"NODES['{name}'] must be (x, y); got: {coord!r} -- "
                "use a 2-tuple of numbers like (12.5, 34.0)"
            )

    # 3. NODE_TYPES coverage and values.
    if not isinstance(node_types, dict):
        raise FieldDataError(
            "NODE_TYPES must be a dict mapping node-name -> type-string; "
            f"got: {type(node_types).__name__}"
        )
    for name in node_types:
        if name not in nodes:
            raise FieldDataError(
                f"NODE_TYPES references unknown node: '{name}' -- "
                "add it to NODES or remove the entry from NODE_TYPES"
            )
    for name in nodes:
        if name not in node_types:
            raise FieldDataError(
                f"node '{name}' missing from NODE_TYPES -- "
                "every NODES key needs a type ('line' or 'blind')"
            )
    for name, value in node_types.items():
        if value not in _ALLOWED_NODE_TYPES:
            raise FieldDataError(
                f"NODE_TYPES['{name}'] = '{value}'; expected one of "
                "{'line','blind'}"
            )

    # 4. LINE_SEGMENTS coverage.
    if not isinstance(line_segments, (list, tuple)):
        raise FieldDataError(
            "LINE_SEGMENTS must be a list of (a, b) node-name pairs; "
            f"got: {type(line_segments).__name__}"
        )
    _validate_edge_list(line_segments, nodes, "LINE_SEGMENTS")

    # 5. BLIND_EDGES coverage.
    if not isinstance(blind_edges, (list, tuple)):
        raise FieldDataError(
            "BLIND_EDGES must be a list of (a, b) node-name pairs; "
            f"got: {type(blind_edges).__name__}"
        )
    _validate_edge_list(blind_edges, nodes, "BLIND_EDGES")

    # Cross-check: a pair should not appear in both LINE_SEGMENTS and
    # BLIND_EDGES (ambiguous semantics).
    line_keys = {_segment_key(a, b) for a, b in line_segments}
    for a, b in blind_edges:
        if _segment_key(a, b) in line_keys:
            raise FieldDataError(
                f"BLIND_EDGES ('{a}','{b}') duplicates a LINE_SEGMENTS entry "
                "-- a pair is either line- or blind-reachable, not both"
            )

    # 6. OBSTACLES shape.
    if not isinstance(obstacles, (list, tuple)):
        raise FieldDataError(
            "OBSTACLES must be a list of (x_min, y_min, x_max, y_max, kind) "
            f"tuples; got: {type(obstacles).__name__}"
        )
    for i, obs in enumerate(obstacles):
        if not isinstance(obs, tuple) or len(obs) != 5:
            raise FieldDataError(
                f"OBSTACLES[{i}] must be a 5-tuple "
                f"(x_min, y_min, x_max, y_max, kind); got: {obs!r}"
            )
        x_min, y_min, x_max, y_max, kind = obs
        for label, num in (
            ("x_min", x_min), ("y_min", y_min),
            ("x_max", x_max), ("y_max", y_max),
        ):
            if not _is_number(num):
                raise FieldDataError(
                    f"OBSTACLES[{i}].{label} must be a number; got: {num!r}"
                )
        if not isinstance(kind, str):
            raise FieldDataError(
                f"OBSTACLES[{i}].kind must be a string label; got: {kind!r}"
            )
        if x_min >= x_max or y_min >= y_max:
            raise FieldDataError(
                f"OBSTACLES[{i}] has invalid bounds: x_min >= x_max or "
                f"y_min >= y_max -- got {obs!r}"
            )

    # 7. Scalars.
    if not _is_number(robot_radius) or robot_radius <= 0:
        raise FieldDataError(
            f"ROBOT_RADIUS_CM must be a positive number; got: {robot_radius!r}"
        )
    if not _is_number(cost_blind_mul) or cost_blind_mul <= 0:
        raise FieldDataError(
            "COST_BLIND_MULTIPLIER must be a positive number; "
            f"got: {cost_blind_mul!r}"
        )
    if not isinstance(start_node, str):
        raise FieldDataError(
            f"START_NODE must be a node-name string; got: {start_node!r}"
        )
    if start_node not in nodes:
        raise FieldDataError(
            f"START_NODE references unknown node: '{start_node}' -- "
            "add it to NODES or pick an existing node"
        )
    if not _is_number(start_heading) or not (0.0 <= float(start_heading) < 360.0):
        raise FieldDataError(
            "START_HEADING_DEG must be a number in [0, 360); "
            f"got: {start_heading!r}"
        )

    # 8. Visualizer scaling.
    if (
        not isinstance(field_mm, tuple)
        or len(field_mm) != 2
        or not _is_number(field_mm[0])
        or not _is_number(field_mm[1])
        or field_mm[0] <= 0
        or field_mm[1] <= 0
    ):
        raise FieldDataError(
            "FIELD_MM must be a (width_mm, height_mm) tuple of positive "
            f"numbers; got: {field_mm!r}"
        )
    if not isinstance(map_png, str) or not map_png:
        raise FieldDataError(
            f"MAP_PNG must be a non-empty path string; got: {map_png!r}"
        )

    # 9. Optional rich-edge table (PR4 / T6). Absent on legacy fixtures.
    _validate_edges_field(field_data)
