"""Strict validator for the path planner's field-data input (PR3 schema).

Inspects the raw module attributes (``NODES``, ``NODE_TYPES``,
``LINE_SEGMENTS``, ``BLIND_EDGES``, ``OBSTACLES``, ``FIELD_MM``,
``MAP_PNG``, the ``BLIND_DECAY_*`` decay knobs, optional ``EDGES``,
optional ``PIVOT_TURNS``, optional ``RESET_NODES``, ...) and raises
:class:`FieldDataError` on the first malformed entry with a precise key
path and a one-line nudge toward the fix.

Wall-zero reset nodes are authored in a sparse top-level ``RESET_NODES``
table keyed by node name (``{"axis", "wall_coord", "facing"}`` per node),
mirroring the ``PIVOT_TURNS`` precedent. A ``"reset"``-typed node and its
``RESET_NODES`` entry must agree both directions; the geometry fail-fasts
live in :func:`_validate_reset_nodes`.

PR3 schema (vs PR1/PR2): drops ``PROJECTIONS`` and ``GOAL_APPROACH``,
adds required ``BLIND_EDGES`` and ``FIELD_MM`` / ``MAP_PNG``.

PR4 (T3/T6) adds an optional ``EDGES`` table for rich per-edge metadata
(per-edge mode / direction / stop-at-line overrides). Absence of ``EDGES``
is benign for backward compatibility.

Issue #64 retires the ``EDGE_PROJECTION`` edge type: a line-follow segment
terminated by encoder distance (no junction under the strip) is now
expressed by typing the *destination* node ``"projection"``. Edge types are
``LINE`` / ``BLIND`` only, and every per-hop primitive falls out of
``(edge_type, destination_node_type)``.

Single-wheel pivots are authored **per turn**, not per edge: a pivot is a
3-node triple ``(u, v, w)`` in ``field_data.PIVOT_TURNS`` (arrive from
``u``, pivot at ``v``, leave to ``w``). The per-turn geometry fail-fasts
live in :func:`_validate_pivot_turns`; the old per-edge ``pivot`` /
``pre_turn_mode`` ``EDGES`` keys are gone (the validator no longer reads
them).

The validator performs no I/O on mutable state and mutates no global
state. It does read two firmware headers (``WRO2026/Config.h`` for the
single-sourced over-travel constants and the chassis track) so the
offline pivot checks stay in lockstep with the values the robot drives
with -- mirroring :func:`tools.path_planner.planner.parse_track_cm_from_config`.

Imports of :mod:`tools.path_planner.graph_builder` and
:mod:`tools.path_planner.pipeline_config` are deferred inside the
EDGES / pivot validators to avoid a circular import (``graph_builder``
imports :func:`validate` from this module).
"""

from __future__ import annotations

import math
import warnings
from pathlib import Path
from typing import Any, Iterable, Optional, Tuple

from tools.path_planner.config_parse import DEFAULT_CONFIG_H_PATH, parse_config_float

_REQUIRED_ATTRS = (
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

_ALLOWED_NODE_TYPES = ("line", "projection", "blind", "reset")

# The off-line node kind: a node with no line under the sensor, reachable only
# by dead reckoning. A LINE (line-follow) edge can never be incident to one --
# you cannot line-follow to a node that has no line -- so the validator rejects
# such an edge (see :func:`_reject_line_follow_into_blind`).
_BLIND_NODE_TYPE = "blind"

# A line-follow waypoint that is on the line but NOT at a junction. The line
# passes through it (longitudinally detectable as a crossing), so a BLIND edge
# can still stop on it; there is just no junction marker, hence the hop into it
# is dead-reckoned for identity/distance. A LINE edge into a projection node is
# DISTANCE-terminated (no junction to overshoot), which flips its pivot regime
# to ``distance`` (issue #64). Mirrors ``graph_builder.NODE_PROJECTION``.
_PROJECTION_NODE_TYPE = "projection"

# The on-line node kind reachable by line-following: it sits at a junction.
# Used by the reachability warning -- a ``line`` node needs >= 1 incident LINE
# edge (a junction must sit on a line edge).
_LINE_NODE_TYPE = "line"

# The wall-zero re-anchor node kind: the chassis dead-reckons up to it, then
# square-turns and presses a known wall to re-localize. Like a ``blind`` node it
# carries no line under the sensor (the wall press happens off the line
# backbone), so a LINE (line-follow) edge can never be incident to one -- a reset
# node must be reached by a BLIND edge (see :func:`_reject_line_follow_into_blind`,
# which rejects both blind and reset endpoints). A LINE edge into a PROJECTION
# node stays LEGAL (ADR-046); only blind and reset are off-line. Mirrors
# ``planner.NODE_TYPE_RESET``.
_RESET_NODE_TYPE = "reset"

# Inclusive upper bound on BLIND_DECAY_EXPONENT. The decay weight is
# ``length * (1 + k * (d / d_ref) ** exponent)``; with no ceiling a fat-fingered
# exponent (e.g. 400) makes ``(d / d_ref) ** exponent`` overflow to an uncaught
# OverflowError deep inside the planner. A ceiling of 8 covers linear through
# strongly-convex penalties with large headroom while keeping the weight finite
# for any sane ``d / d_ref`` ratio, so the failure surfaces here as a precise
# FieldDataError instead of a crash. Shared with the planner guard via
# :func:`validate_blind_decay_domain`.
BLIND_DECAY_EXPONENT_MAX = 8.0

# Axis labels a reset node may re-zero against (a wall parallel to one axis).
_ALLOWED_RESET_AXES = ("x", "y")

# MUST mirror Config::WALL_PIVOT_CLEARANCE_CM in WRO2026/config/ConfigNav.h (the
# pivot sweep radius + safety margin). The firmware cannot include this header, so
# the value is hand-mirrored here for the offline authoring guardrail; keep the two
# in lockstep.
WALL_PIVOT_CLEARANCE_CM = 14.0

# Per-edge pivot authoring keys removed by the per-turn model (issue #44, T5).
# A field that still carries one of these on an EDGES entry pre-dates the
# migration: the key is silently ignored by the per-turn validator and -- worse
# -- the stray ``pivot`` would crash ``graph_builder._extract_directional_attrs``
# with a raw ``ValueError`` (``int(...)`` on a string) when the authored graph is
# built. The EDGES shape check rejects these up front with a precise
# :class:`FieldDataError` (single anti-ambiguity gate, ADR-029) so the raw
# ValueError is unreachable and the author is pointed at the new ``PIVOT_TURNS``
# top-level list.
_OBSOLETE_EDGE_KEYS = ("pivot", "pre_turn_mode")


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


# -- per-turn pivot validation (issue #44 per-turn model, T5) -----------------
#
# A pivot is a 3-node turn ``(u, v, w)`` authored in
# ``field_data.PIVOT_TURNS`` -- arrive along ``u -> v``, pivot the chassis at
# ``v``, depart along ``v -> w``. Geometry (turn angle, over-travel ``delta``,
# wheel) is derived offline; the validator fails fast on any turn the chassis
# cannot physically realise. This replaces the old per-edge ``EDGES["pivot"]``
# model (one trim field on one edge, plus a single-forward-out-edge landing
# rule, finding #36) -- a node ``v`` may now carry several continuations, only
# some of which are pivots, because a pivot is a property of the *turn*, not
# the edge.


# The firmware header owns the over-travel budget constants the pivot checks
# need (the line-sensor-to-axle offset, the trim-underflow floor, the minimum
# authored pivot angle). Like ``planner.py`` does for ``TRACK_MM``, this
# module reads them from ``Config.h`` via the shared, anchored
# ``config_parse.parse_config_float`` -- with a no-stale-fallback policy -- so an
# edit to ``Config.h`` can never silently desync the offline validator from the
# firmware and sim (parity test TC26 asserts these parsed values equal the header
# literals). The alias keeps this module's public parse-function signatures stable.
_CONFIG_H_PATH = DEFAULT_CONFIG_H_PATH


def parse_line_sensor_axle_offset_cm(config_path: Path = _CONFIG_H_PATH) -> float:
    """Return ``Config::LINE_SENSOR_AXLE_OFFSET_CM`` (the FRONT offset) in cm.

    A LINE-incoming pivot realises its correction as a REDUCTION of the
    post-detect ``LINE_OVERSHOOT`` advance, so the over-travel ``delta`` it
    cancels must fit within that advance -- the line-sensor-to-axle offset.

    The per-turn model is **FRONT-only** in v1 (reverse pivots are deferred to
    #54 and rejected by the validator), so the budget is the FRONT offset
    alone, not the min of front/rear the per-edge model used (which had to
    hedge against a runtime direction choice). Single-sourced from Config.h
    so a chassis remeasure cannot desync this check.

    :param config_path: path to ``Config.h`` (defaults to the firmware header).
    :return: the FRONT sensor-to-axle offset in centimetres.
    """
    return parse_config_float("LINE_SENSOR_AXLE_OFFSET_CM", config_path)


def parse_pivot_trim_min_drive_cm_from_config(
    config_path: Path = _CONFIG_H_PATH,
) -> float:
    """Return ``Config::PIVOT_TRIM_MIN_DRIVE_CM`` (firmware's trim-underflow floor) in cm.

    The firmware/sim clamp a trimmed dead-reckon drive (and the LINE overshoot
    advance) to this tiny positive floor so a bad table can never command a
    non-positive distance. The validator gates on the SAME floor: it rejects any
    pivot whose trim would leave less than this much of the edge it shortens, so
    a delta that merely satisfies ``delta < length`` yet trips the runtime clamp
    fails offline instead of overshooting on a table that validated clean. The
    sim and the parity test (TC26) read the same single-sourced constant.

    :param config_path: path to ``Config.h`` (defaults to the firmware header).
    :return: the trim-underflow minimum drive distance in centimetres.
    """
    return parse_config_float("PIVOT_TRIM_MIN_DRIVE_CM", config_path)


def parse_pivot_min_turn_deg_from_config(config_path: Path = _CONFIG_H_PATH) -> float:
    """Return ``Config::PIVOT_MIN_TURN_DEG`` -- the minimum authored pivot angle.

    The validator rejects any authored pivot turn whose geometric ``|theta|`` is
    below this floor. It is a TUNABLE Config.h knob (not derived from the
    firmware tolerance): a pivot fires at runtime only when the pre-turn maneuver
    dispatches (heading delta ``> NAV_TURN_TOLERANCE_DEG``, PathTracker), so a
    too-shallow turn would silently no-op the pivot while its ``delta`` trim is
    still subtracted. Single-sourced here so a retune of the floor cannot desync
    the offline check from the firmware header (TC26 parity).

    :param config_path: path to ``Config.h`` (defaults to the firmware header).
    :return: the minimum authored pivot turn angle in degrees.
    """
    return parse_config_float("PIVOT_MIN_TURN_DEG", config_path)


# Above this absolute turn angle the over-travel formula
# ``delta = (track/2) * tan(|theta| / 2)`` blows up: ``tan`` diverges toward a
# full one-wheel reversal at 180 deg. Reject at/above this threshold rather
# than emit an unboundedly large (garbage) trim. At 179 deg the multiplier is
# already ~115x the half-track, far beyond any realisable maneuver. A pure
# literal (no Config.h read), so it is a module constant.
_PIVOT_MAX_TURN_DEG = 179.0


def _build_authored_graph(field_data):
    """Build the AUTHORED directed graph the per-turn pivot checks read from.

    **Graph-availability approach (the riskiest design call in T5).**
    ``directed_edge_heading`` -- the single reusable source for directed-edge
    geometry (T3) -- needs a built graph. But :func:`validate` runs *inside*
    :func:`tools.path_planner.graph_builder.build_graph`, BEFORE the planner
    graph exists, so calling ``build_graph`` from here would recurse infinitely
    (``build_graph -> validate -> build_graph -> ...``). Instead this rebuilds a
    minimal graph DIRECTLY from the authored field data using ``build_graph``'s
    OWN emission helper (:func:`graph_builder._emit_directed_pair`), so the
    edges carry byte-identical ``type`` / ``distance_cm`` / ``heading_deg`` /
    ``primitive_mode`` and the same LINE -> BLIND -> EDGES last-writer
    precedence the planner will later see -- without ever re-entering
    ``validate``. This is the precedence/bidirectional resolver of review fix 2
    realised through the canonical emitter rather than a bare ``entry.get``.

    Obstacle inflation is deliberately SKIPPED: the validator judges a pivot
    against the topology the AUTHOR wrote, not against whichever edges C-space
    clipping happens to drop (the same stance the removed
    ``_authored_out_neighbors`` documented). Pivot geometry is an authoring
    property; edges lost to clipping are a separate concern (finding #37).

    :param field_data: the field-data module (or namespace) exposing
        ``NODES``, ``LINE_SEGMENTS``, ``BLIND_EDGES``, and optionally
        ``EDGES``.
    :return: a ``networkx.DiGraph`` of the authored topology (no inflation),
        with every directed edge carrying the geometry attributes
        :func:`tools.path_planner.graph_builder.directed_edge_heading` reads.
    """
    # Deferred imports to avoid a circular import: graph_builder imports
    # `validate` from this module.
    import networkx as nx

    from tools.path_planner.graph_builder import (
        EDGE_BLIND,
        EDGE_LINE,
        _emit_directed_pair,
    )

    nodes = getattr(field_data, "NODES", {})
    # Stamp the node ``type`` exactly as build_graph does (graph_builder.py:438,
    # ``type=field_data.NODE_TYPES[name]``) so the shared ``is_projection_node``
    # predicate resolves on this mini-graph identically to the real planner
    # graph. Without it the BLIND->projection-node auto-seek rule in ``_regime``
    # reads projection-ness off an untyped node and always sees False, so the
    # auto-rule is dead in the pivot/regime checks (issue #64). This is the only
    # node-creation site: every ``_emit_directed_pair`` below is guarded by
    # ``a in nodes and b in nodes``, so its ``add_edge`` calls never mint an
    # untyped endpoint node.
    node_types = getattr(field_data, "NODE_TYPES", {})
    g = nx.DiGraph()
    for name, coord in nodes.items():
        g.add_node(
            name,
            x_cm=float(coord[0]),
            y_cm=float(coord[1]),
            type=node_types.get(name),
        )

    # Match build_graph's source precedence exactly: LINE_SEGMENTS, then
    # BLIND_EDGES, then EDGES (a later source's directed edge overwrites an
    # earlier one's attributes -- networkx add_edge updates in place). No
    # obstacle-clip drop here (authored topology).
    for a, b in getattr(field_data, "LINE_SEGMENTS", ()):
        if a in nodes and b in nodes:
            _emit_directed_pair(g, a, b, nodes[a], nodes[b], EDGE_LINE)

    for a, b in getattr(field_data, "BLIND_EDGES", ()):
        if a in nodes and b in nodes:
            _emit_directed_pair(g, a, b, nodes[a], nodes[b], EDGE_BLIND)

    for entry in getattr(field_data, "EDGES", ()) or ():
        if not isinstance(entry, dict):
            continue  # malformed entry; the per-entry shape check reports it.
        eu = entry.get("u")
        ev = entry.get("v")
        etype = entry.get("type")
        if eu not in nodes or ev not in nodes:
            continue  # unknown node; the per-entry shape check reports it.
        if etype not in (EDGE_LINE, EDGE_BLIND):
            continue  # bad type; the per-entry shape check reports it.
        # Geometry is OPTIONAL on an EDGES override entry (issue #79): an
        # override-only overlay (e.g. ``{type, u, v, primitive_mode}``) carries
        # no ``distance_cm`` / ``heading_deg``. Do NOT skip such an entry --
        # ``_emit_directed_pair`` derives both from the node coords (``_euclid``
        # / ``_heading_deg`` on ``nodes[eu]`` / ``nodes[ev]``) whenever a key is
        # absent from ``options``, byte-identically to the planner graph. Earlier
        # this helper ``continue``d past geometry-less entries, which silently
        # dropped every override-only entry -- including a pivot leg -- from the
        # pivot-validation graph (the #1 trap on issue #79's T1). The dict
        # comprehension below already includes only the keys that ARE present, so
        # an absent ``distance_cm`` / ``heading_deg`` flows through as "derive
        # from coords".
        # Belt-and-suspenders for W1: forward ONLY the geometry/mode keys the
        # pivot checks read off the graph, never the whole entry. The shape
        # check (`_validate_edges_field`) already rejects a stale `pivot` /
        # `pre_turn_mode` before this runs, but narrowing the payload means a
        # stray legacy key can never reach `_extract_directional_attrs` and
        # crash on `int("left")` even if this helper is ever called on
        # unvalidated data. ``stop_at_line`` is forwarded too: it is a per-edge
        # seek trigger the pivot regime classification reads (a seeking blind
        # leg is sensor-terminated, exempt from the over-travel trim), and lands
        # on the edge as ``stop_at_line_override``.
        geometry_options = {
            key: entry[key]
            for key in (
                "distance_cm",
                "heading_deg",
                "primitive_mode",
                "direction",
                "stop_at_line",
            )
            if key in entry
        }
        _emit_directed_pair(
            g, eu, ev, nodes[eu], nodes[ev], etype, options=geometry_options
        )

    return g


def _resolve_authored_direction(field_data, a: str, b: str):
    """Return the authored ``direction`` override on the directed edge ``a -> b``.

    The per-edge ``direction`` override is asymmetric in graph_builder
    (:func:`graph_builder._emit_directed_pair` applies it to the forward
    ``u -> v`` edge only). The validator's FRONT-only guard must therefore look
    at the AUTHORED direction on each pivot-adjacent directed edge: a ``"rear"``
    authored on the exact ``u -> v`` or ``v -> w`` leg of a pivot is rejected
    (v1 is FRONT-only; reverse pivots are #54). A ``"rear"`` authored only on
    some other leg (e.g. the reverse ``b -> a``) does not constrain ``a -> b``.

    :param field_data: the field-data module/namespace.
    :param a: source node of the directed edge.
    :param b: destination node of the directed edge.
    :return: ``"front"``, ``"rear"``, or ``None`` (no authored override on the
        exact ``a -> b`` leg).
    """
    direction = None
    for entry in getattr(field_data, "EDGES", ()) or ():
        if not isinstance(entry, dict):
            continue  # malformed entry; the per-entry shape check reports it.
        # Asymmetric: the override lands on the forward (u -> v) leg only, so
        # match the entry's u/v to a/b in that exact sense. Last writer wins,
        # mirroring graph_builder's emission order.
        if entry.get("u") == a and entry.get("v") == b and "direction" in entry:
            direction = entry["direction"]
    return direction


def _validate_pivot_turns(field_data) -> None:
    """Reject per-turn pivots whose two-sided over-travel geometry is unrealisable.

    Validates every triple ``(u, v, w)`` in ``field_data.PIVOT_TURNS`` -- the
    per-turn pivot authoring that replaces the old per-edge ``EDGES["pivot"]``
    model. A pivot fires only when the route takes the turn, so ``v`` may have
    several continuations; this pass checks each authored turn in isolation.

    For each triple it:

    1. requires both legs ``u -> v`` and ``v -> w`` to be authored edges;
    2. derives the turn angle ``theta`` at ``v`` from the two directed-edge
       headings via :func:`tools.path_planner.graph_builder.directed_edge_heading`
       (T3 -- no reimplemented ``hypot`` / ``atan2`` / ``+180`` here) wrapped
       by the one canonical :func:`tools.path_planner.compiler.normalize_turn`
       (the same normalizer the planner's ``build_pivot_turns`` emits with);
    3. classifies each side's edge into a regime from
       ``(edge_type, destination_node_type)`` (issue #64) -- ``"line"`` (a LINE
       edge into a junction ``line`` node, overshoot-budget-bearing), ``"sensor"``
       (a BLIND edge that SEEKS a line at runtime -- via the legacy STOP_AT_LINE
       pipeline default, a per-edge ``stop_at_line`` override of ON, or the
       BLIND->projection-node auto-rule -- self-corrects, no trim, no budget), or
       ``"distance"`` (a non-seeking BLIND edge, OR a LINE edge into a
       ``projection`` node -- dead-reckoned, where the trim bites the follow
       distance). The seek is decided by the one shared
       :func:`tools.path_planner.sim.runtime.resolve_stop_at_line` resolver, not
       re-derived. The auto-rule is now LOCAL (the edge's OWN destination is a
       projection node), so BOTH legs evaluate it -- no goal-relative peek;
    4. computes the single-wheel over-travel
       ``delta = compute_pivot_compensation(theta, track_cm)`` (issue #44
       closed form -- reused, not re-derived); and
    5. fails fast with a precise :class:`FieldDataError` on any configuration
       the geometry cannot realise.

    Fail-fast conditions (each a distinct, precisely-worded error):

    * **Missing leg.** ``u -> v`` or ``v -> w`` is not an authored edge.
    * **Duplicate triple.** The same ``(u, v, w)`` authored twice.
    * **Degenerate / sub-gate turn angle.** ``|theta|`` at/above
      :data:`_PIVOT_MAX_TURN_DEG` (a near-full one-wheel reversal where ``tan``
      diverges) or below ``Config::PIVOT_MIN_TURN_DEG`` (the tunable floor -- the
      turn would slip under the firmware's dispatch gate).
    * **Unsupported regime combo.** Either side's edge type fails to classify.
    * **Trim exceeds a dead-reckoned drive.** On a ``"distance"`` side ``delta``
      is subtracted from that side's encoder drive, so it must stay strictly
      below the edge length. Checked on incoming and outgoing independently.
    * **Trim exceeds the sensor/LINE-incoming overshoot budget.** Applies to
      BOTH a true ``"line"`` edge AND a ``"sensor"`` (STOP_AT_LINE seek)
      incoming side: each reduces the post-detect overshoot by ``delta``,
      bounded by the FRONT sensor-to-axle offset (ADR-044).
    * **Chained pivots over-trim a shared edge.** Two triples sharing the same
      directed edge ``(v, w)`` (one departing it, one approaching it) must
      satisfy ``delta_uv + delta_vw < length(v -> w)``.
    * **Authored REAR on a pivot-adjacent edge.** v1 is FRONT-only; a
      ``direction: "rear"`` on the exact ``u -> v`` or ``v -> w`` leg is
      rejected (reverse pivots are #54).

    The supported incoming x outgoing matrix (``line`` / ``sensor`` =
    sensor-terminated; ``distance`` = dead-reckoned) is::

        incoming \\ outgoing | sensor (LINE->junction / STOP_AT_LINE) | distance (BLIND / LINE->proj)
        --------------------+----------------------------------------+------------------------------
        line / sensor       | overshoot-reduce in;                    | overshoot-reduce in;
        (re-anchor / seek)  | out no-op (self-corrects @w)            | out drive trimmed
        distance            | approach trim in;                       | approach trim in;
                            | out no-op (self-corrects @w)            | out drive trimmed

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing the same top-level attributes -- including
        ``PIVOT_TURNS``).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``PIVOT_TURNS[i]`` key path and a
        one-line fix hint on the first unrealisable pivot.
    """
    pivot_turns = getattr(field_data, "PIVOT_TURNS", None)
    if not pivot_turns:
        # Early return BEFORE touching Config.h: a pivot-free field (the real
        # field today) must not pay the per-build header parse, and must not
        # fail if a constant the pivot checks need is ever absent. This is the
        # only place the firmware constants get read.
        return

    # Deferred imports to avoid a circular import with graph_builder (which
    # imports `validate` from this module). `parse_track_cm_from_config` is the
    # single source of truth for the chassis track (issue #44);
    # `compute_pivot_compensation` is the single source of the delta formula.
    # `compiler.normalize_turn` is the ONE canonical turn normalizer the planner
    # emits with -- imported here (not at module top) because
    # `compiler -> graph_builder -> validator` would otherwise close an import
    # cycle. Using the same normalizer guarantees the validator's gating angle
    # and the planner's emitted wheel/delta are computed identically (the latent
    # +-180 drift the consolidation closes).
    from tools.path_planner.compiler import normalize_turn
    from tools.path_planner.graph_builder import (
        EDGE_BLIND,
        EDGE_LINE,
        directed_edge_heading,
        is_projection_node,
    )
    from tools.path_planner.pipeline_config import (
        PIPELINE_CONFIG,
        StopAtLineMode,
    )
    from tools.path_planner.planner import (
        compute_pivot_compensation,
        parse_track_cm_from_config,
    )
    # The ONE seek resolver (+ its enums and the chassis board count) the
    # compiler, the sim, and the firmware all share -- so a BLIND pivot leg is
    # classified seek-vs-deadreckon identically to runtime rather than by a
    # re-derived ladder that could drift. Deferred like the compiler's own use:
    # ``sim.runtime`` imports from ``compiler`` at module load, so a top-level
    # import here would close a cycle.
    from tools.path_planner.sim.runtime import (
        LINE_BOARD_COUNT,
        Direction,
        StopAtLine,
        resolve_stop_at_line,
    )

    track_cm = parse_track_cm_from_config()

    # Single-source the over-travel budget + min-angle floor from the firmware
    # header, parsed lazily HERE (not at import) so a pivot-free field never pays
    # the read (the nit: kill the per-build dead read). The LINE budget is the
    # FRONT sensor-to-axle offset (FRONT-only); the min-angle floor is the tunable
    # Config::PIVOT_MIN_TURN_DEG knob -- an authored pivot below it would slip
    # under the firmware's turn-dispatch gate and silently no-op.
    line_overshoot_budget_cm = parse_line_sensor_axle_offset_cm()
    pivot_min_turn_deg = parse_pivot_min_turn_deg_from_config()
    # The firmware/sim clamp a trimmed dead-reckon drive (and the LINE overshoot
    # advance) to this floor (PathTracker._pivotTrimDistance and the LINE_FOLLOW
    # overshoot branch; the sim mirrors both). A delta that merely satisfies
    # delta < length leaves length - delta in (0, floor), which the runtime
    # clamps -- driving the floor instead of the geometric remainder and printing
    # "Check validator/config" on a table that validated clean. Gate on the same
    # floor here so such a pivot is rejected offline.
    pivot_trim_min_drive_cm = parse_pivot_trim_min_drive_cm_from_config()

    # The authored topology graph -- built without re-entering validate (see
    # _build_authored_graph). All directed-edge geometry comes from here.
    graph = _build_authored_graph(field_data)

    # Field-wide auto-seek toggle (default ON): a blind hop whose DESTINATION is
    # a projection node seeks a line first to re-anchor. Read once; the per-leg
    # auto-rule below ANDs it with the LOCAL "is the destination a projection
    # node?" lookup (issue #64 -- no goal-relative NEXT_HOP peek).
    auto_seek_enabled = getattr(field_data, "BLIND_PROJECTION_AUTO_SEEK", True)

    # A projection node is NOT a blind-decay anchor -- planner._ANCHOR_NODE_TYPES
    # stays {line, reset} by decision; do not add projection to the anchor set.
    # (Issue #64: a projection arrival is still dead-reckoned for routing cost.)

    def _edge_attrs(a: str, b: str):
        """Return (type, distance_cm, stop_at_line_override) or None."""
        if not graph.has_edge(a, b):
            return None
        data = graph[a][b]
        return (
            data["type"],
            float(data["distance_cm"]),
            data.get("stop_at_line_override"),
        )

    def _regime(edge_type, dest_node_type, stop_override, auto_rule_active) -> str | None:
        """Classify a directed edge from ``(edge_type, destination_node_type)``.

        Returns ``'line'``, ``'sensor'``, ``'distance'``, or ``None`` (issue #64
        retires the projection EDGE type; the destination NODE type now decides
        how a LINE hop terminates):

        ``line``     -- a LINE edge into a ``line`` (junction) node: re-anchors
                        on the junction line; its incoming trim is bounded by the
                        LINE overshoot budget.
        ``distance`` -- a LINE edge into a ``projection`` node (no junction to
                        overshoot, so the hop is DISTANCE-terminated -- the
                        silent-failure trap: its trim draws from the follow
                        distance, not the overshoot budget), OR a non-seeking
                        BLIND edge: dead-reckoned; the trim bites the edge length.
        ``sensor``   -- a BLIND edge that SEEKS a line at runtime, so it self-
                        corrects: takes no trim, and is NOT subject to the
                        overshoot budget. A blind hop seeks when ANY trigger
                        fires -- a per-edge ``stop_at_line`` override of ON, the
                        legacy ``BlindMode.STOP_AT_LINE`` pipeline default, or the
                        BLIND->projection-node auto-rule (``auto_rule_active``).
                        The decision is the ONE shared
                        :func:`resolve_stop_at_line` resolver -- the same
                        precedence ladder the compiler / sim / firmware run --
                        never re-derived here, so the validator can never drift
                        from what the robot actually does.
        ``None``     -- an unclassifiable edge type (unsupported combo).

        Per-edge ``primitive_mode`` no longer feeds this classification: with the
        projection EDGE type gone, a BLIND edge's seek resolves against the
        pipeline-default :class:`BlindMode` (``PIPELINE_CONFIG.blind``) plus its
        per-edge ``stop_at_line`` override and the local auto-rule.

        Pivots are FRONT-only in v1 (reverse pivots are #54), so the seek is
        resolved with ``Direction.FRONT`` -- a front board always faces travel,
        so ``LINE_BOARD_COUNT`` never suppresses it.
        """
        if edge_type == EDGE_LINE:
            # The destination node type decides termination: a junction line
            # node re-anchors (overshoot budget), a projection node is
            # distance-terminated (follow-distance budget).
            return "distance" if dest_node_type == _PROJECTION_NODE_TYPE else "line"
        if edge_type == EDGE_BLIND:
            edge_override = (
                stop_override
                if isinstance(stop_override, StopAtLineMode)
                else StopAtLineMode.DEFAULT
            )
            seek, _motion_mode = resolve_stop_at_line(
                StopAtLine.AUTO,
                edge_override,
                PIPELINE_CONFIG.blind,
                PIPELINE_CONFIG.blind,
                Direction.FRONT,
                LINE_BOARD_COUNT,
                auto_rule_active,
            )
            return "sensor" if seek else "distance"
        return None

    # First pass: per-triple geometry + collect each triple's delta keyed by
    # its outgoing directed edge, so a second pass can sum chained pivots that
    # share an edge.
    seen_triples: set = set()
    # Map directed edge (v, w) -> list of (triple_index, delta) for every
    # pivot whose OUTGOING leg is that edge (departing it) AND every pivot
    # whose INCOMING leg is that edge (approaching it). A shared edge that
    # carries one departing and one approaching trim must fit both.
    out_edge_deltas: dict = {}
    in_edge_deltas: dict = {}

    nodes = getattr(field_data, "NODES", {})
    node_types = getattr(field_data, "NODE_TYPES", {})
    for i, triple in enumerate(pivot_turns):
        # Shape: a pivot turn is a 3-element sequence of known node names. The
        # loader (T1) already produces (u, v, w) string tuples, but the
        # validator is the anti-ambiguity gate -- fail fast with a precise
        # message rather than crash on an unpacking error or an unknown node.
        if not isinstance(triple, (tuple, list)) or len(triple) != 3:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] must be a 3-element (u, v, w) node-name "
                f"sequence; got: {triple!r}"
            )
        u, v, w = triple
        for role, name in (("u", u), ("v", v), ("w", w)):
            if name not in nodes:
                raise FieldDataError(
                    f"PIVOT_TURNS[{i}] references unknown node {role}='{name}' "
                    f"(triple {triple!r}) -- add it to NODES or fix the triple."
                )
        if u == v or v == w:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') has a degenerate leg "
                f"(u==v or v==w) -- a pivot turn needs three distinct steps; "
                f"fix the triple."
            )

        # Normalize to a hashable tuple for duplicate detection and edge keys
        # (the loader yields tuples, but a hand-built fixture may pass a list).
        triple = (u, v, w)

        # Duplicate triple --------------------------------------------------
        if triple in seen_triples:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] is a duplicate pivot turn "
                f"('{u}','{v}','{w}') -- a turn is authored once; remove the "
                f"repeat."
            )
        seen_triples.add(triple)

        # Both legs must be authored edges ----------------------------------
        in_attrs = _edge_attrs(u, v)
        if in_attrs is None:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') names an incoming edge "
                f"('{u}','{v}') that is not authored -- add it to "
                f"LINE_SEGMENTS / BLIND_EDGES / EDGES or fix the triple."
            )
        out_attrs = _edge_attrs(v, w)
        if out_attrs is None:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') names an outgoing edge "
                f"('{v}','{w}') that is not authored -- add it to "
                f"LINE_SEGMENTS / BLIND_EDGES / EDGES or fix the triple."
            )
        in_type, in_distance, in_stop_override = in_attrs
        out_type, out_distance, out_stop_override = out_attrs

        # Destination node types drive the regime (issue #64): a LINE hop into a
        # projection node is DISTANCE-terminated, a LINE hop into a junction line
        # node re-anchors. ``v`` is the head of the incoming leg ``u -> v``;
        # ``w`` is the head of the outgoing leg ``v -> w``.
        in_dest_type = node_types.get(v)
        out_dest_type = node_types.get(w)

        # FRONT-only: reject authored REAR on either pivot-adjacent leg ------
        # (review fix 4 / FRONT-only; reverse pivots are #54). Check the exact
        # u -> v and v -> w legs -- the override is asymmetric in graph_builder.
        for leg_a, leg_b, label in ((u, v, "incoming"), (v, w, "outgoing")):
            if _resolve_authored_direction(field_data, leg_a, leg_b) == "rear":
                raise FieldDataError(
                    f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') has an authored "
                    f"direction:'rear' on its {label} edge ('{leg_a}','{leg_b}'), "
                    f"but single-wheel pivots are FRONT-only in v1 (reverse "
                    f"pivots are tracked by #54). Drop the 'rear' override on "
                    f"that edge or remove the pivot turn."
                )

        # 1. Degenerate / sub-gate turn angle -------------------------------
        in_heading = directed_edge_heading(graph, u, v)
        out_heading = directed_edge_heading(graph, v, w)
        # ONE canonical normalizer: the planner's build_pivot_turns derives its
        # emitted wheel/delta from compiler.normalize_turn (yields (-180, 180]).
        # Computing theta the same way here keeps the validator's gating angle
        # and the planner's emitted geometry byte-identical at every angle --
        # including +-180, where a divergent local wrap would flip the wheel sign.
        theta = normalize_turn(out_heading - in_heading)
        abs_theta = abs(theta)
        if abs_theta >= _PIVOT_MAX_TURN_DEG or abs_theta < pivot_min_turn_deg:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') has a degenerate turn "
                f"angle theta={theta:.2f} deg (must be in "
                f"[{pivot_min_turn_deg:.2f}, {_PIVOT_MAX_TURN_DEG}) deg). A "
                f"near-180 deg turn makes the single-wheel over-travel "
                f"(track/2)*tan(theta/2) diverge, and a sub-{pivot_min_turn_deg:.0f} "
                f"deg turn would slip under the firmware turn-dispatch gate -- "
                f"fix the headings of ('{u}','{v}') / ('{v}','{w}') or remove "
                f"the pivot turn."
            )

        # 2. Unsupported incoming x outgoing combo --------------------------
        # The auto-seek rule is now LOCAL (issue #64): a BLIND hop seeks +
        # re-anchors when its OWN destination is a projection node -- no
        # goal-relative NEXT_HOP peek. The incoming leg ``u -> v`` keys off ``v``;
        # the outgoing leg ``v -> w`` keys off ``w`` (both heads are visible in
        # this per-triple check, so the outgoing side is no longer conservatively
        # forced False). A LINE hop's regime keys off the same destination type
        # (LINE -> projection node is DISTANCE-terminated).
        in_auto_rule = auto_seek_enabled and is_projection_node(graph, v)
        out_auto_rule = auto_seek_enabled and is_projection_node(graph, w)
        in_regime = _regime(in_type, in_dest_type, in_stop_override, in_auto_rule)
        out_regime = _regime(out_type, out_dest_type, out_stop_override, out_auto_rule)
        if in_regime is None or out_regime is None:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') has an unsupported "
                f"regime combo: incoming type {in_type!r} x outgoing type "
                f"{out_type!r}. The two-sided pivot compensation only models "
                f"sensor-terminated (LINE-into-junction / BLIND STOP_AT_LINE) and "
                f"distance-terminated (BLIND / LINE-into-projection) edges -- "
                f"reclassify the offending edge or remove the pivot turn."
            )

        # Single-source the delta formula (no inline re-derive).
        delta = compute_pivot_compensation(theta, track_cm)

        # 3. delta leaves < the trim floor on a DISTANCE-terminated side ----
        # The trim shortens a dead-reckoned drive; the firmware clamps the
        # remainder to PIVOT_TRIM_MIN_DRIVE_CM, so a remainder below that floor
        # would overshoot the node on a table that validated clean. Reject when
        # length - delta < the floor. Incoming and outgoing are independent.
        if in_regime == "distance" and in_distance - delta < pivot_trim_min_drive_cm:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') over-travels "
                f"delta={delta:.3f} cm, leaving {in_distance - delta:.3f} cm of "
                f"the INCOMING dead-reckoned edge length {in_distance:.3f} cm "
                f"(type {in_type!r}) -- below the firmware's trim-underflow floor "
                f"{pivot_trim_min_drive_cm:.3f} cm. The runtime would clamp to "
                f"that floor and overshoot the node -- lengthen ('{u}','{v}'), "
                f"soften the turn, or remove the pivot turn."
            )
        if out_regime == "distance" and out_distance - delta < pivot_trim_min_drive_cm:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') over-travels "
                f"delta={delta:.3f} cm, leaving {out_distance - delta:.3f} cm of "
                f"the OUTGOING dead-reckoned edge length {out_distance:.3f} cm "
                f"(type {out_type!r}) -- below the firmware's trim-underflow floor "
                f"{pivot_trim_min_drive_cm:.3f} cm. The runtime would clamp to "
                f"that floor and overshoot the node -- lengthen ('{v}','{w}'), "
                f"soften the turn, or remove the pivot turn."
            )

        # 4. delta leaves < the trim floor of the sensor/LINE overshoot budget
        # BOTH a 'line' incoming edge (LINE into a junction node) AND a 'sensor'
        # (STOP_AT_LINE seek) incoming edge realise their correction as a
        # reduction of the post-detect overshoot advance, bounded by the same
        # FRONT sensor-to-axle offset (pivots are FRONT-only). The classifier
        # already folds per-edge overrides, the legacy STOP_AT_LINE pipeline
        # default, AND the local auto-rule BLIND->projection-node seek into
        # 'sensor', so both regimes share this one budget (ADR-044). A LINE edge
        # into a PROJECTION node is NOT 'line' here -- it is 'distance', so its
        # trim draws from the follow distance in check 3, not this overshoot
        # budget (issue #64). The firmware clamps the reduced advance to
        # PIVOT_TRIM_MIN_DRIVE_CM, so reject when budget - delta < the floor.
        if (
            in_regime in ("line", "sensor")
            and line_overshoot_budget_cm - delta < pivot_trim_min_drive_cm
        ):
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') over-travels "
                f"delta={delta:.3f} cm, leaving "
                f"{line_overshoot_budget_cm - delta:.3f} cm of the sensor/LINE "
                f"overshoot budget {line_overshoot_budget_cm:.2f} cm (the FRONT "
                f"sensor-to-axle offset; v1 pivots are FRONT-only) -- below the "
                f"firmware's trim-underflow floor {pivot_trim_min_drive_cm:.3f} cm. "
                f"The runtime would clamp the overshoot advance to that floor -- "
                f"approach via a dead-reckon BLIND leg, use a center turn, soften "
                f"the turn, or see issue #75 for ('{u}','{v}','{w}')."
            )

        # Record this triple's delta against the directed edges it trims, for
        # the chained-pivot sum-check below. Only a DISTANCE-terminated leg
        # actually subtracts the trim from that edge's drive; a sensor/line leg
        # absorbs it on the line (no length budget to share). So a chain only
        # stacks on a shared edge that is dead-reckoned for both pivots.
        if out_regime == "distance":
            out_edge_deltas.setdefault((v, w), []).append((i, delta))
        if in_regime == "distance":
            in_edge_deltas.setdefault((u, v), []).append((i, delta))

    # 5. Chained pivots sharing a dead-reckoned edge --------------------------
    # A chain u -> v -> w -> x (pivots at v and w) subtracts TWO trims from the
    # shared edge (v, w): the v-pivot's OUTGOING (departing) trim and the
    # w-pivot's INCOMING (approaching) trim. They must leave at least the
    # firmware's trim floor between them -- length(v -> w) - (delta_uv + delta_vw)
    # >= the floor -- or the firmware's two independent subtractions clamp (or
    # command a non-positive drive).
    for edge, departing in out_edge_deltas.items():
        approaching = in_edge_deltas.get(edge)
        if not approaching:
            continue  # no pivot approaches this edge; the single-side delta >= length gate already covered it.
        v, w = edge
        edge_len = float(graph[v][w]["distance_cm"])
        for dep_i, dep_delta in departing:
            for app_i, app_delta in approaching:
                if dep_i == app_i:
                    continue  # same triple cannot both depart and approach the same edge.
                if edge_len - (dep_delta + app_delta) < pivot_trim_min_drive_cm:
                    raise FieldDataError(
                        f"PIVOT_TURNS[{dep_i}] and PIVOT_TURNS[{app_i}] chain on "
                        f"the shared dead-reckoned edge ('{v}','{w}'): the "
                        f"departing trim {dep_delta:.3f} cm + the approaching "
                        f"trim {app_delta:.3f} cm = {dep_delta + app_delta:.3f} "
                        f"cm leaves {edge_len - (dep_delta + app_delta):.3f} cm of "
                        f"the edge length {edge_len:.3f} cm -- below the firmware's "
                        f"trim-underflow floor {pivot_trim_min_drive_cm:.3f} cm. "
                        f"The runtime would clamp the shared drive to that floor -- "
                        f"lengthen ('{v}','{w}'), soften one turn, or remove one of "
                        f"the chained pivot turns."
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
    entry must be a dict carrying the required keys ``type``, ``u``, ``v``
    and may carry the optional keys ``primitive_mode``, ``direction``,
    ``stop_at_line``, ``distance_cm``, ``heading_deg``. Geometry
    (``distance_cm`` / ``heading_deg``) is OPTIONAL (issue #79): an
    override-only overlay omits it, and downstream
    (:func:`graph_builder._emit_directed_pair`) derives both from the node
    coordinates; range/type checks still apply when geometry IS present, and
    a hand-authored geometry-carrying entry still validates (backward
    compatible).

    ``primitive_mode`` may be authored as an enum INSTANCE (legacy Python:
    ``LineMode.FAST`` / ``BlindMode.STOP_AT_LINE``) OR as the bare string
    member NAME of the matching enum (JSON: ``"FAST"`` / ``"STOP_AT_LINE"``).
    The name is cross-checked against the edge ``type`` -- a :class:`LineMode`
    name (``"NORMAL"`` / ``"SLOW"`` / ``"FAST"``) is legal only on an
    :data:`EDGE_LINE` edge, a :class:`BlindMode` name (``"SYNC_TSHAPE"`` /
    ``"GYRO_TSHAPE"`` / ``"RAW_FLAT"`` / ``"STOP_AT_LINE"``) only on an
    :data:`EDGE_BLIND` edge. The :class:`DEFAULT <pipeline_config.LineMode>`
    sentinel is rejected in BOTH forms -- the enum instance AND the literal
    string ``"DEFAULT"`` -- because ``LineMode["DEFAULT"]`` /
    ``BlindMode["DEFAULT"]`` resolve successfully (DEFAULT is a real member),
    so authoring it explicitly is a confusion (DEFAULT == "absent"). An
    unknown name is rejected with a precise diagnostic.

    An anti-ambiguity cross-check rejects an entry whose unordered pair
    ``{u, v}`` already exists in ``LINE_SEGMENTS`` or ``BLIND_EDGES``
    with a *conflicting* type (:data:`EDGE_LINE` matches LINE_SEGMENTS,
    :data:`EDGE_BLIND` matches BLIND_EDGES): such an entry would silently
    overwrite the earlier edge's type and geometry in :mod:`graph_builder`.
    A *matching*-type entry on an existing pair is a legitimate override
    layer and is accepted.

    An :data:`EDGE_LINE` override incident to a ``"blind"`` or ``"reset"``
    node is rejected (via :func:`_reject_line_follow_into_blind`): neither
    carries a line under the sensor, so a line-follow hop cannot reach it
    (ADR-046 / ADR-048). A LINE edge into a ``"projection"`` node stays
    legal -- the line passes through it (issue #64).

    A per-entry contradiction is rejected (U2 / ADR-048): an entry whose
    ``primitive_mode`` resolves to :attr:`BlindMode.STOP_AT_LINE` (the seek
    bundle) together with ``stop_at_line`` of ``OFF`` (force dead-reckon).
    The two directly oppose; the resolver would silently let OFF win and
    dead-reckon an edge the author asked to stop on. Only this one
    primitive/stop combination is rejected.

    Single-wheel pivots are no longer authored here. The old per-edge
    ``pivot`` / ``pre_turn_mode`` keys are gone; pivots are per-turn
    triples validated by :func:`_validate_pivot_turns`, which runs after
    this per-entry shape check.

    Imports of :class:`LineMode` and :class:`BlindMode` are performed
    inside the function to avoid a circular import with
    :mod:`tools.path_planner.graph_builder`. Edge types are ``LINE`` /
    ``BLIND`` only (issue #64 retired ``EDGE_PROJECTION``); the destination
    NODE type, not the edge type, now carries projection semantics.

    :param field_data: the ``tools.path_planner.field_data`` module (or
        any object exposing the same top-level attributes).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``EDGES[i]`` key path and
        one-line fix hint on the first malformed entry.
    """
    edges = getattr(field_data, "EDGES", None)
    if edges is None:
        # backward-compatible: missing EDGES is fine, but a per-turn pivot can
        # still reference LINE_SEGMENTS / BLIND_EDGES legs, so the pivot pass
        # runs regardless (see validate()).
        return

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
    )
    from tools.path_planner.pipeline_config import (
        BlindMode,
        LineMode,
        StopAtLineMode,
    )

    legal_edge_types = (EDGE_LINE, EDGE_BLIND)
    mode_class_by_edge_type = {
        EDGE_LINE: (LineMode, "LineMode"),
        EDGE_BLIND: (BlindMode, "BlindMode"),
    }
    nodes = getattr(field_data, "NODES", {})
    node_types = getattr(field_data, "NODE_TYPES", {})

    # Anti-ambiguity cross-check (findings #38/#47): an EDGES entry whose
    # unordered pair {u, v} already exists in LINE_SEGMENTS or BLIND_EDGES
    # with a *conflicting* type silently overwrites the earlier edge's type
    # and geometry in graph_builder (LINE then BLIND then EDGES; networkx
    # add_edge replaces attributes). EDGE_LINE matches LINE_SEGMENTS and
    # EDGE_BLIND matches BLIND_EDGES, so an EDGE_BLIND entry on a LINE_SEGMENTS
    # pair (or an EDGE_LINE entry on a BLIND_EDGES pair) is a conflict. A
    # *matching*-type entry (e.g. EDGE_LINE on a LINE_SEGMENTS pair) is a
    # legitimate override layer (distance/heading/direction) and is accepted.
    # This mirrors, via EDGES, the LINE-vs-BLIND guard in :func:`validate`.
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
                "each entry needs at least 'type', 'u', and 'v'"
            )

        # Obsolete per-edge pivot keys (issue #44, T5) ---------------------
        # The per-turn model removed the per-edge ``pivot`` / ``pre_turn_mode``
        # authoring. A stale key here is no longer read by the per-turn pivot
        # validator AND, left in place, a ``pivot`` string would later crash
        # ``graph_builder._extract_directional_attrs`` with a raw ``ValueError``
        # (``int("left")``) when ``_validate_pivot_turns`` builds the authored
        # graph. Fail fast here -- BEFORE that graph build -- with a precise
        # FieldDataError so the raw ValueError is unreachable and the author is
        # pointed at the new top-level ``PIVOT_TURNS`` list (single
        # anti-ambiguity gate, ADR-029).
        for obsolete_key in _OBSOLETE_EDGE_KEYS:
            if obsolete_key in entry:
                raise FieldDataError(
                    f"EDGES[{i}] carries the removed per-edge key "
                    f"'{obsolete_key}' -- single-wheel pivots are no longer "
                    "authored on edges. Drop the 'pivot' / 'pre_turn_mode' key "
                    "and author the turn as a (u, v, w) triple in the top-level "
                    "PIVOT_TURNS list instead (issue #44 per-turn model)."
                )

        # Required keys ----------------------------------------------------
        # Geometry (``distance_cm`` / ``heading_deg``) is OPTIONAL (issue #79):
        # an override-only overlay omits it and the geometry is derived from the
        # node coords downstream (graph_builder._emit_directed_pair). Only the
        # identity keys are required; the geometry range checks below run only
        # when a value IS present.
        for required_key in ("type", "u", "v"):
            if required_key not in entry:
                raise FieldDataError(
                    f"EDGES[{i}] missing required key '{required_key}' -- "
                    "each entry needs at least 'type', 'u', and 'v' "
                    "('distance_cm' / 'heading_deg' are optional and derived "
                    "from the node coordinates when absent)"
                )

        edge_type = entry["type"]
        u = entry["u"]
        v = entry["v"]

        if edge_type not in legal_edge_types:
            raise FieldDataError(
                f"EDGES[{i}].type = {edge_type!r}; expected one of "
                f"{legal_edge_types} -- use EDGE_LINE or EDGE_BLIND from "
                "graph_builder (the projection EDGE type was retired in issue "
                "#64; type the destination NODE 'projection' instead)"
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

        # A LINE override is a line-follow edge; like a LINE_SEGMENTS pair it
        # cannot be incident to a blind OR a reset node (neither carries a line
        # under the sensor), which also keeps the planner's blind-decay fallback
        # correct-by-construction (ADR-041) and a reset node honest as a
        # line-backbone anchor (ADR-048). A LINE edge into a PROJECTION node is
        # LEGAL -- the line passes through the projection node, the hop is just
        # distance-terminated (issue #64 / ADR-046) -- so only a BLIND or RESET
        # destination is rejected here (both handled by the one helper).
        if edge_type == EDGE_LINE:
            _reject_line_follow_into_blind(
                u, v, edge_type, node_types, f"EDGES[{i}]"
            )

        # Conflicting-type cross-check (findings #38/#47). A matching-type
        # override is allowed; only a type that reclassifies an existing
        # LINE/BLIND pair is rejected.
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

        # Optional geometry (issue #79). When ABSENT, both are derived from the
        # node coords by graph_builder._emit_directed_pair -- skip the check.
        # When PRESENT (a hand-authored geometry-carrying entry), keep the full
        # type/range gates so a bad pinned value still fails fast.
        if "distance_cm" in entry:
            distance_cm = entry["distance_cm"]
            if not _is_number(distance_cm) or distance_cm <= 0:
                raise FieldDataError(
                    f"EDGES[{i}].distance_cm must be a positive number; "
                    f"got: {distance_cm!r} -- use a value > 0 in centimetres "
                    "(or omit it to derive the distance from the node coords)"
                )

        if "heading_deg" in entry:
            heading_deg = entry["heading_deg"]
            if (
                not _is_number(heading_deg)
                or not (0.0 <= float(heading_deg) < 360.0)
            ):
                raise FieldDataError(
                    f"EDGES[{i}].heading_deg must be a number in [0, 360); "
                    f"got: {heading_deg!r} -- normalize via "
                    "`heading % 360.0` (or omit it to derive the heading from "
                    "the node coords)"
                )

        # Optional: primitive_mode ----------------------------------------
        # Accepts EITHER an enum INSTANCE (legacy Python authoring:
        # ``LineMode.FAST`` / ``BlindMode.STOP_AT_LINE``) OR the bare string
        # member NAME of the edge type's mode enum (JSON authoring: ``"FAST"`` /
        # ``"STOP_AT_LINE"`` -- the same vocabulary header_writer's
        # ``_override_enum_value`` resolves via ``enum_cls[name]``, issue #79).
        # Both forms are cross-checked against the edge ``type`` (a LineMode tag
        # only on a LINE edge, a BlindMode tag only on a BLIND edge) and BOTH
        # reject the DEFAULT sentinel:
        #   * the enum instance ``expected_cls.DEFAULT``, and
        #   * the literal string ``"DEFAULT"`` -- which ``enum_cls["DEFAULT"]``
        #     would resolve SUCCESSFULLY (DEFAULT is a real member), so the
        #     unknown-name guard below would NOT catch it. DEFAULT == "absent",
        #     so authoring it explicitly is a confusion.
        # An unknown string name is rejected with a precise diagnostic; a value
        # that is neither an enum instance nor a string is rejected as a
        # category error.
        if "primitive_mode" in entry:
            primitive_mode = entry["primitive_mode"]
            expected_cls, expected_name = mode_class_by_edge_type[edge_type]

            if isinstance(primitive_mode, str):
                # Reject the DEFAULT name explicitly BEFORE the enum lookup --
                # enum_cls["DEFAULT"] resolves, so the KeyError guard misses it.
                if primitive_mode == "DEFAULT":
                    raise FieldDataError(
                        f"EDGES[{i}].primitive_mode = 'DEFAULT' -- DEFAULT "
                        "means 'inherit PIPELINE_CONFIG', which is what absence "
                        "already encodes; omit the override or pick a concrete "
                        f"{expected_name} tag"
                    )
                try:
                    resolved = expected_cls[primitive_mode]
                except KeyError:
                    valid_tags = [
                        member.name
                        for member in expected_cls
                        if member is not expected_cls.DEFAULT
                    ]
                    raise FieldDataError(
                        f"EDGES[{i}].primitive_mode = {primitive_mode!r} -- "
                        f"type {edge_type} requires a {expected_name} member "
                        f"name (one of {valid_tags}); the name is unknown. "
                        "A LineMode tag is legal only on a LINE edge and a "
                        "BlindMode tag only on a BLIND edge"
                    )
                # Defensive: a concrete member resolved, but guard against any
                # future alias to DEFAULT slipping through the name check above.
                if resolved == expected_cls.DEFAULT:
                    raise FieldDataError(
                        f"EDGES[{i}].primitive_mode = {primitive_mode!r} "
                        f"resolves to {expected_name}.DEFAULT -- DEFAULT means "
                        "'inherit PIPELINE_CONFIG', which is what absence "
                        "already encodes; omit the override or pick a concrete "
                        f"{expected_name} tag"
                    )
            elif isinstance(primitive_mode, expected_cls):
                if primitive_mode == expected_cls.DEFAULT:
                    raise FieldDataError(
                        f"EDGES[{i}].primitive_mode = {expected_name}.DEFAULT "
                        "-- DEFAULT means 'inherit PIPELINE_CONFIG', which is "
                        "what absence already encodes; omit the override or "
                        "pick a concrete mode"
                    )
            else:
                # Either a string-or-enum of the WRONG enum class, or a value
                # of no enum class at all (and not a string). A LineMode
                # INSTANCE on a BLIND edge (and vice versa) lands here -- a
                # category error -- and must name the expected class so the
                # cross-type message matches the existing diagnostic.
                raise FieldDataError(
                    f"EDGES[{i}].primitive_mode = {primitive_mode!r} -- "
                    f"type {edge_type} requires a {expected_name} instance or "
                    "the string name of one (LineMode on LINE edges, BlindMode "
                    "on BLIND edges)"
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

        # Optional: stop_at_line ------------------------------------------
        # Per-directed-edge stop-on-crossing seek override (asymmetric --
        # applies only to the forward u -> v edge, never mirrored to the
        # reverse, exactly like ``direction``). graph_builder
        # (_extract_directional_attrs, T5) accepts either a StopAtLineMode
        # member (DEFAULT / ON / OFF) or the case-insensitive string
        # vocabulary "on" / "off" -- mirror that here. Absence yields
        # DEFAULT (inherit). "default" as a string is NOT accepted (the
        # ``direction`` key has no DEFAULT string either); use absence or
        # the StopAtLineMode.DEFAULT member to inherit.
        if "stop_at_line" in entry:
            stop_at_line = entry["stop_at_line"]
            if not isinstance(stop_at_line, StopAtLineMode) and (
                not isinstance(stop_at_line, str)
                or stop_at_line.lower() not in ("on", "off")
            ):
                raise FieldDataError(
                    f"EDGES[{i}].stop_at_line must be a StopAtLineMode member "
                    "(DEFAULT, ON, OFF), the string 'on' / 'off' "
                    f"(case-insensitive), or absent; got {stop_at_line!r}"
                )

        # Contradiction: STOP_AT_LINE primitive + stop_at_line='off' (U2) ----
        # A ``primitive_mode`` of ``BlindMode.STOP_AT_LINE`` says "seek the line
        # near the endpoint and re-anchor on it", while ``stop_at_line: OFF`` says
        # "force dead-reckon, do NOT seek". Authored together on one edge the two
        # directly contradict: the motion bundle asks for the seek the per-edge
        # override forbids. The resolver (sim/firmware) would silently let the OFF
        # override win, dead-reckoning an edge the author asked to stop on -- a
        # quiet behavior the author did not intend. Reject it here (the single
        # anti-ambiguity gate, ADR-029) so the contradiction fails loud at author
        # time. This is the ONLY primitive/stop combination rejected; every other
        # pairing (e.g. STOP_AT_LINE + ON, SYNC_TSHAPE + OFF) stays legal. Both
        # fields are already shape-validated above, so a resolve here is safe: a
        # STOP_AT_LINE tag is a BlindMode member name or instance (legal only on a
        # BLIND edge), and an OFF stop is the StopAtLineMode.OFF member or the
        # case-insensitive string "off".
        primitive_is_stop_at_line = False
        if "primitive_mode" in entry:
            pm = entry["primitive_mode"]
            if isinstance(pm, BlindMode):
                primitive_is_stop_at_line = pm == BlindMode.STOP_AT_LINE
            elif isinstance(pm, str):
                primitive_is_stop_at_line = pm == BlindMode.STOP_AT_LINE.name
        stop_is_off = False
        if "stop_at_line" in entry:
            sal = entry["stop_at_line"]
            if isinstance(sal, StopAtLineMode):
                stop_is_off = sal == StopAtLineMode.OFF
            elif isinstance(sal, str):
                stop_is_off = sal.lower() == "off"
        if primitive_is_stop_at_line and stop_is_off:
            raise FieldDataError(
                f"EDGES[{i}] ('{u}','{v}') sets primitive_mode = "
                "BlindMode.STOP_AT_LINE together with stop_at_line='off' -- a "
                "contradiction: STOP_AT_LINE commands the edge to seek the line "
                "and re-anchor, while stop_at_line='off' forces dead-reckon (no "
                "seek). Drop one of the two: keep STOP_AT_LINE (and omit "
                "stop_at_line, or set it ON) to seek, or keep stop_at_line='off' "
                "(and pick a non-seeking BlindMode such as SYNC_TSHAPE) to "
                "dead-reckon."
            )


def _line_and_blind_incident_nodes(field_data) -> Tuple[set, set]:
    """Return ``(line_incident, blind_incident)`` node-name sets from authored topology.

    A node is **LINE-incident** when it is an endpoint of any authored LINE edge
    (a ``LINE_SEGMENTS`` pair, or an ``EDGES`` entry of type :data:`EDGE_LINE`),
    and **BLIND-incident** when it is an endpoint of any authored BLIND edge (a
    ``BLIND_EDGES`` pair, or an ``EDGES`` entry of type :data:`EDGE_BLIND`).
    Authored edges are bidirectional in the planner graph
    (:func:`graph_builder._emit_directed_pair` emits both directions), so an
    endpoint of a BLIND edge always has a BLIND *in*-edge -- which is what makes a
    projection node blind-seek-reachable.

    Reads the AUTHORED topology (no obstacle-clip drop), matching
    :func:`_build_authored_graph`'s stance: reachability is an authoring property,
    and edges lost to C-space clipping are a separate concern (graph_builder warns
    on a fully-disconnected node after inflation). Malformed ``EDGES`` entries are
    skipped here -- :func:`_validate_edges_field` (which runs first) has already
    rejected them.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing ``LINE_SEGMENTS`` / ``BLIND_EDGES`` / optional ``EDGES``).
    :return: a ``(line_incident, blind_incident)`` pair of node-name sets.
    """
    # Deferred import to avoid a circular import: graph_builder imports
    # `validate` from this module.
    from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE

    line_incident: set = set()
    blind_incident: set = set()

    for a, b in getattr(field_data, "LINE_SEGMENTS", ()) or ():
        line_incident.add(a)
        line_incident.add(b)
    for a, b in getattr(field_data, "BLIND_EDGES", ()) or ():
        blind_incident.add(a)
        blind_incident.add(b)

    for entry in getattr(field_data, "EDGES", ()) or ():
        if not isinstance(entry, dict):
            continue
        etype = entry.get("type")
        eu = entry.get("u")
        ev = entry.get("v")
        if eu is None or ev is None:
            continue
        if etype == EDGE_LINE:
            line_incident.add(eu)
            line_incident.add(ev)
        elif etype == EDGE_BLIND:
            blind_incident.add(eu)
            blind_incident.add(ev)

    return line_incident, blind_incident


def _warn_unreachable_nodes(field_data) -> None:
    """Warn (never raise) on a node the authored topology cannot reach correctly.

    Two reachability nudges, both :class:`UserWarning` (authoring guidance, not a
    hard error -- a partial field is still buildable):

    * A ``"line"`` node sits at a junction, so it MUST have at least one incident
      LINE edge (a junction is *defined* by the lines crossing through it). A line
      node with zero incident LINE edges is almost certainly a mis-typed blind /
      projection node -- warn.
    * A ``"projection"`` node is on a line but not at a junction. It is reachable
      either by line-following (an incident LINE edge -- a ``line-proj-line``
      pass-through or a ``line->proj`` distance hop) OR by a blind seek (a BLIND
      in-edge -- ``Blind->Proj`` auto-seek + re-anchor, issue #64). A projection
      node with **only** a ``Blind->Proj`` in-edge is therefore VALID and must NOT
      warn; warn only when it is *otherwise* unreachable -- it has neither an
      incident LINE edge nor an incident BLIND edge.

    Runs after :func:`_validate_edges_field` so the ``EDGES`` rows it scans are
    well-formed. Mutates no state; emits the warnings and returns.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing the same top-level attributes).
    :return: ``None``.
    """
    node_types = getattr(field_data, "NODE_TYPES", {})
    line_incident, blind_incident = _line_and_blind_incident_nodes(field_data)

    for name, node_type in node_types.items():
        if node_type == _LINE_NODE_TYPE and name not in line_incident:
            warnings.warn(
                f"line node '{name}' has no incident LINE edge -- a 'line' node "
                "sits at a junction, which must lie on a line edge. Add a "
                "LINE_SEGMENTS / EDGES(LINE) edge through it, or re-type it "
                "('projection' for an on-line non-junction waypoint, 'blind' for "
                "an off-line dead-reckon node).",
                UserWarning,
                stacklevel=2,
            )
        elif node_type == _PROJECTION_NODE_TYPE and (
            name not in line_incident and name not in blind_incident
        ):
            # A Blind->Proj in-edge (any incident BLIND edge -> a BLIND in-edge,
            # since authored edges are bidirectional) makes it blind-seek-
            # reachable and exempt; an incident LINE edge makes it
            # line-follow-reachable. Only an isolated projection node warns.
            warnings.warn(
                f"projection node '{name}' is unreachable -- it has no incident "
                "LINE edge (line-follow / pass-through) and no incident BLIND "
                "edge (Blind->Proj auto-seek). Wire it to the graph with a LINE "
                "or BLIND edge, or remove it.",
                UserWarning,
                stacklevel=2,
            )


def _validate_reset_nodes(field_data) -> None:
    """Reject malformed wall-zero reset nodes.

    A reset node re-localizes the chassis against a known wall: it carries a
    ``{"axis", "wall_coord", "facing"}`` spec in the sparse top-level
    ``field_data.RESET_NODES`` table (keyed by node name), and is typed
    ``"reset"`` in ``NODE_TYPES``. This mirrors the per-turn ``PIVOT_TURNS``
    precedent -- a top-level sparse table, not a key buried inside a node.

    Validates that:

    1. ``RESET_NODES`` (when present) is a dict keyed by node name;
    2. every key is a node in ``NODES``;
    3. the ``"reset"`` typing is consistent both directions -- every node typed
       ``"reset"`` in ``NODE_TYPES`` has a ``RESET_NODES`` entry, and every
       ``RESET_NODES`` key is typed ``"reset"``;
    4. each spec is a dict with ``axis`` in ``{"x","y"}``, a finite
       ``wall_coord`` (cm), and a finite ``facing`` in ``[0, 360)`` degrees;
    5. ``facing`` points from the node AWAY from the wall (its component along the
       reset axis is OPPOSITE the sign of ``wall_coord - node_coord``) -- the
       firmware presses REARWARD, so the rear (at ``facing + 180``) faces the wall;
    6. the node-to-wall standoff ``abs(wall_coord - node_coord)`` is at least
       ``WALL_PIVOT_CLEARANCE_CM`` -- the chassis pivots in place around the
       wall-zero and needs room to clear the wall (a zero-gap on-wall node is rejected);
    7. the ``START_NODE`` is not typed ``"reset"`` -- the interlude runs only on a
       post-hop arrival, so a reset START node would never re-zero at mission start.

    Like :func:`_validate_pivot_turns`, this early-returns on a reset-free
    field (absent / empty table) so the common case pays nothing.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing the same top-level attributes -- including the optional
        ``RESET_NODES``).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``RESET_NODES['<name>']`` key path
        and a one-line fix hint on the first malformed entry.
    """
    reset_nodes = getattr(field_data, "RESET_NODES", {})
    node_types = getattr(field_data, "NODE_TYPES", {})

    # Even on an empty RESET_NODES we must still catch a node typed "reset"
    # that has NO entry, so only short-circuit when BOTH sides are empty.
    reset_typed = {
        name for name, value in node_types.items() if value == "reset"
    }
    if not reset_nodes and not reset_typed:
        return

    if not isinstance(reset_nodes, dict):
        raise FieldDataError(
            "RESET_NODES must be a dict mapping node-name -> "
            "{axis, wall_coord, facing}; "
            f"got: {type(reset_nodes).__name__}"
        )

    nodes = getattr(field_data, "NODES", {})

    # (#3) The START node may not be a reset node. The wall-zero interlude fires only on a
    # post-hop ARRIVAL (the firmware gates it off the first plan of a goal, and the sim only
    # checks for a reset node after a hop), so a START reset node would never run its interlude
    # at mission start anyway -- typing it "reset" is therefore a no-op trap that suggests a
    # wall re-zero will happen at the start when it will not. Reject it so the author re-homes
    # the wall re-zero onto a mid-route node (or uses the calibration start routine instead).
    start_node = getattr(field_data, "START_NODE", None)
    if start_node in reset_typed:
        raise FieldDataError(
            f"START_NODE '{start_node}' is typed 'reset' -- the wall-zero reset interlude "
            "runs only on a post-hop arrival, never at mission start, so a reset START node "
            "never re-zeroes. Move the reset typing to a mid-route node, or change "
            f"NODE_TYPES['{start_node}']."
        )

    # Consistency: every reset-typed node needs a RESET_NODES entry.
    for name in reset_typed:
        if name not in reset_nodes:
            raise FieldDataError(
                f"node '{name}' is typed 'reset' in NODE_TYPES but has no "
                f"RESET_NODES entry -- add RESET_NODES['{name}'] = "
                "{'axis': 'x'|'y', 'wall_coord': <cm>, 'facing': <deg>} "
                "or change its node type"
            )

    for name, spec in reset_nodes.items():
        # The key must be a real node ...
        if name not in nodes:
            raise FieldDataError(
                f"RESET_NODES references unknown node: '{name}' -- "
                "add it to NODES or remove the entry from RESET_NODES"
            )
        # ... and that node must be typed 'reset' (consistency, other way).
        if node_types.get(name) != "reset":
            raise FieldDataError(
                f"RESET_NODES['{name}'] is set but NODE_TYPES['{name}'] = "
                f"'{node_types.get(name)}' -- a reset node must be typed "
                "'reset'; fix NODE_TYPES or remove the RESET_NODES entry"
            )

        if not isinstance(spec, dict):
            raise FieldDataError(
                f"RESET_NODES['{name}'] must be a dict with keys "
                f"'axis', 'wall_coord', 'facing'; got: {spec!r}"
            )

        for required_key in ("axis", "wall_coord", "facing"):
            if required_key not in spec:
                raise FieldDataError(
                    f"RESET_NODES['{name}'] missing required key "
                    f"'{required_key}' -- each entry needs 'axis', "
                    "'wall_coord', and 'facing'"
                )

        axis = spec["axis"]
        wall_coord = spec["wall_coord"]
        facing = spec["facing"]

        if axis not in _ALLOWED_RESET_AXES:
            raise FieldDataError(
                f"RESET_NODES['{name}'].axis = {axis!r}; expected one of "
                "{'x','y'} -- the wall is parallel to one field axis"
            )

        if not _is_number(wall_coord):
            raise FieldDataError(
                f"RESET_NODES['{name}'].wall_coord must be a finite number "
                f"in centimetres; got: {wall_coord!r}"
            )

        if not _is_number(facing) or not (0.0 <= float(facing) < 360.0):
            raise FieldDataError(
                f"RESET_NODES['{name}'].facing must be a number in [0, 360); "
                f"got: {facing!r} -- normalize via `facing % 360.0`"
            )

        node_x, node_y = nodes[name]
        node_coord = node_x if axis == "x" else node_y
        toward = float(wall_coord) - float(node_coord)

        # Clearance floor: the robot SQUARE-turns to `facing` in place and then
        # presses the wall rear-first, so it needs room to pivot without the rear
        # clipping the wall during the sweep (and after the wall-zero). Reject any
        # standoff below the pivot sweep radius (plus safety). A node authored
        # exactly ON the wall (zero gap) also fails here -- no room to pivot at all.
        if abs(toward) < WALL_PIVOT_CLEARANCE_CM:
            raise FieldDataError(
                f"RESET_NODES['{name}'] stands {abs(toward)} cm from the wall "
                f"(node at {axis}={node_coord}, wall at {axis}={wall_coord}), below "
                f"the {WALL_PIVOT_CLEARANCE_CM} cm pivot clearance floor: the robot "
                "pivots in place around the wall-zero and would clip the wall. Author "
                f"the node at least {WALL_PIVOT_CLEARANCE_CM} cm from the wall."
            )

        # Geometry: `facing` must point from the node AWAY from the wall. The
        # firmware presses REARWARD (a reverse RAW push after an in-place SQUARE
        # turn to `facing`); `facing` is the PARKED heading with the robot's FRONT
        # pointing away from the wall, so the REAR (at facing+180) presses it. If
        # `facing` pointed toward the wall the chassis would reverse off the field,
        # never stall, and the advisory timeout would still re-anchor against thin
        # air -- a large silent position error with no fault. The wall normal is
        # axis-aligned and the world frame is CCW with 0 deg = +x, so the facing's
        # component along the reset axis is cos(facing) on x / sin(facing) on y;
        # that component must be OPPOSITE the sign of (wall_coord - node_coord) so
        # the rear faces the wall. (toward != 0 is guaranteed by the floor above.)
        rad = math.radians(float(facing))
        comp = math.cos(rad) if axis == "x" else math.sin(rad)
        if comp * toward >= 0.0:
            away = "decreasing" if toward > 0.0 else "increasing"
            raise FieldDataError(
                f"RESET_NODES['{name}'].facing = {facing!r} deg does not point away "
                f"from the wall: the node is at {axis}={node_coord} and the wall is "
                f"at {axis}={wall_coord}, so the robot's FRONT (`facing`) must point "
                f"toward {away} {axis} (away from the wall) so the rear presses it "
                "(the firmware presses REARWARD, along facing+180). Re-author "
                "`facing` to point away from the wall."
            )


def validate_blind_decay_domain(
    enabled: Any, k: Any, exponent: Any, d_ref: Any
) -> Optional[Tuple[str, str]]:
    """Return ``None`` if the four decay knobs are in-domain, else ``(field, reason)``.

    Single source of truth for the blind-distance decay parameter domain, shared
    by the field-data validator (:func:`_validate_blind_decay`) and the planner
    guard (``planner._validate_blind_decay_params``) so the two can never drift.
    A direct / bypass caller (sim, CTPS, a hand-built ``BlindDecayParams``) skips
    the field-data validator, so the planner re-runs exactly these rules before
    it can emit a negative / NaN / overflowing Dijkstra weight.

    The rules:

    * ``enabled`` is a real ``bool`` -- not a truthy int / string. ``bool("false")``
      is ``True``, so a non-bool would silently *enable* decay.
    * a disabled config (``enabled is False``) is exempt from the rest -- every
      blind edge short-circuits to factor ``1`` upstream and never reaches the
      weight formula.
    * ``k`` is finite and ``>= 0`` (a negative gain makes blind edges cheaper;
      ``k == 0`` disables the penalty).
    * ``exponent`` is finite and in ``[1, BLIND_DECAY_EXPONENT_MAX]`` -- a
      sub-linear exponent inverts the penalty, an extreme one overflows the
      weight.
    * ``d_ref`` is finite and ``> 0`` (the normalizer; a non-positive value
      divides by zero / flips the sign).

    NaN / infinities are rejected by :func:`_is_number` at every numeric gate; a
    comparison-only guard lets them through (``nan < 0`` is ``False``) and
    corrupts the routing table with no diagnostic.

    :param enabled: the master switch (must be a ``bool``).
    :param k: the penalty gain.
    :param exponent: the convexity exponent.
    :param d_ref: the length normalizer in centimetres.
    :return: ``None`` when every value is in-domain, else a ``(field, reason)``
        pair where ``field`` is the lowercase knob name
        (``"enabled"`` / ``"k"`` / ``"exponent"`` / ``"d_ref"``) and ``reason`` is
        a one-line explanation with no field-name prefix, so each caller prepends
        its own naming (``BLIND_DECAY_*`` for field data, ``BlindDecayParams.*``
        for the planner).
    """
    if not isinstance(enabled, bool):
        return ("enabled", f"must be a bool (true / false); got: {enabled!r}")
    if not enabled:
        return None
    if not _is_number(k) or k < 0:
        return (
            "k",
            f"must be a finite number >= 0 (0 disables the penalty); got: {k!r}",
        )
    if not _is_number(exponent) or not (1.0 <= exponent <= BLIND_DECAY_EXPONENT_MAX):
        return (
            "exponent",
            "must be a finite number in "
            f"[1, {BLIND_DECAY_EXPONENT_MAX:g}] (a sub-linear exponent inverts the "
            "penalty; an extreme exponent overflows the weight); "
            f"got: {exponent!r}",
        )
    if not _is_number(d_ref) or d_ref <= 0:
        return ("d_ref", f"must be a finite number > 0 (cm); got: {d_ref!r}")
    return None


def _reject_line_follow_into_blind(
    u: str, v: str, edge_kind: str, node_types: dict, where: str
) -> None:
    """Reject a line-follow edge incident to an off-line (blind / reset) node.

    A ``LINE`` edge follows a line for its whole length, so *neither* endpoint
    may be typed ``"blind"`` (there is no line under the sensor at a blind node)
    or ``"reset"`` (a wall-zero re-anchor node is reached by dead reckoning and
    presses a wall off the line backbone -- it carries no line under the sensor
    either, exactly like a blind node). A ``"projection"`` destination stays
    LEGAL: the line passes through a projection node (longitudinally detectable
    as a crossing), so a LINE hop can follow into it -- the hop is just
    distance-terminated, not junction-terminated (issue #64 / ADR-046). Only a
    ``"blind"`` or ``"reset"`` endpoint is rejected.

    Beyond being physically impossible, a line-follow edge that *terminates on* a
    blind node breaks the planner's blind-decay model (ADR-041): a blind pocket
    reachable from the line backbone only through such a seam never seeds the
    blind-distance search, so its interior blind hops get ``d == inf`` and the
    ``inf -> factor 1`` fallback prices the deepest-drift hops the *cheapest* --
    silently, because the disconnected-node warning never fires (the node is
    reachable in the full graph). A reset node is a blind-decay anchor
    (``planner._ANCHOR_NODE_TYPES`` is ``{line, reset}``): reaching it by a
    line-follow seam would make it advertise itself as a line-backbone anchor it
    is not. A blind / reset node must therefore be entered by a BLIND edge, never
    a line-follow seam (ADR-046 / ADR-048).

    :param u: the edge tail node name.
    :param v: the edge head node name.
    :param edge_kind: the edge type label for the message (``"LINE"``).
    :param node_types: the validated ``NODE_TYPES`` map.
    :param where: a key-path label for the message (e.g. ``"LINE_SEGMENTS"`` or
        ``"EDGES[3]"``).
    :raises FieldDataError: when either endpoint is typed ``"blind"`` or
        ``"reset"``.
    """
    for endpoint in (u, v):
        if node_types.get(endpoint) == _BLIND_NODE_TYPE:
            raise FieldDataError(
                f"{where} ('{u}','{v}') is a {edge_kind} (line-follow) edge "
                f"incident to blind node '{endpoint}' -- line-following needs a "
                "line under the sensor, so a blind node must be reached by a "
                "BLIND edge (in BLIND_EDGES), not a LINE edge. Re-type the node "
                "in NODE_TYPES or move the pair to BLIND_EDGES."
            )
        if node_types.get(endpoint) == _RESET_NODE_TYPE:
            raise FieldDataError(
                f"{where} ('{u}','{v}') is a {edge_kind} (line-follow) edge "
                f"incident to reset node '{endpoint}' -- a wall-zero reset node "
                "is reached by dead reckoning and presses a wall off the line "
                "backbone, so it carries no line under the sensor (like a blind "
                "node). A reset node must be reached by a BLIND edge (in "
                "BLIND_EDGES), not a LINE edge (ADR-046 / ADR-048). Re-type the "
                "node in NODE_TYPES or move the pair to BLIND_EDGES."
            )


def _validate_blind_decay(field_data) -> None:
    """Reject a malformed blind-distance decay configuration.

    The four ``BLIND_DECAY_*`` top-level scalars parameterise the blind-edge
    cost penalty the planner applies at table-build time (T22):
    ``weight = length * (1 + k * (d / d_ref) ** exponent)``. They live in
    ``field_data.json`` as plain JSON scalars (ADR-022) and replace the old
    ``COST_BLIND_MULTIPLIER`` knob. The domain rules live in the shared
    :func:`validate_blind_decay_domain` predicate (so the planner guard cannot
    drift from this pass); this pass adapts a ``(field, reason)`` result into a
    ``FieldDataError`` naming the ``BLIND_DECAY_*`` key:

    * ``BLIND_DECAY_ENABLED`` -- a ``bool`` (``True`` / ``False``).
    * ``BLIND_DECAY_K`` -- a finite number ``>= 0`` (``k == 0`` disables the
      penalty so blind weights reduce to plain length).
    * ``BLIND_DECAY_EXPONENT`` -- a finite number in ``[1, BLIND_DECAY_EXPONENT_MAX]``
      (a sub-linear exponent inverts the penalty's intent; an extreme exponent
      overflows the weight).
    * ``BLIND_DECAY_D_REF`` -- a finite number ``> 0`` (the normalizer; a
      non-positive ``d_ref`` divides by zero / flips the sign).

    Presence is guaranteed by the ``_REQUIRED_ATTRS`` structural check in
    :func:`validate`, so this pass only judges the values.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing the same top-level attributes).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``BLIND_DECAY_*`` key path and a
        one-line fix hint on the first malformed value.
    """
    enabled = getattr(field_data, "BLIND_DECAY_ENABLED")
    k = getattr(field_data, "BLIND_DECAY_K")
    exponent = getattr(field_data, "BLIND_DECAY_EXPONENT")
    d_ref = getattr(field_data, "BLIND_DECAY_D_REF")

    result = validate_blind_decay_domain(enabled, k, exponent, d_ref)
    if result is not None:
        field, reason = result
        raise FieldDataError(f"BLIND_DECAY_{field.upper()} {reason}")


def _validate_pipeline_config(field_data) -> None:
    """Reject a malformed field-sourced ``PIPELINE_*`` default (issue #79).

    The mission-wide motion-mode defaults baked into ``routes.cpp``'s
    ``PIPELINE_CONFIG`` may be authored in the field data as four optional
    top-level keys carrying enum member NAMES:

    * ``PIPELINE_LINE_MODE``      -- a :class:`LineMode` name      (no ``DEFAULT``)
    * ``PIPELINE_BLIND_MODE``     -- a :class:`BlindMode` name     (no ``DEFAULT``)
    * ``PIPELINE_TURN_MODE``      -- a :class:`TurnMode` name      (no ``DEFAULT``)
    * ``PIPELINE_DIRECTION_MODE`` -- a :class:`DirectionMode` name (``DEFAULT`` OK)

    Each is OPTIONAL: an absent key inherits the ``pipeline_config.py``
    ``PIPELINE_CONFIG`` default at emit time. Reads are non-strict
    (``getattr(field_data, "PIPELINE_*", None)``) so the many ``SimpleNamespace``
    fixtures that omit these keys keep validating unchanged.

    This mirrors :meth:`pipeline_config.PipelineConfig.__post_init__` exactly:
    a category default (line / blind / turn) may NOT be its own ``DEFAULT``
    sentinel -- ``DEFAULT`` means "inherit PIPELINE_CONFIG.<category>", which is
    self-referential when applied to the pipeline config itself and silently
    degrades to the C++ hardcoded fallback. ``DirectionMode.DEFAULT`` is the one
    exception ("defer to the builder auto algorithm") and is allowed. Running
    this check in :func:`validate` (BEFORE emit) means a build never reaches
    ``header_writer`` with a value ``<EnumCls>[name]`` cannot resolve.

    A value that is already an enum INSTANCE (legacy / programmatic authoring)
    is also accepted, cross-checked against the same DEFAULT rule -- so a Python
    fixture that sets ``PIPELINE_LINE_MODE = LineMode.FAST`` validates too.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing the optional ``PIPELINE_*`` attributes).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``PIPELINE_*`` key path and a
        one-line fix hint on the first malformed value.
    """
    # Deferred import to avoid a circular import via graph_builder, which
    # imports `validate` from this module (pipeline_config is a leaf, but keep
    # the import local to mirror the other validators' deferred-import policy and
    # to skip the cost on a field that omits every pipeline key).
    from tools.path_planner.pipeline_config import (
        BlindMode,
        DirectionMode,
        LineMode,
        TurnMode,
    )

    # (attr, EnumCls, enum-class-name, DEFAULT-allowed?, singleton-field-name).
    # line/blind/turn reject DEFAULT; direction allows it -- byte-for-byte the
    # PipelineConfig.__post_init__ split (its _CATEGORY_FIELDS is line/blind/turn;
    # direction is exempt). The 5th element is the actual PipelineConfig singleton
    # field name (``line`` / ``blind`` / ``turn`` / ``direction``, matching
    # PipelineConfig.from_field_data's ``_SPECS`` ctor field), interpolated into
    # the DEFAULT-rejection messages so they name the real field. It is NOT
    # ``enum_name.lower()`` -- ``"LineMode".lower()`` is ``"linemode"``, which
    # names no field on PIPELINE_CONFIG (R8).
    pipeline_specs = (
        ("PIPELINE_LINE_MODE", LineMode, "LineMode", False, "line"),
        ("PIPELINE_BLIND_MODE", BlindMode, "BlindMode", False, "blind"),
        ("PIPELINE_TURN_MODE", TurnMode, "TurnMode", False, "turn"),
        ("PIPELINE_DIRECTION_MODE", DirectionMode, "DirectionMode", True, "direction"),
    )

    for attr, enum_cls, enum_name, default_allowed, config_field in pipeline_specs:
        value = getattr(field_data, attr, None)
        if value is None:
            continue  # absent -> inherit pipeline_config.py default at emit.

        if isinstance(value, str):
            # The DEFAULT name must be judged against `default_allowed` BEFORE
            # the resolve: `enum_cls["DEFAULT"]` resolves (DEFAULT is a real
            # member), so the unknown-name guard would not catch a disallowed
            # DEFAULT.
            if value == "DEFAULT":
                if not default_allowed:
                    raise FieldDataError(
                        f"{attr} = 'DEFAULT' -- {enum_name}.DEFAULT means "
                        f"'inherit PIPELINE_CONFIG.{config_field}', which "
                        "is circular when set as the pipeline default itself; "
                        "omit the key (to inherit the pipeline_config.py "
                        f"default) or pick a concrete {enum_name} tag"
                    )
                continue  # DirectionMode.DEFAULT is a legitimate value.
            try:
                resolved = enum_cls[value]
            except KeyError:
                valid_tags = [
                    member.name
                    for member in enum_cls
                    if default_allowed or member is not enum_cls.DEFAULT
                ]
                raise FieldDataError(
                    f"{attr} = {value!r} -- expected a {enum_name} member name "
                    f"(one of {valid_tags}); the name is unknown. Omit the key "
                    "to inherit the pipeline_config.py default."
                )
            if resolved == enum_cls.DEFAULT and not default_allowed:
                raise FieldDataError(
                    f"{attr} = {value!r} resolves to {enum_name}.DEFAULT -- "
                    f"DEFAULT means 'inherit PIPELINE_CONFIG.{config_field}', "
                    "which is circular when set as the pipeline default itself; "
                    f"omit the key or pick a concrete {enum_name} tag"
                )
        elif isinstance(value, enum_cls):
            if value == enum_cls.DEFAULT and not default_allowed:
                raise FieldDataError(
                    f"{attr} = {enum_name}.DEFAULT -- DEFAULT means 'inherit "
                    f"PIPELINE_CONFIG.{config_field}', which is circular "
                    "when set as the pipeline default itself; omit the key or "
                    f"pick a concrete {enum_name} tag"
                )
        else:
            raise FieldDataError(
                f"{attr} = {value!r} -- expected a {enum_name} member name "
                f"(string) or a {enum_name} instance; omit the key to inherit "
                "the pipeline_config.py default."
            )


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
                "every NODES key needs a type "
                "('line', 'projection', 'blind', or 'reset')"
            )
    for name, value in node_types.items():
        if value not in _ALLOWED_NODE_TYPES:
            raise FieldDataError(
                f"NODE_TYPES['{name}'] = '{value}'; expected one of "
                "{'line','projection','blind','reset'}"
            )

    # 4. LINE_SEGMENTS coverage.
    if not isinstance(line_segments, (list, tuple)):
        raise FieldDataError(
            "LINE_SEGMENTS must be a list of (a, b) node-name pairs; "
            f"got: {type(line_segments).__name__}"
        )
    _validate_edge_list(line_segments, nodes, "LINE_SEGMENTS")

    # 4b. A LINE_SEGMENTS edge is a line-follow segment, so neither endpoint may
    # be a blind OR a reset node (you cannot line-follow to a node with no line;
    # a reset node presses a wall off the line backbone, like a blind node). This
    # also keeps the planner's blind-decay inf->factor-1 fallback correct-by-
    # construction -- a blind pocket must be entered by a BLIND edge, never a
    # line-follow seam (ADR-041) -- and a reset node honest as a line-backbone
    # anchor (ADR-048). Runs after NODE_TYPES (pass 3) so the typing is validated.
    for a, b in line_segments:
        _reject_line_follow_into_blind(a, b, "LINE", node_types, "LINE_SEGMENTS")

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
    if not isinstance(start_node, str):
        raise FieldDataError(
            f"START_NODE must be a node-name string; got: {start_node!r}"
        )
    if start_node not in nodes:
        raise FieldDataError(
            f"START_NODE references unknown node: '{start_node}' -- "
            "add it to NODES or pick an existing node"
        )
    # The chassis is operator-placed at mission start with no prior line-follow,
    # so the START node may not be on-line-and-sensor-anchored: a 'line' node
    # (reached by line-following into a junction) and a 'projection' node
    # (reached by a distance-gated / blind seek) both presume an arrival the
    # robot has not made at t=0. The start origin is dead-reckoned -- a 'blind'
    # node -- (a 'reset' START is separately rejected by _validate_reset_nodes,
    # since its wall-zero interlude only fires post-hop). Reject both here so the
    # author re-homes the spawn onto a blind origin (issue #64).
    start_type = node_types.get(start_node)
    if start_type in (_LINE_NODE_TYPE, _PROJECTION_NODE_TYPE):
        raise FieldDataError(
            f"START_NODE '{start_node}' is typed '{start_type}' -- the chassis "
            "is operator-placed at mission start with no prior line-follow, so a "
            "'line' (junction) or 'projection' (distance-gated) start node "
            "presumes a sensor-anchored arrival that has not happened at t=0. "
            "Author the spawn as a 'blind' node (dead-reckoned origin), or move "
            f"the on-line typing off START_NODE; fix NODE_TYPES['{start_node}']."
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

    # 8b. Optional field-wide stop-at-line toggle (issue #43). When present it
    # must be a bool; absence defaults to True downstream (field_data.py), so
    # it is NOT a required attribute -- only its type is constrained.
    if hasattr(field_data, "BLIND_PROJECTION_AUTO_SEEK"):
        auto_seek = getattr(field_data, "BLIND_PROJECTION_AUTO_SEEK")
        if not isinstance(auto_seek, bool):
            raise FieldDataError(
                "BLIND_PROJECTION_AUTO_SEEK must be a bool (True or False); "
                f"got: {auto_seek!r}"
            )

    # 9. Optional rich-edge table (PR4 / T6). Absent on legacy fixtures.
    _validate_edges_field(field_data)

    # 9b. Reachability warnings (issue #64). Runs after the EDGES shape check so
    # the authored topology it scans is well-formed. Warns (never raises) on a
    # 'line' node with no incident LINE edge and on an otherwise-unreachable
    # 'projection' node -- a 'Blind->Proj' projection node is exempt.
    _warn_unreachable_nodes(field_data)

    # 10. Per-turn single-wheel pivots (issue #44 per-turn model, T5). Runs
    # after the EDGES shape check so the directed-edge legs it reads are
    # well-formed, and after node/segment validation so the authored graph it
    # builds is sound. Inert (early return) on a pivot-free field.
    _validate_pivot_turns(field_data)

    # 11. Wall-zero reset nodes (PR2 T12). Runs after NODE_TYPES validation so
    # the "reset" typing it cross-checks is well-formed. Inert (early return)
    # on a reset-free field.
    _validate_reset_nodes(field_data)

    # 12. Blind-distance decay knobs (PR3 T21). Presence is guaranteed by the
    # _REQUIRED_ATTRS structural check above; this pass enforces each value's
    # domain (k >= 0, exponent >= 1, d_ref > 0, enabled is a bool).
    _validate_blind_decay(field_data)

    # 13. Optional field-sourced PIPELINE_CONFIG defaults (issue #79). Each
    # PIPELINE_* key (when present) must name a valid enum member; line/blind/turn
    # reject DEFAULT, direction allows it (mirrors PipelineConfig.__post_init__).
    # Runs BEFORE the emitter so a build never reaches header_writer with a value
    # `<EnumCls>[name]` cannot resolve. Reads are non-strict, so a field that
    # omits every key (the common case) passes inertly.
    _validate_pipeline_config(field_data)
