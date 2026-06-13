"""Strict validator for the path planner's field-data input (PR3 schema).

Inspects the raw module attributes (``NODES``, ``NODE_TYPES``,
``LINE_SEGMENTS``, ``BLIND_EDGES``, ``OBSTACLES``, ``FIELD_MM``,
``MAP_PNG``, optional ``EDGES``, optional ``PIVOT_TURNS``, ...) and raises
:class:`FieldDataError` on the first malformed entry with a precise key
path and a one-line nudge toward the fix.

PR3 schema (vs PR1/PR2): drops ``PROJECTIONS`` and ``GOAL_APPROACH``,
adds required ``BLIND_EDGES`` and ``FIELD_MM`` / ``MAP_PNG``.

PR4 (T3/T6) adds an optional ``EDGES`` table for rich per-edge metadata
(EDGE_PROJECTION, per-edge mode overrides). Absence of ``EDGES`` is benign
for backward compatibility.

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
from pathlib import Path
from typing import Any, Iterable

from tools.path_planner.config_parse import DEFAULT_CONFIG_H_PATH, parse_config_float

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
        EDGE_PROJECTION,
        _emit_directed_pair,
    )

    nodes = getattr(field_data, "NODES", {})
    g = nx.DiGraph()
    for name, coord in nodes.items():
        g.add_node(name, x_cm=float(coord[0]), y_cm=float(coord[1]))

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
        if etype not in (EDGE_LINE, EDGE_BLIND, EDGE_PROJECTION):
            continue  # bad type; the per-entry shape check reports it.
        if "distance_cm" not in entry or "heading_deg" not in entry:
            continue  # malformed entry; the per-entry shape check reports it.
        # Belt-and-suspenders for W1: forward ONLY the geometry/mode keys the
        # pivot checks read off the graph, never the whole entry. The shape
        # check (`_validate_edges_field`) already rejects a stale `pivot` /
        # `pre_turn_mode` before this runs, but narrowing the payload means a
        # stray legacy key can never reach `_extract_directional_attrs` and
        # crash on `int("left")` even if this helper is ever called on
        # unvalidated data.
        geometry_options = {
            key: entry[key]
            for key in ("distance_cm", "heading_deg", "primitive_mode", "direction")
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
    3. classifies each side's edge into a regime -- ``"line"`` (true LINE,
       overshoot-budget-bearing), ``"sensor"`` (a BLIND edge whose effective
       mode is STOP_AT_LINE -- self-corrects, no trim, no budget), or
       ``"distance"`` (plain BLIND / PROJECTION -- dead-reckoned, where the
       trim actually bites);
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
    * **Trim exceeds the LINE-incoming overshoot budget.** Only when the
      incoming side is a true ``"line"`` edge; ``"sensor"`` (STOP_AT_LINE)
      incoming is EXEMPT (review fix 3).
    * **Chained pivots over-trim a shared edge.** Two triples sharing the same
      directed edge ``(v, w)`` (one departing it, one approaching it) must
      satisfy ``delta_uv + delta_vw < length(v -> w)``.
    * **Authored REAR on a pivot-adjacent edge.** v1 is FRONT-only; a
      ``direction: "rear"`` on the exact ``u -> v`` or ``v -> w`` leg is
      rejected (reverse pivots are #54).

    The supported incoming x outgoing matrix (``line`` / ``sensor`` =
    sensor-terminated; ``distance`` = dead-reckoned) is::

        incoming \\ outgoing | sensor (line / STOP_AT_LINE) | distance (BLIND / PROJ)
        --------------------+------------------------------+-------------------------
        line (re-anchor)    | overshoot-reduce in;          | overshoot-reduce in;
                            | out no-op (self-corrects @w)  | out drive trimmed
        sensor / distance   | approach trim in;             | approach trim in;
                            | out no-op (self-corrects @w)  | out drive trimmed

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
        EDGE_PROJECTION,
        directed_edge_heading,
    )
    from tools.path_planner.pipeline_config import PIPELINE_CONFIG, BlindMode
    from tools.path_planner.planner import (
        compute_pivot_compensation,
        parse_track_cm_from_config,
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

    def _edge_attrs(a: str, b: str):
        """Return the directed edge's (type, distance_cm, primitive_mode) or None."""
        if not graph.has_edge(a, b):
            return None
        data = graph[a][b]
        return data["type"], float(data["distance_cm"]), data.get("primitive_mode")

    def _regime(edge_type, primitive_mode) -> str | None:
        """Classify a directed edge: 'line', 'sensor', 'distance', or None.

        ``line``     -- a true LINE edge: re-anchors on the junction line; its
                        incoming trim is bounded by the LINE overshoot budget.
        ``sensor``   -- a BLIND edge whose EFFECTIVE mode is STOP_AT_LINE
                        (review fix 1: an absent override inherits
                        ``PIPELINE_CONFIG.blind``, so a no-override BLIND classifies
                        sensor when the pipeline default is STOP_AT_LINE). Self-
                        corrects on the line: takes no trim, and is NOT subject to
                        the overshoot budget (review fix 3).
        ``distance`` -- plain BLIND or PROJECTION: dead-reckoned; the trim bites.
        ``None``     -- an unclassifiable edge type (unsupported combo).
        """
        if edge_type == EDGE_LINE:
            return "line"
        if edge_type == EDGE_PROJECTION:
            return "distance"
        if edge_type == EDGE_BLIND:
            # Review fix 1: a BLIND edge with no per-edge override inherits
            # PIPELINE_CONFIG.blind. Classify on the EFFECTIVE mode so a
            # no-override BLIND under a STOP_AT_LINE pipeline default is sensor.
            effective = primitive_mode if isinstance(primitive_mode, BlindMode) else PIPELINE_CONFIG.blind
            if effective == BlindMode.STOP_AT_LINE:
                return "sensor"
            return "distance"
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
        in_type, in_distance, in_primitive = in_attrs
        out_type, out_distance, out_primitive = out_attrs

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
        in_regime = _regime(in_type, in_primitive)
        out_regime = _regime(out_type, out_primitive)
        if in_regime is None or out_regime is None:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') has an unsupported "
                f"regime combo: incoming type {in_type!r} x outgoing type "
                f"{out_type!r}. The two-sided pivot compensation only models "
                f"sensor-terminated (LINE / BLIND STOP_AT_LINE) and "
                f"distance-terminated (BLIND / PROJECTION) edges -- reclassify "
                f"the offending edge or remove the pivot turn."
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

        # 4. delta leaves < the trim floor of the LINE overshoot budget -----
        # ONLY a true LINE incoming edge realises its correction as a reduction
        # of the post-detect LINE_OVERSHOOT advance, bounded by the FRONT
        # sensor-to-axle offset (review fix 3/4: STOP_AT_LINE-incoming is
        # 'sensor', not 'line', and is EXEMPT -- it takes no incoming trim). The
        # firmware clamps the reduced advance to PIVOT_TRIM_MIN_DRIVE_CM, so
        # reject when budget - delta < the floor.
        if in_regime == "line" and line_overshoot_budget_cm - delta < pivot_trim_min_drive_cm:
            raise FieldDataError(
                f"PIVOT_TURNS[{i}] ('{u}','{v}','{w}') over-travels "
                f"delta={delta:.3f} cm, leaving "
                f"{line_overshoot_budget_cm - delta:.3f} cm of the LINE overshoot "
                f"budget {line_overshoot_budget_cm:.2f} cm (the FRONT sensor-to-axle "
                f"offset; v1 pivots are FRONT-only) -- below the firmware's "
                f"trim-underflow floor {pivot_trim_min_drive_cm:.3f} cm. The runtime "
                f"would clamp the overshoot advance to that floor -- soften the "
                f"turn or remove the pivot turn on ('{u}','{v}','{w}')."
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
    entry must be a dict carrying required keys (``type``, ``u``, ``v``,
    ``distance_cm``, ``heading_deg``) and may carry optional keys
    (``primitive_mode``, ``direction``). Mode overrides are cross-checked
    against the edge ``type`` -- e.g. a :class:`LineMode` is illegal on an
    :data:`EDGE_BLIND` edge -- and the :class:`DEFAULT
    <pipeline_config.LineMode>` sentinel is rejected on any override
    (DEFAULT == "absent", so being explicit about it is a confusion).

    An anti-ambiguity cross-check rejects an entry whose unordered pair
    ``{u, v}`` already exists in ``LINE_SEGMENTS`` or ``BLIND_EDGES``
    with a *conflicting* type (:data:`EDGE_LINE` matches LINE_SEGMENTS,
    :data:`EDGE_BLIND` matches BLIND_EDGES, :data:`EDGE_PROJECTION`
    matches neither): such an entry would silently overwrite the earlier
    edge's type and geometry in :mod:`graph_builder`. A *matching*-type
    entry on an existing pair is a legitimate override layer and is
    accepted.

    Single-wheel pivots are no longer authored here. The old per-edge
    ``pivot`` / ``pre_turn_mode`` keys are gone; pivots are per-turn
    triples validated by :func:`_validate_pivot_turns`, which runs after
    this per-entry shape check.

    Imports of :class:`LineMode`, :class:`BlindMode`, and
    :class:`ProjectionMode` are performed inside the function to avoid a
    circular import with :mod:`tools.path_planner.graph_builder`.

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
        EDGE_PROJECTION,
    )
    from tools.path_planner.pipeline_config import (
        BlindMode,
        LineMode,
        ProjectionMode,
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
    # layer (distance/heading/direction) and is accepted. This mirrors, via
    # EDGES, the LINE-vs-BLIND guard in :func:`validate`.
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

    # 10. Per-turn single-wheel pivots (issue #44 per-turn model, T5). Runs
    # after the EDGES shape check so the directed-edge legs it reads are
    # well-formed, and after node/segment validation so the authored graph it
    # builds is sound. Inert (early return) on a pivot-free field.
    _validate_pivot_turns(field_data)
