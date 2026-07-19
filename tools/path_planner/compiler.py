"""Path -> primitive compiler for the WRO 2026 path planner (PR3).

Turns a node-name path (produced by walking
:func:`tools.path_planner.planner.compute_all_pairs_next_hop`) into a flat
list of motion primitives the R4 runtime dispatches as ``Drive`` /
``LineFollower`` calls. The header writer serialises this list into
``routes.cpp``.

Heading convention (matches :mod:`tools.path_planner.graph_builder`):

* Continuous float degrees in ``[0, 360)``.
* ``0 deg`` = +x axis (East), CCW positive.

The per-hop primitive is derived from ``(edge_type, destination_node_type)``
-- the source node type never matters. ``EDGE_PROJECTION`` no longer exists;
a line-follow segment terminated by encoder distance (no physical
intersection beneath the strip) is now expressed by giving the *destination*
node the ``"projection"`` type, probed locally via
:func:`tools.path_planner.graph_builder.is_projection_node`.

================  ===========================  ===========================  ====================
Edge \\ dest      line node                    projection node              blind node
================  ===========================  ===========================  ====================
``EDGE_LINE``     follow-to-intersection       follow-to-distance           illegal (validator)
``EDGE_BLIND``   dead-reckon (seek via flag)  auto-seek stop-at-line       pure dead-reckon
================  ===========================  ===========================  ====================

Primitive vocabulary (forward hops):

* ``("turn_relative", deg)`` -- in-place turn by ``deg`` degrees, CCW
  positive. Emitted between graph edges when the heading delta exceeds
  the turn-dispatch tolerance
  :data:`tools.path_planner.sim.runtime.NAV_TURN_TOLERANCE_DEG` (0.5°) --
  the same gate the firmware and the sim use, so a real field's tiny edge
  residuals never surface as spurious micro-turns (#147).
* ``("follow_line_to_intersection", None)`` -- line-follow until the
  next intersection event. Emitted for an :data:`EDGE_LINE` hop into a
  ``line`` node.
* ``("follow_line_to_distance", cm)`` -- line-follow stopped by encoder
  distance (no junction wait). Emitted for an :data:`EDGE_LINE` hop into
  a ``projection`` node -- the waypoint sits on the line but has no
  physical intersection marker beneath the strip. Its motion profile
  comes from the LINE edge's own ``line_mode`` / ``line_mode_override``.
* ``("forward", cm)`` -- dead-reckoned forward move. Emitted for an
  :data:`EDGE_BLIND` hop whose effective ``BlindMode`` is *not*
  ``STOP_AT_LINE`` and that does not auto-seek.
* ``("stop_at_line", cm)`` -- drive blind toward the expected crossing,
  arm a line detector near the endpoint, and stop on the perpendicular
  line (re-anchoring localization). Emitted for an :data:`EDGE_BLIND`
  hop that seeks -- whose effective ``BlindMode`` is ``STOP_AT_LINE``,
  or whose destination is a ``projection`` node and the field-wide
  ``BLIND_PROJECTION_AUTO_SEEK`` toggle is set (the local auto-rule).

Primitive vocabulary (reverse hops, T12+):

For hops where :func:`tools.path_planner.sim.runtime.resolve_direction`
selects :class:`~tools.path_planner.sim.runtime.Direction.REAR`, the
chassis turns to ``(edge_heading + 180) % 360`` (so the rear sensor
faces along the edge) and dispatches the reverse-mirrored primitive:

* ``("line_reverse", target_node, line_mode)`` -- a REAR :data:`EDGE_LINE`
  hop into a ``line`` (junction) node: reverse line-follow terminated by
  the next intersection event.
* ``("projection_reverse", target_node, distance_cm, line_mode)`` -- a REAR
  :data:`EDGE_LINE` hop into a ``projection`` node. Mirrors the FRONT
  LINE->projection ``ACT_FOLLOW_DIST``: the projection node has no junction
  marker beneath the strip, so the reverse follow stops by encoder distance
  (no intersection wait) instead of sensing a crossing. ``distance_cm`` is
  the RAW ``EDGES`` cell (``edge['distance_cm']``); the sim's pivot-trimmed
  ``effective_dist_cm`` is its own displacement oracle, exactly as the
  FORWARD LINE->projection hop differs (compiler raw vs sim trimmed).
* ``("blind_reverse", target_node, distance_cm, blind_mode)``
* ``("stop_at_line_reverse", target_node, distance_cm, blind_mode)`` --
  a REAR blind hop is mode-sensitive (mirrors the FRONT branch): the
  per-hop decision runs through
  :func:`tools.path_planner.sim.runtime.resolve_stop_at_line` (call flag
  ``AUTO``). A 2-board REAR ``STOP_AT_LINE`` hop emits
  ``ACT_STOP_AT_LINE_REVERSE`` and re-anchors via the rear board's
  crossing detection; any non-``STOP_AT_LINE`` REAR edge -- or a
  forced-REAR ``STOP_AT_LINE`` hop on a 1-board chassis, where the
  resolver returns ``seek=False`` (no rear board to seek with) -- keeps
  the dead-reckoned ``ACT_BLIND_REVERSE``. The 4th tuple element is the
  RAW edge ``blind_mode`` (``resolve_blind_mode(edge)``), not the
  resolver's motion mode, identical to ``sim/runtime.py``.

The emission shape must stay byte-identical to
``PathTrackerSim.go_to(...)`` so the offline simulator and the firmware
agree on every hop (TC10/TC12 cross-module parity).
"""

from __future__ import annotations

from typing import Any, List, Optional, Tuple, Union

import networkx as nx

from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    is_projection_node,
)

ACT_TURN_REL = "turn_relative"
ACT_FOLLOW_INT = "follow_line_to_intersection"
ACT_FOLLOW_DIST = "follow_line_to_distance"
ACT_FORWARD = "forward"
ACT_STOP_AT_LINE = "stop_at_line"

# Forward hops produce 2-tuples; reverse hops (REAR direction, T12+) produce
# 3-tuples (EDGE_LINE into a junction node) or 4-tuples (EDGE_LINE into a
# projection node, plus the EDGE_BLIND variants).
# The second slot of a reverse tuple is the target node name (str), not a float.
# BlindMode / SpeedTier are imported lazily inside compile_path to avoid a
# circular dependency with sim.runtime; Any stands in for those IntEnum types
# here so this module-level alias is importable without triggering that
# circular import.
Primitive = Union[
    Tuple[str, Optional[float]],      # forward: turn_rel / follow_* / forward / stop_at_line
    Tuple[str, str, Any],             # REAR EDGE_LINE -> junction: (act, target_node, line_mode)
    Tuple[str, str, float, Any],      # REAR EDGE_LINE -> projection / EDGE_BLIND: (act, target_node, dist, mode)
]


def normalize_turn(delta_deg: float) -> float:
    """Normalize a heading delta to ``(-180, +180]`` degrees."""
    d = delta_deg % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def final_heading_after(
    path: List[str],
    start_heading_deg: float,
    graph: nx.DiGraph,
    field_data=None,
) -> float:
    """Return the chassis body heading the robot would have at ``path[-1]``.

    Mirrors :func:`compile_path`'s per-hop direction resolution: every hop is
    run through :func:`tools.path_planner.sim.runtime.resolve_direction` with
    the same inputs ``compile_path`` uses (per-edge ``direction_override``,
    the field-sourced ``cfg.direction`` -- see ``field_data`` below --,
    ``Direction.AUTO`` builder overrides, the running heading, the edge's
    forward heading, the last-hop flag, and the module-level kill-switch /
    board-count constants). The body-target heading is ``edge_heading`` for
    FRONT hops and ``(edge_heading + 180) % 360`` for REAR hops, and the
    running heading advances onto that body-target UNCONDITIONALLY every hop
    (the one logical-heading rule of #147, identical to ``compile_path`` and
    ``_emit_turn_to``) -- so a REAR final hop returns the ``+180`` chassis
    heading the robot is actually left on rather than the raw forward edge
    heading, and a chain of individually sub-tolerance hops can never leave a
    residual that this function would report differently from the heading
    ``compile_path`` drives.

    This function can only be guaranteed to agree with ``compile_path`` when
    BOTH are given the SAME ``field_data`` (#148). The pipeline direction
    default is resolved per-call via
    :meth:`~tools.path_planner.pipeline_config.PipelineConfig.from_field_data`,
    so calling this function with one ``field_data`` (or none) while
    ``compile_path`` is compiling the same path against a different
    ``field_data`` can legitimately resolve a different ``pipeline_default``
    and disagree on a field that authors a non-default
    ``PIPELINE_DIRECTION_MODE`` -- there is no implicit sharing between the
    two calls.

    :param path: ordered list of node names; first element is the chassis's
        current node.
    :param start_heading_deg: heading at ``path[0]`` in degrees.
    :param graph: directed graph from :func:`build_graph`.
    :param field_data: field data module (or any object exposing the optional
        ``PIPELINE_*`` attributes) to field-source the pipeline direction
        default from. ``None`` (the default) makes
        :meth:`~tools.path_planner.pipeline_config.PipelineConfig.from_field_data`
        fall back to the module-level
        :data:`~tools.path_planner.pipeline_config.PIPELINE_CONFIG` singleton
        (every ``getattr`` lookup misses) -- byte-identical to this
        function's behavior before ``field_data`` existed, so every
        pre-#148 caller that omits this argument is unaffected.
    :return: chassis body heading in degrees at ``path[-1]``.
    """
    if len(path) <= 1:
        return start_heading_deg

    # Local imports mirror ``compile_path``: ``sim.runtime`` imports the ACT_*
    # tags and ``normalize_turn`` from this module, so the direction-resolution
    # machinery is deferred to call time to avoid the circular dependency.
    from tools.path_planner.pipeline_config import PipelineConfig
    from tools.path_planner.sim.runtime import (
        BIDIRECTIONAL_AUTO_ENABLED,
        LINE_BOARD_COUNT,
        Direction,
        _read_edge_dir_override,
        resolve_direction,
    )

    # Field-source the pipeline direction default exactly as ``compile_path``
    # does: a ``field_data`` that omits ``PIPELINE_DIRECTION_MODE`` (including
    # ``None``) falls back to the ``PIPELINE_CONFIG`` singleton, so this stays
    # byte-identical to the old singleton read for every caller that does not
    # pass ``field_data``.
    cfg = PipelineConfig.from_field_data(field_data)

    current_heading = start_heading_deg
    last_index = len(path) - 1

    for i, (u, v) in enumerate(zip(path[:-1], path[1:])):
        edge = graph.edges[u, v]
        edge_heading = float(edge["heading_deg"])
        is_last_hop = ((i + 1) == last_index)

        direction = resolve_direction(
            edge_dir_override=_read_edge_dir_override(edge),
            pipeline_default=cfg.direction,
            builder_arrival=Direction.AUTO,
            builder_route=Direction.AUTO,
            current_heading=current_heading,
            edge_heading=edge_heading,
            is_last_hop=is_last_hop,
            bidirectional_enabled=BIDIRECTIONAL_AUTO_ENABLED,
            line_board_count=LINE_BOARD_COUNT,
        )

        if direction == Direction.REAR:
            body_target = (edge_heading + 180.0) % 360.0
        else:
            body_target = edge_heading % 360.0

        # One logical-heading rule (#147): advance the running heading onto the
        # hop's body-target UNCONDITIONALLY -- whether or not ``compile_path``
        # would emit a turn for this hop. ``compile_path`` and ``_emit_turn_to``
        # do the same, so the three surfaces track one logical heading and a
        # chain of individually sub-tolerance hops can never accumulate a
        # residual that would make this function disagree with the heading
        # ``compile_path`` drives. (Turn EMISSION is gated on
        # ``NAV_TURN_TOLERANCE_DEG`` in ``compile_path``; here there is no
        # emission, so the heading simply follows the edge.)
        current_heading = body_target

    return current_heading


def compile_path(
    path: List[str],
    start_heading_deg: float,
    graph: nx.DiGraph,
    field_data,
) -> List[Primitive]:
    """Compile a node-name path into a primitive sequence.

    Algorithm:

    1. Validate ``start_heading_deg in [0, 360)``. Empty path returns ``[]``.
    2. For every consecutive pair ``(u, v)``:

       * Look up ``edge = graph.edges[u, v]`` (KeyError on missing edge).
       * Resolve the chassis travel direction for the hop via
         :func:`tools.path_planner.sim.runtime.resolve_direction` (the
         same helper the runtime FSM and ``PathTrackerSim.go_to`` call).
         Inputs are the per-edge ``direction_override``, the field-sourced
         ``cfg.direction`` (see ``field_data`` below), the running
         ``current_heading``, the edge's forward heading, whether this is
         the last hop, and the module-level kill-switch / board-count
         constants from :mod:`tools.path_planner.sim.runtime`. Builder overrides
         (``arrival`` / ``route``) are always
         :class:`~tools.path_planner.sim.runtime.Direction.AUTO` at
         compile time -- those are runtime-only.
       * Compute the body-target heading: ``edge_heading`` for FRONT
         hops, ``(edge_heading + 180) % 360`` for REAR hops.
       * If ``abs(normalize_turn(body_target - current_heading))
         > NAV_TURN_TOLERANCE_DEG`` (0.5°), emit ``(ACT_TURN_REL, delta)``.
       * Emit one motion primitive based on ``(edge['type'],
         destination_node_type)`` and direction. The destination node
         type is read **locally** from ``v`` (the end of this hop) via
         :func:`is_projection_node` -- there is no goal-relative
         ``NEXT_HOP`` peek.
         FRONT ``EDGE_LINE``: into a ``projection`` node ->
         ``(ACT_FOLLOW_DIST, distance_cm)`` (line-follow stopped by
         encoder distance, no junction wait); otherwise ->
         ``(ACT_FOLLOW_INT, None)``.
         FRONT ``EDGE_BLIND``: ``(ACT_STOP_AT_LINE, distance_cm)`` when
         :func:`resolve_stop_at_line` returns ``seek=True`` (the edge's
         effective :class:`BlindMode` is ``STOP_AT_LINE``, or the local
         auto-rule fires because ``field_data.BLIND_PROJECTION_AUTO_SEEK``
         is set and ``v`` is a ``projection`` node), else
         ``(ACT_FORWARD, distance_cm)``.
         REAR ``EDGE_LINE``: into a ``projection`` node ->
         ``(ACT_PROJECTION_REVERSE, v, distance_cm, line_mode)`` (reverse
         line-follow stopped by encoder distance, no junction wait, the REAR
         mirror of the FRONT ``ACT_FOLLOW_DIST``); otherwise ->
         ``(ACT_LINE_REVERSE, v, line_mode)``.
         REAR ``EDGE_BLIND`` -> ``(ACT_STOP_AT_LINE_REVERSE, ...)`` /
         ``(ACT_BLIND_REVERSE, v, distance_cm, blind_mode)``.
       * Update ``current_heading = body_target`` UNCONDITIONALLY every hop
         (the one logical-heading rule of #147) -- whether or not a turn was
         emitted. Mirrors :meth:`PathTrackerSim.go_to` /
         :meth:`PathTrackerSim._emit_turn_to`, which snaps the chassis heading
         onto the target even on a skipped sub-tolerance turn, so a chain of
         individually sub-tolerance hops can never accumulate into a spurious
         compiler-only turn on a later hop.

    The emission shape MUST stay byte-identical to
    ``PathTrackerSim.go_to(...)`` so the offline simulator and the C++
    runtime agree on every hop. This parity is verified by
    ``test_property_random_walks`` in ``test_runtime_sim.py``.

    :param path: ordered list of node names; first element is the
        chassis's current node.
    :param start_heading_deg: heading at ``path[0]`` in ``[0, 360)``.
    :param graph: directed graph from :func:`build_graph`.
    :param field_data: read for two things. (1) ``BLIND_PROJECTION_AUTO_SEEK``
        (the local auto-rule that fires a ``stop_at_line`` /
        ``stop_at_line_reverse`` seek on a blind hop whose *destination* node
        is a ``projection`` node); ``getattr`` fallback to ``True`` keeps old
        callers that predate T11 working unchanged. (2) the four mission-wide
        ``PIPELINE_*`` defaults, via
        :meth:`PipelineConfig.from_field_data` -- the same field-sourced
        ``cfg`` :func:`final_heading_after` resolves, which is what lets the
        two agree (#148). ``None`` falls back to the ``PIPELINE_CONFIG``
        singleton for every absent key, so old callers stay unchanged.
    :return: list of :data:`Primitive` tuples.
    :raises ValueError: if ``start_heading_deg`` is not in ``[0, 360)``
        or an edge has an unknown type tag.
    :raises KeyError: if any consecutive pair is not a graph edge.
    """
    if not (0.0 <= start_heading_deg < 360.0):
        raise ValueError(
            f"start_heading_deg must be in [0, 360); got {start_heading_deg!r}"
        )

    out: List[Primitive] = []
    if not path:
        return out

    # Local imports to avoid a circular dependency: ``sim.runtime`` imports
    # the ACT_* primitive tags and ``normalize_turn`` from this module. We
    # only need direction-resolution machinery inside the loop, so deferring
    # the import here keeps the module-level graph clean.
    from tools.path_planner.pipeline_config import PipelineConfig
    from tools.path_planner.sim.runtime import (
        ACT_BLIND_REVERSE,
        ACT_LINE_REVERSE,
        ACT_PROJECTION_REVERSE,
        ACT_STOP_AT_LINE_REVERSE,
        BIDIRECTIONAL_AUTO_ENABLED,
        LINE_BOARD_COUNT,
        NAV_TURN_TOLERANCE_DEG,
        Direction,
        StopAtLine,
        _read_edge_dir_override,
        _read_edge_stop_at_line_override,
        resolve_blind_mode,
        resolve_direction,
        resolve_speed,
        resolve_stop_at_line,
    )

    # R4: resolve the field-sourced pipeline defaults ONCE and thread the same
    # ``cfg`` through every direction / line / blind resolution below, so a
    # field-set ``PIPELINE_*_MODE`` reaches the compiled primitives exactly as
    # it reaches the emitted ``routes.cpp`` (and the sim, which threads the
    # identical config). ``from_field_data`` falls back to the
    # ``PIPELINE_CONFIG`` singleton for a field that omits the keys, so the
    # many ``SimpleNamespace`` fixtures stay byte-identical.
    cfg = PipelineConfig.from_field_data(field_data)

    current_heading = start_heading_deg
    last_index = len(path) - 1

    for i, (u, v) in enumerate(zip(path[:-1], path[1:])):
        edge = graph.edges[u, v]
        edge_heading = float(edge["heading_deg"])
        edge_type = edge["type"]
        is_last_hop = ((i + 1) == last_index)

        # auto_rule_active CONTRACT (issue #64): the BLIND->projection
        # auto-seek is a purely LOCAL decision on THIS hop's destination
        # node ``v`` -- no goal-relative ``path[i+2]`` / ``NEXT_HOP[next][goal]``
        # peek. It fires only when the edge is BLIND, ``v`` is a projection
        # node, and the field-wide toggle is on. The same bool flows
        # unchanged into the FROZEN ``resolve_stop_at_line`` resolver
        # (computing it differently is the ONLY seek-path change). The sim
        # (T5) and firmware (T10) compute the identical local predicate via
        # ``is_projection_node`` / ``getNodeType(dest) == PROJECTION``.
        auto_rule_active = (
            edge_type == EDGE_BLIND
            and is_projection_node(graph, v)
            and getattr(field_data, "BLIND_PROJECTION_AUTO_SEEK", True)
        )

        direction = resolve_direction(
            edge_dir_override=_read_edge_dir_override(edge),
            pipeline_default=cfg.direction,
            builder_arrival=Direction.AUTO,
            builder_route=Direction.AUTO,
            current_heading=current_heading,
            edge_heading=edge_heading,
            is_last_hop=is_last_hop,
            bidirectional_enabled=BIDIRECTIONAL_AUTO_ENABLED,
            line_board_count=LINE_BOARD_COUNT,
        )

        # Body target heading: forward edge for FRONT, opposite for REAR
        # (so the rear sensor faces along the edge). Mirrors
        # ``_body_heading_for`` in ``sim.runtime``.
        if direction == Direction.REAR:
            body_target = (edge_heading + 180.0) % 360.0
        else:
            body_target = edge_heading % 360.0

        delta = normalize_turn(body_target - current_heading)
        if abs(delta) > NAV_TURN_TOLERANCE_DEG:
            out.append((ACT_TURN_REL, delta))
        # One logical-heading rule (#147): advance the running heading onto
        # ``body_target`` UNCONDITIONALLY -- whether or not a turn was emitted
        # above. :meth:`PathTrackerSim._emit_turn_to` snaps the chassis heading
        # onto the target even on a skipped sub-tolerance turn, and
        # :func:`final_heading_after` advances identically, so the three
        # surfaces track one logical heading. Leaving it on the prior heading
        # below the tolerance would let a chain of individually sub-tolerance
        # hops accumulate a >tolerance drift and surface a spurious
        # compiler-only ``ACT_TURN_REL`` on a later hop (the #147 bug). Emission
        # stays gated on ``NAV_TURN_TOLERANCE_DEG`` above; only the heading
        # advance is unconditional.
        current_heading = body_target

        if direction == Direction.REAR:
            if edge_type == EDGE_LINE:
                # The destination node type (read LOCALLY from ``v``) decides
                # how the REAR LINE follow terminates, mirroring the FRONT
                # branch. A ``projection`` node sits on the line but at NO
                # junction, so an intersection-terminated reverse follow can
                # never be sensed there; the hop instead line-follows backward
                # yet stops by encoder distance (ACT_PROJECTION_REVERSE), a
                # distance-terminated reverse. Any other destination (a real
                # junction) keeps the intersection-terminated ACT_LINE_REVERSE.
                # The distance payload is the RAW EDGES cell ``distance_cm`` --
                # the sim's ``effective_dist_cm`` pivot-trim is the sim-side
                # displacement oracle, identical to how the FORWARD
                # LINE->projection hop differs (compiler raw vs sim trimmed).
                # Mirrors the REAR LINE->projection branch of
                # ``PathTrackerSim.go_to`` in ``sim/runtime.py`` cell-for-cell.
                #
                # Pass-through is NOT compiled (T20, see the FRONT branch below):
                # a reverse ``line(A)–projection(B)–line(C)`` collapse is an FSM
                # transition the firmware owns, so the compiler emits the plain
                # ``ACT_PROJECTION_REVERSE`` here; the sim emits the sim-only
                # ``ACT_PROJECTION_REVERSE_PASS`` on the pass-through.
                if is_projection_node(graph, v):
                    out.append((
                        ACT_PROJECTION_REVERSE,
                        v,
                        float(edge["distance_cm"]),
                        resolve_speed(edge, cfg),
                    ))
                else:
                    out.append((ACT_LINE_REVERSE, v, resolve_speed(edge, cfg)))
            elif edge_type == EDGE_BLIND:
                # REAR blind is mode-sensitive via the unified resolver: a
                # 2-board REAR STOP_AT_LINE hop seeks with the rear board and
                # re-anchors localization (ACT_STOP_AT_LINE_REVERSE), while a
                # non-STOP_AT_LINE edge -- or a forced-REAR STOP_AT_LINE hop on
                # a 1-board chassis (resolver returns seek=False) -- dead-reckons
                # backward (ACT_BLIND_REVERSE). The 4th tuple element is the RAW
                # edge mode (``resolve_blind_mode(edge)``), NOT ``_motion_mode``,
                # to stay byte-identical to the REAR branch of
                # ``PathTrackerSim.go_to`` in ``sim/runtime.py`` (TC10/TC12 gate
                # the two emission streams against each other).
                seek, _motion_mode = resolve_stop_at_line(
                    StopAtLine.AUTO,
                    _read_edge_stop_at_line_override(edge),
                    resolve_blind_mode(edge, cfg),
                    cfg.blind,
                    direction,
                    LINE_BOARD_COUNT,
                    auto_rule_active,
                )
                if seek:
                    out.append((
                        ACT_STOP_AT_LINE_REVERSE,
                        v,
                        float(edge["distance_cm"]),
                        resolve_blind_mode(edge, cfg),
                    ))
                else:
                    out.append((
                        ACT_BLIND_REVERSE,
                        v,
                        float(edge["distance_cm"]),
                        resolve_blind_mode(edge, cfg),
                    ))
            else:
                raise ValueError(
                    f"unknown edge type {edge_type!r} on edge ({u!r}, {v!r})"
                )
        else:
            if edge_type == EDGE_LINE:
                # The destination node type (read LOCALLY from ``v``) decides
                # how the LINE follow terminates: a ``projection`` node sits on
                # the line but at NO junction, so the hop line-follows yet stops
                # by encoder distance (no intersection wait) -- its motion
                # profile still comes from the LINE edge's own line_mode /
                # line_mode_override, not a projection mode. Any other
                # destination (a real junction) follows to the intersection
                # marker and re-anchors there.
                #
                # PASS-THROUGH IS NOT COMPILED (T20, issue #64). A
                # ``line(A)–projection(B)–line(C)`` triple routed PAST B collapses
                # to one continuous A→C follow at runtime, but -- exactly like the
                # collinear line-line-line ADR-023 pass-through -- that collapse is
                # an FSM *transition* the firmware owns (the PROJECTION_FOLLOW
                # `_projPassThrough` distance-gate branch, T19), NOT a compiled
                # primitive. The compiler stays pass-through-UNAWARE: it emits the
                # plain ``ACT_FOLLOW_DIST`` here whether or not B would pass through,
                # just as it emits ``ACT_FOLLOW_INT`` for a line-line-line collapse
                # without ever emitting the sim-only ``ACT_LINE_PASS``. The sim
                # mirrors the firmware by emitting ``ACT_FOLLOW_DIST_PASS`` on the
                # pass-through; the compiler↔sim parity tests are constructed on
                # graphs whose NEXT_HOP never line-follows past a both-LINE-flanked
                # projection node (the toy field keeps a direct A-B edge), so the
                # two streams agree without the compiler tracking pass-through.
                if is_projection_node(graph, v):
                    out.append((ACT_FOLLOW_DIST, float(edge["distance_cm"])))
                else:
                    out.append((ACT_FOLLOW_INT, None))
            elif edge_type == EDGE_BLIND:
                # FRONT blind is mode-sensitive via the unified resolver: a
                # STOP_AT_LINE edge stops on the physical crossing (and
                # re-anchors localization downstream) instead of dead-reckoning
                # forward. Every other BlindMode keeps the plain forward hop.
                # Mirrors the FRONT branch of ``PathTrackerSim.go_to`` in
                # ``sim/runtime.py`` (T6); the two emission streams must stay
                # byte-identical (``test_property_random_walks`` / TC12).
                #
                seek, _motion_mode = resolve_stop_at_line(
                    StopAtLine.AUTO,
                    _read_edge_stop_at_line_override(edge),
                    resolve_blind_mode(edge, cfg),
                    cfg.blind,
                    direction,
                    LINE_BOARD_COUNT,
                    auto_rule_active,
                )
                if seek:
                    out.append((ACT_STOP_AT_LINE, float(edge["distance_cm"])))
                else:
                    out.append((ACT_FORWARD, float(edge["distance_cm"])))
            else:
                raise ValueError(
                    f"unknown edge type {edge_type!r} on edge ({u!r}, {v!r})"
                )

    return out


__all__ = [
    "ACT_TURN_REL",
    "ACT_FOLLOW_INT",
    "ACT_FOLLOW_DIST",
    "ACT_FORWARD",
    "ACT_STOP_AT_LINE",
    "EDGE_LINE",
    "EDGE_BLIND",
    "Primitive",
    "compile_path",
    "final_heading_after",
    "normalize_turn",
]
