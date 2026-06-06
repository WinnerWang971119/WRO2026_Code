"""Path -> primitive compiler for the WRO 2026 path planner (PR3).

Turns a node-name path (produced by walking
:func:`tools.path_planner.planner.compute_all_pairs_next_hop`) into a flat
list of motion primitives the R4 runtime dispatches as ``Drive`` /
``LineFollower`` calls. The header writer serialises this list into
``routes.cpp``.

Heading convention (matches :mod:`tools.path_planner.graph_builder`):

* Continuous float degrees in ``[0, 360)``.
* ``0 deg`` = +x axis (East), CCW positive.

Primitive vocabulary (forward hops):

* ``("turn_relative", deg)`` -- in-place turn by ``deg`` degrees, CCW
  positive. Emitted between graph edges when the heading delta exceeds
  the per-degree epsilon.
* ``("follow_line_to_intersection", None)`` -- line-follow until the
  next intersection event. Emitted for :data:`EDGE_LINE` traversals.
* ``("follow_line_to_distance", cm)`` -- line-follow stopped by encoder
  distance. Emitted for :data:`EDGE_PROJECTION` traversals -- used when
  the waypoint has no physical intersection marker beneath the strip.
* ``("forward", cm)`` -- dead-reckoned forward move. Emitted for
  :data:`EDGE_BLIND` traversals whose effective ``BlindMode`` is *not*
  ``STOP_AT_LINE``.
* ``("stop_at_line", cm)`` -- drive blind toward the expected crossing,
  arm a line detector near the endpoint, and stop on the perpendicular
  line (re-anchoring localization). Emitted for :data:`EDGE_BLIND`
  traversals whose effective ``BlindMode`` is ``STOP_AT_LINE``.

Primitive vocabulary (reverse hops, T12+):

For hops where :func:`tools.path_planner.sim.runtime.resolve_direction`
selects :class:`~tools.path_planner.sim.runtime.Direction.REAR`, the
chassis turns to ``(edge_heading + 180) % 360`` (so the rear sensor
faces along the edge) and dispatches the reverse-mirrored primitive:

* ``("line_reverse", target_node, line_mode)``
* ``("blind_reverse", target_node, distance_cm, blind_mode)``
* ``("projection_reverse", target_node, distance_cm, projection_mode)``
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

from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION

ACT_TURN_REL = "turn_relative"
ACT_FOLLOW_INT = "follow_line_to_intersection"
ACT_FOLLOW_DIST = "follow_line_to_distance"
ACT_FORWARD = "forward"
ACT_STOP_AT_LINE = "stop_at_line"

# Forward hops produce 2-tuples; reverse hops (REAR direction, T12+) produce
# 3-tuples (EDGE_LINE) or 4-tuples (EDGE_PROJECTION / EDGE_BLIND variants).
# The second slot of a reverse tuple is the target node name (str), not a float.
# BlindMode / LineMode / ProjectionMode are imported lazily inside compile_path
# to avoid a circular dependency with sim.runtime; Any stands in for those
# IntEnum types here so this module-level alias is importable without
# triggering that circular import.
Primitive = Union[
    Tuple[str, Optional[float]],      # forward: turn_rel / follow_* / forward / stop_at_line
    Tuple[str, str, Any],             # REAR EDGE_LINE: (act, target_node, line_mode)
    Tuple[str, str, float, Any],      # REAR EDGE_PROJECTION / EDGE_BLIND: (act, target_node, dist, mode)
]

_TURN_EPSILON_DEG = 1e-6


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
) -> float:
    """Return the chassis body heading the robot would have at ``path[-1]``.

    Mirrors :func:`compile_path`'s per-hop direction resolution so the two
    can never disagree: every hop is run through
    :func:`tools.path_planner.sim.runtime.resolve_direction` with the same
    inputs ``compile_path`` uses (per-edge ``direction_override``, the global
    :data:`PIPELINE_CONFIG.direction`, ``Direction.AUTO`` builder overrides,
    the running heading, the edge's forward heading, the last-hop flag, and
    the module-level kill-switch / board-count constants). The body-target
    heading is ``edge_heading`` for FRONT hops and ``(edge_heading + 180) %
    360`` for REAR hops, and the running heading advances only when the turn
    delta exceeds the epsilon -- identical to ``compile_path`` -- so a REAR
    final hop returns the ``+180`` chassis heading the robot is actually left
    on rather than the raw forward edge heading.

    :param path: ordered list of node names; first element is the chassis's
        current node.
    :param start_heading_deg: heading at ``path[0]`` in degrees.
    :param graph: directed graph from :func:`build_graph`.
    :return: chassis body heading in degrees at ``path[-1]``.
    """
    if len(path) <= 1:
        return start_heading_deg

    # Local imports mirror ``compile_path``: ``sim.runtime`` imports the ACT_*
    # tags and ``normalize_turn`` from this module, so the direction-resolution
    # machinery is deferred to call time to avoid the circular dependency.
    from tools.path_planner.pipeline_config import PIPELINE_CONFIG
    from tools.path_planner.sim.runtime import (
        BIDIRECTIONAL_AUTO_ENABLED,
        LINE_BOARD_COUNT,
        Direction,
        _read_edge_dir_override,
        resolve_direction,
    )

    current_heading = start_heading_deg
    last_index = len(path) - 1

    for i, (u, v) in enumerate(zip(path[:-1], path[1:])):
        edge = graph.edges[u, v]
        edge_heading = float(edge["heading_deg"])
        is_last_hop = ((i + 1) == last_index)

        direction = resolve_direction(
            edge_dir_override=_read_edge_dir_override(edge),
            pipeline_default=PIPELINE_CONFIG.direction,
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

        # Advance the running heading only when ``compile_path`` would emit a
        # turn (delta above the epsilon) -- below the threshold the chassis is
        # left on the prior heading, so mirroring the guard keeps the returned
        # pose byte-identical to the heading ``compile_path`` drives.
        delta = normalize_turn(body_target - current_heading)
        if abs(delta) > _TURN_EPSILON_DEG:
            current_heading = body_target

    return current_heading


def compile_path(
    path: List[str],
    start_heading_deg: float,
    graph: nx.DiGraph,
    field_data,  # noqa: ARG001 -- kept for API stability with PR2 callers
) -> List[Primitive]:
    """Compile a node-name path into a primitive sequence.

    Algorithm:

    1. Validate ``start_heading_deg in [0, 360)``. Empty path returns ``[]``.
    2. For every consecutive pair ``(u, v)``:

       * Look up ``edge = graph.edges[u, v]`` (KeyError on missing edge).
       * Resolve the chassis travel direction for the hop via
         :func:`tools.path_planner.sim.runtime.resolve_direction` (the
         same helper the runtime FSM and ``PathTrackerSim.go_to`` call).
         Inputs are the per-edge ``direction_override``, the global
         :data:`PIPELINE_CONFIG.direction`, the running ``current_heading``,
         the edge's forward heading, whether this is the last hop, and the
         module-level kill-switch / board-count constants from
         :mod:`tools.path_planner.sim.runtime`. Builder overrides
         (``arrival`` / ``route``) are always
         :class:`~tools.path_planner.sim.runtime.Direction.AUTO` at
         compile time -- those are runtime-only.
       * Compute the body-target heading: ``edge_heading`` for FRONT
         hops, ``(edge_heading + 180) % 360`` for REAR hops.
       * If ``abs(normalize_turn(body_target - current_heading))
         > 1e-6``, emit ``(ACT_TURN_REL, delta)``.
       * Emit one motion primitive based on ``edge['type']`` and
         direction. FRONT: ``EDGE_LINE`` -> ``(ACT_FOLLOW_INT, None)``,
         ``EDGE_PROJECTION`` -> ``(ACT_FOLLOW_DIST, distance_cm)``,
         ``EDGE_BLIND`` -> ``(ACT_STOP_AT_LINE, distance_cm)`` when the
         edge's effective :class:`BlindMode` is ``STOP_AT_LINE`` (via
         :func:`resolve_blind_mode`), else ``(ACT_FORWARD, distance_cm)``.
         REAR: ``EDGE_LINE`` ->
         ``(ACT_LINE_REVERSE, v, line_mode)``, ``EDGE_PROJECTION`` ->
         ``(ACT_PROJECTION_REVERSE, v, distance_cm, projection_mode)``,
         ``EDGE_BLIND`` ->
         ``(ACT_BLIND_REVERSE, v, distance_cm, blind_mode)``.
       * Update ``current_heading = body_target`` ONLY when a turn was
         actually emitted (delta exceeded the epsilon). Mirrors
         :meth:`PathTrackerSim.go_to` -- leaving the running heading
         untouched below the threshold avoids 1-ULP drift that would
         break byte-identical parity on the next hop.

    The emission shape MUST stay byte-identical to
    ``PathTrackerSim.go_to(...)`` so the offline simulator and the C++
    runtime agree on every hop. This parity is verified by
    ``test_property_random_walks`` in ``test_runtime_sim.py``.

    :param path: ordered list of node names; first element is the
        chassis's current node.
    :param start_heading_deg: heading at ``path[0]`` in ``[0, 360)``.
    :param graph: directed graph from :func:`build_graph`.
    :param field_data: unused; kept for API stability with PR2 callers.
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
    from tools.path_planner.pipeline_config import PIPELINE_CONFIG
    from tools.path_planner.sim.runtime import (
        ACT_BLIND_REVERSE,
        ACT_LINE_REVERSE,
        ACT_PROJECTION_REVERSE,
        ACT_STOP_AT_LINE_REVERSE,
        BIDIRECTIONAL_AUTO_ENABLED,
        LINE_BOARD_COUNT,
        Direction,
        StopAtLine,
        _read_edge_dir_override,
        resolve_blind_mode,
        resolve_direction,
        resolve_line_mode,
        resolve_projection_mode,
        resolve_stop_at_line,
    )

    current_heading = start_heading_deg
    last_index = len(path) - 1

    for i, (u, v) in enumerate(zip(path[:-1], path[1:])):
        edge = graph.edges[u, v]
        edge_heading = float(edge["heading_deg"])
        edge_type = edge["type"]
        is_last_hop = ((i + 1) == last_index)

        direction = resolve_direction(
            edge_dir_override=_read_edge_dir_override(edge),
            pipeline_default=PIPELINE_CONFIG.direction,
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
        if abs(delta) > _TURN_EPSILON_DEG:
            out.append((ACT_TURN_REL, delta))
            # Only update the running heading when a turn is actually
            # emitted -- mirrors :meth:`PathTrackerSim.go_to` which leaves
            # ``chassis.heading_deg`` untouched when the delta is below
            # the epsilon. Skipping this guard would diverge by 1 ULP on
            # the next hop and break the byte-identical parity check
            # (TC18 / ``test_property_random_walks``).
            current_heading = body_target

        if direction == Direction.REAR:
            if edge_type == EDGE_LINE:
                out.append((ACT_LINE_REVERSE, v, resolve_line_mode(edge)))
            elif edge_type == EDGE_PROJECTION:
                out.append((
                    ACT_PROJECTION_REVERSE,
                    v,
                    float(edge["distance_cm"]),
                    resolve_projection_mode(edge),
                ))
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
                    resolve_blind_mode(edge),
                    PIPELINE_CONFIG.blind,
                    direction,
                    LINE_BOARD_COUNT,
                )
                if seek:
                    out.append((
                        ACT_STOP_AT_LINE_REVERSE,
                        v,
                        float(edge["distance_cm"]),
                        resolve_blind_mode(edge),
                    ))
                else:
                    out.append((
                        ACT_BLIND_REVERSE,
                        v,
                        float(edge["distance_cm"]),
                        resolve_blind_mode(edge),
                    ))
            else:
                raise ValueError(
                    f"unknown edge type {edge_type!r} on edge ({u!r}, {v!r})"
                )
        else:
            if edge_type == EDGE_LINE:
                out.append((ACT_FOLLOW_INT, None))
            elif edge_type == EDGE_PROJECTION:
                out.append((ACT_FOLLOW_DIST, float(edge["distance_cm"])))
            elif edge_type == EDGE_BLIND:
                # FRONT blind is mode-sensitive via the unified resolver: a
                # STOP_AT_LINE edge stops on the physical crossing (and
                # re-anchors localization downstream) instead of dead-reckoning
                # forward. Every other BlindMode keeps the plain forward hop.
                # Mirrors the FRONT branch of ``PathTrackerSim.go_to`` in
                # ``sim/runtime.py`` (T6); the two emission streams must stay
                # byte-identical (``test_property_random_walks`` / TC12).
                seek, _motion_mode = resolve_stop_at_line(
                    StopAtLine.AUTO,
                    resolve_blind_mode(edge),
                    PIPELINE_CONFIG.blind,
                    direction,
                    LINE_BOARD_COUNT,
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
    "EDGE_PROJECTION",
    "Primitive",
    "compile_path",
    "final_heading_after",
    "normalize_turn",
]
