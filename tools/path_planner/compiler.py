"""Path -> primitive compiler for the WRO 2026 path planner.

This module turns a node-name path (as produced by walking
:func:`tools.path_planner.planner.compute_all_pairs_next_hop`) into a flat
list of motion *primitives* the PR 2 R4 runtime dispatches as ``Drive`` /
``LineFollower`` calls. The header writer (T7) serialises this list into
``routes.cpp``.

Heading convention (must match :mod:`tools.path_planner.graph_builder`):

* Continuous float degrees in ``[0, 360)``.
* ``0 deg`` = +x axis (East).
* CCW positive (matches the R4 IMU yaw sign so PR 2's ``Drive.turnDeg(deg)``
  consumes a CCW-positive ``turn_relative`` directly without a sign flip;
  i.e. compiler's ``("turn_relative", +90)`` maps to ``Drive.turnDeg(90)``).

Primitive vocabulary (v1; see ``docs/PHASE2_DEFERRED.md`` for the
``wall_zero`` extension scheduled in T9):

* ``("turn_relative", deg)`` -- in-place turn by ``deg`` degrees,
  CCW positive. Emitted between graph edges and inside goal blind sequences.
* ``("turn_to_heading", deg)`` -- absolute target heading in ``[0, 360)``.
  Emitted exactly once per goal arrival when the chassis's heading after
  the path does not match ``GOAL_APPROACH[goal]['approach_heading_deg']``;
  uses an absolute turn so float drift from a string of relative turns
  does not accumulate at the most safety-critical handoff.
* ``("follow_line_to_intersection", None)`` -- line-follow until the next
  intersection event. Emitted for ``EDGE_LINE`` traversals.
* ``("follow_line_distance", cm)`` -- line-follow for a measured distance.
  Emitted for ``EDGE_PROJECTION`` traversals (the projection node has no
  physical line marker, so the runtime stops by encoder distance).
* ``("forward", cm)`` -- dead-reckoned forward move. Emitted for
  ``EDGE_BLIND`` traversals and for ``("forward", cm)`` entries inside
  ``GOAL_APPROACH[goal]['blind_moves']``.

Out of scope for v1:

* ``wall_zero`` calibration nodes (deferred to T9 + PR 2 follow-up).
* Reverse / backward moves (the toy field has none).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import networkx as nx

from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION

# -- public constants ---------------------------------------------------------

# Action tags. Keep these as module-level constants so a refactor (e.g.
# "rename follow_line_to_intersection -> line_to_node") flows through every
# call site, including the header writer in T7 and tests below.
ACT_TURN_REL = "turn_relative"
ACT_TURN_TO = "turn_to_heading"
ACT_FOLLOW_INT = "follow_line_to_intersection"
ACT_FOLLOW_DIST = "follow_line_distance"
ACT_FORWARD = "forward"

# Public type alias. The header writer expects this exact shape: a 2-tuple
# whose first element is a known action string and whose second element is
# either a float or ``None`` (only ``ACT_FOLLOW_INT`` carries ``None``).
Primitive = Tuple[str, Optional[float]]

# Floats coming from atan2 + modular arithmetic land within ~1e-12 of the
# nominal value; 1e-6 deg is well below any motor resolution and serves as
# the "is this turn worth emitting?" threshold throughout this module.
_TURN_EPSILON_DEG = 1e-6


# -- public helpers -----------------------------------------------------------


def normalize_turn(delta_deg: float) -> float:
    """Normalize a heading delta to ``(-180, +180]`` degrees.

    Preserves the CCW-positive sign convention. The half-open interval
    is canonical: ``+180`` represents a half-rotation, and an input of
    ``-180`` is folded to ``+180`` by the modulo so a single signed turn
    direction is always emitted (the chassis will physically prefer
    CCW for an exact 180-degree flip; this is fine because the line is
    symmetric for the reverse case).

    :param delta_deg: signed heading delta in degrees. Any finite float
        is accepted; the function does the modular reduction.
    :return: the equivalent delta in ``(-180, +180]``.
    """
    # ``a % 360`` in Python lands in ``[0, 360)`` regardless of the sign
    # of ``a`` (unlike C's ``fmod``), so the subsequent guard only has to
    # handle the ``> 180`` case.
    d = delta_deg % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def final_heading_after(
    path: List[str],
    start_heading_deg: float,
    graph: nx.DiGraph,
) -> float:
    """Return the heading the chassis would have at ``path[-1]``.

    Walks the graph the same way :func:`compile_path` does (looking up
    ``edge['heading_deg']`` per consecutive pair) but ignores any goal
    blind moves. Useful for tests that need to know whether the
    auto-correct ``turn_to_heading`` will fire without re-implementing
    the propagation logic.

    :param path: ordered list of node names. Empty / single-node paths
        yield ``start_heading_deg`` unchanged.
    :param start_heading_deg: chassis heading at ``path[0]``.
    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`.
    :return: heading at ``path[-1]`` in the original ``[0, 360)`` range
        (whatever ``edge['heading_deg']`` of the last edge was).
    :raises KeyError: if any consecutive pair is not a graph edge.
    """
    if len(path) <= 1:
        return start_heading_deg

    current = start_heading_deg
    for u, v in zip(path[:-1], path[1:]):
        # ``graph.edges[u, v]`` raises KeyError on a missing edge - we
        # let that bubble unchanged to mirror compile_path's contract.
        edge = graph.edges[u, v]
        current = edge["heading_deg"]
    return current


# -- public API: compile_path -------------------------------------------------


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
       * ``delta = normalize_turn(edge['heading_deg'] - current_heading)``.
         If ``abs(delta) > 1e-6`` emit ``(ACT_TURN_REL, delta)``.
       * Emit one motion primitive based on ``edge['type']``:
         ``EDGE_LINE`` -> ``(ACT_FOLLOW_INT, None)``,
         ``EDGE_PROJECTION`` -> ``(ACT_FOLLOW_DIST, distance_cm)``,
         ``EDGE_BLIND`` -> ``(ACT_FORWARD, distance_cm)``.
       * Update ``current_heading = edge['heading_deg']``.

    3. If ``path[-1]`` is a goal node listed in ``field_data.GOAL_APPROACH``:

       * If the chassis heading does not match
         ``approach_heading_deg``, emit a single
         ``(ACT_TURN_TO, approach_heading_deg)`` and snap
         ``current_heading`` to the absolute target. This is intentionally
         an *absolute* turn so float drift from a chain of relative turns
         (or float-quantised IMU yaw) does not propagate into the blind
         hand-off.
       * For each entry in ``blind_moves``: emit
         ``(ACT_TURN_REL, deg)`` (skipping zero-turn no-ops) or
         ``(ACT_FORWARD, cm)`` (skipping non-positive distances).

    :param path: ordered list of node names. The first element MUST be the
        node the chassis is currently at.
    :param start_heading_deg: heading at ``path[0]`` in ``[0, 360)``,
        CCW positive, ``0 deg`` = +x.
    :param graph: directed graph from :func:`build_graph`.
    :param field_data: validated field-data module / namespace exposing
        ``NODE_TYPES`` and ``GOAL_APPROACH``.
    :return: list of :data:`Primitive` tuples, ready for header emission.
    :raises ValueError: if ``start_heading_deg`` is not in ``[0, 360)``.
    :raises KeyError: if any consecutive pair in ``path`` is not a graph edge.
    """
    if not (0.0 <= start_heading_deg < 360.0):
        raise ValueError(
            f"start_heading_deg must be in [0, 360); got {start_heading_deg!r}"
        )

    out: List[Primitive] = []

    if not path:
        return out

    current_heading = start_heading_deg

    # 2. Walk the path edge-by-edge.
    for u, v in zip(path[:-1], path[1:]):
        # graph.edges[u, v] raises KeyError on missing edges; that's the
        # documented contract so we don't wrap it.
        edge = graph.edges[u, v]
        edge_heading = float(edge["heading_deg"])
        edge_type = edge["type"]

        delta = normalize_turn(edge_heading - current_heading)
        if abs(delta) > _TURN_EPSILON_DEG:
            out.append((ACT_TURN_REL, delta))

        if edge_type == EDGE_LINE:
            out.append((ACT_FOLLOW_INT, None))
        elif edge_type == EDGE_PROJECTION:
            out.append((ACT_FOLLOW_DIST, float(edge["distance_cm"])))
        elif edge_type == EDGE_BLIND:
            out.append((ACT_FORWARD, float(edge["distance_cm"])))
        else:
            # Defensive: graph_builder only emits the three known types,
            # but a future contributor swapping the tag set should fail
            # loud here rather than silently dropping a motion primitive.
            raise ValueError(
                f"unknown edge type {edge_type!r} on edge ({u!r}, {v!r})"
            )

        current_heading = edge_heading

    # 3. Goal arrival blind sequence.
    final_node = path[-1]
    is_goal = field_data.NODE_TYPES.get(final_node) == "goal"
    if is_goal and final_node in field_data.GOAL_APPROACH:
        spec = field_data.GOAL_APPROACH[final_node]
        approach = float(spec["approach_heading_deg"])

        # Insert an absolute correction iff the heading mismatch exceeds
        # the same tolerance used for relative turns. Using
        # normalize_turn() here folds e.g. 359.999 vs 0.0 into a tiny
        # delta (no spurious correction) while a real 90-degree mismatch
        # produces a clean wide-window absolute turn.
        if abs(normalize_turn(approach - current_heading)) > _TURN_EPSILON_DEG:
            out.append((ACT_TURN_TO, approach))
            current_heading = approach

        for action, value in spec["blind_moves"]:
            if action == ACT_TURN_REL:
                deg = float(value)
                if abs(deg) > _TURN_EPSILON_DEG:
                    out.append((ACT_TURN_REL, deg))
                    # Goal blind turns are relative; modulo back into
                    # [0, 360) so any subsequent absolute reasoning stays
                    # consistent with the rest of the pipeline.
                    current_heading = (current_heading + deg) % 360.0
            elif action == ACT_FORWARD:
                cm = float(value)
                if cm > 0.0:
                    out.append((ACT_FORWARD, cm))
            else:
                raise ValueError(
                    f"unsupported blind-move action {action!r} for goal "
                    f"{final_node!r}; expected one of "
                    f"({ACT_TURN_REL!r}, {ACT_FORWARD!r})"
                )

    return out


# Re-export the edge-type constants so callers (and tests) can match on
# them via ``from tools.path_planner.compiler import ...`` if they prefer
# a single import location. They remain authoritative in graph_builder.
__all__ = [
    "ACT_TURN_REL",
    "ACT_TURN_TO",
    "ACT_FOLLOW_INT",
    "ACT_FOLLOW_DIST",
    "ACT_FORWARD",
    "EDGE_LINE",
    "EDGE_PROJECTION",
    "EDGE_BLIND",
    "Primitive",
    "compile_path",
    "final_heading_after",
    "normalize_turn",
]
