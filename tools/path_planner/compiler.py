"""Path -> primitive compiler for the WRO 2026 path planner (PR3).

Turns a node-name path (produced by walking
:func:`tools.path_planner.planner.compute_all_pairs_next_hop`) into a flat
list of motion primitives the R4 runtime dispatches as ``Drive`` /
``LineFollower`` calls. The header writer serialises this list into
``routes.cpp``.

Heading convention (matches :mod:`tools.path_planner.graph_builder`):

* Continuous float degrees in ``[0, 360)``.
* ``0 deg`` = +x axis (East), CCW positive.

Primitive vocabulary:

* ``("turn_relative", deg)`` -- in-place turn by ``deg`` degrees, CCW
  positive. Emitted between graph edges when the heading delta exceeds
  the per-degree epsilon.
* ``("follow_line_to_intersection", None)`` -- line-follow until the
  next intersection event. Emitted for :data:`EDGE_LINE` traversals.
* ``("follow_line_to_distance", cm)`` -- line-follow stopped by encoder
  distance. Emitted for :data:`EDGE_PROJECTION` traversals -- used when
  the waypoint has no physical intersection marker beneath the strip.
* ``("forward", cm)`` -- dead-reckoned forward move. Emitted for
  :data:`EDGE_BLIND` traversals.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import networkx as nx

from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION

ACT_TURN_REL = "turn_relative"
ACT_FOLLOW_INT = "follow_line_to_intersection"
ACT_FOLLOW_DIST = "follow_line_to_distance"
ACT_FORWARD = "forward"

Primitive = Tuple[str, Optional[float]]

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
    """Return the heading the chassis would have at ``path[-1]``."""
    if len(path) <= 1:
        return start_heading_deg
    current = start_heading_deg
    for u, v in zip(path[:-1], path[1:]):
        current = graph.edges[u, v]["heading_deg"]
    return current


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
       * If ``abs(normalize_turn(edge['heading_deg'] - current_heading))
         > 1e-6``, emit ``(ACT_TURN_REL, delta)``.
       * Emit one motion primitive based on ``edge['type']``:
         ``EDGE_LINE`` -> ``(ACT_FOLLOW_INT, None)``,
         ``EDGE_PROJECTION`` -> ``(ACT_FOLLOW_DIST, distance_cm)``,
         ``EDGE_BLIND`` -> ``(ACT_FORWARD, distance_cm)``.
       * Update ``current_heading = edge['heading_deg']``.

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

    current_heading = start_heading_deg

    for u, v in zip(path[:-1], path[1:]):
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
            raise ValueError(
                f"unknown edge type {edge_type!r} on edge ({u!r}, {v!r})"
            )

        current_heading = edge_heading

    return out


__all__ = [
    "ACT_TURN_REL",
    "ACT_FOLLOW_INT",
    "ACT_FOLLOW_DIST",
    "ACT_FORWARD",
    "EDGE_LINE",
    "EDGE_BLIND",
    "EDGE_PROJECTION",
    "Primitive",
    "compile_path",
    "final_heading_after",
    "normalize_turn",
]
