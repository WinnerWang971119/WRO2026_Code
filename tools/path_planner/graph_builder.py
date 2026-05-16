"""Graph builder + obstacle inflation for the WRO 2026 path planner (PR3).

Turns the validated ``field_data`` into a ``networkx.DiGraph`` ready for
:mod:`tools.path_planner.planner` (Dijkstra) and
:mod:`tools.path_planner.compiler` (heading propagation).

Heading convention (matches PR1/PR2 PathTracker):

* Continuous float degrees in ``[0, 360)``.
* ``0 deg`` = +x axis (East).
* CCW positive.
* Forward edge heading: ``math.degrees(math.atan2(dy, dx)) % 360.0``.
* Reverse heading: ``(forward + 180.0) % 360.0``.

Edge attributes (every edge in the returned graph carries these):

* ``type``        : :data:`EDGE_LINE` or :data:`EDGE_BLIND`.
* ``distance_cm`` : float, Euclidean length of the directed edge.
* ``heading_deg`` : float in ``[0, 360)`` per the convention above.

Node attributes:

* ``x_cm``, ``y_cm`` : float coordinates.
* ``type``           : ``"line" | "blind"`` (from ``field_data.NODE_TYPES``).

PR3 schema vs PR2: no projection nodes, no goal-approach blind sequences.
Off-line goals are first-class ``blind`` nodes joined to the line graph
by explicit :data:`field_data.BLIND_EDGES` (bidirectional by default,
so the chassis can return the way it came).
"""

from __future__ import annotations

import math
import warnings
from typing import Iterable, List, Sequence, Tuple

import networkx as nx
from shapely.geometry import LineString, Polygon, box

from tools.path_planner.validator import validate

EDGE_LINE = "LINE"
EDGE_BLIND = "BLIND"

# Back-compat aliases — header_writer / compiler / sim still reference
# these names. Projection edges are no longer emitted but the tag is kept
# so a future PR can re-introduce them without churning every import.
EDGE_PROJECTION = "PROJECTION"


def _heading_deg(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx)) % 360.0


def _reverse_heading(heading_deg: float) -> float:
    return (heading_deg + 180.0) % 360.0


def _euclid(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def inflate_obstacles(
    obstacles: Iterable[Tuple[float, float, float, float, str]],
    robot_radius_cm: float,
) -> List[Polygon]:
    """Return one inflated :class:`shapely.geometry.Polygon` per obstacle.

    Each axis-aligned rectangle ``(x_min, y_min, x_max, y_max, kind)`` is
    expanded by ``robot_radius_cm`` via ``buffer(..., cap_style=2, join_style=2)``
    so the inflated footprint stays rectangular.

    :param obstacles: 5-tuples ``(x_min, y_min, x_max, y_max, kind)``.
    :param robot_radius_cm: radius to inflate by, in centimetres.
    :return: list of shapely polygons in input order.
    """
    inflated: List[Polygon] = []
    for obs in obstacles:
        x_min, y_min, x_max, y_max, _kind = obs
        rect = box(x_min, y_min, x_max, y_max)
        inflated.append(rect.buffer(robot_radius_cm, cap_style=2, join_style=2))
    return inflated


def edge_clips_obstacle(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    inflated_polygons: Sequence[Polygon],
) -> bool:
    """Return ``True`` iff the segment ``p1 -> p2`` clips any polygon."""
    if not inflated_polygons:
        return False
    line = LineString([p1, p2])
    for poly in inflated_polygons:
        if line.intersects(poly):
            return True
    return False


def _emit_directed_pair(
    g: nx.DiGraph,
    u: str,
    v: str,
    pu: Tuple[float, float],
    pv: Tuple[float, float],
    edge_type: str,
) -> None:
    """Add the forward + reverse directed edges for an undirected pair."""
    distance_cm = _euclid(pu, pv)
    heading = _heading_deg(pu, pv)
    g.add_edge(
        u, v,
        type=edge_type,
        distance_cm=distance_cm,
        heading_deg=heading,
    )
    g.add_edge(
        v, u,
        type=edge_type,
        distance_cm=distance_cm,
        heading_deg=_reverse_heading(heading),
    )


def build_graph(field_data) -> nx.DiGraph:
    """Build the planner's directed graph from validated field data.

    Pipeline:

    1. :func:`validate` -- bubbles ``FieldDataError`` unchanged.
    2. Add every node with attrs ``x_cm``, ``y_cm``, ``type``.
    3. Inflate obstacles by ``ROBOT_RADIUS_CM``.
    4. Emit bidirectional :data:`EDGE_LINE` edges for every ``LINE_SEGMENTS``
       entry. Drop a pair if its segment clips any inflated obstacle.
    5. Emit bidirectional :data:`EDGE_BLIND` edges for every ``BLIND_EDGES``
       entry, with the same obstacle-clip drop.
    6. Warn (``UserWarning``) for any node that ended up with no incident
       edges.

    :param field_data: the ``tools.path_planner.field_data`` module.
    :return: ``networkx.DiGraph`` with attrs as documented above.
    :raises FieldDataError: bubbled from :func:`validate`.
    """
    validate(field_data)

    g = nx.DiGraph()

    for name, (x, y) in field_data.NODES.items():
        g.add_node(
            name,
            x_cm=float(x),
            y_cm=float(y),
            type=field_data.NODE_TYPES[name],
        )

    inflated = inflate_obstacles(field_data.OBSTACLES, field_data.ROBOT_RADIUS_CM)

    for u, v in field_data.LINE_SEGMENTS:
        pu = field_data.NODES[u]
        pv = field_data.NODES[v]
        if edge_clips_obstacle(pu, pv, inflated):
            continue
        _emit_directed_pair(g, u, v, pu, pv, EDGE_LINE)

    for u, v in field_data.BLIND_EDGES:
        pu = field_data.NODES[u]
        pv = field_data.NODES[v]
        if edge_clips_obstacle(pu, pv, inflated):
            warnings.warn(
                f"BLIND_EDGES ('{u}','{v}') is clipped by an inflated "
                "obstacle and was dropped.",
                UserWarning,
                stacklevel=2,
            )
            continue
        _emit_directed_pair(g, u, v, pu, pv, EDGE_BLIND)

    for node in g.nodes():
        if g.in_degree(node) == 0 and g.out_degree(node) == 0:
            warnings.warn(
                f"Node '{node}' has no incident edges after C-space "
                "inflation; it will be unreachable in the planner.",
                UserWarning,
                stacklevel=2,
            )

    return g
