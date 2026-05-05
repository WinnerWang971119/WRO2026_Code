"""Graph builder + obstacle inflation for the WRO 2026 path planner.

This module turns the validated ``field_data`` into a ``networkx.DiGraph``
ready to be consumed by :mod:`tools.path_planner.planner` (Dijkstra) and
:mod:`tools.path_planner.compiler` (heading propagation).

Heading convention (non-negotiable, per PR 1 spec):

* Continuous float degrees in ``[0, 360)``.
* ``0 deg`` = +x axis (East).
* CCW positive (matches the R4 IMU yaw sign so PR 2's ``Drive.turnToYaw``
  consumes ``heading_deg`` directly without a sign flip).
* Forward edge heading: ``math.degrees(math.atan2(dy, dx)) % 360.0``.
* Reverse heading: ``(forward + 180.0) % 360.0``.

Edge attributes (every edge in the returned graph carries these):

* ``type``        : one of :data:`EDGE_LINE`, :data:`EDGE_PROJECTION`,
                    :data:`EDGE_BLIND` (uppercase strings).
* ``distance_cm`` : float, Euclidean length of the directed edge.
* ``heading_deg`` : float in ``[0, 360)`` per the convention above.

Node attributes mirror the input data:

* ``x_cm``, ``y_cm`` : float coordinates.
* ``type``           : ``"line" | "projection" | "goal"`` (from
                       ``field_data.NODE_TYPES``).

Obstacle inflation uses a Minkowski-sum approach via
``shapely.geometry.box(...).buffer(R, cap_style=2, join_style=2)`` so the
inflated footprint stays roughly rectangular (``cap_style=2`` is square,
``join_style=2`` is mitre). Any directed edge whose linestring intersects
any inflated polygon is silently dropped. ``intersects()`` is conservative:
touching at a single point or being entirely contained both count.
"""

from __future__ import annotations

import math
import warnings
from typing import Iterable, List, Sequence, Tuple

import networkx as nx
from shapely.geometry import LineString, Polygon, box

from tools.path_planner.validator import validate

# Edge-type tags. Stored as the ``type`` attribute on every edge in the
# returned DiGraph. PR 2's runtime maps these to the EdgeType enum in
# routes.h; the strings here are the canonical source of truth.
EDGE_LINE = "LINE"
EDGE_PROJECTION = "PROJECTION"
EDGE_BLIND = "BLIND"


# -- internal helpers ---------------------------------------------------------


def _heading_deg(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Return the directed-edge heading from ``p1`` to ``p2`` in degrees.

    :param p1: source point ``(x, y)``.
    :param p2: target point ``(x, y)``.
    :return: float heading in ``[0, 360)``, ``0 deg`` = +x, CCW positive.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx)) % 360.0


def _reverse_heading(heading_deg: float) -> float:
    """Return ``(heading + 180) mod 360`` for the reverse traversal."""
    return (heading_deg + 180.0) % 360.0


def _euclid(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Euclidean distance between two ``(x, y)`` tuples."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


# -- public helpers (also imported by tests) ----------------------------------


def inflate_obstacles(
    obstacles: Iterable[Tuple[float, float, float, float, str]],
    robot_radius_cm: float,
) -> List[Polygon]:
    """Return one inflated :class:`shapely.geometry.Polygon` per obstacle.

    Each axis-aligned rectangle ``(x_min, y_min, x_max, y_max, kind)`` is
    converted with :func:`shapely.geometry.box` and then expanded by
    ``robot_radius_cm`` via ``buffer(..., cap_style=2, join_style=2)``
    (square cap, mitre join) so the inflated footprint remains rectangular.
    A round buffer would over-inflate corners and reject grazing paths.

    :param obstacles: iterable of 5-tuples
        ``(x_min, y_min, x_max, y_max, kind)``. ``kind`` is ignored here.
    :param robot_radius_cm: radius to inflate by, in centimetres.
        Must be non-negative; ``0`` returns the un-inflated rectangles.
    :return: list of shapely ``Polygon`` instances, one per input obstacle,
        preserving input order.
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
    """Return ``True`` iff the line segment ``p1 -> p2`` clips any polygon.

    Uses :meth:`shapely.geometry.base.BaseGeometry.intersects`, which is
    conservative: it reports True for boundary touches and full
    containment as well as crossings. This is the behaviour we want
    because grazing the inflated obstacle still means the un-inflated
    robot would brush the wall.

    :param p1: source point ``(x, y)``.
    :param p2: target point ``(x, y)``.
    :param inflated_polygons: polygons returned by :func:`inflate_obstacles`.
    :return: ``True`` if any polygon intersects the linestring;
        ``False`` if the path is clear (or there are no polygons).
    """
    if not inflated_polygons:
        return False
    line = LineString([p1, p2])
    for poly in inflated_polygons:
        if line.intersects(poly):
            return True
    return False


# -- internal: projection -> segment splitting -------------------------------


def _project_distance_along_segment(
    field_data,
    seg_a: str,
    seg_b: str,
    proj_node: str,
) -> float:
    """Distance from ``seg_a`` to the projection node along the segment.

    The projection's coordinates are already in ``field_data.NODES``, so we
    derive the distance directly from the geometry rather than re-using
    the (possibly endpoint-swapped) ``distance_cm`` field on the
    projection spec. This way the ordering along the segment is correct
    regardless of which endpoint the spec named in ``"from"``.

    :param field_data: the validated field-data module/namespace.
    :param seg_a: canonical first endpoint of the segment.
    :param seg_b: canonical second endpoint of the segment.
    :param proj_node: name of the projection node (must lie on the segment).
    :return: scalar distance along ``seg_a -> seg_b`` from ``seg_a``.
    """
    nodes = field_data.NODES
    return _euclid(nodes[seg_a], nodes[proj_node])


def _build_segment_split_index(field_data) -> dict:
    """Group projection nodes by the undirected segment they belong to.

    :return: dict keyed by ``frozenset({a, b})`` whose values are lists of
        projection node names. The lists are unsorted at this stage; the
        caller sorts by distance from a chosen endpoint when emitting edges.
    """
    splits: dict = {}
    for proj_node, spec in field_data.PROJECTIONS.items():
        seg = spec["on_segment"]
        key = frozenset(seg)
        splits.setdefault(key, []).append(proj_node)
    return splits


def _ordered_chain_for_segment(
    field_data,
    seg_a: str,
    seg_b: str,
    splits: dict,
) -> List[str]:
    """Return the node chain ``[seg_a, ...projections-by-distance..., seg_b]``.

    Projections are sorted by Euclidean distance from ``seg_a`` so
    consecutive pairs in the chain are colinear sub-edges of the original
    segment.

    :param field_data: validated field-data module/namespace.
    :param seg_a: first endpoint as listed in ``LINE_SEGMENTS``.
    :param seg_b: second endpoint as listed in ``LINE_SEGMENTS``.
    :param splits: output of :func:`_build_segment_split_index`.
    :return: ordered list of node names from ``seg_a`` to ``seg_b``,
        with all projections inserted in between.
    """
    key = frozenset((seg_a, seg_b))
    proj_nodes = splits.get(key, [])
    if not proj_nodes:
        return [seg_a, seg_b]
    decorated = [
        (_project_distance_along_segment(field_data, seg_a, seg_b, p), p)
        for p in proj_nodes
    ]
    # Stable secondary sort on name keeps output deterministic when two
    # projections share a distance (degenerate but cheap to guard).
    decorated.sort(key=lambda t: (t[0], t[1]))
    return [seg_a] + [name for _, name in decorated] + [seg_b]


# -- public API: build_graph -------------------------------------------------


def build_graph(field_data) -> nx.DiGraph:
    """Build the planner's directed graph from validated field data.

    Pipeline:

    1. :func:`tools.path_planner.validator.validate` - bubbles
       :class:`~tools.path_planner.validator.FieldDataError` unchanged.
    2. Add every node from ``field_data.NODES`` with attrs
       ``x_cm``, ``y_cm``, ``type``.
    3. Inflate obstacles by ``ROBOT_RADIUS_CM`` (Minkowski sum, square
       cap + mitre join).
    4. For every entry in ``LINE_SEGMENTS``, walk the
       ``[a, projections..., b]`` chain and emit two directed edges per
       consecutive pair (forward + reverse). An edge is dropped when its
       linestring clips any inflated polygon. The edge ``type`` is
       :data:`EDGE_PROJECTION` if either endpoint is a projection node,
       otherwise :data:`EDGE_LINE`.
    5. For every entry in ``GOAL_APPROACH``, emit a single directed
       :data:`EDGE_BLIND` edge ``entry -> goal``. If the linestring clips
       an inflated polygon, the edge is omitted and a :class:`UserWarning`
       names the unreachable goal. No reverse edge is emitted - blind
       moves are one-way in v1.
    6. Warn (via :func:`warnings.warn`) for any node that ended up with
       no incident edges after inflation - this surfaces in the planner
       as ``NEXT_HOP[*][node] = 0xFF``.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing the same top-level attributes).
    :return: a :class:`networkx.DiGraph`. Nodes carry
        ``{"x_cm","y_cm","type"}``; edges carry
        ``{"type","distance_cm","heading_deg"}``. Already C-space inflated.
    :raises tools.path_planner.validator.FieldDataError: bubbled from
        :func:`validate` on malformed input.
    """
    # 1. Validate first; let FieldDataError bubble up unchanged.
    validate(field_data)

    g = nx.DiGraph()

    # 2. Add nodes with their coordinates and type.
    for name, (x, y) in field_data.NODES.items():
        g.add_node(
            name,
            x_cm=float(x),
            y_cm=float(y),
            type=field_data.NODE_TYPES[name],
        )

    # 3. C-space inflation.
    inflated = inflate_obstacles(field_data.OBSTACLES, field_data.ROBOT_RADIUS_CM)

    # 4. Pre-compute projection -> segment grouping.
    splits = _build_segment_split_index(field_data)

    # 5. Emit line / projection edges per segment.
    for seg_a, seg_b in field_data.LINE_SEGMENTS:
        chain = _ordered_chain_for_segment(field_data, seg_a, seg_b, splits)
        for u, v in zip(chain[:-1], chain[1:]):
            pu = field_data.NODES[u]
            pv = field_data.NODES[v]
            if edge_clips_obstacle(pu, pv, inflated):
                continue
            edge_type = (
                EDGE_PROJECTION
                if (
                    field_data.NODE_TYPES[u] == "projection"
                    or field_data.NODE_TYPES[v] == "projection"
                )
                else EDGE_LINE
            )
            distance_cm = _euclid(pu, pv)
            heading_uv = _heading_deg(pu, pv)
            g.add_edge(
                u,
                v,
                type=edge_type,
                distance_cm=distance_cm,
                heading_deg=heading_uv,
            )
            g.add_edge(
                v,
                u,
                type=edge_type,
                distance_cm=distance_cm,
                heading_deg=_reverse_heading(heading_uv),
            )

    # 6. Emit blind edges per goal.
    for goal_name, spec in field_data.GOAL_APPROACH.items():
        entry = spec["entry"]
        p_entry = field_data.NODES[entry]
        p_goal = field_data.NODES[goal_name]
        if edge_clips_obstacle(p_entry, p_goal, inflated):
            warnings.warn(
                f"Goal '{goal_name}' is unreachable from entry "
                f"'{entry}': blind path clipped by an inflated obstacle. "
                "Adjust ROBOT_RADIUS_CM, the obstacle layout, or the entry node.",
                UserWarning,
                stacklevel=2,
            )
            continue
        distance_cm = _euclid(p_entry, p_goal)
        heading_eg = _heading_deg(p_entry, p_goal)
        g.add_edge(
            entry,
            goal_name,
            type=EDGE_BLIND,
            distance_cm=distance_cm,
            heading_deg=heading_eg,
        )

    # 7. Surface fully isolated nodes as warnings (TC14-friendly).
    for node in g.nodes():
        if g.in_degree(node) == 0 and g.out_degree(node) == 0:
            warnings.warn(
                f"Node '{node}' has no incident edges after C-space "
                "inflation; it will be unreachable in the planner.",
                UserWarning,
                stacklevel=2,
            )

    return g
