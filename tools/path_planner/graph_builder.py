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

* ``type``        : :data:`EDGE_LINE`, :data:`EDGE_BLIND`, or
  :data:`EDGE_PROJECTION`.
* ``distance_cm`` : float, Euclidean length of the directed edge.
* ``heading_deg`` : float in ``[0, 360)`` per the convention above.

Optional edge attributes (only present when supplied on the source edge
dict; forwarded verbatim for downstream consumers like the validator,
compiler, and header writer):

* ``primitive_mode`` : opaque string tag selecting the per-edge primitive
  pipeline mode. Not validated here.
* ``pre_turn_mode``  : opaque string tag selecting the pre-turn alignment
  pipeline mode. Not validated here.
* ``pivot``          : integer (the value of a :class:`Pivot` member).
  Marks the turn entering this edge as a single-wheel pivot. Absent =
  inherit / no pivot.
* ``direction_override`` : integer (the value of a
  :class:`pipeline_config.DirectionMode` member, 1 = FORCE_FORWARD,
  2 = FORCE_REVERSE). Forces forward/reverse traversal of this directed
  edge regardless of heading delta. Absent = inherit
  PIPELINE_CONFIG.direction. Asymmetric: applied to the forward edge
  only -- the reverse edge needs its own ``EDGES`` entry to opt in.

Edge-type semantics:

* :data:`EDGE_LINE`       -- line-follow segment terminated by an
  intersection marker. Cost multiplier 1.0.
* :data:`EDGE_PROJECTION` -- line-follow segment terminated by encoder
  distance (no physical intersection beneath the strip). Used for
  waypoints that sit on the line graph but have no intersection cross.
  Cost multiplier 1.0 (same as LINE -- the chassis is still tracking
  the line, just with a distance gate).
* :data:`EDGE_BLIND`      -- dead-reckoning move with no line under the
  sensor. Cost multiplied by ``cost_blind_multiplier``.

Node attributes:

* ``x_cm``, ``y_cm`` : float coordinates.
* ``type``           : ``"line" | "blind"`` (from ``field_data.NODE_TYPES``).

PR3 schema vs PR2: off-line goals are first-class ``blind`` nodes joined
to the line graph by explicit :data:`field_data.BLIND_EDGES`
(bidirectional by default, so the chassis can return the way it came).
:data:`EDGE_PROJECTION` is re-introduced for line-follow waypoints
without an intersection marker.
"""

from __future__ import annotations

import math
import warnings
from enum import IntEnum
from typing import Iterable, List, Sequence, Tuple

import networkx as nx
from shapely.geometry import LineString, Polygon, box

from tools.path_planner.validator import validate

EDGE_LINE = "LINE"
EDGE_BLIND = "BLIND"

# Line-follow segment terminated by encoder distance (not by an
# intersection marker). Used for waypoints that sit on the line graph
# but have no physical cross beneath the strip. Cost-equivalent to
# :data:`EDGE_LINE` (the chassis is still tracking the line, just with
# a distance gate instead of an intersection gate). Consumers:
# ``validator`` (T6), ``compiler`` (T8), ``header_writer`` (T7),
# ``simulator`` (T9).
EDGE_PROJECTION = "PROJECTION"


class Pivot(IntEnum):
    """Single-wheel pivot selector for the turn entering an edge.

    This is the single source of truth for the pivot vocabulary used
    across :mod:`tools.path_planner.field_data`,
    :mod:`tools.path_planner.validator`,
    :mod:`tools.path_planner.planner`, and
    :mod:`tools.path_planner.header_writer`. Those modules import
    :class:`Pivot` from here so the integer encoding cannot drift.

    The values are written to the compiled header verbatim, so the
    integer assignments are part of the wire contract and must not be
    renumbered without bumping the header schema.
    """

    NONE = 0   # default -- counter-rotation turn (PIPELINE_CONFIG.turn applies)
    LEFT = 1   # single-wheel pivot around the LEFT wheel
    RIGHT = 2  # single-wheel pivot around the RIGHT wheel


__all__ = [
    "EDGE_LINE",
    "EDGE_BLIND",
    "EDGE_PROJECTION",
    "Pivot",
    "inflate_obstacles",
    "edge_clips_obstacle",
    "build_graph",
]


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


# Symmetric: describes HOW to traverse the segment, not which end is the
# entry. Safe to copy onto both directions of a bidirectional pair.
_BIDIRECTIONAL_OPTIONAL_KEYS = ("primitive_mode",)

# Direction-dependent: ``pre_turn_mode`` selects the alignment for the
# turn that PRECEDES this edge, ``pivot`` selects which wheel the
# chassis pivots around when ENTERING this edge, and ``direction`` forces
# forward/reverse traversal of the directed edge. All three depend on
# which end the chassis is approaching from, so they must NOT be copied
# to the reverse edge (a one-way override would otherwise silently steer
# the robot with the wrong wheel pattern -- or, for ``direction``, force
# the wrong traversal sense -- on backward traversal). This is the
# Codex P1 invariant: asymmetric overrides stay on the forward (u -> v)
# edge only; to override the reverse direction too, the author must add
# a separate ``EDGES`` entry with ``u`` and ``v`` swapped.
_DIRECTIONAL_OPTIONAL_KEYS = ("pre_turn_mode", "direction")

# String-to-integer mapping for the ``direction`` per-edge override.
# Header_writer (T3) reads the integer via ``edge_attrs.get("direction_override")``
# and resolves it through :class:`DirectionMode` (DEFAULT=0, FORCE_FORWARD=1,
# FORCE_REVERSE=2). Keep this mapping in lockstep with the C++ enum order;
# the integer encoding is part of the wire contract.
_DIRECTION_STR_TO_INT = {
    "front": 1,  # DirectionMode::FORCE_FORWARD
    "rear":  2,  # DirectionMode::FORCE_REVERSE
}


def _extract_bidirectional_attrs(options: dict) -> dict:
    """Pull symmetric metadata out of a source-edge options dict.

    Symmetric attributes (currently just ``primitive_mode``) describe the
    segment itself rather than the entry direction, so they are copied
    verbatim onto both the forward and reverse directed edges. Absent
    keys stay absent so downstream consumers can distinguish "no
    override" from an explicit value.

    :param options: dict-like source-edge attributes (may be empty).
    :return: dict of attributes to splat onto both directions.
    """
    forwarded: dict = {}
    for key in _BIDIRECTIONAL_OPTIONAL_KEYS:
        if key in options:
            forwarded[key] = options[key]
    return forwarded


def _extract_directional_attrs(options: dict) -> dict:
    """Pull direction-dependent metadata out of a source-edge options dict.

    These attributes (``pre_turn_mode``, ``pivot``, ``direction``)
    describe behavior that depends on which end is the entry, so the
    caller must apply the returned dict ONLY to the forward (u -> v)
    directed edge -- never to the reverse. The integer encoding of
    ``pivot`` and the string-to-integer mapping of ``direction`` are
    stored so downstream consumers don't have to import the IntEnums.

    ``direction`` is translated from the field-data string vocabulary
    (``"front"`` / ``"rear"``) into the wire integer (1 / 2) and stored
    under the key ``direction_override`` -- the name the header writer
    (T3) reads to emit the ``DirectionMode::*`` literal. Absence yields
    no attribute, so header_writer falls back to DEFAULT (0).

    :param options: dict-like source-edge attributes (may be empty).
    :return: dict of attributes to splat onto the forward edge only.
    """
    forwarded: dict = {}
    for key in _DIRECTIONAL_OPTIONAL_KEYS:
        if key == "direction":
            continue  # handled below with string->integer translation
        if key in options:
            forwarded[key] = options[key]
    if "pivot" in options:
        forwarded["pivot"] = int(options["pivot"])
    if "direction" in options:
        # Validator (T4) has already constrained this to "front" or "rear".
        forwarded["direction_override"] = _DIRECTION_STR_TO_INT[options["direction"]]
    return forwarded


def _emit_directed_pair(
    g: nx.DiGraph,
    u: str,
    v: str,
    pu: Tuple[float, float],
    pv: Tuple[float, float],
    edge_type: str,
    options: dict = None,
) -> None:
    """Add the forward + reverse directed edges for an undirected pair.

    Optional ``options`` carries two flavors of metadata:

    * **Bidirectional** (``primitive_mode``): describes the segment
      itself; copied onto both directions.
    * **Directional** (``pre_turn_mode``, ``pivot``): describes behavior
      that depends on which end is the entry. Applied ONLY to the
      forward (u -> v) edge. Mirroring these to the reverse edge would
      steer the chassis with the wrong wheel pattern on backward
      traversal (Codex P1). To override the reverse direction too, add a
      separate ``EDGES`` entry with ``u`` and ``v`` swapped.

    Geometric attributes (``distance_cm``, ``heading_deg``) default to
    the Euclidean / atan2 of ``pu`` and ``pv``. When ``options`` supplies
    explicit overrides (the validator enforces sanity on EDGES entries),
    those overrides are honored: distance is symmetric on both directions,
    and the reverse edge's heading is :func:`_reverse_heading` of the
    overridden forward heading -- not of the computed geometry -- so the
    pair stays self-consistent.
    """
    distance_cm = _euclid(pu, pv)
    heading = _heading_deg(pu, pv)
    # EDGES override: when field_data.EDGES provides explicit distance_cm /
    # heading_deg, honor them instead of the computed geometry. Validator
    # enforces sanity. Distance is symmetric on both directions; heading on
    # the reverse edge is _reverse_heading(forward_heading).
    if options:
        if "distance_cm" in options:
            distance_cm = float(options["distance_cm"])
        if "heading_deg" in options:
            heading = float(options["heading_deg"])
    bidirectional = _extract_bidirectional_attrs(options) if options else {}
    directional   = _extract_directional_attrs(options)   if options else {}
    g.add_edge(
        u, v,
        type=edge_type,
        distance_cm=distance_cm,
        heading_deg=heading,
        **bidirectional,
        **directional,
    )
    g.add_edge(
        v, u,
        type=edge_type,
        distance_cm=distance_cm,
        heading_deg=_reverse_heading(heading),
        **bidirectional,
    )


_LEGAL_EDGE_TYPES = (EDGE_LINE, EDGE_BLIND, EDGE_PROJECTION)


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
    6. **Optional**: emit bidirectional edges for every dict in
       ``field_data.EDGES`` (if the attribute exists). Each dict supplies
       ``type`` (one of :data:`EDGE_LINE`, :data:`EDGE_BLIND`,
       :data:`EDGE_PROJECTION`), ``u``, and ``v``, plus any of the
       optional keys ``primitive_mode``, ``pre_turn_mode``, ``pivot``
       which are forwarded verbatim to the resulting directed edges.
       Same obstacle-clip drop applies; PROJECTION drops warn the same
       way BLIND_EDGES drops do because the routing intent is explicit.
    7. Warn (``UserWarning``) for any node that ended up with no incident
       edges.

    :param field_data: the ``tools.path_planner.field_data`` module.
    :return: ``networkx.DiGraph`` with attrs as documented above.
    :raises FieldDataError: bubbled from :func:`validate`.
    :raises ValueError: if an entry in ``field_data.EDGES`` is malformed
        (missing ``type``/``u``/``v``, unknown node name, illegal
        ``type``, or unknown self-loop). The validator owns the strict
        schema check (T6); this guard prevents silent corruption when
        ``build_graph`` is called on unvalidated fixtures.
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

    # Optional rich-edge list. Absent on legacy fixtures; present once T4
    # introduces example PROJECTION / pivot edges. Iteration order is
    # preserved so deterministic graph output stays diff-friendly.
    rich_edges = getattr(field_data, "EDGES", None)
    if rich_edges is not None:
        if not isinstance(rich_edges, (list, tuple)):
            raise ValueError(
                "field_data.EDGES must be a list/tuple of dicts; "
                f"got: {type(rich_edges).__name__}"
            )
        for i, edge in enumerate(rich_edges):
            if not isinstance(edge, dict):
                raise ValueError(
                    f"EDGES[{i}] must be a dict; got: {edge!r}"
                )
            try:
                edge_type = edge["type"]
                u = edge["u"]
                v = edge["v"]
            except KeyError as missing:
                raise ValueError(
                    f"EDGES[{i}] missing required key {missing!s}; "
                    "each entry needs 'type', 'u', and 'v'"
                ) from None
            if edge_type not in _LEGAL_EDGE_TYPES:
                raise ValueError(
                    f"EDGES[{i}].type = {edge_type!r}; expected one of "
                    f"{_LEGAL_EDGE_TYPES}"
                )
            if u == v:
                raise ValueError(
                    f"EDGES[{i}] is a self-loop ('{u}','{v}'); remove or fix"
                )
            if u not in field_data.NODES:
                raise ValueError(
                    f"EDGES[{i}] references unknown node: '{u}' -- "
                    "add it to NODES or fix the entry"
                )
            if v not in field_data.NODES:
                raise ValueError(
                    f"EDGES[{i}] references unknown node: '{v}' -- "
                    "add it to NODES or fix the entry"
                )

            pu = field_data.NODES[u]
            pv = field_data.NODES[v]
            if edge_clips_obstacle(pu, pv, inflated):
                warnings.warn(
                    f"EDGES[{i}] ('{u}','{v}', type={edge_type}) is clipped "
                    "by an inflated obstacle and was dropped.",
                    UserWarning,
                    stacklevel=2,
                )
                continue
            _emit_directed_pair(g, u, v, pu, pv, edge_type, options=edge)

    for node in g.nodes():
        if g.in_degree(node) == 0 and g.out_degree(node) == 0:
            warnings.warn(
                f"Node '{node}' has no incident edges after C-space "
                "inflation; it will be unreachable in the planner.",
                UserWarning,
                stacklevel=2,
            )

    return g
