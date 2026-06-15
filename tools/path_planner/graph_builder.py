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
* ``direction_override`` : integer (the value of a
  :class:`pipeline_config.DirectionMode` member, 1 = FORCE_FORWARD,
  2 = FORCE_REVERSE). Forces forward/reverse traversal of this directed
  edge regardless of heading delta. Absent = inherit
  PIPELINE_CONFIG.direction. Asymmetric: applied to the forward edge
  only -- the reverse edge needs its own ``EDGES`` entry to opt in.
* ``stop_at_line_override`` : a :class:`pipeline_config.StopAtLineMode`
  member (ON = force the stop-on-crossing seek, OFF = force dead-reckon).
  Absent = inherit (DEFAULT). Asymmetric like ``direction_override``:
  applied to the forward edge only -- the reverse edge needs its own
  ``EDGES`` entry to opt in.

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
from typing import Iterable, List, Sequence, Tuple

import networkx as nx
from shapely.geometry import LineString, Polygon, box

from tools.path_planner.pipeline_config import StopAtLineMode
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


__all__ = [
    "EDGE_LINE",
    "EDGE_BLIND",
    "EDGE_PROJECTION",
    "inflate_obstacles",
    "edge_clips_obstacle",
    "build_graph",
    "directed_edge_heading",
]


def _heading_deg(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.degrees(math.atan2(dy, dx)) % 360.0


def _reverse_heading(heading_deg: float) -> float:
    return (heading_deg + 180.0) % 360.0


def _euclid(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def directed_edge_heading(graph: "nx.DiGraph", u: str, v: str) -> float:
    """Return the heading of the directed edge ``u -> v`` in degrees.

    Reads the ``heading_deg`` attribute stored on the edge by
    :func:`build_graph`, which respects any ``heading_deg`` override
    supplied via ``field_data.EDGES``.  The heading follows the module
    convention: ``[0, 360)``; ``0 deg`` = East (+x); CCW positive.

    This is the single reusable source for all downstream heading
    arithmetic (planner ``build_pivot_turns``, validator
    ``_validate_pivot_turns``).  Callers must **not** re-implement the
    ``atan2`` / ``% 360`` / ``+ 180`` logic -- subtract this helper's
    return values and pass the difference through a normalizer.

    :param graph: directed graph returned by :func:`build_graph`.
    :param u:     source node name.
    :param v:     destination node name.
    :return:      heading in ``[0, 360)`` degrees.
    :raises KeyError: if the edge ``u -> v`` does not exist in *graph*.
    """
    return float(graph[u][v]["heading_deg"])


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


# Minimum overlap length (cm) below which a segment is treated as merely
# grazing an inflated obstacle rather than clipping it. A segment that only
# touches the inflated polygon at a single point (tangent / shared endpoint
# at the legal clearance) yields a zero-length intersection and must NOT be
# dropped; anything that runs through the interior or along the boundary for
# a measurable stretch does clip.
_CLIP_OVERLAP_EPS_CM = 1e-9


def edge_clips_obstacle(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    inflated_polygons: Sequence[Polygon],
) -> bool:
    """Return ``True`` iff the segment ``p1 -> p2`` clips any polygon.

    A *clip* means the segment passes through the inflated obstacle's
    interior, or runs collinearly along its boundary for a measurable
    stretch. A segment that merely grazes the polygon at a single point --
    a node placed exactly one robot-radius from a wall, i.e. the intended
    minimal clearance -- is NOT a clip: ``LineString.intersects`` would
    flag that boundary touch as a hit, so we instead measure the length of
    the intersection and require it to exceed :data:`_CLIP_OVERLAP_EPS_CM`.
    The length test (rather than ``LineString.crosses``) is deliberate:
    ``crosses`` returns ``False`` for a collinear overlap that lies on the
    boundary, which would wrongly admit an edge scraping along an obstacle
    wall.
    """
    if not inflated_polygons:
        return False
    line = LineString([p1, p2])
    for poly in inflated_polygons:
        if line.intersection(poly).length > _CLIP_OVERLAP_EPS_CM:
            return True
    return False


# Symmetric: describes HOW to traverse the segment, not which end is the
# entry. Safe to copy onto both directions of a bidirectional pair.
_BIDIRECTIONAL_OPTIONAL_KEYS = ("primitive_mode",)

# Direction-dependent: ``direction`` forces forward/reverse traversal of
# the directed edge, and ``stop_at_line`` forces (or suppresses) the
# stop-on-crossing seek for the directed edge. Both depend on which end the
# chassis is approaching from, so they must NOT be copied to the reverse
# edge (a one-way override would otherwise force the wrong traversal sense /
# seek behavior on backward traversal). This is the Codex P1 invariant: the
# asymmetric override stays on the forward (u -> v) edge only; to override
# the reverse direction too, the author must add a separate ``EDGES`` entry
# with ``u`` and ``v`` swapped.
_DIRECTIONAL_OPTIONAL_KEYS = ("direction", "stop_at_line")

# String-to-integer mapping for the ``direction`` per-edge override.
# Header_writer (T3) reads the integer via ``edge_attrs.get("direction_override")``
# and resolves it through :class:`DirectionMode` (DEFAULT=0, FORCE_FORWARD=1,
# FORCE_REVERSE=2). Keep this mapping in lockstep with the C++ enum order;
# the integer encoding is part of the wire contract.
_DIRECTION_STR_TO_INT = {
    "front": 1,  # DirectionMode::FORCE_FORWARD
    "rear":  2,  # DirectionMode::FORCE_REVERSE
}

# String-to-enum mapping for the ``stop_at_line`` per-edge override. Mirrors
# the case-insensitive string tolerance of ``direction``: the field-data
# vocabulary ("on" / "off") resolves to the :class:`StopAtLineMode` member
# stored under ``stop_at_line_override``. Absence yields no attribute, so
# header_writer falls back to ``StopAtLineMode::DEFAULT`` (0).
_STOP_AT_LINE_STR_TO_MODE = {
    "on":  StopAtLineMode.ON,
    "off": StopAtLineMode.OFF,
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

    These attributes (``direction``, ``stop_at_line``) describe behavior
    that depends on which end is the entry, so the caller must apply the
    returned dict ONLY to the forward (u -> v) directed edge -- never to
    the reverse. The string mapping of each is resolved here so
    downstream consumers don't have to re-derive it.

    ``direction`` is translated from the field-data string vocabulary
    (``"front"`` / ``"rear"``) into the wire integer (1 / 2) and stored
    under the key ``direction_override`` -- the name the header writer
    (T3) reads to emit the ``DirectionMode::*`` literal. Absence yields
    no attribute, so header_writer falls back to DEFAULT (0).

    ``stop_at_line`` is resolved to a :class:`StopAtLineMode` member --
    either passed through when already a member, or coerced from the
    case-insensitive string vocabulary (``"on"`` / ``"off"``) -- and
    stored under the key ``stop_at_line_override``, the name the header
    writer reads to emit the ``StopAtLineMode::*`` literal. Absence yields
    no attribute, so header_writer falls back to DEFAULT (0).

    :param options: dict-like source-edge attributes (may be empty).
    :return: dict of attributes to splat onto the forward edge only.
    """
    forwarded: dict = {}
    if "direction" in options:
        # Validator (T4) has already constrained this to "front" or "rear".
        forwarded["direction_override"] = _DIRECTION_STR_TO_INT[options["direction"]]
    if "stop_at_line" in options:
        # Validator (T4) has already constrained this to a StopAtLineMode
        # member or one of the strings "on" / "off". A member passes through;
        # a string is coerced case-insensitively (mirroring ``direction``).
        value = options["stop_at_line"]
        if isinstance(value, StopAtLineMode):
            forwarded["stop_at_line_override"] = value
        else:
            forwarded["stop_at_line_override"] = _STOP_AT_LINE_STR_TO_MODE[value.lower()]
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
    * **Directional** (``direction``, ``stop_at_line``): describes behavior
      that depends on which end is the entry. Applied ONLY to the forward
      (u -> v) edge. Mirroring it to the reverse edge would force the wrong
      traversal sense / seek behavior on backward traversal (Codex P1). To
      override the reverse direction too, add a separate ``EDGES`` entry
      with ``u`` and ``v`` swapped.

    .. warning::
       **A directional override is SINGLE-DIRECTION and silently skipped
       on reverse traversal.** ``direction_override`` lands on the u -> v
       edge only; the reverse (v -> u) edge here gets just the symmetric
       ``primitive_mode``. Because every pair is bidirectional and
       Dijkstra picks whichever direction is shorter, ``NEXT_HOP`` may
       route v -> u -- in which case the override does NOT apply and the
       hop falls back to ``PIPELINE_CONFIG.direction`` with no warning.
       If you need the override in both senses, author a second ``EDGES``
       entry with ``u`` and ``v`` swapped (see finding #40).

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
       optional keys ``primitive_mode``, ``direction``, ``stop_at_line``
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
            warnings.warn(
                f"LINE_SEGMENTS ('{u}','{v}') is clipped by an inflated "
                "obstacle and was dropped.",
                UserWarning,
                stacklevel=2,
            )
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
    # introduces example PROJECTION / direction-override edges. Iteration
    # order is preserved so deterministic graph output stays diff-friendly.
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
            # AUTHORING WARNING (finding #40): a directional override on this
            # EDGES entry -- ``direction`` ("front"/"rear") or ``stop_at_line``
            # ("on"/"off") -- applies ONLY to the u -> v hop. The reverse
            # (v -> u) edge below inherits just the symmetric ``primitive_mode``.
            # Edges are bidirectional and Dijkstra picks the shorter sense, so if
            # NEXT_HOP routes v -> u the override is SILENTLY ignored and the hop
            # falls back to the pipeline default. To force the override on the
            # reverse hop too, add a second EDGES entry with u/v swapped.
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
