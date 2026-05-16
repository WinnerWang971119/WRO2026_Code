"""Strict validator for the path planner's field-data input (PR3 schema).

Inspects the raw module attributes (``NODES``, ``NODE_TYPES``,
``LINE_SEGMENTS``, ``BLIND_EDGES``, ``OBSTACLES``, ``FIELD_MM``,
``MAP_PNG``, ...) and raises :class:`FieldDataError` on the first
malformed entry with a precise key path and a one-line nudge toward the
fix.

PR3 schema (vs PR1/PR2): drops ``PROJECTIONS`` and ``GOAL_APPROACH``,
adds required ``BLIND_EDGES`` and ``FIELD_MM`` / ``MAP_PNG``.

The validator performs no I/O, no logging, and mutates no global state.
"""

from __future__ import annotations

from typing import Any, Iterable

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


class FieldDataError(Exception):
    """Raised by :func:`validate` when the field-data module is malformed.

    Carries a single human-readable message that includes the offending
    key path and a hint about the fix.
    """


def _is_number(value: Any) -> bool:
    """Return True for ``int`` or ``float`` (but not ``bool``)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _segment_key(a: str, b: str) -> frozenset:
    """Order-independent identity for an undirected segment."""
    return frozenset((a, b))


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
