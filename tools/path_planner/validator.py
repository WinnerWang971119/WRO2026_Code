"""Strict validator for the path planner's field-data input.

This module is the user-friendly first line of defense before any geometry
or graph math runs. It inspects the raw module attributes (`NODES`,
`NODE_TYPES`, `LINE_SEGMENTS`, ...) and raises :class:`FieldDataError`
on the first malformed entry, with a precise key path and a one-line
nudge toward the fix.

Public surface:

* :class:`FieldDataError` -- single-message exception type.
* :func:`validate` -- pure function; returns ``None`` on success,
  raises ``FieldDataError`` on the first problem.

The validator performs no I/O, no logging, and mutates no global state.
"""

from __future__ import annotations

import math
from typing import Any, Iterable

# Required top-level attributes on the field_data module.
_REQUIRED_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "PROJECTIONS",
    "GOAL_APPROACH",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
)

_ALLOWED_NODE_TYPES = ("line", "projection", "goal")
_ALLOWED_BLIND_ACTIONS = ("turn_relative", "forward")


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


def _known_segment_keys(line_segments: Iterable) -> set:
    """Build the set of order-independent segment keys, defensively."""
    keys = set()
    for seg in line_segments:
        if isinstance(seg, tuple) and len(seg) == 2:
            keys.add(_segment_key(seg[0], seg[1]))
    return keys


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
    projections = getattr(field_data, "PROJECTIONS")
    goal_approach = getattr(field_data, "GOAL_APPROACH")
    obstacles = getattr(field_data, "OBSTACLES")
    robot_radius = getattr(field_data, "ROBOT_RADIUS_CM")
    cost_blind_mul = getattr(field_data, "COST_BLIND_MULTIPLIER")
    start_node = getattr(field_data, "START_NODE")
    start_heading = getattr(field_data, "START_HEADING_DEG")

    # 2. NODES coords.
    if not isinstance(nodes, dict):
        raise FieldDataError(
            "NODES must be a dict mapping node-name -> (x, y); "
            f"got: {type(nodes).__name__}"
        )
    for name, coord in nodes.items():
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
                "every NODES key needs a type ('line', 'projection', or 'goal')"
            )
    for name, value in node_types.items():
        if value not in _ALLOWED_NODE_TYPES:
            raise FieldDataError(
                f"NODE_TYPES['{name}'] = '{value}'; expected one of "
                "{'line','projection','goal'}"
            )

    # 4. LINE_SEGMENTS coverage.
    if not isinstance(line_segments, (list, tuple)):
        raise FieldDataError(
            "LINE_SEGMENTS must be a list of (a, b) node-name pairs; "
            f"got: {type(line_segments).__name__}"
        )
    for i, seg in enumerate(line_segments):
        if not isinstance(seg, tuple) or len(seg) != 2:
            raise FieldDataError(
                f"LINE_SEGMENTS[{i}] must be a 2-tuple of node names; "
                f"got: {seg!r}"
            )
        a, b = seg
        if a not in nodes:
            raise FieldDataError(
                f"LINE_SEGMENTS contains unknown node: '{a}' "
                f"(segment {i}: ('{a}', '{b}')) -- "
                "add it to NODES or fix the segment endpoint"
            )
        if b not in nodes:
            raise FieldDataError(
                f"LINE_SEGMENTS contains unknown node: '{b}' "
                f"(segment {i}: ('{a}', '{b}')) -- "
                "add it to NODES or fix the segment endpoint"
            )

    segment_keys = _known_segment_keys(line_segments)

    # 5. PROJECTIONS shape and refs.
    if not isinstance(projections, dict):
        raise FieldDataError(
            "PROJECTIONS must be a dict mapping projection-node -> spec; "
            f"got: {type(projections).__name__}"
        )
    for key, spec in projections.items():
        if node_types.get(key) != "projection":
            raise FieldDataError(
                f"PROJECTIONS['{key}'] must have NODE_TYPES['{key}'] == "
                "'projection' -- fix the type or remove the projection entry"
            )
        if not isinstance(spec, dict):
            raise FieldDataError(
                f"PROJECTIONS['{key}'] must be a dict with keys "
                "'on_segment', 'from', 'distance_cm'; "
                f"got: {type(spec).__name__}"
            )
        for required in ("on_segment", "from", "distance_cm"):
            if required not in spec:
                raise FieldDataError(
                    f"PROJECTIONS['{key}'] missing key '{required}' -- "
                    "spec needs on_segment, from, distance_cm"
                )
        on_seg = spec["on_segment"]
        if (
            not isinstance(on_seg, tuple)
            or len(on_seg) != 2
            or not isinstance(on_seg[0], str)
            or not isinstance(on_seg[1], str)
        ):
            raise FieldDataError(
                f"PROJECTIONS['{key}'].on_segment must be a 2-tuple of node "
                f"names; got: {on_seg!r}"
            )
        if _segment_key(on_seg[0], on_seg[1]) not in segment_keys:
            raise FieldDataError(
                f"PROJECTIONS['{key}'].on_segment ('{on_seg[0]}','{on_seg[1]}') "
                "is not a known LINE_SEGMENTS entry -- "
                "add the segment to LINE_SEGMENTS or fix the endpoints"
            )
        from_node = spec["from"]
        if from_node not in (on_seg[0], on_seg[1]):
            raise FieldDataError(
                f"PROJECTIONS['{key}'].from = '{from_node}' must be one of "
                f"the segment endpoints ('{on_seg[0]}', '{on_seg[1]}')"
            )
        distance = spec["distance_cm"]
        if not _is_number(distance) or distance < 0:
            raise FieldDataError(
                f"PROJECTIONS['{key}'].distance_cm must be a non-negative "
                f"number; got: {distance!r}"
            )
        # Need both endpoints to be valid in NODES for length check.
        ax, ay = nodes[on_seg[0]]
        bx, by = nodes[on_seg[1]]
        seg_length = math.hypot(bx - ax, by - ay)
        if distance >= seg_length:
            raise FieldDataError(
                f"PROJECTIONS['{key}'].distance_cm = {distance} must be "
                f"strictly less than segment length {seg_length:.4f} -- "
                "move the projection inside the segment"
            )

    # 6. GOAL_APPROACH shape and refs.
    if not isinstance(goal_approach, dict):
        raise FieldDataError(
            "GOAL_APPROACH must be a dict mapping goal-node -> spec; "
            f"got: {type(goal_approach).__name__}"
        )
    for goal, spec in goal_approach.items():
        if node_types.get(goal) != "goal":
            raise FieldDataError(
                f"GOAL_APPROACH['{goal}'] must have NODE_TYPES['{goal}'] == "
                "'goal' -- fix the type or remove the goal entry"
            )
        if not isinstance(spec, dict):
            raise FieldDataError(
                f"GOAL_APPROACH['{goal}'] must be a dict with keys "
                "'entry', 'approach_heading_deg', 'blind_moves'; "
                f"got: {type(spec).__name__}"
            )
        for required in ("entry", "approach_heading_deg", "blind_moves"):
            if required not in spec:
                raise FieldDataError(
                    f"GOAL_APPROACH['{goal}'] missing key '{required}' -- "
                    "spec needs entry, approach_heading_deg, blind_moves"
                )
        entry = spec["entry"]
        if not isinstance(entry, str):
            raise FieldDataError(
                f"GOAL_APPROACH['{goal}'].entry must be a node-name string; "
                f"got: {entry!r}"
            )
        if entry not in nodes:
            raise FieldDataError(
                f"GOAL_APPROACH['{goal}'].entry references unknown node: "
                f"'{entry}' -- add it to NODES or pick an existing node"
            )
        heading = spec["approach_heading_deg"]
        if not _is_number(heading) or not (0.0 <= float(heading) < 360.0):
            raise FieldDataError(
                f"GOAL_APPROACH['{goal}'].approach_heading_deg must be a "
                f"number in [0, 360); got: {heading!r}"
            )
        moves = spec["blind_moves"]
        if not isinstance(moves, list):
            raise FieldDataError(
                f"GOAL_APPROACH['{goal}'].blind_moves must be a list of "
                f"(action, value) tuples; got: {type(moves).__name__}"
            )
        for j, move in enumerate(moves):
            if not isinstance(move, tuple) or len(move) != 2:
                raise FieldDataError(
                    f"GOAL_APPROACH['{goal}'].blind_moves[{j}] must be a "
                    f"2-tuple (action, value); got: {move!r}"
                )
            action, value = move
            if action not in _ALLOWED_BLIND_ACTIONS:
                raise FieldDataError(
                    f"GOAL_APPROACH['{goal}'].blind_moves[{j}] action "
                    f"'{action}' invalid; expected one of "
                    "{'turn_relative','forward'}"
                )
            if not _is_number(value):
                raise FieldDataError(
                    f"GOAL_APPROACH['{goal}'].blind_moves[{j}] value must "
                    f"be a number; got: {value!r}"
                )

    # 7. OBSTACLES shape.
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
            ("x_min", x_min),
            ("y_min", y_min),
            ("x_max", x_max),
            ("y_max", y_max),
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

    # 8. Scalars.
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
