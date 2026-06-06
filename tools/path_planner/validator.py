"""Strict validator for the path planner's field-data input (PR3 schema).

Inspects the raw module attributes (``NODES``, ``NODE_TYPES``,
``LINE_SEGMENTS``, ``BLIND_EDGES``, ``OBSTACLES``, ``FIELD_MM``,
``MAP_PNG``, optional ``EDGES``, ...) and raises :class:`FieldDataError`
on the first malformed entry with a precise key path and a one-line
nudge toward the fix.

PR3 schema (vs PR1/PR2): drops ``PROJECTIONS`` and ``GOAL_APPROACH``,
adds required ``BLIND_EDGES`` and ``FIELD_MM`` / ``MAP_PNG``.

PR4 (T3/T6) adds an optional ``EDGES`` table for rich per-edge metadata
(EDGE_PROJECTION, per-edge mode overrides, single-wheel pivot tags).
Absence of ``EDGES`` is benign for backward compatibility.

The validator performs no I/O, no logging, and mutates no global state.

Imports of :mod:`tools.path_planner.graph_builder` and
:mod:`tools.path_planner.pipeline_config` are deferred inside the EDGES
validator to avoid a circular import (``graph_builder`` imports
:func:`validate` from this module).
"""

from __future__ import annotations

import math
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
    """Return True for a finite ``int`` or ``float`` (but not ``bool``).

    NaN and infinities are rejected: they slip through every downstream
    numeric gate (``x_min >= x_max`` is False when an operand is NaN,
    ``<= 0`` is False for NaN, etc.) and corrupt the planner graph with no
    diagnostic. Requiring :func:`math.isfinite` here closes all gates at the
    root. ``math.isfinite`` accepts ordinary ``int`` values unchanged.
    """
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


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


def _validate_edges_field(field_data) -> None:
    """Validate the optional ``field_data.EDGES`` rich-edge table.

    Runs only when ``field_data`` exposes an ``EDGES`` attribute. Each
    entry must be a dict carrying required keys (``type``, ``u``, ``v``,
    ``distance_cm``, ``heading_deg``) and may carry optional keys
    (``primitive_mode``, ``pre_turn_mode``, ``pivot``). Mode overrides
    are cross-checked against the edge ``type`` -- e.g. a
    :class:`LineMode` is illegal on an :data:`EDGE_BLIND` edge -- and
    the :class:`DEFAULT <pipeline_config.LineMode>` sentinel is rejected
    on any override (DEFAULT == "absent", so being explicit about it is
    a confusion).

    An anti-ambiguity cross-check rejects an entry whose unordered pair
    ``{u, v}`` already exists in ``LINE_SEGMENTS`` or ``BLIND_EDGES``
    with a *conflicting* type (:data:`EDGE_LINE` matches LINE_SEGMENTS,
    :data:`EDGE_BLIND` matches BLIND_EDGES, :data:`EDGE_PROJECTION`
    matches neither): such an entry would silently overwrite the earlier
    edge's type and geometry in :mod:`graph_builder`. A *matching*-type
    entry on an existing pair is a legitimate override layer and is
    accepted.

    The cross-edge pivot-compensation check (does the upcoming turn
    delta exceed the minimum useful angle?) is deliberately deferred to
    :mod:`tools.path_planner.planner` (T5); this validator only checks
    the per-entry shape.

    Imports of :class:`Pivot`, :class:`LineMode`, :class:`BlindMode`,
    :class:`ProjectionMode`, and :class:`TurnMode` are performed inside
    the function to avoid a circular import with
    :mod:`tools.path_planner.graph_builder`.

    :param field_data: the ``tools.path_planner.field_data`` module (or
        any object exposing the same top-level attributes).
    :return: ``None`` on success.
    :raises FieldDataError: with a precise ``EDGES[i]`` key path and
        one-line fix hint on the first malformed entry.
    """
    edges = getattr(field_data, "EDGES", None)
    if edges is None:
        return  # backward-compatible: missing EDGES is fine.

    if not isinstance(edges, (list, tuple)):
        raise FieldDataError(
            "EDGES must be a list/tuple of dicts; "
            f"got: {type(edges).__name__} -- "
            "define EDGES = [ {...}, {...}, ... ] in field_data.py"
        )

    # Deferred imports to avoid circular import via graph_builder, which
    # itself imports `validate` from this module.
    from tools.path_planner.graph_builder import (
        EDGE_BLIND,
        EDGE_LINE,
        EDGE_PROJECTION,
        Pivot,
    )
    from tools.path_planner.pipeline_config import (
        BlindMode,
        LineMode,
        ProjectionMode,
        TurnMode,
    )

    legal_edge_types = (EDGE_LINE, EDGE_BLIND, EDGE_PROJECTION)
    mode_class_by_edge_type = {
        EDGE_LINE: (LineMode, "LineMode"),
        EDGE_BLIND: (BlindMode, "BlindMode"),
        EDGE_PROJECTION: (ProjectionMode, "ProjectionMode"),
    }
    nodes = getattr(field_data, "NODES", {})

    # Anti-ambiguity cross-check (findings #38/#47): an EDGES entry whose
    # unordered pair {u, v} already exists in LINE_SEGMENTS or BLIND_EDGES
    # with a *conflicting* type silently overwrites the earlier edge's type
    # and geometry in graph_builder (LINE then BLIND then EDGES; networkx
    # add_edge replaces attributes). EDGE_LINE matches LINE_SEGMENTS and
    # EDGE_BLIND matches BLIND_EDGES; EDGE_PROJECTION matches neither, so a
    # PROJECTION entry on a LINE/BLIND pair is a conflict. A *matching*-type
    # entry (e.g. EDGE_LINE on a LINE_SEGMENTS pair) is a legitimate override
    # layer (distance/heading/pivot/direction) and is accepted. This mirrors,
    # via EDGES, the LINE-vs-BLIND guard in :func:`validate`.
    line_segment_keys = {
        _segment_key(a, b)
        for a, b in getattr(field_data, "LINE_SEGMENTS", ())
    }
    blind_edge_keys = {
        _segment_key(a, b)
        for a, b in getattr(field_data, "BLIND_EDGES", ())
    }

    for i, entry in enumerate(edges):
        if not isinstance(entry, dict):
            raise FieldDataError(
                f"EDGES[{i}] must be a dict; got: {entry!r} -- "
                "each entry needs at least 'type', 'u', 'v', "
                "'distance_cm', 'heading_deg'"
            )

        # Required keys ----------------------------------------------------
        for required_key in ("type", "u", "v", "distance_cm", "heading_deg"):
            if required_key not in entry:
                raise FieldDataError(
                    f"EDGES[{i}] missing required key '{required_key}' -- "
                    "each entry needs 'type', 'u', 'v', 'distance_cm', "
                    "and 'heading_deg'"
                )

        edge_type = entry["type"]
        u = entry["u"]
        v = entry["v"]
        distance_cm = entry["distance_cm"]
        heading_deg = entry["heading_deg"]

        if edge_type not in legal_edge_types:
            raise FieldDataError(
                f"EDGES[{i}].type = {edge_type!r}; expected one of "
                f"{legal_edge_types} -- use EDGE_LINE, EDGE_BLIND, or "
                "EDGE_PROJECTION from graph_builder"
            )

        if u == v:
            raise FieldDataError(
                f"EDGES[{i}] is a self-loop ('{u}','{v}'); "
                "remove the entry or fix one of the endpoints"
            )

        if u not in nodes:
            raise FieldDataError(
                f"EDGES[{i}] references unknown node u='{u}' -- "
                "add it to NODES or fix the entry"
            )

        if v not in nodes:
            raise FieldDataError(
                f"EDGES[{i}] references unknown node v='{v}' -- "
                "add it to NODES or fix the entry"
            )

        # Conflicting-type cross-check (findings #38/#47). A matching-type
        # override is allowed; only a type that reclassifies an existing
        # LINE/BLIND pair (or a PROJECTION entry over either) is rejected.
        pair_key = _segment_key(u, v)
        if pair_key in line_segment_keys and edge_type != EDGE_LINE:
            raise FieldDataError(
                f"EDGES[{i}] ('{u}','{v}') is type {edge_type} but the pair "
                "is already a LINE_SEGMENTS entry -- this silently overwrites "
                "the line edge's type and geometry in graph_builder. Use "
                "EDGE_LINE to override the existing line edge, or remove the "
                "pair from LINE_SEGMENTS if it is not line-reachable"
            )
        if pair_key in blind_edge_keys and edge_type != EDGE_BLIND:
            raise FieldDataError(
                f"EDGES[{i}] ('{u}','{v}') is type {edge_type} but the pair "
                "is already a BLIND_EDGES entry -- this silently overwrites "
                "the blind edge's type and geometry in graph_builder. Use "
                "EDGE_BLIND to override the existing blind edge, or remove the "
                "pair from BLIND_EDGES if it is not blind-reachable"
            )

        if not _is_number(distance_cm) or distance_cm <= 0:
            raise FieldDataError(
                f"EDGES[{i}].distance_cm must be a positive number; "
                f"got: {distance_cm!r} -- use a value > 0 in centimetres"
            )

        if (
            not _is_number(heading_deg)
            or not (0.0 <= float(heading_deg) < 360.0)
        ):
            raise FieldDataError(
                f"EDGES[{i}].heading_deg must be a number in [0, 360); "
                f"got: {heading_deg!r} -- normalize via "
                "`heading % 360.0`"
            )

        # Optional: primitive_mode ----------------------------------------
        # Any concrete member of the edge type's mode enum is accepted --
        # for BLIND edges this includes BlindMode.STOP_AT_LINE. Only the
        # DEFAULT sentinel (== "absent") and values of the wrong enum type
        # (or no enum type at all) are rejected.
        if "primitive_mode" in entry:
            primitive_mode = entry["primitive_mode"]
            expected_cls, expected_name = mode_class_by_edge_type[edge_type]
            if not isinstance(primitive_mode, expected_cls):
                raise FieldDataError(
                    f"EDGES[{i}].primitive_mode = {primitive_mode!r} -- "
                    f"type {edge_type} requires a {expected_name} instance "
                    "(LineMode on LINE edges, BlindMode on BLIND edges, "
                    "ProjectionMode on PROJECTION edges)"
                )
            if primitive_mode == expected_cls.DEFAULT:
                raise FieldDataError(
                    f"EDGES[{i}].primitive_mode = {expected_name}.DEFAULT -- "
                    "DEFAULT means 'inherit PIPELINE_CONFIG', which is "
                    "what absence already encodes; omit the override or "
                    "pick a concrete mode"
                )

        # Optional: pre_turn_mode -----------------------------------------
        if "pre_turn_mode" in entry:
            pre_turn_mode = entry["pre_turn_mode"]
            if not isinstance(pre_turn_mode, TurnMode):
                raise FieldDataError(
                    f"EDGES[{i}].pre_turn_mode = {pre_turn_mode!r} -- "
                    "must be a TurnMode instance "
                    "(COUNTER_ROT, PIVOT_LEFT, or PIVOT_RIGHT)"
                )
            if int(pre_turn_mode) == 0:
                raise FieldDataError(
                    f"EDGES[{i}].pre_turn_mode = TurnMode.DEFAULT -- "
                    "DEFAULT means 'inherit PIPELINE_CONFIG.turn', which "
                    "is what absence already encodes; omit the override "
                    "or pick a concrete mode"
                )

        # Optional: direction --------------------------------------------
        # Per-directed-edge traversal override (asymmetric -- applies only
        # to the forward u -> v edge, never mirrored to the reverse).
        # Allowed values: "front" (FORCE_FORWARD), "rear" (FORCE_REVERSE),
        # or absent (DEFAULT / inherit PIPELINE_CONFIG.direction).
        if "direction" in entry:
            direction = entry["direction"]
            if direction not in ("front", "rear"):
                raise FieldDataError(
                    f"EDGES[{i}].direction must be 'front', 'rear', or "
                    f"absent; got {direction!r}"
                )

        # Optional: pivot -------------------------------------------------
        if "pivot" in entry:
            pivot = entry["pivot"]
            if not isinstance(pivot, Pivot):
                raise FieldDataError(
                    f"EDGES[{i}].pivot = {pivot!r} -- "
                    "must be a Pivot instance (Pivot.LEFT, Pivot.RIGHT, "
                    "or Pivot.NONE; though NONE is redundant since "
                    "absence already encodes it)"
                )
            # NONE is allowed but redundant -- do not reject it as
            # DEFAULT-sentinel, because Pivot.NONE is the meaningful
            # "counter-rotation" value, not a placeholder. The
            # cross-edge geometry check (is the upcoming turn delta
            # large enough for compensation to matter?) is deferred to
            # planner.py (T5).


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
        # The header writer emits each name verbatim into a C++ string
        # literal (`NodeInfo.name = "<name>"`). A double-quote, backslash, or
        # any non-printable/control char (newline, tab, ...) would produce a
        # malformed literal that fails the Arduino build on the auto-generated
        # routes.cpp -- far from the field-data root cause. Reject those here
        # so the error names the offending node at the right layer.
        if not name.isprintable() or '"' in name or "\\" in name:
            raise FieldDataError(
                f"NODES key {name!r} contains a disallowed character "
                "(double-quote, backslash, or non-printable/control char) -- "
                "node names are emitted into C++ string literals, so use only "
                "printable characters without quotes or backslashes"
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

    # 9. Optional rich-edge table (PR4 / T6). Absent on legacy fixtures.
    _validate_edges_field(field_data)
