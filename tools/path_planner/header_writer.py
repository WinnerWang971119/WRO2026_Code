"""Header writer: emit ``WRO2026/routes.h`` and ``WRO2026/routes.cpp`` (PR3).

Serialises the validated graph and next-hop table into a pair of C++
files the runtime ``PathTracker`` consumes:

* :data:`DEFAULT_HEADER_PATH` (``WRO2026/routes.h``) -- declarations
  (``constexpr`` sizes, enums, struct definitions, ``extern`` arrays).
* :data:`DEFAULT_SOURCE_PATH` (``WRO2026/routes.cpp``) -- definitions
  (``NODES``, ``NEXT_HOP``, ``EDGES``, ``PIPELINE_CONFIG``).

PR3 simplification: drops ``GoalApproach`` struct, ``BlindAction`` enum,
projection-related node category, and the per-goal blind-move arrays.
Every node is either a ``LINE`` or ``BLIND`` type; the runtime reaches
each one via the corresponding edge type.

Pipeline-mode extension (T7): emits mission-wide motion-mode defaults
(``PIPELINE_CONFIG``) plus four optional per-edge override slots
(``line_mode_override``, ``blind_mode_override``,
``projection_mode_override``, ``pre_turn_mode_override``). Each enum
carries a ``DEFAULT = 0`` sentinel meaning "inherit
``PIPELINE_CONFIG.<category>``" at runtime dispatch.

Determinism contract: byte-identical re-emit on identical inputs.

* Node ordering: alphabetical by name. Index 0 is the alphabetically
  first node; ``START_NODE`` is the index of ``field_data.START_NODE``.
* All dict iteration uses ``sorted(...)``.
* All floats are formatted ``{:.6f}f``.
* Line endings are LF only (``newline=''``).
* No ``datetime.now()`` in the output -- the embedded SHA256 of
  ``field_data.py`` is the only "version stamp."
* ``PIPELINE_CONFIG`` values are baked in from the Python singleton at
  emit time; they are intentionally NOT part of the SHA256 input (the
  hash continues to cover ``field_data.py`` only -- changing that is a
  separate behavior decision deferred past T7).
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Tuple

import networkx as nx

from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION
from tools.path_planner.pipeline_config import (
    PIPELINE_CONFIG,
    BlindMode,
    DirectionMode,
    LineMode,
    ProjectionMode,
    TurnMode,
)

DEFAULT_HEADER_PATH = Path("WRO2026/routes.h")
DEFAULT_SOURCE_PATH = Path("WRO2026/routes.cpp")
FIELD_DATA_PATH = Path("tools/path_planner/field_data.py")

NEXT_HOP_NONE_HEX = "0xFF"

_REGEN_COMMAND = "uv run python tools/path_planner/build_routes.py"
_INDENT = "    "

_EDGE_TYPE_TO_ENUM: Dict[str, str] = {
    EDGE_LINE:       "EdgeType::LINE",
    EDGE_BLIND:      "EdgeType::BLIND",
    EDGE_PROJECTION: "EdgeType::PROJECTION",
}

_NODE_TYPE_TO_ENUM: Dict[str, str] = {
    "line":  "NodeType::LINE",
    "blind": "NodeType::BLIND",
}

# Which override slot on EdgeInfo a ``primitive_mode`` value should land
# on, keyed by the edge's transport type. ``pre_turn_mode`` always lands
# on ``pre_turn_mode_override`` regardless of edge type (it describes the
# turn that precedes the edge, not the edge traversal itself).
_PRIMITIVE_MODE_SLOT_BY_EDGE_TYPE: Dict[str, str] = {
    EDGE_LINE:       "line_mode_override",
    EDGE_BLIND:      "blind_mode_override",
    EDGE_PROJECTION: "projection_mode_override",
}

# Default sentinel strings emitted when an override slot is unset. Each
# corresponds to the ``DEFAULT = 0`` member on the matching enum so a
# zero-initialised EdgeInfo means "inherit PIPELINE_CONFIG".
_LINE_MODE_DEFAULT       = "LineMode::DEFAULT"
_BLIND_MODE_DEFAULT      = "BlindMode::DEFAULT"
_PROJECTION_MODE_DEFAULT = "ProjectionMode::DEFAULT"
_TURN_MODE_DEFAULT       = "TurnMode::DEFAULT"
_DIRECTION_MODE_DEFAULT  = "DirectionMode::DEFAULT"

# The five per-category motion-mode enums, in the order their C++ bodies are
# emitted into routes.h. The bodies are generated from these Python ``IntEnum``
# definitions (single source of truth) so a member added/renamed in
# ``pipeline_config.py`` can never drift out of sync with routes.h. NodeType and
# EdgeType are intentionally excluded -- they are defined in graph_builder, not
# pipeline_config, and stay hand-written here.
_MODE_ENUMS: Tuple[type, ...] = (
    LineMode,
    BlindMode,
    ProjectionMode,
    TurnMode,
    DirectionMode,
)


def _emit_mode_enum_lines() -> List[str]:
    """Generate the C++ ``enum class`` bodies for the five mode enums.

    Each line is built from the Python :class:`~enum.IntEnum` members
    (``NAME = value`` in definition order). The enum-name column is padded so
    the ``: uint8_t`` bases align, matching the surrounding hand-written enum
    style in :func:`_build_routes_h_lines`.
    """
    name_width = max(len(enum_cls.__name__) for enum_cls in _MODE_ENUMS) + 1
    lines: List[str] = []
    for enum_cls in _MODE_ENUMS:
        members = ", ".join(
            member.name + " = " + str(member.value) for member in enum_cls
        )
        lines.append(
            _INDENT
            + "enum class "
            + enum_cls.__name__.ljust(name_width)
            + ": uint8_t { "
            + members
            + " };"
        )
    return lines


def _format_float(value: float) -> str:
    return "{:.6f}f".format(float(value))


def _build_index_lookup(node_names: List[str]) -> Dict[str, int]:
    return {name: i for i, name in enumerate(node_names)}


def _next_hop_cell(
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    from_name: str,
    to_name: str,
    index_of: Mapping[str, int],
) -> str:
    cell = next_hop_table[from_name][to_name]
    if cell is None:
        return NEXT_HOP_NONE_HEX
    return str(index_of[cell])


def _override_enum_value(
    enum_cls,
    mode_tag,
    default_sentinel: str,
) -> str:
    """Resolve a ``primitive_mode`` / ``pre_turn_mode`` tag to a C++ enum literal.

    Accepts either an :class:`enum.IntEnum` member from ``pipeline_config``
    or the bare string name of one (e.g. ``"FAST"`` or
    ``LineMode.FAST``). Unknown tags fall back to the DEFAULT sentinel
    rather than raising -- the validator (T6) owns strict checking, and
    we want the header writer to remain robust to legacy fixtures.
    """
    if mode_tag is None:
        return default_sentinel
    if isinstance(mode_tag, enum_cls):
        return enum_cls.__name__ + "::" + mode_tag.name
    if isinstance(mode_tag, str):
        try:
            member = enum_cls[mode_tag]
        except KeyError:
            return default_sentinel
        return enum_cls.__name__ + "::" + member.name
    # Integer or other -- best-effort lookup; fall back to DEFAULT.
    try:
        member = enum_cls(int(mode_tag))
    except (ValueError, TypeError):
        return default_sentinel
    return enum_cls.__name__ + "::" + member.name


def _resolve_overrides(edge_attrs: Mapping) -> Tuple[str, str, str, str, str]:
    """Compute the five override-enum literals for one edge.

    Returns
    ``(line_override, blind_override, projection_override, turn_override, direction_override)``
    in the same order they appear on the C++ ``EdgeInfo`` struct. The
    ``primitive_mode`` attribute lands on exactly one slot, picked by the
    edge's ``type``; the other two primitive slots stay DEFAULT. The
    ``pre_turn_mode`` attribute always lands on the turn slot. The
    ``direction_override`` attribute (populated by T4 in ``graph_builder``)
    always lands on the direction slot; absent today, so defaults to
    ``DirectionMode::DEFAULT``.
    """
    line_override       = _LINE_MODE_DEFAULT
    blind_override      = _BLIND_MODE_DEFAULT
    projection_override = _PROJECTION_MODE_DEFAULT
    turn_override       = _TURN_MODE_DEFAULT
    direction_override  = _DIRECTION_MODE_DEFAULT

    primitive_mode = edge_attrs.get("primitive_mode")
    if primitive_mode is not None:
        edge_type = edge_attrs.get("type")
        slot = _PRIMITIVE_MODE_SLOT_BY_EDGE_TYPE.get(edge_type)
        if slot == "line_mode_override":
            line_override = _override_enum_value(
                LineMode, primitive_mode, _LINE_MODE_DEFAULT
            )
        elif slot == "blind_mode_override":
            blind_override = _override_enum_value(
                BlindMode, primitive_mode, _BLIND_MODE_DEFAULT
            )
        elif slot == "projection_mode_override":
            projection_override = _override_enum_value(
                ProjectionMode, primitive_mode, _PROJECTION_MODE_DEFAULT
            )
        # Unknown edge_type -> drop the override silently; validator owns
        # the strict check.

    pre_turn_mode = edge_attrs.get("pre_turn_mode")
    if pre_turn_mode is not None:
        turn_override = _override_enum_value(
            TurnMode, pre_turn_mode, _TURN_MODE_DEFAULT
        )

    direction_mode = edge_attrs.get("direction_override")
    if direction_mode is not None:
        direction_override = _override_enum_value(
            DirectionMode, direction_mode, _DIRECTION_MODE_DEFAULT
        )

    return (
        line_override,
        blind_override,
        projection_override,
        turn_override,
        direction_override,
    )


def _none_edge_cell() -> str:
    return (
        "{ EdgeType::NONE, "
        + _format_float(0.0) + ", " + _format_float(0.0) + ", "
        + _LINE_MODE_DEFAULT + ", "
        + _BLIND_MODE_DEFAULT + ", "
        + _PROJECTION_MODE_DEFAULT + ", "
        + _TURN_MODE_DEFAULT + ", "
        + _DIRECTION_MODE_DEFAULT
        + " }"
    )


def _edge_cell(graph: nx.DiGraph, from_name: str, to_name: str) -> str:
    if not graph.has_edge(from_name, to_name):
        return _none_edge_cell()
    edge = graph.edges[from_name, to_name]
    enum_name = _EDGE_TYPE_TO_ENUM[edge["type"]]
    distance = _format_float(edge["distance_cm"])
    heading = _format_float(edge["heading_deg"])
    (
        line_override,
        blind_override,
        projection_override,
        turn_override,
        direction_override,
    ) = _resolve_overrides(edge)
    return (
        "{ " + enum_name + ", " + distance + ", " + heading + ", "
        + line_override + ", "
        + blind_override + ", "
        + projection_override + ", "
        + turn_override + ", "
        + direction_override
        + " }"
    )


def _node_row(
    name: str,
    field_data,
    index: int,
    name_column_width: int,
) -> str:
    x, y = field_data.NODES[name]
    node_type_enum = _NODE_TYPE_TO_ENUM[field_data.NODE_TYPES[name]]
    quoted = '"' + name + '"'
    padded = quoted + " " * max(0, name_column_width - len(quoted))
    return (
        "{ " + _format_float(x) + ", " + _format_float(y) + ", "
        + node_type_enum + ", " + padded + " },"
        + "     // index " + str(index)
    )


def compute_source_hash(field_data_source_path: Path) -> str:
    """Return the SHA256 hex digest of ``field_data.py``'s LF-normalized bytes.

    The digest is computed over the file's content with CRLF sequences collapsed
    to LF so the hash is stable across Windows (which checks the file out as
    CRLF under git's autocrlf=true) and Linux/CI (which checks it out as LF).
    Without this normalization, the embedded ``Source SHA256:`` stamp in
    routes.h / routes.cpp diverges between platforms and CI's
    idempotent-re-emit check fails on every Windows-authored regen.
    """
    raw = Path(field_data_source_path).read_bytes()
    normalized = raw.replace(b"\r\n", b"\n")
    return hashlib.sha256(normalized).hexdigest()


def _routes_h_banner(source_hash: str) -> List[str]:
    return [
        "// routes.h - AUTO-GENERATED by " + _REGEN_COMMAND,
        "// Source SHA256: " + source_hash,
        "// Do not hand-edit. Regenerate with: " + _REGEN_COMMAND,
    ]


def _routes_cpp_banner(source_hash: str) -> List[str]:
    return [
        "// routes.cpp - AUTO-GENERATED by " + _REGEN_COMMAND,
        "// Source SHA256: " + source_hash,
    ]


def _build_routes_h_lines(
    field_data,
    node_names: List[str],
    source_hash: str,
    start_node_index: int,
) -> List[str]:
    n_nodes = len(node_names)
    start_heading = _format_float(field_data.START_HEADING_DEG)

    lines: List[str] = []
    lines.extend(_routes_h_banner(source_hash))
    lines.append("")
    lines.append("#ifndef ROUTES_H")
    lines.append("#define ROUTES_H")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("namespace Routes {")
    lines.append(_INDENT + "constexpr uint8_t N_NODES = " + str(n_nodes) + ";")
    lines.append("")
    lines.append(_INDENT + "// Node category. LINE nodes are reached by line-following")
    lines.append(_INDENT + "// to the next intersection; BLIND nodes are reached by a")
    lines.append(_INDENT + "// dead-reckoned turn + forward move from a connected node.")
    lines.append(_INDENT + "enum class NodeType : uint8_t { LINE = 0, BLIND = 1 };")
    lines.append("")
    lines.append(_INDENT + "// Edge motion class, dispatched by the runtime.")
    lines.append(
        _INDENT
        + "enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2, "
        + "PROJECTION = 3 };"
    )
    lines.append("")
    lines.append(
        _INDENT + "// Per-category motion-mode selectors. DEFAULT (= 0) means"
    )
    lines.append(
        _INDENT + "// 'inherit PIPELINE_CONFIG.<category>' at dispatch time."
    )
    lines.extend(_emit_mode_enum_lines())
    lines.append("")
    lines.append(
        _INDENT + "// Mission-wide motion-mode defaults, baked in from"
    )
    lines.append(
        _INDENT + "// tools/path_planner/pipeline_config.py at build time."
    )
    lines.append(_INDENT + "struct PipelineConfig {")
    lines.append(_INDENT * 2 + "LineMode       line;")
    lines.append(_INDENT * 2 + "BlindMode      blind;")
    lines.append(_INDENT * 2 + "ProjectionMode projection;")
    lines.append(_INDENT * 2 + "TurnMode       turn;")
    lines.append(_INDENT * 2 + "DirectionMode  direction;")
    lines.append(_INDENT + "};")
    lines.append(_INDENT + "extern const PipelineConfig PIPELINE_CONFIG;")
    lines.append("")
    lines.append(
        _INDENT
        + "struct NodeInfo { float x_cm; float y_cm; NodeType type; "
        + "const char* name; };"
    )
    lines.append(_INDENT + "struct EdgeInfo {")
    lines.append(_INDENT * 2 + "EdgeType       type;")
    lines.append(_INDENT * 2 + "float          distance_cm;")
    lines.append(_INDENT * 2 + "float          heading_deg;")
    lines.append(_INDENT * 2 + "LineMode       line_mode_override;")
    lines.append(_INDENT * 2 + "BlindMode      blind_mode_override;")
    lines.append(_INDENT * 2 + "ProjectionMode projection_mode_override;")
    lines.append(_INDENT * 2 + "TurnMode       pre_turn_mode_override;")
    lines.append(_INDENT * 2 + "DirectionMode  direction_override;")
    lines.append(_INDENT + "};")
    lines.append("")
    lines.append(_INDENT + "extern const NodeInfo NODES[N_NODES];")
    lines.append(_INDENT + "extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];")
    lines.append(_INDENT + "extern const EdgeInfo EDGES   [N_NODES][N_NODES];")
    lines.append("")
    lines.append(
        _INDENT + "// Sentinel for unreachable next-hop entries; PathTracker"
    )
    lines.append(
        _INDENT + "// treats this as 'no path; halt and report'."
    )
    lines.append(
        _INDENT + "constexpr uint8_t NEXT_HOP_NONE = " + NEXT_HOP_NONE_HEX + ";"
    )
    lines.append(
        _INDENT + "constexpr uint8_t START_NODE = " + str(start_node_index) + ";"
    )
    lines.append(
        _INDENT + "constexpr float   START_HEADING_DEG = " + start_heading + ";"
    )
    lines.append("}  // namespace Routes")
    lines.append("#endif  // ROUTES_H")
    return lines


def _emit_nodes_array(field_data, node_names: List[str]) -> List[str]:
    # ``default=1`` mirrors the sibling _emit_next_hop_table alignment calc and
    # keeps an empty-node fixture degrading gracefully instead of raising
    # ValueError from max() over an empty sequence.
    name_column_width = max((len('"' + n + '"') for n in node_names), default=1)
    out: List[str] = []
    out.append(_INDENT + "const NodeInfo NODES[N_NODES] = {")
    for i, name in enumerate(node_names):
        out.append(_INDENT * 2 + _node_row(name, field_data, i, name_column_width))
    out.append(_INDENT + "};")
    return out


def _emit_next_hop_table(
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    node_names: List[str],
    index_of: Mapping[str, int],
) -> List[str]:
    cells: List[List[str]] = [
        [_next_hop_cell(next_hop_table, frm, to, index_of) for to in node_names]
        for frm in node_names
    ]
    cell_width = max((len(c) for row in cells for c in row), default=1)

    out: List[str] = []
    out.append(_INDENT + "const uint8_t NEXT_HOP[N_NODES][N_NODES] = {")
    out.append(
        _INDENT * 2
        + "// [from][to] = next-hop node index; "
        + NEXT_HOP_NONE_HEX
        + " = unreachable"
    )
    for i, name in enumerate(node_names):
        padded_cells = [c.rjust(cell_width) for c in cells[i]]
        joined = ", ".join(padded_cells)
        out.append(
            _INDENT * 2 + "{ " + joined + " }, "
            + "// from " + name + " (index " + str(i) + ")"
        )
    out.append(_INDENT + "};")
    return out


def _emit_edges_table(graph: nx.DiGraph, node_names: List[str]) -> List[str]:
    out: List[str] = []
    out.append(_INDENT + "const EdgeInfo EDGES[N_NODES][N_NODES] = {")
    out.append(_INDENT * 2 + "// [from][to]; type=NONE if no direct edge")
    for i, frm in enumerate(node_names):
        out.append(_INDENT * 2 + "{ // from " + frm + " (index " + str(i) + ")")
        for to in node_names:
            cell = _edge_cell(graph, frm, to)
            out.append(_INDENT * 3 + cell + ", // to " + to)
        out.append(_INDENT * 2 + "},")
    out.append(_INDENT + "};")
    return out


def _emit_pipeline_config() -> List[str]:
    """Bake the Python-side PIPELINE_CONFIG singleton into a C++ definition."""
    return [
        _INDENT + "const PipelineConfig PIPELINE_CONFIG = {",
        _INDENT * 2 + "LineMode::"       + PIPELINE_CONFIG.line.name       + ",",
        _INDENT * 2 + "BlindMode::"      + PIPELINE_CONFIG.blind.name      + ",",
        _INDENT * 2 + "ProjectionMode::" + PIPELINE_CONFIG.projection.name + ",",
        _INDENT * 2 + "TurnMode::"       + PIPELINE_CONFIG.turn.name       + ",",
        _INDENT * 2 + "DirectionMode::"  + PIPELINE_CONFIG.direction.name  + ",",
        _INDENT + "};",
    ]


def _build_routes_cpp_lines(
    field_data,
    graph: nx.DiGraph,
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    node_names: List[str],
    index_of: Mapping[str, int],
    source_hash: str,
) -> List[str]:
    lines: List[str] = []
    lines.extend(_routes_cpp_banner(source_hash))
    lines.append("")
    lines.append('#include "routes.h"')
    lines.append("")
    lines.append("namespace Routes {")

    lines.extend(_emit_pipeline_config())
    lines.append("")
    lines.extend(_emit_nodes_array(field_data, node_names))
    lines.append("")
    lines.extend(_emit_next_hop_table(next_hop_table, node_names, index_of))
    lines.append("")
    lines.extend(_emit_edges_table(graph, node_names))
    lines.append("}  // namespace Routes")
    return lines


def emit_routes(
    field_data,
    graph: nx.DiGraph,
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    *,
    header_path: Path = DEFAULT_HEADER_PATH,
    source_path: Path = DEFAULT_SOURCE_PATH,
    field_data_source_path: Path = FIELD_DATA_PATH,
) -> Tuple[Path, Path]:
    """Emit ``routes.h`` and ``routes.cpp`` to disk; return their paths."""
    header_path = Path(header_path)
    source_path = Path(source_path)
    field_data_source_path = Path(field_data_source_path)

    node_names: List[str] = sorted(field_data.NODES.keys())
    n_nodes = len(node_names)
    if n_nodes >= 256:
        raise ValueError(
            f"emit_routes: N_NODES={n_nodes} exceeds the uint8_t/NEXT_HOP_NONE(0xFF)"
            " ceiling (max 255 nodes). Node ids are 0..N_NODES-1, so the 0xFF"
            " sentinel only collides with a valid id once N_NODES reaches 256."
        )
    index_of = _build_index_lookup(node_names)

    if field_data.START_NODE not in index_of:
        raise KeyError(
            "field_data.START_NODE = "
            + repr(field_data.START_NODE)
            + " is not present in NODES"
        )
    start_node_index = index_of[field_data.START_NODE]

    source_hash = compute_source_hash(field_data_source_path)

    header_lines = _build_routes_h_lines(
        field_data, node_names, source_hash, start_node_index
    )
    cpp_lines = _build_routes_cpp_lines(
        field_data, graph, next_hop_table, node_names, index_of, source_hash
    )

    header_path.parent.mkdir(parents=True, exist_ok=True)
    source_path.parent.mkdir(parents=True, exist_ok=True)

    header_text = "\n".join(header_lines) + "\n"
    cpp_text = "\n".join(cpp_lines) + "\n"

    with open(header_path, "w", encoding="utf-8", newline="") as fh:
        fh.write(header_text)
    with open(source_path, "w", encoding="utf-8", newline="") as fh:
        fh.write(cpp_text)

    return header_path, source_path


__all__ = [
    "DEFAULT_HEADER_PATH",
    "DEFAULT_SOURCE_PATH",
    "FIELD_DATA_PATH",
    "NEXT_HOP_NONE_HEX",
    "compute_source_hash",
    "emit_routes",
]
