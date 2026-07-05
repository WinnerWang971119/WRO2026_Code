"""Header writer: emit ``WRO2026/src/nav/routes.h`` and ``WRO2026/src/nav/routes.cpp`` (PR3).

Serialises the validated graph and next-hop table into a pair of C++
files the runtime ``PathTracker`` consumes:

* :data:`DEFAULT_HEADER_PATH` (``WRO2026/src/nav/routes.h``) -- declarations
  (``constexpr`` sizes, enums, struct definitions, ``extern`` arrays).
* :data:`DEFAULT_SOURCE_PATH` (``WRO2026/src/nav/routes.cpp``) -- definitions
  (``NODES``, ``NEXT_HOP``, ``EDGES``, ``PIPELINE_CONFIG``).

PR3 simplification: drops ``GoalApproach`` struct, ``BlindAction`` enum,
and the per-goal blind-move arrays.

Node category (issue #64): a node is a ``LINE``, ``BLIND``, ``RESET``, or
``PROJECTION`` type. ``EdgeType`` is reduced to ``NONE`` / ``LINE`` /
``BLIND`` -- projection is a node property, not an edge type, so the
per-hop behavior falls out of ``(edge_type, destination_node_type)``.

Pipeline-mode extension (T7): emits mission-wide motion-mode defaults
(``PIPELINE_CONFIG``) plus two optional per-edge mode-override slots
(``line_mode_override``, ``blind_mode_override``) and a
``direction_override`` slot. Each enum carries a ``DEFAULT = 0`` sentinel
meaning "inherit ``PIPELINE_CONFIG.<category>``" at runtime dispatch.

Per-turn pivot extension (ADR-036, supersedes the per-edge issue-#44
model): single-wheel pivots are no longer a per-edge field. The planner's
:func:`build_pivot_turns` resolves every authored turn triple ``(u, v, w)``
into a :class:`~tools.path_planner.planner.PivotTurnRecord` (geometry-derived
wheel + over-travel δ). The emitter serialises those records into a sparse,
sorted ``PivotTurn PIVOT_TURNS[]`` table plus an ``N_PIVOT_TURNS`` truth
count -- one record per turn the route may take, not an ``N**3`` dense array.
The old per-edge ``pivot_trim_cm`` / ``pre_turn_mode_override`` fields and
their ``offsetof`` layout guard are removed from ``EdgeInfo`` entirely; the
firmware resolves the wheel and the trim from one ``PivotTurn`` lookup, so
the two can never disagree.

Determinism contract: byte-identical re-emit on identical inputs.

* Node ordering: alphabetical by name. Index 0 is the alphabetically
  first node; ``START_NODE`` is the index of ``field_data.START_NODE``.
* All dict iteration uses ``sorted(...)``.
* All floats are formatted ``{:.6f}f``.
* Line endings are LF only (``newline=''``).
* No ``datetime.now()`` in the output -- the embedded SHA256 of
  ``field_data.py`` is the only "version stamp."
* ``PIPELINE_CONFIG`` values are field-sourced (ADR-047): the four
  optional ``PIPELINE_*`` keys in ``field_data.json`` resolve to enum
  members, falling back to the ``pipeline_config.py`` singleton when a key
  is absent. Because they live in ``field_data.json`` -- which IS folded
  into the ``Source SHA256:`` stamp via ``compute_source_hash`` /
  ``emit_routes(..., extra_source_paths=...)`` -- editing a pipeline
  default DOES change the hash. A field that omits every key emits the
  singleton defaults, so committed routes stay byte-identical.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import (
    TYPE_CHECKING,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)

import networkx as nx

from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE
from tools.path_planner.pipeline_config import (
    BlindMode,
    DirectionMode,
    LineMode,
    PipelineConfig,
    StopAtLineMode,
    TurnMode,
)

if TYPE_CHECKING:
    # Type-only import: the emitter consumes pivot records structurally
    # (it reads ``.u`` / ``.v`` / ``.w`` / ``.wheel`` / ``.trim_cm``), so a
    # runtime dependency on the planner module is unnecessary and would
    # tighten the import graph for no benefit. Importing under TYPE_CHECKING
    # keeps the annotation precise without the runtime coupling.
    from tools.path_planner.planner import PivotTurnRecord

DEFAULT_HEADER_PATH = Path("WRO2026/src/nav/routes.h")
DEFAULT_SOURCE_PATH = Path("WRO2026/src/nav/routes.cpp")
FIELD_DATA_PATH = Path("tools/path_planner/field_data.py")
FIELD_DATA_JSON_PATH = Path("tools/path_planner/field_data.json")

NEXT_HOP_NONE_HEX = "0xFF"

_REGEN_COMMAND = "uv run python tools/path_planner/build_routes.py"
_INDENT = "    "

_EDGE_TYPE_TO_ENUM: Dict[str, str] = {
    EDGE_LINE:  "EdgeType::LINE",
    EDGE_BLIND: "EdgeType::BLIND",
}

_NODE_TYPE_TO_ENUM: Dict[str, str] = {
    "line":       "NodeType::LINE",
    "blind":      "NodeType::BLIND",
    "reset":      "NodeType::RESET",
    "projection": "NodeType::PROJECTION",
}

# Wall-zero reset axis encoding -- the load-bearing cross-task contract
# (T15 ``reanchorAtWall(uint8_t axis, ...)`` decodes it). The authored
# ``RESET_NODES[name].axis`` string maps to the ``uint8_t`` emitted into
# the ``ResetNode.axis`` cell: ``"x" -> 0``, ``"y" -> 1``. Do NOT reorder.
_RESET_AXIS_TO_UINT8: Dict[str, int] = {
    "x": 0,
    "y": 1,
}

# Which override slot on EdgeInfo a ``primitive_mode`` value should land
# on, keyed by the edge's transport type. The turn mode is no longer a
# per-edge field -- single-wheel pivots moved to the sparse ``PIVOT_TURNS``
# table (ADR-036) -- so there is no longer a per-edge turn slot here.
_PRIMITIVE_MODE_SLOT_BY_EDGE_TYPE: Dict[str, str] = {
    EDGE_LINE:  "line_mode_override",
    EDGE_BLIND: "blind_mode_override",
}

# Default sentinel strings emitted when an override slot is unset. Each
# corresponds to the ``DEFAULT = 0`` member on the matching enum so a
# zero-initialised EdgeInfo means "inherit PIPELINE_CONFIG".
_LINE_MODE_DEFAULT         = "LineMode::DEFAULT"
_BLIND_MODE_DEFAULT        = "BlindMode::DEFAULT"
_DIRECTION_MODE_DEFAULT    = "DirectionMode::DEFAULT"
_STOP_AT_LINE_MODE_DEFAULT = "StopAtLineMode::DEFAULT"

# C++ qualified-name prefix for ``TurnMode`` enum literals emitted into the
# ``PIVOT_TURNS`` table (e.g. ``TurnMode::PIVOT_LEFT``). The pivot wheel is
# emitted from the record's ``TurnMode`` member by name, consistent with how
# ``_emit_pipeline_config`` qualifies its ``TurnMode`` value.
_TURN_MODE_PREFIX = "TurnMode::"

# The per-category motion-mode enums, in the order their C++ bodies are
# emitted into routes.h. The bodies are generated from these Python ``IntEnum``
# definitions (single source of truth) so a member added/renamed in
# ``pipeline_config.py`` can never drift out of sync with routes.h. NodeType and
# EdgeType are intentionally excluded -- they are defined in graph_builder, not
# pipeline_config, and stay hand-written here.
_MODE_ENUMS: Tuple[type, ...] = (
    LineMode,
    BlindMode,
    TurnMode,
    DirectionMode,
)


def _emit_mode_enum_lines() -> List[str]:
    """Generate the C++ ``enum class`` bodies for the mode enums.

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
    """Resolve a ``primitive_mode`` / ``direction`` tag to a C++ enum literal.

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
    # Integer or other -- best-effort lookup by value; fall back to DEFAULT.
    # Cross-enum int-coercion hazard: enum_cls(int(mode_tag)) matches on
    # *value*, not name, so the same integer can resolve to a member of a
    # different enum class if their value spaces overlap (e.g. LineMode(2)
    # and BlindMode(2) are distinct members).  This branch is latent-safe
    # only because field_data builders always bake real enum *members* (never
    # raw ints), so an out-of-range or wrong-type int reaching here indicates
    # a validator gap, not a normal code path.  If that assumption ever
    # breaks, the int would silently mis-map rather than raise.
    try:
        member = enum_cls(int(mode_tag))
    except (ValueError, TypeError):
        return default_sentinel
    return enum_cls.__name__ + "::" + member.name


def _resolve_overrides(edge_attrs: Mapping) -> Tuple[str, str, str, str]:
    """Compute the four override-enum literals for one edge.

    Returns ``(line_override, blind_override, direction_override,
    stop_at_line_override)`` in the same order they appear on the C++
    ``EdgeInfo`` struct. The ``primitive_mode`` attribute lands on exactly one
    slot, picked by the edge's ``type`` (LINE or BLIND); the other primitive
    slot stays DEFAULT. The ``direction_override`` attribute always lands on
    the direction slot; absent today, so defaults to ``DirectionMode::DEFAULT``.
    The ``stop_at_line_override`` attribute is a second directional slot read
    straight off the edge (a ``StopAtLineMode`` member baked on by the graph
    builder); absent today, so defaults to ``StopAtLineMode::DEFAULT``.

    The turn mode is no longer a per-edge override: single-wheel pivots
    moved to the sparse ``PIVOT_TURNS`` table (ADR-036), so ``pre_turn_mode``
    is neither read here nor emitted on ``EdgeInfo``.
    """
    line_override         = _LINE_MODE_DEFAULT
    blind_override        = _BLIND_MODE_DEFAULT
    direction_override    = _DIRECTION_MODE_DEFAULT
    stop_at_line_override = _STOP_AT_LINE_MODE_DEFAULT

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
        # Unknown edge_type -> drop the override silently; validator owns
        # the strict check.

    direction_mode = edge_attrs.get("direction_override")
    if direction_mode is not None:
        direction_override = _override_enum_value(
            DirectionMode, direction_mode, _DIRECTION_MODE_DEFAULT
        )

    stop_at_line_mode = edge_attrs.get("stop_at_line_override")
    if stop_at_line_mode is not None:
        stop_at_line_override = _override_enum_value(
            StopAtLineMode, stop_at_line_mode, _STOP_AT_LINE_MODE_DEFAULT
        )

    return (
        line_override,
        blind_override,
        direction_override,
        stop_at_line_override,
    )


def _none_edge_cell() -> str:
    # Field order MUST match the C++ ``EdgeInfo`` struct positionally (ADR-036:
    # ``pivot_trim_cm`` / ``pre_turn_mode_override`` are gone -- pivots live in
    # the sparse ``PIVOT_TURNS`` table now; issue #64: ``projection_mode_override``
    # is gone -- projection is a node type, not an edge type). Both this and
    # ``_edge_cell`` must emit the same four override slots in the same order
    # (line, blind, direction, stop_at_line) or the brace-enclosed aggregate
    # initialiser misaligns.
    return (
        "{ EdgeType::NONE, "
        + _format_float(0.0) + ", " + _format_float(0.0) + ", "
        + _LINE_MODE_DEFAULT + ", "
        + _BLIND_MODE_DEFAULT + ", "
        + _DIRECTION_MODE_DEFAULT
        + ", " + _STOP_AT_LINE_MODE_DEFAULT
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
        direction_override,
        stop_at_line_override,
    ) = _resolve_overrides(edge)
    return (
        "{ " + enum_name + ", " + distance + ", " + heading + ", "
        + line_override + ", "
        + blind_override + ", "
        + direction_override
        + ", " + stop_at_line_override
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


def compute_source_hash(*field_data_source_paths: Path) -> str:
    """Return the SHA256 hex digest of one or more field-data sources.

    Each path's bytes are read with CRLF sequences collapsed to LF so the hash
    is stable across Windows (which checks files out as CRLF under git's
    autocrlf=true) and Linux/CI (which checks them out as LF). Without this
    normalization, the embedded ``Source SHA256:`` stamp in routes.h /
    routes.cpp diverges between platforms and CI's idempotent-re-emit check
    fails on every Windows-authored regen.

    When more than one path is supplied the LF-normalized contents are hashed
    in argument order with a NUL byte separating files, so the stamp covers
    every input that feeds the emitter. The single-path digest is unchanged
    (no separator) so legacy callers and goldens stay stable. Geometry now
    lives in ``field_data.json`` while the rich ``EDGES`` table stays in
    ``field_data.py``; pass both so editing either bumps the stamp.

    :param field_data_source_paths: one or more source files to hash, in order.
    :return: the 64-character hex SHA256 digest.
    """
    if not field_data_source_paths:
        raise ValueError("compute_source_hash: at least one source path is required")
    digest = hashlib.sha256()
    for index, source_path in enumerate(field_data_source_paths):
        if index:
            digest.update(b"\0")
        raw = Path(source_path).read_bytes()
        digest.update(raw.replace(b"\r\n", b"\n"))
    return digest.hexdigest()


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
    pivot_count: int,
    reset_count: int,
) -> List[str]:
    n_nodes = len(node_names)
    start_heading = _format_float(field_data.START_HEADING_DEG)
    # The array bound is never zero: a zero-length array bound is ill-formed
    # in standard C++, so a pivot-free field still emits a single zeroed
    # placeholder element while ``N_PIVOT_TURNS`` reports the truth count (0).
    pivot_array_len = max(1, pivot_count)
    # Same min-size-1 placeholder discipline for the reset table: a reset-free
    # field still emits a single zeroed placeholder while ``N_RESET_NODES``
    # reports the truth count (0).
    reset_array_len = max(1, reset_count)

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
    lines.append(
        _INDENT
        + "// Field-wide: auto stop-at-line seek on a BLIND hop that precedes"
    )
    lines.append(_INDENT + "// a PROJECTION.")
    lines.append(
        _INDENT
        + "constexpr bool BLIND_PROJECTION_AUTO_SEEK = "
        + ("true" if field_data.BLIND_PROJECTION_AUTO_SEEK else "false")
        + ";"
    )
    lines.append("")
    lines.append(_INDENT + "// Node category. LINE nodes are reached by line-following")
    lines.append(_INDENT + "// to the next intersection; BLIND nodes are reached by a")
    lines.append(_INDENT + "// dead-reckoned turn + forward move from a connected node;")
    lines.append(_INDENT + "// PROJECTION nodes sit on a line but at no junction (dead-")
    lines.append(_INDENT + "// reckon to identity; a BLIND hop into one auto-seeks).")
    lines.append(
        _INDENT
        + "enum class NodeType : uint8_t { LINE = 0, BLIND = 1, RESET = 2, "
        + "PROJECTION = 3 };"
    )
    lines.append("")
    lines.append(_INDENT + "// Edge motion class, dispatched by the runtime.")
    lines.append(
        _INDENT
        + "enum class EdgeType : uint8_t { NONE = 0, LINE = 1, BLIND = 2 };"
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
        _INDENT + "// Per-edge stop-at-line seek override, decoupled from BlindMode."
    )
    lines.append(
        _INDENT
        + "enum class StopAtLineMode : uint8_t { "
        + ", ".join(
            member.name + " = " + str(member.value) for member in StopAtLineMode
        )
        + " };"
    )
    lines.append("")
    lines.append(
        _INDENT + "// Mission-wide motion-mode defaults, baked in from"
    )
    lines.append(
        _INDENT + "// tools/path_planner/pipeline_config.py at build time."
    )
    lines.append(_INDENT + "struct PipelineConfig {")
    lines.append(_INDENT * 2 + "LineMode      line;")
    lines.append(_INDENT * 2 + "BlindMode     blind;")
    lines.append(_INDENT * 2 + "TurnMode      turn;")
    lines.append(_INDENT * 2 + "DirectionMode direction;")
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
    lines.append(_INDENT * 2 + "DirectionMode  direction_override;")
    lines.append(_INDENT * 2 + "StopAtLineMode stop_at_line_override;")
    lines.append(_INDENT + "};")
    lines.append("")
    lines.append(
        _INDENT + "// Single-wheel pivot turn (ADR-036). A pivot is a property"
    )
    lines.append(
        _INDENT + "// of a turn (arrive from u, pivot at v, leave to w), not of"
    )
    lines.append(
        _INDENT + "// an edge: the firmware resolves the wheel and the over-"
    )
    lines.append(
        _INDENT + "// travel trim from ONE record, so the two cannot disagree."
    )
    lines.append(_INDENT + "struct PivotTurn {")
    lines.append(_INDENT * 2 + "uint8_t  u;          // node index: arrive-from")
    lines.append(_INDENT * 2 + "uint8_t  v;          // node index: pivot-at")
    lines.append(_INDENT * 2 + "uint8_t  w;          // node index: leave-to")
    lines.append(
        _INDENT * 2 + "TurnMode wheel;      // PIVOT_LEFT | PIVOT_RIGHT (uint8_t enum)"
    )
    lines.append(_INDENT * 2 + "float    trim_cm;    // delta over-travel, cm")
    lines.append(_INDENT + "};")
    lines.append("")
    lines.append(
        _INDENT + "// Wall-zero reset node (PR2). A reset node re-anchors the"
    )
    lines.append(
        _INDENT + "// chassis against a known wall: drive to the wall on `axis`,"
    )
    lines.append(
        _INDENT + "// re-zero that coordinate to `wall_coord`, and adopt `facing`"
    )
    lines.append(
        _INDENT + "// as the heading. Reached like any node; the reset payload"
    )
    lines.append(
        _INDENT + "// lives ONLY here, not on NODES[] (sparse, like PivotTurn)."
    )
    lines.append(_INDENT + "struct ResetNode {")
    lines.append(_INDENT * 2 + "uint8_t  node;        // node index")
    # AXIS ENCODING CONTRACT (load-bearing): "x" -> 0, "y" -> 1. T15's
    # reanchorAtWall(uint8_t axis, ...) decodes this exact mapping.
    lines.append(
        _INDENT * 2 + "uint8_t  axis;        // 0 = x, 1 = y (wall parallel to axis)"
    )
    lines.append(_INDENT * 2 + "float    wall_coord;  // cm; the re-zeroed coordinate")
    lines.append(_INDENT * 2 + "int16_t  facing;      // degrees [0, 360)")
    lines.append(_INDENT + "};")
    lines.append("")
    lines.append(_INDENT + "extern const NodeInfo NODES[N_NODES];")
    lines.append(_INDENT + "extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];")
    lines.append(_INDENT + "extern const EdgeInfo EDGES   [N_NODES][N_NODES];")
    lines.append("")
    lines.append(
        _INDENT + "// Sparse, sorted single-wheel pivot table (ADR-036). Sorted by"
    )
    lines.append(
        _INDENT + "// (u, v, w) node-index tuple for a deterministic golden and a"
    )
    lines.append(
        _INDENT + "// short bounded linear scan in FieldMap::findPivotTurn."
    )
    lines.append(
        _INDENT
        + "constexpr uint8_t N_PIVOT_TURNS = "
        + str(pivot_count)
        + ";"
    )
    lines.append(
        _INDENT
        + "extern const PivotTurn PIVOT_TURNS["
        + str(pivot_array_len)
        + "];"
    )
    lines.append("")
    lines.append(
        _INDENT + "// Sparse, sorted wall-zero reset table (PR2). Sorted by node"
    )
    lines.append(
        _INDENT + "// index for a deterministic golden and a short bounded linear"
    )
    lines.append(
        _INDENT + "// scan in FieldMap::getResetInfo."
    )
    lines.append(
        _INDENT
        + "constexpr uint8_t N_RESET_NODES = "
        + str(reset_count)
        + ";"
    )
    lines.append(
        _INDENT
        + "extern const ResetNode RESET_NODES["
        + str(reset_array_len)
        + "];"
    )
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


def _emit_pipeline_config(field_data) -> List[str]:
    """Bake the field-sourced PIPELINE_CONFIG defaults into a C++ definition.

    Resolves the four mission-wide motion-mode defaults from the field data's
    optional ``PIPELINE_*`` attributes (ADR-047), each an enum member NAME, an
    enum INSTANCE, or ``None`` (absent -> inherit the ``pipeline_config.py``
    :data:`PIPELINE_CONFIG` singleton default for that category). A field that
    omits every key yields a config byte-identical to the singleton, so the
    committed goldens stay unchanged.

    Resolution and the DEFAULT-sentinel rule live in
    :meth:`PipelineConfig.from_field_data`; the validator
    (``_validate_pipeline_config``) catches a bad value before emit, so an
    unresolvable name reaching here raises :class:`KeyError` rather than
    emitting garbage.
    """
    config = PipelineConfig.from_field_data(field_data)
    return [
        _INDENT + "const PipelineConfig PIPELINE_CONFIG = {",
        _INDENT * 2 + "LineMode::"      + config.line.name      + ",",
        _INDENT * 2 + "BlindMode::"     + config.blind.name     + ",",
        _INDENT * 2 + "TurnMode::"      + config.turn.name      + ",",
        _INDENT * 2 + "DirectionMode::" + config.direction.name + ",",
        _INDENT + "};",
    ]


def _wheel_enum_literal(wheel) -> str:
    """Render a pivot record's ``wheel`` as a ``TurnMode::<NAME>`` literal.

    Accepts a :class:`~tools.path_planner.pipeline_config.TurnMode` member
    (the normal case from :func:`build_pivot_turns`) or the bare string name
    / integer value of one, mirroring :func:`_override_enum_value`'s
    tolerance of legacy / hand-built fixtures. An unresolvable value falls
    back to ``TurnMode::DEFAULT`` rather than raising -- the validator owns
    strict checking, and the emitter stays robust.
    """
    if isinstance(wheel, TurnMode):
        return _TURN_MODE_PREFIX + wheel.name
    if isinstance(wheel, str):
        try:
            return _TURN_MODE_PREFIX + TurnMode[wheel].name
        except KeyError:
            return _TURN_MODE_PREFIX + TurnMode.DEFAULT.name
    try:
        return _TURN_MODE_PREFIX + TurnMode(int(wheel)).name
    except (ValueError, TypeError):
        return _TURN_MODE_PREFIX + TurnMode.DEFAULT.name


def _resolve_pivot_rows(
    pivot_records: Sequence["PivotTurnRecord"],
    index_of: Mapping[str, int],
) -> List[Tuple[int, int, int, str, float]]:
    """Resolve pivot records to sorted ``(u, v, w, wheel_literal, trim_cm)`` rows.

    Each record's node NAMES (``u`` / ``v`` / ``w``) are mapped to their
    integer indices via ``index_of`` (the same NODES-order lookup the rest of
    the emitter uses). Rows are sorted by the ``(u, v, w)`` index tuple so the
    emitted ``PIVOT_TURNS`` table is deterministic for golden byte-compares
    and supports the firmware's ordered linear scan.

    A record naming a node absent from ``index_of`` is skipped defensively
    (mirrors :func:`build_pivot_turns`' edge-existence guard): the validator
    guarantees every triple's nodes exist, so this only fires for a hand-built
    caller that bypasses validation, and skipping keeps the emitter from
    raising :class:`KeyError`. The truth count ``N_PIVOT_TURNS`` is derived
    from the returned rows, so a skipped record drops out of the count too.
    """
    rows: List[Tuple[int, int, int, str, float]] = []
    for record in pivot_records:
        if (
            record.u not in index_of
            or record.v not in index_of
            or record.w not in index_of
        ):
            continue
        rows.append(
            (
                index_of[record.u],
                index_of[record.v],
                index_of[record.w],
                _wheel_enum_literal(record.wheel),
                float(record.trim_cm),
            )
        )
    # Sort by the (u, v, w) index tuple only; the wheel/trim payload never
    # participates in ordering so two records with the same triple (which the
    # validator rejects as a duplicate) would still sort deterministically.
    rows.sort(key=lambda row: (row[0], row[1], row[2]))
    return rows


def _emit_pivot_turns(
    pivot_rows: Sequence[Tuple[int, int, int, str, float]],
) -> List[str]:
    """Emit the ``const PivotTurn PIVOT_TURNS[...]`` definition.

    ``pivot_rows`` are the already-resolved, already-sorted rows from
    :func:`_resolve_pivot_rows`. When empty, a single zeroed placeholder
    element is emitted (the array bound is ``max(1, count)``; a zero-length
    array bound is ill-formed in standard C++) while ``N_PIVOT_TURNS`` in
    routes.h reports the truth count of ``0``.
    """
    array_len = max(1, len(pivot_rows))
    out: List[str] = []
    out.append(
        _INDENT
        + "const PivotTurn PIVOT_TURNS["
        + str(array_len)
        + "] = {"
    )
    if pivot_rows:
        out.append(_INDENT * 2 + "// sorted by (u, v, w) node index")
        for u, v, w, wheel_literal, trim_cm in pivot_rows:
            out.append(
                _INDENT * 2
                + "{ "
                + str(u) + ", " + str(v) + ", " + str(w) + ", "
                + wheel_literal + ", "
                + _format_float(trim_cm)
                + " },"
            )
    else:
        # Pivot-free field: one zeroed placeholder so the array bound is >= 1.
        # N_PIVOT_TURNS == 0 tells FieldMap::findPivotTurn to scan nothing, so
        # this element is never read at runtime.
        out.append(
            _INDENT * 2
            + "// placeholder: N_PIVOT_TURNS == 0, never read at runtime"
        )
        out.append(
            _INDENT * 2
            + "{ 0, 0, 0, "
            + _TURN_MODE_PREFIX + TurnMode.DEFAULT.name + ", "
            + _format_float(0.0)
            + " },"
        )
    out.append(_INDENT + "};")
    return out


def _resolve_reset_rows(
    reset_nodes: Mapping[str, Mapping],
    index_of: Mapping[str, int],
) -> List[Tuple[int, int, float, int]]:
    """Resolve ``RESET_NODES`` to sorted ``(node, axis, wall_coord, facing)`` rows.

    Each entry in the sparse ``field_data.RESET_NODES`` table is keyed by a node
    NAME mapped to its integer index via ``index_of`` (the same NODES-order
    lookup the rest of the emitter uses, mirroring :func:`_resolve_pivot_rows`).
    The ``axis`` string is encoded to a ``uint8_t`` via
    :data:`_RESET_AXIS_TO_UINT8` (``"x" -> 0``, ``"y" -> 1`` -- the load-bearing
    cross-task contract T15 decodes). ``facing`` is rounded to the nearest
    integer and reduced mod 360 for the emitted ``int16_t`` field, so a value
    that rounds up to 360 wraps to 0 and stays inside the ``[0, 360)`` contract.

    Rows are sorted by node index so the emitted ``RESET_NODES`` table is
    deterministic for golden byte-compares and supports the firmware's ordered
    linear scan.

    An entry naming a node absent from ``index_of`` -- or an unknown ``axis``
    string -- is skipped defensively (the validator guarantees both, so this
    only fires for a hand-built caller that bypasses validation, and skipping
    keeps the emitter from raising :class:`KeyError`). The truth count
    ``N_RESET_NODES`` is derived from the returned rows, so a skipped entry
    drops out of the count too.

    :param reset_nodes: ``field_data.RESET_NODES`` -- a dict mapping node name
        to a ``{"axis", "wall_coord", "facing"}`` spec.
    :param index_of: node-name -> NODES-order index lookup.
    :return: rows ``(node_index, axis_uint8, wall_coord, facing_int)`` sorted
        ascending by ``node_index``.
    """
    rows: List[Tuple[int, int, float, int]] = []
    for name, spec in reset_nodes.items():
        if name not in index_of:
            continue
        axis = _RESET_AXIS_TO_UINT8.get(spec["axis"])
        if axis is None:
            continue
        rows.append(
            (
                index_of[name],
                axis,
                float(spec["wall_coord"]),
                # Round to the nearest integer for the emitted int16_t, then
                # reduce mod 360 so a value rounding UP to 360 (e.g. 359.7 ->
                # 360) wraps to 0 and stays inside the `facing; // degrees
                # [0, 360)` contract on Routes::ResetNode.
                int(round(float(spec["facing"]))) % 360,
            )
        )
    rows.sort(key=lambda row: row[0])
    return rows


def _emit_reset_nodes(
    reset_rows: Sequence[Tuple[int, int, float, int]],
) -> List[str]:
    """Emit the ``const ResetNode RESET_NODES[...]`` definition.

    ``reset_rows`` are the already-resolved, already-sorted rows from
    :func:`_resolve_reset_rows`. When empty, a single zeroed placeholder element
    is emitted (the array bound is ``max(1, count)``; a zero-length array bound
    is ill-formed in standard C++) while ``N_RESET_NODES`` in routes.h reports
    the truth count of ``0`` -- exactly the empty-``PIVOT_TURNS`` discipline.

    Field order MUST match the ``ResetNode`` struct declared in routes.h
    (``node, axis, wall_coord, facing``) and the ``FieldMap::getResetInfo``
    accessor that reads it: the brace-enclosed aggregate initialiser is
    positional.
    """
    array_len = max(1, len(reset_rows))
    out: List[str] = []
    out.append(
        _INDENT
        + "const ResetNode RESET_NODES["
        + str(array_len)
        + "] = {"
    )
    if reset_rows:
        out.append(_INDENT * 2 + "// sorted by node index; axis 0 = x, 1 = y")
        for node, axis, wall_coord, facing in reset_rows:
            out.append(
                _INDENT * 2
                + "{ "
                + str(node) + ", " + str(axis) + ", "
                + _format_float(wall_coord) + ", "
                + str(facing)
                + " },"
            )
    else:
        # Reset-free field: one zeroed placeholder so the array bound is >= 1.
        # N_RESET_NODES == 0 tells FieldMap::getResetInfo to scan nothing, so
        # this element is never read at runtime.
        out.append(
            _INDENT * 2
            + "// placeholder: N_RESET_NODES == 0, never read at runtime"
        )
        out.append(
            _INDENT * 2
            + "{ 0, 0, "
            + _format_float(0.0) + ", 0"
            + " },"
        )
    out.append(_INDENT + "};")
    return out


def _build_routes_cpp_lines(
    field_data,
    graph: nx.DiGraph,
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    node_names: List[str],
    index_of: Mapping[str, int],
    source_hash: str,
    pivot_rows: Sequence[Tuple[int, int, int, str, float]],
    reset_rows: Sequence[Tuple[int, int, float, int]],
    header_include: str,
) -> List[str]:
    lines: List[str] = []
    lines.extend(_routes_cpp_banner(source_hash))
    lines.append("")
    # The generated .cpp includes its OWN sibling header by basename so it
    # resolves same-directory regardless of where the pair is emitted:
    # production routes.cpp -> "routes.h", each generated fixture/toy .cpp ->
    # its own "routes_fixture.h" / "routes_toy.h". Derived from the output
    # header path's basename in emit_routes, never hardcoded.
    lines.append('#include "' + header_include + '"')
    lines.append("")
    lines.append("namespace Routes {")

    lines.extend(_emit_pipeline_config(field_data))
    lines.append("")
    lines.extend(_emit_nodes_array(field_data, node_names))
    lines.append("")
    lines.extend(_emit_next_hop_table(next_hop_table, node_names, index_of))
    lines.append("")
    lines.extend(_emit_edges_table(graph, node_names))
    lines.append("")
    lines.extend(_emit_pivot_turns(pivot_rows))
    lines.append("")
    lines.extend(_emit_reset_nodes(reset_rows))
    lines.append("}  // namespace Routes")
    return lines


def emit_routes(
    field_data,
    graph: nx.DiGraph,
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    *,
    pivot_records: Optional[Sequence["PivotTurnRecord"]] = None,
    header_path: Path = DEFAULT_HEADER_PATH,
    source_path: Path = DEFAULT_SOURCE_PATH,
    field_data_source_path: Path = FIELD_DATA_PATH,
    extra_source_paths: Sequence[Path] = (),
) -> Tuple[Path, Path]:
    """Emit ``routes.h`` and ``routes.cpp`` to disk; return their paths.

    ``pivot_records`` is the list of
    :class:`~tools.path_planner.planner.PivotTurnRecord` produced by
    :func:`~tools.path_planner.planner.build_pivot_turns` and threaded in by
    the orchestrator (``build_routes``). Each record's node NAMES are mapped
    to NODES-order indices and emitted as a sparse, sorted ``PIVOT_TURNS[]``
    table plus an ``N_PIVOT_TURNS`` truth count. ``None`` (the default) is
    treated as an empty list -- ``N_PIVOT_TURNS == 0`` with a single zeroed
    placeholder element -- so callers and tests that predate the orchestrator
    wiring keep working unchanged.

    ``extra_source_paths`` are additional files folded into the banner
    ``Source SHA256:`` stamp alongside ``field_data_source_path`` (e.g. the
    JSON geometry that ``field_data.py`` loads). Defaults to empty so the
    toy-field goldens, which hash a single anchor file, stay byte-stable.
    """
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

    source_hash = compute_source_hash(field_data_source_path, *extra_source_paths)

    # Resolve pivot records to sorted (u, v, w, wheel, trim) index rows ONCE so
    # the count in routes.h and the table in routes.cpp are derived from the
    # same list (a record skipped for an unknown node drops out of both).
    pivot_rows = _resolve_pivot_rows(pivot_records or (), index_of)

    # Resolve the sparse RESET_NODES table the same way: read it strictly off
    # field_data (every fixture carries RESET_NODES = {}), resolve node-name
    # keys to NODES-order indices, and derive the count in routes.h and the
    # table in routes.cpp from the SAME sorted list.
    reset_rows = _resolve_reset_rows(field_data.RESET_NODES, index_of)

    header_lines = _build_routes_h_lines(
        field_data,
        node_names,
        source_hash,
        start_node_index,
        len(pivot_rows),
        len(reset_rows),
    )
    cpp_lines = _build_routes_cpp_lines(
        field_data,
        graph,
        next_hop_table,
        node_names,
        index_of,
        source_hash,
        pivot_rows,
        reset_rows,
        header_path.name,
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
    "FIELD_DATA_JSON_PATH",
    "NEXT_HOP_NONE_HEX",
    "compute_source_hash",
    "emit_routes",
]
