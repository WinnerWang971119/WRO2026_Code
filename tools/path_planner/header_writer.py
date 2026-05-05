"""Header writer: emit ``WRO2026/routes.h`` and ``WRO2026/routes.cpp``.

This module is the final stage of the offline path-planner pipeline. It
serialises the validated graph and next-hop table into a pair of C++
files PR 2's runtime ``PathTracker`` consumes:

* :data:`DEFAULT_HEADER_PATH` (``WRO2026/routes.h``) -- declarations
  (``constexpr`` sizes, enums, struct definitions, ``extern`` arrays).
* :data:`DEFAULT_SOURCE_PATH` (``WRO2026/routes.cpp``) -- definitions
  (``NODES``, ``NEXT_HOP``, ``EDGES``, ``GOAL_APPROACH``, plus per-goal
  blind-move arrays in an anonymous namespace).

Both files are AUTO-GENERATED. Reviewers verify the emitted bytes match
their source by checking the SHA256 hash embedded in the ``routes.h``
banner against the hash of ``tools/path_planner/field_data.py``.

Determinism contract (AC2):

* Node ordering: alphabetical by name. Index 0 is the alphabetically
  first node; the C++ ``START_NODE`` constant is the index of
  ``field_data.START_NODE`` under this ordering.
* All dict iteration uses ``sorted(...)``.
* All floats are formatted with ``{:.6f}f`` -- 6 decimal places of
  precision (cm-scale fields) plus the C++ ``f`` literal suffix to
  avoid implicit ``double`` warnings under ``-Wfloat-conversion``.
* Line endings are LF only, written via ``newline=''`` so Windows hosts
  do not silently insert CRLF. Run on Windows + Linux yields byte-
  identical output.
* No ``datetime.now()`` in the output -- the embedded SHA256 of
  ``field_data.py`` is the only "version stamp" and it is purely a
  function of the input bytes.

Heading and edge-type conventions are inherited from
:mod:`tools.path_planner.graph_builder` and :mod:`tools.path_planner.compiler`;
this module only translates them into the C++ enum names declared in
``routes.h``.
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Tuple

import networkx as nx

from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    EDGE_PROJECTION,
)

# -- public constants --------------------------------------------------------

#: Default output location for the C++ declarations file.
DEFAULT_HEADER_PATH = Path("WRO2026/routes.h")

#: Default output location for the C++ definitions file.
DEFAULT_SOURCE_PATH = Path("WRO2026/routes.cpp")

#: Default location of the field-data Python source whose bytes are
#: hashed and embedded in the routes.h banner. The runtime never reads
#: this file -- it exists purely so reviewers can verify the emitted
#: tables match a known input.
FIELD_DATA_PATH = Path("tools/path_planner/field_data.py")

#: Sentinel emitted in the ``NEXT_HOP`` table when a target node is
#: unreachable from a source node. Mirrors the ``NEXT_HOP_NONE``
#: ``constexpr`` declared in ``routes.h``.
NEXT_HOP_NONE_HEX = "0xFF"

#: The regen command embedded as a comment in the routes.h banner. PR 2
#: code reviewers re-run this command and diff the output to confirm
#: ``routes.{h,cpp}`` were not hand-edited.
_REGEN_COMMAND = "uv run python tools/path_planner/build_routes.py"

#: Indent used inside the ``namespace Routes { ... }`` block (4 spaces).
_INDENT = "    "

#: Pattern used by :func:`_sanitise_identifier` to strip non-identifier
#: characters from goal names before splicing them into the C++ symbol
#: ``_<sanitised>_blind`` (e.g. a goal named ``"red-1"`` would become
#: ``red_1_blind`` then prefixed with ``_`` -> ``_red_1_blind``).
_NON_IDENT_RE = re.compile(r"[^A-Za-z0-9_]")


# -- C++ EdgeType / NodeType / BlindAction mapping ---------------------------

# Maps the canonical edge-type string from graph_builder onto the
# enumerator name in routes.h. The strings here are the source of truth;
# graph_builder owns ``EDGE_LINE``/``EDGE_PROJECTION``/``EDGE_BLIND``.
_EDGE_TYPE_TO_ENUM: Dict[str, str] = {
    EDGE_LINE: "EdgeType::LINE_E",
    EDGE_PROJECTION: "EdgeType::PROJECTION_E",
    EDGE_BLIND: "EdgeType::BLIND_E",
}

# Maps the validated node-type string onto the C++ ``NodeType`` enumerator.
_NODE_TYPE_TO_ENUM: Dict[str, str] = {
    "line": "NodeType::LINE",
    "projection": "NodeType::PROJECTION",
    "goal": "NodeType::GOAL",
}

# Maps a blind-move action string onto the C++ ``BlindAction`` enumerator.
# The plan's field_data only authors ``turn_relative`` and ``forward``;
# ``turn_to_heading`` is supported here defensively because the C++ enum
# in routes.h carries it (PR 2's PathTracker may emit it dynamically).
_BLIND_ACTION_TO_ENUM: Dict[str, str] = {
    "turn_relative": "BlindAction::TURN_RELATIVE",
    "forward": "BlindAction::FORWARD",
    "turn_to_heading": "BlindAction::TURN_TO_HEADING",
}


# -- internal helpers --------------------------------------------------------


def _format_float(value: float) -> str:
    """Format a float as a C++ ``float`` literal (six decimals + ``f``).

    Six decimals is enough for cm-scale fields and degree headings while
    remaining short enough to keep the emitted tables readable. The
    trailing ``f`` suffix tells the compiler the literal is a ``float``
    rather than a ``double``, which silences ``-Wfloat-conversion``
    warnings on the runtime build.

    :param value: any finite float-convertible value.
    :return: a string of the form ``"30.000000f"``.
    """
    return "{:.6f}f".format(float(value))


def _sanitise_identifier(name: str) -> str:
    """Convert a goal-node name into a valid C++ identifier core.

    Replaces every non-``[A-Za-z0-9_]`` character with ``_``. The caller
    is responsible for prefixing with a leading underscore (so the
    eventual symbol is ``_<sanitised>_blind`` and never starts with a
    digit, even when a goal name does).

    :param name: the goal-node name from ``field_data.GOAL_APPROACH``.
    :return: the sanitised stem.
    """
    return _NON_IDENT_RE.sub("_", name)


def _blind_array_symbol(goal_name: str) -> str:
    """Return the unique anonymous-namespace symbol for a goal's blind moves.

    Format: ``_<sanitised goal>_blind``. The leading underscore is added
    only when ``sanitised goal`` does not already start with one, so the
    symbol never collapses to ``__foo`` (reserved in C++).

    Examples:
      - ``"G"``       -> ``"_G_blind"``
      - ``"_proj_G"`` -> ``"_proj_G_blind"``
      - ``"red-1"``   -> ``"_red_1_blind"``

    :param goal_name: goal node name from ``field_data.GOAL_APPROACH``.
    :return: a unique C++ identifier safe for a file-scoped array.
    """
    stem = _sanitise_identifier(goal_name)
    if not stem.startswith("_"):
        stem = "_" + stem
    return stem + "_blind"


def _build_index_lookup(node_names: List[str]) -> Dict[str, int]:
    """Return a ``{node_name: zero_based_index}`` lookup.

    :param node_names: canonical, alphabetically-sorted node-name list.
    :return: dict mapping each name to its index in the input list.
    """
    return {name: i for i, name in enumerate(node_names)}


def _next_hop_cell(
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    from_name: str,
    to_name: str,
    index_of: Mapping[str, int],
) -> str:
    """Render one ``NEXT_HOP[from][to]`` cell as a C++ literal.

    Per the planner's contract:
      - ``next_hop_table[s][s] == s`` -> emit ``s``'s own index.
      - ``next_hop_table[s][t] is None`` -> emit ``0xFF`` (unreachable).
      - Otherwise -> emit the index of the next-hop node.

    :param next_hop_table: nested dict from
        :func:`tools.path_planner.planner.compute_all_pairs_next_hop`.
    :param from_name: outer-key node name.
    :param to_name: inner-key node name.
    :param index_of: ``{name: index}`` lookup for canonical ordering.
    :return: a C++ integer literal as a string (no trailing comma).
    """
    cell = next_hop_table[from_name][to_name]
    if cell is None:
        return NEXT_HOP_NONE_HEX
    # The self-loop case (cell == from_name when from_name == to_name)
    # collapses naturally into index_of[cell].
    return str(index_of[cell])


def _edge_cell(
    graph: nx.DiGraph,
    from_name: str,
    to_name: str,
) -> str:
    """Render one ``EDGES[from][to]`` cell as a C++ struct initialiser.

    A directed edge is looked up via ``graph.edges[from, to]``; if absent,
    the cell is the ``EdgeType::NONE`` placeholder with zero distance and
    heading. Otherwise the edge's ``type``, ``distance_cm`` and
    ``heading_deg`` attributes are translated into the C++ schema.

    :param graph: the directed graph from
        :func:`tools.path_planner.graph_builder.build_graph`. Already
        C-space inflated so missing edges signal "unreachable".
    :param from_name: source node name.
    :param to_name: target node name.
    :return: a C++ struct initialiser like
        ``"{ EdgeType::LINE_E, 60.000000f, 0.000000f }"`` (no comma).
    """
    if not graph.has_edge(from_name, to_name):
        return "{ EdgeType::NONE, " + _format_float(0.0) + ", " + _format_float(0.0) + " }"
    edge = graph.edges[from_name, to_name]
    enum_name = _EDGE_TYPE_TO_ENUM[edge["type"]]
    distance = _format_float(edge["distance_cm"])
    heading = _format_float(edge["heading_deg"])
    return "{ " + enum_name + ", " + distance + ", " + heading + " }"


def _node_row(
    name: str,
    field_data,
    index: int,
    name_column_width: int,
) -> str:
    """Format one row of the ``NODES`` array.

    Layout: ``{ <x>, <y>, <NodeType>, "<name>" },     // index <i>``.
    The node-name column is right-padded to ``name_column_width`` so
    the ``// index`` comments line up vertically -- still deterministic
    because the width is computed from ``field_data`` once.

    :param name: node name (canonical-order key).
    :param field_data: validated field-data namespace.
    :param index: zero-based index assigned to ``name``.
    :param name_column_width: width to pad the quoted name field to.
    :return: one full source line, without the leading indent.
    """
    x, y = field_data.NODES[name]
    node_type_enum = _NODE_TYPE_TO_ENUM[field_data.NODE_TYPES[name]]
    quoted = '"' + name + '"'
    padded = quoted + " " * max(0, name_column_width - len(quoted))
    return (
        "{ " + _format_float(x) + ", " + _format_float(y) + ", "
        + node_type_enum + ", " + padded + " },"
        + "     // index " + str(index)
    )


# -- public API: source-hash helper ------------------------------------------


def compute_source_hash(field_data_source_path: Path) -> str:
    """Return the SHA256 hex digest of ``field_data.py``'s raw bytes.

    The full 64-character lowercase hex digest is returned. Callers that
    only need a short visual identifier may truncate; the test suite
    asserts the full digest appears verbatim in the emitted ``routes.h``
    banner so any change to the field data trips the comparison.

    :param field_data_source_path: filesystem path to the field-data
        Python module. Read in binary mode -- BOMs / trailing whitespace
        are part of the hash, intentionally.
    :return: 64-character lowercase hex string.
    :raises FileNotFoundError: if ``field_data_source_path`` does not exist.
    :raises OSError: on any other I/O failure.
    """
    return hashlib.sha256(Path(field_data_source_path).read_bytes()).hexdigest()


# -- internal: assemble the routes.h banner ----------------------------------


def _routes_h_banner(source_hash: str) -> List[str]:
    """Banner comment lines for ``routes.h``.

    :param source_hash: full SHA256 hex of the field_data source.
    :return: list of source lines (no trailing newlines).
    """
    return [
        "// routes.h - AUTO-GENERATED by " + _REGEN_COMMAND,
        "// Source SHA256: " + source_hash,
        "// Do not hand-edit. Regenerate with: " + _REGEN_COMMAND,
    ]


def _routes_cpp_banner(source_hash: str) -> List[str]:
    """Banner comment lines for ``routes.cpp``.

    :param source_hash: full SHA256 hex of the field_data source.
    :return: list of source lines (no trailing newlines).
    """
    return [
        "// routes.cpp - AUTO-GENERATED by " + _REGEN_COMMAND,
        "// Source SHA256: " + source_hash,
    ]


# -- internal: routes.h body -------------------------------------------------


def _build_routes_h_lines(
    field_data,
    node_names: List[str],
    source_hash: str,
    start_node_index: int,
) -> List[str]:
    """Return the full ``routes.h`` content as a list of source lines.

    The schema matches ``docs/plans/2026-05-05-phase2-pr1-path-planner-python.md``
    plus the deviations documented at the top of this module
    (``BlindAction::TURN_TO_HEADING`` and ``NEXT_HOP_NONE`` are added).

    :param field_data: validated field-data namespace.
    :param node_names: canonical sorted list of node names.
    :param source_hash: full SHA256 hex of the field_data source.
    :param start_node_index: index of ``field_data.START_NODE`` in
        ``node_names``.
    :return: list of source lines (no trailing newlines).
    """
    n_nodes = len(node_names)
    n_goals = len(field_data.GOAL_APPROACH)
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
    lines.append(_INDENT + "constexpr uint8_t N_GOALS = " + str(n_goals) + ";")
    lines.append("")
    lines.append(
        _INDENT
        + "// Node category. PROJECTION nodes are virtual waypoints inserted"
    )
    lines.append(
        _INDENT
        + "// on a line segment by the path planner; the runtime stops by"
    )
    lines.append(_INDENT + "// encoder distance rather than line intersection.")
    lines.append(
        _INDENT
        + "enum class NodeType : uint8_t { LINE = 0, PROJECTION = 1, GOAL = 2 };"
    )
    lines.append("")
    lines.append(_INDENT + "// Edge motion class, dispatched by PR 2 runtime.")
    lines.append(
        _INDENT
        + "enum class EdgeType : uint8_t { NONE = 0, LINE_E = 1, "
        + "PROJECTION_E = 2, BLIND_E = 3 };"
    )
    lines.append("")
    lines.append(
        _INDENT + "// Note: TURN_TO_HEADING is added beyond the field_data"
    )
    lines.append(
        _INDENT + "// authoring vocabulary so PR 2's PathTracker can emit"
    )
    lines.append(
        _INDENT + "// an absolute heading correction at goal arrival without"
    )
    lines.append(_INDENT + "// extending this enum at runtime.")
    lines.append(
        _INDENT
        + "enum class BlindAction : uint8_t { TURN_RELATIVE = 0, "
        + "FORWARD = 1, TURN_TO_HEADING = 2 };"
    )
    lines.append("")
    lines.append(
        _INDENT
        + "struct NodeInfo { float x_cm; float y_cm; NodeType type; "
        + "const char* name; };"
    )
    lines.append(
        _INDENT
        + "struct EdgeInfo { EdgeType type; float distance_cm; float heading_deg; };"
    )
    lines.append(_INDENT + "struct BlindMove { BlindAction action; float value; };")
    lines.append(_INDENT + "struct GoalApproach {")
    lines.append(_INDENT * 2 + "uint8_t entry_node;")
    lines.append(_INDENT * 2 + "float   approach_heading_deg;")
    lines.append(_INDENT * 2 + "const BlindMove* blind_moves;")
    lines.append(_INDENT * 2 + "uint8_t blind_move_count;")
    lines.append(_INDENT + "};")
    lines.append("")
    lines.append(_INDENT + "extern const NodeInfo NODES[N_NODES];")
    lines.append(_INDENT + "extern const uint8_t  NEXT_HOP[N_NODES][N_NODES];")
    lines.append(_INDENT + "extern const EdgeInfo EDGES   [N_NODES][N_NODES];")
    lines.append(_INDENT + "extern const GoalApproach GOAL_APPROACH[N_GOALS];")
    lines.append("")
    lines.append(
        _INDENT
        + "// Sentinel value for unreachable next-hop entries. PR 2's"
    )
    lines.append(
        _INDENT
        + "// PathTracker treats this as 'no path; halt and report'."
    )
    lines.append(
        _INDENT
        + "constexpr uint8_t NEXT_HOP_NONE = "
        + NEXT_HOP_NONE_HEX
        + ";"
    )
    lines.append(
        _INDENT
        + "constexpr uint8_t START_NODE = "
        + str(start_node_index)
        + ";"
    )
    lines.append(
        _INDENT
        + "constexpr float   START_HEADING_DEG = "
        + start_heading
        + ";"
    )
    lines.append("}  // namespace Routes")
    lines.append("#endif  // ROUTES_H")
    return lines


# -- internal: routes.cpp body -----------------------------------------------


def _emit_nodes_array(field_data, node_names: List[str]) -> List[str]:
    """Emit the ``NODES`` array body lines (one source row per node).

    :param field_data: validated field-data namespace.
    :param node_names: canonical sorted list of node names.
    :return: source lines, including the array header and closing ``};``.
    """
    # Pad the quoted-name column so the ``// index N`` comments align.
    name_column_width = max(len('"' + n + '"') for n in node_names)
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
    """Emit the ``NEXT_HOP`` 2D array, one inner row per source line.

    Cells are pre-formatted as up to three characters (``0xFF`` is four;
    most indices are one or two), then padded to the row's max width so
    the matrix renders as a clean rectangle in the file.

    :param next_hop_table: planner's all-pairs next-hop dict.
    :param node_names: canonical sorted list of node names.
    :param index_of: ``{name: index}`` lookup.
    :return: source lines, including comment header and closing ``};``.
    """
    # Pre-format every cell so we can compute a uniform column width.
    cells: List[List[str]] = [
        [
            _next_hop_cell(next_hop_table, frm, to, index_of)
            for to in node_names
        ]
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


def _emit_edges_table(
    graph: nx.DiGraph,
    node_names: List[str],
) -> List[str]:
    """Emit the ``EDGES`` 2D array, one inner row per source line.

    Each cell renders as a struct initialiser (see :func:`_edge_cell`).
    Rows are not column-aligned because struct initialisers are bulky --
    we instead put one cell per line inside each row to keep diffs
    reviewable.

    :param graph: directed graph from
        :func:`tools.path_planner.graph_builder.build_graph`.
    :param node_names: canonical sorted list of node names.
    :return: source lines, including comment header and closing ``};``.
    """
    out: List[str] = []
    out.append(_INDENT + "const EdgeInfo EDGES[N_NODES][N_NODES] = {")
    out.append(
        _INDENT * 2
        + "// [from][to]; type=NONE if no direct edge"
    )
    for i, frm in enumerate(node_names):
        out.append(_INDENT * 2 + "{ // from " + frm + " (index " + str(i) + ")")
        for to in node_names:
            cell = _edge_cell(graph, frm, to)
            out.append(_INDENT * 3 + cell + ", // to " + to)
        out.append(_INDENT * 2 + "},")
    out.append(_INDENT + "};")
    return out


def _emit_blind_arrays_and_goal_approach(
    field_data,
    index_of: Mapping[str, int],
) -> Tuple[List[str], List[str]]:
    """Emit the per-goal blind-move arrays and the ``GOAL_APPROACH`` array.

    The blind arrays live in an anonymous namespace (file-scoped, no
    external linkage) so they cannot collide with other translation
    units. The ``GOAL_APPROACH`` array's pointers reference them.

    Iteration order over goals is alphabetical for determinism.

    :param field_data: validated field-data namespace.
    :param index_of: ``{name: index}`` lookup for the entry-node index.
    :return: ``(blind_namespace_lines, goal_approach_lines)``. The caller
        weaves them into the final ``routes.cpp`` in the right order.
    """
    sorted_goals = sorted(field_data.GOAL_APPROACH.keys())

    blind_lines: List[str] = []
    blind_lines.append(_INDENT + "namespace {")
    blind_lines.append(
        _INDENT * 2
        + "// Per-goal blind-move arrays (file-scoped, anonymous namespace)."
    )

    for goal in sorted_goals:
        spec = field_data.GOAL_APPROACH[goal]
        symbol = _blind_array_symbol(goal)
        moves = spec["blind_moves"]
        blind_lines.append(
            _INDENT * 2 + "const BlindMove " + symbol + "[] = {"
        )
        for action, value in moves:
            if action not in _BLIND_ACTION_TO_ENUM:
                raise ValueError(
                    "GOAL_APPROACH[" + repr(goal) + "].blind_moves contains "
                    "unsupported action " + repr(action) + "; expected one of "
                    + repr(sorted(_BLIND_ACTION_TO_ENUM.keys()))
                )
            enum_name = _BLIND_ACTION_TO_ENUM[action]
            blind_lines.append(
                _INDENT * 3
                + "{ " + enum_name + ", " + _format_float(value) + " },"
            )
        # Emit a trailing element so the empty-list case still produces
        # a well-formed initialiser. We never expect empty blind_moves
        # in practice (the validator would let it pass but it would
        # imply a no-op goal), so we fall back to "no extra elements"
        # rather than padding with a sentinel.
        blind_lines.append(_INDENT * 2 + "};")

    blind_lines.append(_INDENT + "}")

    approach_lines: List[str] = []
    approach_lines.append(
        _INDENT + "const GoalApproach GOAL_APPROACH[N_GOALS] = {"
    )
    for goal in sorted_goals:
        spec = field_data.GOAL_APPROACH[goal]
        entry = spec["entry"]
        if entry not in index_of:
            raise KeyError(
                "GOAL_APPROACH[" + repr(goal) + "].entry references "
                "unknown node " + repr(entry)
            )
        entry_index = index_of[entry]
        approach_heading = _format_float(spec["approach_heading_deg"])
        symbol = _blind_array_symbol(goal)
        count = len(spec["blind_moves"])
        approach_lines.append(
            _INDENT * 2
            + "{ /*entry_node=*/" + str(entry_index)
            + " /*=" + entry + "*/, "
            + "/*approach_heading_deg=*/" + approach_heading + ", "
            + "/*blind_moves=*/" + symbol + ", "
            + "/*blind_move_count=*/" + str(count) + " },"
            + " // goal " + goal
        )
    approach_lines.append(_INDENT + "};")

    return blind_lines, approach_lines


def _build_routes_cpp_lines(
    field_data,
    graph: nx.DiGraph,
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    node_names: List[str],
    index_of: Mapping[str, int],
    source_hash: str,
) -> List[str]:
    """Return the full ``routes.cpp`` content as a list of source lines.

    Sections (in order):
      1. Banner.
      2. ``#include "routes.h"``.
      3. ``namespace Routes {``.
      4. ``NODES`` array.
      5. ``NEXT_HOP`` array.
      6. ``EDGES`` array.
      7. Anonymous namespace with per-goal ``_<goal>_blind`` arrays.
      8. ``GOAL_APPROACH`` array.
      9. Closing ``}``.

    :param field_data: validated field-data namespace.
    :param graph: directed graph from build_graph.
    :param next_hop_table: planner's all-pairs next-hop dict.
    :param node_names: canonical sorted list of node names.
    :param index_of: ``{name: index}`` lookup.
    :param source_hash: full SHA256 hex of the field_data source.
    :return: list of source lines (no trailing newlines).
    """
    lines: List[str] = []
    lines.extend(_routes_cpp_banner(source_hash))
    lines.append("")
    lines.append('#include "routes.h"')
    lines.append("")
    lines.append("namespace Routes {")

    nodes_section = _emit_nodes_array(field_data, node_names)
    next_hop_section = _emit_next_hop_table(next_hop_table, node_names, index_of)
    edges_section = _emit_edges_table(graph, node_names)
    blind_section, approach_section = _emit_blind_arrays_and_goal_approach(
        field_data, index_of
    )

    lines.extend(nodes_section)
    lines.append("")
    lines.extend(next_hop_section)
    lines.append("")
    lines.extend(edges_section)
    lines.append("")
    lines.extend(blind_section)
    lines.append("")
    lines.extend(approach_section)
    lines.append("}  // namespace Routes")
    return lines


# -- public API: emit_routes -------------------------------------------------


def emit_routes(
    field_data,
    graph: nx.DiGraph,
    next_hop_table: Mapping[str, Mapping[str, Optional[str]]],
    *,
    header_path: Path = DEFAULT_HEADER_PATH,
    source_path: Path = DEFAULT_SOURCE_PATH,
    field_data_source_path: Path = FIELD_DATA_PATH,
) -> Tuple[Path, Path]:
    """Emit ``routes.h`` and ``routes.cpp`` to disk; return their paths.

    The output is **deterministic**: running this function twice with
    identical inputs produces byte-identical files (AC2). All dict
    iteration is sorted, all floats are formatted with a fixed
    precision-and-suffix template, and line endings are forced to LF
    via ``newline=''`` so Windows hosts cannot silently inject CRLF.

    Node ordering is **alphabetical by name**. Index 0 is the
    alphabetically first node. The C++ ``START_NODE`` constant is the
    index of ``field_data.START_NODE`` under this ordering.

    Note: the C++ ``GOAL_APPROACH[].blind_moves`` array stores the
    field_data-authored blind moves verbatim. It does **not** store the
    compiler's per-path output (which depends on the path walked). The
    runtime PathTracker (PR 2) decides at goal arrival whether to
    prepend a ``turn_to_heading`` correction; this module only
    serialises the static authored sequence.

    :param field_data: validated field-data module / namespace exposing
        ``NODES``, ``NODE_TYPES``, ``GOAL_APPROACH``, ``START_NODE``,
        ``START_HEADING_DEG``.
    :param graph: directed graph as produced by
        :func:`tools.path_planner.graph_builder.build_graph`.
    :param next_hop_table: nested dict from
        :func:`tools.path_planner.planner.compute_all_pairs_next_hop`.
    :param header_path: keyword-only output path for the declarations
        file. Defaults to :data:`DEFAULT_HEADER_PATH`. Parent directory
        is created if missing.
    :param source_path: keyword-only output path for the definitions
        file. Defaults to :data:`DEFAULT_SOURCE_PATH`. Parent directory
        is created if missing.
    :param field_data_source_path: keyword-only path to the field_data
        Python source whose bytes are hashed and embedded in the
        ``routes.h`` banner. Defaults to :data:`FIELD_DATA_PATH`.
    :return: ``(header_path, source_path)`` -- the two paths that were
        written, as :class:`pathlib.Path` instances.
    :raises FileNotFoundError: if ``field_data_source_path`` is missing.
    :raises KeyError: if a goal's ``entry`` references an unknown node.
    :raises ValueError: if a blind-move action is unsupported.
    """
    header_path = Path(header_path)
    source_path = Path(source_path)
    field_data_source_path = Path(field_data_source_path)

    # Canonical alphabetical ordering -- the *only* place node ordering
    # is decided in this pipeline. Every downstream table is keyed off
    # this list.
    node_names: List[str] = sorted(field_data.NODES.keys())
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

    # newline='' suppresses the host's universal-newline translation so
    # the explicit '\n' separators below survive verbatim. This is the
    # core of the AC2 cross-platform-byte-identical guarantee.
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
