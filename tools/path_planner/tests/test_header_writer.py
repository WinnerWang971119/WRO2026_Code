"""Tests for tools.path_planner.header_writer (PR3 schema).

Drops the PR2 ``GoalApproach`` / ``BlindAction`` enum / blind-moves
fixture tests; PR3 emits only ``NODES``, ``NEXT_HOP``, ``EDGES``.

Golden snapshot tests assert the emitted ``routes.h`` / ``routes.cpp``
match the regenerated fixtures under
``tools/path_planner/tests/golden/``. Update the golden files by re-
running ``uv run python tools/path_planner/build_routes.py`` and copying
``WRO2026/routes.{h,cpp}`` into the golden directory.
"""

from __future__ import annotations

import copy
import hashlib
import warnings
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.graph_builder import build_graph
from tools.path_planner.header_writer import (
    DEFAULT_HEADER_PATH,
    DEFAULT_SOURCE_PATH,
    FIELD_DATA_PATH,
    NEXT_HOP_NONE_HEX,
    compute_source_hash,
    emit_routes,
)
from tools.path_planner.planner import compute_all_pairs_next_hop

_FIELD_ATTRS = (
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

_GOLDEN_DIR = Path(__file__).parent / "golden"
_GOLDEN_HEADER = _GOLDEN_DIR / "routes_toy.h"
_GOLDEN_SOURCE = _GOLDEN_DIR / "routes_toy.cpp"
_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIELD_DATA_SRC = _REPO_ROOT / FIELD_DATA_PATH


def _clone_field_data() -> SimpleNamespace:
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


@pytest.fixture
def emit_targets(tmp_path: Path) -> tuple[Path, Path]:
    return tmp_path / "routes.h", tmp_path / "routes.cpp"


def _emit_with_real_field(emit_targets: tuple[Path, Path]) -> tuple[Path, Path]:
    header_path, source_path = emit_targets
    graph = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(graph, real_field_data.COST_BLIND_MULTIPLIER)
    return emit_routes(
        real_field_data,
        graph,
        next_hop,
        header_path=header_path,
        source_path=source_path,
        field_data_source_path=_FIELD_DATA_SRC,
    )


def test_emit_writes_both_files(emit_targets):
    header_path, source_path = emit_targets
    returned_header, returned_source = _emit_with_real_field(emit_targets)
    assert returned_header == header_path
    assert returned_source == source_path
    assert header_path.exists() and header_path.stat().st_size > 0
    assert source_path.exists() and source_path.stat().st_size > 0


def test_routes_h_matches_golden(emit_targets):
    header_path, _ = emit_targets
    _emit_with_real_field(emit_targets)
    if not _GOLDEN_HEADER.exists():
        pytest.skip("golden routes_toy.h missing; regenerate it")
    assert header_path.read_bytes() == _GOLDEN_HEADER.read_bytes()


def test_routes_cpp_matches_golden(emit_targets):
    _, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    if not _GOLDEN_SOURCE.exists():
        pytest.skip("golden routes_toy.cpp missing; regenerate it")
    assert source_path.read_bytes() == _GOLDEN_SOURCE.read_bytes()


def test_emit_is_idempotent(emit_targets):
    header_path, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    h1 = hashlib.sha256(header_path.read_bytes()).hexdigest()
    c1 = hashlib.sha256(source_path.read_bytes()).hexdigest()
    _emit_with_real_field(emit_targets)
    h2 = hashlib.sha256(header_path.read_bytes()).hexdigest()
    c2 = hashlib.sha256(source_path.read_bytes()).hexdigest()
    assert h1 == h2
    assert c1 == c2


def test_source_sha256_matches_field_data(emit_targets):
    header_path, _ = emit_targets
    _emit_with_real_field(emit_targets)
    expected_hash = compute_source_hash(_FIELD_DATA_SRC)
    text = header_path.read_text(encoding="utf-8")
    assert expected_hash in text
    assert "Source SHA256: " + expected_hash in text


def test_unreachable_pair_emits_0xff(emit_targets):
    """Removing BLIND_EDGES makes G unreachable; NEXT_HOP must show 0xFF."""
    fd = _clone_field_data()
    fd.BLIND_EDGES = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        graph = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(graph, fd.COST_BLIND_MULTIPLIER)
    header_path, source_path = emit_targets
    emit_routes(
        fd, graph, next_hop,
        header_path=header_path, source_path=source_path,
        field_data_source_path=_FIELD_DATA_SRC,
    )
    cpp_text = source_path.read_text(encoding="utf-8")
    assert NEXT_HOP_NONE_HEX in cpp_text
    assert cpp_text.count(NEXT_HOP_NONE_HEX) >= 2


def test_node_indexing_alphabetical(emit_targets):
    _, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")
    sorted_names = sorted(real_field_data.NODES.keys())
    assert sorted_names[0] == "A"
    last = sorted_names[-1]

    nodes_header = "const NodeInfo NODES[N_NODES] = {"
    body_after_header = cpp_text.split(nodes_header, 1)[1]
    first_row_end = body_after_header.index("\n", body_after_header.index('"A"'))
    first_row = body_after_header[:first_row_end]
    assert '"A"' in first_row
    assert "// index 0" in first_row

    expected_last_index = len(sorted_names) - 1
    expected_last_marker = '"' + last + '"'
    last_row_marker = "// index " + str(expected_last_index)
    last_row_idx = cpp_text.index(last_row_marker)
    line_start = cpp_text.rfind("\n", 0, last_row_idx) + 1
    last_line = cpp_text[line_start:last_row_idx]
    assert expected_last_marker in last_line


def test_edge_types_use_enum(emit_targets):
    """Edges render as ``EdgeType::*`` literals, not integers."""
    _, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")
    assert "EdgeType::LINE_E" in cpp_text
    assert "EdgeType::BLIND_E" in cpp_text


def test_compute_source_hash_is_full_sha256():
    digest = compute_source_hash(_FIELD_DATA_SRC)
    # compute_source_hash LF-normalizes before hashing so the digest is stable
    # whether field_data.py was checked out CRLF (Windows autocrlf) or LF.
    raw = _FIELD_DATA_SRC.read_bytes()
    expected = hashlib.sha256(raw.replace(b"\r\n", b"\n")).hexdigest()
    assert digest == expected
    assert len(digest) == 64
    assert all(c in "0123456789abcdef" for c in digest)


def test_default_paths_are_under_repo():
    assert DEFAULT_HEADER_PATH == Path("WRO2026/routes.h")
    assert DEFAULT_SOURCE_PATH == Path("WRO2026/routes.cpp")
    assert FIELD_DATA_PATH == Path("tools/path_planner/field_data.py")
