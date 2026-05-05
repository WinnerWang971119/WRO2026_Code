"""Tests for tools.path_planner.header_writer.

Covers TC11 (golden ``routes.h`` snapshot), TC12 (idempotent build),
and a battery of structural guardrails:

* Both files are written to the requested paths.
* Both files match the byte-identical golden snapshots in
  ``tools/path_planner/tests/golden/``.
* Re-emitting yields identical bytes (AC2 idempotency).
* The SHA256 of ``field_data.py`` appears verbatim in the ``routes.h``
  banner (lets reviewers verify the source-of-truth lock).
* Disconnecting the goal yields ``0xFF`` cells in ``NEXT_HOP``.
* Node ordering is alphabetical (`A` at index 0, `_proj_G` last).
* Blind-move actions render as ``BlindAction::*`` enum names rather
  than raw integers.
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

# Mirrors the list used in sibling tests so cloning stays in lock-step
# if the field-data surface ever grows.
_FIELD_ATTRS = (
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

# Golden snapshot files are co-located with this test module.
_GOLDEN_DIR = Path(__file__).parent / "golden"
_GOLDEN_HEADER = _GOLDEN_DIR / "routes_toy.h"
_GOLDEN_SOURCE = _GOLDEN_DIR / "routes_toy.cpp"

# The real field_data.py path the writer should hash. We compute the hash
# once in a fixture so the subsequent assertions stay cheap.
_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIELD_DATA_SRC = _REPO_ROOT / FIELD_DATA_PATH


def _clone_field_data() -> SimpleNamespace:
    """Return a deep-copied SimpleNamespace mirror of the real field_data.

    Mutating the returned namespace (or any nested container) does not
    affect the live ``tools.path_planner.field_data`` module.
    """
    snapshot = {
        attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS
    }
    return SimpleNamespace(**snapshot)


@pytest.fixture
def emit_targets(tmp_path: Path) -> tuple[Path, Path]:
    """Per-test header / source target paths under ``tmp_path``.

    Using a function-scoped ``tmp_path`` keeps tests hermetic; re-emit
    tests can call :func:`emit_routes` multiple times to the same paths
    without polluting the working tree.
    """
    return tmp_path / "routes.h", tmp_path / "routes.cpp"


def _emit_with_real_field(emit_targets: tuple[Path, Path]) -> tuple[Path, Path]:
    """Emit ``routes.{h,cpp}`` for the unmodified toy field data.

    :param emit_targets: ``(header_path, source_path)`` from the
        :func:`emit_targets` fixture.
    :return: same ``(header_path, source_path)`` tuple, after writing.
    """
    header_path, source_path = emit_targets
    graph = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(
        graph, real_field_data.COST_BLIND_MULTIPLIER
    )
    return emit_routes(
        real_field_data,
        graph,
        next_hop,
        header_path=header_path,
        source_path=source_path,
        field_data_source_path=_FIELD_DATA_SRC,
    )


# -- basic file-output guarantees --------------------------------------------


def test_emit_writes_both_files(emit_targets: tuple[Path, Path]) -> None:
    """Both ``routes.h`` and ``routes.cpp`` exist on disk after emit.

    ``emit_routes`` returns the two paths; we assert both ends of the
    contract: the returned paths AND the on-disk presence.
    """
    header_path, source_path = emit_targets
    returned_header, returned_source = _emit_with_real_field(emit_targets)
    assert returned_header == header_path
    assert returned_source == source_path
    assert header_path.exists() and header_path.stat().st_size > 0
    assert source_path.exists() and source_path.stat().st_size > 0


# -- TC11: golden snapshot for routes.h --------------------------------------


def test_routes_h_matches_golden(emit_targets: tuple[Path, Path]) -> None:
    """TC11: emitted ``routes.h`` is byte-identical to the golden snapshot.

    Locking the byte sequence catches accidental schema drift (whitespace
    changes, comment edits, reordered fields) at PR-review time. Update
    the golden when the schema legitimately changes; never silently.
    """
    header_path, _ = emit_targets
    _emit_with_real_field(emit_targets)
    assert header_path.read_bytes() == _GOLDEN_HEADER.read_bytes()


def test_routes_cpp_matches_golden(emit_targets: tuple[Path, Path]) -> None:
    """Emitted ``routes.cpp`` is byte-identical to the golden snapshot."""
    _, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    assert source_path.read_bytes() == _GOLDEN_SOURCE.read_bytes()


# -- TC12: idempotency -------------------------------------------------------


def test_emit_is_idempotent(emit_targets: tuple[Path, Path]) -> None:
    """TC12: re-emitting yields byte-identical files (AC2).

    SHA256 the bytes of the first emit, run the writer again to the same
    paths, then re-hash. Any non-determinism (e.g. hash-randomised dict
    iteration, datetime stamps, locale-dependent float formatting) would
    surface as a digest mismatch.
    """
    header_path, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    h1 = hashlib.sha256(header_path.read_bytes()).hexdigest()
    c1 = hashlib.sha256(source_path.read_bytes()).hexdigest()

    _emit_with_real_field(emit_targets)
    h2 = hashlib.sha256(header_path.read_bytes()).hexdigest()
    c2 = hashlib.sha256(source_path.read_bytes()).hexdigest()

    assert h1 == h2, "routes.h SHA256 changed across re-emits"
    assert c1 == c2, "routes.cpp SHA256 changed across re-emits"


# -- source-hash banner ------------------------------------------------------


def test_source_sha256_matches_field_data(emit_targets: tuple[Path, Path]) -> None:
    """The SHA256 hex of ``field_data.py`` appears verbatim in routes.h.

    This is the reviewer's anchor: hand-editing either file (without
    regenerating) leaves the banner hash mismatched against
    ``compute_source_hash(field_data.py)``.
    """
    header_path, _ = emit_targets
    _emit_with_real_field(emit_targets)

    expected_hash = compute_source_hash(_FIELD_DATA_SRC)
    text = header_path.read_text(encoding="utf-8")
    assert expected_hash in text
    # Also assert the banner explicitly labels it.
    assert "Source SHA256: " + expected_hash in text


# -- TC14-style: unreachable goal yields 0xFF in NEXT_HOP --------------------


def test_unreachable_pair_emits_0xff(emit_targets: tuple[Path, Path]) -> None:
    """A wall that disconnects ``G`` produces ``0xFF`` next-hop cells.

    Mirrors TC14 from test_graph: insert a wall along the
    ``_proj_G -> G`` blind linestring, suppress the resulting UserWarning
    so test output stays clean, then build, plan, and emit. Read the
    emitted ``routes.cpp`` and confirm the sentinel appears.
    """
    fd = _clone_field_data()
    # Box centred on the blind linestring's y=30 row, well within the
    # robot-radius-inflated band.
    fd.OBSTACLES = [(70.0, 25.0, 80.0, 35.0, "wall")]

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        graph = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(graph, fd.COST_BLIND_MULTIPLIER)

    header_path, source_path = emit_targets
    emit_routes(
        fd,
        graph,
        next_hop,
        header_path=header_path,
        source_path=source_path,
        field_data_source_path=_FIELD_DATA_SRC,
    )

    cpp_text = source_path.read_text(encoding="utf-8")
    # ``G`` is unreachable from every other node, so 0xFF must occur
    # multiple times in the emitted NEXT_HOP table.
    assert NEXT_HOP_NONE_HEX in cpp_text
    assert cpp_text.count(NEXT_HOP_NONE_HEX) >= 2


# -- node indexing -----------------------------------------------------------


def test_node_indexing_alphabetical(emit_targets: tuple[Path, Path]) -> None:
    """``NODES`` is emitted in alphabetical-by-name order.

    Spot-check that the index-0 row references ``A`` and the final index
    references the lexicographically last node. ASCII puts uppercase
    letters before underscore, so ``_proj_G`` follows ``G``.
    """
    _, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")

    sorted_names = sorted(real_field_data.NODES.keys())
    assert sorted_names[0] == "A"
    last = sorted_names[-1]

    # The first NODES row must mention "A" before any other node name.
    nodes_header = "const NodeInfo NODES[N_NODES] = {"
    body_after_header = cpp_text.split(nodes_header, 1)[1]
    first_row_end = body_after_header.index("\n", body_after_header.index('"A"'))
    first_row = body_after_header[:first_row_end]
    assert '"A"' in first_row
    assert "// index 0" in first_row

    # The last NODES row carries the lexicographically last node name.
    expected_last_index = len(sorted_names) - 1
    expected_last_marker = '"' + last + '"'
    last_row_marker = "// index " + str(expected_last_index)
    last_row_idx = cpp_text.index(last_row_marker)
    # Walk back to the start of that row and confirm the right name.
    line_start = cpp_text.rfind("\n", 0, last_row_idx) + 1
    last_line = cpp_text[line_start:last_row_idx]
    assert expected_last_marker in last_line


# -- blind moves use enum names ---------------------------------------------


def test_blind_move_actions_use_enum(emit_targets: tuple[Path, Path]) -> None:
    """Blind moves render as ``BlindAction::*`` literals, not integers.

    A regression here would mean PR 2's runtime is forced to compare
    against magic numbers, which is fragile. Confirm both authored
    actions in the toy field appear by enum name.
    """
    _, source_path = emit_targets
    _emit_with_real_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")

    assert "BlindAction::TURN_RELATIVE" in cpp_text
    assert "BlindAction::FORWARD" in cpp_text


# -- compute_source_hash sanity ---------------------------------------------


def test_compute_source_hash_is_full_sha256() -> None:
    """``compute_source_hash`` returns the full 64-char hex digest.

    Mirrors :func:`hashlib.sha256(...).hexdigest()` exactly, so callers
    that want to truncate can do so deliberately.
    """
    digest = compute_source_hash(_FIELD_DATA_SRC)
    expected = hashlib.sha256(_FIELD_DATA_SRC.read_bytes()).hexdigest()
    assert digest == expected
    assert len(digest) == 64
    assert all(c in "0123456789abcdef" for c in digest)


# -- defaults --------------------------------------------------------------


def test_default_paths_are_under_repo() -> None:
    """The exported defaults point at the canonical Arduino sketch dir.

    PR 2 expects ``WRO2026/routes.h`` and ``WRO2026/routes.cpp`` next to
    the ``.ino``. A regression that re-roots them would silently break
    the build.
    """
    assert DEFAULT_HEADER_PATH == Path("WRO2026/routes.h")
    assert DEFAULT_SOURCE_PATH == Path("WRO2026/routes.cpp")
    assert FIELD_DATA_PATH == Path("tools/path_planner/field_data.py")
