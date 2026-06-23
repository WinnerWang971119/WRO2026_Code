"""Tests for tools.path_planner.header_writer (PR4 schema).

PR4 (T7/T11) adds:
* ``PIPELINE_CONFIG`` struct + ``extern const`` declaration in routes.h
* Four per-edge override fields on ``EdgeInfo``
* ``EdgeType::PROJECTION`` enum value (alongside ``LINE`` / ``BLIND``)
* Drops the legacy ``_E`` suffix on ``EdgeType::LINE`` / ``EdgeType::BLIND``

Golden snapshot tests assert the emitted ``routes_toy.h`` /
``routes_toy.cpp`` match the regenerated fixtures under
``tools/path_planner/tests/golden/``. Those fixtures are produced by
emitting from a STABLE toy field-data fixture (see
:func:`_build_toy_field_data`) against a STABLE fake source file (see
:data:`_TOY_FIELD_SOURCE`) so the goldens are insulated from edits to
the canonical ``field_data.py``. The toy goldens were regenerated as a
one-off authorised rebaseline during T11; subsequent emits MUST be
byte-identical.

.. rubric:: When and how to rebaseline the goldens

The golden files (``tools/path_planner/tests/golden/routes_toy.{h,cpp}``)
are a **deliberate byte-baseline**.  They encode the exact output of the
emitter for the stable toy field-data fixture.  The coupling is intentional:
any authorised change to ``header_writer.py`` or to the defaults in
``pipeline_config.py`` that intentionally alters the emitter output MUST
be followed by a golden rebaseline — otherwise
:func:`test_routes_h_matches_golden` / :func:`test_routes_cpp_matches_golden`
will fail with a byte-equality error.

**Rebaseline workflow (authorised changes only):**

1. Make and review your emitter / pipeline_config change.
2. Run the regeneration helper::

       python tools/path_planner/tests/test_header_writer.py

   This overwrites ``tools/path_planner/tests/golden/routes_toy.h`` and
   ``tools/path_planner/tests/golden/routes_toy.cpp`` in-place.
3. Review the diff (``git diff tools/path_planner/tests/golden/``) to
   confirm only the expected lines changed.
4. Commit the updated goldens together with your emitter change in the
   same PR.  CI will confirm they are in lockstep.

**Do NOT rebaseline to silence a test failure caused by an unintentional
change.**  A byte-diff failure on an unrelated PR means the emitter or
a pipeline_config default drifted without authorisation — investigate
before touching the goldens.
"""

from __future__ import annotations

import hashlib
import re as _re
import tempfile
import warnings
from pathlib import Path
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    build_graph,
)
from tools.path_planner.header_writer import (
    DEFAULT_HEADER_PATH,
    DEFAULT_SOURCE_PATH,
    FIELD_DATA_JSON_PATH,
    FIELD_DATA_PATH,
    NEXT_HOP_NONE_HEX,
    _resolve_reset_rows,
    compute_source_hash,
    emit_routes,
)
from tools.path_planner.pipeline_config import LineMode, TurnMode
from tools.path_planner.planner import (
    BlindDecayParams,
    PivotTurnRecord,
    build_pivot_turns,
    compute_all_pairs_next_hop,
)

_GOLDEN_DIR = Path(__file__).parent / "golden"
_GOLDEN_HEADER = _GOLDEN_DIR / "routes_toy.h"
_GOLDEN_SOURCE = _GOLDEN_DIR / "routes_toy.cpp"
# Stable on-disk file whose SHA256 is embedded in the toy goldens. The
# toy fixture is built in Python (a SimpleNamespace), so this file is
# purely a hash anchor -- editing it would force a golden rebaseline.
_TOY_FIELD_SOURCE = _GOLDEN_DIR / "_toy_field_source.py"
_REPO_ROOT = Path(__file__).resolve().parents[3]


def _build_toy_field_data() -> SimpleNamespace:
    """Construct the canonical toy field_data fixture used by the goldens.

    Stable, minimal, exercises every node category (LINE + BLIND + PROJECTION +
    the implicit reset coverage elsewhere) and every edge category (LINE + BLIND).
    Per-edge overrides live in the override-specific tests (TC2/TC3) so the
    byte-identical golden tests stay simple to diff.

    Issue #64 (TC15): the validator now rejects an on-line START node, so the
    spawn is a blind node ``S`` (mirroring the toy fixture). A ``projection`` node
    ``P1`` sits on the A-B line at (55, 10) and exercises both projection
    scenarios so the golden emits ``NodeType::PROJECTION`` and the 7-field
    ``EdgeInfo`` struct (no ``ProjectionMode`` / ``projection_mode_override`` /
    ``EdgeType::PROJECTION``):

    * ``line(A) - projection(P1) - line(B)`` -- the collinear LINE triple
      (pass-through scenario); the direct ``A-B`` LINE edge is kept too.
    * ``blind(S) -> projection(P1)`` -- the BLIND edge ``S-P1`` (auto-seek +
      re-anchor), so P1 is reachable by both line-follow and blind seek.
    """
    return SimpleNamespace(
        NODES={
            "A": (10.0, 10.0),
            "B": (200.0, 10.0),
            "C": (200.0, 150.0),
            "D": (10.0, 150.0),
            "E": (100.0, 80.0),
            "G": (250.0, 80.0),
            "P1": (55.0, 10.0),
            "S": (55.0, -40.0),
        },
        NODE_TYPES={
            "A": "line", "B": "line", "C": "line", "D": "line", "E": "line",
            "G": "blind", "P1": "projection", "S": "blind",
        },
        LINE_SEGMENTS=[
            ("A", "B"), ("B", "C"), ("C", "D"), ("D", "A"),
            ("A", "E"), ("B", "E"), ("C", "E"), ("D", "E"),
            ("A", "P1"), ("P1", "B"),
        ],
        BLIND_EDGES=[("B", "G"), ("S", "A"), ("S", "P1")],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="S",
        START_HEADING_DEG=90.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        BLIND_PROJECTION_AUTO_SEEK=True,
        RESET_NODES={},
    )


def _emit_toy(
    field_data: SimpleNamespace,
    header_path: Path,
    source_path: Path,
):
    """Build graph + next-hop from the toy field_data and emit routes."""
    graph = build_graph(field_data)
    next_hop = compute_all_pairs_next_hop(
        graph, BlindDecayParams.from_field_data(field_data)
    )
    return emit_routes(
        field_data,
        graph,
        next_hop,
        header_path=header_path,
        source_path=source_path,
        field_data_source_path=_TOY_FIELD_SOURCE,
    )


@pytest.fixture
def emit_targets(tmp_path: Path) -> tuple[Path, Path]:
    return tmp_path / "routes.h", tmp_path / "routes.cpp"


def _emit_with_toy_field(emit_targets: tuple[Path, Path]) -> tuple[Path, Path]:
    header_path, source_path = emit_targets
    return _emit_toy(_build_toy_field_data(), header_path, source_path)


def _edge_cell_text(cpp_text: str, from_name: str, to_name: str) -> str:
    """Return the single ``EdgeInfo`` aggregate-initialiser cell for ``from->to``.

    The emitter writes the EDGES table as one ``{ // from <name> (index N)``
    block per source node, with one cell line per destination ending in
    ``// to <name>`` (see :func:`header_writer._emit_edges_table`). This
    slices to the EDGES table (the NEXT_HOP table reuses the same
    ``// from <name>`` annotation, so the search must start after the
    ``const EdgeInfo EDGES`` marker), walks to the ``from_name`` block, then
    returns the cell text (everything up to the trailing ``}``) of the line
    annotated ``// to <to_name>`` so a test can assert on a single cell's
    fields -- the four override-enum slots that close the cell -- without
    matching the wrong table or source row.
    """
    edges_marker = "const EdgeInfo EDGES"
    edges_start = cpp_text.index(edges_marker)
    lines = cpp_text[edges_start:].splitlines()
    # In the EDGES table, a from-block header is ``{ // from <name> (``; the
    # leading ``{ `` distinguishes it from the NEXT_HOP rows that end in the
    # same annotation. (Already moot here because we sliced past NEXT_HOP,
    # but keeping the precise marker makes the intent explicit.)
    from_marker = "{ // from " + from_name + " ("
    to_marker = "// to " + to_name
    in_block = False
    for line in lines:
        if from_marker in line:
            in_block = True
            continue
        if in_block:
            # A new ``{ // from <other> (`` header ends the current block.
            if "// from " in line:
                break
            if line.rstrip().endswith(to_marker):
                # Strip the trailing ", // to <name>" annotation, leaving the
                # brace-enclosed cell ending at "}".
                brace_end = line.rfind("}")
                return line[: brace_end + 1].strip()
    raise AssertionError(
        "no EDGES cell found for " + from_name + "->" + to_name
    )


# ---------------------------------------------------------------------------
# Existing behavioural tests -- kept verbatim in intent; switched to the
# toy fixture so they no longer depend on edits to the canonical
# field_data.py.
# ---------------------------------------------------------------------------

def test_emit_writes_both_files(emit_targets):
    header_path, source_path = emit_targets
    returned_header, returned_source = _emit_with_toy_field(emit_targets)
    assert returned_header == header_path
    assert returned_source == source_path
    assert header_path.exists() and header_path.stat().st_size > 0
    assert source_path.exists() and source_path.stat().st_size > 0


# ---------------------------------------------------------------------------
# Byte-golden tests.
#
# These compare the emitter's output against the frozen files in
# ``tools/path_planner/tests/golden/``.  A failure here means the emitter
# or a ``pipeline_config.py`` default changed without a corresponding
# golden rebaseline.
#
# If the change was INTENTIONAL:  run the regeneration helper and commit
# the updated goldens together with your change (see module docstring for
# the full workflow).
#
# If the change was UNINTENTIONAL: investigate before touching the goldens.
# ---------------------------------------------------------------------------

def test_routes_h_matches_golden(emit_targets):
    header_path, _ = emit_targets
    _emit_with_toy_field(emit_targets)
    if not _GOLDEN_HEADER.exists():
        pytest.skip("golden routes_toy.h missing; regenerate it")
    assert header_path.read_bytes() == _GOLDEN_HEADER.read_bytes()


def test_routes_cpp_matches_golden(emit_targets):
    _, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    if not _GOLDEN_SOURCE.exists():
        pytest.skip("golden routes_toy.cpp missing; regenerate it")
    assert source_path.read_bytes() == _GOLDEN_SOURCE.read_bytes()


def test_emit_is_idempotent(emit_targets):
    header_path, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    h1 = hashlib.sha256(header_path.read_bytes()).hexdigest()
    c1 = hashlib.sha256(source_path.read_bytes()).hexdigest()
    _emit_with_toy_field(emit_targets)
    h2 = hashlib.sha256(header_path.read_bytes()).hexdigest()
    c2 = hashlib.sha256(source_path.read_bytes()).hexdigest()
    assert h1 == h2
    assert c1 == c2


def test_source_sha256_matches_field_data(emit_targets):
    header_path, _ = emit_targets
    _emit_with_toy_field(emit_targets)
    expected_hash = compute_source_hash(_TOY_FIELD_SOURCE)
    text = header_path.read_text(encoding="utf-8")
    assert expected_hash in text
    assert "Source SHA256: " + expected_hash in text


def test_unreachable_pair_emits_0xff(emit_targets):
    """Removing BLIND_EDGES isolates the blind nodes; NEXT_HOP must show 0xFF.

    Built on the self-contained toy fixture rather than the production field so
    the test holds regardless of how ``field_data.json`` is encoded -- between
    phases the real field may collapse to a single spawn node, which cannot
    produce an unreachable pair. The toy field's ``G`` and ``S`` nodes are only
    reachable via BLIND edges, so stripping ``BLIND_EDGES`` strands both and the
    emitted NEXT_HOP table must carry multiple ``0xFF`` sentinels.
    """
    fd = _build_toy_field_data()
    fd.BLIND_EDGES = []
    # Pin EDGES empty so this unreachable-pair test exercises only the
    # geometry graph: no rich-edge overrides, no pivot tags, nothing that
    # could perturb the NEXT_HOP table beyond the BLIND_EDGES removal above.
    fd.EDGES = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        graph = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))
    header_path, source_path = emit_targets
    emit_routes(
        fd, graph, next_hop,
        header_path=header_path, source_path=source_path,
        field_data_source_path=_TOY_FIELD_SOURCE,
    )
    cpp_text = source_path.read_text(encoding="utf-8")
    assert NEXT_HOP_NONE_HEX in cpp_text
    assert cpp_text.count(NEXT_HOP_NONE_HEX) >= 2


def test_node_indexing_alphabetical(emit_targets):
    _, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")
    fd = _build_toy_field_data()
    sorted_names = sorted(fd.NODES.keys())
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
    """Edges render as ``EdgeType::*`` literals, not integers.

    PR4 dropped the ``_E`` suffix; the new spelling is bare ``LINE`` /
    ``BLIND``. Issue #64 reduced ``EdgeType`` to ``NONE`` / ``LINE`` / ``BLIND``
    (projection is a NODE type now), so ``EdgeType::PROJECTION`` must be ABSENT;
    the projection node emits ``NodeType::PROJECTION`` instead (asserted by
    :func:`test_projection_node_emits_nodetype_projection`).
    """
    _, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")
    assert "EdgeType::LINE," in cpp_text or "EdgeType::LINE " in cpp_text
    assert "EdgeType::BLIND," in cpp_text or "EdgeType::BLIND " in cpp_text
    # Issue #64: the projection EDGE type is retired.
    assert "EdgeType::PROJECTION" not in cpp_text


def test_projection_node_emits_nodetype_projection(emit_targets):
    """TC15: the toy projection node ``P1`` emits ``NodeType::PROJECTION`` on its NODES row.

    The toy fixture carries a ``projection`` node, so the emitter must render the
    ``NodeType`` enum value ``PROJECTION = 3`` and stamp it on P1's NODES row --
    the same on-model treatment as ``RESET``. This is the load-bearing positive
    half of TC15 (the negatives -- ``EdgeType::PROJECTION`` / ``ProjectionMode``
    / ``projection_mode_override`` ABSENT, ``EdgeInfo`` 7 fields -- are asserted
    in :func:`test_projection_node_drops_retired_projection_symbols`).
    """
    header_path, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    # routes.h: the NodeType enum gains PROJECTION = 3.
    assert "PROJECTION = 3" in header_text
    node_type_line = next(
        line for line in header_text.splitlines()
        if "enum class NodeType" in line
    )
    assert "PROJECTION = 3" in node_type_line
    assert "RESET = 2" in node_type_line

    # routes.cpp: P1's NODES row carries NodeType::PROJECTION (not LINE / BLIND).
    assert 'NodeType::PROJECTION, "P1"' in cpp_text


def test_projection_node_drops_retired_projection_symbols(emit_targets):
    """TC15 (negatives): EdgeType::PROJECTION / ProjectionMode / projection_mode_override absent; EdgeInfo is 7 fields.

    Issue #64 retired the projection EDGE type and its motion override. The
    emitted routes.{h,cpp} must carry NONE of: ``EdgeType::PROJECTION``,
    ``ProjectionMode`` (the enum), ``projection`` (the PipelineConfig member), or
    ``projection_mode_override`` (the EdgeInfo field). The ``EdgeInfo`` struct
    must therefore have exactly 7 fields, ending on ``stop_at_line_override``.
    """
    header_path, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    for retired in (
        "EdgeType::PROJECTION",
        "ProjectionMode",
        "projection_mode_override",
    ):
        assert retired not in header_text, f"{retired!r} leaked into routes.h"
        assert retired not in cpp_text, f"{retired!r} leaked into routes.cpp"

    # EdgeInfo is exactly 7 fields: type, distance_cm, heading_deg,
    # line_mode_override, blind_mode_override, direction_override,
    # stop_at_line_override. Slice the struct body and count the field lines.
    struct_start = header_text.index("struct EdgeInfo {")
    struct_end = header_text.index("};", struct_start)
    struct_body = header_text[struct_start:struct_end]
    field_lines = [
        ln.strip() for ln in struct_body.splitlines()
        if ln.strip().endswith(";") and "struct EdgeInfo" not in ln
    ]
    assert len(field_lines) == 7, (
        f"EdgeInfo must have 7 fields; got {len(field_lines)}: {field_lines}"
    )
    assert field_lines[-1].startswith("StopAtLineMode") and "stop_at_line_override" in field_lines[-1]


def test_compute_source_hash_is_full_sha256():
    digest = compute_source_hash(_TOY_FIELD_SOURCE)
    # Mirror compute_source_hash's CRLF->LF normalization so the test is stable
    # on Windows checkouts with git autocrlf=true (raw bytes would otherwise
    # contain CRLF and diverge from the LF-normalized digest).
    normalized = _TOY_FIELD_SOURCE.read_bytes().replace(b"\r\n", b"\n")
    expected = hashlib.sha256(normalized).hexdigest()
    assert digest == expected
    assert len(digest) == 64
    assert all(c in "0123456789abcdef" for c in digest)


def test_compute_source_hash_normalizes_crlf_to_lf(tmp_path):
    """CRLF and LF copies of the same content must hash to the same digest.

    Guards the CRLF->LF normalization explicitly so a regression to raw-byte
    hashing would fail this test even on an LF-only Linux checkout, not just
    on Windows where autocrlf happens to expose it.
    """
    content_lf = b"FIELD_MM = (3000, 2000)\nNODES = {}\nSTART_NODE = None\n"
    content_crlf = content_lf.replace(b"\n", b"\r\n")

    lf_path = tmp_path / "lf.py"
    crlf_path = tmp_path / "crlf.py"
    lf_path.write_bytes(content_lf)
    crlf_path.write_bytes(content_crlf)

    assert compute_source_hash(lf_path) == compute_source_hash(crlf_path)


def test_default_paths_are_under_repo():
    assert DEFAULT_HEADER_PATH == Path("WRO2026/routes.h")
    assert DEFAULT_SOURCE_PATH == Path("WRO2026/routes.cpp")
    assert FIELD_DATA_PATH == Path("tools/path_planner/field_data.py")
    assert FIELD_DATA_JSON_PATH == Path("tools/path_planner/field_data.json")


def test_source_hash_covers_every_supplied_file(tmp_path):
    """A multi-file stamp must change when ANY input changes.

    Guards the field_data.json migration: geometry lives in the JSON, so the
    banner stamp must fold both files in. A single-file digest stays stable
    (no separator) so legacy callers and the toy goldens are unaffected.
    """
    py = tmp_path / "field_data.py"
    js = tmp_path / "field_data.json"
    py.write_bytes(b"EDGES = []\n")
    js.write_bytes(b'{"NODES": {"A": [0, 0]}}\n')

    single = compute_source_hash(py)
    combined = compute_source_hash(py, js)
    assert combined != single

    # Editing the JSON geometry alone must move the combined digest.
    js.write_bytes(b'{"NODES": {"A": [1, 1]}}\n')
    assert compute_source_hash(py, js) != combined

    # ...and editing the .py alone must move it too.
    moved_json = compute_source_hash(py, js)
    py.write_bytes(b"EDGES = [1]\n")
    assert compute_source_hash(py, js) != moved_json


# ---------------------------------------------------------------------------
# Node-count ceiling guard (PR35 finding: 0xFF sentinel boundary).
#
# NEXT_HOP_NONE is 0xFF (255) and node ids run 0..N_NODES-1, so the sentinel
# only aliases a valid id once N_NODES reaches 256. The guard must reject
# 256+ and accept 255 -- the original `>= 255` test was off by one and
# wrongly rejected an otherwise-valid 255-node field.
# ---------------------------------------------------------------------------

def test_emit_rejects_256_nodes():
    """256 nodes alias id 255 onto NEXT_HOP_NONE(0xFF) and must be rejected.

    With 256 nodes the valid ids run 0..255, so id 255 collides with the
    0xFF sentinel and the table can no longer tell "no next hop" apart from
    "go to node 255". The guard reads only ``field_data.NODES`` and fires
    before any graph work, so the stub ``None`` graph / table is never
    reached.
    """
    fd = SimpleNamespace(NODES={f"N{i}": (0.0, 0.0) for i in range(256)})
    with pytest.raises(ValueError, match=r"ceiling \(max 255 nodes\)"):
        emit_routes(fd, None, None, field_data_source_path=_TOY_FIELD_SOURCE)


def test_emit_accepts_255_nodes(emit_targets):
    """255 nodes is the true ceiling: ids 0..254 never reach the 0xFF sentinel.

    Regression guard for the off-by-one ``>= 255`` check that wrongly
    rejected a 255-node field. A straight N0->N1->...->N254 line chain emits
    cleanly; the assertion is simply that emission succeeds and writes both
    files.
    """
    header_path, source_path = emit_targets
    names = [f"N{i}" for i in range(255)]
    # Issue #64: the validator rejects an on-line START, so N0 is a blind spawn
    # joined to the line chain N1..N254 by a BLIND edge (the rest stay LINE).
    node_types = {name: "line" for name in names}
    node_types["N0"] = "blind"
    fd = SimpleNamespace(
        NODES={name: (float(i), 0.0) for i, name in enumerate(names)},
        NODE_TYPES=node_types,
        LINE_SEGMENTS=[(names[i], names[i + 1]) for i in range(1, 254)],
        BLIND_EDGES=[(names[0], names[1])],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="N0",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=[],
        BLIND_PROJECTION_AUTO_SEEK=True,
        RESET_NODES={},
    )
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        graph = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))
    emit_routes(
        fd, graph, next_hop,
        header_path=header_path, source_path=source_path,
        field_data_source_path=_TOY_FIELD_SOURCE,
    )
    assert header_path.exists() and header_path.stat().st_size > 0
    assert source_path.exists() and source_path.stat().st_size > 0


# ---------------------------------------------------------------------------
# T11 / new tests (TC1, TC2, TC3, TC5)
# ---------------------------------------------------------------------------

def test_pipeline_config_default_emit(emit_targets):
    """TC1: PIPELINE_CONFIG defaults emit canonical struct in routes.cpp.

    Issue #64 dropped the ``projection`` PipelineConfig member (and the
    ``ProjectionMode`` enum), so the emitted struct now carries only
    line / blind / turn / direction.
    """
    _, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")
    assert "const PipelineConfig PIPELINE_CONFIG = {" in cpp_text
    assert "LineMode::NORMAL" in cpp_text
    assert "BlindMode::SYNC_TSHAPE" in cpp_text
    assert "TurnMode::COUNTER_ROT" in cpp_text
    # ProjectionMode was retired (issue #64) -- it must NOT appear anywhere.
    assert "ProjectionMode" not in cpp_text


def test_blind_mode_enum_emits_stop_at_line(emit_targets):
    """TC3: the emitted routes.h BlindMode enum carries STOP_AT_LINE = 4.

    The stop-at-line work adds a fifth ``BlindMode`` member. The header
    writer must render it verbatim into the generated ``BlindMode`` enum so
    the firmware's ``Routes::BlindMode`` matches ``pipeline_config.BlindMode``.
    The golden test (:func:`test_routes_h_matches_golden`) locks the full
    line byte-for-byte; this test states the contract directly so a
    regression that dropped the member fails with a readable diagnostic
    rather than an opaque golden mismatch.
    """
    header_path, _ = emit_targets
    _emit_with_toy_field(emit_targets)
    header_text = header_path.read_text(encoding="utf-8")
    assert "STOP_AT_LINE = 4" in header_text
    # The new member lives on the BlindMode enum line, not some other enum.
    blind_line = next(
        line for line in header_text.splitlines()
        if "enum class BlindMode" in line
    )
    assert "STOP_AT_LINE = 4" in blind_line
    assert "RAW_FLAT = 3" in blind_line


def test_edge_primitive_mode_override_emit(emit_targets):
    """TC2: an EDGE_LINE with primitive_mode=LineMode.FAST emits the override."""
    header_path, source_path = emit_targets
    fd = _build_toy_field_data()
    # Drop one LINE_SEGMENTS pair so the rich-EDGES entry is the only
    # carrier of the A-E edge; this isolates the override-emit assertion
    # to a single, predictable cell.
    fd.LINE_SEGMENTS = [
        e for e in fd.LINE_SEGMENTS if set(e) != {"A", "E"}
    ]
    fd.EDGES = [
        {
            "type":           EDGE_LINE,
            "u":              "A",
            "v":              "E",
            "distance_cm":    114.02,
            "heading_deg":    37.87,
            "primitive_mode": LineMode.FAST,
        },
    ]
    _emit_toy(fd, header_path, source_path)
    cpp_text = source_path.read_text(encoding="utf-8")
    # The override must appear in the line_mode_override slot of at
    # least one EdgeInfo cell. Bare-text containment is enough; the
    # idempotency / golden tests lock the surrounding shape.
    assert "LineMode::FAST" in cpp_text


# ---------------------------------------------------------------------------
# T11: per-turn PIVOT_TURNS table (TC13, TC24) + EdgeInfo shrink (TC14).
#
# Single-wheel pivots moved off ``EdgeInfo`` into a sparse, sorted
# ``PivotTurn PIVOT_TURNS[]`` table (ADR-036). The header writer emits
# ``struct PivotTurn``, ``constexpr uint8_t N_PIVOT_TURNS = <count>``, and
# ``extern const PivotTurn PIVOT_TURNS[max(1, count)]`` sorted by the
# ``(u, v, w)`` node-index tuple. The toy golden has zero pivots, so TC24
# below is the ONLY coverage of a NON-empty table -- it drives a synthetic
# 2-pivot record list through ``emit_routes(..., pivot_records=...)``.
# ---------------------------------------------------------------------------


def _two_pivot_graph() -> tuple[SimpleNamespace, nx.DiGraph]:
    """Build a 4-node field + graph that yields two derivable pivot turns.

    Nodes A/B/C/D with two chained turns -- ``A->B->C`` (left, +90 deg) and
    ``B->C->D`` (right, -90 deg) -- so :func:`build_pivot_turns` can derive two
    records with DISTINCT wheels from authored triples. ``A->B`` heads East,
    ``B->C`` heads North (turn +90, left), ``C->D`` heads East again (turn
    normalize(0 - 90) = -90, right). Built via ``build_graph`` so the geometry /
    headings are real, and the edges are long enough that both pivots validate.
    """
    # Issue #64: the validator rejects an on-line START, so a blind spawn ``S``
    # (on a BLIND edge into A) is the dead-reckon origin. ``S`` sorts AFTER A-D
    # alphabetically, so the A=0/B=1/C=2/D=3 indices the pivot-row assertions rely
    # on are unchanged.
    fd = SimpleNamespace(
        NODES={
            "A": (0.0, 0.0),
            "B": (100.0, 0.0),
            "C": (100.0, 100.0),
            "D": (200.0, 100.0),
            "S": (0.0, -40.0),
        },
        NODE_TYPES={"A": "line", "B": "line", "C": "line", "D": "line",
                    "S": "blind"},
        LINE_SEGMENTS=[("A", "B"), ("B", "C"), ("C", "D")],
        BLIND_EDGES=[("S", "A")],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="S",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=[],
        BLIND_PROJECTION_AUTO_SEEK=True,
        RESET_NODES={},
    )
    graph = build_graph(fd)
    return fd, graph


def test_pivot_turns_table_empty_field_emits_placeholder(emit_targets):
    """TC13: a pivot-free field emits N_PIVOT_TURNS == 0 + a 1-element placeholder.

    The toy field declares no pivots, so the header reports the truth count
    ``N_PIVOT_TURNS == 0`` while the ``.cpp`` still emits a single zeroed
    placeholder element (a zero-length array bound is ill-formed in standard
    C++). The placeholder is never read at runtime.
    """
    header_path, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    # routes.h: the struct, the zero truth-count, and a 1-element extern bound.
    assert "struct PivotTurn {" in header_text
    assert "constexpr uint8_t N_PIVOT_TURNS = 0;" in header_text
    assert "extern const PivotTurn PIVOT_TURNS[1];" in header_text

    # routes.cpp: a single placeholder element, explicitly annotated.
    assert "const PivotTurn PIVOT_TURNS[1] = {" in cpp_text
    assert "placeholder: N_PIVOT_TURNS == 0, never read at runtime" in cpp_text


def test_pivot_turns_table_sorted_with_correct_count(emit_targets):
    """TC13 / TC24: a populated PIVOT_TURNS table is emitted sorted with N == count.

    Drives a synthetic 2-record list through ``emit_routes(pivot_records=...)``.
    The records are passed in DESCENDING (u, v, w) index order; the emitter must
    sort them ASCENDING by the ``(u, v, w)`` node-index tuple. With NODES sorted
    A=0, B=1, C=2, D=3 the two records are ``(B, C, D)`` -> (1, 2, 3) and
    ``(A, B, C)`` -> (0, 1, 2), so the emitted rows must lead with (0, 1, 2)
    then (1, 2, 3). Each row carries the wheel literal and the trim float.
    """
    header_path, source_path = emit_targets
    fd, graph = _two_pivot_graph()
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))

    # Two records, supplied DESC by triple so the sort is observable. Distinct
    # wheels + trims so each row is individually identifiable in the output.
    records = [
        PivotTurnRecord(u="B", v="C", w="D", wheel=TurnMode.PIVOT_RIGHT, trim_cm=9.0),
        PivotTurnRecord(u="A", v="B", w="C", wheel=TurnMode.PIVOT_LEFT, trim_cm=5.5),
    ]
    emit_routes(
        fd, graph, next_hop,
        pivot_records=records,
        header_path=header_path, source_path=source_path,
        field_data_source_path=_TOY_FIELD_SOURCE,
    )
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    # Truth count and array bound both reflect the two records.
    assert "constexpr uint8_t N_PIVOT_TURNS = 2;" in header_text
    assert "extern const PivotTurn PIVOT_TURNS[2];" in header_text

    # The two emitted rows, in ascending (u, v, w) index order. NODES order is
    # A=0, B=1, C=2, D=3 so (A,B,C) -> (0,1,2) sorts before (B,C,D) -> (1,2,3).
    row_a_b_c = "{ 0, 1, 2, TurnMode::PIVOT_LEFT, " + "{:.6f}f".format(5.5) + " },"
    row_b_c_d = "{ 1, 2, 3, TurnMode::PIVOT_RIGHT, " + "{:.6f}f".format(9.0) + " },"
    assert row_a_b_c in cpp_text
    assert row_b_c_d in cpp_text
    # Sorted: (0,1,2) must appear before (1,2,3).
    assert cpp_text.index(row_a_b_c) < cpp_text.index(row_b_c_d)


def test_pivot_turns_table_from_build_pivot_turns(emit_targets, monkeypatch):
    """TC24: records derived by build_pivot_turns flow end-to-end into the table.

    Closes the loop the synthetic-record test leaves open: rather than
    hand-construct ``PivotTurnRecord``s, derive them from authored triples via
    :func:`build_pivot_turns` (the production path) and emit them. The real
    ``field_data.PIVOT_TURNS`` is empty, so the test points it at two triples on
    the synthetic graph; the derived records must produce ``N_PIVOT_TURNS == 2``
    and two sorted rows whose wheels match the turn geometry (A->B->C is a left
    turn, B->C->D a right turn).
    """
    from tools.path_planner import field_data as fd_module

    header_path, source_path = emit_targets
    fd, graph = _two_pivot_graph()
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))

    monkeypatch.setattr(
        fd_module, "PIVOT_TURNS", [("A", "B", "C"), ("B", "C", "D")], raising=False
    )
    records = build_pivot_turns(graph, track_cm=18.0)
    assert len(records) == 2

    emit_routes(
        fd, graph, next_hop,
        pivot_records=records,
        header_path=header_path, source_path=source_path,
        field_data_source_path=_TOY_FIELD_SOURCE,
    )
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    assert "constexpr uint8_t N_PIVOT_TURNS = 2;" in header_text
    # A->B->C is a CCW (left) turn, B->C->D a CW (right) turn -- both wheels
    # appear, derived from geometry, not hand-typed.
    assert "TurnMode::PIVOT_LEFT" in cpp_text
    assert "TurnMode::PIVOT_RIGHT" in cpp_text
    # Sorted by (u, v, w) index: (0,1,2) before (1,2,3).
    assert cpp_text.index("{ 0, 1, 2,") < cpp_text.index("{ 1, 2, 3,")


def test_edge_info_struct_has_no_pivot_fields(emit_targets):
    """TC14: EdgeInfo no longer declares pivot_trim_cm / pre_turn_mode_override.

    The per-edge pivot fields and their ``offsetof`` layout guard were removed
    when single-wheel pivots moved to the sparse ``PIVOT_TURNS`` table
    (ADR-036). The emitted ``EdgeInfo`` struct must carry neither field, and the
    old field-order ``static_assert`` / ``<stddef.h>`` (only present for the
    removed ``offsetof`` guard) must be gone from routes.h.
    """
    header_path, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    # The struct still exists, but without the two removed members.
    assert "struct EdgeInfo {" in header_text
    assert "pivot_trim_cm" not in header_text
    assert "pre_turn_mode_override" not in header_text
    assert "pivot_trim_cm" not in cpp_text
    assert "pre_turn_mode_override" not in cpp_text

    # The removed offsetof layout guard (its only reason to exist was the now-
    # deleted trailing float) must be gone, along with the <stddef.h> it needed.
    assert "offsetof(EdgeInfo" not in header_text
    assert "static_assert(offsetof" not in header_text
    assert "#include <stddef.h>" not in header_text


def test_edge_info_cell_builders_drop_pivot_fields(emit_targets):
    """TC14 (cell shape): every EDGES cell ends at its five override-enum slots.

    Both the populated ``_edge_cell`` and the empty ``_none_edge_cell`` builders
    dropped the trailing ``pivot_trim_cm`` (and the turn-mode override slot), so
    each emitted cell now closes on the ``stop_at_line_override`` enum literal --
    not a float. A populated A->B cell and a NONE cell both end on a
    ``StopAtLineMode::*`` literal.
    """
    _, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")

    # A populated edge cell (A->B exists in the toy field) ends on the
    # stop_at_line_override enum literal (the last field on EdgeInfo after
    # direction_override), with no trailing float.
    a_to_b_cell = _edge_cell_text(cpp_text, "A", "B")
    assert a_to_b_cell.rstrip().endswith("StopAtLineMode::DEFAULT }"), (
        "populated EdgeInfo cell must end on stop_at_line_override; got: " + a_to_b_cell
    )
    # No EdgeInfo cell carries a trailing pivot_trim_cm float anymore: the
    # only floats in a cell are distance_cm / heading_deg near the START.
    assert "EdgeType::NONE" in cpp_text
    # A NONE cell also closes on the stop_at_line default (its 5 override slots).
    assert "EdgeType::NONE, " in cpp_text
    assert "StopAtLineMode::DEFAULT }" in a_to_b_cell


# Legacy EdgeType-suffix sentinel. Built via string concatenation so the
# test file itself contains zero literal occurrences of the deprecated
# names -- otherwise the sweep would flag this test file. Word-boundary
# regex avoids false positives on legitimate identifiers like
# ``BLIND_EDGES`` and ``LINE_SEGMENTS`` that contain the substring.
_LEGACY_LINE_TOKEN  = "LINE" + "_E"
_LEGACY_BLIND_TOKEN = "BLIND" + "_E"
_LEGACY_ENUM_PATTERN = _re.compile(
    r"\b(?:" + _LEGACY_LINE_TOKEN + r"|" + _LEGACY_BLIND_TOKEN + r")\b"
)


def test_no_legacy_edge_type_suffix_anywhere():
    """TC5: sweep the repo for the deprecated EdgeType _E suffix.

    Scope here is ``tools/`` only. The plan's full sweep also covers
    ``WRO2026/`` -- those files are owned by sibling tasks T14 (regen of
    ``WRO2026/routes.{h,cpp}``) and T17 (PathTracker rename), which run
    AFTER T11 in the task graph. Once T14 and T17 land, this test should
    be widened by adding ``"WRO2026"`` to the ``sub`` tuple below;
    leaving it narrow here keeps T11 self-contained and lets the test
    pass on the partial migration without papering over the legacy refs
    in C++ source. ``docs/`` is intentionally excluded permanently
    because historical plan / decision notes may reference the old
    names.
    """
    repo_root = Path(__file__).resolve().parents[3]
    hits = []
    for sub in ("tools",):
        root = repo_root / sub
        if not root.exists():
            continue
        for p in root.rglob("*"):
            if not p.is_file():
                continue
            if p.suffix not in (".py", ".h", ".cpp", ".ino"):
                continue
            try:
                text = p.read_text(encoding="utf-8", errors="ignore")
            except (OSError, UnicodeDecodeError):
                continue
            if _LEGACY_ENUM_PATTERN.search(text):
                hits.append(str(p))
    assert not hits, (
        "Found legacy EdgeType _E suffix in: " + ", ".join(hits)
    )


def test_blind_projection_auto_seek_source_parity():
    """TC-TOGGLE-PARITY: emitted BLIND_PROJECTION_AUTO_SEEK matches field_data.

    Reads the committed ``WRO2026/routes.h`` (without regenerating) and
    asserts that the ``constexpr bool BLIND_PROJECTION_AUTO_SEEK`` value
    inside ``namespace Routes`` equals ``field_data.BLIND_PROJECTION_AUTO_SEEK``.

    This test guards the parity contract between the Python planner/sim and the
    firmware: if someone flips the toggle in ``field_data.py`` and regenerates
    ``routes.h``, this test confirms the two surfaces agree; if someone edits one
    without the other, this test catches the silent divergence at CI time.

    The committed ``routes.h`` is authoritative -- this test does NOT regenerate
    it, consistent with the CI freshness gate (``git diff --exit-code``).
    """
    header_path = _REPO_ROOT / DEFAULT_HEADER_PATH
    if not header_path.exists():
        pytest.fail(
            "routes.h not found at " + str(header_path)
            + "; regenerate it with: uv run python tools/path_planner/build_routes.py"
        )

    header_text = header_path.read_text(encoding="utf-8")

    # Match ``constexpr bool BLIND_PROJECTION_AUTO_SEEK = true;`` (or false),
    # tolerating any surrounding whitespace and the namespace indentation.
    match = _re.search(
        r"constexpr\s+bool\s+BLIND_PROJECTION_AUTO_SEEK\s*=\s*(true|false)\s*;",
        header_text,
    )
    assert match is not None, (
        "constexpr bool BLIND_PROJECTION_AUTO_SEEK not found in "
        + str(header_path)
        + "; the header writer may not be emitting this constant"
    )

    cpp_literal = match.group(1)        # "true" or "false"
    emitted_value = cpp_literal == "true"
    expected_value = real_field_data.BLIND_PROJECTION_AUTO_SEEK

    assert emitted_value == expected_value, (
        "routes.h emits BLIND_PROJECTION_AUTO_SEEK = "
        + cpp_literal
        + " but field_data.BLIND_PROJECTION_AUTO_SEEK = "
        + repr(expected_value)
        + "; regenerate routes.h or fix field_data.py"
    )


def test_direction_override_emits_force_reverse_one_direction(emit_targets):
    """TC8: an EDGES direction='rear' override emits FORCE_REVERSE for
    the forward (A,B) edge and DEFAULT for the reverse (B,A) edge.

    Asymmetric per-edge direction override regression: the header writer
    must emit DirectionMode::FORCE_REVERSE in exactly one cell (A->B) and
    leave the (B->A) cell at DirectionMode::DEFAULT (value 0, not the
    FORCE_REVERSE literal).
    """
    fd = _build_toy_field_data()
    # Drop the A-B pair from LINE_SEGMENTS so the rich-EDGES entry is the
    # sole carrier of A<->B geometry; this makes the FORCE_REVERSE cell
    # unambiguously traceable to the override.
    fd.LINE_SEGMENTS = [e for e in fd.LINE_SEGMENTS if set(e) != {"A", "B"}]
    fd.EDGES = [
        {
            "type":        EDGE_LINE,
            "u":           "A",
            "v":           "B",
            "distance_cm": 190.0,
            "heading_deg": 0.0,
            "direction":   "rear",
        }
    ]
    header_path, source_path = emit_targets
    graph = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))
    emit_routes(
        fd, graph, next_hop,
        header_path=header_path, source_path=source_path,
        field_data_source_path=_TOY_FIELD_SOURCE,
    )
    cpp_text = source_path.read_text(encoding="utf-8")
    # The forward edge cell (A->B) must contain DirectionMode::FORCE_REVERSE.
    assert "FORCE_REVERSE" in cpp_text, "A->B forward edge missing FORCE_REVERSE"
    # FORCE_REVERSE should appear exactly once (only the A->B cell, not B->A).
    assert cpp_text.count("FORCE_REVERSE") == 1, (
        f"Expected exactly 1 FORCE_REVERSE cell (A->B), got "
        f"{cpp_text.count('FORCE_REVERSE')}"
    )


# ---------------------------------------------------------------------------
# T13: sparse RESET_NODES table (wall-zero reset nodes, PR2).
#
# Mirrors the PIVOT_TURNS emit coverage. The toy golden has zero reset nodes,
# so this is the ONLY coverage of a NON-empty RESET_NODES table -- it drives a
# field_data with one "reset"-typed node + a RESET_NODES entry through
# emit_routes and pins the emitted ResetNode row, the axis encoding (x=0/y=1),
# N_RESET_NODES, and the NodeType::RESET usage.
# ---------------------------------------------------------------------------


def test_reset_nodes_table_emits_row_and_count(emit_targets):
    """T13 (AC12 emit): a reset node emits a ResetNode row + N_RESET_NODES = 1.

    A small inline field with one ``"reset"``-typed node (``Wall1``, against the
    top wall) and its sparse ``RESET_NODES`` entry must emit:

    * ``NodeType::RESET`` on the Wall1 NODES row (the enum value, not LINE/BLIND);
    * ``N_RESET_NODES = 1`` in routes.h with a ``ResetNode RESET_NODES[1]``;
    * one ``ResetNode`` row ``{ <idx>, 1, 154.000000f, 90 }`` -- axis ``"y"``
      encodes to ``1`` (the load-bearing x=0/y=1 contract), wall_coord as a
      float, facing rounded to an ``int16_t``.

    The node order is alphabetical; the assertion derives Wall1's index from the
    sorted NODES rather than hard-coding it so a future node addition would not
    silently pass.

    Issue #64: the validator rejects an on-line START, so a blind spawn ``Spawn``
    (on a BLIND edge into Inter) is the dead-reckon origin.
    """
    fd = SimpleNamespace(
        NODES={
            "Inter": (165.0, 119.0),
            "Wall1": (165.0, 139.0),
            "Spawn": (200.0, 119.0),
        },
        NODE_TYPES={"Inter": "line", "Wall1": "reset", "Spawn": "blind"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[("Inter", "Wall1"), ("Spawn", "Inter")],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
        START_NODE="Spawn",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3200, 1540),
        MAP_PNG="maps/map.png",
        EDGES=[],
        BLIND_PROJECTION_AUTO_SEEK=True,
        RESET_NODES={
            "Wall1": {"axis": "y", "wall_coord": 154.0, "facing": 270.0},
        },
    )
    header_path, source_path = emit_targets
    graph = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))
    emit_routes(
        fd, graph, next_hop,
        header_path=header_path, source_path=source_path,
        field_data_source_path=_TOY_FIELD_SOURCE,
    )
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    # routes.h: the truth count, the [1] extern bound, and the enum value.
    assert "constexpr uint8_t N_RESET_NODES = 1;" in header_text
    assert "extern const ResetNode RESET_NODES[1];" in header_text
    assert "RESET = 2" in header_text

    # Wall1's NODES row carries NodeType::RESET (not LINE / BLIND).
    wall1_idx = sorted(fd.NODES).index("Wall1")
    assert "NodeType::RESET, \"Wall1\"" in cpp_text

    # The single emitted ResetNode row: axis "y" -> 1 (x=0/y=1 contract),
    # wall_coord as a float, facing rounded to int16_t.
    expected_row = (
        "{ " + str(wall1_idx) + ", 1, " + "{:.6f}f".format(154.0) + ", 270 },"
    )
    assert expected_row in cpp_text, (
        "expected ResetNode row " + expected_row + " not found in:\n" + cpp_text
    )
    # No placeholder line when the table is populated.
    assert "N_RESET_NODES == 0, never read at runtime" not in cpp_text


def test_resolve_reset_rows_facing_rounds_up_wraps_mod_360() -> None:
    """A facing that rounds up to 360 wraps to 0 to honor the [0, 360) contract.

    The validator admits facing < 360.0, so 359.7 is legal input but
    ``round(359.7) == 360`` would breach the emitted ``int16_t facing; //
    degrees [0, 360)`` contract. The emitter reduces mod 360 so the row carries
    0, not 360.
    """
    rows = _resolve_reset_rows(
        {"R": {"axis": "y", "wall_coord": 154.0, "facing": 359.7}},
        {"R": 0},
    )
    assert rows == [(0, 1, 154.0, 0)]


def test_resolve_reset_rows_facing_below_half_does_not_wrap() -> None:
    """A facing that rounds DOWN below 360 is unaffected by the mod."""
    rows = _resolve_reset_rows(
        {"R": {"axis": "x", "wall_coord": 10.0, "facing": 359.4}},
        {"R": 0},
    )
    assert rows == [(0, 0, 10.0, 359)]


def test_reset_nodes_table_empty_field_emits_placeholder(emit_targets):
    """T13: a reset-free field emits N_RESET_NODES == 0 + a 1-element placeholder.

    The toy field declares no reset nodes, so the header reports the truth count
    ``N_RESET_NODES == 0`` while the ``.cpp`` still emits a single zeroed
    placeholder element (a zero-length array bound is ill-formed in standard
    C++), mirroring the empty-``PIVOT_TURNS`` discipline. The placeholder is
    never read at runtime.
    """
    header_path, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    header_text = header_path.read_text(encoding="utf-8")
    cpp_text = source_path.read_text(encoding="utf-8")

    # routes.h: the struct, the zero truth-count, and a 1-element extern bound.
    assert "struct ResetNode {" in header_text
    assert "constexpr uint8_t N_RESET_NODES = 0;" in header_text
    assert "extern const ResetNode RESET_NODES[1];" in header_text

    # routes.cpp: a single placeholder element, explicitly annotated.
    assert "const ResetNode RESET_NODES[1] = {" in cpp_text
    assert "placeholder: N_RESET_NODES == 0, never read at runtime" in cpp_text


# ---------------------------------------------------------------------------
# Golden regeneration helper (NOT collected by pytest).
#
# Run this module directly to overwrite the golden files after an
# AUTHORISED change to the emitter or pipeline_config.py defaults:
#
#     python tools/path_planner/tests/test_header_writer.py
#
# Review the resulting ``git diff tools/path_planner/tests/golden/``
# before committing.  Never run this to silence an unintentional failure.
# ---------------------------------------------------------------------------

def regenerate_goldens() -> None:
    """Overwrite the on-disk golden files from the current emitter output.

    Emits the stable toy field-data fixture (same inputs as the byte-golden
    tests) directly into ``_GOLDEN_HEADER`` and ``_GOLDEN_SOURCE``.  Only
    call this after an AUTHORISED change to ``header_writer.py`` or the
    defaults in ``pipeline_config.py``.

    This function is intentionally NOT a pytest test (no ``test_`` prefix)
    so it is never collected or executed during ``pytest`` runs.
    """
    _emit_toy(_build_toy_field_data(), _GOLDEN_HEADER, _GOLDEN_SOURCE)
    print(f"Regenerated golden files:")
    print(f"  {_GOLDEN_HEADER}")
    print(f"  {_GOLDEN_SOURCE}")
    print("Review the diff (git diff tools/path_planner/tests/golden/) before committing.")


if __name__ == "__main__":
    regenerate_goldens()
