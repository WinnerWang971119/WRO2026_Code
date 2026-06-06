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

import copy
import hashlib
import re as _re
import tempfile
import warnings
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner import field_data as real_field_data
from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    EDGE_LINE,
    Pivot,
    build_graph,
)
from tools.path_planner.header_writer import (
    DEFAULT_HEADER_PATH,
    DEFAULT_SOURCE_PATH,
    FIELD_DATA_PATH,
    NEXT_HOP_NONE_HEX,
    compute_source_hash,
    emit_routes,
)
from tools.path_planner.pipeline_config import LineMode, TurnMode
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
# Stable on-disk file whose SHA256 is embedded in the toy goldens. The
# toy fixture is built in Python (a SimpleNamespace), so this file is
# purely a hash anchor -- editing it would force a golden rebaseline.
_TOY_FIELD_SOURCE = _GOLDEN_DIR / "_toy_field_source.py"
_REPO_ROOT = Path(__file__).resolve().parents[3]
_FIELD_DATA_SRC = _REPO_ROOT / FIELD_DATA_PATH


def _clone_field_data() -> SimpleNamespace:
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


def _build_toy_field_data() -> SimpleNamespace:
    """Construct the canonical toy field_data fixture used by the goldens.

    Stable, minimal, exercises every node category (LINE + BLIND) and
    every classic edge category (LINE + BLIND). Per-edge overrides and
    PROJECTION edges live in the override-specific tests (TC2/TC3) so
    the byte-identical golden tests stay simple to diff.
    """
    return SimpleNamespace(
        NODES={
            "A": (10.0, 10.0),
            "B": (200.0, 10.0),
            "C": (200.0, 150.0),
            "D": (10.0, 150.0),
            "E": (100.0, 80.0),
            "G": (250.0, 80.0),
        },
        NODE_TYPES={
            "A": "line", "B": "line", "C": "line", "D": "line", "E": "line",
            "G": "blind",
        },
        LINE_SEGMENTS=[
            ("A", "B"), ("B", "C"), ("C", "D"), ("D", "A"),
            ("A", "E"), ("B", "E"), ("C", "E"), ("D", "E"),
        ],
        BLIND_EDGES=[("B", "G")],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        COST_BLIND_MULTIPLIER=5.0,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
    )


def _emit_toy(
    field_data: SimpleNamespace,
    header_path: Path,
    source_path: Path,
):
    """Build graph + next-hop from the toy field_data and emit routes."""
    graph = build_graph(field_data)
    next_hop = compute_all_pairs_next_hop(graph, field_data.COST_BLIND_MULTIPLIER)
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
    """Removing BLIND_EDGES makes G unreachable; NEXT_HOP must show 0xFF."""
    fd = _clone_field_data()
    fd.BLIND_EDGES = []
    # Pin EDGES empty so this unreachable-pair test exercises only the
    # geometry graph: no rich-edge overrides, no pivot tags, nothing that
    # could perturb the NEXT_HOP table beyond the BLIND_EDGES removal above.
    fd.EDGES = []
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
    ``BLIND`` (and a new ``PROJECTION``). The toy fixture has no
    PROJECTION edges, so only LINE + BLIND are asserted here -- the
    PROJECTION emit path is covered separately in
    :func:`test_edge_primitive_mode_override_emit` via a fixture that
    seeds a PROJECTION edge.
    """
    _, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")
    assert "EdgeType::LINE," in cpp_text or "EdgeType::LINE " in cpp_text
    assert "EdgeType::BLIND," in cpp_text or "EdgeType::BLIND " in cpp_text


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


# ---------------------------------------------------------------------------
# T11 / new tests (TC1, TC2, TC3, TC5)
# ---------------------------------------------------------------------------

def test_pipeline_config_default_emit(emit_targets):
    """TC1: PIPELINE_CONFIG defaults emit canonical struct in routes.cpp."""
    _, source_path = emit_targets
    _emit_with_toy_field(emit_targets)
    cpp_text = source_path.read_text(encoding="utf-8")
    assert "const PipelineConfig PIPELINE_CONFIG = {" in cpp_text
    assert "LineMode::NORMAL" in cpp_text
    assert "BlindMode::SYNC_TSHAPE" in cpp_text
    assert "ProjectionMode::NORMAL" in cpp_text
    assert "TurnMode::COUNTER_ROT" in cpp_text


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


def test_edge_pre_turn_mode_override_emit(emit_targets):
    """TC3: an edge with pre_turn_mode=TurnMode.PIVOT_LEFT emits the override.

    Three-node A->B->C fixture: ``A->B`` carries the pivot override, and
    ``B->C`` is the single forward continuation needed by
    ``apply_pivot_compensations`` (the back-edge ``B->A`` is excluded
    from the count so it does not trip the multi-successor guard).
    Directional overrides (``pre_turn_mode``, ``pivot``) are forward-only
    after the Codex P1 fix, so the reverse traversal would land at A
    with no pivot tag -- the compensation iteration finds only the
    forward (A,B) entry.
    """
    header_path, source_path = emit_targets
    fd = SimpleNamespace(
        NODES={"A": (0.0, 0.0), "B": (100.0, 0.0), "C": (100.0, 100.0)},
        NODE_TYPES={"A": "line", "B": "blind", "C": "blind"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        COST_BLIND_MULTIPLIER=5.0,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=[
            {
                "type":          EDGE_BLIND,
                "u":             "A",
                "v":             "B",
                "distance_cm":   100.0,
                "heading_deg":   0.0,
                "pre_turn_mode": TurnMode.PIVOT_LEFT,
                "pivot":         Pivot.LEFT,
            },
            {
                "type":          EDGE_BLIND,
                "u":             "B",
                "v":             "C",
                "distance_cm":   100.0,
                "heading_deg":   90.0,
            },
        ],
    )
    _emit_toy(fd, header_path, source_path)
    cpp_text = source_path.read_text(encoding="utf-8")
    assert "TurnMode::PIVOT_LEFT" in cpp_text


def test_pivot_compensation_reaches_emitted_distance(emit_targets):
    """TC9 / finding #42: the shortened pivot distance lands in routes.cpp.

    Before Decision B, ``apply_pivot_compensations`` ran only on a
    discarded weighted copy inside ``compute_all_pairs_next_hop``, so the
    EDGES table in routes.cpp carried the UN-shortened ``distance_cm`` and
    the robot overshot every pivot landing. Now the compensation mutates
    the same ``graph`` object that ``_emit_toy`` hands to ``emit_routes``,
    so the compensated distance must appear in the emitted source.

    Three-node A->B->C fixture (mirrors
    :func:`test_edge_pre_turn_mode_override_emit`): ``A->B`` is a 90 deg
    LEFT pivot, ``B->C`` is the single forward continuation. For
    TRACK_CM=15 and a 90 deg turn the forward chord component is 7.5 cm,
    so ``B->C`` is shortened from 30.0 to 22.5 cm. ``22.500000f`` is the
    compensated literal; ``30.000000f`` is the pre-compensation value that
    must NOT survive on the forward B->C cell.
    """
    header_path, source_path = emit_targets
    fd = SimpleNamespace(
        NODES={"A": (0.0, 0.0), "B": (50.0, 0.0), "C": (50.0, 70.0)},
        NODE_TYPES={"A": "line", "B": "blind", "C": "blind"},
        LINE_SEGMENTS=[],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=8.0,
        COST_BLIND_MULTIPLIER=5.0,
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(3000, 2000),
        MAP_PNG="maps/map.png",
        EDGES=[
            {
                "type":        EDGE_BLIND,
                "u":           "A",
                "v":           "B",
                "distance_cm": 40.0,
                "heading_deg": 0.0,
                "pivot":       Pivot.LEFT,
            },
            {
                "type":        EDGE_BLIND,
                "u":           "B",
                "v":           "C",
                "distance_cm": 30.0,
                "heading_deg": 90.0,
            },
        ],
    )
    _emit_toy(fd, header_path, source_path)
    cpp_text = source_path.read_text(encoding="utf-8")
    # The compensated B->C distance (22.5 cm) reached the emitted EDGES
    # table -- proof the compensation now flows to emission, not just cost.
    assert "22.500000f" in cpp_text, (
        "compensated B->C distance (22.5 cm) missing from routes.cpp"
    )


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
    next_hop = compute_all_pairs_next_hop(graph, fd.COST_BLIND_MULTIPLIER)
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
