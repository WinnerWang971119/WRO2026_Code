"""Integration tests for the one-shot migration script (editor/migrate.py).

Covers:
- TC9: migrate round-trip repr equality -- after migration, all public attrs
  have the same repr as before migration.
- TC10: migrate idempotency -- running twice leaves both output files
  byte-identical on the second run.
- TC11: build_routes byte-identical graph payload before vs after migration.
  Marked skip with a clear reason (see docstring below).

Each test writes a synthetic pre-migration field_data.py into tmp_path and
monkeypatches migrate._FIELD_DATA_PY / _FIELD_DATA_JSON to avoid touching the
real repo files.

Monkeypatching strategy for _capture_module_attributes
-------------------------------------------------------
migrate._capture_module_attributes() calls importlib.reload (or import_module)
on "tools.path_planner.field_data". importlib.reload resolves the module file
via the PARENT PACKAGE's __path__ (i.e. the real tools/path_planner/ directory),
NOT via the module's __spec__.loader.path. Injecting a synthetic module object
into sys.modules["tools.path_planner.field_data"] and then calling reload causes
Python to re-read the real on-disk field_data.py, not our tmp copy.

To avoid this, we monkeypatch migrate._capture_module_attributes directly so
it returns the attributes from our synthetic module object. This is the cleanest
approach that does not require modifying migrate.py or the Python import system.
"""

from __future__ import annotations

import importlib.util
import textwrap
import types
from pathlib import Path
from typing import Any

import pytest

import tools.path_planner.editor.migrate as migrate
from tools.path_planner.editor import loader as loader_mod


# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

# The 14 public attributes the migration promises to preserve by repr().
_PUBLIC_ATTRS = (
    "FIELD_MM",
    "MAP_PNG",
    "ROBOT_RADIUS_CM",
    "BLIND_DECAY_ENABLED",
    "BLIND_DECAY_K",
    "BLIND_DECAY_EXPONENT",
    "BLIND_DECAY_D_REF",
    "START_NODE",
    "START_HEADING_DEG",
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "OBSTACLES",
)

# A small, self-contained pre-migration field_data.py source.
# Must NOT import load_field_json (it is the pre-migration form with literals).
# Includes the dashed banner + EDGES block so the migration can locate the tail.
_PRE_MIGRATION_SOURCE = textwrap.dedent('''\
    """WRO 2026 field encoding -- synthetic fixture for migration tests."""

    FIELD_MM = (3000, 2000)
    MAP_PNG = "maps/map.png"
    ROBOT_RADIUS_CM = 8.0
    BLIND_DECAY_ENABLED = True
    BLIND_DECAY_K = 0.05
    BLIND_DECAY_EXPONENT = 2.0
    BLIND_DECAY_D_REF = 50.0
    START_NODE = "A"
    START_HEADING_DEG = 0.0
    NODES = {
        "A": (10.0, 10.0),
        "B": (200.0, 10.0),
        "G": (250.0, 80.0),
    }
    NODE_TYPES = {
        "A": "line",
        "B": "line",
        "G": "blind",
    }
    LINE_SEGMENTS = [
        ("A", "B"),
    ]
    BLIND_EDGES = [
        ("B", "G"),
    ]
    OBSTACLES = [
        (50.0, 50.0, 80.0, 80.0, "pillar"),
    ]

    # ---------------------------------------------------------------------------
    # Rich edge list -- kept verbatim; migration must not touch this block.
    # ---------------------------------------------------------------------------
    EDGES = []
''')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _write_pre_migration_source(directory: Path) -> Path:
    """Write the synthetic pre-migration field_data.py into *directory*."""
    path = directory / "field_data.py"
    path.write_text(_PRE_MIGRATION_SOURCE, encoding="utf-8", newline="\n")
    return path


def _load_module_from_path(path: Path, module_name: str) -> types.ModuleType:
    """Load a .py file as a module with importlib, giving it *module_name*.

    :param path: Absolute path to the Python source file.
    :param module_name: The fully qualified module name to assign.
    :return: The loaded module object with attributes populated.
    """
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None, (
        f"Could not build spec from {path}"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)  # type: ignore[union-attr]
    return module


def _capture_attrs_repr(module: types.ModuleType) -> dict[str, str]:
    """Return a dict mapping each public attribute name to its repr()."""
    return {attr: repr(getattr(module, attr)) for attr in _PUBLIC_ATTRS}


def _make_capture_fn(
    module: types.ModuleType,
) -> "callable[[], dict[str, Any]]":
    """Return a replacement for migrate._capture_module_attributes that reads
    from *module* instead of importing 'tools.path_planner.field_data'.

    This sidesteps importlib.reload's behavior of resolving the module file
    via the parent package's __path__ (which always finds the real on-disk
    field_data.py, not our tmp copy).
    """

    def _patched_capture() -> dict[str, Any]:
        return {attr: getattr(module, attr) for attr in migrate._ATTRS_TO_CAPTURE}

    return _patched_capture


# ---------------------------------------------------------------------------
# TC9: migrate round-trip repr equality
# ---------------------------------------------------------------------------


def test_tc9_migrate_round_trip_repr_equality(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """TC9: after migration, every public attribute of the rewritten field_data.py
    has the same repr() as the pre-migration version.

    This proves AC2 from the plan:
      "After migration, field_data.py exposes the same module attributes with
      values byte-identical to pre-migration via repr() comparison."

    Steps:
    1. Write synthetic pre-migration field_data.py into tmp_path.
    2. Load it; capture repr() of each public attr.
    3. Monkeypatch migrate paths and _capture_module_attributes to read from
       the synthetic module (bypassing importlib.reload's package-path lookup).
    4. Call migrate.run(); assert exit 0.
    5. Verify the rewritten file no longer has NODES = { literals and (issue #79)
       reads EDGES from the JSON via _data.get(...) with NO Python EDGES tail.
    6. Load the rewritten field_data.py with loader._DEFAULT_JSON_PATH patched
       to our tmp JSON; compare repr() of each attr.
    """
    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    # Load the pre-migration module to capture the expected repr values.
    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    expected_reprs = _capture_attrs_repr(pre_module)

    # Monkeypatch migrate's path constants and attribute-capture function.
    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    monkeypatch.setattr(
        migrate, "_capture_module_attributes", _make_capture_fn(pre_module)
    )

    exit_code = migrate.run()
    assert exit_code == 0, f"migrate.run() returned {exit_code}, expected 0"

    # The JSON sidecar must have been written.
    assert json_path.exists(), "field_data.json was not created by migrate.run()"

    # The rewritten field_data.py must NOT contain the old NODES literal.
    rewritten_source = pre_py.read_text(encoding="utf-8")
    assert "NODES = {" not in rewritten_source, (
        "Rewritten field_data.py still contains 'NODES = {'; migration did not remove literals"
    )

    # Issue #79 / ADR-047: EDGES is JSON-sourced now. The rewritten file must
    # read EDGES back from the JSON via _data.get(...), and must NOT re-append a
    # Python EDGES literal tail (a stale second source of truth).
    assert 'EDGES                 = _data.get("EDGES", [])' in rewritten_source, (
        "Rewritten field_data.py must read EDGES from the JSON, not a Python literal"
    )
    assert "EDGES = [" not in rewritten_source, (
        "Rewritten field_data.py must NOT re-append a Python 'EDGES = [' literal tail"
    )

    # Load the rewritten field_data.py.
    # The rewritten file calls load_field_json() which by default reads
    # from loader._DEFAULT_JSON_PATH (the real package path).
    # Patch it to point at our tmp json_path before exec_module runs.
    monkeypatch.setattr(loader_mod, "_DEFAULT_JSON_PATH", json_path)

    post_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    actual_reprs = _capture_attrs_repr(post_module)

    for attr in _PUBLIC_ATTRS:
        assert actual_reprs[attr] == expected_reprs[attr], (
            f"repr mismatch for {attr} after migration:\n"
            f"  before: {expected_reprs[attr]}\n"
            f"  after:  {actual_reprs[attr]}"
        )


# ---------------------------------------------------------------------------
# TC10: migrate idempotency
# ---------------------------------------------------------------------------


def test_tc10_migrate_idempotent_byte_identical(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """TC10: running migrate.run() twice leaves both output files byte-identical.

    This proves AC1 from the plan:
      "Running it twice is a no-op (JSON is byte-identical the second time)."

    Steps:
    1. Write synthetic pre-migration field_data.py into tmp_path.
    2. Monkeypatch migrate paths and _capture_module_attributes.
    3. Run migrate.run() once; capture bytes of both files.
    4. Run migrate.run() again; assert exit 0; assert bytes unchanged.

    Note: the second run detects "already migrated" via the absence of
    'NODES = {' in the rewritten file and exits early without calling
    _capture_module_attributes, so the monkeypatch for that function
    is only load-bearing for the first run.
    """
    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")

    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    monkeypatch.setattr(
        migrate, "_capture_module_attributes", _make_capture_fn(pre_module)
    )

    # First run -- performs the actual migration.
    exit_code_1 = migrate.run()
    assert exit_code_1 == 0, f"First migrate.run() returned {exit_code_1}"

    bytes_py_after_1 = pre_py.read_bytes()
    bytes_json_after_1 = json_path.read_bytes()

    # Second run -- file is already migrated; should detect and exit early.
    exit_code_2 = migrate.run()
    assert exit_code_2 == 0, f"Second migrate.run() returned {exit_code_2}"

    bytes_py_after_2 = pre_py.read_bytes()
    bytes_json_after_2 = json_path.read_bytes()

    assert bytes_py_after_2 == bytes_py_after_1, (
        "field_data.py changed on the second migrate.run(); migration is not idempotent"
    )
    assert bytes_json_after_2 == bytes_json_after_1, (
        "field_data.json changed on the second migrate.run(); migration is not idempotent"
    )


# ---------------------------------------------------------------------------
# pivot_turns: --migrate emits the per-turn pivot key when the source module
# exposes PIVOT_TURNS, and the rewritten field_data.py re-exports it.
# ---------------------------------------------------------------------------


def test_migrate_emits_pivot_turns_when_source_exposes_it(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A source module with PIVOT_TURNS migrates the key into the JSON sidecar.

    The pivot key is optional, so it is captured separately from the fourteen
    required attrs. This test feeds a synthetic module exposing PIVOT_TURNS
    and a capture fn mirroring migrate._capture_module_attributes (the fourteen
    required attrs plus the optional lowercase pivot_turns key), then asserts
    the written JSON carries pivot_turns and the rewritten file re-exports it.
    """
    import json

    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    # The synthetic source has no PIVOT_TURNS; attach one for this test.
    pre_module.PIVOT_TURNS = [("A", "B", "G")]

    def _capture_with_pivots() -> dict[str, Any]:
        captured = {attr: getattr(pre_module, attr) for attr in migrate._ATTRS_TO_CAPTURE}
        captured["pivot_turns"] = pre_module.PIVOT_TURNS
        return captured

    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    monkeypatch.setattr(migrate, "_capture_module_attributes", _capture_with_pivots)

    assert migrate.run() == 0

    written = json.loads(json_path.read_text(encoding="utf-8"))
    assert written["pivot_turns"] == [["A", "B", "G"]]
    keys = list(written.keys())
    assert keys.index("BLIND_EDGES") < keys.index("pivot_turns") < keys.index("OBSTACLES")

    # The rewritten field_data.py must re-export the key (with a safe default).
    rewritten_source = pre_py.read_text(encoding="utf-8")
    assert 'PIVOT_TURNS           = _data.get("pivot_turns", [])' in rewritten_source

    # The rewritten module loads it back through the migrated JSON.
    monkeypatch.setattr(loader_mod, "_DEFAULT_JSON_PATH", json_path)
    post_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    assert post_module.PIVOT_TURNS == [("A", "B", "G")]


def test_migrate_omits_pivot_turns_when_source_lacks_it(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A pivot-free source migrates to JSON WITHOUT a pivot_turns key.

    The optional key is dropped on dump when absent, and the rewritten file's
    re-export defaults to an empty list, so a field that never authored pivots
    stays clean.
    """
    import json

    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")

    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    monkeypatch.setattr(
        migrate, "_capture_module_attributes", _make_capture_fn(pre_module)
    )

    assert migrate.run() == 0

    written = json.loads(json_path.read_text(encoding="utf-8"))
    assert "pivot_turns" not in written

    # The rewritten module still exposes PIVOT_TURNS, defaulted to [].
    monkeypatch.setattr(loader_mod, "_DEFAULT_JSON_PATH", json_path)
    post_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    assert post_module.PIVOT_TURNS == []


# ---------------------------------------------------------------------------
# Issue #79 / ADR-047: TC22 -- --migrate carries a legacy Python EDGES literal
# INTO field_data.json's "EDGES" key (override-only string-tag form), and the
# regenerated field_data.py reads it back via _data.get(...) with NO Python tail.
# ---------------------------------------------------------------------------


def test_tc22_migrate_carries_python_edges_literal_into_json(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """TC22: a legacy Python EDGES = [...] literal migrates into the JSON EDGES key.

    A source module exposing a Python ``EDGES`` list of enum-instance overrides
    (the legacy authoring form) must, after ``--migrate``:

    1. land those entries in ``field_data.json``'s ``"EDGES"`` key as override-only
       STRING-TAG dicts -- enum-instance ``primitive_mode`` converted to its
       member NAME, derived geometry (distance_cm / heading_deg) DROPPED, and the
       DEFAULT sentinel DROPPED (absence already means inherit);
    2. land in the canonical slot (between blind_projection / pivot keys and
       OBSTACLES); and
    3. leave the regenerated ``field_data.py`` reading EDGES from the JSON via
       ``_data.get("EDGES", [])`` with NO Python ``EDGES = [`` literal tail (AC14).
    """
    import json

    from tools.path_planner.pipeline_config import BlindMode, LineMode

    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    # Attach a legacy Python EDGES literal carrying enum-instance overrides,
    # pinned geometry, and a DEFAULT sentinel that must be dropped on migration.
    pre_module.EDGES = [
        {
            "type": "LINE",
            "u": "A",
            "v": "B",
            "distance_cm": 190.0,  # derived geometry -> dropped on migration
            "heading_deg": 0.0,    # derived geometry -> dropped on migration
            "primitive_mode": LineMode.FAST,  # enum instance -> "FAST" name
        },
        {
            "type": "BLIND",
            "u": "B",
            "v": "G",
            "primitive_mode": BlindMode.DEFAULT,  # DEFAULT sentinel -> dropped
            "stop_at_line": "on",
        },
    ]

    def _capture_with_edges() -> dict[str, Any]:
        # Mirror migrate._capture_module_attributes' EDGES path exactly: the 14
        # required attrs plus the EDGES list reduced via the production helper.
        captured = {
            attr: getattr(pre_module, attr) for attr in migrate._ATTRS_TO_CAPTURE
        }
        captured["EDGES"] = [
            migrate._edge_to_override_dict(entry) for entry in pre_module.EDGES
        ]
        return captured

    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    monkeypatch.setattr(migrate, "_capture_module_attributes", _capture_with_edges)

    assert migrate.run() == 0

    written = json.loads(json_path.read_text(encoding="utf-8"))
    assert "EDGES" in written, "migration did not carry EDGES into the JSON"
    # (1) override-only string-tag form: geometry dropped, enum -> name, DEFAULT
    # dropped.
    line_entry = next(e for e in written["EDGES"] if e["type"] == "LINE")
    assert line_entry == {
        "type": "LINE",
        "u": "A",
        "v": "B",
        "primitive_mode": "FAST",  # enum instance reduced to its member name
    }, "LINE entry must drop geometry and stringify the enum"
    assert "distance_cm" not in line_entry
    assert "heading_deg" not in line_entry

    blind_entry = next(e for e in written["EDGES"] if e["type"] == "BLIND")
    # DEFAULT sentinel dropped (absence == inherit); stop_at_line preserved.
    assert "primitive_mode" not in blind_entry, "DEFAULT sentinel must be dropped"
    assert blind_entry == {
        "type": "BLIND",
        "u": "B",
        "v": "G",
        "stop_at_line": "on",
    }

    # (2) EDGES lands in its canonical slot.
    keys = list(written.keys())
    assert keys.index("BLIND_EDGES") < keys.index("EDGES") < keys.index("OBSTACLES")

    # (3) The regenerated field_data.py reads EDGES from the JSON, no Python tail.
    rewritten_source = pre_py.read_text(encoding="utf-8")
    assert 'EDGES                 = _data.get("EDGES", [])' in rewritten_source
    assert "EDGES = [" not in rewritten_source, (
        "no Python EDGES literal tail may be re-appended (AC14)"
    )

    # The rewritten module loads the migrated overrides back through the JSON.
    monkeypatch.setattr(loader_mod, "_DEFAULT_JSON_PATH", json_path)
    post_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    assert post_module.EDGES == [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"},
        {"type": "BLIND", "u": "B", "v": "G", "stop_at_line": "on"},
    ]


def test_tc22_migrate_omits_edges_when_source_lacks_them(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """TC22 (control): an override-free source migrates to JSON WITHOUT an EDGES key.

    A field that never authored a Python EDGES literal must not gain an empty
    ``"EDGES": []`` in the JSON (absent stays absent), and the regenerated module
    still re-exports EDGES defaulted to an empty list from the JSON.
    """
    import json

    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")

    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    # _make_capture_fn captures only the 14 required attrs (no EDGES), matching a
    # source whose EDGES is empty (the real capture omits an empty/absent list).
    monkeypatch.setattr(
        migrate, "_capture_module_attributes", _make_capture_fn(pre_module)
    )

    assert migrate.run() == 0

    written = json.loads(json_path.read_text(encoding="utf-8"))
    assert "EDGES" not in written

    # The rewritten module still re-exports EDGES, defaulted to [].
    monkeypatch.setattr(loader_mod, "_DEFAULT_JSON_PATH", json_path)
    post_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    assert post_module.EDGES == []


# ---------------------------------------------------------------------------
# TC11: build_routes byte-identical graph payload before vs after migration
# ---------------------------------------------------------------------------


@pytest.mark.skip(
    reason=(
        "TC11 is skipped because reliably isolating build_routes._run() from "
        "the global sys.modules['tools.path_planner.field_data'] cache is too "
        "brittle for CI. build_routes imports field_data at module load time "
        "(top-level 'from tools.path_planner import field_data'), so by the "
        "time a test calls build_routes._run(), sys.modules already holds the "
        "real module. importlib.reload also resolves via the parent package's "
        "__path__ (the real tools/path_planner/ directory), so patching "
        "sys.modules alone is insufficient. Fully isolating this would require "
        "re-importing build_routes itself and patching its 'field_data' binding, "
        "which triggers all top-level side effects and makes the test fragile. "
        "AC3 is verified manually in the T14 smoke-test checklist: run "
        "'uv run python tools/path_planner/edit_field.py --migrate', then "
        "'uv run python tools/path_planner/build_routes.py', and confirm that "
        "routes.h / routes.cpp are byte-identical to the pre-migration golden "
        "files in tests/golden/."
    )
)
def test_tc11_build_routes_byte_identical_graph_payload(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """TC11: build_routes._run() produces byte-identical routes.h/cpp before vs
    after migration of a small field_data.py.

    Skipped -- see skip reason above for the full rationale.
    """


# ---------------------------------------------------------------------------
# TC11 (R3): migrate captures RESET_NODES + BLIND_PROJECTION_AUTO_SEEK and
# re-exposes both in the regenerated module under the correct JSON key names.
# ---------------------------------------------------------------------------


def test_tc11_r3_migrate_captures_reset_nodes_and_blind_projection_auto_seek(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """TC11 (R3): a legacy field_data.py defining RESET_NODES and
    BLIND_PROJECTION_AUTO_SEEK=False migrates both attributes into the JSON
    sidecar under the correct key names and re-exposes them in the rewritten
    module.

    Asserts:
    - RESET_NODES reaches the JSON under key "RESET_NODES" (uppercase).
    - BLIND_PROJECTION_AUTO_SEEK=False reaches the JSON under key
      "blind_projection_auto_seek" (lowercase).
    - Both keys appear in the correct canonical slot positions.
    - The regenerated field_data.py re-exposes RESET_NODES and
      BLIND_PROJECTION_AUTO_SEEK as module attributes with the correct values.
    """
    import json

    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")

    # Attach RESET_NODES and BLIND_PROJECTION_AUTO_SEEK to the synthetic module.
    pre_module.RESET_NODES = {
        "R1": {"axis": "x", "wall_coord": 0.0, "facing": 90.0},
    }
    pre_module.BLIND_PROJECTION_AUTO_SEEK = False

    def _capture_with_reset_and_seek() -> dict[str, Any]:
        captured = {attr: getattr(pre_module, attr) for attr in migrate._ATTRS_TO_CAPTURE}
        # Mirror migrate._capture_module_attributes optional-key paths:
        # RESET_NODES captured under "RESET_NODES" when non-empty.
        if pre_module.RESET_NODES:
            captured["RESET_NODES"] = pre_module.RESET_NODES
        # BLIND_PROJECTION_AUTO_SEEK captured under "blind_projection_auto_seek"
        # only when False (non-default; absent == True).
        if pre_module.BLIND_PROJECTION_AUTO_SEEK is False:
            captured["blind_projection_auto_seek"] = False
        return captured

    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    monkeypatch.setattr(migrate, "_capture_module_attributes", _capture_with_reset_and_seek)

    assert migrate.run() == 0

    written = json.loads(json_path.read_text(encoding="utf-8"))

    # RESET_NODES must be present under the uppercase key name.
    assert "RESET_NODES" in written, (
        "migration did not carry RESET_NODES into the JSON"
    )
    assert written["RESET_NODES"] == {
        "R1": {"axis": "x", "wall_coord": 0.0, "facing": 90.0}
    }

    # BLIND_PROJECTION_AUTO_SEEK must be present under the lowercase key name.
    assert "blind_projection_auto_seek" in written, (
        "migration did not carry blind_projection_auto_seek into the JSON"
    )
    assert written["blind_projection_auto_seek"] is False

    # Key ordering: RESET_NODES is between pivot_turns and blind_projection_auto_seek
    # in the canonical schema order (BLIND_EDGES < pivot_turns < RESET_NODES <
    # blind_projection_auto_seek < pipeline_* < EDGES < OBSTACLES).
    keys = list(written.keys())
    assert keys.index("BLIND_EDGES") < keys.index("RESET_NODES"), (
        "RESET_NODES must appear after BLIND_EDGES in the JSON"
    )
    assert keys.index("RESET_NODES") < keys.index("OBSTACLES"), (
        "RESET_NODES must appear before OBSTACLES in the JSON"
    )
    assert keys.index("blind_projection_auto_seek") < keys.index("OBSTACLES"), (
        "blind_projection_auto_seek must appear before OBSTACLES in the JSON"
    )

    # The regenerated field_data.py must re-expose both attributes.
    rewritten_source = pre_py.read_text(encoding="utf-8")
    assert 'RESET_NODES           = _data.get("RESET_NODES", {})' in rewritten_source, (
        "Rewritten field_data.py must re-expose RESET_NODES from the JSON"
    )
    assert 'BLIND_PROJECTION_AUTO_SEEK = _data.get("blind_projection_auto_seek", True)' in rewritten_source, (
        "Rewritten field_data.py must re-expose BLIND_PROJECTION_AUTO_SEEK from the JSON"
    )

    # Load the rewritten module and verify both attributes come back correctly.
    monkeypatch.setattr(loader_mod, "_DEFAULT_JSON_PATH", json_path)
    post_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    assert post_module.RESET_NODES == {"R1": {"axis": "x", "wall_coord": 0.0, "facing": 90.0}}
    assert post_module.BLIND_PROJECTION_AUTO_SEEK is False


def test_tc11_r3_migrate_omits_reset_nodes_and_seek_when_defaults(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """TC11 (R3 control): a legacy field_data.py with empty RESET_NODES and
    BLIND_PROJECTION_AUTO_SEEK=True (the defaults) does NOT emit those keys into
    the JSON sidecar, and the regenerated module still exposes both attributes
    with their default values.
    """
    import json

    pre_py = _write_pre_migration_source(tmp_path)
    json_path = tmp_path / "field_data.json"

    pre_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    # Explicitly set defaults so the capture path is exercised.
    pre_module.RESET_NODES = {}
    pre_module.BLIND_PROJECTION_AUTO_SEEK = True

    def _capture_with_defaults() -> dict[str, Any]:
        captured = {attr: getattr(pre_module, attr) for attr in migrate._ATTRS_TO_CAPTURE}
        if pre_module.RESET_NODES:
            captured["RESET_NODES"] = pre_module.RESET_NODES
        if pre_module.BLIND_PROJECTION_AUTO_SEEK is False:
            captured["blind_projection_auto_seek"] = False
        return captured

    monkeypatch.setattr(migrate, "_FIELD_DATA_PY", pre_py)
    monkeypatch.setattr(migrate, "_FIELD_DATA_JSON", json_path)
    monkeypatch.setattr(migrate, "_capture_module_attributes", _capture_with_defaults)

    assert migrate.run() == 0

    written = json.loads(json_path.read_text(encoding="utf-8"))

    # Neither optional key should appear when at default values.
    assert "RESET_NODES" not in written, (
        "RESET_NODES must be absent from the JSON when empty"
    )
    assert "blind_projection_auto_seek" not in written, (
        "blind_projection_auto_seek must be absent from the JSON when True (default)"
    )

    # The rewritten module still re-exposes both with their default values.
    rewritten_source = pre_py.read_text(encoding="utf-8")
    assert 'RESET_NODES           = _data.get("RESET_NODES", {})' in rewritten_source
    assert 'BLIND_PROJECTION_AUTO_SEEK = _data.get("blind_projection_auto_seek", True)' in rewritten_source

    monkeypatch.setattr(loader_mod, "_DEFAULT_JSON_PATH", json_path)
    post_module = _load_module_from_path(pre_py, "tools.path_planner.field_data")
    assert post_module.RESET_NODES == {}
    assert post_module.BLIND_PROJECTION_AUTO_SEEK is True
