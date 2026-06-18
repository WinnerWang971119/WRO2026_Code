"""Tests for tools.path_planner.build_routes (CLI orchestrator, PR3)."""

from __future__ import annotations

import copy
import warnings
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner import build_routes
from tools.path_planner import field_data as real_field_data
from tools.path_planner.tests.fixtures import toy_field_data

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
    "BLIND_PROJECTION_AUTO_SEEK",
    # Optional sparse table (empty on every fixture here) -- carried so the
    # strict header_writer read of field_data.RESET_NODES never AttributeErrors.
    "RESET_NODES",
)


def _clone_field_data() -> SimpleNamespace:
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


def _clone_toy_field_data() -> SimpleNamespace:
    """Deep-copy the stable toy fixture into a standalone field-data object.

    Sourced from ``tools.path_planner.tests.fixtures.toy_field_data`` rather
    than the live ``field_data`` module so editor saves to the canonical
    ``field_data.json`` cannot change which nodes exist or how they connect
    (see ADR-022 and the fixture's own docstring). Returning a fresh
    :class:`SimpleNamespace` keeps mutations local to the caller so the
    shared fixture module stays untouched for sibling tests.
    """
    snapshot = {attr: copy.deepcopy(getattr(toy_field_data, attr)) for attr in _FIELD_ATTRS}
    snapshot["EDGES"] = copy.deepcopy(getattr(toy_field_data, "EDGES", []))
    return SimpleNamespace(**snapshot)


def test_run_with_defaults_returns_zero(tmp_path, monkeypatch):
    header_target = tmp_path / "routes.h"
    source_target = tmp_path / "routes.cpp"
    map_target = tmp_path / "map.png"

    monkeypatch.setattr("tools.path_planner.build_routes.DEFAULT_HEADER_PATH", header_target)
    monkeypatch.setattr("tools.path_planner.build_routes.DEFAULT_SOURCE_PATH", source_target)
    monkeypatch.setattr("tools.path_planner.build_routes.DEFAULT_MAP_PNG", map_target)

    rc = build_routes._run()

    assert rc == 0
    assert header_target.exists() and header_target.stat().st_size > 0
    assert source_target.exists() and source_target.stat().st_size > 0
    assert map_target.exists() and map_target.stat().st_size > 0


def test_run_with_no_viz(tmp_path):
    out_dir = tmp_path / "wro2026"
    map_target = tmp_path / "map.png"
    rc = build_routes._run(out_dir=out_dir, map_out=map_target, no_viz=True)
    assert rc == 0
    assert (out_dir / "routes.h").exists()
    assert (out_dir / "routes.cpp").exists()
    assert not map_target.exists()


def test_run_with_invalid_field_data_returns_two(tmp_path, monkeypatch):
    bad = _clone_field_data()
    bad.LINE_SEGMENTS = [("A", "ZZZ")]
    monkeypatch.setattr("tools.path_planner.build_routes.field_data", bad)
    out_dir = tmp_path / "wro2026"
    rc = build_routes._run(out_dir=out_dir, no_viz=True)
    assert rc == 2
    assert not (out_dir / "routes.h").exists()
    assert not (out_dir / "routes.cpp").exists()


def test_run_idempotent(tmp_path):
    out_dir = tmp_path / "out"
    assert build_routes._run(out_dir=out_dir, no_viz=True) == 0
    h1 = (out_dir / "routes.h").read_bytes()
    c1 = (out_dir / "routes.cpp").read_bytes()
    assert build_routes._run(out_dir=out_dir, no_viz=True) == 0
    h2 = (out_dir / "routes.h").read_bytes()
    c2 = (out_dir / "routes.cpp").read_bytes()
    assert h1 == h2
    assert c1 == c2


def test_run_verbose_logs_to_stderr(tmp_path, capsys):
    out_dir = tmp_path / "out"
    rc = build_routes._run(out_dir=out_dir, no_viz=True, verbose=True)
    assert rc == 0
    err = capsys.readouterr().err
    assert "validating" in err
    assert "building graph" in err
    assert "computing" in err
    assert "emitting" in err


def test_run_emits_populated_pivot_turns_table(tmp_path, monkeypatch):
    """End-to-end: _run on a one-pivot field emits a populated PIVOT_TURNS table.

    Every shipped golden (routes.cpp + the toy golden) has N_PIVOT_TURNS == 0,
    and the header-writer pivot tests drive ``emit_routes`` directly. This closes
    that gap: it runs the real validate -> build_graph -> build_pivot_turns ->
    emit_routes orchestration on a synthetic one-pivot field and asserts the
    emitted routes.{h,cpp} carry the single derived record.
    """
    from tools.path_planner.tests.fixtures import pivot_field_data

    field = pivot_field_data.make_dead_reckon_pivot_field()
    # The orchestrator/emitter may getattr EDGES; pivots live in PIVOT_TURNS.
    field.EDGES = []
    monkeypatch.setattr("tools.path_planner.build_routes.field_data", field)
    # _run calls build_pivot_turns(graph, track_cm) WITHOUT pivot_turns=, so it
    # reads the module-global field_data.PIVOT_TURNS -- point that at the same
    # triple the fixture graph carries so the record is derived against it.
    monkeypatch.setattr(real_field_data, "PIVOT_TURNS", field.PIVOT_TURNS, raising=False)

    out_dir = tmp_path / "out"
    rc = build_routes._run(out_dir=out_dir, no_viz=True)
    assert rc == 0

    header = (out_dir / "routes.h").read_text(encoding="utf-8")
    cpp = (out_dir / "routes.cpp").read_text(encoding="utf-8")
    # One authored pivot (U->V->W, a -90 deg right turn) derived end to end.
    assert "constexpr uint8_t N_PIVOT_TURNS = 1;" in header
    assert "extern const PivotTurn PIVOT_TURNS[1];" in header
    assert "TurnMode::PIVOT_RIGHT" in cpp


def test_run_disconnected_node_warns_but_succeeds(tmp_path, monkeypatch, capsys):
    # Use the stable toy fixture, not the live field_data module: in the toy
    # layout node "G" is a blind node whose only edge is the BLIND_EDGES pair
    # ("B", "G"). Emptying BLIND_EDGES therefore isolates "G" deterministically,
    # so build_graph's "no incident edges" UserWarning fires regardless of how
    # the editor-mutable field_data.json evolves (trace #71).
    fd = _clone_toy_field_data()
    fd.BLIND_EDGES = []
    monkeypatch.setattr("tools.path_planner.build_routes.field_data", fd)
    out_dir = tmp_path / "out"
    with warnings.catch_warnings():
        warnings.simplefilter("default")
        rc = build_routes._run(out_dir=out_dir, no_viz=True)
    captured = capsys.readouterr()
    assert rc == 0, f"disconnected node should warn, not fail; stderr={captured.err!r}"
    assert (out_dir / "routes.h").exists()
    assert (out_dir / "routes.cpp").exists()
    assert "WARN:" in captured.err
    # The isolation of "G" must be the cause -- guard against the warning
    # coming only from an unrelated incidental isolated node.
    assert "'G'" in captured.err
