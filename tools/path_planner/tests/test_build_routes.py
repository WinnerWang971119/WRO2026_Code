"""Tests for tools.path_planner.build_routes (CLI orchestrator, PR3)."""

from __future__ import annotations

import copy
import warnings
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.path_planner import build_routes
from tools.path_planner import field_data as real_field_data

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


def _clone_field_data() -> SimpleNamespace:
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
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


def test_run_disconnected_node_warns_but_succeeds(tmp_path, monkeypatch, capsys):
    fd = _clone_field_data()
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
