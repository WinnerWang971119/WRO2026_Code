"""Tests for tools.path_planner.visualizer (PR3)."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from tools.path_planner import field_data
from tools.path_planner.graph_builder import build_graph
from tools.path_planner import visualizer


@pytest.fixture(scope="module")
def graph():
    return build_graph(field_data)


def test_render_writes_png(tmp_path, graph):
    out = tmp_path / "map.png"
    result = visualizer.render_map(field_data, graph, out_path=out)
    assert result == out
    assert out.exists()
    assert os.path.getsize(out) > 1024


def test_render_with_highlighted_path(tmp_path, graph):
    out = tmp_path / "map_highlighted.png"
    result = visualizer.render_map(
        field_data, graph,
        out_path=out,
        highlighted_path=["A", "E", "B", "G"],
    )
    assert result == out
    assert out.exists()
    assert os.path.getsize(out) > 1024


def test_render_creates_parent_dir(tmp_path, graph):
    out = tmp_path / "deep" / "nested" / "map.png"
    assert not out.parent.exists()
    result = visualizer.render_map(field_data, graph, out_path=out)
    assert out.parent.exists()
    assert result == out
    assert out.exists()
    assert os.path.getsize(out) > 1024


def test_invalid_highlighted_path_warns(tmp_path, graph):
    """A pair with no direct edge emits UserWarning but rendering succeeds."""
    out = tmp_path / "map_warn.png"
    with pytest.warns(UserWarning, match=r"no directed edge 'A' -> 'G'"):
        result = visualizer.render_map(
            field_data, graph,
            out_path=out,
            highlighted_path=["A", "G"],
        )
    assert result == out
    assert out.exists()
    assert os.path.getsize(out) > 1024
