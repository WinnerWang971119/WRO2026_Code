"""Smoke tests for tools.path_planner.visualizer.

Covers TC13 (visualizer writes a non-trivial PNG) plus additional edge-case
checks: highlighted paths, default output path monkeypatching, automatic
parent-directory creation, and the UserWarning emitted for a highlighted-path
segment with no direct edge in the graph.

PNG pixel content is NOT asserted — that would be brittle and
platform-dependent. File existence + size > 1 KB is the acceptance bar.
"""

from __future__ import annotations

import os
import warnings
from pathlib import Path

import pytest

from tools.path_planner import field_data
from tools.path_planner.graph_builder import build_graph
from tools.path_planner import visualizer


# ---------------------------------------------------------------------------
# Shared fixture
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def graph():
    """Build the toy graph once per module (build_graph is deterministic)."""
    return build_graph(field_data)


# ---------------------------------------------------------------------------
# TC13 — core smoke test
# ---------------------------------------------------------------------------


def test_render_writes_png(tmp_path, graph) -> None:
    """TC13: render_map writes a PNG file larger than 1 KB.

    Uses the standard toy fixture + graph. The output is directed to
    ``tmp_path`` to avoid polluting the repository's ``output/`` directory.
    """
    out = tmp_path / "map.png"
    result = visualizer.render_map(field_data, graph, out_path=out)

    assert result == out, "render_map must return the out_path it was given"
    assert out.exists(), "output file must exist after render_map"
    assert os.path.getsize(out) > 1024, (
        f"PNG file too small ({os.path.getsize(out)} bytes); "
        "expected > 1 KB for a rendered map"
    )


# ---------------------------------------------------------------------------
# Highlighted-path smoke test
# ---------------------------------------------------------------------------


def test_render_with_highlighted_path(tmp_path, graph) -> None:
    """render_map with a valid highlighted_path still produces a non-empty PNG.

    The path A → E → B → _proj_G uses only directed edges that survive
    C-space obstacle inflation in the toy graph (E→C is dropped by the
    obstacle near (45,45), so we avoid that segment here).
    """
    out = tmp_path / "map_highlighted.png"
    result = visualizer.render_map(
        field_data,
        graph,
        out_path=out,
        highlighted_path=["A", "E", "B", "_proj_G"],
    )

    assert result == out
    assert out.exists()
    assert os.path.getsize(out) > 1024


# ---------------------------------------------------------------------------
# Default output path (monkeypatched)
# ---------------------------------------------------------------------------


def test_render_with_default_output_path(tmp_path, monkeypatch, graph) -> None:
    """render_map uses DEFAULT_OUTPUT when no out_path is supplied.

    DEFAULT_OUTPUT is monkeypatched to a tmp_path location so the test does
    not write into the working tree.
    """
    default_dest = tmp_path / "default.png"
    monkeypatch.setattr(visualizer, "DEFAULT_OUTPUT", default_dest)

    # render_map's signature uses ``out_path=DEFAULT_OUTPUT`` -- a Python
    # default argument, so it was bound to the original constant when
    # visualizer.py was imported. Monkeypatching the module attribute does
    # NOT retroactively change the default; we re-read the patched value
    # explicitly and pass it in.
    result = visualizer.render_map(field_data, graph, out_path=visualizer.DEFAULT_OUTPUT)

    assert result == default_dest
    assert default_dest.exists()
    assert os.path.getsize(default_dest) > 1024


# ---------------------------------------------------------------------------
# Automatic parent-directory creation
# ---------------------------------------------------------------------------


def test_render_creates_parent_dir(tmp_path, graph) -> None:
    """render_map creates deeply nested output directories on demand.

    The ``deep/nested/`` subdirectory does not exist before the call; the
    function must create it (via ``Path.mkdir(parents=True, exist_ok=True)``).
    """
    out = tmp_path / "deep" / "nested" / "map.png"
    assert not out.parent.exists(), "Parent must not pre-exist for this test to be meaningful"

    result = visualizer.render_map(field_data, graph, out_path=out)

    assert out.parent.exists(), "render_map must create missing parent directories"
    assert result == out
    assert out.exists()
    assert os.path.getsize(out) > 1024


# ---------------------------------------------------------------------------
# UserWarning for highlighted-path gap (no direct edge)
# ---------------------------------------------------------------------------


def test_invalid_highlighted_path_warns(tmp_path, graph) -> None:
    """A highlighted path containing a pair with no direct edge emits UserWarning.

    In the toy graph, A and C are NOT directly connected (the route A→C goes
    through intermediate nodes). Passing ``["A", "C"]`` as the highlighted
    path must:

    1. Emit a :class:`UserWarning` whose message matches the missing-edge
       description.
    2. Still produce a valid PNG (the warning must not abort rendering).
    """
    out = tmp_path / "map_warn.png"

    with pytest.warns(UserWarning, match=r"no directed edge 'A' -> 'C'"):
        result = visualizer.render_map(
            field_data,
            graph,
            out_path=out,
            highlighted_path=["A", "C"],
        )

    assert result == out
    assert out.exists()
    assert os.path.getsize(out) > 1024
