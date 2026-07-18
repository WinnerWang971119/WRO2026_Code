"""Tests for tools.path_planner.build_routes (CLI orchestrator, PR3)."""

from __future__ import annotations

import copy
import warnings
from pathlib import Path
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner import build_routes
from tools.path_planner import field_data as real_field_data
from tools.path_planner import header_writer
from tools.path_planner.graph_builder import EDGE_LINE
from tools.path_planner.tests.fixtures import toy_field_data

_FIELD_ATTRS = (
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "OBSTACLES",
    "ROBOT_RADIUS_CM",
    # Blind-decay knobs (replace the old COST_BLIND_MULTIPLIER; PR3 / T22).
    # _run builds BlindDecayParams.from_field_data, so a cloned/monkeypatched
    # field-data namespace must carry all four or the planner call AttributeErrors.
    "BLIND_DECAY_ENABLED",
    "BLIND_DECAY_K",
    "BLIND_DECAY_EXPONENT",
    "BLIND_DECAY_D_REF",
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


# ---------------------------------------------------------------------------
# Issue #79 / ADR-047: TC19 build_routes freshness.
#
# The plan's CI gate runs build_routes then `git diff --exit-code`. T2 edited the
# hashed field_data.py, so the committed routes.{h,cpp} Source-SHA stamp
# legitimately changed (the orchestrator regenerates + commits routes in a later
# phase). These tests therefore assert the emitter is DETERMINISTIC against the
# REAL committed field -- re-running build_routes produces zero diff -- WITHOUT
# hardcoding the old SHA. This is exactly what `git diff --exit-code` checks once
# routes are regenerated: a second regeneration matches the first byte-for-byte.
# ---------------------------------------------------------------------------


def test_tc19_build_routes_is_deterministic_on_real_field(tmp_path) -> None:
    """TC19: regenerating routes.{h,cpp} from the real field is byte-stable.

    Runs the full validate -> build_graph -> compute_next_hop -> emit pipeline on
    the committed ``field_data`` twice into separate output dirs and asserts the
    two emissions are byte-identical. This is the deterministic-emitter guarantee
    the `git diff --exit-code` freshness gate relies on: once the routes are
    regenerated and committed, a re-run leaves no diff. No SHA is hardcoded, so
    the test stays robust across the legitimate T2 stamp change.
    """
    out_a = tmp_path / "a"
    out_b = tmp_path / "b"
    assert build_routes._run(out_dir=out_a, no_viz=True) == 0
    assert build_routes._run(out_dir=out_b, no_viz=True) == 0
    assert (out_a / "routes.h").read_bytes() == (out_b / "routes.h").read_bytes()
    assert (out_a / "routes.cpp").read_bytes() == (out_b / "routes.cpp").read_bytes()


def test_tc19_committed_routes_match_a_fresh_regeneration(tmp_path) -> None:
    """TC19: the committed routes.{h,cpp} equal a fresh regeneration (freshness gate).

    Mirrors the CI `git diff --exit-code` check directly: regenerate routes into a
    temp dir from the real field and compare byte-for-byte against the committed
    ``WRO2026/routes.{h,cpp}``. The always-green determinism guarantee is
    test_tc19_build_routes_is_deterministic_on_real_field. Skips gracefully if the
    committed files are absent so the suite stays green on a partial checkout.
    """
    repo_root = Path(__file__).resolve().parents[3]
    committed_h = repo_root / header_writer.DEFAULT_HEADER_PATH
    committed_cpp = repo_root / header_writer.DEFAULT_SOURCE_PATH
    if not (committed_h.exists() and committed_cpp.exists()):
        pytest.skip(f"committed {committed_h} / {committed_cpp} not present")

    out_dir = tmp_path / "fresh"
    assert build_routes._run(out_dir=out_dir, no_viz=True) == 0

    fresh_h = (out_dir / "routes.h").read_bytes()
    fresh_cpp = (out_dir / "routes.cpp").read_bytes()
    # Normalize CRLF -> LF so an autocrlf=true Windows checkout of the committed
    # files does not spuriously diverge from the LF-only emitter output.
    assert fresh_h == committed_h.read_bytes().replace(b"\r\n", b"\n"), (
        f"committed {committed_h} is stale -- regenerate with "
        "`uv run python tools/path_planner/build_routes.py` and commit"
    )
    assert fresh_cpp == committed_cpp.read_bytes().replace(b"\r\n", b"\n"), (
        f"committed {committed_cpp} is stale -- regenerate with "
        "`uv run python tools/path_planner/build_routes.py` and commit"
    )


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


# ---------------------------------------------------------------------------
# TC12 (issue #156 / ADR-069 / AC12): the reverse-traversal lint. A biased LINE
# edge (mode=LEFT|RIGHT) whose reverse traversal is actually driven by a
# NEXT_HOP route, with NO companion authored entry biasing that reverse hop,
# is an authoring smell -- `find_unpaired_biased_line_reversals` returns one
# loud warning per such gap; it is silent when the reverse is paired (its own
# authored `mode`), unused (never in NEXT_HOP), or the edge is centered.
# ---------------------------------------------------------------------------


def _line_reversal_graph(fwd_mode, rev_mode) -> nx.DiGraph:
    """Build a 2-node directed graph A<->B with LINE edges carrying the given modes.

    ``fwd_mode`` / ``rev_mode`` are the raw ``mode`` attribute for A->B / B->A
    (a bare tag string, or ``None`` for an unbiased edge that carries no
    ``mode`` key at all -- mirroring how graph_builder omits the key).
    """
    g = nx.DiGraph()
    fwd_attrs = {"type": EDGE_LINE}
    if fwd_mode is not None:
        fwd_attrs["mode"] = fwd_mode
    rev_attrs = {"type": EDGE_LINE}
    if rev_mode is not None:
        rev_attrs["mode"] = rev_mode
    g.add_edge("A", "B", **fwd_attrs)
    g.add_edge("B", "A", **rev_attrs)
    return g


def test_tc12_lint_warns_for_used_unpaired_biased_reverse() -> None:
    """A biased A->B whose reverse B->A is NEXT_HOP-driven + uncompanioned warns."""
    graph = _line_reversal_graph(fwd_mode="LEFT", rev_mode=None)
    # From B, routing to destination A steps to A -> the reverse hop B->A IS used.
    next_hop = {"B": {"A": "A"}, "A": {"B": "B"}}
    messages = build_routes.find_unpaired_biased_line_reversals(graph, next_hop)
    assert len(messages) == 1, messages
    msg = messages[0]
    assert "'A' -> 'B'" in msg
    assert "LEFT" in msg
    assert "'B' -> 'A'" in msg


def test_tc12_lint_silent_when_reverse_is_paired() -> None:
    """A companion reverse override (B->A carries its own mode) suppresses the warning."""
    graph = _line_reversal_graph(fwd_mode="LEFT", rev_mode="RIGHT")
    next_hop = {"B": {"A": "A"}, "A": {"B": "B"}}
    assert build_routes.find_unpaired_biased_line_reversals(graph, next_hop) == []


def test_tc12_lint_silent_when_reverse_unused() -> None:
    """An uncompanioned biased reverse that no NEXT_HOP route drives is silent."""
    graph = _line_reversal_graph(fwd_mode="LEFT", rev_mode=None)
    # B never steps to A for any destination -> the reverse hop is unused.
    next_hop = {"B": {}, "A": {"B": "B"}}
    assert build_routes.find_unpaired_biased_line_reversals(graph, next_hop) == []


def test_tc12_lint_silent_for_centered_edge() -> None:
    """A centered (MIDDLE) or unbiased forward edge is never a lint target."""
    # Explicit MIDDLE forward -> not a biased mode, so no warning even when used.
    graph_middle = _line_reversal_graph(fwd_mode="MIDDLE", rev_mode=None)
    next_hop = {"B": {"A": "A"}, "A": {"B": "B"}}
    assert (
        build_routes.find_unpaired_biased_line_reversals(graph_middle, next_hop)
        == []
    )
    # No mode at all -> likewise silent.
    graph_none = _line_reversal_graph(fwd_mode=None, rev_mode=None)
    assert (
        build_routes.find_unpaired_biased_line_reversals(graph_none, next_hop)
        == []
    )
