"""Tests for tools.path_planner.planner (PR3 schema)."""

from __future__ import annotations

import copy
import math
import warnings
from pathlib import Path
from types import SimpleNamespace

import networkx as nx
import pytest

from tools.path_planner.tests.fixtures import toy_field_data as real_field_data
from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, build_graph
from tools.path_planner.pipeline_config import TurnMode
from tools.path_planner.planner import (
    build_pivot_turns,
    compute_all_pairs_next_hop,
    compute_path_cost,
    compute_pivot_compensation,
)

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


def _make_blind_vs_line_graph() -> nx.DiGraph:
    """Three-node graph for the line-vs-blind cost-tradeoff test.

    Layout::

        A --[BLIND, 10cm]--> B
        A --[LINE, 10cm]--> C --[LINE, 12cm]--> B
    """
    g = nx.DiGraph()
    for name, (x, y) in (("A", (0.0, 0.0)), ("B", (20.0, 0.0)), ("C", (10.0, 5.0))):
        g.add_node(name, x_cm=x, y_cm=y, type="line")
    g.add_edge("A", "B", type=EDGE_BLIND, distance_cm=10.0, heading_deg=0.0)
    g.add_edge("A", "C", type=EDGE_LINE, distance_cm=10.0, heading_deg=0.0)
    g.add_edge("C", "B", type=EDGE_LINE, distance_cm=12.0, heading_deg=0.0)
    return g


def test_self_loop_sentinel() -> None:
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, real_field_data.COST_BLIND_MULTIPLIER)
    for node in g.nodes():
        assert next_hop[node][node] == node


def test_dijkstra_prefers_line_over_blind() -> None:
    g = _make_blind_vs_line_graph()
    table_penalised = compute_all_pairs_next_hop(g, cost_blind_multiplier=5.0)
    assert table_penalised["A"]["B"] == "C"
    table_unpenalised = compute_all_pairs_next_hop(g, cost_blind_multiplier=1.0)
    assert table_unpenalised["A"]["B"] == "B"


def test_unreachable_pair_is_none() -> None:
    """Removing the blind edge to G leaves G unreachable from every other node."""
    fd = _clone_field_data()
    fd.BLIND_EDGES = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(g, fd.COST_BLIND_MULTIPLIER)
    for source in g.nodes():
        if source == "G":
            assert next_hop["G"]["G"] == "G"
            continue
        assert next_hop[source]["G"] is None


def test_next_hop_table_has_every_pair() -> None:
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, real_field_data.COST_BLIND_MULTIPLIER)
    nodes = set(g.nodes())
    assert set(next_hop.keys()) == nodes
    for source, row in next_hop.items():
        assert set(row.keys()) == nodes


def test_compute_path_cost_line_edge() -> None:
    g = build_graph(real_field_data)
    # A=(10,10), B=(200,10): straight line 190 cm.
    expected = 190.0
    cost = compute_path_cost(g, ["A", "B"], real_field_data.COST_BLIND_MULTIPLIER)
    assert cost == pytest.approx(expected, abs=1e-6)


def test_compute_path_cost_blind_edge_multiplier() -> None:
    g = build_graph(real_field_data)
    edge = g.edges["B", "G"]
    assert edge["type"] == EDGE_BLIND
    expected = edge["distance_cm"] * real_field_data.COST_BLIND_MULTIPLIER
    actual = compute_path_cost(g, ["B", "G"], real_field_data.COST_BLIND_MULTIPLIER)
    assert actual == pytest.approx(expected, abs=1e-6)


def test_compute_path_cost_zero_length_path() -> None:
    g = build_graph(real_field_data)
    assert compute_path_cost(g, ["A"], real_field_data.COST_BLIND_MULTIPLIER) == 0.0
    assert compute_path_cost(g, [], real_field_data.COST_BLIND_MULTIPLIER) == 0.0


def test_compute_path_cost_missing_edge_raises() -> None:
    g = build_graph(real_field_data)
    # A and G have no direct edge (G is reached via B).
    assert not g.has_edge("A", "G")
    with pytest.raises(KeyError):
        compute_path_cost(g, ["A", "G"], real_field_data.COST_BLIND_MULTIPLIER)


# -- pivot compensation tests (T13) ------------------------------------------


def test_pivot_compensation_zero_turn_no_displacement():
    """A 0 deg pivot has no forward over-travel: delta == 0 (new 2-arg scalar)."""
    from tools.path_planner.planner import compute_pivot_compensation

    delta = compute_pivot_compensation(turn_deg=0.0, track_cm=15.0)
    assert delta == pytest.approx(0.0)


def test_pivot_compensation_90deg_with_track_15cm_yields_75cm_reduction():
    """90 deg pivot, TRACK_CM=15 -> delta = (15/2)*tan(45 deg) = 7.5 cm.

    The new closed form is delta = (track_cm / 2) * tan(|theta| / 2). At
    theta == 90 deg this is (15/2) * tan(45 deg) = 7.5 * 1.0 = 7.5 cm -- the
    one angle where it still agrees with the old chord-decomposition value.
    """
    from tools.path_planner.planner import compute_pivot_compensation

    delta = compute_pivot_compensation(turn_deg=90.0, track_cm=15.0)
    assert delta == pytest.approx(7.5, abs=1e-6)


def test_pivot_compensation_neg_90deg_yields_same_magnitude():
    """Negative turn (right pivot) yields same magnitude as positive turn.

    Only |turn_deg| enters the formula, so a -90 deg right pivot over-travels
    by the same delta as a +90 deg left pivot.
    """
    from tools.path_planner.planner import compute_pivot_compensation

    pos = compute_pivot_compensation(+90.0, 15.0)
    neg = compute_pivot_compensation(-90.0, 15.0)
    assert pos == pytest.approx(neg)
    assert pos == pytest.approx(7.5, abs=1e-6)


def test_pivot_compensation_grows_monotonically_with_angle():
    """delta increases monotonically with |theta| over (0, 180).

    Replaces the old 180-deg clamp test: the helper no longer clamps a
    following-edge distance (it returns a bare scalar), and the near-180
    divergence is the validator's fail-fast (T7), not this helper's concern.
    What stays true and worth pinning is that (track/2)*tan(|theta|/2) is
    strictly increasing in |theta| on (0, 180), so a sharper pivot always
    over-travels further.
    """
    from tools.path_planner.planner import compute_pivot_compensation

    track_cm = 18.0
    angles = [10.0, 30.0, 60.0, 90.0, 120.0, 150.0, 170.0]
    deltas = [compute_pivot_compensation(theta, track_cm) for theta in angles]
    assert deltas == sorted(deltas)
    # Strictly increasing: no two adjacent angles collapse to the same delta.
    for prev, curr in zip(deltas[:-1], deltas[1:]):
        assert curr > prev
    # The sign of the turn is irrelevant -- a right pivot grows identically.
    neg_deltas = [compute_pivot_compensation(-theta, track_cm) for theta in angles]
    assert neg_deltas == pytest.approx(deltas)


def test_pivot_compensation_tangent_form_matches_expected_values():
    """TC1: delta == (T/2)*tan(theta/2) at theta = 0/60/90/120 with T = 18.

    Locks the corrected closed form against hand-computed values and confirms
    it equals the OLD chord-decomposition estimate (T*sin^2(theta/2)) ONLY at
    theta == 90 deg (both = 0.5*T) and DIFFERS at 60 / 120 deg, where the old
    form gave 0.25*T / 0.75*T but the tangent form gives ~0.2887*T / ~0.8660*T.
    """
    from tools.path_planner.planner import compute_pivot_compensation

    track_cm = 18.0
    half = track_cm / 2.0

    # theta = 0 -> tan(0) = 0 -> delta = 0.
    assert compute_pivot_compensation(0.0, track_cm) == pytest.approx(0.0)

    # theta = 90 -> tan(45) = 1 -> delta = 0.5 * T = 9.0. Old form agrees here.
    expected_90 = half * math.tan(math.radians(45.0))
    delta_90 = compute_pivot_compensation(90.0, track_cm)
    assert delta_90 == pytest.approx(expected_90, abs=1e-9)
    assert delta_90 == pytest.approx(0.5 * track_cm, abs=1e-9)
    old_90 = track_cm * math.sin(math.radians(45.0)) ** 2
    assert delta_90 == pytest.approx(old_90, abs=1e-9)  # equal at 90 deg only

    # theta = 60 -> tan(30) ~= 0.5774 -> delta = T/2 * tan(30) ~= 0.2887 * T.
    expected_60 = half * math.tan(math.radians(30.0))
    delta_60 = compute_pivot_compensation(60.0, track_cm)
    assert delta_60 == pytest.approx(expected_60, abs=1e-9)
    assert delta_60 == pytest.approx(0.288675 * track_cm, abs=1e-4)
    old_60 = track_cm * math.sin(math.radians(30.0)) ** 2  # = 0.25 * T = 4.5
    assert delta_60 != pytest.approx(old_60, abs=1e-3)  # differs from old form

    # theta = 120 -> tan(60) ~= 1.7321 -> delta = T/2 * tan(60) ~= 0.8660 * T.
    expected_120 = half * math.tan(math.radians(60.0))
    delta_120 = compute_pivot_compensation(120.0, track_cm)
    assert delta_120 == pytest.approx(expected_120, abs=1e-9)
    assert delta_120 == pytest.approx(0.866025 * track_cm, abs=1e-4)
    old_120 = track_cm * math.sin(math.radians(60.0)) ** 2  # = 0.75 * T = 13.5
    assert delta_120 != pytest.approx(old_120, abs=1e-3)  # differs from old form


def test_compute_pivot_compensation_track_drives_delta(tmp_path):
    """TC3: an edited Config::TRACK_MM flows through parse -> delta.

    The planner single-sources the chassis track from ``Config::TRACK_MM``
    (issue #44). Parsing a temp header with a different ``TRACK_MM`` must
    yield the matching cm value, and feeding that into the delta helper must
    reflect the edited track: ``TRACK_MM = 200.0f`` -> 20.0 cm -> delta@90 =
    10.0 cm. Also confirms no second hard-coded track literal survives in
    ``planner.py`` (the stale ``TRACK_CM = 15`` constant is gone).
    """
    from tools.path_planner import planner as planner_module
    from tools.path_planner.planner import (
        compute_pivot_compensation,
        parse_track_cm_from_config,
    )

    config_path = tmp_path / "Config.h"
    config_path.write_text(
        "namespace Config {\n"
        "    constexpr float TRACK_MM = 200.0f;\n"
        "}\n",
        encoding="utf-8",
    )
    track_cm = parse_track_cm_from_config(config_path)
    assert track_cm == pytest.approx(20.0)
    # delta at 90 deg = (track/2) * tan(45) = track/2 = 10.0 cm for a 20 cm track.
    assert compute_pivot_compensation(90.0, track_cm) == pytest.approx(10.0, abs=1e-6)

    # No second hard-coded track literal survives in the planner source: the
    # stale ``TRACK_CM = 15`` / ``= 15.0`` assignment was removed when the
    # value moved to the firmware header.
    source = Path(planner_module.__file__).read_text(encoding="utf-8")
    assert "TRACK_CM = 15" not in source
    assert "TRACK_CM = 15.0" not in source


# -- TC1 / TC2: per-turn pivot derivation (build_pivot_turns) ---------------
#
# A single-wheel pivot is a property of a TURN, not an edge: a 3-node triple
# ``(u, v, w)`` declared in ``field_data.PIVOT_TURNS``. ``build_pivot_turns``
# reads that module-level list, so each derivation test monkeypatches it onto
# the real ``field_data`` module (restored by the autouse fixture below) and
# hands a hand-built 3-node graph whose two edge headings pin ``theta`` exactly.


@pytest.fixture
def patch_pivot_turns(monkeypatch):
    """Return a setter that points ``field_data.PIVOT_TURNS`` at a triple list.

    ``build_pivot_turns`` reads the pivot triples off the module-level
    ``field_data.PIVOT_TURNS`` (deferred import), not off the graph, so a
    derivation test must replace that attribute. ``monkeypatch.setattr``
    restores the original (empty) list after the test, keeping the real
    field's pivot-free state intact for every other test in the suite.
    """
    from tools.path_planner import field_data

    def _set(triples) -> None:
        monkeypatch.setattr(field_data, "PIVOT_TURNS", list(triples), raising=False)

    return _set


def _triple_graph(theta_deg: float) -> nx.DiGraph:
    """Three-node A->B->C graph: ``u->v`` heads 0 deg, ``v->w`` heads ``theta``.

    The incoming leg ``A->B`` is fixed at heading 0 deg and the outgoing leg
    ``B->C`` at ``theta_deg`` (mod 360), so the signed turn at ``B`` is exactly
    ``normalize(theta - 0) == theta``. Distances are inert for the geometry
    derivation, so both legs are a flat 100 cm.
    """
    g = nx.DiGraph()
    for name in ("A", "B", "C"):
        g.add_node(name)
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=100.0, heading_deg=0.0)
    g.add_edge("B", "C", type=EDGE_LINE, distance_cm=100.0, heading_deg=theta_deg % 360.0)
    return g


@pytest.mark.parametrize(
    "theta, expected_wheel",
    [
        (90.0, TurnMode.PIVOT_LEFT),
        (-90.0, TurnMode.PIVOT_RIGHT),
        (60.0, TurnMode.PIVOT_LEFT),
        (120.0, TurnMode.PIVOT_LEFT),
    ],
)
def test_build_pivot_turns_derives_trim_and_wheel(patch_pivot_turns, theta, expected_wheel):
    """TC1: build_pivot_turns derives trim + wheel from the turn geometry.

    For a 3-node triple whose signed turn at ``v`` is ``theta`` and a track of
    18 cm, the resolved record must carry the over-travel
    ``trim_cm == (track/2)*tan(|theta|/2)`` and the wheel from ``sign(theta)``
    (CCW / theta > 0 -> PIVOT_LEFT, CW / theta < 0 -> PIVOT_RIGHT). The
    expected trim is taken from the canonical
    :func:`compute_pivot_compensation` helper so the test never re-inlines the
    closed form.
    """
    track_cm = 18.0
    patch_pivot_turns([("A", "B", "C")])
    records = build_pivot_turns(_triple_graph(theta), track_cm=track_cm)

    assert len(records) == 1
    record = records[0]
    assert (record.u, record.v, record.w) == ("A", "B", "C")
    assert record.wheel == expected_wheel
    # trim == (track/2)*tan(|theta|/2), pulled from the canonical helper.
    expected_trim = compute_pivot_compensation(theta, track_cm)
    assert record.trim_cm == pytest.approx(expected_trim, abs=1e-9)


def test_build_pivot_turns_skips_collinear_zero_turn(patch_pivot_turns):
    """TC1b: a collinear triple (theta == 0) is not a pivot and emits no record.

    A direct/bypass caller (the validator's raised min-angle floor rejects this
    in production) that declares collinear legs must not produce a spurious
    zero-trim record, and must never fall through the ``sign(theta)`` split to a
    default ``PIVOT_RIGHT``. ``build_pivot_turns`` skips the triple entirely.
    """
    patch_pivot_turns([("A", "B", "C")])
    records = build_pivot_turns(_triple_graph(0.0), track_cm=18.0)
    assert records == []


def test_build_pivot_turns_reuses_compute_pivot_compensation(patch_pivot_turns):
    """TC2: build_pivot_turns reuses compute_pivot_compensation verbatim.

    The derived ``trim_cm`` must be byte-identical to a direct call of
    :func:`compute_pivot_compensation` at the same ``theta`` / ``track_cm`` --
    proving the planner has exactly ONE over-travel formula and that
    ``build_pivot_turns`` does not carry a second, possibly-divergent copy.
    Tested across several angles, including the 90 deg point where the old
    chord form happened to agree and the 60 / 120 deg points where it did not.
    """
    track_cm = 18.0
    for theta in (45.0, 60.0, 90.0, 120.0, -75.0):
        patch_pivot_turns([("A", "B", "C")])
        records = build_pivot_turns(_triple_graph(theta), track_cm=track_cm)
        assert len(records) == 1
        # Value-identical to a direct helper call -- no second formula.
        direct = compute_pivot_compensation(theta, track_cm)
        assert records[0].trim_cm == direct


# -- Codex P1 regression: directional overrides must not mirror to reverse --


def test_direction_override_not_mirrored_to_reverse_edge():
    """TC15 / Codex P1 regression: EDGES `direction` is direction-dependent.

    An override on (A, B) with direction='rear' must produce
    direction_override = FORCE_REVERSE on the (A, B) directed edge ONLY.
    The reverse (B, A) edge must retain DEFAULT -- mirroring would silently
    flip the chassis on backward traversal of an unrelated hop.
    """
    fd = SimpleNamespace(
        NODES={"A": (0.0, 0.0), "B": (100.0, 0.0)},
        NODE_TYPES={"A": "line", "B": "line"},
        LINE_SEGMENTS=[("A", "B")],
        BLIND_EDGES=[],
        OBSTACLES=[],
        ROBOT_RADIUS_CM=1.0,
        COST_BLIND_MULTIPLIER=1.0,
        EDGES=[{"type": EDGE_LINE, "u": "A", "v": "B", "distance_cm": 100.0,
                "heading_deg": 0.0, "direction": "rear"}],
        START_NODE="A",
        START_HEADING_DEG=0.0,
        FIELD_MM=(200, 200),
        MAP_PNG="maps/map.png",
    )
    graph = build_graph(fd)
    fwd = graph.edges["A", "B"]
    rev = graph.edges["B", "A"]
    # Forward edge has the override (integer 2 = FORCE_REVERSE).
    assert fwd["direction_override"] == 2
    # Reverse edge must NOT have the override copied (absent or DEFAULT==0).
    assert rev.get("direction_override", 0) == 0
