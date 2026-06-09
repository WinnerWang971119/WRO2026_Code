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
from tools.path_planner.planner import (
    compute_all_pairs_next_hop,
    compute_path_cost,
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


def test_apply_pivot_compensations_skips_multi_successor():
    """A pivot landing with >1 forward out-edge is skipped, not raised.

    The out-degree precondition is now enforced by the validator (finding
    #36), so ``apply_pivot_compensations`` no longer raises on a mismatch:
    for direct / hand-built callers that bypass ``validate`` it skips the
    offending pivot edge, leaving every following edge uncompensated rather
    than mis-compensating an arbitrary branch. The back-edge to ``src`` is
    excluded from the count -- a bidirectional pair plus 2 real forward
    continuations is still a multi-successor landing.
    """
    from tools.path_planner.graph_builder import Pivot
    from tools.path_planner.planner import apply_pivot_compensations

    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_node("C")
    g.add_node("D")
    g.add_edge("A", "B", distance_cm=10.0, heading_deg=0.0, pivot=int(Pivot.LEFT))
    g.add_edge("B", "C", distance_cm=20.0, heading_deg=90.0)
    g.add_edge("B", "D", distance_cm=15.0, heading_deg=-90.0)

    # Must not raise; must not touch either successor's distance.
    apply_pivot_compensations(g, track_cm=15.0)
    assert g.edges["B", "C"]["distance_cm"] == pytest.approx(20.0)
    assert g.edges["B", "D"]["distance_cm"] == pytest.approx(15.0)


# -- Codex P1 regression: directional overrides must not mirror to reverse --


def test_emit_directed_pair_pivot_not_mirrored_to_reverse():
    """`pre_turn_mode` and `pivot` are direction-dependent and must stay forward-only.

    Codex P1 regression: mirroring these onto the reverse edge would steer
    the chassis with the wrong wheel pattern on backward traversal.
    """
    from tools.path_planner.graph_builder import EDGE_LINE, Pivot, _emit_directed_pair

    g = nx.DiGraph()
    g.add_node("U")
    g.add_node("V")
    _emit_directed_pair(
        g,
        "U",
        "V",
        pu=(0.0, 0.0),
        pv=(10.0, 0.0),
        edge_type=EDGE_LINE,
        options={
            "pivot": Pivot.LEFT,
            "pre_turn_mode": "PIVOT_LEFT",
            "primitive_mode": "NORMAL",
        },
    )
    forward = g.edges["U", "V"]
    reverse = g.edges["V", "U"]

    # Forward edge carries every override.
    assert forward.get("pivot") == int(Pivot.LEFT)
    assert forward.get("pre_turn_mode") == "PIVOT_LEFT"
    assert forward.get("primitive_mode") == "NORMAL"

    # Reverse edge must NOT inherit the directional overrides.
    assert "pivot" not in reverse
    assert "pre_turn_mode" not in reverse
    # `primitive_mode` is direction-symmetric (selects how to follow the
    # segment, not which way) so it stays mirrored.
    assert reverse.get("primitive_mode") == "NORMAL"


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


def test_apply_pivot_compensations_ignores_back_edge():
    """Bidirectional graph: pivot dst with back-edge + 1 forward edge must compensate.

    Codex P1 regression: counting all out-edges (including the back-edge
    to ``src``) makes pivot compensation unusable on the bidirectional
    graphs ``build_graph`` actually produces -- the landing node ``B`` must
    resolve to its single FORWARD continuation ``B->C`` (the back-edge
    ``B->A`` excluded) to measure the turn.

    Issue #44: the trim is now STORED on the pivot edge ``A->B`` as
    ``pivot_trim_cm`` rather than subtracted from the following edge's
    distance. The 90 deg pivot's over-travel delta = (15/2)*tan(45) = 7.5 cm
    for TRACK_CM=15, so ``A->B`` gains ``pivot_trim_cm == 7.5`` while every
    edge's ``distance_cm`` -- the following edge ``B->C`` included -- stays
    byte-identical to the input graph.
    """
    from tools.path_planner.graph_builder import Pivot
    from tools.path_planner.planner import apply_pivot_compensations

    g = nx.DiGraph()
    g.add_node("A")
    g.add_node("B")
    g.add_node("C")
    # Forward pivot edge A->B, plus the bidirectional back-edge B->A.
    g.add_edge("A", "B", distance_cm=10.0, heading_deg=0.0, pivot=int(Pivot.LEFT))
    g.add_edge("B", "A", distance_cm=10.0, heading_deg=180.0)
    # Single real continuation off B (plus the back-edge above).
    g.add_edge("B", "C", distance_cm=30.0, heading_deg=90.0)
    g.add_edge("C", "B", distance_cm=30.0, heading_deg=270.0)

    apply_pivot_compensations(g, track_cm=15.0)

    # The trim lands on the PIVOT edge A->B: 7.5 cm for TRACK_CM=15,
    # |theta|=90 deg, via delta = (15/2)*tan(45).
    assert g.edges["A", "B"]["pivot_trim_cm"] == pytest.approx(7.5, abs=1e-6)
    # The following edge B->C is NOT shortened -- its geometry is untouched.
    assert g.edges["B", "C"]["distance_cm"] == pytest.approx(30.0)
    assert "pivot_trim_cm" not in g.edges["B", "C"]
    # The back-edge B->A must NOT have been touched at all.
    assert g.edges["B", "A"]["distance_cm"] == pytest.approx(10.0)
    assert "pivot_trim_cm" not in g.edges["B", "A"]


def test_apply_pivot_compensations_idempotent_on_shared_following_edge():
    """Finding #94 (issue #44): two pivots sharing one following edge leave it intact.

    The #44 fix moves the trim off the shared following edge and onto each
    pivot edge as its own ``pivot_trim_cm``. The cross-pivot
    double-compensation finding #94 guarded against -- two pivots from
    non-adjacent sources both subtracting from one shared edge -- is now
    structurally impossible: the following edge ``M -> N`` is never mutated,
    so there is nothing to compound.

    Layout: two pivot edges ``P1 -> M`` and ``P2 -> M`` land on the SAME
    node ``M`` from non-adjacent sources. ``M`` has exactly ONE forward
    out-edge, the shared following edge ``M -> N`` (the pivots are
    one-directional, so ``M`` has no back-edge to either source and the
    per-pivot ``len(forward_out_edges) != 1`` check passes for both).

    For a 90 deg pivot (heading 0 deg entering ``M``, ``M -> N`` heading
    90 deg) with TRACK_CM=15 the over-travel delta = (15/2)*tan(45) = 7.5 cm.
    Each pivot edge carries its OWN 7.5 cm trim; ``M -> N`` keeps its full
    30.0 cm distance. Idempotency now means a second pass re-stores the same
    per-pivot scalar and never touches the following edge.
    """
    from tools.path_planner.graph_builder import Pivot
    from tools.path_planner.planner import apply_pivot_compensations

    g = nx.DiGraph()
    for name in ("P1", "P2", "M", "N"):
        g.add_node(name)
    # Two pivot edges from non-adjacent sources, both landing on M, both
    # heading 0 deg. One-directional (no back-edge M->P1 / M->P2), so M's
    # only out-edge is the shared following edge M->N.
    g.add_edge("P1", "M", distance_cm=10.0, heading_deg=0.0, pivot=int(Pivot.LEFT))
    g.add_edge("P2", "M", distance_cm=12.0, heading_deg=0.0, pivot=int(Pivot.RIGHT))
    # The single shared following edge off M: heading 90 deg -> 90 deg turn.
    g.add_edge("M", "N", distance_cm=30.0, heading_deg=90.0)

    apply_pivot_compensations(g, track_cm=15.0)

    # Each pivot edge owns its 7.5 cm trim; the shared following edge is
    # untouched -- no shortening, no double application possible.
    assert g.edges["P1", "M"]["pivot_trim_cm"] == pytest.approx(7.5, abs=1e-6)
    assert g.edges["P2", "M"]["pivot_trim_cm"] == pytest.approx(7.5, abs=1e-6)
    assert g.edges["M", "N"]["distance_cm"] == pytest.approx(30.0, abs=1e-6)
    assert "pivot_trim_cm" not in g.edges["M", "N"]

    # Calling again on the already-trimmed graph must re-store the same
    # per-pivot scalar and still leave the following edge alone.
    apply_pivot_compensations(g, track_cm=15.0)
    assert g.edges["P1", "M"]["pivot_trim_cm"] == pytest.approx(7.5, abs=1e-6)
    assert g.edges["P2", "M"]["pivot_trim_cm"] == pytest.approx(7.5, abs=1e-6)
    assert g.edges["M", "N"]["distance_cm"] == pytest.approx(30.0, abs=1e-6)


def test_compute_all_pairs_next_hop_compensates_input_graph_in_place():
    """TC9 / finding #42 (issue #44): pivot trim reaches the SHARED graph.

    ``compute_all_pairs_next_hop`` applies pivot compensation to its input
    ``graph`` in place (not to a discarded weighted copy), because the caller
    hands that same ``graph`` object to ``emit_routes`` immediately
    afterward. The issue #44 fix changes WHAT gets written: the trim now
    lands on the pivot edge ``A->B`` as ``pivot_trim_cm``, and NO edge's
    ``distance_cm`` is mutated -- so routes.cpp emits the true geometric
    distance on every edge plus the per-pivot trim scalar.

    Bidirectional layout mirroring
    :func:`test_apply_pivot_compensations_ignores_back_edge`: A->B is a
    90 deg LEFT pivot, B has exactly one forward continuation B->C
    (the back-edge B->A is excluded from the out-degree count). This test
    omits ``track_cm`` so the planner parses the live ``Config::TRACK_MM``
    (180 mm = 18.0 cm) -- the same value the production pipeline drives with.
    For an 18.0 cm track and a 90 deg pivot the over-travel
    delta = (18/2)*tan(45) = 9.0 cm, stored on A->B; B->C keeps its full
    30.0 cm distance.
    """
    from tools.path_planner.graph_builder import Pivot

    g = nx.DiGraph()
    for name, (x, y) in (("A", (0.0, 0.0)), ("B", (10.0, 0.0)), ("C", (10.0, 30.0))):
        g.add_node(name, x_cm=x, y_cm=y, type="line")
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=10.0, heading_deg=0.0,
               pivot=int(Pivot.LEFT))
    g.add_edge("B", "A", type=EDGE_LINE, distance_cm=10.0, heading_deg=180.0)
    g.add_edge("B", "C", type=EDGE_LINE, distance_cm=30.0, heading_deg=90.0)
    g.add_edge("C", "B", type=EDGE_LINE, distance_cm=30.0, heading_deg=270.0)

    # Drive the FULL planner flow (not apply_pivot_compensations directly).
    # No track_cm -> the planner reads Config::TRACK_MM (18.0 cm).
    compute_all_pairs_next_hop(g, cost_blind_multiplier=1.0)

    # The trim is stored on the PIVOT edge A->B: delta = (18/2)*tan(45) = 9.0.
    assert g.edges["A", "B"]["pivot_trim_cm"] == pytest.approx(9.0, abs=1e-6)
    # The following edge B->C keeps its FULL geometric distance (the #44 fix):
    # it is no longer shortened, and carries no trim of its own.
    assert g.edges["B", "C"]["distance_cm"] == pytest.approx(30.0)
    assert "pivot_trim_cm" not in g.edges["B", "C"]
    # The pivot edge's own distance and the back-edge are untouched.
    assert g.edges["A", "B"]["distance_cm"] == pytest.approx(10.0)
    assert g.edges["B", "A"]["distance_cm"] == pytest.approx(10.0)
    # The back-edge B->A is not a pivot edge, so it never carries a trim.
    assert "pivot_trim_cm" not in g.edges["B", "A"]
