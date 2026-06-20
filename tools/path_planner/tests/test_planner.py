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
    BlindDecayParams,
    build_pivot_turns,
    compute_all_pairs_next_hop,
    compute_blind_distance_to_anchor,
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
    # Blind-decay knobs (replace the old COST_BLIND_MULTIPLIER; PR3 / T22).
    "BLIND_DECAY_ENABLED",
    "BLIND_DECAY_K",
    "BLIND_DECAY_EXPONENT",
    "BLIND_DECAY_D_REF",
    "START_NODE",
    "START_HEADING_DEG",
    "FIELD_MM",
    "MAP_PNG",
)

# The toy field's blind-decay params, reused so the migrated tests pin the
# planner against the same knobs the live fixture carries.
_TOY_DECAY = BlindDecayParams.from_field_data(real_field_data)

# A factor-1 (no-penalty) params object: ``k == 0`` reduces every blind edge
# to plain length, the AC14b regression anchor and the migration of the old
# "unpenalised multiplier == 1.0" assertions.
_NO_DECAY = BlindDecayParams(enabled=True, k=0.0, exponent=2.0, d_ref=50.0)


def _clone_field_data() -> SimpleNamespace:
    snapshot = {attr: copy.deepcopy(getattr(real_field_data, attr)) for attr in _FIELD_ATTRS}
    return SimpleNamespace(**snapshot)


def _walk_next_hop(next_hop, source: str, target: str) -> list:
    """Reconstruct the full ``source -> target`` path from a NEXT_HOP table.

    Repeatedly indexes ``next_hop[current][target]`` to step toward the goal,
    exactly as the firmware / sim walk it. Returns the inclusive node-name
    list ``[source, ..., target]``. Used by the AC15 fixtures to assert which
    nodes the chosen route threads.

    :param next_hop: an all-pairs ``{from: {to: next | None}}`` table.
    :param source: start node name.
    :param target: goal node name.
    :return: the inclusive path ``[source, ..., target]``.
    :raises ValueError: if ``target`` is unreachable from ``source``.
    """
    if source == target:
        return [source]
    path = [source]
    current = source
    while current != target:
        nxt = next_hop[current][target]
        if nxt is None:
            raise ValueError(f"{target!r} unreachable from {current!r}")
        path.append(nxt)
        current = nxt
    return path


# Decay params used by the AC15 routing fixtures. ``exponent`` / ``d_ref``
# match the toy field; only ``k`` is varied to flip the chosen route.
_AC15_D_REF: float = 50.0
_AC15_EXPONENT: float = 2.0
# Default k routes through the reset (penalty wins); lowered k takes the
# line-free shortcut. Derived from the fixture geometry below: the detour
# undercuts the shortcut iff k > 1/3 (see the fixture docstring), so 0.5 is
# decisively "default" and 0.05 decisively "lowered".
_AC15_K_DEFAULT: float = 0.5
_AC15_K_LOWERED: float = 0.05


def _params(k: float) -> BlindDecayParams:
    """Return a :class:`BlindDecayParams` at gain ``k`` (toy exponent / d_ref)."""
    return BlindDecayParams(
        enabled=True, k=k, exponent=_AC15_EXPONENT, d_ref=_AC15_D_REF
    )


def _add_blind_pair(g: nx.DiGraph, u: str, v: str, distance_cm: float) -> None:
    """Add a bidirectional BLIND edge pair ``u<->v`` (mirrors build_graph)."""
    g.add_edge(u, v, type=EDGE_BLIND, distance_cm=distance_cm, heading_deg=0.0)
    g.add_edge(v, u, type=EDGE_BLIND, distance_cm=distance_cm, heading_deg=180.0)


def _make_blind_chain_graph() -> nx.DiGraph:
    """Line anchor ``L`` feeding a two-hop blind chain ``L<->M<->N``.

    Used by the ``compute_blind_distance_to_anchor`` direct test and the
    anchor-tail factor check::

        L(line) <--BLIND 30--> M(blind) <--BLIND 40--> N(blind)

    Expected d-map: ``d(L)=0`` (it is the anchor), ``d(M)=30`` (one blind hop),
    ``d(N)=70`` (two blind hops). Monotonically increasing with depth.
    """
    g = nx.DiGraph()
    g.add_node("L", x_cm=0.0, y_cm=0.0, type="line")
    g.add_node("M", x_cm=30.0, y_cm=0.0, type="blind")
    g.add_node("N", x_cm=70.0, y_cm=0.0, type="blind")
    _add_blind_pair(g, "L", "M", 30.0)
    _add_blind_pair(g, "M", "N", 40.0)
    return g


def _make_line_projection_graph() -> nx.DiGraph:
    """Three line nodes: a LINE edge ``A<->B`` and a PROJECTION edge ``B<->C``.

    The two non-blind edge types sit on DISTINCT node pairs (a ``DiGraph``
    keeps only one edge per ordered pair, so a parallel A->B would clobber the
    first), letting the TC14 assertion exercise one LINE (30 cm) and one
    PROJECTION (45 cm) edge in a single graph.
    """
    from tools.path_planner.graph_builder import EDGE_PROJECTION

    g = nx.DiGraph()
    g.add_node("A", x_cm=0.0, y_cm=0.0, type="line")
    g.add_node("B", x_cm=30.0, y_cm=0.0, type="line")
    g.add_node("C", x_cm=75.0, y_cm=0.0, type="line")
    g.add_edge("A", "B", type=EDGE_LINE, distance_cm=30.0, heading_deg=0.0)
    g.add_edge("B", "A", type=EDGE_LINE, distance_cm=30.0, heading_deg=180.0)
    g.add_edge("B", "C", type=EDGE_PROJECTION, distance_cm=45.0, heading_deg=0.0)
    g.add_edge("C", "B", type=EDGE_PROJECTION, distance_cm=45.0, heading_deg=180.0)
    return g


def _make_shortcut_vs_reset_graph(reset_node: bool) -> nx.DiGraph:
    """Diamond: a line-free deep-blind shortcut vs a longer reset-threading detour.

    Two all-blind branches from the line start ``S`` to the blind goal ``Gl``::

        SHORTCUT (raw 150):  S --50-- A1 --50-- A2 --50-- Gl
        DETOUR   (raw 200):  S --50-- B1 --50-- R --50-- B2 --50-- Gl

    When ``reset_node`` is True the mid-detour node ``R`` is a ``"reset"``
    anchor (``d(R)=0``), so the detour's blind hops have low-``d`` tails (max
    ``d`` along it is 50) and are barely penalised. The shortcut's interior
    node ``A2`` sits two blind hops (``d=100``) from the only reachable anchor
    ``S`` (``Gl`` is blind, not an anchor), so the shortcut is penalised hard.

    Hand-computed weighted costs (exponent 2, d_ref 50):

        shortcut = 150 + 250*k     detour = 200 + 100*k

    The detour undercuts the shortcut iff ``150 + 250k > 200 + 100k`` i.e.
    ``k > 1/3``. So at :data:`_AC15_K_DEFAULT` (0.5) the planner threads ``R``;
    at :data:`_AC15_K_LOWERED` (0.05) it takes the shortcut.

    When ``reset_node`` is False the same node is a plain ``"blind"`` node
    (the negative control): with no anchor in the detour middle its blind hops
    are penalised even harder, so the shortcut wins at default k and the chosen
    path threads no reset (there is none).

    :param reset_node: classify the mid-detour node ``R`` as ``"reset"`` (True)
        or plain ``"blind"`` (False).
    :return: the directed diamond graph.
    """
    g = nx.DiGraph()
    g.add_node("S", x_cm=0.0, y_cm=0.0, type="line")
    g.add_node("Gl", x_cm=200.0, y_cm=0.0, type="blind")
    g.add_node("A1", x_cm=50.0, y_cm=10.0, type="blind")
    g.add_node("A2", x_cm=100.0, y_cm=10.0, type="blind")
    g.add_node("B1", x_cm=50.0, y_cm=-10.0, type="blind")
    g.add_node("B2", x_cm=150.0, y_cm=-10.0, type="blind")
    g.add_node("R", x_cm=100.0, y_cm=-10.0, type="reset" if reset_node else "blind")

    # Shortcut branch: raw 150.
    _add_blind_pair(g, "S", "A1", 50.0)
    _add_blind_pair(g, "A1", "A2", 50.0)
    _add_blind_pair(g, "A2", "Gl", 50.0)
    # Detour branch threading R: raw 200.
    _add_blind_pair(g, "S", "B1", 50.0)
    _add_blind_pair(g, "B1", "R", 50.0)
    _add_blind_pair(g, "R", "B2", 50.0)
    _add_blind_pair(g, "B2", "Gl", 50.0)
    return g


def test_self_loop_sentinel() -> None:
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, _TOY_DECAY)
    for node in g.nodes():
        assert next_hop[node][node] == node


def test_unreachable_pair_is_none() -> None:
    """Removing the blind edge to G leaves G unreachable from every other node."""
    fd = _clone_field_data()
    fd.BLIND_EDGES = []
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        g = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(g, BlindDecayParams.from_field_data(fd))
    for source in g.nodes():
        if source == "G":
            assert next_hop["G"]["G"] == "G"
            continue
        assert next_hop[source]["G"] is None


def test_next_hop_table_has_every_pair() -> None:
    g = build_graph(real_field_data)
    next_hop = compute_all_pairs_next_hop(g, _TOY_DECAY)
    nodes = set(g.nodes())
    assert set(next_hop.keys()) == nodes
    for source, row in next_hop.items():
        assert set(row.keys()) == nodes


def test_compute_path_cost_line_edge() -> None:
    g = build_graph(real_field_data)
    # A=(10,10), B=(200,10): straight line 190 cm. LINE edges keep factor 1
    # under decay, so the cost is the raw length (TC14).
    expected = 190.0
    cost = compute_path_cost(g, ["A", "B"], _TOY_DECAY)
    assert cost == pytest.approx(expected, abs=1e-6)


def test_compute_path_cost_anchor_tail_blind_edge_factor_one() -> None:
    """TC15 (anchor-tail clause of AC14): a blind edge off an anchor costs its length.

    In the toy field the only blind edge is B<->G. ``B`` is a line node, so it
    is an anchor with ``d(B) == 0`` and the factor on B->G is
    ``1 + k*(0/d_ref)**exp == 1`` -- the "first blind hop off an anchor is
    free" caveat (ADR-041). The cost therefore equals the raw 86.02 cm length,
    NOT the old ``length * multiplier``.
    """
    g = build_graph(real_field_data)
    edge = g.edges["B", "G"]
    assert edge["type"] == EDGE_BLIND
    # Confirm the tail is an anchor with d == 0 so the factor really is 1.
    d_map = compute_blind_distance_to_anchor(g)
    assert d_map["B"] == pytest.approx(0.0)
    expected = edge["distance_cm"]  # factor 1, no penalty
    actual = compute_path_cost(g, ["B", "G"], _TOY_DECAY)
    assert actual == pytest.approx(expected, abs=1e-6)


def test_compute_path_cost_blind_edge_decay_formula() -> None:
    """TC15 (AC14): the reverse blind hop G->B carries the convex decay factor.

    G is a blind-only node whose nearest anchor (B, a line node) is one 86.02
    cm blind hop away, so ``d(G) == 86.02``. The G->B edge tail is therefore
    deep in blind territory and its weight is
    ``length * (1 + k*(d(G)/d_ref)**exp)`` -- a strictly > 1 factor, confirming
    the penalty escalates convexly off an anchor.
    """
    g = build_graph(real_field_data)
    edge = g.edges["G", "B"]
    assert edge["type"] == EDGE_BLIND
    length = edge["distance_cm"]

    d_map = compute_blind_distance_to_anchor(g)
    d_tail = d_map["G"]
    assert d_tail == pytest.approx(length, abs=1e-6)  # one blind hop to anchor B

    factor = 1.0 + _TOY_DECAY.k * (d_tail / _TOY_DECAY.d_ref) ** _TOY_DECAY.exponent
    assert factor > 1.0
    expected = length * factor
    actual = compute_path_cost(g, ["G", "B"], _TOY_DECAY)
    assert actual == pytest.approx(expected, abs=1e-6)


def test_compute_path_cost_k_zero_blind_edge_is_plain_length() -> None:
    """TC14b: at k == 0 every blind edge costs exactly its length.

    The G->B hop has a non-anchor tail (``d(G) > 0``), so it would be penalised
    for any positive ``k``. At ``k == 0`` the factor collapses to 1 and the
    cost is the raw length -- the regression anchor proving the decay model is
    a strict generalisation of a flat multiplier.
    """
    g = build_graph(real_field_data)
    length = g.edges["G", "B"]["distance_cm"]
    actual = compute_path_cost(g, ["G", "B"], _NO_DECAY)
    assert actual == pytest.approx(length, abs=1e-6)


def test_compute_path_cost_zero_length_path() -> None:
    g = build_graph(real_field_data)
    assert compute_path_cost(g, ["A"], _TOY_DECAY) == 0.0
    assert compute_path_cost(g, [], _TOY_DECAY) == 0.0


def test_compute_path_cost_missing_edge_raises() -> None:
    g = build_graph(real_field_data)
    # A and G have no direct edge (G is reached via B).
    assert not g.has_edge("A", "G")
    with pytest.raises(KeyError):
        compute_path_cost(g, ["A", "G"], _TOY_DECAY)


# -- blind-decay cost model tests (T22 / AC13-AC16) --------------------------


def test_blind_distance_to_anchor_chain() -> None:
    """compute_blind_distance_to_anchor: anchors d=0, depth grows monotonically.

    On the ``L(line) <-> M(blind) <-> N(blind)`` chain the line node ``L`` is
    the anchor (``d=0``) and the two blind nodes step out 30 then 70 cm. The
    distances strictly increase with depth into blind territory.
    """
    g = _make_blind_chain_graph()
    d_map = compute_blind_distance_to_anchor(g)

    assert d_map["L"] == pytest.approx(0.0)        # anchor: its own nearest anchor
    assert d_map["M"] == pytest.approx(30.0)       # one blind hop to L
    assert d_map["N"] == pytest.approx(70.0)       # two blind hops (30 + 40)
    # Monotone in depth.
    assert d_map["L"] < d_map["M"] < d_map["N"]


def test_blind_distance_to_anchor_reset_node_is_anchor() -> None:
    """A ``"reset"``-typed node anchors the search exactly like a line node.

    On the diamond fixture the mid-detour node ``R`` is a reset node, so it
    must report ``d(R)==0`` and its neighbours ``B1`` / ``B2`` must anchor to
    it at one 50 cm hop (``d==50``), not to the farther line node ``S``.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    d_map = compute_blind_distance_to_anchor(g)

    assert d_map["S"] == pytest.approx(0.0)   # line anchor
    assert d_map["R"] == pytest.approx(0.0)   # reset anchor
    # B2 is only one hop from the reset anchor R (50), not from S.
    assert d_map["B2"] == pytest.approx(50.0)


def test_blind_distance_to_anchor_unreachable_is_inf() -> None:
    """A blind node with NO anchor reachable over blind edges maps to inf.

    Two blind nodes joined only to each other, with the single line anchor
    sitting on a disjoint component, leaves the blind pair anchor-less: their
    ``d`` is :data:`math.inf` (the factor-1 fallback), never a finite value.
    """
    g = nx.DiGraph()
    g.add_node("Iso", x_cm=0.0, y_cm=0.0, type="line")     # disjoint anchor
    g.add_node("P", x_cm=10.0, y_cm=0.0, type="blind")
    g.add_node("Q", x_cm=20.0, y_cm=0.0, type="blind")
    _add_blind_pair(g, "P", "Q", 15.0)
    d_map = compute_blind_distance_to_anchor(g)

    assert d_map["Iso"] == pytest.approx(0.0)
    assert math.isinf(d_map["P"])
    assert math.isinf(d_map["Q"])


def test_blind_distance_unreachable_tail_falls_back_to_factor_one() -> None:
    """An anchor-less blind hop costs its plain length (inf -> factor 1).

    The P<->Q pair from the unreachable fixture has ``d == inf`` on both tails,
    so a positive-k params object must still price the hop at its raw length --
    the unreachable fallback keeps the weight finite and never penalises a hop
    with no decay signal.
    """
    g = nx.DiGraph()
    g.add_node("Iso", x_cm=0.0, y_cm=0.0, type="line")
    g.add_node("P", x_cm=10.0, y_cm=0.0, type="blind")
    g.add_node("Q", x_cm=20.0, y_cm=0.0, type="blind")
    _add_blind_pair(g, "P", "Q", 15.0)

    cost = compute_path_cost(g, ["P", "Q"], _params(_AC15_K_DEFAULT))
    assert cost == pytest.approx(15.0, abs=1e-6)


def test_line_and_projection_edges_keep_weight_equal_length() -> None:
    """TC14 (AC13): LINE and PROJECTION edges cost exactly their length.

    Neither edge type carries the blind-decay factor, so every non-blind edge
    in the weighted graph keeps ``cost == distance_cm`` even at a high gain
    ``k``. The fixture carries one LINE (30 cm) and one PROJECTION (45 cm)
    edge so both non-blind types are exercised.
    """
    from tools.path_planner.graph_builder import EDGE_LINE as _EL, EDGE_PROJECTION as _EP
    from tools.path_planner.planner import _weighted_copy

    g = _make_line_projection_graph()
    high_k = _params(2.0)

    # LINE A->B: 30 cm, factor 1 (also checks compute_path_cost on a LINE hop).
    assert compute_path_cost(g, ["A", "B"], high_k) == pytest.approx(30.0, abs=1e-6)

    # Every LINE / PROJECTION cell in the weighted copy carries cost == length.
    weighted = _weighted_copy(g, high_k)
    non_blind = [
        (u, v, data)
        for u, v, data in weighted.edges(data=True)
        if data["type"] in (_EL, _EP)
    ]
    assert non_blind, "fixture must contain at least one LINE / PROJECTION edge"
    for _u, _v, data in non_blind:
        assert data["cost"] == pytest.approx(data["distance_cm"], abs=1e-6)


def test_k_zero_every_blind_weight_equals_length() -> None:
    """TC14b (AC14b): at k == 0 every blind-edge weight equals its length.

    The strongest regression anchor: on the deep-blind diamond, where several
    blind tails have large ``d``, ``k == 0`` must collapse EVERY blind edge's
    weighted cost back to its raw ``distance_cm``.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    from tools.path_planner.planner import _weighted_copy

    weighted = _weighted_copy(g, _NO_DECAY)
    for u, v, data in weighted.edges(data=True):
        if data["type"] == EDGE_BLIND:
            assert data["cost"] == pytest.approx(data["distance_cm"], abs=1e-9)


def test_blind_weight_formula_matches_hand_computation() -> None:
    """TC15 (AC14): a deep blind tail's weight matches the convex formula by hand.

    On the diamond, the shortcut edge ``A2->Gl`` has tail ``A2`` two blind hops
    (``d=100``) from the only reachable anchor ``S``. Its weighted cost must
    equal ``50 * (1 + k*(100/50)**2)`` exactly, and an anchor-tail edge
    (``S->A1``, ``d(S)=0``) must stay at factor 1.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    params = _params(_AC15_K_DEFAULT)
    d_map = compute_blind_distance_to_anchor(g)
    assert d_map["A2"] == pytest.approx(100.0)
    assert d_map["S"] == pytest.approx(0.0)

    from tools.path_planner.planner import _weighted_copy

    weighted = _weighted_copy(g, params)
    deep_factor = 1.0 + params.k * (100.0 / params.d_ref) ** params.exponent
    assert weighted.edges["A2", "Gl"]["cost"] == pytest.approx(
        50.0 * deep_factor, abs=1e-6
    )
    # Anchor-tail blind edge: factor ~ 1 (the first blind hop off an anchor).
    assert weighted.edges["S", "A1"]["cost"] == pytest.approx(50.0, abs=1e-6)


def _has_reset(path: list, graph: nx.DiGraph) -> bool:
    """True iff any node on ``path`` is typed ``"reset"`` in ``graph``."""
    return any(graph.nodes[n].get("type") == "reset" for n in path)


def test_ac15_default_k_routes_through_reset() -> None:
    """TC16 (AC15): at default k the chosen S->Gl route threads the reset node.

    The deep-blind shortcut (raw 150) is penalised harder than the longer
    reset-threading detour (raw 200) once ``k > 1/3``; at the default ``k=0.5``
    Dijkstra picks the detour, so the NEXT_HOP-walked path contains ``R``.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    next_hop = compute_all_pairs_next_hop(g, _params(_AC15_K_DEFAULT))

    path = _walk_next_hop(next_hop, "S", "Gl")
    assert _has_reset(path, g), f"expected the detour through R; got {path}"
    assert "R" in path
    # It is specifically the detour branch (B1/B2), not the shortcut (A1/A2).
    assert "A2" not in path


def test_ac15_lowered_k_takes_shortcut_without_reset() -> None:
    """TC16 (AC15): at lowered k the chosen route is the shortcut, no reset.

    At ``k=0.05`` the tiny penalty no longer overcomes the 50 cm raw advantage
    of the shortcut, so Dijkstra takes ``S->A1->A2->Gl`` and the path threads
    no reset node.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    next_hop = compute_all_pairs_next_hop(g, _params(_AC15_K_LOWERED))

    path = _walk_next_hop(next_hop, "S", "Gl")
    assert not _has_reset(path, g), f"expected the reset-free shortcut; got {path}"
    assert "R" not in path
    assert "A1" in path and "A2" in path


def test_ac15_negative_no_reset_anchor_chosen_on_weight_alone() -> None:
    """TC16 negative control: with the reset relabelled blind, no reset is threaded.

    The same diamond geometry but the mid-detour node is a plain ``"blind"``
    node (not an anchor). The detour is now penalised even harder, so at the
    default ``k`` Dijkstra still picks the shortcut -- proving the positive
    test's reset-threading is driven by the reset's anchor status, not by the
    topology. The chosen path provably contains no reset-typed node.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=False)
    next_hop = compute_all_pairs_next_hop(g, _params(_AC15_K_DEFAULT))

    path = _walk_next_hop(next_hop, "S", "Gl")
    assert not _has_reset(path, g), f"no node is reset-typed; got {path}"
    # The shortcut branch is chosen on weight alone.
    assert "A1" in path and "A2" in path


def test_decay_table_shape_is_flat_next_hop() -> None:
    """TC17 (AC16): the decay table stays a flat NEXT_HOP[from][to], no new dim.

    The decay penalty changes edge *weights*, never the table shape: the
    all-pairs result is still a 2-level ``{from: {to: next | None}}`` dict with
    every node as a key on both levels (no augmented ``(node, blind_bucket)``
    dimension).
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    next_hop = compute_all_pairs_next_hop(g, _params(_AC15_K_DEFAULT))

    nodes = set(g.nodes())
    assert set(next_hop.keys()) == nodes
    for source, row in next_hop.items():
        assert isinstance(row, dict)
        assert set(row.keys()) == nodes
        for target, hop in row.items():
            # Each cell is a node name or None -- never a nested dict / tuple.
            assert hop is None or hop in nodes


def test_disabled_params_skip_penalty_entirely() -> None:
    """enabled=False reduces every blind edge to plain length (no d-map signal).

    A disabled params object must price the deep ``A2->Gl`` blind hop at its
    raw 50 cm even though its tail is deep (``d=100``) -- the master switch
    short-circuits the penalty before the formula runs.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    disabled = BlindDecayParams(enabled=False, k=0.5, exponent=2.0, d_ref=50.0)
    from tools.path_planner.planner import _weighted_copy

    weighted = _weighted_copy(g, disabled)
    assert weighted.edges["A2", "Gl"]["cost"] == pytest.approx(50.0, abs=1e-9)


@pytest.mark.parametrize(
    "bad_params, needle",
    [
        (BlindDecayParams(enabled=True, k=0.05, exponent=2.0, d_ref=0.0), "d_ref"),
        (BlindDecayParams(enabled=True, k=-0.1, exponent=2.0, d_ref=50.0), "k"),
        (BlindDecayParams(enabled=True, k=0.05, exponent=0.5, d_ref=50.0), "exponent"),
        # Non-finite values slip through a comparison-only guard (nan < 0 is
        # False), so the shared predicate rejects them at the finiteness gate.
        (BlindDecayParams(enabled=True, k=float("nan"), exponent=2.0, d_ref=50.0), "k"),
        (BlindDecayParams(enabled=True, k=0.05, exponent=float("nan"), d_ref=50.0), "exponent"),
        (BlindDecayParams(enabled=True, k=0.05, exponent=float("inf"), d_ref=50.0), "exponent"),
        (BlindDecayParams(enabled=True, k=0.05, exponent=2.0, d_ref=float("nan")), "d_ref"),
        # A finite-but-extreme exponent would overflow the weight; the cap stops
        # it at authoring time with a ValueError, never an OverflowError.
        (BlindDecayParams(enabled=True, k=0.05, exponent=400.0, d_ref=50.0), "exponent"),
        # bool("false") is True, so a truthy non-bool enabled would silently
        # enable decay; the type check rejects it.
        (BlindDecayParams(enabled="false", k=0.05, exponent=2.0, d_ref=50.0), "enabled"),
    ],
    ids=[
        "d_ref<=0", "k<0", "exponent<1",
        "k=nan", "exponent=nan", "exponent=inf", "d_ref=nan",
        "exponent>max", "enabled non-bool",
    ],
)
def test_out_of_domain_params_raise(bad_params, needle) -> None:
    """A bypass caller passing out-of-domain params fails loudly (mirrors validator).

    Each malformed field raises ValueError naming it, so a direct caller that
    skips the field-data validator cannot emit a negative / NaN / overflowing
    Dijkstra weight or silently enable decay through a truthy non-bool. The
    domain rules are the single shared ``validator.validate_blind_decay_domain``
    predicate, so this guard can never drift from the field-data validator.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    with pytest.raises(ValueError, match=needle):
        compute_all_pairs_next_hop(g, bad_params)


def test_extreme_exponent_raises_valueerror_not_overflow() -> None:
    """The exponent cap converts a would-be OverflowError into a clean ValueError.

    Regression for the unbounded-exponent crash: a fat-fingered exponent of 400
    on a graph with deep blind tails used to raise OverflowError deep inside
    networkx Dijkstra. The shared guard now rejects it up front with a ValueError
    naming the exponent -- and never an OverflowError.
    """
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    bad = BlindDecayParams(enabled=True, k=0.05, exponent=400.0, d_ref=50.0)
    with pytest.raises(ValueError, match="exponent"):
        compute_all_pairs_next_hop(g, bad)


def test_exponent_at_cap_is_accepted() -> None:
    """An exponent exactly at the cap is in-domain and prices a real table."""
    g = _make_shortcut_vs_reset_graph(reset_node=True)
    ok = BlindDecayParams(enabled=True, k=0.05, exponent=8.0, d_ref=50.0)
    next_hop = compute_all_pairs_next_hop(g, ok)
    # A populated table, not an exception, is the assertion.
    assert next_hop and all(src in next_hop[src] for src in next_hop)


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
        BLIND_DECAY_ENABLED=True,
        BLIND_DECAY_K=0.05,
        BLIND_DECAY_EXPONENT=2.0,
        BLIND_DECAY_D_REF=50.0,
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
