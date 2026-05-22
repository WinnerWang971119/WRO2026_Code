"""Tests for :mod:`tools.path_planner.editor.auto_edges`.

Covers TC5--TC13 of the iteration-2 auto-edges plan:

* TC5  -- 4-node square, no obstacles, generous length cap -> 5 edges (4
  perimeter + 1 Delaunay diagonal), all classified ``"line"``, sorted.
* TC6  -- same square, length cap below the diagonal -> only the 4
  perimeter edges survive.
* TC7  -- 4-node square with a central inflated obstacle -> both diagonals
  are filtered out, perimeter survives.
* TC8  -- 4-node config with one ``"blind"`` node -> every edge touching
  the blind node gets ``kind="blind"``; the line/line edges stay
  ``"line"``.
* TC9  -- pre-existing line edge is not re-emitted.
* TC10 -- pre-existing blind edge is not re-emitted.
* TC11 -- 2-node model returns ``[]``.
* TC12 -- 3 collinear nodes return ``[]``.
* TC13 -- :func:`_classify_edge_type` parametrised over the 4 type pairs.

The module under test is pure logic: no GUI, no file I/O.  Tests build a
tiny :class:`~tools.path_planner.editor.model.FieldModel` literal per case
via the :func:`_model` helper.
"""

from __future__ import annotations

import pytest

from tools.path_planner.editor.auto_edges import (
    _classify_edge_type,
    propose_edges,
)
from tools.path_planner.editor.model import FieldModel


# ---------------------------------------------------------------------------
# Fixture factory
# ---------------------------------------------------------------------------


def _model(
    nodes: dict[str, tuple[float, float]],
    types: dict[str, str],
    line_segments: tuple = (),
    blind_edges: tuple = (),
    obstacles: tuple = (),
    robot_radius_cm: float = 2.0,
) -> FieldModel:
    """Build a minimally-populated FieldModel for an auto-edges test.

    The non-edge fields (``field_mm``, ``map_png``, etc.) are filled with
    inert defaults; ``start_node`` defaults to the first node so the model
    stays self-consistent without forcing each test to spell it out.

    :param nodes: ``{name: (x_cm, y_cm)}``.
    :param types: ``{name: "line" | "blind"}``.
    :param line_segments: Iterable of ``(a, b)`` line edges.
    :param blind_edges: Iterable of ``(a, b)`` blind edges.
    :param obstacles: Iterable of ``(x_min, y_min, x_max, y_max, kind)``.
    :param robot_radius_cm: Inflation radius in centimetres.
    :return: A fresh :class:`FieldModel`.
    """
    return FieldModel(
        field_mm=(2000.0, 2000.0),
        map_png="",
        robot_radius_cm=robot_radius_cm,
        cost_blind_multiplier=2.0,
        start_node=next(iter(nodes)),
        start_heading_deg=0.0,
        nodes=dict(nodes),
        node_types=dict(types),
        line_segments=list(line_segments),
        blind_edges=list(blind_edges),
        obstacles=list(obstacles),
    )


# ---------------------------------------------------------------------------
# TC5 -- 4-node square, all line, no obstacles
# ---------------------------------------------------------------------------


def test_tc5_square_no_obstacles_all_five_edges() -> None:
    """4-node square Delaunay = 2 triangles sharing one diagonal -> 5 edges.

    All four nodes are typed ``"line"`` so every output edge is ``"line"``.
    The exact diagonal (a-c vs b-d) depends on shapely's tie-breaking, so we
    assert on the *set* of undirected pairs and the *count*, while still
    confirming the public sort guarantee holds via a separate check on the
    list-of-pairs being in ascending order.
    """
    model = _model(
        nodes={
            "a": (0.0, 0.0),
            "b": (10.0, 0.0),
            "c": (10.0, 10.0),
            "d": (0.0, 10.0),
        },
        types={"a": "line", "b": "line", "c": "line", "d": "line"},
    )

    result = propose_edges(model, max_edge_length_cm=100.0)

    # All five edges, all classified "line".
    assert len(result) == 5
    assert all(kind == "line" for _, _, kind in result)

    # The four perimeter pairs are always present.
    pair_set = {frozenset((a, b)) for a, b, _ in result}
    for perimeter in [("a", "b"), ("b", "c"), ("c", "d"), ("a", "d")]:
        assert frozenset(perimeter) in pair_set

    # Exactly one diagonal is present (a-c xor b-d).
    has_ac = frozenset(("a", "c")) in pair_set
    has_bd = frozenset(("b", "d")) in pair_set
    assert has_ac ^ has_bd, "expected exactly one Delaunay diagonal"

    # Public guarantee: output sorted lexicographically by (a, b) with a < b.
    pairs = [(a, b) for a, b, _ in result]
    assert pairs == sorted(pairs)
    for a, b, _ in result:
        assert a < b, f"endpoint ordering violated: ({a!r}, {b!r})"


# ---------------------------------------------------------------------------
# TC6 -- same square, length cap below the diagonal
# ---------------------------------------------------------------------------


def test_tc6_square_length_cap_drops_diagonal() -> None:
    """max=12 < diagonal length (~14.14 cm) -> only 4 perimeter edges."""
    model = _model(
        nodes={
            "a": (0.0, 0.0),
            "b": (10.0, 0.0),
            "c": (10.0, 10.0),
            "d": (0.0, 10.0),
        },
        types={"a": "line", "b": "line", "c": "line", "d": "line"},
    )

    result = propose_edges(model, max_edge_length_cm=12.0)

    assert len(result) == 4
    pair_set = {frozenset((a, b)) for a, b, _ in result}
    assert pair_set == {
        frozenset(("a", "b")),
        frozenset(("b", "c")),
        frozenset(("c", "d")),
        frozenset(("a", "d")),
    }
    assert all(kind == "line" for _, _, kind in result)


# ---------------------------------------------------------------------------
# TC7 -- 4-node square with central obstacle -> diagonals filtered out
# ---------------------------------------------------------------------------


def test_tc7_obstacle_filters_diagonal() -> None:
    """Central obstacle at (20,20,30,30) clips both diagonals of a 50x50 square.

    The four perimeter edges run along x=0, x=50, y=0, y=50 (each 10+ cm
    clear of the inflated obstacle even after a 2 cm buffer), so they
    survive.
    """
    model = _model(
        nodes={
            "a": (0.0, 0.0),
            "b": (50.0, 0.0),
            "c": (50.0, 50.0),
            "d": (0.0, 50.0),
        },
        types={"a": "line", "b": "line", "c": "line", "d": "line"},
        obstacles=((20.0, 20.0, 30.0, 30.0, "box"),),
        robot_radius_cm=2.0,
    )

    result = propose_edges(model, max_edge_length_cm=100.0)

    pair_set = {frozenset((a, b)) for a, b, _ in result}
    # Neither diagonal should appear.
    assert frozenset(("a", "c")) not in pair_set
    assert frozenset(("b", "d")) not in pair_set
    # All four perimeter edges should appear.
    for perimeter in [("a", "b"), ("b", "c"), ("c", "d"), ("a", "d")]:
        assert frozenset(perimeter) in pair_set
    assert len(result) == 4


# ---------------------------------------------------------------------------
# TC8 -- one blind node, kind classification
# ---------------------------------------------------------------------------


def test_tc8_blind_node_taints_incident_edges() -> None:
    """Every edge touching the blind node "d" gets kind="blind"; the rest
    stay "line"."""
    model = _model(
        nodes={
            "a": (0.0, 0.0),
            "b": (10.0, 0.0),
            "c": (10.0, 10.0),
            "d": (0.0, 10.0),
        },
        types={"a": "line", "b": "line", "c": "line", "d": "blind"},
    )

    result = propose_edges(model, max_edge_length_cm=100.0)

    # Verify the kind for every output edge.
    for a, b, kind in result:
        endpoints = {a, b}
        expected = "blind" if "d" in endpoints else "line"
        assert kind == expected, (
            f"edge ({a!r}, {b!r}) got kind={kind!r}, expected {expected!r}"
        )

    # At least one "line" edge exists -- the b-c perimeter -- so we know the
    # default case isn't swallowed by the blind taint.
    assert any(kind == "line" for _, _, kind in result)
    # And at least one "blind" edge -- a-d or c-d perimeter -- must exist.
    assert any(kind == "blind" for _, _, kind in result)


# ---------------------------------------------------------------------------
# TC9 -- pre-existing line edge is not re-emitted
# ---------------------------------------------------------------------------


def test_tc9_skips_existing_line_segment() -> None:
    """3-node triangle with one edge already in line_segments -> output
    must not contain that pair."""
    model = _model(
        nodes={
            "a": (0.0, 0.0),
            "b": (10.0, 0.0),
            "c": (5.0, 10.0),
        },
        types={"a": "line", "b": "line", "c": "line"},
        line_segments=(("a", "b"),),
    )

    result = propose_edges(model, max_edge_length_cm=100.0)

    pair_set = {frozenset((a, b)) for a, b, _ in result}
    assert frozenset(("a", "b")) not in pair_set
    # The other two triangle edges should still be proposed.
    assert frozenset(("a", "c")) in pair_set
    assert frozenset(("b", "c")) in pair_set


# ---------------------------------------------------------------------------
# TC10 -- pre-existing blind edge is not re-emitted
# ---------------------------------------------------------------------------


def test_tc10_skips_existing_blind_edge() -> None:
    """Same triangle, but the pre-existing edge is in blind_edges instead
    of line_segments."""
    model = _model(
        nodes={
            "a": (0.0, 0.0),
            "b": (10.0, 0.0),
            "c": (5.0, 10.0),
        },
        types={"a": "line", "b": "line", "c": "line"},
        blind_edges=(("a", "b"),),
    )

    result = propose_edges(model, max_edge_length_cm=100.0)

    pair_set = {frozenset((a, b)) for a, b, _ in result}
    assert frozenset(("a", "b")) not in pair_set
    assert frozenset(("a", "c")) in pair_set
    assert frozenset(("b", "c")) in pair_set


# ---------------------------------------------------------------------------
# TC11 -- 2-node model returns []
# ---------------------------------------------------------------------------


def test_tc11_two_node_model_returns_empty() -> None:
    """Delaunay needs at least 3 points -> 2-node input returns []."""
    model = _model(
        nodes={"a": (0.0, 0.0), "b": (10.0, 0.0)},
        types={"a": "line", "b": "line"},
    )

    assert propose_edges(model, max_edge_length_cm=100.0) == []


# ---------------------------------------------------------------------------
# TC12 -- 3 collinear nodes return []
# ---------------------------------------------------------------------------


def test_tc12_collinear_nodes_return_empty() -> None:
    """Three collinear points have no valid triangulation -> []."""
    model = _model(
        nodes={
            "a": (0.0, 0.0),
            "b": (10.0, 0.0),
            "c": (20.0, 0.0),
        },
        types={"a": "line", "b": "line", "c": "line"},
    )

    assert propose_edges(model, max_edge_length_cm=100.0) == []


# ---------------------------------------------------------------------------
# TC13 -- _classify_edge_type parametrised over the 4 type pairs
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "type_a,type_b,expected",
    [
        ("line", "line", "line"),
        ("line", "blind", "blind"),
        ("blind", "line", "blind"),
        ("blind", "blind", "blind"),
    ],
)
def test_tc13_classify_edge_type(
    type_a: str, type_b: str, expected: str
) -> None:
    """Edge classification: any blind endpoint poisons the whole edge."""
    node_types = {"x": type_a, "y": type_b}
    assert _classify_edge_type(node_types, "x", "y") == expected
