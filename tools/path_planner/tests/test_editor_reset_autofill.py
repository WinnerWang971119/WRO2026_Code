"""Tests for the reset-node auto-fill stack of the editor restyle wave (T9).

Five layers, from pure geometry up to the app handler:

* **TC5** -- ``projection_edges.reset_autofill`` geometry for both axes near
  each wall, cross-checked against ``validator._validate_reset_nodes`` so the
  auto-filled ``(wall_coord, facing)`` is a spec the Save-time validator
  ACCEPTS; plus a wall-hugging node whose best-effort spec is still returned
  (and is then correctly rejected by the validator's clearance floor).
* **TC8** -- the type<->spec invariant at Save: a :class:`FieldModel` with a
  node typed ``"reset"`` but NO ``reset_nodes`` entry must fail
  ``model.validate()`` (the divergence the inline tier never re-checks).
* **TC6** -- the app's ``_on_panel_reset_spec_change`` recompute behaviour: an
  AXIS change re-derives wall + facing from the autofill, while a wall/facing
  hand-edit with the axis UNCHANGED dispatches the operator's values verbatim
  (no clobber). Implemented via the headless ``EditorApp.__new__`` +
  injected-collaborator pattern (mirrors ``test_editor_inspector.py``).
* **TC1–TC4** -- ``reset_autofill_auto`` pure-geometry tests: nearest-wall for
  all four walls, the reser2/test reference geometries, and the exact-tie
  tie-break.
* **TC7, TC9–TC12** -- app-handler tests: convert-to-reset, add-node-reset,
  manual axis-change, and the too-close warning message content.

TC5 and TC8 are pure logic. TC6 and TC7/TC9–TC12 drive the real ``EditorApp``
handler, which imports ``editor.canvas`` (an import-time
``matplotlib.use("TkAgg")``); pin the headless-safe Agg backend and no-op that
one switch for the duration of the import, exactly as
``test_editor_inspector.py`` does. No real Tk window is built.
"""

from __future__ import annotations

import types

import matplotlib
import pytest

# Pin the headless-safe Agg backend before importing the editor, then no-op the
# one ``matplotlib.use("TkAgg")`` switch that ``editor.canvas`` performs at
# import time (it would raise on headless CI and clobber the Agg backend the
# other tests rely on). Restore the real ``use`` immediately afterwards.
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor.app import EditorApp  # noqa: E402
finally:
    matplotlib.use = _real_mpl_use

from tools.path_planner.editor import projection_edges  # noqa: E402
from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.validator import (  # noqa: E402
    FieldDataError,
    WALL_PIVOT_CLEARANCE_CM,
    _validate_reset_nodes,
)

# A 300 cm x 200 cm field (millimetres, the ``FieldModel.field_mm`` convention).
_FIELD_MM: tuple[float, float] = (3000.0, 2000.0)
_FIELD_W_CM: float = _FIELD_MM[0] / 10.0  # 300.0
_FIELD_H_CM: float = _FIELD_MM[1] / 10.0  # 200.0

# Reference field matching the spec's worked-check examples: 320x154 cm.
_RESER2_FIELD_MM: tuple[float, float] = (3200.0, 1540.0)


# ===========================================================================
# TC1 -- reset_autofill_auto: nearest wall for each of the four walls
# ===========================================================================


@pytest.mark.parametrize(
    "node_xy, expected_axis, expected_wall, expected_facing",
    [
        # Near x=0 wall (standoff 10): all other walls are farther.
        ((10.0, 100.0), "x", 0.0, 0.0),
        # Near x=300 wall (standoff 10): all other walls are farther.
        ((290.0, 100.0), "x", _FIELD_W_CM, 180.0),
        # Near y=0 wall (standoff 10): all other walls are farther.
        ((150.0, 10.0), "y", 0.0, 90.0),
        # Near y=200 wall (standoff 10): all other walls are farther.
        ((150.0, 190.0), "y", _FIELD_H_CM, 270.0),
    ],
)
def test_tc1_reset_autofill_auto_nearest_of_four_walls(
    node_xy, expected_axis, expected_wall, expected_facing
) -> None:
    """``reset_autofill_auto`` returns the nearest wall's ``(axis, wall, facing)``
    for each of the four walls in a 300x200 cm field."""
    axis, wall, facing = projection_edges.reset_autofill_auto(
        node_xy, _FIELD_MM
    )

    assert axis == expected_axis
    assert wall == pytest.approx(expected_wall)
    assert facing == pytest.approx(expected_facing)


# ===========================================================================
# TC2 -- reset_autofill_auto: reser2 reference geometry (y-wall nearest)
# ===========================================================================


def test_tc2_reset_autofill_auto_reser2_geometry() -> None:
    """Node ``(238.8, 19.6)`` in a 320x154 field: y=0 is the nearest wall at
    standoff 19.6 cm, so ``reset_autofill_auto`` must pick ``("y", 0.0, 90.0)``."""
    # reser2: standoffs x0=238.8, x320=81.2, y0=19.6, y154=134.4 -> nearest y=0.
    axis, wall, facing = projection_edges.reset_autofill_auto(
        (238.8, 19.6), _RESER2_FIELD_MM
    )

    assert axis == "y"
    assert wall == pytest.approx(0.0)
    assert facing == pytest.approx(90.0)


# ===========================================================================
# TC3 -- reset_autofill_auto: exact tie-break (prefer y, then low wall)
# ===========================================================================


def test_tc3_reset_autofill_auto_cross_axis_tie_prefers_y() -> None:
    """Exact tie between x=0 and y=0 (both standoff 30) resolves to ``"y"``.

    Node ``(30, 30)`` in a 200x200 field: standoffs x0=30, x200=170, y0=30,
    y200=170. Both x=0 and y=0 are tied at 30 cm; the tie-break must choose
    ``"y"`` (axis_rank y=0, x=1). Integer coords make the tie bit-exact.
    """
    # 200x200 cm = 2000x2000 mm
    field_mm_square = (2000.0, 2000.0)

    axis, wall, facing = projection_edges.reset_autofill_auto(
        (30.0, 30.0), field_mm_square
    )

    assert axis == "y"
    assert wall == pytest.approx(0.0)
    assert facing == pytest.approx(90.0)


def test_tc3_reset_autofill_auto_within_axis_tie_prefers_low_wall() -> None:
    """A node equidistant from all four walls resolves to ``("y", 0.0, 90.0)``.

    ``(100, 100)`` in a 200x200 field stands off all four walls by 100 cm, so
    every standoff ties: the cross-axis rule picks ``"y"`` and the within-axis
    result is the low wall ``0.0``. This covers the centered/all-equal case
    only; because the candidate list is built low-wall-first, it does NOT
    isolate the ``wall_rank`` tie-break term (the cross-axis test above is the
    strong tie-break assertion).
    """
    field_mm_square = (2000.0, 2000.0)

    # All four walls have standoff 100: x0=100, x200=100, y0=100, y200=100.
    # Tie-break: prefer y, then prefer low wall 0.
    axis, wall, facing = projection_edges.reset_autofill_auto(
        (100.0, 100.0), field_mm_square
    )

    assert axis == "y"
    assert wall == pytest.approx(0.0)
    assert facing == pytest.approx(90.0)


# ===========================================================================
# TC4 -- reset_autofill_auto: seeds too-close wall without raising
# ===========================================================================


def test_tc4_reset_autofill_auto_seeds_too_close_wall() -> None:
    """``reset_autofill_auto`` returns the nearest wall even when it is under
    the pivot-clearance floor (standoff < 14 cm), and never raises.

    ``test`` geometry ``(309.4, 69.1)`` in 320x154: standoffs x0=309.4,
    x320=10.6 (nearest), y0=69.1, y154=84.9 -> nearest ``x=320`` at 10.6 cm.
    The function returns ``("x", 320.0, 180.0)``; the caller surfaces the
    warning. Separately, ``is_reset_wall_too_close`` reports ``True`` for this
    wall (10.6 < 14).
    """
    axis, wall, facing = projection_edges.reset_autofill_auto(
        (309.4, 69.1), _RESER2_FIELD_MM
    )

    assert axis == "x"
    assert wall == pytest.approx(320.0)
    assert facing == pytest.approx(180.0)

    # The returned wall is below the clearance floor.
    assert projection_edges.is_reset_wall_too_close(
        (309.4, 69.1), "x", 320.0
    ) is True


# ===========================================================================
# TC5 -- reset_autofill geometry + validator cross-check
# ===========================================================================


@pytest.mark.parametrize(
    "node_xy, axis, expected_wall, expected_facing",
    [
        # axis x, node near the low (x=0) wall: both walls clear the standoff,
        # the nearer (0.0) wins, facing 0 deg points +x away from that wall.
        ((20.0, 100.0), "x", 0.0, 0.0),
        # axis x, node near the high (x=300) wall: nearer wall is 300, facing
        # 180 deg points -x away from it.
        ((280.0, 100.0), "x", _FIELD_W_CM, 180.0),
        # axis y, node near the low (y=0) wall: nearer is 0, facing 90 deg (+y).
        ((100.0, 30.0), "y", 0.0, 90.0),
        # axis y, node near the high (y=200) wall: nearer is 200, facing 270.
        ((100.0, 180.0), "y", _FIELD_H_CM, 270.0),
    ],
)
def test_tc5_reset_autofill_wall_and_facing(
    node_xy, axis, expected_wall, expected_facing
) -> None:
    """The auto-filled ``(wall_coord, facing)`` matches the per-axis spec for a
    node placed near each wall."""
    wall, facing = projection_edges.reset_autofill(node_xy, axis, _FIELD_MM)

    assert wall == pytest.approx(expected_wall)
    assert facing == pytest.approx(expected_facing)


@pytest.mark.parametrize(
    "node_xy, axis",
    [
        ((20.0, 100.0), "x"),
        ((280.0, 100.0), "x"),
        ((100.0, 30.0), "y"),
        ((100.0, 180.0), "y"),
    ],
)
def test_tc5_autofill_spec_is_validator_accepted(node_xy, axis) -> None:
    """A reset node placed ``>= WALL_PIVOT_CLEARANCE_CM`` from its wall produces
    an auto-filled spec ``validator._validate_reset_nodes`` does NOT reject.

    Builds a ``field_data``-shaped namespace carrying the reset node + the
    auto-filled spec and asserts the Save-time validator accepts it -- the
    auto-fill must hand the author a realisable reset, not one the validator
    will flag.
    """
    wall, facing = projection_edges.reset_autofill(node_xy, axis, _FIELD_MM)

    # Sanity: the chosen wall genuinely clears the pivot floor for this node.
    node_coord = node_xy[0] if axis == "x" else node_xy[1]
    assert abs(wall - node_coord) >= WALL_PIVOT_CLEARANCE_CM

    # A tiny field_data proxy: one reset node (R) + a plain start node (S) so
    # the START-node-is-not-reset rule has a non-reset start to land on.
    proxy = types.SimpleNamespace(
        NODES={"R": tuple(node_xy), "S": (150.0, 100.0)},
        NODE_TYPES={"R": "reset", "S": "line"},
        RESET_NODES={
            "R": {"axis": axis, "wall_coord": wall, "facing": facing}
        },
        START_NODE="S",
    )

    # Must not raise -- the auto-fill produced a validator-accepted spec.
    _validate_reset_nodes(proxy)


def test_tc5_wall_hugging_node_returns_best_effort_spec() -> None:
    """A node too close to BOTH walls on the axis still gets a best-effort spec
    back (the nearest wall, which under the new "nearest, not nearest-qualifying"
    rule is the low wall 0.0 due to the within-axis tie-break), which the
    validator then flags on its clearance floor.

    The node is symmetric: (10, 5) in a 200mm narrow field -> x=0 and x=20 are
    equidistant at standoff 10 cm. The new ``reset_autofill`` picks the nearest
    wall on the axis, ties to the low wall 0.0 (wall_rank 0 < 1). Both old
    ``max(..., key=standoff)`` on a tie and new ``min``-with-low-wall-first
    resolve to wall 0.0 for this symmetric case, so the asserts below are
    correct under both the old and new selection rule.

    This test definitively does NOT change: the node is symmetric, both walls
    are at standoff 10 (below the 14 cm floor), and the low-wall-first tie
    means ``wall in (0.0, field_w)`` is satisfied by either direction. The
    important postcondition is that a spec is returned (never raises) and the
    Save-time validator rejects it.
    """
    # A narrow 20 cm-wide field (200 mm): node centred at x=10 stands 10 cm from
    # each wall, both below the 14 cm pivot floor, so NEITHER qualifies.
    narrow_field_mm = (200.0, 200.0)
    hug_node = (10.0, 5.0)

    wall, facing = projection_edges.reset_autofill(
        hug_node, "x", narrow_field_mm
    )

    # A spec is still returned (best-effort) -- the function never raises here.
    assert wall in (0.0, narrow_field_mm[0] / 10.0)
    assert 0.0 <= facing < 360.0

    # ... and the returned standoff is genuinely below the floor, so the
    # Save-time validator rejects it (proving "best-effort, not silently
    # accepted").
    assert abs(wall - hug_node[0]) < WALL_PIVOT_CLEARANCE_CM

    proxy = types.SimpleNamespace(
        NODES={"R": hug_node},
        NODE_TYPES={"R": "reset"},
        RESET_NODES={
            "R": {"axis": "x", "wall_coord": wall, "facing": facing}
        },
        START_NODE=None,
    )
    with pytest.raises(FieldDataError):
        _validate_reset_nodes(proxy)


# ===========================================================================
# TC5 -- is_reset_wall_too_close boundary
# ===========================================================================


def test_tc5_is_reset_wall_too_close_below_floor_is_true() -> None:
    """Standoff strictly less than 14 cm -> too close."""
    # Node at x=5, wall at x=0: standoff = 5 < 14.
    assert projection_edges.is_reset_wall_too_close((5.0, 50.0), "x", 0.0) is True
    # Node at y=5, wall at y=0: standoff = 5 < 14.
    assert projection_edges.is_reset_wall_too_close((50.0, 5.0), "y", 0.0) is True


def test_tc5_is_reset_wall_too_close_exactly_at_floor_is_false() -> None:
    """Standoff exactly equal to 14 cm -> NOT too close (boundary is exclusive)."""
    clearance = WALL_PIVOT_CLEARANCE_CM  # 14.0
    # Node at x=clearance, wall at x=0: standoff = clearance exactly.
    assert (
        projection_edges.is_reset_wall_too_close(
            (clearance, 50.0), "x", 0.0
        )
        is False
    )
    # Node at y=clearance, wall at y=0.
    assert (
        projection_edges.is_reset_wall_too_close(
            (50.0, clearance), "y", 0.0
        )
        is False
    )


def test_tc5_is_reset_wall_too_close_above_floor_is_false() -> None:
    """Standoff above 14 cm -> NOT too close."""
    clearance = WALL_PIVOT_CLEARANCE_CM
    # Standoff of clearance + 1.
    node_x = clearance + 1.0
    assert (
        projection_edges.is_reset_wall_too_close(
            (node_x, 50.0), "x", 0.0
        )
        is False
    )
    node_y = clearance + 1.0
    assert (
        projection_edges.is_reset_wall_too_close(
            (50.0, node_y), "y", 0.0
        )
        is False
    )


def test_tc5_is_reset_wall_too_close_uses_correct_axis_coordinate() -> None:
    """The predicate uses ``node_xy[0]`` for axis ``"x"`` and ``node_xy[1]``
    for axis ``"y"`` -- never the wrong coordinate."""
    # Node at (5, 50): x-standoff=5 (too close), y-standoff=50 (not too close).
    node = (5.0, 50.0)
    assert projection_edges.is_reset_wall_too_close(node, "x", 0.0) is True
    assert projection_edges.is_reset_wall_too_close(node, "y", 0.0) is False


# ===========================================================================
# TC6 -- reset_autofill (single axis) now picks nearest, not nearest-qualifying
# ===========================================================================


def test_tc6_reset_autofill_returns_near_wall_even_when_too_close() -> None:
    """The single-axis ``reset_autofill`` returns the NEAREST wall on the axis,
    even when that wall is under the clearance floor and the far wall is not.

    Changed behaviour (T1): the old selection skipped the too-close near wall
    and returned the far qualifying wall. The new selection picks the nearest
    regardless -- the operator aimed at the near wall, and the warning + Save
    validator handle the clearance.

    Geometry: axis ``"x"``, field 300x200 cm, node at ``(5, 100)``.
    x0 standoff = 5 (< 14, too close).  x300 standoff = 295 (>= 14, ok).
    Expected: nearest wall = x=0 with facing 0.0.
    """
    # Node very close to x=0 wall.
    node_xy = (5.0, 100.0)
    wall, facing = projection_edges.reset_autofill(node_xy, "x", _FIELD_MM)

    # Must return the NEAR wall (x=0), not the far qualifying one (x=300).
    assert wall == pytest.approx(0.0)
    assert facing == pytest.approx(0.0)
    # And it is indeed too close (verifying the test geometry is correct).
    assert projection_edges.is_reset_wall_too_close(node_xy, "x", 0.0) is True


def test_tc6_reset_autofill_centered_node_picks_low_wall() -> None:
    """A node equidistant from both walls on the axis resolves to wall ``0.0``
    (the low-wall tie-break), consistent with the within-axis ``wall_rank``
    in the composite sort key."""
    # A 200 cm-wide field on x; node at x=100 -> standoff 100 to both walls.
    field_mm = (2000.0, 2000.0)
    node_xy = (100.0, 100.0)

    wall, facing = projection_edges.reset_autofill(node_xy, "x", field_mm)

    assert wall == pytest.approx(0.0)
    assert facing == pytest.approx(0.0)


# ===========================================================================
# TC8 -- reset type<->spec divergence is rejected at Save
# ===========================================================================


def _model_with_reset_typed_node_no_spec() -> FieldModel:
    """Return a model with a node typed ``"reset"`` but no ``reset_nodes`` entry.

    Every other field is valid, so ``validate()`` fails ONLY on the missing
    reset spec -- isolating the type<->spec invariant under test.

    :return: A :class:`FieldModel` that must fail :meth:`FieldModel.validate`.
    """
    return FieldModel(
        field_mm=(3000.0, 2000.0),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        # ``S`` is a dead-reckon spawn (blind) so the issue #64 validator -- which
        # rejects a ``"line"`` / ``"projection"`` START node -- has a legal start
        # to land on, isolating the TC8 failure on the reset divergence alone.
        start_node="S",
        start_heading_deg=0.0,
        nodes={
            "S": (250.0, 150.0),
            "A": (0.0, 50.0),
            "B": (200.0, 10.0),
            "G": (250.0, 80.0),
        },
        # ``G`` is typed reset but has NO reset_nodes entry -> divergence.
        node_types={"S": "blind", "A": "line", "B": "line", "G": "reset"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G"), ("S", "G")],
        obstacles=[],
        reset_nodes={},  # the missing half of the invariant
    )


def test_tc8_reset_typed_node_without_spec_fails_validate() -> None:
    """A ``"reset"``-typed node with no ``reset_nodes`` entry makes
    ``model.validate()`` raise -- the divergence the inline tier never catches
    is caught at Save."""
    model = _model_with_reset_typed_node_no_spec()

    with pytest.raises(FieldDataError):
        model.validate()


def test_tc8_matching_reset_spec_validates() -> None:
    """Counter-test: adding the matching spec makes the SAME model validate, so
    the TC8 failure is the divergence, not a dead validator."""
    model = _model_with_reset_typed_node_no_spec()
    # G at (250, 80): wall at x=280 -> 30 cm standoff (>= 14), facing 180 deg
    # (cos 180 = -1) points toward decreasing x, away from the wall.
    model.reset_nodes = {
        "G": {"axis": "x", "wall_coord": 280.0, "facing": 180.0}
    }

    model.validate()  # must not raise


# ===========================================================================
# Shared headless app harness
# ===========================================================================


class _PanelsSpy:
    """Minimal stand-in for ``SidePanels`` recording the handler's side effects.

    ``_on_panel_reset_spec_change`` only touches ``set_status`` (via
    ``_set_status``), ``set_validation_badge`` (via ``_reject_inline_edit``),
    and ``refresh`` (via ``_refresh_view``); each call is recorded so a test can
    assert the handler's effects without a real Tk panel.

    ``set_selection_display`` is included because ``_select_node`` (called from
    ``_add_reset_node``) routes through it; recording it is harmless.
    """

    def __init__(self) -> None:
        self.status_messages: list[str] = []
        self.refresh_count: int = 0
        self.validation_badges: list[bool] = []

    def set_status(self, text: str) -> None:
        self.status_messages.append(text)

    def refresh(self, model) -> None:
        self.refresh_count += 1

    def set_validation_badge(self, ok: bool) -> None:
        self.validation_badges.append(ok)

    def set_selection_display(self, text: str) -> None:
        """Accept and discard the selection-line text (not under test here)."""


def _make_app() -> tuple[EditorApp, _PanelsSpy, FieldModel]:
    """Return a bare ``EditorApp`` wired for headless reset-spec handler testing.

    Real ``_history`` / ``_model`` (so a dispatch is observable on both) and a
    ``_PanelsSpy`` for ``_panels``. ``_set_status`` / ``_reject_inline_edit``
    are the REAL ``EditorApp`` methods so the reject path runs end-to-end;
    ``_refresh_view`` and ``_mark_dirty`` are stubbed to drive only the panel
    refresh the handler relies on (the real ``_refresh_view`` also touches the
    canvas + undo/redo enables the spy does not provide).

    The model has a valid reset node ``G`` already (axis x, wall 280, facing
    180) so an axis flip exercises the "axis changed" recompute branch.

    :return: ``(app, panels_spy, model)``.
    """
    model = FieldModel(
        field_mm=(3000.0, 2000.0),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        # ``S`` is a dead-reckon spawn (blind): the issue #64 validator rejects a
        # ``"line"`` / ``"projection"`` START node, so the start must be a blind
        # origin for the handler's Save-path validate to stay clean.
        start_node="S",
        start_heading_deg=0.0,
        nodes={
            "S": (250.0, 150.0),
            "A": (0.0, 50.0),
            "B": (200.0, 10.0),
            "G": (250.0, 80.0),
        },
        node_types={"S": "blind", "A": "line", "B": "line", "G": "reset"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G"), ("S", "G")],
        obstacles=[],
        reset_nodes={"G": {"axis": "x", "wall_coord": 280.0, "facing": 180.0}},
    )
    panels = _PanelsSpy()

    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._selected_node = "G"
    app._mark_dirty = lambda: None
    app._refresh_view = lambda: panels.refresh(model)
    return app, panels, model


# ===========================================================================
# TC6 -- app _on_panel_reset_spec_change recompute behaviour
# ===========================================================================


def test_tc6_axis_change_recomputes_wall_and_facing() -> None:
    """Changing the axis (x -> y) re-derives wall + facing from the autofill,
    ignoring the hand-typed wall/facing that belonged to the old axis."""
    app, panels, model = _make_app()

    # The stored axis is "x"; the operator flips to "y". The typed wall (9999)
    # and facing (123) belong to the old axis and must be DISCARDED in favour of
    # the autofill against the new axis.
    expected_wall, expected_facing = projection_edges.reset_autofill(
        model.nodes["G"], "y", model.field_mm
    )

    app._on_panel_reset_spec_change("G", "y", 9999.0, 123.0)

    spec = model.reset_nodes["G"]
    assert spec["axis"] == "y"
    assert spec["wall_coord"] == pytest.approx(expected_wall)
    assert spec["facing"] == pytest.approx(expected_facing)
    # The recompute discarded the operator's stale wall/facing.
    assert spec["wall_coord"] != pytest.approx(9999.0)
    assert spec["facing"] != pytest.approx(123.0)
    assert app._history.can_undo is True


def test_tc6_axis_unchanged_edit_is_not_clobbered() -> None:
    """A wall/facing hand-edit with the axis UNCHANGED dispatches the operator's
    values verbatim -- the recompute never fires, so a manual edit is kept."""
    app, panels, model = _make_app()

    # Same axis "x" as stored; the operator hand-edits the wall to 285 and the
    # facing to a valid in-range 180 (still away-from-wall). Both must be kept.
    app._on_panel_reset_spec_change("G", "x", 285.0, 180.0)

    spec = model.reset_nodes["G"]
    assert spec["axis"] == "x"
    assert spec["wall_coord"] == pytest.approx(285.0), "manual wall must be kept"
    assert spec["facing"] == pytest.approx(180.0), "manual facing must be kept"
    assert app._history.can_undo is True


def test_tc6_axis_unchanged_out_of_range_facing_is_rejected() -> None:
    """An axis-unchanged hand-edit with an out-of-range facing is rejected
    inline -- no dispatch, the stored spec is unchanged, the badge drops."""
    app, panels, model = _make_app()
    original_spec = dict(model.reset_nodes["G"])

    # facing 400 is outside [0, 360); the unchanged-axis branch range-checks it.
    app._on_panel_reset_spec_change("G", "x", 285.0, 400.0)

    assert model.reset_nodes["G"] == original_spec, "spec unchanged on reject"
    assert app._history.can_undo is False
    assert panels.status_messages, "a reject status message must be set"
    assert panels.validation_badges[-1] is False


# ===========================================================================
# TC7 -- Convert plain -> reset auto-derives the axis (rewrite of existing test)
# ===========================================================================


def test_convert_to_reset_autofills_wall_and_facing() -> None:
    """Flipping a plain node to ``"reset"`` via the inspector type combobox
    seeds the spec against the nearest of all four walls (``reset_autofill_auto``),
    not the old hardcoded axis ``"x"``.

    Node ``B`` at ``(200, 10)`` in a 300x200 cm field:
    - standoffs: x0=200, x300=100, y0=10, y200=190
    - nearest wall: y=0 at standoff 10 cm
    - seeded spec: ``axis="y"``, ``wall_coord=0.0``, ``facing=90.0``
    - standoff 10 < WALL_PIVOT_CLEARANCE_CM (14) -> a too-close status message
      is recorded and the validation badge is set to Invalid.
    """
    app, panels, model = _make_app()

    app._on_panel_node_type_change("B", "reset")

    spec = model.reset_nodes["B"]
    assert model.node_types["B"] == "reset"
    # Auto-axis: nearest wall for (200,10) in 300x200 is y=0 (standoff 10).
    assert spec["axis"] == "y"
    assert spec["wall_coord"] == pytest.approx(0.0)
    assert spec["facing"] == pytest.approx(90.0)
    # undo is available -- the SetResetSpec was dispatched.
    assert app._history.can_undo is True
    # standoff 10 < 14: too-close warning must have been emitted.
    assert any("too close" in m.lower() or "clearance" in m.lower() or "14" in m
               for m in panels.status_messages), (
        f"expected a too-close warning in status_messages, got: "
        f"{panels.status_messages!r}"
    )
    # The badge must have been set Invalid.
    assert panels.validation_badges, "set_validation_badge must have been called"
    assert panels.validation_badges[-1] is False


# ===========================================================================
# TC8 -- Add-node reset auto-derives the axis
# ===========================================================================


def test_tc8_add_reset_node_auto_derives_axis() -> None:
    """Adding a new reset node via ``_add_reset_node`` seeds the same
    nearest-wall spec as the convert path for the same coordinates.

    This test drives ``SetResetSpec.autofilled_auto`` directly (bypassing
    ``_add_reset_node``) to verify the seeded spec for ``(200, 10)`` in a
    300x200 field, and separately calls ``_add_reset_node`` in a wired headless
    app to confirm the spec lands on the model.

    Note: ``_add_reset_node`` also calls ``_select_node`` (which accesses
    ``_panels._selected_node_var``) and ``_reveal_reset_spec_section`` (which
    uses ``getattr(_panels, "_collapsibles", None)`` and returns early when
    absent). ``_select_node`` is stubbed and ``_reveal_reset_spec_section`` is
    stubbed as a no-op to keep the test headless. The ``_warn_if_reset_too_close``
    helper is the real method so the too-close warning is observable.
    """
    # Verify the seeded spec via the classmethod directly.
    cmd = __import__(
        "tools.path_planner.editor.commands", fromlist=["SetResetSpec"]
    ).SetResetSpec.autofilled_auto(
        "R", node_xy=(200.0, 10.0), field_mm=_FIELD_MM
    )
    assert cmd.axis == "y"
    assert cmd.wall_coord == pytest.approx(0.0)
    assert cmd.facing == pytest.approx(90.0)

    # Drive _add_reset_node headlessly to confirm model mutation + warning.
    app, panels, model = _make_app()
    # Stub the post-dispatch selection/reveal calls (they touch real Tk panels).
    app._select_node = lambda name: None
    app._reveal_reset_spec_section = lambda: None

    app._add_reset_node("R", (200.0, 10.0))

    assert "R" in model.nodes
    assert model.node_types["R"] == "reset"
    spec = model.reset_nodes["R"]
    assert spec["axis"] == "y"
    assert spec["wall_coord"] == pytest.approx(0.0)
    assert spec["facing"] == pytest.approx(90.0)
    # Too-close warning: standoff 10 < 14.
    assert any("14" in m or "clearance" in m.lower() for m in panels.status_messages), (
        f"expected too-close warning, got: {panels.status_messages!r}"
    )
    assert panels.validation_badges and panels.validation_badges[-1] is False


# ===========================================================================
# TC9 -- Seed warning fires ONLY when too close
# ===========================================================================


def test_tc9_no_too_close_warning_when_wall_is_far_enough() -> None:
    """Converting a plain node whose nearest wall is >= 14 cm away seeds the
    spec but emits NO too-close warning.

    Node ``P`` at ``(50, 100)`` in 300x200: nearest wall x=0 at standoff 50 cm
    (>= 14). The seeded spec is applied; no too-close status message is recorded
    and the helper does not force the validation badge Invalid.
    """
    # Build a model with an extra plain node at a location well away from all walls.
    model = FieldModel(
        field_mm=(3000.0, 2000.0),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="S",
        start_heading_deg=0.0,
        nodes={
            "S": (250.0, 150.0),
            # P at (50, 100): standoffs x0=50, x300=250, y0=100, y200=100.
            # Nearest wall: x=0 at 50 cm (>= 14); no too-close warning expected.
            "P": (50.0, 100.0),
        },
        node_types={"S": "blind", "P": "line"},
        line_segments=[],
        blind_edges=[("S", "P")],
        obstacles=[],
        reset_nodes={},
    )
    panels = _PanelsSpy()
    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._selected_node = "P"
    app._mark_dirty = lambda: None
    app._refresh_view = lambda: panels.refresh(model)

    app._on_panel_node_type_change("P", "reset")

    assert model.node_types["P"] == "reset"
    spec = model.reset_nodes["P"]
    # Nearest wall for (50, 100) in 300x200: x=0 at standoff 50 -> axis="x",
    # wall=0, facing=0. Standoff 50 >= 14: no too-close warning.
    assert spec["axis"] == "x"
    assert spec["wall_coord"] == pytest.approx(0.0)
    assert spec["facing"] == pytest.approx(0.0)

    # No too-close message should appear.
    too_close_messages = [
        m for m in panels.status_messages
        if "14" in m or "clearance" in m.lower() or "too close" in m.lower()
        or "pivot" in m.lower()
    ]
    assert not too_close_messages, (
        f"unexpected too-close warning for a safe node: "
        f"{too_close_messages!r}"
    )
    # The badge must NOT have been forced Invalid by the helper.
    # (validation_badges might be empty, or the last entry might be True
    #  if _refresh_view triggered something — but the helper must not have
    #  appended False).
    # Since _refresh_view is stubbed and doesn't call set_validation_badge,
    # the spy's validation_badges list must be empty after a safe seed.
    assert not any(v is False for v in panels.validation_badges), (
        f"helper must not force badge Invalid for a safe node; "
        f"validation_badges={panels.validation_badges!r}"
    )


# ===========================================================================
# TC10 -- Manual axis-change re-derives nearest-on-axis + warns
# ===========================================================================


def test_tc10_axis_change_warns_when_nearest_on_axis_is_too_close() -> None:
    """Flipping the axis combobox to a new axis re-derives the spec against the
    nearest wall on the chosen axis. When that nearest wall is under the pivot-
    clearance floor, a too-close status message is recorded.

    Use a model whose node stands very close to an x-wall. Starting with axis
    ``"y"`` stored, flipping to ``"x"`` should derive the x=0 wall (standoff < 14)
    and trigger the too-close warning.

    Node ``N`` at ``(5, 50)`` in 300x200: x0 standoff=5 (< 14), x300=295,
    y0=50, y200=150. Nearest on x-axis = x=0 at standoff 5.
    """
    model = FieldModel(
        field_mm=(3000.0, 2000.0),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="S",
        start_heading_deg=0.0,
        nodes={
            "S": (250.0, 150.0),
            # N at (5, 50): very close to x=0 wall.
            "N": (5.0, 50.0),
        },
        node_types={"S": "blind", "N": "reset"},
        line_segments=[],
        blind_edges=[("S", "N")],
        obstacles=[],
        # N is already reset with axis="y" (y=0 standoff=50, ok). Flip to "x".
        reset_nodes={"N": {"axis": "y", "wall_coord": 0.0, "facing": 90.0}},
    )
    panels = _PanelsSpy()
    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._selected_node = "N"
    app._mark_dirty = lambda: None
    app._refresh_view = lambda: panels.refresh(model)

    # Flip axis to "x" with stale wall/facing from the old y spec.
    app._on_panel_reset_spec_change("N", "x", 9999.0, 123.0)

    spec = model.reset_nodes["N"]
    # Re-derived on axis x: nearest wall is x=0 (standoff=5).
    assert spec["axis"] == "x"
    assert spec["wall_coord"] == pytest.approx(0.0)
    assert spec["facing"] == pytest.approx(0.0)
    # Standoff 5 < 14: too-close warning must fire.
    assert any("14" in m or "clearance" in m.lower() or "5.0" in m or "5 " in m
               for m in panels.status_messages), (
        f"expected too-close warning, got: {panels.status_messages!r}"
    )
    assert panels.validation_badges and panels.validation_badges[-1] is False


def test_tc10_axis_change_no_warning_when_reser2_fix() -> None:
    """Flipping from the wrong axis to ``"y"`` on a reser2-style geometry
    (standoff 19.6 cm >= 14) records NO too-close warning.

    Node at ``(238.8, 19.6)`` in 320x154: stored as axis ``"x"`` (wrong).
    Operator flips to ``"y"``: nearest y-wall is y=0 at standoff 19.6 >= 14.
    Expected: spec updated; no too-close message.
    """
    # Use a 320x154 cm field (the reser2 field): 3200x1540 mm.
    reser2_field_mm = (3200.0, 1540.0)
    model = FieldModel(
        field_mm=reser2_field_mm,
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="S",
        start_heading_deg=0.0,
        nodes={
            "S": (250.0, 100.0),
            "reser2": (238.8, 19.6),
        },
        node_types={"S": "blind", "reser2": "reset"},
        line_segments=[],
        blind_edges=[("S", "reser2")],
        obstacles=[],
        # Stored incorrectly as axis="x", wall=320 (old auto-fill bug).
        reset_nodes={
            "reser2": {"axis": "x", "wall_coord": 320.0, "facing": 180.0}
        },
    )
    panels = _PanelsSpy()
    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._selected_node = "reser2"
    app._mark_dirty = lambda: None
    app._refresh_view = lambda: panels.refresh(model)

    # Operator flips axis to "y" to fix the stored spec.
    app._on_panel_reset_spec_change("reser2", "y", 9999.0, 123.0)

    spec = model.reset_nodes["reser2"]
    assert spec["axis"] == "y"
    assert spec["wall_coord"] == pytest.approx(0.0)
    assert spec["facing"] == pytest.approx(90.0)

    # Standoff 19.6 >= 14: no too-close warning.
    too_close_messages = [
        m for m in panels.status_messages
        if "14" in m or "clearance" in m.lower() or "pivot" in m.lower()
    ]
    assert not too_close_messages, (
        f"unexpected too-close warning for reser2 fix: {too_close_messages!r}"
    )


# ===========================================================================
# TC11 -- Axis-unchanged hand-edit is unaffected (existing behavior preserved)
# ===========================================================================


def test_tc11_axis_unchanged_verbatim_hand_edit_no_too_close_warning() -> None:
    """A wall/facing hand-edit with the SAME axis does NOT trigger the too-close
    helper -- the helper is only called on the axis-CHANGED re-derive path.

    Even if the operator types in a wall that happens to be < 14 cm away, the
    axis-unchanged path dispatches it verbatim and does NOT call
    ``_warn_if_reset_too_close`` (the Save-time validator catches it instead).
    """
    app, panels, model = _make_app()

    # Hand-edit wall to 2.0 (2 cm from x=0, very close) with axis unchanged.
    app._on_panel_reset_spec_change("G", "x", 2.0, 180.0)

    spec = model.reset_nodes["G"]
    # Verbatim hand-edit: the operator's values are stored.
    assert spec["axis"] == "x"
    assert spec["wall_coord"] == pytest.approx(2.0)
    assert spec["facing"] == pytest.approx(180.0)
    assert app._history.can_undo is True

    # No too-close warning from the helper on the axis-unchanged path.
    # (The real _revalidate would catch it at Save, but that's stubbed.)
    too_close_messages = [
        m for m in panels.status_messages
        if "14" in m or "clearance" in m.lower() or "pivot" in m.lower()
    ]
    assert not too_close_messages, (
        f"axis-unchanged path must not emit too-close warning, "
        f"got: {too_close_messages!r}"
    )


def test_tc11_axis_unchanged_out_of_range_facing_still_rejected() -> None:
    """An out-of-range facing on the axis-unchanged path is still rejected inline
    -- the existing AC7 behavior is unaffected by the new warning helper."""
    app, panels, model = _make_app()
    original_spec = dict(model.reset_nodes["G"])

    app._on_panel_reset_spec_change("G", "x", 285.0, 400.0)

    assert model.reset_nodes["G"] == original_spec, "spec unchanged on reject"
    assert app._history.can_undo is False
    assert panels.status_messages, "a reject status message must be set"
    assert panels.validation_badges[-1] is False


# ===========================================================================
# TC12 -- Too-close warning message content
# ===========================================================================


def test_tc12_too_close_warning_message_content() -> None:
    """The too-close status message contains the numeric standoff, the clearance
    value (``"14"``), and is ASCII-only.

    Uses node ``B`` at ``(200, 10)`` in 300x200 cm: nearest wall y=0 at
    standoff 10.0 cm (< 14). The message must contain ``"10.0"`` (formatted
    standoff) and ``"14"`` (the WALL_PIVOT_CLEARANCE_CM constant value), and
    ``message.isascii()`` must be ``True``.
    """
    app, panels, model = _make_app()

    app._on_panel_node_type_change("B", "reset")

    # Find the too-close warning message.
    too_close_messages = [
        m for m in panels.status_messages
        if "14" in m
    ]
    assert too_close_messages, (
        f"no too-close message found; status_messages={panels.status_messages!r}"
    )
    message = too_close_messages[0]
    # Must contain the numeric standoff: B at y=10, wall at y=0 -> standoff 10.0.
    assert "10.0" in message, (
        f"message must contain '10.0' (standoff); got: {message!r}"
    )
    # Must contain the clearance value.
    assert str(int(WALL_PIVOT_CLEARANCE_CM)) in message, (
        f"message must contain '{int(WALL_PIVOT_CLEARANCE_CM)}' (clearance); "
        f"got: {message!r}"
    )
    # Must be ASCII-only (no non-ASCII glyphs like >=, em-dash, etc.).
    assert message.isascii(), (
        f"message must be ASCII-only; got: {message!r}"
    )
