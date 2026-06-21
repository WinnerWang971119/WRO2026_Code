"""Tests for the reset-node auto-fill stack of the editor restyle wave (T9).

Three cases, layered from pure geometry up to the app handler:

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

TC5 and TC8 are pure logic. TC6 drives the real ``EditorApp`` handler, which
imports ``editor.canvas`` (an import-time ``matplotlib.use("TkAgg")``); pin the
headless-safe Agg backend and no-op that one switch for the duration of the
import, exactly as ``test_editor_inspector.py`` does. No real Tk window is built.
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
    back (the larger standoff), which the validator then flags on its clearance
    floor -- the author is never silently handed an unrealisable reset."""
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
        start_node="A",
        start_heading_deg=0.0,
        nodes={"A": (0.0, 50.0), "B": (200.0, 10.0), "G": (250.0, 80.0)},
        # ``G`` is typed reset but has NO reset_nodes entry -> divergence.
        node_types={"A": "line", "B": "line", "G": "reset"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G")],
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
# TC6 -- app _on_panel_reset_spec_change recompute behaviour
# ===========================================================================


class _PanelsSpy:
    """Minimal stand-in for ``SidePanels`` recording the handler's side effects.

    ``_on_panel_reset_spec_change`` only touches ``set_status`` (via
    ``_set_status``), ``set_validation_badge`` (via ``_reject_inline_edit``),
    and ``refresh`` (via ``_refresh_view``); each call is recorded so a test can
    assert the handler's effects without a real Tk panel.
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
        start_node="A",
        start_heading_deg=0.0,
        nodes={"A": (0.0, 50.0), "B": (200.0, 10.0), "G": (250.0, 80.0)},
        node_types={"A": "line", "B": "line", "G": "reset"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G")],
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
