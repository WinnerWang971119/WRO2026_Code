"""Selection, handler, make-straight, and multi-nudge tests for the field
editor's multi-node select + "Make straight" alignment feature (issue #161,
T7 of the plan).

Covers TC13-TC32 and TC37 from the plan's Testing Strategy table (TC11 is a
pure canvas ``find_nodes_in_rect`` test and lives in T8's
``test_editor_marquee_blit.py`` -- it is intentionally NOT duplicated here):

* **TC13-TC16** -- ``_set_selection``, the single writer of selection state:
  set / clear / dedupe-preserving-first-order / filter-against-model.
* **TC17-TC20** -- the modifier-click handlers: shift adds, ctrl toggles, a
  plain click replaces or clears, funnelled through ``_handle_select_press`` /
  ``_apply_modifier_selection`` / ``_handle_select_click``.
* **TC21** -- ``_finish_marquee`` committing a boxed selection.
* **TC22-TC24** -- ``_handle_align_request``: a real dispatch, the no-op
  refusal, and the out-of-bounds refusal.
* **TC25-TC26** -- undo atomicity for a make-straight, and ``MacroCommand``
  rollback on a mid-apply failure -- both against a REAL ``History`` +
  ``FieldModel`` (no mock), per the plan's Style-A carve-out.
* **TC27-TC30** -- ``_nudge_selected``'s rigid multi-nudge: the plain
  3-selected case, the zero-clamp no-shear regression guard, the
  partial-clamp uniform-shift case, and the 1-selected back-compat path.
* **TC31** -- ``_clear_transient_state`` clears the marquee anchor/cursor but
  preserves the node selection across a mode switch.
* **TC32** -- the single ``render`` call site unions ``_edge_selection`` and
  ``_selected_nodes`` into ``highlighted_nodes``.
* **TC37** -- the ALIGN readout recomputes on undo (AC14): the residual
  reported to ``set_align_state`` after an undo reflects the REVERTED model,
  not a stale post-alignment ~0.0.

Style A throughout (per the plan): a bare ``EditorApp.__new__(EditorApp)``
with only the collaborators each code path reads injected as
``SimpleNamespace`` / lambdas / ``_FakeVar``. TC25, TC26, and TC37 are the
documented exception -- they build a REAL ``History`` + REAL ``FieldModel``
to prove undo/rollback atomicity, which a mocked model/history cannot
demonstrate. The matplotlib import dance and the ``_FakeVar`` helper are
copied verbatim from ``test_editor_select_mode.py`` (lines 31-70).

The make-straight / nudge-clamp cases that exercise the real motivating
defect (TC22, TC25, TC27, TC30, TC37) use the REAL x~=50.5 column
coordinates from ``field_data.json`` (``M, MB2, MG2, MW2, MY2, i1``) per the
plan's Testing Strategy note -- an axis-aligned toy fixture would hide the
exact sub-tolerance behaviour this feature targets. The no-op (TC23),
out-of-bounds (TC24), and clamp (TC28/TC29) cases need specific geometry
relative to the field walls / a shared axis to trigger their branch, so they
use small synthetic fixtures instead, each with a comment explaining why the
real column cannot be reused for that case.
"""

from __future__ import annotations

import itertools

import matplotlib

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

from types import SimpleNamespace  # noqa: E402

import pytest  # noqa: E402

from tools.path_planner.editor import align  # noqa: E402
from tools.path_planner.editor import app as app_module  # noqa: E402
from tools.path_planner.editor.commands import MacroCommand, MoveNode  # noqa: E402
from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402


class _FakeVar:
    """A tiny stand-in for a Tk ``StringVar`` (just ``get`` / ``set``).

    The injected panel fakes expose ``mode_var`` / ``_selected_node_var`` as one
    of these so the handlers can read and write the variable without a real Tk
    root. It holds a single string value, defaulting to the empty string.
    """

    def __init__(self, value: str = "") -> None:
        self._value = value

    def get(self) -> str:
        return self._value

    def set(self, value: str) -> None:
        self._value = value


#: The real x~=50.5 column from ``field_data.json`` -- the actual motivating
#: defect (Decisions: "the 5 'aligned' nodes ... sit at x = 50.484010371650825;
#: the mean target is 50.4867"). Used by TC22, TC25, TC27, TC30, TC37.
_REAL_COLUMN_NODES: dict[str, tuple[float, float]] = {
    "M": (50.484010371650825, 69.4522369016751),
    "MB2": (50.484010371650825, 60.95324525661604),
    "MG2": (50.484010371650825, 77.46557188130221),
    "MW2": (50.484010371650825, 93.24941350784047),
    "MY2": (50.5, 45.9),
    "i1": (50.484010371650825, 27.328592007243696),
}

#: The real field's FIELD_MM (matches ``field_data.json``), width_cm=236,
#: height_cm=114 -- wide enough that the column above sits nowhere near a wall.
_REAL_FIELD_MM: tuple[float, float] = (2360.0, 1140.0)


def _make_field_model(nodes: dict[str, tuple[float, float]]) -> FieldModel:
    """Build a small but fully-valid real ``FieldModel`` over *nodes*.

    Mirrors the ``make_model()`` fixture factory in ``test_editor_commands.py``:
    every node is a plain ``"line"`` type, no edges/obstacles/pivots. Used by
    the TC25/TC26/TC37 cases that require a REAL model (not a fake), per the
    plan's Style-A carve-out for undo/rollback atomicity.

    :param nodes: Node name -> ``(x_cm, y_cm)`` to seed ``model.nodes`` with.
    :return: A fully-constructed ``FieldModel``.
    """
    return FieldModel(
        field_mm=_REAL_FIELD_MM,
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node=next(iter(nodes)),
        start_heading_deg=0.0,
        nodes=dict(nodes),
        node_types={name: "line" for name in nodes},
        line_segments=[],
        blind_edges=[],
        obstacles=[],
    )


def _make_selection_app(
    nodes: dict[str, tuple[float, float]],
    *,
    selected_nodes: list[str] | None = None,
    selected_node: str | None = None,
) -> EditorApp:
    """Build a bare ``EditorApp`` with only the selection-writer collaborators.

    Injects a fake ``_model`` (just ``.nodes``), a fake ``_panels`` exposing
    ``_selected_node_var`` (a ``_FakeVar``) and a no-op ``set_selection_display``,
    and stubs ``_refresh_view`` so ``_set_selection`` / the modifier-click
    handlers can be exercised in isolation from the render pipeline (TC32
    exercises the real ``_refresh_view`` separately).

    :param nodes: Node name -> ``(x_cm, y_cm)`` seeding the fake model.
    :param selected_nodes: Pre-existing ``_selected_nodes`` list, if any.
    :param selected_node: Pre-existing ``_selected_node`` latch, if any.
    :return: A ready-to-use bare ``EditorApp``.
    """
    app = EditorApp.__new__(EditorApp)
    app._model = SimpleNamespace(nodes=dict(nodes))
    app._panels = SimpleNamespace(
        _selected_node_var=_FakeVar(selected_node or ""),
        set_selection_display=lambda *_a, **_k: None,
    )
    app._selected_nodes = list(selected_nodes) if selected_nodes else []
    app._selected_node = selected_node
    app._refresh_view = lambda: None
    app._set_status = lambda *_a: None
    return app


def _make_dispatch_app(
    nodes: dict[str, tuple[float, float]],
    field_mm: tuple[float, float] = _REAL_FIELD_MM,
) -> tuple[EditorApp, list, list[str]]:
    """Build a bare ``EditorApp`` with a spy ``_dispatch`` + ``_set_status``.

    Shared by the ``_handle_align_request`` (TC22-TC24) and ``_nudge_selected``
    (TC27-TC30) cases: both dispatch a command and report a status string, and
    neither needs a real canvas / panels / history for their own logic (the
    command-shape assertion is what matters, not the render pipeline).

    :param nodes: Node name -> ``(x_cm, y_cm)`` seeding the fake model.
    :param field_mm: ``(width_mm, height_mm)`` for the fake model's bounds.
    :return: ``(app, dispatch_calls, statuses)`` -- *dispatch_calls* records
        every command passed to ``_dispatch`` (which always returns ``True``);
        *statuses* records every ``_set_status`` string.
    """
    app = EditorApp.__new__(EditorApp)
    app._model = SimpleNamespace(nodes=dict(nodes), field_mm=field_mm)
    dispatch_calls: list = []
    app._dispatch = lambda command: dispatch_calls.append(command) or True
    statuses: list[str] = []
    app._set_status = statuses.append
    return app, dispatch_calls, statuses


# ---------------------------------------------------------------------------
# TC13-TC16 -- ``_set_selection``, the single writer of selection state.
# ---------------------------------------------------------------------------


def test_set_selection_sets_list_anchor_and_panel_var() -> None:
    """TC13: ``_set_selection(["A","B","C"])`` sets ``_selected_nodes`` to the
    full ordered list, ``_selected_node`` to the LAST entry, and mirrors that
    anchor into the panel's ``_selected_node_var``."""
    app = _make_selection_app({"A": (0.0, 0.0), "B": (1.0, 1.0), "C": (2.0, 2.0)})

    app._set_selection(["A", "B", "C"])

    assert app._selected_nodes == ["A", "B", "C"]
    assert app._selected_node == "C"
    assert app._panels._selected_node_var.get() == "C"


def test_set_selection_empty_clears_everything() -> None:
    """TC14: ``_set_selection([])`` clears ``_selected_nodes`` to ``[]``,
    ``_selected_node`` to ``None``, and blanks the panel var."""
    app = _make_selection_app(
        {"A": (0.0, 0.0)}, selected_nodes=["A"], selected_node="A"
    )

    app._set_selection([])

    assert app._selected_nodes == []
    assert app._selected_node is None
    assert app._panels._selected_node_var.get() == ""


def test_set_selection_dedupes_preserving_first_order() -> None:
    """TC15: ``_set_selection(["A","B","A"])`` dedupes the repeated ``"A"``,
    keeping its FIRST position -- the documented rule -- rather than dropping
    the first occurrence or reordering to the duplicate's position."""
    app = _make_selection_app({"A": (0.0, 0.0), "B": (1.0, 1.0)})

    app._set_selection(["A", "B", "A"])

    assert app._selected_nodes == ["A", "B"]
    assert app._selected_node == "B"


def test_set_selection_filters_names_absent_from_model() -> None:
    """TC16: ``_set_selection(["A","ghost"])`` drops ``"ghost"`` (not in
    ``model.nodes``) before it ever reaches ``_selected_nodes``."""
    app = _make_selection_app({"A": (0.0, 0.0), "B": (1.0, 1.0)})

    app._set_selection(["A", "ghost"])

    assert app._selected_nodes == ["A"]
    assert app._selected_node == "A"


# ---------------------------------------------------------------------------
# TC17-TC20 -- modifier-click handlers (shift add / ctrl toggle / plain
# replace / plain clear).
# ---------------------------------------------------------------------------


def test_shift_click_unselected_node_appends_to_selection() -> None:
    """TC17: a shift-click (``key="shift"``) on an unselected node APPENDS it
    to the current selection -- the prior selection is retained in full."""
    app = _make_selection_app(
        {"A": (0.0, 0.0), "B": (1.0, 1.0), "C": (2.0, 2.0)},
        selected_nodes=["A", "B"],
        selected_node="B",
    )
    app._canvas = SimpleNamespace(find_node_at=lambda x, y: "C")

    app._handle_select_press(5.0, 5.0, key="shift")

    assert app._selected_nodes == ["A", "B", "C"]
    assert app._selected_node == "C"


def test_ctrl_click_selected_node_removes_it_keeps_rest() -> None:
    """TC18: a ctrl-click (``key="control"``) on an ALREADY-selected node
    TOGGLES it off -- it is removed while every other selected node is
    retained (order preserved)."""
    app = _make_selection_app(
        {"A": (0.0, 0.0), "B": (1.0, 1.0), "C": (2.0, 2.0)},
        selected_nodes=["A", "B", "C"],
        selected_node="C",
    )
    app._canvas = SimpleNamespace(find_node_at=lambda x, y: "B")

    app._handle_select_press(5.0, 5.0, key="control")

    assert app._selected_nodes == ["A", "C"]


def test_plain_click_with_prior_selection_replaces_it() -> None:
    """TC19: a PLAIN click (no modifier) on a node, with 3 nodes already
    selected, REPLACES the whole selection with just the clicked node."""
    app = _make_selection_app(
        {"A": (0.0, 0.0), "B": (1.0, 1.0), "C": (2.0, 2.0), "D": (3.0, 3.0)},
        selected_nodes=["A", "B", "C"],
        selected_node="C",
    )
    app._canvas = SimpleNamespace(find_node_at=lambda x, y: "D")

    app._handle_select_press(5.0, 5.0, key=None)

    assert app._selected_nodes == ["D"]
    assert app._selected_node == "D"


def test_plain_click_on_empty_canvas_clears_selection() -> None:
    """TC20: a plain click (invoked directly as ``_handle_select_click``, the
    single-node latch behind the press handler's plain on-node branch) on
    EMPTY canvas CLEARS the selection entirely."""
    app = _make_selection_app(
        {"A": (0.0, 0.0)}, selected_nodes=["A"], selected_node="A"
    )
    app._canvas = SimpleNamespace(find_node_at=lambda x, y: None)

    app._handle_select_click(5.0, 5.0)

    assert app._selected_nodes == []
    assert app._selected_node is None


# ---------------------------------------------------------------------------
# TC21 -- marquee release commits the boxed selection exactly once.
# ---------------------------------------------------------------------------


def test_finish_marquee_selects_boxed_nodes_and_tears_down_once() -> None:
    """TC21: a marquee release boxing 4 nodes calls ``_set_selection`` with
    exactly those 4 names, and ``end_marquee_drag`` fires exactly once (the
    blit fast-path teardown, mirroring the obstacle rubber-band)."""
    app = EditorApp.__new__(EditorApp)
    app._marquee_anchor = (1.0, 2.0)
    app._marquee_cursor = (5.0, 6.0)
    end_calls: list[None] = []
    app._canvas = SimpleNamespace(
        end_marquee_drag=lambda: end_calls.append(None),
        find_nodes_in_rect=lambda x0, y0, x1, y1: ["A", "B", "C", "D"],
    )
    set_selection_calls: list[list[str]] = []
    app._set_selection = set_selection_calls.append
    app._report_selection_status = lambda: None

    app._finish_marquee(5.0, 6.0, key=None)

    assert set_selection_calls == [["A", "B", "C", "D"]]
    assert len(end_calls) == 1
    assert app._marquee_anchor is None
    assert app._marquee_cursor is None


# ---------------------------------------------------------------------------
# TC22-TC24 -- ``_handle_align_request``: success dispatch, no-op refusal,
# out-of-bounds refusal.
# ---------------------------------------------------------------------------


def test_handle_align_request_dispatches_one_macro_of_six_move_nodes() -> None:
    """TC22: make-straight on the real 6-node x~=50.5 column dispatches ONE
    ``MacroCommand`` holding exactly 6 ``MoveNode`` steps -- one per selected
    node -- matching what :func:`align.align_points` computes directly (the
    oracle), so the assertion never hardcodes the target scalar."""
    app, dispatch_calls, _statuses = _make_dispatch_app(_REAL_COLUMN_NODES)
    app._selected_nodes = list(_REAL_COLUMN_NODES)

    app._handle_align_request("auto")

    assert len(dispatch_calls) == 1
    command = dispatch_calls[0]
    assert isinstance(command, MacroCommand)
    assert len(command.commands) == 6
    assert all(isinstance(step, MoveNode) for step in command.commands)

    expected_targets = align.align_points(
        _REAL_COLUMN_NODES, align.AlignAxis.AUTO
    )
    moved = {step.name: step.new_xy for step in command.commands}
    assert moved == expected_targets
    assert set(moved) == set(_REAL_COLUMN_NODES)


def test_handle_align_request_noop_when_already_collinear() -> None:
    """TC23: make-straight on a selection ALREADY at its rounded target
    dispatches nothing and reports the exact refusal string.

    Synthetic fixture (not the real column): the real column is deliberately
    NOT already at its rounded mean -- that gap is the motivating defect this
    feature exists to fix -- so a genuine no-op needs two nodes that already
    share one axis exactly.
    """
    nodes = {"P": (50.0, 10.0), "Q": (50.0, 20.0)}
    app, dispatch_calls, statuses = _make_dispatch_app(nodes)
    app._selected_nodes = ["P", "Q"]

    app._handle_align_request("auto")

    assert dispatch_calls == []
    assert statuses == ["Already straight. Nothing moved."]
    assert statuses == [app_module._STATUS_ALIGN_NOOP]


def test_handle_align_request_fit_out_of_bounds_refuses_and_leaves_model() -> None:
    """TC24: a ``FIT`` alignment whose projected line would push a node
    outside the field bounds refuses the WHOLE operation -- no dispatch, the
    exact refusal string, and the model dict untouched.

    Synthetic fixture on a small field: a near-vertical run with one outlier
    (``D``) whose total-least-squares projection lands node ``A`` at
    x = -0.6999 -- verified below via ``align.align_points`` directly -- which
    is outside ``[0, 20]`` on this deliberately small field. The real column
    sits safely mid-field, so it cannot exercise this branch.
    """
    nodes = {
        "A": (1.0, 0.0),
        "B": (1.0, 5.0),
        "C": (1.0, 10.0),
        "D": (9.5, 15.0),
    }
    original_nodes = dict(nodes)
    # Sanity: prove the fixture actually drives a node out of bounds via the
    # real geometry, so the refusal below is exercising the true branch.
    fit_targets = align.align_points(nodes, align.AlignAxis.FIT)
    assert not (0.0 <= fit_targets["A"][0] <= 20.0)

    app, dispatch_calls, statuses = _make_dispatch_app(
        nodes, field_mm=(200.0, 150.0)
    )
    app._selected_nodes = ["A", "B", "C", "D"]

    app._handle_align_request("fit")

    assert dispatch_calls == []
    assert statuses == [
        "Make straight: alignment would leave the field. Nothing moved."
    ]
    assert statuses == [app_module._STATUS_ALIGN_OUT_OF_BOUNDS]
    assert app._model.nodes == original_nodes


# ---------------------------------------------------------------------------
# TC25-TC26 -- undo atomicity + MacroCommand rollback, REAL History + REAL
# FieldModel (no mock).
# ---------------------------------------------------------------------------


def test_undo_after_make_straight_restores_all_six_original_coords() -> None:
    """TC25: with a REAL ``History`` + REAL ``FieldModel``, one ``undo()``
    after a make-straight restores every one of the 6 moved nodes' original
    coordinates exactly -- proving the whole straighten is ONE undo entry."""
    model = _make_field_model(_REAL_COLUMN_NODES)
    history = History()
    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = history
    app._selected_nodes = list(_REAL_COLUMN_NODES)
    # Isolate the History/Command mechanics from the render pipeline -- the
    # dispatch -> history.do -> mark-dirty path is what TC25 verifies, not
    # canvas/panel rendering.
    app._refresh_view = lambda: None
    app._mark_dirty = lambda: None
    app._set_status = lambda *_a: None

    app._handle_align_request("auto")

    # Sanity: the alignment actually moved all six nodes onto the shared x.
    assert all(
        model.nodes[name][0] == pytest.approx(50.4867) for name in _REAL_COLUMN_NODES
    )

    assert history.undo(model) is True
    for name, xy in _REAL_COLUMN_NODES.items():
        assert model.nodes[name] == xy


def test_macro_command_rollback_on_missing_node_restores_applied_prefix() -> None:
    """TC26: a ``MacroCommand`` whose last ``MoveNode`` names a node absent
    from the model raises, and every ALREADY-APPLIED node in the prefix is
    restored to its original coordinate -- against a REAL ``History`` + REAL
    ``FieldModel``, proving the rollback (not the app layer)."""
    model = _make_field_model({"A": (1.0, 1.0), "B": (2.0, 2.0)})
    history = History()
    macro = MacroCommand(
        [
            MoveNode("A", (10.0, 10.0)),
            MoveNode("B", (20.0, 20.0)),
            MoveNode("ghost", (30.0, 30.0)),
        ]
    )

    with pytest.raises(KeyError):
        history.do(macro, model)

    assert model.nodes["A"] == (1.0, 1.0)
    assert model.nodes["B"] == (2.0, 2.0)
    assert history.can_undo is False


# ---------------------------------------------------------------------------
# TC27-TC30 -- ``_nudge_selected``'s rigid multi-nudge.
# ---------------------------------------------------------------------------


def test_nudge_selected_three_nodes_rigid_macro_identical_delta() -> None:
    """TC27: an arrow-key nudge with 3 selected, clear of every wall, moves
    all 3 by the IDENTICAL requested delta in ONE ``MacroCommand``."""
    nodes = {
        "M": _REAL_COLUMN_NODES["M"],
        "MB2": _REAL_COLUMN_NODES["MB2"],
        "MG2": _REAL_COLUMN_NODES["MG2"],
    }
    app, dispatch_calls, _statuses = _make_dispatch_app(nodes)
    app._selected_node = None
    app._selected_nodes = list(nodes)

    app._nudge_selected(1.0, -0.5)

    assert len(dispatch_calls) == 1
    command = dispatch_calls[0]
    assert isinstance(command, MacroCommand)
    assert len(command.commands) == 3
    assert all(isinstance(step, MoveNode) for step in command.commands)
    moved = {step.name: step.new_xy for step in command.commands}
    for name, (x, y) in nodes.items():
        assert moved[name] == (x + 1.0, y - 0.5)


def test_nudge_selected_clamps_to_zero_at_wall_no_dispatch_no_shear() -> None:
    """TC28: nudging LEFT when one node sits flush against ``x=0`` clamps the
    RIGID delta to zero (never a per-node clamp), so NOTHING is dispatched --
    the no-shear regression guard for AC9. Asserts the model is byte-identical
    afterward (no partial/sheared move slipped through).

    Synthetic fixture: needs a node pinned exactly at the wall to force the
    clamp, which the real column (safely mid-field) cannot provide.
    """
    nodes = {"A": (0.0, 50.0), "B": (10.0, 55.0), "C": (20.0, 45.0)}
    app, dispatch_calls, _statuses = _make_dispatch_app(nodes)
    app._selected_node = None
    app._selected_nodes = list(nodes)

    app._nudge_selected(-1.0, 0.0)

    assert dispatch_calls == []
    assert app._model.nodes == nodes


def test_nudge_selected_partial_clamp_shifts_whole_selection_uniformly() -> None:
    """TC29: when the requested delta partially clamps (one node is close to,
    but not flush against, a wall), EVERY node moves by the SAME reduced
    delta -- relative spacing between every pair is byte-identical before and
    after, proving the rigid (not per-node) clamp.

    Synthetic fixture: needs a node close enough to a wall to force a
    non-zero partial clamp at a known reduced delta.
    """
    nodes = {"A": (2.0, 50.0), "B": (10.0, 55.0), "C": (20.0, 45.0)}
    app, dispatch_calls, _statuses = _make_dispatch_app(nodes)
    app._selected_node = None
    app._selected_nodes = list(nodes)

    # Requested dx=-5.0 exceeds the 2.0 cm of room left of A (the leftmost
    # node); the rigid clamp reduces it to exactly -2.0 for THE WHOLE SELECTION.
    app._nudge_selected(-5.0, 0.0)

    assert len(dispatch_calls) == 1
    command = dispatch_calls[0]
    assert isinstance(command, MacroCommand)
    moved = {step.name: step.new_xy for step in command.commands}
    for name, (x, y) in nodes.items():
        assert moved[name] == (x - 2.0, y)

    # Relative spacing preserved byte-identically -- every pairwise delta
    # between movers is unchanged by the rigid, once-clamped shift.
    for a, b in itertools.combinations(nodes, 2):
        orig_dx = nodes[a][0] - nodes[b][0]
        orig_dy = nodes[a][1] - nodes[b][1]
        new_dx = moved[a][0] - moved[b][0]
        new_dy = moved[a][1] - moved[b][1]
        assert new_dx == orig_dx
        assert new_dy == orig_dy


def test_nudge_selected_single_node_dispatches_plain_move_node() -> None:
    """TC30: an arrow-key nudge with exactly 1 selected dispatches a single
    ``MoveNode`` -- NOT a ``MacroCommand`` -- the back-compat path unchanged
    from before multi-select existed."""
    name = "M"
    nodes = {name: _REAL_COLUMN_NODES[name]}
    app, dispatch_calls, _statuses = _make_dispatch_app(nodes)
    app._selected_node = name
    app._selected_nodes = [name]

    app._nudge_selected(1.0, -2.0)

    assert len(dispatch_calls) == 1
    command = dispatch_calls[0]
    assert isinstance(command, MoveNode)
    assert not isinstance(command, MacroCommand)
    assert command.name == name
    x, y = nodes[name]
    assert command.new_xy == (x + 1.0, y - 2.0)


# ---------------------------------------------------------------------------
# TC31 -- ``_clear_transient_state`` clears the marquee but preserves
# selection.
# ---------------------------------------------------------------------------


def test_clear_transient_state_clears_marquee_keeps_selection() -> None:
    """TC31: after a mode switch mid-marquee, ``_clear_transient_state``
    resets ``_marquee_anchor`` / ``_marquee_cursor`` to ``None`` but leaves
    ``_selected_nodes`` completely untouched."""
    app = EditorApp.__new__(EditorApp)
    app._drag_target = None
    app._drag_origin_xy = None
    app._obstacle_anchor = None
    app._obstacle_cursor = None
    app._marquee_anchor = (3.0, 4.0)
    app._marquee_cursor = (5.0, 6.0)
    app._edge_selection = ["X"]
    app._proposed_edges = None
    app._selected_nodes = ["A", "B", "C"]

    app._clear_transient_state()

    assert app._marquee_anchor is None
    assert app._marquee_cursor is None
    assert app._selected_nodes == ["A", "B", "C"]


# ---------------------------------------------------------------------------
# TC32 -- the single ``render`` call site unions edge-selection + node
# selection.
# ---------------------------------------------------------------------------


def test_refresh_view_unions_edge_selection_and_node_selection_for_render() -> None:
    """TC32: the ``_refresh_view`` render call site passes ``highlighted_nodes``
    as the union (de-duplicated, order-preserving) of ``_edge_selection`` and
    ``_selected_nodes`` -- proving selection finally draws (AC3) alongside the
    pre-existing edge-mode endpoint highlight."""
    app = EditorApp.__new__(EditorApp)
    render_calls: list[dict] = []
    app._canvas = SimpleNamespace(
        render=lambda model, **kwargs: render_calls.append(kwargs)
    )
    app._model = SimpleNamespace(nodes={})
    app._edge_selection = ["E1", "E2"]
    app._selected_nodes = ["N1", "E1"]  # E1 overlaps -- proves de-dup
    app._proposed_edges = None
    app._align_preview = None
    app._compute_align_state = lambda: None
    app._panels = SimpleNamespace(
        refresh=lambda *_a, **_k: None,
        set_undo_enabled=lambda *_a: None,
        set_redo_enabled=lambda *_a: None,
    )
    app._history = SimpleNamespace(can_undo=False, can_redo=False)
    app._update_title = lambda: None
    app._revalidate = lambda: None

    app._refresh_view()

    assert len(render_calls) == 1
    assert render_calls[0]["highlighted_nodes"] == ["E1", "E2", "N1"]
    assert render_calls[0]["proposed_edges"] is None
    assert render_calls[0]["align_preview"] is None


# ---------------------------------------------------------------------------
# TC37 -- ALIGN readout after undo (AC14): no stale post-alignment residual.
# ---------------------------------------------------------------------------


def test_align_readout_after_undo_reflects_reverted_model_not_stale_zero() -> None:
    """TC37: after ``_handle_align_request`` then a real ``_undo()``, the
    LAST ``set_align_state`` residual reflects the REVERTED (pre-alignment)
    model's true residual (~0.0133 cm on the real column), not the stale
    post-alignment ~0.0 -- proving ``_compute_align_state`` re-fires on undo.

    Uses a REAL ``History`` + REAL ``FieldModel`` (undo must actually mutate
    node coordinates for the readout to have anything real to reflect) with a
    fake ``_panels`` that only records ``set_align_state`` calls.
    """
    model = _make_field_model(_REAL_COLUMN_NODES)
    history = History()
    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = history
    app._selected_nodes = list(_REAL_COLUMN_NODES)
    app._align_axis = align.AlignAxis.AUTO
    app._edge_selection = []
    app._proposed_edges = None

    align_state_calls: list[tuple] = []
    app._panels = SimpleNamespace(
        set_align_state=lambda *args: align_state_calls.append(args),
        refresh=lambda *_a, **_k: None,
        set_undo_enabled=lambda *_a: None,
        set_redo_enabled=lambda *_a: None,
        set_dirty=lambda *_a: None,
    )
    app._canvas = SimpleNamespace(render=lambda *_a, **_k: None)
    app._set_status = lambda *_a: None
    app._update_title = lambda: None
    app._revalidate = lambda: None

    app._handle_align_request("auto")
    # After a successful straighten the residual collapses to (near) zero.
    post_align_residual = align_state_calls[-1][2]
    assert post_align_residual == pytest.approx(0.0, abs=1e-9)

    app._undo()

    # The undo re-ran _compute_align_state via _refresh_view; the LAST
    # recorded residual must reflect the REVERTED model, not the stale
    # post-alignment ~0.0.
    reverted_residual = align_state_calls[-1][2]
    expected_residual = align.max_residual_cm(
        _REAL_COLUMN_NODES, align.AlignAxis.AUTO
    )
    assert reverted_residual == pytest.approx(expected_residual)
    assert reverted_residual > 0.01  # sanity: materially different from ~0
    for name, xy in _REAL_COLUMN_NODES.items():
        assert model.nodes[name] == xy
