"""App-level tests for the inspector inline guards + arrow-key nudge (T9).

Covers the real ``EditorApp`` handlers that back the property inspector and the
keyboard nudge, exercised through a bare ``EditorApp.__new__`` instance with the
non-Tk collaborators injected -- the same headless pattern as
``test_editor_app_renames.py``. No real Tk window is constructed.

Test cases (from the plan):

* **TC8** -- inline reject: each handler refuses a bad input BEFORE dispatch
  (no new history entry, model byte-identical), writes a status message to the
  panel spy, and re-syncs the widget via ``panels.refresh`` / ``_refresh_view``.
* **TC9** -- arrow nudge: ``_nudge_selected`` moves the selected node by exactly
  the fine / coarse step, clamps at the field bounds (no empty undo at a corner),
  the focus-guard suppresses a nudge typed into an Entry, and a no-selection
  nudge sets a status message without dispatching.

``EditorApp`` builds a Tk window in ``__init__``; Tk is flaky / absent under
pytest, so we never call it. Importing ``EditorApp`` pulls in ``editor.canvas``,
which calls ``matplotlib.use("TkAgg")`` at import time -- on headless CI that
raises. Pin the Agg backend and no-op that one switch for the duration of the
import (mirrors ``test_editor_app_renames.py``).
"""

from __future__ import annotations

import types

import matplotlib
import pytest

matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor.app import (  # noqa: E402
        EditorApp,
        _NUDGE_COARSE_CM,
        _NUDGE_FINE_CM,
        _STATUS_NUDGE_NO_SELECTION,
    )
finally:
    matplotlib.use = _real_mpl_use

from tools.path_planner.editor import dialogs  # noqa: E402
from tools.path_planner.editor.dialogs import ShortcutsDialog  # noqa: E402
from tools.path_planner.editor.history import History  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402


# ---------------------------------------------------------------------------
# Headless harness
# ---------------------------------------------------------------------------


class _PanelsSpy:
    """Minimal stand-in for ``SidePanels`` recording the handler's side effects.

    The inspector handlers and the nudge only touch a handful of panel methods:
    ``set_status`` / ``refresh`` / ``set_validation_badge`` (and, for symmetry
    with the real surface, ``set_mode_display`` / ``set_selection_display``).
    Each call is recorded so a test can assert it fired without a real Tk panel.
    """

    def __init__(self) -> None:
        self.status_messages: list[str] = []
        self.refresh_count: int = 0
        self.validation_badges: list[bool] = []
        self.mode_displays: list[str] = []
        self.selection_displays: list[str] = []

    def set_status(self, text: str) -> None:
        self.status_messages.append(text)

    def refresh(self, model) -> None:
        self.refresh_count += 1

    def set_validation_badge(self, ok: bool) -> None:
        self.validation_badges.append(ok)

    def set_mode_display(self, text: str) -> None:
        self.mode_displays.append(text)

    def set_selection_display(self, text: str) -> None:
        self.selection_displays.append(text)


def make_model() -> FieldModel:
    """Return a small, fully-valid :class:`FieldModel` built in memory.

    ``A`` -- ``B`` is a line segment and ``B`` -- ``G`` a blind edge, so
    ``(A, B, G)`` is a valid pivot (both legs incident); ``A`` sits at x=0 for
    the left-edge clamp test.
    """
    return FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="A",
        start_heading_deg=0.0,
        nodes={"A": (0.0, 50.0), "B": (200.0, 10.0), "G": (250.0, 80.0)},
        node_types={"A": "line", "B": "line", "G": "blind"},
        line_segments=[("A", "B")],
        blind_edges=[("B", "G")],
        obstacles=[],
    )


def make_app() -> tuple[EditorApp, _PanelsSpy, FieldModel]:
    """Return a bare ``EditorApp`` wired for headless handler testing.

    Real ``_history`` and ``_model`` (so a dispatch is observable on both), a
    ``_PanelsSpy`` for ``_panels``, and the Tk-touching helpers stubbed. The
    inline-reject path needs ``set_status`` / ``set_validation_badge`` /
    ``refresh`` -- all on the spy -- and ``_mark_dirty``; the latter is stubbed.

    :return: ``(app, panels_spy, model)``.
    """
    model = make_model()
    panels = _PanelsSpy()

    app = EditorApp.__new__(EditorApp)
    app._model = model
    app._history = History()
    app._panels = panels
    app._selected_node = None
    app._mark_dirty = lambda: None
    # ``_set_status`` and ``_refresh_view`` are the real EditorApp methods; they
    # delegate to the panel spy (``set_status`` / ``refresh``) but ``_refresh_view``
    # also calls the canvas + undo/redo enables, which the spy does not provide.
    # Stub ``_refresh_view`` to drive only the panel refresh the handlers rely on,
    # so a real ``_dispatch`` / ``_reject_inline_edit`` runs end-to-end.
    app._refresh_view = lambda: panels.refresh(model)
    return app, panels, model


# ---------------------------------------------------------------------------
# TC8 -- inline reject: no dispatch, status set, widget reverted, model intact
# ---------------------------------------------------------------------------


def test_reset_spec_bad_axis_is_rejected_inline() -> None:
    """TC8: a reset-spec change with axis 'z' (not in {x, y}) is rejected before
    dispatch -- no history entry, the model is byte-identical, a status message is
    set, and the panel is refreshed to revert the widget."""
    app, panels, model = make_app()
    pre = repr(model.to_json_dict())

    app._on_panel_reset_spec_change("G", "z", 280.0, 180.0)

    assert app._history.can_undo is False
    assert repr(model.to_json_dict()) == pre
    assert "G" not in model.reset_nodes
    assert model.node_types["G"] == "blind"
    assert panels.status_messages, "a status message must be set"
    assert panels.validation_badges[-1] is False
    assert panels.refresh_count >= 1, "the widget must be reverted via refresh"


def test_global_knob_negative_k_is_rejected_inline() -> None:
    """TC8: a global-knob change with k < 0 is rejected before dispatch -- no
    history entry, the knob attribute is unchanged, a status message is set, and
    the panel is refreshed to revert the widget."""
    app, panels, model = make_app()
    original_k = model.blind_decay_k
    pre = repr(model.to_json_dict())

    app._on_panel_global_knob_change("blind_decay_k", -1.0)

    assert app._history.can_undo is False
    assert model.blind_decay_k == original_k
    assert repr(model.to_json_dict()) == pre
    assert panels.status_messages
    assert panels.validation_badges[-1] is False
    assert panels.refresh_count >= 1


def test_add_pivot_unknown_node_is_rejected_inline() -> None:
    """TC8: an add-pivot referencing an unknown node is rejected before dispatch --
    no history entry, pivot_turns unchanged, a status message is set, and the
    validation badge drops to INVALID."""
    app, panels, model = make_app()
    pre = repr(model.to_json_dict())

    app._on_panel_add_pivot("A", "B", "ZZZ")

    assert app._history.can_undo is False
    assert model.pivot_turns == []
    assert repr(model.to_json_dict()) == pre
    assert panels.status_messages
    assert panels.validation_badges[-1] is False


def test_add_pivot_non_incident_leg_is_rejected_inline() -> None:
    """TC8: an add-pivot whose legs are not real incident edges is rejected before
    dispatch -- ``(A, G, B)`` needs edges A-G and G-B, neither of which exists --
    so no history entry, pivot_turns unchanged, status set, badge INVALID."""
    app, panels, model = make_app()
    # A-G is not an edge (A-B and B-G are), so the arrive leg is non-incident.
    assert not app._edge_exists("A", "G")
    pre = repr(model.to_json_dict())

    app._on_panel_add_pivot("A", "G", "B")

    assert app._history.can_undo is False
    assert model.pivot_turns == []
    assert repr(model.to_json_dict()) == pre
    assert panels.status_messages
    assert panels.validation_badges[-1] is False


def test_inline_accept_path_does_dispatch() -> None:
    """Sanity counter-test: a WELL-FORMED inline edit DOES dispatch, so the reject
    assertions above are meaningful (not vacuously passing on a dead handler)."""
    app, panels, model = make_app()

    app._on_panel_global_knob_change("blind_decay_k", 0.25)

    assert app._history.can_undo is True
    assert model.blind_decay_k == 0.25


# ---------------------------------------------------------------------------
# TC9 -- arrow-key nudge
# ---------------------------------------------------------------------------


def test_nudge_moves_node_by_fine_step() -> None:
    """TC9: a fine-step nudge moves the selected node by exactly 0.2 cm."""
    app, panels, model = make_app()
    app._selected_node = "B"
    bx, by = model.nodes["B"]

    app._nudge_selected(_NUDGE_FINE_CM, 0.0)

    assert model.nodes["B"] == pytest.approx((bx + _NUDGE_FINE_CM, by))
    assert _NUDGE_FINE_CM == pytest.approx(0.2)
    assert app._history.can_undo is True


def test_nudge_moves_node_by_coarse_step() -> None:
    """TC9: a coarse-step nudge moves the selected node by exactly 1.0 cm."""
    app, panels, model = make_app()
    app._selected_node = "B"
    bx, by = model.nodes["B"]

    app._nudge_selected(0.0, _NUDGE_COARSE_CM)

    assert model.nodes["B"] == pytest.approx((bx, by + _NUDGE_COARSE_CM))
    assert _NUDGE_COARSE_CM == pytest.approx(1.0)
    assert app._history.can_undo is True


def test_nudge_clamps_at_field_left_bound_and_is_noop() -> None:
    """TC9: nudging a node already at x=0 further LEFT clamps to 0 -- the node does
    not move and no empty undo entry is created."""
    app, panels, model = make_app()
    app._selected_node = "A"  # A is at x=0
    assert model.nodes["A"][0] == 0.0
    pre = repr(model.to_json_dict())

    app._nudge_selected(-_NUDGE_FINE_CM, 0.0)

    assert model.nodes["A"][0] == 0.0  # clamped, unmoved
    assert repr(model.to_json_dict()) == pre
    assert app._history.can_undo is False  # no empty undo entry


def test_nudge_with_no_selection_sets_status_and_does_not_dispatch() -> None:
    """TC9: a nudge with no node selected writes the no-selection status and does
    not dispatch (no history entry)."""
    app, panels, model = make_app()
    app._selected_node = None
    pre = repr(model.to_json_dict())

    app._nudge_selected(_NUDGE_FINE_CM, 0.0)

    assert app._history.can_undo is False
    assert repr(model.to_json_dict()) == pre
    assert _STATUS_NUDGE_NO_SELECTION in panels.status_messages


def test_focus_guard_suppresses_nudge_while_typing_in_entry() -> None:
    """TC9: the focus-guard returns False for an event whose widget reports the Tk
    class ``"Entry"`` -- so a nudge handler wrapped by the guard stays inert while
    the operator is typing into a text field (#67)."""
    # A fake Tk event whose widget identifies as an Entry (text-input class).
    fake_entry_widget = types.SimpleNamespace(winfo_class=lambda: "Entry")
    entry_event = types.SimpleNamespace(widget=fake_entry_widget)

    assert EditorApp._shortcut_focus_allows(entry_event) is False

    # A non-text widget (the canvas) allows the shortcut, proving the guard is
    # selective rather than always-deny.
    fake_canvas_widget = types.SimpleNamespace(winfo_class=lambda: "Canvas")
    canvas_event = types.SimpleNamespace(widget=fake_canvas_widget)
    assert EditorApp._shortcut_focus_allows(canvas_event) is True


def test_focus_guard_blocks_actual_nudge_dispatch_in_entry() -> None:
    """TC9: end-to-end -- a guarded nudge handler does NOT move the node when the
    focused widget is an Entry, even with a node selected."""
    app, panels, model = make_app()
    app._selected_node = "B"
    pre = repr(model.to_json_dict())

    # Build the same wrapped handler _wire_keyboard_shortcuts binds, then fire it
    # with an Entry-focused event: the guard must short-circuit before the nudge.
    handler = app._make_nudge_handler(_NUDGE_FINE_CM, 0.0)
    guarded = app._shortcut_guard(handler)
    fake_entry_widget = types.SimpleNamespace(winfo_class=lambda: "Entry")
    guarded(types.SimpleNamespace(widget=fake_entry_widget))

    assert repr(model.to_json_dict()) == pre
    assert app._history.can_undo is False


# ---------------------------------------------------------------------------
# TC7 -- the add-node dialog offers the "reset" type (module-scoped values)
# ---------------------------------------------------------------------------


def test_dialog_node_type_values_offer_reset() -> None:
    """TC7: ``dialogs._NODE_TYPE_VALUES`` is exactly ``("line", "blind",
    "reset")`` -- the add-node dialog offers the reset type (T5).

    Pinning the module-scoped tuple guards the add-node dialog's combo values:
    the restyle made ``"reset"`` selectable, and a future edit that drops it (or
    re-orders the tuple) would break the "create a reset node" flow silently.
    Module-scoped (not a per-instance read) so no Tk is needed.
    """
    assert dialogs._NODE_TYPE_VALUES == ("line", "blind", "reset")


# ---------------------------------------------------------------------------
# TC10 -- the ShortcutsDialog builds headless and retains every label
# ---------------------------------------------------------------------------


# Sample shortcut groups passed to the dialog. Named so the TC10 assertion reads
# as deliberate fixture data; each key_label + description must surface in the
# dialog's retained labels.
_SAMPLE_SHORTCUT_GROUPS = (
    (
        "Editing",
        (
            ("Ctrl+Z", "Undo the last edit"),
            ("Ctrl+Y", "Redo the last undone edit"),
        ),
    ),
    (
        "Files",
        (
            ("Ctrl+S", "Save the field"),
            ("F5", "Save & build routes"),
        ),
    ),
)


def test_shortcuts_dialog_builds_and_retains_labels(tk_root) -> None:
    """TC10: ``ShortcutsDialog`` constructs under the shared root without raising,
    its retained labels contain every sample key_label + description, and it
    dismisses cleanly.

    Builds the REAL dialog on the session-scoped ``tk_root`` (never its own
    ``tk.Tk()`` -- see ``conftest.py``). The dialog retains its rendered key /
    description labels on ``_row_labels`` and its headers on ``_header_labels``
    so the test can introspect the rendered text without the window being
    mapped.
    """
    dialog = ShortcutsDialog(tk_root, _SAMPLE_SHORTCUT_GROUPS)
    try:
        # Constructing it is the first half of the assertion (it must not raise).
        assert isinstance(dialog, ShortcutsDialog)

        # Every header title surfaces (UPPERCASED by the dialog).
        header_texts = {label.cget("text") for label in dialog._header_labels}
        for category_title, _rows in _SAMPLE_SHORTCUT_GROUPS:
            assert category_title.upper() in header_texts

        # Every (key_label, description) pair surfaces in the retained rows.
        key_texts = {key.cget("text") for key, _desc in dialog._row_labels}
        desc_texts = {desc.cget("text") for _key, desc in dialog._row_labels}
        for _category_title, rows in _SAMPLE_SHORTCUT_GROUPS:
            for key_label, description in rows:
                assert key_label in key_texts, f"missing key label {key_label!r}"
                assert description in desc_texts, (
                    f"missing description {description!r}"
                )
    finally:
        # The dialog must dismiss cleanly (releases its grab, restores focus,
        # destroys the toplevel) without raising.
        dialog._on_close()

    # After dismissal the toplevel is gone (winfo_exists -> 0).
    assert not dialog.winfo_exists()


# ---------------------------------------------------------------------------
# TC12 -- JSON round-trip is stable and carries no projection edges
# ---------------------------------------------------------------------------


def test_json_round_trip_is_stable_and_has_no_projection_edges() -> None:
    """TC12: ``from_json_dict(to_json_dict())`` reproduces an equal model and the
    JSON dict carries NO projection edges (they live in ``field_data.py``'s
    ``EDGES`` table, not the editor JSON).

    Round-trips a model that exercises every optional slot (pivots + a valid
    reset node) so the stability check is meaningful, then asserts the JSON dict
    has no projection-edge key and no ``EDGE_PROJECTION`` marker anywhere in its
    values -- the editor's JSON sidecar never carries projection edges.
    """
    model = make_model()
    # Convert G to a valid reset node and add a pivot so the optional slots are
    # populated; G at (250, 80), wall x=280 -> 30 cm standoff, facing 180.
    model.node_types["G"] = "reset"
    model.reset_nodes = {
        "G": {"axis": "x", "wall_coord": 280.0, "facing": 180.0}
    }
    model.pivot_turns = [("A", "B", "G")]

    json_dict = model.to_json_dict()
    round_tripped = FieldModel.from_json_dict(json_dict)

    # The round-trip reproduces an equal model, and re-serialising is stable.
    assert round_tripped == model
    assert round_tripped.to_json_dict() == json_dict

    # No projection-edge surface in the JSON dict: not as a top-level key ...
    for key in json_dict:
        assert "PROJECTION" not in key.upper(), (
            f"JSON dict must not carry a projection key, found {key!r}"
        )
    assert "EDGES" not in json_dict, (
        "the editor JSON has no EDGES table -- projection edges live in "
        "field_data.py, not the JSON sidecar"
    )

    # ... and no EDGE_PROJECTION marker buried anywhere in the serialised values.
    assert "EDGE_PROJECTION" not in repr(json_dict)
