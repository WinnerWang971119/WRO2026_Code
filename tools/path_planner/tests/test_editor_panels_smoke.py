"""Restyle smoke test for the live field editor's side panels (Issue #78, T14).

The restyle wave (T11 themed panels, T13 app theme startup) repainted the
right-hand control column with the warm "drafting-table" palette, swapped in
themed ttk styles + plain-tk factories, added a mode-toolbar accent strip, and
enlarged the status surface.  None of that should have touched the panel's
*contract* -- the ``on_*`` callback slots the app wires after construction and
the public methods/properties the app drives.  TC11 is the automated guard for
that: it builds the REAL :class:`SidePanels` headless and asserts the exact set
of callbacks and methods survived the restyle, so a future rename fails the
suite instead of silently breaking the app wiring (AC9, AC10).

Headless pattern (mirrors ``test_editor_canvas_blit.py``): pin the ``Agg``
backend and no-op ``matplotlib.use`` for the duration of the editor import so
``editor.canvas``'s import-time ``use("TkAgg")`` is swallowed.  The single
SESSION-SCOPED ``tk_root`` (withdrawn, reused, destroyed once at teardown) lives
in ``conftest.py`` and backs every panel build -- one root per session, shared
with the blit tests, because repeatedly creating ``tk.Tk()`` within one process
is unstable; ``pytest.skip`` fires only when that single root genuinely cannot
be created (no display, even under xvfb).
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

# Pin the headless-safe Agg backend before importing the editor, then no-op the
# one ``matplotlib.use("TkAgg")`` switch that ``editor.canvas`` performs at
# import time (it would raise on headless CI and clobber the Agg backend the
# other tests rely on).  Restore the real ``use`` immediately afterwards.
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor import canvas as _canvas_mod  # noqa: F401, E402
finally:
    matplotlib.use = _real_mpl_use

import tkinter as tk  # noqa: E402

import pytest  # noqa: E402

from tools.path_planner.editor import loader  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.editor.panels import (  # noqa: E402
    _INSPECTOR_WIDTH_PX,
    _MODE_RAIL_WIDTH_PX,
    SidePanels,
)

_FIXTURE = Path(__file__).parent / "fixtures" / "field_data_small.json"

# Generous parent geometry for the three-pane layout test.  The window must be
# wider than (mode rail + inspector) so the centre canvas column gets a real
# positive width and the two fixed bands render at their declared pixel widths
# rather than being squeezed.  Tall enough that the title / status bands also
# take their natural height with the canvas row absorbing the slack.
_LAYOUT_PARENT_WIDTH_PX: int = 900
_LAYOUT_PARENT_HEIGHT_PX: int = 600

# Every public ``on_*`` callback slot the app (T12) assigns AFTER construction.
# All start at ``None``; SidePanels invokes whichever are set when the user
# touches a widget.  Enumerated explicitly (not introspected) so a rename in
# panels.py that drops or renames a slot fails this test loudly.
_EXPECTED_CALLBACKS: tuple[str, ...] = (
    "on_node_select",
    "on_node_type_change",
    "on_node_coords_change",
    # Inspector in-place rename + Delete button (property-panel edit).
    "on_node_rename",
    "on_node_delete",
    "on_auto_create_edges",
    "on_accept_proposed",
    "on_reject_proposed",
    "on_start_node_change",
    "on_start_heading_change",
    "on_undo",
    "on_redo",
    "on_save",
    "on_save_and_build",
    "on_reset_spec_change",
    "on_reset_spec_clear",
    "on_add_pivot",
    "on_delete_pivot",
    "on_global_knob_change",
    # The status-strip key-hint that opens the shortcuts dialog. Enumerated here
    # so a future rename of the slot fails this guard loudly (the T6 review note).
    # (The "+ add projection" button slot was removed in issue #64 -- projection
    # is now an on-model node TYPE set via the inspector combobox, not an edge
    # added through a panel button.)
    "on_show_shortcuts",
)

# Every public method / property the app drives.  ``max_edge_length_cm`` is a
# read property; the rest are setters / the refresh entry point.  Enumerated so
# a restyle that drops one is caught here rather than at runtime in the app.
_EXPECTED_PUBLIC_MEMBERS: tuple[str, ...] = (
    "refresh",
    "set_status",
    "set_mode_display",
    "set_selection_display",
    "set_validation_badge",
    "set_validation",
    "set_undo_enabled",
    "set_redo_enabled",
    "set_auto_create_state",
    # Added by the restyle wave (T6): the title-bar dirty dot and filename label
    # the app drives after every mutation / load. Enumerated so a dropped setter
    # surfaces here rather than as a runtime AttributeError (the T6 review note).
    "set_dirty",
    "set_filename",
    "max_edge_length_cm",
)

# Mode-display / selection-display strings passed to the status-surface setters
# in the second smoke check.  Named so the assertion reads as a deliberate themed
# update rather than a bare literal.
_SAMPLE_MODE_TEXT: str = "Mode: Move / select"
_SAMPLE_SELECTION_TEXT: str = "Selection: A"


def _load_model() -> FieldModel:
    """Load the small fixture into a fresh :class:`FieldModel`."""
    return FieldModel.from_json_dict(loader.load_field_json(_FIXTURE))


@pytest.fixture
def side_panels(tk_root):
    """A fresh :class:`SidePanels` built on the shared Tk root.

    Function-scoped so every test gets a clean widget tree / callback state,
    while the underlying Tk root is the single session-scoped ``tk_root`` from
    ``conftest.py`` (one root per session -- see that fixture's docstring for why
    it is shared across the GUI test modules rather than redefined here).  The
    build itself is the first half of the TC11 assertion: it must not raise
    after the T11 restyle.
    """
    return SidePanels(tk_root)


# ---------------------------------------------------------------------------
# TC11 -- the restyled SidePanels still exposes its full callback + method API.
# ---------------------------------------------------------------------------


def test_side_panels_builds_headless_after_restyle(side_panels):
    """Constructing :class:`SidePanels` must not raise after the T11 restyle.

    The ``side_panels`` fixture already performed the build; reaching this body
    means the themed widget tree (ttk styles, accent strip, enlarged status
    surface, mono numeric fields) constructed cleanly under the headless Agg /
    withdrawn-root context.  The explicit type check documents that intent.
    """
    assert isinstance(side_panels, SidePanels)


def test_side_panels_exposes_every_callback_slot(side_panels):
    """Every ``on_*`` callback attribute exists and defaults to ``None``.

    The app wires these AFTER construction, so each must be present and unset at
    build time.  Enumerating the exact set means a rename or a dropped slot in
    the restyled panels fails here instead of silently breaking app wiring.
    """
    for name in _EXPECTED_CALLBACKS:
        assert hasattr(side_panels, name), f"missing callback slot: {name}"
        assert getattr(side_panels, name) is None, (
            f"callback slot {name} must default to None at construction"
        )


def test_side_panels_exposes_every_public_member(side_panels):
    """Every public method / property survived the restyle.

    Each setter / the ``refresh`` entry point must be a callable, and
    ``max_edge_length_cm`` must be a readable property returning a float.  The
    app drives all of these, so a dropped member would surface as a runtime
    ``AttributeError`` -- catch it here instead.
    """
    for name in _EXPECTED_PUBLIC_MEMBERS:
        assert hasattr(side_panels, name), f"missing public member: {name}"

    # Everything except the read-only property must be callable.
    for name in _EXPECTED_PUBLIC_MEMBERS:
        if name == "max_edge_length_cm":
            continue
        assert callable(getattr(side_panels, name)), (
            f"public member {name} must be callable"
        )

    # The property must read back as a float (its parse-or-default contract).
    assert isinstance(side_panels.max_edge_length_cm, float)


# ---------------------------------------------------------------------------
# TC11 (cont.) -- the restyled widgets accept the themed runtime updates the
# app drives: refresh from a model, both validation-badge states, and the
# mode / selection display lines.  None may raise.
# ---------------------------------------------------------------------------


def test_side_panels_accepts_themed_runtime_updates(side_panels):
    """``refresh`` + the status-surface setters run cleanly on themed widgets.

    Drives the same surfaces the app touches after every model mutation:
    :meth:`refresh` rebuilds the listbox / inspector / pivots / knobs from a
    real model, :meth:`set_validation_badge` recolours the badge in BOTH the
    valid and invalid states, and the mode / selection lines update.  The test
    asserts none of these raise -- i.e. the restyled widgets still accept the
    themed updates -- which is the runtime half of the contract TC11 guards.
    """
    model = _load_model()

    # A full refresh against a real model: rebuilds every dynamic widget.
    side_panels.refresh(model)

    # Both badge states must recolour without raising on the dedicated label.
    side_panels.set_validation_badge(True)
    side_panels.set_validation_badge(False)

    # Per-move validation drives badge + message slot together (both states).
    side_panels.set_validation(False, "pivot angle 4 < min 15")
    side_panels.set_validation(True)

    # Mode / selection / status lines accept arbitrary themed text.
    side_panels.set_mode_display(_SAMPLE_MODE_TEXT)
    side_panels.set_selection_display(_SAMPLE_SELECTION_TEXT)
    side_panels.set_status("Ready.")

    # Re-refreshing after the updates must also stay clean (the suppress-callback
    # replay path runs over the themed widgets a second time).
    side_panels.refresh(model)


# ---------------------------------------------------------------------------
# TC9 -- the Save and Save & Build action buttons exist, are placed, and their
# commands invoke the app's ``on_save`` / ``on_save_and_build`` slots.
# ---------------------------------------------------------------------------


def test_side_panels_save_buttons_exist_and_invoke_callbacks(side_panels):
    """The Save and Save & Build buttons are real, grid-placed widgets whose
    commands fire the wired ``on_save`` / ``on_save_and_build`` callbacks.

    Verifies the action block survived the restyle end-to-end: both button
    widgets exist on the panel, were placed by the grid geometry manager (a
    withdrawn test root never reports ``winfo_ismapped``, so the geometry
    manager is the robust "it's in the layout" check), and clicking each --
    driven via :meth:`tk.Button.invoke`, which runs the bound command
    synchronously without the window being visible -- fires the slot the app
    assigns. A future rename that decouples the button from its slot fails here
    rather than silently at runtime.
    """
    # Both action buttons exist as real Tk buttons.
    assert isinstance(side_panels._save_button, tk.Button)
    assert isinstance(side_panels._save_build_button, tk.Button)

    # Each was placed via the grid geometry manager (the action block grids its
    # four buttons into a two-column frame).
    assert side_panels._save_button.winfo_manager() == "grid"
    assert side_panels._save_build_button.winfo_manager() == "grid"

    # Wire sentinel callbacks, then invoke each button's command. ``invoke``
    # runs the bound command synchronously -- no event loop / visible window
    # required -- so the count proves the button -> slot wiring is intact.
    save_calls = {"n": 0}
    build_calls = {"n": 0}
    side_panels.on_save = lambda: save_calls.__setitem__("n", save_calls["n"] + 1)
    side_panels.on_save_and_build = lambda: build_calls.__setitem__(
        "n", build_calls["n"] + 1
    )

    side_panels._save_button.invoke()
    side_panels._save_build_button.invoke()

    assert save_calls["n"] == 1, "Save button must fire on_save"
    assert build_calls["n"] == 1, "Save & Build button must fire on_save_and_build"


# ---------------------------------------------------------------------------
# F1 -- the true three-pane layout: ``layout_three_pane`` grids four chrome
# bands + the canvas into one shared parent, the mode rail / inspector hold
# their declared fixed pixel widths, and the title / status bands span all
# three columns.  The exact band-width regression the F1 review caught (the
# rail and inspector must render at 90 / 312 px, not collapse to content) is
# locked in here.
# ---------------------------------------------------------------------------


@pytest.fixture
def laid_out_panels(tk_root):
    """A :class:`SidePanels` whose bands + a stand-in canvas are gridded live.

    Builds a sized parent :class:`tk.Frame` under the shared session root, a
    :class:`SidePanels` parented to it (so its four bands are siblings of the
    parent grid), and a plain :class:`tk.Frame` standing in for the app's
    canvas widget.  Calls the single placement entry point
    :meth:`SidePanels.layout_three_pane` and forces a geometry pass with
    ``update_idletasks`` so every ``winfo_*`` query below reads a realised
    layout rather than the pre-map ``1`` placeholder.

    The parent frame is given an explicit width/height and ``pack_propagate``
    is disabled so it cannot shrink to its children -- the fixed bands then
    render at their declared widths instead of being squeezed by a zero-size
    parent.  Yields ``(panels, parent, canvas)`` and tears the parent down
    after the test (the shared root survives for the session).
    """
    parent = tk.Frame(
        tk_root,
        width=_LAYOUT_PARENT_WIDTH_PX,
        height=_LAYOUT_PARENT_HEIGHT_PX,
    )
    parent.pack_propagate(False)
    parent.pack()

    panels = SidePanels(parent)
    canvas = tk.Frame(parent)
    panels.layout_three_pane(canvas)

    # Force a geometry pass so winfo_width/winfo_manager report the realised
    # grid rather than the pre-map placeholders.
    parent.update_idletasks()
    parent.update()

    try:
        yield panels, parent, canvas
    finally:
        parent.destroy()


def test_layout_three_pane_grids_every_band_and_canvas(laid_out_panels):
    """All four chrome bands AND the canvas are placed by the GRID manager.

    ``layout_three_pane`` is the single placement entry point; every band plus
    the canvas must report ``winfo_manager() == "grid"`` (a withdrawn root never
    reports ``winfo_ismapped``, so the geometry manager is the robust
    "it's in the layout" check used elsewhere in this file).
    """
    panels, _parent, canvas = laid_out_panels

    for band_name, band in (
        ("title", panels._title_frame),
        ("mode rail", panels._mode_frame),
        ("inspector", panels._inspector_frame),
        ("status", panels._status_frame),
    ):
        assert band.winfo_manager() == "grid", (
            f"{band_name} band must be gridded by layout_three_pane"
        )
    assert canvas.winfo_manager() == "grid", "canvas must be gridded"


def test_layout_three_pane_band_widths_are_fixed(laid_out_panels):
    """The mode rail and inspector render at their declared fixed pixel widths.

    This is the exact regression the F1 review caught: the fixed bands must hold
    :data:`_MODE_RAIL_WIDTH_PX` (90) and :data:`_INSPECTOR_WIDTH_PX` (312), not
    collapse to their content width.  ``grid_propagate``/``pack_propagate`` are
    disabled on each band so a sized parent renders them at exactly these
    widths.
    """
    panels, _parent, _canvas = laid_out_panels

    assert panels._mode_frame.winfo_width() == _MODE_RAIL_WIDTH_PX
    assert panels._inspector_frame.winfo_width() == _INSPECTOR_WIDTH_PX
    # Document the literals the constants currently carry so a silent retune of
    # either band width also trips this guard.
    assert _MODE_RAIL_WIDTH_PX == 90
    assert _INSPECTOR_WIDTH_PX == 312


def test_layout_three_pane_title_and_status_span_three_columns(laid_out_panels):
    """The title and status bands span all three columns; the centre bands sit
    in their own single columns.

    ``grid_info`` reports each band's placement: the full-width title / status
    strips carry ``columnspan == 3`` (so they stretch across rail + canvas +
    inspector), while the mode rail and inspector occupy columns 0 and 2 of the
    middle row with the canvas between them in column 1.
    """
    panels, _parent, canvas = laid_out_panels

    title_info = panels._title_frame.grid_info()
    status_info = panels._status_frame.grid_info()
    assert int(title_info["columnspan"]) == 3, "title band must span 3 columns"
    assert int(status_info["columnspan"]) == 3, "status band must span 3 columns"
    assert int(title_info["row"]) == 0
    assert int(status_info["row"]) == 2

    # The middle row: rail (col 0) | canvas (col 1) | inspector (col 2).
    assert int(panels._mode_frame.grid_info()["column"]) == 0
    assert int(canvas.grid_info()["column"]) == 1
    assert int(panels._inspector_frame.grid_info()["column"]) == 2
    assert int(panels._mode_frame.grid_info()["row"]) == 1
    assert int(canvas.grid_info()["row"]) == 1
    assert int(panels._inspector_frame.grid_info()["row"]) == 1
