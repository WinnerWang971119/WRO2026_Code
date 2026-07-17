"""Real-Tk smoke tests for the multi-select marquee + ALIGN panel (issue #161, T8).

Covers TC11 and TC33-TC36 from the plan's Testing Strategy:

* TC11 -- :meth:`FieldCanvas.find_nodes_in_rect` (the box hit-test the
  marquee release resolves through) returns exactly the nodes whose
  coordinate falls inside a known box, in the model's dict iteration
  order; returns ``[]`` before :meth:`~FieldCanvas.render` has ever run;
  and normalizes an "inverted" box (``x1 < x0`` and/or ``y1 < y0``) to the
  same result as the equivalent normalized box, inclusive on all four
  edges.
* TC33 -- the node-selection marquee blit fast-path (:meth:`FieldCanvas.
  begin_marquee_drag` / :meth:`~FieldCanvas.update_marquee_preview` /
  :meth:`~FieldCanvas.end_marquee_drag`) is a parallel clone of the obstacle
  rubber-band: arming it adds exactly one animated preview rectangle, updating
  it resizes the rectangle and composites via the blit fast-path (not a full
  redraw), and tearing it down removes the rectangle and leaves the axes'
  patch count exactly where it was before arming.
* TC34 -- the :class:`SidePanels` **ALIGN** section builds disabled at zero
  selected nodes, :meth:`~SidePanels.set_align_state` populates the readout
  and enables the controls, and the ``Make straight`` button / axis radios
  are wired to :attr:`~SidePanels.on_align_request` /
  :attr:`~SidePanels.on_axis_change`.
* TC35 -- the ``EXTENDED`` node listbox forwards its full selection (in
  listbox/sorted order) to :attr:`~SidePanels.on_nodes_select`, and
  :meth:`~SidePanels.refresh` restores that same selection afterwards
  without re-firing the callback (the ``_suppress_callbacks`` guard).
* TC36 -- :meth:`FieldCanvas.render`'s ``align_preview`` keyword draws exactly
  one teal guide-line artist plus one teal ghost-dot collection, and clears
  both when the preview reverts to ``None`` (AC13).

Headless pattern (mirrors ``test_editor_canvas_blit.py`` /
``test_editor_panels_smoke.py``): pin the ``Agg`` backend and no-op
``matplotlib.use`` for the duration of the editor import so
``editor.canvas``'s import-time ``use("TkAgg")`` is swallowed. The single
SESSION-SCOPED ``tk_root`` fixture lives in ``conftest.py`` and is shared with
every other GUI test module in this suite -- a second ``tk.Tk()`` root
defined here would recreate the exact cross-file flakiness that fixture was
consolidated to remove (intermittent ``TclError`` and silent green-by-skip
under a full-suite / ``xvfb-run`` invocation), so every fixture below takes
``tk_root`` as a parameter rather than constructing its own root.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

# Pin the headless-safe Agg backend before importing the editor, then no-op
# the one ``matplotlib.use("TkAgg")`` switch that ``editor.canvas`` performs
# at import time (it would raise on headless CI and clobber the Agg backend
# the other tests rely on). Restore the real ``use`` immediately afterwards.
matplotlib.use("Agg")
_real_mpl_use = matplotlib.use
matplotlib.use = lambda *args, **kwargs: None
try:
    from tools.path_planner.editor import canvas as _canvas_mod  # noqa: F401, E402
finally:
    matplotlib.use = _real_mpl_use

import tkinter as tk  # noqa: E402

import pytest  # noqa: E402
from matplotlib.colors import to_rgba  # noqa: E402

from tools.path_planner.editor import loader  # noqa: E402
from tools.path_planner.editor import theme  # noqa: E402
from tools.path_planner.editor.align import AlignPreview  # noqa: E402
from tools.path_planner.editor.canvas import FieldCanvas  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.editor.panels import SidePanels  # noqa: E402

_FIXTURE = Path(__file__).parent / "fixtures" / "field_data_small.json"

# The fixture's four nodes (S, A, B, G) sorted the same way the node listbox
# sorts them -- used to compute expected listbox indices/order in TC35.
_SORTED_FIXTURE_NODE_NAMES: tuple[str, ...] = ("A", "B", "G", "S")


def _load_model() -> FieldModel:
    """Load the small fixture into a fresh :class:`FieldModel`."""
    return FieldModel.from_json_dict(loader.load_field_json(_FIXTURE))


@pytest.fixture
def field_canvas(tk_root):
    """A fresh :class:`FieldCanvas` (Agg backend) built on the shared Tk root.

    Function-scoped so every test gets a clean canvas / artist state, while
    the underlying Tk root is the single session-scoped ``tk_root`` from
    ``conftest.py`` -- mirrors ``test_editor_canvas_blit.py:78-87``.
    """
    return FieldCanvas(tk_root)


@pytest.fixture
def side_panels(tk_root):
    """A fresh :class:`SidePanels` built on the shared Tk root.

    Function-scoped so every test gets a clean widget tree / callback state,
    while the underlying Tk root is the single session-scoped ``tk_root`` from
    ``conftest.py`` -- mirrors ``test_editor_panels_smoke.py:137-148``.
    """
    return SidePanels(tk_root)


class _CompositeSpy:
    """Counting wrapper for the canvas' ``restore_region`` / ``blit`` methods.

    Mirrors ``test_editor_canvas_blit.py``'s ``_CompositeSpy``: replacing the
    embedded ``FigureCanvasTkAgg``'s compositing methods with this spy lets a
    test count composites without asserting pixel equality (font / AA /
    matplotlib-version flaky). Each spy delegates to the real method so the
    underlying backend state stays consistent.
    """

    def __init__(self, real_method):
        self._real_method = real_method
        self.calls = 0

    def __call__(self, *args, **kwargs):
        self.calls += 1
        return self._real_method(*args, **kwargs)


def _install_composite_spies(canvas: FieldCanvas):
    """Replace ``restore_region`` and ``blit`` with counting spies.

    :return: ``(restore_spy, blit_spy)`` -- inspect ``.calls`` on each.
    """
    restore_spy = _CompositeSpy(canvas._canvas.restore_region)
    blit_spy = _CompositeSpy(canvas._canvas.blit)
    canvas._canvas.restore_region = restore_spy
    canvas._canvas.blit = blit_spy
    return restore_spy, blit_spy


def _make_nodes_model(nodes: dict[str, tuple[float, float]]) -> FieldModel:
    """Build a minimal but fully-valid :class:`FieldModel` over *nodes*.

    Mirrors ``_make_field_model`` in ``test_editor_align.py``: every node is
    a plain ``"line"`` type, no edges/obstacles/pivots -- enough for
    :meth:`FieldCanvas.render` without needing a fixture file, so the TC11
    tests below can pick exact, known node coordinates.

    :param nodes: Node name -> ``(x_cm, y_cm)`` to seed ``model.nodes`` with.
    :return: A fully-constructed :class:`FieldModel` on a 300cm x 200cm field.
    """
    return FieldModel(
        field_mm=(3000.0, 2000.0),
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


# ---------------------------------------------------------------------------
# TC11 -- FieldCanvas.find_nodes_in_rect: the marquee's box hit-test.
# ---------------------------------------------------------------------------


def test_find_nodes_in_rect_returns_matches_in_dict_order_and_empty_before_render(
    field_canvas,
):
    """TC11a: before ``render`` has ever run, ``find_nodes_in_rect`` returns
    ``[]`` rather than raising (mirrors ``find_node_at``'s pre-render guard).

    After ``render(model)``, a box covering exactly 3 of 6 known nodes
    returns exactly those 3 names, in the model's dict iteration order (not
    sorted, not click order -- the order ``model.nodes.items()`` yields).
    """
    # No render() has happened yet on this fresh canvas -- must be a no-op.
    assert field_canvas.find_nodes_in_rect(0.0, 0.0, 20.0, 20.0) == []

    nodes = {
        "A": (0.0, 0.0),
        "B": (10.0, 10.0),
        "C": (5.0, 15.0),
        "D": (100.0, 100.0),
        "E": (110.0, 90.0),
        "F": (150.0, 120.0),
    }
    field_canvas.render(_make_nodes_model(nodes))

    # The box [0, 20] x [0, 20] covers A, B, C; D/E/F sit far outside it.
    result = field_canvas.find_nodes_in_rect(0.0, 0.0, 20.0, 20.0)

    assert result == ["A", "B", "C"]


def test_find_nodes_in_rect_normalizes_inverted_box_with_inclusive_edges(
    field_canvas,
):
    """TC12: an "inverted" box (``x1 < x0`` and/or ``y1 < y0`` -- what a
    marquee dragged up-and-left naturally produces) returns the exact same
    set as the equivalent normalized box, in every combination of inverted
    axis (both, x-only, y-only).

    The fixture also pins inclusive-edge semantics: node ``A`` sits exactly
    on the box's bottom-left corner, ``B`` on its top-right corner, and ``C``
    on its bottom edge only (interior x, boundary y) -- all three must be
    included, not just the strictly-interior ``D``.
    """
    nodes = {
        "A": (0.0, 0.0),  # on both the x_min and y_min edges
        "B": (20.0, 20.0),  # on both the x_max and y_max edges
        "C": (15.0, 0.0),  # on the y_min edge only; x is strictly interior
        "D": (10.0, 10.0),  # strictly interior
        "E": (250.0, 150.0),  # well outside the box
        "F": (280.0, 190.0),  # well outside the box
    }
    field_canvas.render(_make_nodes_model(nodes))

    expected = ["A", "B", "C", "D"]

    normalized = field_canvas.find_nodes_in_rect(0.0, 0.0, 20.0, 20.0)
    both_inverted = field_canvas.find_nodes_in_rect(20.0, 20.0, 0.0, 0.0)
    x_inverted = field_canvas.find_nodes_in_rect(20.0, 0.0, 0.0, 20.0)
    y_inverted = field_canvas.find_nodes_in_rect(0.0, 20.0, 20.0, 0.0)

    assert normalized == expected
    assert both_inverted == expected
    assert x_inverted == expected
    assert y_inverted == expected


# ---------------------------------------------------------------------------
# TC33 -- the node-selection marquee blit fast-path: begin_/update_/end_.
# ---------------------------------------------------------------------------


def test_marquee_blit_lifecycle_adds_updates_and_removes_one_artist(
    field_canvas,
):
    """``begin_``/``update_``/``end_marquee_drag`` is the obstacle rubber-band
    beat-for-beat, with its own preview artist and its own colour.

    ``begin_marquee_drag`` must add exactly one *animated* preview rectangle
    to the axes (the patch count grows by exactly one). ``update_marquee_
    preview`` must reposition that rectangle's bounds AND composite via the
    restore/blit fast-path -- not a full ``ax.clear`` + re-render -- mirroring
    how ``test_editor_canvas_blit.py`` inspects the obstacle preview artist's
    ``get_x``/``get_y``/``get_width``/``get_height``. ``end_marquee_drag``
    must remove the rectangle and leave the axes' patch count exactly where
    it was before ``begin_marquee_drag`` ran (no leaked artist).
    """
    model = _load_model()
    field_canvas.render(model)

    patches_before = len(field_canvas.axes.patches)

    field_canvas.begin_marquee_drag()
    rect = field_canvas._marquee_preview_artist
    assert rect is not None
    assert rect.get_animated() is True
    # Exactly one new patch -- the marquee preview rectangle -- was added.
    assert len(field_canvas.axes.patches) == patches_before + 1

    # Spy AFTER arming (begin uses draw/copy_from_bbox, not restore/blit).
    restore_spy, blit_spy = _install_composite_spies(field_canvas)
    field_canvas.update_marquee_preview(10.0, 20.0, 40.0, 60.0)

    # The blit compositing pair fired (NOT a full ax.clear + render).
    assert restore_spy.calls >= 1
    assert blit_spy.calls >= 1

    # The rectangle bounds span the dragged extent (lower-left + width/height).
    # ``Rectangle`` exposes no ``get_bounds`` (only the ``set_bounds`` mutator
    # used by the canvas itself); ``get_x``/``get_y``/``get_width``/
    # ``get_height`` are the readback surface, matching how
    # ``test_editor_canvas_blit.py`` inspects the obstacle preview rectangle.
    assert rect.get_x() == pytest.approx(10.0)
    assert rect.get_y() == pytest.approx(20.0)
    assert rect.get_width() == pytest.approx(30.0)
    assert rect.get_height() == pytest.approx(40.0)

    field_canvas.end_marquee_drag()
    assert field_canvas._marquee_preview_artist is None
    assert field_canvas._marquee_blit_background is None
    # The preview rectangle is gone; every other artist was rebuilt fresh by
    # the teardown's full render, so the patch count returns to its baseline.
    assert len(field_canvas.axes.patches) == patches_before


def test_update_marquee_preview_without_begin_falls_back_to_full_render(
    field_canvas,
):
    """Calling ``update_marquee_preview`` while disarmed must not crash.

    Mirrors the obstacle/node-drag defensive fallback: with no
    ``begin_marquee_drag`` having run, ``_marquee_blit_background`` is
    ``None``, so the update degrades to a plain repaint of the last model
    rather than raising (there is nothing to blit onto).
    """
    field_canvas.render(_load_model())

    assert field_canvas._marquee_blit_background is None
    field_canvas.update_marquee_preview(0.0, 0.0, 10.0, 10.0)
    assert field_canvas._marquee_blit_background is None


# ---------------------------------------------------------------------------
# TC34 -- SidePanels ALIGN section: construction, populate/enable, wiring.
# ---------------------------------------------------------------------------


def test_align_panel_starts_disabled_at_zero_selected(side_panels):
    """The ALIGN section builds with every control disabled and blank.

    Before the app ever calls :meth:`SidePanels.set_align_state`, the section
    reflects "zero nodes selected": the axis radios and the ``Make straight``
    button are disabled, and the residual/target readout lines are blank.
    """
    for radio in side_panels._align_axis_radios:
        assert radio.cget("state") == tk.DISABLED
    assert side_panels._align_make_straight_button.cget("state") == tk.DISABLED
    assert side_panels._align_residual_var.get() == ""
    assert side_panels._align_target_var.get() == ""


def test_set_align_state_populates_readout_and_enables_controls(side_panels):
    """``set_align_state`` writes the count/axis/residual/target readout and
    flips every ALIGN control to enabled.

    Uses the exact example from the plan's UI/UX Specification: 6 selected
    nodes, ``AUTO`` resolved to ``X``, a ``0.0133`` cm residual, target
    ``x = 50.4867``.
    """
    side_panels.set_align_state(6, "Auto — X", 0.0133, "x = 50.4867", True)

    assert side_panels._align_count_var.get() == "6 nodes selected"
    assert side_panels._align_auto_label_var.get() == "Auto — X"
    assert side_panels._align_residual_var.get() == "max off-axis 0.0133cm"
    assert side_panels._align_target_var.get() == "x = 50.4867"
    for radio in side_panels._align_axis_radios:
        assert radio.cget("state") == tk.NORMAL
    assert side_panels._align_make_straight_button.cget("state") == tk.NORMAL


def test_set_align_state_blanks_readout_when_residual_is_none(side_panels):
    """``residual_cm=None`` blanks both readout lines (the <2-selected state).

    :meth:`SidePanels.set_align_state` is also the vehicle for the disabled
    state (the app calls it with ``enabled=False`` when the selection drops
    below 2); this pins that ``residual_cm=None`` clears any stale readout
    text left over from a prior selection.
    """
    side_panels.set_align_state(6, "Auto — X", 0.0133, "x = 50.4867", True)
    side_panels.set_align_state(0, "Auto", None, "", False)

    assert side_panels._align_count_var.get() == "0 nodes selected"
    assert side_panels._align_residual_var.get() == ""
    assert side_panels._align_target_var.get() == ""
    for radio in side_panels._align_axis_radios:
        assert radio.cget("state") == tk.DISABLED
    assert side_panels._align_make_straight_button.cget("state") == tk.DISABLED


def test_make_straight_button_and_axis_radio_wiring(side_panels):
    """The ``Make straight`` button fires ``on_align_request``; flipping the
    axis radio fires ``on_axis_change`` -- both with the current/new
    ``AlignAxis`` value string, driven directly via widget commands.

    Drives the widgets exactly as the sibling smoke test drives the Save
    buttons (``button.invoke()``): invoking a ``tk.Radiobutton`` both sets its
    bound variable to its own ``value`` and runs its ``command``, so a single
    ``invoke()`` call exercises the "set the radio var then call its command"
    path without simulating a raw mouse click.
    """
    side_panels.set_align_state(6, "Auto — X", 0.0133, "x = 50.4867", True)

    align_calls: list[str] = []
    axis_calls: list[str] = []
    side_panels.on_align_request = align_calls.append
    side_panels.on_axis_change = axis_calls.append

    # The axis var defaults to "auto"; clicking Make straight fires with it.
    assert side_panels._align_axis_var.get() == "auto"
    side_panels._align_make_straight_button.invoke()
    assert align_calls == ["auto"]

    # Flip to the "X" radio (index 1 -- [auto, x, y, fit], per
    # _ALIGN_STATIC_AXIS_CHOICES); fires on_axis_change with the new value.
    side_panels._align_axis_radios[1].invoke()
    assert axis_calls == ["x"]
    assert side_panels._align_axis_var.get() == "x"

    # A subsequent Make straight click now reflects the newly chosen axis.
    side_panels._align_make_straight_button.invoke()
    assert align_calls == ["auto", "x"]


# ---------------------------------------------------------------------------
# TC35 -- the EXTENDED node listbox: selection round-trip through refresh.
# ---------------------------------------------------------------------------


def test_extended_listbox_select_fires_on_nodes_select_in_listbox_order(
    side_panels,
):
    """Selecting 3 rows fires ``on_nodes_select`` with those 3 names, in
    listbox (sorted) order -- not click/insertion order.

    The listbox is populated (by :meth:`SidePanels.refresh`) from
    ``sorted(model.nodes.keys())`` == ``("A", "B", "G", "S")`` for the small
    fixture. Selecting "S", "A", "G" out of order must still report them back
    as ``["A", "G", "S"]`` because ``Listbox.curselection()`` always returns
    ascending index order regardless of selection order.
    """
    model = _load_model()
    side_panels.refresh(model)

    selected: list[list[str]] = []
    side_panels.on_nodes_select = selected.append

    chosen_out_of_order = ("S", "A", "G")
    for name in chosen_out_of_order:
        side_panels._node_listbox.selection_set(
            _SORTED_FIXTURE_NODE_NAMES.index(name)
        )

    # Drive the bound handler directly (real Tk, not a synthetic event) --
    # mirrors driving button/radio commands directly elsewhere in this file.
    side_panels._handle_listbox_select(None)

    assert selected == [["A", "G", "S"]]


def test_refresh_restores_full_selection_and_fires_no_callback(side_panels):
    """``refresh`` restores the full EXTENDED selection but never re-fires
    ``on_nodes_select`` -- the ``_suppress_callbacks`` guard.

    Selects 3 of the 4 fixture nodes, confirms the callback fired once with
    that selection, then calls :meth:`SidePanels.refresh` again (as the app
    does after every model mutation / undo / redo) and asserts BOTH that the
    same 3 nodes remain highlighted in the rebuilt listbox AND that
    ``on_nodes_select`` was not invoked a second time.
    """
    model = _load_model()
    side_panels.refresh(model)

    selected: list[list[str]] = []
    side_panels.on_nodes_select = selected.append

    chosen = ("A", "G", "S")
    for name in chosen:
        side_panels._node_listbox.selection_set(
            _SORTED_FIXTURE_NODE_NAMES.index(name)
        )
    side_panels._handle_listbox_select(None)
    assert selected == [["A", "G", "S"]]

    # A programmatic replay (model unchanged is enough to exercise the path).
    side_panels.refresh(model)

    restored_indices = side_panels._node_listbox.curselection()
    restored_names = [
        side_panels._node_listbox.get(index) for index in restored_indices
    ]
    assert restored_names == list(chosen)

    # refresh() must not have fired on_nodes_select again.
    assert selected == [["A", "G", "S"]]


# ---------------------------------------------------------------------------
# TC36 -- render(align_preview=...) draws then clears the guide + ghosts.
# ---------------------------------------------------------------------------


def test_render_align_preview_adds_one_guide_line_and_one_ghost_collection(
    field_canvas,
):
    """``render(model, align_preview=preview)`` adds exactly one teal
    guide-line artist and one teal ghost-dot collection; reverting to
    ``align_preview=None`` removes both (AC13).

    Neither committed edges (``ax.plot`` Line2D, but drawn in the edge
    colour, not teal) nor node dots (also individual ``Line2D``s, not a
    collection) contribute a false positive here, so a plain before/after
    count of ``axes.lines`` / ``axes.collections`` isolates the preview
    overlay precisely. The guide line's colour and the ghost collection's
    edge colour are both checked against ``theme.TEAL`` (== ``PROPOSED_EDGE``)
    to confirm they are the make-straight preview artists, not some other
    overlay.
    """
    model = _load_model()

    field_canvas.render(model)
    lines_before = len(field_canvas.axes.lines)
    collections_before = len(field_canvas.axes.collections)

    preview = AlignPreview(
        guide=((10.0, 10.0), (200.0, 10.0)),
        ghosts={"A": (10.0, 20.0)},
    )
    field_canvas.render(model, align_preview=preview)

    lines_after = field_canvas.axes.lines
    assert len(lines_after) == lines_before + 1
    guide_line = lines_after[-1]
    assert guide_line.get_color() == theme.TEAL

    collections_after = field_canvas.axes.collections
    assert len(collections_after) == collections_before + 1
    ghost_collection = collections_after[-1]
    expected_edgecolor = to_rgba(theme.TEAL)
    actual_edgecolor = tuple(ghost_collection.get_edgecolor()[0])
    assert actual_edgecolor == pytest.approx(expected_edgecolor)

    # Reverting to no preview leaves neither artist behind.
    field_canvas.render(model, align_preview=None)
    assert len(field_canvas.axes.lines) == lines_before
    assert len(field_canvas.axes.collections) == collections_before


def test_render_align_preview_with_no_ghosts_draws_only_the_guide_line(
    field_canvas,
):
    """An already-collinear selection (empty ``ghosts``) draws the guide line
    but no ghost-dot collection -- mirrors the no-op readout state where every
    node already sits on its target.
    """
    model = _load_model()

    field_canvas.render(model)
    lines_before = len(field_canvas.axes.lines)
    collections_before = len(field_canvas.axes.collections)

    preview = AlignPreview(
        guide=((10.0, 10.0), (200.0, 10.0)),
        ghosts={},
    )
    field_canvas.render(model, align_preview=preview)

    assert len(field_canvas.axes.lines) == lines_before + 1
    assert len(field_canvas.axes.collections) == collections_before
