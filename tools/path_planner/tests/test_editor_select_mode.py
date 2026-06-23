"""Select-mode tests for the live field editor (T3 of the Select-mode plan).

T1 + T2 split the old combined Move/Select mode into two distinct editor modes:
a dedicated ``Mode.SELECT`` (left-click latches the node under the cursor; an
empty click clears the selection) that sits FIRST in the toolbar, and the
existing ``Mode.MOVE`` (left-click arms a drag). T2 also retired the keyboard /
toolbar entry point for ``Mode.START`` -- it is now reachable only via the
toolbar radio button, never a shortcut. This suite is the automated guard for
that split. It covers TC1-TC11 from the plan's Testing Strategy table:

* **TC1-TC5** -- the static wiring: the ``Mode.SELECT`` enum value, the toolbar
  label order (``Sel`` first), the ``s`` -> ``Mode.SELECT`` shortcut (and the
  absence of any ``Mode.START`` shortcut), the status-surface display labels,
  and the shortcuts-dialog ``Modes`` group.
* **TC6-TC9** -- the runtime handlers: the SELECT-click hit / miss branches,
  the zero-arg mode-switch handler, and the mode-display refresh.
* **TC10** -- the keyboard-binding layer: ``_wire_keyboard_shortcuts`` binds
  ``s`` / ``S`` to a SELECT switch and never builds a ``Mode.START`` handler.
* **TC11** -- the right-click guard: a button-3 canvas click opens the context
  menu and never falls through to the SELECT handler.

All tests build a bare ``EditorApp.__new__(EditorApp)`` and inject only the
collaborators each code path reads -- no real Tk window, ``FieldCanvas``, or
``SidePanels`` is constructed, so the suite needs no Tk root and no ``conftest``
``tk_root`` fixture. The matplotlib import dance mirrors the sibling editor
tests: pin Agg and swallow ``editor.canvas``'s import-time ``use("TkAgg")``.
"""

from __future__ import annotations

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

from tools.path_planner.editor import app as app_module  # noqa: E402
from tools.path_planner.editor import panels as panels_module  # noqa: E402
from tools.path_planner.editor.panels import Mode  # noqa: E402


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


class _BindingRecorder:
    """A fake Tk root that records every ``bind_all`` sequence string.

    ``EditorApp._wire_keyboard_shortcuts`` only calls ``self._root.bind_all``
    (plus the real ``self._shortcut_guard`` / handler-factory methods, which run
    on a bare instance), so recording the bound sequences is enough to assert the
    full keybinding set without building a real Tk window.
    """

    def __init__(self) -> None:
        self.sequences: list[str] = []

    def bind_all(self, sequence: str, _handler) -> None:
        self.sequences.append(sequence)


# ---------------------------------------------------------------------------
# TC1-TC5 -- static wiring (enum value, toolbar order, shortcut map, display
# labels, shortcuts dialog).
# ---------------------------------------------------------------------------


def test_select_mode_enum_value() -> None:
    """TC1: ``Mode.SELECT`` carries the ``"select"`` value and ``Mode("select")``
    round-trips back to the same member -- the value the panel ``mode_var`` and
    the ``_MODE_SHORTCUT_KEYS`` / display-label dicts key off."""
    assert Mode.SELECT.value == "select"
    assert Mode("select") is Mode.SELECT


def test_toolbar_label_order_puts_select_first() -> None:
    """TC2: the toolbar radio labels list ``Sel`` FIRST, then ``Move`` -- the new
    Select mode leads the mode toolbar ahead of Move (the order T1 set)."""
    assert panels_module._MODE_LABELS[0] == (Mode.SELECT, "Sel")
    assert panels_module._MODE_LABELS[1] == (Mode.MOVE, "Move")


def test_select_shortcut_key_maps_to_select_and_start_unbound() -> None:
    """TC3: the ``s`` key maps to ``Mode.SELECT`` and no shortcut binds
    ``Mode.START`` -- T2 dropped the Start keyboard entry point entirely (Start
    is reachable only from the toolbar)."""
    assert app_module._MODE_SHORTCUT_KEYS["s"] is Mode.SELECT
    assert Mode.START not in app_module._MODE_SHORTCUT_KEYS.values()


def test_mode_display_labels_for_select_and_move() -> None:
    """TC4: the status-surface display labels read ``"Select"`` and ``"Move"`` for
    the two split modes -- the human-readable names the mode line surfaces."""
    assert app_module._MODE_DISPLAY_LABELS["select"] == "Select"
    assert app_module._MODE_DISPLAY_LABELS["move"] == "Move"


def test_shortcuts_dialog_modes_group_lists_select_and_move() -> None:
    """TC5: the shortcuts-dialog ``Modes`` group lists ``(S, Select)`` and
    ``(M, Move)`` and carries NEITHER the retired ``(S, Set start)`` nor the old
    combined ``(M, Move / select)`` row -- the help text matches the T2 split."""
    modes_group = next(
        rows for title, rows in app_module._SHORTCUT_GROUPS if title == "Modes"
    )
    assert ("S", "Select") in modes_group
    assert ("M", "Move") in modes_group
    assert ("S", "Set start") not in modes_group
    assert ("M", "Move / select") not in modes_group


# ---------------------------------------------------------------------------
# TC6-TC7 -- the SELECT-click handler (hit latches the node; miss clears).
# ---------------------------------------------------------------------------


def test_select_click_hit_latches_node_and_status() -> None:
    """TC6: a SELECT click ON a node latches it as the selection, mirrors it into
    ``_selected_node_var``, and surfaces the ``"Selected node 'N1'."`` status.

    ``_select_node`` and ``_update_selection_display`` run for real (they only
    touch the injected fakes); ``_refresh_view`` and ``_set_status`` are stubbed,
    the latter capturing the surfaced status text.
    """
    app = EditorApp.__new__(EditorApp)
    app._canvas = SimpleNamespace(find_node_at=lambda x, y: "N1")
    app._panels = SimpleNamespace(
        _selected_node_var=_FakeVar(),
        set_selection_display=lambda *_a, **_k: None,
    )
    app._selected_node = None
    app._refresh_view = lambda: None
    statuses: list[str] = []
    app._set_status = statuses.append

    app._handle_select_click(1.0, 2.0)

    assert app._selected_node == "N1"
    assert app._panels._selected_node_var.get() == "N1"
    assert statuses == ["Selected node 'N1'."]


def test_select_click_miss_clears_selection() -> None:
    """TC7: a SELECT click on EMPTY canvas clears the current selection -- it blanks
    ``_selected_node_var`` and routes through the real ``_on_panel_node_select``
    deselect handler, leaving ``_selected_node`` ``None`` and the var empty.

    ``_on_panel_node_select`` runs for real; its ``_set_status`` /
    ``_update_selection_display`` / ``_refresh_view`` side-effects are stubbed so
    the test isolates the selection-clearing behaviour.
    """
    var = _FakeVar("N1")
    app = EditorApp.__new__(EditorApp)
    app._canvas = SimpleNamespace(find_node_at=lambda x, y: None)
    app._panels = SimpleNamespace(
        _selected_node_var=var,
        set_selection_display=lambda *_a, **_k: None,
    )
    app._selected_node = "N1"
    app._refresh_view = lambda: None
    app._set_status = lambda *_a: None
    app._update_selection_display = lambda *_a: None

    app._handle_select_click(1.0, 2.0)

    assert app._selected_node is None
    assert app._panels._selected_node_var.get() == ""


# ---------------------------------------------------------------------------
# TC8-TC9 -- the mode-switch handler factory and the mode-display refresh.
# ---------------------------------------------------------------------------


def test_mode_switch_handler_sets_mode_var() -> None:
    """TC8: the zero-arg handler from ``_make_mode_switch_handler(Mode.SELECT)``
    writes ``Mode.SELECT.value`` into ``mode_var`` when called with no args -- the
    same path ``_wire_keyboard_shortcuts`` drives for the ``s`` key."""
    app = EditorApp.__new__(EditorApp)
    app._panels = SimpleNamespace(mode_var=_FakeVar(Mode.MOVE.value))

    handler = app._make_mode_switch_handler(Mode.SELECT)
    handler()

    assert app._panels.mode_var.get() == "select"


def test_update_mode_display_renders_select_then_move() -> None:
    """TC9: ``_update_mode_display`` pushes ``"Mode: Select"`` while ``mode_var``
    holds ``"select"`` and ``"Mode: Move"`` after it flips to ``"move"`` -- the
    mode line tracks the active mode through the display-prefix + label."""
    shown: list[str] = []
    app = EditorApp.__new__(EditorApp)
    app._panels = SimpleNamespace(
        mode_var=_FakeVar("select"),
        set_mode_display=shown.append,
    )

    app._update_mode_display()
    assert shown[-1] == app_module._MODE_DISPLAY_PREFIX + "Select"

    app._panels.mode_var.set("move")
    app._update_mode_display()
    assert shown[-1] == "Mode: Move"


# ---------------------------------------------------------------------------
# TC10 -- the keyboard-binding layer binds s / S to a SELECT switch and never
# builds a START handler.
# ---------------------------------------------------------------------------


def test_wire_keyboard_shortcuts_binds_select_not_start() -> None:
    """TC10: ``_wire_keyboard_shortcuts`` binds ``<KeyPress-s>`` / ``<KeyPress-S>``
    and builds a ``Mode.SELECT`` switch handler, but NEVER builds a ``Mode.START``
    one -- the T2 split bound Select to ``s`` and dropped the Start shortcut.

    The real registration runs against a recording fake root; a construction spy
    wraps ``_make_mode_switch_handler`` to record which modes get a handler built,
    proving the rebind without invoking the focus-guarded handlers (they need the
    real Tk root). A known mode key (``<KeyPress-m>``) is asserted present so the
    registration is proven to have actually run.
    """
    app = EditorApp.__new__(EditorApp)
    recorder = _BindingRecorder()
    app._root = recorder
    built: list[Mode] = []
    app._make_mode_switch_handler = lambda mode: built.append(mode) or (
        lambda: None
    )

    app._wire_keyboard_shortcuts()

    assert "<KeyPress-s>" in recorder.sequences
    assert "<KeyPress-S>" in recorder.sequences
    assert Mode.SELECT in built
    assert Mode.START not in built
    # Sanity: the registration actually ran (a known mode key is present), so the
    # assertions above are meaningful and not checking against an empty set.
    assert "<KeyPress-m>" in recorder.sequences


# ---------------------------------------------------------------------------
# TC11 -- a right-click opens the context menu and never reaches SELECT.
# ---------------------------------------------------------------------------


def test_right_click_opens_context_menu_not_select() -> None:
    """TC11: a button-3 (right-click) canvas click routes to ``_show_context_menu``
    and short-circuits before the SELECT handler -- the right-click context-menu
    guard wins over the per-mode left-click dispatch."""
    app = EditorApp.__new__(EditorApp)
    app._panels = SimpleNamespace(mode_var=_FakeVar("select"))
    ctx: list[tuple[float, float]] = []
    app._show_context_menu = lambda x, y: ctx.append((x, y))
    sel: list[tuple[float, float]] = []
    app._handle_select_click = lambda x, y: sel.append((x, y))

    app._on_canvas_click(3.0, 4.0, button=3)

    assert ctx == [(3.0, 4.0)]
    assert sel == []
