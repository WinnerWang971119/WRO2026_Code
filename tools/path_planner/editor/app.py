"""Application orchestrator for the WRO 2026 live field editor (Issue #10, T12).

:class:`EditorApp` is the seam that glues together the four pieces built in
previous tasks of the live-editor plan:

* :class:`~tools.path_planner.editor.model.FieldModel`         (T2)
* :class:`~tools.path_planner.editor.history.History`          (T5)
* :mod:`tools.path_planner.editor.commands`                    (T5)
* :class:`~tools.path_planner.editor.canvas.FieldCanvas`       (T10)
* :class:`~tools.path_planner.editor.panels.SidePanels`        (T11)

The app owns:

* A single :class:`FieldModel` instance loaded from ``field_data.json``.
* A single :class:`History` instance — every mutation funnelled through
  ``History.do`` for unbounded undo/redo.
* A reference to the imported ``tools.path_planner.field_data`` module,
  used solely to peek at the rich ``EDGES`` table when a node delete is
  attempted (the model doesn't own ``EDGES`` — see T5's note on
  :class:`~tools.path_planner.editor.commands.DeleteNode`).
* A set of pending node renames (``_pending_renames``) accumulated since
  the last successful Save; flushed to ``field_data.py`` via
  :func:`tools.path_planner.editor.ast_rewrite.rewrite_edges_uv` during
  the Save handler.

Mouse handling is mode-driven: the canvas exposes raw ``(x, y, button)``
events and this class routes them through the panel's currently selected
:class:`~tools.path_planner.editor.panels.Mode`. The mapping is documented
inline above each handler.

No business logic lives in :func:`tools.path_planner.edit_field.main`; the
CLI is the thin shell, this class is the entire application.
"""

from __future__ import annotations

import io
import os
import tkinter as tk
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from tkinter import messagebox, simpledialog

from tools.path_planner.editor.dialogs import AddNodeDialog, ShortcutsDialog
from typing import Optional

from tools.path_planner.editor import auto_edges, loader, projection_edges
from tools.path_planner.editor.canvas import FieldCanvas
from tools.path_planner.editor.commands import (
    AddBlindEdge,
    AddEdgesBatch,
    AddLineEdge,
    AddNode,
    AddObstacle,
    AddPivotTurn,
    ClearResetSpec,
    DeleteEdge,
    DeleteNode,
    DeleteObstacle,
    DeletePivotTurn,
    MacroCommand,
    MoveNode,
    RenameNode,
    SetGlobalKnob,
    SetNodeType,
    SetResetSpec,
    SetStart,
    SetStartHeading,
)
from tools.path_planner.editor.history import History
from tools.path_planner.editor.model import (
    NODE_TYPE_VALUES,
    RESET_AXIS_VALUES,
    FieldModel,
)
from tools.path_planner.editor.panels import Mode, SidePanels
from tools.path_planner.editor import theme


# ---------------------------------------------------------------------------
# Constants -- no magic numbers in the code body.
# ---------------------------------------------------------------------------

#: Default window title (without dirty marker).
_WINDOW_TITLE: str = "WRO 2026 Field Editor"

#: Suffix appended to the window title when the model has unsaved changes.
_DIRTY_TITLE_SUFFIX: str = " *"

#: Initial canvas area width in pixels. The canvas height is derived from the
#: loaded field's aspect ratio so the map renders without huge empty bars
#: above and below it. The two fixed chrome columns (the mode rail +
#: inspector, ``_CHROME_TOTAL_WIDTH_PX``) and a small chrome margin are added
#: on top of these to get the window geometry.
_DEFAULT_CANVAS_WIDTH_PX: int = 900

#: Width of the far-left MODES rail band, in pixels. Mirrors
#: ``tools.path_planner.editor.panels._MODE_RAIL_WIDTH_PX``. Duplicated here
#: rather than imported because the panels module pins its own band widths with
#: ``pack_propagate(False)`` -- these constants only feed the initial window
#: sizing math.
_MODE_RAIL_WIDTH_PX: int = 90

#: Width of the right PROPERTIES inspector band, in pixels. Mirrors
#: ``tools.path_planner.editor.panels._INSPECTOR_WIDTH_PX``. Duplicated for the
#: same reason as :data:`_MODE_RAIL_WIDTH_PX`.
_INSPECTOR_WIDTH_PX: int = 312

#: Total fixed-chrome width the two side bands consume now that the layout is a
#: true three-pane window (rail + inspector). The canvas is the only band that
#: expands, so this is what the initial geometry reserves around the canvas.
#: Replaces the old single 240-px right-column figure (F1 flagged it stale).
_CHROME_TOTAL_WIDTH_PX: int = _MODE_RAIL_WIDTH_PX + _INSPECTOR_WIDTH_PX

#: Extra vertical pixels reserved for the matplotlib figure's axis labels plus
#: the full-width title band and status band that now bracket the canvas in the
#: three-pane layout, so the field rectangle itself fits inside the canvas area
#: without being clipped.
_CANVAS_CHROME_PX: int = 120

#: Minimum starting width for the expanding canvas band, in pixels. The window
#: floor (:data:`_MIN_WINDOW_WIDTH_PX`) is derived from this plus the two fixed
#: chrome columns so a degenerately tall / thin field still opens with a usable
#: canvas rather than a window only as wide as the chrome.
_MIN_CANVAS_WIDTH_PX: int = 480

#: Hard floor on the initial window dimensions so a degenerately tall or thin
#: field still produces a usable starting window. The width floor reserves the
#: two fixed chrome columns (rail + inspector) on top of a minimum canvas band.
_MIN_WINDOW_WIDTH_PX: int = _CHROME_TOTAL_WIDTH_PX + _MIN_CANVAS_WIDTH_PX
_MIN_WINDOW_HEIGHT_PX: int = 400

#: Valid node-type strings as displayed in dialogs and combobox submenus.
#: ``"reset"`` joins the inspector combobox's whitelist (a reset node IS a
#: selectable type), but it is never assigned through the plain ``SetNodeType``
#: path -- :meth:`EditorApp._on_panel_node_type_change` routes a transition into
#: or out of ``"reset"`` through :class:`SetResetSpec` / :class:`ClearResetSpec`
#: instead. The right-click "Set type" submenu still offers only line / blind.
_NODE_TYPE_LINE: str = "line"
_NODE_TYPE_BLIND: str = "blind"
_NODE_TYPE_RESET: str = "reset"
#: All node types the inline guard accepts; single-sourced from
#: ``model.NODE_TYPE_VALUES`` so the GUI never accepts a type the validator
#: rejects.
_VALID_NODE_TYPES: tuple[str, ...] = NODE_TYPE_VALUES

#: Plain (non-reset) node types a node may be retyped TO without touching a
#: reset spec.  Used by the type-change router to decide whether a transition
#: out of ``"reset"`` is a clear-spec (line / blind target) operation.
_PLAIN_NODE_TYPES: tuple[str, ...] = (_NODE_TYPE_LINE, _NODE_TYPE_BLIND)

#: Edge-kind tags for the two undirected plain edge lists. These are the string
#: values threaded through the ``L`` / ``B`` override flow and reported on the
#: EDGE-mode status line. They share their spelling with the node-type constants
#: above but name an EDGE's list (``line_segments`` vs ``blind_edges``), not a
#: node type -- kept distinct so the override / status code reads in edge terms.
_EDGE_KIND_LINE: str = "line"
_EDGE_KIND_BLIND: str = "blind"

#: Allowed wall-zero axis values for an inline reset-spec edit; single-sourced
#: from ``model.RESET_AXIS_VALUES`` (which mirrors
#: ``validator._ALLOWED_RESET_AXES``).  The inline guard rejects anything else
#: before the command is dispatched.
_RESET_AXES: tuple[str, ...] = RESET_AXIS_VALUES

#: Inclusive lower / exclusive upper bound (degrees, CCW from +x) for a reset
#: node's parked facing and any other heading the inline guards check.  An
#: out-of-range facing is an AC7 inline-reject case.
_FACING_MIN_DEG: float = 0.0
_FACING_MAX_DEG_EXCLUSIVE: float = 360.0

#: Decimal places the inspector entry fields DISPLAY numeric values at (the
#: panel formats coords / heading / reset coords with ``%.1f``).  The
#: skip-if-unchanged guards compare an incoming committed value against the
#: model's value AT THIS PRECISION: a field shows the rounded value, so a bare
#: tab-away (no real edit) returns the rounded value and must be treated as
#: unchanged -- comparing against full precision instead would dispatch a
#: spurious move that truncates the stored value to 0.1 and dirties the buffer.
_INSPECTOR_DISPLAY_DP: int = 1

#: Boolean global knob (the blind-decay enable toggle) -- its value arrives as a
#: ``bool`` and needs no numeric range check.  Mirrors
#: ``panels._BOOL_KNOB_NAME`` and the matching ``commands`` knob name.
_KNOB_BLIND_DECAY_ENABLED: str = "blind_decay_enabled"

#: Float global-knob names + their inline lower bounds.  Each maps a knob name to
#: ``(minimum, inclusive)`` where ``inclusive`` selects ``>=`` vs ``>``.  These
#: mirror the authoritative per-knob bounds in
#: :meth:`commands.SetGlobalKnob._coerce_and_validate`; the inline guard is the
#: cheap tier-1 copy that reverts a bad widget value without a dispatch round
#: trip, while the command remains the authority on apply.
_KNOB_BLIND_DECAY_K: str = "blind_decay_k"
_KNOB_BLIND_DECAY_EXPONENT: str = "blind_decay_exponent"
_KNOB_BLIND_DECAY_D_REF: str = "blind_decay_d_ref"
_KNOB_ROBOT_RADIUS_CM: str = "robot_radius_cm"
#: Per-knob ``(minimum, inclusive)`` bound. ``inclusive`` True -> value must be
#: ``>= minimum``; False -> strictly ``> minimum``.
_FLOAT_KNOB_BOUNDS: dict[str, tuple[float, bool]] = {
    _KNOB_BLIND_DECAY_K: (0.0, True),
    _KNOB_BLIND_DECAY_EXPONENT: (1.0, True),
    _KNOB_BLIND_DECAY_D_REF: (0.0, False),
    _KNOB_ROBOT_RADIUS_CM: (0.0, False),
}

#: Human-readable labels for the status-surface mode line, keyed by the
#: :class:`Mode` enum's ``value`` string. Mirrors the radio-button labels in
#: ``panels._MODE_LABELS`` so the surface names the mode the way the toolbar
#: does.
_MODE_DISPLAY_LABELS: dict[str, str] = {
    "add_node": "Add node",
    "move": "Move / select",
    "edge": "Edge (L / B)",
    "obstacle": "Obstacle",
    "start": "Set start",
}
#: Fallback shown on the mode line when ``mode_var`` holds an unknown string.
_MODE_DISPLAY_UNKNOWN: str = "--"

#: Prefix + placeholder for the status-surface selection line.
_SELECTION_DISPLAY_PREFIX: str = "Selection: "
_SELECTION_DISPLAY_NONE: str = "none"
#: Prefix for the status-surface mode line.
_MODE_DISPLAY_PREFIX: str = "Mode: "

#: Mouse button numbers per the matplotlib MouseEvent contract.
_MOUSE_BUTTON_LEFT: int = 1
_MOUSE_BUTTON_RIGHT: int = 3

#: Tolerance in centimetres around an obstacle drag's anchor before the
#: release is treated as a meaningful rectangle. Smaller drags are dropped.
_OBSTACLE_MIN_SIDE_CM: float = 0.5

#: Default obstacle ``kind`` tag offered in the dialog if the user does
#: not type one in. Matches a free-form string field; the validator does
#: not constrain it further.
_DEFAULT_OBSTACLE_KIND: str = "obstacle"

# The transient obstacle-drag preview rectangle's style now lives with the blit
# fast-path that draws it -- see ``canvas._OBSTACLE_PREVIEW_*`` (#11).

#: Edge-selection cap. Two clicks = a candidate pair. A third click drops
#: the oldest entry so the most recent two clicks always win.
_EDGE_SELECTION_MAX: int = 2

#: Single-key mode shortcuts (AC5). Each entry maps a Tk key *keysym* to the
#: :class:`Mode` it activates; the handler sets ``mode_var`` to the mode's
#: ``value`` (firing the existing ``_on_mode_change`` trace). Both the lower-
#: and upper-case form of every letter is bound in
#: :meth:`EditorApp._wire_keyboard_shortcuts`, mirroring the existing ``L`` /
#: ``B`` edge-action keys.
#:
#: The keys are deliberately disjoint from the existing shortcuts -- the Ctrl
#: chords (``Ctrl-Z`` / ``Ctrl-Y`` / ``Ctrl-S``), ``Delete``, and the EDGE-mode
#: edge-action keys ``L`` / ``B`` -- and from the arrow-key nudge below. Plain
#: ``s`` (Set start) does NOT shadow ``Ctrl-S`` (Save): Tk dispatches the most
#: specific binding for a modified keypress, so a ``Ctrl-S`` event fires only
#: ``<Control-s>`` and never the plain ``<KeyPress-s>`` binding.
_MODE_SHORTCUT_KEYS: dict[str, Mode] = {
    "a": Mode.ADD_NODE,
    "m": Mode.MOVE,
    "e": Mode.EDGE,
    "o": Mode.OBSTACLE,
    "s": Mode.START,
}

#: Arrow-key nudge step (centimetres) for the currently-selected node (AC4).
#: A plain arrow key nudges by the fine step; holding Shift uses the coarse
#: step. The new position is clamped to the field bounds before dispatch.
_NUDGE_FINE_CM: float = 0.2
_NUDGE_COARSE_CM: float = 1.0

#: Field-coordinate floor used when clamping a nudged node. The field's lower
#: corner is the origin; the upper bound is derived from ``model.field_mm``.
_FIELD_ORIGIN_CM: float = 0.0

#: Millimetres-per-centimetre conversion for deriving the field's width/height
#: in centimetres from ``model.field_mm`` (which is stored in millimetres).
_MM_PER_CM: float = 10.0

#: Status message shown when an arrow-key nudge fires with no node selected.
_STATUS_NUDGE_NO_SELECTION: str = "Select a node to nudge."

#: Status messages — collected here so the test suite (future work) and the
#: developer can grep for the exact text the GUI surfaces.
_STATUS_READY: str = "Ready."
_STATUS_SAVED: str = "Saved field_data.json"
_STATUS_BUILT: str = "Built routes.h + routes.cpp"
_STATUS_BUILD_FAILED: str = "Build failed; see dialog for details."

#: Sibling-of-field_data paths the Save handler writes to.
_PLANNER_ROOT: Path = Path(__file__).resolve().parent.parent
_FIELD_DATA_PY_PATH: Path = _PLANNER_ROOT / "field_data.py"
_FIELD_DATA_JSON_PATH: Path = _PLANNER_ROOT / "field_data.json"

#: Canonical keyboard-shortcut reference, the SINGLE source of truth for the
#: shortcuts dialog (AC9). Each group is ``(category_title, [(key, desc), ...])``
#: and mirrors the bindings installed in :meth:`EditorApp._wire_keyboard_shortcuts`.
#: When a binding is added or changed there, this table must change with it so
#: the operator-facing help stays accurate. The shape matches
#: :class:`dialogs.ShortcutsDialog`'s ``shortcut_groups`` parameter exactly.
_SHORTCUT_GROUPS: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = (
    (
        "Modes",
        (
            ("A", "Add node"),
            ("M", "Move / select"),
            ("E", "Edge"),
            ("O", "Obstacle"),
            ("S", "Set start"),
        ),
    ),
    (
        "Edit",
        (
            ("Ctrl+Z", "Undo"),
            ("Ctrl+Y", "Redo"),
            ("Delete", "Delete selected edge (EDGE mode)"),
            ("Arrows", "Nudge selected node 0.2 cm"),
            ("Shift+Arrows", "Nudge selected node 1 cm"),
        ),
    ),
    (
        "Edges",
        (
            ("L", "Add line edge (EDGE mode)"),
            ("B", "Add blind edge (EDGE mode)"),
        ),
    ),
    (
        "File",
        (
            ("Ctrl+S", "Save"),
            ("? / F1", "Show this help"),
        ),
    ),
)


class EditorApp:
    """Top-level Tk application class for the live field editor.

    Constructor side effects:

    * Loads ``field_data.json`` via the loader; raises if missing.
    * Constructs the in-memory :class:`FieldModel`.
    * Creates (or adopts) a :class:`tkinter.Tk` root window.
    * Applies the warm "drafting-table" theme to that root via
      :mod:`tools.path_planner.editor.theme` -- fonts then ttk palette then
      root chrome -- *before* the panels/canvas build so they pick up the
      base styling. Every theme call fails soft (no-op) when sv_ttk, a
      bundled font, or a display is unavailable.
    * Builds the :class:`FieldCanvas` and :class:`SidePanels`, then grids
      them into the true three-pane window via the panels' single placement
      entry point :meth:`SidePanels.layout_three_pane` (title + status bands
      full-width, mode rail + inspector fixed-width, canvas expanding).
    * Wires every panel callback, every canvas mouse hook, and the
      keyboard shortcuts (``Ctrl-Z`` / ``Ctrl-Y`` / ``Ctrl-S`` /
      ``Delete``).
    * Renders the initial frame.

    :param root: Optional pre-existing :class:`tk.Tk` instance. Used by
        tests that want to drive the app without spawning a fresh window.
        When ``None`` (the default) a new ``Tk`` is created.
    :param field_data_json_path: Optional override for the JSON sidecar
        path. ``None`` uses the default sibling of ``field_data.py``.
    """

    def __init__(
        self,
        root: Optional[tk.Tk] = None,
        *,
        field_data_json_path: Optional[Path] = None,
    ) -> None:
        """Build the Tk root window, side panels, canvas, and wiring.

        :param root: Optional Tk root to reuse (for tests).
        :param field_data_json_path: Optional explicit JSON path override.
        :raises tools.path_planner.validator.FieldDataError: If
            ``field_data.json`` cannot be read or parsed.
        """
        # ---- 1. Load the model from disk ----
        json_path = field_data_json_path or _FIELD_DATA_JSON_PATH
        data = loader.load_field_json(json_path)
        self._json_path: Path = json_path
        self._model: FieldModel = FieldModel.from_json_dict(data)

        # ---- 2. Import the field_data module to peek at EDGES on delete ----
        # Done lazily so unit tests that monkeypatch the JSON path can
        # avoid the import. The reference is captured once -- live edits
        # to field_data.py during a session are intentionally NOT
        # picked up (the user must restart the editor for those).
        try:
            from tools.path_planner import field_data as _field_data_module
            self._field_data_module = _field_data_module
        except Exception:  # noqa: BLE001 -- module import is best-effort
            self._field_data_module = None

        # ---- 3. History + dirty tracking + rename queue ----
        self._history: History = History()
        self._dirty: bool = False
        # Net node renames awaiting flush into field_data.py EDGES on the next
        # Save, keyed {current-EDGES-name: target-name}. Maintained as a delta:
        # a fresh rename records forward, an undo records the inverse, a redo
        # re-records forward (see _undo / _redo). It is NEVER rebuilt from the
        # history stacks -- doing so was the source of the undo-saved-rename
        # divergence, because History mutates (redo is cleared by any new edit).
        self._pending_renames: dict[str, str] = {}

        # ---- 4. Tk root window ----
        self._owns_root: bool = root is None
        self._root: tk.Tk = root if root is not None else tk.Tk()
        self._root.title(_WINDOW_TITLE)
        self._root.geometry(self._initial_window_geometry())

        # ---- 5. Warm theme (must precede panels/canvas) ----
        # Apply the drafting-table theme to the freshly-created root BEFORE the
        # side panels and canvas are constructed so they pick up the base ttk
        # styling and any bundled fonts as they build. Every call here fails
        # soft (no-op) when sv_ttk / a bundled font / a display is unavailable,
        # so this adds no new failure mode -- the __new__-bypassing headless
        # tests never reach it. Order is deliberate:
        #   1. Register bundled fonts first so panels resolve real families.
        #   2. Apply the base ttk theme + palette. T11's panels then define
        #      their OWN named styles on top, so this MUST run first or those
        #      named styles would be overwritten by the base recolor.
        #   3. Paint the root chrome the panel SURFACE tone so the window edge
        #      has no cold-grey seam against the side panels.
        theme.load_bundled_fonts()
        theme.apply_ttk_theme(self._root)
        self._root.configure(background=theme.SURFACE)

        # ---- 6. Side panels (chrome bands, not yet placed) ----
        # SidePanels builds the four chrome bands (title / mode rail /
        # inspector / status) as siblings of the canvas under the shared root,
        # but does NOT place them -- the single ``layout_three_pane`` call below
        # grids every band plus the canvas into the true three-pane window.
        self._panels: SidePanels = SidePanels(self._root)

        # ---- 7. Canvas (centre band, expands) ----
        # The canvas resolves ``model.map_png`` against the planner root
        # itself; it does not need a field_data module reference.
        self._canvas: FieldCanvas = FieldCanvas(self._root)
        self._size_canvas_to_field()

        # ---- 7b. Three-pane placement (single entry point) ----
        # Now that BOTH the chrome bands and the canvas widget exist, hand the
        # canvas to the panels' one placement method: it grids the title +
        # status bands full-width, the mode rail + inspector at their fixed
        # widths, and the canvas in the expanding centre cell.
        self._panels.layout_three_pane(self._canvas.tk_widget)

        # ---- 8. Mouse / mode state ----
        # Move mode: latched node currently being dragged.
        self._drag_target: Optional[str] = None
        # Move mode: original coords captured at press, so on release we
        # can restore them BEFORE issuing a MoveNode command (otherwise
        # the command's "_original_xy" would already be the dragged value).
        self._drag_origin_xy: Optional[tuple[float, float]] = None
        # Edge mode: up to two node names selected for the next L/B press.
        self._edge_selection: list[str] = []
        # Obstacle mode: anchor of the in-progress drag rectangle.
        self._obstacle_anchor: Optional[tuple[float, float]] = None
        # Obstacle mode: live (x, y) of the drag cursor, used by the
        # preview overlay. ``None`` when no drag is in progress.
        self._obstacle_cursor: Optional[tuple[float, float]] = None
        # Auto-create preview state (T8). ``None`` when no preview is active.
        # When non-None, the canvas renders the list as a dashed cyan overlay
        # and the SidePanels show Accept / Reject instead of Auto-create.
        self._proposed_edges: Optional[list[tuple[str, str, str]]] = None
        # Name of the node currently selected in the inspector listbox, or
        # ``None`` when nothing is selected. Maintained by
        # _on_panel_node_select and consumed by the arrow-key nudge (AC4) so a
        # nudge knows which node to move without re-reading the panel widget.
        self._selected_node: Optional[str] = None

        # ---- 9. Spinbox de-dup guard (reviewer note T11 W1) ----
        # SidePanels' Spinbox <FocusOut> may spuriously fire with the
        # same value already committed. Compare against this latch and
        # short-circuit if equal.
        self._last_committed_heading: float = float(
            self._model.start_heading_deg
        )

        # ---- 10. Wire callbacks ----
        self._wire_panel_callbacks()
        self._wire_canvas_callbacks()
        self._wire_keyboard_shortcuts()
        self._wire_window_close()

        # ---- 11. Initial render + panel population ----
        # Seed the title-bar filename from the JSON sidecar's basename so the
        # chrome names the file being edited; the dirty dot starts hidden
        # (a freshly loaded model is clean).
        self._panels.set_filename(self._json_path.name)
        self._panels.set_dirty(False)
        # Seed the status surface's mode + selection lines so they read the
        # current truth from the first frame (nothing is selected yet, so the
        # selection line shows the "none" placeholder).
        self._update_mode_display()
        self._update_selection_display(None)
        self._refresh_view()

    # ------------------------------------------------------------------
    # Initial sizing (canvas + window match the field's aspect ratio)
    # ------------------------------------------------------------------

    def _field_aspect_ratio(self) -> float:
        """Return ``field_mm[0] / field_mm[1]`` with a safe fallback.

        Used by :meth:`_initial_window_geometry` and
        :meth:`_size_canvas_to_field` to pick a starting size that mirrors
        the field's real-world proportions. A non-positive width or height
        falls back to ``1.0`` (square) so a degenerate model still yields a
        usable window instead of a ``ZeroDivisionError`` in the sizing math
        (``_initial_window_geometry`` divides ``canvas_w`` by this value).
        """
        width_mm, height_mm = self._model.field_mm
        if width_mm <= 0.0 or height_mm <= 0.0:
            return 1.0
        return float(width_mm) / float(height_mm)

    def _initial_window_geometry(self) -> str:
        """Compute a ``"WxH"`` geometry string sized to the field aspect.

        The canvas area is ``_DEFAULT_CANVAS_WIDTH_PX`` wide; its height is
        derived from the field aspect so the map fills the canvas without large
        empty bars. The window adds the two fixed chrome columns (the mode rail
        + inspector, :data:`_CHROME_TOTAL_WIDTH_PX`) on top of the canvas width
        and a small vertical chrome margin for matplotlib's axis labels plus the
        title / status bands.
        """
        aspect = self._field_aspect_ratio()
        canvas_w = _DEFAULT_CANVAS_WIDTH_PX
        canvas_h = max(int(round(canvas_w / aspect)), _MIN_WINDOW_HEIGHT_PX)
        window_w = max(canvas_w + _CHROME_TOTAL_WIDTH_PX, _MIN_WINDOW_WIDTH_PX)
        window_h = max(canvas_h + _CANVAS_CHROME_PX, _MIN_WINDOW_HEIGHT_PX)
        return f"{window_w}x{window_h}"

    def _size_canvas_to_field(self) -> None:
        """Resize the matplotlib figure to match the field aspect ratio.

        The canvas is laid out with ``fill="both", expand=True`` so the Tk
        widget tracks the window, but matplotlib's initial figsize still
        seeds the first paint. Without this, a 4:3 default figure paired
        with a 2:1 field leaves big gray bars above and below the map.
        """
        aspect = self._field_aspect_ratio()
        # Pick a width in inches and let the height follow the field aspect.
        # 9.0 in keeps the figure within the default Tk DPI render budget
        # while leaving room for axis labels.
        width_in = 9.0
        height_in = max(width_in / aspect, 2.5)
        self._canvas.figure.set_size_inches(width_in, height_in, forward=False)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Enter the Tk main loop. Blocks until the window is closed."""
        self._root.mainloop()

    # ------------------------------------------------------------------
    # Wiring helpers (called once from the constructor)
    # ------------------------------------------------------------------

    def _wire_panel_callbacks(self) -> None:
        """Hook every :class:`SidePanels` callback slot to a method on self."""
        self._panels.on_node_select = self._on_panel_node_select
        self._panels.on_node_type_change = self._on_panel_node_type_change
        self._panels.on_node_coords_change = self._on_panel_node_coords_change
        self._panels.on_node_rename = self._on_panel_node_rename
        self._panels.on_node_delete = self._on_panel_node_delete
        self._panels.on_start_node_change = self._on_panel_start_node_change
        self._panels.on_start_heading_change = (
            self._on_panel_start_heading_change
        )
        self._panels.on_undo = self._undo
        self._panels.on_redo = self._redo
        self._panels.on_save = self._save
        self._panels.on_save_and_build = self._save_and_build
        self._panels.on_auto_create_edges = self._on_panel_auto_create_edges
        self._panels.on_accept_proposed = self._on_panel_accept_proposed
        self._panels.on_reject_proposed = self._on_panel_reject_proposed

        # Property-inspector callbacks (T7): reset-spec edits, pivot add /
        # delete, and the global decay / robot-radius knobs. Each handler does
        # cheap tier-1 well-formedness checks before dispatching; the
        # authoritative geometry validation stays in model.validate() at Save.
        self._panels.on_reset_spec_change = self._on_panel_reset_spec_change
        self._panels.on_add_pivot = self._on_panel_add_pivot
        self._panels.on_delete_pivot = self._on_panel_delete_pivot
        self._panels.on_global_knob_change = self._on_panel_global_knob_change

        # Status-strip shortcuts hint: opens the read-only reference dialog.
        self._panels.on_show_shortcuts = self._show_shortcuts_dialog

        # Clear transient drag / preview / selection state whenever the
        # user switches modes — otherwise a half-finished obstacle drag
        # or edge pick would latch across a mode change and surprise the
        # user the next time they clicked.
        self._panels.mode_var.trace_add(
            "write", lambda *_args: self._on_mode_change()
        )

    def _wire_canvas_callbacks(self) -> None:
        """Hook the three :class:`FieldCanvas` mouse slots to self handlers."""
        self._canvas.on_click = self._on_canvas_click
        self._canvas.on_drag = self._on_canvas_drag
        self._canvas.on_release = self._on_canvas_release

    def _wire_keyboard_shortcuts(self) -> None:
        """Bind global keyboard shortcuts on the Tk root.

        Bindings (case-insensitive where Tk allows):

        * ``Ctrl-Z``                    -> :meth:`_undo`
        * ``Ctrl-Y`` / ``Ctrl-Shift-Z`` -> :meth:`_redo`
        * ``Ctrl-S``                    -> :meth:`_save`
        * ``Delete``                    -> :meth:`_handle_delete_key`
        * ``L``                         -> :meth:`_handle_edge_line_key`
        * ``B``                         -> :meth:`_handle_edge_blind_key`
        * ``?`` / ``F1``                -> :meth:`_show_shortcuts_dialog`
        * ``A`` / ``M`` / ``E`` / ``O`` / ``S`` -> switch mode (AC5), per
          :data:`_MODE_SHORTCUT_KEYS` (Add node / Move / Edge / Obstacle /
          Set start).
        * ``Up`` / ``Down`` / ``Left`` / ``Right`` -> nudge the selected node
          by :data:`_NUDGE_FINE_CM`; with ``Shift`` by :data:`_NUDGE_COARSE_CM`
          (AC4).

        Every binding is routed through :meth:`_shortcut_guard`, which drops
        the event when keyboard focus is on a text-entry or dialog widget so
        typing an ``l`` / ``b`` / ``a`` / ``Delete`` / an arrow (or a Ctrl
        chord) into the rename dialog or a coordinate Entry does not fire an
        editor action behind it (#67). ``bind_all`` is kept (the canvas is a
        matplotlib widget that is awkward to focus reliably); the focus check
        does the gating.

        Plain ``s`` (Set start) and ``Ctrl-S`` (Save) coexist: Tk dispatches
        the most specific binding for a modified keypress, so a ``Ctrl-S``
        event fires ``<Control-s>`` only and never the plain ``<KeyPress-s>``.
        """
        guard = self._shortcut_guard
        self._root.bind_all("<Control-z>", guard(self._undo))
        self._root.bind_all("<Control-Z>", guard(self._undo))
        self._root.bind_all("<Control-y>", guard(self._redo))
        self._root.bind_all("<Control-Y>", guard(self._redo))
        self._root.bind_all("<Control-Shift-Z>", guard(self._redo))
        self._root.bind_all("<Control-Shift-z>", guard(self._redo))
        self._root.bind_all("<Control-s>", guard(self._save))
        self._root.bind_all("<Control-S>", guard(self._save))
        self._root.bind_all("<Delete>", guard(self._handle_delete_key))
        self._root.bind_all("<KeyPress-l>", guard(self._handle_edge_line_key))
        self._root.bind_all("<KeyPress-L>", guard(self._handle_edge_line_key))
        self._root.bind_all("<KeyPress-b>", guard(self._handle_edge_blind_key))
        self._root.bind_all("<KeyPress-B>", guard(self._handle_edge_blind_key))

        # Shortcuts-help dialog (AC9): ``?`` and ``F1`` both open it. ``?`` is
        # Shift+/ -- bind the keysym ``question`` (its plain glyph) so the
        # focus-guard still gates it while typing into a field.
        self._root.bind_all(
            "<KeyPress-question>", guard(self._show_shortcuts_dialog)
        )
        self._root.bind_all("<F1>", guard(self._show_shortcuts_dialog))

        # Mode-switch shortcuts (AC5). Bind both the lower- and upper-case form
        # of each letter (mirroring L / B) so the key works regardless of Caps
        # Lock / Shift state. Each closure captures its own ``mode`` -- a bare
        # ``mode`` reference would late-bind to the loop's final value.
        for key, mode in _MODE_SHORTCUT_KEYS.items():
            handler = self._make_mode_switch_handler(mode)
            self._root.bind_all(f"<KeyPress-{key}>", guard(handler))
            self._root.bind_all(
                f"<KeyPress-{key.upper()}>", guard(handler)
            )

        # Arrow-key nudge of the selected node (AC4). Plain arrows use the fine
        # step; Shift-arrows use the coarse step. Up = +y, Down = -y, Left =
        # -x, Right = +x (y is North/up in this model). Each closure captures
        # its own (dx, dy) so the guard can fire it as a zero-arg handler.
        nudge_bindings: tuple[tuple[str, float, float], ...] = (
            ("<Up>", 0.0, _NUDGE_FINE_CM),
            ("<Down>", 0.0, -_NUDGE_FINE_CM),
            ("<Left>", -_NUDGE_FINE_CM, 0.0),
            ("<Right>", _NUDGE_FINE_CM, 0.0),
            ("<Shift-Up>", 0.0, _NUDGE_COARSE_CM),
            ("<Shift-Down>", 0.0, -_NUDGE_COARSE_CM),
            ("<Shift-Left>", -_NUDGE_COARSE_CM, 0.0),
            ("<Shift-Right>", _NUDGE_COARSE_CM, 0.0),
        )
        for sequence, dx_cm, dy_cm in nudge_bindings:
            handler = self._make_nudge_handler(dx_cm, dy_cm)
            self._root.bind_all(sequence, guard(handler))

    def _shortcut_guard(self, handler):
        """Wrap a shortcut *handler* so it only fires when focus is not text.

        ``bind_all`` delivers key events to every widget, including the rename
        ``simpledialog`` and the side-panel Entry / Spinbox / Combobox fields.
        This wrapper inspects ``event.widget`` (the widget that actually owns
        the keyboard focus for the event) and suppresses the handler when that
        widget is a text-input class, so a literal keystroke typed into a field
        never triggers an editor action (#67).

        :param handler: Zero-argument callable to invoke on an allowed event.
        :return: A Tk event callback (takes the event, returns ``None``).
        """

        def _wrapped(event=None) -> None:
            if not self._shortcut_focus_allows(event):
                return
            handler()

        return _wrapped

    @staticmethod
    def _shortcut_focus_allows(event) -> bool:
        """Return ``True`` when a global shortcut may fire for *event*.

        Returns ``False`` when the focused widget is a text-input or dialog
        entry (Tk class name ending in ``Entry``, or a ``Text`` / ``Spinbox`` /
        ``TCombobox``), so shortcuts are inert while the operator is typing.
        Any non-text widget (the canvas, buttons, the root, a listbox) allows
        the shortcut. A missing widget reference is treated as allowed so the
        shortcuts still work when Tk does not populate ``event.widget``.

        :param event: The Tk key event (may be ``None``).
        :return: ``True`` if the shortcut should run, ``False`` to suppress.
        """
        widget = getattr(event, "widget", None)
        if widget is None:
            return True
        # ``winfo_class`` is the reliable cross-widget way to identify the
        # focused widget's type. Entry/ttk.Entry report "Entry"/"TEntry",
        # Spinbox "Spinbox"/"TSpinbox", Combobox "TCombobox", and the Text
        # widget "Text". A simpledialog's input is a plain Entry.
        try:
            widget_class = widget.winfo_class()
        except Exception:  # noqa: BLE001 -- non-widget or torn-down focus
            return True
        text_input_classes = {
            "Entry",
            "TEntry",
            "Spinbox",
            "TSpinbox",
            "TCombobox",
            "Text",
        }
        return widget_class not in text_input_classes

    def _make_mode_switch_handler(self, mode: Mode):
        """Return a zero-arg handler that switches the active mode to *mode*.

        The returned callable is what :meth:`_shortcut_guard` wraps: the guard
        does the text-focus check, then invokes this handler with no arguments.
        Each call binds *mode* per-closure so a loop over
        :data:`_MODE_SHORTCUT_KEYS` does not late-bind to the final value.

        :param mode: The :class:`Mode` this shortcut activates.
        :return: A zero-argument callable suitable for :meth:`_shortcut_guard`.
        """

        def _handler() -> None:
            self._switch_mode(mode)

        return _handler

    def _make_nudge_handler(self, dx_cm: float, dy_cm: float):
        """Return a zero-arg handler that nudges the selected node by a delta.

        The returned callable is what :meth:`_shortcut_guard` wraps: the guard
        does the text-focus check, then invokes this handler with no arguments.
        Each call binds *(dx_cm, dy_cm)* per-closure so the arrow-binding loop
        does not late-bind every arrow key to the final delta.

        :param dx_cm: Signed x displacement in centimetres.
        :param dy_cm: Signed y displacement in centimetres.
        :return: A zero-argument callable suitable for :meth:`_shortcut_guard`.
        """

        def _handler() -> None:
            self._nudge_selected(dx_cm, dy_cm)

        return _handler

    def _switch_mode(self, mode: Mode) -> None:
        """Set the active editing mode (AC5) by writing ``mode_var``.

        Writing ``mode_var`` fires the existing ``_on_mode_change`` write trace,
        which clears any in-progress drag / edge pick / obstacle preview and
        re-renders -- the same path the toolbar radio buttons take -- so a
        keyboard mode switch behaves identically to a click on the toolbar.
        Writing the value it already holds is a Tk no-op (Tk only fires the
        write trace when ``set`` is called, but the cleared transient state is
        idempotent, so re-pressing the active mode's key is harmless).

        :param mode: The :class:`Mode` to activate.
        """
        self._panels.mode_var.set(mode.value)

    def _nudge_selected(self, dx_cm: float, dy_cm: float) -> None:
        """Move the selected node by *(dx_cm, dy_cm)*, clamped to the field (AC4).

        Acts only when a node is selected (tracked in :attr:`_selected_node`).
        The new position is the current coordinates plus the delta, clamped to
        the field bounds ``[0, width_cm] x [0, height_cm]`` derived from
        ``model.field_mm``. A nudge that leaves the node unmoved -- already
        sitting on the clamped bound -- is a no-op (no dispatch, no empty undo
        entry), mirroring the inspector's "skip if unchanged" coords guard.

        :param dx_cm: Signed x displacement in centimetres (Left -, Right +).
        :param dy_cm: Signed y displacement in centimetres (Down -, Up +).
        """
        node = self._selected_node
        if node is None:
            self._set_status(_STATUS_NUDGE_NO_SELECTION)
            return
        # Defensive: the selection latch may name a node that was deleted or
        # renamed since it was last set. Drop the stale latch and bail rather
        # than dispatch a MoveNode against a missing node.
        current = self._model.nodes.get(node)
        if current is None:
            self._selected_node = None
            self._set_status(_STATUS_NUDGE_NO_SELECTION)
            return

        width_mm, height_mm = self._model.field_mm
        width_cm = float(width_mm) / _MM_PER_CM
        height_cm = float(height_mm) / _MM_PER_CM

        cx, cy = float(current[0]), float(current[1])
        new_x = self._clamp(cx + dx_cm, _FIELD_ORIGIN_CM, width_cm)
        new_y = self._clamp(cy + dy_cm, _FIELD_ORIGIN_CM, height_cm)

        # No-op shortcut: a nudge that would not move the node (already on the
        # clamped bound) should not burn an undo entry.
        if new_x == cx and new_y == cy:
            return

        self._dispatch(MoveNode(node, (new_x, new_y)))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        """Clamp *value* to the inclusive range ``[low, high]``.

        Degenerate ranges (``high < low``, possible only for a field with a
        non-positive dimension) collapse to *low* so the result still lies on a
        defined bound rather than producing an inverted clamp.

        :param value: The value to clamp.
        :param low: Inclusive lower bound.
        :param high: Inclusive upper bound.
        :return: *value* clamped into ``[low, high]``.
        """
        if high < low:
            return low
        if value < low:
            return low
        if value > high:
            return high
        return value

    def _wire_window_close(self) -> None:
        """Intercept the window-manager close ("X") so we can warn on dirty."""
        self._root.protocol("WM_DELETE_WINDOW", self._on_close_request)

    # ------------------------------------------------------------------
    # View refresh + dirty bookkeeping
    # ------------------------------------------------------------------

    def _refresh_view(self) -> None:
        """Re-render canvas + side panels + button enable states + title.

        Ends with a :meth:`_revalidate` so the validation badge + message slot
        track the model after every mutation (dispatch / undo / redo all funnel
        through here) -- "per move" validation, not just at Save.
        """
        highlighted = list(self._edge_selection)
        self._canvas.render(
            self._model,
            highlighted_nodes=highlighted,
            proposed_edges=self._proposed_edges,
        )
        self._panels.refresh(self._model)
        self._panels.set_undo_enabled(self._history.can_undo)
        self._panels.set_redo_enabled(self._history.can_redo)
        self._update_title()
        self._revalidate()

    def _revalidate(self) -> None:
        """Re-run the full validator and reflect the verdict in the status bar.

        Hooked into :meth:`_refresh_view`, so it fires after every model
        mutation -- this is the "per move" validation the operator sees live
        rather than only at Save.  The full geometry validator runs in well
        under a millisecond on a field this size, so there is no debounce; the
        verdict drives the badge + message slot via
        :meth:`SidePanels.set_validation`.

        A refresh must never crash on a validator hiccup, so any exception is
        caught and surfaced as the INVALID detail (the Save-time gate uses the
        same broad catch).
        """
        from tools.path_planner.validator import (  # noqa: PLC0415
            FieldDataError,
        )

        try:
            self._model.validate()
        except FieldDataError as exc:
            self._panels.set_validation(False, str(exc))
            return
        except Exception as exc:  # noqa: BLE001 -- a refresh must not crash
            self._panels.set_validation(False, f"validation error: {exc}")
            return
        self._panels.set_validation(True)

    def _update_title(self) -> None:
        """Reflect :attr:`_dirty` in the window title bar."""
        suffix = _DIRTY_TITLE_SUFFIX if self._dirty else ""
        self._root.title(f"{_WINDOW_TITLE}{suffix}")

    def _mark_dirty(self) -> None:
        """Set the dirty flag, update the title, and show the dirty dot."""
        self._dirty = True
        self._update_title()
        self._panels.set_dirty(True)

    def _clear_dirty(self) -> None:
        """Clear the dirty flag, update the title, and hide the dirty dot."""
        self._dirty = False
        self._update_title()
        self._panels.set_dirty(False)

    def _set_status(self, text: str) -> None:
        """Push *text* into the status bar."""
        self._panels.set_status(text)

    # ------------------------------------------------------------------
    # Command dispatch helper
    # ------------------------------------------------------------------

    def _dispatch(self, command) -> bool:
        """Run *command* through ``history.do`` and refresh on success.

        On success: marks the model dirty, refreshes the view, returns
        ``True``. On failure: reports the exception's message to the
        status bar, leaves the view untouched, returns ``False``. In both
        cases the model is left in a coherent state (either fully applied
        or fully untouched -- guaranteed by the Command contract).

        :param command: Any object satisfying the
            :class:`~tools.path_planner.editor.commands.Command` protocol.
        :return: ``True`` on success, ``False`` on rejected pre-condition.
        """
        try:
            self._history.do(command, self._model)
        except (ValueError, KeyError, IndexError) as exc:
            self._set_status(f"{type(command).__name__}: {exc}")
            return False
        self._mark_dirty()
        self._refresh_view()
        return True

    # ------------------------------------------------------------------
    # Undo / redo
    # ------------------------------------------------------------------

    def _undo(self) -> None:
        """Pop one command off the undo stack and re-render."""
        # Peek the command BEFORE undo pops it so we can mirror its effect onto
        # the pending-rename delta.
        undone = self._history.undo_stack[-1] if self._history.can_undo else None
        if not self._history.undo(self._model):
            self._set_status("Nothing to undo.")
            return
        # Any undo brings the buffer back into "dirty vs saved" ambiguity --
        # we conservatively mark dirty so the user is prompted on close.
        self._mark_dirty()
        # Undoing a rename reverts new -> old in the model, so the pending
        # EDGES rewrite must move the same way. _record_pending_rename's
        # chain-collapse cancels a not-yet-saved rename (A->B then its inverse
        # collapses to {}) and inverts an already-saved one (leaving {B: A} so
        # the next Save reverts field_data.py to match field_data.json) with
        # the same call -- no need to know whether the rename was saved.
        if isinstance(undone, RenameNode):
            self._record_pending_rename(undone.new, undone.old)
        self._refresh_view()
        self._set_status("Undid last action.")

    def _redo(self) -> None:
        """Re-apply one command from the redo stack and re-render."""
        # Peek the command BEFORE redo pops it (redo re-applies redo_stack[-1]).
        redone = self._history.redo_stack[-1] if self._history.can_redo else None
        if not self._history.redo(self._model):
            self._set_status("Nothing to redo.")
            return
        self._mark_dirty()
        # Re-applying a rename re-establishes old -> new; record it forward so
        # the queue is symmetric with _undo. A non-rename redo leaves the queue
        # untouched.
        if isinstance(redone, RenameNode):
            self._record_pending_rename(redone.old, redone.new)
        self._refresh_view()
        self._set_status("Redid last action.")

    # ------------------------------------------------------------------
    # Panel callback handlers
    # ------------------------------------------------------------------

    def _on_panel_node_select(self, node_name: Optional[str]) -> None:
        """Called when the listbox selection changes.  Updates the status bar
        and triggers a full view refresh so the inspector (driven by
        SidePanels.refresh) reflects the new selection.

        :param node_name: Selected node name, or ``None`` if cleared.
        """
        # Track the selection so the arrow-key nudge (AC4) knows which node to
        # move; ``None`` clears it.
        self._selected_node = node_name
        if node_name is None:
            self._set_status("Selection cleared.")
        else:
            self._set_status(f"Selected node {node_name!r}.")
        self._update_selection_display(node_name)
        self._refresh_view()

    def _on_panel_node_type_change(self, node_name: str, new_type: str) -> None:
        """Route an inspector type-combobox change to the correct command.

        The plain :class:`SetNodeType` command refuses any transition that
        touches the ``"reset"`` type in either direction, so this handler
        dispatches by case:

        * ``new_type == "reset"`` and the node is NOT already reset -> plant a
          reset spec via :meth:`SetResetSpec.autofilled_auto`, seeding axis /
          wall_coord / facing from the node geometry (the nearest of all four
          walls, facing away) so the operator gets a real suggestion, not a
          placeholder. The values stay editable and recompute on an axis /
          position change.
        * ``new_type`` in {line, blind} and the node IS currently reset ->
          :class:`ClearResetSpec`, which removes the spec and re-types the node.
        * Otherwise (a plain line <-> blind change) -> :class:`SetNodeType`.

        :param node_name: Name of the node whose type was edited.
        :param new_type: New type string (already constrained to one of
            :data:`_VALID_NODE_TYPES` by the panel combobox).
        """
        if new_type not in _VALID_NODE_TYPES:
            self._set_status(f"Invalid node type {new_type!r}.")
            return
        # Skip the round-trip if the type is unchanged -- the panel's
        # spurious-event protection mostly handles this, but a belt-and-
        # braces check costs nothing and avoids a useless undo entry.
        current_type = self._model.node_types.get(node_name)
        if current_type == new_type:
            return

        if new_type == _NODE_TYPE_RESET:
            # Plain -> reset: SetResetSpec atomically sets type=reset AND writes
            # the reset_nodes spec, keeping the validator's type-and-spec
            # invariant intact at every step. Seed axis / wall_coord / facing
            # from the node geometry via reset_autofill_auto (the NEAREST of all
            # four walls, facing away), mirroring the Add-node-reset path so the
            # operator gets a real suggestion instead of a (0,0) placeholder.
            # Both stay editable and recompute on an axis / position change.
            if self._dispatch(
                SetResetSpec.autofilled_auto(
                    node_name,
                    node_xy=self._model.nodes[node_name],
                    field_mm=self._model.field_mm,
                )
            ):
                # SetResetSpec.apply has written the auto-derived spec; read it
                # back to warn the operator if the nearest wall is under the
                # pivot-clearance floor (the spec stands; Save is the hard gate).
                spec = self._model.reset_nodes[node_name]
                self._warn_if_reset_too_close(
                    node_name, spec["axis"], spec["wall_coord"]
                )
            return

        if current_type == _NODE_TYPE_RESET:
            # Reset -> line/blind: ClearResetSpec removes the spec and re-types.
            # SetNodeType would refuse a currently-reset node.
            self._dispatch(ClearResetSpec(node_name, new_type))
            return

        # Plain line <-> blind: no reset spec involved.
        self._dispatch(SetNodeType(node_name, new_type))

    def _on_panel_node_coords_change(
        self, node_name: str, x_cm: float, y_cm: float
    ) -> None:
        """Dispatch a :class:`MoveNode` when the inspector x/y entries commit.

        Skips the round-trip if the new coordinates equal the current ones --
        prevents an empty undo entry on a FocusOut that wasn't actually edited.

        :param node_name: Node whose coordinates were edited.
        :param x_cm: New x in centimetres.
        :param y_cm: New y in centimetres.
        """
        current = self._model.nodes.get(node_name)
        if current is None:
            return
        cx, cy = current
        # Compare at the inspector's DISPLAY precision: the entry shows the
        # rounded coordinate, so a bare tab-away returns that rounded value.
        # Comparing against full precision would treat an unedited focus-out as
        # a real move and truncate the stored coord to 0.1 cm (#8).
        if round(float(cx), _INSPECTOR_DISPLAY_DP) == round(
            x_cm, _INSPECTOR_DISPLAY_DP
        ) and round(float(cy), _INSPECTOR_DISPLAY_DP) == round(
            y_cm, _INSPECTOR_DISPLAY_DP
        ):
            return
        self._dispatch(MoveNode(node_name, (x_cm, y_cm)))

    def _on_panel_node_rename(self, old: str, new: str) -> None:
        """Rename the selected node from the inspector's name entry.

        Delegates to :meth:`_perform_rename` (shared with the right-click
        rename).  When the rename is rejected -- a duplicate / invalid name, or
        a dispatch pre-condition failure -- the model is untouched and we
        re-sync the panel so the name entry snaps back to the node's current
        name instead of keeping the rejected text.

        :param old: The node's current name (the selection at edit time).
        :param new: The proposed new name typed into the entry.
        """
        if not self._perform_rename(old, new):
            self._refresh_view()

    def _on_panel_node_delete(self, node: str) -> None:
        """Delete the selected node from the inspector's Delete button.

        Routes to :meth:`_context_delete` so the inspector button runs the same
        start-node / EDGES-reference guards and confirmation dialog as the
        right-click delete.

        :param node: The node to delete (the current selection).
        """
        self._context_delete(node)

    def _on_panel_start_node_change(self, node_name: str) -> None:
        """Dispatch a :class:`SetStart` when the start-node combobox changes.

        :param node_name: New start-node name (already validated by the
            panel as one of the currently-known nodes).
        """
        if node_name == self._model.start_node:
            return
        self._dispatch(SetStart(node_name))

    def _on_panel_start_heading_change(self, heading_deg: float) -> None:
        """Dispatch a :class:`SetStartHeading` when the spinbox commits.

        De-duplicates spurious ``<FocusOut>`` events (reviewer note T11 W1):
        the incoming value is compared against the model's current heading at
        the inspector's DISPLAY precision. The spinbox shows the rounded
        heading, so a bare tab-away returns that rounded value; comparing at
        full precision (the old latch held the full-precision committed value)
        treated an unedited focus-out as a change and truncated the stored
        heading to 0.1 deg (#8). The latch is still updated after a real commit.

        :param heading_deg: New heading in degrees, ``[0, 360)``.
        """
        if round(heading_deg, _INSPECTOR_DISPLAY_DP) == round(
            self._model.start_heading_deg, _INSPECTOR_DISPLAY_DP
        ):
            return
        if self._dispatch(SetStartHeading(heading_deg)):
            self._last_committed_heading = heading_deg

    # ------------------------------------------------------------------
    # Property-inspector handlers (T7): reset specs, pivots, global knobs.
    #
    # Two-tier validation. These handlers run cheap, hand-written tier-1
    # well-formedness guards (membership / range / incidence) BEFORE dispatch.
    # On a failed guard they write a specific status message, re-sync the panel
    # from the model via _refresh_view() so the bad widget value reverts, and do
    # NOT dispatch -- so a malformed field value never reaches the model.
    # The full geometry validator (model.validate()) is NOT run from these
    # tier-1 guards -- it would fire on every rejected keystroke. Instead it
    # runs once per COMMITTED move from _revalidate() (hooked into
    # _refresh_view), which is fast enough (sub-millisecond on a field this
    # size) to drive the live VALID/INVALID badge after each mutation; Save
    # re-runs the same validator as the authoritative gate.
    # ------------------------------------------------------------------

    def _reject_inline_edit(self, message: str) -> None:
        """Report a rejected tier-1 inline edit and revert the offending widget.

        Centralises the reject-and-revert path the inspector handlers share:
        writes *message* to the status bar, drops the validation badge to its
        INVALID state (the edit did not pass), and re-syncs every panel widget
        from the model via :meth:`_refresh_view` so the rejected value is
        overwritten with the model's current truth (mirrors the panel layer's
        own reject-and-revert pattern in ``_handle_coords_commit``).

        :param message: Specific reason the edit was rejected.
        """
        self._set_status(message)
        self._panels.set_validation_badge(False)
        self._refresh_view()

    def _warn_if_reset_too_close(
        self, name: str, axis: str, wall_coord: float
    ) -> None:
        """Warn (status + INVALID badge) when an auto-derived reset wall is too close.

        Called right after a seed / axis-change :class:`SetResetSpec` dispatch
        succeeds. When the auto-derived wall stands off the node by less than
        :data:`projection_edges.WALL_PIVOT_CLEARANCE_CM` (the pivot-sweep
        clearance the Save-time validator enforces), the spec is still applied --
        auto-fill suggests, it never traps -- but the operator is told the actual
        standoff and the distance to move the node back, and the validation badge
        drops to INVALID. When the wall clears the floor this is a no-op.

        The message is ASCII-only (no ``>=`` glyph, no em-dash) to avoid any
        source / console encoding surprise; the clearance figure is interpolated
        from the shared constant, never a hardcoded ``14``.

        This deliberately calls :meth:`SidePanels.set_validation_badge` (not
        :meth:`SidePanels.set_validation`): both drive the same badge widget, but
        the headless test double implements only ``set_status`` / ``refresh`` /
        ``set_validation_badge``. In the live app ``_dispatch -> _refresh_view ->
        _revalidate`` already calls ``set_validation(False, ...)`` for a < 14 cm
        node, so the two agree (both INVALID) rather than fighting.

        :param name: Name of the reset node whose seeded spec is being checked.
        :param axis: The auto-derived wall axis (``"x"`` or ``"y"``).
        :param wall_coord: The auto-derived wall coordinate in centimetres.
        """
        node_xy = self._model.nodes[name]
        if not projection_edges.is_reset_wall_too_close(node_xy, axis, wall_coord):
            return
        node_coord = node_xy[0] if axis == "x" else node_xy[1]
        standoff = abs(wall_coord - node_coord)
        clearance = projection_edges.WALL_PIVOT_CLEARANCE_CM
        self._set_status(
            f"Reset node '{name}' is {standoff:.1f} cm from the "
            f"{axis}={wall_coord:g} wall, below the {clearance:g} cm pivot "
            f"clearance; move it back to at least {clearance:g} cm."
        )
        self._panels.set_validation_badge(False)

    def _on_panel_reset_spec_change(
        self, name: str, axis: str, wall_coord: float, facing: float
    ) -> None:
        """Dispatch a :class:`SetResetSpec` from the reset-spec sub-panel.

        Tier-1 guards (cheap, before dispatch): the node must exist and the axis
        must be ``"x"`` / ``"y"``. Then the handler branches on whether the axis
        changed:

        * **Axis changed** (new axis != the node's stored axis): the typed wall
          / facing belong to the old axis, so both are RE-DERIVED from the node
          position + field size against the new axis via
          :func:`projection_edges.reset_autofill` and that auto-filled pair is
          dispatched. This is the "can it auto-fill?" win -- flipping the axis
          re-suggests a sensible wall + facing rather than carrying a stale one.
        * **Axis unchanged** (the operator hand-edited wall or facing): the
          facing is range-checked (``[0, 360)``; an out-of-range value is an AC7
          inline-reject) and the operator's values are dispatched verbatim --
          no recompute, so a manual edit is never clobbered.

        The wall coordinate is unconstrained at this tier -- whether it actually
        lies on the field wall is a geometry question for the Save-time
        validator. On any failed guard the edit is rejected and the panel
        reverts; otherwise the spec is dispatched.

        :param name: Node whose reset spec is being authored.
        :param axis: Wall-zero axis, expected to be ``"x"`` or ``"y"``.
        :param wall_coord: Wall coordinate in centimetres along *axis* (used
            only on an unchanged-axis edit).
        :param facing: Parked heading in degrees, expected in ``[0, 360)``
            (used only on an unchanged-axis edit).
        """
        if name not in self._model.nodes:
            self._reject_inline_edit(
                f"Reset spec: unknown node {name!r}."
            )
            return
        if axis not in _RESET_AXES:
            self._reject_inline_edit(
                f"Reset spec: axis must be one of {_RESET_AXES!r}, "
                f"got {axis!r}."
            )
            return

        # Axis-change auto-fill (the "can it auto-fill?" win): when the operator
        # picks a DIFFERENT axis than the node currently stores, the hand-typed
        # wall / facing belong to the old axis and would be wrong for the new
        # one. Re-derive both from the node position + field size against the
        # new axis and dispatch THAT, so the panel shows a sensible wall +
        # facing immediately. When the axis is UNCHANGED the operator is editing
        # the wall or facing by hand -- dispatch their values verbatim (no
        # recompute) so a manual edit is never clobbered.
        current_axis = self._model.reset_nodes.get(name, {}).get("axis")
        if axis != current_axis:
            try:
                new_wall, new_facing = projection_edges.reset_autofill(
                    self._model.nodes[name], axis, self._model.field_mm
                )
            except ValueError as exc:
                # axis was already membership-checked above; this is defensive.
                self._reject_inline_edit(f"Reset spec: {exc}")
                return
            if self._dispatch(SetResetSpec(name, axis, new_wall, new_facing)):
                # The re-derived axis is auto-fill too: warn if the nearest wall
                # on the chosen axis is under the pivot-clearance floor (the
                # spec stands; Save is the hard gate). We hold new_wall locally,
                # so no read-back is needed here.
                self._warn_if_reset_too_close(name, axis, new_wall)
            return

        # Axis unchanged: the operator edited wall / facing by hand. Range-check
        # the facing (an out-of-range facing is an AC7 inline-reject case); the
        # wall coordinate is unconstrained here -- whether it lies on the field
        # wall is a geometry question for the Save-time validator.
        if not (_FACING_MIN_DEG <= facing < _FACING_MAX_DEG_EXCLUSIVE):
            self._reject_inline_edit(
                f"Reset spec: facing must be in "
                f"[{_FACING_MIN_DEG}, {_FACING_MAX_DEG_EXCLUSIVE}), "
                f"got {facing}."
            )
            return
        # Skip a no-op focus-out: the wall / facing entries SHOW rounded values,
        # so a bare tab-away on an unchanged axis returns the same rounded pair.
        # Without this guard the verbatim dispatch would truncate the stored
        # spec to 0.1 and dirty the buffer on a focus-out that edited nothing
        # (#8). A genuine edit differs at display precision and still dispatches.
        stored = self._model.reset_nodes.get(name, {})
        stored_wall = stored.get("wall_coord")
        stored_facing = stored.get("facing")
        if (
            stored_wall is not None
            and stored_facing is not None
            and round(float(stored_wall), _INSPECTOR_DISPLAY_DP)
            == round(wall_coord, _INSPECTOR_DISPLAY_DP)
            and round(float(stored_facing), _INSPECTOR_DISPLAY_DP)
            == round(facing, _INSPECTOR_DISPLAY_DP)
        ):
            return
        self._dispatch(SetResetSpec(name, axis, wall_coord, facing))

    def _on_panel_add_pivot(self, u: str, v: str, w: str) -> None:
        """Dispatch an :class:`AddPivotTurn` from the pivots add-row.

        Tier-1 guards (cheap, before dispatch): all three endpoints must be
        known nodes, and both legs must be REAL incident edges -- each of the
        pairs ``{u, v}`` and ``{v, w}`` must exist as a ``line_segments`` /
        ``blind_edges`` entry (see :meth:`_edge_exists`).
        Without an incident leg the turn could never be executed, so it is
        rejected inline. On any failed guard
        the edit is rejected (no panel revert is needed -- the readonly add-row
        pickers keep their chosen names so the operator can fix the third one).

        :param u: Arrive-from node name.
        :param v: Pivot node name.
        :param w: Leave-to node name.
        """
        for role, node in (("u", u), ("v", v), ("w", w)):
            if node not in self._model.nodes:
                self._set_status(
                    f"Pivot: unknown node {node!r} ({role})."
                )
                self._panels.set_validation_badge(False)
                return
        # Distinctness (mirrors AddPivotTurn.apply + the Save-time validator):
        # a pivot needs three distinct steps. Reject a degenerate triple inline
        # so it is caught at the same tier as the other guards instead of only
        # failing at Save and leaving the badge INVALID (#10).
        if u == v or v == w:
            self._set_status(
                f"Pivot: degenerate triple ({u!r}, {v!r}, {w!r}) -- "
                "u==v or v==w; pick three distinct nodes."
            )
            self._panels.set_validation_badge(False)
            return
        if not self._edge_exists(u, v):
            self._set_status(
                f"Pivot: no edge between {u!r} and {v!r}; "
                "the arrive leg must be a real incident edge."
            )
            self._panels.set_validation_badge(False)
            return
        if not self._edge_exists(v, w):
            self._set_status(
                f"Pivot: no edge between {v!r} and {w!r}; "
                "the leave leg must be a real incident edge."
            )
            self._panels.set_validation_badge(False)
            return
        self._dispatch(AddPivotTurn(u, v, w))

    def _on_panel_delete_pivot(self, turn) -> None:
        """Dispatch a :class:`DeletePivotTurn` for a pivot row's Remove button.

        Coerces *turn* to a ``tuple`` before constructing the command:
        :class:`DeletePivotTurn` matches the stored turn by ``==``, and a list
        would never equal a stored tuple, so a list payload would silently fail
        to match (the command would raise "not found"). No model read is needed
        -- the row only exists because the model holds the turn; the command's
        own apply raises if it has since vanished, and ``_dispatch`` surfaces
        that to the status bar.

        :param turn: The ``(u, v, w)`` pivot to delete (any sequence; coerced).
        """
        self._dispatch(DeletePivotTurn(tuple(turn)))

    def _on_panel_global_knob_change(self, knob: str, value) -> None:
        """Dispatch a :class:`SetGlobalKnob` from a global-knob widget.

        Tier-1 guards (cheap, before dispatch): the boolean
        ``blind_decay_enabled`` knob needs no range check; every float knob is
        bounded per :data:`_FLOAT_KNOB_BOUNDS` (``blind_decay_k >= 0``,
        ``blind_decay_exponent >= 1``, ``blind_decay_d_ref > 0``,
        ``robot_radius_cm > 0``). An unknown knob name or an out-of-range float
        is rejected inline and the panel reverts; otherwise the value is
        dispatched.

        :param knob: Canonical knob name (one of the keys the panel feeds).
        :param value: New value -- ``bool`` for ``blind_decay_enabled``,
            otherwise a ``float``.
        """
        if knob == _KNOB_BLIND_DECAY_ENABLED:
            # Boolean toggle: no range to enforce.
            self._dispatch(SetGlobalKnob(knob, bool(value)))
            return

        bound = _FLOAT_KNOB_BOUNDS.get(knob)
        if bound is None:
            self._reject_inline_edit(f"Unknown global knob {knob!r}.")
            return

        try:
            number = float(value)
        except (TypeError, ValueError):
            self._reject_inline_edit(
                f"Global knob {knob!r}: {value!r} is not a number."
            )
            return

        minimum, inclusive = bound
        in_range = number >= minimum if inclusive else number > minimum
        if not in_range:
            comparator = ">=" if inclusive else ">"
            self._reject_inline_edit(
                f"Global knob {knob!r} must be {comparator} {minimum}, "
                f"got {number}."
            )
            return

        # Skip a no-op focus-out: each float knob maps to a same-named model
        # field shown at DISPLAY precision, so a bare tab-away returns the
        # rounded value. Without this guard every FocusOut dispatched a
        # SetGlobalKnob -- truncating the stored knob to 0.1 and dirtying the
        # buffer on an edit that changed nothing (#8).
        current = getattr(self._model, knob, None)
        if isinstance(current, (int, float)) and round(
            float(current), _INSPECTOR_DISPLAY_DP
        ) == round(number, _INSPECTOR_DISPLAY_DP):
            return

        self._dispatch(SetGlobalKnob(knob, number))

    def _edge_exists(self, a: str, b: str) -> bool:
        """Return ``True`` if any edge connects ``{a, b}`` (line / blind).

        Scans ``line_segments`` and ``blind_edges`` (each stored 2-tuple is
        undirected, so either ordering matches). Used by the pivot guard: each
        leg must be a real incident edge, otherwise the turn could never be
        executed.

        :param a: First endpoint node name.
        :param b: Second endpoint node name.
        :return: ``True`` iff a line or blind edge connects the pair.
        """
        pair = {a, b}
        for edge in self._model.line_segments:
            if set(edge) == pair:
                return True
        for edge in self._model.blind_edges:
            if set(edge) == pair:
                return True
        return False

    def _plain_edge_kind(self, a: str, b: str) -> Optional[str]:
        """Return the kind of the plain edge on ``{a, b}`` -- line / blind / None.

        Undirected scan of ``model.line_segments`` then ``model.blind_edges``
        (each stored 2-tuple matches either ordering). At most one plain edge
        exists on a pair (the L / B overrides enforce mutual exclusion), so the
        first match wins.

        :param a: First endpoint node name.
        :param b: Second endpoint node name.
        :return: :data:`_EDGE_KIND_LINE` / :data:`_EDGE_KIND_BLIND` for the
            existing plain edge, or ``None`` when no line / blind edge connects
            the pair.
        """
        pair = {a, b}
        for edge in self._model.line_segments:
            if set(edge) == pair:
                return _EDGE_KIND_LINE
        for edge in self._model.blind_edges:
            if set(edge) == pair:
                return _EDGE_KIND_BLIND
        return None

    def _describe_edge_on_pair(self, a: str, b: str) -> str:
        """Return a human-readable description of the edge on ``{a, b}``.

        Used by the EDGE-mode status line so the operator can see what (if
        anything) currently connects the two picked nodes before pressing
        L / B / Delete. Names the plain line / blind edge on the pair, or
        ``"none"`` when the pair has no edge.

        :param a: First picked node name.
        :param b: Second picked node name.
        :return: e.g. ``"line"``, ``"blind"``, or ``"none"``.
        """
        kind = self._plain_edge_kind(a, b)
        if kind is not None:
            return kind
        return "none"

    # ------------------------------------------------------------------
    # Auto-create-edges (T8)
    # ------------------------------------------------------------------

    def _on_panel_auto_create_edges(self) -> None:
        """Compute proposed edges + enter preview mode.

        Reads ``max_edge_length_cm`` from the panel at click time (no
        caching). On empty result -- < 3 non-collinear nodes, or every
        candidate filtered by obstacles / pre-existing pairs / length cap --
        writes a status-bar message and does NOT enter preview mode. On a
        non-empty result, stores the list on ``self._proposed_edges``,
        switches the panel to "preview" state (revealing Accept / Reject),
        and re-renders so the canvas paints the overlay.

        :return: ``None``.
        """
        max_len_cm = self._panels.max_edge_length_cm
        proposals = auto_edges.propose_edges(self._model, max_len_cm)
        if not proposals:
            self._set_status(
                "No edges to propose (need >= 3 non-collinear nodes / all candidates filtered)."
            )
            return
        line_count = sum(1 for _a, _b, k in proposals if k == "line")
        blind_count = sum(1 for _a, _b, k in proposals if k == "blind")
        self._proposed_edges = proposals
        self._panels.set_auto_create_state("preview")
        self._set_status(
            f"Proposed {len(proposals)} edges (line={line_count}, blind={blind_count}). "
            "Accept or Reject."
        )
        self._refresh_view()

    def _on_panel_accept_proposed(self) -> None:
        """Commit the proposed edges via :class:`AddEdgesBatch`.

        Clears the preview state BEFORE dispatching so the canvas refresh
        triggered by :meth:`_dispatch` does not re-paint the (now-committed)
        edges as a dashed cyan overlay. On dispatch failure, the preview is
        restored so the user can inspect the offending edges and decide
        whether to Reject or fix the model.

        :return: ``None``.
        """
        if self._proposed_edges is None:
            return
        batch = list(self._proposed_edges)
        # Clear preview FIRST so _dispatch's own _refresh_view doesn't
        # re-paint the now-committed edges as a cyan overlay.
        self._proposed_edges = None
        self._panels.set_auto_create_state("idle")
        if self._dispatch(AddEdgesBatch(batch)):
            self._set_status(f"Added {len(batch)} edges.")
        else:
            # Dispatch already wrote a status message about the conflict.
            # Restore the preview so the user can inspect / Reject.
            self._proposed_edges = batch
            self._panels.set_auto_create_state("preview")
            self._refresh_view()

    def _on_panel_reject_proposed(self) -> None:
        """Discard the preview without touching the model.

        :return: ``None``.
        """
        if self._proposed_edges is None:
            return
        self._proposed_edges = None
        self._panels.set_auto_create_state("idle")
        self._set_status("Proposal rejected.")
        self._refresh_view()

    # ------------------------------------------------------------------
    # Shortcuts dialog
    # ------------------------------------------------------------------

    def _show_shortcuts_dialog(self) -> None:
        """Open the read-only keyboard-shortcuts reference dialog (AC9).

        Wired to the status-strip key-hint click (``on_show_shortcuts``) and to
        the ``?`` / ``F1`` shortcuts. The dialog renders the canonical
        :data:`_SHORTCUT_GROUPS` table -- the single source of truth that mirrors
        the bindings installed in :meth:`_wire_keyboard_shortcuts` -- and is
        modal until dismissed. Construction is wrapped so a transient Tk failure
        (e.g. a torn-down root in a test) surfaces on the status bar rather than
        raising out of an event callback.
        """
        groups = [
            (title, list(rows)) for title, rows in _SHORTCUT_GROUPS
        ]
        try:
            ShortcutsDialog(self._root, groups)
        except tk.TclError as exc:  # pragma: no cover -- defensive
            self._set_status(f"Could not open shortcuts: {exc}")

    # ------------------------------------------------------------------
    # Canvas mouse handlers (mode-routed)
    # ------------------------------------------------------------------

    def _current_mode(self) -> Optional[Mode]:
        """Read the active mode from the panel.

        :return: A :class:`Mode` enum member or ``None`` if the variable
            holds an unrecognised string (which should not happen given
            the panel's radio buttons, but is handled defensively).
        """
        raw = self._panels.mode_var.get()
        try:
            return Mode(raw)
        except ValueError:
            return None

    def _clear_transient_state(self) -> None:
        """Reset all per-mode transient interaction state.

        Cleared:

        * MOVE-mode drag latch (``_drag_target``, ``_drag_origin_xy``).
        * OBSTACLE-mode rubber-band rectangle anchor + cursor
          (``_obstacle_anchor``, ``_obstacle_cursor``).
        * EDGE-mode two-click endpoint queue (``_edge_selection``).
        * Auto-create preview state (``_proposed_edges``) -- mode change
          kills any in-progress proposal so the Accept/Reject buttons do
          not survive into a different mode.

        Called whenever the user switches mode so a half-finished drag
        or partial edge selection does not bleed into the next mode.
        Does **not** touch model state or the dirty flag.
        """
        self._drag_target = None
        self._drag_origin_xy = None
        self._obstacle_anchor = None
        self._obstacle_cursor = None
        self._edge_selection = []
        if self._proposed_edges is not None:
            self._proposed_edges = None
            self._panels.set_auto_create_state("idle")

    def _on_mode_change(self) -> None:
        """``mode_var`` write trace: drop transient state and re-render.

        The re-render is what makes the change visible: it wipes any
        highlight ring around the previously-selected edge endpoints and
        any in-progress obstacle preview rectangle.  Also pushes the new mode
        name onto the status surface's mode line so it always names the active
        editing mode (AC8).
        """
        self._clear_transient_state()
        self._update_mode_display()
        self._refresh_view()

    def _update_mode_display(self) -> None:
        """Push a readable name for the active mode onto the status surface.

        Reads ``mode_var`` directly (rather than via :meth:`_current_mode`) so
        an unrecognised value still renders a clear ``"--"`` placeholder rather
        than vanishing.
        """
        raw = self._panels.mode_var.get()
        label = _MODE_DISPLAY_LABELS.get(raw, _MODE_DISPLAY_UNKNOWN)
        self._panels.set_mode_display(f"{_MODE_DISPLAY_PREFIX}{label}")

    def _update_selection_display(self, node_name: Optional[str]) -> None:
        """Push the current selection onto the status surface's selection line.

        :param node_name: Selected node name, or ``None`` when the selection is
            cleared (rendered as the ``"none"`` placeholder).
        """
        shown = node_name if node_name else _SELECTION_DISPLAY_NONE
        self._panels.set_selection_display(
            f"{_SELECTION_DISPLAY_PREFIX}{shown}"
        )

    def _on_canvas_click(self, x: float, y: float, button: int) -> None:
        """Route a press event to the per-mode handler.

        :param x: World x coordinate (centimetres).
        :param y: World y coordinate (centimetres).
        :param button: Matplotlib button number (1 = left, 3 = right).
        """
        if button == _MOUSE_BUTTON_RIGHT:
            self._show_context_menu(x, y)
            return

        if button != _MOUSE_BUTTON_LEFT:
            return

        mode = self._current_mode()
        if mode is Mode.ADD_NODE:
            self._handle_add_node_click(x, y)
        elif mode is Mode.MOVE:
            self._handle_move_click(x, y)
        elif mode is Mode.EDGE:
            self._handle_edge_click(x, y)
        elif mode is Mode.OBSTACLE:
            self._handle_obstacle_click(x, y)
        elif mode is Mode.START:
            self._handle_start_click(x, y)

    def _on_canvas_drag(self, x: float, y: float) -> None:
        """Route a drag event to the per-mode handler.

        Only MOVE (live node drag) and OBSTACLE (rubber-band rectangle)
        consume drags; other modes drop them silently.

        :param x: World x coordinate.
        :param y: World y coordinate.
        """
        mode = self._current_mode()
        if mode is Mode.MOVE and self._drag_target is not None:
            # Route the live drag through the canvas blit fast-path armed by
            # _handle_move_click's begin_node_drag: it animates ONLY the
            # dragged node's dot, label, and incident edges (no model write,
            # no listbox rebuild, no per-frame full render). The model is left
            # untouched until the MoveNode command on release; until then the
            # canvas merely slides the animated artists to follow the cursor.
            self._canvas.update_node_drag(self._drag_target, x, y)
        elif mode is Mode.OBSTACLE and self._obstacle_anchor is not None:
            # De-lagged rubber-band: slide the preview rectangle through the
            # canvas blit fast-path (no ax.clear, no full render, no validator,
            # no per-frame obstacle inflation) -- mirrors the MOVE drag (#11).
            self._obstacle_cursor = (x, y)
            ax_anchor, ay_anchor = self._obstacle_anchor
            self._canvas.update_obstacle_preview(ax_anchor, ay_anchor, x, y)

    def _on_canvas_release(self, x: float, y: float, button: int) -> None:
        """Route a release event to the per-mode handler.

        :param x: World x coordinate at release.
        :param y: World y coordinate at release.
        :param button: Matplotlib button number.
        """
        if button != _MOUSE_BUTTON_LEFT:
            return

        mode = self._current_mode()
        if mode is Mode.MOVE and self._drag_target is not None:
            self._finish_move(x, y)
        elif mode is Mode.OBSTACLE and self._obstacle_anchor is not None:
            self._finish_obstacle(x, y)

    # ------------------------------------------------------------------
    # Per-mode click handlers
    # ------------------------------------------------------------------

    def _handle_add_node_click(self, x: float, y: float) -> None:
        """Prompt for name + type then dispatch the right command(s).

        Opens the combined :class:`AddNodeDialog` (T3). OK creates the node at
        the clicked coordinates; Cancel is a no-op. The dialog now also offers
        ``"reset"`` -- but :class:`AddNode` refuses a reset type (a reset node
        needs a paired spec), so a reset selection is created in two steps
        instead: an :class:`AddNode` as a plain ``"line"`` node, then a
        :meth:`SetResetSpec.autofilled_auto` that flips it to ``"reset"`` with
        an auto-suggested axis + wall + facing. The new node is then selected
        and the
        Reset-spec inspector section opened so the operator can confirm /
        tweak the auto-filled geometry. (Two undo steps for a reset node is
        acceptable.) Validation of duplicate / illegal names happens downstream
        in the commands' ``apply`` -- any ``ValueError`` is surfaced by
        ``_dispatch`` in the status bar.

        :param x: World x coordinate of the click.
        :param y: World y coordinate of the click.
        """
        dlg = AddNodeDialog(self._root, initial_type=_NODE_TYPE_LINE)
        dlg.wait_window()
        if dlg.result is None:
            return
        name, node_type = dlg.result
        # Forbid reusing a name that is still the source of an unflushed rename
        # (#65). The pending-rename map keys on the original (old) name, so a
        # new node reusing that freed name would let a later rename of the new
        # node clobber the original mapping and corrupt the Save-time EDGES
        # rewrite. Block it until the pending rename is flushed by a Save.
        if name in self._pending_renames:
            messagebox.showerror(
                "Cannot add node",
                f"Cannot reuse the name {name!r} -- it is the source of a "
                "pending rename that has not been saved yet. Save first, then "
                "reuse the name.",
                parent=self._root,
            )
            return

        if node_type == _NODE_TYPE_RESET:
            self._add_reset_node(name, (x, y))
            return

        # Plain line / blind node: one AddNode dispatch.
        self._dispatch(AddNode(name, (x, y), node_type))

    def _add_reset_node(
        self, name: str, xy: tuple[float, float]
    ) -> None:
        """Create a reset node in two undoable steps and open its spec panel.

        :class:`AddNode` rejects the ``"reset"`` type, so a reset node is built
        as a plain ``"line"`` node first, then
        :meth:`SetResetSpec.autofilled_auto` flips it to ``"reset"`` with an
        auto-suggested axis / wall coordinate / facing (the NEAREST of all four
        field walls, facing away). If the first :class:`AddNode` is rejected
        (duplicate name etc.) the second dispatch is skipped so a half-formed
        reset node is never planted. On success, if the seeded wall sits under
        the pivot-clearance floor the operator is warned (the spec still stands;
        Save is the hard gate), then the new node is selected and the Reset-spec
        inspector section is opened so the operator sees and confirms the
        auto-fill.

        :param name: Name of the new reset node.
        :param xy: ``(x_cm, y_cm)`` coordinates of the click.
        """
        if not self._dispatch(AddNode(name, xy, _NODE_TYPE_LINE)):
            return
        spec = SetResetSpec.autofilled_auto(
            name,
            node_xy=xy,
            field_mm=self._model.field_mm,
        )
        if not self._dispatch(spec):
            # The autofill flip failed; the plain node is still in the model.
            # Leave it selected so the operator can fix the type by hand.
            self._select_node(name)
            return
        # SetResetSpec.apply has written the auto-derived spec; read it back and
        # warn the operator BEFORE the select / reveal below if the nearest wall
        # is under the pivot-clearance floor (the spec stands; Save is the hard
        # gate). The select / reveal calls do not overwrite the status message.
        seeded = self._model.reset_nodes[name]
        self._warn_if_reset_too_close(name, seeded["axis"], seeded["wall_coord"])
        # Select the new reset node and reveal its spec section so the
        # auto-filled wall / facing are visible for confirmation.
        self._select_node(name)
        self._panels.refresh(self._model)
        self._reveal_reset_spec_section()

    def _select_node(self, name: str) -> None:
        """Latch *name* as the selected node and sync the selection surfaces.

        Mirrors what :meth:`_on_panel_node_select` does for a user click, but
        is driven programmatically (e.g. right after a node is created): it
        updates the arrow-nudge latch and the status-strip selection line.
        :meth:`SidePanels.refresh` (run by the caller's :meth:`_refresh_view`
        or directly) reselects the listbox row and populates the inspector.

        :param name: Name of the node to select.
        """
        self._selected_node = name
        # The panel keys its listbox-restore + inspector-populate off this var.
        self._panels._selected_node_var.set(name)
        self._update_selection_display(name)

    def _reveal_reset_spec_section(self) -> None:
        """Open the inspector's collapsible Reset-spec section if it is closed.

        Lets the operator immediately see the auto-filled wall / facing after a
        reset node is created. Guarded against panel internals being absent
        (the section starts open by default, so this is usually a no-op).
        """
        collapsibles = getattr(self._panels, "_collapsibles", None)
        if not collapsibles:
            return
        record = collapsibles.get("reset_spec")
        if record is not None and not record.is_open:
            self._panels._toggle_collapsible("reset_spec")

    def _handle_move_click(self, x: float, y: float) -> None:
        """Pick a node under the cursor and latch it for dragging.

        :param x: World x coordinate.
        :param y: World y coordinate.
        """
        hit = self._canvas.find_node_at(x, y)
        if hit is None:
            self._set_status("No node at click.")
            self._drag_target = None
            self._drag_origin_xy = None
            return
        self._drag_target = hit
        self._drag_origin_xy = self._model.nodes[hit]
        # Arm the canvas blit fast-path before the first drag motion so each
        # _on_canvas_drag tick only re-composites the dragged node's animated
        # artists against a baked static background (see FieldCanvas.
        # begin_node_drag / update_node_drag). _finish_move issues the matching
        # end_node_drag on release.
        self._canvas.begin_node_drag(hit)
        self._set_status(f"Dragging {hit!r}.")

    def _handle_edge_click(self, x: float, y: float) -> None:
        """Add the clicked node to the edge-selection queue.

        Queue cap is two: a third click drops the oldest.

        :param x: World x coordinate.
        :param y: World y coordinate.
        """
        hit = self._canvas.find_node_at(x, y)
        if hit is None:
            self._set_status("No node at click.")
            return
        # Don't allow the same node to occupy both slots.
        if self._edge_selection and self._edge_selection[-1] == hit:
            return
        self._edge_selection.append(hit)
        if len(self._edge_selection) > _EDGE_SELECTION_MAX:
            del self._edge_selection[0]
        if len(self._edge_selection) == _EDGE_SELECTION_MAX:
            a, b = self._edge_selection
            current = self._describe_edge_on_pair(a, b)
            self._set_status(
                f"Edge endpoints: {a!r} -> {b!r} (current: {current}). "
                f"Press L (line), B (blind), or Delete."
            )
        else:
            self._set_status(
                f"Selected {hit!r}; click a second node to form an edge."
            )
        self._refresh_view()

    def _handle_obstacle_click(self, x: float, y: float) -> None:
        """Latch the obstacle drag anchor at the press point.

        :param x: World x coordinate.
        :param y: World y coordinate.
        """
        self._obstacle_anchor = (x, y)
        self._obstacle_cursor = (x, y)
        # Arm the blit fast-path: bake the static scene once now so each motion
        # only re-draws the rubber-band rectangle instead of a full re-render
        # (#11). begin_obstacle_drag is a no-op-safe call.
        self._canvas.begin_obstacle_drag()
        self._set_status("Drag to size the obstacle, release to commit.")

    def _handle_start_click(self, x: float, y: float) -> None:
        """Set the clicked node as ``start_node`` via :class:`SetStart`.

        :param x: World x coordinate.
        :param y: World y coordinate.
        """
        hit = self._canvas.find_node_at(x, y)
        if hit is None:
            self._set_status("No node at click.")
            return
        if hit == self._model.start_node:
            self._set_status(f"{hit!r} is already the start node.")
            return
        self._dispatch(SetStart(hit))

    # ------------------------------------------------------------------
    # Drag completion
    # ------------------------------------------------------------------

    def _finish_move(self, x: float, y: float) -> None:
        """Commit the dragged position via :class:`MoveNode`.

        With the T3 blit fast-path the model is NOT mutated during the drag:
        :meth:`FieldCanvas.update_node_drag` animates the dragged node's
        on-canvas artists while the model keeps holding the pre-drag value.
        First tear down the canvas drag state via
        :meth:`FieldCanvas.end_node_drag` (clears the animated flags, drops the
        blit background, and re-bakes the static scene -- safe even if
        ``begin_node_drag`` never armed anything), then dispatch a single
        :class:`MoveNode` so the move lands as one fully-reversible undo entry.
        ``_dispatch`` runs the one full :meth:`_refresh_view` that re-syncs the
        inspector / listbox to the final position.

        :param x: World x coordinate at release (the final position).
        :param y: World y coordinate at release.
        """
        target = self._drag_target
        origin = self._drag_origin_xy
        # Clear state up front so any early return below leaves the app
        # ready for the next interaction.
        self._drag_target = None
        self._drag_origin_xy = None

        # Always tear down the canvas drag fast-path on a MOVE release. It
        # no-ops when no drag was armed and re-bakes the static scene either
        # way, so it is safe to call before the early returns below.
        self._canvas.end_node_drag()

        if target is None or origin is None:
            return

        # The model was never live-mutated (the canvas animated artists
        # instead), so it still holds ``origin``; this assignment is a
        # defensive no-op kept only to guarantee the pre-drag value before
        # MoveNode.apply captures it as ``_original_xy``.
        self._model.nodes[target] = origin

        # No-op shortcut: if the drag ended where it started, there's no
        # mutation to record. end_node_drag already re-rendered the canvas;
        # refresh once more so the panels / inspector stay consistent.
        if (x, y) == origin:
            self._refresh_view()
            self._set_status(f"{target!r} unchanged.")
            return

        self._dispatch(MoveNode(target, (x, y)))

    def _finish_obstacle(self, x: float, y: float) -> None:
        """Prompt for ``kind`` and dispatch :class:`AddObstacle`.

        :param x: World x coordinate at release.
        :param y: World y coordinate at release.
        """
        anchor = self._obstacle_anchor
        # Always clear the drag state up front.
        self._obstacle_anchor = None
        self._obstacle_cursor = None
        # Tear down the blit fast-path (remove the preview rect, re-bake the
        # static scene). No-op-safe even if begin_obstacle_drag never armed (#11).
        self._canvas.end_obstacle_drag()
        if anchor is None:
            return

        ax, ay = anchor
        xmin = min(ax, x)
        ymin = min(ay, y)
        xmax = max(ax, x)
        ymax = max(ay, y)

        # Drop trivial drags so a stray click doesn't open the dialog.
        if (xmax - xmin) < _OBSTACLE_MIN_SIDE_CM or (
            ymax - ymin
        ) < _OBSTACLE_MIN_SIDE_CM:
            self._set_status("Obstacle drag too small; ignored.")
            self._refresh_view()
            return

        kind = simpledialog.askstring(
            "Obstacle kind",
            "Tag (e.g. wall, pillar, box):",
            initialvalue=_DEFAULT_OBSTACLE_KIND,
            parent=self._root,
        )
        if kind is None:
            self._refresh_view()
            return
        kind = kind.strip() or _DEFAULT_OBSTACLE_KIND

        self._dispatch(AddObstacle(xmin, ymin, xmax, ymax, kind))

    # ------------------------------------------------------------------
    # Keyboard shortcut handlers
    # ------------------------------------------------------------------

    def _handle_delete_key(self) -> None:
        """``Delete`` -- if EDGE mode + two endpoints selected, drop the edge.

        Deletes the undirected line / blind edge on ``{a, b}`` via
        :class:`DeleteEdge`. If the pair has no edge, a status message is
        shown and nothing is dispatched.

        In other modes the key is a no-op (deleting nodes is intentionally
        constrained to the right-click context menu so a stray Delete does
        not silently destroy hand-authored data).
        """
        mode = self._current_mode()
        if mode is not Mode.EDGE:
            return
        if len(self._edge_selection) != _EDGE_SELECTION_MAX:
            self._set_status(
                "EDGE mode: select two nodes before pressing Delete."
            )
            return
        a, b = self._edge_selection
        if self._plain_edge_kind(a, b) is not None:
            if self._dispatch(DeleteEdge(a, b)):
                self._edge_selection.clear()
                self._set_status(f"Deleted edge ({a!r}, {b!r}).")
                self._refresh_view()
            return
        self._set_status(f"No edge between ({a!r}, {b!r}) to delete.")

    def _handle_edge_line_key(self) -> None:
        """``L`` -- add a line edge, overriding any blind edge on the pair.

        Overrides whatever currently connects the two picked nodes: an existing
        BLIND edge is deleted first, then the undirected line edge is added in
        its place (a two-entry delete-then-add). A pair that is already a line
        edge is a friendly no-op. With nothing there a plain
        :class:`AddLineEdge` is dispatched. See
        :meth:`_add_plain_edge_with_override` for the full override contract.
        """
        if self._current_mode() is not Mode.EDGE:
            return
        if len(self._edge_selection) != _EDGE_SELECTION_MAX:
            self._set_status("EDGE mode: pick two nodes first.")
            return
        a, b = self._edge_selection
        self._add_plain_edge_with_override(
            AddLineEdge(a, b), a, b, _EDGE_KIND_LINE
        )

    def _handle_edge_blind_key(self) -> None:
        """``B`` -- add a blind edge, overriding any line edge on the pair.

        Symmetric to :meth:`_handle_edge_line_key`: an existing LINE edge on the
        picked pair is deleted first, then a plain :class:`AddBlindEdge` is added
        in its place. A pair that is already a blind edge is a friendly no-op.
        """
        if self._current_mode() is not Mode.EDGE:
            return
        if len(self._edge_selection) != _EDGE_SELECTION_MAX:
            self._set_status("EDGE mode: pick two nodes first.")
            return
        a, b = self._edge_selection
        self._add_plain_edge_with_override(
            AddBlindEdge(a, b), a, b, _EDGE_KIND_BLIND
        )

    def _add_plain_edge_with_override(
        self, add_command, a: str, b: str, kind: str
    ) -> None:
        """Add a line / blind edge, overriding the other plain kind on the pair.

        Shared body for the ``L`` and ``B`` handlers. The new undirected line /
        blind edge replaces whatever currently connects ``{a, b}``:

        * Same plain kind already there -> nothing to change; a friendly status
          message, no dispatch (avoids the command's "already exists" error).
        * The OTHER plain kind (line <-> blind) -> deleted FIRST via
          :class:`DeleteEdge`, then the add (override line <-> blind), so the
          operator can flip an edge's type in place without a manual delete.
        * Nothing there -> the add runs directly.

        Any override is delete-then-add, bundled into ONE :class:`MacroCommand`
        so a single undo restores the WHOLE override instead of just the add
        (which would leave the pair with neither edge). If the delete step is
        rejected, ``MacroCommand.apply`` rolls back so the model is never left
        half-overridden, and ``_dispatch`` surfaces the error (#12).

        :param add_command: The :class:`AddLineEdge` / :class:`AddBlindEdge` to
            dispatch once any override is cleared.
        :param a: First picked node name.
        :param b: Second picked node name.
        :param kind: :data:`_EDGE_KIND_LINE` / :data:`_EDGE_KIND_BLIND` -- shapes
            the override decision and the status text.
        """
        # Already the requested kind: a no-op the operator would otherwise hit as
        # the command's "already exists" error. Report it plainly and stop.
        if self._plain_edge_kind(a, b) == kind:
            self._edge_selection.clear()
            self._set_status(f"({a!r}, {b!r}) is already a {kind} edge.")
            self._refresh_view()
            return
        # The same-kind case returned above, so any plain edge on the pair is the
        # OTHER kind: delete it first so the add won't collide, recording the old
        # kind for the status text.
        overridden_kind = self._plain_edge_kind(a, b)
        steps: list = []
        if overridden_kind is not None:
            steps.append(DeleteEdge(a, b))
        steps.append(add_command)
        command = MacroCommand(steps) if len(steps) > 1 else steps[0]
        if self._dispatch(command):
            self._edge_selection.clear()
            if overridden_kind is not None:
                self._set_status(
                    f"Changed ({a!r}, {b!r}) from {overridden_kind} to {kind}."
                )
            else:
                self._set_status(f"Added {kind} edge ({a!r}, {b!r}).")
            self._refresh_view()

    # ------------------------------------------------------------------
    # Right-click context menu
    # ------------------------------------------------------------------

    def _show_context_menu(self, x: float, y: float) -> None:
        """Route a right-click to either the node or the obstacle context menu.

        Nodes are visually on top of obstacles, so a click that lands on a
        node always shows the node menu. Falling back to obstacle hit-testing
        only when no node is under the cursor preserves the existing
        node-rename / set-type / delete flow.

        :param x: World x coordinate of the click.
        :param y: World y coordinate of the click.
        """
        node_target = self._canvas.find_node_at(x, y)
        if node_target is not None:
            self._show_node_context_menu(node_target)
            return

        obstacle_index = self._canvas.find_obstacle_at(x, y)
        if obstacle_index is not None:
            self._show_obstacle_context_menu(obstacle_index)
            return

        self._set_status("Right-click: no node or obstacle at click point.")

    def _show_node_context_menu(self, target: str) -> None:
        """Pop up a Rename / Set-type / Delete menu over a clicked node.

        :param target: The node name under the cursor.
        """
        menu = tk.Menu(self._root, tearoff=False)
        menu.add_command(
            label=f"Rename {target!r} ...",
            command=lambda: self._context_rename(target),
        )

        type_submenu = tk.Menu(menu, tearoff=False)
        type_submenu.add_command(
            label="line",
            command=lambda: self._context_set_type(target, _NODE_TYPE_LINE),
        )
        type_submenu.add_command(
            label="blind",
            command=lambda: self._context_set_type(target, _NODE_TYPE_BLIND),
        )
        menu.add_cascade(label="Set type", menu=type_submenu)

        menu.add_separator()
        menu.add_command(
            label=f"Delete {target!r}",
            command=lambda: self._context_delete(target),
        )

        # tk_popup needs absolute screen coordinates; pull from the
        # pointer. ``winfo_pointerxy`` is the simplest cross-platform
        # source.
        try:
            screen_x = self._root.winfo_pointerx()
            screen_y = self._root.winfo_pointery()
            menu.tk_popup(screen_x, screen_y)
        finally:
            menu.grab_release()

    def _show_obstacle_context_menu(self, obstacle_index: int) -> None:
        """Pop up a Delete menu over a right-clicked obstacle.

        Mirrors :meth:`_show_node_context_menu` but for obstacles, which
        have no rename / type-change verbs -- delete is the only mutation
        available from the context menu. The menu shows the obstacle's
        ``kind`` tag so the operator can sanity-check which rectangle is
        about to disappear when several obstacles overlap.

        :param obstacle_index: Index into :attr:`FieldModel.obstacles` of
            the obstacle under the cursor (returned by
            :meth:`FieldCanvas.find_obstacle_at`).
        """
        obstacles = self._model.obstacles
        if obstacle_index < 0 or obstacle_index >= len(obstacles):
            return
        kind = obstacles[obstacle_index][4]

        menu = tk.Menu(self._root, tearoff=False)
        menu.add_command(
            label=f"Delete obstacle [{obstacle_index}] ({kind!r})",
            command=lambda: self._context_delete_obstacle(obstacle_index),
        )

        try:
            screen_x = self._root.winfo_pointerx()
            screen_y = self._root.winfo_pointery()
            menu.tk_popup(screen_x, screen_y)
        finally:
            menu.grab_release()

    def _context_delete_obstacle(self, obstacle_index: int) -> None:
        """Confirm + dispatch :class:`DeleteObstacle` for *obstacle_index*.

        Validates the index against the live ``model.obstacles`` list so a
        stale menu (e.g. obstacle deleted by undo/redo between the right
        click and the menu selection) shows a status-bar warning instead
        of raising ``IndexError`` out of the command.

        :param obstacle_index: Index into :attr:`FieldModel.obstacles`.
        """
        obstacles = self._model.obstacles
        if obstacle_index < 0 or obstacle_index >= len(obstacles):
            self._set_status(
                f"Obstacle index {obstacle_index} is no longer valid; "
                "model changed since the menu opened."
            )
            return
        xmin, ymin, xmax, ymax, kind = obstacles[obstacle_index]

        confirm = messagebox.askyesno(
            "Confirm delete",
            f"Delete obstacle [{obstacle_index}] {kind!r} at "
            f"({xmin:.1f}, {ymin:.1f}) -> ({xmax:.1f}, {ymax:.1f})?",
            parent=self._root,
        )
        if not confirm:
            return

        if self._dispatch(DeleteObstacle(obstacle_index)):
            self._set_status(
                f"Deleted obstacle [{obstacle_index}] ({kind!r})."
            )

    def _context_rename(self, target: str) -> None:
        """Prompt for a new name and rename *target* via :meth:`_perform_rename`.

        :param target: Existing node name.
        """
        new_name = simpledialog.askstring(
            "Rename node",
            f"New name for {target!r}:",
            initialvalue=target,
            parent=self._root,
        )
        if new_name is None:
            return
        self._perform_rename(target, new_name)

    def _perform_rename(self, target: str, new_name: str) -> bool:
        """Rename *target* to *new_name*, recording the pending EDGES rewrite.

        Shared by the right-click rename and the inspector's in-place name edit.
        A blank or unchanged *new_name* is a no-op; on a successful dispatch the
        rename is queued into :attr:`_pending_renames` for the next Save and --
        if *target* was the selected node -- the selection follows it to
        *new_name* (latch + status line + inspector), so the renamed node stays
        selected and a follow-up arrow nudge targets it.

        :param target: Existing node name.
        :param new_name: Proposed new name (stripped here).
        :return: ``True`` if the rename was dispatched, ``False`` otherwise.
        """
        new_name = new_name.strip()
        if not new_name or new_name == target:
            return False
        was_selected = self._selected_node == target
        if not self._dispatch(RenameNode(target, new_name)):
            return False
        self._record_pending_rename(target, new_name)
        if was_selected:
            # The dispatch's refresh ran while the panel still keyed off the OLD
            # name (so it dropped the listbox/inspector selection). Re-point the
            # canonical selection var + status line at the renamed node.
            self._select_node(new_name)
        # Refresh once more: if the selection moved, re-select the renamed node
        # and repopulate the inspector. The dispatch's earlier refresh painted
        # the stale names.
        self._refresh_view()
        return True

    def _context_set_type(self, target: str, new_type: str) -> None:
        """Dispatch a node-type change from the right-click submenu.

        The submenu only ever offers the plain ``"line"`` / ``"blind"`` types
        (reset is created / edited via the inspector). But the target may
        CURRENTLY be a reset node, and :class:`SetNodeType` refuses to retype a
        reset node, so this routes a reset -> line/blind change through
        :class:`ClearResetSpec` (which removes the spec and retypes) instead.

        :param target: Node whose type is being changed.
        :param new_type: ``"line"`` or ``"blind"`` (the only submenu options).
        """
        current_type = self._model.node_types.get(target)
        if current_type == new_type:
            self._set_status(f"{target!r} is already {new_type!r}.")
            return
        if current_type == _NODE_TYPE_RESET:
            # SetNodeType would refuse a currently-reset node; clear the spec
            # and retype atomically instead.
            self._dispatch(ClearResetSpec(target, new_type))
            return
        self._dispatch(SetNodeType(target, new_type))

    def _context_delete(self, target: str) -> None:
        """Confirm + dispatch :class:`DeleteNode`, checking EDGES first.

        Refuses the delete if the imported ``field_data.EDGES`` references
        the node (the model doesn't own ``EDGES``; we peek at the module's
        live attribute). The user is told the offending entry index.

        :param target: Node to delete.
        """
        # Guard the start node up front so the operator gets a clear message
        # instead of an opaque "START_NODE references unknown node" failure at
        # the next Save (DeleteNode.apply enforces the same rule, but this
        # surfaces it before the confirm dialog).
        if target == self._model.start_node:
            messagebox.showerror(
                "Cannot delete node",
                f"Cannot delete {target!r} -- it is the current start node. "
                "Pick a different start node first.",
                parent=self._root,
            )
            return

        edges_index = self._find_edges_reference(target)
        if edges_index is not None:
            messagebox.showerror(
                "Cannot delete node",
                f"Cannot delete {target!r} -- referenced by "
                f"EDGES[{edges_index}]. Edit field_data.py first.",
                parent=self._root,
            )
            return

        confirm = messagebox.askyesno(
            "Confirm delete",
            f"Delete node {target!r}?\n"
            "This will also fail if line_segments or blind_edges still "
            "reference it -- delete those first if needed.",
            parent=self._root,
        )
        if not confirm:
            return

        if self._dispatch(DeleteNode(target)) and self._selected_node == target:
            # The selected node is gone; drop the stale latch (AC4) so an arrow
            # nudge no longer targets it, and clear the selection line.
            self._selected_node = None
            self._update_selection_display(None)

    def _find_edges_reference(self, node_name: str) -> Optional[int]:
        """Return the index of the first ``EDGES`` entry referencing *node_name*.

        Scans the live ``tools.path_planner.field_data.EDGES`` list (if the
        module was importable). Returns ``None`` when the node is unmentioned
        or when the module / attribute is unavailable.

        The cached ``EDGES`` list is only refreshed on a successful Save, so it
        still holds the *pre-rename* names for any in-session rename that has
        not been flushed yet. Resolve *node_name* back through the pending
        rename map (target ``Y`` -> source ``X``) before scanning, so deleting
        a renamed-but-unsaved node still sees the EDGES reference under its old
        name and the guard fires.

        :param node_name: Node to search for (its current, possibly renamed,
            display name).
        :return: Index into ``EDGES`` or ``None``.
        """
        if self._field_data_module is None:
            return None
        edges = getattr(self._field_data_module, "EDGES", None)
        if not isinstance(edges, list):
            return None
        # Map the current display name back to the name the cached EDGES table
        # still uses. A pending rename X -> Y means EDGES holds 'X'; deleting
        # 'Y' must search for 'X'. The map keys on the original source name and
        # values are the latest target, so invert it for the lookup.
        search_name = node_name
        for source, target in self._pending_renames.items():
            if target == node_name:
                search_name = source
                break
        for index, entry in enumerate(edges):
            if not isinstance(entry, dict):
                continue
            if entry.get("u") == search_name or entry.get("v") == search_name:
                return index
        return None

    def _record_pending_rename(self, old: str, new: str) -> None:
        """Accumulate a rename for the next Save, collapsing chains.

        If a prior rename mapped ``X -> Y`` and the user now renames
        ``Y -> Z``, the resulting pending entry must be ``X -> Z`` (and
        the intermediate ``X -> Y`` mapping is dropped) so the LibCST
        rewriter only sees the latest target name for each original.

        Degenerate chains that collapse to identity (``X -> Y -> X``)
        are dropped entirely rather than recorded as ``{X: X}`` — the
        latter would force :func:`tools.path_planner.editor.ast_rewrite.
        rewrite_edges_uv` to rewrite the EDGES tree to a byte-identical
        result and burn mtime for no reason, violating its
        no-op-preserves-mtime contract.

        :param old: Name as it was before this rename.
        :param new: New name.
        """
        # Find any prior rename whose target was ``old`` and forward it.
        forwarded_source: Optional[str] = None
        for src, dst in list(self._pending_renames.items()):
            if dst == old:
                forwarded_source = src
                break
        if forwarded_source is not None:
            # Chain collapse: the original source was already in the queue.
            if forwarded_source == new:
                # X -> Y followed by Y -> X collapses to identity; drop the
                # entry entirely so Save doesn't trigger a no-op rewrite.
                del self._pending_renames[forwarded_source]
            else:
                self._pending_renames[forwarded_source] = new
        else:
            # Fresh rename. Don't insert an identity entry in the first
            # place (the panel layer already short-circuits ``old == new``,
            # but be defensive in case a future caller bypasses it).
            if old != new:
                self._pending_renames[old] = new

    # ------------------------------------------------------------------
    # Save / Save & Build
    # ------------------------------------------------------------------

    def _save(self) -> bool:
        """Validate, write JSON, then flush pending EDGES renames.

        :return: ``True`` on success, ``False`` on validation or I/O failure
            (the user is informed via a modal dialog in that case).
        """
        # 1. Validate first so we never write a half-broken JSON file. This is
        # the authoritative tier-2 gate -- the full geometry validator -- and
        # the only place the validation badge is driven from a real validate()
        # call (the per-edit inline guards only ever drop it to INVALID).
        try:
            self._model.validate()
        except Exception as exc:  # noqa: BLE001 -- show any validator error
            self._panels.set_validation_badge(False)
            messagebox.showerror(
                "Validation failed",
                f"Cannot save: {exc}",
                parent=self._root,
            )
            self._set_status("Save aborted: validation failed.")
            return False

        # 2. Write the JSON sidecar.
        try:
            loader.dump_field_json(self._model.to_json_dict(), self._json_path)
        except OSError as exc:
            messagebox.showerror(
                "Write failed",
                f"Could not write {self._json_path}: {exc}",
                parent=self._root,
            )
            self._set_status("Save aborted: JSON write failed.")
            return False

        # 3. Flush pending node renames into field_data.py's EDGES table.
        if self._pending_renames:
            try:
                from tools.path_planner.editor import ast_rewrite  # noqa: PLC0415
                ast_rewrite.rewrite_edges_uv(
                    _FIELD_DATA_PY_PATH, dict(self._pending_renames)
                )
            except Exception as exc:  # noqa: BLE001 -- LibCST errors etc.
                messagebox.showerror(
                    "EDGES rewrite failed",
                    f"field_data.json was written, but rewriting EDGES in "
                    f"{_FIELD_DATA_PY_PATH} failed: {exc}",
                    parent=self._root,
                )
                self._set_status(
                    "Save partial: JSON saved, EDGES rewrite failed."
                )
                # Keep the pending renames so the user can retry. JSON was
                # written, but the rewrite did not happen.
                return False
            # Refresh the cached field_data module so _find_edges_reference
            # sees the rewritten u/v values on the next delete attempt
            # (Codex PR29 #14). Without this, the in-memory EDGES list keeps
            # the pre-rewrite names and a later delete of the new name would
            # silently succeed even though field_data.py still references it.
            if self._field_data_module is not None:
                import importlib  # noqa: PLC0415
                try:
                    self._field_data_module = importlib.reload(
                        self._field_data_module
                    )
                except Exception:  # noqa: BLE001 -- best-effort refresh
                    # If the reload fails (e.g. the rewriter produced a file
                    # the import machinery rejects), keep the stale reference
                    # rather than wiping it. JSON has already been saved and
                    # the rewrite already touched disk; the worst case is the
                    # delete-guard staying as accurate as it was pre-save.
                    pass
            self._pending_renames.clear()

        # After a successful Save, field_data.json, field_data.py EDGES, and
        # the model are all consistent, so the pending-rename delta is empty
        # (cleared above when a flush ran; already empty otherwise). It now
        # tracks only renames made AFTER this Save -- no history-derived anchor
        # is needed, which is what makes undo-after-save robust.

        # The model passed the full validator and reached disk: flip the badge
        # to VALID. This is the only success path that runs the real validator,
        # so it is the only place the badge is set TRUE.
        self._panels.set_validation_badge(True)
        self._clear_dirty()
        self._set_status(_STATUS_SAVED)
        return True

    def _save_and_build(self) -> None:
        """Run :meth:`_save`, then invoke ``build_routes._run`` in-process.

        On non-zero exit, captures stdout/stderr and shows them in a
        scrollable dialog so the user can diagnose without leaving the
        editor.
        """
        if not self._save():
            return

        from tools.path_planner import build_routes  # noqa: PLC0415

        stdout_buf = io.StringIO()
        stderr_buf = io.StringIO()
        try:
            with redirect_stdout(stdout_buf), redirect_stderr(stderr_buf):
                exit_code = build_routes._run(no_viz=False)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror(
                "Build failed",
                f"build_routes._run raised {type(exc).__name__}: {exc}\n\n"
                f"--- stdout ---\n{stdout_buf.getvalue()}\n"
                f"--- stderr ---\n{stderr_buf.getvalue()}",
                parent=self._root,
            )
            self._set_status(_STATUS_BUILD_FAILED)
            return

        if exit_code == 0:
            self._set_status(_STATUS_BUILT)
        else:
            messagebox.showerror(
                f"Build failed (exit {exit_code})",
                f"--- stdout ---\n{stdout_buf.getvalue()}\n"
                f"--- stderr ---\n{stderr_buf.getvalue()}",
                parent=self._root,
            )
            self._set_status(_STATUS_BUILD_FAILED)

    # ------------------------------------------------------------------
    # Window-close protocol
    # ------------------------------------------------------------------

    def _on_close_request(self) -> None:
        """Handle the ``WM_DELETE_WINDOW`` event with a dirty-state prompt.

        Behaviour:

        * Clean buffer: close immediately.
        * Dirty buffer: ask "Save before closing? Yes / No / Cancel".
          *Cancel* aborts the close. *Yes* runs :meth:`_save` and only
          closes if the save succeeded. *No* discards changes and closes.
        """
        if not self._dirty:
            self._destroy_root()
            return

        choice = messagebox.askyesnocancel(
            "Unsaved changes",
            "There are unsaved changes. Save before closing?",
            parent=self._root,
        )
        if choice is None:
            # Cancel -- abort the close.
            return
        if choice:
            if not self._save():
                # Save failed; don't close so the user can fix it.
                return
        self._destroy_root()

    def _destroy_root(self) -> None:
        """Tear down the Tk root window if we created it."""
        try:
            self._root.destroy()
        except tk.TclError:
            # Already destroyed -- nothing to do.
            pass
