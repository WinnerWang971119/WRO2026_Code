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

from tools.path_planner.editor.dialogs import AddNodeDialog
from typing import Optional

from tools.path_planner.editor import auto_edges, loader
from tools.path_planner.editor.canvas import FieldCanvas
from tools.path_planner.editor.commands import (
    AddBlindEdge,
    AddEdgesBatch,
    AddLineEdge,
    AddNode,
    AddObstacle,
    DeleteEdge,
    DeleteNode,
    DeleteObstacle,
    MoveNode,
    RenameNode,
    SetNodeType,
    SetStart,
    SetStartHeading,
)
from tools.path_planner.editor.history import History
from tools.path_planner.editor.model import FieldModel
from tools.path_planner.editor.panels import Mode, SidePanels


# ---------------------------------------------------------------------------
# Constants -- no magic numbers in the code body.
# ---------------------------------------------------------------------------

#: Default window title (without dirty marker).
_WINDOW_TITLE: str = "WRO 2026 Field Editor"

#: Suffix appended to the window title when the model has unsaved changes.
_DIRTY_TITLE_SUFFIX: str = " *"

#: Initial canvas area width in pixels. The canvas height is derived from the
#: loaded field's aspect ratio so the map renders without huge empty bars
#: above and below it. The right-hand side panel (``_PANEL_WIDTH_PX``) and a
#: small chrome margin are added on top of these to get the window geometry.
_DEFAULT_CANVAS_WIDTH_PX: int = 900

#: Width of the right-hand side panel column. Mirrors
#: ``tools.path_planner.editor.panels._PANEL_WIDTH_PX``. Duplicated here
#: rather than imported because the panels module pins its own column width
#: with ``pack_propagate(0)`` -- this constant only feeds the initial window
#: sizing math.
_SIDE_PANEL_WIDTH_PX: int = 240

#: Extra vertical pixels reserved for the matplotlib figure's axis labels,
#: title bar, and status row so the field rectangle itself fits inside the
#: canvas area without being clipped.
_CANVAS_CHROME_PX: int = 80

#: Hard floor on the initial window dimensions so a degenerately tall or thin
#: field still produces a usable starting window.
_MIN_WINDOW_WIDTH_PX: int = 800
_MIN_WINDOW_HEIGHT_PX: int = 400

#: Valid node-type strings as displayed in dialogs and combobox submenus.
_NODE_TYPE_LINE: str = "line"
_NODE_TYPE_BLIND: str = "blind"
_VALID_NODE_TYPES: tuple[str, ...] = (_NODE_TYPE_LINE, _NODE_TYPE_BLIND)

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

#: Matplotlib zorder for the transient obstacle-drag preview rectangle —
#: above the normal obstacle layer but below the nodes.
_OBSTACLE_PREVIEW_ZORDER: int = 5
_OBSTACLE_PREVIEW_EDGECOLOR: str = "blue"
_OBSTACLE_PREVIEW_LINESTYLE: str = ":"
_OBSTACLE_PREVIEW_LINEWIDTH: float = 1.5

#: Edge-selection cap. Two clicks = a candidate pair. A third click drops
#: the oldest entry so the most recent two clicks always win.
_EDGE_SELECTION_MAX: int = 2

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


class EditorApp:
    """Top-level Tk application class for the live field editor.

    Constructor side effects:

    * Loads ``field_data.json`` via the loader; raises if missing.
    * Constructs the in-memory :class:`FieldModel`.
    * Creates (or adopts) a :class:`tkinter.Tk` root window.
    * Builds the :class:`FieldCanvas` and :class:`SidePanels`, packs them
      into a horizontal layout (canvas on the left, panels on the right).
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
        self._pending_renames: dict[str, str] = {}

        # ---- 4. Tk root window ----
        self._owns_root: bool = root is None
        self._root: tk.Tk = root if root is not None else tk.Tk()
        self._root.title(_WINDOW_TITLE)
        self._root.geometry(self._initial_window_geometry())

        # ---- 5. Side panels (right column, fixed width) ----
        self._panels: SidePanels = SidePanels(self._root)
        self._panels.pack(side="right", fill="y")

        # ---- 6. Canvas (left, expands) ----
        # The canvas resolves ``model.map_png`` against the planner root
        # itself; it does not need a field_data module reference.
        self._canvas: FieldCanvas = FieldCanvas(self._root)
        self._size_canvas_to_field()
        self._canvas.tk_widget.pack(side="left", fill="both", expand=True)

        # ---- 7. Mouse / mode state ----
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

        # ---- 8. Spinbox de-dup guard (reviewer note T11 W1) ----
        # SidePanels' Spinbox <FocusOut> may spuriously fire with the
        # same value already committed. Compare against this latch and
        # short-circuit if equal.
        self._last_committed_heading: float = float(
            self._model.start_heading_deg
        )

        # ---- 9. Wire callbacks ----
        self._wire_panel_callbacks()
        self._wire_canvas_callbacks()
        self._wire_keyboard_shortcuts()
        self._wire_window_close()

        # ---- 10. Initial render + panel population ----
        self._refresh_view()

    # ------------------------------------------------------------------
    # Initial sizing (canvas + window match the field's aspect ratio)
    # ------------------------------------------------------------------

    def _field_aspect_ratio(self) -> float:
        """Return ``field_mm[0] / field_mm[1]`` with a safe fallback.

        Used by :meth:`_initial_window_geometry` and
        :meth:`_size_canvas_to_field` to pick a starting size that mirrors
        the field's real-world proportions. A non-positive height falls
        back to ``1.0`` (square) so a degenerate model still yields a
        usable window.
        """
        width_mm, height_mm = self._model.field_mm
        if height_mm <= 0.0:
            return 1.0
        return float(width_mm) / float(height_mm)

    def _initial_window_geometry(self) -> str:
        """Compute a ``"WxH"`` geometry string sized to the field aspect.

        The canvas area is ``_DEFAULT_CANVAS_WIDTH_PX`` wide; its height is
        derived from the field aspect so the map fills the canvas without
        large empty bars. The window adds the side panel width and a small
        vertical chrome margin for matplotlib's axis labels.
        """
        aspect = self._field_aspect_ratio()
        canvas_w = _DEFAULT_CANVAS_WIDTH_PX
        canvas_h = max(int(round(canvas_w / aspect)), _MIN_WINDOW_HEIGHT_PX)
        window_w = max(canvas_w + _SIDE_PANEL_WIDTH_PX, _MIN_WINDOW_WIDTH_PX)
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
        """
        self._root.bind_all("<Control-z>", lambda _e: self._undo())
        self._root.bind_all("<Control-Z>", lambda _e: self._undo())
        self._root.bind_all("<Control-y>", lambda _e: self._redo())
        self._root.bind_all("<Control-Y>", lambda _e: self._redo())
        self._root.bind_all("<Control-Shift-Z>", lambda _e: self._redo())
        self._root.bind_all("<Control-Shift-z>", lambda _e: self._redo())
        self._root.bind_all("<Control-s>", lambda _e: self._save())
        self._root.bind_all("<Control-S>", lambda _e: self._save())
        self._root.bind_all("<Delete>", lambda _e: self._handle_delete_key())
        self._root.bind_all("<KeyPress-l>", lambda _e: self._handle_edge_line_key())
        self._root.bind_all("<KeyPress-L>", lambda _e: self._handle_edge_line_key())
        self._root.bind_all("<KeyPress-b>", lambda _e: self._handle_edge_blind_key())
        self._root.bind_all("<KeyPress-B>", lambda _e: self._handle_edge_blind_key())

    def _wire_window_close(self) -> None:
        """Intercept the window-manager close ("X") so we can warn on dirty."""
        self._root.protocol("WM_DELETE_WINDOW", self._on_close_request)

    # ------------------------------------------------------------------
    # View refresh + dirty bookkeeping
    # ------------------------------------------------------------------

    def _refresh_view(self) -> None:
        """Re-render canvas + side panels + button enable states + title."""
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

    def _update_title(self) -> None:
        """Reflect :attr:`_dirty` in the window title bar."""
        suffix = _DIRTY_TITLE_SUFFIX if self._dirty else ""
        self._root.title(f"{_WINDOW_TITLE}{suffix}")

    def _mark_dirty(self) -> None:
        """Set the dirty flag and update the title."""
        self._dirty = True
        self._update_title()

    def _clear_dirty(self) -> None:
        """Clear the dirty flag and update the title."""
        self._dirty = False
        self._update_title()

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
        if not self._history.undo(self._model):
            self._set_status("Nothing to undo.")
            return
        # Any undo brings the buffer back into "dirty vs saved" ambiguity --
        # we conservatively mark dirty so the user is prompted on close.
        self._mark_dirty()
        self._refresh_view()
        self._set_status("Undid last action.")

    def _redo(self) -> None:
        """Re-apply one command from the redo stack and re-render."""
        if not self._history.redo(self._model):
            self._set_status("Nothing to redo.")
            return
        self._mark_dirty()
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
        if node_name is None:
            self._set_status("Selection cleared.")
        else:
            self._set_status(f"Selected node {node_name!r}.")
        self._refresh_view()

    def _on_panel_node_type_change(self, node_name: str, new_type: str) -> None:
        """Dispatch a :class:`SetNodeType` command when the combobox changes.

        :param node_name: Name of the node whose type was edited.
        :param new_type: New type string (already constrained to
            ``"line"``/``"blind"`` by the panel).
        """
        if new_type not in _VALID_NODE_TYPES:
            self._set_status(f"Invalid node type {new_type!r}.")
            return
        # Skip the round-trip if the type is unchanged -- the panel's
        # spurious-event protection mostly handles this, but a belt-and-
        # braces check costs nothing and avoids a useless undo entry.
        if self._model.node_types.get(node_name) == new_type:
            return
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
        if float(cx) == x_cm and float(cy) == y_cm:
            return
        self._dispatch(MoveNode(node_name, (x_cm, y_cm)))

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
        if the value equals the last successfully committed heading, the
        change is ignored. The latch is updated only after the command
        actually applies.

        :param heading_deg: New heading in degrees, ``[0, 360)``.
        """
        if heading_deg == self._last_committed_heading:
            return
        if self._dispatch(SetStartHeading(heading_deg)):
            self._last_committed_heading = heading_deg

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
        any in-progress obstacle preview rectangle.
        """
        self._clear_transient_state()
        self._refresh_view()

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
            # Live LIVE-update the model so the canvas can re-render the
            # node + its incident edges following the cursor. The
            # eventual MoveNode command (issued on release) will undo
            # this temporary mutation and re-apply the final delta as a
            # single undo entry.
            self._model.nodes[self._drag_target] = (x, y)
            self._refresh_view()
        elif mode is Mode.OBSTACLE and self._obstacle_anchor is not None:
            self._obstacle_cursor = (x, y)
            self._refresh_view()
            self._draw_obstacle_preview()

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
        """Prompt for name + type then dispatch :class:`AddNode`.

        Opens the combined :class:`AddNodeDialog` (T3). OK dispatches
        :class:`AddNode` at the clicked coordinates; Cancel is a no-op.
        Validation of duplicate / illegal names happens downstream in
        ``AddNode.apply`` -- any ``ValueError`` is surfaced by ``_dispatch``
        in the status bar.

        :param x: World x coordinate of the click.
        :param y: World y coordinate of the click.
        """
        dlg = AddNodeDialog(self._root, initial_type=_NODE_TYPE_LINE)
        dlg.wait_window()
        if dlg.result is None:
            return
        name, node_type = dlg.result
        self._dispatch(AddNode(name, (x, y), node_type))

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
            self._set_status(
                f"Edge endpoints: {self._edge_selection[0]!r} -> "
                f"{self._edge_selection[1]!r}. Press L (line) or B (blind)."
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
        """Commit the live-dragged position via :class:`MoveNode`.

        The model was mutated LIVE during the drag (so the canvas could
        follow the cursor) -- restore the pre-drag value first so the
        MoveNode command captures the correct ``_original_xy`` and
        produces a single, fully-reversible undo entry.

        :param x: World x coordinate at release (the final position).
        :param y: World y coordinate at release.
        """
        target = self._drag_target
        origin = self._drag_origin_xy
        # Clear state up front so any early return below leaves the app
        # ready for the next interaction.
        self._drag_target = None
        self._drag_origin_xy = None

        if target is None or origin is None:
            return

        # Restore the original coordinates so MoveNode.apply captures them
        # correctly as _original_xy.
        self._model.nodes[target] = origin

        # No-op shortcut: if the drag ended where it started, there's no
        # mutation to record.
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

    def _draw_obstacle_preview(self) -> None:
        """Overlay a dotted rectangle on the axes during an obstacle drag.

        Called after :meth:`_refresh_view` clears and redraws everything;
        the preview is therefore drawn on top of the freshly rendered
        scene but is wiped on the next refresh, which keeps it ephemeral
        without touching the model.
        """
        if self._obstacle_anchor is None or self._obstacle_cursor is None:
            return
        import matplotlib.patches as mpatches  # noqa: PLC0415

        ax_anchor, ay_anchor = self._obstacle_anchor
        ax_cursor, ay_cursor = self._obstacle_cursor
        xmin = min(ax_anchor, ax_cursor)
        ymin = min(ay_anchor, ay_cursor)
        width = abs(ax_cursor - ax_anchor)
        height = abs(ay_cursor - ay_anchor)
        if width <= 0.0 or height <= 0.0:
            return
        rect = mpatches.Rectangle(
            (xmin, ymin),
            width,
            height,
            linewidth=_OBSTACLE_PREVIEW_LINEWIDTH,
            edgecolor=_OBSTACLE_PREVIEW_EDGECOLOR,
            linestyle=_OBSTACLE_PREVIEW_LINESTYLE,
            facecolor="none",
            zorder=_OBSTACLE_PREVIEW_ZORDER,
        )
        self._canvas.axes.add_patch(rect)
        self._canvas.figure.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Keyboard shortcut handlers
    # ------------------------------------------------------------------

    def _handle_delete_key(self) -> None:
        """``Delete`` -- if EDGE mode + two endpoints selected, drop the edge.

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
        if self._dispatch(DeleteEdge(a, b)):
            self._edge_selection.clear()
            self._set_status(f"Deleted edge ({a!r}, {b!r}).")
            self._refresh_view()

    def _handle_edge_line_key(self) -> None:
        """``L`` -- dispatch :class:`AddLineEdge` if two endpoints picked."""
        if self._current_mode() is not Mode.EDGE:
            return
        if len(self._edge_selection) != _EDGE_SELECTION_MAX:
            self._set_status("EDGE mode: pick two nodes first.")
            return
        a, b = self._edge_selection
        if self._dispatch(AddLineEdge(a, b)):
            self._edge_selection.clear()
            self._set_status(f"Added line edge ({a!r}, {b!r}).")
            self._refresh_view()

    def _handle_edge_blind_key(self) -> None:
        """``B`` -- dispatch :class:`AddBlindEdge` if two endpoints picked."""
        if self._current_mode() is not Mode.EDGE:
            return
        if len(self._edge_selection) != _EDGE_SELECTION_MAX:
            self._set_status("EDGE mode: pick two nodes first.")
            return
        a, b = self._edge_selection
        if self._dispatch(AddBlindEdge(a, b)):
            self._edge_selection.clear()
            self._set_status(f"Added blind edge ({a!r}, {b!r}).")
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
        """Prompt for a new name and dispatch :class:`RenameNode`.

        On success, queues the rename into :attr:`_pending_renames` so
        the next Save flushes it through the LibCST EDGES rewriter.

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
        new_name = new_name.strip()
        if not new_name or new_name == target:
            return
        if self._dispatch(RenameNode(target, new_name)):
            self._record_pending_rename(target, new_name)

    def _context_set_type(self, target: str, new_type: str) -> None:
        """Dispatch :class:`SetNodeType` from the right-click submenu.

        :param target: Node whose type is being changed.
        :param new_type: ``"line"`` or ``"blind"``.
        """
        if self._model.node_types.get(target) == new_type:
            self._set_status(f"{target!r} is already {new_type!r}.")
            return
        self._dispatch(SetNodeType(target, new_type))

    def _context_delete(self, target: str) -> None:
        """Confirm + dispatch :class:`DeleteNode`, checking EDGES first.

        Refuses the delete if the imported ``field_data.EDGES`` references
        the node (the model doesn't own ``EDGES``; we peek at the module's
        live attribute). The user is told the offending entry index.

        :param target: Node to delete.
        """
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

        self._dispatch(DeleteNode(target))

    def _find_edges_reference(self, node_name: str) -> Optional[int]:
        """Return the index of the first ``EDGES`` entry referencing *node_name*.

        Scans the live ``tools.path_planner.field_data.EDGES`` list (if the
        module was importable). Returns ``None`` when the node is unmentioned
        or when the module / attribute is unavailable.

        :param node_name: Node to search for.
        :return: Index into ``EDGES`` or ``None``.
        """
        if self._field_data_module is None:
            return None
        edges = getattr(self._field_data_module, "EDGES", None)
        if not isinstance(edges, list):
            return None
        for index, entry in enumerate(edges):
            if not isinstance(entry, dict):
                continue
            if entry.get("u") == node_name or entry.get("v") == node_name:
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
        # 1. Validate first so we never write a half-broken JSON file.
        try:
            self._model.validate()
        except Exception as exc:  # noqa: BLE001 -- show any validator error
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
            self._pending_renames.clear()

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
