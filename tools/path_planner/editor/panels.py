"""Right-side Tk control panels for the WRO 2026 live field editor.

This module owns the **widget tree** for the editor's right-hand column: a
mode selector, a node list + inspector, the start-pose form, four action
buttons, and a one-line status bar.  It does **not** own any application
logic -- mouse handling on the map, the undo/redo stack, dialog flows, and
the actual Save / Build pipeline all live in T10 (canvas) and T12
(``EditorApp``).

The :class:`SidePanels` frame exposes its outputs in two ways:

* As **Tk variables** that the app reads on demand
  (e.g. ``panels.mode_var.get()``).
* As **callback attributes** that the app assigns *after* construction and
  that :class:`SidePanels` invokes when the user touches a widget
  (e.g. ``panels.on_node_select = my_callback``).  Every callback defaults
  to ``None``; missing callbacks are silently skipped, so the panel can be
  instantiated and shown without any wiring (useful for visual smoke tests).

The :meth:`SidePanels.refresh` method re-syncs every widget from a
:class:`~tools.path_planner.editor.model.FieldModel`.  T12 calls
``refresh(model)`` after every model mutation so the listbox, inspector,
and start-pose form always reflect the current truth.

This file is intentionally self-contained: it does not import
``canvas.py`` (both are wired together by T12), and it never calls
``mainloop()`` (that is also T12's job).
"""

from __future__ import annotations

import enum
import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING, Callable, Optional, Union

if TYPE_CHECKING:
    from tools.path_planner.editor.model import FieldModel


# ---------------------------------------------------------------------------
# Mode enum -- exported for the app (T12) so it can switch on the active mode.
# ---------------------------------------------------------------------------


class Mode(enum.Enum):
    """Editor mode controlling how mouse events on the canvas are interpreted.

    The ``value`` of each member is the string the radio buttons store in the
    ``mode_var`` :class:`tk.StringVar`.  The app reads the active value via
    ``Mode(panels.mode_var.get())``.

    :cvar ADD_NODE: Left-click on the canvas creates a new node.
    :cvar MOVE: Left-click selects; drag moves the picked node.
    :cvar EDGE: Click two nodes, then press ``L`` / ``B`` to add an edge.
    :cvar OBSTACLE: Drag a rectangle to create a new obstacle.
    :cvar START: Click a node to set it as ``START_NODE``.
    """

    ADD_NODE = "add_node"
    MOVE = "move"
    EDGE = "edge"
    OBSTACLE = "obstacle"
    START = "start"


# Human-readable labels shown next to each radio button, in display order.
_MODE_LABELS: tuple[tuple[Mode, str], ...] = (
    (Mode.ADD_NODE, "Add node"),
    (Mode.MOVE, "Move / select"),
    (Mode.EDGE, "Edge (L / B)"),
    (Mode.OBSTACLE, "Obstacle"),
    (Mode.START, "Set start"),
)

# Allowed values for a node's ``type`` field; mirrors validator rules.
# ``"reset"`` is editable in the inspector combobox (the app routes the
# selection to the right command in T7), but the add-node dialog stays
# line/blind -- a reset node cannot be created from scratch.
_NODE_TYPE_VALUES: tuple[str, ...] = ("line", "blind", "reset")

# The node ``type`` string that enables the reset-spec sub-panel.
_RESET_NODE_TYPE: str = "reset"

# Allowed values for a reset node's wall-zero axis.
_RESET_AXIS_VALUES: tuple[str, ...] = ("x", "y")

# Keys of a reset-node spec dict in ``model.reset_nodes`` (mirrors the
# validator's ``RESET_NODES`` schema).
_RESET_AXIS_KEY: str = "axis"
_RESET_WALL_COORD_KEY: str = "wall_coord"
_RESET_FACING_KEY: str = "facing"

# ---------------------------------------------------------------------------
# Layout / sizing constants.  No magic numbers in the code body.
# ---------------------------------------------------------------------------

# Outer padding around the whole SidePanels frame (pixels).
_PANEL_OUTER_PAD = 6
# Padding inside each Labelframe (pixels).
_GROUP_INNER_PAD = 4
# Vertical gap between stacked sub-frames (pixels).
_SECTION_SPACING = 6
# Fixed width of the panel column (pixels) so the matplotlib canvas to its
# left does not jitter when widget content changes.
_PANEL_WIDTH_PX = 240
# Node listbox visible-row count.
_NODE_LIST_HEIGHT = 10
# Node listbox visible-column width (characters).
_NODE_LIST_WIDTH = 22
# Heading spinbox bounds (degrees).
_HEADING_MIN_DEG = 0.0
_HEADING_MAX_DEG = 359.9
# Spinbox heading increment (degrees per arrow click).
_HEADING_STEP_DEG = 0.1
# Decimal places shown in the (x, y) inspector label.
_COORD_DECIMALS = 1
# Format string for the heading spinbox value.
_HEADING_FORMAT = "%.1f"
# Coordinate-entry format string (centimetres, 1 decimal).
_COORD_FORMAT: str = "%.1f"
# Coordinate-entry width in characters.
_COORD_ENTRY_WIDTH: int = 8
# Initial status-bar message before the app sets anything.
_DEFAULT_STATUS_TEXT = "Ready."

# Max-edge-length Spinbox bounds (centimetres).
_MAX_EDGE_LENGTH_MIN: float = 10.0
_MAX_EDGE_LENGTH_MAX: float = 300.0
_MAX_EDGE_LENGTH_STEP: float = 5.0
_MAX_EDGE_LENGTH_DEFAULT: float = 60.0
_MAX_EDGE_LENGTH_FORMAT: str = "%.1f"

# Reset-spec wall-coordinate entry width (characters).
_RESET_COORD_ENTRY_WIDTH: int = 8
# Reset-spec facing Spinbox width (characters).
_RESET_FACING_SPINBOX_WIDTH: int = 8
# Reset-spec wall-coordinate entry format string (centimetres, 1 decimal).
_RESET_COORD_FORMAT: str = "%.1f"

# Pivot list visible-row count (scrollable area height in pixels).
_PIVOT_LIST_HEIGHT_PX: int = 96
# Pivot add-row Combobox width (characters) for each of the three node pickers.
_PIVOT_COMBO_WIDTH: int = 6
# Arrow glyph rendered between the three nodes of a pivot triple.
_PIVOT_ARROW: str = "->"

# Global-knob Entry width (characters).
_KNOB_ENTRY_WIDTH: int = 8
# Names of the float-valued global knobs, in display order.  Each maps to a
# float attribute on the model and fires ``on_global_knob_change(name, value)``.
_FLOAT_KNOB_LABELS: tuple[tuple[str, str], ...] = (
    ("blind_decay_k", "Decay k:"),
    ("blind_decay_exponent", "Decay exp:"),
    ("blind_decay_d_ref", "Decay d_ref (cm):"),
    ("robot_radius_cm", "Robot radius (cm):"),
)
# Name of the boolean global knob driven by the checkbutton.
_BOOL_KNOB_NAME: str = "blind_decay_enabled"

# Default mode-display text before the app sets anything.
_DEFAULT_MODE_TEXT: str = "Mode: --"
# Default selection-display text before the app sets anything.
_DEFAULT_SELECTION_TEXT: str = "Selection: none"
# Validation-badge text for the valid / invalid / unknown states.
_BADGE_VALID_TEXT: str = "VALID"
_BADGE_INVALID_TEXT: str = "INVALID"
_BADGE_UNKNOWN_TEXT: str = "VALIDATION: --"


# ---------------------------------------------------------------------------
# SidePanels frame
# ---------------------------------------------------------------------------


class SidePanels(tk.Frame):
    """Right-hand control column for the live field editor.

    Construction is pure widget-building: every callback attribute starts as
    ``None``, every Tk variable starts at its safe default, and no model is
    consulted.  The owning app (T12) is expected to:

    1. Instantiate the frame: ``panels = SidePanels(root)``.
    2. Pack / grid it: ``panels.pack(side="right", fill="y")``.
    3. Assign every callback attribute (``on_node_select``,
       ``on_save_and_build``, etc.) to a closure that performs the real work.
    4. Call :meth:`refresh` once with the freshly loaded
       :class:`FieldModel`, and again after every model mutation.

    :ivar mode_var: ``tk.StringVar`` holding the active :class:`Mode`'s
        ``value``.  Read via ``Mode(self.mode_var.get())``.
    :ivar on_node_select: Fired when the listbox selection changes.  The
        sole argument is the selected node name, or ``None`` if the
        selection was cleared.
    :ivar on_node_type_change: Fired when the inspector's type combobox
        changes.  Arguments are ``(node_name, new_type)``.
    :ivar on_node_coords_change: Fired when the inspector's x/y entry
        widgets commit a valid in-range value.  Arguments are
        ``(node_name, x_cm, y_cm)``.
    :ivar on_auto_create_edges: Fired when the **Auto-create edges** button
        is clicked.  No arguments.
    :ivar on_accept_proposed: Fired when the **Accept** button is clicked
        in preview mode.  No arguments.
    :ivar on_reject_proposed: Fired when the **Reject** button is clicked
        in preview mode.  No arguments.
    :ivar on_start_node_change: Fired when the start-pose node combobox
        changes.  Sole argument is the chosen node name.
    :ivar on_start_heading_change: Fired when the start-pose spinbox
        changes.  Sole argument is the new heading in degrees.
    :ivar on_undo: Fired when the **Undo** button is clicked.
    :ivar on_redo: Fired when the **Redo** button is clicked.
    :ivar on_save: Fired when the **Save** button is clicked.
    :ivar on_save_and_build: Fired when the **Save & Build** button is
        clicked.
    :ivar on_reset_spec_change: Fired when the reset-spec sub-panel commits a
        valid edit (combobox-select / ``<Return>`` / ``<FocusOut>``) while a
        reset-typed node is selected.  Arguments are
        ``(node_name, axis, wall_coord, facing)`` with ``axis`` a string in
        :data:`_RESET_AXIS_VALUES` and ``wall_coord`` / ``facing`` floats.  A
        bad numeric parse sets a status message and does **not** fire.
    :ivar on_reset_spec_clear: Optional slot for an explicit "clear reset spec"
        affordance.  This panel exposes no such button -- the type combobox
        already routes a ``reset`` -> ``line``/``blind`` change through
        :attr:`on_node_type_change`, which the app (T7) treats as the
        clear-reset path.  The attribute exists so a future affordance can wire
        to it without changing this signature; it is never fired from here.
    :ivar on_add_pivot: Fired when the pivots **Add** button is clicked with
        all three node pickers chosen.  Arguments are ``(u, v, w)`` node names.
    :ivar on_delete_pivot: Fired when a pivot row's **Remove** button is
        clicked.  Sole argument is the ``(u, v, w)`` triple to delete.
    :ivar on_global_knob_change: Fired when a global-knob widget commits.  For
        the ``blind_decay_enabled`` checkbutton the value is a ``bool``; for
        the float entries it is a ``float``.  Arguments are
        ``(knob_name, value)``.  A bad numeric parse sets a status message and
        does **not** fire.
    """

    def __init__(self, master: tk.Misc) -> None:
        """Build the widget tree.

        No model is consulted here -- all widgets start in their default
        empty state and must be populated by a subsequent call to
        :meth:`refresh`.

        :param master: The parent Tk container (usually the root window or
            an outer ``ttk.PanedWindow``).
        """
        super().__init__(master, width=_PANEL_WIDTH_PX)
        # Prevent the frame from shrinking to fit smaller children, so its
        # width stays fixed regardless of which sub-section is tallest.
        self.pack_propagate(False)

        # ---- Public callback attributes (assigned by T12 after init) ----
        self.on_node_select: Optional[Callable[[Optional[str]], None]] = None
        self.on_node_type_change: Optional[Callable[[str, str], None]] = None
        self.on_node_coords_change: Optional[Callable[[str, float, float], None]] = None
        self.on_auto_create_edges: Optional[Callable[[], None]] = None
        self.on_accept_proposed: Optional[Callable[[], None]] = None
        self.on_reject_proposed: Optional[Callable[[], None]] = None
        self.on_start_node_change: Optional[Callable[[str], None]] = None
        self.on_start_heading_change: Optional[Callable[[float], None]] = None
        self.on_undo: Optional[Callable[[], None]] = None
        self.on_redo: Optional[Callable[[], None]] = None
        self.on_save: Optional[Callable[[], None]] = None
        self.on_save_and_build: Optional[Callable[[], None]] = None
        self.on_reset_spec_change: Optional[
            Callable[[str, str, float, float], None]
        ] = None
        self.on_reset_spec_clear: Optional[Callable[[str], None]] = None
        self.on_add_pivot: Optional[Callable[[str, str, str], None]] = None
        self.on_delete_pivot: Optional[
            Callable[[tuple[str, str, str]], None]
        ] = None
        self.on_global_knob_change: Optional[
            Callable[[str, Union[bool, float]], None]
        ] = None

        # ---- Tk variables (single source of truth for widget state) -----
        self.mode_var = tk.StringVar(value=Mode.MOVE.value)
        self._selected_node_var = tk.StringVar(value="")
        self._selected_type_var = tk.StringVar(value="")
        self._selected_x_var = tk.StringVar(value="")
        self._selected_y_var = tk.StringVar(value="")
        self._start_node_var = tk.StringVar(value="")
        self._start_heading_var = tk.StringVar(
            value=_HEADING_FORMAT % _HEADING_MIN_DEG
        )
        self._max_edge_length_var = tk.StringVar(
            value=_MAX_EDGE_LENGTH_FORMAT % _MAX_EDGE_LENGTH_DEFAULT
        )
        self._status_var = tk.StringVar(value=_DEFAULT_STATUS_TEXT)
        self._mode_display_var = tk.StringVar(value=_DEFAULT_MODE_TEXT)
        self._selection_display_var = tk.StringVar(value=_DEFAULT_SELECTION_TEXT)
        self._badge_var = tk.StringVar(value=_BADGE_UNKNOWN_TEXT)

        # ---- Reset-spec sub-panel Tk variables -------------------------
        self._reset_axis_var = tk.StringVar(value="")
        self._reset_wall_coord_var = tk.StringVar(value="")
        self._reset_facing_var = tk.StringVar(value="")

        # ---- Pivots add-row Tk variables (three node pickers) ----------
        self._pivot_u_var = tk.StringVar(value="")
        self._pivot_v_var = tk.StringVar(value="")
        self._pivot_w_var = tk.StringVar(value="")

        # ---- Global-knob Tk variables ----------------------------------
        self._blind_decay_enabled_var = tk.BooleanVar(value=False)
        # One StringVar per float knob, keyed by the canonical knob name so the
        # commit handler can look up which model attribute a widget feeds.
        self._float_knob_vars: dict[str, tk.StringVar] = {
            name: tk.StringVar(value="") for name, _label in _FLOAT_KNOB_LABELS
        }

        # Field bounds cached from refresh() so coord validation doesn't need
        # to round-trip through the app -> model on every keystroke.
        # ``(width_cm, height_cm)`` once the first refresh runs; ``None`` until then.
        self._field_extent_cm: Optional[tuple[float, float]] = None

        # Re-entrancy guard: ``refresh`` writes to Tk variables, which would
        # otherwise re-fire user-callback bindings.  Setting this flag tells
        # the handlers to swallow events caused by programmatic updates.
        self._suppress_callbacks = False

        # ---- Build every sub-frame + status surface --------------------
        # The enlarged status surface is built first and packed to the bottom
        # so it always anchors the column floor regardless of how tall the
        # scrollable middle sections grow.
        self._build_status_bar()
        self._build_mode_selector()
        self._build_node_list_and_inspector()
        self._build_reset_spec_panel()
        self._build_pivots_panel()
        self._build_global_knobs_panel()
        self._build_start_pose_panel()
        self._build_action_buttons()

    # ------------------------------------------------------------------
    # Sub-frame builders
    # ------------------------------------------------------------------

    def _build_mode_selector(self) -> None:
        """Create the top **Mode** labelframe with one radio button per mode."""
        group = ttk.Labelframe(self, text="Mode", padding=_GROUP_INNER_PAD)
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(_PANEL_OUTER_PAD, _SECTION_SPACING),
        )

        for mode, label in _MODE_LABELS:
            btn = tk.Radiobutton(
                group,
                text=label,
                variable=self.mode_var,
                value=mode.value,
                anchor="w",
            )
            btn.pack(side="top", fill="x", anchor="w")

    def _build_node_list_and_inspector(self) -> None:
        """Create the **Nodes** labelframe: list + name/type/coords inspector."""
        group = ttk.Labelframe(self, text="Nodes", padding=_GROUP_INNER_PAD)
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        # --- Node listbox (with vertical scrollbar) ---
        list_frame = tk.Frame(group)
        list_frame.pack(side="top", fill="x")

        scrollbar = tk.Scrollbar(list_frame, orient="vertical")
        scrollbar.pack(side="right", fill="y")

        self._node_listbox = tk.Listbox(
            list_frame,
            height=_NODE_LIST_HEIGHT,
            width=_NODE_LIST_WIDTH,
            selectmode=tk.SINGLE,
            exportselection=False,
            yscrollcommand=scrollbar.set,
        )
        self._node_listbox.pack(side="left", fill="x", expand=True)
        scrollbar.config(command=self._node_listbox.yview)

        self._node_listbox.bind("<<ListboxSelect>>", self._handle_listbox_select)

        # --- Inspector: read-only name entry ---
        name_row = tk.Frame(group)
        name_row.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))
        tk.Label(name_row, text="Name:").pack(side="left")
        self._name_entry = tk.Entry(
            name_row,
            textvariable=self._selected_node_var,
            state="readonly",
        )
        self._name_entry.pack(side="left", fill="x", expand=True)

        # --- Inspector: node type combobox ---
        type_row = tk.Frame(group)
        type_row.pack(side="top", fill="x")
        tk.Label(type_row, text="Type:").pack(side="left")
        self._type_combo = ttk.Combobox(
            type_row,
            textvariable=self._selected_type_var,
            values=list(_NODE_TYPE_VALUES),
            state="disabled",
        )
        self._type_combo.pack(side="left", fill="x", expand=True)
        self._type_combo.bind(
            "<<ComboboxSelected>>", self._handle_type_change
        )

        # --- Inspector: x / y editable entries ---
        coords_row = tk.Frame(group)
        coords_row.pack(side="top", fill="x")
        tk.Label(coords_row, text="x:").pack(side="left")
        self._x_entry = tk.Entry(
            coords_row,
            textvariable=self._selected_x_var,
            width=_COORD_ENTRY_WIDTH,
            state="disabled",
        )
        self._x_entry.pack(side="left", padx=(0, 6))
        self._x_entry.bind("<Return>", self._handle_coords_commit)
        self._x_entry.bind("<FocusOut>", self._handle_coords_commit)

        tk.Label(coords_row, text="y:").pack(side="left")
        self._y_entry = tk.Entry(
            coords_row,
            textvariable=self._selected_y_var,
            width=_COORD_ENTRY_WIDTH,
            state="disabled",
        )
        self._y_entry.pack(side="left")
        self._y_entry.bind("<Return>", self._handle_coords_commit)
        self._y_entry.bind("<FocusOut>", self._handle_coords_commit)

        # --- Auto-create edges row (T7) ---
        autocreate_row = tk.Frame(group)
        autocreate_row.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))

        length_label_row = tk.Frame(autocreate_row)
        length_label_row.pack(side="top", fill="x")
        tk.Label(length_label_row, text="Max edge length (cm):").pack(side="left")
        self._max_edge_length_spinbox = ttk.Spinbox(
            length_label_row,
            from_=_MAX_EDGE_LENGTH_MIN,
            to=_MAX_EDGE_LENGTH_MAX,
            increment=_MAX_EDGE_LENGTH_STEP,
            textvariable=self._max_edge_length_var,
            format=_MAX_EDGE_LENGTH_FORMAT,
            width=6,
        )
        self._max_edge_length_spinbox.pack(side="right")

        # Button sub-row: hosts the Auto-create button in idle mode and
        # the Accept / Reject pair in preview mode.  The state is toggled
        # via :meth:`set_auto_create_state`.
        self._autocreate_button_frame = tk.Frame(autocreate_row)
        self._autocreate_button_frame.pack(side="top", fill="x", pady=(4, 0))

        self._auto_create_button = tk.Button(
            self._autocreate_button_frame,
            text="Auto-create edges",
            command=self._handle_auto_create_edges,
        )
        self._accept_button = tk.Button(
            self._autocreate_button_frame,
            text="Accept",
            command=self._handle_accept_proposed,
        )
        self._reject_button = tk.Button(
            self._autocreate_button_frame,
            text="Reject",
            command=self._handle_reject_proposed,
        )

        # Start in idle mode -- Accept / Reject hidden until preview opens.
        self.set_auto_create_state("idle")

    def _build_reset_spec_panel(self) -> None:
        """Create the **Reset spec** labelframe for the selected reset node.

        Holds an axis combobox, a wall-coordinate entry, and a facing spinbox.
        The whole sub-panel is enabled only while a ``reset``-typed node is
        selected (see :meth:`_apply_reset_spec` / :meth:`_clear_reset_spec`);
        committing any of the three widgets fires
        :attr:`on_reset_spec_change` with the parsed triple, mirroring the
        coordinate-entry commit pattern.
        """
        group = ttk.Labelframe(
            self, text="Reset spec", padding=_GROUP_INNER_PAD
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        axis_row = tk.Frame(group)
        axis_row.pack(side="top", fill="x")
        tk.Label(axis_row, text="Axis:").pack(side="left")
        self._reset_axis_combo = ttk.Combobox(
            axis_row,
            textvariable=self._reset_axis_var,
            values=list(_RESET_AXIS_VALUES),
            state="disabled",
        )
        self._reset_axis_combo.pack(side="left", fill="x", expand=True)
        self._reset_axis_combo.bind(
            "<<ComboboxSelected>>", self._handle_reset_spec_commit
        )

        wall_row = tk.Frame(group)
        wall_row.pack(side="top", fill="x")
        tk.Label(wall_row, text="Wall coord (cm):").pack(side="left")
        self._reset_wall_entry = tk.Entry(
            wall_row,
            textvariable=self._reset_wall_coord_var,
            width=_RESET_COORD_ENTRY_WIDTH,
            state="disabled",
        )
        self._reset_wall_entry.pack(side="right")
        self._reset_wall_entry.bind("<Return>", self._handle_reset_spec_commit)
        self._reset_wall_entry.bind(
            "<FocusOut>", self._handle_reset_spec_commit
        )

        facing_row = tk.Frame(group)
        facing_row.pack(side="top", fill="x")
        tk.Label(facing_row, text="Facing (deg):").pack(side="left")
        self._reset_facing_spinbox = ttk.Spinbox(
            facing_row,
            from_=_HEADING_MIN_DEG,
            to=_HEADING_MAX_DEG,
            increment=_HEADING_STEP_DEG,
            textvariable=self._reset_facing_var,
            format=_HEADING_FORMAT,
            width=_RESET_FACING_SPINBOX_WIDTH,
            command=self._handle_reset_spec_commit,
            state="disabled",
        )
        self._reset_facing_spinbox.pack(side="right")
        # The Spinbox ``command`` only fires on arrow clicks; bind keyboard /
        # focus-loss commits like the start-heading spinbox does.
        self._reset_facing_spinbox.bind(
            "<Return>", self._handle_reset_spec_commit
        )
        self._reset_facing_spinbox.bind(
            "<FocusOut>", self._handle_reset_spec_commit
        )

    def _build_pivots_panel(self) -> None:
        """Create the **Pivots** labelframe: list of triples + an add-row.

        The list area is a scrollable column of ``u -> v -> w`` rows, each with
        a **Remove** button that fires :attr:`on_delete_pivot`.  Below it an
        add-row of three node-name comboboxes plus an **Add** button fires
        :attr:`on_add_pivot` once all three pickers hold a value.  The list is
        rebuilt and the picker dropdowns repopulated on every :meth:`refresh`.
        """
        group = ttk.Labelframe(self, text="Pivots", padding=_GROUP_INNER_PAD)
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        # --- Scrollable list of existing pivot triples ---
        list_frame = tk.Frame(group, height=_PIVOT_LIST_HEIGHT_PX)
        list_frame.pack(side="top", fill="x")
        # Keep the canvas at the requested height even when it has no rows.
        list_frame.pack_propagate(False)

        self._pivot_canvas = tk.Canvas(
            list_frame, height=_PIVOT_LIST_HEIGHT_PX, highlightthickness=0
        )
        pivot_scroll = tk.Scrollbar(
            list_frame, orient="vertical", command=self._pivot_canvas.yview
        )
        pivot_scroll.pack(side="right", fill="y")
        self._pivot_canvas.pack(side="left", fill="both", expand=True)
        self._pivot_canvas.configure(yscrollcommand=pivot_scroll.set)

        # Inner frame that actually holds the rows; rebuilt by ``refresh``.
        self._pivot_rows_frame = tk.Frame(self._pivot_canvas)
        self._pivot_canvas.create_window(
            (0, 0), window=self._pivot_rows_frame, anchor="nw"
        )
        # Keep the scrollregion in sync with the row frame's natural size.
        self._pivot_rows_frame.bind(
            "<Configure>",
            lambda _event: self._pivot_canvas.configure(
                scrollregion=self._pivot_canvas.bbox("all")
            ),
        )

        # --- Add-row: three node pickers + Add button ---
        add_row = tk.Frame(group)
        add_row.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))

        self._pivot_u_combo = ttk.Combobox(
            add_row,
            textvariable=self._pivot_u_var,
            values=[],
            state="readonly",
            width=_PIVOT_COMBO_WIDTH,
        )
        self._pivot_u_combo.pack(side="left")
        tk.Label(add_row, text=_PIVOT_ARROW).pack(side="left")
        self._pivot_v_combo = ttk.Combobox(
            add_row,
            textvariable=self._pivot_v_var,
            values=[],
            state="readonly",
            width=_PIVOT_COMBO_WIDTH,
        )
        self._pivot_v_combo.pack(side="left")
        tk.Label(add_row, text=_PIVOT_ARROW).pack(side="left")
        self._pivot_w_combo = ttk.Combobox(
            add_row,
            textvariable=self._pivot_w_var,
            values=[],
            state="readonly",
            width=_PIVOT_COMBO_WIDTH,
        )
        self._pivot_w_combo.pack(side="left")

        self._pivot_add_button = tk.Button(
            add_row, text="Add", command=self._handle_add_pivot
        )
        self._pivot_add_button.pack(side="right")

    def _build_global_knobs_panel(self) -> None:
        """Create the **Global knobs** labelframe.

        Exposes the blind-decay toggle (a checkbutton bound to a
        :class:`tk.BooleanVar`) and one float entry per knob in
        :data:`_FLOAT_KNOB_LABELS`.  Each widget commit fires
        :attr:`on_global_knob_change` with the canonical knob name and the
        parsed value; a bad float parse sets a status message and does not
        fire.
        """
        group = ttk.Labelframe(
            self, text="Global knobs", padding=_GROUP_INNER_PAD
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        toggle_row = tk.Frame(group)
        toggle_row.pack(side="top", fill="x")
        self._blind_decay_check = tk.Checkbutton(
            toggle_row,
            text="Blind-decay enabled",
            variable=self._blind_decay_enabled_var,
            anchor="w",
            command=self._handle_blind_decay_toggle,
        )
        self._blind_decay_check.pack(side="left", fill="x")

        # One labelled entry per float knob.  Bind both commit triggers and
        # capture the knob name in the lambda's default arg (late-binding trap).
        self._float_knob_entries: dict[str, tk.Entry] = {}
        for knob_name, label_text in _FLOAT_KNOB_LABELS:
            row = tk.Frame(group)
            row.pack(side="top", fill="x")
            tk.Label(row, text=label_text).pack(side="left")
            entry = tk.Entry(
                row,
                textvariable=self._float_knob_vars[knob_name],
                width=_KNOB_ENTRY_WIDTH,
            )
            entry.pack(side="right")
            entry.bind(
                "<Return>",
                lambda _event, name=knob_name: self._handle_float_knob_commit(
                    name
                ),
            )
            entry.bind(
                "<FocusOut>",
                lambda _event, name=knob_name: self._handle_float_knob_commit(
                    name
                ),
            )
            self._float_knob_entries[knob_name] = entry

    def _build_start_pose_panel(self) -> None:
        """Create the **Start** labelframe: node combobox + heading spinbox."""
        group = ttk.Labelframe(self, text="Start", padding=_GROUP_INNER_PAD)
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        node_row = tk.Frame(group)
        node_row.pack(side="top", fill="x")
        tk.Label(node_row, text="Node:").pack(side="left")
        self._start_node_combo = ttk.Combobox(
            node_row,
            textvariable=self._start_node_var,
            values=[],
            state="readonly",
        )
        self._start_node_combo.pack(side="left", fill="x", expand=True)
        self._start_node_combo.bind(
            "<<ComboboxSelected>>", self._handle_start_node_change
        )

        heading_row = tk.Frame(group)
        heading_row.pack(side="top", fill="x")
        tk.Label(heading_row, text="Heading:").pack(side="left")
        self._heading_spinbox = ttk.Spinbox(
            heading_row,
            from_=_HEADING_MIN_DEG,
            to=_HEADING_MAX_DEG,
            increment=_HEADING_STEP_DEG,
            textvariable=self._start_heading_var,
            format=_HEADING_FORMAT,
            command=self._handle_heading_change,
        )
        self._heading_spinbox.pack(side="left", fill="x", expand=True)
        # The Spinbox's ``command`` only fires on arrow clicks; bind
        # <Return> and <FocusOut> so manual edits to the entry also commit.
        self._heading_spinbox.bind("<Return>", self._handle_heading_change)
        self._heading_spinbox.bind("<FocusOut>", self._handle_heading_change)

    def _build_action_buttons(self) -> None:
        """Create the bottom 2x2 grid of action buttons."""
        group = tk.Frame(self)
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        # Two equal-width columns so the four buttons line up cleanly.
        group.grid_columnconfigure(0, weight=1, uniform="actions")
        group.grid_columnconfigure(1, weight=1, uniform="actions")

        self._undo_button = tk.Button(
            group, text="Undo", command=self._handle_undo
        )
        self._undo_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

        self._redo_button = tk.Button(
            group, text="Redo", command=self._handle_redo
        )
        self._redo_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

        self._save_button = tk.Button(
            group, text="Save", command=self._handle_save
        )
        self._save_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

        self._save_build_button = tk.Button(
            group, text="Save & Build", command=self._handle_save_and_build
        )
        self._save_build_button.grid(
            row=1, column=1, sticky="ew", padx=2, pady=2
        )

        # Buttons start disabled until the app says otherwise.
        self.set_undo_enabled(False)
        self.set_redo_enabled(False)

    def _build_status_bar(self) -> None:
        """Create the multi-line status surface at the bottom of the panel.

        The surface stacks four labels inside a sunken frame: the validation
        **badge**, the current **mode**, the current **selection**, and the
        free-form one-line **status** message.  Each has its own
        :class:`tk.Label` (and the badge is kept distinct so T11 can colour it)
        driven by a dedicated Tk variable updated through the public setters
        :meth:`set_validation_badge`, :meth:`set_mode_display`,
        :meth:`set_selection_display`, and :meth:`set_status`.
        """
        group = tk.Frame(self, relief="sunken", bd=1)
        group.pack(
            side="bottom",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _PANEL_OUTER_PAD),
        )

        # Validation badge -- kept on its own label so T11 can recolour it.
        self._badge_label = tk.Label(
            group, textvariable=self._badge_var, anchor="w"
        )
        self._badge_label.pack(side="top", fill="x")

        self._mode_label = tk.Label(
            group, textvariable=self._mode_display_var, anchor="w"
        )
        self._mode_label.pack(side="top", fill="x")

        self._selection_label = tk.Label(
            group, textvariable=self._selection_display_var, anchor="w"
        )
        self._selection_label.pack(side="top", fill="x")

        # Free-form status line: wraps so a long message stays readable across
        # the enlarged surface instead of being clipped to one row.
        self._status_label = tk.Label(
            group,
            textvariable=self._status_var,
            anchor="w",
            justify="left",
            wraplength=_PANEL_WIDTH_PX - 2 * _PANEL_OUTER_PAD,
        )
        self._status_label.pack(side="top", fill="x")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def refresh(self, model: "FieldModel") -> None:
        """Re-sync every widget from *model*.

        Should be called once at app start and after every model mutation
        (T12 owns the wiring).  This method intentionally does **not** fire
        any of the ``on_*`` callbacks: callbacks are user-driven, while
        :meth:`refresh` is driven by programmatic state replays (model
        load, undo, redo, etc.).

        The method preserves the listbox selection when possible: if the
        previously selected node still exists in the refreshed model, it is
        reselected; otherwise the selection (and the inspector) is cleared.

        :param model: The current :class:`FieldModel` to render.
        """
        self._suppress_callbacks = True
        try:
            # Cache the field bounds (cm) so coord-entry validation can read them
            # without a back-reference to the model.
            width_mm, height_mm = model.field_mm
            self._field_extent_cm = (float(width_mm) / 10.0, float(height_mm) / 10.0)

            previously_selected = self._selected_node_var.get() or None

            # --- Node listbox: rebuild from sorted node names. ---
            sorted_names = sorted(model.nodes.keys())
            self._node_listbox.delete(0, tk.END)
            for name in sorted_names:
                self._node_listbox.insert(tk.END, name)

            # Restore the previous selection if the node still exists.  The
            # reset-spec sub-panel tracks the selection: it is enabled +
            # populated only when the selected node is reset-typed, and cleared
            # otherwise (including the no-selection case).
            if previously_selected in model.nodes:
                idx = sorted_names.index(previously_selected)
                self._node_listbox.selection_set(idx)
                self._node_listbox.see(idx)
                self._apply_inspector(model, previously_selected)
                if model.node_types.get(previously_selected) == _RESET_NODE_TYPE:
                    self._apply_reset_spec(model, previously_selected)
                else:
                    self._clear_reset_spec()
            else:
                self._clear_inspector()
                self._clear_reset_spec()

            # --- Pivots: rebuild the rows + repopulate the add-row pickers. ---
            self._rebuild_pivot_rows(model)
            for combo in (
                self._pivot_u_combo,
                self._pivot_v_combo,
                self._pivot_w_combo,
            ):
                combo.configure(values=sorted_names)

            # --- Global knobs: sync the checkbutton + float entries. ---
            self._blind_decay_enabled_var.set(bool(model.blind_decay_enabled))
            for knob_name, _label in _FLOAT_KNOB_LABELS:
                value = float(getattr(model, knob_name))
                self._float_knob_vars[knob_name].set(_COORD_FORMAT % value)

            # --- Start-pose combobox: refresh node list + current value. ---
            self._start_node_combo.configure(values=sorted_names)
            self._start_node_var.set(model.start_node)

            # --- Start-pose heading: clamp into the spinbox range. ---
            heading = max(
                _HEADING_MIN_DEG,
                min(_HEADING_MAX_DEG, float(model.start_heading_deg)),
            )
            self._start_heading_var.set(_HEADING_FORMAT % heading)
        finally:
            self._suppress_callbacks = False

    def set_status(self, text: str) -> None:
        """Replace the free-form status line with *text*.

        This drives the bottom-most line of the status surface.  The label
        wraps, so a longer-than-one-line message stays readable rather than
        being clipped.

        :param text: Status message to display.
        """
        self._status_var.set(text)

    def set_mode_display(self, text: str) -> None:
        """Replace the status surface's **mode** line with *text*.

        Called by the app whenever the active :class:`Mode` changes so the
        surface always names the current editing mode.

        :param text: Mode description (e.g. ``"Mode: Move / select"``).
        """
        self._mode_display_var.set(text)

    def set_selection_display(self, text: str) -> None:
        """Replace the status surface's **selection** line with *text*.

        Called by the app whenever the current selection changes so the
        surface always names the selected node (or "none").

        :param text: Selection description (e.g. ``"Selection: A1"``).
        """
        self._selection_display_var.set(text)

    def set_validation_badge(self, ok: bool) -> None:
        """Set the validation badge to a valid / invalid state.

        Only the badge **text** is changed here; colouring is deferred to the
        warm restyle (T11), which is why the badge owns its own dedicated
        :class:`tk.Label`.

        :param ok: ``True`` to show the valid badge, ``False`` for invalid.
        """
        self._badge_var.set(_BADGE_VALID_TEXT if ok else _BADGE_INVALID_TEXT)

    def set_undo_enabled(self, enabled: bool) -> None:
        """Enable or disable the **Undo** button.

        :param enabled: ``True`` to make the button clickable; ``False`` to
            grey it out.
        """
        self._undo_button.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def set_redo_enabled(self, enabled: bool) -> None:
        """Enable or disable the **Redo** button.

        :param enabled: ``True`` to make the button clickable; ``False`` to
            grey it out.
        """
        self._redo_button.configure(state=tk.NORMAL if enabled else tk.DISABLED)

    def set_auto_create_state(self, mode: str) -> None:
        """Toggle the auto-create button row between idle and preview modes.

        Idle mode shows only the **Auto-create edges** button.  Preview
        mode hides it and shows **Accept** + **Reject** instead.  The two
        callback slots :attr:`on_accept_proposed` / :attr:`on_reject_proposed`
        wire to those buttons.

        :param mode: ``"idle"`` or ``"preview"``.
        :raises ValueError: If *mode* is not one of the two strings.
        """
        if mode == "idle":
            self._accept_button.pack_forget()
            self._reject_button.pack_forget()
            self._auto_create_button.pack(side="left", fill="x", expand=True)
        elif mode == "preview":
            self._auto_create_button.pack_forget()
            self._accept_button.pack(side="left", fill="x", expand=True, padx=(0, 2))
            self._reject_button.pack(side="left", fill="x", expand=True, padx=(2, 0))
        else:
            raise ValueError(
                f"set_auto_create_state: mode must be 'idle' or 'preview', "
                f"got {mode!r}"
            )

    @property
    def max_edge_length_cm(self) -> float:
        """Current Spinbox value parsed as a float.

        Returns :data:`_MAX_EDGE_LENGTH_DEFAULT` if the Spinbox holds an
        unparseable string (defensive against a transient empty entry during
        a mid-edit FocusOut).  Out-of-range values are NOT clamped here; the
        Spinbox widget enforces its own ``from_``/``to`` bounds visually, and
        a manually typed out-of-range value will simply propagate to the
        auto-create caller.  Callers that need clamping should add their own
        bounds check.

        :return: Max edge length in centimetres, as a float.
        """
        raw = self._max_edge_length_var.get()
        try:
            return float(raw)
        except ValueError:
            return _MAX_EDGE_LENGTH_DEFAULT

    # ------------------------------------------------------------------
    # Internal helpers (inspector population)
    # ------------------------------------------------------------------

    def _apply_inspector(self, model: "FieldModel", node_name: str) -> None:
        """Populate the inspector widgets from *node_name* in *model*.

        :param model: Source of truth for the node's coordinates and type.
        :param node_name: Name of the node to display.
        """
        self._selected_node_var.set(node_name)
        self._selected_type_var.set(model.node_types.get(node_name, ""))
        x, y = model.nodes[node_name]
        self._selected_x_var.set(_COORD_FORMAT % float(x))
        self._selected_y_var.set(_COORD_FORMAT % float(y))
        self._type_combo.configure(state="readonly")
        self._x_entry.configure(state="normal")
        self._y_entry.configure(state="normal")

    def _clear_inspector(self) -> None:
        """Reset the inspector to its empty / no-selection state."""
        self._selected_node_var.set("")
        self._selected_type_var.set("")
        self._selected_x_var.set("")
        self._selected_y_var.set("")
        self._type_combo.configure(state="disabled")
        self._x_entry.configure(state="disabled")
        self._y_entry.configure(state="disabled")

    def _apply_reset_spec(self, model: "FieldModel", node_name: str) -> None:
        """Populate + enable the reset-spec sub-panel from *node_name*.

        Reads the node's ``{axis, wall_coord, facing}`` spec out of
        ``model.reset_nodes`` (a missing or partial spec falls back to safe
        defaults so the widgets never carry a stale value from another node),
        then switches the three widgets to their editable state.

        :param model: Source of truth for ``reset_nodes``.
        :param node_name: Name of the reset-typed node being inspected.
        """
        spec = model.reset_nodes.get(node_name, {})
        axis = spec.get(_RESET_AXIS_KEY, "")
        self._reset_axis_var.set(axis if axis in _RESET_AXIS_VALUES else "")

        wall_coord = spec.get(_RESET_WALL_COORD_KEY)
        if wall_coord is None:
            self._reset_wall_coord_var.set("")
        else:
            self._reset_wall_coord_var.set(_RESET_COORD_FORMAT % float(wall_coord))

        facing = spec.get(_RESET_FACING_KEY)
        if facing is None:
            self._reset_facing_var.set("")
        else:
            clamped = max(
                _HEADING_MIN_DEG, min(_HEADING_MAX_DEG, float(facing))
            )
            self._reset_facing_var.set(_HEADING_FORMAT % clamped)

        self._reset_axis_combo.configure(state="readonly")
        self._reset_wall_entry.configure(state="normal")
        self._reset_facing_spinbox.configure(state="normal")

    def _clear_reset_spec(self) -> None:
        """Reset the reset-spec sub-panel to its empty / disabled state."""
        self._reset_axis_var.set("")
        self._reset_wall_coord_var.set("")
        self._reset_facing_var.set("")
        self._reset_axis_combo.configure(state="disabled")
        self._reset_wall_entry.configure(state="disabled")
        self._reset_facing_spinbox.configure(state="disabled")

    def _rebuild_pivot_rows(self, model: "FieldModel") -> None:
        """Rebuild the pivot list rows from ``model.pivot_turns``.

        Every existing row widget is destroyed and re-created so the list
        always mirrors the model exactly (no stale rows survive an undo /
        delete).  Each row shows ``u -> v -> w`` and a **Remove** button whose
        command captures its own triple.

        :param model: Source of truth for ``pivot_turns``.
        """
        for child in self._pivot_rows_frame.winfo_children():
            child.destroy()

        for turn in model.pivot_turns:
            triple = tuple(turn)
            row = tk.Frame(self._pivot_rows_frame)
            row.pack(side="top", fill="x")
            label_text = f" {_PIVOT_ARROW} ".join(str(part) for part in triple)
            tk.Label(row, text=label_text, anchor="w").pack(
                side="left", fill="x", expand=True
            )
            # Capture the triple in the default arg to dodge the late-binding
            # closure trap (every button would otherwise see the last triple).
            tk.Button(
                row,
                text="Remove",
                command=lambda t=triple: self._handle_delete_pivot(t),
            ).pack(side="right")

        # Recompute the scrollregion now that the row count changed.
        self._pivot_rows_frame.update_idletasks()
        self._pivot_canvas.configure(scrollregion=self._pivot_canvas.bbox("all"))

    # ------------------------------------------------------------------
    # Widget event handlers -- forward to user callbacks, if assigned.
    # ------------------------------------------------------------------

    def _handle_listbox_select(self, _event: tk.Event) -> None:
        """Forward listbox selection changes to :attr:`on_node_select`."""
        if self._suppress_callbacks:
            return
        selection = self._node_listbox.curselection()
        node_name: Optional[str]
        if selection:
            node_name = self._node_listbox.get(selection[0])
            # Mirror the selection into the inspector's name variable so the
            # combobox handler below can read it without re-querying the
            # listbox.  (Type/coords are filled by the next ``refresh``.)
            self._selected_node_var.set(node_name)
        else:
            node_name = None
            self._clear_inspector()
        if self.on_node_select is not None:
            self.on_node_select(node_name)

    def _handle_type_change(self, _event: tk.Event) -> None:
        """Forward type-combobox changes to :attr:`on_node_type_change`."""
        if self._suppress_callbacks:
            return
        node_name = self._selected_node_var.get()
        new_type = self._selected_type_var.get()
        # Guard: only fire when both a node is selected AND the chosen type
        # is one of the valid values.  This protects downstream commands
        # from seeing empty strings on transient widget states.
        if not node_name or new_type not in _NODE_TYPE_VALUES:
            return
        if self.on_node_type_change is not None:
            self.on_node_type_change(node_name, new_type)

    def _handle_coords_commit(self, _event: tk.Event) -> None:
        """Parse x/y entries, validate, fire ``on_node_coords_change``.

        Triggered by ``<Return>`` or ``<FocusOut>`` on either entry. If the
        parsed value is invalid (not a float, out of field bounds), reset the
        entry to the model's current value via a status-bar message and do
        NOT fire the callback. The reset happens by writing back the var the
        *next* refresh would supply -- not by parsing the entry text -- so a
        partial edit cannot survive an invalid commit.

        :param _event: Tk event (unused).
        """
        if self._suppress_callbacks:
            return
        node_name = self._selected_node_var.get()
        if not node_name or self._field_extent_cm is None:
            return

        raw_x = self._selected_x_var.get()
        raw_y = self._selected_y_var.get()
        width_cm, height_cm = self._field_extent_cm
        try:
            x = float(raw_x)
        except ValueError:
            self._status_var.set("x must be a number")
            return
        try:
            y = float(raw_y)
        except ValueError:
            self._status_var.set("y must be a number")
            return
        if not (0.0 <= x <= width_cm):
            self._status_var.set(f"x must be in [0, {width_cm:.1f}]")
            return
        if not (0.0 <= y <= height_cm):
            self._status_var.set(f"y must be in [0, {height_cm:.1f}]")
            return

        if self.on_node_coords_change is not None:
            self.on_node_coords_change(node_name, x, y)

    def _handle_reset_spec_commit(self, _event: tk.Event | None = None) -> None:
        """Parse the reset-spec widgets and fire :attr:`on_reset_spec_change`.

        Triggered by the axis combobox selection, or ``<Return>`` /
        ``<FocusOut>`` / arrow-click on the wall-coordinate entry and facing
        spinbox.  Mirrors :meth:`_handle_coords_commit`: a missing selection,
        non-reset selection (the widgets are disabled then, but guard anyway),
        or unparseable float sets a status message and does **not** fire.  The
        facing value is clamped into ``[0, 360)`` like the start heading.

        :param _event: Tk event (unused; the spinbox ``command`` passes none).
        """
        if self._suppress_callbacks:
            return
        node_name = self._selected_node_var.get()
        if not node_name:
            return
        # Only a reset-typed node has an editable spec; the widgets are
        # disabled otherwise, but FocusOut can still fire on a state change.
        if self._selected_type_var.get() != _RESET_NODE_TYPE:
            return

        axis = self._reset_axis_var.get()
        if axis not in _RESET_AXIS_VALUES:
            self._status_var.set("axis must be 'x' or 'y'")
            return

        raw_wall = self._reset_wall_coord_var.get()
        try:
            wall_coord = float(raw_wall)
        except ValueError:
            self._status_var.set("wall_coord must be a number")
            return

        raw_facing = self._reset_facing_var.get()
        try:
            facing = float(raw_facing)
        except ValueError:
            self._status_var.set("facing must be a number")
            return
        facing = max(_HEADING_MIN_DEG, min(_HEADING_MAX_DEG, facing))

        if self.on_reset_spec_change is not None:
            self.on_reset_spec_change(node_name, axis, wall_coord, facing)

    def _handle_add_pivot(self) -> None:
        """Fire :attr:`on_add_pivot` once all three pickers hold a node.

        The add-row comboboxes are ``readonly`` so they can only carry valid
        node names; this guard only rejects the transient all-empty state.
        """
        if self._suppress_callbacks:
            return
        u = self._pivot_u_var.get()
        v = self._pivot_v_var.get()
        w = self._pivot_w_var.get()
        if not (u and v and w):
            self._status_var.set("pick u, v, and w to add a pivot")
            return
        if self.on_add_pivot is not None:
            self.on_add_pivot(u, v, w)

    def _handle_delete_pivot(self, triple: tuple[str, str, str]) -> None:
        """Fire :attr:`on_delete_pivot` for a pivot row's **Remove** button.

        :param triple: The ``(u, v, w)`` pivot to delete, captured from the row.
        """
        if self._suppress_callbacks:
            return
        if self.on_delete_pivot is not None:
            self.on_delete_pivot(triple)

    def _handle_blind_decay_toggle(self) -> None:
        """Fire :attr:`on_global_knob_change` for the blind-decay checkbutton."""
        if self._suppress_callbacks:
            return
        if self.on_global_knob_change is not None:
            self.on_global_knob_change(
                _BOOL_KNOB_NAME, bool(self._blind_decay_enabled_var.get())
            )

    def _handle_float_knob_commit(self, knob_name: str) -> None:
        """Parse a float-knob entry and fire :attr:`on_global_knob_change`.

        Triggered by ``<Return>`` / ``<FocusOut>`` on the entry bound to
        *knob_name*.  An unparseable value sets a status message and does
        **not** fire (mirrors the coordinate-entry commit pattern).

        :param knob_name: Canonical knob name (a key of
            :attr:`_float_knob_vars`) identifying which entry committed.
        """
        if self._suppress_callbacks:
            return
        raw = self._float_knob_vars[knob_name].get()
        try:
            value = float(raw)
        except ValueError:
            self._status_var.set(f"{knob_name} must be a number")
            return
        if self.on_global_knob_change is not None:
            self.on_global_knob_change(knob_name, value)

    def _handle_start_node_change(self, _event: tk.Event) -> None:
        """Forward start-node combobox changes to :attr:`on_start_node_change`."""
        if self._suppress_callbacks:
            return
        value = self._start_node_var.get()
        if not value:
            return
        if self.on_start_node_change is not None:
            self.on_start_node_change(value)

    def _handle_heading_change(self, _event: tk.Event | None = None) -> None:
        """Forward start-heading spinbox changes to
        :attr:`on_start_heading_change`.

        The spinbox stores a string; this handler parses it and silently
        ignores unparseable input rather than raising, so a transient
        partial edit (e.g. ``"3."``) does not crash the app.
        """
        if self._suppress_callbacks:
            return
        raw = self._start_heading_var.get()
        try:
            value = float(raw)
        except ValueError:
            return
        # Clamp into the spinbox's nominal range so callers never see a
        # value outside [0, 360).
        value = max(_HEADING_MIN_DEG, min(_HEADING_MAX_DEG, value))
        if self.on_start_heading_change is not None:
            self.on_start_heading_change(value)

    def _handle_undo(self) -> None:
        """Fire :attr:`on_undo` if assigned."""
        if self.on_undo is not None:
            self.on_undo()

    def _handle_redo(self) -> None:
        """Fire :attr:`on_redo` if assigned."""
        if self.on_redo is not None:
            self.on_redo()

    def _handle_auto_create_edges(self) -> None:
        """Fire :attr:`on_auto_create_edges` if assigned."""
        if self.on_auto_create_edges is not None:
            self.on_auto_create_edges()

    def _handle_accept_proposed(self) -> None:
        """Fire :attr:`on_accept_proposed` if assigned."""
        if self.on_accept_proposed is not None:
            self.on_accept_proposed()

    def _handle_reject_proposed(self) -> None:
        """Fire :attr:`on_reject_proposed` if assigned."""
        if self.on_reject_proposed is not None:
            self.on_reject_proposed()

    def _handle_save(self) -> None:
        """Fire :attr:`on_save` if assigned."""
        if self.on_save is not None:
            self.on_save()

    def _handle_save_and_build(self) -> None:
        """Fire :attr:`on_save_and_build` if assigned."""
        if self.on_save_and_build is not None:
            self.on_save_and_build()
