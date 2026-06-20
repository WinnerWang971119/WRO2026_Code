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

from tools.path_planner.editor import theme

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
# Theme constants (T11).  Named ttk styles + font sizes + accent-bar geometry.
# Colours and font families come from the shared ``theme`` module; only the
# sizing literals live here so the code body stays free of magic numbers.
# ---------------------------------------------------------------------------

# Named ttk styles defined by :meth:`SidePanels._apply_theme` and assigned to
# the ttk widgets at build time so SidePanels reads as themed even before the
# app applies a root-level base theme (T13).
_STYLE_LABELFRAME: str = "Editor.TLabelframe"
_STYLE_LABELFRAME_LABEL: str = "Editor.TLabelframe.Label"
_STYLE_BUTTON: str = "Editor.TButton"
_STYLE_COMBOBOX: str = "Editor.TCombobox"
_STYLE_SPINBOX: str = "Editor.TSpinbox"
_STYLE_MONO_SPINBOX: str = "Editor.Mono.TSpinbox"

# Base font sizes (points) for the panel's text roles.
_FONT_SIZE_UI: int = 10
_FONT_SIZE_MONO: int = 10
# The status surface reads a touch larger so it stays legible as the floor of
# the column; the badge is the largest, boldest line of the four.
_FONT_SIZE_STATUS: int = 10
_FONT_SIZE_BADGE: int = 11

# Mode-toolbar accent bar: a thin vertical terracotta strip on the left edge of
# the active mode's row.  Width in pixels; the inactive rows show a same-width
# strip painted in the panel surface tone so the rows stay aligned.
_ACCENT_BAR_WIDTH_PX: int = 4
# Horizontal gap (pixels) between the accent strip and its mode label.
_MODE_ROW_GAP_PX: int = 6
# Vertical padding (pixels) around each mode row so the toolbar breathes.
_MODE_ROW_PAD_Y: int = 2

# Highlight thickness (pixels) used to flatten the chrome on plain tk widgets
# (Listbox / Canvas / buttons) so they sit cleanly on the warm surface.
_FLAT_BORDER_PX: int = 0
# Sunken border (pixels) kept on the status surface so it still reads as a
# distinct sunk plane at the floor of the column.
_STATUS_BORDER_PX: int = 1


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

        # ---- Resolved fonts (T11) --------------------------------------
        # Resolve the theme's fallback chains once, now that a Tk root almost
        # certainly exists (this frame has a master).  ``resolve_font`` degrades
        # to the chain's generic tail when no families are queryable, so this is
        # headless-safe.  Stored as ``(family, size)`` tuples ready to hand to
        # any widget's ``font=`` option.
        self._font_ui: tuple[str, int] = (
            theme.resolve_font(theme.FONT_SANS),
            _FONT_SIZE_UI,
        )
        self._font_mono: tuple[str, int] = (
            theme.resolve_font(theme.FONT_MONO),
            _FONT_SIZE_MONO,
        )
        self._font_status: tuple[str, int] = (
            theme.resolve_font(theme.FONT_SANS),
            _FONT_SIZE_STATUS,
        )
        self._font_badge: tuple[str, int, str] = (
            theme.resolve_font(theme.FONT_DISPLAY),
            _FONT_SIZE_BADGE,
            "bold",
        )

        # Paint the panel column itself with the warm surface tone so it no
        # longer reads as default grey, even before the app applies a base
        # theme to the root (T13).
        self.configure(background=theme.SURFACE)

        # Per-mode accent strips, populated by :meth:`_build_mode_selector`
        # and recoloured by :meth:`_update_mode_accent` on every ``mode_var``
        # write.  Keyed by the mode's ``value`` string.
        self._mode_accent_bars: dict[str, tk.Frame] = {}

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

        # ---- Define the named ttk styles before any widget is built ----
        # so the ttk Labelframes / Comboboxes / Spinboxes below can opt into
        # them by name.  Guarded internally; a no-op if no live root exists.
        self._apply_theme()

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

        # ---- Drive the mode-toolbar accent bar from ``mode_var`` --------
        # A pure-visual trace: it only recolours the accent strips, never fires
        # a user callback, so it is safe to leave installed for the panel's
        # lifetime and needs no ``_suppress_callbacks`` gate.  Paint the initial
        # accent for the default mode now that the strips exist.
        self.mode_var.trace_add("write", self._on_mode_var_write)
        self._update_mode_accent()

    # ------------------------------------------------------------------
    # Theme application (T11) -- named ttk styles + the accent-bar driver.
    # ------------------------------------------------------------------

    def _apply_theme(self) -> None:
        """Define the panel's named ``ttk.Style`` entries from the palette.

        This configures *named* styles (``Editor.TLabelframe`` etc.) rather
        than touching the base theme or calling ``theme_use`` -- that root-level
        work belongs to the app (T13).  Configuring named styles here means the
        ttk widgets in this panel look themed even when :class:`SidePanels` is
        built standalone under the default theme (the headless smoke test).

        Every step is guarded: a :class:`ttk.Style` needs a live Tk root, so if
        one cannot be obtained the method degrades to a no-op rather than
        raising, keeping the headless build green.
        """
        try:
            style = ttk.Style(self)
        except tk.TclError:
            return

        ui_font = self._font_ui
        mono_font = self._font_mono

        # Labelframe: warm surface plane with an ink heading in the UI font.
        self._safe_configure(
            style, _STYLE_LABELFRAME, background=theme.SURFACE
        )
        self._safe_configure(
            style,
            _STYLE_LABELFRAME_LABEL,
            background=theme.SURFACE,
            foreground=theme.INK,
            font=ui_font,
        )

        # Buttons rest on the surface and flash terracotta on press / hover.
        self._safe_configure(
            style,
            _STYLE_BUTTON,
            background=theme.SURFACE,
            foreground=theme.INK,
            font=ui_font,
        )
        self._safe_map(
            style,
            _STYLE_BUTTON,
            background=[("pressed", theme.TERRACOTTA), ("active", theme.TERRACOTTA)],
            foreground=[("pressed", theme.PAPER), ("active", theme.PAPER)],
        )

        # Combobox: light paper field, ink text, UI font.
        self._safe_configure(
            style,
            _STYLE_COMBOBOX,
            fieldbackground=theme.PAPER_LIGHT,
            background=theme.SURFACE,
            foreground=theme.INK,
            font=ui_font,
        )

        # Spinboxes: a UI-font variant (start heading) and a mono variant for
        # the numeric reset-facing field.  Both read on the light paper tone.
        self._safe_configure(
            style,
            _STYLE_SPINBOX,
            fieldbackground=theme.PAPER_LIGHT,
            background=theme.SURFACE,
            foreground=theme.INK,
            font=ui_font,
        )
        self._safe_configure(
            style,
            _STYLE_MONO_SPINBOX,
            fieldbackground=theme.PAPER_LIGHT,
            background=theme.SURFACE,
            foreground=theme.INK,
            font=mono_font,
        )

    @staticmethod
    def _safe_configure(style: ttk.Style, name: str, **kwargs) -> None:
        """Call ``style.configure`` swallowing backend-specific rejections.

        Some option names (``fieldbackground``) are rejected by some platform
        themes; attempting the configure and ignoring a rejection keeps the
        rest of the theming applied rather than aborting on the first miss.

        :param style: A live :class:`ttk.Style`.
        :param name: The named style to configure.
        :param kwargs: Style options forwarded to ``configure``.
        """
        try:
            style.configure(name, **kwargs)
        except tk.TclError:
            pass

    @staticmethod
    def _safe_map(style: ttk.Style, name: str, **kwargs) -> None:
        """Call ``style.map`` swallowing backend-specific rejections.

        :param style: A live :class:`ttk.Style`.
        :param name: The named style to map.
        :param kwargs: State-keyed option lists forwarded to ``map``.
        """
        try:
            style.map(name, **kwargs)
        except tk.TclError:
            pass

    def _on_mode_var_write(self, *_args: object) -> None:
        """``mode_var`` write trace: repaint the active-mode accent strip.

        Purely visual -- it never invokes a user ``on_*`` callback, so it is
        unaffected by (and does not need to consult) :attr:`_suppress_callbacks`.

        :param _args: Tk trace positional args (var name, index, op); unused.
        """
        self._update_mode_accent()

    def _update_mode_accent(self) -> None:
        """Colour the accent strip of the active mode and clear the others.

        The active mode's left-edge strip is painted :data:`theme.TERRACOTTA`;
        every other strip is painted the panel surface tone so the rows stay
        aligned but only the active one shows the accent.  Safe to call before
        all strips exist (it iterates whatever is registered).
        """
        active = self.mode_var.get()
        for value, bar in self._mode_accent_bars.items():
            color = theme.TERRACOTTA if value == active else theme.SURFACE
            try:
                bar.configure(background=color)
            except tk.TclError:
                pass

    # ------------------------------------------------------------------
    # Themed plain-tk widget factories (T11)
    # ------------------------------------------------------------------
    # ttk styling cannot reach plain ``tk`` widgets, so these helpers stamp the
    # warm palette + fonts onto the Frames / Labels / Buttons / Entries the tree
    # still uses.  Keeping them in one place stops the per-widget bg/fg/font
    # triples from scattering naked palette references through every builder.

    def _themed_frame(self, master: tk.Misc, **kwargs) -> tk.Frame:
        """Return a :class:`tk.Frame` painted with the panel surface tone."""
        kwargs.setdefault("background", theme.SURFACE)
        return tk.Frame(master, **kwargs)

    def _themed_label(self, master: tk.Misc, **kwargs) -> tk.Label:
        """Return a surface-toned, ink, UI-font :class:`tk.Label`."""
        kwargs.setdefault("background", theme.SURFACE)
        kwargs.setdefault("foreground", theme.INK)
        kwargs.setdefault("font", self._font_ui)
        return tk.Label(master, **kwargs)

    def _themed_button(self, master: tk.Misc, **kwargs) -> tk.Button:
        """Return a :class:`tk.Button` styled like the warm ttk buttons.

        Resting state sits on the panel surface; the active (hover / press)
        state flips to the terracotta accent on paper, mirroring the
        ``Editor.TButton`` ttk map so the plain buttons match the styled ones.
        """
        kwargs.setdefault("background", theme.SURFACE)
        kwargs.setdefault("foreground", theme.INK)
        kwargs.setdefault("activebackground", theme.TERRACOTTA)
        kwargs.setdefault("activeforeground", theme.PAPER)
        kwargs.setdefault("highlightthickness", _FLAT_BORDER_PX)
        kwargs.setdefault("font", self._font_ui)
        return tk.Button(master, **kwargs)

    def _themed_entry(
        self, master: tk.Misc, *, mono: bool, **kwargs
    ) -> tk.Entry:
        """Return a paper-field :class:`tk.Entry`.

        :param mono: ``True`` for numeric fields (coordinates, knobs) so they
            use :data:`theme.FONT_MONO`; ``False`` for text fields, which use
            the UI sans font.
        """
        kwargs.setdefault("background", theme.PAPER_LIGHT)
        kwargs.setdefault("foreground", theme.INK)
        kwargs.setdefault("insertbackground", theme.INK)
        kwargs.setdefault("disabledbackground", theme.SURFACE)
        kwargs.setdefault("disabledforeground", theme.INK_MUTED)
        kwargs.setdefault("readonlybackground", theme.SURFACE)
        kwargs.setdefault("highlightthickness", _FLAT_BORDER_PX)
        kwargs.setdefault("font", self._font_mono if mono else self._font_ui)
        return tk.Entry(master, **kwargs)

    # ------------------------------------------------------------------
    # Sub-frame builders
    # ------------------------------------------------------------------

    def _build_mode_selector(self) -> None:
        """Create the top **Mode** toolbar: one accented radio row per mode.

        Each mode is rendered as a row holding a thin left-edge accent strip
        (:data:`theme.TERRACOTTA` for the active mode, surface-toned otherwise)
        followed by a radio button bound to the unchanged :attr:`mode_var`.  The
        accent is driven by the ``mode_var`` write trace installed in
        ``__init__`` so it tracks the active mode regardless of who set it.
        """
        group = ttk.Labelframe(
            self, text="Mode", padding=_GROUP_INNER_PAD, style=_STYLE_LABELFRAME
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(_PANEL_OUTER_PAD, _SECTION_SPACING),
        )

        for mode, label in _MODE_LABELS:
            row = tk.Frame(group, background=theme.SURFACE)
            row.pack(side="top", fill="x", pady=_MODE_ROW_PAD_Y)

            # Thin left-edge accent strip; initial colour is set by the trace's
            # first run after the toolbar finishes building.
            accent = tk.Frame(
                row,
                width=_ACCENT_BAR_WIDTH_PX,
                background=theme.SURFACE,
            )
            accent.pack(side="left", fill="y")
            # Keep the strip at its fixed width regardless of row height.
            accent.pack_propagate(False)
            self._mode_accent_bars[mode.value] = accent

            btn = tk.Radiobutton(
                row,
                text=label,
                variable=self.mode_var,
                value=mode.value,
                anchor="w",
                background=theme.SURFACE,
                foreground=theme.INK,
                activebackground=theme.SURFACE,
                activeforeground=theme.TERRACOTTA,
                selectcolor=theme.PAPER_LIGHT,
                highlightthickness=_FLAT_BORDER_PX,
                font=self._font_ui,
            )
            btn.pack(
                side="left", fill="x", expand=True, padx=(_MODE_ROW_GAP_PX, 0)
            )

    def _build_node_list_and_inspector(self) -> None:
        """Create the **Nodes** labelframe: list + name/type/coords inspector."""
        group = ttk.Labelframe(
            self, text="Nodes", padding=_GROUP_INNER_PAD, style=_STYLE_LABELFRAME
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        # --- Node listbox (with vertical scrollbar) ---
        list_frame = self._themed_frame(group)
        list_frame.pack(side="top", fill="x")

        scrollbar = tk.Scrollbar(
            list_frame,
            orient="vertical",
            background=theme.SURFACE,
            troughcolor=theme.PAPER,
            highlightthickness=_FLAT_BORDER_PX,
        )
        scrollbar.pack(side="right", fill="y")

        self._node_listbox = tk.Listbox(
            list_frame,
            height=_NODE_LIST_HEIGHT,
            width=_NODE_LIST_WIDTH,
            selectmode=tk.SINGLE,
            exportselection=False,
            yscrollcommand=scrollbar.set,
            background=theme.PAPER_LIGHT,
            foreground=theme.INK,
            selectbackground=theme.TERRACOTTA,
            selectforeground=theme.PAPER,
            highlightthickness=_FLAT_BORDER_PX,
            font=self._font_ui,
        )
        self._node_listbox.pack(side="left", fill="x", expand=True)
        scrollbar.config(command=self._node_listbox.yview)

        self._node_listbox.bind("<<ListboxSelect>>", self._handle_listbox_select)

        # --- Inspector: read-only name entry ---
        name_row = self._themed_frame(group)
        name_row.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))
        self._themed_label(name_row, text="Name:").pack(side="left")
        self._name_entry = self._themed_entry(
            name_row,
            mono=False,
            textvariable=self._selected_node_var,
            state="readonly",
        )
        self._name_entry.pack(side="left", fill="x", expand=True)

        # --- Inspector: node type combobox ---
        type_row = self._themed_frame(group)
        type_row.pack(side="top", fill="x")
        self._themed_label(type_row, text="Type:").pack(side="left")
        self._type_combo = ttk.Combobox(
            type_row,
            textvariable=self._selected_type_var,
            values=list(_NODE_TYPE_VALUES),
            state="disabled",
            style=_STYLE_COMBOBOX,
        )
        self._type_combo.pack(side="left", fill="x", expand=True)
        self._type_combo.bind(
            "<<ComboboxSelected>>", self._handle_type_change
        )

        # --- Inspector: x / y editable entries (numeric -> mono font) ---
        coords_row = self._themed_frame(group)
        coords_row.pack(side="top", fill="x")
        self._themed_label(coords_row, text="x:").pack(side="left")
        self._x_entry = self._themed_entry(
            coords_row,
            mono=True,
            textvariable=self._selected_x_var,
            width=_COORD_ENTRY_WIDTH,
            state="disabled",
        )
        self._x_entry.pack(side="left", padx=(0, 6))
        self._x_entry.bind("<Return>", self._handle_coords_commit)
        self._x_entry.bind("<FocusOut>", self._handle_coords_commit)

        self._themed_label(coords_row, text="y:").pack(side="left")
        self._y_entry = self._themed_entry(
            coords_row,
            mono=True,
            textvariable=self._selected_y_var,
            width=_COORD_ENTRY_WIDTH,
            state="disabled",
        )
        self._y_entry.pack(side="left")
        self._y_entry.bind("<Return>", self._handle_coords_commit)
        self._y_entry.bind("<FocusOut>", self._handle_coords_commit)

        # --- Auto-create edges row (T7) ---
        autocreate_row = self._themed_frame(group)
        autocreate_row.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))

        length_label_row = self._themed_frame(autocreate_row)
        length_label_row.pack(side="top", fill="x")
        self._themed_label(length_label_row, text="Max edge length (cm):").pack(
            side="left"
        )
        self._max_edge_length_spinbox = ttk.Spinbox(
            length_label_row,
            from_=_MAX_EDGE_LENGTH_MIN,
            to=_MAX_EDGE_LENGTH_MAX,
            increment=_MAX_EDGE_LENGTH_STEP,
            textvariable=self._max_edge_length_var,
            format=_MAX_EDGE_LENGTH_FORMAT,
            width=6,
            style=_STYLE_MONO_SPINBOX,
        )
        self._max_edge_length_spinbox.pack(side="right")

        # Button sub-row: hosts the Auto-create button in idle mode and
        # the Accept / Reject pair in preview mode.  The state is toggled
        # via :meth:`set_auto_create_state`.
        self._autocreate_button_frame = self._themed_frame(autocreate_row)
        self._autocreate_button_frame.pack(side="top", fill="x", pady=(4, 0))

        self._auto_create_button = self._themed_button(
            self._autocreate_button_frame,
            text="Auto-create edges",
            command=self._handle_auto_create_edges,
        )
        self._accept_button = self._themed_button(
            self._autocreate_button_frame,
            text="Accept",
            command=self._handle_accept_proposed,
        )
        self._reject_button = self._themed_button(
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
            self,
            text="Reset spec",
            padding=_GROUP_INNER_PAD,
            style=_STYLE_LABELFRAME,
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        axis_row = self._themed_frame(group)
        axis_row.pack(side="top", fill="x")
        self._themed_label(axis_row, text="Axis:").pack(side="left")
        self._reset_axis_combo = ttk.Combobox(
            axis_row,
            textvariable=self._reset_axis_var,
            values=list(_RESET_AXIS_VALUES),
            state="disabled",
            style=_STYLE_COMBOBOX,
        )
        self._reset_axis_combo.pack(side="left", fill="x", expand=True)
        self._reset_axis_combo.bind(
            "<<ComboboxSelected>>", self._handle_reset_spec_commit
        )

        wall_row = self._themed_frame(group)
        wall_row.pack(side="top", fill="x")
        self._themed_label(wall_row, text="Wall coord (cm):").pack(side="left")
        self._reset_wall_entry = self._themed_entry(
            wall_row,
            mono=True,
            textvariable=self._reset_wall_coord_var,
            width=_RESET_COORD_ENTRY_WIDTH,
            state="disabled",
        )
        self._reset_wall_entry.pack(side="right")
        self._reset_wall_entry.bind("<Return>", self._handle_reset_spec_commit)
        self._reset_wall_entry.bind(
            "<FocusOut>", self._handle_reset_spec_commit
        )

        facing_row = self._themed_frame(group)
        facing_row.pack(side="top", fill="x")
        self._themed_label(facing_row, text="Facing (deg):").pack(side="left")
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
            style=_STYLE_MONO_SPINBOX,
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
        group = ttk.Labelframe(
            self, text="Pivots", padding=_GROUP_INNER_PAD, style=_STYLE_LABELFRAME
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        # --- Scrollable list of existing pivot triples ---
        list_frame = self._themed_frame(group, height=_PIVOT_LIST_HEIGHT_PX)
        list_frame.pack(side="top", fill="x")
        # Keep the canvas at the requested height even when it has no rows.
        list_frame.pack_propagate(False)

        self._pivot_canvas = tk.Canvas(
            list_frame,
            height=_PIVOT_LIST_HEIGHT_PX,
            highlightthickness=_FLAT_BORDER_PX,
            background=theme.SURFACE,
        )
        pivot_scroll = tk.Scrollbar(
            list_frame,
            orient="vertical",
            command=self._pivot_canvas.yview,
            background=theme.SURFACE,
            troughcolor=theme.PAPER,
            highlightthickness=_FLAT_BORDER_PX,
        )
        pivot_scroll.pack(side="right", fill="y")
        self._pivot_canvas.pack(side="left", fill="both", expand=True)
        self._pivot_canvas.configure(yscrollcommand=pivot_scroll.set)

        # Inner frame that actually holds the rows; rebuilt by ``refresh``.
        self._pivot_rows_frame = self._themed_frame(self._pivot_canvas)
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
        add_row = self._themed_frame(group)
        add_row.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))

        self._pivot_u_combo = ttk.Combobox(
            add_row,
            textvariable=self._pivot_u_var,
            values=[],
            state="readonly",
            width=_PIVOT_COMBO_WIDTH,
            style=_STYLE_COMBOBOX,
        )
        self._pivot_u_combo.pack(side="left")
        self._themed_label(add_row, text=_PIVOT_ARROW).pack(side="left")
        self._pivot_v_combo = ttk.Combobox(
            add_row,
            textvariable=self._pivot_v_var,
            values=[],
            state="readonly",
            width=_PIVOT_COMBO_WIDTH,
            style=_STYLE_COMBOBOX,
        )
        self._pivot_v_combo.pack(side="left")
        self._themed_label(add_row, text=_PIVOT_ARROW).pack(side="left")
        self._pivot_w_combo = ttk.Combobox(
            add_row,
            textvariable=self._pivot_w_var,
            values=[],
            state="readonly",
            width=_PIVOT_COMBO_WIDTH,
            style=_STYLE_COMBOBOX,
        )
        self._pivot_w_combo.pack(side="left")

        self._pivot_add_button = self._themed_button(
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
            self,
            text="Global knobs",
            padding=_GROUP_INNER_PAD,
            style=_STYLE_LABELFRAME,
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        toggle_row = self._themed_frame(group)
        toggle_row.pack(side="top", fill="x")
        self._blind_decay_check = tk.Checkbutton(
            toggle_row,
            text="Blind-decay enabled",
            variable=self._blind_decay_enabled_var,
            anchor="w",
            command=self._handle_blind_decay_toggle,
            background=theme.SURFACE,
            foreground=theme.INK,
            activebackground=theme.SURFACE,
            activeforeground=theme.TERRACOTTA,
            selectcolor=theme.PAPER_LIGHT,
            highlightthickness=_FLAT_BORDER_PX,
            font=self._font_ui,
        )
        self._blind_decay_check.pack(side="left", fill="x")

        # One labelled entry per float knob (numeric -> mono font).  Bind both
        # commit triggers and capture the knob name in the lambda's default arg
        # (late-binding trap).
        self._float_knob_entries: dict[str, tk.Entry] = {}
        for knob_name, label_text in _FLOAT_KNOB_LABELS:
            row = self._themed_frame(group)
            row.pack(side="top", fill="x")
            self._themed_label(row, text=label_text).pack(side="left")
            entry = self._themed_entry(
                row,
                mono=True,
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
        group = ttk.Labelframe(
            self, text="Start", padding=_GROUP_INNER_PAD, style=_STYLE_LABELFRAME
        )
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        node_row = self._themed_frame(group)
        node_row.pack(side="top", fill="x")
        self._themed_label(node_row, text="Node:").pack(side="left")
        self._start_node_combo = ttk.Combobox(
            node_row,
            textvariable=self._start_node_var,
            values=[],
            state="readonly",
            style=_STYLE_COMBOBOX,
        )
        self._start_node_combo.pack(side="left", fill="x", expand=True)
        self._start_node_combo.bind(
            "<<ComboboxSelected>>", self._handle_start_node_change
        )

        heading_row = self._themed_frame(group)
        heading_row.pack(side="top", fill="x")
        self._themed_label(heading_row, text="Heading:").pack(side="left")
        self._heading_spinbox = ttk.Spinbox(
            heading_row,
            from_=_HEADING_MIN_DEG,
            to=_HEADING_MAX_DEG,
            increment=_HEADING_STEP_DEG,
            textvariable=self._start_heading_var,
            format=_HEADING_FORMAT,
            command=self._handle_heading_change,
            style=_STYLE_MONO_SPINBOX,
        )
        self._heading_spinbox.pack(side="left", fill="x", expand=True)
        # The Spinbox's ``command`` only fires on arrow clicks; bind
        # <Return> and <FocusOut> so manual edits to the entry also commit.
        self._heading_spinbox.bind("<Return>", self._handle_heading_change)
        self._heading_spinbox.bind("<FocusOut>", self._handle_heading_change)

    def _build_action_buttons(self) -> None:
        """Create the bottom 2x2 grid of action buttons."""
        group = self._themed_frame(self)
        group.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _SECTION_SPACING),
        )

        # Two equal-width columns so the four buttons line up cleanly.
        group.grid_columnconfigure(0, weight=1, uniform="actions")
        group.grid_columnconfigure(1, weight=1, uniform="actions")

        self._undo_button = self._themed_button(
            group, text="Undo", command=self._handle_undo
        )
        self._undo_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

        self._redo_button = self._themed_button(
            group, text="Redo", command=self._handle_redo
        )
        self._redo_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

        self._save_button = self._themed_button(
            group, text="Save", command=self._handle_save
        )
        self._save_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

        self._save_build_button = self._themed_button(
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
        group = self._themed_frame(
            self, relief="sunken", bd=_STATUS_BORDER_PX
        )
        group.pack(
            side="bottom",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(0, _PANEL_OUTER_PAD),
        )

        # Validation badge -- kept on its own label so it can be recoloured by
        # :meth:`set_validation_badge`.  It uses the largest, boldest font of
        # the four status lines and starts in the muted "unknown" tone until the
        # app reports a validation result.
        self._badge_label = self._themed_label(
            group,
            textvariable=self._badge_var,
            anchor="w",
            foreground=theme.INK_MUTED,
            font=self._font_badge,
        )
        self._badge_label.pack(side="top", fill="x")

        self._mode_label = self._themed_label(
            group,
            textvariable=self._mode_display_var,
            anchor="w",
            font=self._font_status,
        )
        self._mode_label.pack(side="top", fill="x")

        self._selection_label = self._themed_label(
            group,
            textvariable=self._selection_display_var,
            anchor="w",
            font=self._font_status,
        )
        self._selection_label.pack(side="top", fill="x")

        # Free-form status line: wraps so a long message stays readable across
        # the enlarged surface instead of being clipped to one row.
        self._status_label = self._themed_label(
            group,
            textvariable=self._status_var,
            anchor="w",
            justify="left",
            wraplength=_PANEL_WIDTH_PX - 2 * _PANEL_OUTER_PAD,
            foreground=theme.INK_MUTED,
            font=self._font_status,
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

        Updates both the badge **text** and its **colour**: a valid result is
        shown in :data:`theme.VALID`, an invalid one in :data:`theme.INVALID`.
        The badge owns its own dedicated :class:`tk.Label` so recolouring it
        does not disturb the other status lines.

        :param ok: ``True`` to show the valid badge, ``False`` for invalid.
        """
        self._badge_var.set(_BADGE_VALID_TEXT if ok else _BADGE_INVALID_TEXT)
        self._badge_label.configure(
            foreground=theme.VALID if ok else theme.INVALID
        )

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
            row = self._themed_frame(self._pivot_rows_frame)
            row.pack(side="top", fill="x")
            label_text = f" {_PIVOT_ARROW} ".join(str(part) for part in triple)
            self._themed_label(row, text=label_text, anchor="w").pack(
                side="left", fill="x", expand=True
            )
            # Capture the triple in the default arg to dodge the late-binding
            # closure trap (every button would otherwise see the last triple).
            self._themed_button(
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
