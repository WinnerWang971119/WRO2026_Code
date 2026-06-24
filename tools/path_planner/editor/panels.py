"""Tk control panels for the WRO 2026 live field editor.

This module owns the **widget tree** for the editor's chrome.  Since the
true-3-pane layout refactor (F1) the chrome is split into **four band frames**
that are siblings of the app's canvas widget under one shared ``parent`` grid,
so the window reads as a real three-pane editor rather than one fixed-width
right column:

* A **title bar** (:attr:`SidePanels._title_frame`) -- a full-width strip
  carrying the product name, the edited filename, and an unsaved "dirty" dot.
* A vertical **MODES rail** (:attr:`SidePanels._mode_frame`) -- one accented
  row per :class:`Mode`, pinned to the far left at a fixed narrow width.
* A **PROPERTIES inspector** (:attr:`SidePanels._inspector_frame`) -- the node
  fields, a collapsible reset-spec / pivots / global-knobs stack, the
  auto-create + start-pose forms, ghost add buttons, and the always-visible
  action-button block at its floor, pinned to the right at a fixed comfortable
  width.
* A **status strip** (:attr:`SidePanels._status_frame`) -- a full-width strip
  at the very bottom with a validation badge, the active mode, the current
  selection, and a clickable key-hint.

The canvas (owned by the app) sits between the mode rail and the inspector and
is the only band that expands.  :class:`SidePanels` does **not** place itself
or its bands; the app calls :meth:`SidePanels.layout_three_pane` once -- the
single placement entry point -- to grid all four bands plus the canvas into the
shared ``parent``.

It does **not** own any application logic -- mouse handling on the map, the
undo/redo stack, dialog flows, and the actual Save / Build pipeline all live in
the canvas and :class:`~tools.path_planner.editor.app.EditorApp`.

The :class:`SidePanels` frame exposes its outputs in two ways:

* As **Tk variables** that the app reads on demand
  (e.g. ``panels.mode_var.get()``).
* As **callback attributes** that the app assigns *after* construction and
  that :class:`SidePanels` invokes when the user touches a widget
  (e.g. ``panels.on_node_select = my_callback``).  Every callback defaults
  to ``None``; missing callbacks are silently skipped, so the panel can be
  instantiated and shown without any wiring (useful for visual smoke tests).

The :meth:`SidePanels.refresh` method re-syncs every widget from a
:class:`~tools.path_planner.editor.model.FieldModel`.  The app calls
``refresh(model)`` after every model mutation so the listbox, inspector,
and start-pose form always reflect the current truth.

This file is intentionally self-contained: it does not import
``canvas.py`` (both are wired together by the app), and it never calls
``mainloop()`` (that is also the app's job).
"""

from __future__ import annotations

import enum
import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING, Callable, Optional, Union

from tools.path_planner.editor import theme
from tools.path_planner.editor.commands import _find_override_index
from tools.path_planner.editor.model import NODE_TYPE_VALUES, RESET_AXIS_VALUES
from tools.path_planner.pipeline_config import (
    BlindMode,
    DirectionMode,
    LineMode,
    StopAtLineMode,
    TurnMode,
)

if TYPE_CHECKING:
    from tools.path_planner.editor.model import FieldModel


# ---------------------------------------------------------------------------
# Mode enum -- exported for the app so it can switch on the active mode.
# ---------------------------------------------------------------------------


class Mode(enum.Enum):
    """Editor mode controlling how mouse events on the canvas are interpreted.

    The ``value`` of each member is the string the radio buttons store in the
    ``mode_var`` :class:`tk.StringVar`.  The app reads the active value via
    ``Mode(panels.mode_var.get())``.

    :cvar ADD_NODE: Left-click on the canvas creates a new node.
    :cvar MOVE: Left-click arms a drag; drag moves the picked node.
    :cvar EDGE: Click two nodes, then press ``L`` / ``B`` to add an edge.
    :cvar OBSTACLE: Drag a rectangle to create a new obstacle.
    :cvar START: Click a node to set it as ``START_NODE``.
    :cvar SELECT: Left-click selects the node under the cursor; an empty click clears the selection.
    """

    ADD_NODE = "add_node"
    MOVE = "move"
    EDGE = "edge"
    OBSTACLE = "obstacle"
    START = "start"
    SELECT = "select"


# Human-readable labels shown next to each mode row, in display order.  These
# are the short labels for the compact left toolbar column.
_MODE_LABELS: tuple[tuple[Mode, str], ...] = (
    (Mode.SELECT, "Sel"),
    (Mode.MOVE, "Move"),
    (Mode.ADD_NODE, "Add"),
    (Mode.EDGE, "Edge"),
    (Mode.OBSTACLE, "Obst"),
    (Mode.START, "Strt"),
)

# Allowed values for a node's ``type`` field; single-sourced from
# ``model.NODE_TYPE_VALUES`` so the inspector combobox cannot offer a type the
# validator rejects. ``"reset"`` is editable in the inspector combobox (the app
# routes the selection to the right command), but the add-node dialog stays
# line/blind -- a reset node cannot be created from scratch.
_NODE_TYPE_VALUES: tuple[str, ...] = NODE_TYPE_VALUES

# The node ``type`` string that enables the reset-spec sub-panel.
_RESET_NODE_TYPE: str = "reset"

# Allowed values for a reset node's wall-zero axis (single-sourced).
_RESET_AXIS_VALUES: tuple[str, ...] = RESET_AXIS_VALUES

# Keys of a reset-node spec dict in ``model.reset_nodes`` (mirrors the
# validator's ``RESET_NODES`` schema).
_RESET_AXIS_KEY: str = "axis"
_RESET_WALL_COORD_KEY: str = "wall_coord"
_RESET_FACING_KEY: str = "facing"

# ---------------------------------------------------------------------------
# Layout / sizing constants.  No magic numbers in the code body.
# ---------------------------------------------------------------------------

# Outer padding around the chrome band frames (pixels).
_PANEL_OUTER_PAD = 6
# Padding inside each section body (pixels).
_GROUP_INNER_PAD = 4
# Vertical gap between stacked sub-frames (pixels).
_SECTION_SPACING = 6
# Fixed width of the far-left MODES rail band (pixels).  Just wide enough for
# the short mode labels plus the accent strip; the rail never expands.
_MODE_RAIL_WIDTH_PX = 90
# Fixed width of the right PROPERTIES inspector band (pixels).  Sized
# comfortably so section headers and field labels are NOT clipped -- the user
# reported words being cut off in the old 240-px column.
_INSPECTOR_WIDTH_PX = 312
# Node listbox visible-row count.
_NODE_LIST_HEIGHT = 8
# Node listbox visible-column width (characters).
_NODE_LIST_WIDTH = 18
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
_PIVOT_LIST_HEIGHT_PX: int = 84
# Pivot add-row Combobox width (characters) for each of the three node pickers.
_PIVOT_COMBO_WIDTH: int = 6
# Arrow glyph rendered between the three nodes of a pivot triple.
_PIVOT_ARROW: str = "→"  # rightwards arrow

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

# ---------------------------------------------------------------------------
# Edge-overrides / pipeline-defaults / extended-globals constants (issue #79).
# ---------------------------------------------------------------------------

# The two edge kinds an override can attach to.  The override ``type`` field is
# always one of these; it is AUTO-DERIVED from the matching existing segment and
# shown read-only (the operator cannot author a conflicting-type override).
_EDGE_TYPE_LINE: str = "LINE"
_EDGE_TYPE_BLIND: str = "BLIND"

# The sentinel a combobox shows for "no override key -- inherit the pipeline
# default".  Selecting it means the corresponding key is OMITTED from the
# override entry (and from the dispatched value, which is ``None``).  Presented
# verbatim per the sidecar's "default (inherit)" wording for primitive_mode and
# the bare "default" wording for direction / stop_at_line.
_INHERIT_PRIMITIVE_LABEL: str = "default (inherit)"
_INHERIT_LABEL: str = "default"

# ``primitive_mode`` tag vocabularies, single-sourced from the planner enums so
# the GUI can never offer a tag the validator rejects.  ``DEFAULT`` is excluded
# from the authorable tags (it == absence; the inherit sentinel covers it).
_LINE_MODE_TAGS: tuple[str, ...] = tuple(
    member.name for member in LineMode if member is not LineMode.DEFAULT
)
_BLIND_MODE_TAGS: tuple[str, ...] = tuple(
    member.name for member in BlindMode if member is not BlindMode.DEFAULT
)

# Combobox values for the primitive-mode picker, keyed by the derived edge type.
# The inherit sentinel leads each list so it is the natural "no override" choice.
_PRIMITIVE_VALUES_BY_TYPE: dict[str, tuple[str, ...]] = {
    _EDGE_TYPE_LINE: (_INHERIT_PRIMITIVE_LABEL, *_LINE_MODE_TAGS),
    _EDGE_TYPE_BLIND: (_INHERIT_PRIMITIVE_LABEL, *_BLIND_MODE_TAGS),
}

# ``direction`` override vocabulary (forward ``u->v`` only).  The inherit
# sentinel maps to an omitted key; ``front`` / ``rear`` map to those literals.
_DIRECTION_VALUES: tuple[str, ...] = (_INHERIT_LABEL, "front", "rear")
# ``stop_at_line`` override vocabulary (forward only).  Inherit -> omitted key.
_STOP_AT_LINE_VALUES: tuple[str, ...] = (_INHERIT_LABEL, "on", "off")

# ``PIPELINE DEFAULTS`` combobox vocabularies, single-sourced from the enums.
# NONE of the four categories offers ``DEFAULT``: the empty leading "unset"
# sentinel is the single way to mean "inherit the pipeline_config.py default"
# (== key absent), so a redundant ``DEFAULT`` member would be a second spelling
# of the same intent and is excluded everywhere (R12).
_PIPELINE_UNSET_LABEL: str = ""
_PIPELINE_LINE_VALUES: tuple[str, ...] = (
    _PIPELINE_UNSET_LABEL,
    *(member.name for member in LineMode if member is not LineMode.DEFAULT),
)
_PIPELINE_BLIND_VALUES: tuple[str, ...] = (
    _PIPELINE_UNSET_LABEL,
    *(member.name for member in BlindMode if member is not BlindMode.DEFAULT),
)
_PIPELINE_TURN_VALUES: tuple[str, ...] = (
    _PIPELINE_UNSET_LABEL,
    *(member.name for member in TurnMode if member is not TurnMode.DEFAULT),
)
_PIPELINE_DIRECTION_VALUES: tuple[str, ...] = (
    _PIPELINE_UNSET_LABEL,
    *(
        member.name
        for member in DirectionMode
        if member is not DirectionMode.DEFAULT
    ),
)

# The four pipeline-mode categories, as ``(which, label, values_tuple)`` triples
# in display order.  ``which`` is the string the command/contract keys on; it
# also names the model attribute ``pipeline_<which>_mode``.
_PIPELINE_MODE_SPECS: tuple[tuple[str, str, tuple[str, ...]], ...] = (
    ("line", "Line:", _PIPELINE_LINE_VALUES),
    ("blind", "Blind:", _PIPELINE_BLIND_VALUES),
    ("turn", "Turn:", _PIPELINE_TURN_VALUES),
    ("direction", "Direction:", _PIPELINE_DIRECTION_VALUES),
)

# Canonical flag name for the field-wide blind->projection auto-seek toggle.
_BLIND_PROJECTION_AUTO_SEEK_FLAG: str = "blind_projection_auto_seek"

# Edge-override list visible-row count (scrollable area height in pixels),
# matching the pivots-list idiom.
_EDGE_OVERRIDE_LIST_HEIGHT_PX: int = 84
# Edge-override add-row u/v combobox width (characters).
_EDGE_OVERRIDE_COMBO_WIDTH: int = 6
# Field-dimension entry width (characters).
_FIELD_MM_ENTRY_WIDTH: int = 7
# Field-dimension entry format string (millimetres, integer).
_FIELD_MM_FORMAT: str = "%d"

# Ghost-button text for the edge-overrides add affordance.
_GHOST_ADD_OVERRIDE_TEXT: str = "+ add override"
# Per-row edit / delete glyphs (mirroring the pivots-row delete glyph).
_OVERRIDE_EDIT_GLYPH: str = "e"
_OVERRIDE_DELETE_GLYPH: str = "✕"  # multiplication x as a delete glyph
# Directed-pair arrow rendered in an override row summary (forward-only).
_OVERRIDE_ARROW: str = "->"
# Empty-state hint shown when no overrides exist.
_OVERRIDE_EMPTY_HINT: str = "no edge overrides"

# Status message naming the one rejected primitive/stop combination (U2): a
# ``primitive_mode`` of ``BlindMode.STOP_AT_LINE`` (a seek bundle) together with
# ``stop_at_line='off'`` (force dead-reckon) directly contradict, so the inline
# editor refuses the Save and leaves the form open for correction.  Mirrors the
# validator's single anti-ambiguity gate (``validator._validate_edges_field``)
# so the GUI and the Save-to-disk validator never disagree.
_OVERRIDE_STOP_AT_LINE_CONFLICT_MSG: str = (
    "STOP_AT_LINE seek conflicts with stop_at_line=off -- drop one: keep "
    "STOP_AT_LINE (omit stop_at_line or set it on) to seek, or pick a "
    "non-seeking primitive to dead-reckon."
)

# Default mode-display text before the app sets anything.
_DEFAULT_MODE_TEXT: str = "Mode: --"
# Default selection-display text before the app sets anything.
_DEFAULT_SELECTION_TEXT: str = "Selection: none"
# Validation-badge text for the valid / invalid / unknown states.
_BADGE_VALID_TEXT: str = "Valid"
_BADGE_INVALID_TEXT: str = "Invalid"
_BADGE_UNKNOWN_TEXT: str = "--"

# ---------------------------------------------------------------------------
# Title-bar + status-strip text constants.
# ---------------------------------------------------------------------------

# Product wordmark shown at the head of the title bar.
_TITLE_TEXT: str = "WRO 2026 · Field Editor"
# Default filename shown in the title bar before the app sets a real one.
_DEFAULT_FILENAME_TEXT: str = "field_data.json"
# The dirty marker glyph painted in the title bar when there are unsaved edits.
_DIRTY_DOT_GLYPH: str = "●"  # solid circle
# The badge indicator glyph that precedes the validation word in the status
# strip; recoloured per validation state.
_BADGE_DOT_GLYPH: str = "⬤"  # large solid circle
# Fixed key-hint shown on the status strip's second line; clicking it opens the
# shortcuts dialog via ``on_show_shortcuts``.
_KEY_HINT_TEXT: str = "arrows = 0.2 cm · shift = 1 cm"

# ---------------------------------------------------------------------------
# Section-header / collapsible constants.
# ---------------------------------------------------------------------------

# Section-header labels are rendered UPPERCASE to approximate a small-caps look.
_SECTION_NODE_TITLE: str = "Node"
_SECTION_RESET_TITLE: str = "Reset spec"
_SECTION_PIVOTS_TITLE: str = "Pivots"
_SECTION_EDGE_OVERRIDES_TITLE: str = "Edge overrides"
_SECTION_PIPELINE_DEFAULTS_TITLE: str = "Pipeline defaults"
_SECTION_GLOBALS_TITLE: str = "Global knobs"
_SECTION_NODES_LIST_TITLE: str = "Nodes"
_SECTION_START_TITLE: str = "Start"
# Trailing rule character repeated after a section header to suggest a hairline.
_HEADER_RULE_HEIGHT_PX: int = 1
# Horizontal gap (pixels) between a collapsible header's chevron and its text.
_CHEVRON_GAP_PX: int = 4

# Ghost-button text.
_GHOST_ADD_PIVOT_TEXT: str = "+ add pivot"
# Thin border width (pixels) drawn around a ghost button at rest.
_GHOST_BUTTON_BORDER_PX: int = 1


# ---------------------------------------------------------------------------
# Theme constants.  Named ttk styles + font sizes + accent geometry.  Colours
# and font families come from the shared ``theme`` module; only the sizing
# literals live here so the code body stays free of magic numbers.
# ---------------------------------------------------------------------------

# Named ttk styles defined by :meth:`SidePanels._apply_theme` and assigned to
# the ttk widgets at build time so SidePanels reads as themed even before the
# app applies a root-level base theme.
_STYLE_BUTTON: str = "Editor.TButton"
_STYLE_COMBOBOX: str = "Editor.TCombobox"
_STYLE_SPINBOX: str = "Editor.TSpinbox"
_STYLE_MONO_SPINBOX: str = "Editor.Mono.TSpinbox"

# Base font sizes (points) for the panel's text roles.
_FONT_SIZE_UI: int = 10
_FONT_SIZE_MONO: int = 10
# The status surface reads a touch larger so it stays legible as the floor of
# the column; the badge is the largest, boldest line.
_FONT_SIZE_STATUS: int = 10
_FONT_SIZE_BADGE: int = 11
# The key-hint reads slightly smaller / muted than the badge line.
_FONT_SIZE_HINT: int = 9

# Mode-toolbar accent bar: a thin vertical terracotta strip on the left edge of
# the active mode's row.  Width in pixels; the inactive rows show a same-width
# strip painted in the panel surface tone so the rows stay aligned.
_ACCENT_BAR_WIDTH_PX: int = 3
# Horizontal gap (pixels) between the accent strip and its mode label.
_MODE_ROW_GAP_PX: int = 4
# Vertical padding (pixels) around each mode row so the toolbar breathes.
_MODE_ROW_PAD_Y: int = 1

# Highlight thickness (pixels) used to flatten the chrome on plain tk widgets
# (Listbox / Canvas / buttons) so they sit cleanly on the warm surface.
_FLAT_BORDER_PX: int = 0
# Sunken border (pixels) kept on the status surface so it still reads as a
# distinct sunk plane at the floor of the window.
_STATUS_BORDER_PX: int = 1
# Horizontal gap (pixels) between the status strip's segments.  The strip lays
# its badge / message / mode / selection / key-hint segments out left-to-right
# across the full-window width, so this spaces them instead of the old
# vertical-stack padding.
_STATUS_SEGMENT_GAP_PX: int = 14


def _theme_color(name: str, fallback: str) -> str:
    """Return ``theme.<name>`` if defined, otherwise *fallback*.

    Defensive accessor so a missing optional theme token (an in-progress
    palette edit, an older ``theme.py``) degrades to a sensible colour rather
    than raising :class:`AttributeError` during the headless panel build.

    :param name: Attribute name to read off the :mod:`theme` module.
    :param fallback: Colour string to use when the attribute is absent.
    :return: The resolved ``#rrggbb`` colour string.
    """
    return getattr(theme, name, fallback)


# ---------------------------------------------------------------------------
# SidePanels frame
# ---------------------------------------------------------------------------


class SidePanels(tk.Frame):
    """Chrome controller for the live field editor's true three-pane window.

    :class:`SidePanels` owns the four chrome band frames (title, mode rail,
    inspector, status) that frame the app's canvas.  The bands are built as
    children of the *master* parent so they are siblings of the canvas widget
    and share one grid; :class:`SidePanels` itself is a (never placed) frame
    that just holds the controller identity.

    Construction is pure widget-building: every callback attribute starts as
    ``None``, every Tk variable starts at its safe default, and no model is
    consulted.  The owning app is expected to:

    1. Instantiate the controller: ``panels = SidePanels(root)``.
    2. Place the chrome + canvas with the single placement entry point:
       ``panels.layout_three_pane(canvas.tk_widget)`` -- it grids the title and
       status bands full-width, the mode rail and inspector at fixed widths, and
       the canvas in the expanding centre cell.
    3. Assign every callback attribute (``on_node_select``,
       ``on_save_and_build``, etc.) to a closure that performs the real work.
    4. Call :meth:`refresh` once with the freshly loaded
       :class:`FieldModel`, and again after every model mutation.

    :ivar mode_var: ``tk.StringVar`` holding the active :class:`Mode`'s
        ``value``.  Read via ``Mode(self.mode_var.get())``.
    :ivar _title_frame: Full-width title band (wordmark + filename + dirty dot),
        gridded across the top row by :meth:`layout_three_pane`.
    :ivar _mode_frame: Fixed-width far-left MODES rail band, gridded into the
        left column of the middle row.
    :ivar _inspector_frame: Fixed-width right PROPERTIES inspector band, gridded
        into the right column of the middle row.
    :ivar _status_frame: Full-width status band, gridded across the bottom row.
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
        :attr:`on_node_type_change`, which the app treats as the clear-reset
        path.  The attribute exists so a future affordance can wire to it
        without changing this signature; it is never fired from here.
    :ivar on_add_pivot: Fired when the pivots **Add** button is clicked with
        all three node pickers chosen.  Arguments are ``(u, v, w)`` node names.
    :ivar on_delete_pivot: Fired when a pivot row's delete button is
        clicked.  Sole argument is the ``(u, v, w)`` triple to delete.
    :ivar on_global_knob_change: Fired when a global-knob widget commits.  For
        the ``blind_decay_enabled`` checkbutton the value is a ``bool``; for
        the float entries it is a ``float``.  Arguments are
        ``(knob_name, value)``.  A bad numeric parse sets a status message and
        does **not** fire.
    :ivar on_show_shortcuts: Fired when the status strip's key-hint area is
        clicked.  No arguments.  Defaults to ``None``; the app wires it to open
        the shortcuts dialog.
    """

    def __init__(self, master: tk.Misc) -> None:
        """Build the widget tree.

        No model is consulted here -- all widgets start in their default
        empty state and must be populated by a subsequent call to
        :meth:`refresh`.

        The four chrome bands (title, mode rail, inspector, status) are built
        as children of *master* so they are siblings of the app's canvas widget
        and can share one grid.  Construction does **not** place any band -- the
        app calls :meth:`layout_three_pane` once to grid every band plus the
        canvas into *master*.  :class:`SidePanels` itself remains a (never
        placed) :class:`tk.Frame` purely so it keeps a Tk widget identity for
        the smoke-test ``isinstance`` check; it is never the parent of a band.

        :param master: The parent Tk container (the root window) into which the
            four bands and the canvas are gridded.
        """
        super().__init__(master)
        # The shared parent every band frame is built into and gridded under by
        # :meth:`layout_three_pane`.  Held so the band builders can parent their
        # frame to it rather than to ``self`` (``self`` is never placed).
        self._parent: tk.Misc = master

        # ---- Resolved fonts --------------------------------------------
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
        self._font_title: tuple[str, int, str] = (
            theme.resolve_font(theme.FONT_DISPLAY),
            _theme_size("FONT_SIZE_TITLE", 16),
            "bold",
        )
        self._font_section: tuple[str, int, str] = (
            theme.resolve_font(theme.FONT_SANS),
            _theme_size("FONT_SIZE_SECTION", 9),
            "bold",
        )
        self._font_hint: tuple[str, int] = (
            theme.resolve_font(theme.FONT_SANS),
            _FONT_SIZE_HINT,
        )

        # Paint the panel column itself with the warm surface tone so it no
        # longer reads as default grey, even before the app applies a base
        # theme to the root.
        self.configure(background=theme.SURFACE)

        # Per-mode accent strips, populated by :meth:`_build_mode_toolbar`
        # and recoloured by :meth:`_update_mode_accent` on every ``mode_var``
        # write.  Keyed by the mode's ``value`` string.
        self._mode_accent_bars: dict[str, tk.Frame] = {}
        # Per-mode row frames (the strip + the radio button) keyed by mode
        # value, so :meth:`_update_mode_accent` can tint the active row's
        # background as well as its accent strip.
        self._mode_rows: dict[str, tk.Frame] = {}
        self._mode_radios: dict[str, tk.Radiobutton] = {}

        # Registry of collapsible sections, keyed by a stable id string, so the
        # header click handler can toggle the matching body.  Each entry holds
        # the body frame, the chevron label, and the BooleanVar tracking open
        # state.
        self._collapsibles: dict[str, _Collapsible] = {}

        # ---- Public callback attributes (assigned by the app after init) ----
        self.on_node_select: Optional[Callable[[Optional[str]], None]] = None
        self.on_node_type_change: Optional[Callable[[str, str], None]] = None
        self.on_node_coords_change: Optional[
            Callable[[str, float, float], None]
        ] = None
        # Inspector rename / delete: ``on_node_rename(old, new)`` commits an
        # in-place name edit; ``on_node_delete(name)`` removes the selected node.
        self.on_node_rename: Optional[Callable[[str, str], None]] = None
        self.on_node_delete: Optional[Callable[[str], None]] = None
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
        # ---- Edge-overrides / pipeline-defaults / extended-globals slots ----
        # Wired by the app (T8) to the new editor commands.  This panel only
        # surfaces the controls and fires the slots; it never instantiates or
        # dispatches a command itself.  Every slot defaults to ``None`` so the
        # panel still builds + refreshes standalone (headless smoke test).
        #
        # ``on_set_edge_override`` fires when the inline editor's Save commits
        # for a derived-type edge.  Arguments are
        # ``(edge_type, u, v, primitive_mode, direction, stop_at_line)`` where
        # ``edge_type`` is ``"LINE"`` / ``"BLIND"`` (derived, not operator-set),
        # ``u`` / ``v`` are node names, and the three override values are each a
        # string tag or ``None`` (``None`` == "inherit", key omitted).  The app
        # routes this to a ``SetEdgeOverride`` command.
        self.on_set_edge_override: Optional[
            Callable[
                [str, str, str, Optional[str], Optional[str], Optional[str]],
                None,
            ]
        ] = None
        # ``on_clear_edge_override`` fires when a row's delete (``x``) button is
        # clicked.  Arguments are ``(edge_type, u, v)`` -- the full
        # ``(type, u, v)`` identity triple.  Routed to ``ClearEdgeOverride``.
        self.on_clear_edge_override: Optional[
            Callable[[str, str, str], None]
        ] = None
        # ``on_pipeline_mode_change`` fires when a PIPELINE DEFAULTS combobox
        # commits.  Arguments are ``(which, value)`` where ``which`` is one of
        # ``"line"`` / ``"blind"`` / ``"turn"`` / ``"direction"`` and ``value``
        # is the chosen mode-member name or ``None`` (the unset sentinel, key
        # absent -> planner falls back to the pipeline_config.py default).
        # Routed to ``SetPipelineMode``.
        self.on_pipeline_mode_change: Optional[
            Callable[[str, Optional[str]], None]
        ] = None
        # ``on_field_wide_flag_change`` fires when a field-wide boolean toggle
        # commits.  Arguments are ``(flag, value)`` -- today only
        # ``flag == "blind_projection_auto_seek"``.  Routed to
        # ``SetFieldWideFlag``.
        self.on_field_wide_flag_change: Optional[
            Callable[[str, bool], None]
        ] = None
        # ``on_field_dimensions_change`` fires when the FIELD_MM width/height
        # entries commit a valid pair.  Arguments are ``(width_mm, height_mm)``
        # as floats.  A bad numeric parse sets a status message and does NOT
        # fire.  Routed to ``SetFieldDimensions``.
        self.on_field_dimensions_change: Optional[
            Callable[[float, float], None]
        ] = None
        # New slots (wired by the app in a later task).  Defaulted to None /
        # no-op like every other slot so the panel builds standalone.
        self.on_show_shortcuts: Optional[Callable[[], None]] = None

        # ---- Tk variables (single source of truth for widget state) -----
        self.mode_var = tk.StringVar(value=Mode.SELECT.value)
        self._selected_node_var = tk.StringVar(value="")
        # The editable name entry binds to its OWN var, kept distinct from the
        # canonical ``_selected_node_var`` (which the coords / type / reset
        # handlers read) so a half-typed rename never poses as the selection.
        self._name_edit_var = tk.StringVar(value="")
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
        # True while the message slot is painted with a validation error (so the
        # next VALID re-validate clears the stale complaint instead of leaving it
        # behind in the muted tone).  See :meth:`set_validation` / :meth:`set_status`.
        self._status_showing_error: bool = False

        # ---- Title-bar Tk variables ------------------------------------
        self._filename_var = tk.StringVar(value=_DEFAULT_FILENAME_TEXT)
        # The dirty dot glyph is shown / blanked rather than packed / unpacked
        # so it never shifts the filename's position.
        self._dirty_dot_var = tk.StringVar(value="")

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

        # ---- Edge-overrides add/edit form Tk variables (issue #79) -----
        # The inline editor sets the edge by u/v comboboxes; ``type`` is
        # auto-derived and shown read-only.  The three override controls each
        # default to their inherit sentinel (no override key).
        self._edge_override_u_var = tk.StringVar(value="")
        self._edge_override_v_var = tk.StringVar(value="")
        self._edge_override_type_var = tk.StringVar(value="")
        self._edge_override_primitive_var = tk.StringVar(
            value=_INHERIT_PRIMITIVE_LABEL
        )
        self._edge_override_direction_var = tk.StringVar(value=_INHERIT_LABEL)
        self._edge_override_stop_at_line_var = tk.StringVar(value=_INHERIT_LABEL)

        # ---- Pipeline-defaults Tk variables (issue #79) ----------------
        # One readonly-combobox StringVar per pipeline category, keyed by the
        # ``which`` string the contract / command uses.
        self._pipeline_mode_vars: dict[str, tk.StringVar] = {
            which: tk.StringVar(value=_PIPELINE_UNSET_LABEL)
            for which, _label, _values in _PIPELINE_MODE_SPECS
        }

        # ---- Extended global-knobs Tk variables (issue #79) ------------
        # Field-wide blind->projection auto-seek flag (default True) + the
        # FIELD_MM width/height millimetre entries.
        self._blind_projection_auto_seek_var = tk.BooleanVar(value=True)
        self._field_width_mm_var = tk.StringVar(value="")
        self._field_height_mm_var = tk.StringVar(value="")

        # The model's per-edge override overlay, cached from the most recent
        # :meth:`refresh` so the inline editor / graft methods can derive an
        # edge's ``type`` and look up an existing override without a back-ref to
        # the app.  ``None`` until the first refresh runs.
        self._edges_overlay: Optional[list[dict]] = None
        # Existing LINE / BLIND segment pairs (each a frozenset of two node
        # names) cached from refresh so the inline editor can derive the
        # ``type`` of the picked ``u`` / ``v`` edge.  Empty until first refresh.
        self._line_segment_pairs: set[frozenset[str]] = set()
        self._blind_segment_pairs: set[frozenset[str]] = set()
        # The override row frames keyed by their ``(type, u, v)`` identity, so
        # the graft's :meth:`prefill_edit_row` can highlight an existing override's
        # row and a later refresh can clear stale highlights.  Rebuilt every
        # refresh.
        self._edge_override_row_frames: dict[
            tuple[str, str, str], tk.Frame
        ] = {}
        # The identity of the override row currently highlighted by the graft, so
        # the next highlight / refresh can clear the prior terracotta ring.
        self._highlighted_override_row: Optional[tuple[str, str, str]] = None
        # Dirty-check signature of the override-list inputs as of the last
        # rebuild: a deep snapshot of ``model.edges`` plus the two segment-pair
        # sets.  :meth:`refresh` skips the (full-teardown) row rebuild when this
        # is unchanged, so a node nudge / coord edit -- which never touches edges
        # or segments -- does not needlessly destroy + re-create the whole widget
        # tree (R11).  ``None`` until the first rebuild seeds it.
        self._override_rows_signature: Optional[
            tuple[
                tuple[tuple[tuple[str, object], ...], ...],
                frozenset[frozenset[str]],
                frozenset[frozenset[str]],
            ]
        ] = None

        # Field bounds cached from refresh() so coord validation doesn't need
        # to round-trip through the app -> model on every keystroke.
        # ``(width_cm, height_cm)`` once the first refresh runs; ``None``
        # until then.
        self._field_extent_cm: Optional[tuple[float, float]] = None

        # The model's exact ``field_mm`` pair cached from the last refresh, kept
        # at FULL precision (the FIELD_MM entries DISPLAY it rounded to integer
        # millimetres via ``_FIELD_MM_FORMAT``).  The dimensions commit reads the
        # un-edited dimension from here rather than its rounded display var, so
        # editing only one dimension never truncates the other's sub-mm value
        # (R6).  ``None`` until the first refresh seeds it.
        self._field_mm: Optional[tuple[float, float]] = None

        # Re-entrancy guard: ``refresh`` writes to Tk variables, which would
        # otherwise re-fire user-callback bindings.  Setting this flag tells
        # the handlers to swallow events caused by programmatic updates.
        self._suppress_callbacks = False

        # ---- Define the named ttk styles before any widget is built ----
        # so the ttk Comboboxes / Spinboxes below can opt into them by name.
        # Guarded internally; a no-op if no live root exists.
        self._apply_theme()

        # ---- Build the four chrome bands (no placement yet) ------------
        # Each builder creates ONE band frame as a child of ``self._parent``
        # and stores it on ``self``; none of them grids/packs the band -- that
        # is :meth:`layout_three_pane`'s job, called once by the app.  Build
        # order here is irrelevant to the final layout (grid is positional), so
        # it just follows top-to-bottom reading order.
        self._build_title_bar()
        self._build_mode_rail()
        self._build_inspector_band()
        self._build_status_strip()

        # ---- Drive the mode-toolbar accent from ``mode_var`` -----------
        # A pure-visual trace: it only recolours the accent strips / row tint,
        # never fires a user callback, so it is safe to leave installed for the
        # panel's lifetime and needs no ``_suppress_callbacks`` gate.  Paint the
        # initial accent for the default mode now that the rows exist.
        self.mode_var.trace_add("write", self._on_mode_var_write)
        self._update_mode_accent()

    # ------------------------------------------------------------------
    # Theme application -- named ttk styles + the accent driver.
    # ------------------------------------------------------------------

    def _apply_theme(self) -> None:
        """Define the panel's named ``ttk.Style`` entries from the palette.

        This configures *named* styles (``Editor.TButton`` etc.) rather than
        touching the base theme or calling ``theme_use`` -- that root-level work
        belongs to the app.  Configuring named styles here means the ttk widgets
        in this panel look themed even when :class:`SidePanels` is built
        standalone under the default theme (the headless smoke test).

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
            background=[
                ("pressed", theme.TERRACOTTA),
                ("active", theme.TERRACOTTA),
            ],
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
        """``mode_var`` write trace: repaint the active-mode accent.

        Purely visual -- it never invokes a user ``on_*`` callback, so it is
        unaffected by (and does not need to consult) :attr:`_suppress_callbacks`.

        :param _args: Tk trace positional args (var name, index, op); unused.
        """
        self._update_mode_accent()

    def _update_mode_accent(self) -> None:
        """Highlight the active mode's row and clear every other.

        The active mode's left-edge strip is painted :data:`theme.ACCENT_BAR`
        and its row background is tinted :data:`theme.ACCENT_SOFT`; every other
        row's strip and background fall back to the panel surface tone so the
        rows stay aligned but only the active one shows the accent.  Safe to
        call before all rows exist (it iterates whatever is registered).
        """
        active = self.mode_var.get()
        accent_bar = _theme_color("ACCENT_BAR", theme.TERRACOTTA)
        accent_soft = _theme_color("ACCENT_SOFT", theme.SURFACE)
        for value, bar in self._mode_accent_bars.items():
            color = accent_bar if value == active else theme.SURFACE
            try:
                bar.configure(background=color)
            except tk.TclError:
                pass
        for value, row in self._mode_rows.items():
            tint = accent_soft if value == active else theme.SURFACE
            try:
                row.configure(background=tint)
            except tk.TclError:
                pass
            radio = self._mode_radios.get(value)
            if radio is not None:
                try:
                    radio.configure(background=tint, activebackground=tint)
                except tk.TclError:
                    pass

    # ------------------------------------------------------------------
    # Themed plain-tk widget factories
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

    def _ghost_button(self, master: tk.Misc, **kwargs) -> tk.Button:
        """Return a flat "ghost" :class:`tk.Button` using the ghost palette.

        At rest the button is borderless-feeling: muted text on the surface
        tone with a single hairline border; on hover / press it lifts to the
        terracotta accent like the other buttons so the affordance reads.

        :param master: Parent container.
        :param kwargs: Forwarded button options (``text`` / ``command`` etc.).
        :return: The configured ghost button.
        """
        kwargs.setdefault(
            "background", _theme_color("GHOST_BUTTON_BG", theme.SURFACE)
        )
        kwargs.setdefault(
            "foreground", _theme_color("GHOST_BUTTON_FG", theme.INK_MUTED)
        )
        kwargs.setdefault("activebackground", theme.TERRACOTTA)
        kwargs.setdefault("activeforeground", theme.PAPER)
        kwargs.setdefault(
            "highlightbackground",
            _theme_color("GHOST_BUTTON_BORDER", theme.SPINE),
        )
        kwargs.setdefault(
            "highlightcolor", _theme_color("GHOST_BUTTON_BORDER", theme.SPINE)
        )
        kwargs.setdefault("highlightthickness", _GHOST_BUTTON_BORDER_PX)
        kwargs.setdefault("borderwidth", 0)
        kwargs.setdefault("relief", "flat")
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

    def _section_header(
        self, master: tk.Misc, title: str
    ) -> tk.Frame:
        """Build a static small-caps section header + hairline rule.

        Renders *title* UPPERCASE in the muted section-header colour, with a
        thin hairline rule beneath it.  Used for the non-collapsible sections
        (Node, Nodes list, Start).

        :param master: Parent container the header packs into.
        :param title: Header text (rendered UPPERCASE).
        :return: The header container frame (already packed by the caller's
            placement; this only returns it for symmetry).
        """
        container = self._themed_frame(master)
        container.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))
        self._themed_label(
            container,
            text=title.upper(),
            anchor="w",
            foreground=_theme_color("SECTION_HEADER_FG", theme.INK_MUTED),
            font=self._font_section,
        ).pack(side="top", fill="x")
        rule = tk.Frame(
            container,
            height=_HEADER_RULE_HEIGHT_PX,
            background=_theme_color("HAIRLINE", theme.SPINE),
        )
        rule.pack(side="top", fill="x")
        return container

    def _collapsible_section(
        self, master: tk.Misc, key: str, title: str, *, start_open: bool
    ) -> tk.Frame:
        """Build a collapsible section: clickable header + toggleable body.

        The header is a small-caps title prefixed by a chevron glyph
        (:data:`theme.CHEVRON_OPEN` when expanded, :data:`theme.CHEVRON_COLLAPSED`
        when collapsed) and underlined by a hairline rule.  Clicking the header
        (or its chevron / label) toggles the body's visibility.  The returned
        body frame is where the caller packs the section's content.

        :param master: Parent container the section packs into.
        :param key: Stable id used to register the section in
            :attr:`_collapsibles`.
        :param title: Header text (rendered UPPERCASE).
        :param start_open: Whether the body is visible initially.
        :return: The body frame to populate.
        """
        outer = self._themed_frame(master)
        outer.pack(side="top", fill="x", pady=(_SECTION_SPACING, 0))

        header = self._themed_frame(outer)
        header.pack(side="top", fill="x")

        open_glyph = _theme_color("CHEVRON_OPEN", "◢")
        closed_glyph = _theme_color("CHEVRON_COLLAPSED", "▸")
        chevron = self._themed_label(
            header,
            text=open_glyph if start_open else closed_glyph,
            foreground=_theme_color("SECTION_HEADER_FG", theme.INK_MUTED),
            font=self._font_section,
        )
        chevron.pack(side="left", padx=(0, _CHEVRON_GAP_PX))
        title_label = self._themed_label(
            header,
            text=title.upper(),
            anchor="w",
            foreground=_theme_color("SECTION_HEADER_FG", theme.INK_MUTED),
            font=self._font_section,
        )
        title_label.pack(side="left", fill="x", expand=True)

        rule = tk.Frame(
            outer,
            height=_HEADER_RULE_HEIGHT_PX,
            background=_theme_color("HAIRLINE", theme.SPINE),
        )
        rule.pack(side="top", fill="x")

        body = self._themed_frame(outer)
        if start_open:
            body.pack(side="top", fill="x")

        record = _Collapsible(
            body=body,
            chevron=chevron,
            open_glyph=open_glyph,
            closed_glyph=closed_glyph,
            is_open=start_open,
        )
        self._collapsibles[key] = record

        # Make the whole header row clickable, not just one label.
        for widget in (header, chevron, title_label):
            widget.bind(
                "<Button-1>", lambda _e, k=key: self._toggle_collapsible(k)
            )

        return body

    def _toggle_collapsible(self, key: str) -> None:
        """Show / hide a collapsible section body and flip its chevron glyph.

        :param key: The :attr:`_collapsibles` id of the section to toggle.
        """
        record = self._collapsibles.get(key)
        if record is None:
            return
        if record.is_open:
            record.body.pack_forget()
            record.is_open = False
            record.chevron.configure(text=record.closed_glyph)
        else:
            record.body.pack(side="top", fill="x")
            record.is_open = True
            record.chevron.configure(text=record.open_glyph)

    # ------------------------------------------------------------------
    # Band builders: title / mode rail / inspector / status (built as
    # siblings of the canvas under ``self._parent``; placed by
    # ``layout_three_pane``).
    # ------------------------------------------------------------------

    def _build_title_bar(self) -> None:
        """Create the full-width title band: wordmark + filename + dirty dot.

        Builds :attr:`_title_frame` as a child of :attr:`_parent` (it is **not**
        placed here -- :meth:`layout_three_pane` grids it full-width across the
        top row).  It shows the product wordmark in the display font, the edited
        filename, and a dirty dot that appears only when there are unsaved edits
        (driven by :meth:`set_dirty`).
        """
        bar = self._themed_frame(self._parent)
        self._title_frame = bar

        self._themed_label(
            bar,
            text=_TITLE_TEXT,
            anchor="w",
            font=self._font_title,
        ).pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=(_PANEL_OUTER_PAD, 0),
        )

        meta_row = self._themed_frame(bar)
        meta_row.pack(side="top", fill="x", padx=_PANEL_OUTER_PAD)
        self._themed_label(
            meta_row,
            textvariable=self._filename_var,
            anchor="w",
            foreground=theme.INK_MUTED,
            font=self._font_hint,
        ).pack(side="left")
        # The dirty dot label always exists; its text is blanked when clean so
        # the layout never shifts.
        self._dirty_dot_label = self._themed_label(
            meta_row,
            textvariable=self._dirty_dot_var,
            foreground=_theme_color("DIRTY_DOT", theme.TERRACOTTA),
            font=self._font_hint,
        )
        self._dirty_dot_label.pack(side="right")

        # Hairline rule under the whole title strip.
        tk.Frame(
            bar,
            height=_HEADER_RULE_HEIGHT_PX,
            background=_theme_color("HAIRLINE", theme.SPINE),
        ).pack(side="top", fill="x", pady=(_PANEL_OUTER_PAD // 2, 0))

    def _build_mode_rail(self) -> None:
        """Create the far-left **MODES rail** band (fixed narrow width).

        Builds :attr:`_mode_frame` as a child of :attr:`_parent` and pins it to
        a fixed :data:`_MODE_RAIL_WIDTH_PX` width via ``grid_propagate(False)``
        so the rail stays just wide enough for the short mode labels + accent
        strip and never steals width from the expanding canvas.  The band is
        **not** placed here -- :meth:`layout_three_pane` grids it into the
        left column of the middle row.
        """
        rail = self._themed_frame(self._parent, width=_MODE_RAIL_WIDTH_PX)
        # Hold the fixed width regardless of how tall/narrow the rows are.  The
        # rail's only child is the pack-managed inner frame below, so
        # pack_propagate(False) -- not grid_propagate -- is what stops the band
        # from collapsing to its content width.
        rail.grid_propagate(False)
        rail.pack_propagate(False)
        self._mode_frame = rail

        # Inner frame so the rail's content keeps padding without fighting the
        # fixed-width outer band (a directly-padded fixed band would clip).
        inner = self._themed_frame(rail)
        inner.pack(
            side="top",
            fill="both",
            expand=True,
            padx=_PANEL_OUTER_PAD,
            pady=(_PANEL_OUTER_PAD, 0),
        )
        self._build_mode_toolbar(inner)

    def _build_inspector_band(self) -> None:
        """Create the right **PROPERTIES inspector** band (fixed wide width).

        Builds :attr:`_inspector_frame` as a child of :attr:`_parent` and pins
        it to a fixed :data:`_INSPECTOR_WIDTH_PX` width via
        ``grid_propagate(False)`` -- chosen wide enough that section headers and
        field labels are not clipped.  The band is **not** placed here --
        :meth:`layout_three_pane` grids it into the right column of the middle
        row.  Its contents (node fields, collapsible reset/pivots/globals stack,
        auto-create + start-pose forms, ghost add buttons, and the
        always-visible action block at the floor) are built by
        :meth:`_build_inspector`.
        """
        band = self._themed_frame(self._parent, width=_INSPECTOR_WIDTH_PX)
        # Hold the comfortable fixed width so labels never re-clip when content
        # changes; only the canvas column is allowed to expand.  The band's only
        # child is the pack-managed inner frame below, so pack_propagate(False)
        # -- not grid_propagate -- is what keeps it from shrinking to content.
        band.grid_propagate(False)
        band.pack_propagate(False)
        self._inspector_frame = band

        # Inner frame carries the section content with consistent padding while
        # the fixed-width band holds the column geometry.
        inner = self._themed_frame(band)
        inner.pack(
            side="top",
            fill="both",
            expand=True,
            padx=_PANEL_OUTER_PAD,
            pady=(_PANEL_OUTER_PAD, 0),
        )
        self._build_inspector(inner)

    def _build_mode_toolbar(self, master: tk.Misc) -> None:
        """Populate the left **MODES** toolbar column.

        A small-caps "MODES" header followed by one row per :class:`Mode`.
        Each row holds a thin left-edge accent strip (terracotta + soft tint
        for the active mode, surface-toned otherwise) and a radio button bound
        to the unchanged :attr:`mode_var`.  The accent is driven by the
        ``mode_var`` write trace installed in ``__init__``.

        :param master: The fixed-width toolbar column frame.
        """
        self._themed_label(
            master,
            text="MODES",
            anchor="w",
            foreground=_theme_color("SECTION_HEADER_FG", theme.INK_MUTED),
            font=self._font_section,
        ).pack(side="top", fill="x")
        tk.Frame(
            master,
            height=_HEADER_RULE_HEIGHT_PX,
            background=_theme_color("HAIRLINE", theme.SPINE),
        ).pack(side="top", fill="x")

        for mode, label in _MODE_LABELS:
            row = tk.Frame(master, background=theme.SURFACE)
            row.pack(side="top", fill="x", pady=_MODE_ROW_PAD_Y)
            self._mode_rows[mode.value] = row

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
                indicatoron=False,
                background=theme.SURFACE,
                foreground=theme.INK,
                activebackground=theme.SURFACE,
                activeforeground=theme.TERRACOTTA,
                selectcolor=theme.SURFACE,
                highlightthickness=_FLAT_BORDER_PX,
                borderwidth=0,
                font=self._font_ui,
            )
            btn.pack(
                side="left",
                fill="x",
                expand=True,
                padx=(_MODE_ROW_GAP_PX, 0),
            )
            self._mode_radios[mode.value] = btn

    def _build_inspector(self, master: tk.Misc) -> None:
        """Populate the right **PROPERTIES** inspector column.

        Packs, from top to bottom: a "PROPERTIES" header, the node list + node
        inspector, the collapsible reset-spec / pivots / global-knobs stack,
        the start-pose form, and -- pinned to the column floor so it is always
        visible -- the action-button block.

        :param master: The inspector column frame.
        """
        self._themed_label(
            master,
            text="PROPERTIES",
            anchor="w",
            foreground=_theme_color("SECTION_HEADER_FG", theme.INK_MUTED),
            font=self._font_section,
        ).pack(side="top", fill="x")
        tk.Frame(
            master,
            height=_HEADER_RULE_HEIGHT_PX,
            background=_theme_color("HAIRLINE", theme.SPINE),
        ).pack(side="top", fill="x")

        # The action block is pinned to the bottom FIRST so it always wins a
        # guaranteed slot at the column floor -- it can never be clipped below
        # the fold no matter how tall the sections above grow.
        self._build_action_buttons(master)

        # Everything else packs top-down above the pinned action block.
        self._build_node_list_and_inspector(master)
        self._build_reset_spec_panel(master)
        self._build_pivots_panel(master)
        # EDGE OVERRIDES sits directly below Pivots, mirroring its idiom; then
        # PIPELINE DEFAULTS, then the (extended) Global knobs section.
        self._build_edge_overrides_panel(master)
        self._build_pipeline_defaults_panel(master)
        self._build_global_knobs_panel(master)
        self._build_start_pose_panel(master)

    # ------------------------------------------------------------------
    # Inspector section builders
    # ------------------------------------------------------------------

    def _build_node_list_and_inspector(self, master: tk.Misc) -> None:
        """Create the **Node** section: list + name/type/coords inspector.

        :param master: Parent inspector column.
        """
        # --- Nodes list section ---
        self._section_header(master, _SECTION_NODES_LIST_TITLE)
        list_frame = self._themed_frame(master)
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

        # --- Node inspector section header ---
        self._section_header(master, _SECTION_NODE_TITLE)

        # --- Inspector: editable name entry + Delete button ---
        # The name entry renames the SELECTED node in place: committing it
        # (Return / focus-out) fires ``on_node_rename(old, new)``.  The Delete
        # button removes the selected node via ``on_node_delete(name)``.  Both
        # start disabled and are enabled only while a node is selected (see
        # :meth:`_apply_inspector` / :meth:`_clear_inspector`).
        name_row = self._themed_frame(master)
        name_row.pack(side="top", fill="x")
        self._themed_label(name_row, text="Name:").pack(side="left")
        # Pack the Delete button to the RIGHT first so it reserves its width;
        # the name entry then fills the gap between the label and the button.
        self._delete_node_button = self._themed_button(
            name_row,
            text="Delete",
            command=self._handle_node_delete,
        )
        self._delete_node_button.pack(side="right", padx=(6, 0))
        self._delete_node_button.configure(state=tk.DISABLED)
        self._name_entry = self._themed_entry(
            name_row,
            mono=False,
            textvariable=self._name_edit_var,
            state="disabled",
        )
        self._name_entry.pack(side="left", fill="x", expand=True)
        self._name_entry.bind("<Return>", self._handle_name_commit)
        self._name_entry.bind("<FocusOut>", self._handle_name_commit)

        # --- Inspector: node type combobox ---
        type_row = self._themed_frame(master)
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
        self._type_combo.bind("<<ComboboxSelected>>", self._handle_type_change)

        # --- Inspector: x / y editable entries (numeric -> mono font) ---
        coords_row = self._themed_frame(master)
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

        # --- Auto-create edges row ---
        autocreate_row = self._themed_frame(master)
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

        # Button sub-row: hosts the Auto-create button in idle mode and the
        # Accept / Reject pair in preview mode.  The state is toggled via
        # :meth:`set_auto_create_state`.
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

    def _build_reset_spec_panel(self, master: tk.Misc) -> None:
        """Create the collapsible **Reset spec** section.

        Holds an axis combobox, a wall-coordinate entry, and a facing spinbox.
        The whole sub-panel is enabled only while a ``reset``-typed node is
        selected (see :meth:`_apply_reset_spec` / :meth:`_clear_reset_spec`);
        committing any of the three widgets fires
        :attr:`on_reset_spec_change` with the parsed triple, mirroring the
        coordinate-entry commit pattern.

        :param master: Parent inspector column.
        """
        body = self._collapsible_section(
            master, "reset_spec", _SECTION_RESET_TITLE, start_open=False
        )

        axis_row = self._themed_frame(body)
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

        wall_row = self._themed_frame(body)
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

        facing_row = self._themed_frame(body)
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

    def _build_pivots_panel(self, master: tk.Misc) -> None:
        """Create the collapsible **Pivots** section: triples + add-row.

        The list area is a scrollable column of ``u -> v -> w`` rows, each with
        a hover-red delete button that fires :attr:`on_delete_pivot`.  Below it
        an add-row of three node-name comboboxes plus an **Add** button fires
        :attr:`on_add_pivot` once all three pickers hold a value, and a
        **+ add pivot** ghost button repeats that add affordance.  The list is
        rebuilt and the picker dropdowns repopulated on every :meth:`refresh`.

        :param master: Parent inspector column.
        """
        body = self._collapsible_section(
            master, "pivots", _SECTION_PIVOTS_TITLE, start_open=False
        )

        # --- Scrollable list of existing pivot triples ---
        list_frame = self._themed_frame(body, height=_PIVOT_LIST_HEIGHT_PX)
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
        add_row = self._themed_frame(body)
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

        # --- Ghost add-button row: + add pivot ---
        ghost_row = self._themed_frame(body)
        ghost_row.pack(side="top", fill="x", pady=(4, 0))
        self._add_pivot_ghost_button = self._ghost_button(
            ghost_row,
            text=_GHOST_ADD_PIVOT_TEXT,
            command=self._handle_add_pivot,
        )
        self._add_pivot_ghost_button.pack(side="left", fill="x", expand=True)

    def _build_edge_overrides_panel(self, master: tk.Misc) -> None:
        """Create the collapsible **Edge overrides** section (issue #79).

        Mirrors the Pivots list idiom exactly: a count badge ``(N)`` in the
        header, a scrollable column of override rows (each reading
        ``u -> v  [TYPE]  <tag summary>`` with an edit ``e`` + delete ``x``
        affordance), a ``+ add override`` ghost button, and an inline editor
        sub-frame that is hidden until the operator clicks ``+ add override`` or
        a row's ``e``.  The inline editor sets the edge by ``u`` / ``v``
        comboboxes, derives + locks the ``type``, and exposes the
        ``primitive_mode`` / ``direction`` / ``stop_at_line`` override controls;
        geometry (distance / heading) is never shown.

        The list is rebuilt and the picker dropdowns repopulated on every
        :meth:`refresh`.  This builder wires the widgets only; it never
        instantiates a command -- the app (T8) connects the callback slots to
        the ``SetEdgeOverride`` / ``ClearEdgeOverride`` commands.

        :param master: Parent inspector column.
        """
        body = self._collapsible_section(
            master,
            "edge_overrides",
            _SECTION_EDGE_OVERRIDES_TITLE,
            start_open=False,
        )
        # The count badge label rides in the header alongside the title; the
        # collapsible helper owns the chevron + title, so the badge is packed
        # into the section body's own header proxy here.  Keep it in the body so
        # it collapses with the section.  A dedicated label so refresh can update
        # just the count.
        self._edge_override_count_label = self._themed_label(
            body,
            text="(0)",
            anchor="e",
            foreground=_theme_color("SECTION_HEADER_FG", theme.INK_MUTED),
            font=self._font_section,
        )
        self._edge_override_count_label.pack(side="top", fill="x")

        # --- Scrollable list of existing override rows ---
        list_frame = self._themed_frame(
            body, height=_EDGE_OVERRIDE_LIST_HEIGHT_PX
        )
        list_frame.pack(side="top", fill="x")
        list_frame.pack_propagate(False)

        self._edge_override_canvas = tk.Canvas(
            list_frame,
            height=_EDGE_OVERRIDE_LIST_HEIGHT_PX,
            highlightthickness=_FLAT_BORDER_PX,
            background=theme.SURFACE,
        )
        override_scroll = tk.Scrollbar(
            list_frame,
            orient="vertical",
            command=self._edge_override_canvas.yview,
            background=theme.SURFACE,
            troughcolor=theme.PAPER,
            highlightthickness=_FLAT_BORDER_PX,
        )
        override_scroll.pack(side="right", fill="y")
        self._edge_override_canvas.pack(side="left", fill="both", expand=True)
        self._edge_override_canvas.configure(
            yscrollcommand=override_scroll.set
        )

        # Inner frame that actually holds the rows; rebuilt by ``refresh``.
        self._edge_override_rows_frame = self._themed_frame(
            self._edge_override_canvas
        )
        self._edge_override_canvas.create_window(
            (0, 0), window=self._edge_override_rows_frame, anchor="nw"
        )
        self._edge_override_rows_frame.bind(
            "<Configure>",
            lambda _event: self._edge_override_canvas.configure(
                scrollregion=self._edge_override_canvas.bbox("all")
            ),
        )

        # --- Ghost add-button row: + add override ---
        ghost_row = self._themed_frame(body)
        ghost_row.pack(side="top", fill="x", pady=(4, 0))
        self._add_override_ghost_button = self._ghost_button(
            ghost_row,
            text=_GHOST_ADD_OVERRIDE_TEXT,
            command=self._handle_open_add_override,
        )
        self._add_override_ghost_button.pack(side="left", fill="x", expand=True)

        # --- Inline editor sub-frame (hidden until add / edit) ---
        # Held packed-or-unpacked via :meth:`_show_override_editor` /
        # :meth:`_hide_override_editor`; built once here.
        self._edge_override_editor = self._themed_frame(body)

        # Edge row: u / v comboboxes + a read-only derived type label.
        edge_row = self._themed_frame(self._edge_override_editor)
        edge_row.pack(side="top", fill="x")
        self._themed_label(edge_row, text="Edge:").pack(side="left")
        self._edge_override_u_combo = ttk.Combobox(
            edge_row,
            textvariable=self._edge_override_u_var,
            values=[],
            state="readonly",
            width=_EDGE_OVERRIDE_COMBO_WIDTH,
            style=_STYLE_COMBOBOX,
        )
        self._edge_override_u_combo.pack(side="left", padx=(0, 6))
        self._edge_override_u_combo.bind(
            "<<ComboboxSelected>>", self._handle_override_edge_pick
        )
        self._themed_label(edge_row, text=_OVERRIDE_ARROW).pack(side="left")
        self._edge_override_v_combo = ttk.Combobox(
            edge_row,
            textvariable=self._edge_override_v_var,
            values=[],
            state="readonly",
            width=_EDGE_OVERRIDE_COMBO_WIDTH,
            style=_STYLE_COMBOBOX,
        )
        self._edge_override_v_combo.pack(side="left", padx=(6, 0))
        self._edge_override_v_combo.bind(
            "<<ComboboxSelected>>", self._handle_override_edge_pick
        )

        # Type row: a read-only derived-type label (never operator-editable).
        type_row = self._themed_frame(self._edge_override_editor)
        type_row.pack(side="top", fill="x")
        self._themed_label(type_row, text="Type:").pack(side="left")
        # A plain label (not a combobox) so the derived type can never be picked
        # by the operator; tinted by edge kind on derive.
        self._edge_override_type_label = self._themed_label(
            type_row,
            textvariable=self._edge_override_type_var,
            anchor="w",
            font=self._font_mono,
        )
        self._edge_override_type_label.pack(side="left", padx=(6, 0))

        # Primitive-mode row: combobox whose VALUES switch on the derived type.
        primitive_row = self._themed_frame(self._edge_override_editor)
        primitive_row.pack(side="top", fill="x")
        self._themed_label(primitive_row, text="Primitive:").pack(side="left")
        self._edge_override_primitive_combo = ttk.Combobox(
            primitive_row,
            textvariable=self._edge_override_primitive_var,
            values=list(_PRIMITIVE_VALUES_BY_TYPE[_EDGE_TYPE_LINE]),
            state="readonly",
            style=_STYLE_COMBOBOX,
        )
        self._edge_override_primitive_combo.pack(
            side="right", fill="x", expand=True
        )

        # Direction row.
        direction_row = self._themed_frame(self._edge_override_editor)
        direction_row.pack(side="top", fill="x")
        self._themed_label(direction_row, text="Direction:").pack(side="left")
        self._edge_override_direction_combo = ttk.Combobox(
            direction_row,
            textvariable=self._edge_override_direction_var,
            values=list(_DIRECTION_VALUES),
            state="readonly",
            style=_STYLE_COMBOBOX,
        )
        self._edge_override_direction_combo.pack(
            side="right", fill="x", expand=True
        )

        # Stop-at-line row.
        stop_row = self._themed_frame(self._edge_override_editor)
        stop_row.pack(side="top", fill="x")
        self._themed_label(stop_row, text="Stop@line:").pack(side="left")
        self._edge_override_stop_combo = ttk.Combobox(
            stop_row,
            textvariable=self._edge_override_stop_at_line_var,
            values=list(_STOP_AT_LINE_VALUES),
            state="readonly",
            style=_STYLE_COMBOBOX,
        )
        self._edge_override_stop_combo.pack(side="right", fill="x", expand=True)

        # A muted hint that geometry is auto-derived (never an editable field).
        self._themed_label(
            self._edge_override_editor,
            text="(distance / heading auto-derived)",
            anchor="w",
            foreground=theme.INK_MUTED,
            font=self._font_hint,
        ).pack(side="top", fill="x")

        # Save / Cancel buttons.
        buttons_row = self._themed_frame(self._edge_override_editor)
        buttons_row.pack(side="top", fill="x", pady=(4, 0))
        self._override_save_button = self._themed_button(
            buttons_row, text="save", command=self._handle_save_override
        )
        self._override_save_button.pack(side="left", fill="x", expand=True, padx=(0, 2))
        self._override_cancel_button = self._themed_button(
            buttons_row, text="cancel", command=self._handle_cancel_override
        )
        self._override_cancel_button.pack(
            side="left", fill="x", expand=True, padx=(2, 0)
        )

    def _build_pipeline_defaults_panel(self, master: tk.Misc) -> None:
        """Create the collapsible **Pipeline defaults** section (issue #79).

        Four mode comboboxes (Line / Blind / Turn / Direction).  Line / Blind /
        Turn cannot be ``DEFAULT`` (matches ``PipelineConfig.__post_init__``);
        Direction may.  Leaving a value at the empty "unset" sentinel keeps the
        corresponding field absent so the planner falls back to the
        ``pipeline_config.py`` default.  Each combobox commit fires
        :attr:`on_pipeline_mode_change` with the category ``which`` and the
        chosen value (or ``None`` for the unset sentinel).

        :param master: Parent inspector column.
        """
        body = self._collapsible_section(
            master,
            "pipeline_defaults",
            _SECTION_PIPELINE_DEFAULTS_TITLE,
            start_open=False,
        )

        # One labelled readonly combobox per pipeline category.  Capture
        # ``which`` in the lambda's default arg to dodge the late-binding trap.
        self._pipeline_mode_combos: dict[str, ttk.Combobox] = {}
        for which, label_text, values in _PIPELINE_MODE_SPECS:
            row = self._themed_frame(body)
            row.pack(side="top", fill="x")
            self._themed_label(row, text=label_text).pack(side="left")
            combo = ttk.Combobox(
                row,
                textvariable=self._pipeline_mode_vars[which],
                values=list(values),
                state="readonly",
                style=_STYLE_COMBOBOX,
            )
            combo.pack(side="right", fill="x", expand=True)
            combo.bind(
                "<<ComboboxSelected>>",
                lambda _event, key=which: self._handle_pipeline_mode_change(
                    key
                ),
            )
            self._pipeline_mode_combos[which] = combo

    def _build_global_knobs_panel(self, master: tk.Misc) -> None:
        """Create the collapsible **Global knobs** section (starts collapsed).

        Exposes the blind-decay toggle (a checkbutton bound to a
        :class:`tk.BooleanVar`) and one float entry per knob in
        :data:`_FLOAT_KNOB_LABELS`.  Each widget commit fires
        :attr:`on_global_knob_change` with the canonical knob name and the
        parsed value; a bad float parse sets a status message and does not
        fire.

        :param master: Parent inspector column.
        """
        body = self._collapsible_section(
            master, "global_knobs", _SECTION_GLOBALS_TITLE, start_open=False
        )

        toggle_row = self._themed_frame(body)
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
            row = self._themed_frame(body)
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

        # --- NEW (issue #79): field-wide blind->projection auto-seek toggle ---
        auto_seek_row = self._themed_frame(body)
        auto_seek_row.pack(side="top", fill="x")
        self._blind_projection_auto_seek_check = tk.Checkbutton(
            auto_seek_row,
            text="blind_projection_auto_seek",
            variable=self._blind_projection_auto_seek_var,
            anchor="w",
            command=self._handle_blind_projection_auto_seek_toggle,
            background=theme.SURFACE,
            foreground=theme.INK,
            activebackground=theme.SURFACE,
            activeforeground=theme.TERRACOTTA,
            selectcolor=theme.PAPER_LIGHT,
            highlightthickness=_FLAT_BORDER_PX,
            font=self._font_ui,
        )
        self._blind_projection_auto_seek_check.pack(side="left", fill="x")

        # --- NEW (issue #79): FIELD_MM width / height entries (no MAP_PNG) ---
        # A single row: "Field size  W[..] H[..] mm" matching the sidecar.
        field_mm_row = self._themed_frame(body)
        field_mm_row.pack(side="top", fill="x")
        self._themed_label(field_mm_row, text="Field size:").pack(side="left")
        self._themed_label(field_mm_row, text="mm").pack(side="right")
        self._field_height_entry = self._themed_entry(
            field_mm_row,
            mono=True,
            textvariable=self._field_height_mm_var,
            width=_FIELD_MM_ENTRY_WIDTH,
        )
        self._field_height_entry.pack(side="right", padx=(0, 4))
        self._themed_label(field_mm_row, text="H").pack(side="right")
        self._field_width_entry = self._themed_entry(
            field_mm_row,
            mono=True,
            textvariable=self._field_width_mm_var,
            width=_FIELD_MM_ENTRY_WIDTH,
        )
        self._field_width_entry.pack(side="right", padx=(6, 4))
        self._themed_label(field_mm_row, text="W").pack(side="right")
        for entry in (self._field_width_entry, self._field_height_entry):
            entry.bind("<Return>", self._handle_field_dimensions_commit)
            entry.bind("<FocusOut>", self._handle_field_dimensions_commit)

    def _build_start_pose_panel(self, master: tk.Misc) -> None:
        """Create the **Start** section: node combobox + heading spinbox.

        :param master: Parent inspector column.
        """
        self._section_header(master, _SECTION_START_TITLE)

        node_row = self._themed_frame(master)
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

        heading_row = self._themed_frame(master)
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

    def _build_action_buttons(self, master: tk.Misc) -> None:
        """Create the bottom 2x2 grid of action buttons.

        Pinned to the bottom of the inspector column so it is ALWAYS visible --
        it can never be pushed below the fold by tall sections above it.  Save &
        Build is terracotta-filled to mark it as the primary action.

        :param master: Parent inspector column.
        """
        group = self._themed_frame(master)
        group.pack(side="bottom", fill="x", pady=(_SECTION_SPACING, 0))

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

        # Save & Build is the primary action: terracotta fill, paper text, and
        # a paper active state so it stays the loudest button in the block.
        self._save_build_button = self._themed_button(
            group,
            text="Save & Build",
            command=self._handle_save_and_build,
            background=theme.TERRACOTTA,
            foreground=theme.PAPER,
            activebackground=theme.PAPER,
            activeforeground=theme.TERRACOTTA,
        )
        self._save_build_button.grid(
            row=1, column=1, sticky="ew", padx=2, pady=2
        )

        # Buttons start disabled until the app says otherwise.
        self.set_undo_enabled(False)
        self.set_redo_enabled(False)

    def _build_status_strip(self) -> None:
        """Create the full-width status band as a single horizontal row.

        Builds :attr:`_status_frame` as a child of :attr:`_parent` (it is
        **not** placed here -- :meth:`layout_three_pane` grids it full-width
        across the bottom row).  The strip is wide and short, so its segments
        lay out **left-to-right** to use that width instead of stacking short
        lines vertically::

            [<dot> Valid]   [message / validation error -- expands]   [Mode  Sel]   [key-hint]

        The validation badge (a coloured indicator dot + a Valid/Invalid word)
        anchors the left and reads as the dominant element; the free-form
        message / validation-error slot takes the elastic middle on ONE line;
        the active mode and selection ride as compact chips on the right,
        followed by the clickable key-hint that opens the shortcuts dialog via
        :attr:`on_show_shortcuts`.  The segments are driven by the public
        setters :meth:`set_validation_badge` / :meth:`set_validation`,
        :meth:`set_mode_display`, :meth:`set_selection_display`, and
        :meth:`set_status`.
        """
        group = self._themed_frame(
            self._parent, relief="sunken", bd=_STATUS_BORDER_PX
        )
        self._status_frame = group

        # Inner frame carries the row with consistent padding; the band's own
        # edges are flush so the sunken relief reads as a full-width plane.
        body = self._themed_frame(group)
        body.pack(
            side="top",
            fill="x",
            padx=_PANEL_OUTER_PAD,
            pady=_PANEL_OUTER_PAD // 2,
        )

        # --- LEFT: the validation badge (dot + word), the dominant element ---
        # The indicator dot is a separate label so it can be recoloured per
        # validation state without disturbing the word next to it.
        self._badge_dot_label = self._themed_label(
            body,
            text=_BADGE_DOT_GLYPH,
            foreground=theme.INK_MUTED,
            font=self._font_badge,
        )
        self._badge_dot_label.pack(side="left", padx=(0, _CHEVRON_GAP_PX))
        # The validation word ("Valid" / "Invalid" / "--").  Kept on its own
        # label so it can be recoloured alongside the dot.
        self._badge_label = self._themed_label(
            body,
            textvariable=self._badge_var,
            anchor="w",
            foreground=theme.INK_MUTED,
            font=self._font_badge,
        )
        self._badge_label.pack(side="left", padx=(0, _STATUS_SEGMENT_GAP_PX))

        # --- RIGHT: key-hint, then mode / selection chips.  These are packed
        #     BEFORE the elastic middle so they hug the right edge; pack order
        #     is right-to-left, so the key-hint ends up furthest right. ---
        self._key_hint_label = self._themed_label(
            body,
            text=_KEY_HINT_TEXT,
            anchor="e",
            foreground=theme.INK_MUTED,
            font=self._font_hint,
            cursor="hand2",
        )
        self._key_hint_label.pack(side="right", padx=(_STATUS_SEGMENT_GAP_PX, 0))
        self._key_hint_label.bind("<Button-1>", self._handle_show_shortcuts)

        self._selection_label = self._themed_label(
            body,
            textvariable=self._selection_display_var,
            anchor="e",
            font=self._font_status,
        )
        self._selection_label.pack(side="right", padx=(_STATUS_SEGMENT_GAP_PX, 0))

        self._mode_label = self._themed_label(
            body,
            textvariable=self._mode_display_var,
            anchor="e",
            font=self._font_status,
        )
        self._mode_label.pack(side="right", padx=(_STATUS_SEGMENT_GAP_PX, 0))

        # --- MIDDLE: the free-form message / validation-error slot.  Packed
        #     LAST with ``expand`` so it absorbs the slack between the badge and
        #     the right-hand chips.  It stays on ONE line (no wraplength) -- the
        #     wide strip means a message reads across, not down; an over-long
        #     message clips at the slot edge rather than growing the band tall. ---
        self._status_label = self._themed_label(
            body,
            textvariable=self._status_var,
            anchor="w",
            justify="left",
            foreground=theme.INK_MUTED,
            font=self._font_status,
        )
        self._status_label.pack(side="left", fill="x", expand=True)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def layout_three_pane(self, canvas_widget: tk.Widget) -> None:
        """Grid the four chrome bands + *canvas_widget* into the shared parent.

        This is the **single placement entry point** for the editor chrome.  It
        grids a true three-pane window into :attr:`_parent`::

            row 0:  [ ---------------- TITLE band (full width) ---------------- ]
            row 1:  [ MODE rail ] [ ------ CANVAS (expands) ------ ] [ INSPECTOR ]
            row 2:  [ ---------------- STATUS band (full width) --------------- ]

        Only the canvas row/column carries weight, so the title and status bands
        stay full-width, the mode rail and inspector hold their fixed widths,
        and the canvas absorbs every spare pixel on resize.  The app replaces
        its old ``panels.pack(...)`` + ``canvas.tk_widget.pack(...)`` calls with
        a single call to this method.

        :param canvas_widget: The app's canvas Tk widget (a sibling of the four
            bands under the same parent) to seat in the centre cell.
        """
        self._title_frame.grid(
            row=0, column=0, columnspan=3, sticky="ew"
        )
        self._mode_frame.grid(row=1, column=0, sticky="ns")
        canvas_widget.grid(row=1, column=1, sticky="nsew")
        self._inspector_frame.grid(row=1, column=2, sticky="ns")
        self._status_frame.grid(
            row=2, column=0, columnspan=3, sticky="ew"
        )

        # Only the canvas row/column expand; the rail + inspector keep their
        # fixed widths and the title + status bands keep their natural heights.
        self._parent.grid_rowconfigure(1, weight=1)
        self._parent.grid_columnconfigure(1, weight=1)

    def refresh(self, model: "FieldModel") -> None:
        """Re-sync every widget from *model*.

        Should be called once at app start and after every model mutation.
        This method intentionally does **not** fire any of the ``on_*``
        callbacks: callbacks are user-driven, while :meth:`refresh` is driven
        by programmatic state replays (model load, undo, redo, etc.).

        The method preserves the listbox selection when possible: if the
        previously selected node still exists in the refreshed model, it is
        reselected; otherwise the selection (and the inspector) is cleared.

        :param model: The current :class:`FieldModel` to render.
        """
        self._suppress_callbacks = True
        try:
            # Cache the field bounds (cm) so coord-entry validation can read
            # them without a back-reference to the model.
            width_mm, height_mm = model.field_mm
            self._field_extent_cm = (
                float(width_mm) / 10.0,
                float(height_mm) / 10.0,
            )
            # Cache the exact (full-precision) field_mm pair so the dimensions
            # commit can carry forward the un-edited dimension's true value rather
            # than the rounded display var (R6).
            self._field_mm = (float(width_mm), float(height_mm))

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

            # --- Edge overrides: cache the overlay + segment pairs, rebuild the
            #     rows + count badge, and repopulate the add-row u/v pickers. ---
            self._edges_overlay = [dict(entry) for entry in model.edges]
            self._line_segment_pairs = {
                frozenset(seg) for seg in model.line_segments
            }
            self._blind_segment_pairs = {
                frozenset(edge) for edge in model.blind_edges
            }
            # Dirty-check (R11): the override-list widget tree depends ONLY on
            # ``model.edges`` (the rows) and the line/blind segment-pair sets
            # (which gate whether a pair is overridable).  A node nudge / coord
            # edit leaves all three untouched, so a full row teardown + rebuild
            # is wasted work -- skip it and PRESERVE ``_edge_override_row_frames``
            # so the edge-mode graft (which reads it post-refresh) still resolves.
            new_signature = (
                self._edges_signature(model.edges),
                frozenset(self._line_segment_pairs),
                frozenset(self._blind_segment_pairs),
            )
            if new_signature != self._override_rows_signature:
                self._rebuild_edge_override_rows(model)
                self._override_rows_signature = new_signature
            for combo in (
                self._edge_override_u_combo,
                self._edge_override_v_combo,
            ):
                combo.configure(values=sorted_names)
            # A model replay (load / undo / redo) closes the inline editor so a
            # half-entered override never survives the state change.
            self._hide_override_editor()

            # --- Pipeline defaults: sync each combobox from the model. ---
            for which, _label, _values in _PIPELINE_MODE_SPECS:
                stored = getattr(model, f"pipeline_{which}_mode")
                self._pipeline_mode_vars[which].set(
                    stored if stored is not None else _PIPELINE_UNSET_LABEL
                )

            # --- Global knobs: sync the checkbutton + float entries. ---
            self._blind_decay_enabled_var.set(bool(model.blind_decay_enabled))
            for knob_name, _label in _FLOAT_KNOB_LABELS:
                value = float(getattr(model, knob_name))
                self._float_knob_vars[knob_name].set(_COORD_FORMAT % value)

            # --- Extended globals (issue #79): auto-seek flag + FIELD_MM. ---
            self._blind_projection_auto_seek_var.set(
                bool(model.blind_projection_auto_seek)
            )
            field_width_mm, field_height_mm = model.field_mm
            self._field_width_mm_var.set(_FIELD_MM_FORMAT % round(field_width_mm))
            self._field_height_mm_var.set(
                _FIELD_MM_FORMAT % round(field_height_mm)
            )

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
        """Replace the free-form message slot with *text* in the normal tone.

        This drives the elastic middle segment of the status strip.  A status
        message is a transient, neutral note, so it is painted in the muted ink
        tone and clears the validation-error flag -- the next VALID re-validate
        then has nothing stale to wipe.

        :param text: Status message to display.
        """
        self._status_var.set(text)
        self._status_label.configure(foreground=theme.INK_MUTED)
        self._status_showing_error = False

    def set_validation(self, ok: bool, detail: str = "") -> None:
        """Reflect a full-model validation result in the badge AND message slot.

        Drives the per-move validation feedback: the app re-validates the whole
        model after every mutation and calls this.  It recolours the badge via
        :meth:`set_validation_badge` and, when *ok* is ``False``, paints the
        message slot with the validator's first-error *detail* in the INVALID
        tone.  When *ok* is ``True`` it restores the slot to the normal tone and
        -- only if a validation error was showing -- clears that stale complaint
        back to the default text, so fixing the offending field does not leave
        the old message behind.  A transient :meth:`set_status` note set by the
        same edit is left untouched.

        :param ok: ``True`` if ``model.validate()`` passed, ``False`` otherwise.
        :param detail: The validator's first-error message, shown when invalid.
        """
        self.set_validation_badge(ok)
        if ok:
            if self._status_showing_error:
                self._status_var.set(_DEFAULT_STATUS_TEXT)
                self._status_showing_error = False
            self._status_label.configure(foreground=theme.INK_MUTED)
        else:
            self._status_var.set(detail)
            self._status_label.configure(
                foreground=_theme_color("STATUS_INVALID", theme.INVALID)
            )
            self._status_showing_error = True

    def set_mode_display(self, text: str) -> None:
        """Replace the status strip's **mode** line with *text*.

        Called by the app whenever the active :class:`Mode` changes so the
        strip always names the current editing mode.

        :param text: Mode description (e.g. ``"Mode: Select"``).
        """
        self._mode_display_var.set(text)

    def set_selection_display(self, text: str) -> None:
        """Replace the status strip's **selection** line with *text*.

        Called by the app whenever the current selection changes so the strip
        always names the selected node (or "none").

        :param text: Selection description (e.g. ``"Selection: A1"``).
        """
        self._selection_display_var.set(text)

    def set_validation_badge(self, ok: bool) -> None:
        """Set the validation badge to a valid / invalid state.

        Updates the badge **word** and recolours both its indicator dot and the
        word: a valid result is shown in :data:`theme.STATUS_VALID`, an invalid
        one in :data:`theme.STATUS_INVALID`.  The badge owns dedicated
        :class:`tk.Label` widgets so recolouring it does not disturb the other
        status lines.

        :param ok: ``True`` to show the valid badge, ``False`` for invalid.
        """
        valid_color = _theme_color("STATUS_VALID", theme.VALID)
        invalid_color = _theme_color("STATUS_INVALID", theme.INVALID)
        color = valid_color if ok else invalid_color
        self._badge_var.set(_BADGE_VALID_TEXT if ok else _BADGE_INVALID_TEXT)
        self._badge_label.configure(foreground=color)
        self._badge_dot_label.configure(foreground=color)

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

        Idle mode shows only the **Auto-create edges** button.  Preview mode
        hides it and shows **Accept** + **Reject** instead.  The two callback
        slots :attr:`on_accept_proposed` / :attr:`on_reject_proposed` wire to
        those buttons.

        :param mode: ``"idle"`` or ``"preview"``.
        :raises ValueError: If *mode* is not one of the two strings.
        """
        if mode == "idle":
            self._accept_button.pack_forget()
            self._reject_button.pack_forget()
            self._auto_create_button.pack(side="left", fill="x", expand=True)
        elif mode == "preview":
            self._auto_create_button.pack_forget()
            self._accept_button.pack(
                side="left", fill="x", expand=True, padx=(0, 2)
            )
            self._reject_button.pack(
                side="left", fill="x", expand=True, padx=(2, 0)
            )
        else:
            raise ValueError(
                f"set_auto_create_state: mode must be 'idle' or 'preview', "
                f"got {mode!r}"
            )

    def set_dirty(self, dirty: bool) -> None:
        """Show or hide the title-bar unsaved-changes dot.

        Wired to the app's dirty tracking: the app calls ``set_dirty(True)``
        whenever the model is mutated and ``set_dirty(False)`` after a
        successful Save.  The dot's text is blanked rather than the label
        unpacked, so the filename's position never shifts.

        :param dirty: ``True`` to show the dirty dot, ``False`` to hide it.
        """
        self._dirty_dot_var.set(_DIRTY_DOT_GLYPH if dirty else "")

    def set_filename(self, filename: str) -> None:
        """Set the filename shown in the title bar.

        :param filename: The display name of the edited file (e.g.
            ``"field_data.json"``).
        """
        self._filename_var.set(filename)

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
        self._name_edit_var.set(node_name)
        self._selected_type_var.set(model.node_types.get(node_name, ""))
        x, y = model.nodes[node_name]
        self._selected_x_var.set(_COORD_FORMAT % float(x))
        self._selected_y_var.set(_COORD_FORMAT % float(y))
        self._type_combo.configure(state="readonly")
        self._x_entry.configure(state="normal")
        self._y_entry.configure(state="normal")
        self._name_entry.configure(state="normal")
        self._delete_node_button.configure(state=tk.NORMAL)

    def _clear_inspector(self) -> None:
        """Reset the inspector to its empty / no-selection state."""
        self._selected_node_var.set("")
        self._name_edit_var.set("")
        self._selected_type_var.set("")
        self._selected_x_var.set("")
        self._selected_y_var.set("")
        self._type_combo.configure(state="disabled")
        self._x_entry.configure(state="disabled")
        self._y_entry.configure(state="disabled")
        self._name_entry.configure(state="disabled")
        self._delete_node_button.configure(state=tk.DISABLED)

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
            self._reset_wall_coord_var.set(
                _RESET_COORD_FORMAT % float(wall_coord)
            )

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
        delete).  Each row shows ``u -> v -> w`` and a hover-red delete button
        whose command captures its own triple.

        :param model: Source of truth for ``pivot_turns``.
        """
        for child in self._pivot_rows_frame.winfo_children():
            child.destroy()

        delete_fg = _theme_color("GHOST_BUTTON_FG_ACCENT", theme.TERRACOTTA)
        for turn in model.pivot_turns:
            triple = tuple(turn)
            row = self._themed_frame(self._pivot_rows_frame)
            row.pack(side="top", fill="x")
            label_text = f" {_PIVOT_ARROW} ".join(
                str(part) for part in triple
            )
            self._themed_label(
                row,
                text=label_text,
                anchor="w",
                font=self._font_mono,
            ).pack(side="left", fill="x", expand=True)
            # A hover-red delete glyph button.  Capture the triple in the
            # default arg to dodge the late-binding closure trap (every button
            # would otherwise see the last triple).
            delete_button = self._themed_button(
                row,
                text="✕",  # multiplication x as a delete glyph
                command=lambda t=triple: self._handle_delete_pivot(t),
                background=theme.SURFACE,
                foreground=theme.INK_MUTED,
                activebackground=delete_fg,
                activeforeground=theme.PAPER,
            )
            delete_button.pack(side="right")

        # Recompute the scrollregion now that the row count changed.
        self._pivot_rows_frame.update_idletasks()
        self._pivot_canvas.configure(
            scrollregion=self._pivot_canvas.bbox("all")
        )

    # ------------------------------------------------------------------
    # Edge-overrides helpers (issue #79)
    # ------------------------------------------------------------------

    def _derive_edge_type(self, u: str, v: str) -> Optional[str]:
        """Return the edge ``type`` of the existing ``u`` / ``v`` segment.

        The override ``type`` is never operator-chosen -- it is derived from the
        matching existing ``LINE_SEGMENTS`` / ``BLIND_EDGES`` pair (undirected,
        so ``u`` / ``v`` order does not matter) cached on the last
        :meth:`refresh`.  Returns ``"LINE"`` / ``"BLIND"`` if exactly one kind
        matches the pair, or ``None`` if the pair is not an existing segment of
        either kind (or ``u`` / ``v`` are empty / identical), so the inline
        editor can reject an add that has no segment to attach to.

        :param u: First node name.
        :param v: Second node name.
        :return: ``"LINE"`` / ``"BLIND"`` or ``None`` when no segment matches.
        """
        if not u or not v or u == v:
            return None
        pair = frozenset((u, v))
        is_line = pair in self._line_segment_pairs
        is_blind = pair in self._blind_segment_pairs
        # A pair that is BOTH a line and a blind segment is ambiguous; the
        # operator must disambiguate via the canvas/edge-mode graft, so report
        # None here rather than silently guessing a type.
        if is_line and not is_blind:
            return _EDGE_TYPE_LINE
        if is_blind and not is_line:
            return _EDGE_TYPE_BLIND
        return None

    @staticmethod
    def _override_identity(entry: dict) -> tuple[str, str, str]:
        """Return the ``(type, u, v)`` identity triple of an overlay *entry*.

        The single place panels derives a row / dirty-snapshot key from an
        overlay dict, mirroring the planner's directed-typed identity rule
        (``commands._find_override_index``).  ``str``-coerces each component so a
        legacy non-string value never produces a tuple a later string lookup
        cannot match.

        :param entry: An EDGES overlay dict ``{type, u, v, + optional keys}``.
        :return: The ``(type, u, v)`` identity triple, each component a ``str``.
        """
        return (
            str(entry.get("type", "")),
            str(entry.get("u", "")),
            str(entry.get("v", "")),
        )

    @staticmethod
    def _edges_signature(
        edges: list[dict],
    ) -> tuple[tuple[tuple[str, object], ...], ...]:
        """Return a hashable, order-sensitive snapshot of an ``edges`` overlay.

        Each overlay entry (a plain dict) becomes a tuple of its
        ``(key, value)`` items sorted by key, so two refreshes whose
        ``model.edges`` are content-equal (same entries, same order, same keys
        and values) produce the SAME signature -- the basis for the dirty-check
        that skips a needless full row-rebuild (R11).  Entry order is preserved
        (a reordered overlay is a real change worth re-rendering); only
        per-entry key order is normalised.

        :param edges: The model's ``edges`` overlay list.
        :return: A nested tuple usable as a comparison key.
        """
        return tuple(
            tuple(sorted(entry.items(), key=lambda kv: kv[0]))
            for entry in edges
        )

    def _rebuild_edge_override_rows(self, model: "FieldModel") -> None:
        """Rebuild the edge-override list rows + count badge from ``model.edges``.

        Every existing row widget is destroyed and re-created so the list always
        mirrors the model exactly (no stale rows survive an undo / delete).  Each
        row shows ``u -> v  [TYPE]`` with an edit (``e``) and delete (``x``)
        affordance, mirroring a pivot row.  The override's tag values are seen by
        clicking edit; keeping the row short stops a long summary from pushing the
        buttons off the (vertical-only-scrolling) canvas.  The ``(type, u, v)``
        identity of each row is recorded in :attr:`_edge_override_row_frames` so
        the graft's :meth:`prefill_edit_row` can highlight it.

        :param model: Source of truth for ``edges``.
        """
        for child in self._edge_override_rows_frame.winfo_children():
            child.destroy()
        self._edge_override_row_frames = {}
        # A fresh row set drops any prior highlight (the widget is gone).
        self._highlighted_override_row = None

        accent_fg = _theme_color("GHOST_BUTTON_FG_ACCENT", theme.TERRACOTTA)
        for entry in model.edges:
            identity = self._override_identity(entry)
            edge_type, u, v = identity

            row = self._themed_frame(self._edge_override_rows_frame)
            row.pack(side="top", fill="x")
            self._edge_override_row_frames[identity] = row

            # The directed pair + bracketed type + tag summary.  The type tag is
            # tinted by kind (BLIND -> brick, LINE/other -> ink) per the sidecar.
            tag_color = (
                theme.BRICK if edge_type == _EDGE_TYPE_BLIND else theme.INK
            )
            self._themed_label(
                row,
                text=f"{u}{_OVERRIDE_ARROW}{v}",
                anchor="w",
                font=self._font_mono,
            ).pack(side="left")
            self._themed_label(
                row,
                text=f"[{edge_type}]",
                anchor="w",
                foreground=tag_color,
                font=self._font_mono,
            ).pack(side="left", padx=(6, 0))

            # Per-row delete (x) then edit (e), packed right-to-left so edit ends
            # up left of delete.  Capture the identity in default args to dodge
            # the late-binding closure trap.
            delete_button = self._themed_button(
                row,
                text=_OVERRIDE_DELETE_GLYPH,
                command=lambda i=identity: self._handle_delete_override(i),
                background=theme.SURFACE,
                foreground=theme.INK_MUTED,
                activebackground=accent_fg,
                activeforeground=theme.PAPER,
            )
            delete_button.pack(side="right")
            edit_button = self._themed_button(
                row,
                text=_OVERRIDE_EDIT_GLYPH,
                command=lambda e=entry: self._handle_edit_override(e),
                background=theme.SURFACE,
                foreground=theme.INK_MUTED,
                activebackground=accent_fg,
                activeforeground=theme.PAPER,
            )
            edit_button.pack(side="right", padx=(0, 4))

        # Update the header count badge.
        self._edge_override_count_label.configure(
            text=f"({len(model.edges)})"
        )
        if not model.edges:
            # Empty-state hint when no overrides exist.
            hint = self._themed_label(
                self._edge_override_rows_frame,
                text=_OVERRIDE_EMPTY_HINT,
                anchor="w",
                foreground=theme.INK_MUTED,
                font=self._font_hint,
            )
            hint.pack(side="top", fill="x")

        # Recompute the scrollregion now that the row count changed.
        self._edge_override_rows_frame.update_idletasks()
        self._edge_override_canvas.configure(
            scrollregion=self._edge_override_canvas.bbox("all")
        )

    def _find_override_entry(
        self, edge_type: str, u: str, v: str
    ) -> Optional[dict]:
        """Return the cached override entry for ``(edge_type, u, v)``, if any.

        Identity is the full ``(type, u, v)`` triple -- a directed pair is not
        unique across types -- so all three must match.  Reads the overlay cached
        on the last :meth:`refresh`; returns ``None`` before the first refresh or
        when no entry matches.  The triple-match itself is delegated to the
        single shared identity helper :func:`commands._find_override_index` so the
        rule lives in exactly one place (R10).

        :param edge_type: The edge ``type`` (``"LINE"`` / ``"BLIND"``).
        :param u: The directed-pair source node name.
        :param v: The directed-pair destination node name.
        :return: The matching overlay dict, or ``None``.
        """
        if self._edges_overlay is None:
            return None
        index = _find_override_index(self._edges_overlay, edge_type, u, v)
        if index < 0:
            return None
        return self._edges_overlay[index]

    def _set_override_primitive_values(self, edge_type: str) -> None:
        """Switch the primitive-mode combobox VALUES for the derived edge type.

        ``LineMode`` tags for a LINE edge, ``BlindMode`` tags for a BLIND edge
        (each led by the inherit sentinel).  An unknown / empty derived type
        falls back to the LINE list so the combobox is never value-less.

        :param edge_type: The derived edge type (``"LINE"`` / ``"BLIND"``).
        """
        values = _PRIMITIVE_VALUES_BY_TYPE.get(
            edge_type, _PRIMITIVE_VALUES_BY_TYPE[_EDGE_TYPE_LINE]
        )
        self._edge_override_primitive_combo.configure(values=list(values))

    def _set_override_form(self, entry: Optional[dict]) -> None:
        """Populate the inline-editor override controls from *entry*.

        Each control is set to the entry's authored tag, or to its inherit
        sentinel when the key is absent / *entry* is ``None`` (the add case).
        The type-derived primitive-mode VALUES are NOT set here -- the caller
        sets them via :meth:`_set_override_primitive_values` once the type is
        known.

        :param entry: An overlay dict to read from, or ``None`` for a blank
            add-form.
        """
        if entry is None:
            self._edge_override_primitive_var.set(_INHERIT_PRIMITIVE_LABEL)
            self._edge_override_direction_var.set(_INHERIT_LABEL)
            self._edge_override_stop_at_line_var.set(_INHERIT_LABEL)
            return
        primitive = entry.get("primitive_mode")
        if primitive is None:
            self._edge_override_primitive_var.set(_INHERIT_PRIMITIVE_LABEL)
        else:
            self._edge_override_primitive_var.set(
                getattr(primitive, "name", str(primitive))
            )
        direction = entry.get("direction")
        self._edge_override_direction_var.set(
            direction if direction is not None else _INHERIT_LABEL
        )
        stop_at_line = entry.get("stop_at_line")
        self._edge_override_stop_at_line_var.set(
            stop_at_line if stop_at_line is not None else _INHERIT_LABEL
        )

    def _apply_derived_type_to_form(self, edge_type: Optional[str]) -> None:
        """Reflect a derived edge ``type`` in the read-only type label + combo.

        Sets the type label text (blank when no segment matches) and tints it by
        kind, then switches the primitive-mode combobox VALUES to the matching
        tag set.

        :param edge_type: The derived type, or ``None`` when the picked pair is
            not an existing segment.
        """
        self._edge_override_type_var.set(edge_type or "")
        tag_color = theme.BRICK if edge_type == _EDGE_TYPE_BLIND else theme.INK
        self._edge_override_type_label.configure(foreground=tag_color)
        self._set_override_primitive_values(edge_type or _EDGE_TYPE_LINE)

    def _show_override_editor(self) -> None:
        """Reveal the inline override editor sub-frame (idempotent)."""
        if not self._edge_override_editor.winfo_manager():
            self._edge_override_editor.pack(side="top", fill="x", pady=(4, 0))

    def _hide_override_editor(self) -> None:
        """Hide the inline override editor sub-frame (idempotent)."""
        if self._edge_override_editor.winfo_manager():
            self._edge_override_editor.pack_forget()

    def _highlight_override_row(
        self, identity: Optional[tuple[str, str, str]]
    ) -> None:
        """Paint a TERRACOTTA selection ring on the row for *identity*.

        Clears any previously highlighted row first, then -- if *identity* names
        a live row -- draws a 1-px terracotta border around it (the sidecar's
        selection ring) and records it so the next call / refresh can clear it.

        :param identity: The ``(type, u, v)`` row to highlight, or ``None`` to
            just clear the prior highlight.
        """
        prior = self._highlighted_override_row
        if prior is not None:
            prior_row = self._edge_override_row_frames.get(prior)
            if prior_row is not None:
                try:
                    prior_row.configure(
                        highlightthickness=_FLAT_BORDER_PX,
                        highlightbackground=theme.SURFACE,
                    )
                except tk.TclError:
                    pass
        self._highlighted_override_row = None

        if identity is None:
            return
        row = self._edge_override_row_frames.get(identity)
        if row is None:
            return
        try:
            row.configure(
                highlightthickness=_GHOST_BUTTON_BORDER_PX,
                highlightbackground=_theme_color(
                    "SELECTION_RING", theme.TERRACOTTA
                ),
                highlightcolor=_theme_color("SELECTION_RING", theme.TERRACOTTA),
            )
        except tk.TclError:
            pass
        self._highlighted_override_row = identity

    # ------------------------------------------------------------------
    # Edge-overrides graft methods (issue #79) -- invoked by the app (T8)
    # ------------------------------------------------------------------

    def prefill_add_row(self, edge_type: str, u: str, v: str) -> None:
        """Open the inline editor pre-filled from a clicked edge's ``u``/``v``.

        The graft's other half: when the operator clicks an existing edge that
        has NO override row yet, the app calls this to open the add-row inline
        editor with the clicked edge's ``u`` / ``v`` selected and its derived
        ``type`` shown read-only, so the operator can attach a fresh override.
        The three override controls open at their inherit sentinels (no override
        key).  *edge_type* is taken as the authoritative derived type (the app
        derives it from the clicked segment); the override controls' VALUES are
        switched to match.

        :param edge_type: The edge ``type`` (``"LINE"`` / ``"BLIND"``) derived
            from the clicked segment by the app.
        :param u: The directed-pair source node name.
        :param v: The directed-pair destination node name.
        """
        self._highlight_override_row(None)
        self._edge_override_u_var.set(u)
        self._edge_override_v_var.set(v)
        self._apply_derived_type_to_form(edge_type)
        self._set_override_form(None)
        self._show_override_editor()

    def prefill_edit_row(self, edge_type: str, u: str, v: str) -> bool:
        """Open the inline editor PRE-FILLED from an existing override, for edit.

        The edge-mode graft's existing-override half (U1/U3): when the operator
        clicks an edge that ALREADY carries an override, the app calls this to
        OPEN the inline editor populated from that override -- the directed pair,
        the derived (read-only) type, and the three override controls at their
        authored tags -- so the operator can correct it in place.  Re-Saving
        dispatches a fresh :attr:`on_set_edge_override` for the same identity (the
        command replaces in place, keyed on ``(type, u, v)``), so editing never
        creates a duplicate.  The matching row is highlighted so the operator sees
        which override the editor is amending.

        Reads the overlay cached on the last :meth:`refresh`; the app calls this
        AFTER the refresh has rebuilt that cache, so the entry is current.

        :param edge_type: The edge ``type`` (``"LINE"`` / ``"BLIND"``) derived
            from the clicked segment by the app.
        :param u: The directed-pair source node name.
        :param v: The directed-pair destination node name.
        :return: ``True`` if an override existed for ``(type, u, v)`` and the
            editor was opened pre-filled for edit; ``False`` if no such override
            exists (the app then falls back to :meth:`prefill_add_row`).
        """
        entry = self._find_override_entry(edge_type, u, v)
        if entry is None:
            return False
        self._edge_override_u_var.set(u)
        self._edge_override_v_var.set(v)
        self._apply_derived_type_to_form(edge_type)
        self._set_override_form(entry)
        self._highlight_override_row((edge_type, u, v))
        self._show_override_editor()
        return True

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

    def _handle_name_commit(self, _event: tk.Event | None = None) -> None:
        """Commit an in-place name edit, firing :attr:`on_node_rename`.

        Triggered by ``<Return>`` or ``<FocusOut>`` on the name entry.  Reads
        the OLD name from the canonical :attr:`_selected_node_var` and the new
        text from the editable :attr:`_name_edit_var`.  An empty or unchanged
        value is a no-op that reverts the entry to the canonical name (so a
        stray focus-out never blanks or corrupts the name); otherwise the app's
        :attr:`on_node_rename` performs the rename and re-syncs the panel.

        :param _event: Tk event (unused; the binding passes it).
        """
        if self._suppress_callbacks:
            return
        old = self._selected_node_var.get()
        if not old:
            return
        new = self._name_edit_var.get().strip()
        if not new or new == old:
            # Nothing to do: snap the entry back to the canonical name.
            self._name_edit_var.set(old)
            return
        if self.on_node_rename is not None:
            self.on_node_rename(old, new)

    def _handle_node_delete(self) -> None:
        """Delete the selected node, firing :attr:`on_node_delete`.

        Triggered by the inspector's Delete button.  Reads the selected node
        from the canonical :attr:`_selected_node_var`; the app handler runs the
        same guards + confirmation as the right-click delete.
        """
        if self._suppress_callbacks:
            return
        node = self._selected_node_var.get()
        if not node:
            return
        if self.on_node_delete is not None:
            self.on_node_delete(node)

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
        """Fire :attr:`on_delete_pivot` for a pivot row's delete button.

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

    # ------------------------------------------------------------------
    # Edge-overrides / pipeline-defaults / extended-globals handlers (#79)
    # ------------------------------------------------------------------

    def _handle_open_add_override(self) -> None:
        """Open a BLANK inline override editor (the ``+ add override`` ghost).

        Clears any located-row highlight and resets the add-form to its empty
        state (no edge picked, no derived type, every control at its inherit
        sentinel) so a prior edit does not leak into a fresh add.
        """
        if self._suppress_callbacks:
            return
        self._highlight_override_row(None)
        self._edge_override_u_var.set("")
        self._edge_override_v_var.set("")
        self._apply_derived_type_to_form(None)
        self._set_override_form(None)
        self._show_override_editor()

    def _handle_edit_override(self, entry: dict) -> None:
        """Open the inline editor pre-filled from an existing row's *entry*.

        Triggered by a row's ``e`` button.  Loads the edge ``u`` / ``v``, the
        derived (read-only) type, and the three override controls from the
        clicked overlay entry so the operator can amend it; re-Saving dispatches
        a fresh :attr:`on_set_edge_override` for the same identity (the command
        replaces in place, keyed on ``(type, u, v)``).

        :param entry: The overlay dict captured from the edited row.
        """
        if self._suppress_callbacks:
            return
        edge_type = str(entry.get("type", ""))
        u = str(entry.get("u", ""))
        v = str(entry.get("v", ""))
        self._highlight_override_row(None)
        self._edge_override_u_var.set(u)
        self._edge_override_v_var.set(v)
        self._apply_derived_type_to_form(edge_type)
        self._set_override_form(entry)
        self._show_override_editor()

    def _handle_override_edge_pick(self, _event: tk.Event) -> None:
        """Re-derive the override ``type`` when ``u`` / ``v`` changes.

        Triggered by either edge combobox.  Looks up the picked pair against the
        cached segment sets, shows the derived (read-only) type, and
        context-switches the primitive-mode combobox VALUES.  A pair that is not
        an existing segment leaves the type blank (Save then inline-rejects).
        A pre-existing override for the newly-picked edge pre-loads its controls
        so re-picking an already-overridden edge surfaces its current values.

        :param _event: Tk combobox-selected event (unused).
        """
        if self._suppress_callbacks:
            return
        u = self._edge_override_u_var.get()
        v = self._edge_override_v_var.get()
        edge_type = self._derive_edge_type(u, v)
        self._apply_derived_type_to_form(edge_type)
        existing = (
            self._find_override_entry(edge_type, u, v)
            if edge_type is not None
            else None
        )
        self._set_override_form(existing)

    def _handle_save_override(self) -> None:
        """Validate the inline editor and fire :attr:`on_set_edge_override`.

        Reads ``u`` / ``v`` and re-derives the ``type`` (never trusting a stale
        label).  An edge that does not name an existing LINE / BLIND segment is
        inline-rejected with a status message (mirrors the existing inline-reject
        pattern).  Each override control maps its inherit sentinel to ``None``
        (key omitted) and any tag to the verbatim string; the slot then receives
        ``(edge_type, u, v, primitive_mode, direction, stop_at_line)``.  The
        editor is hidden on a successful dispatch.
        """
        if self._suppress_callbacks:
            return
        u = self._edge_override_u_var.get()
        v = self._edge_override_v_var.get()
        edge_type = self._derive_edge_type(u, v)
        if edge_type is None:
            self._status_var.set(
                "pick an existing line or blind edge to override"
            )
            return

        primitive_label = self._edge_override_primitive_var.get()
        primitive_mode = (
            None
            if primitive_label == _INHERIT_PRIMITIVE_LABEL
            else primitive_label
        )
        direction_label = self._edge_override_direction_var.get()
        direction = None if direction_label == _INHERIT_LABEL else direction_label
        stop_label = self._edge_override_stop_at_line_var.get()
        stop_at_line = None if stop_label == _INHERIT_LABEL else stop_label

        # U2: reject the one contradiction -- primitive_mode = STOP_AT_LINE (a
        # seek bundle) together with stop_at_line='off' (force dead-reckon).  The
        # same predicate the validator's single anti-ambiguity gate uses, mirrored
        # here for fast feedback so the GUI and the Save-to-disk validator agree.
        # On a hit: name the contradiction in the status bar, do NOT dispatch, and
        # leave the editor OPEN so the operator can correct one of the two fields.
        if self._is_stop_at_line_conflict(primitive_mode, stop_at_line):
            self._status_var.set(_OVERRIDE_STOP_AT_LINE_CONFLICT_MSG)
            return

        if self.on_set_edge_override is not None:
            self.on_set_edge_override(
                edge_type, u, v, primitive_mode, direction, stop_at_line
            )
        self._hide_override_editor()

    @staticmethod
    def _is_stop_at_line_conflict(
        primitive_mode: Optional[str], stop_at_line: Optional[str]
    ) -> bool:
        """Return ``True`` for the rejected STOP_AT_LINE + ``stop_at_line='off'`` pair.

        Mirrors the contradiction predicate in
        :func:`tools.path_planner.validator._validate_edges_field`: a
        ``primitive_mode`` resolving to :attr:`BlindMode.STOP_AT_LINE` together
        with ``stop_at_line`` resolving to ``OFF``.  Both arguments arrive as the
        inline editor's verbatim tag strings (or ``None`` for "inherit"), but the
        check also accepts the enum forms so it stays identical to the validator's
        predicate should a caller pass an enum instance.

        :param primitive_mode: The override's ``primitive_mode`` tag / enum, or
            ``None`` when the key is inherited.
        :param stop_at_line: The override's ``stop_at_line`` tag / enum, or
            ``None`` when the key is inherited.
        :return: ``True`` iff the pair is the rejected contradiction.
        """
        primitive_is_stop_at_line = False
        if isinstance(primitive_mode, BlindMode):
            primitive_is_stop_at_line = primitive_mode == BlindMode.STOP_AT_LINE
        elif isinstance(primitive_mode, str):
            primitive_is_stop_at_line = (
                primitive_mode == BlindMode.STOP_AT_LINE.name
            )
        stop_is_off = False
        if isinstance(stop_at_line, StopAtLineMode):
            stop_is_off = stop_at_line == StopAtLineMode.OFF
        elif isinstance(stop_at_line, str):
            stop_is_off = stop_at_line.lower() == "off"
        return primitive_is_stop_at_line and stop_is_off

    def _handle_cancel_override(self) -> None:
        """Close the inline override editor without dispatching (Cancel)."""
        if self._suppress_callbacks:
            return
        self._hide_override_editor()

    def _handle_delete_override(
        self, identity: tuple[str, str, str]
    ) -> None:
        """Fire :attr:`on_clear_edge_override` for a row's delete (``x``) button.

        :param identity: The ``(type, u, v)`` triple captured from the row.
        """
        if self._suppress_callbacks:
            return
        edge_type, u, v = identity
        if self.on_clear_edge_override is not None:
            self.on_clear_edge_override(edge_type, u, v)

    def _handle_pipeline_mode_change(self, which: str) -> None:
        """Fire :attr:`on_pipeline_mode_change` for a pipeline combobox.

        Triggered by a PIPELINE DEFAULTS combobox selection.  The empty unset
        sentinel maps to ``None`` (key absent -> inherit the pipeline_config.py
        default); any member name is forwarded verbatim.

        :param which: The pipeline category (``"line"`` / ``"blind"`` /
            ``"turn"`` / ``"direction"``) whose combobox committed.
        """
        if self._suppress_callbacks:
            return
        raw = self._pipeline_mode_vars[which].get()
        value = None if raw == _PIPELINE_UNSET_LABEL else raw
        if self.on_pipeline_mode_change is not None:
            self.on_pipeline_mode_change(which, value)

    def _handle_blind_projection_auto_seek_toggle(self) -> None:
        """Fire :attr:`on_field_wide_flag_change` for the auto-seek checkbutton."""
        if self._suppress_callbacks:
            return
        if self.on_field_wide_flag_change is not None:
            self.on_field_wide_flag_change(
                _BLIND_PROJECTION_AUTO_SEEK_FLAG,
                bool(self._blind_projection_auto_seek_var.get()),
            )

    def _handle_field_dimensions_commit(
        self, _event: tk.Event | None = None
    ) -> None:
        """Parse the FIELD_MM entries and fire :attr:`on_field_dimensions_change`.

        Triggered by ``<Return>`` / ``<FocusOut>`` on either dimension entry.
        Both width and height must parse to positive numbers; an unparseable or
        non-positive value sets a status message and does NOT fire (mirrors the
        coordinate-entry commit pattern).

        The two entries DISPLAY their dimension rounded to integer millimetres
        (``_FIELD_MM_FORMAT``), so a dimension the operator did not touch reads
        back as the rounded integer -- which would TRUNCATE any sub-mm value the
        model holds for it.  To preserve precision, an un-edited dimension (its
        entry still rounding to the cached ``field_mm`` value) is carried forward
        from the cached full-precision ``model.field_mm`` rather than its rounded
        display var, so editing only one dimension never alters the other's
        stored value (R6).

        :param _event: Tk event (unused; the binding passes it).
        """
        if self._suppress_callbacks:
            return
        raw_w = self._field_width_mm_var.get()
        raw_h = self._field_height_mm_var.get()
        try:
            width_mm = float(raw_w)
        except ValueError:
            self._status_var.set("field width must be a number")
            return
        try:
            height_mm = float(raw_h)
        except ValueError:
            self._status_var.set("field height must be a number")
            return
        if width_mm <= 0.0:
            self._status_var.set("field width must be > 0")
            return
        if height_mm <= 0.0:
            self._status_var.set("field height must be > 0")
            return
        # Carry forward the un-edited dimension's exact stored value (R6): if an
        # entry still rounds to the model's cached dimension (the integer the
        # entry displays), the operator did not edit it, so use the cached
        # full-precision value instead of the rounded readback -- this prevents a
        # single-dimension edit from truncating the other dimension's sub-mm
        # value.  A genuine edit differs at integer precision and is dispatched.
        width_mm, height_mm = self._preserve_unedited_field_mm(width_mm, height_mm)
        if self.on_field_dimensions_change is not None:
            self.on_field_dimensions_change(width_mm, height_mm)

    def _preserve_unedited_field_mm(
        self, width_mm: float, height_mm: float
    ) -> tuple[float, float]:
        """Substitute the cached exact value for an un-edited FIELD_MM dimension.

        For each of *width_mm* / *height_mm*, if the committed value rounds to the
        same integer millimetre as the cached :attr:`_field_mm` dimension (the
        integer the entry shows), the operator did not change it -- so the cached
        full-precision value is returned in its place.  An edited dimension (one
        that differs at the displayed integer precision) passes through verbatim.
        With no cache yet (no refresh has run) both values pass through unchanged.

        :param width_mm: The width parsed from the entry (rounded integer mm if
            untouched).
        :param height_mm: The height parsed from the entry.
        :return: The ``(width_mm, height_mm)`` pair with any un-edited dimension
            replaced by its cached exact value.
        """
        if self._field_mm is None:
            return width_mm, height_mm
        cached_w, cached_h = self._field_mm
        if round(width_mm) == round(cached_w):
            width_mm = cached_w
        if round(height_mm) == round(cached_h):
            height_mm = cached_h
        return width_mm, height_mm

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

    def _handle_show_shortcuts(self, _event: tk.Event | None = None) -> None:
        """Fire :attr:`on_show_shortcuts` if assigned.

        Bound to the status-strip key-hint click.  Defensive against the
        unassigned slot like every other handler.

        :param _event: Tk event (unused).
        """
        if self.on_show_shortcuts is not None:
            self.on_show_shortcuts()


# ---------------------------------------------------------------------------
# Collapsible-section record
# ---------------------------------------------------------------------------


class _Collapsible:
    """Mutable record tracking one collapsible inspector section.

    Holds the body frame to show / hide, the chevron label whose glyph flips on
    toggle, the two chevron glyphs, and the current open/closed state.  Kept as
    a tiny plain object (not a dataclass) so this module needs no extra import.

    :ivar body: The :class:`tk.Frame` shown when expanded, hidden when
        collapsed.
    :ivar chevron: The :class:`tk.Label` whose text is the chevron glyph.
    :ivar open_glyph: Glyph shown when the section is expanded.
    :ivar closed_glyph: Glyph shown when the section is collapsed.
    :ivar is_open: ``True`` while the body is visible.
    """

    __slots__ = ("body", "chevron", "open_glyph", "closed_glyph", "is_open")

    def __init__(
        self,
        *,
        body: tk.Frame,
        chevron: tk.Label,
        open_glyph: str,
        closed_glyph: str,
        is_open: bool,
    ) -> None:
        self.body = body
        self.chevron = chevron
        self.open_glyph = open_glyph
        self.closed_glyph = closed_glyph
        self.is_open = is_open


def _theme_size(name: str, fallback: int) -> int:
    """Return ``theme.<name>`` as an int if defined, otherwise *fallback*.

    Companion to :func:`_theme_color` for the integer font-size tokens so a
    missing optional token degrades to a sensible default rather than raising.

    :param name: Attribute name to read off the :mod:`theme` module.
    :param fallback: Integer to use when the attribute is absent.
    :return: The resolved point size.
    """
    value = getattr(theme, name, fallback)
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback
