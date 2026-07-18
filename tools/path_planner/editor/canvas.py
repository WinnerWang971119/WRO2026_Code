"""Tk-embedded matplotlib canvas for the WRO 2026 live field editor.

:class:`FieldCanvas` is the visual surface of the editor: it renders the
field map (background PNG, obstacles, edges, nodes, optional highlight
ring) inside a :mod:`matplotlib` figure that is embedded into a Tk parent
widget through :class:`~matplotlib.backends.backend_tkagg.FigureCanvasTkAgg`.

Mouse and key wiring is exposed as plain Python callback attributes
(``on_click``, ``on_drag``, ``on_release``) which the editor application
populates after construction. The canvas does **not** own any mode logic
— it merely forwards in-axes mouse events to those callbacks in world
coordinates (centimetres). Mode routing is the responsibility of the
editor application (task T12 of the live field editor plan).

This module deliberately does **not** import from
:mod:`tools.path_planner.visualizer`, which selects the headless ``Agg``
backend at import time. Sharing imports would race the backend selector
and break either the static PNG build or this interactive canvas.

Coordinate convention (inherited from
:class:`tools.path_planner.editor.model.FieldModel`):
    Origin at the field SW corner. X is East, Y is North. Distances in
    centimetres. Headings in degrees, ``[0, 360)``, CCW from the +x axis.
"""

from __future__ import annotations

import time
import warnings
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Optional

import tkinter as tk

import matplotlib

matplotlib.use("TkAgg")  # MUST come before any other matplotlib import.

import matplotlib.image as mpimg  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
from matplotlib.axes import Axes  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from tools.path_planner.editor import theme  # noqa: E402
from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.graph_builder import inflate_obstacles  # noqa: E402

if TYPE_CHECKING:
    # Type-only: ``align.py`` is a sibling module this canvas never imports at
    # runtime (no Tk/matplotlib in that module, and importing it here would
    # risk a needless coupling). ``render``'s ``align_preview`` parameter
    # accepts any duck-typed object with ``.guide`` / ``.ghosts`` -- see
    # :meth:`FieldCanvas._draw_align_preview`.
    from tools.path_planner.editor.align import AlignPreview


# ---------------------------------------------------------------------------
# Rendering constants. Sizes / z-orders / alphas are local; every COLOR is
# sourced from ``tools.path_planner.editor.theme`` (the warm "drafting-table"
# palette) so the canvas, panels, and app share one palette. The static
# visualizer is intentionally NOT imported here -- doing so would race the
# matplotlib backend selector (Agg vs TkAgg) and break headless PNG dumps.
# ---------------------------------------------------------------------------

# Background image alpha (matches ``visualizer.py``).
_BACKGROUND_ALPHA: float = 0.6
_BACKGROUND_ZORDER: int = 0

# Sentinel stored in the per-instance background cache when a map PNG is
# missing or fails to decode. Caching the *failure* (rather than leaving the
# key absent) is what stops :meth:`FieldCanvas._draw_background` from
# re-attempting a doomed ``imread`` -- and re-warning -- on every render tick
# during a drag. A cache value of ``_BACKGROUND_NONE`` means "this path has
# no usable background; do not try again".
_BACKGROUND_NONE: object = object()

# Raw obstacle outline (axis-aligned rectangle from ``field_data.OBSTACLES``).
_OBSTACLE_RAW_LINEWIDTH: float = 0.8
_OBSTACLE_RAW_EDGECOLOR: str = "black"
_OBSTACLE_RAW_LINESTYLE: str = "--"
_OBSTACLE_RAW_ZORDER: int = 1

# C-space inflated obstacle polygon (used by edge-clip / planner checks).
_OBSTACLE_INFLATED_FACECOLOR: str = "lightgray"
_OBSTACLE_INFLATED_EDGECOLOR: str = "gray"
_OBSTACLE_INFLATED_ALPHA: float = 0.5
_OBSTACLE_INFLATED_ZORDER: int = 2

# Edge styling. Colors come from the theme's edge palette (line = ink,
# blind = brick); linestyle / linewidth stay local.
_EDGE_LINE_COLOR: str = theme.EDGE_COLORS["line"]
_EDGE_LINE_LINESTYLE: str = "solid"
_EDGE_LINE_LINEWIDTH: float = 1.5

_EDGE_BLIND_COLOR: str = theme.EDGE_COLORS["blind"]
_EDGE_BLIND_LINESTYLE: str = "dashed"
_EDGE_BLIND_LINEWIDTH: float = 1.5

_EDGE_ZORDER: int = 3

# Proposed-edge preview styling (used by FieldCanvas.render's proposed_edges
# kwarg -- drawn ABOVE committed edges, BELOW the highlight ring, so the
# operator can see the proposal sitting on top of the existing graph. The
# color is the theme's proposed-edge accent (teal).
_PROPOSED_EDGE_COLOR: str = theme.PROPOSED_EDGE
_PROPOSED_EDGE_LINESTYLE: tuple = (0, (3, 3))  # dashed
_PROPOSED_EDGE_LINEWIDTH: float = 1.8
_PROPOSED_EDGE_ZORDER: int = 4  # between _EDGE_ZORDER (3) and _NODE_ZORDER (6)

# Hairline grid linewidth (points). Thin enough to read as a drafting-table
# reference grid behind the nodes/edges without competing with the geometry.
# The grid COLOR is set by ``theme.apply_to_axes`` (the warm tan ``theme.GRID``);
# this only pins the stroke weight.
_GRID_LINEWIDTH: float = 0.5
_GRID_ZORDER: int = 0  # behind edges (3) and nodes (6); level with background

# Node dots + labels.
_NODE_DOT_SIZE: float = 60.0
# Per-node dot ``markersize`` (points) for the ``ax.plot``-drawn dots that the
# drag fast-path animates individually. ``ax.scatter`` sizes markers by area in
# points^2 (``s``), but ``ax.plot`` sizes them by diameter in points
# (``markersize``), so the two are NOT interchangeable: the visual equivalent of
# ``s`` is ``sqrt(s)``. ``sqrt(60) ~= 7.746`` keeps the per-node dots the same
# apparent size as the old single-scatter-per-type dots. Rounded to one decimal
# to read as a deliberate constant rather than a raw float tail.
_NODE_DOT_MARKERSIZE: float = 7.7
_NODE_ZORDER: int = 6
_NODE_LABEL_FONTSIZE: int = 8
_NODE_LABEL_OFFSET_CM: float = 1.5
_NODE_LABEL_ZORDER: int = 7

# Node fill palette keyed by the model's node type. Sourced from the theme
# (line = ink, projection = ochre, blind = brick, reset = teal); an unknown
# type falls back to the theme's muted-ink fallback.
_NODE_COLOR: dict[str, str] = theme.NODE_COLORS
_NODE_COLOR_FALLBACK: str = theme.NODE_COLOR_FALLBACK

# Highlight ring drawn on top of the selected node(s). The ring color is the
# theme's selection accent (terracotta); linewidth / dot size / z-order stay
# local.
_HIGHLIGHT_COLOR: str = theme.SELECTION_RING
_HIGHLIGHT_LINEWIDTH: float = 2.5
_HIGHLIGHT_EDGECOLOR: str = theme.SELECTION_RING
_HIGHLIGHT_DOT_SIZE: float = 140.0
_HIGHLIGHT_ZORDER: int = 8

# Default tolerance for :meth:`FieldCanvas.find_node_at`, in centimetres.
# Calibrated for the default scatter dot at standard zoom -- big enough to
# forgive a sloppy click on the dot, small enough that two adjacent nodes
# don't both match.
_DEFAULT_HIT_TOLERANCE_CM: float = 4.0

# Default figure size in inches. The aspect ratio is locked to the field
# extent via :meth:`Axes.set_aspect` so this is just an initial hint.
_DEFAULT_FIGSIZE_IN: tuple[float, float] = (8.0, 6.0)

# Subplot margins (figure fractions) applied on every render so the map fills
# the figure instead of floating in matplotlib's default ~12% borders.  The
# left / bottom insets reserve just enough room for the y / x tick labels and
# the "x (cm)" / "y (cm)" axis labels; the right / top sit near the edge.  This
# lifts the drawn map from ~59% of the figure to ~78% -- the "too much blank"
# fix -- while ``set_aspect("equal")`` keeps the field undistorted.
_SUBPLOT_MARGINS: dict[str, float] = {
    "left": 0.075,
    "right": 0.985,
    "bottom": 0.095,
    "top": 0.985,
}

# Mouse button numbers per the matplotlib MouseEvent contract.
_MOUSE_BUTTON_LEFT: int = 1
_MOUSE_BUTTON_RIGHT: int = 3

# Minimum wall-clock interval (seconds) between two blit composites during a
# node drag. ``motion_notify_event`` can fire faster than the display can
# usefully repaint, so :meth:`FieldCanvas.update_node_drag` always updates the
# animated artists' data but only re-composites (restore_region -> draw_artist
# -> blit) when at least this long has elapsed since the last composite. ~1/60 s
# caps the visible repaint at roughly 60 Hz; the artist data is still current on
# every tick, so the throttled-away frames lose nothing the next paint (or
# :meth:`end_node_drag`) does not immediately restore.
_BLIT_MIN_INTERVAL_S: float = 1.0 / 60.0

# Sentinel ``_last_blit_time`` value that forces the first
# :meth:`FieldCanvas.update_node_drag` of a drag to composite unconditionally.
# ``0.0`` is far enough in the past relative to ``time.monotonic()`` (which is
# always strongly positive after process start) that the first interval check
# can never throttle the opening frame.
_BLIT_TIME_FORCE_FIRST: float = 0.0

# Style of the dotted rubber-band rectangle drawn while an OBSTACLE is being
# dragged. Lives here (not in the app) because the obstacle-drag blit fast-path
# owns the preview artist now; the app no longer draws it.
_OBSTACLE_PREVIEW_ZORDER: int = 5
_OBSTACLE_PREVIEW_EDGECOLOR: str = "blue"
_OBSTACLE_PREVIEW_LINESTYLE: str = ":"
_OBSTACLE_PREVIEW_LINEWIDTH: float = 1.5

# Style of the dotted rubber-band rectangle drawn while a node-selection
# MARQUEE is being dragged in ``Mode.SELECT``. A separate, parallel constant
# set from the obstacle preview above -- same shape of blit fast-path, but its
# own colour because a different colour encodes a different verb: select
# (this box) vs. create (the obstacle box, left untouched). Sourced from the
# theme's selection accent so the marquee reads as "this is what I'm
# selecting", consistent with the terracotta highlight ring drawn on the
# selected nodes themselves.
_MARQUEE_PREVIEW_ZORDER: int = 11
_MARQUEE_PREVIEW_EDGECOLOR: str = theme.SELECTION_RING
_MARQUEE_PREVIEW_LINESTYLE: str = ":"
_MARQUEE_PREVIEW_LINEWIDTH: float = 1.5

# Make-straight preview overlay (``FieldCanvas.render``'s ``align_preview``
# kwarg): a guide line spanning the alignment axis plus hollow dots at each
# mover's target. Styled like the proposed-edge overlay (same TEAL token,
# same "this is where things would go" meaning) but drawn ABOVE the highlight
# ring so the ghost targets stay legible against a selected (ringed) node.
# These are STATIC artists added in the normal render pass -- unlike the
# marquee/obstacle rubber-bands above, there is no drag to blit-fast-path
# here; the preview only changes when the selection or axis choice changes,
# which already triggers a full ``render``.
_ALIGN_GUIDE_LINESTYLE: tuple = (0, (3, 3))  # dashed, matches _PROPOSED_EDGE
_ALIGN_GUIDE_LINEWIDTH: float = 1.8
_ALIGN_GUIDE_ZORDER: int = 9

_ALIGN_GHOST_DOT_SIZE: float = 100.0
_ALIGN_GHOST_LINEWIDTH: float = 2.0
_ALIGN_GHOST_ZORDER: int = 10

# Resolve ``map_png`` relative to this directory (same as ``visualizer.py``
# and ``pick_xy.py``: ``tools/path_planner/``).
_PLANNER_ROOT: Path = Path(__file__).resolve().parent.parent


class FieldCanvas:
    """Tk-embedded matplotlib canvas that renders a :class:`FieldModel`.

    The canvas owns its own :class:`~matplotlib.figure.Figure`,
    :class:`~matplotlib.axes.Axes`, and
    :class:`~matplotlib.backends.backend_tkagg.FigureCanvasTkAgg`. The Tk
    widget for the embedded canvas is exposed via :attr:`tk_widget` so the
    caller can ``.pack`` / ``.grid`` it inside any parent layout.

    Mouse handling is wired as three public attributes that callers set
    directly (no Observer pattern, no subclassing required):

    * :attr:`on_click`   — fires on left/right ``button_press_event``.
    * :attr:`on_drag`    — fires on ``motion_notify_event`` while a button
      is held.
    * :attr:`on_release` — fires on ``button_release_event``.

    Each callback receives world coordinates already converted to
    centimetres (``world_x_cm``, ``world_y_cm``), and click/release also
    receive the matplotlib button number (``1`` = left, ``3`` = right) and
    the modifier key held during the event (``event.key``: ``None``,
    ``"shift"``, ``"control"``, ...), so the editor application can
    implement shift-add / ctrl-toggle selection. Callbacks are only invoked
    when the underlying event is inside this canvas' axes; out-of-axes
    events are silently dropped (release is the one exception -- see
    :meth:`_on_button_release`).

    :param parent: Parent Tk widget (or :class:`tkinter.Tk`) that will
        host the embedded canvas widget.
    """

    def __init__(self, parent: tk.Widget | tk.Tk) -> None:
        """Construct the figure, axes, and embedded Tk canvas.

        :param parent: Parent Tk widget to embed the canvas into.
        """
        self._parent: tk.Widget | tk.Tk = parent

        # Mouse-drag tracking state. ``_active_button`` is the button
        # number currently held, or ``None`` if no button is pressed. This
        # mirrors the typical matplotlib drag pattern: ``button_press``
        # latches a button, ``motion_notify`` fires while it is latched,
        # ``button_release`` clears it.
        self._active_button: Optional[int] = None

        self._figure: Figure = Figure(figsize=_DEFAULT_FIGSIZE_IN)
        self._ax: Axes = self._figure.add_subplot(111)

        self._canvas: FigureCanvasTkAgg = FigureCanvasTkAgg(
            self._figure, master=self._parent
        )
        # Force an initial draw so the widget reports a valid size before
        # the caller calls ``.pack``/``.grid`` on it. Without this, the
        # first ``render`` call on some Tk backends paints into a 1x1 px
        # surface.
        self._canvas.draw()

        # Three callback slots the editor application sets directly. Each
        # is invoked only when the originating event is inside ``self._ax``
        # (release is the one exception -- see :meth:`_on_button_release`).
        # ``on_click`` / ``on_release`` carry ``(x, y, button, key)`` -- the
        # 4th arg is matplotlib's ``event.key`` (``None``, ``"shift"``,
        # ``"control"``, ...) so the app can implement modifier-click
        # selection (shift = add, ctrl = toggle).
        self.on_click: Optional[
            Callable[[float, float, int, Optional[str]], None]
        ] = None
        self.on_drag: Optional[Callable[[float, float], None]] = None
        self.on_release: Optional[
            Callable[[float, float, int, Optional[str]], None]
        ] = None

        # Cache the most recently rendered model so callers (and the
        # find-node helper) can ask "what's at (x, y)?" without
        # re-passing the model on every query.
        self._last_model: Optional[FieldModel] = None

        # Per-instance background cache keyed by the *resolved* absolute map
        # path string. A hit reuses the already-decoded image array; a miss
        # calls ``mpimg.imread`` exactly once. Failed/missing loads store the
        # ``_BACKGROUND_NONE`` sentinel so a doomed decode -- and its warning
        # -- is not re-attempted on every render tick during a drag. The
        # decoded array's type is backend-specific (a NumPy ``ndarray``), so
        # the value is typed loosely as ``object``.
        self._bg_cache: dict[str, object] = {}

        # Last render's overlay arguments, cached so the drag fast-path
        # (begin/update/end_node_drag) and the blit fallback can re-issue an
        # identical full render -- same highlight ring, same proposed-edge
        # overlay, same make-straight preview -- as the discrete-edit path
        # produced. Refreshed on every :meth:`render` call.
        self._last_highlighted_nodes: Optional[list[str]] = None
        self._last_proposed_edges: Optional[
            list[tuple[str, str, str]]
        ] = None
        self._last_align_preview: Optional["AlignPreview"] = None

        # ---- Per-element artist retention (rebuilt every ``render``) ----
        # ``ax.clear()`` drops every artist, so these are repopulated as the
        # nodes / edges are drawn. They give the drag fast-path direct handles
        # to the individual artists it animates instead of the type-grouped
        # PathCollections the discrete-render path used to keep.
        #
        # ``_node_dot_artists``: node name -> its dedicated ``Line2D`` dot.
        # ``_node_label_artists``: node name -> its ``Text`` label.
        # ``_edge_artists``: ``(u, v, kind, Line2D)`` per committed edge, where
        #   ``kind`` is ``"line"`` or ``"blind"``; used to find the edges
        #   incident to a dragged node and to move the dragged endpoint.
        self._node_dot_artists: dict[str, object] = {}
        self._node_label_artists: dict[str, object] = {}
        self._edge_artists: list[tuple[str, str, str, object]] = []

        # ---- Drag-scoped blit state (set by ``begin_node_drag``) ----
        # ``_drag_node``: the node name currently being dragged, or ``None``.
        # ``_blit_background``: the baked static-scene region captured by
        #   ``copy_from_bbox`` in ``begin_node_drag``; ``None`` when no drag is
        #   active (a missing background forces ``update_node_drag`` down its
        #   defensive full-render path).
        # ``_animated_artists``: the artists flagged ``.set_animated(True)`` for
        #   the active drag (the dragged dot, its label, its incident edges).
        # ``_last_blit_time``: ``time.monotonic()`` of the last composite,
        #   used by the ~60 Hz blit throttle.
        self._drag_node: Optional[str] = None
        self._blit_background: object = None
        self._animated_artists: list[object] = []
        self._last_blit_time: float = _BLIT_TIME_FORCE_FIRST

        # ---- Obstacle rubber-band blit state (set by ``begin_obstacle_drag``) ----
        # Mirrors the node-drag blit fast-path so sizing an obstacle restores a
        # baked background + re-draws ONLY the dotted preview rectangle instead
        # of clearing + re-rendering the whole scene per motion event.
        # ``_obstacle_preview_artist``: the animated ``Rectangle`` patch, or
        #   ``None`` when no obstacle drag is armed.
        # ``_obstacle_blit_background``: the baked static-scene region.
        # ``_last_obstacle_blit_time``: ``time.monotonic()`` of the last
        #   composite, used by the same ~60 Hz throttle.
        self._obstacle_preview_artist: Optional[object] = None
        self._obstacle_blit_background: object = None
        self._last_obstacle_blit_time: float = _BLIT_TIME_FORCE_FIRST

        # ---- Node-selection marquee blit state (set by begin_marquee_drag) ----
        # A separate, parallel clone of the obstacle rubber-band blit state
        # above -- its own preview artist, its own baked background, its own
        # throttle clock -- so a Mode.SELECT marquee drag never shares state
        # with a Mode.OBSTACLE resize. See :meth:`begin_marquee_drag` /
        # :meth:`update_marquee_preview` / :meth:`end_marquee_drag`.
        self._marquee_preview_artist: Optional[object] = None
        self._marquee_blit_background: object = None
        self._last_marquee_blit_time: float = _BLIT_TIME_FORCE_FIRST

        # Wire matplotlib's event system into our three Python callbacks.
        # Connection IDs are stashed so a future ``destroy`` method (out of
        # scope here) could ``mpl_disconnect`` cleanly if needed.
        self._cid_press: int = self._figure.canvas.mpl_connect(
            "button_press_event", self._on_button_press
        )
        self._cid_motion: int = self._figure.canvas.mpl_connect(
            "motion_notify_event", self._on_motion_notify
        )
        self._cid_release: int = self._figure.canvas.mpl_connect(
            "button_release_event", self._on_button_release
        )

    # ------------------------------------------------------------------
    # Public attribute access
    # ------------------------------------------------------------------

    @property
    def tk_widget(self) -> tk.Widget:
        """Return the embedded Tk widget for layout management.

        The returned widget is the one produced by
        :meth:`FigureCanvasTkAgg.get_tk_widget`; pack it, grid it, or
        place it in any Tk container.

        :return: Tk widget that draws the matplotlib figure.
        """
        return self._canvas.get_tk_widget()

    @property
    def figure(self) -> Figure:
        """Return the underlying :class:`matplotlib.figure.Figure`."""
        return self._figure

    @property
    def axes(self) -> Axes:
        """Return the underlying :class:`matplotlib.axes.Axes`."""
        return self._ax

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def render(
        self,
        model: FieldModel,
        *,
        highlighted_nodes: Optional[list[str]] = None,
        proposed_edges: Optional[list[tuple[str, str, str]]] = None,
        align_preview: "AlignPreview | None" = None,
    ) -> None:
        """Redraw the canvas from scratch for the given model.

        Clears the axes, restores the cosmetic limits / labels / grid,
        then paints in fixed z-order: background → obstacles → edges →
        proposed-edge overlay → nodes → highlight ring → make-straight
        preview. Calling this after every model mutation is the editor's
        standard refresh path.

        :param model: The :class:`FieldModel` to render. Stored on the
            canvas for use by :meth:`find_node_at`.
        :param highlighted_nodes: Optional list of node names to draw a
            thick terracotta ring around (on top of the regular node dots).
            Unknown names are silently skipped.
        :param proposed_edges: Optional list of ``(a, b, kind)`` triples
            drawn as a dashed teal overlay above the committed edges.
            ``kind`` is informational (caller may use it for status
            reporting); the canvas styles all proposed edges identically.
            Unknown node names are silently skipped (consistent with
            ``highlighted_nodes``).
        :param align_preview: Optional make-straight preview (the
            ``editor.align.AlignPreview`` dataclass, or any duck-typed
            object exposing the same ``.guide`` / ``.ghosts`` shape). When
            given, draws a teal guide line and hollow teal ghost dots at
            each mover's target -- see :meth:`_draw_align_preview`. These are
            plain static artists in this normal render pass, not part of any
            blit fast-path.
        :return: ``None``.
        """
        self._last_model = model
        # Remember the overlay args so the drag fast-path and the blit
        # fallback can replay an identical full render.
        self._last_highlighted_nodes = highlighted_nodes
        self._last_proposed_edges = proposed_edges
        self._last_align_preview = align_preview

        ax = self._ax
        ax.clear()

        # ``ax.clear()`` just dropped every retained artist; rebuild the
        # per-element handle collections from scratch as we draw below.
        self._node_dot_artists = {}
        self._node_label_artists = {}
        self._edge_artists = []

        width_cm, height_cm = self._field_extent_cm(model)

        # Pull the axes out to near the figure edges so the map uses the space
        # instead of floating in the default wide borders (the "too much blank"
        # fix). ``ax.clear()`` does not reset subplot params, but re-applying is
        # cheap and keeps the layout correct after a figure resize.
        self._figure.subplots_adjust(**_SUBPLOT_MARGINS)

        ax.set_aspect("equal")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_xlim(0.0, width_cm)
        ax.set_ylim(0.0, height_cm)
        # Hairline reference grid: thin warm-tan lines sitting behind the
        # nodes/edges. ``apply_to_axes`` (below) re-issues ``grid(True, color=...)``
        # to set the COLOR from the theme; this opening call pins the stroke
        # WEIGHT and z-order so the grid reads as a subtle drafting-table
        # underlay rather than a default-weight matplotlib grid.
        ax.grid(True, linewidth=_GRID_LINEWIDTH, zorder=_GRID_ZORDER)

        # Recolor the figure / axes / grid / spines / ticks from the warm
        # "drafting-table" theme so the map area is the cream PAPER tone rather
        # than the default matplotlib grey/white. This runs AFTER the ``grid``
        # call above because ``apply_to_axes`` re-issues ``grid(True, color=...)``
        # to set the grid color (it does not touch linewidth, so the hairline
        # weight set above is preserved). The background PNG still draws on top
        # via ``imshow`` at ``_BACKGROUND_ALPHA``, so the paper tone shows through.
        theme.apply_to_axes(ax, self._figure)

        self._draw_background(model, width_cm, height_cm)
        self._draw_obstacles(model)
        self._draw_edges(model)
        if proposed_edges:
            self._draw_proposed_edges(model, proposed_edges)
        self._draw_nodes(model)
        if highlighted_nodes:
            self._draw_highlight(model, highlighted_nodes)
        if align_preview is not None:
            self._draw_align_preview(align_preview)

        self._canvas.draw_idle()

    # ------------------------------------------------------------------
    # Drag-scoped blit fast-path
    # ------------------------------------------------------------------

    def begin_node_drag(self, name: str) -> None:
        """Arm the blit fast-path for an interactive drag of node *name*.

        Renders the current model once the normal way (so the static scene is
        up to date), then flags the dragged node's dot, its label, and every
        committed edge incident to it as ``animated``. A full
        :meth:`FigureCanvasTkAgg.draw` bakes the *static* scene -- matplotlib
        excludes ``animated`` artists from that bake -- and the resulting
        pixels are captured with ``copy_from_bbox`` so each subsequent
        :meth:`update_node_drag` only has to restore that frozen background and
        re-draw the handful of moving artists.

        Degrades gracefully: if *name* is not in the model or has no retained
        dot artist (e.g. a stale selection), the fast-path is left disarmed
        (``_blit_background`` stays ``None``) so the next
        :meth:`update_node_drag` falls back to a defensive full render.

        :param name: Name of the node about to be dragged.
        :return: ``None``.
        """
        model = self._last_model
        if model is None:
            return

        # Render the current model the normal way first; the app calls
        # ``begin_node_drag`` right after a normal render, but re-rendering
        # here keeps this method correct in isolation and rebuilds the
        # per-element artist handles this method is about to look up.
        self.render(
            model,
            highlighted_nodes=self._last_highlighted_nodes,
            proposed_edges=self._last_proposed_edges,
            align_preview=self._last_align_preview,
        )

        dot_artist = self._node_dot_artists.get(name)
        if dot_artist is None:
            # Unknown / stale node: leave the fast-path disarmed.
            return

        animated: list[object] = [dot_artist]
        label_artist = self._node_label_artists.get(name)
        if label_artist is not None:
            animated.append(label_artist)
        for u, v, _kind, edge_artist in self._edge_artists:
            if u == name or v == name:
                animated.append(edge_artist)

        for artist in animated:
            artist.set_animated(True)

        # Bake the static scene (animated artists excluded) and snapshot it.
        self._canvas.draw()
        self._animated_artists = animated
        self._blit_background = self._canvas.copy_from_bbox(self._ax.bbox)
        self._drag_node = name
        # Force the first ``update_node_drag`` to composite unconditionally.
        self._last_blit_time = _BLIT_TIME_FORCE_FIRST

    def update_node_drag(self, name: str, x: float, y: float) -> None:
        """Move node *name* to ``(x, y)`` and blit, throttled to ~60 Hz.

        Updates the dragged node's dot + label artist data and slides the
        moved endpoint of each incident edge artist, then composites the frame
        by restoring the baked background and re-drawing only the animated
        artists. The artist data is updated on *every* call (so the artists
        always hold the latest position); only the visible composite is
        throttled -- a call that arrives within :data:`_BLIT_MIN_INTERVAL_S`
        of the last composite updates the data but skips the
        restore/draw/blit, letting the next un-throttled tick (or
        :meth:`end_node_drag`) paint the final state.

        This method's ``(self, name, x, y)`` signature is a fixed contract
        with the editor application (task T3): do not change it.

        Defensive paths:

        * If the fast-path was never armed (``_blit_background is None``) or
          *name* does not match the armed drag node, a full :meth:`render` is
          issued instead.
        * If the composite raises on an exotic backend, it is caught and a
          full :meth:`render` is issued for that tick -- a drag must never
          crash the editor.

        :param name: Name of the node being dragged (must match the node
            passed to :meth:`begin_node_drag`).
        :param x: New world x coordinate, in centimetres.
        :param y: New world y coordinate, in centimetres.
        :return: ``None``.
        """
        if self._blit_background is None or name != self._drag_node:
            # Fast-path not armed for this node: fall back to a full render so
            # the move is still reflected on screen.
            self._full_render_during_drag(name, x, y)
            return

        # Always update the artist data, even on a throttled tick, so the
        # artists hold the latest position for the next paint.
        self._apply_drag_geometry(name, x, y)

        now = time.monotonic()
        if now - self._last_blit_time < _BLIT_MIN_INTERVAL_S:
            # Throttle the COMPOSITE only; data above is already current.
            return

        try:
            self._canvas.restore_region(self._blit_background)
            for artist in self._animated_artists:
                self._ax.draw_artist(artist)
            self._canvas.blit(self._ax.bbox)
        except Exception:  # noqa: BLE001 -- exotic backend: never crash a drag.
            # Disarm the fast-path: the about-to-run full render clears and
            # rebuilds every artist, leaving the captured background stale, so
            # subsequent ticks must take the full-render path until the next
            # ``begin_node_drag`` re-bakes a fresh background.
            self._blit_background = None
            self._full_render_during_drag(name, x, y)
            return

        self._last_blit_time = now

    def end_node_drag(self) -> None:
        """Tear down the blit fast-path and re-bake the full static scene.

        Clears the ``animated`` flag on every tracked artist, drops the drag
        state, and issues a full :meth:`render` so the moved node and its
        edges rejoin the normal static scene. A double render on release (the
        app typically refreshes again afterwards) is harmless -- this runs
        once per drag, not once per frame.

        Safe to call when no drag is active: the tracked-artist list is empty
        and the render simply repaints the current model.

        :return: ``None``.
        """
        for artist in self._animated_artists:
            artist.set_animated(False)
        self._animated_artists = []
        self._blit_background = None
        self._drag_node = None
        self._last_blit_time = _BLIT_TIME_FORCE_FIRST

        if self._last_model is not None:
            self.render(
                self._last_model,
                highlighted_nodes=self._last_highlighted_nodes,
                proposed_edges=self._last_proposed_edges,
                align_preview=self._last_align_preview,
            )

    # ------------------------------------------------------------------
    # Obstacle rubber-band blit fast-path
    # ------------------------------------------------------------------

    def begin_obstacle_drag(self) -> None:
        """Arm the blit fast-path for an obstacle rubber-band drag.

        Renders the current model once the normal way (so the static scene is
        current), adds a single zero-size ``animated`` ``Rectangle`` for the
        dotted preview, bakes the static scene with a full :meth:`draw`
        (matplotlib excludes ``animated`` artists from that bake), and snapshots
        it with ``copy_from_bbox``. Each subsequent
        :meth:`update_obstacle_preview` then only restores that frozen
        background and re-draws the one rectangle -- no ``ax.clear``, no model
        re-render, no validator, no per-frame obstacle inflation (the lag the
        old full-:meth:`render`-per-motion path caused).

        Degrades gracefully: with no model rendered yet the fast-path is left
        disarmed (``_obstacle_blit_background`` stays ``None``) so the next
        :meth:`update_obstacle_preview` falls back to a defensive full render.

        :return: ``None``.
        """
        model = self._last_model
        if model is None:
            return

        # Render the current model first so the static scene under the preview
        # is up to date, then rebuild the per-element artist handles.
        self.render(
            model,
            highlighted_nodes=self._last_highlighted_nodes,
            proposed_edges=self._last_proposed_edges,
            align_preview=self._last_align_preview,
        )

        rect = mpatches.Rectangle(
            (0.0, 0.0),
            0.0,
            0.0,
            linewidth=_OBSTACLE_PREVIEW_LINEWIDTH,
            edgecolor=_OBSTACLE_PREVIEW_EDGECOLOR,
            linestyle=_OBSTACLE_PREVIEW_LINESTYLE,
            facecolor="none",
            zorder=_OBSTACLE_PREVIEW_ZORDER,
            animated=True,
        )
        self._ax.add_patch(rect)
        self._obstacle_preview_artist = rect

        # Bake the static scene (the animated rect is excluded) and snapshot it.
        self._canvas.draw()
        self._obstacle_blit_background = self._canvas.copy_from_bbox(
            self._ax.bbox
        )
        # Force the first ``update_obstacle_preview`` to composite.
        self._last_obstacle_blit_time = _BLIT_TIME_FORCE_FIRST

    def update_obstacle_preview(
        self, x0: float, y0: float, x1: float, y1: float
    ) -> None:
        """Resize the preview rectangle to span ``(x0, y0)``-``(x1, y1)`` and blit.

        Updates the rectangle's lower-left corner + width/height on every call
        (so the artist always holds the latest extent), then composites the
        frame by restoring the baked background and re-drawing only the
        rectangle. As with :meth:`update_node_drag`, only the visible composite
        is throttled to ~60 Hz; a call arriving within
        :data:`_BLIT_MIN_INTERVAL_S` of the last composite updates the geometry
        but skips the restore/draw/blit.

        Defensive paths mirror the node drag: if the fast-path was never armed
        (``_obstacle_blit_background is None``) a full :meth:`render` is issued;
        if the composite raises on an exotic backend it is caught, the
        fast-path disarmed, and a full render issued for that tick so an
        obstacle drag never crashes the editor.

        :param x0: Anchor x (drag start), in centimetres.
        :param y0: Anchor y (drag start), in centimetres.
        :param x1: Cursor x (current), in centimetres.
        :param y1: Cursor y (current), in centimetres.
        :return: ``None``.
        """
        rect = self._obstacle_preview_artist
        if self._obstacle_blit_background is None or rect is None:
            # Fast-path not armed: fall back to a plain repaint so the drag is
            # still reflected (defensive -- the app arms it on anchor-set).
            if self._last_model is not None:
                self.render(
                    self._last_model,
                    highlighted_nodes=self._last_highlighted_nodes,
                    proposed_edges=self._last_proposed_edges,
                    align_preview=self._last_align_preview,
                )
            return

        # Always update the artist geometry, even on a throttled tick.
        xmin = min(x0, x1)
        ymin = min(y0, y1)
        rect.set_bounds(xmin, ymin, abs(x1 - x0), abs(y1 - y0))

        now = time.monotonic()
        if now - self._last_obstacle_blit_time < _BLIT_MIN_INTERVAL_S:
            return

        try:
            self._canvas.restore_region(self._obstacle_blit_background)
            self._ax.draw_artist(rect)
            self._canvas.blit(self._ax.bbox)
        except Exception:  # noqa: BLE001 -- exotic backend: never crash a drag.
            self._obstacle_blit_background = None
            if self._last_model is not None:
                self.render(
                    self._last_model,
                    highlighted_nodes=self._last_highlighted_nodes,
                    proposed_edges=self._last_proposed_edges,
                    align_preview=self._last_align_preview,
                )
            return

        self._last_obstacle_blit_time = now

    def end_obstacle_drag(self) -> None:
        """Tear down the obstacle blit fast-path and re-bake the static scene.

        Removes the preview rectangle, drops the drag state, and issues a full
        :meth:`render` so the scene returns to normal. Safe to call when no
        obstacle drag is active (the preview artist is simply ``None``).

        :return: ``None``.
        """
        rect = self._obstacle_preview_artist
        if rect is not None:
            try:
                rect.remove()
            except (ValueError, NotImplementedError):
                # Already detached (e.g. a full render cleared the axes): fine.
                pass
        self._obstacle_preview_artist = None
        self._obstacle_blit_background = None
        self._last_obstacle_blit_time = _BLIT_TIME_FORCE_FIRST

        if self._last_model is not None:
            self.render(
                self._last_model,
                highlighted_nodes=self._last_highlighted_nodes,
                proposed_edges=self._last_proposed_edges,
                align_preview=self._last_align_preview,
            )

    # ------------------------------------------------------------------
    # Node-selection marquee blit fast-path
    # ------------------------------------------------------------------

    def begin_marquee_drag(self) -> None:
        """Arm the blit fast-path for a node-selection marquee drag.

        A separate, parallel clone of :meth:`begin_obstacle_drag`: renders
        the current model once the normal way (so the static scene is up to
        date), adds a single zero-size ``animated`` dotted ``Rectangle`` for
        the marquee preview, bakes the static scene with a full :meth:`draw`
        (matplotlib excludes ``animated`` artists from that bake), and
        snapshots it with ``copy_from_bbox``. Each subsequent
        :meth:`update_marquee_preview` then only restores that frozen
        background and re-draws the one rectangle -- no ``ax.clear``, no
        model re-render, per motion event.

        This is a distinct implementation from :meth:`begin_obstacle_drag`
        -- its own preview artist, its own baked background, its own
        throttle clock -- so a ``Mode.SELECT`` marquee drag never shares
        state with a ``Mode.OBSTACLE`` resize, and the obstacle path's
        hardcoded ``"blue"`` styling is left untouched.

        Degrades gracefully: with no model rendered yet the fast-path is
        left disarmed (``_marquee_blit_background`` stays ``None``) so the
        next :meth:`update_marquee_preview` falls back to a defensive full
        render.

        :return: ``None``.
        """
        model = self._last_model
        if model is None:
            return

        # Render the current model first so the static scene under the
        # preview is up to date, then rebuild the per-element artist
        # handles.
        self.render(
            model,
            highlighted_nodes=self._last_highlighted_nodes,
            proposed_edges=self._last_proposed_edges,
            align_preview=self._last_align_preview,
        )

        rect = mpatches.Rectangle(
            (0.0, 0.0),
            0.0,
            0.0,
            linewidth=_MARQUEE_PREVIEW_LINEWIDTH,
            edgecolor=_MARQUEE_PREVIEW_EDGECOLOR,
            linestyle=_MARQUEE_PREVIEW_LINESTYLE,
            facecolor="none",
            zorder=_MARQUEE_PREVIEW_ZORDER,
            animated=True,
        )
        self._ax.add_patch(rect)
        self._marquee_preview_artist = rect

        # Bake the static scene (the animated rect is excluded) and
        # snapshot it.
        self._canvas.draw()
        self._marquee_blit_background = self._canvas.copy_from_bbox(
            self._ax.bbox
        )
        # Force the first ``update_marquee_preview`` to composite.
        self._last_marquee_blit_time = _BLIT_TIME_FORCE_FIRST

    def update_marquee_preview(
        self, x0: float, y0: float, x1: float, y1: float
    ) -> None:
        """Resize the marquee rectangle to span ``(x0, y0)``-``(x1, y1)`` and blit.

        Updates the rectangle's lower-left corner + width/height on every
        call (so the artist always holds the latest extent), then composites
        the frame by restoring the baked background and re-drawing only the
        rectangle. As with :meth:`update_obstacle_preview`, only the visible
        composite is throttled to ~60 Hz; a call arriving within
        :data:`_BLIT_MIN_INTERVAL_S` of the last composite updates the
        geometry but skips the restore/draw/blit.

        Defensive paths mirror the obstacle drag: if the fast-path was never
        armed (``_marquee_blit_background is None``) a full :meth:`render`
        is issued; if the composite raises on an exotic backend it is
        caught, the fast-path disarmed, and a full render issued for that
        tick so a marquee drag never crashes the editor.

        :param x0: Anchor x (drag start), in centimetres.
        :param y0: Anchor y (drag start), in centimetres.
        :param x1: Cursor x (current), in centimetres.
        :param y1: Cursor y (current), in centimetres.
        :return: ``None``.
        """
        rect = self._marquee_preview_artist
        if self._marquee_blit_background is None or rect is None:
            # Fast-path not armed: fall back to a plain repaint so the drag
            # is still reflected (defensive -- the app arms it on
            # anchor-set).
            if self._last_model is not None:
                self.render(
                    self._last_model,
                    highlighted_nodes=self._last_highlighted_nodes,
                    proposed_edges=self._last_proposed_edges,
                    align_preview=self._last_align_preview,
                )
            return

        # Always update the artist geometry, even on a throttled tick.
        xmin = min(x0, x1)
        ymin = min(y0, y1)
        rect.set_bounds(xmin, ymin, abs(x1 - x0), abs(y1 - y0))

        now = time.monotonic()
        if now - self._last_marquee_blit_time < _BLIT_MIN_INTERVAL_S:
            return

        try:
            self._canvas.restore_region(self._marquee_blit_background)
            self._ax.draw_artist(rect)
            self._canvas.blit(self._ax.bbox)
        except Exception:  # noqa: BLE001 -- exotic backend: never crash a drag.
            self._marquee_blit_background = None
            if self._last_model is not None:
                self.render(
                    self._last_model,
                    highlighted_nodes=self._last_highlighted_nodes,
                    proposed_edges=self._last_proposed_edges,
                    align_preview=self._last_align_preview,
                )
            return

        self._last_marquee_blit_time = now

    def end_marquee_drag(self) -> None:
        """Tear down the marquee blit fast-path and re-bake the static scene.

        Removes the preview rectangle, drops the drag state, and issues a
        full :meth:`render` so the scene returns to normal. Safe to call
        when no marquee drag is active (the preview artist is simply
        ``None``).

        :return: ``None``.
        """
        rect = self._marquee_preview_artist
        if rect is not None:
            try:
                rect.remove()
            except (ValueError, NotImplementedError):
                # Already detached (e.g. a full render cleared the axes):
                # fine.
                pass
        self._marquee_preview_artist = None
        self._marquee_blit_background = None
        self._last_marquee_blit_time = _BLIT_TIME_FORCE_FIRST

        if self._last_model is not None:
            self.render(
                self._last_model,
                highlighted_nodes=self._last_highlighted_nodes,
                proposed_edges=self._last_proposed_edges,
                align_preview=self._last_align_preview,
            )

    def _apply_drag_geometry(self, name: str, x: float, y: float) -> None:
        """Move the dragged node's animated artists to ``(x, y)``.

        Repositions the node's dot and label (applying the same
        :data:`_NODE_LABEL_OFFSET_CM` offset :meth:`_draw_nodes` uses) and
        slides the endpoint that equals *name* on each incident line/blind edge
        artist, leaving the opposite endpoint of every incident edge fixed. The
        model is NOT mutated here -- the editor application owns the model write;
        this method only moves the on-canvas artists.

        :param name: Name of the dragged node.
        :param x: New world x coordinate, in centimetres.
        :param y: New world y coordinate, in centimetres.
        :return: ``None``.
        """
        dot_artist = self._node_dot_artists.get(name)
        if dot_artist is not None:
            dot_artist.set_data([x], [y])
        label_artist = self._node_label_artists.get(name)
        if label_artist is not None:
            label_artist.set_position(
                (x + _NODE_LABEL_OFFSET_CM, y + _NODE_LABEL_OFFSET_CM)
            )
        for u, v, _kind, edge_artist in self._edge_artists:
            # An edge can loop u == v defensively; update both ends if so.
            xdata = list(edge_artist.get_xdata())
            ydata = list(edge_artist.get_ydata())
            if len(xdata) != 2 or len(ydata) != 2:
                # Defensive: edge artists are always 2-point lines, but never
                # index blindly into a malformed handle.
                continue
            if u == name:
                xdata[0] = x
                ydata[0] = y
            if v == name:
                xdata[1] = x
                ydata[1] = y
            if u == name or v == name:
                edge_artist.set_data(xdata, ydata)

    def _full_render_during_drag(self, name: str, x: float, y: float) -> None:
        """Fallback full render when the blit fast-path is unavailable.

        Used when :meth:`update_node_drag` cannot blit (fast-path disarmed,
        wrong node, or a backend that raised mid-composite). The drag must
        still show the node under the cursor without mutating the model -- the
        canvas never writes ``model.nodes`` during a drag (the caller dispatches
        the final ``MoveNode`` on release).

        Ordering matters: :meth:`render` does ``ax.clear()`` and rebuilds every
        artist from :attr:`_last_model`, whose node coordinates still hold the
        PRE-DRAG ORIGIN. So the live ``(x, y)`` geometry must be applied to the
        artists AFTER they are rebuilt, not before -- applying it first would be
        discarded by the clear-and-rebuild and the dragged node would snap back
        to its origin on every fallback tick.

        1. :meth:`render` rebuilds all artists at the model (origin) positions
           and draws that scene (with the cached overlay arguments).
        2. :meth:`_apply_drag_geometry` slides the freshly-rebuilt dot / label /
           incident-edge artists to the cursor (it looks them up from the dicts
           :meth:`render` just repopulated, so it MUST run after the render).
           It no-ops gracefully if *name* has no rebuilt artist (e.g. the node
           vanished from the model).
        3. A forced :meth:`FigureCanvasTkAgg.draw_idle` paints those moved
           artists, since :meth:`_apply_drag_geometry` only mutates artist data
           -- the render in step 1 already drew the origin-positioned scene.

        :param name: Name of the dragged node.
        :param x: New world x coordinate, in centimetres.
        :param y: New world y coordinate, in centimetres.
        :return: ``None``.
        """
        if self._last_model is None:
            return
        self.render(
            self._last_model,
            highlighted_nodes=self._last_highlighted_nodes,
            proposed_edges=self._last_proposed_edges,
            align_preview=self._last_align_preview,
        )
        self._apply_drag_geometry(name, x, y)
        self._canvas.draw_idle()

    # ------------------------------------------------------------------
    # Hit-testing + coordinate helpers
    # ------------------------------------------------------------------

    def find_node_at(
        self,
        x: float,
        y: float,
        tolerance_cm: float = _DEFAULT_HIT_TOLERANCE_CM,
    ) -> Optional[str]:
        """Return the closest node within *tolerance_cm* of ``(x, y)``.

        Searches the most recently rendered model (see :meth:`render`).
        Distance is plain Euclidean in centimetres. If multiple nodes are
        within tolerance, the closest one wins; ties are broken by the
        first matching name in dict iteration order.

        :param x: World x coordinate, in centimetres.
        :param y: World y coordinate, in centimetres.
        :param tolerance_cm: Maximum hit radius in centimetres. Default
            ``4.0`` cm, tuned for the default scatter dot size at the
            initial zoom level.
        :return: Node name as a ``str``, or ``None`` when no node is
            within tolerance (also returns ``None`` if :meth:`render` has
            never been called).
        """
        if self._last_model is None:
            return None
        if tolerance_cm <= 0.0:
            return None

        best_name: Optional[str] = None
        best_dist_sq: float = tolerance_cm * tolerance_cm
        for name, (nx, ny) in self._last_model.nodes.items():
            dx = float(nx) - x
            dy = float(ny) - y
            dist_sq = dx * dx + dy * dy
            if dist_sq <= best_dist_sq:
                # Use ``<=`` so an exact-radius hit also wins; tie-break
                # picks the first one in dict iteration order, which is
                # deterministic in Python 3.7+.
                if dist_sq < best_dist_sq or best_name is None:
                    best_dist_sq = dist_sq
                    best_name = name
        return best_name

    def find_nodes_in_rect(
        self, x0: float, y0: float, x1: float, y1: float
    ) -> list[str]:
        """Return every node whose coordinate falls inside the given box.

        Searches the most recently rendered model (see :meth:`render`). The
        box test is inclusive on all four edges (``x_min <= nx <= x_max``
        and ``y_min <= ny <= y_max``), matching the boundary-inclusive
        convention already used by :meth:`find_obstacle_at`. The corners
        need not be given in any particular order -- an "inverted" box
        (``x1 < x0`` and/or ``y1 < y0``, which a marquee dragged up-and-left
        naturally produces) is normalized before testing, so the caller
        never has to sort the anchor/cursor pair itself.

        :param x0: One corner's x coordinate, in centimetres.
        :param y0: One corner's y coordinate, in centimetres.
        :param x1: The opposite corner's x coordinate, in centimetres.
        :param y1: The opposite corner's y coordinate, in centimetres.
        :return: Node names inside the box, in the model's dict iteration
            order. Empty if no node matches, or if :meth:`render` has never
            been called.
        """
        if self._last_model is None:
            return []

        x_min, x_max = (x0, x1) if x0 <= x1 else (x1, x0)
        y_min, y_max = (y0, y1) if y0 <= y1 else (y1, y0)

        matches: list[str] = []
        for name, (nx, ny) in self._last_model.nodes.items():
            if x_min <= float(nx) <= x_max and y_min <= float(ny) <= y_max:
                matches.append(name)
        return matches

    def find_obstacle_at(self, x: float, y: float) -> Optional[int]:
        """Return the index of the topmost obstacle whose raw rectangle
        contains ``(x, y)``, or ``None`` if no obstacle contains the point.

        Hit-testing uses the *raw* (un-inflated) axis-aligned bounding box
        stored in ``model.obstacles`` -- the inflated polygon is a planner
        visualization and should not change which obstacle the user can
        click on. The point is considered inside an obstacle if it is
        within or on the boundary of the rectangle (``xmin <= x <= xmax``
        and ``ymin <= y <= ymax``).

        Iteration walks the obstacle list in reverse so that, when two
        obstacles overlap at ``(x, y)``, the one drawn *last* (i.e. the
        one visually on top) wins. Disjoint obstacles -- the common
        case -- are unaffected by the iteration order.

        :param x: World x coordinate, in centimetres.
        :param y: World y coordinate, in centimetres.
        :return: Index into :attr:`FieldModel.obstacles`, or ``None`` if
            the point misses every obstacle (also returns ``None`` if
            :meth:`render` has never been called).
        """
        if self._last_model is None:
            return None
        obstacles = self._last_model.obstacles
        for index in range(len(obstacles) - 1, -1, -1):
            xmin, ymin, xmax, ymax, _kind = obstacles[index]
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return index
        return None

    def figure_to_world(
        self, event
    ) -> Optional[tuple[float, float]]:
        """Convert a matplotlib mouse event to world coordinates (cm).

        Returns ``None`` for any event that did not land inside this
        canvas' axes (which covers both "outside the plot box" and
        "inside a different axes" cases). Otherwise returns a fresh
        ``(x_cm, y_cm)`` tuple of floats.

        :param event: A matplotlib :class:`MouseEvent`.
        :return: ``(world_x_cm, world_y_cm)`` or ``None``.
        """
        if not self._event_in_axes(event):
            return None
        if event.xdata is None or event.ydata is None:
            return None
        return float(event.xdata), float(event.ydata)

    def _clamped_world_coords(
        self, event
    ) -> Optional[tuple[float, float]]:
        """Return world coords for *event*, clamped to the axes' data limits.

        For an in-axes event this is identical to :meth:`figure_to_world`.
        For an off-axes event (the cursor is over the side panel or window
        chrome at release), ``event.xdata`` / ``event.ydata`` are ``None``,
        so the pixel coordinates (``event.x`` / ``event.y``) are projected
        back into data space via the axes' inverse data transform and then
        clamped to the current ``xlim`` / ``ylim``. This lets a drag that is
        released off the plot still resolve to the nearest valid field
        position rather than being dropped (#61).

        :param event: A matplotlib :class:`MouseEvent`.
        :return: ``(world_x_cm, world_y_cm)`` clamped to the axes bounds, or
            ``None`` when neither data nor pixel coordinates are available.
        """
        # Fast path: cursor inside the axes -> exact data coordinates.
        in_axes = self.figure_to_world(event)
        if in_axes is not None:
            return in_axes

        # Off-axes (or no inaxes): recover data coords from pixel position.
        px = getattr(event, "x", None)
        py = getattr(event, "y", None)
        if px is None or py is None:
            return None
        try:
            data_x, data_y = self._ax.transData.inverted().transform(
                (px, py)
            )
        except (ValueError, TypeError):
            return None

        # Clamp into the visible data window so the editor never receives a
        # position outside the field rectangle from an off-plot release.
        x_lo, x_hi = self._ax.get_xlim()
        y_lo, y_hi = self._ax.get_ylim()
        # ``get_xlim``/``get_ylim`` are normally ascending here (render sets
        # them via ``set_xlim(0, width)``) but guard against an inverted axis.
        x_min, x_max = (x_lo, x_hi) if x_lo <= x_hi else (x_hi, x_lo)
        y_min, y_max = (y_lo, y_hi) if y_lo <= y_hi else (y_hi, y_lo)
        clamped_x = min(max(float(data_x), x_min), x_max)
        clamped_y = min(max(float(data_y), y_min), y_max)
        return clamped_x, clamped_y

    # ------------------------------------------------------------------
    # Internal: matplotlib event handlers
    # ------------------------------------------------------------------

    def _on_button_press(self, event) -> None:
        """Forward an in-axes ``button_press_event`` to :attr:`on_click`."""
        coords = self.figure_to_world(event)
        if coords is None:
            return
        button = self._normalize_button(event.button)
        if button is None:
            return
        # Latch the active button so motion events know which drag is in
        # progress. Releasing the button clears this in
        # :meth:`_on_button_release`.
        self._active_button = button
        # ``getattr`` (rather than ``event.key`` directly) tolerates minimal
        # hand-built test events that omit the attribute; real matplotlib
        # ``MouseEvent``s always carry ``key`` (``None`` when no modifier is
        # held).
        key = getattr(event, "key", None)
        if self.on_click is not None:
            self.on_click(coords[0], coords[1], button, key)

    def _on_motion_notify(self, event) -> None:
        """Forward an in-axes ``motion_notify_event`` to :attr:`on_drag`.

        Only fires while a mouse button is held (i.e. between a press and
        the matching release). Bare hover events without a held button
        are dropped.
        """
        if self._active_button is None:
            return
        coords = self.figure_to_world(event)
        if coords is None:
            return
        if self.on_drag is not None:
            self.on_drag(coords[0], coords[1])

    def _on_button_release(self, event) -> None:
        """Forward a ``button_release_event`` to :attr:`on_release`.

        A release ends the in-progress drag, so the latched
        ``_active_button`` is cleared UNCONDITIONALLY — the canvas only ever
        tracks one button at a time. Clearing only on a button-matched release
        (the old behaviour) left the latch stuck when a backend delivered a
        *different* button's release, or ``None`` for an out-of-window release,
        with ``_on_motion_notify`` then firing ``on_drag`` on every bare hover.

        Unlike :meth:`_on_button_press` / :meth:`_on_motion_notify`, the
        release fires :attr:`on_release` even when it lands *outside* the
        axes. An off-axes release passes the cursor position clamped to the
        axes' data limits (see :meth:`_clamped_world_coords`). Without this,
        a drag released over the side panel or window chrome would skip the
        editor's release logic entirely — a MOVE-mode drag would leave the
        node live-mutated with no undo command, and an OBSTACLE drag would
        never finish. Releasing off the plot is a common gesture when
        dragging toward the field edge. When the backend reports ``None`` for
        the button on such a release, the LATCHED button is passed through so
        the release still commits (the bug that left a node visually stuck
        with no undo entry).

        As with :meth:`_on_button_press`, the modifier key (``event.key``)
        is forwarded as the 4th argument to :attr:`on_release`, read via
        ``getattr`` so a minimal hand-built test event without the
        attribute still works.
        """
        released_button = self._normalize_button(event.button)
        # A release ends any in-progress drag; clear the single-button latch
        # unconditionally so a stray / mismatched / None-button release can
        # never leave bare-hover motion firing on_drag.
        active_button = self._active_button
        self._active_button = None

        # No drag was in progress: ignore the release entirely.
        if active_button is None:
            return

        coords = self._clamped_world_coords(event)
        if coords is None:
            return
        if self.on_release is not None:
            # Prefer the released button; fall back to the latched one when the
            # backend reports None for an out-of-window release, so a MOVE /
            # OBSTACLE drag released off the plot still commits.
            button = (
                released_button
                if released_button is not None
                else active_button
            )
            key = getattr(event, "key", None)
            self.on_release(coords[0], coords[1], button, key)

    # ------------------------------------------------------------------
    # Internal: rendering helpers
    # ------------------------------------------------------------------

    def _draw_background(
        self,
        model: FieldModel,
        width_cm: float,
        height_cm: float,
    ) -> None:
        """Overlay the field PNG (or the bare themed background if missing).

        Resolution order matches :mod:`tools.path_planner.visualizer`:
        absolute paths are used as-is; relative paths are resolved
        against ``tools/path_planner/``. A missing file is non-fatal —
        the axes are simply left with the warm PAPER facecolor that
        :meth:`render` applied from the theme before this helper ran.

        The decoded PNG is cached per-instance in :attr:`_bg_cache`, keyed by
        the resolved absolute path string, so it is read from disk only once
        and reused on every subsequent render. This matters because
        :meth:`render` runs on every mouse-motion tick during a drag; without
        the cache the PNG would be re-decoded dozens of times per second. A
        missing or unreadable PNG caches a "no background" sentinel so the
        doomed decode (and its warning) is not re-attempted every frame.

        :param model: Source of the ``map_png`` path and field extent.
        :param width_cm: Field width in centimetres (image extent x-max).
        :param height_cm: Field height in centimetres (image extent y-max).
        :return: ``None``.
        """
        map_path = self._resolve_map_png(model)
        if map_path is None:
            # No ``map_png`` configured at all; the warm themed axes facecolor
            # (applied in ``render``) is fine. Nothing to cache or draw.
            return

        image = self._cached_background(map_path)
        if image is _BACKGROUND_NONE:
            # Missing/unreadable PNG (already warned + cached as the
            # sentinel). Render on the warm themed axes facecolor.
            return

        # ``origin="upper"`` (vs. the visualizer's ``"lower"``) flips the
        # PNG vertically so the editor view matches the operator's
        # physical orientation of the field mat. Nodes, edges, obstacles,
        # and axis labels stay in the model's SW-origin frame -- only the
        # image's pixel-to-extent mapping is mirrored.
        # Omit ``aspect`` so imshow inherits the axes' ``set_aspect("equal")``
        # set in :meth:`render`. Passing ``aspect="auto"`` would override
        # that and re-stretch the field rectangle on every window resize.
        self._ax.imshow(
            image,
            extent=(0.0, width_cm, 0.0, height_cm),
            origin="upper",
            alpha=_BACKGROUND_ALPHA,
            zorder=_BACKGROUND_ZORDER,
        )

    def _cached_background(self, map_path: Path) -> object:
        """Return the decoded background array for *map_path*, caching it.

        On a cache miss the PNG is decoded from disk exactly once via
        ``mpimg.imread`` and the resulting array is stored under the resolved
        absolute path string. If the file is missing or ``imread`` raises, the
        :data:`_BACKGROUND_NONE` sentinel is cached instead (warning once, the
        first time the failure is recorded for that path) so the doomed decode
        is never retried on subsequent renders. On a cache hit the stored
        value — a decoded array or the sentinel — is returned untouched
        without touching the disk.

        :param map_path: Resolved absolute path to the candidate map PNG (from
            :meth:`_resolve_map_png`).
        :return: The decoded image array on success, or :data:`_BACKGROUND_NONE`
            when the PNG is missing or could not be decoded.
        """
        key = str(map_path)
        cached = self._bg_cache.get(key, None)
        if cached is not None:
            return cached

        if not map_path.exists():
            # Cache the failure so we don't re-stat / re-warn every frame.
            # A missing file is non-fatal: the visualizer parity behaviour is
            # to render on the default white facecolor, so we stay silent here
            # to match the pre-cache contract (which only warned on a decode
            # error, not on a plain absent file).
            self._bg_cache[key] = _BACKGROUND_NONE
            return _BACKGROUND_NONE

        try:
            image = mpimg.imread(key)
        except (FileNotFoundError, PermissionError, OSError, ValueError) as exc:
            warnings.warn(
                f"FieldCanvas: failed to load map.png at {map_path} "
                f"({type(exc).__name__}: {exc}); rendering without "
                "background.",
                UserWarning,
                stacklevel=2,
            )
            self._bg_cache[key] = _BACKGROUND_NONE
            return _BACKGROUND_NONE

        self._bg_cache[key] = image
        return image

    def _draw_obstacles(self, model: FieldModel) -> None:
        """Draw raw rectangles + C-space inflated polygons for obstacles."""
        if not model.obstacles:
            return
        inflated_polys = inflate_obstacles(
            model.obstacles, model.robot_radius_cm
        )
        for idx, obs in enumerate(model.obstacles):
            x_min, y_min, x_max, y_max, _kind = obs
            width = x_max - x_min
            height = y_max - y_min
            raw_rect = mpatches.Rectangle(
                (x_min, y_min),
                width,
                height,
                linewidth=_OBSTACLE_RAW_LINEWIDTH,
                edgecolor=_OBSTACLE_RAW_EDGECOLOR,
                facecolor="none",
                linestyle=_OBSTACLE_RAW_LINESTYLE,
                zorder=_OBSTACLE_RAW_ZORDER,
            )
            self._ax.add_patch(raw_rect)

            inflated = inflated_polys[idx]
            # ``inflate_obstacles`` always returns a ``shapely.Polygon``;
            # ``.exterior.xy`` yields the closed ring as two coord lists.
            xs, ys = inflated.exterior.xy
            inflated_patch = mpatches.Polygon(
                list(zip(xs, ys)),
                closed=True,
                facecolor=_OBSTACLE_INFLATED_FACECOLOR,
                edgecolor=_OBSTACLE_INFLATED_EDGECOLOR,
                alpha=_OBSTACLE_INFLATED_ALPHA,
                zorder=_OBSTACLE_INFLATED_ZORDER,
            )
            self._ax.add_patch(inflated_patch)

    def _draw_edges(self, model: FieldModel) -> None:
        """Draw line-follow + blind edges, retaining a handle per edge.

        Each edge's :class:`~matplotlib.lines.Line2D` is appended to
        :attr:`_edge_artists` as a ``(u, v, kind, artist)`` tuple (``kind`` is
        ``"line"`` or ``"blind"``). The drag fast-path uses those handles to
        find the edges incident to a dragged node and to slide the moved
        endpoint without a full re-render.
        """
        nodes = model.nodes
        for u, v in model.line_segments:
            if u not in nodes or v not in nodes:
                # Defensive: the GUI may briefly hold a half-deleted state
                # mid-edit. Skip rather than crash the render.
                continue
            ux, uy = nodes[u]
            vx, vy = nodes[v]
            (artist,) = self._ax.plot(
                [ux, vx],
                [uy, vy],
                color=_EDGE_LINE_COLOR,
                linestyle=_EDGE_LINE_LINESTYLE,
                linewidth=_EDGE_LINE_LINEWIDTH,
                zorder=_EDGE_ZORDER,
            )
            self._edge_artists.append((u, v, "line", artist))
        for u, v in model.blind_edges:
            if u not in nodes or v not in nodes:
                continue
            ux, uy = nodes[u]
            vx, vy = nodes[v]
            (artist,) = self._ax.plot(
                [ux, vx],
                [uy, vy],
                color=_EDGE_BLIND_COLOR,
                linestyle=_EDGE_BLIND_LINESTYLE,
                linewidth=_EDGE_BLIND_LINEWIDTH,
                zorder=_EDGE_ZORDER,
            )
            self._edge_artists.append((u, v, "blind", artist))

    def _draw_proposed_edges(
        self,
        model: FieldModel,
        proposed_edges: list[tuple[str, str, str]],
    ) -> None:
        """Overlay the given (a, b, kind) edges in dashed teal above committed edges.

        The third tuple element (``kind``) is currently informational only — every
        proposed edge is styled identically. Unknown node names are silently
        skipped to match the defensive style used elsewhere in this class
        (the model may briefly hold a half-deleted state mid-edit).

        :param model: Source of node coordinates.
        :param proposed_edges: List of ``(a, b, kind)`` triples.
        """
        nodes = model.nodes
        for u, v, _kind in proposed_edges:
            if u not in nodes or v not in nodes:
                continue
            ux, uy = nodes[u]
            vx, vy = nodes[v]
            self._ax.plot(
                [ux, vx],
                [uy, vy],
                color=_PROPOSED_EDGE_COLOR,
                linestyle=_PROPOSED_EDGE_LINESTYLE,
                linewidth=_PROPOSED_EDGE_LINEWIDTH,
                zorder=_PROPOSED_EDGE_ZORDER,
            )

    def _draw_nodes(self, model: FieldModel) -> None:
        """Draw a coloured dot + name label per node, retaining each handle.

        Unlike the old type-grouped single ``scatter`` per node type, each
        node gets its OWN ``Line2D`` dot (drawn with ``marker="o"`` /
        ``linestyle="none"``) and its own ``Text`` label, retained in
        :attr:`_node_dot_artists` / :attr:`_node_label_artists` keyed by node
        name. Dedicated per-node artists are what let the drag fast-path
        animate a single node out of the scene; a shared PathCollection could
        not be re-positioned one point at a time.

        ``ax.plot`` sizes markers by diameter in points (``markersize``),
        whereas ``ax.scatter`` sized them by area in points^2 (``s``); see
        :data:`_NODE_DOT_MARKERSIZE` for the area->diameter conversion that
        keeps the dots visually identical to the pre-refactor look.

        ``node_types`` is the source of truth; nodes missing a type entry fall
        back to ``"line"`` (consistent with ``visualizer.py``). Sorting by
        ``(type, name)`` preserves the previous, deterministic draw order.
        """
        entries: list[tuple[str, float, float, str]] = []
        for name, (x, y) in model.nodes.items():
            node_type = model.node_types.get(name, "line")
            entries.append((node_type, float(x), float(y), name))
        for node_type, x, y, name in sorted(entries):
            color = _NODE_COLOR.get(node_type, _NODE_COLOR_FALLBACK)
            (dot_artist,) = self._ax.plot(
                [x],
                [y],
                marker="o",
                linestyle="none",
                color=color,
                markersize=_NODE_DOT_MARKERSIZE,
                zorder=_NODE_ZORDER,
            )
            self._node_dot_artists[name] = dot_artist
            label_artist = self._ax.text(
                x + _NODE_LABEL_OFFSET_CM,
                y + _NODE_LABEL_OFFSET_CM,
                name,
                fontsize=_NODE_LABEL_FONTSIZE,
                color=color,
                zorder=_NODE_LABEL_ZORDER,
            )
            self._node_label_artists[name] = label_artist

    def _draw_highlight(
        self,
        model: FieldModel,
        highlighted_nodes: list[str],
    ) -> None:
        """Outline the given nodes with a thick terracotta ring above all else.

        Unknown node names are silently skipped — the editor may pass a
        stale selection during a multi-step mutation and we don't want to
        crash the render over it.
        """
        xs: list[float] = []
        ys: list[float] = []
        for name in highlighted_nodes:
            coords = model.nodes.get(name)
            if coords is None:
                continue
            xs.append(float(coords[0]))
            ys.append(float(coords[1]))
        if not xs:
            return
        self._ax.scatter(
            xs,
            ys,
            s=_HIGHLIGHT_DOT_SIZE,
            facecolors="none",
            edgecolors=_HIGHLIGHT_EDGECOLOR,
            linewidths=_HIGHLIGHT_LINEWIDTH,
            zorder=_HIGHLIGHT_ZORDER,
        )

    def _draw_align_preview(self, align_preview: "AlignPreview") -> None:
        """Overlay the make-straight guide line + ghost target dots.

        Draws ``align_preview.guide`` as a dashed teal line (styled like the
        proposed-edge overlay: same colour token, same "this is where things
        would go" meaning) and, when ``align_preview.ghosts`` is non-empty,
        a hollow teal dot at each mover's target coordinate. A node already
        sitting exactly on its target is not a "mover" and has no entry in
        ``ghosts`` (see :mod:`editor.align`), so it gets no ghost.

        This method only ever runs from the normal :meth:`render` pass, not
        from any blit fast-path -- the preview changes only when the
        selection or axis choice changes, both of which already trigger a
        full render, so there is no per-motion-event drag to fast-path here.

        Accepts any duck-typed object exposing ``.guide`` (an
        ``((x0, y0), (x1, y1))`` pair, in centimetres) and ``.ghosts`` (a
        ``dict[str, tuple[float, float]]`` mapping mover name to target
        ``(x, y)``) so this module never needs a hard runtime import of
        :mod:`tools.path_planner.editor.align` -- see the ``TYPE_CHECKING``
        guard at the top of this file.

        :param align_preview: The preview payload built by
            ``editor.align.build_preview``.
        :return: ``None``.
        """
        (x0, y0), (x1, y1) = align_preview.guide
        self._ax.plot(
            [x0, x1],
            [y0, y1],
            color=theme.TEAL,
            linestyle=_ALIGN_GUIDE_LINESTYLE,
            linewidth=_ALIGN_GUIDE_LINEWIDTH,
            zorder=_ALIGN_GUIDE_ZORDER,
        )

        ghosts = align_preview.ghosts
        if not ghosts:
            return
        ghost_xs = [float(gx) for gx, _gy in ghosts.values()]
        ghost_ys = [float(gy) for _gx, gy in ghosts.values()]
        self._ax.scatter(
            ghost_xs,
            ghost_ys,
            s=_ALIGN_GHOST_DOT_SIZE,
            facecolors="none",
            edgecolors=theme.TEAL,
            linewidths=_ALIGN_GHOST_LINEWIDTH,
            zorder=_ALIGN_GHOST_ZORDER,
        )

    # ------------------------------------------------------------------
    # Internal: small utilities
    # ------------------------------------------------------------------

    def _event_in_axes(self, event) -> bool:
        """Return ``True`` iff *event* originated inside this canvas' axes."""
        return getattr(event, "inaxes", None) is self._ax

    @staticmethod
    def _normalize_button(raw_button) -> Optional[int]:
        """Coerce a matplotlib button value to ``1`` (left) or ``3`` (right).

        Middle clicks (button 2) and unknown values are mapped to
        ``None`` so the callback layer can drop them silently. We do this
        in one place so :meth:`_on_button_press` / :meth:`_on_button_release`
        never have to think about non-LMB/RMB clicks.
        """
        try:
            button = int(raw_button)
        except (TypeError, ValueError):
            return None
        if button == _MOUSE_BUTTON_LEFT:
            return _MOUSE_BUTTON_LEFT
        if button == _MOUSE_BUTTON_RIGHT:
            return _MOUSE_BUTTON_RIGHT
        return None

    @staticmethod
    def _field_extent_cm(model: FieldModel) -> tuple[float, float]:
        """Return ``(width_cm, height_cm)`` derived from ``model.field_mm``."""
        width_mm, height_mm = model.field_mm
        return float(width_mm) / 10.0, float(height_mm) / 10.0

    def _resolve_map_png(self, model: FieldModel) -> Optional[Path]:
        """Return absolute :class:`Path` to ``model.map_png`` or ``None``.

        Relative paths are resolved against the planner root
        (``tools/path_planner/``), matching the resolution logic in
        :mod:`tools.path_planner.visualizer` and
        :mod:`tools.path_planner.pick_xy`.
        """
        raw = getattr(model, "map_png", None)
        if not raw:
            return None
        p = Path(raw)
        if not p.is_absolute():
            p = _PLANNER_ROOT / p
        return p
