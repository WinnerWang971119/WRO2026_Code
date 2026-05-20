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

import warnings
from pathlib import Path
from typing import Callable, Optional

import tkinter as tk

import matplotlib

matplotlib.use("TkAgg")  # MUST come before any other matplotlib import.

import matplotlib.image as mpimg  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
from matplotlib.axes import Axes  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from tools.path_planner.editor.model import FieldModel  # noqa: E402
from tools.path_planner.graph_builder import inflate_obstacles  # noqa: E402


# ---------------------------------------------------------------------------
# Rendering constants. Mirrors the static visualizer's look-and-feel but is
# intentionally duplicated here -- importing the visualizer would race the
# matplotlib backend selector (Agg vs TkAgg) and break headless PNG dumps.
# ---------------------------------------------------------------------------

# Background image alpha (matches ``visualizer.py``).
_BACKGROUND_ALPHA: float = 0.6
_BACKGROUND_ZORDER: int = 0

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

# Edge styling.
_EDGE_LINE_COLOR: str = "black"
_EDGE_LINE_LINESTYLE: str = "solid"
_EDGE_LINE_LINEWIDTH: float = 1.5

_EDGE_BLIND_COLOR: str = "red"
_EDGE_BLIND_LINESTYLE: str = "dashed"
_EDGE_BLIND_LINEWIDTH: float = 1.5

_EDGE_ZORDER: int = 3

# Proposed-edge preview styling (used by FieldCanvas.render's proposed_edges
# kwarg -- drawn ABOVE committed edges, BELOW the highlight ring, so the
# operator can see the proposal sitting on top of the existing graph.
_PROPOSED_EDGE_COLOR: str = "cyan"
_PROPOSED_EDGE_LINESTYLE: tuple = (0, (3, 3))  # dashed
_PROPOSED_EDGE_LINEWIDTH: float = 1.8
_PROPOSED_EDGE_ZORDER: int = 4  # between _EDGE_ZORDER (3) and _NODE_ZORDER (6)

# Node dots + labels.
_NODE_DOT_SIZE: float = 60.0
_NODE_ZORDER: int = 6
_NODE_LABEL_FONTSIZE: int = 8
_NODE_LABEL_OFFSET_CM: float = 1.5
_NODE_LABEL_ZORDER: int = 7

_NODE_COLOR: dict[str, str] = {
    "line": "black",
    "blind": "red",
}
_NODE_COLOR_FALLBACK: str = "gray"

# Highlight ring drawn on top of the selected node(s).
_HIGHLIGHT_COLOR: str = "yellow"
_HIGHLIGHT_LINEWIDTH: float = 2.5
_HIGHLIGHT_EDGECOLOR: str = "yellow"
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

# Mouse button numbers per the matplotlib MouseEvent contract.
_MOUSE_BUTTON_LEFT: int = 1
_MOUSE_BUTTON_RIGHT: int = 3

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
    receive the matplotlib button number (``1`` = left, ``3`` = right).
    Callbacks are only invoked when the underlying event is inside this
    canvas' axes; out-of-axes events are silently dropped.

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
        # is invoked only when the originating event is inside ``self._ax``.
        self.on_click: Optional[
            Callable[[float, float, int], None]
        ] = None
        self.on_drag: Optional[Callable[[float, float], None]] = None
        self.on_release: Optional[
            Callable[[float, float, int], None]
        ] = None

        # Cache the most recently rendered model so callers (and the
        # find-node helper) can ask "what's at (x, y)?" without
        # re-passing the model on every query.
        self._last_model: Optional[FieldModel] = None

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
    ) -> None:
        """Redraw the canvas from scratch for the given model.

        Clears the axes, restores the cosmetic limits / labels / grid,
        then paints in fixed z-order: background → obstacles → edges →
        proposed-edge overlay → nodes → highlight ring. Calling this
        after every model mutation is the editor's standard refresh path.

        :param model: The :class:`FieldModel` to render. Stored on the
            canvas for use by :meth:`find_node_at`.
        :param highlighted_nodes: Optional list of node names to draw a
            thick yellow ring around (on top of the regular node dots).
            Unknown names are silently skipped.
        :param proposed_edges: Optional list of ``(a, b, kind)`` triples
            drawn as a dashed cyan overlay above the committed edges.
            ``kind`` is informational (caller may use it for status
            reporting); the canvas styles all proposed edges identically.
            Unknown node names are silently skipped (consistent with
            ``highlighted_nodes``).
        :return: ``None``.
        """
        self._last_model = model

        ax = self._ax
        ax.clear()

        width_cm, height_cm = self._field_extent_cm(model)

        ax.set_aspect("equal")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_xlim(0.0, width_cm)
        ax.set_ylim(0.0, height_cm)
        ax.grid(True)

        self._draw_background(model, width_cm, height_cm)
        self._draw_obstacles(model)
        self._draw_edges(model)
        if proposed_edges:
            self._draw_proposed_edges(model, proposed_edges)
        self._draw_nodes(model)
        if highlighted_nodes:
            self._draw_highlight(model, highlighted_nodes)

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
        if self.on_click is not None:
            self.on_click(coords[0], coords[1], button)

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
        """Forward an in-axes ``button_release_event`` to :attr:`on_release`.

        Always clears the latched ``_active_button`` once the matching
        release is observed, even if the release happens outside the
        axes — otherwise a drag that ends off-canvas would leave the
        canvas stuck thinking the mouse is still down.
        """
        released_button = self._normalize_button(event.button)
        # Clear the latch regardless of whether the release is in-axes.
        # Without this, dragging off the canvas and releasing there would
        # leave the next motion event firing as if a drag were still in
        # progress.
        if released_button is not None and released_button == self._active_button:
            self._active_button = None
        elif released_button is None and self._active_button is not None:
            # Some matplotlib backends pass ``None`` for the button on
            # an out-of-window release. Be conservative and clear the
            # latch in that case too.
            self._active_button = None

        coords = self.figure_to_world(event)
        if coords is None or released_button is None:
            return
        if self.on_release is not None:
            self.on_release(coords[0], coords[1], released_button)

    # ------------------------------------------------------------------
    # Internal: rendering helpers
    # ------------------------------------------------------------------

    def _draw_background(
        self,
        model: FieldModel,
        width_cm: float,
        height_cm: float,
    ) -> None:
        """Overlay the field PNG (or a plain white background if missing).

        Resolution order matches :mod:`tools.path_planner.visualizer`:
        absolute paths are used as-is; relative paths are resolved
        against ``tools/path_planner/``. A missing file is non-fatal —
        the axes are simply left with their default white facecolor.
        """
        map_path = self._resolve_map_png(model)
        if map_path is None or not map_path.exists():
            # No background image; the default white axes facecolor is
            # fine. Render nothing.
            return
        try:
            img = mpimg.imread(str(map_path))
        except (FileNotFoundError, PermissionError, OSError, ValueError) as exc:
            warnings.warn(
                f"FieldCanvas: failed to load map.png at {map_path} "
                f"({type(exc).__name__}: {exc}); rendering without "
                "background.",
                UserWarning,
                stacklevel=2,
            )
            return
        # ``origin="upper"`` (vs. the visualizer's ``"lower"``) flips the
        # PNG vertically so the editor view matches the operator's
        # physical orientation of the field mat. Nodes, edges, obstacles,
        # and axis labels stay in the model's SW-origin frame -- only the
        # image's pixel-to-extent mapping is mirrored.
        self._ax.imshow(
            img,
            extent=(0.0, width_cm, 0.0, height_cm),
            origin="upper",
            alpha=_BACKGROUND_ALPHA,
            zorder=_BACKGROUND_ZORDER,
            aspect="auto",
        )

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
        """Draw line-follow + blind edges with the standard styling."""
        nodes = model.nodes
        for u, v in model.line_segments:
            if u not in nodes or v not in nodes:
                # Defensive: the GUI may briefly hold a half-deleted state
                # mid-edit. Skip rather than crash the render.
                continue
            ux, uy = nodes[u]
            vx, vy = nodes[v]
            self._ax.plot(
                [ux, vx],
                [uy, vy],
                color=_EDGE_LINE_COLOR,
                linestyle=_EDGE_LINE_LINESTYLE,
                linewidth=_EDGE_LINE_LINEWIDTH,
                zorder=_EDGE_ZORDER,
            )
        for u, v in model.blind_edges:
            if u not in nodes or v not in nodes:
                continue
            ux, uy = nodes[u]
            vx, vy = nodes[v]
            self._ax.plot(
                [ux, vx],
                [uy, vy],
                color=_EDGE_BLIND_COLOR,
                linestyle=_EDGE_BLIND_LINESTYLE,
                linewidth=_EDGE_BLIND_LINEWIDTH,
                zorder=_EDGE_ZORDER,
            )

    def _draw_proposed_edges(
        self,
        model: FieldModel,
        proposed_edges: list[tuple[str, str, str]],
    ) -> None:
        """Overlay the given (a, b, kind) edges in dashed cyan above committed edges.

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
        """Draw a coloured dot + name label per node."""
        # Group by type so we can call ``scatter`` once per type instead
        # of once per node. ``node_types`` is the source of truth; nodes
        # missing a type entry fall back to ``"line"`` (consistent with
        # ``visualizer.py``).
        by_type: dict[str, list[tuple[float, float, str]]] = {}
        for name, (x, y) in model.nodes.items():
            node_type = model.node_types.get(name, "line")
            by_type.setdefault(node_type, []).append(
                (float(x), float(y), name)
            )
        for node_type, entries in sorted(by_type.items()):
            xs = [e[0] for e in entries]
            ys = [e[1] for e in entries]
            color = _NODE_COLOR.get(node_type, _NODE_COLOR_FALLBACK)
            self._ax.scatter(
                xs,
                ys,
                color=color,
                s=_NODE_DOT_SIZE,
                zorder=_NODE_ZORDER,
            )
            for x, y, name in entries:
                self._ax.text(
                    x + _NODE_LABEL_OFFSET_CM,
                    y + _NODE_LABEL_OFFSET_CM,
                    name,
                    fontsize=_NODE_LABEL_FONTSIZE,
                    color=color,
                    zorder=_NODE_LABEL_ZORDER,
                )

    def _draw_highlight(
        self,
        model: FieldModel,
        highlighted_nodes: list[str],
    ) -> None:
        """Outline the given nodes with a thick yellow ring above all else.

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
