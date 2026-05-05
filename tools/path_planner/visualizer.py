"""Map visualizer for the WRO 2026 path planner.

Renders a matplotlib PNG showing:

* Nodes colored by type: ``line`` = black, ``projection`` = blue, ``goal`` = red.
* Edges colored by type: ``LINE`` = black solid, ``PROJECTION`` = blue solid,
  ``BLIND`` = red dashed.
* Obstacles drawn twice: raw bounding rectangle as a thin dashed outline;
  C-space inflated polygon (via :func:`graph_builder.inflate_obstacles`) as a
  filled gray polygon.
* An optional *highlighted_path*: a list of node names whose consecutive
  directed edges are overdrawn in green.

Usage::

    from tools.path_planner import field_data, graph_builder, visualizer

    g = graph_builder.build_graph(field_data)
    visualizer.render_map(field_data, g, highlighted_path=["A", "E", "C"])

The Agg backend is selected at module import time so the module works
headless (no Tkinter, Qt, or display server required on Diego's Windows dev
machine or any CI runner).
"""

from __future__ import annotations

# Agg backend MUST be set before the first pyplot import. Using the
# matplotlib.use() call here — not in a conftest or caller — guarantees
# the backend is set regardless of import order.
import matplotlib  # noqa: E402 (must precede pyplot import)

matplotlib.use("Agg")

import warnings  # noqa: E402
from pathlib import Path
from typing import Iterable, Optional

import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import networkx as nx  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from tools.path_planner.graph_builder import (  # noqa: E402
    EDGE_BLIND,
    EDGE_LINE,
    EDGE_PROJECTION,
    inflate_obstacles,
)

# ---------------------------------------------------------------------------
# Public constants
# ---------------------------------------------------------------------------

DEFAULT_OUTPUT: Path = Path("tools/path_planner/output/map.png")

# Node-type → scatter color mapping.
_NODE_COLOR: dict[str, str] = {
    "line":       "black",
    "projection": "blue",
    "goal":       "red",
}
_NODE_ZORDER: dict[str, int] = {
    "line":       4,
    "projection": 4,
    "goal":       4,
}

# Edge-type → (color, linestyle, linewidth) mapping.
_EDGE_STYLE: dict[str, tuple[str, str, float]] = {
    EDGE_LINE:       ("black", "solid",  1.5),
    EDGE_PROJECTION: ("blue",  "solid",  1.5),
    EDGE_BLIND:      ("red",   "dashed", 1.5),
}

_DEFAULT_TITLE = "WRO 2026 Path Planner — Toy Field"


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def render_map(
    field_data,
    graph: nx.DiGraph,
    *,
    out_path: Path = DEFAULT_OUTPUT,
    highlighted_path: Optional[Iterable[str]] = None,
    title: Optional[str] = None,
) -> Path:
    """Render the field map as a PNG at *out_path* and return *out_path*.

    :param field_data: the ``tools.path_planner.field_data`` module (or any
        object exposing ``NODES``, ``NODE_TYPES``, ``OBSTACLES``, and
        ``ROBOT_RADIUS_CM``).
    :param graph: a :class:`networkx.DiGraph` as returned by
        :func:`~tools.path_planner.graph_builder.build_graph`. Every node
        must carry ``x_cm`` / ``y_cm`` / ``type`` attributes; every edge
        must carry a ``type`` attribute.
    :param out_path: destination PNG file path. Created (including any missing
        parent directories) if it does not already exist.
    :param highlighted_path: optional ordered list of node names forming a
        path. Consecutive pairs are overdrawn in green with a slightly heavier
        linewidth. If a consecutive pair has no directed edge in *graph*, a
        :class:`UserWarning` is emitted and that segment is skipped — the
        rest of the path is still drawn.
    :param title: figure title. Defaults to
        ``"WRO 2026 Path Planner — Toy Field"`` if ``None``.
    :return: *out_path* (the resolved destination path).

    .. note::
        The figure is **always** closed after ``savefig`` to prevent
        matplotlib's figure-leak accumulation across repeated calls in tests.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect("equal")
    ax.set_title(title if title is not None else _DEFAULT_TITLE, fontsize=12)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")

    # 1. Draw obstacles: raw outline first, inflated fill on top.
    _draw_obstacles(ax, field_data)

    # 2. Draw graph edges.
    _draw_edges(ax, graph)

    # 3. Highlighted path overlay (before nodes so nodes render on top).
    if highlighted_path is not None:
        _draw_highlighted_path(ax, graph, list(highlighted_path))

    # 4. Draw nodes (scatter per type to produce distinct legend handles).
    _draw_nodes(ax, graph)

    # 5. Build and attach the legend.
    _attach_legend(ax)

    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight", dpi=150)
    plt.close(fig)

    return out_path


# ---------------------------------------------------------------------------
# Private helpers
# ---------------------------------------------------------------------------


def _draw_obstacles(ax: plt.Axes, field_data) -> None:
    """Draw raw obstacle rectangles (dashed outline) and inflated polygons (gray fill)."""
    obstacles = field_data.OBSTACLES
    robot_radius_cm = field_data.ROBOT_RADIUS_CM
    inflated_polys = inflate_obstacles(obstacles, robot_radius_cm)

    for idx, obs in enumerate(obstacles):
        x_min, y_min, x_max, y_max, _kind = obs
        width = x_max - x_min
        height = y_max - y_min

        # Raw rectangle: thin dashed outline, no fill.
        raw_rect = mpatches.Rectangle(
            (x_min, y_min),
            width,
            height,
            linewidth=0.8,
            edgecolor="black",
            facecolor="none",
            linestyle="--",
            zorder=1,
        )
        ax.add_patch(raw_rect)

        # Inflated polygon: semi-transparent gray fill.
        inflated = inflated_polys[idx]
        xs, ys = inflated.exterior.xy
        inflated_patch = mpatches.Polygon(
            list(zip(xs, ys)),
            closed=True,
            facecolor="lightgray",
            edgecolor="gray",
            alpha=0.5,
            zorder=2,
        )
        ax.add_patch(inflated_patch)


def _draw_edges(ax: plt.Axes, graph: nx.DiGraph) -> None:
    """Draw all edges in the graph, colored by edge type.

    Undirected LINE and PROJECTION edges are drawn only once (both directed
    counterparts overlap perfectly). BLIND edges are always drawn because
    they are intentionally one-directional.
    """
    seen_undirected: set[frozenset] = set()

    for u, v, data in graph.edges(data=True):
        edge_type: str = data.get("type", EDGE_LINE)

        # Deduplicate symmetric line/projection edges.
        if edge_type in (EDGE_LINE, EDGE_PROJECTION):
            key = frozenset((u, v))
            if key in seen_undirected:
                continue
            seen_undirected.add(key)

        ux = graph.nodes[u]["x_cm"]
        uy = graph.nodes[u]["y_cm"]
        vx = graph.nodes[v]["x_cm"]
        vy = graph.nodes[v]["y_cm"]

        color, linestyle, linewidth = _EDGE_STYLE.get(
            edge_type, ("gray", "solid", 1.0)
        )
        ax.plot(
            [ux, vx],
            [uy, vy],
            color=color,
            linestyle=linestyle,
            linewidth=linewidth,
            zorder=3,
        )


def _draw_highlighted_path(
    ax: plt.Axes,
    graph: nx.DiGraph,
    path: list[str],
) -> None:
    """Overdraw consecutive node pairs from *path* in green.

    If a pair ``(path[i], path[i+1])`` has no directed edge in *graph*, a
    :class:`UserWarning` is emitted and that segment is skipped.
    """
    for i in range(len(path) - 1):
        u = path[i]
        v = path[i + 1]
        if not graph.has_edge(u, v):
            warnings.warn(
                f"highlighted_path: no directed edge '{u}' -> '{v}' in graph; "
                "skipping this segment.",
                UserWarning,
                stacklevel=4,
            )
            continue
        ux = graph.nodes[u]["x_cm"]
        uy = graph.nodes[u]["y_cm"]
        vx = graph.nodes[v]["x_cm"]
        vy = graph.nodes[v]["y_cm"]
        ax.plot(
            [ux, vx],
            [uy, vy],
            color="green",
            linewidth=3.5,
            alpha=0.7,
            zorder=5,
            solid_capstyle="round",
        )


def _draw_nodes(ax: plt.Axes, graph: nx.DiGraph) -> None:
    """Scatter-plot each node type separately and label each node."""
    # Group by type so each call to scatter() produces one legend handle.
    by_type: dict[str, list[tuple[float, float, str]]] = {}
    for node, attrs in graph.nodes(data=True):
        node_type: str = attrs.get("type", "line")
        by_type.setdefault(node_type, []).append(
            (attrs["x_cm"], attrs["y_cm"], node)
        )

    for node_type, entries in sorted(by_type.items()):
        xs = [e[0] for e in entries]
        ys = [e[1] for e in entries]
        color = _NODE_COLOR.get(node_type, "gray")
        ax.scatter(
            xs,
            ys,
            color=color,
            s=60,
            zorder=6,
            label=f"node: {node_type}",
        )
        # Label each node slightly offset so the text doesn't overlap the dot.
        for x, y, name in entries:
            ax.text(
                x + 1.5,
                y + 1.5,
                name,
                fontsize=8,
                zorder=7,
                color=color,
            )


def _attach_legend(ax: plt.Axes) -> None:
    """Build a manual legend with Line2D proxies for edge types and a Patch for obstacles."""
    edge_handles = [
        Line2D([0], [0], color="black", linestyle="solid",  linewidth=1.5,
               label="edge: LINE"),
        Line2D([0], [0], color="blue",  linestyle="solid",  linewidth=1.5,
               label="edge: PROJECTION"),
        Line2D([0], [0], color="red",   linestyle="dashed", linewidth=1.5,
               label="edge: BLIND"),
        Line2D([0], [0], color="green", linestyle="solid",  linewidth=3.5,
               alpha=0.7, label="highlighted path"),
    ]
    obstacle_handles = [
        mpatches.Patch(
            facecolor="lightgray",
            edgecolor="gray",
            alpha=0.5,
            label="inflated obstacle",
        ),
        mpatches.Patch(
            facecolor="none",
            edgecolor="black",
            linestyle="--",
            linewidth=0.8,
            label="raw obstacle",
        ),
    ]
    # Node handles come from the scatter calls, but we rebuild them manually
    # here for consistent ordering and formatting.
    node_handles = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor=color,
               markersize=8, label=f"node: {ntype}")
        for ntype, color in sorted(_NODE_COLOR.items())
    ]

    ax.legend(
        handles=edge_handles + obstacle_handles + node_handles,
        loc="upper left",
        fontsize=7,
        framealpha=0.8,
    )
