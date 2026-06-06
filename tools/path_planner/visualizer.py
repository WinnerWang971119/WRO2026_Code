"""Map visualizer for the WRO 2026 path planner (PR3).

Renders a matplotlib PNG showing:

* The printed field PNG (``field_data.MAP_PNG``) as the axes background,
  scaled to ``field_data.FIELD_MM`` so node coordinates map 1:1 onto the
  image. Missing / corrupt PNGs fall back to a plain white axes with a
  one-line warning so the rest of the build still produces a useful map.
* Nodes coloured by type: ``line`` = black, ``blind`` = red.
* Edges coloured by type: ``LINE`` = black solid, ``BLIND`` = red dashed.
* Obstacles: raw rectangle (dashed thin outline) + C-space inflated
  polygon (semi-transparent gray fill).
* An optional ``highlighted_path``: consecutive directed edges overdrawn
  in green.

The Agg backend is selected at import so the module runs headless.
"""

from __future__ import annotations

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import warnings  # noqa: E402
from pathlib import Path
from typing import Iterable, Optional

import matplotlib.image as mpimg  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import networkx as nx  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from tools.path_planner.graph_builder import (  # noqa: E402
    EDGE_BLIND,
    EDGE_LINE,
    inflate_obstacles,
)

DEFAULT_OUTPUT: Path = Path("tools/path_planner/output/map.png")

# Root of the path_planner tree; map.png paths are resolved relative to it.
_PLANNER_ROOT: Path = Path(__file__).resolve().parent

_NODE_COLOR: dict[str, str] = {
    "line":  "black",
    "blind": "red",
}

_EDGE_STYLE: dict[str, tuple[str, str, float]] = {
    EDGE_LINE:  ("black", "solid",  1.5),
    EDGE_BLIND: ("red",   "dashed", 1.5),
}

_DEFAULT_TITLE = "WRO 2026 Path Planner"


def _resolve_map_png(field_data) -> Optional[Path]:
    """Return the absolute path to ``field_data.MAP_PNG`` or ``None``."""
    raw = getattr(field_data, "MAP_PNG", None)
    if not raw:
        return None
    p = Path(raw)
    if not p.is_absolute():
        p = _PLANNER_ROOT / p
    return p


def _field_extent_cm(field_data) -> tuple[float, float]:
    """Return ``(width_cm, height_cm)`` derived from ``FIELD_MM``."""
    width_mm, height_mm = field_data.FIELD_MM
    return float(width_mm) / 10.0, float(height_mm) / 10.0


def render_map(
    field_data,
    graph: nx.DiGraph,
    *,
    out_path: Path = DEFAULT_OUTPUT,
    highlighted_path: Optional[Iterable[str]] = None,
    title: Optional[str] = None,
) -> Path:
    """Render the field map as a PNG at *out_path* and return *out_path*.

    :param field_data: validated ``tools.path_planner.field_data`` module
        exposing ``NODES``, ``NODE_TYPES``, ``OBSTACLES``,
        ``ROBOT_RADIUS_CM``, ``FIELD_MM``, ``MAP_PNG``.
    :param graph: ``networkx.DiGraph`` from
        :func:`tools.path_planner.graph_builder.build_graph`. Nodes carry
        ``x_cm``/``y_cm``/``type``; edges carry ``type``.
    :param out_path: destination PNG (parent dirs created).
    :param highlighted_path: optional ordered list of node names; each
        consecutive pair is overdrawn in green. Missing edges emit a
        UserWarning and are skipped.
    :param title: figure title. Defaults to ``"WRO 2026 Path Planner"``.
    :return: ``out_path``.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    width_cm, height_cm = _field_extent_cm(field_data)

    fig, ax = plt.subplots(figsize=(10, 10 * height_cm / max(width_cm, 1.0)))
    ax.set_aspect("equal")
    ax.set_title(title if title is not None else _DEFAULT_TITLE, fontsize=12)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_xlim(0, width_cm)
    ax.set_ylim(0, height_cm)

    _draw_background(ax, field_data, width_cm, height_cm)
    _draw_obstacles(ax, field_data)
    _draw_edges(ax, graph)
    if highlighted_path is not None:
        _draw_highlighted_path(ax, graph, list(highlighted_path))
    _draw_nodes(ax, graph)
    _attach_legend(ax)

    plt.tight_layout()
    fig.savefig(out_path, bbox_inches="tight", dpi=150)
    plt.close(fig)

    return out_path


def _draw_background(
    ax: plt.Axes,
    field_data,
    width_cm: float,
    height_cm: float,
) -> None:
    """Overlay ``field_data.MAP_PNG`` as the axes background if it exists."""
    map_path = _resolve_map_png(field_data)
    if map_path is None or not map_path.exists():
        if map_path is not None:
            warnings.warn(
                f"map.png not found at {map_path}; rendering without "
                "background. Drop the real WRO 2026 field PNG there.",
                UserWarning,
                stacklevel=3,
            )
        return
    try:
        img = mpimg.imread(str(map_path))
    except Exception as exc:    # noqa: BLE001 -- want to keep rendering
        warnings.warn(
            f"failed to load map.png ({type(exc).__name__}: {exc}); "
            "rendering without background.",
            UserWarning,
            stacklevel=3,
        )
        return
    # Do not pass aspect="auto" here: imshow forwards it to
    # ax.set_aspect("auto"), which would clobber the set_aspect("equal")
    # locked in render_map and distort a non-square field.
    ax.imshow(
        img,
        extent=(0.0, width_cm, 0.0, height_cm),
        origin="lower",
        alpha=0.6,
        zorder=0,
    )


def _draw_obstacles(ax: plt.Axes, field_data) -> None:
    obstacles = field_data.OBSTACLES
    robot_radius_cm = field_data.ROBOT_RADIUS_CM
    inflated_polys = inflate_obstacles(obstacles, robot_radius_cm)

    for idx, obs in enumerate(obstacles):
        x_min, y_min, x_max, y_max, _kind = obs
        width = x_max - x_min
        height = y_max - y_min
        raw_rect = mpatches.Rectangle(
            (x_min, y_min), width, height,
            linewidth=0.8,
            edgecolor="black",
            facecolor="none",
            linestyle="--",
            zorder=1,
        )
        ax.add_patch(raw_rect)
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
    seen_undirected: set[frozenset] = set()
    for u, v, data in graph.edges(data=True):
        edge_type: str = data.get("type", EDGE_LINE)
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
            [ux, vx], [uy, vy],
            color=color, linestyle=linestyle, linewidth=linewidth,
            zorder=3,
        )


def _draw_highlighted_path(
    ax: plt.Axes,
    graph: nx.DiGraph,
    path: list[str],
) -> None:
    for i in range(len(path) - 1):
        u = path[i]
        v = path[i + 1]
        if not graph.has_edge(u, v):
            warnings.warn(
                f"highlighted_path: no directed edge '{u}' -> '{v}'; skipping.",
                UserWarning,
                stacklevel=4,
            )
            continue
        ux = graph.nodes[u]["x_cm"]
        uy = graph.nodes[u]["y_cm"]
        vx = graph.nodes[v]["x_cm"]
        vy = graph.nodes[v]["y_cm"]
        ax.plot(
            [ux, vx], [uy, vy],
            color="green", linewidth=3.5, alpha=0.7,
            zorder=5, solid_capstyle="round",
        )


def _draw_nodes(ax: plt.Axes, graph: nx.DiGraph) -> None:
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
        # No label= here: _attach_legend builds the node legend from an
        # explicit handles list derived from _NODE_COLOR, so any scatter
        # label would be silently discarded.
        ax.scatter(xs, ys, color=color, s=60, zorder=6)
        for x, y, name in entries:
            ax.text(x + 1.5, y + 1.5, name, fontsize=8, zorder=7, color=color)


def _attach_legend(ax: plt.Axes) -> None:
    edge_handles = [
        Line2D([0], [0], color="black", linestyle="solid", linewidth=1.5,
               label="edge: LINE"),
        Line2D([0], [0], color="red",   linestyle="dashed", linewidth=1.5,
               label="edge: BLIND"),
        Line2D([0], [0], color="green", linestyle="solid",  linewidth=3.5,
               alpha=0.7, label="highlighted path"),
    ]
    obstacle_handles = [
        mpatches.Patch(facecolor="lightgray", edgecolor="gray", alpha=0.5,
                       label="inflated obstacle"),
        mpatches.Patch(facecolor="none", edgecolor="black", linestyle="--",
                       linewidth=0.8, label="raw obstacle"),
    ]
    node_handles = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor=color,
               markersize=8, label=f"node: {ntype}")
        for ntype, color in sorted(_NODE_COLOR.items())
    ]
    ax.legend(
        handles=edge_handles + obstacle_handles + node_handles,
        loc="upper left", fontsize=7, framealpha=0.8,
    )
