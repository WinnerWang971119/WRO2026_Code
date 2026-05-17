"""Click-to-read xy coords from ``tools/path_planner/maps/map.png``.

Opens a matplotlib window with the printed WRO 2026 field PNG as the
background, the existing ``field_data.NODES`` overlaid as dim grey
reference dots, and a click handler that prints the world coordinates
(in centimetres) of each left-click to stdout. Used to author the real
``NODES`` table after measuring the printed field.

Run from the repo root:

.. code-block:: powershell

    uv run python tools/path_planner/pick_xy.py

Keybinds:

* left-click — print ``(x, y)`` in cm (3 dp) and drop a red marker.
* ``c`` — clear every picker marker (NODES overlay stays).
* ``u`` — undo the most recent picker marker.
* close the window — exit cleanly.

The tool is purely read-only — it does not modify ``field_data.py``.
Paste the printed coordinates into the ``NODES`` dict by hand and
re-run ``build_routes.py`` to propagate.

Resolves issue #10 scope 1.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Allow running as ``python tools/path_planner/pick_xy.py`` (no ``uv run``,
# no ``-m`` flag) by threading the repo root onto sys.path before the
# absolute ``tools.path_planner.*`` import below. Mirrors the bootstrap
# in ``tools/path_planner/sim/demo.py`` and ``tools/path_planner/build_routes.py``.
_PLANNER_ROOT: Path = Path(__file__).resolve().parent
_REPO_ROOT: Path = _PLANNER_ROOT.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import matplotlib.image as mpimg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

from tools.path_planner import field_data  # noqa: E402

PICKER_DOT_COLOR        = "red"
PICKER_DOT_SIZE         = 60
PICKER_LABEL_FONTSIZE   = 8
PICKER_LABEL_OFFSET_PT  = (5, 5)

NODES_OVERLAY_COLOR          = "0.55"
NODES_OVERLAY_ALPHA          = 0.4
NODES_OVERLAY_DOT_SIZE       = 40
NODES_OVERLAY_LABEL_FONTSIZE = 8

_TITLE = "Left-click to read coords; c=clear, u=undo, close window to exit"


def _field_extent_cm(field_data) -> tuple[float, float]:
    """Return ``(width_cm, height_cm)`` derived from ``FIELD_MM``."""
    width_mm, height_mm = field_data.FIELD_MM
    return float(width_mm) / 10.0, float(height_mm) / 10.0


def _resolve_map_png(field_data) -> Path | None:
    """Return absolute Path to ``field_data.MAP_PNG`` or ``None``."""
    raw = getattr(field_data, "MAP_PNG", None)
    if not raw:
        return None
    p = Path(raw)
    if not p.is_absolute():
        p = _PLANNER_ROOT / p
    return p


def _format_stdout(x: float, y: float) -> str:
    """Return ``'(12.345, 45.678)'`` style string for stdout."""
    return f"({x:.3f}, {y:.3f})"


def _format_label(x: float, y: float) -> str:
    """Return ``'(12.3, 45.7)'`` style string for the on-plot label."""
    return f"({x:.1f}, {y:.1f})"


def _draw_nodes_overlay(ax, field_data) -> None:
    """Plot existing ``NODES`` in dim grey on the axes."""
    nodes = getattr(field_data, "NODES", {})
    if not nodes:
        return
    xs = [coord[0] for coord in nodes.values()]
    ys = [coord[1] for coord in nodes.values()]
    ax.scatter(
        xs, ys,
        color=NODES_OVERLAY_COLOR,
        alpha=NODES_OVERLAY_ALPHA,
        s=NODES_OVERLAY_DOT_SIZE,
        zorder=4,
    )
    for name, (x, y) in nodes.items():
        ax.annotate(
            name,
            xy=(x, y),
            xytext=PICKER_LABEL_OFFSET_PT,
            textcoords="offset points",
            fontsize=NODES_OVERLAY_LABEL_FONTSIZE,
            color=NODES_OVERLAY_COLOR,
            alpha=NODES_OVERLAY_ALPHA,
            zorder=5,
        )


def _make_click_handler(ax, picked_artists: list):
    """Return a closure for ``mpl_connect('button_press_event')``."""

    def _on_click(event) -> None:
        # event.button == 1 is left-click; ignore everything else.
        if event.button != 1:
            return
        # Click outside any axes.
        if event.xdata is None or event.ydata is None:
            return
        # Defensive: ignore clicks landing in other axes.
        if event.inaxes is not ax:
            return
        x = float(event.xdata)
        y = float(event.ydata)
        print(_format_stdout(x, y), flush=True)
        scatter = ax.scatter(
            [x], [y],
            color=PICKER_DOT_COLOR,
            s=PICKER_DOT_SIZE,
            zorder=10,
        )
        text = ax.annotate(
            _format_label(x, y),
            xy=(x, y),
            xytext=PICKER_LABEL_OFFSET_PT,
            textcoords="offset points",
            fontsize=PICKER_LABEL_FONTSIZE,
            color=PICKER_DOT_COLOR,
            zorder=11,
        )
        picked_artists.append((scatter, text))
        ax.figure.canvas.draw_idle()

    return _on_click


def _make_key_handler(picked_artists: list, fig):
    """Return a closure for ``mpl_connect('key_press_event')``.

    ``c`` clears every picker marker; ``u`` undoes the most recent one.
    """

    def _on_key(event) -> None:
        if event.key == "c":
            for scatter, text in picked_artists:
                scatter.remove()
                text.remove()
            picked_artists.clear()
            fig.canvas.draw_idle()
        elif event.key == "u":
            if not picked_artists:
                return
            scatter, text = picked_artists.pop()
            scatter.remove()
            text.remove()
            fig.canvas.draw_idle()

    return _on_key


def main() -> int:
    """CLI entry point. Returns 0 on clean exit, 2 if map.png is missing."""
    map_path = _resolve_map_png(field_data)
    if map_path is None:
        print(
            "pick_xy: MAP_PNG is not configured in field_data.py",
            file=sys.stderr,
        )
        return 2
    try:
        img = mpimg.imread(str(map_path))
    except (FileNotFoundError, PermissionError, OSError) as err:
        print(
            f"pick_xy: cannot read map at {map_path}: {err}",
            file=sys.stderr,
        )
        return 2

    width_cm, height_cm = _field_extent_cm(field_data)

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_title(_TITLE, fontsize=10)
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_xlim(0, width_cm)
    ax.set_ylim(0, height_cm)
    ax.grid(True)
    ax.imshow(
        img,
        extent=(0.0, width_cm, 0.0, height_cm),
        origin="lower",
        alpha=0.6,
        zorder=0,
        aspect="auto",
    )
    _draw_nodes_overlay(ax, field_data)

    picked_artists: list = []
    fig.canvas.mpl_connect(
        "button_press_event", _make_click_handler(ax, picked_artists)
    )
    fig.canvas.mpl_connect(
        "key_press_event", _make_key_handler(picked_artists, fig)
    )

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
