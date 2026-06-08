"""HTML player emitter: render a CTPS :class:`Track` into one self-contained page.

This module turns a playback-ready :class:`~tools.path_planner.ctps.track_builder.Track`
(Data Model B) into a single ``.html`` file that animates the robot driving the
recorded mission over the field map — with play / pause / seek transport, a
trajectory "T" polyline, and a side panel narrating the current sub-command.

Design constraints (CTPS plan, T8):

* **Self-contained.** The output embeds everything inline — the exact Track
  JSON, the CSS, the contents of :data:`PLAYER_JS_PATH` (``assets/player.js``)
  read at render time, and (optionally) the printed field map as a base64
  ``data:`` URI. There are **no** external ``src=`` / ``href=`` references to
  local files, so the page works on ``file://`` with zero network or local
  dependencies (TC5).
* **Source-agnostic.** :func:`render` accepts either a :class:`Track` object
  (whose :meth:`Track.to_dict` is called) or a plain dict (used as-is), so a
  hand-built LRS-shaped track dict renders unchanged (AC7 / TC6).
* **Pure string/file assembly.** No compiler, no network, no geometry — the
  geometry lives in the track and the animation lives in ``player.js``.

The interesting runtime logic (the bounded-search + lerp seek that satisfies
AC3a / TC12) lives in ``assets/player.js``; this module only inlines it.
"""

from __future__ import annotations

import base64
import json
import warnings
from pathlib import Path
from typing import Any, Dict

__all__ = ["render", "PLAYER_JS_PATH"]

#: Path to the JS runtime inlined into every emitted page. Resolved relative to
#: this module so the renderer works regardless of the caller's cwd.
PLAYER_JS_PATH: Path = Path(__file__).resolve().parent / "assets" / "player.js"

#: Inline stylesheet. Layout uses CSS classes only — no inline element styles
#: (per project frontend conventions). Fonts are system stacks (no web fonts),
#: keeping the page fully offline.
_PLAYER_CSS: str = """\
* { box-sizing: border-box; }
html, body {
    margin: 0;
    padding: 0;
    height: 100%;
    background: #0b0e13;
    color: #cdd6e0;
    font-family: ui-sans-serif, system-ui, "Segoe UI", Arial, sans-serif;
}
.ctps-app {
    display: flex;
    flex-direction: column;
    height: 100vh;
    gap: 8px;
    padding: 10px;
}
.ctps-stage {
    display: flex;
    flex: 1 1 auto;
    gap: 10px;
    min-height: 0;
}
.ctps-canvas-wrap {
    flex: 1 1 auto;
    min-width: 0;
    background: #11151c;
    border: 1px solid #232a36;
    border-radius: 6px;
    overflow: hidden;
}
#ctps-canvas {
    display: block;
    width: 100%;
    height: 100%;
}
.ctps-panel {
    flex: 0 0 280px;
    display: flex;
    flex-direction: column;
    gap: 8px;
    background: #11151c;
    border: 1px solid #232a36;
    border-radius: 6px;
    padding: 12px;
    min-height: 0;
}
.ctps-panel-title {
    font-size: 11px;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: #6b7686;
}
.ctps-command {
    font-family: ui-monospace, "Cascadia Code", Consolas, monospace;
    font-size: 15px;
    color: #e8edf3;
    word-break: break-word;
    min-height: 1.4em;
}
.ctps-log {
    flex: 1 1 auto;
    overflow-y: auto;
    border-top: 1px solid #232a36;
    padding-top: 8px;
    font-family: ui-monospace, "Cascadia Code", Consolas, monospace;
    font-size: 12px;
    line-height: 1.5;
    color: #97a2b3;
}
.ctps-log-line { white-space: pre-wrap; word-break: break-word; }
.ctps-warn { color: #ffb454; }
.ctps-warn.ctps-command { color: #ff7a59; }
.ctps-transport {
    display: flex;
    align-items: center;
    gap: 12px;
    flex: 0 0 auto;
    background: #11151c;
    border: 1px solid #232a36;
    border-radius: 6px;
    padding: 8px 12px;
}
.ctps-btn {
    flex: 0 0 auto;
    min-width: 72px;
    padding: 6px 14px;
    background: #2a64c8;
    color: #fff;
    border: none;
    border-radius: 4px;
    font-size: 13px;
    cursor: pointer;
}
.ctps-btn:hover { background: #356fda; }
#ctps-seek { flex: 1 1 auto; }
.ctps-readout {
    flex: 0 0 auto;
    font-family: ui-monospace, "Cascadia Code", Consolas, monospace;
    font-size: 13px;
    color: #97a2b3;
    min-width: 130px;
    text-align: right;
}
"""

#: Page body. The transport-bar markers (`#ctps-play`, `#ctps-seek`,
#: `#ctps-time`) are exactly what TC5 looks for. `{track_json}` and
#: `{player_js}` are filled at render time; the CSS is pre-substituted so its
#: own braces never collide with ``str.format``.
_HTML_TEMPLATE: str = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CTPS — Clear Trajectory Prediction Simulator</title>
<style>
{player_css}
</style>
</head>
<body>
<div class="ctps-app">
    <div class="ctps-stage">
        <div class="ctps-canvas-wrap">
            <canvas id="ctps-canvas"></canvas>
        </div>
        <aside class="ctps-panel">
            <div class="ctps-panel-title">Current command</div>
            <div id="ctps-command" class="ctps-command">&mdash;</div>
            <div class="ctps-panel-title">Log</div>
            <div id="ctps-log" class="ctps-log"></div>
        </aside>
    </div>
    <div class="ctps-transport">
        <button id="ctps-play" type="button" class="ctps-btn">Play</button>
        <input id="ctps-seek" type="range" min="0" max="1000" step="1" value="0"
               aria-label="Seek">
        <span id="ctps-time" class="ctps-readout">0.00 / 0.00 s</span>
    </div>
</div>
<script>
const TRACK = {track_json};
</script>
<script>
const MAP_IMAGE = {map_image_js};
</script>
<script>
{player_js}
</script>
</body>
</html>
"""


def _as_track_dict(track: Any) -> Dict[str, Any]:
    """Return the Data Model B dict for ``track``.

    Accepts either a :class:`~tools.path_planner.ctps.track_builder.Track`
    object (detected via a callable ``to_dict`` attribute) or a plain mapping
    that already matches the Track schema (used unchanged). This is what keeps
    the player source-agnostic (AC7 / TC6).

    :param track: a ``Track`` object or a plain dict in Data Model B shape.
    :return: the JSON-serialisable track dict.
    :raises TypeError: if ``track`` is neither a ``to_dict``-bearing object nor
        a dict.
    """
    if hasattr(track, "to_dict") and callable(track.to_dict):
        return track.to_dict()
    if isinstance(track, dict):
        return track
    raise TypeError(
        "ctps.player_html.render: track must be a Track (with .to_dict()) "
        f"or a dict, got {type(track).__name__!r}."
    )


def _map_data_uri(map_png: "str | Path | None") -> str:
    """Return a JS literal for the optional field-map background.

    With no map requested (``map_png is None``) returns the literal ``"null"``
    so the player draws its vector field fallback. Otherwise reads the image
    bytes, base64-encodes them into a self-contained ``data:`` URI, and returns
    it as a quoted JS string (via :func:`json.dumps`). A missing file is
    non-fatal: it warns and returns ``"null"`` so the page still renders without
    the map (mirroring the visualizer's best-effort map overlay).

    The data URI embeds no external reference, keeping the emitted page offline
    (TC5); base64 contains no ``<`` so it cannot break out of the inline
    ``<script>``.

    :param map_png: path to the field map image, or ``None`` for no map.
    :return: a JS expression string — either ``"null"`` or a quoted ``data:``
        URI.
    """
    if map_png is None:
        return "null"
    path = Path(map_png)
    if not path.is_file():
        warnings.warn(
            f"ctps.player_html.render: map image not found at {path}; "
            "rendering without a map background.",
            stacklevel=2,
        )
        return "null"
    suffix = path.suffix.lower().lstrip(".") or "png"
    mime = "jpeg" if suffix in ("jpg", "jpeg") else suffix
    b64 = base64.b64encode(path.read_bytes()).decode("ascii")
    return json.dumps(f"data:image/{mime};base64,{b64}")


def render(
    track: Any, out_path: "str | Path", map_png: "str | Path | None" = None
) -> Path:
    """Render ``track`` into one self-contained HTML player at ``out_path``.

    Emits a single ``.html`` file that embeds the **exact** Track JSON inline
    (round-trips byte-for-byte through :func:`json.loads`), the CSS in a
    ``<style>`` block, and the full contents of :data:`PLAYER_JS_PATH` in a
    ``<script>`` tag. The page has **no** external ``src`` / ``href`` to local
    files, so it runs offline on ``file://`` (TC5).

    This is pure string/file assembly: no compiler, no network, no geometry.
    Parent directories of ``out_path`` are created as needed.

    When ``map_png`` is given it is base64-embedded as a self-contained
    ``data:`` URI background (the page stays offline); otherwise the player
    falls back to its vector field render. A missing map file warns and is
    treated as no map.

    :param track: a :class:`~tools.path_planner.ctps.track_builder.Track`
        object (``.to_dict()`` is called) or a plain dict already in Data
        Model B shape (used unchanged) — see :func:`_as_track_dict`.
    :param out_path: destination ``.html`` path (``str`` or
        :class:`pathlib.Path`).
    :param map_png: optional path to the printed field map image to embed as the
        canvas background; ``None`` (default) renders the vector field only.
    :return: the resolved :class:`pathlib.Path` that was written.
    :raises TypeError: if ``track`` is neither a ``to_dict`` object nor a dict.
    :raises FileNotFoundError: if ``assets/player.js`` is missing.
    """
    track_dict = _as_track_dict(track)

    # Embed the EXACT track JSON. ``ensure_ascii=False`` keeps any non-ASCII
    # labels readable; the result still round-trips through json.loads (TC5).
    # ``</`` is escaped so a label containing "</script>" cannot break out of
    # the inline <script> tag — the embedded text stays valid JSON either way.
    track_json = json.dumps(track_dict, ensure_ascii=False).replace("</", "<\\/")

    # Optional field-map background as a self-contained data URI (or "null").
    map_image_js = _map_data_uri(map_png)

    player_js = PLAYER_JS_PATH.read_text(encoding="utf-8")

    html = _HTML_TEMPLATE.format(
        player_css=_PLAYER_CSS,
        track_json=track_json,
        map_image_js=map_image_js,
        player_js=player_js,
    )

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    # LF line endings, UTF-8, no BOM — deterministic across platforms.
    with open(out, "w", encoding="utf-8", newline="") as fh:
        fh.write(html)

    return out
