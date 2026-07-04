"""HTML player emitter: render a CTPS :class:`Track` into one self-contained page.

This module turns a playback-ready :class:`~tools.path_planner.ctps.track_builder.Track`
(Data Model B) into a single ``.html`` file that animates the robot driving the
recorded mission over the field map — the design-3a "Instrument" console with a
layered canvas stage, right rail, transport bar, and multi-lane timeline dock.

Design constraints (CTPS UI/UX rework, T2 — issue #125):

* **Modular source, single-file output.** The runtime lives as an *ordered* set
  of ``assets/*.js`` + ``assets/*.css`` files (see :data:`_ASSET_JS` /
  :data:`_ASSET_CSS`). :func:`render` reads them at emit time and inlines them —
  the CSS concatenated into one ``<style>``; **each JS asset in its own
  ``<script>``** so a throwing top-level IIFE in one module cannot abort the
  others. Load order is the contract: ``player.js`` first, ``boot.js`` last.
* **Self-contained / offline.** The output embeds everything inline — the exact
  Track JSON, an inline :data:`PLAYER_META` constant, the CSS, every JS asset,
  and (optionally) the printed field map as a base64 ``data:`` URI. There are
  **no** external ``src=`` / ``href=`` references, so the page opens on
  ``file://`` with zero network or local dependencies (TC5).
* **Emit-time self-containment lint.** :func:`_assert_self_contained` inspects
  every asset + the HTML template markup (but never the embedded
  TRACK / PLAYER_META / MAP_IMAGE JSON *values*, which carry arbitrary user text
  such as ``print`` labels) and raises :class:`ValueError` on any forbidden
  substring (AC-lint).
* **Source-agnostic.** :func:`render` accepts either a :class:`Track` object
  (whose :meth:`Track.to_dict` is called) or a plain dict (used as-is), so a
  hand-built LRS-shaped track dict renders unchanged (AC7 / TC6).
* **Separate ``<script>`` per inline constant.** ``const TRACK``,
  ``const MAP_IMAGE`` and ``const PLAYER_META`` are each emitted in their own
  ``<script>`` element so the emitter test's ``const TRACK = ...;</script>``
  regex terminates at TRACK alone (a same-script ``PLAYER_META`` would be
  swallowed and break the TRACK round-trip).

The interesting runtime logic (the bounded-search + lerp seek that satisfies
TC12) lives in ``assets/player.js``; this module only inlines it.
"""

from __future__ import annotations

import base64
import json
import warnings
from pathlib import Path
from typing import Any, Dict, List, Tuple

__all__ = ["render", "PLAYER_JS_PATH"]

#: Directory holding the modular runtime assets, resolved relative to this
#: module so the renderer works regardless of the caller's cwd.
_ASSETS_DIR: Path = Path(__file__).resolve().parent / "assets"

#: Path to the pinned seek-core JS runtime. Kept as a named export (and the
#: first entry of :data:`_ASSET_JS`) because TC12's regex watches exactly this
#: file — do not relocate ``findKeyframeIndex`` / ``poseAt`` out of it.
PLAYER_JS_PATH: Path = _ASSETS_DIR / "player.js"

#: Ordered CSS assets, concatenated into the single inline ``<style>`` block.
#: ``tokens.css`` defines the custom properties the later sheets style against,
#: so it MUST come first.
_ASSET_CSS: Tuple[Path, ...] = (
    _ASSETS_DIR / "tokens.css",
    _ASSETS_DIR / "layout.css",
    _ASSETS_DIR / "components.css",
)

#: Ordered JS assets, each emitted in its OWN ``<script>`` element. The order is
#: the load contract: ``player.js`` (namespace + engine) first so every later
#: module finds ``window.CTPS``; ``boot.js`` last so it runs the registered
#: inits once all modules have registered.
_ASSET_JS: Tuple[Path, ...] = (
    PLAYER_JS_PATH,
    _ASSETS_DIR / "store.js",
    _ASSETS_DIR / "renderer.js",
    _ASSETS_DIR / "transport.js",
    _ASSETS_DIR / "timeline.js",
    _ASSETS_DIR / "panels.js",
    _ASSETS_DIR / "selection.js",
    _ASSETS_DIR / "chrome.js",
    _ASSETS_DIR / "boot.js",
)

#: Literal substrings that would break the page's offline self-containment if
#: they appeared in an asset or the HTML template markup:
#:
#: * ``src=`` / ``href=`` — an attribute pointing at an external/local resource
#:   (note the missing space is deliberate: ``img.src = MAP_IMAGE`` is fine).
#: * ``http://`` / ``https://`` — a network URL.
#: * ``data:image`` — an inline image background in an asset (the MAP_IMAGE
#:   ``data:image/png`` *value* is exempt — it is a JSON value, not an asset).
#: * ``</script>`` — would prematurely close the inline ``<script>`` wrapper an
#:   asset is inlined into.
_FORBIDDEN_SUBSTRINGS: Tuple[str, ...] = (
    "src=",
    "href=",
    "http://",
    "https://",
    "data:image",
    "</script>",
)

#: Static page body markup (design 3a "Instrument"). Class-based only — no inline
#: element styles (per project frontend conventions); every dynamic value is set
#: later by JS through a CSS custom property (the ADR-053 carve-out). Icons are
#: unicode glyphs emitted as numeric HTML entities (never ``<use href>`` /
#: ``<image href>``, which the self-containment ban forbids). The transport /
#: canvas / rail markers (``#ctps-play``, ``#ctps-seek``, ``#ctps-time``, the
#: three stacked ``#ctps-canvas*`` layers, ``#ctps-command``, ``#ctps-log``) are
#: the emitter test contract; the five container classes ``.ctps-header`` /
#: ``.ctps-stage`` / ``.ctps-rail`` / ``.ctps-transport`` / ``.ctps-timeline``
#: are asserted by TC-STRUCT. This constant is linted by
#: :func:`_assert_self_contained` (it carries no user data).
_BODY_MARKUP: str = """\
<div class="ctps-app">
    <header class="ctps-header">
        <span class="ctps-wordmark">CTPS</span>
        <span id="ctps-filename" class="ctps-filename"></span>
        <span id="ctps-source-badge" class="ctps-badge">PREDICTED</span>
        <span class="ctps-header-spacer"></span>
        <button id="ctps-drift-chip" class="ctps-drift-chip" type="button" hidden>
            <span class="ctps-drift-glyph">&#9650;</span>
            <span id="ctps-drift-chip-text" class="ctps-drift-chip-text">drift</span>
        </button>
        <button id="ctps-help" class="ctps-help-btn" type="button">? Help</button>
    </header>
    <div class="ctps-main">
        <section class="ctps-stage">
            <div class="ctps-canvas-stack">
                <canvas id="ctps-canvas-static" class="ctps-canvas-layer"></canvas>
                <canvas id="ctps-canvas-trail" class="ctps-canvas-layer"></canvas>
                <canvas id="ctps-canvas" class="ctps-canvas-layer"></canvas>
            </div>
            <div class="ctps-layer-chips">
                <button id="ctps-layer-map" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="map">MAP</button>
                <button id="ctps-layer-trail" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="trail">TRAIL</button>
                <button id="ctps-layer-nodes" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="nodes">NODES</button>
            </div>
            <div class="ctps-scale-bar">
                <span class="ctps-scale-line"></span>
                <span class="ctps-scale-label">50 cm</span>
            </div>
            <button id="ctps-fullview" class="ctps-fullview-btn" type="button">
                <span class="ctps-fullview-glyph">&#9974;</span>
                <span id="ctps-fullview-text" class="ctps-fullview-text">Full</span>
            </button>
            <div id="ctps-hud" class="ctps-hud" hidden>
                <div id="ctps-hud-command" class="ctps-hud-command">&mdash;</div>
                <div id="ctps-hud-holding" class="ctps-hud-holding">holding 0 / 3</div>
            </div>
        </section>
        <aside class="ctps-rail">
            <section class="ctps-panel ctps-panel--command">
                <div class="ctps-panel-title">Current command</div>
                <div id="ctps-command" class="ctps-command">&mdash;</div>
                <div id="ctps-command-detail" class="ctps-command-detail"></div>
            </section>
            <section class="ctps-panel ctps-panel--holding">
                <div class="ctps-panel-head">
                    <span class="ctps-panel-title">Holding</span>
                    <span id="ctps-hold-count" class="ctps-hold-count">0 / 3</span>
                </div>
                <div id="ctps-hold-list" class="ctps-hold-list">
                    <div class="ctps-hold-empty"></div>
                    <div class="ctps-hold-empty"></div>
                    <div class="ctps-hold-empty"></div>
                </div>
            </section>
            <section class="ctps-panel ctps-panel--tabs">
                <div class="ctps-tab-row" role="tablist">
                    <button id="ctps-tab-log" class="ctps-tab ctps-tab--active"
                            type="button" data-tab="log">Log</button>
                    <button id="ctps-tab-inspector" class="ctps-tab"
                            type="button" data-tab="inspector">Inspector</button>
                </div>
                <div id="ctps-log" class="ctps-log" data-tab-panel="log"></div>
                <div id="ctps-inspector" class="ctps-inspector"
                     data-tab-panel="inspector" hidden></div>
            </section>
        </aside>
    </div>
    <div class="ctps-transport">
        <button id="ctps-home" class="ctps-transport-btn" type="button"
                aria-label="Restart">&#9198;</button>
        <button id="ctps-play" class="ctps-transport-btn ctps-transport-btn--play"
                type="button" aria-label="Play / pause">&#9654;</button>
        <button id="ctps-end" class="ctps-transport-btn" type="button"
                aria-label="Jump to end">&#9197;</button>
        <span class="ctps-transport-divider"></span>
        <button id="ctps-drift-prev" class="ctps-drift-jump" type="button"
                aria-label="Previous drift">&#9873;&#8249;</button>
        <button id="ctps-drift-next" class="ctps-drift-jump" type="button"
                aria-label="Next drift">&#9873;&#8250;</button>
        <div class="ctps-seek-wrap">
            <input type="range" id="ctps-seek" class="ctps-seek"
                   min="0" max="1000" step="1" value="0" aria-label="Seek">
            <div id="ctps-seek-ticks" class="ctps-seek-ticks"></div>
        </div>
        <span id="ctps-time" class="ctps-time">0.00 / 0.00 s</span>
        <div class="ctps-speed" role="group" aria-label="Playback speed">
            <button class="ctps-speed-btn" type="button" data-speed="0.5">0.5&#215;</button>
            <button class="ctps-speed-btn ctps-speed-btn--active" type="button"
                    data-speed="1">1&#215;</button>
            <button class="ctps-speed-btn" type="button" data-speed="2">2&#215;</button>
            <button class="ctps-speed-btn" type="button" data-speed="4">4&#215;</button>
        </div>
    </div>
    <div class="ctps-timeline">
        <div id="ctps-ruler" class="ctps-ruler"></div>
        <div id="ctps-lanes" class="ctps-lanes"></div>
        <div id="ctps-playhead" class="ctps-playhead"></div>
    </div>
</div>"""

#: Page skeleton. ``{styles}`` receives the concatenated CSS; ``{body}`` the
#: static :data:`_BODY_MARKUP`; ``{scripts}`` the assembled ``<script>`` blocks
#: (TRACK / MAP_IMAGE / PLAYER_META constants + one per JS asset). The skeleton
#: deliberately holds NO ``<script>`` tags of its own — every script lives in
#: ``{scripts}`` — so this template string never contains a literal
#: ``</script>`` and passes the self-containment lint. ``str.format``
#: substitutes the values verbatim: the ``{`` / ``}`` inside the CSS/JS values
#: are inert, so only these three placeholders are interpreted.
_HTML_TEMPLATE: str = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CTPS &mdash; Clear Trajectory Prediction Simulator</title>
<style>
{styles}
</style>
</head>
<body>
{body}
{scripts}
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


def _derive_filename(track_dict: Dict[str, Any], filename: "str | Path | None") -> str:
    """Return the trace filename for the header / :data:`PLAYER_META`.

    Resolution order, most specific first:

    #. an explicit ``filename`` argument — reduced to its basename so a path is
       shown as a bare file name in the header;
    #. a schema-tolerant ``"filename"`` key on the track dict (a future logged
       trace, issue #23, may carry one);
    #. ``""`` when neither is available (the honest fallback — the *output*
       ``.html`` name is not the trace name, so it is never substituted here).

    :param track_dict: the Data Model B track dict.
    :param filename: an explicit trace file name / path, or ``None``.
    :return: the resolved basename string (possibly empty).
    """
    if filename is not None:
        return Path(filename).name
    embedded = track_dict.get("filename")
    if embedded:
        return str(embedded)
    return ""


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


def _read_asset(path: Path) -> str:
    """Read one runtime asset as UTF-8 text.

    :param path: absolute path to a CSS / JS asset under :data:`_ASSETS_DIR`.
    :return: the file's contents.
    :raises FileNotFoundError: if the asset does not exist — re-raised with the
        offending path so a missing sibling asset (created by a later task)
        fails loud at emit time (TC-ASSETS).
    """
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(
            f"ctps.player_html.render: required runtime asset not found at {path}."
        ) from exc


def _assert_self_contained(sources: Dict[str, str]) -> None:
    """Lint each named source for offline self-containment (AC-lint).

    Raises on the first source that contains any of
    :data:`_FORBIDDEN_SUBSTRINGS` — an external reference, network URL, inline
    image background, or a literal ``</script>`` that would break out of the
    inline wrapper. Only the **assets and the HTML template markup** are passed
    here; the embedded TRACK / PLAYER_META / MAP_IMAGE JSON *values* are NOT
    linted (they carry arbitrary user text such as ``print`` labels, and the
    MAP_IMAGE ``data:image/png`` value is a legitimate self-contained URI).

    :param sources: mapping of a human-readable source name (asset file name or
        ``"HTML template"``) to its content.
    :raises ValueError: naming the source and the offending substring.
    """
    for name, content in sources.items():
        for forbidden in _FORBIDDEN_SUBSTRINGS:
            if forbidden in content:
                raise ValueError(
                    "ctps.player_html: self-containment lint failed — "
                    f"{name!r} contains the forbidden substring {forbidden!r}. "
                    "Assets and the HTML template must be fully offline: no "
                    "'src='/'href=' attributes, no 'http(s)://' URLs, no "
                    "'data:image' background, and no literal '</script>' "
                    "(it would break out of the inline <script> wrapper)."
                )


def _script(inner: str) -> str:
    """Wrap ``inner`` in its own ``<script>`` element.

    Each inline constant and each JS asset gets a dedicated element so a
    throwing top-level IIFE cannot abort the siblings, and so the emitter test's
    ``const TRACK = ...;</script>`` regex terminates at TRACK alone.

    :param inner: the raw script body (already ``</`` -escaped where it embeds
        user text).
    :return: a ``<script> ... </script>`` block.
    """
    return f"<script>\n{inner}\n</script>"


def render(
    track: Any,
    out_path: "str | Path",
    map_png: "str | Path | None" = None,
    source: str = "PREDICTED",
    filename: "str | Path | None" = None,
) -> Path:
    """Render ``track`` into one self-contained HTML player at ``out_path``.

    Emits a single ``.html`` file that embeds, all inline and each in its own
    ``<script>`` element, the **exact** Track JSON (round-trips byte-for-byte
    through :func:`json.loads`), the optional ``MAP_IMAGE`` data URI, and a
    :data:`PLAYER_META` constant; then the concatenated CSS assets in one
    ``<style>`` and every JS asset (:data:`_ASSET_JS`) in load order, each in its
    own ``<script>``. The page has **no** external ``src`` / ``href`` to local
    files, so it runs offline on ``file://`` (TC5). Before writing,
    :func:`_assert_self_contained` lints the assets + template markup and raises
    if any is not self-contained (AC-lint).

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
    :param source: trace source badge for :data:`PLAYER_META` (``"PREDICTED"``
        today; ``"LOGGED"`` for a future recorded trace, issue #23).
    :param filename: optional trace file name shown in the header; when omitted
        it is taken from the track dict's ``"filename"`` key if present, else
        left empty (see :func:`_derive_filename`).
    :return: the resolved :class:`pathlib.Path` that was written.
    :raises TypeError: if ``track`` is neither a ``to_dict`` object nor a dict.
    :raises FileNotFoundError: if any required asset (e.g. ``assets/player.js``)
        is missing.
    :raises ValueError: if an asset or the HTML template is not self-contained.
    """
    track_dict = _as_track_dict(track)

    # Embed the EXACT track JSON. ``ensure_ascii=False`` keeps any non-ASCII
    # labels readable; the result still round-trips through json.loads (TC5).
    # ``</`` is escaped so a label containing "</script>" cannot break out of
    # the inline <script> tag — the embedded text stays valid JSON either way.
    track_json = json.dumps(track_dict, ensure_ascii=False).replace("</", "<\\/")

    # Optional field-map background as a self-contained data URI (or "null").
    map_image_js = _map_data_uri(map_png)

    # Render-time knowledge for the JS side. Round-trips like TRACK (own
    # <script>, same ``</`` escape) so a sibling regex can pull it back out.
    player_meta: Dict[str, Any] = {
        "source": source,
        "filename": _derive_filename(track_dict, filename),
        "lanes": ["task", "goto", "holding", "drift"],
        "tabs": ["log", "inspector"],
        "render_mode": "box",
    }
    meta_json = json.dumps(player_meta, ensure_ascii=False).replace("</", "<\\/")

    # Read every asset (missing sibling -> FileNotFoundError with its path).
    css_contents: List[str] = [_read_asset(path) for path in _ASSET_CSS]
    js_contents: List[str] = [_read_asset(path) for path in _ASSET_JS]

    # Lint the assets + the template markup (never the embedded JSON values).
    lint_sources: Dict[str, str] = {
        "HTML template": _HTML_TEMPLATE,
        "HTML body markup": _BODY_MARKUP,
    }
    for path, content in zip(_ASSET_CSS, css_contents):
        lint_sources[path.name] = content
    for path, content in zip(_ASSET_JS, js_contents):
        lint_sources[path.name] = content
    _assert_self_contained(lint_sources)

    styles = "\n\n".join(css_contents)

    # Inline constants first (player.js ingests TRACK; boot.js reads
    # PLAYER_META), then each JS asset in its own <script> in load order.
    script_blocks: List[str] = [
        _script(f"const TRACK = {track_json};"),
        _script(f"const MAP_IMAGE = {map_image_js};"),
        _script(f"const PLAYER_META = {meta_json};"),
    ]
    script_blocks.extend(_script(content) for content in js_contents)
    scripts = "\n".join(script_blocks)

    html = _HTML_TEMPLATE.format(styles=styles, body=_BODY_MARKUP, scripts=scripts)

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    # LF line endings, UTF-8, no BOM — deterministic across platforms.
    with open(out, "w", encoding="utf-8", newline="") as fh:
        fh.write(html)

    return out
