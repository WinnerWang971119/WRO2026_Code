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
* **Conditional ``const OBJECTS`` (object/cargo plan, T4 — issue #57).**
  :func:`render` accepts an optional ``objects`` dict (a ``layout.json``-shaped
  mapping, see :mod:`tools.path_planner.ctps.layout`). When truthy it is
  strictly validated then embedded, byte-for-byte, as its OWN
  ``const OBJECTS = {...};`` ``<script>`` (same ``</`` escape as TRACK) —
  mirroring the ``Track.to_dict`` **pivots**-omission precedent: the key is
  emitted only when present, never as a ``null`` placeholder. ``objects=None``
  (the default) emits no ``const OBJECTS`` at all, and :data:`PLAYER_META` is
  **not** touched by this feature; the player detects a layout via
  ``typeof OBJECTS !== "undefined"`` (AC11, suite TC9 stays green).
* **Unconditional vision assets, conditional mosaic DATA (randomized-vision
  plan, T10 — ADR-060).** ``assets/vision.js``, the ``#ctps-vision`` rail panel
  markup, and the ``mosaic`` / ``vision-cone`` layer chips ship on EVERY player
  page (ADR-056 upload-leg rule — an uploaded ``mosaic``/``robot.vision`` layout
  must render in a page emitted mosaic-free). Only the DATA is conditional:
  ``const MOSAIC_META`` + ``const MOSAIC`` are emitted (each in its own
  ``<script>``, same ``</`` escape) **only when a mosaic is renderable** — the
  layout carries a ``mosaic`` geometry block (the trace's echoed roll supplies
  the colours; its absence draws the grid outline with a "no roll recorded"
  badge). A mosaic-free render emits NEITHER block and leaves :data:`PLAYER_META`
  byte-identical (AC12). See :func:`_mosaic_scripts`.

The interesting runtime logic (the bounded-search + lerp seek that satisfies
TC12) lives in ``assets/player.js``; this module only inlines it.

The emit-scaffold helpers (``read_asset`` / ``assert_self_contained`` /
``script`` / ``FORBIDDEN_SUBSTRINGS``) are owned by
:mod:`tools.path_planner.ctps._page_emit` (T1, issue #57) and imported here
under their historical private names so a second page emitter
(``layout_editor_html.py``, T7) can share them without duplicating this
module's lint/asset-read/script-wrap logic.
"""

from __future__ import annotations

import base64
import json
import warnings
from pathlib import Path
from typing import Any, Dict, List, Tuple

from tools.path_planner.ctps import mosaic as _mosaic
from tools.path_planner.ctps._page_emit import FORBIDDEN_SUBSTRINGS as _FORBIDDEN_SUBSTRINGS
from tools.path_planner.ctps._page_emit import assert_self_contained as _assert_self_contained
from tools.path_planner.ctps._page_emit import read_asset as _read_asset
from tools.path_planner.ctps._page_emit import script as _script
from tools.path_planner.ctps.layout import validate as _validate_layout

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
#: module finds ``window.CTPS``; ``objects_draw.js`` then ``cargo.js`` (object/
#: cargo plan, T4 -- issue #57) then ``vision.js`` (randomized-vision plan, T10)
#: come immediately before ``boot.js`` -- ``objects_draw.js`` establishes
#: ``window.CTPS_OBJ`` and both ``cargo.js`` and ``vision.js`` reference its
#: ``CTPS_OBJ.project``/``localToWorld``/``drawObject`` exports at IIFE time, so
#: this order is a pinned contract, not incidental; ``boot.js`` stays last so
#: it runs the registered inits once all modules (including these) have
#: registered. ``vision.js`` ships on EVERY page (unconditional asset, ADR-056
#: upload leg) even when no mosaic is rolled -- only the emitted ``MOSAIC_META``
#: / ``const MOSAIC`` DATA is conditional (AC12).
_ASSET_JS: Tuple[Path, ...] = (
    PLAYER_JS_PATH,
    _ASSETS_DIR / "store.js",
    _ASSETS_DIR / "renderer.js",
    _ASSETS_DIR / "transport.js",
    _ASSETS_DIR / "timeline.js",
    _ASSETS_DIR / "timeline_view.js",
    _ASSETS_DIR / "panels.js",
    _ASSETS_DIR / "selection.js",
    _ASSETS_DIR / "chrome.js",
    _ASSETS_DIR / "objects_draw.js",
    _ASSETS_DIR / "cargo.js",
    _ASSETS_DIR / "vision.js",
    _ASSETS_DIR / "boot.js",
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
#:
#: Object/cargo plan additions (T4 -- issue #57), markup only (behavior is
#: T5/T6): the ``.ctps-layer-chips`` row gains ``OBJECTS``/``ZONES``/``DEPLOY``
#: ``data-layer`` chips (``chrome.js`` wires every ``.ctps-layer-chips
#: .ctps-chip`` generically, so it needs no change); the header gains an
#: ``#ctps-layout-file`` upload input + its ``#ctps-layout-upload-btn`` button
#: (in-browser layout upload leg of AC5); and the old ``.ctps-panel--holding``
#: shell is replaced by the cargo panel (``#ctps-cargo``,
#: ``.ctps-cargo-tab[data-cargo-tab]``, ``#ctps-cargo-count-mag`` /
#: ``#ctps-cargo-count-inv``, ``#ctps-mag-grid``, ``#ctps-inv-list`` -- the DOM
#: contract T6 queries), which also carries the ADR-060 Vision-read tab (below).
#: ``#ctps-command``/``#ctps-log`` are unchanged, so
#: TC-STRUCT is unaffected.
#:
#: The drift-jump (``#ctps-drift-prev``/``#ctps-drift-next``) and play-speed
#: (``.ctps-speed`` ``data-speed`` buttons) controls moved off the transport
#: bar into the timeline dock's toolbar (``.ctps-tl-transport-group``,
#: alongside the zoom widget in ``.ctps-tl-zoom-group``) so the transport bar's
#: seek scrubber gets the full bar width. The wiring is id/class based and
#: document-wide (``transport.js``), so the relocation needs no JS change.
#:
#: Randomized-vision additions (T10, ADR-060), markup only (behaviour is T11
#: ``vision.js``): the ``.ctps-layer-chips`` row gains ``MOSAIC``/``CONE``
#: (``data-layer="mosaic"`` / ``data-layer="vision-cone"``) chips (``chrome.js``
#: wires every ``.ctps-layer-chips .ctps-chip`` generically, so no JS change);
#: and the per-frame read HUD, which lives as a THIRD tab of the cargo panel
#: (``#ctps-cargo-tab-vision`` / ``data-cargo-tab="vision"`` ->
#: ``#ctps-cargo-panel-vision``, beside Magazine + Inventory) rather than its own
#: rail section -- consolidating the rail to save vertical space. The tab body
#: keeps the frozen ``#ctps-vision`` DOM ids (``#ctps-vision-hud-count`` /
#: ``#ctps-vision-hud-detail`` + a ``#ctps-vision-badges`` slot for the no-roll /
#: geom-mismatch badges) so ``vision.js`` binds unchanged; the tab's own count
#: badge (``#ctps-cargo-count-vision``) mirrors the live ``k / 12`` read. These
#: ship on EVERY page (unconditional markup, ADR-056 upload leg) regardless of
#: whether a mosaic was rolled;
#: ``#ctps-command``/``#ctps-log``/``PLAYER_META`` are untouched, so TC-STRUCT
#: and the frozen ``PLAYER_META`` literal are unaffected.
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
        <button id="ctps-layout-upload-btn" class="ctps-help-btn" type="button">Load layout&hellip;</button>
        <input type="file" id="ctps-layout-file" class="ctps-layout-file"
               accept="application/json,.json" hidden>
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
                <button id="ctps-layer-path" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="path">PATH</button>
                <button id="ctps-layer-nodes" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="nodes">NODES</button>
                <button id="ctps-layer-objects" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="objects">OBJECTS</button>
                <button id="ctps-layer-zones" class="ctps-chip"
                        type="button" data-layer="zones">ZONES</button>
                <button id="ctps-layer-deploy" class="ctps-chip"
                        type="button" data-layer="deploy">DEPLOY</button>
                <button id="ctps-layer-mosaic" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="mosaic">MOSAIC</button>
                <button id="ctps-layer-vision-cone" class="ctps-chip ctps-chip--active"
                        type="button" data-layer="vision-cone">CONE</button>
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
            <section id="ctps-cargo" class="ctps-panel ctps-panel--cargo">
                <div class="ctps-cargo-tab-row" role="tablist">
                    <button id="ctps-cargo-tab-magazine"
                            class="ctps-cargo-tab ctps-cargo-tab--active"
                            type="button" role="tab" data-cargo-tab="magazine">
                        <span class="ctps-cargo-tab-label">MAG</span>
                        <span id="ctps-cargo-count-mag"
                              class="ctps-cargo-tab-count">0/12</span>
                    </button>
                    <button id="ctps-cargo-tab-inventory" class="ctps-cargo-tab"
                            type="button" role="tab" data-cargo-tab="inventory">
                        <span class="ctps-cargo-tab-label">INV</span>
                        <span id="ctps-cargo-count-inv"
                              class="ctps-cargo-tab-count">0/1</span>
                    </button>
                    <button id="ctps-cargo-tab-vision" class="ctps-cargo-tab"
                            type="button" role="tab" data-cargo-tab="vision">
                        <span class="ctps-cargo-tab-label">READ</span>
                        <span id="ctps-cargo-count-vision"
                              class="ctps-cargo-tab-count">&mdash;/12</span>
                    </button>
                </div>
                <div id="ctps-cargo-panel-magazine" class="ctps-cargo-panel"
                     data-cargo-panel="magazine">
                    <div id="ctps-mag-grid" class="ctps-mag-grid"></div>
                </div>
                <div id="ctps-cargo-panel-inventory" class="ctps-cargo-panel"
                     data-cargo-panel="inventory" hidden>
                    <div id="ctps-inv-list" class="ctps-inv-list"></div>
                </div>
                <div id="ctps-cargo-panel-vision" class="ctps-cargo-panel"
                     data-cargo-panel="vision" hidden>
                    <div id="ctps-vision" class="ctps-vision">
                        <div id="ctps-vision-badges" class="ctps-vision-badges"></div>
                        <div id="ctps-vision-hud" class="ctps-vision-hud">
                            <span id="ctps-vision-hud-count"
                                  class="ctps-vision-hud-count">&mdash;</span>
                            <div id="ctps-vision-hud-detail"
                                 class="ctps-vision-hud-detail"></div>
                        </div>
                    </div>
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
        <div class="ctps-seek-wrap">
            <input type="range" id="ctps-seek" class="ctps-seek"
                   min="0" max="1000" step="1" value="0" aria-label="Seek">
            <div id="ctps-seek-ticks" class="ctps-seek-ticks"></div>
        </div>
        <span id="ctps-time" class="ctps-time">0.00 / 0.00 s</span>
    </div>
    <div class="ctps-timeline">
        <div class="ctps-tl-toolbar">
            <div class="ctps-tl-transport-group">
                <button id="ctps-drift-prev" class="ctps-drift-jump" type="button"
                        aria-label="Previous drift">&#9873;&#8249;</button>
                <button id="ctps-drift-next" class="ctps-drift-jump" type="button"
                        aria-label="Next drift">&#9873;&#8250;</button>
                <div class="ctps-speed" role="group" aria-label="Playback speed">
                    <button class="ctps-speed-btn" type="button" data-speed="0.5">0.5&#215;</button>
                    <button class="ctps-speed-btn ctps-speed-btn--active" type="button"
                            data-speed="1">1&#215;</button>
                    <button class="ctps-speed-btn" type="button" data-speed="2">2&#215;</button>
                    <button class="ctps-speed-btn" type="button" data-speed="4">4&#215;</button>
                </div>
            </div>
            <div class="ctps-tl-zoom-group">
                <button id="ctps-zoom-out" class="ctps-zoom-btn" type="button" aria-label="Zoom out">&#8722;</button>
                <input type="range" id="ctps-zoom" class="ctps-zoom-slider"
                       min="0" max="1000" step="1" value="0" aria-label="Zoom">
                <button id="ctps-zoom-in" class="ctps-zoom-btn" type="button" aria-label="Zoom in">+</button>
                <button id="ctps-zoom-fit" class="ctps-zoom-fit" type="button">Fit</button>
                <span id="ctps-zoom-level" class="ctps-zoom-level">1.0&#215;</span>
            </div>
        </div>
        <div id="ctps-tl-viewport" class="ctps-tl-viewport">
            <div id="ctps-tl-content" class="ctps-tl-content">
                <div id="ctps-ruler" class="ctps-ruler"></div>
                <div id="ctps-lanes" class="ctps-lanes"></div>
                <div id="ctps-playhead" class="ctps-playhead"></div>
            </div>
        </div>
        <div class="ctps-tl-scroll">
            <div id="ctps-tl-thumb" class="ctps-tl-scroll-thumb"></div>
        </div>
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


def _objects_script(objects: "Dict[str, Any] | None") -> "str | None":
    """Return the ``const OBJECTS`` ``<script>`` block, or ``None`` if absent.

    A truthy ``objects`` is strictly validated via
    :func:`tools.path_planner.ctps.layout.validate` (raises
    :class:`~tools.path_planner.ctps.layout.LayoutError` LOUD on the first
    malformed field) then embedded byte-for-byte -- the same ``</`` -> ``<\\/``
    escape as TRACK/PLAYER_META, so a layout carrying a literal
    ``</script>``-shaped string cannot break out of the inline wrapper.

    A falsy ``objects`` (``None`` or ``{}``) returns ``None`` so
    :func:`render` emits **no** ``const OBJECTS`` at all -- the
    ``Track.to_dict`` pivots-omission precedent, never a ``null`` placeholder
    (AC11).

    :param objects: a ``layout.json``-shaped dict, or ``None``.
    :return: the ``<script>const OBJECTS = ...;</script>`` block, or ``None``
        when ``objects`` is falsy.
    :raises tools.path_planner.ctps.layout.LayoutError: if ``objects`` fails
        strict schema validation.
    """
    if not objects:
        return None
    _validate_layout(objects)
    objects_json = json.dumps(objects, ensure_ascii=False).replace("</", "<\\/")
    return _script(f"const OBJECTS = {objects_json};")


def _mosaic_meta_script() -> str:
    """Return the ``const MOSAIC_META`` ``<script>`` block (single colour source).

    Emits the mosaic colour vocabulary — the names in
    :data:`tools.path_planner.ctps.mosaic.COLOR_NAMES` order and the display hex
    per :data:`tools.path_planner.ctps.mosaic.COLOR_HEX` — as
    ``const MOSAIC_META = {"names":[...], "hex":{...}};`` so ``vision.js`` reads
    the colour contract from ONE place (never a hand-synced JS copy). Only ever
    emitted alongside a ``const MOSAIC`` block (never on its own).

    :return: the ``<script>const MOSAIC_META = ...;</script>`` block.
    """
    meta = {
        "names": list(_mosaic.COLOR_NAMES),
        "hex": {name: _mosaic.COLOR_HEX[name] for name in _mosaic.COLOR_NAMES},
    }
    meta_json = json.dumps(meta, ensure_ascii=False).replace("</", "<\\/")
    return _script(f"const MOSAIC_META = {meta_json};")


def _mosaic_scripts(
    track_dict: Dict[str, Any], objects: "Dict[str, Any] | None"
) -> List[str]:
    """Return the conditional ``MOSAIC_META`` + ``const MOSAIC`` script blocks.

    The randomized-vision DATA is CONDITIONAL (unlike the always-shipped
    ``vision.js`` asset + ``#ctps-vision`` markup): it is emitted only when a
    mosaic can actually be drawn. Three renderable states (AC10), driven by the
    layout mosaic GEOMETRY (``objects["mosaic"]``) and the trace mosaic ROLL
    (``track_dict["mosaic"]``):

    #. **layout geometry + trace roll** — the primary case: the 12 cells resolve
       from the layout GEOMETRY x the trace COLORS
       (:func:`tools.path_planner.ctps.mosaic.resolve_cells`), each carrying a
       ``color``. ``geom_match`` compares the CURRENT layout geometry hash
       against the trace's echoed ``geom_hash`` (``False`` -> the player badges
       "geometry changed since recording"). ``roll`` is ``True``.
    #. **layout geometry, NO trace roll** — the grid-outline case (old trace, or
       a dump run without ``--mosaic``): the cells resolve to POSITIONS only (no
       synthesized ``color``), ``roll`` is ``False``, ``geom_match`` is ``True``
       (the geometry IS the current layout), and no ``seed`` is carried — the
       player draws the outline with a "no roll recorded" badge.
    #. **no layout geometry** — nothing to place: emit NOTHING (feature inactive,
       per Error Handling), even if the trace carried colours (they cannot be
       positioned without geometry).

    A mosaic-free render (no layout geometry, no trace roll) therefore emits
    NEITHER block (AC12), leaving ``PLAYER_META`` and every frozen literal
    untouched.

    :param track_dict: the Data Model B track dict; ``track_dict.get("mosaic")``
        is the trace's echoed roll (``{"cells", "geom_hash", "injected",
        "seed"?}``) or absent.
    :param objects: the ``layout.json``-shaped dict, or ``None``;
        ``objects.get("mosaic")`` is the layout's mosaic GEOMETRY block.
    :return: ``[MOSAIC_META_script, MOSAIC_script]`` when a mosaic is
        renderable, else ``[]``.
    """
    layout_mosaic = objects.get("mosaic") if isinstance(objects, dict) else None
    if not isinstance(layout_mosaic, dict):
        # No layout geometry -> nothing to place, feature inactive (even if the
        # trace carried colours, they cannot be positioned).
        return []

    geom = _mosaic.MosaicConfig.from_dict(layout_mosaic)
    layout_geom_hash = _mosaic.geom_hash(geom)

    trace_mosaic = track_dict.get("mosaic")
    has_roll = (
        isinstance(trace_mosaic, dict)
        and isinstance(trace_mosaic.get("cells"), list)
        and len(trace_mosaic["cells"]) == _mosaic.CELL_COUNT
    )

    if has_roll:
        # Primary case: layout geometry x trace colours -> coloured cells.
        colors: List[str] = [str(c) for c in trace_mosaic["cells"]]
        cells = _mosaic.resolve_cells(geom, colors)
        trace_geom_hash = trace_mosaic.get("geom_hash")
        mosaic_data: Dict[str, Any] = {
            "cells": cells,
            "mode": geom.mode,
            "geom_hash": layout_geom_hash,
            "geom_match": (trace_geom_hash == layout_geom_hash),
            "roll": True,
        }
        seed = trace_mosaic.get("seed")
        if seed is not None:
            mosaic_data["seed"] = seed
    else:
        # Grid-outline case: layout geometry only, no recorded roll. Resolve the
        # cell POSITIONS (placeholder colours dropped) so the player can draw the
        # outline + a "no roll recorded" badge — never synthesize colours.
        placeholder = [_mosaic.COLOR_NAMES[0]] * _mosaic.CELL_COUNT
        positioned = _mosaic.resolve_cells(geom, placeholder)
        cells = [
            {"index": c["index"], "x_cm": c["x_cm"], "y_cm": c["y_cm"]}
            for c in positioned
        ]
        mosaic_data = {
            "cells": cells,
            "mode": geom.mode,
            "geom_hash": layout_geom_hash,
            "geom_match": True,
            "roll": False,
        }

    mosaic_json = json.dumps(mosaic_data, ensure_ascii=False).replace("</", "<\\/")
    return [
        _mosaic_meta_script(),
        _script(f"const MOSAIC = {mosaic_json};"),
    ]


def render(
    track: Any,
    out_path: "str | Path",
    map_png: "str | Path | None" = None,
    source: str = "PREDICTED",
    filename: "str | Path | None" = None,
    objects: "Dict[str, Any] | None" = None,
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

    When ``objects`` (a ``layout.json``-shaped dict, object/cargo plan, T4 --
    issue #57) is truthy it is validated via
    :func:`tools.path_planner.ctps.layout.validate` and embedded as its own
    ``const OBJECTS = {...};`` ``<script>``; a falsy/omitted ``objects``
    emits **no** ``const OBJECTS`` at all and leaves :data:`PLAYER_META`
    untouched (AC11 -- see :func:`_objects_script`).

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
    :param objects: optional ``layout.json``-shaped dict (field objects +
        robot manipulator zones); ``None`` (default) renders with no
        ``const OBJECTS`` at all (AC11).
    :return: the resolved :class:`pathlib.Path` that was written.
    :raises TypeError: if ``track`` is neither a ``to_dict`` object nor a dict.
    :raises FileNotFoundError: if any required asset (e.g. ``assets/player.js``)
        is missing.
    :raises ValueError: if an asset or the HTML template is not self-contained.
    :raises tools.path_planner.ctps.layout.LayoutError: if a truthy ``objects``
        fails strict schema validation.
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
    # "drift" here is the lane ID, not its display label — the dock label was
    # relabelled to "LOCALIZATION" in timeline.js (issue #130); the id itself
    # is kept unchanged so this manifest and the registered lane stay wired.
    player_meta: Dict[str, Any] = {
        "source": source,
        "filename": _derive_filename(track_dict, filename),
        "lanes": ["task", "goto", "magazine", "rearbox", "drift"],
        "tabs": ["log", "inspector"],
        "render_mode": "wheel-arc" if track_dict.get("pivots") else "box",
    }
    meta_json = json.dumps(player_meta, ensure_ascii=False).replace("</", "<\\/")

    # Conditional const OBJECTS -- None when no layout was given (AC11); a
    # truthy layout is validated (raises LayoutError loud) before anything is
    # written. Never touches player_meta -- the player detects a layout via
    # `typeof OBJECTS !== "undefined"`.
    objects_script = _objects_script(objects)

    # Conditional MOSAIC_META + const MOSAIC (randomized-vision plan, T10) --
    # emitted only when a mosaic is renderable (layout geometry present; the
    # trace roll adds colours, its absence draws the outline). A mosaic-free
    # render emits neither block, leaving PLAYER_META + every frozen literal
    # byte-identical (AC12). The always-shipped vision.js / #ctps-vision markup
    # (ADR-056 upload leg) is emitted unconditionally, above.
    mosaic_scripts = _mosaic_scripts(track_dict, objects)

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
    # const OBJECTS -- when present -- rides alongside the other inline
    # constants, own <script>, emitted only when a layout was given (AC11).
    script_blocks: List[str] = [
        _script(f"const TRACK = {track_json};"),
        _script(f"const MAP_IMAGE = {map_image_js};"),
        _script(f"const PLAYER_META = {meta_json};"),
    ]
    if objects_script is not None:
        script_blocks.append(objects_script)
    script_blocks.extend(mosaic_scripts)
    script_blocks.extend(_script(content) for content in js_contents)
    scripts = "\n".join(script_blocks)

    html = _HTML_TEMPLATE.format(styles=styles, body=_BODY_MARKUP, scripts=scripts)

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    # LF line endings, UTF-8, no BOM — deterministic across platforms.
    with open(out, "w", encoding="utf-8", newline="") as fh:
        fh.write(html)

    return out
