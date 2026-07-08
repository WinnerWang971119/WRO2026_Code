"""HTML emitter for the offline CTPS layout editor (``layout_editor.html``).

The layout editor is a **second, standalone offline page** (object/cargo plan,
T7 -- issue #57): a two-tab authoring tool (Field tab -> field objects; Robot
tab -> manipulator zones) that saves/loads one ``layout.json``. It is a
deliberately SEPARATE page from the player (``player_html.py``) so the frozen
player seek core and its exact test contracts stay out of editor churn (the
plan's "two offline pages, not one author-mode player" decision).

This module mirrors :mod:`tools.path_planner.ctps.player_html`'s emit shape but
owns its OWN, smaller asset tuple and its OWN self-containment lint pass:

* **Modular source, single-file output.** The editor runtime lives as an
  *ordered* set of ``assets/*.js`` + ``assets/*.css`` files (see
  :data:`_ASSET_JS` / :data:`_ASSET_CSS`). :func:`render` reads them at emit
  time and inlines them -- the CSS concatenated into one ``<style>``; **each JS
  asset in its own ``<script>``** so a throwing top-level IIFE in one module
  cannot abort the others. Load order is the contract: ``objects_draw.js``
  FIRST (it establishes ``window.CTPS_OBJ``, which the editor canvas reuses),
  then the editor modules (``editor_core.js`` -> ``editor_tools.js`` ->
  ``editor_io.js`` -> ``editor_boot.js``, boot last).
* **Self-contained / offline.** The output embeds everything inline -- the CSS,
  every JS asset, an inline ``const FIELD`` field-backdrop constant (derived
  from ``field_data``, the same block the player draws), and an optional printed
  field map as a base64 ``data:`` URI. There are **no** external ``src=`` /
  ``href=`` references, so the page opens on ``file://`` with zero network or
  local dependency (AC1).
* **Emit-time self-containment lint.** :func:`_assert_self_contained` (shared
  with the player via :mod:`tools.path_planner.ctps._page_emit`, T1) inspects
  every asset + the HTML template markup -- but never the embedded ``FIELD`` /
  ``MAP_IMAGE`` JSON *values*, which carry arbitrary field text -- and raises
  :class:`ValueError` on any forbidden substring (AC1, P-TC7).

``objects_draw.js`` is owned by the player-side tasks (T4/T5) and is *inlined*
here unchanged so the shared ``window.CTPS_OBJ`` draw helpers exist for the
editor canvas; this module never creates or edits it. The editor-only assets
(``editor_core.js`` / ``editor_tools.js`` / ``editor_io.js`` / ``editor_boot.js``
and ``editor.css``) ship as inert placeholders in T7 and are filled by T8 -- the
emitter + its tests exercise the final asset layout now.
"""

from __future__ import annotations

import base64
import json
import re
import warnings
from pathlib import Path
from typing import Any, Dict, List, Tuple

from tools.path_planner.ctps._page_emit import assert_self_contained as _assert_self_contained
from tools.path_planner.ctps._page_emit import read_asset as _read_asset
from tools.path_planner.ctps._page_emit import script as _script

# The field backdrop the editor draws is the SAME block the player renders, so
# it is single-sourced from the track builder rather than re-derived here (which
# would drift from the player's field render). This is an intra-package reuse of
# a stable helper -- not a refactor of the frozen player/renderer core.
from tools.path_planner.ctps.track_builder import build_field_block

__all__ = ["render", "reembed_layout"]

#: Directory holding the modular runtime assets, resolved relative to this
#: module so the emitter works regardless of the caller's cwd.
_ASSETS_DIR: Path = Path(__file__).resolve().parent / "assets"

#: Ordered CSS assets, concatenated into the single inline ``<style>`` block.
#: ``tokens.css`` defines the custom properties ``editor.css`` styles against,
#: so it MUST come first.
_ASSET_CSS: Tuple[Path, ...] = (
    _ASSETS_DIR / "tokens.css",
    _ASSETS_DIR / "editor.css",
)

#: Ordered JS assets, each emitted in its OWN ``<script>`` element. The order is
#: the load contract: ``objects_draw.js`` FIRST -- it establishes the shared
#: ``window.CTPS_OBJ`` draw helpers the editor canvas reuses -- then the editor
#: modules; ``editor_core.js`` seeds ``window.CTPS_EDITOR`` + the duplicated
#: world->pixel view transform, ``editor_tools.js`` / ``editor_io.js`` build on
#: it, and ``editor_boot.js`` stays LAST so it runs the one-time init once every
#: other module has registered. ``server_session.js`` (the shared self-closing-
#: server heartbeat, ADR-057) is self-contained and order-independent -- it
#: replaces the editor's former in-``editor_io.js`` session leg so the player and
#: editor never desync on the liveness protocol.
_ASSET_JS: Tuple[Path, ...] = (
    _ASSETS_DIR / "objects_draw.js",
    _ASSETS_DIR / "editor_core.js",
    _ASSETS_DIR / "editor_tools.js",
    _ASSETS_DIR / "editor_io.js",
    _ASSETS_DIR / "server_session.js",
    _ASSETS_DIR / "editor_boot.js",
)

#: Static page body markup (the sidecar's Change 3 layout: a two-tab editor with
#: a Field/Robot tab row, a Rect/Circle/Select tool row, a Robot-tab render-size
#: (zoom) control, Save/Load controls, one canvas stage, and a Properties panel
#: authoring every ``layout.json`` schema field). Class-based only -- no inline
#: element styles (per project frontend
#: conventions); every dynamic value is set later by JS through a CSS custom
#: property (the ADR-053 carve-out). Icons are unicode glyphs emitted as numeric
#: HTML entities (never ``<use href>`` / ``<image href>``, which the
#: self-containment ban forbids). The interactions (draw/select/save/load, and
#: the rect-vs-circle field show/hide) are T8's job -- this constant only lays
#: out the DOM T8 wires onto. Linted by :func:`_assert_self_contained` (it
#: carries no field data).
_BODY_MARKUP: str = """\
<div class="ctps-app ctps-editor">
    <header class="ctps-header ctps-editor-header">
        <span class="ctps-wordmark">CTPS</span>
        <span class="ctps-editor-subtitle">Layout Editor</span>
        <div id="ctps-editor-tabs" class="ctps-editor-tabs" role="tablist">
            <button id="ctps-editor-tab-field"
                    class="ctps-editor-tab ctps-editor-tab--active"
                    type="button" role="tab" data-editor-tab="field">Field</button>
            <button id="ctps-editor-tab-robot" class="ctps-editor-tab"
                    type="button" role="tab" data-editor-tab="robot">Robot</button>
        </div>
        <span class="ctps-header-spacer"></span>
        <div id="ctps-editor-tools" class="ctps-editor-tools" role="group"
             aria-label="Draw tools">
            <button id="ctps-tool-select"
                    class="ctps-editor-tool ctps-editor-tool--active"
                    type="button" data-tool="select">&#9662; Select</button>
            <button id="ctps-tool-rect" class="ctps-editor-tool"
                    type="button" data-tool="rect">&#9645; Rect</button>
            <button id="ctps-tool-circle" class="ctps-editor-tool"
                    type="button" data-tool="circle">&#9711; Circle</button>
        </div>
        <div id="ctps-editor-zoom" class="ctps-editor-zoom" role="group"
             aria-label="Robot render size" hidden>
            <span class="ctps-editor-zoom-label">Zoom</span>
            <input type="range" id="ctps-editor-zoom-range"
                   class="ctps-editor-zoom-range" min="25" max="200" step="5"
                   value="65" aria-label="Robot render size (percent)">
            <span id="ctps-editor-zoom-readout"
                  class="ctps-editor-zoom-readout">65%</span>
        </div>
        <button id="ctps-editor-save" class="ctps-help-btn" type="button">
            &#8615; Save</button>
        <span id="ctps-editor-save-state" class="ctps-editor-save-state"
              role="status" aria-live="polite"></span>
        <button id="ctps-editor-load-btn" class="ctps-help-btn" type="button">
            &#8613; Load</button>
        <input type="file" id="ctps-editor-file" class="ctps-layout-file"
               accept="application/json,.json" hidden>
    </header>
    <div class="ctps-main ctps-editor-main">
        <section class="ctps-stage ctps-editor-stage">
            <div class="ctps-canvas-stack">
                <canvas id="ctps-editor-canvas" class="ctps-canvas-layer"></canvas>
            </div>
            <div class="ctps-scale-bar">
                <span class="ctps-scale-line"></span>
                <span class="ctps-scale-label">50 cm</span>
            </div>
            <div id="ctps-editor-status" class="ctps-editor-status"></div>
        </section>
        <aside class="ctps-rail ctps-editor-rail">
            <section class="ctps-panel ctps-panel--props">
                <div class="ctps-panel-title">Properties</div>
                <div id="ctps-editor-props" class="ctps-editor-props">
                    <div id="ctps-editor-props-empty" class="ctps-editor-props-empty">
                        No object selected
                    </div>
                    <div id="ctps-editor-props-fields" class="ctps-editor-props-fields"
                         hidden>
                        <label class="ctps-editor-field">
                            <span class="ctps-editor-field-label">Name</span>
                            <input type="text" id="ctps-editor-prop-name"
                                   class="ctps-editor-input">
                        </label>
                        <label class="ctps-editor-field">
                            <span class="ctps-editor-field-label">Type</span>
                            <select id="ctps-editor-prop-type" class="ctps-editor-input">
                                <option value="tile">Tile</option>
                                <option value="cargo">Cargo</option>
                                <option value="fixed">Fixed</option>
                            </select>
                        </label>
                        <div class="ctps-editor-field">
                            <span class="ctps-editor-field-label">Color</span>
                            <input type="color" id="ctps-editor-prop-color"
                                   class="ctps-editor-input ctps-editor-input--color">
                            <div id="ctps-editor-swatches"
                                 class="ctps-editor-swatches"></div>
                        </div>
                        <div class="ctps-editor-field-row">
                            <label class="ctps-editor-field">
                                <span class="ctps-editor-field-label">X (cm)</span>
                                <input type="number" step="0.1" id="ctps-editor-prop-x"
                                       class="ctps-editor-input">
                            </label>
                            <label class="ctps-editor-field">
                                <span class="ctps-editor-field-label">Y (cm)</span>
                                <input type="number" step="0.1" id="ctps-editor-prop-y"
                                       class="ctps-editor-input">
                            </label>
                        </div>
                        <div id="ctps-editor-fields-rect" class="ctps-editor-rect-fields">
                            <div class="ctps-editor-field-row">
                                <label class="ctps-editor-field">
                                    <span class="ctps-editor-field-label">W (cm)</span>
                                    <input type="number" step="0.1"
                                           id="ctps-editor-prop-w"
                                           class="ctps-editor-input">
                                </label>
                                <label class="ctps-editor-field">
                                    <span class="ctps-editor-field-label">H (cm)</span>
                                    <input type="number" step="0.1"
                                           id="ctps-editor-prop-h"
                                           class="ctps-editor-input">
                                </label>
                            </div>
                            <label class="ctps-editor-field">
                                <span class="ctps-editor-field-label">Rotation (deg)</span>
                                <input type="number" step="1"
                                       id="ctps-editor-prop-heading"
                                       class="ctps-editor-input">
                            </label>
                        </div>
                        <div id="ctps-editor-fields-circle"
                             class="ctps-editor-circle-fields" hidden>
                            <label class="ctps-editor-field">
                                <span class="ctps-editor-field-label">Radius (cm)</span>
                                <input type="number" step="0.1"
                                       id="ctps-editor-prop-radius"
                                       class="ctps-editor-input">
                            </label>
                        </div>
                        <button id="ctps-editor-delete" class="ctps-editor-delete"
                                type="button">Delete object</button>
                    </div>
                </div>
            </section>
        </aside>
    </div>
</div>"""

#: Page skeleton. ``{styles}`` receives the concatenated CSS; ``{body}`` the
#: static :data:`_BODY_MARKUP`; ``{scripts}`` the assembled ``<script>`` blocks
#: (the ``FIELD`` / ``MAP_IMAGE`` constants + one per JS asset). The skeleton
#: deliberately holds NO ``<script>`` tags of its own -- every script lives in
#: ``{scripts}`` -- so this template string never contains a literal
#: ``</script>`` and passes the self-containment lint. ``str.format``
#: substitutes the values verbatim: the ``{`` / ``}`` inside the CSS/JS values
#: are inert, so only these three placeholders are interpreted.
_HTML_TEMPLATE: str = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CTPS &mdash; Layout Editor</title>
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


def _map_data_uri(map_png: "str | Path | None") -> str:
    """Return a JS literal for the optional field-map background.

    With no map requested (``map_png is None``) returns the literal ``"null"``
    so the editor draws its vector field fallback. Otherwise reads the image
    bytes, base64-encodes them into a self-contained ``data:`` URI, and returns
    it as a quoted JS string (via :func:`json.dumps`). A missing file is
    non-fatal: it warns and returns ``"null"`` so the page still renders without
    the map (mirroring :mod:`tools.path_planner.ctps.player_html`).

    The data URI embeds no external reference, keeping the emitted page offline
    (AC1); base64 contains no ``<`` so it cannot break out of the inline
    ``<script>``. Emitted as a JSON *value*, it is never passed through the
    self-containment lint (the ``data:image`` ban targets assets/markup only).

    :param map_png: path to the field map image, or ``None`` for no map.
    :return: a JS expression string -- either ``"null"`` or a quoted ``data:``
        URI.
    """
    if map_png is None:
        return "null"
    path = Path(map_png)
    if not path.is_file():
        warnings.warn(
            f"ctps.layout_editor_html.render: map image not found at {path}; "
            "rendering the editor without a map background.",
            stacklevel=2,
        )
        return "null"
    suffix = path.suffix.lower().lstrip(".") or "png"
    mime = "jpeg" if suffix in ("jpg", "jpeg") else suffix
    b64 = base64.b64encode(path.read_bytes()).decode("ascii")
    return json.dumps(f"data:image/{mime};base64,{b64}")


#: The ``id`` on the ``<script>`` that carries ``const LAYOUT``. The edit-server
#: swaps this whole element on every GET (see :func:`reembed_layout`) so a reload
#: reflects the CURRENT on-disk ``layout.json`` — not the stale one-time embed
#: baked at server start, which a still-alive server would otherwise keep serving
#: after a Save (the fixed-port reopen bug: every field object vanishes while the
#: robot falls back to its default).
_LAYOUT_SCRIPT_ID: str = "ctps-layout-embed"

#: Matches the whole id'd LAYOUT ``<script>`` element for a per-GET swap. Its body
#: is a single ``const LAYOUT = <json>;`` line whose ``</`` is escaped, so the
#: non-greedy ``.*?`` can never terminate on an earlier ``</script>``.
_LAYOUT_SCRIPT_RE = re.compile(
    r'<script id="' + re.escape(_LAYOUT_SCRIPT_ID) + r'">.*?</script>',
    re.DOTALL,
)


def _layout_literal(layout: "Dict[str, Any] | None") -> str:
    """Return the JS literal for ``const LAYOUT`` — the layout JSON, or ``null``.

    Emitted as a JSON VALUE (like ``FIELD``): ``</`` is escaped so an author's
    object name can never break out of the inline ``<script>``. A falsy
    ``layout`` yields ``"null"`` so the editor boots empty.
    """
    if layout:
        return json.dumps(layout, ensure_ascii=False).replace("</", "<\\/")
    return "null"


def _layout_script(layout: "Dict[str, Any] | None") -> str:
    """Build the id'd ``<script>`` element carrying ``const LAYOUT``.

    The ``id`` lets the edit-server replace this element wholesale per GET
    (:func:`reembed_layout`); the ``const LAYOUT = ...;`` line itself is kept
    intact so the boot loader and the emitter tests read it unchanged.
    """
    return (
        f'<script id="{_LAYOUT_SCRIPT_ID}">\n'
        f"const LAYOUT = {_layout_literal(layout)};\n"
        "</script>"
    )


def reembed_layout(html_text: str, layout: "Dict[str, Any] | None") -> str:
    """Return ``html_text`` with its embedded ``const LAYOUT`` replaced by ``layout``.

    The edit-server calls this on every ``GET`` so a reload/refresh shows the
    CURRENT on-disk layout rather than the stale one-time embed captured at
    server start. Without it, a fixed-port reopen where the old server never
    rebinds keeps serving that start-time embed after a Save — on a fresh clone
    (no ``layout.json`` yet) an empty scene, so every just-authored field object
    disappears while the robot shows its :func:`~...editor_core.defaultRobot`
    fallback. A falsy ``layout`` re-embeds ``const LAYOUT = null;``.

    :param html_text: a page previously emitted by :func:`render`.
    :param layout: the layout dict to embed, or ``None``/falsy for an empty scene.
    :return: the page text with the LAYOUT ``<script>`` element swapped.
    """
    # A function replacement (not a template string) so a backslash in the
    # embedded JSON is never interpreted as a ``re.sub`` group reference.
    return _LAYOUT_SCRIPT_RE.sub(lambda _m: _layout_script(layout), html_text, count=1)


def render(
    field_data: Any,
    out_path: "str | Path",
    map_png: "str | Path | None" = None,
    layout: "Dict[str, Any] | None" = None,
) -> Path:
    """Render the offline layout editor to one self-contained page at ``out_path``.

    Emits a single ``layout_editor.html`` that embeds, all inline: the
    concatenated CSS assets in one ``<style>``; a ``const FIELD`` field-backdrop
    constant derived from ``field_data`` (the same block the player draws -- see
    :func:`tools.path_planner.ctps.track_builder.build_field_block`); an
    optional ``const MAP_IMAGE`` data URI; a ``const LAYOUT`` the editor
    auto-loads on open (``null`` when none); and every JS asset
    (:data:`_ASSET_JS`) in load order, each in its own ``<script>``
    (``objects_draw.js`` first so ``window.CTPS_OBJ`` exists for the editor
    canvas, then the editor modules, ``editor_boot.js`` last). The page has
    **no** external ``src`` / ``href`` to local files, so it runs offline on
    ``file://`` with zero network or local dependency (AC1).

    Before writing, :func:`_assert_self_contained` lints the assets + the HTML
    template markup (never the embedded ``FIELD`` / ``MAP_IMAGE`` JSON values,
    which carry arbitrary field text) and raises if any is not self-contained
    (AC1, P-TC7).

    When ``map_png`` is given it is base64-embedded as a self-contained
    ``data:`` URI background (the page stays offline); otherwise the editor
    falls back to its vector field render. A missing map file warns and is
    treated as no map.

    :param field_data: a validated field-data namespace exposing ``NODES``,
        ``OBSTACLES``, ``FIELD_MM`` and ``ROBOT_RADIUS_CM`` (e.g. the live
        :mod:`tools.path_planner.field_data`); used to build ``const FIELD``.
    :param out_path: destination ``.html`` path (``str`` or
        :class:`pathlib.Path`).
    :param map_png: optional path to the printed field map image to embed as the
        canvas background; ``None`` (default) renders the vector field only.
    :param layout: optional ``layout.json``-shaped dict (field objects + robot
        zones) to embed as ``const LAYOUT`` so the editor auto-loads it on open
        (the recently-saved layout, not a fresh scene); ``None``/falsy emits
        ``const LAYOUT = null;`` and the editor boots empty. Embedded as a JSON
        VALUE (like ``FIELD``) and TOLERANTLY ingested at boot — never strictly
        validated here, so a slightly-off saved layout still opens for editing.
    :return: the resolved :class:`pathlib.Path` that was written.
    :raises FileNotFoundError: if any required asset (e.g. ``editor_core.js``)
        is missing.
    :raises ValueError: if an asset or the HTML template is not self-contained.
    :raises AttributeError: if ``field_data`` omits a required attribute.
    """
    # The editor's field backdrop -- single-sourced from the player's field
    # block so both pages draw the same nodes/obstacles/bounds. Emitted as a
    # JSON VALUE (like the player's TRACK): ``</`` is escaped so a node label
    # containing "</script>" cannot break out of the inline <script>, and it is
    # NOT passed through the self-containment lint.
    field_block: Dict[str, Any] = build_field_block(field_data)
    field_json = json.dumps(field_block, ensure_ascii=False).replace("</", "<\\/")

    # The layout to auto-load (recently-saved objects + zones) is emitted by
    # `_layout_script` below as an id'd <script> the edit-server re-embeds per GET.

    # Optional field-map background as a self-contained data URI (or "null").
    map_image_js = _map_data_uri(map_png)

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

    # Inline constants first (editor_core/editor_boot ingest FIELD/MAP_IMAGE),
    # then each JS asset in its own <script> in load order.
    script_blocks: List[str] = [
        _script(f"const FIELD = {field_json};"),
        _script(f"const MAP_IMAGE = {map_image_js};"),
        _layout_script(layout),
    ]
    script_blocks.extend(_script(content) for content in js_contents)
    scripts = "\n".join(script_blocks)

    html = _HTML_TEMPLATE.format(styles=styles, body=_BODY_MARKUP, scripts=scripts)

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    # LF line endings, UTF-8, no BOM -- deterministic across platforms.
    with open(out, "w", encoding="utf-8", newline="") as fh:
        fh.write(html)

    return out
