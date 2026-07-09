"""Hermetic tests for the CTPS layout-editor HTML emitter (object/cargo plan,
T7 -- issue #57): P-TC7 (self-contained + offline) and P-TC8 (every editor
asset + ``objects_draw.js`` inlined verbatim).

These exercise :func:`tools.path_planner.ctps.layout_editor_html.render` and the
inlined editor ``assets/*`` runtime as **text** -- no browser, no compiler, no
network. They are the automated leg of AC1; the editor's runtime behavior
(draw / select / save / load) is manual-browser-verified per the plan.

Covered cases:

* **P-TC7** -- rendering the editor writes exactly one self-contained ``.html``
  whose embedded ``const FIELD = {...}`` round-trips through :func:`json.loads`
  to the player's own field block, carries the editor's DOM scaffold (the
  Field/Robot tabs, the Rect/Circle tools, the canvas, the Properties panel),
  and references no external ``src`` / ``href`` and no ``http``/``https``
  resource (offline on ``file://``).
* **P-TC8** -- every registered asset (``objects_draw.js`` + the four
  ``editor_*.js`` + ``editor.css`` + ``tokens.css``) exists on disk and is
  inlined verbatim, in the pinned load order (``objects_draw.js`` first,
  ``editor_boot.js`` last); a missing one fails the render loud.

Plus the emit-time self-containment lint (positive + negative), the optional
map-background data URI, and the no-inline-style guard on the new editor assets
-- the same invariants the player suite pins, applied to the editor's own asset
tuple.
"""

from __future__ import annotations

import base64
import json
import re

import pytest

from tools.path_planner.ctps import layout_editor_html as editor_module
from tools.path_planner.ctps.layout_editor_html import (
    _ASSET_CSS,
    _ASSET_JS,
    _assert_self_contained,
    render,
)
from tools.path_planner.ctps.track_builder import build_field_block
from tools.path_planner.tests.fixtures import toy_field_data

#: Captures the JSON document assigned to ``const FIELD`` in the emitted page.
#: ``const FIELD`` lives in its OWN ``<script>`` (same non-greedy-to-
#: ``</script>`` shape as the player emitter's TRACK/PLAYER_META regexes), so
#: this terminates at the FIELD assignment alone.
_FIELD_RE = re.compile(r"const FIELD = (?P<json>.*?);\s*</script>", re.DOTALL)


def _extract_embedded_field(html: str) -> dict:
    """Pull the embedded ``const FIELD = {...}`` dict back out of the page.

    The emitter escapes ``</`` to ``<\\/`` so a node label can never break out
    of the inline ``<script>``; ``<\\/`` is still valid JSON, but we reverse it
    first so the recovered text round-trips cleanly.

    :param html: the full rendered page text.
    :return: the parsed field-backdrop dict.
    """
    match = _FIELD_RE.search(html)
    assert match is not None, "could not locate the embedded const FIELD assignment"
    raw_json = match.group("json").replace("<\\/", "</")
    return json.loads(raw_json)


#: An assignment-shaped inline-style match ONLY -- ``.style.setProperty(`` (the
#: sanctioned ADR-053 styling channel) is never followed by ``=`` so it never
#: matches. Mirrors the player suite's TC-NOINLINE regex.
_INLINE_STYLE_ASSIGNMENT_RE = re.compile(r"\.style\.[A-Za-z_]\w*\s*=")


# ---------------------------------------------------------------------------
# P-TC7 -- render() emits one self-contained, offline editor page
# ---------------------------------------------------------------------------


def test_p_tc7_render_is_self_contained(tmp_path):
    """P-TC7: render() writes one offline .html whose FIELD round-trips (AC1).

    Key assertion: exactly one ``.html`` is written; the embedded
    ``const FIELD`` round-trips via :func:`json.loads` to the player's own
    field block (:func:`build_field_block`); the editor DOM scaffold (the
    Field/Robot tab row, the Rect/Circle tools, the canvas, the Properties
    panel) is present; and the page references no external local file
    (no ``src=``/``href=``) and no ``http``/``https`` resource.
    """
    out_path = tmp_path / "layout_editor.html"
    returned = render(toy_field_data, out_path)

    # Exactly one .html written, at the requested location.
    assert returned == out_path
    html_files = list(tmp_path.glob("*.html"))
    assert html_files == [out_path]

    html = out_path.read_text(encoding="utf-8")

    # The embedded field backdrop round-trips to the player's own field block.
    embedded = _extract_embedded_field(html)
    assert embedded == build_field_block(toy_field_data)

    # Editor DOM scaffold (sidecar Change 3) is present.
    assert 'data-editor-tab="field"' in html
    assert 'data-editor-tab="robot"' in html
    assert 'data-tool="rect"' in html
    assert 'data-tool="circle"' in html
    assert 'id="ctps-editor-canvas"' in html
    assert "<canvas" in html
    assert 'id="ctps-editor-props"' in html
    assert 'id="ctps-editor-save"' in html
    assert '<input type="file" id="ctps-editor-file"' in html

    # Self-contained: no external resource references.
    assert "http://" not in html
    assert "https://" not in html
    # No <script src=...> / <link href=...> to a local asset; everything inline.
    assert "src=" not in html
    assert "href=" not in html
    # No map requested -> no inline image URI at all.
    assert "data:image" not in html
    assert "const MAP_IMAGE = null;" in html


# ---------------------------------------------------------------------------
# P-TC8 -- every editor asset (+ objects_draw.js) is inlined verbatim
# ---------------------------------------------------------------------------


def test_p_tc8_every_editor_asset_exists_and_is_inlined(tmp_path):
    """P-TC8: every registered editor asset exists on disk and is inlined verbatim.

    Key assertion: for each path in ``_ASSET_JS + _ASSET_CSS``, the file exists
    and its exact on-disk content appears as a contiguous substring of the
    rendered page -- the emitter neither drops nor mangles a sibling asset (each
    JS file goes into its own ``<script>``, the CSS files concatenate into one
    ``<style>``). Explicitly covers the shared ``objects_draw.js`` and each of
    the four ``editor_*.js`` placeholders plus ``editor.css``.
    """
    out_path = tmp_path / "assets.html"
    render(toy_field_data, out_path)
    html = out_path.read_text(encoding="utf-8")

    for path in (*_ASSET_JS, *_ASSET_CSS):
        assert path.is_file(), f"missing editor asset: {path}"
        content = path.read_text(encoding="utf-8")
        assert content in html, f"{path.name} was not inlined verbatim into the page"

    # The exact asset set the plan pins for this page is registered.
    js_names = [p.name for p in _ASSET_JS]
    assert js_names == [
        "objects_draw.js",
        "editor_core.js",
        "editor_tools.js",
        "editor_io.js",
        "server_session.js",
        "editor_boot.js",
    ]
    assert [p.name for p in _ASSET_CSS] == ["tokens.css", "editor.css"]


def test_p_tc8_pinned_load_order_objects_draw_first_boot_last(tmp_path):
    """P-TC8: objects_draw.js is inlined first and editor_boot.js last.

    Key assertion: ``objects_draw.js`` (establishes ``window.CTPS_OBJ`` the
    editor canvas reuses) precedes every editor module, and ``editor_boot.js``
    (the one-time init) follows them -- the pinned load contract.
    """
    out_path = tmp_path / "order.html"
    render(toy_field_data, out_path)
    html = out_path.read_text(encoding="utf-8")

    assets_dir = _ASSET_JS[0].parent
    objects_draw = (assets_dir / "objects_draw.js").read_text(encoding="utf-8")
    core = (assets_dir / "editor_core.js").read_text(encoding="utf-8")
    tools = (assets_dir / "editor_tools.js").read_text(encoding="utf-8")
    io = (assets_dir / "editor_io.js").read_text(encoding="utf-8")
    boot = (assets_dir / "editor_boot.js").read_text(encoding="utf-8")

    assert (
        html.index(objects_draw)
        < html.index(core)
        < html.index(tools)
        < html.index(io)
        < html.index(boot)
    )


# ---------------------------------------------------------------------------
# Self-containment lint -- positive (bad asset blocks the render) + integration
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "offender",
    ["src=", "href=", "http://", "https://", "data:image", "</script>"],
)
def test_lint_rejects_each_forbidden_substring(offender):
    """The shared lint raises naming the offender for each banned substring."""
    sources = {"evil_editor_asset.js": f"(function () {{ var x = '{offender}'; }})();"}
    with pytest.raises(ValueError) as exc_info:
        _assert_self_contained(sources)
    message = str(exc_info.value)
    assert "evil_editor_asset.js" in message
    assert offender in message


def test_render_raises_on_forbidden_asset(tmp_path, monkeypatch):
    """A banned substring in a real editor asset blocks the render (AC1).

    Points the emitter at one extra temp asset carrying a network URL by
    monkeypatching ``layout_editor_html._ASSET_JS`` and asserts :func:`render`
    raises :class:`ValueError` naming the offending asset -- and never writes
    the output file.
    """
    bad_asset = tmp_path / "evil_editor_asset.js"
    bad_asset.write_text(
        "(function () { var u = 'https://example.com/tracker.js'; })();",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        editor_module, "_ASSET_JS", editor_module._ASSET_JS + (bad_asset,)
    )

    out_path = tmp_path / "should_not_exist.html"
    with pytest.raises(ValueError) as exc_info:
        render(toy_field_data, out_path)
    message = str(exc_info.value)
    assert bad_asset.name in message
    assert "https://" in message
    assert not out_path.exists()


def test_render_raises_on_missing_asset(tmp_path, monkeypatch):
    """A missing sibling asset fails render() loud, naming its path (AC1)."""
    missing = tmp_path / "does_not_exist.js"
    monkeypatch.setattr(editor_module, "_ASSET_JS", (missing,))

    out_path = tmp_path / "missing_asset.html"
    with pytest.raises(FileNotFoundError, match=re.escape(str(missing))):
        render(toy_field_data, out_path)
    assert not out_path.exists()


# ---------------------------------------------------------------------------
# Optional map background -- self-contained data URI, or null fallback
# ---------------------------------------------------------------------------


def test_map_png_embeds_self_contained_data_uri(tmp_path):
    """render(map_png=...) embeds the image as an offline ``data:`` URI.

    Key assertion: the page carries ``const MAP_IMAGE = "data:image/png;base64,
    <b64>"`` for the supplied bytes, stays self-contained (no http/https), and
    the embedded ``const FIELD`` still round-trips unchanged alongside the map.
    """
    map_file = tmp_path / "field.png"
    raw = b"\x89PNG\r\n\x1a\nctps-editor-map-bytes"
    map_file.write_bytes(raw)

    out_path = tmp_path / "with_map.html"
    render(toy_field_data, out_path, map_png=map_file)
    html = out_path.read_text(encoding="utf-8")

    expected_b64 = base64.b64encode(raw).decode("ascii")
    assert f'const MAP_IMAGE = "data:image/png;base64,{expected_b64}";' in html
    assert "http://" not in html
    assert "https://" not in html
    assert _extract_embedded_field(html) == build_field_block(toy_field_data)


def test_no_map_png_emits_null(tmp_path):
    """Without ``map_png`` the editor embeds ``MAP_IMAGE = null`` (vector fallback)."""
    out_path = tmp_path / "no_map.html"
    render(toy_field_data, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert "const MAP_IMAGE = null;" in html
    assert "data:image" not in html


def test_missing_map_png_warns_and_falls_back(tmp_path):
    """A ``map_png`` that does not exist warns and degrades to ``MAP_IMAGE = null``."""
    out_path = tmp_path / "missing_map.html"
    with pytest.warns(UserWarning, match="map image not found"):
        render(toy_field_data, out_path, map_png=tmp_path / "does_not_exist.png")
    html = out_path.read_text(encoding="utf-8")

    assert "const MAP_IMAGE = null;" in html


# ---------------------------------------------------------------------------
# No-inline-style guard on the new editor assets (TC-NOINLINE analogue)
# ---------------------------------------------------------------------------


def test_no_inline_style_assignment_in_any_editor_asset():
    """No editor JS asset sets an inline style via direct assignment (P-TC9 leg).

    Key assertion: for every registered JS asset, an assignment-shaped
    ``.style.<prop> =`` never appears; ``.style.setProperty(...)`` (the ADR-053
    carve-out) is unaffected since it is never followed by ``=``.
    """
    for path in _ASSET_JS:
        js = path.read_text(encoding="utf-8")
        matches = _INLINE_STYLE_ASSIGNMENT_RE.findall(js)
        assert matches == [], f"{path.name} sets an inline style directly: {matches}"


def test_editor_assets_have_no_static_inline_style_attribute():
    """The editor markup + assets carry no static ``style=`` attribute either."""
    for path in (*_ASSET_JS, *_ASSET_CSS):
        assert "style=" not in path.read_text(encoding="utf-8"), (
            f"{path.name} carries a static inline style= attribute"
        )
    assert "style=" not in editor_module._BODY_MARKUP


# ---------------------------------------------------------------------------
# const LAYOUT — the editor auto-loads the recently-saved layout on open
# ---------------------------------------------------------------------------

#: Captures the JSON document assigned to ``const LAYOUT`` (own ``<script>``,
#: same non-greedy-to-``</script>`` shape as the FIELD regex above).
_LAYOUT_RE = re.compile(r"const LAYOUT = (?P<json>.*?);\s*</script>", re.DOTALL)

#: A minimal well-formed layout.json-shaped dict (one cargo rect + robot zones).
_SAMPLE_LAYOUT = {
    "schema_version": 1,
    "objects": [
        {
            "id": "o1", "name": "block", "shape": "rect", "type": "cargo",
            "color": "#5bc8e0", "x_cm": 12.0, "y_cm": -3.0,
            "heading_deg": 0.0, "w_cm": 5.0, "h_cm": 5.0,
        }
    ],
    "robot": {
        "grab_zone": {"x_cm": -1.0, "y_cm": 8.0, "w_cm": 6.0, "h_cm": 14.0},
        "box_zone": {"x_cm": -9.0, "y_cm": 0.0, "w_cm": 6.0, "h_cm": 16.0},
        "drop_points": [{"x_cm": 4.0, "y_cm": -6.0}],
    },
}


def test_layout_embeds_const_for_editor_autoload(tmp_path):
    """render(layout=...) embeds a ``const LAYOUT`` that round-trips to the dict.

    Key assertion: the passed layout is embedded verbatim (round-trips via
    :func:`json.loads`) and the boot script ingests it BEFORE the save-state
    baseline, so a fresh open shows the recently-saved scene, not a blank one.
    """
    out_path = tmp_path / "autoload.html"
    render(toy_field_data, out_path, layout=_SAMPLE_LAYOUT)
    html = out_path.read_text(encoding="utf-8")

    match = _LAYOUT_RE.search(html)
    assert match is not None, "could not locate the embedded const LAYOUT"
    embedded = json.loads(match.group("json").replace("<\\/", "</"))
    assert embedded == _SAMPLE_LAYOUT

    # Boot ingests LAYOUT before seeding the clean save-state baseline.
    assert "E.ingestLayout(LAYOUT)" in html
    assert html.index("E.ingestLayout(LAYOUT)") < html.index("E.initSaveState()")


def test_no_layout_embeds_null(tmp_path):
    """Without ``layout`` the editor embeds ``const LAYOUT = null;`` (boots empty)."""
    out_path = tmp_path / "no_layout.html"
    render(toy_field_data, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert "const LAYOUT = null;" in html


def test_reembed_layout_swaps_the_embedded_layout(tmp_path):
    """``reembed_layout`` replaces a page's ``const LAYOUT`` in place.

    The edit-server's per-GET hook: a page emitted with ``LAYOUT = null`` (a
    fresh clone with no saved file) is re-embedded with the just-saved layout so
    a reload shows it instead of the stale empty scene. The rest of the page —
    the FIELD block, the boot ``E.ingestLayout(LAYOUT)`` call — is untouched.
    """
    out_path = tmp_path / "empty.html"
    render(toy_field_data, out_path)  # LAYOUT = null
    empty_html = out_path.read_text(encoding="utf-8")
    assert "const LAYOUT = null;" in empty_html

    reembedded = editor_module.reembed_layout(empty_html, _SAMPLE_LAYOUT)

    match = _LAYOUT_RE.search(reembedded)
    assert match is not None, "re-embedded page lost its const LAYOUT script"
    assert json.loads(match.group("json").replace("<\\/", "</")) == _SAMPLE_LAYOUT
    assert "const LAYOUT = null;" not in reembedded
    # The swap is surgical: only the LAYOUT script changes.
    assert "E.ingestLayout(LAYOUT)" in reembedded
    assert "const FIELD = " in reembedded


def test_reembed_layout_none_reverts_to_null(tmp_path):
    """Re-embedding ``None`` restores ``const LAYOUT = null;`` (an empty scene)."""
    out_path = tmp_path / "full.html"
    render(toy_field_data, out_path, layout=_SAMPLE_LAYOUT)
    full_html = out_path.read_text(encoding="utf-8")
    assert "const LAYOUT = null;" not in full_html

    reverted = editor_module.reembed_layout(full_html, None)
    assert "const LAYOUT = null;" in reverted
