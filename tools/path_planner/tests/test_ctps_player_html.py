"""Hermetic tests for the CTPS HTML player emitter (T10 + T14: TC5/TC6/TC12 +
the design-3a rework's TC-META/TC-LINT/TC-STRUCT/TC-ASSETS/TC-SEEKINPUT/
TC-TRAIL/TC-GOTOEND/TC-NOINLINE/TC-DUPREG) plus the pivot-wheel-arc issue's
TC8/TC9 (``render_mode`` selection, issue #55 / T6).

These tests exercise :func:`tools.path_planner.ctps.player_html.render` and
the inlined ``assets/*.js`` runtime as **text** — no browser, no compiler, no
network.

Covered cases (CTPS UI/UX rework plan):

* **TC5** — rendering a :class:`Track` writes exactly one self-contained
  ``.html`` whose embedded ``const TRACK = {...}`` round-trips through
  :func:`json.loads`, carries the transport-bar markers, and has no external
  ``src`` / ``href`` to a local file and no ``http``/``https`` resource.
* **TC6** — rendering a hand-built LRS-shaped Track DICT (poses + events only,
  no command stream) embeds that dict unchanged, proving the player is
  source-agnostic (AC7).
* **TC12** — the inlined ``player.js`` resolves a seek via a bounded binary
  search (``findKeyframeIndex`` referenced by ``poseAt``), not an unbounded
  per-frame full keyframe scan (AC3a).
* **TC-META** — ``const PLAYER_META = {...}`` embeds and round-trips
  alongside (but independently of) TRACK (AC-meta).
* **TC-LINT-pos/neg** — the emit-time self-containment lint rejects a
  forbidden substring in an asset (naming the offender) and never raises on a
  normal render with a real map image (AC-lint).
* **TC-STRUCT** — the design-3a container classes + rail ids are present
  (AC-contract).
* **TC-ASSETS** — every registered asset exists on disk and is inlined
  verbatim; a missing one fails loud.
* **TC-SEEKINPUT** — the seek control is a native ``<input type="range"
  id="ctps-seek">``.
* **TC-TRAIL** — the traveled-trail rebuild in ``renderer.js`` is bounded via
  ``findKeyframeIndex``, never a full per-frame keyframe scan (AC-scrub).
* **TC-GOTOEND** — the GOTO lane in ``timeline.js`` discriminates success
  spans (``t_end``) from ``UNREACHABLE`` fault pins (AC-timeline).
* **TC-NOINLINE** — no JS asset sets an inline style via direct assignment;
  only ``.style.setProperty(...)`` is used (the ADR-053 carve-out).
* **TC-DUPREG** — a duplicate registry id is non-fatal: ``register()`` never
  throws, and ``boot.js`` surfaces accumulated ``_errors``.
* **TC8** — rendering a track whose dict carries a non-empty ``pivots`` list
  emits ``PLAYER_META.render_mode == "wheel-arc"``, and the embedded
  ``TRACK`` carries that exact ``pivots`` list unchanged (AC5).
* **TC9** — rendering a pivot-free track keeps ``render_mode == "box"``, the
  embedded ``TRACK`` has no ``"pivots"`` key, and ``PLAYER_META`` equals the
  expected literal dict verbatim — structural equality stands in for a
  byte-vs-pre-change diff, which a test cannot perform (AC6).

Also covers the object/cargo plan's emitter leg (T4 — issue #57):

* **P-TC5** — ``render(objects=None)`` (the default) emits no
  ``const OBJECTS`` at all; ``TRACK`` still round-trips and ``PLAYER_META``
  equals the exact TC9 literal — proving the new cargo markup/assets never
  perturb the pre-existing contract (AC11).
* **P-TC6** — ``render(objects=<layout>)`` embeds ``const OBJECTS`` (round-
  trips byte-for-byte), inlines ``objects_draw.js`` then ``cargo.js`` (in that
  order, both before ``boot.js``), and ``PLAYER_META`` is STILL the unchanged
  TC9 literal (AC5 embed leg).
* **TC-CARGO-STRUCT** — the two-tab cargo panel DOM contract
  (``#ctps-cargo``, ``.ctps-cargo-tab[data-cargo-tab]``, the count badges, the
  magazine grid, the inventory list), the new ``OBJECTS``/``ZONES`` layer
  chips, and the ``#ctps-layout-file`` upload input are present, and the
  retired ``.ctps-panel--holding`` shell is gone (AC5 chips + the DOM contract
  T6 queries).
"""

from __future__ import annotations

import base64
import json
import re
import warnings
from pathlib import Path

import pytest

from tools.path_planner.ctps import player_html as player_html_module
from tools.path_planner.ctps.layout import LayoutError
from tools.path_planner.ctps.player_html import (
    PLAYER_JS_PATH,
    _ASSET_CSS,
    _ASSET_JS,
    _assert_self_contained,
    render,
)
from tools.path_planner.ctps.track_builder import build
from tools.path_planner.ctps.trace_loader import load
from tools.path_planner.tests.fixtures import toy_field_data

_FIXTURES = Path(__file__).resolve().parent / "fixtures"

#: Captures the JSON document assigned to ``const TRACK`` in the emitted page.
#: Non-greedy up to the ``;`` that terminates the assignment statement.
_TRACK_RE = re.compile(r"const TRACK = (?P<json>.*?);\s*</script>", re.DOTALL)


def _extract_embedded_track(html: str) -> dict:
    """Pull the embedded ``const TRACK = {...}`` dict back out of the page.

    The emitter escapes ``</`` to ``<\\/`` so a label can never break out of
    the inline ``<script>`` tag; ``<\\/`` is still valid JSON, but we reverse
    it first so the recovered text is byte-identical to the original
    ``json.dumps`` output and round-trips cleanly.

    :param html: the full rendered page text.
    :return: the parsed Data Model B track dict.
    """
    match = _TRACK_RE.search(html)
    assert match is not None, "could not locate the embedded const TRACK assignment"
    raw_json = match.group("json").replace("<\\/", "</")
    return json.loads(raw_json)


#: Captures the JSON document assigned to ``const PLAYER_META`` in the emitted
#: page. PLAYER_META lives in its OWN ``<script>`` (see player_html.py
#: Contracts -> Emitter), so this sibling regex — same non-greedy-to-
#: ``</script>`` shape as :data:`_TRACK_RE` — terminates at PLAYER_META alone
#: and never swallows (or is swallowed by) the TRACK assignment.
_META_RE = re.compile(r"const PLAYER_META = (?P<json>.*?);\s*</script>", re.DOTALL)


def _extract_player_meta(html: str) -> dict:
    """Pull the embedded ``const PLAYER_META = {...}`` dict back out of the page.

    Mirrors :func:`_extract_embedded_track`: the same ``</`` -> ``<\\/`` escape
    is reversed before parsing so the recovered text round-trips cleanly.

    :param html: the full rendered page text.
    :return: the parsed PLAYER_META dict.
    """
    match = _META_RE.search(html)
    assert match is not None, "could not locate the embedded const PLAYER_META assignment"
    raw_json = match.group("json").replace("<\\/", "</")
    return json.loads(raw_json)


#: Captures the JSON document assigned to ``const OBJECTS`` in the emitted
#: page (object/cargo plan, T4 — issue #57). ``const OBJECTS`` lives in its
#: OWN ``<script>`` (same non-greedy-to-``</script>`` shape as
#: :data:`_TRACK_RE`/:data:`_META_RE`) and is only emitted when a truthy
#: ``objects`` layout was given to :func:`render`, so this regex legitimately
#: has no match on a layout-free render (AC11).
_OBJECTS_RE = re.compile(r"const OBJECTS = (?P<json>.*?);\s*</script>", re.DOTALL)


def _extract_embedded_objects(html: str) -> "dict | None":
    """Pull the embedded ``const OBJECTS = {...}`` dict out of the page.

    Mirrors :func:`_extract_embedded_track`. Returns ``None`` (rather than
    asserting) when no ``const OBJECTS`` assignment is present at all — the
    expected shape for a layout-free render (P-TC5).

    :param html: the full rendered page text.
    :return: the parsed OBJECTS dict, or ``None`` if absent.
    """
    match = _OBJECTS_RE.search(html)
    if match is None:
        return None
    raw_json = match.group("json").replace("<\\/", "</")
    return json.loads(raw_json)


def _valid_layout_dict() -> dict:
    """Return a fresh, schema-valid ``layout.json`` dict (schema_version 1).

    A minimal but fully-populated layout (one rect Tile object + a complete
    ``robot`` zone section) — mirrors the shared fixture in
    ``test_ctps_layout.py`` (T2) without importing across test modules.
    """
    return {
        "schema_version": 1,
        "objects": [
            {
                "id": "o1",
                "name": "green tile",
                "shape": "rect",
                "type": "tile",
                "color": "#57b36b",
                "x_cm": 40.0,
                "y_cm": 120.0,
                "heading_deg": 0.0,
                "w_cm": 5.0,
                "h_cm": 5.0,
            },
        ],
        "robot": {
            "grab_zone": {"x_cm": -1.0, "y_cm": 8.0, "w_cm": 6.0, "h_cm": 14.0},
            "box_zone": {"x_cm": -9.0, "y_cm": 0.0, "w_cm": 6.0, "h_cm": 16.0},
            "drop_points": [
                {"x_cm": 4.0, "y_cm": -6.0},
                {"x_cm": 4.0, "y_cm": -2.0},
                {"x_cm": 4.0, "y_cm": 2.0},
                {"x_cm": 4.0, "y_cm": 6.0},
            ],
        },
    }


#: The exact TC9 literal (player_html.py's Contracts -> Emitter): both P-TC5
#: and P-TC6 assert PLAYER_META equals this verbatim, proving the object/
#: cargo feature never perturbs it either with or without a layout.
_EXPECTED_TC9_META = {
    "source": "PREDICTED",
    "filename": "",
    "lanes": ["task", "goto", "holding", "drift"],
    "tabs": ["log", "inspector"],
    "render_mode": "box",
}


# ---------------------------------------------------------------------------
# TC5 — render a Track to one self-contained, offline HTML page
# ---------------------------------------------------------------------------


def test_tc5_render_track_is_self_contained(tmp_path):
    """TC5: render() emits one offline .html whose TRACK round-trips via json.loads.

    Key assertion: the embedded ``const TRACK`` JSON parses back, via
    :func:`json.loads`, to exactly the input track's :meth:`Track.to_dict`;
    the page carries the transport markers (``id="ctps-play"``, a
    ``type="range"`` ``id="ctps-time"`` slider/readout, a ``<canvas``); and it
    references no external local file (no ``src=``/``href=`` to a file) and no
    ``http``/``https`` resource.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        track = build(load(_FIXTURES / "ctps_two_goto.json"), toy_field_data)

    out_path = tmp_path / "player.html"
    returned = render(track, out_path)

    # Exactly one .html written, at the requested location.
    assert returned == out_path
    html_files = list(tmp_path.glob("*.html"))
    assert html_files == [out_path]

    html = out_path.read_text(encoding="utf-8")

    # The embedded track round-trips to the exact input dict.
    embedded = _extract_embedded_track(html)
    assert embedded == track.to_dict()

    # Transport-bar / canvas markers TC5 looks for.
    assert 'id="ctps-play"' in html
    assert "<input" in html
    assert 'type="range"' in html
    assert 'id="ctps-time"' in html
    assert "<canvas" in html

    # Self-contained: no external resource references.
    assert "http://" not in html
    assert "https://" not in html
    # No <script src=...> / <link href=...> to a local asset; everything inline.
    assert "src=" not in html
    assert "href=" not in html


# ---------------------------------------------------------------------------
# TC6 — render a hand-built LRS-shaped Track DICT unchanged (source-agnostic)
# ---------------------------------------------------------------------------


def test_tc6_render_plain_dict_round_trips(tmp_path):
    """TC6: a hand-built LRS-shaped Track DICT renders unchanged (AC7).

    Key assertion: passing a plain dict (poses + events only, no command
    stream, never through the builder) straight to :func:`render` embeds it
    such that the recovered JSON equals the input dict exactly — proving the
    player consumes Data Model B regardless of its source.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )

    out_path = tmp_path / "lrs.html"
    render(track_dict, out_path)

    html = out_path.read_text(encoding="utf-8")
    embedded = _extract_embedded_track(html)

    assert embedded == track_dict
    # The LRS-shaped track carries a drift_warn event that survives verbatim.
    assert {"t": 9.0, "type": "drift_warn",
            "label": "chassis 14.2 cm off node E"} in embedded["events"]


# ---------------------------------------------------------------------------
# TC12 — player.js seek path is a bounded binary search, not a full scan
# ---------------------------------------------------------------------------


def test_tc12_player_js_seek_is_bounded_binary_search():
    """TC12: player.js seeks via a bisect, with poseAt delegating to it (AC3a).

    Key assertion (text heuristic, robust to formatting): ``player.js`` defines
    ``findKeyframeIndex``, that function uses a bisect pattern (``Math.floor``
    midpoint with ``lo``/``hi`` bounds), ``poseAt`` references
    ``findKeyframeIndex`` rather than scanning, and the seek path contains no
    unbounded ``for`` loop over the full ``keyframes`` array.
    """
    js = PLAYER_JS_PATH.read_text(encoding="utf-8")

    # findKeyframeIndex is defined.
    assert re.search(r"function\s+findKeyframeIndex\s*\(", js)

    # Bisect pattern: Math.floor midpoint with lo / hi window bounds.
    assert "Math.floor" in js
    assert re.search(r"\blo\b", js)
    assert re.search(r"\bhi\b", js)
    # A shrinking window loop (hi - lo > 1) is the bisect's signature.
    assert re.search(r"hi\s*-\s*lo\s*>\s*1", js)

    # poseAt resolves its index THROUGH findKeyframeIndex (no inline scan).
    pose_at = re.search(r"function\s+poseAt\s*\([\s\S]*?\n    \}", js)
    assert pose_at is not None, "could not isolate the poseAt function body"
    assert "findKeyframeIndex" in pose_at.group(0)

    # No unbounded per-frame full keyframe scan in the seek path: poseAt must
    # not iterate the whole keyframes array with a for-loop.
    assert not re.search(r"for\s*\([^)]*keyframes\.length", pose_at.group(0))


# ---------------------------------------------------------------------------
# Map background — optional, self-contained data URI (or null fallback)
# ---------------------------------------------------------------------------


def test_map_png_embeds_self_contained_data_uri(tmp_path):
    """render(map_png=...) embeds the image as an offline ``data:`` URI.

    Key assertion: the page carries ``const MAP_IMAGE = "data:image/png;base64,
    <b64>"`` for the supplied bytes, stays self-contained (no http/https), and
    the embedded ``const TRACK`` still round-trips unchanged alongside the map.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    map_file = tmp_path / "field.png"
    raw = b"\x89PNG\r\n\x1a\nctps-test-map-bytes"
    map_file.write_bytes(raw)

    out_path = tmp_path / "with_map.html"
    render(track_dict, out_path, map_png=map_file)
    html = out_path.read_text(encoding="utf-8")

    expected_b64 = base64.b64encode(raw).decode("ascii")
    assert f'const MAP_IMAGE = "data:image/png;base64,{expected_b64}";' in html
    # Still offline / self-contained.
    assert "http://" not in html
    assert "https://" not in html
    # The track survives unchanged next to the embedded map.
    assert _extract_embedded_track(html) == track_dict


def test_no_map_png_emits_null(tmp_path):
    """Without ``map_png`` the player embeds ``MAP_IMAGE = null`` (vector fallback)."""
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "no_map.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert "const MAP_IMAGE = null;" in html
    assert "data:image" not in html


def test_missing_map_png_warns_and_falls_back(tmp_path):
    """A ``map_png`` that does not exist warns and degrades to ``MAP_IMAGE = null``."""
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "missing_map.html"
    with pytest.warns(UserWarning, match="map image not found"):
        render(track_dict, out_path, map_png=tmp_path / "does_not_exist.png")
    html = out_path.read_text(encoding="utf-8")

    assert "const MAP_IMAGE = null;" in html


# ---------------------------------------------------------------------------
# TC-META — PLAYER_META is embedded and round-trips like TRACK (AC-meta)
# ---------------------------------------------------------------------------


def test_tc_meta_embedded_and_round_trips(tmp_path):
    """TC-META: ``const PLAYER_META = {...};`` embeds and round-trips (AC-meta).

    Key assertion: PLAYER_META lives in its OWN ``<script>`` (never swallowed
    into the TRACK assignment — T2's Contracts -> Emitter), parses via
    :func:`json.loads` to a dict, and carries the source badge, trace
    filename, the lane manifest (``task``/``goto``/``holding``/``drift``, in
    dock order), the tab manifest (``log``/``inspector``), and the default
    render mode (``"box"``) — while TRACK still round-trips unchanged
    alongside it.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "meta.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    meta = _extract_player_meta(html)
    assert _extract_embedded_track(html) == track_dict

    assert isinstance(meta, dict)
    assert meta["source"] == "PREDICTED"
    assert isinstance(meta["filename"], str)
    assert meta["lanes"] == ["task", "goto", "holding", "drift"]
    assert meta["tabs"] == ["log", "inspector"]
    assert meta["render_mode"] == "box"


# ---------------------------------------------------------------------------
# TC8 / TC9 — render_mode selection: wheel-arc when pivots, box otherwise
# (issue #55 / T6, AC5 + AC6)
# ---------------------------------------------------------------------------


def test_tc8_render_mode_is_wheel_arc_when_pivots_present(tmp_path):
    """TC8: a track carrying a non-empty ``pivots`` list renders wheel-arc.

    Key assertion: a hand-built LRS-shaped track dict (source-agnostic, mirrors
    TC6) that carries a ``"pivots"`` list emits
    ``PLAYER_META.render_mode == "wheel-arc"``, and the embedded ``TRACK``
    carries that exact ``pivots`` list unchanged (AC5).
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    pivots = [
        {
            "t0": 3.0,
            "t1": 3.9,
            "cx": 69.0,
            "cy": 0.0,
            "theta_deg": -90.0,
            "wheel": "right",
        }
    ]
    track_dict["pivots"] = pivots

    out_path = tmp_path / "wheel_arc.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    meta = _extract_player_meta(html)
    embedded = _extract_embedded_track(html)

    assert meta["render_mode"] == "wheel-arc"
    assert embedded["pivots"] == pivots


def test_tc9_render_mode_is_box_and_meta_is_expected_literal_when_pivot_free(
    tmp_path,
):
    """TC9: a pivot-free track keeps render_mode "box" and an exact-literal meta.

    Key assertion: the embedded ``TRACK`` carries no ``"pivots"`` key,
    ``PLAYER_META.render_mode == "box"``, and ``PLAYER_META`` equals the
    expected literal dict below — structural equality, since a test cannot
    diff against pre-change code (AC6). (True byte-vs-today equivalence is
    verified once by hand per the plan's Notes.)
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    assert "pivots" not in track_dict  # fixture precondition: pivot-free

    out_path = tmp_path / "pivot_free.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    embedded = _extract_embedded_track(html)
    meta = _extract_player_meta(html)

    assert "pivots" not in embedded

    expected_meta = {
        "source": "PREDICTED",
        "filename": "",
        "lanes": ["task", "goto", "holding", "drift"],
        "tabs": ["log", "inspector"],
        "render_mode": "box",
    }
    assert meta == expected_meta


# ---------------------------------------------------------------------------
# TC-LINT-pos / TC-LINT-neg — the emit-time self-containment lint (AC-lint)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "offender",
    ["src=", "href=", "http://", "https://", "data:image", "</script>"],
)
def test_tc_lint_pos_assert_self_contained_rejects_each_forbidden_substring(offender):
    """TC-LINT-pos: ``_assert_self_contained`` raises naming the offender.

    Key assertion: EACH of the six banned substrings (an external/local
    resource attribute, a network URL, an inline image background, or a
    literal ``</script>`` that would break out of the inline wrapper) fails
    the lint on its own, and the raised :class:`ValueError` message names
    both the offending source and the offending substring — never a silent
    pass.
    """
    sources = {"evil_asset.js": f"(function () {{ var x = '{offender}'; }})();"}
    with pytest.raises(ValueError) as exc_info:
        _assert_self_contained(sources)
    message = str(exc_info.value)
    assert "evil_asset.js" in message
    assert offender in message


def test_tc_lint_pos_render_raises_on_forbidden_asset(tmp_path, monkeypatch):
    """TC-LINT-pos (integration): render() lints the REAL asset pipeline too.

    Points the emitter at one extra temp asset containing a banned substring
    by monkeypatching ``player_html._ASSET_JS`` (per T2's Contracts ->
    Emitter) and asserts :func:`render` raises :class:`ValueError` naming the
    offending asset — and never writes the output file.
    """
    bad_asset = tmp_path / "evil_asset.js"
    bad_asset.write_text(
        "(function () { var u = 'http://example.com/tracker.js'; })();",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        player_html_module, "_ASSET_JS", player_html_module._ASSET_JS + (bad_asset,)
    )

    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "should_not_exist.html"
    with pytest.raises(ValueError) as exc_info:
        render(track_dict, out_path)
    message = str(exc_info.value)
    assert bad_asset.name in message
    assert "http://" in message
    assert not out_path.exists()


def test_tc_lint_neg_clean_assets_with_map_do_not_raise(tmp_path):
    """TC-LINT-neg: a normal render() with a real map_png never raises.

    Key assertion: the lint bans ``data:image`` in ASSETS, but the embedded
    ``MAP_IMAGE`` ``data:image/png;base64,...`` VALUE is a JSON value, not an
    asset — it is explicitly exempt, so a normal render() with a map must not
    raise, and that exact value must appear in the output.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    map_file = tmp_path / "field.png"
    map_file.write_bytes(b"\x89PNG\r\n\x1a\nctps-lint-neg-map-bytes")

    out_path = tmp_path / "lint_neg.html"
    render(track_dict, out_path, map_png=map_file)  # must not raise

    html = out_path.read_text(encoding="utf-8")
    assert "data:image/png;base64," in html


# ---------------------------------------------------------------------------
# TC-STRUCT — design-3a structural markers present (AC-contract)
# ---------------------------------------------------------------------------


def test_tc_struct_container_classes_and_rail_ids_present(tmp_path):
    """TC-STRUCT: the five 3a container classes + the rail ids are in the HTML.

    Key assertion: the design-3a layout scaffolds — header, canvas stage,
    right rail, transport bar, and timeline dock — render as the five
    class-based containers pinned by the Contracts (CSS class inventory), and
    the Current-command / Log rail markers (``#ctps-command``, ``#ctps-log``)
    are present for T7's panels to hydrate onto.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "struct.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    for cls in (
        "ctps-header",
        "ctps-stage",
        "ctps-rail",
        "ctps-transport",
        "ctps-timeline",
    ):
        assert f'class="{cls}"' in html

    assert 'id="ctps-command"' in html
    assert 'id="ctps-log"' in html


# ---------------------------------------------------------------------------
# TC-ASSETS — every _ASSET_JS / _ASSET_CSS file exists and is inlined
# ---------------------------------------------------------------------------


def test_tc_assets_every_asset_file_exists_and_is_inlined(tmp_path):
    """TC-ASSETS: every registered asset exists on disk and is inlined verbatim.

    Key assertion: for each path in ``_ASSET_JS + _ASSET_CSS``, the file
    exists, and its exact on-disk content appears as a contiguous substring
    of the rendered page — the emitter neither drops nor mangles a sibling
    asset (each JS file goes into its own ``<script>``, the CSS files
    concatenate into one ``<style>``).
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "assets.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    for path in (*_ASSET_JS, *_ASSET_CSS):
        assert path.is_file(), f"missing runtime asset: {path}"
        content = path.read_text(encoding="utf-8")
        assert content in html, f"{path.name} was not inlined verbatim into the page"


def test_tc_assets_missing_asset_raises_file_not_found(tmp_path, monkeypatch):
    """TC-ASSETS: a missing sibling asset fails render() loud, naming its path."""
    missing = tmp_path / "does_not_exist.js"
    monkeypatch.setattr(player_html_module, "_ASSET_JS", (missing,))

    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "missing_asset.html"
    with pytest.raises(FileNotFoundError, match=re.escape(str(missing))):
        render(track_dict, out_path)


# ---------------------------------------------------------------------------
# TC-SEEKINPUT — seek is a native range input with the pinned id
# ---------------------------------------------------------------------------


def test_tc_seekinput_native_range_element_present(tmp_path):
    """TC-SEEKINPUT: the styled native range input carries ``id="ctps-seek"``.

    Key assertion: the seek control is a real ``<input type="range">`` (a11y
    and keyboard step behavior come from the browser, not a custom widget)
    with the exact contiguous marker T5/T13 hydrate/style onto.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "seekinput.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert '<input type="range" id="ctps-seek"' in html


# ---------------------------------------------------------------------------
# TC-TRAIL — the traveled-trail rebuild is bounded, not a full keyframe scan
# ---------------------------------------------------------------------------


def test_tc_trail_bounded_rebuild_not_full_scan():
    """TC-TRAIL: renderer.js's trail path resolves through findKeyframeIndex.

    Key assertion (text heuristic, robust to formatting): the trail layer
    references the pinned bisect ``findKeyframeIndex`` (never re-implements
    its own scan), and there is no unbounded ``for`` loop over the full
    keyframe array's length — checked against both the un-aliased name and
    the file's own ``keyframes`` -> ``kf`` alias.
    """
    js = (PLAYER_JS_PATH.parent / "renderer.js").read_text(encoding="utf-8")

    assert "findKeyframeIndex" in js
    assert not re.search(r"for\s*\([^)]*keyframes\.length", js)
    assert not re.search(r"for\s*\([^)]*kf\.length", js)


# ---------------------------------------------------------------------------
# TC-GOTOEND — the GOTO lane reads live success spans + UNREACHABLE pins
# ---------------------------------------------------------------------------


def test_tc_gotoend_timeline_lane_reads_spans():
    """TC-GOTOEND: timeline.js's goto lane discriminates on t_end / UNREACHABLE.

    Key assertion (text heuristic): a successful goto event (``t ->
    t_end``) renders as a span and an UNREACHABLE fault goto (no ``t_end``)
    renders as a pin — ``timeline.js`` must reference both discriminators.
    """
    js = (PLAYER_JS_PATH.parent / "timeline.js").read_text(encoding="utf-8")

    assert "t_end" in js
    assert "UNREACHABLE" in js


# ---------------------------------------------------------------------------
# TC-NOINLINE — no inline-style assignment in any JS asset (ADR-053 carve-out)
# ---------------------------------------------------------------------------

#: An assignment-shaped match ONLY — ``.style.setProperty(`` is the sanctioned
#: styling channel and must NEVER match this. A loose ``.style.`` substring
#: scan would false-positive on doc comments like ``.style.<prop>`` (the
#: literal ``<`` breaks ``[A-Za-z_]\w*``, so this regex correctly ignores
#: them).
_INLINE_STYLE_ASSIGNMENT_RE = re.compile(r"\.style\.[A-Za-z_]\w*\s*=")


def test_tc_noinline_no_inline_style_assignment_in_any_js_asset():
    """TC-NOINLINE: JS sets styling only via ``.style.setProperty()``, never ``=``.

    Key assertion: for every JS asset, an assignment-shaped ``.style.<prop>
    =`` never appears; ``.style.setProperty(...)`` (the ADR-053 carve-out —
    one custom property per animated axis) is unaffected since it is never
    followed by ``=``.
    """
    for path in _ASSET_JS:
        js = path.read_text(encoding="utf-8")
        matches = _INLINE_STYLE_ASSIGNMENT_RE.findall(js)
        assert matches == [], f"{path.name} sets an inline style directly: {matches}"


# ---------------------------------------------------------------------------
# TC-DUPREG — a duplicate registry id is non-fatal (records into _errors)
# ---------------------------------------------------------------------------


def test_tc_dupreg_register_never_throws_and_boot_surfaces_errors():
    """TC-DUPREG: register() records a dup id into _errors — never throws.

    Key assertion (text heuristic): player.js's registry ``register`` path
    has no top-level ``throw`` statement anywhere (a throwing ``register()``
    inside one module's top-level IIFE could abort that whole ``<script>``,
    defeating the "one bad module can't kill the page" contract) and does
    push a duplicate into ``_errors``; ``boot.js`` references ``_errors`` so
    the accumulated duplicates are actually surfaced (via ``console.error``).
    """
    player_js = PLAYER_JS_PATH.read_text(encoding="utf-8")
    boot_js = (PLAYER_JS_PATH.parent / "boot.js").read_text(encoding="utf-8")

    assert re.search(r"\bthrow\b", player_js) is None
    assert "_errors.push" in player_js
    assert "_errors" in boot_js


# ---------------------------------------------------------------------------
# P-TC5 — render(objects=None): no const OBJECTS; TRACK + PLAYER_META unchanged
# (object/cargo plan, T4 — issue #57, AC11)
# ---------------------------------------------------------------------------


def test_p_tc5_render_objects_none_emits_no_objects_and_meta_unchanged(tmp_path):
    """P-TC5: ``render(objects=None)`` emits no ``const OBJECTS`` at all (AC11).

    Key assertion: with no ``objects`` argument (the default), the page has
    no ``const OBJECTS`` assignment whatsoever — not even a ``null``
    placeholder — the embedded ``TRACK`` still round-trips unchanged, and
    ``PLAYER_META`` equals the exact TC9 literal verbatim, proving the new
    cargo markup/asset additions never perturb the pre-existing contract.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "no_objects.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert "const OBJECTS" not in html
    assert _extract_embedded_objects(html) is None
    assert _extract_embedded_track(html) == track_dict
    assert _extract_player_meta(html) == _EXPECTED_TC9_META


def test_p_tc5_render_objects_empty_dict_emits_no_objects(tmp_path):
    """P-TC5 (falsy edge case): ``objects={}`` is falsy — treated like ``None``."""
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "empty_objects.html"
    render(track_dict, out_path, objects={})
    html = out_path.read_text(encoding="utf-8")

    assert "const OBJECTS" not in html


# ---------------------------------------------------------------------------
# P-TC6 — render(objects=<layout>): const OBJECTS embeds; new assets inline
# (object/cargo plan, T4 — issue #57, AC5 embed leg)
# ---------------------------------------------------------------------------


def test_p_tc6_render_objects_embeds_and_inlines_cargo_assets_in_order(tmp_path):
    """P-TC6: ``render(objects=<layout>)`` embeds ``const OBJECTS`` + inlines assets.

    Key assertion: a truthy ``objects`` layout is validated then embedded
    verbatim as its own ``const OBJECTS`` script (round-trips through
    :func:`json.loads`); ``objects_draw.js`` then ``cargo.js`` are inlined —
    in that exact order, both before ``boot.js`` — and ``PLAYER_META`` is
    STILL the unchanged TC9 literal, alongside the still-unchanged ``TRACK``.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    layout_dict = _valid_layout_dict()

    out_path = tmp_path / "with_objects.html"
    render(track_dict, out_path, objects=layout_dict)
    html = out_path.read_text(encoding="utf-8")

    assert _extract_embedded_objects(html) == layout_dict
    assert _extract_embedded_track(html) == track_dict
    assert _extract_player_meta(html) == _EXPECTED_TC9_META

    objects_draw_js = (PLAYER_JS_PATH.parent / "objects_draw.js").read_text(encoding="utf-8")
    cargo_js = (PLAYER_JS_PATH.parent / "cargo.js").read_text(encoding="utf-8")
    boot_js = (PLAYER_JS_PATH.parent / "boot.js").read_text(encoding="utf-8")

    assert objects_draw_js in html
    assert cargo_js in html
    assert boot_js in html
    # Pinned load order: objects_draw.js -> cargo.js -> boot.js.
    assert html.index(objects_draw_js) < html.index(cargo_js) < html.index(boot_js)


def test_p_tc6_render_objects_invalid_layout_raises_layout_error(tmp_path):
    """P-TC6 (negative): an invalid ``objects`` layout raises, naming the field.

    Key assertion: :func:`render` validates a truthy ``objects`` via
    :mod:`tools.path_planner.ctps.layout` BEFORE writing anything — a
    malformed layout raises :class:`LayoutError` and the output file is never
    created (mirrors the field-data validator's LOUD-raise convention).
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    bad_layout = _valid_layout_dict()
    del bad_layout["objects"][0]["type"]

    out_path = tmp_path / "should_not_exist.html"
    with pytest.raises(LayoutError, match="objects\\[0\\]"):
        render(track_dict, out_path, objects=bad_layout)
    assert not out_path.exists()


# ---------------------------------------------------------------------------
# TC-CARGO-STRUCT — cargo panel + chips + upload markup present (AC5, DOM
# contract T6 queries; object/cargo plan, T4 — issue #57)
# ---------------------------------------------------------------------------


def test_tc_cargo_struct_panel_chips_and_upload_markup_present(tmp_path):
    """TC-CARGO-STRUCT: the cargo panel + OBJECTS/ZONES chips + upload input render.

    Key assertion: the two-tab cargo panel DOM contract T6 queries
    (``#ctps-cargo``, the ``.ctps-cargo-tab[data-cargo-tab]`` tabs, the count
    badges, the magazine grid, the inventory list) is present; the new
    ``OBJECTS``/``ZONES`` layer chips join the existing ``.ctps-layer-chips``
    row; the ``#ctps-layout-file`` upload input is present; the pre-existing
    ``#ctps-command``/``#ctps-log`` markers are unaffected; and the retired
    ``.ctps-panel--holding`` shell (and its ``#ctps-hold-count``/
    ``#ctps-hold-list`` ids) is gone.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "cargo_struct.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert 'id="ctps-cargo"' in html
    assert 'data-cargo-tab="magazine"' in html
    assert 'data-cargo-tab="inventory"' in html
    assert "ctps-cargo-tab" in html
    assert 'id="ctps-cargo-count-mag"' in html
    assert 'id="ctps-cargo-count-inv"' in html
    assert 'id="ctps-mag-grid"' in html
    assert 'id="ctps-inv-list"' in html

    assert 'data-layer="objects"' in html
    assert 'data-layer="zones"' in html

    assert '<input type="file" id="ctps-layout-file"' in html

    # Pre-existing rail markers unaffected (TC-STRUCT stays green alongside).
    assert 'id="ctps-command"' in html
    assert 'id="ctps-log"' in html

    # The retired Holding shell is fully gone.
    assert "ctps-panel--holding" not in html
    assert 'id="ctps-hold-count"' not in html
    assert 'id="ctps-hold-list"' not in html
