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
* **P-TC10** — an end-to-end synthetic Data-Model-B cargo trace
  (``ctps_cargo_trace.json``) rendered together with a real layout
  (``ctps_cargo_layout.json``) round-trips both ``const TRACK`` and
  ``const OBJECTS``, keeps the structured ``method``/``arg`` keys on the
  fixture's grab/sortTo/dump/boxClose/boxOpen manip sequence, and never trips
  the self-containment lint.
* **P-TC13** — a track carrying both a non-empty ``pivots`` list and a truthy
  ``objects`` layout renders both intact on the shared script-emit path,
  ``PLAYER_META.render_mode`` stays driven by ``pivots`` alone
  (``"wheel-arc"``), and the lint doesn't trip — pivots and objects don't
  interfere.

Also covers the CTPS timeline zoom + pan plan's emitter leg (T6):

* **TC1** — the new timeline dock markup (toolbar, scroll viewport/content,
  custom scrollbar thumb, zoom widget) renders with the exact ids/classes
  pinned by the DOM contract, alongside the still-present
  ``class="ctps-timeline"`` dock (AC14).
* **TC3** — ``#ctps-ruler``, ``#ctps-lanes``, ``#ctps-playhead`` are
  preserved (moved inside the new scroll wrapper, never removed).
* **TC5** — the contract markers ``--ctps-content-w``/``--ctps-tl-thumb-w``
  (in both ``components.css`` and ``timeline_view.js``),
  ``CTPS.timelineView`` (in ``timeline_view.js``), and ``rebuildRuler`` (in
  both ``timeline.js`` and ``timeline_view.js``) are pinned by reading the
  on-disk asset sources directly.
"""

from __future__ import annotations

import base64
import json
import re
import warnings
from pathlib import Path

import pytest

from tools.path_planner.ctps import mosaic as mosaic_module
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


#: Captures the JSON document assigned to ``const MOSAIC_META`` in the emitted
#: page (randomized-vision plan, T10/T21 -- ADR-060). Own ``<script>`` (same
#: non-greedy-to-``</script>`` shape as the sibling regexes above); only
#: emitted alongside a renderable mosaic, so a mosaic-free render has no match
#: (TC18/AC12).
_MOSAIC_META_RE = re.compile(r"const MOSAIC_META = (?P<json>.*?);\s*</script>", re.DOTALL)

#: Captures the JSON document assigned to ``const MOSAIC`` in the emitted page.
#: Deliberately does NOT match ``MOSAIC_META`` (the literal ``const MOSAIC =``
#: — note the trailing space before ``=`` — never matches
#: ``const MOSAIC_META =``, whose ``_META`` sits directly against ``MOSAIC``
#: with no space).
_MOSAIC_RE = re.compile(r"const MOSAIC = (?P<json>.*?);\s*</script>", re.DOTALL)


def _extract_mosaic_meta(html: str) -> "dict | None":
    """Pull the embedded ``const MOSAIC_META = {...}`` dict out of the page.

    Mirrors :func:`_extract_embedded_objects`: returns ``None`` (rather than
    asserting) when absent -- the expected shape for a mosaic-free render
    (TC18/AC12).
    """
    match = _MOSAIC_META_RE.search(html)
    if match is None:
        return None
    raw_json = match.group("json").replace("<\\/", "</")
    return json.loads(raw_json)


def _extract_mosaic(html: str) -> "dict | None":
    """Pull the embedded ``const MOSAIC = {...}`` dict out of the page.

    Mirrors :func:`_extract_embedded_objects`: returns ``None`` (rather than
    asserting) when absent -- the expected shape for a mosaic-free render
    (TC18/AC12).
    """
    match = _MOSAIC_RE.search(html)
    if match is None:
        return None
    raw_json = match.group("json").replace("<\\/", "</")
    return json.loads(raw_json)


#: A fixed, pinned 12-cell mosaic roll (directed-mode order) used across the
#: TC17/TC19 fixtures below so the resolved world positions + colours are
#: stable and hand-verifiable.
_PINNED_MOSAIC_CELLS = [
    "YELLOW", "BLUE", "GREEN", "WHITE",
    "YELLOW", "BLUE", "GREEN", "WHITE",
    "YELLOW", "BLUE", "GREEN", "WHITE",
]


def _mosaic_geometry_block() -> dict:
    """Return a fresh, schema-valid ``mosaic`` GEOMETRY block (directed mode).

    Directed mode with every cell pinned to :data:`_PINNED_MOSAIC_CELLS` --
    matches the trace roll fixture below cell-for-cell, so TC17's "geometry x
    trace colours" resolve is trivially self-consistent (and TC19's grid-
    outline case reuses the identical geometry with no trace roll).
    """
    return {
        "mode": "directed",
        "center_x_cm": 50.0,
        "center_y_cm": 30.0,
        "heading_deg": 0.0,
        "rows": 4,
        "cols": 3,
        "cell_w_cm": 5.0,
        "cell_h_cm": 5.0,
        "row_gap_cm": 1.0,
        "col_gap_cm": 1.0,
        "palette": ["YELLOW", "BLUE", "GREEN", "WHITE"],
        "cells": list(_PINNED_MOSAIC_CELLS),
        "counts": {},
        "seed": None,
    }


def _layout_with_mosaic_and_vision() -> dict:
    """Return a fresh, schema-valid layout dict carrying a mosaic + vision cone.

    Built on :func:`_valid_layout_dict` (same field-objects/robot shape every
    other player test uses) plus the top-level ``mosaic`` geometry
    (:func:`_mosaic_geometry_block`) and a ``robot.vision`` forward cone --
    the two ADR-060 blocks :func:`render` reads to resolve/embed
    ``MOSAIC_META`` + ``const MOSAIC`` and the cone.
    """
    layout = _valid_layout_dict()
    layout["mosaic"] = _mosaic_geometry_block()
    layout["robot"]["vision"] = {
        "origin_x_cm": 12.0,
        "origin_y_cm": 0.0,
        "half_angle_deg": 25.0,
        "range_cm": 60.0,
    }
    return layout


def _seeded_mosaic_trace_dict() -> dict:
    """Return a hand-built LRS-shaped track dict carrying a pinned mosaic roll.

    Mirrors TC6's source-agnostic pattern (poses + events only, no command
    stream) but adds the trace-carried top-level ``mosaic`` echo block (Data
    Model -- "Trace additions"): a fixed seed, the SAME 12 pinned colours as
    :func:`_mosaic_geometry_block`'s cells (so the resolved world cells carry
    exactly those colours), and a ``geom_hash`` computed from the SAME
    geometry so ``geom_match`` comes back ``True``.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    geom = mosaic_module.MosaicConfig.from_dict(_mosaic_geometry_block())
    track_dict["mosaic"] = {
        "cells": list(_PINNED_MOSAIC_CELLS),
        "seed": 12345,
        "geom_hash": mosaic_module.geom_hash(geom),
        "injected": True,
    }
    return track_dict


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
#: NOTE (issue #130): the "drift" entry is the lane ID, not its display
#: label — the dock label was relabelled to "LOCALIZATION" in timeline.js,
#: but the id (and therefore this manifest) is unchanged by design, so the
#: literal stays as-is.
_EXPECTED_TC9_META = {
    "source": "PREDICTED",
    "filename": "",
    "lanes": ["task", "goto", "magazine", "rearbox", "drift"],
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
    filename, the lane manifest
    (``task``/``goto``/``magazine``/``rearbox``/``drift``, in dock order —
    the ``drift`` entry is the lane ID; its dock LABEL is "LOCALIZATION" as
    of issue #130, but the id/manifest is unchanged by design), the
    tab manifest (``log``/``inspector``), and the default
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
    assert meta["lanes"] == ["task", "goto", "magazine", "rearbox", "drift"]
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

    # "drift" is the lane ID (unchanged, issue #130); its dock LABEL is
    # relabelled to "LOCALIZATION" in timeline.js — see test_tl_localization_*.
    expected_meta = {
        "source": "PREDICTED",
        "filename": "",
        "lanes": ["task", "goto", "magazine", "rearbox", "drift"],
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
# TC1 / TC3 / TC5 (CTPS Timeline Zoom + Pan plan) — new dock markup, preserved
# ids, and asset contract markers
# ---------------------------------------------------------------------------


def test_tl_tc1_zoom_pan_timeline_markup_present(tmp_path):
    """TC1: the timeline zoom/pan restructuring's new markup renders.

    Key assertion: the zoom toolbar, the scroll viewport + content wrapper,
    the custom scrollbar thumb, and the zoom widget (buttons/slider/readout)
    are all present as the DOM contract's exact ids/classes, and the dock
    itself is still the pinned ``class="ctps-timeline"`` container.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "tl_markup.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    for marker in (
        'id="ctps-tl-viewport"',
        'id="ctps-tl-content"',
        'id="ctps-tl-thumb"',
        'id="ctps-zoom"',
        'id="ctps-zoom-in"',
        'id="ctps-zoom-out"',
        'id="ctps-zoom-fit"',
        'id="ctps-zoom-level"',
        'class="ctps-tl-toolbar"',
        'class="ctps-tl-scroll"',
        'class="ctps-timeline"',
    ):
        assert marker in html, f"missing timeline dock marker: {marker}"


def test_tl_tc3_preserved_ruler_lanes_playhead_ids_present(tmp_path):
    """TC3: the pre-existing ruler/lanes/playhead ids are preserved, not removed.

    Key assertion: ``#ctps-ruler``, ``#ctps-lanes``, ``#ctps-playhead`` still
    appear verbatim in the rendered page — the restructuring moves them
    inside the new ``#ctps-tl-content`` scroll wrapper but keeps their ids so
    ``timeline.js`` still finds and fills them.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "tl_preserved_ids.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    assert 'id="ctps-ruler"' in html
    assert 'id="ctps-lanes"' in html
    assert 'id="ctps-playhead"' in html


def test_tl_tc5_contract_markers_present_in_assets():
    """TC5: the zoom/pan contract markers are pinned at their asset SOURCES.

    Key assertion: reads the on-disk asset files directly (via
    :data:`player_html_module._ASSETS_DIR`), not the inlined page, so the
    test pins the contract at its origin rather than merely re-checking the
    inlining covered by TC-ASSETS. ``--ctps-content-w`` and
    ``--ctps-tl-thumb-w`` each appear in both ``components.css`` and
    ``timeline_view.js``; ``CTPS.timelineView`` is assigned in
    ``timeline_view.js``; and ``rebuildRuler`` is defined in ``timeline.js``
    and referenced from ``timeline_view.js`` (the T3/T4 seam in the
    Contracts section).
    """
    components_css = (player_html_module._ASSETS_DIR / "components.css").read_text(
        encoding="utf-8"
    )
    timeline_view_js = (
        player_html_module._ASSETS_DIR / "timeline_view.js"
    ).read_text(encoding="utf-8")
    timeline_js = (player_html_module._ASSETS_DIR / "timeline.js").read_text(
        encoding="utf-8"
    )

    assert "--ctps-content-w" in components_css
    assert "--ctps-content-w" in timeline_view_js
    assert "--ctps-tl-thumb-w" in components_css
    assert "--ctps-tl-thumb-w" in timeline_view_js
    assert "CTPS.timelineView" in timeline_view_js
    assert "rebuildRuler" in timeline_js
    assert "rebuildRuler" in timeline_view_js


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
# TC-LOCALIZATION — drift lane relabelled to LOCALIZATION; renders all three
# localization-lane event types (issue #130, Contracts "Localization-lane
# event types" seam: track_builder/T11 emits, timeline.js/T12 renders)
# ---------------------------------------------------------------------------


def test_tc_localization_lane_id_kept_drift_label_relabelled():
    """TC-LOCALIZATION: the lane id stays `drift`; only the dock label changes.

    Key assertion (text heuristic): ``timeline.js`` still registers the lane
    under ``id: "drift"`` (PLAYER_META's manifest and the DOM class
    ``ctps-lane--drift``/``ctps-lane-label--drift`` depend on the id staying
    put) but its ``label`` is now the literal ``"LOCALIZATION"`` — matching
    the existing dock's uppercase casing (TASK/GOTO/MAGAZINE/REAR BOX).
    """
    js = (PLAYER_JS_PATH.parent / "timeline.js").read_text(encoding="utf-8")

    assert re.search(r'id:\s*"drift"', js)
    assert re.search(r'label:\s*"LOCALIZATION"', js)
    # The old "DRIFT" dock label is gone (not just co-present).
    assert not re.search(r'label:\s*"DRIFT"', js)


def test_tc_localization_lane_builder_handles_all_three_event_types():
    """TC-LOCALIZATION: buildDriftLane renders drift_warn + the two new types.

    Key assertion (text heuristic): ``timeline.js``'s lane builder still
    branches on ``drift_warn`` (unchanged rendering — a pin + a span) and now
    also branches on ``return_to_node`` and ``declare_position``, each
    emitting a distinctly-classed pin (``ctps-pin--return`` /
    ``ctps-pin--declare``) per the Contracts event-type seam T11 emits into.
    """
    js = (PLAYER_JS_PATH.parent / "timeline.js").read_text(encoding="utf-8")

    assert '"drift_warn"' in js
    assert '"return_to_node"' in js
    assert '"declare_position"' in js
    assert "ctps-pin--drift" in js
    assert "ctps-pin--return" in js
    assert "ctps-pin--declare" in js


def test_tc_localization_lane_marker_classes_styled_in_css():
    """TC-LOCALIZATION: the two new marker classes get their own CSS look.

    Key assertion: ``components.css`` defines a rule for each of
    ``ctps-pin--return`` and ``ctps-pin--declare`` (distinct visual classes,
    alongside the pre-existing ``ctps-pin--drift``) so the three localization
    marker types are visually distinguishable on the timeline.
    """
    css = (PLAYER_JS_PATH.parent / "components.css").read_text(encoding="utf-8")

    assert ".ctps-pin--drift" in css
    assert ".ctps-pin--return" in css
    assert ".ctps-pin--declare" in css


def test_tc_localization_lane_renders_return_and_declare_markers(tmp_path):
    """TC-LOCALIZATION (integration): a track with the new events renders both.

    Key assertion: a hand-built LRS-shaped track dict (source-agnostic, mirrors
    TC6) carrying one ``return_to_node`` and one ``declare_position`` event
    round-trips those events unchanged through the embedded ``const TRACK``,
    and the page still inlines ``timeline.js`` with both new marker classes —
    proving the render path never drops or chokes on the new event types.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    track_dict["events"] = list(track_dict["events"]) + [
        {"t": 5.0, "type": "return_to_node", "label": "return to D"},
        {"t": 6.5, "type": "declare_position", "label": "declared at E"},
    ]

    out_path = tmp_path / "localization_events.html"
    render(track_dict, out_path)  # must not raise
    html = out_path.read_text(encoding="utf-8")

    embedded = _extract_embedded_track(html)
    assert {"t": 5.0, "type": "return_to_node", "label": "return to D"} in embedded["events"]
    assert {"t": 6.5, "type": "declare_position", "label": "declared at E"} in embedded["events"]

    assert "ctps-pin--return" in html
    assert "ctps-pin--declare" in html


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


def test_p_tc6_cargo_grab_nearest_metric_is_robot_body():
    """P-TC6 addendum: grab picks nearest-to-robot-body in-zone tile candidates.

    The resolver's in-zone filter stays unchanged, but the tie-break metric for
    ``grab`` must be distance to the chassis footprint (robot body), not
    distance to the zone centre. This guards the direct call-site contract in
    ``cargo.js`` so a future refactor cannot silently drift back.
    """
    cargo_js = (PLAYER_JS_PATH.parent / "cargo.js").read_text(encoding="utf-8")
    assert "_distSqToRobotBody" in cargo_js
    assert "_robotHalfSideCm()" in cargo_js
    assert "_nearestInZone(" in cargo_js


# ---------------------------------------------------------------------------
# P-TC10 — end-to-end synthetic trace + layout render together (object/cargo
# plan Testing Strategy, issue #57)
# ---------------------------------------------------------------------------


def test_ptc10_synthetic_cargo_trace_and_layout_render_end_to_end(tmp_path):
    """P-TC10: a Data-Model-B cargo trace + a layout render together, cleanly.

    Key assertion: ``ctps_cargo_trace.json`` (keyframes + events, no command
    stream) goes straight into :func:`render` alongside
    ``ctps_cargo_layout.json`` — no trace_loader/track_builder involved, the
    same source-agnostic path as TC6. The embedded ``const OBJECTS``
    round-trips to the layout dict; the embedded ``const TRACK`` round-trips
    to the trace dict and its manip events keep the structured
    ``method``/``arg`` keys across the fixture's grab/sortTo/dump/boxClose/
    boxOpen sequence; and the render never trips the self-containment lint
    (no :class:`ValueError`).
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_cargo_trace.json").read_text(encoding="utf-8")
    )
    layout_dict = json.loads(
        (_FIXTURES / "ctps_cargo_layout.json").read_text(encoding="utf-8")
    )

    out_path = tmp_path / "cargo_e2e.html"
    returned = render(track_dict, out_path, objects=layout_dict)  # must not raise
    assert returned == out_path

    html = out_path.read_text(encoding="utf-8")

    # (a) const OBJECTS is present and round-trips to the layout dict.
    embedded_objects = _extract_embedded_objects(html)
    assert embedded_objects is not None
    assert embedded_objects == layout_dict

    # (b) the embedded TRACK round-trips, manip events keep method/arg.
    embedded_track = _extract_embedded_track(html)
    assert embedded_track == track_dict

    manip_events = [e for e in embedded_track["events"] if e["type"] == "manip"]
    manip_calls = [(e["method"], e["arg"]) for e in manip_events]
    assert manip_calls == [
        ("sortTo", 0),
        ("grab", 0),
        ("sortTo", 1),
        ("grab", 0),
        ("sortTo", 2),
        ("grab", 0),
        ("sortTo", 3),
        ("grab", 0),
        ("dump", 0),
        ("boxClose", 0),
        ("boxOpen", 0),
    ]

    # (c) the render passed the emitter's self-containment lint (no raise
    # above) and the page stays offline.
    assert "http://" not in html
    assert "https://" not in html


# ---------------------------------------------------------------------------
# P-TC13 — pivots + objects co-present render without interfering (object/
# cargo plan Testing Strategy, issue #57)
# ---------------------------------------------------------------------------


def test_ptc13_pivots_and_objects_co_present_render_cleanly(tmp_path):
    """P-TC13: a track with pivots AND an objects layout render together.

    Key assertion: a non-empty ``pivots`` list and a truthy ``objects``
    layout share the same script-emit path without interfering — both
    ``const OBJECTS`` and the ``pivots`` list embed intact, the page renders
    without tripping the self-containment lint, and (belt-and-suspenders)
    ``PLAYER_META.render_mode`` is still driven by ``pivots`` alone
    (``"wheel-arc"``), unaffected by the objects layout riding alongside it.
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
    layout_dict = _valid_layout_dict()

    out_path = tmp_path / "pivots_and_objects.html"
    render(track_dict, out_path, objects=layout_dict)  # must not raise
    html = out_path.read_text(encoding="utf-8")

    # const OBJECTS is present and round-trips.
    assert _extract_embedded_objects(html) == layout_dict

    # The pivots list survives in TRACK unchanged.
    embedded_track = _extract_embedded_track(html)
    assert embedded_track["pivots"] == pivots

    # render_mode still reflects pivots alone — objects never interfere.
    meta = _extract_player_meta(html)
    assert meta["render_mode"] == "wheel-arc"

    # Self-contained: the lint didn't trip and no network resource crept in.
    assert "http://" not in html
    assert "https://" not in html


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


# ---------------------------------------------------------------------------
# DEPLOY chip + layer markup/JS-presence assertions (manip-area deploy toggle
# plan, T5 — AC1, AC2, AC6, AC7)
# ---------------------------------------------------------------------------


def test_deploy_chip_and_layer_present(tmp_path):
    """The DEPLOY chip, store default, and inlined draw/layer JS are present.

    Covers TC1-TC3 of the manip-area deploy toggle plan: the chip markup
    (``data-layer="deploy"``, ``>DEPLOY<``, ``id="ctps-layer-deploy"``) is
    default-OFF (class exactly ``ctps-chip``, never ``ctps-chip--active``);
    ``store.layers`` defaults ``deploy: false``; and the inlined JS carries
    both the masked draw function (``drawDeployedZones``) and the ``deploy``
    layer registration (``id: "deploy"``).
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "deploy_chip.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    # Chip markup present.
    assert 'data-layer="deploy"' in html
    assert ">DEPLOY<" in html
    assert 'id="ctps-layer-deploy"' in html

    # Chip is default-OFF: class exactly "ctps-chip", never "--active".
    assert 'id="ctps-layer-deploy" class="ctps-chip"' in html
    assert 'id="ctps-layer-deploy" class="ctps-chip ctps-chip--active"' not in html

    # Store default.
    assert "deploy: false" in html

    # Inlined JS: the masked draw fn and the layer registration.
    assert "drawDeployedZones" in html
    assert 'id: "deploy"' in html


def test_cone_chip_defaults_off(tmp_path):
    """The CONE chip opens OFF: bare ``ctps-chip`` markup + ``vision-cone: false``.

    The page ships the CONE chip and the ``vision-cone`` layer unconditionally
    (ADR-056), but the cone must not be drawn until the operator toggles it on:
    the chip markup carries exactly ``ctps-chip`` (never ``ctps-chip--active``)
    and ``store.layers`` defaults ``"vision-cone": false``. MOSAIC stays ON.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "cone_chip.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    # Chip markup present but default-OFF: class exactly "ctps-chip".
    assert 'id="ctps-layer-vision-cone" class="ctps-chip"' in html
    assert (
        'id="ctps-layer-vision-cone" class="ctps-chip ctps-chip--active"'
        not in html
    )

    # Store default: cone hidden, mosaic still shown.
    assert '"vision-cone": false' in html
    assert "mosaic: true" in html


# ---------------------------------------------------------------------------
# TC17 — player, seeded mosaic + cone: MOSAIC_META/MOSAIC/#ctps-vision regions
# (randomized-vision plan, T21 -- ADR-060, AC10/AC12)
# ---------------------------------------------------------------------------

#: The pinned :func:`resolve_cells` output for :func:`_mosaic_geometry_block`'s
#: geometry x :data:`_PINNED_MOSAIC_CELLS` colours (hand-derived the same way
#: as test_ctps_mosaic.py's TC5 -- center=(50,30), heading=0, rows=4/cols=3,
#: cell=5x5cm, gaps=1/1cm; row 0 at +y "top", col 0 at -x "left").
_TC17_EXPECTED_CELLS = [
    {"index": 0, "x_cm": 44.0, "y_cm": 39.0, "color": "YELLOW"},
    {"index": 1, "x_cm": 50.0, "y_cm": 39.0, "color": "BLUE"},
    {"index": 2, "x_cm": 56.0, "y_cm": 39.0, "color": "GREEN"},
    {"index": 3, "x_cm": 44.0, "y_cm": 33.0, "color": "WHITE"},
    {"index": 4, "x_cm": 50.0, "y_cm": 33.0, "color": "YELLOW"},
    {"index": 5, "x_cm": 56.0, "y_cm": 33.0, "color": "BLUE"},
    {"index": 6, "x_cm": 44.0, "y_cm": 27.0, "color": "GREEN"},
    {"index": 7, "x_cm": 50.0, "y_cm": 27.0, "color": "WHITE"},
    {"index": 8, "x_cm": 56.0, "y_cm": 27.0, "color": "YELLOW"},
    {"index": 9, "x_cm": 44.0, "y_cm": 21.0, "color": "BLUE"},
    {"index": 10, "x_cm": 50.0, "y_cm": 21.0, "color": "GREEN"},
    {"index": 11, "x_cm": 56.0, "y_cm": 21.0, "color": "WHITE"},
]


def test_tc17_player_seeded_mosaic_and_cone_regions_are_present_and_stable(tmp_path):
    """TC17: a seeded mosaic + cone render stable MOSAIC_META/MOSAIC/#ctps-vision.

    Key assertion: rendering a layout carrying a mosaic GEOMETRY block plus a
    trace carrying a pinned roll (fixed seed, fixed 12-cell colours) resolves
    ``const MOSAIC`` to the exact HAND-DERIVED world cells (positions +
    colours, matching test_ctps_mosaic.py's TC5 derivation style),
    ``geom_match`` true (the trace's geom_hash matches the layout's current
    geometry), ``roll`` true, and the pinned ``seed``; ``MOSAIC_META`` carries
    the single-sourced colour vocabulary; and the unconditional
    ``#ctps-vision`` panel + its layer chips render alongside.
    """
    track_dict = _seeded_mosaic_trace_dict()
    layout_dict = _layout_with_mosaic_and_vision()

    out_path = tmp_path / "mosaic_seeded.html"
    render(track_dict, out_path, objects=layout_dict)
    html = out_path.read_text(encoding="utf-8")

    # MOSAIC_META -- the single-sourced colour vocabulary (names + display hex).
    meta = _extract_mosaic_meta(html)
    assert meta is not None
    assert meta["names"] == list(mosaic_module.COLOR_NAMES)
    assert meta["hex"] == {
        name: mosaic_module.COLOR_HEX[name] for name in mosaic_module.COLOR_NAMES
    }

    # const MOSAIC -- 12 resolved cells (position + colour), geom_match, roll,
    # and the pinned seed, all stable across a re-render (seeded fixture).
    mosaic_data = _extract_mosaic(html)
    assert mosaic_data is not None
    assert len(mosaic_data["cells"]) == 12
    assert mosaic_data["cells"] == _TC17_EXPECTED_CELLS
    assert mosaic_data["mode"] == "directed"
    assert mosaic_data["geom_match"] is True
    assert mosaic_data["roll"] is True
    assert mosaic_data["seed"] == 12345

    # #ctps-vision panel + its layer chips (unconditional markup, ADR-056).
    assert 'id="ctps-vision"' in html
    assert 'data-layer="vision-cone"' in html
    assert 'data-layer="mosaic"' in html

    # Re-rendering the SAME fixture is stable (a seeded golden, not a re-roll).
    out_path_2 = tmp_path / "mosaic_seeded_2.html"
    render(track_dict, out_path_2, objects=layout_dict)
    html_2 = out_path_2.read_text(encoding="utf-8")
    assert _extract_mosaic(html_2) == mosaic_data
    assert _extract_mosaic_meta(html_2) == meta


# ---------------------------------------------------------------------------
# TC18 — player, mosaic-free (the AC12 guard): no MOSAIC_META/MOSAIC, TC9
# PLAYER_META literal unchanged, existing suite green (randomized-vision plan,
# T21 -- AC12)
# ---------------------------------------------------------------------------


def test_tc18_player_mosaic_free_emits_no_mosaic_data_and_meta_is_unchanged(tmp_path):
    """TC18: a mosaic-free render emits no MOSAIC_META/MOSAIC; TC9 meta intact.

    Key assertion (the AC12 guard): with no ``mosaic`` block anywhere (no
    layout at all -- the plain TC9 fixture), the page has neither
    ``const MOSAIC_META`` nor ``const MOSAIC``, and ``PLAYER_META`` equals the
    exact frozen TC9 literal verbatim -- proving the new unconditional
    vision.js/#ctps-vision additions never perturb the pre-existing contract.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    assert "pivots" not in track_dict  # fixture precondition: pivot-free
    assert "mosaic" not in track_dict  # fixture precondition: roll-free

    out_path = tmp_path / "mosaic_free.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    # No `const MOSAIC_META =` / `const MOSAIC =` ASSIGNMENT anywhere -- note
    # this deliberately does not assert the bare substring "MOSAIC_META" is
    # absent, since vision.js's own doc comments reference the global by name
    # (the asset itself ships unconditionally, ADR-056); only the DATA is
    # conditional.
    assert "const MOSAIC_META =" not in html
    assert "const MOSAIC =" not in html
    assert _extract_mosaic_meta(html) is None
    assert _extract_mosaic(html) is None

    meta = _extract_player_meta(html)
    assert meta == _EXPECTED_TC9_META


def test_tc18_player_mosaic_free_with_layout_but_no_mosaic_block_emits_no_data(tmp_path):
    """TC18 (companion): a layout WITHOUT a mosaic block still emits no MOSAIC data.

    Key assertion: ``objects`` truthy (the ordinary cargo layout fixture, no
    ``mosaic`` key) still triggers no ``MOSAIC_META``/``MOSAIC`` -- the
    resolve is gated on the layout's OWN ``mosaic`` geometry, never on
    ``objects`` being present at all.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    layout_dict = _valid_layout_dict()
    assert "mosaic" not in layout_dict

    out_path = tmp_path / "mosaic_free_with_layout.html"
    render(track_dict, out_path, objects=layout_dict)
    html = out_path.read_text(encoding="utf-8")

    assert _extract_mosaic_meta(html) is None
    assert _extract_mosaic(html) is None
    # The layout itself still embeds fine (the cargo path is unaffected).
    assert _extract_embedded_objects(html) == layout_dict


def test_tc18_vision_js_is_inlined_in_the_frozen_asset_pipeline(tmp_path):
    """TC18: the generic asset-pipeline test naturally covers the new vision.js.

    Key assertion (belt-and-suspenders on top of TC-ASSETS): ``vision.js`` is
    a member of :data:`_ASSET_JS` (the unconditional-asset rule, ADR-056) and
    its on-disk content is inlined verbatim into a MOSAIC-FREE render, proving
    the asset ships even with no mosaic renderable.
    """
    vision_js_paths = [p for p in _ASSET_JS if p.name == "vision.js"]
    assert len(vision_js_paths) == 1, "vision.js must be registered exactly once in _ASSET_JS"

    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    out_path = tmp_path / "vision_asset.html"
    render(track_dict, out_path)
    html = out_path.read_text(encoding="utf-8")

    vision_js = vision_js_paths[0].read_text(encoding="utf-8")
    assert vision_js in html


# ---------------------------------------------------------------------------
# TC19 — player, mosaic geometry present but trace has NO roll: grid-outline
# path (randomized-vision plan, T21 -- ADR-060, AC10)
# ---------------------------------------------------------------------------


def test_tc19_player_geometry_present_no_roll_renders_grid_outline(tmp_path):
    """TC19: layout mosaic geometry + a roll-free trace renders the outline path.

    Key assertion: with a layout carrying mosaic GEOMETRY but a track dict
    with no top-level ``mosaic`` echo at all (an old trace, or a dump run
    without ``--mosaic``), ``const MOSAIC`` still resolves (positions only, no
    synthesized colour), ``roll`` is ``False``, ``geom_match`` is ``True`` (the
    geometry IS the current layout), no ``seed`` key is carried, and the
    ``#ctps-vision-badges`` slot markup is present for the client-rendered
    "no roll recorded" badge (the badge TEXT itself is drawn by vision.js at
    runtime, not baked into the static page -- see vision.js's
    ``_renderBadges``).
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    assert "mosaic" not in track_dict  # no roll recorded
    layout_dict = _layout_with_mosaic_and_vision()

    out_path = tmp_path / "mosaic_no_roll.html"
    render(track_dict, out_path, objects=layout_dict)
    html = out_path.read_text(encoding="utf-8")

    mosaic_data = _extract_mosaic(html)
    assert mosaic_data is not None
    assert len(mosaic_data["cells"]) == 12
    # Positions only -- no synthesized "color" key on any cell.
    for cell in mosaic_data["cells"]:
        assert "color" not in cell
        assert isinstance(cell["x_cm"], float)
        assert isinstance(cell["y_cm"], float)
    assert mosaic_data["roll"] is False
    assert mosaic_data["geom_match"] is True
    assert "seed" not in mosaic_data

    # MOSAIC_META still ships (the colour vocabulary is present whenever a
    # mosaic is renderable at all, roll or not).
    assert _extract_mosaic_meta(html) is not None

    # The badge SLOT markup (client-rendered text) is present in the static
    # page even though the "no roll recorded" text itself is JS-driven.
    assert 'id="ctps-vision-badges"' in html
    assert '"no roll recorded"' in (PLAYER_JS_PATH.parent / "vision.js").read_text(
        encoding="utf-8"
    )

    # Box overlay is EVENT-driven (boxClose shows, boxOpen clears), NOT gated on
    # the cargo model's captured-cargo `scene.rearBox` — so it reflects the rear
    # box on any trace regardless of whether a Cargo object lay in the box zone.
    assert "_tInAnyBoxSpan" in html
    assert "windows.box" in html
    assert "box: _tInAnyBoxSpan(t, windows.box)" in html


# ---------------------------------------------------------------------------
# Axle->body offset plan (T8) — axle_offset embed + JS draw/resolution pins
# (issue #165, AC1-AC4, AC8)
# ---------------------------------------------------------------------------


def test_axle_tc6_objects_script_embeds_axle_offset_verbatim(tmp_path):
    """TC6: a layout carrying ``robot.axle_offset`` embeds it verbatim (AC8).

    Key assertion: mirrors P-TC6's embed-and-round-trip pattern — a layout
    built on the shared :func:`_valid_layout_dict` fixture plus a
    ``robot.axle_offset = {forward_cm, lateral_cm}`` block renders with the
    offset present, byte-for-byte, inside the emitted ``const OBJECTS``
    assignment (both keys, unchanged values) — proving the T1 schema change
    reaches the page with zero ``player_html.py`` change, as the plan's
    Decisions section documents (``_objects_script`` embeds the raw dict via
    ``json.dumps`` after ``layout.validate``).
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )
    layout_dict = _valid_layout_dict()
    layout_dict["robot"]["axle_offset"] = {"forward_cm": 8.0, "lateral_cm": -1.5}

    out_path = tmp_path / "axle_offset.html"
    render(track_dict, out_path, objects=layout_dict)
    html = out_path.read_text(encoding="utf-8")

    embedded_objects = _extract_embedded_objects(html)
    assert embedded_objects == layout_dict
    assert embedded_objects["robot"]["axle_offset"] == {
        "forward_cm": 8.0,
        "lateral_cm": -1.5,
    }


def test_axle_tc7_no_objects_and_offset_free_layout_leave_meta_unchanged(tmp_path):
    """TC7: no-layout and offset-free-layout renders leave PLAYER_META/TC9 green.

    Key assertion (NOT a full-page byte assertion — the plan's AC8 is explicit
    that the PAGE bytes DO change because ``renderer.js``/``vision.js``/
    ``cargo.js`` are now different inlined text): ``render(objects=None)``
    still emits no ``const OBJECTS`` at all (mirrors the pre-existing P-TC5
    contract, re-asserted here so a future axle_offset regression can't
    quietly widen when OBJECTS is emitted), and a render carrying an
    ``axle_offset``-FREE layout still leaves ``PLAYER_META`` equal to the
    exact TC9 literal (:data:`_EXPECTED_TC9_META`) — both legs prove the
    axle-offset feature is additive and never perturbs the frozen
    PLAYER_META/TC9/TC-META contract.
    """
    track_dict = json.loads(
        (_FIXTURES / "ctps_lrs_track.json").read_text(encoding="utf-8")
    )

    # Leg 1: no layout at all — no const OBJECTS, TC9 meta unchanged.
    no_objects_path = tmp_path / "axle_tc7_no_objects.html"
    render(track_dict, no_objects_path)
    no_objects_html = no_objects_path.read_text(encoding="utf-8")
    assert "const OBJECTS" not in no_objects_html
    assert _extract_embedded_objects(no_objects_html) is None
    assert _extract_player_meta(no_objects_html) == _EXPECTED_TC9_META

    # Leg 2: a layout present but carrying no axle_offset — OBJECTS embeds,
    # PLAYER_META is STILL the unchanged TC9 literal.
    offset_free_layout = _valid_layout_dict()
    assert "axle_offset" not in offset_free_layout["robot"]
    offset_free_path = tmp_path / "axle_tc7_offset_free.html"
    render(track_dict, offset_free_path, objects=offset_free_layout)
    offset_free_html = offset_free_path.read_text(encoding="utf-8")
    assert _extract_embedded_objects(offset_free_html) == offset_free_layout
    assert _extract_player_meta(offset_free_html) == _EXPECTED_TC9_META


def test_axle_tc8_draw_sites_reference_axle_to_body_pose_helper():
    """TC8: source-text pin — every robot-local DRAW site calls axleToBodyPose(.

    Key assertion (tamper-evidence for AC1-AC3, no JS runner exists): the
    on-disk ``renderer.js`` (body marker + hit-test), ``vision.js`` (cone +
    the ``k/12`` mosaic-visibility HUD), and ``cargo.js`` (grab/box/drop-zone
    overlays incl. deployed zones) each reference ``axleToBodyPose(`` at their
    draw sites — proving the four draw sites were actually wired to the
    shared ``CTPS_OBJ.axleToBodyPose`` helper (T2) rather than left on the raw
    axle pose. A future edit that quietly reverts one draw site back to the
    axle pose fails this test immediately.
    """
    renderer_js = (PLAYER_JS_PATH.parent / "renderer.js").read_text(encoding="utf-8")
    vision_js = (PLAYER_JS_PATH.parent / "vision.js").read_text(encoding="utf-8")
    cargo_js = (PLAYER_JS_PATH.parent / "cargo.js").read_text(encoding="utf-8")

    assert "axleToBodyPose(" in renderer_js
    assert "axleToBodyPose(" in vision_js
    assert "axleToBodyPose(" in cargo_js


#: The five cargo EVENT-RESOLUTION functions that must stay on the raw axle
#: pose (AC4) — the plan's explicit "resolution is NOT moved" decision.
_CARGO_RESOLUTION_FUNCTIONS = (
    "cargo_resolve",
    "_doGrab",
    "_doDump",
    "_doBoxClose",
    "_doBoxOpen",
)


def test_axle_tc9_cargo_resolution_functions_do_not_use_axle_to_body_pose():
    """TC9: source-text pin — cargo EVENT RESOLUTION stays on the axle pose (AC4).

    Key assertion (text heuristic, robust to formatting — mirrors TC12's
    isolate-the-function-body technique): for each of ``cargo_resolve``,
    ``_doGrab``, ``_doDump``, ``_doBoxClose``, and ``_doBoxOpen``, the isolated
    top-level function body (from ``function <name>(`` to its closing brace at
    the same 4-space indent) contains NO reference to ``axleToBodyPose`` —
    proving the drawn zones (T5, moved to the body pose) and the RESOLVED
    zones (unchanged, raw axle pose from ``CTPS.engine.poseAt``) stay
    deliberately divergent, per the plan's Decisions: "the cargo event
    resolution stays on the axle — user choice". A sanity assertion confirms
    ``axleToBodyPose`` still appears SOMEWHERE in ``cargo.js`` (the draw
    sites, TC8) so a failure here reads as "resolution moved," never "the
    helper vanished from the file."
    """
    cargo_js = (PLAYER_JS_PATH.parent / "cargo.js").read_text(encoding="utf-8")
    assert "axleToBodyPose(" in cargo_js  # sanity: draw sites still wired (TC8)

    for name in _CARGO_RESOLUTION_FUNCTIONS:
        match = re.search(
            r"function\s+" + re.escape(name) + r"\s*\([\s\S]*?\n    \}", cargo_js
        )
        assert match is not None, f"could not isolate {name}'s function body in cargo.js"
        assert "axleToBodyPose" not in match.group(0), (
            f"{name} must resolve against the raw axle pose (AC4) but its body "
            "references axleToBodyPose"
        )
