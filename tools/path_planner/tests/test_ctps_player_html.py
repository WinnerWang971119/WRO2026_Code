"""Hermetic tests for the CTPS HTML player emitter (T10: TC5, TC6, TC12).

These tests exercise :func:`tools.path_planner.ctps.player_html.render` and
the inlined ``assets/player.js`` runtime as **text** — no browser, no compiler,
no network.

Covered cases (CTPS plan, T10):

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
"""

from __future__ import annotations

import base64
import json
import re
import warnings
from pathlib import Path

import pytest

from tools.path_planner.ctps.player_html import PLAYER_JS_PATH, render
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
