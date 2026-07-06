"""Shared emit-scaffold helpers for CTPS offline HTML page emitters.

Extracted out of :mod:`tools.path_planner.ctps.player_html` (T1, issue #57) so
a second page emitter (:mod:`tools.path_planner.ctps.layout_editor_html`, T7)
can reuse the exact same self-containment lint / asset reader / script-wrap
helpers instead of duplicating them. This is a pure move — behavior is
unchanged from the pre-extraction versions that lived in ``player_html.py``.

Exposes:

* :func:`read_asset` — read one on-disk CSS/JS asset as UTF-8 text, raising a
  path-naming :class:`FileNotFoundError` if it is missing.
* :func:`assert_self_contained` — lint a mapping of named sources for the
  offline self-containment invariant (no external ``src=``/``href=``, no
  network URL, no inline ``data:image`` background, no literal ``</script>``).
* :func:`script` — wrap a JS body in its own ``<script>`` element.
* :data:`FORBIDDEN_SUBSTRINGS` — the substrings :func:`assert_self_contained`
  bans.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

__all__ = [
    "read_asset",
    "assert_self_contained",
    "script",
    "FORBIDDEN_SUBSTRINGS",
]

#: Literal substrings that would break a page's offline self-containment if
#: they appeared in an asset or the HTML template markup:
#:
#: * ``src=`` / ``href=`` — an attribute pointing at an external/local resource
#:   (note the missing space is deliberate: ``img.src = MAP_IMAGE`` is fine).
#: * ``http://`` / ``https://`` — a network URL.
#: * ``data:image`` — an inline image background in an asset (a ``data:``
#:   image URI carried as a JSON *value* — e.g. ``MAP_IMAGE`` — is exempt; it
#:   is never passed through this lint).
#: * ``</script>`` — would prematurely close the inline ``<script>`` wrapper an
#:   asset is inlined into.
FORBIDDEN_SUBSTRINGS: Tuple[str, ...] = (
    "src=",
    "href=",
    "http://",
    "https://",
    "data:image",
    "</script>",
)


def read_asset(path: Path) -> str:
    """Read one runtime asset as UTF-8 text.

    :param path: absolute path to a CSS / JS asset.
    :return: the file's contents.
    :raises FileNotFoundError: if the asset does not exist — re-raised with the
        offending path so a missing sibling asset (created by a later task)
        fails loud at emit time (TC-ASSETS).
    """
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(
            f"ctps: required runtime asset not found at {path}."
        ) from exc


def assert_self_contained(sources: Dict[str, str]) -> None:
    """Lint each named source for offline self-containment (AC-lint).

    Raises on the first source that contains any of
    :data:`FORBIDDEN_SUBSTRINGS` — an external reference, network URL, inline
    image background, or a literal ``</script>`` that would break out of the
    inline wrapper. Callers must pass only assets + HTML template markup —
    never the embedded JSON *values* (e.g. TRACK / PLAYER_META / OBJECTS /
    MAP_IMAGE / FIELD), which carry arbitrary user text.

    :param sources: mapping of a human-readable source name (asset file name or
        ``"HTML template"``) to its content.
    :raises ValueError: naming the source and the offending substring.
    """
    for name, content in sources.items():
        for forbidden in FORBIDDEN_SUBSTRINGS:
            if forbidden in content:
                raise ValueError(
                    "ctps: self-containment lint failed — "
                    f"{name!r} contains the forbidden substring {forbidden!r}. "
                    "Assets and the HTML template must be fully offline: no "
                    "'src='/'href=' attributes, no 'http(s)://' URLs, no "
                    "'data:image' background, and no literal '</script>' "
                    "(it would break out of the inline <script> wrapper)."
                )


def script(inner: str) -> str:
    """Wrap ``inner`` in its own ``<script>`` element.

    Each inline constant and each JS asset gets a dedicated element so a
    throwing top-level IIFE cannot abort the siblings, and so a sibling regex
    (e.g. the player emitter test's ``const TRACK = ...;</script>``)
    terminates at one constant alone.

    :param inner: the raw script body (already ``</`` -escaped where it embeds
        user text).
    :return: a ``<script> ... </script>`` block.
    """
    return f"<script>\n{inner}\n</script>"
