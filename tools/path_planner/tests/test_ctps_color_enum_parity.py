"""Colour-enum parity guard: ``Config::Color`` (C++) vs ``mosaic.COLOR_NAMES`` (Python).

CTPS randomized-vision plan (ADR-060), T3 / AC4 / TC8. ``mosaic.py`` is the
single Python source of the mosaic colour vocabulary
(:data:`tools.path_planner.ctps.mosaic.COLOR_NAMES`); ``Config::Color`` in
``WRO2026/config/ConfigVision.h`` is the single C++ source. Nothing keeps the
two lists in sync except this test: it regex-parses the enumerator names
straight out of the header (in declaration order, sentinel stripped) and
asserts they equal ``COLOR_NAMES`` name-for-name and order-for-order. Any
future edit that adds/renames/reorders a colour on one side without the
other fails CI immediately instead of drifting silently (mirrors the
``parse_track_cm_from_config`` / ``config_parse.py`` header-parse precedent
used elsewhere in this pipeline for firmware constants).
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from tools.path_planner.ctps.mosaic import COLOR_NAMES

#: The sentinel value ``Config::Color`` carries that has no Python counterpart
#: (a failed classification) — stripped before comparing against COLOR_NAMES.
_SENTINEL = "UNKNOWN"

#: Matches the ``enum class Color : uint8_t { ... }`` declaration and captures
#: everything between the braces (the enumerator-list body). ``re.DOTALL`` lets
#: ``.`` cross the real newlines inside the multi-line enum body; the base-type
#: clause (``: uint8_t``) is consumed by the pattern, not captured. This
#: intentionally does NOT match the surrounding ``#pragma push_macro("WHITE")``
#: / ``#undef WHITE`` / ``#pragma pop_macro("WHITE")`` lines: those sit outside
#: the ``{ ... }`` pair, so anchoring on `enum class Color ... { (body) }`
#: naturally excludes them from the captured body.
_ENUM_BLOCK_RE = re.compile(
    r"enum\s+class\s+Color\s*(?::\s*\w+\s*)?\{(?P<body>.*?)\}",
    re.DOTALL,
)

#: Matches one enumerator name inside the captured body: an identifier,
#: optionally followed by `= <value>` (only ``UNKNOWN = 0`` uses this in the
#: real header, but the regex tolerates any enumerator initializer), ignoring
#: a trailing comma. Applied line-by-line so a `//` end-of-line comment never
#: has to be stripped from within the name itself.
_ENUMERATOR_RE = re.compile(r"^([A-Za-z_]\w*)\s*(?:=\s*[^,]+)?,?\s*$")


def _locate_repo_root(start: Path) -> Path:
    """Walk upward from ``start`` to find the repository root.

    Robust to this test file's exact depth under the repo (works whether it
    lives at ``tools/path_planner/tests/`` or is moved later) and to being
    invoked from any working directory on Windows or Linux/macOS CI: it never
    hardcodes an absolute path and never depends on ``cwd``. The root is
    identified by the presence of the ``WRO2026`` firmware directory, which is
    unambiguous in this repository layout (see ``ARCHITECTURE.md``).

    :param start: a file path to start the upward walk from (typically
        ``Path(__file__)``).
    :return: the resolved repository root directory.
    :raises RuntimeError: if no ancestor directory contains ``WRO2026/``
        (e.g. the file was copied out of the repo entirely).
    """
    current = start.resolve()
    for candidate in (current, *current.parents):
        if (candidate / "WRO2026").is_dir():
            return candidate
    raise RuntimeError(
        f"_locate_repo_root: no ancestor of {start!s} contains a 'WRO2026' "
        "directory; is this file still inside the repository?"
    )


def _config_vision_header_path() -> Path:
    """Return the resolved path to ``WRO2026/config/ConfigVision.h``.

    :return: the header path, located relative to the repo root (never a
        hardcoded absolute path), so this passes on Windows and on
        Linux/macOS CI alike.
    """
    repo_root = _locate_repo_root(Path(__file__))
    return repo_root / "WRO2026" / "config" / "ConfigVision.h"


def _parse_color_enum_names(header_text: str) -> list[str]:
    """Regex-parse the ``enum class Color`` enumerator names, in declaration order.

    Captures only the body between the braces of ``enum class Color : uint8_t
    { ... }`` — the surrounding ``#pragma push_macro("WHITE")`` / ``#undef
    WHITE`` / ``#pragma pop_macro("WHITE")`` guard lines sit outside that brace
    pair and are therefore never part of the captured text. Comments and the
    ``: uint8_t`` base-type clause are likewise excluded (the base-type clause
    is consumed, not captured, by :data:`_ENUM_BLOCK_RE`; there are no
    inline comments inside the real enumerator list to strip).

    :param header_text: the full text of ``ConfigVision.h``.
    :return: enumerator names in source order (``UNKNOWN`` still included —
        the caller strips it).
    :raises AssertionError: if no ``enum class Color { ... }`` block is found,
        or if the body contains no parseable enumerator names — a loud
        failure is preferable to silently asserting an empty list matches.
    """
    match = _ENUM_BLOCK_RE.search(header_text)
    assert match is not None, (
        "test_ctps_color_enum_parity: could not find an "
        "'enum class Color { ... }' declaration in ConfigVision.h — "
        "did the enum get renamed, or its brace layout change?"
    )

    body = match.group("body")
    names: list[str] = []
    for raw_line in body.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("//"):
            continue
        enumerator_match = _ENUMERATOR_RE.match(line)
        assert enumerator_match is not None, (
            f"test_ctps_color_enum_parity: could not parse enumerator line "
            f"{raw_line!r} out of the Config::Color body; the regex may need "
            "updating to match a new enum layout."
        )
        names.append(enumerator_match.group(1))

    assert names, (
        "test_ctps_color_enum_parity: parsed an empty enumerator list out of "
        "the Config::Color body — the enum block was found but contained no "
        "recognizable names."
    )
    return names


def test_color_names_match_config_color_enum() -> None:
    """``mosaic.COLOR_NAMES`` must equal ``Config::Color`` (minus ``UNKNOWN``), in order.

    Regression guard (AC4 / TC8): regex-parses the enumerator list straight out
    of the real firmware header and compares it to the Python colour
    vocabulary, both by NAME and by ORDER. A future edit to either side that is
    not mirrored on the other (e.g. adding a fifth colour, renaming ``BLUE`` to
    ``CYAN``, or reordering the declaration) fails this assertion instead of
    silently desyncing the C++ mock's letter->``Config::Color`` parse from the
    Python roll/render pipeline.
    """
    header_path = _config_vision_header_path()
    header_text = header_path.read_text(encoding="utf-8")

    parsed_names = _parse_color_enum_names(header_text)

    assert parsed_names[0] == _SENTINEL, (
        f"test_ctps_color_enum_parity: expected the first Config::Color "
        f"enumerator to be the {_SENTINEL!r} sentinel; got "
        f"{parsed_names[0]!r}. Parsed order: {parsed_names!r}."
    )
    parsed_colors = parsed_names[1:]

    assert parsed_colors == list(COLOR_NAMES), (
        "test_ctps_color_enum_parity: Config::Color (WRO2026/config/"
        f"ConfigVision.h, {_SENTINEL} stripped) parsed as {parsed_colors!r}, "
        f"but tools.path_planner.ctps.mosaic.COLOR_NAMES is "
        f"{list(COLOR_NAMES)!r}. These two lists must match in name AND "
        "order -- update mosaic.COLOR_NAMES (and COLOR_LETTER / COLOR_HEX) "
        "to match the firmware enum, or vice versa."
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
