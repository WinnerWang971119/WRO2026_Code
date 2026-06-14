"""Single-source parser for ``constexpr float`` constants in WRO2026/Config.h.

The firmware header ``WRO2026/Config.h`` owns the chassis constants the offline
path planner and validator reuse (the chassis track, the line-sensor-to-axle
offset, the pivot-trim floor, the turn-dispatch tolerance). Both modules parse
those values out of the header at build time rather than mirroring literals, so
an edit to ``Config.h`` cannot silently desync the offline tools from the value
the robot drives with (issue #44).

This module is the ONE place that parsing lives, so ``planner`` and ``validator``
cannot drift in HOW they read the header. Previously each carried its own
near-identical ``constexpr float`` regex, so a fix to one (for example,
anchoring the match) had to be duplicated in the other or silently diverge.

The pattern is anchored with :data:`re.MULTILINE` to the first non-whitespace
token on a line, so a ``//``-commented or example declaration above the real one
is skipped instead of shadowing it: :func:`re.Pattern.search` returns the first
match, and a commented line is no longer a match.
"""

from __future__ import annotations

import re
from pathlib import Path

# ``planner.py`` / ``validator.py`` / this module all live at
# ``<repo>/tools/path_planner/<mod>.py`` and the firmware header at
# ``<repo>/WRO2026/Config.h``. Resolve relative to THIS file (two parents up to
# the repo root) so the lookup ignores the caller's working directory.
DEFAULT_CONFIG_H_PATH = Path(__file__).resolve().parents[2] / "WRO2026" / "Config.h"


def config_float_pattern(name: str) -> re.Pattern:
    """Return an anchored regex capturing ``constexpr float <name> = <value>[f];``.

    Tolerates an optional trailing ``f`` / ``F`` suffix and surrounding
    whitespace, and captures the numeric literal (integer or decimal) in group
    1. Anchored with :data:`re.MULTILINE` to the first non-whitespace token on a
    line, so a ``//``-commented or mid-line declaration of the same constant is
    NOT a match and cannot shadow the real one.

    :param name: the C++ ``constexpr float`` identifier to match.
    :return: a compiled :class:`re.Pattern` with the value in group 1.
    """
    return re.compile(
        r"^[ \t]*constexpr\s+float\s+"
        + re.escape(name)
        + r"\s*=\s*([0-9]*\.?[0-9]+)\s*[fF]?\s*;",
        re.MULTILINE,
    )


def parse_config_float(name: str, config_path: Path = DEFAULT_CONFIG_H_PATH) -> float:
    """Parse a ``constexpr float <name>`` literal out of ``WRO2026/Config.h``.

    The firmware header is the single source of truth for the chassis constants
    the offline tools reuse. This reads the named ``constexpr float``
    declaration and returns its value verbatim (no unit conversion -- callers
    convert if needed).

    Fails loudly with no stale fallback (issue #44 policy): a missing header or
    a missing declaration raises rather than returning a guessed value, because
    a wrong constant silently mis-compensates every pivot in the field.

    :param name: the ``constexpr float`` identifier to read.
    :param config_path: path to ``Config.h``; defaults to the firmware header
        resolved relative to this module (independent of the cwd).
    :return: the parsed float value, in the header's own units.
    :raises OSError: when ``config_path`` cannot be read.
    :raises ValueError: when no matching declaration is found.
    """
    try:
        text = config_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise OSError(
            f"Cannot parse '{name}': firmware header at {config_path} is "
            f"unreadable ({exc}). Config.h is the single source of truth for the "
            f"offline tools' chassis constants (issue #44); there is no fallback "
            f"literal."
        ) from exc

    match = config_float_pattern(name).search(text)
    if match is None:
        raise ValueError(
            f"Cannot parse '{name}': no 'constexpr float {name} = <value>;' "
            f"declaration found in {config_path}. The offline tools refuse to "
            f"fall back to a stale literal (issue #44) -- fix the header or the "
            f"parser, do not hard-code the value."
        )

    return float(match.group(1))
