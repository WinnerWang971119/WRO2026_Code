"""Single-source parser for ``constexpr`` constants in the firmware Config sub-headers.

The firmware headers under ``WRO2026/config/`` own the chassis + tuning constants
the offline path planner, validator, and CTPS profile kernel reuse (the chassis
track, the line-sensor-to-axle offset, the pivot-trim floor, the turn-dispatch
tolerance, the T-shape ramp/PWM constants). These modules parse those values out
of the headers at build time rather than mirroring literals, so an edit to a
Config sub-header cannot silently desync the offline tools from the value the
robot drives with (issue #44).

This module is the ONE place that parsing lives, so callers cannot drift in HOW
they read the headers. Previously each carried its own near-identical
``constexpr float`` regex, so a fix to one (for example, anchoring the match)
had to be duplicated in the other or silently diverge.

The float pattern is anchored with :data:`re.MULTILINE` to the first
non-whitespace token on a line, so a ``//``-commented or example declaration
above the real one is skipped instead of shadowing it: :func:`re.Pattern.search`
returns the first match, and a commented line is no longer a match. The int
parser (:func:`parse_config_int`) follows the identical anchoring discipline and
adds bounded, one-level identifier-RHS resolution (e.g.
``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE``) across a fixed, small set of headers
-- never an unbounded chain, never an expression (issue-#44 fail-loud policy).
"""

from __future__ import annotations

import re
from pathlib import Path

# ``planner.py`` / ``validator.py`` / this module all live at
# ``<repo>/tools/path_planner/<mod>.py`` and the firmware headers at
# ``<repo>/WRO2026/config/*.h``. Resolve relative to THIS file (two
# parents up to the repo root) so the lookup ignores the caller's working directory.
_CONFIG_DIR = Path(__file__).resolve().parents[2] / "WRO2026" / "config"

DEFAULT_CONFIG_H_PATH = _CONFIG_DIR / "ConfigDrive.h"

# The bounded set of headers a symbolic RHS (e.g. `CRUISE_SPEED = SPEED_NORMAL`)
# may be resolved across (per the plan's Decisions: "resolved across
# ConfigDrive.h / ConfigGlobal.h / ConfigLine.h only"). Deliberately NOT every
# header under config/ -- an unbounded search space would let a symbolic RHS
# silently resolve against the wrong domain header.
DEFAULT_INT_SEARCH_HEADERS: tuple[Path, ...] = (
    _CONFIG_DIR / "ConfigDrive.h",
    _CONFIG_DIR / "ConfigGlobal.h",
    _CONFIG_DIR / "ConfigLine.h",
)


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
    """Parse a ``constexpr float <name>`` literal out of ``WRO2026/config/ConfigDrive.h``.

    The firmware header is the single source of truth for the chassis constants
    the offline tools reuse. This reads the named ``constexpr float``
    declaration and returns its value verbatim (no unit conversion -- callers
    convert if needed).

    Fails loudly with no stale fallback (issue #44 policy): a missing header or
    a missing declaration raises rather than returning a guessed value, because
    a wrong constant silently mis-compensates every pivot in the field.

    :param name: the ``constexpr float`` identifier to read.
    :param config_path: path to ``ConfigDrive.h``; defaults to the firmware
        header resolved relative to this module (independent of the cwd).
    :return: the parsed float value, in the header's own units.
    :raises OSError: when ``config_path`` cannot be read.
    :raises ValueError: when no matching declaration is found.
    """
    try:
        text = config_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise OSError(
            f"Cannot parse '{name}': firmware header at {config_path} is "
            f"unreadable ({exc}). ConfigDrive.h is the single source of truth for "
            f"the offline tools' chassis constants (issue #44); there is no fallback "
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


# A bare C++ identifier: letters/underscore, then letters/digits/underscore.
_IDENTIFIER_RE = r"[A-Za-z_][A-Za-z0-9_]*"


def config_int_pattern(name: str) -> re.Pattern:
    """Return an anchored regex capturing a ``constexpr int16_t``/``uint8_t`` RHS.

    Matches ``[static ]constexpr (int16_t|uint8_t) <name> = <rhs>;`` where
    ``<rhs>`` is either a signed integer literal or a bare identifier (captured
    verbatim in group 1 -- the caller decides whether it is a literal or a
    symbolic reference). Tolerates an optional leading ``static`` (the
    ``MatrixTT``/``LegoEV3`` profile structs in ``ConfigDrive.h`` use
    ``static constexpr`` for their nested members) and surrounding whitespace.

    Anchored with :data:`re.MULTILINE` to the first non-whitespace token on a
    line, mirroring :func:`config_float_pattern`, so a ``//``-commented or
    mid-line declaration of the same constant is NOT a match and cannot shadow
    the real one.

    :param name: the C++ ``constexpr int16_t``/``uint8_t`` identifier to match.
    :return: a compiled :class:`re.Pattern` with the raw RHS text in group 1.
    """
    return re.compile(
        r"^[ \t]*(?:static\s+)?constexpr\s+(?:int16_t|uint8_t)\s+"
        + re.escape(name)
        + r"\s*=\s*(-?[0-9]+|"
        + _IDENTIFIER_RE
        + r")\s*;",
        re.MULTILINE,
    )


_INT_LITERAL_RE = re.compile(r"^-?[0-9]+$")


def parse_config_int(
    name: str,
    config_path: Path = DEFAULT_CONFIG_H_PATH,
    search_headers: tuple[Path, ...] = DEFAULT_INT_SEARCH_HEADERS,
) -> int:
    """Parse a ``constexpr int16_t``/``uint8_t`` constant, resolving a symbolic RHS.

    Reads the named declaration out of ``config_path``. If the right-hand side
    is a plain integer literal (e.g. ``TSHAPE_ACCEL_PWM_PER_CM = 5``), that
    value is returned directly. If the right-hand side is a bare identifier
    (e.g. ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`` or
    ``CRUISE_SPEED = SPEED_NORMAL``), this performs **exactly one level** of
    resolution: it searches ``search_headers`` (in order) for an integer-literal
    declaration of that identifier and returns its value.

    This is a bounded resolver, not a general C++ evaluator (issue #44
    no-stale-fallback policy):

    - Only ONE identifier hop is followed. If the resolved declaration's own
      RHS is itself a bare identifier (a two-level chain), this raises rather
      than continuing to chase it.
    - Only a bare integer literal or a single bare identifier is accepted as an
      RHS. Any expression (``A + B``, ``A * 2``, a cast, a ternary, ...) fails
      to match :func:`config_int_pattern` at all and raises.
    - The symbolic lookup is scoped to ``search_headers`` (by default
      ``ConfigDrive.h`` / ``ConfigGlobal.h`` / ``ConfigLine.h`` -- the only
      headers named in the CTPS motion-profile plan's Decisions), never an
      open-ended search of every header under ``config/``.

    :param name: the ``constexpr int16_t``/``uint8_t`` identifier to read.
    :param config_path: header to read ``name``'s own declaration from;
        defaults to ``ConfigDrive.h`` resolved relative to this module.
    :param search_headers: headers searched, in order, to resolve a symbolic
        RHS one level deep; defaults to
        :data:`DEFAULT_INT_SEARCH_HEADERS`.
    :return: the resolved integer value.
    :raises OSError: when ``config_path`` (or, during symbolic resolution, one
        of ``search_headers``) cannot be read.
    :raises ValueError: when no matching declaration is found, the RHS is
        neither a literal nor a bare identifier (i.e. an expression), or the
        resolved symbol's own RHS is not itself a literal (a two-level chain).
    """
    try:
        text = config_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise OSError(
            f"Cannot parse '{name}': firmware header at {config_path} is "
            f"unreadable ({exc}). The Config sub-headers are the single source "
            f"of truth for the offline tools' constants (issue #44); there is "
            f"no fallback literal."
        ) from exc

    match = config_int_pattern(name).search(text)
    if match is None:
        raise ValueError(
            f"Cannot parse '{name}': no '[static ]constexpr (int16_t|uint8_t) "
            f"{name} = <literal-or-identifier>;' declaration found in "
            f"{config_path}. The offline tools refuse to fall back to a stale "
            f"literal (issue #44) -- fix the header or the parser, do not "
            f"hard-code the value. Note: an RHS that is an expression (e.g. "
            f"'A + B') is intentionally NOT matched -- only a bare integer "
            f"literal or a single bare identifier is supported."
        )

    rhs = match.group(1)
    if _INT_LITERAL_RE.match(rhs):
        return int(rhs)

    # Symbolic RHS: exactly one level of identifier resolution, bounded to
    # search_headers. Do NOT recurse into parse_config_int here -- recursion
    # would silently permit an arbitrarily long chain, which the plan's
    # Decisions explicitly rule out ("fail loud on any expression or chain").
    for header in search_headers:
        try:
            header_text = header.read_text(encoding="utf-8")
        except OSError as exc:
            raise OSError(
                f"Cannot resolve '{name} = {rhs}': symbolic-RHS search header "
                f"{header} is unreadable ({exc}). There is no fallback literal "
                f"(issue #44)."
            ) from exc

        resolved_match = config_int_pattern(rhs).search(header_text)
        if resolved_match is None:
            continue

        resolved_rhs = resolved_match.group(1)
        if not _INT_LITERAL_RE.match(resolved_rhs):
            raise ValueError(
                f"Cannot resolve '{name} = {rhs}': '{rhs}' itself resolves to "
                f"'{resolved_rhs}' in {header}, a second identifier -- this is "
                f"a two-level chain. The parser resolves exactly ONE level of "
                f"symbolic RHS and fails loud rather than chasing a chain "
                f"(issue #44) -- flatten the header so '{rhs}' is a literal, or "
                f"extend the resolver deliberately (not via recursion)."
            )
        return int(resolved_rhs)

    searched = ", ".join(str(h) for h in search_headers)
    raise ValueError(
        f"Cannot resolve '{name} = {rhs}': no integer-literal declaration of "
        f"'{rhs}' found in any of [{searched}]. The offline tools refuse to "
        f"fall back to a stale literal (issue #44) -- fix the header or the "
        f"parser, do not hard-code the value."
    )
