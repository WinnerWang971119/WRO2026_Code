"""Deterministic ``#include "..."`` rewriter for production files (T1 item 3).

Encodes the spec's "Include-form rules" table exactly. Angle-bracket ``<...>``
includes never change -- the regex below only ever matches a quote-include, so
an angle-bracket line is structurally invisible to this module. A quote-include
of a header this migration does not track (a test-only header, a vendor shim
reached only behind a build flag, ...) is also left untouched: only basenames
present in :data:`tools.reorg.mapping.MAPPING` or
:data:`tools.reorg.mapping.CONFIG_SUBHEADERS` are ever rewrite candidates.

No-op / idempotency guarantee
------------------------------
The new include form depends on the (includer location, target location) pair,
both resolved through :func:`locate`. Two ``Layout`` modes exist:

* ``"old"`` (:func:`locate_old_layout`) -- classifies EVERY file as still
  sitting at the flat pre-move root (or ``config/``), exactly matching today's
  actual tree. Because it can never report a ``src/<domain>`` location for
  either side of an include, :func:`rewrite_form` can never leave its
  root/root and root/config "unchanged" branches -- so running the rewriter in
  this mode over the current tree is a no-op *by construction*, not just by
  incidental correctness. This is what "green/no-op under the old layout"
  means and is exercised directly by the test suite against the real tree.
* ``"new"`` (:func:`locate_new_layout`) -- classifies every file per the
  mapping table's destination, independent of whether ``git mv`` has actually
  run yet. T5/T6 invoke the rewriter in this mode AFTER performing the move,
  at which point physical reality and this classification agree.

Rewriting is idempotent under either mode: the target's basename is always
re-extracted from the (possibly already-rewritten) quoted path via
``PurePosixPath(...).name`` before recomputing the form, so re-running the
rewriter on already-correct text reproduces the same text.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Callable, Dict, Optional, Tuple

from .mapping import CONFIG_SUBHEADERS, DOMAINS, MAPPING

#: Matches a quote-include line, capturing the leading `#include` token, the
#: quoted path, and any trailing text (spacing + a `// comment`) so rewriting
#: preserves formatting byte-for-byte apart from the path itself.
#:
#: Anchored so a `//`-prefixed line that merely MENTIONS an `#include "X.h"`
#: in prose (several WRO2026/test/ docblocks do this) never matches: the line
#: itself must start with `#` (after optional leading whitespace), which a
#: `//` comment line does not.
#:
#: `re.DOTALL` makes the `suffix` group's `.*` also swallow the line's own
#: trailing `\n` (rather than stopping just short of it, which `$` alone
#: would otherwise allow -- silently dropping every newline once lines are
#: rejoined with `"".join(...)` and running consecutive `#include`s together).
_INCLUDE_RE = re.compile(
    r'^(?P<prefix>[ \t]*#[ \t]*include[ \t]*)"(?P<path>[^"]+)"(?P<suffix>.*)$', re.DOTALL
)

#: ``"old"`` (pre-move, flat) or ``"new"`` (post-move, per the mapping table).
Layout = str


@dataclass(frozen=True)
class Location:
    """Where a file lives, for include-resolution purposes."""

    kind: str  # "root" | "config" | "src"
    domain: Optional[str] = None

    def __post_init__(self) -> None:
        if self.kind == "src":
            assert self.domain in DOMAINS, f"bad domain {self.domain!r} for a src location"
        else:
            assert self.kind in ("root", "config"), f"bad location kind {self.kind!r}"
            assert self.domain is None, f"a {self.kind!r} location must not carry a domain"


#: The sketch root itself (operator files: ``.ino``, ``Config.h``, ``Tasks.*``, ...).
ROOT = Location(kind="root")
#: The ``WRO2026/config/`` sub-header directory.
CONFIG = Location(kind="config")


def src(domain: str) -> Location:
    """A ``WRO2026/src/<domain>/`` location."""
    return Location(kind="src", domain=domain)


def locate_old_layout(basename: str) -> Location:
    """Classify `basename` under the CURRENT (pre-move) flat layout.

    Every one of the classified root files -- including the ones that will
    eventually move -- is still physically sitting at the flat ``WRO2026/``
    root today, so this NEVER reports a ``src/<domain>`` location. See the
    module docstring for why that is exactly what makes the rewriter provably
    a no-op pre-move.
    """
    if basename in CONFIG_SUBHEADERS:
        return CONFIG
    if basename in MAPPING:
        return ROOT
    raise KeyError(f"{basename!r} is not a migration-tracked basename (see mapping.MAPPING)")


def locate_new_layout(basename: str) -> Location:
    """Classify `basename` under the migration's target layout.

    Reads ONLY the mapping table (the single source of truth) -- independent
    of whether the physical ``git mv`` has actually happened on disk yet.
    Callers invoke this once the move has run, so physical reality and this
    classification agree, but the function itself never touches the filesystem.
    """
    if basename in CONFIG_SUBHEADERS:
        return CONFIG
    if basename in MAPPING:
        domain = MAPPING[basename]
        return ROOT if domain is None else src(domain)
    raise KeyError(f"{basename!r} is not a migration-tracked basename (see mapping.MAPPING)")


_LAYOUT_FUNCS: Dict[str, Callable[[str], Location]] = {
    "old": locate_old_layout,
    "new": locate_new_layout,
}


def locate(basename: str, layout: Layout) -> Location:
    """Dispatch to :func:`locate_old_layout` / :func:`locate_new_layout` by name."""
    try:
        fn = _LAYOUT_FUNCS[layout]
    except KeyError as exc:
        raise ValueError(f"unknown layout {layout!r}; expected 'old' or 'new'") from exc
    return fn(basename)


def rewrite_form(includer: Location, target_basename: str, target: Location) -> str:
    """The new (unquoted) include-path text, per the Contracts include-form-rules table.

    | includer   | target   | form                              |
    |------------|----------|-----------------------------------|
    | src/A      | src/A    | ``Header.h`` (bare, unchanged)     |
    | src/A      | src/B    | ``../B/Header.h``                 |
    | src/A      | root     | ``../../Header.h``                |
    | src/A      | config   | ``../../config/Header.h``         |
    | root       | src/B    | ``src/B/Header.h``                 |
    | root       | root     | ``Header.h`` (bare, unchanged)     |
    | root       | config   | ``config/Header.h``                |
    | config     | config   | ``Header.h`` (bare, unchanged)     |
    """
    if includer.kind == "src":
        if target.kind == "src":
            if target.domain == includer.domain:
                return target_basename
            return f"../{target.domain}/{target_basename}"
        if target.kind == "config":
            return f"../../config/{target_basename}"
        return f"../../{target_basename}"  # target.kind == "root"

    if includer.kind == "root":
        if target.kind == "src":
            return f"src/{target.domain}/{target_basename}"
        if target.kind == "config":
            return f"config/{target_basename}"
        return target_basename  # root -> root

    # includer.kind == "config": a config sub-header including a sibling
    # config sub-header is always bare (Contracts row 7). Config sub-headers
    # never include a src/<domain> framework header or a root file, so no
    # other target.kind is reachable here in practice.
    return target_basename


def parse_include(line: str) -> Optional[Tuple[str, str, str]]:
    """Return ``(prefix, included_path, suffix)`` for a quote-include line, else ``None``."""
    m = _INCLUDE_RE.match(line)
    if m is None:
        return None
    return m.group("prefix"), m.group("path"), m.group("suffix")


def rewrite_production_line(line: str, includer: Location, layout: Layout = "new") -> str:
    """Rewrite one source line if it is a quote-include of a migration-tracked header.

    Returns `line` unchanged for: angle-bracket includes (never matched at
    all), non-include lines, and quote-includes of a basename this migration
    does not track.
    """
    parsed = parse_include(line)
    if parsed is None:
        return line
    prefix, included, suffix = parsed
    target_basename = PurePosixPath(included).name
    if target_basename not in MAPPING and target_basename not in CONFIG_SUBHEADERS:
        return line
    target = locate(target_basename, layout)
    new_path = rewrite_form(includer, target_basename, target)
    return f'{prefix}"{new_path}"{suffix}'
