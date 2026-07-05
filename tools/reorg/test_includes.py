"""``WRO2026/test/`` include classification + rewriter (T1 item 4).

For a ``WRO2026/test/`` file, a bare production include like ``"PathTracker.h"``
may mean the MOCK (resolved via a force-include or ``-I`` search order) or the
REAL header -- NOT decidable from the file's own content alone in general. This
module derives, for every basename this migration tracks, an auditable
classification:

1. **Unambiguous basenames** -- a MAPPING/CONFIG_SUBHEADERS basename that has NO
   mock counterpart anywhere under ``WRO2026/test/`` (:func:`discover_mock_basenames`)
   can only resolve to the real header: there is nothing else it could mean. Any
   quote-include of one of these is always real intent.
2. **Ambiguous basenames** (:func:`ambiguous_basenames`) -- a basename that DOES
   have a mock counterpart somewhere. Its per-file classification is resolved,
   in order:

   a. **Same-directory sibling** -- if a file with that exact basename exists in
      the SAME directory as the includer, the C++ source-relative include rule
      resolves the bare quote-include to that sibling FIRST, before any ``-I``
      search path is ever consulted. That sibling is always a mock in this
      repo's convention, so this is unambiguously MOCK.
   b. **Explicit real-include manifest** (:data:`REAL_INCLUDE_MANIFEST`) -- the
      known set of (file, basename) pairs that resolve to the real header via
      CMake target context (a leading ``${SKETCH_DIR}`` search path with no
      colliding mock dir on that particular target's include list). Verified
      by hand against ``WRO2026/test/CMakeLists.txt`` at authoring time; see
      each entry's comment for the specific target(s) that justify it.
   c. **Default** -- MOCK. This matches the overwhelming majority convention in
      this repo (every ambiguous include not covered by (a) or (b) really is a
      mock reference in the current tree -- verified by
      ``test_test_includes.py``'s exact pinned classification set).

Generated fixture/toy TUs (:data:`GENERATED_FILE_EXCLUSIONS`) are skipped
entirely, regardless of what the above would otherwise say -- their
``#include`` line comes from the emitter (T8), never a hand-edit.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Dict, FrozenSet, List, Set, Tuple

from .include_rules import locate_new_layout, parse_include
from .mapping import CONFIG_SUBHEADERS, MAPPING

#: The mock-shadow directories under ``WRO2026/test/`` (see
#: ``WRO2026/test/CMakeLists.txt``). Each holds header files that shadow a
#: same-named production header for one or more host-test targets, via
#: force-include or ``-I`` search order.
MOCK_DIRS: Tuple[str, ...] = ("mocks", "hal_mocks", "path_mocks", "task_mocks", "mission_mocks", "ctps_mocks")

#: Generated fixture/toy TUs living under ``WRO2026/test/`` (Contracts:
#: "Host-test shadow invariant"). Their ``#include "routes.h"`` comes from the
#: emitter (T8), never a hand-edit -- skip these files ENTIRELY.
#:
#: ``routes_reset_passthrough_forced.{h,cpp}`` is HAND-WRITTEN (its own banner
#: literally says "HAND-WRITTEN, NOT AUTO-GENERATED") but the plan's Contracts
#: section still names it alongside the two truly-emitted files as owned by
#: T8 going forward -- excluded here on that say-so, not on a content sniff
#: (see :func:`looks_emitter_generated`, which this file's banner defeats: it
#: contains the literal substring "NOT AUTO-GENERATED").
#:
#: The third member of the Contracts trio, ``routes_toy.{h,cpp}``, lives under
#: ``tools/path_planner/tests/golden/`` -- outside ``WRO2026/test/`` entirely --
#: so it is out of this module's scan scope by construction; T8 owns it too.
GENERATED_FILE_EXCLUSIONS: FrozenSet[str] = frozenset({
    "fixtures/routes_fixture.cpp",
    "fixtures/routes_fixture.h",
    "fixtures/routes_reset_passthrough_forced.cpp",
    "fixtures/routes_reset_passthrough_forced.h",
})

#: Explicit real-include manifest (Contracts "Host-test shadow invariant" known
#: set). Keyed by the includer's path RELATIVE TO ``WRO2026/test/`` (POSIX
#: separators). Lists ONLY basenames that are ambiguous (see
#: :func:`ambiguous_basenames`) AND real in that file -- an ambiguous basename
#: with no entry here, and no same-directory sibling mock, defaults to mock.
REAL_INCLUDE_MANIFEST: Dict[str, FrozenSet[str]] = {
    # The five real-linked PathTracker-mirror targets' shared harness (issue
    # #106): each target's `target_include_directories` leads with
    # `${SKETCH_DIR}` and never lists `task_mocks/` / `mission_mocks/` /
    # `ctps_mocks/` (the only dirs that would shadow these three names), so
    # all three resolve to the real headers.
    "test_path_tracker_harness.h": frozenset({"Drive.h", "Localization.h", "PathTracker.h"}),
    # Same target family, a direct ambiguous-basename include outside the harness.
    "test_path_tracker_bidirectional.cpp": frozenset({"PathTracker.h"}),
    # LineFollower.cpp is this target's SUT_SOURCES; `mocks/` (the only mock
    # dir on its COMMON_INCLUDES) has no LineFollower.h of its own.
    "test_line_follower_overshoot.cpp": frozenset({"LineFollower.h"}),
    # Manipulator.cpp is this target's SUT; `mocks/` has no Manipulator.h.
    "test_manipulator.cpp": frozenset({"Manipulator.h"}),
    # Localization.cpp is the SUT in each of these four targets; no mock dir
    # on any of their COMMON_INCLUDES defines Localization.h.
    "test_localization_offset.cpp": frozenset({"Localization.h"}),
    "test_reanchor_at_wall.cpp": frozenset({"Localization.h"}),
    "test_reanchor_at_wall_strict.cpp": frozenset({"Localization.h"}),
    "test_reanchor_heading.cpp": frozenset({"Localization.h"}),
}

_AUTO_GENERATED_BANNER_RE = re.compile(r"^\s*//.*\bAUTO-GENERATED\b")
_NOT_AUTO_GENERATED_RE = re.compile(r"\bNOT AUTO-GENERATED\b")


def discover_mock_basenames(test_root: Path) -> FrozenSet[str]:
    """Every header basename that exists as a file directly inside a :data:`MOCK_DIRS` dir.

    A live filesystem scan (not a hardcoded list): a newly added mock header
    automatically joins the ambiguous set, and :func:`audit_test_includes` will
    then flag (as ``"default-mock"``, unless someone updates
    :data:`REAL_INCLUDE_MANIFEST`) any real-intent use of it this module
    doesn't yet know about.
    """
    found: Set[str] = set()
    for mock_dir_name in MOCK_DIRS:
        mock_dir = test_root / mock_dir_name
        if not mock_dir.is_dir():
            continue
        for entry in mock_dir.iterdir():
            if entry.is_file() and entry.suffix == ".h":
                found.add(entry.name)
    return frozenset(found)


def ambiguous_basenames(test_root: Path) -> FrozenSet[str]:
    """Migration-tracked basenames that ALSO exist as a mock somewhere under ``test/``.

    These are the only names whose real-vs-mock intent is not decidable from
    the basename alone. Every other tracked basename has no mock anywhere in
    the tree, so a quote-include of it in a test file can only mean the real
    header (see :func:`classify_include`'s ``"unambiguous"`` branch).
    """
    mocked = discover_mock_basenames(test_root)
    return frozenset(name for name in mocked if name in MAPPING)


def _posix_relpath(path: Path, test_root: Path) -> str:
    return path.relative_to(test_root).as_posix()


def is_excluded_generated(path: Path, test_root: Path) -> bool:
    """True if `path` is one of the emitter-owned files this tool must never touch."""
    return _posix_relpath(path, test_root) in GENERATED_FILE_EXCLUSIONS


def looks_emitter_generated(path: Path, peek_lines: int = 5) -> bool:
    """Content heuristic: does the file open with an ``AUTO-GENERATED`` banner?

    A SECONDARY, defensive signal for :func:`audit_test_includes` -- NOT the
    exclusion gate (:data:`GENERATED_FILE_EXCLUSIONS` is authoritative). Known
    to be fooled by ``routes_reset_passthrough_forced.{h,cpp}``, whose
    hand-written banner literally contains the substring "NOT AUTO-GENERATED";
    that phrase short-circuits this function back to ``False`` so the
    heuristic does not misreport that specific file as emitter output for the
    wrong reason.
    """
    try:
        with path.open("r", encoding="utf-8", errors="replace") as fh:
            head = [next(fh, "") for _ in range(peek_lines)]
    except OSError:
        return False
    text = "\n".join(head)
    if _NOT_AUTO_GENERATED_RE.search(text):
        return False
    return bool(_AUTO_GENERATED_BANNER_RE.search(text))


@dataclass(frozen=True)
class Classification:
    """The result of classifying a single ambiguous include."""

    classification: str  # "real" | "mock" | "unmapped"
    evidence: str  # "unambiguous" | "same-dir-sibling" | "manifest" | "default-mock" | "unmapped"


def classify_include(
    test_relpath: str,
    includer_sibling_basenames: FrozenSet[str],
    included_basename: str,
    ambiguous: FrozenSet[str],
) -> Classification:
    """Classify one ``#include "included_basename"`` found in `test_relpath`.

    `includer_sibling_basenames` is the set of file basenames living in the
    SAME directory as the includer -- see the module docstring's
    same-directory-sibling rule.
    """
    if included_basename not in MAPPING and included_basename not in CONFIG_SUBHEADERS:
        return Classification("unmapped", "unmapped")
    if included_basename not in ambiguous:
        return Classification("real", "unambiguous")
    if included_basename in includer_sibling_basenames:
        return Classification("mock", "same-dir-sibling")
    if included_basename in REAL_INCLUDE_MANIFEST.get(test_relpath, frozenset()):
        return Classification("real", "manifest")
    return Classification("mock", "default-mock")


def rewrite_test_line(
    line: str,
    test_relpath: str,
    includer_sibling_basenames: FrozenSet[str],
    ambiguous: FrozenSet[str],
) -> str:
    """Rewrite one test-file source line per the Contracts host-test rule.

    Only REAL-classified includes of a MOVED basename are rewritten, and
    always to the root-relative ``"src/<domain>/Header.h"`` form (never a
    ``../``-relative form) -- test files resolve production headers via the
    ``-I ${SKETCH_DIR}`` search path, not source-relative directory nesting, so
    the root-relative form is correct regardless of how deep under
    ``WRO2026/test/`` the includer lives.
    """
    parsed = parse_include(line)
    if parsed is None:
        return line
    prefix, included, suffix = parsed
    basename = PurePosixPath(included).name
    result = classify_include(test_relpath, includer_sibling_basenames, basename, ambiguous)
    if result.classification != "real":
        return line
    target = locate_new_layout(basename)
    if target.kind != "src":
        return line  # root-stay / config header: already correctly resolvable, unchanged
    new_path = f"src/{target.domain}/{basename}"
    return f'{prefix}"{new_path}"{suffix}'


@dataclass(frozen=True)
class ClassifiedInclude:
    """One classified include occurrence, as reported by :func:`audit_test_includes`."""

    test_relpath: str
    line_no: int
    included_basename: str
    classification: str
    evidence: str


def audit_test_includes(test_root: Path) -> List[ClassifiedInclude]:
    """Walk every hand-written test file and classify each ambiguous-basename include.

    The "heuristic/reporting pass" companion to :data:`REAL_INCLUDE_MANIFEST`:
    re-derives same-directory-sibling evidence live from disk for every
    ambiguous include in the tree, so a newly added ambiguous include nobody
    taught this module about is visible (as ``evidence == "default-mock"``)
    instead of silently guessed. Skips :data:`GENERATED_FILE_EXCLUSIONS`.
    """
    ambiguous = ambiguous_basenames(test_root)
    results: List[ClassifiedInclude] = []
    for path in sorted(test_root.rglob("*")):
        if not path.is_file() or path.suffix not in (".h", ".cpp"):
            continue
        relpath = _posix_relpath(path, test_root)
        if relpath in GENERATED_FILE_EXCLUSIONS:
            continue
        sibling_basenames = frozenset(p.name for p in path.parent.iterdir() if p.is_file())
        text = path.read_text(encoding="utf-8", errors="replace")
        for line_no, line in enumerate(text.splitlines(), start=1):
            parsed = parse_include(line)
            if parsed is None:
                continue
            basename = PurePosixPath(parsed[1]).name
            if basename not in ambiguous:
                continue
            result = classify_include(relpath, sibling_basenames, basename, ambiguous)
            results.append(
                ClassifiedInclude(relpath, line_no, basename, result.classification, result.evidence)
            )
    return results
