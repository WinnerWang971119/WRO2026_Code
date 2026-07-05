#!/usr/bin/env python3
"""Static include-resolvability checker for the WRO2026 sketch tree (issue #66 / T3).

The sketch-src reorg (docs/plans/2026-07-05-sketch-src-reorg.md) moves 38 framework
modules from the flat `WRO2026/` root into `WRO2026/src/<domain>/` subfolders while
keeping the operator-editable files at the root. Arduino resolves a quote-form
`#include "..."` ONLY relative to the including file's own directory -- no `-I`
search path -- so every cross-boundary include must be rewritten to its correct
relative (or, for host-test files, root-relative) form. This script is the
permanent guard that catches a wrong include form BEFORE `arduino-cli` or ctest
would (and, for build-flag-gated includes, where NO compiled CI target would).

Two independent, pure-text checks run against the working tree:

1. **Include resolution** (`check_includes`): every `#include "..."` found under
   `WRO2026/` is checked against the rule for its including file --

   - A **production** file (under `WRO2026/`, excluding `test/`) must resolve its
     include SOURCE-RELATIVE to its own directory -- exactly how Arduino resolves a
     quote-include. The one documented exception is an include sitting inside an
     `#ifdef`/`#if defined(...)` region gated on a known debug/record-only flag
     (`DEBUG_ONLY_INCLUDE_FLAGS`, e.g. `CTPS_RECORD`, `PT_PARITY_TRACE`) -- such a
     flag is defined ONLY by a dedicated host CMake target (never by Arduino, never
     by the default ctest build), so the include is actually resolved through that
     target's `-I` list, not Arduino source-relative lookup; it is checked with the
     same candidate-root set as a test file instead.
   - A **`WRO2026/test/`** file is checked against a candidate-root set (its own
     directory, `${SKETCH_DIR}`, `${TEST_DIR}`, every mock/fixture dir, and the toy
     golden dir) since legitimate test includes reach `"Arduino.h"`, `"../Tasks.h"`,
     a mock header, etc. -- a sketch-dir-only rule would false-fail them. An include
     resolves if ANY candidate root contains the target.

   Every `#include "..."` line is walked regardless of `#ifdef`/build-flag state
   (see the plan's "Includes hidden behind build flags" note): a target gated behind
   a flag no CI target defines is otherwise checked by nothing.

2. **Operator root set** (`check_operator_root_set`, AC8): the set of files directly
   at the `WRO2026/` root (excluding the `config/`, `src/`, `test/` directories) must
   equal exactly the AC8 whitelist. This assertion is a NO-OP under the pre-move
   flat layout (`WRO2026/src/` does not exist yet) -- so the checker lands green
   before the move -- and becomes strict once `WRO2026/src/` exists.

Usage::

    python tools/reorg/check_includes.py

Exits ``0`` when both checks are clean, ``1`` on any violation.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import NamedTuple

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: File extensions scanned for `#include "..."` lines. Matches the extensions
#: actually used under `WRO2026/` (see CLAUDE.md's project layout).
SOURCE_EXTENSIONS = {".h", ".hpp", ".cpp", ".ino"}

#: Directory names that are never source -- build output / caches. Skipped
#: wherever they appear in a scanned path so a local `cmake --build` or a stray
#: `__pycache__` never pollutes the walk (CI's fresh checkout never has them).
SKIP_DIR_NAMES = {"build", "__pycache__", ".git"}

#: A quote-form `#include "target"` line, anchored to (optional leading
#: whitespace +) `#include`. Trailing content (a `// comment`) after the closing
#: quote is ignored by construction (`[^"]+` stops at the next `"`).
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')

#: Preprocessor conditional directives this script tracks well enough to know
#: whether an `#include` sits inside a gated region. This is NOT a full
#: preprocessor -- it only distinguishes "gated on one of DEBUG_ONLY_INCLUDE_FLAGS"
#: from everything else, which is all the include-resolution rule needs.
_IFDEF_RE = re.compile(r"^\s*#\s*ifdef\s+(\w+)")
_IFNDEF_RE = re.compile(r"^\s*#\s*ifndef\s+(\w+)")
_IF_DEFINED_RE = re.compile(r"^\s*#\s*if\s+defined\s*\(\s*(\w+)\s*\)")
_IF_RE = re.compile(r"^\s*#\s*if\b")
_ELIF_RE = re.compile(r"^\s*#\s*elif\b")
_ELSE_RE = re.compile(r"^\s*#\s*else\b")
_ENDIF_RE = re.compile(r"^\s*#\s*endif\b")

#: Macros known to be defined ONLY by a dedicated host CMake target (parity dump,
#: ctps record/replay) -- never by `arduino-cli compile`, never by the default
#: ctest build. An `#include` gated on one of these, even inside a production
#: file, is therefore resolved through that target's `-I` list rather than
#: Arduino source-relative lookup -- see the plan's "Includes hidden behind
#: build flags" note. Extend this set if a new debug/record-only flag is added.
DEBUG_ONLY_INCLUDE_FLAGS = {"CTPS_RECORD", "PT_PARITY_TRACE"}

#: Mock/fixture directory names under `WRO2026/test/` that are valid resolution
#: roots for a test file's `#include` (Contracts: "Host-test shadow invariant").
TEST_MOCK_DIR_NAMES = (
    "mocks",
    "hal_mocks",
    "path_mocks",
    "task_mocks",
    "mission_mocks",
    "ctps_mocks",
    "fixtures",
)

#: AC8 operator root whitelist -- the exact files that may sit directly at the
#: `WRO2026/` root once the move has happened (framework modules live under
#: `src/<domain>/`; `config/`/`src/`/`test/` are the only allowed subdirectories).
AC8_OPERATOR_ROOT_FILES = {
    "WRO2026.ino",
    "Config.h",
    "MissionScript.h",
    "MissionScript.cpp",
    "Tasks.h",
    "Tasks.cpp",
    "TasksTypes.h",
}
AC8_OPERATOR_ROOT_DIRS = {"config", "src", "test"}

#: Anti-silent-green sanity floors for `check_includes` (matches the plan's
#: Error Handling section, and mirrors the non-empty floor `check_host_coverage.py`
#: (T2) enforces). A walk that finds implausibly few files or includes almost
#: certainly means the walk itself is broken -- a future rename of `WRO2026/`, a
#: `SOURCE_EXTENSIONS` regression, or a `repo_root()`/`parents[N]` miscount --
#: NOT that the tree legitimately has zero source. `WRO2026/` holds 129 scanned
#: files (54 production, 75 test) totalling 307 quote-includes at the time this
#: floor was written; 30 is picked comfortably below either real count but far
#: above zero, so a broken walk fails loud (a violation, non-zero exit) instead
#: of silently reporting zero violations.
MIN_SOURCE_FILES_SCANNED = 30
MIN_INCLUDES_SEEN = 30


# ---------------------------------------------------------------------------
# Repository layout
# ---------------------------------------------------------------------------


def repo_root() -> Path:
    """Resolve the repository toplevel from this file's location.

    The script lives at ``tools/reorg/check_includes.py``, so ``parents[2]`` is
    the repo root. Resolved from ``__file__`` (not the cwd) so a CI step and a
    local invocation from any directory find the same files.

    :return: Absolute path to the repository root.
    """
    return Path(__file__).resolve().parents[2]


def _rel_display(path: Path, root: Path) -> str:
    """Render ``path`` relative to ``root`` (POSIX separators) for messages.

    Falls back to the absolute path if ``path`` is not under ``root`` (e.g. a
    candidate root that has since moved outside the repo -- defensive only).
    """
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


# ---------------------------------------------------------------------------
# Source enumeration
# ---------------------------------------------------------------------------


def _iter_source_files(sketch_dir: Path):
    """Yield every scannable source file under ``sketch_dir``, sorted.

    Scans ``SOURCE_EXTENSIONS`` recursively, skipping any path that passes
    through a ``SKIP_DIR_NAMES`` directory (build output, caches).
    """
    for path in sorted(sketch_dir.rglob("*")):
        if not path.is_file() or path.suffix not in SOURCE_EXTENSIONS:
            continue
        dir_parts = path.relative_to(sketch_dir).parts[:-1]
        if SKIP_DIR_NAMES.isdisjoint(dir_parts):
            yield path


def _is_test_file(rel_path: Path) -> bool:
    """True if ``rel_path`` (relative to ``WRO2026/``) sits under ``test/``."""
    return rel_path.parts[0] == "test"


# ---------------------------------------------------------------------------
# Include + conditional-gate extraction
# ---------------------------------------------------------------------------


def _extract_includes(path: Path) -> list[tuple[int, str, bool]]:
    """Return ``(line_number, target, debug_gated)`` for each quote-include in ``path``.

    ``debug_gated`` is True when the include sits inside an ``#ifdef`` / ``#if
    defined(...)`` region keyed on a name in :data:`DEBUG_ONLY_INCLUDE_FLAGS`.
    Nesting is tracked with a simple stack (one bool per open conditional level);
    ``#else`` flips the CURRENT level's gate state, ``#elif`` is treated
    conservatively as "not gated" (no re-evaluation of its condition -- no
    `#elif defined(<debug flag>)` exists in this repo today). An ``#ifndef`` is
    always "not gated" here: it is the normal/production branch, never the
    debug-only one.

    Note: `_IF_DEFINED_RE` only recognizes `#if defined(FLAG)` where the debug
    flag is the FIRST (and only matched) `defined()` token; a leading
    non-debug `defined()` in a compound `#if` (e.g. `#if defined(OTHER) ||
    defined(CTPS_RECORD)`) is not detected as gated. This is the safe failure
    direction -- it only makes the check STRICTER (treats the include as
    ordinary production code), never masks a real problem.

    :param path: Source file to scan.
    :return: One tuple per quote-include line, in file order.
    """
    text = path.read_text(encoding="utf-8", errors="replace")
    results: list[tuple[int, str, bool]] = []
    gate_stack: list[bool] = []

    for lineno, line in enumerate(text.splitlines(), start=1):
        ifdef_match = _IFDEF_RE.match(line)
        if_defined_match = _IF_DEFINED_RE.match(line)
        ifndef_match = _IFNDEF_RE.match(line)

        if ifdef_match:
            gate_stack.append(ifdef_match.group(1) in DEBUG_ONLY_INCLUDE_FLAGS)
            continue
        if if_defined_match:
            gate_stack.append(if_defined_match.group(1) in DEBUG_ONLY_INCLUDE_FLAGS)
            continue
        if ifndef_match:
            gate_stack.append(False)
            continue
        if _IF_RE.match(line):
            gate_stack.append(False)
            continue
        if _ELIF_RE.match(line):
            if gate_stack:
                gate_stack[-1] = False
            continue
        if _ELSE_RE.match(line):
            if gate_stack:
                gate_stack[-1] = not gate_stack[-1]
            continue
        if _ENDIF_RE.match(line):
            if gate_stack:
                gate_stack.pop()
            continue

        include_match = INCLUDE_RE.match(line)
        if include_match:
            results.append((lineno, include_match.group(1), any(gate_stack)))

    return results


# ---------------------------------------------------------------------------
# Candidate roots
# ---------------------------------------------------------------------------


def _test_candidate_roots(sketch_dir: Path, root: Path) -> list[Path]:
    """Static (file-independent) candidate roots for test-style resolution.

    Per Contracts: the including file's own dir (added by the caller per-file),
    ``${SKETCH_DIR}``, ``${TEST_DIR}``, every mock/fixture dir, and the toy
    golden dir (``tools/path_planner/tests/golden`` -- outside `WRO2026/`
    entirely, but a legitimate force-include root per `CMakeLists.txt`'s
    `TOY_ROUTES_DIR`).
    """
    test_dir = sketch_dir / "test"
    roots = [sketch_dir, test_dir]
    roots.extend(test_dir / name for name in TEST_MOCK_DIR_NAMES)
    roots.append(root / "tools" / "path_planner" / "tests" / "golden")
    return roots


# ---------------------------------------------------------------------------
# Check 1: include resolution
# ---------------------------------------------------------------------------


class IncludeScanStats(NamedTuple):
    """Counts from one `check_includes` walk (anti-silent-green: printed in the
    report so a reader/CI log can see the walk actually visited files, not just
    read an unqualified "OK")."""

    files_scanned: int
    production_files_scanned: int
    test_files_scanned: int
    includes_seen: int
    production_includes_seen: int
    test_includes_seen: int


def check_includes(sketch_dir: Path, root: Path) -> tuple[list[str], IncludeScanStats]:
    """Verify every ``#include "..."`` under ``sketch_dir`` resolves correctly.

    Walks every scannable file, classifies each quote-include per the rule
    (production strict source-relative / test candidate-root / debug-gated
    candidate-root -- see module docstring), and reports one violation per
    include that resolves under none of its applicable roots. Also tallies how
    many files and includes the walk actually saw, and fails loud (rather than
    silently reporting zero violations) if either count is implausibly low --
    see :data:`MIN_SOURCE_FILES_SCANNED` / :data:`MIN_INCLUDES_SEEN`.

    :param sketch_dir: The ``WRO2026`` sketch directory.
    :param root: The repository root (for the toy-golden candidate root).
    :return: ``(violations, stats)`` -- ``violations`` is empty on a clean pass.
    """
    violations: list[str] = []
    static_test_roots = _test_candidate_roots(sketch_dir, root)

    files_scanned = 0
    production_files_scanned = 0
    test_files_scanned = 0
    includes_seen = 0
    production_includes_seen = 0
    test_includes_seen = 0

    for path in _iter_source_files(sketch_dir):
        rel = path.relative_to(sketch_dir)
        is_test = _is_test_file(rel)

        files_scanned += 1
        if is_test:
            test_files_scanned += 1
        else:
            production_files_scanned += 1

        file_includes = _extract_includes(path)
        includes_seen += len(file_includes)
        if is_test:
            test_includes_seen += len(file_includes)
        else:
            production_includes_seen += len(file_includes)

        for lineno, target, debug_gated in file_includes:
            if is_test or debug_gated:
                candidate_roots = [path.parent] + static_test_roots
                if any((candidate / target).is_file() for candidate in candidate_roots):
                    continue
                root_labels = sorted(
                    {_rel_display(candidate, root) or "." for candidate in candidate_roots}
                )
                reason = (
                    "debug-gated include" if (debug_gated and not is_test) else "test include"
                )
                violations.append(
                    f"{rel.as_posix()}:{lineno}: {reason} \"{target}\" does not resolve "
                    f"against any candidate root ({', '.join(root_labels)})"
                )
            else:
                resolved = path.parent / target
                if resolved.is_file():
                    continue
                own_dir = _rel_display(path.parent, root) or "."
                violations.append(
                    f"{rel.as_posix()}:{lineno}: production #include \"{target}\" does not "
                    f"resolve source-relative to {own_dir}/ (Arduino quote-include rule)"
                )

    # Anti-silent-green floors: an empty (or implausibly small) walk must be a
    # violation, not a vacuous pass -- see MIN_SOURCE_FILES_SCANNED's docstring.
    if files_scanned < MIN_SOURCE_FILES_SCANNED:
        violations.append(
            f"sanity floor breach: only {files_scanned} source file(s) were scanned under "
            f"{_rel_display(sketch_dir, root) or '.'}/ (expected at least "
            f"{MIN_SOURCE_FILES_SCANNED} -- this usually means the walk itself is broken "
            "(WRO2026/ renamed, SOURCE_EXTENSIONS regressed, or repo_root()/parents[N] "
            "miscounting), not that the tree legitimately has almost no source)"
        )
    if includes_seen < MIN_INCLUDES_SEEN:
        violations.append(
            f"sanity floor breach: only {includes_seen} #include \"...\" line(s) were seen "
            f"across {files_scanned} scanned file(s) (expected at least {MIN_INCLUDES_SEEN} -- "
            "same broken-walk risk as scanning zero files, just caught from the include side)"
        )

    stats = IncludeScanStats(
        files_scanned=files_scanned,
        production_files_scanned=production_files_scanned,
        test_files_scanned=test_files_scanned,
        includes_seen=includes_seen,
        production_includes_seen=production_includes_seen,
        test_includes_seen=test_includes_seen,
    )
    return violations, stats


# ---------------------------------------------------------------------------
# Check 2: operator root set (AC8)
# ---------------------------------------------------------------------------


def check_operator_root_set(sketch_dir: Path) -> tuple[list[str], bool]:
    """Verify the `WRO2026/` root holds exactly the AC8 operator whitelist.

    A NO-OP (returns no violations, ``enforced=False``) under the pre-move flat
    layout -- detected by the absence of ``WRO2026/src/`` -- per the plan's
    requirement that this checker "lands green under the old layout". Once
    ``WRO2026/src/`` exists, enforcement is strict: every non-directory entry at
    the root must be in :data:`AC8_OPERATOR_ROOT_FILES` (no more, no less), and
    every directory entry must be in :data:`AC8_OPERATOR_ROOT_DIRS`.

    :param sketch_dir: The ``WRO2026`` sketch directory.
    :return: ``(violations, enforced)`` -- ``enforced`` is False when this check
        was a pre-move no-op.
    """
    if not (sketch_dir / "src").is_dir():
        return [], False

    violations: list[str] = []
    observed_files: set[str] = set()

    for entry in sketch_dir.iterdir():
        if entry.is_dir():
            if entry.name not in AC8_OPERATOR_ROOT_DIRS:
                violations.append(
                    f"unexpected directory at WRO2026/ root: {entry.name}/ (post-move root "
                    f"directories must be exactly {sorted(AC8_OPERATOR_ROOT_DIRS)})"
                )
            continue
        observed_files.add(entry.name)

    missing = AC8_OPERATOR_ROOT_FILES - observed_files
    extra = observed_files - AC8_OPERATOR_ROOT_FILES
    for name in sorted(missing):
        violations.append(f"operator root set is missing required file: {name}")
    for name in sorted(extra):
        violations.append(
            f"operator root set has unexpected file: {name} (framework modules must "
            f"live under src/<domain>/)"
        )

    return violations, True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _print_check(header: str, violations: list[str], info_lines: list[str] | None = None) -> None:
    """Print a per-check header, optional info lines, then each violation.

    :param info_lines: Non-violation context (e.g. scan counts) printed before
        the OK/VIOLATION lines so a reader/CI log can see that scanning
        actually happened, not just read an unqualified "OK".
    """
    print(f"== {header} ==")
    for line in info_lines or []:
        print(f"  {line}")
    if violations:
        for violation in violations:
            print(f"  VIOLATION: {violation}")
    else:
        print("  OK")


def main(argv: list[str] | None = None) -> int:
    """Run both checks and report the combined result.

    :param argv: Unused; accepted for a conventional CLI signature.
    :return: ``0`` when both checks are clean, ``1`` on any violation.
    """
    root = repo_root()
    sketch_dir = root / "WRO2026"

    include_violations, include_stats = check_includes(sketch_dir, root)
    operator_violations, operator_enforced = check_operator_root_set(sketch_dir)

    _print_check(
        "Include resolution (AC7)",
        include_violations,
        info_lines=[
            f"scanned {include_stats.files_scanned} source file(s) "
            f"({include_stats.production_files_scanned} production, "
            f"{include_stats.test_files_scanned} test); saw {include_stats.includes_seen} "
            f'#include "..." line(s) ({include_stats.production_includes_seen} production, '
            f"{include_stats.test_includes_seen} test)"
        ],
    )
    if operator_enforced:
        _print_check("Operator root set (AC8)", operator_violations)
    else:
        print("== Operator root set (AC8) ==")
        print("  SKIP (pre-move flat layout: WRO2026/src/ does not exist yet)")

    all_violations = include_violations + operator_violations
    if all_violations:
        print(
            f"FAIL: {len(include_violations)} include-resolution, "
            f"{len(operator_violations)} operator-root-set violations"
        )
        return 1

    print(
        f"OK: every #include \"...\" under WRO2026/ resolves; operator root set "
        f"{'enforced' if operator_enforced else 'skipped (pre-move layout)'}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
