#!/usr/bin/env python3
"""Static CI guards for the WRO2026 host-test harness (epic #107).

This stdlib-only script runs two independent text checks against the working
tree and exits non-zero on any violation:

1. **No-new-mirror guard** (``check_mirrors``): the observed per-file count of
   ``MIRROR:`` replica annotations under ``WRO2026/test/`` must exactly equal a
   committed allowlist (``mirror_allowlist.json``). A rise, a drop, a new file,
   or a stale entry all fail. A ``MIRROR:`` comment that cites no concrete
   reference (and is not known documentation prose) fails as well.

2. **Host-coverage guard** (``check_coverage``): every production
   ``WRO2026/*.cpp`` must be host-linked in a source-bearing ``CMakeLists.txt``
   command, or be listed in a committed allowlist
   (``host_coverage_allowlist.json``). Allowlist entries that have since become
   linked, or that no longer exist, also fail.

Both checks are pure text parsing -- no CMake or compiler is invoked. The two
checks run independently: an input error (missing/malformed allowlist, missing
CMakeLists) for one is reported as that check's violation while the other still
runs, so a single CI run surfaces every problem at once.

Usage::

    python WRO2026/test/check_host_coverage.py

Exits ``0`` when both guards are green, ``1`` on any violation.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants -- the load-bearing patterns. See the plan's "Contracts & Interfaces".
# ---------------------------------------------------------------------------

#: A genuine mirror annotation: a ``//``/``///`` line comment or ``*``/``/*``
#: block-comment marker carrying a ``MIRROR:`` tag followed -- anywhere on the
#: line -- by a concrete reference (``.cpp``/``.h``:digit citation or a
#: ``Class::member``). Applied PER LINE so ``.`` never crosses a newline. The
#: concrete-ref requirement excludes convention prose that uses a ``<line>``
#: placeholder or a backtick-quoted example with no real citation.
MIRROR_RE = re.compile(r"(?://+|\*).*?\bMIRROR:.*?(?:\.(?:cpp|h):\d|\w::\w)")

#: Known-prose signature: a backtick-quoted comment marker immediately before
#: ``MIRROR:``, or a ``<...>`` placeholder after it. Used to whitelist the
#: documentation lines so they do not trip the ref-less hardening.
PROSE_RE = re.compile(r"`\s*//+\s*MIRROR:|MIRROR:[^\n]*<[^>]+>")

#: A host-linked production source as it appears in CMake: ``${SKETCH_DIR}/<name>.cpp``.
#: Captures the bare ``<name>`` (the ``.cpp`` suffix is re-appended by the caller).
LINK_RE = re.compile(r"\$\{SKETCH_DIR\}/([A-Za-z_][A-Za-z0-9_]*)\.cpp")

#: CMake commands whose argument bodies may legitimately list ``${SKETCH_DIR}/*.cpp``.
SOURCE_COMMANDS = ("add_executable", "target_sources", "set")


class AllowlistError(Exception):
    """Raised by :func:`load_allowlist` on any schema or read/parse violation."""


# ---------------------------------------------------------------------------
# Repository layout
# ---------------------------------------------------------------------------


def repo_root() -> Path:
    """Resolve the repository toplevel from this file's location.

    The script lives at ``WRO2026/test/check_host_coverage.py``, so
    ``parents[2]`` is the repo root. Resolved from ``__file__`` (not the cwd) so
    the CI step and the pytest import both find the same files.

    :return: Absolute path to the repository root.
    """
    return Path(__file__).resolve().parents[2]


# ---------------------------------------------------------------------------
# Mirror scan
# ---------------------------------------------------------------------------


def _iter_source_files(test_dir: Path):
    """Yield every ``*.cpp``/``*.h`` file under ``test_dir`` in sorted order."""
    files = set(test_dir.rglob("*.cpp")) | set(test_dir.rglob("*.h"))
    for path in sorted(files):
        if path.is_file():
            yield path


def count_mirrors(test_dir: Path) -> dict[str, int]:
    """Count genuine ``MIRROR:`` annotations per source file under ``test_dir``.

    Scans ``*.cpp``/``*.h`` recursively, applying :data:`MIRROR_RE` LINE BY LINE
    so the position-independent regex never matches across a newline.

    :param test_dir: The ``WRO2026/test`` directory to scan.
    :return: ``{test_relative_posix_path: match_count}`` for files with >=1
        match. Files with zero matches are omitted.
    """
    counts: dict[str, int] = {}
    for path in _iter_source_files(test_dir):
        text = path.read_text(encoding="utf-8")
        count = sum(1 for line in text.splitlines() if MIRROR_RE.search(line))
        if count:
            counts[path.relative_to(test_dir).as_posix()] = count
    return counts


def find_refless_mirrors(test_dir: Path) -> list[str]:
    """Find ``MIRROR:`` comment lines that cite no concrete reference.

    A "comment line carrying a ``MIRROR:`` tag" is one that contains ``MIRROR:``
    and a ``//`` or ``*`` comment marker. Such a line fails when it matches
    NEITHER :data:`MIRROR_RE` (a concrete ref) NOR :data:`PROSE_RE` (a known
    documentation signature) -- closing the evasion where a new replica is tagged
    ``// MIRROR: copied logic`` with no ``file:line`` citation.

    :param test_dir: The ``WRO2026/test`` directory to scan.
    :return: One violation message per offending line (empty == none).
    """
    violations: list[str] = []
    for path in _iter_source_files(test_dir):
        rel = path.relative_to(test_dir).as_posix()
        text = path.read_text(encoding="utf-8")
        for lineno, line in enumerate(text.splitlines(), start=1):
            if "MIRROR:" not in line:
                continue
            if "//" not in line and "*" not in line:
                continue
            if MIRROR_RE.search(line) or PROSE_RE.search(line):
                continue
            violations.append(
                f"{rel}:{lineno}: mirror annotation missing concrete reference "
                f"-- cite file:line"
            )
    return violations


# ---------------------------------------------------------------------------
# Allowlist loading + strict schema validation
# ---------------------------------------------------------------------------


def load_allowlist(path: Path, *, with_count: bool) -> list[dict]:
    """Read and strictly schema-validate a JSON allowlist.

    Envelope must be ``{"entries": [ ... ]}`` with ``entries`` a list of dicts.
    Each entry requires a non-empty ``file`` string (unique across the file) and
    a non-empty ``reason`` string; ``issue`` must be a string or ``null`` (or
    absent). When ``with_count`` is true each entry additionally requires an
    integer ``count`` >= 1 (a ``bool`` is rejected even though it is an ``int``
    subclass).

    :param path: The allowlist JSON file.
    :param with_count: Whether to require + validate a ``count`` field.
    :return: The list of validated entry dicts.
    :raises AllowlistError: On any read, parse, envelope, or schema violation.
    """
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise AllowlistError(f"cannot read file ({exc})") from exc

    try:
        data = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise AllowlistError(f"malformed JSON ({exc})") from exc

    if not isinstance(data, dict) or "entries" not in data:
        raise AllowlistError("envelope must be an object with an 'entries' key")

    entries = data["entries"]
    if not isinstance(entries, list):
        raise AllowlistError("'entries' must be a list")

    seen_files: set[str] = set()
    validated: list[dict] = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise AllowlistError(f"entry #{index} is not an object")

        file = entry.get("file")
        if not isinstance(file, str) or not file:
            raise AllowlistError(
                f"entry #{index} has a missing or empty 'file'"
            )
        if file in seen_files:
            raise AllowlistError(f"duplicate 'file' entry: {file!r}")
        seen_files.add(file)

        reason = entry.get("reason")
        if not isinstance(reason, str) or not reason:
            raise AllowlistError(
                f"entry {file!r} has a missing or empty 'reason'"
            )

        issue = entry.get("issue", None)
        if issue is not None and not isinstance(issue, str):
            raise AllowlistError(
                f"entry {file!r} has an 'issue' that is neither a string nor null"
            )

        if with_count:
            if "count" not in entry:
                raise AllowlistError(f"entry {file!r} is missing 'count'")
            count = entry["count"]
            # isinstance(True, int) is True, so reject bool explicitly.
            if isinstance(count, bool) or not isinstance(count, int):
                raise AllowlistError(
                    f"entry {file!r} has a non-integer 'count'"
                )
            if count < 1:
                raise AllowlistError(f"entry {file!r} has a 'count' < 1")

        validated.append(entry)

    return validated


# ---------------------------------------------------------------------------
# Mirror check
# ---------------------------------------------------------------------------


def check_mirrors(test_dir: Path, allowlist_path: Path) -> list[str]:
    """Verify the observed mirror map exactly equals the committed allowlist.

    Fails when (a) the observed ``{file: count}`` map differs from the allowlist
    map -- reporting rises, drops, new files, and stale entries each with the
    expected-vs-found counts; OR (b) :func:`find_refless_mirrors` reports any;
    OR (c) the allowlist fails to load/validate (the :class:`AllowlistError` is
    converted to a violation, not raised out).

    :param test_dir: The ``WRO2026/test`` directory to scan.
    :param allowlist_path: Path to ``mirror_allowlist.json``.
    :return: Violation messages (empty == pass).
    """
    try:
        entries = load_allowlist(allowlist_path, with_count=True)
    except AllowlistError as exc:
        return [f"cannot use {allowlist_path}: {exc}"]

    allow_map: dict[str, int] = {entry["file"]: entry["count"] for entry in entries}
    observed = count_mirrors(test_dir)

    violations: list[str] = []
    for file in sorted(set(allow_map) | set(observed)):
        expected = allow_map.get(file)
        found = observed.get(file, 0)
        if expected is None:
            violations.append(
                f"{file}: {found} MIRROR annotation(s) found but file is not in "
                f"the mirror allowlist"
            )
        elif found == 0:
            violations.append(
                f"{file}: stale mirror allowlist entry -- expected {expected}, "
                f"found 0 (no MIRROR annotations)"
            )
        elif found != expected:
            violations.append(
                f"{file}: mirror count mismatch -- expected {expected}, "
                f"found {found}"
            )

    violations.extend(find_refless_mirrors(test_dir))
    return violations


# ---------------------------------------------------------------------------
# CMake parsing + coverage check
# ---------------------------------------------------------------------------


def _strip_cmake_comments(text: str) -> str:
    """Drop everything from the first ``#`` to end-of-line, per line.

    Handles whole-line and inline ``#`` comments so a ``${SKETCH_DIR}/X.cpp``
    mentioned only in a comment is not mistaken for a linked source.

    :param text: Raw ``CMakeLists.txt`` contents.
    :return: The text with comments removed, lines rejoined with ``\\n``.
    """
    stripped_lines = []
    for line in text.splitlines():
        hash_index = line.find("#")
        if hash_index != -1:
            line = line[:hash_index]
        stripped_lines.append(line)
    return "\n".join(stripped_lines)


def _command_bodies(text: str, names: tuple[str, ...]) -> str:
    """Return the concatenated argument bodies of named CMake commands.

    Scans for an identifier immediately followed by ``(`` whose (case-insensitive)
    name is in ``names``, then reads to the matching ``)`` with paren-depth
    tracking, so a multi-line ``set()`` / ``target_sources()`` body is captured
    whole. Pure text -- no cmake is invoked.

    :param text: The (comment-stripped) CMake text to scan.
    :param names: Command names whose bodies to collect (matched case-insensitively).
    :return: All matched bodies joined by ``\\n``.
    """
    wanted = {name.lower() for name in names}
    length = len(text)
    bodies: list[str] = []

    for match in re.finditer(r"([A-Za-z_]\w*)\s*\(", text):
        if match.group(1).lower() not in wanted:
            continue
        body_start = match.end()
        depth = 1
        index = body_start
        while index < length:
            char = text[index]
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    break
            index += 1
        bodies.append(text[body_start:index])

    return "\n".join(bodies)


def linked_sources(cmakelists_path: Path) -> set[str]:
    """Collect the production ``.cpp`` basenames host-linked in CMakeLists.

    Comment-strips the file first, then captures :data:`LINK_RE` basenames found
    only inside the bodies of source-bearing commands (:data:`SOURCE_COMMANDS`).

    :param cmakelists_path: Path to ``WRO2026/test/CMakeLists.txt``.
    :return: The set of linked ``<name>.cpp`` basenames (with the ``.cpp`` suffix).
    """
    text = cmakelists_path.read_text(encoding="utf-8")
    bodies = _command_bodies(_strip_cmake_comments(text), SOURCE_COMMANDS)
    return {f"{name}.cpp" for name in LINK_RE.findall(bodies)}


def check_coverage(
    prod_dir: Path, cmakelists_path: Path, allowlist_path: Path
) -> list[str]:
    """Verify every production ``.cpp`` is host-linked or allowlisted.

    The production set is ``prod_dir.glob("*.cpp")`` (NON-recursive). A source is
    OK when it is linked (see :func:`linked_sources`) or allowlisted. Fails on an
    un-linked-and-un-allowlisted source, on a stale allowlist entry that is now
    linked, and on a stale allowlist entry whose file no longer exists. The
    allowlist :class:`AllowlistError` and a missing CMakeLists are each converted
    to a clear violation rather than raised out.

    :param prod_dir: The ``WRO2026`` sketch directory holding the production sources.
    :param cmakelists_path: Path to ``WRO2026/test/CMakeLists.txt``.
    :param allowlist_path: Path to ``host_coverage_allowlist.json``.
    :return: Violation messages (empty == pass).
    """
    try:
        entries = load_allowlist(allowlist_path, with_count=False)
    except AllowlistError as exc:
        return [f"cannot use {allowlist_path}: {exc}"]

    if not cmakelists_path.is_file():
        return [
            f"cannot use {cmakelists_path}: file not found "
            f"(repo-layout assumption broken)"
        ]
    try:
        linked = linked_sources(cmakelists_path)
    except OSError as exc:
        return [f"cannot use {cmakelists_path}: cannot read file ({exc})"]

    allowed = {entry["file"] for entry in entries}
    production = {path.name for path in prod_dir.glob("*.cpp")}

    violations: list[str] = []

    for name in sorted(production):
        if name in linked or name in allowed:
            continue
        violations.append(
            f"{name}: production .cpp is neither host-linked nor in the coverage "
            f"allowlist"
        )

    for entry in entries:
        name = entry["file"]
        if name in linked:
            violations.append(
                f"{name}: stale coverage allowlist entry -- file is now "
                f"host-linked; remove it from the allowlist"
            )
        elif name not in production:
            violations.append(
                f"{name}: stale coverage allowlist entry -- file no longer "
                f"exists in {prod_dir.name}/"
            )

    return violations


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def _print_check(header: str, violations: list[str]) -> None:
    """Print a per-check header and each violation on its own line."""
    print(f"== {header} ==")
    if violations:
        for violation in violations:
            print(f"  VIOLATION: {violation}")
    else:
        print("  OK")


def main(argv: list[str] | None = None) -> int:
    """Run both guards independently and report the combined result.

    Each check runs in its own ``try``/``except`` so an unexpected input error in
    one is turned into that check's violation while the other still runs.

    :param argv: Unused; accepted for a conventional CLI signature.
    :return: ``0`` when both guards are clean, ``1`` on any violation.
    """
    root = repo_root()
    test_dir = root / "WRO2026" / "test"
    prod_dir = root / "WRO2026"
    cmakelists_path = test_dir / "CMakeLists.txt"
    mirror_allowlist_path = test_dir / "mirror_allowlist.json"
    coverage_allowlist_path = test_dir / "host_coverage_allowlist.json"

    try:
        mirror_violations = check_mirrors(test_dir, mirror_allowlist_path)
    except Exception as exc:  # defensive: never let one check abort the other.
        mirror_violations = [f"mirror check failed unexpectedly: {exc}"]

    try:
        coverage_violations = check_coverage(
            prod_dir, cmakelists_path, coverage_allowlist_path
        )
    except Exception as exc:  # defensive: never let one check abort the other.
        coverage_violations = [f"coverage check failed unexpectedly: {exc}"]

    _print_check("Mirror guard", mirror_violations)
    _print_check("Host-coverage guard", coverage_violations)

    if mirror_violations or coverage_violations:
        print(
            f"FAIL: {len(mirror_violations)} mirror, "
            f"{len(coverage_violations)} coverage violations"
        )
        return 1

    observed = count_mirrors(test_dir)
    baseline = sum(observed.values())
    production = {path.name for path in prod_dir.glob("*.cpp")}
    linked_count = len(production & linked_sources(cmakelists_path))
    allowlisted_count = len(production) - linked_count
    print(
        f"OK: mirror baseline {baseline} across {len(observed)} files; "
        f"{linked_count} production .cpp linked, {allowlisted_count} allowlisted"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
