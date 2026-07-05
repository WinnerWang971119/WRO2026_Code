#!/usr/bin/env python3
"""Static CI guards for the WRO2026 host-test harness (epic #107).

This stdlib-only script runs two independent text checks against the working
tree and exits non-zero on any violation:

1. **No-new-mirror guard** (``check_mirrors``): the observed per-file count of
   ``MIRROR:`` replica annotations under ``WRO2026/test/`` must exactly equal a
   committed allowlist (``mirror_allowlist.json``). A rise, a drop, a new file,
   or a stale entry all fail. A ``MIRROR:`` comment that cites no concrete
   reference (and is not known documentation prose) fails as well.

2. **Host-coverage guard** (``check_coverage``): every production ``.cpp`` under
   ``WRO2026/`` (discovered **recursively**, excluding ``test/``, so it finds
   sources both at the flat sketch root and nested under ``src/<domain>/``) must
   be host-linked in a source-bearing ``CMakeLists.txt`` command, or be listed
   in a committed allowlist (``host_coverage_allowlist.json``). The production
   set and the linked set are keyed by **sketch-relative path** (not basename),
   so two same-named sources in different domains cannot collide and hide a gap;
   a duplicate-basename guard fails loudly if that ever happens. The allowlist
   stays **basename-keyed** (layout-independent) and is matched against the
   basenames of the un-linked production set. Allowlist entries that have since
   become linked, or that no longer exist, also fail. Anti-vacuous-pass sanity
   floors reject a run where the production set OR the ``linked & production``
   intersection is empty (a glob/``LINK_RE`` bug would otherwise pass silently
   green).

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

#: A concrete reference token: a ``file.cpp``/``file.h``:digit citation or a
#: ``Class::member`` qualified name. Marks a ``MIRROR:`` comment as genuinely
#: cited; also spares a bare continuation line that names its source from the
#: ref-less hardening.
_REF = r"(?:\.(?:cpp|h):\d|\w::\w)"
REF_RE = re.compile(_REF)

#: A genuine mirror annotation on ONE line: a ``//``/``///`` line comment or
#: ``*``/``/*`` block-comment marker, a ``MIRROR:`` tag, and a concrete
#: reference -- in EITHER order (the citation may sit before or after the tag).
#: Applied PER LINE so ``.`` never crosses a newline, and matched per occurrence
#: (``findall``) so two annotations on one line count as two. The concrete-ref
#: requirement excludes convention prose that uses a ``<line>`` placeholder or a
#: backtick-quoted example with no real citation.
MIRROR_RE = re.compile(
    r"(?://+|\*).*?(?:\bMIRROR:.*?" + _REF + r"|" + _REF + r".*?\bMIRROR:)"
)

#: Known-prose signature: a backtick-quoted comment marker immediately before
#: ``MIRROR:``, or the literal ``<line>`` placeholder after it. Whitelists the
#: convention-doc lines so they do not trip the ref-less hardening. The
#: placeholder is the LITERAL token ``<line>`` (not any ``<...>``), so a real
#: replica annotated with an unrelated template type (e.g. ``test<Foo>``) is NOT
#: whitelisted -- that evasion is the regression this tightening closes.
PROSE_RE = re.compile(r"`\s*//+\s*MIRROR:|MIRROR:[^\n]*<line>")

#: A host-linked production source as it appears in CMake:
#: ``${SKETCH_DIR}/<path>.cpp`` where ``<path>`` is a bare name at the flat root
#: (``${SKETCH_DIR}/Drive.cpp``) OR a nested ``src/<domain>/<name>`` after the
#: reorg (``${SKETCH_DIR}/src/drive/Drive.cpp``). Captures the sketch-relative
#: stem (``/``-separated identifier segments); the ``.cpp`` suffix is re-appended
#: by the caller, yielding the sketch-relative path used as the comparison key.
LINK_RE = re.compile(
    r"\$\{SKETCH_DIR\}/((?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*)\.cpp"
)

#: CMake commands whose argument bodies may legitimately list ``${SKETCH_DIR}/*.cpp``.
#: ``set``/``list`` capture ``set(VAR ...)`` / ``list(APPEND VAR ...)`` source
#: declarations; ``add_executable``/``add_library``/``target_sources`` the direct
#: wiring. As with any pure-text match, a source named only inside an unexpanded
#: ``set``/``list`` variable still reads as linked (accepted residual limit).
SOURCE_COMMANDS = ("add_executable", "add_library", "target_sources", "set", "list")


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


def _is_mirror_comment_line(line: str) -> bool:
    """True if ``line`` carries a ``MIRROR:`` tag in a comment context.

    A comment context is a ``//`` or ``*`` marker anywhere on the line, OR the
    line's first non-space token being ``MIRROR:`` -- a bare ``/* ... */``
    continuation line (e.g. ``   MIRROR: copied loop``) that carries no marker of
    its own. Catching the bare form closes the evasion where a new replica is
    written on an unmarked block-comment line.
    """
    if "MIRROR:" not in line:
        return False
    return "//" in line or "*" in line or re.match(r"\s*MIRROR:", line) is not None


def _count_mirrors_on_line(line: str) -> int:
    """Count genuine, cited mirror annotations on a single ``line``.

    Marker-led annotations are counted per occurrence (a line may legitimately
    carry more than one). A bare block-comment continuation line (no ``//``/``*``)
    that begins with ``MIRROR:`` and cites a concrete reference counts as one.
    """
    count = len(MIRROR_RE.findall(line))
    if (
        count == 0
        and "//" not in line
        and "*" not in line
        and re.match(r"\s*MIRROR:", line) is not None
        and REF_RE.search(line) is not None
    ):
        count = 1
    return count


def count_mirrors(test_dir: Path) -> dict[str, int]:
    """Count genuine ``MIRROR:`` annotations per source file under ``test_dir``.

    Scans ``*.cpp``/``*.h`` recursively, counting PER ANNOTATION on each line (so
    two citations on one line count as two) via :func:`_count_mirrors_on_line`.
    Files are read with ``errors="replace"`` so a stray non-UTF-8 byte degrades to
    U+FFFD rather than crashing the scan.

    :param test_dir: The ``WRO2026/test`` directory to scan.
    :return: ``{test_relative_posix_path: match_count}`` for files with >=1
        match. Files with zero matches are omitted.
    """
    counts: dict[str, int] = {}
    for path in _iter_source_files(test_dir):
        text = path.read_text(encoding="utf-8", errors="replace")
        count = sum(_count_mirrors_on_line(line) for line in text.splitlines())
        if count:
            counts[path.relative_to(test_dir).as_posix()] = count
    return counts


def find_refless_mirrors(test_dir: Path) -> list[str]:
    """Find ``MIRROR:`` comment lines that cite no concrete reference.

    A "comment line carrying a ``MIRROR:`` tag" is recognised by
    :func:`_is_mirror_comment_line` (a ``//``/``*`` marker, or a bare block-comment
    continuation that starts with ``MIRROR:``). Such a line fails when it matches
    NONE of :data:`MIRROR_RE` (a cited annotation), :data:`PROSE_RE` (a known
    documentation signature), or :data:`REF_RE` (any concrete citation elsewhere
    on the line) -- closing the evasion where a new replica is tagged
    ``// MIRROR: copied logic`` with no ``file:line`` citation, while sparing a
    line that cites its source before the tag or on an unmarked continuation line.

    :param test_dir: The ``WRO2026/test`` directory to scan.
    :return: One violation message per offending line (empty == none).
    """
    violations: list[str] = []
    for path in _iter_source_files(test_dir):
        rel = path.relative_to(test_dir).as_posix()
        text = path.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), start=1):
            if not _is_mirror_comment_line(line):
                continue
            if MIRROR_RE.search(line) or PROSE_RE.search(line) or REF_RE.search(line):
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
    """Drop each line's first UNQUOTED ``#`` comment to end-of-line.

    Tracks double-quote state per line before cutting, so a ``#`` inside a quoted
    argument (a ``"-DTAG=#1"`` define, a path) is not mistaken for a comment and a
    following ``${SKETCH_DIR}/X.cpp`` on the same line survives. Bracket comments
    (``#[[ ]]``) and escaped quotes are out of scope -- no CMakeLists line in this
    repo uses them.

    :param text: Raw ``CMakeLists.txt`` contents.
    :return: The text with comments removed, lines rejoined with ``\\n``.
    """
    stripped_lines = []
    for line in text.splitlines():
        in_quote = False
        cut = len(line)
        for index, char in enumerate(line):
            if char == '"':
                in_quote = not in_quote
            elif char == "#" and not in_quote:
                cut = index
                break
        stripped_lines.append(line[:cut])
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
    """Collect the production ``.cpp`` sketch-relative paths host-linked in CMake.

    Comment-strips the file first, then captures :data:`LINK_RE` stems found only
    inside the bodies of source-bearing commands (:data:`SOURCE_COMMANDS`) and
    re-appends the ``.cpp`` suffix. A flat ``${SKETCH_DIR}/Drive.cpp`` yields
    ``"Drive.cpp"``; a nested ``${SKETCH_DIR}/src/drive/Drive.cpp`` yields
    ``"src/drive/Drive.cpp"`` -- the same sketch-relative key
    :func:`production_sources` produces, so the two sets compare directly.

    :param cmakelists_path: Path to ``WRO2026/test/CMakeLists.txt``.
    :return: The set of linked sketch-relative ``<path>.cpp`` POSIX paths.
    """
    text = cmakelists_path.read_text(encoding="utf-8", errors="replace")
    bodies = _command_bodies(_strip_cmake_comments(text), SOURCE_COMMANDS)
    return {f"{stem}.cpp" for stem in LINK_RE.findall(bodies)}


def production_sources(prod_dir: Path) -> set[str]:
    """Discover every production ``.cpp`` under ``prod_dir`` except those in ``test/``.

    Walks ``prod_dir`` **recursively** (``rglob``) so a source is found whether
    it sits at the flat sketch root (today) or nested under ``src/<domain>/``
    (after the reorg). Anything under the ``test/`` harness subtree -- including
    its ``build/`` artifacts -- is skipped. Paths are keyed relative to
    ``prod_dir`` (the sketch dir, i.e. ``${SKETCH_DIR}``) as POSIX strings, so
    they compare directly against :func:`linked_sources`.

    :param prod_dir: The ``WRO2026`` sketch directory.
    :return: Sketch-relative POSIX paths of every non-test production ``.cpp``.
    """
    sources: set[str] = set()
    for path in prod_dir.rglob("*.cpp"):
        if not path.is_file():
            continue
        rel = path.relative_to(prod_dir)
        if rel.parts and rel.parts[0] == "test":
            continue
        sources.add(rel.as_posix())
    return sources


def check_coverage(
    prod_dir: Path, cmakelists_path: Path, allowlist_path: Path
) -> list[str]:
    """Verify every production ``.cpp`` is host-linked or allowlisted.

    The production set is :func:`production_sources` (recursive under ``prod_dir``,
    excluding ``test/``) and the linked set is :func:`linked_sources`; both are
    keyed by sketch-relative path, so the layout (flat root vs nested
    ``src/<domain>/``) does not affect the comparison. A source is OK when its
    path is linked, or when its basename is in the (basename-keyed) allowlist.

    Fails on an un-linked-and-un-allowlisted source, on a stale allowlist entry
    that is now linked, and on a stale allowlist entry whose file no longer
    exists. Two hardening guards run first so the check can never pass vacuously:
    a **duplicate-basename guard** (two production files sharing a basename would
    make the basename-keyed allowlist match ambiguous), and **anti-vacuous-pass
    sanity floors** (an empty production set, or an empty ``linked & production``
    intersection, signals a broken glob / ``LINK_RE`` and must go red rather than
    green). The
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
    production = production_sources(prod_dir)

    violations: list[str] = []

    # --- Duplicate-basename guard --------------------------------------------
    # The allowlist is basename-keyed (layout-independent), so two production
    # files sharing a basename in different domains would make an allowlist match
    # ambiguous and could hide a coverage gap. Fail loudly if that ever happens.
    by_basename: dict[str, list[str]] = {}
    for rel in production:
        by_basename.setdefault(Path(rel).name, []).append(rel)
    for basename, paths in sorted(by_basename.items()):
        if len(paths) > 1:
            violations.append(
                f"{basename}: duplicate production basename across "
                f"{', '.join(sorted(paths))} -- basename-keyed coverage cannot "
                f"disambiguate; rename one or key that pair by path"
            )

    # --- Anti-vacuous-pass sanity floors -------------------------------------
    # A bug in the recursive glob or in LINK_RE that empties either set would
    # yield zero coverage violations -> a silent green. Force red instead.
    if not production:
        violations.append(
            f"production set is empty -- recursive .cpp discovery under "
            f"{prod_dir.name}/ (excluding test/) found nothing; a glob/exclusion "
            f"bug must not pass vacuously"
        )
    if not (linked & production):
        violations.append(
            "linked/production intersection is empty -- no host-linked source "
            "matches a discovered production path; a LINK_RE or CMake source-path "
            "bug must not pass vacuously"
        )

    # --- Coverage: each production source is linked or basename-allowlisted ---
    for rel in sorted(production):
        if rel in linked:
            continue
        if Path(rel).name in allowed:
            continue
        violations.append(
            f"{rel}: production .cpp is neither host-linked nor in the coverage "
            f"allowlist (recognized link commands: {', '.join(SOURCE_COMMANDS)})"
        )

    # --- Stale allowlist entries (basename-keyed) ----------------------------
    linked_basenames = {Path(p).name for p in linked}
    production_basenames = {Path(p).name for p in production}
    for entry in entries:
        name = entry["file"]
        if name in linked_basenames:
            violations.append(
                f"{name}: stale coverage allowlist entry -- file is now "
                f"host-linked; remove it from the allowlist"
            )
        elif name not in production_basenames:
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
    production = production_sources(prod_dir)
    linked_count = len(production & linked_sources(cmakelists_path))
    allowlisted_count = len(production) - linked_count
    print(
        f"OK: mirror baseline {baseline} across {len(observed)} files; "
        f"{len(production)} production .cpp discovered, {linked_count} linked, "
        f"{allowlisted_count} allowlisted"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
