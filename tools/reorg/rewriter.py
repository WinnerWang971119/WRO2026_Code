"""File-walking orchestration over :mod:`include_rules` / :mod:`test_includes`.

Everything in :mod:`include_rules` and :mod:`test_includes` is a pure function
over strings -- this module is the thin, side-effecting layer that reads real
files, computes their rewritten text, and (only when asked) writes it back.
Diff *computation* never touches disk beyond reading; only :func:`apply_diffs`
with ``check=False`` writes. Callers that want to verify a tree without
mutating it (T1's own tests against the real repo, CI freshness checks) should
call the ``rewrite_*_tree`` functions and inspect ``FileDiff.changed`` without
ever calling :func:`apply_diffs`.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

from .include_rules import Layout, Location, locate, rewrite_production_line
from .mapping import MAPPING
from .test_includes import GENERATED_FILE_EXCLUSIONS, ambiguous_basenames, rewrite_test_line

#: File suffixes this tool ever rewrites includes in.
_SOURCE_SUFFIXES = (".h", ".cpp", ".ino")


@dataclass(frozen=True)
class FileDiff:
    """The original and rewritten text for one file (whether or not it changed)."""

    path: Path
    original: str
    rewritten: str

    @property
    def changed(self) -> bool:
        return self.original != self.rewritten


def _iter_production_files(sketch_root: Path, layout: Layout) -> List[Path]:
    """Every production source file under `sketch_root`, for the given layout.

    Under ``"old"`` (pre-move) all classified root files sit directly under
    `sketch_root` (the movers have not moved yet). Under ``"new"``
    (post-move) only the 7 root-stay files remain directly under
    `sketch_root`; the movers instead sit under
    ``sketch_root/src/<domain>/``. Either way ``test/`` is never walked here --
    see :func:`rewrite_test_tree` for that -- and ``config/`` is walked
    identically in both layouts (it never moves).
    """
    files: List[Path] = []
    for entry in sorted(sketch_root.iterdir()):
        if entry.is_file() and entry.suffix in _SOURCE_SUFFIXES:
            files.append(entry)

    if layout == "new":
        src_dir = sketch_root / "src"
        if src_dir.is_dir():
            for domain_dir in sorted(p for p in src_dir.iterdir() if p.is_dir()):
                for entry in sorted(domain_dir.iterdir()):
                    if entry.is_file() and entry.suffix in _SOURCE_SUFFIXES:
                        files.append(entry)
    elif layout != "old":
        raise ValueError(f"unknown layout {layout!r}; expected 'old' or 'new'")

    config_dir = sketch_root / "config"
    if config_dir.is_dir():
        files.extend(sorted(p for p in config_dir.iterdir() if p.is_file() and p.suffix == ".h"))
    return files


def _includer_location(path: Path, layout: Layout) -> Location:
    if path.name in MAPPING or path.suffix == ".h" and path.parent.name == "config":
        return locate(path.name, layout)
    raise ValueError(f"{path} is not a migration-tracked file (checked mapping.MAPPING / config/)")


def rewrite_production_tree(sketch_root: Path, layout: Layout = "new") -> List[FileDiff]:
    """Compute (but do not write) the rewritten text for every production file.

    Returns one :class:`FileDiff` per file, whether or not it actually changed,
    so callers can inspect ``diff.changed`` for a dry-run/CI report or pass the
    list to :func:`apply_diffs` to write it.
    """
    diffs: List[FileDiff] = []
    for path in _iter_production_files(sketch_root, layout):
        includer = _includer_location(path, layout)
        original = path.read_text(encoding="utf-8")
        lines = original.splitlines(keepends=True)
        rewritten = "".join(rewrite_production_line(line, includer, layout) for line in lines)
        diffs.append(FileDiff(path=path, original=original, rewritten=rewritten))
    return diffs


def rewrite_test_tree(test_root: Path) -> List[FileDiff]:
    """Compute (but do not write) the rewritten text for every hand-written test file.

    Walks `test_root` (``WRO2026/test/``) recursively for ``.h``/``.cpp`` files,
    skipping :data:`tools.reorg.test_includes.GENERATED_FILE_EXCLUSIONS`
    entirely. Every include of an ambiguous basename is classified (see
    :mod:`tools.reorg.test_includes`) and only the ones resolved to ``"real"``
    are rewritten -- always to the post-move ``src/<domain>/`` form, since a
    test file's own location never moves.
    """
    ambiguous = ambiguous_basenames(test_root)
    diffs: List[FileDiff] = []
    for path in sorted(test_root.rglob("*")):
        if not path.is_file() or path.suffix not in (".h", ".cpp"):
            continue
        relpath = path.relative_to(test_root).as_posix()
        if relpath in GENERATED_FILE_EXCLUSIONS:
            continue
        sibling_basenames = frozenset(p.name for p in path.parent.iterdir() if p.is_file())
        original = path.read_text(encoding="utf-8")
        lines = original.splitlines(keepends=True)
        rewritten = "".join(
            rewrite_test_line(line, relpath, sibling_basenames, ambiguous) for line in lines
        )
        diffs.append(FileDiff(path=path, original=original, rewritten=rewritten))
    return diffs


def apply_diffs(diffs: List[FileDiff], check: bool = False) -> List[Path]:
    """Write changed files to disk (or, in `check` mode, just report them).

    Returns the paths that changed (would change, under `check`). This is the
    ONLY function in this module that ever writes -- `rewrite_production_tree`
    / `rewrite_test_tree` only read and compute.
    """
    changed = [d for d in diffs if d.changed]
    if not check:
        for d in changed:
            d.path.write_text(d.rewritten, encoding="utf-8", newline="\n")
    return [d.path for d in changed]
