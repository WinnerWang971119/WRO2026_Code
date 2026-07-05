"""``git mv`` planner (T1 item 2).

Lists the ``git mv <old> <new>`` operations needed to relocate the 39 framework
files into ``WRO2026/src/<domain>/``. This module only PLANS the moves -- it
never shells out to git itself. The orchestrator (T5) runs the printed/returned
commands; this keeps the tool side-effect-free and trivially unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

from .mapping import MAPPING, destination_repo_path, source_repo_path


@dataclass(frozen=True)
class MoveOp:
    """One planned ``git mv`` operation for a single framework file."""

    basename: str
    domain: str
    src: Path
    dst: Path

    def as_git_mv_args(self) -> List[str]:
        """The literal argv the orchestrator should execute (POSIX-separated paths)."""
        return ["git", "mv", "--", self.src.as_posix(), self.dst.as_posix()]

    def __str__(self) -> str:
        return " ".join(self.as_git_mv_args())


def plan_moves(repo_root: Path = Path(".")) -> List[MoveOp]:
    """Build the ordered list of ``git mv`` operations for the 39 moved files.

    Verifies each source file exists under ``repo_root`` before including it in
    the plan -- a missing source means the mapping table has drifted from the
    actual tree, which is a hard error, not a silently-skipped file.

    Ordered alphabetically by basename for a deterministic, reviewable plan.
    """
    ops: List[MoveOp] = []
    movers = sorted(name for name, domain in MAPPING.items() if domain is not None)
    for basename in movers:
        domain = MAPPING[basename]
        assert domain is not None  # narrowed by the filter above; documents intent
        src = repo_root / source_repo_path(basename)
        if not src.is_file():
            raise FileNotFoundError(
                f"T1 mapping table lists {basename!r} -> src/{domain}/, but {src} does "
                "not exist on disk -- the mapping table has drifted from the actual tree."
            )
        dst = repo_root / destination_repo_path(basename)
        ops.append(MoveOp(basename=basename, domain=domain, src=src, dst=dst))
    return ops


def format_plan(ops: List[MoveOp]) -> str:
    """Render a plan as one ``git mv`` command per line, for printing / piping."""
    return "\n".join(str(op) for op in ops)
