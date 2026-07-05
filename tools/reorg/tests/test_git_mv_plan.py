"""Tests for tools.reorg.git_mv_plan -- the git-mv planner.

Never invokes git itself (nothing in this module does); `plan_moves` only
reads the filesystem to verify sources exist.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tools.reorg import git_mv_plan, mapping


def test_plan_moves_against_the_real_repo_returns_all_movers(repo_root: Path) -> None:
    """Every mapped source file exists at the flat root today, so this must succeed."""
    if (repo_root / "WRO2026" / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    ops = git_mv_plan.plan_moves(repo_root)
    assert len(ops) == 39
    assert {op.basename for op in ops} == mapping.FRAMEWORK_FILES


def test_plan_is_sorted_alphabetically_by_basename(repo_root: Path) -> None:
    if (repo_root / "WRO2026" / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    ops = git_mv_plan.plan_moves(repo_root)
    basenames = [op.basename for op in ops]
    assert basenames == sorted(basenames)


def test_each_op_src_and_dst_match_the_mapping(repo_root: Path) -> None:
    if (repo_root / "WRO2026" / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    ops = git_mv_plan.plan_moves(repo_root)
    by_name = {op.basename: op for op in ops}
    drive_op = by_name["Drive.cpp"]
    assert drive_op.domain == "drive"
    assert drive_op.src == repo_root / "WRO2026" / "Drive.cpp"
    assert drive_op.dst == repo_root / "WRO2026" / "src" / "drive" / "Drive.cpp"

    turn_sign_op = by_name["TurnSignPolicy.h"]
    assert turn_sign_op.domain == "drive"  # the documented drive-only exception
    assert turn_sign_op.dst == repo_root / "WRO2026" / "src" / "drive" / "TurnSignPolicy.h"


def test_as_git_mv_args_uses_posix_separators_and_double_dash(repo_root: Path) -> None:
    if (repo_root / "WRO2026" / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    ops = git_mv_plan.plan_moves(repo_root)
    op = next(o for o in ops if o.basename == "Drive.cpp")
    args = op.as_git_mv_args()
    assert args[:3] == ["git", "mv", "--"]
    assert "/" in args[3] and "\\" not in args[3]
    assert "/" in args[4] and "\\" not in args[4]
    assert args[3].endswith("WRO2026/Drive.cpp")
    assert args[4].endswith("WRO2026/src/drive/Drive.cpp")


def test_str_matches_as_git_mv_args_joined(repo_root: Path) -> None:
    if (repo_root / "WRO2026" / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    ops = git_mv_plan.plan_moves(repo_root)
    op = ops[0]
    assert str(op) == " ".join(op.as_git_mv_args())


def test_format_plan_is_one_command_per_line(repo_root: Path) -> None:
    if (repo_root / "WRO2026" / "src").is_dir():
        pytest.skip("pre-move only: this snapshots the flat pre-move tree; the reorg move has run")
    ops = git_mv_plan.plan_moves(repo_root)
    text = git_mv_plan.format_plan(ops)
    lines = text.splitlines()
    assert len(lines) == 39
    assert all(line.startswith("git mv --") for line in lines)


def test_plan_moves_raises_on_missing_source(tmp_path: Path) -> None:
    """A mapped file missing from disk is a hard error (mapping/tree drift), not a skip."""
    sketch_dir = tmp_path / "WRO2026"
    sketch_dir.mkdir()
    # Create every mapped source EXCEPT one, to prove the check fires precisely on it.
    missing = "Drive.cpp"
    for basename, domain in mapping.MAPPING.items():
        if domain is not None and basename != missing:
            (sketch_dir / basename).write_text("// stub\n", encoding="utf-8")

    with pytest.raises(FileNotFoundError, match=missing):
        git_mv_plan.plan_moves(tmp_path)
