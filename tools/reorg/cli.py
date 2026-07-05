"""CLI entry point for the sketch-src reorg tooling (issue #66).

Usage (from the repo root)::

    uv run python -m tools.reorg git-mv-plan
    uv run python -m tools.reorg rewrite-production --layout old --check
    uv run python -m tools.reorg rewrite-production --layout new
    uv run python -m tools.reorg rewrite-tests --check
    uv run python -m tools.reorg audit-test-includes

T5 (production move) drives this as: run ``git-mv-plan``, execute each printed
``git mv`` command itself, then run ``rewrite-production`` (default
``--layout new``) to fix up every cross-boundary include. T6 (test-include
rewrite) runs ``rewrite-tests`` afterward. Both support ``--check`` for a
dry-run CI gate (exits 1 if anything would change).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from . import git_mv_plan, rewriter
from .test_includes import audit_test_includes


def _cmd_git_mv_plan(args: argparse.Namespace) -> int:
    ops = git_mv_plan.plan_moves(Path(args.repo_root))
    print(git_mv_plan.format_plan(ops))
    return 0


def _report_and_exit(diffs, check: bool) -> int:
    changed = rewriter.apply_diffs(diffs, check=check)
    verb = "would rewrite" if check else "rewrote"
    for path in changed:
        print(f"{verb}: {path}")
    if not changed:
        print("no changes needed")
    return 1 if (check and changed) else 0


def _cmd_rewrite_production(args: argparse.Namespace) -> int:
    sketch_root = Path(args.repo_root) / "WRO2026"
    diffs = rewriter.rewrite_production_tree(sketch_root, layout=args.layout)
    return _report_and_exit(diffs, args.check)


def _cmd_rewrite_tests(args: argparse.Namespace) -> int:
    test_root = Path(args.repo_root) / "WRO2026" / "test"
    diffs = rewriter.rewrite_test_tree(test_root)
    return _report_and_exit(diffs, args.check)


def _cmd_audit_test_includes(args: argparse.Namespace) -> int:
    test_root = Path(args.repo_root) / "WRO2026" / "test"
    entries = audit_test_includes(test_root)
    for e in entries:
        print(f"{e.test_relpath}:{e.line_no}: {e.included_basename} -> {e.classification} ({e.evidence})")
    unexplained = [e for e in entries if e.evidence == "default-mock"]
    print(f"\n{len(entries)} ambiguous includes, {len(unexplained)} resolved by default (mock, unlisted)")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m tools.reorg",
        description=(
            "Path-mapping + include-rewrite tooling for the WRO2026 sketch-src reorg "
            "(issue #66). See docs/plans/2026-07-05-sketch-src-reorg.md."
        ),
    )
    parser.add_argument(
        "--repo-root", default=".", help="repo root containing WRO2026/ (default: cwd)"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_plan = sub.add_parser(
        "git-mv-plan",
        help="list the `git mv <src> <dst>` operations for the 39 moved files (orchestrator runs them)",
    )
    p_plan.set_defaults(func=_cmd_git_mv_plan)

    p_prod = sub.add_parser(
        "rewrite-production",
        help="rewrite production quote-includes under WRO2026/ (excluding test/)",
    )
    p_prod.add_argument(
        "--layout",
        choices=("old", "new"),
        default="new",
        help="'new' (default) assumes git mv already ran; 'old' is a no-op sanity check",
    )
    p_prod.add_argument(
        "--check", action="store_true", help="dry-run: report would-be changes, exit 1 if any"
    )
    p_prod.set_defaults(func=_cmd_rewrite_production)

    p_test = sub.add_parser(
        "rewrite-tests",
        help="rewrite real-intent production includes under WRO2026/test/ to their src/<domain>/ form",
    )
    p_test.add_argument(
        "--check", action="store_true", help="dry-run: report would-be changes, exit 1 if any"
    )
    p_test.set_defaults(func=_cmd_rewrite_tests)

    p_audit = sub.add_parser(
        "audit-test-includes",
        help="report the real/mock classification + evidence for every ambiguous include under WRO2026/test/",
    )
    p_audit.set_defaults(func=_cmd_audit_test_includes)

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
