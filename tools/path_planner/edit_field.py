"""Live field editor -- GUI for editing ``field_data.json`` + rich EDGES renames.

Usage::

    uv run python tools/path_planner/edit_field.py             # open the GUI
    uv run python tools/path_planner/edit_field.py --migrate   # one-shot port
    uv run python tools/path_planner/edit_field.py --migrate --dry-run

The GUI mode (default) opens a Tk window backed by an embedded matplotlib
canvas. ``--migrate`` runs the one-shot port from a Python-literal
``field_data.py`` to the JSON sidecar layout; on an already-migrated repo
it prints a status line and exits ``0``. ``--dry-run`` (with ``--migrate``)
reports what would change without touching disk.

This file is intentionally a thin CLI shell: argparse + import + dispatch.
All application logic lives in :class:`tools.path_planner.editor.app.EditorApp`
(GUI) and :func:`tools.path_planner.editor.migrate.run` (one-shot migration).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running as ``python tools/path_planner/edit_field.py`` (no -m). When
# invoked that way, Python's automatic ``sys.path[0]`` is ``tools/path_planner/``,
# which makes ``from tools.path_planner...`` fail. Prepending the repo root
# (two parents up from this file: ``tools/path_planner/edit_field.py`` ->
# ``tools/path_planner`` -> ``tools`` -> repo root) lets both invocation
# styles resolve identically. The ``-m`` invocation already has the repo
# root on sys.path, so the insertion is a no-op there.
_HERE = Path(__file__).resolve()
_REPO_ROOT = _HERE.parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


# Exit codes mirror the migration script's contract (see ``editor/migrate.py``).
_EXIT_OK: int = 0
_EXIT_ERROR: int = 2


def main(argv: list[str] | None = None) -> int:
    """Argparse entry point.

    :param argv: Optional explicit argv list (excluding the program name).
        ``None`` reads from :data:`sys.argv`.
    :return: Process exit code -- ``0`` on success, ``2`` on migration
        failure (the GUI returns ``0`` after the window closes regardless
        of whether the user saved).
    """
    parser = argparse.ArgumentParser(
        prog="edit_field.py",
        description=(
            "Live field editor for tools/path_planner/field_data.json. "
            "Default mode opens a Tk window; --migrate runs the one-shot "
            "port from the legacy Python-literal layout."
        ),
    )
    parser.add_argument(
        "--migrate",
        action="store_true",
        help=(
            "Run the one-shot migration from the legacy Python-literal "
            "field_data.py to the JSON sidecar. Idempotent: a no-op on an "
            "already-migrated repo."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "Used with --migrate: report what would change without "
            "touching disk. Has no effect without --migrate."
        ),
    )
    args = parser.parse_args(argv)

    if args.migrate:
        # Deferred import: keeps the GUI-only path from pulling in the
        # migration helpers (which import the field_data module).
        from tools.path_planner.editor import migrate  # noqa: PLC0415
        return migrate.run(dry_run=args.dry_run)

    if args.dry_run:
        # ``--dry-run`` on its own is meaningless; print a hint and exit
        # cleanly so a misinvocation does not silently do nothing.
        print(
            "edit_field.py: --dry-run only applies to --migrate; "
            "no action taken.",
            file=sys.stderr,
        )
        return _EXIT_OK

    # GUI mode -- deferred import keeps the migration path independent of
    # Tk + matplotlib, which would otherwise crash a headless --migrate
    # invocation on a CI runner without a display.
    from tools.path_planner.editor.app import EditorApp  # noqa: PLC0415

    app = EditorApp()
    app.run()
    return _EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
