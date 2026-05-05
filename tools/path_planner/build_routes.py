#!/usr/bin/env python3
"""WRO 2026 path-planner build script.

Reads ``tools/path_planner/field_data.py``, validates it, builds the
graph, runs Dijkstra all-pairs next-hop, emits ``routes.h`` +
``routes.cpp`` into ``WRO2026/`` (or a user-chosen output directory),
and renders the visual map.

This is the CLI orchestrator: every layer below is a pure-Python module
(:mod:`tools.path_planner.validator`,
:mod:`tools.path_planner.graph_builder`,
:mod:`tools.path_planner.planner`,
:mod:`tools.path_planner.header_writer`,
:mod:`tools.path_planner.visualizer`); this file only sequences them
and decides exit codes.

Run from the repo root::

    uv run python tools/path_planner/build_routes.py
    # or
    python -m tools.path_planner.build_routes

Idempotency:
  Re-running this with no input changes produces a byte-identical
  ``routes.h`` / ``routes.cpp`` pair (AC2). The visualization is
  regenerated in-place on every run; matplotlib output is not
  byte-stable, so ``map.png`` is not part of the AC2 idempotency
  contract.

Exit codes:
  ``0``  Success.
  ``1``  Visualization or unrelated non-fatal failure
         (header still wrote).
  ``2``  Validation / data error -- header was NOT written.
"""

from __future__ import annotations

import argparse
import sys
import warnings
from pathlib import Path
from typing import Optional

# When invoked as ``python tools/path_planner/build_routes.py`` (i.e.
# the file is the top-level script, NOT imported as a module via
# ``python -m tools.path_planner.build_routes``), Python's automatic
# ``sys.path[0]`` is the script's own directory -- ``tools/path_planner/``
# -- so ``from tools.path_planner import ...`` fails with
# ModuleNotFoundError. Detect that case and prepend the repo root
# (two parents up: ``tools/path_planner/build_routes.py`` ->
# ``tools/path_planner`` -> ``tools`` -> repo root) so both invocation
# styles resolve identically. This is a no-op under ``-m`` because the
# CWD is already on ``sys.path``.
if __name__ == "__main__" and __package__ in (None, ""):
    _REPO_ROOT = Path(__file__).resolve().parents[2]
    if str(_REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT))

from tools.path_planner import field_data  # noqa: E402
from tools.path_planner.graph_builder import build_graph  # noqa: E402
from tools.path_planner.header_writer import (  # noqa: E402
    DEFAULT_HEADER_PATH,
    DEFAULT_SOURCE_PATH,
    emit_routes,
)
from tools.path_planner.planner import compute_all_pairs_next_hop  # noqa: E402
from tools.path_planner.validator import FieldDataError, validate  # noqa: E402
from tools.path_planner.visualizer import (  # noqa: E402
    DEFAULT_OUTPUT as DEFAULT_MAP_PNG,
)
from tools.path_planner.visualizer import render_map  # noqa: E402


def main(argv: Optional[list[str]] = None) -> int:
    """Argparse entry point.

    :param argv: optional explicit argv list (excluding the program
        name). When ``None`` (the default) :mod:`argparse` reads from
        :data:`sys.argv`.
    :return: process exit code (see module docstring).
    """
    parser = argparse.ArgumentParser(
        prog="build_routes.py",
        description="Build routes.h + routes.cpp from field_data.py.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help=(
            "Override the default output directory for routes.h / "
            "routes.cpp (WRO2026/ in repo). The two files are written "
            "as <out-dir>/routes.h and <out-dir>/routes.cpp."
        ),
    )
    parser.add_argument(
        "--map-out",
        type=Path,
        default=None,
        help=(
            "Override the visualization output path "
            "(default: tools/path_planner/output/map.png)."
        ),
    )
    parser.add_argument(
        "--no-viz",
        action="store_true",
        help="Skip rendering the visualization map.png.",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print per-step progress to stderr.",
    )
    args = parser.parse_args(argv)

    return _run(
        out_dir=args.out_dir,
        map_out=args.map_out,
        no_viz=args.no_viz,
        verbose=args.verbose,
    )


def _run(
    *,
    out_dir: Optional[Path] = None,
    map_out: Optional[Path] = None,
    no_viz: bool = False,
    verbose: bool = False,
) -> int:
    """Pure-function entry point so tests can call without arg parsing.

    Mirrors the argparse layer in :func:`main` but accepts already-parsed
    parameters. The two layers are intentionally separated so the test
    suite can drive the orchestrator without subprocess overhead and
    without monkeypatching ``sys.argv``.

    :param out_dir: override the output directory for ``routes.h`` /
        ``routes.cpp``. ``None`` falls back to the
        :data:`tools.path_planner.header_writer.DEFAULT_HEADER_PATH` and
        :data:`tools.path_planner.header_writer.DEFAULT_SOURCE_PATH`
        constants resolved at call time (so monkeypatching them in tests
        works).
    :param map_out: override the visualization output path. ``None``
        falls back to
        :data:`tools.path_planner.visualizer.DEFAULT_OUTPUT`,
        again resolved at call time.
    :param no_viz: when True, skip visualization entirely.
    :param verbose: when True, write per-step progress to stderr.
    :return: process exit code (see module docstring).
    """

    def log(msg: str) -> None:
        if verbose:
            print(msg, file=sys.stderr)

    # 1. Validate the field-data module before doing any geometry work.
    #    Validation failure is fatal: routes.h must NOT be written.
    log("validating field_data.py ...")
    try:
        validate(field_data)
    except FieldDataError as exc:
        print(f"ERROR: field_data invalid: {exc}", file=sys.stderr)
        return 2

    # 2. Build the directed graph (with C-space inflation).
    #    build_graph emits UserWarnings for disconnected goals and
    #    isolated nodes; we forward them to stderr as WARN: lines so a
    #    quiet test environment still surfaces them.
    log("building graph + applying obstacle inflation ...")
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        graph = build_graph(field_data)
    for w in caught:
        print(f"WARN: {w.message}", file=sys.stderr)

    # 3. Compute the all-pairs next-hop table.
    log("computing all-pairs next-hop table ...")
    next_hop = compute_all_pairs_next_hop(
        graph, field_data.COST_BLIND_MULTIPLIER
    )

    # 4. Resolve output paths. We re-import the defaults at call time so
    #    test monkeypatching of header_writer's module-level constants
    #    works without restructuring the imports above.
    if out_dir is not None:
        header_path = out_dir / "routes.h"
        source_path = out_dir / "routes.cpp"
    else:
        header_path = DEFAULT_HEADER_PATH
        source_path = DEFAULT_SOURCE_PATH

    # 5. Emit routes.h + routes.cpp. emit_routes handles parent-dir
    #    creation so we don't pre-create out_dir here.
    log(f"emitting routes header to {header_path} ...")
    emit_routes(
        field_data,
        graph,
        next_hop,
        header_path=header_path,
        source_path=source_path,
    )

    # 6. Render the visualization. Failure is non-fatal but still flagged
    #    via exit code 1 so a build server picks it up. We catch
    #    Exception (not BaseException) so KeyboardInterrupt /
    #    SystemExit still propagate.
    if not no_viz:
        viz_path = map_out if map_out is not None else DEFAULT_MAP_PNG
        log(f"rendering visualization to {viz_path} ...")
        try:
            render_map(field_data, graph, out_path=viz_path)
        except Exception as exc:  # noqa: BLE001 - intentional broad catch
            print(
                f"WARN: visualization failed "
                f"({type(exc).__name__}: {exc}); "
                "routes header was still written.",
                file=sys.stderr,
            )
            return 1

    log(f"OK: wrote {header_path} and {source_path}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
