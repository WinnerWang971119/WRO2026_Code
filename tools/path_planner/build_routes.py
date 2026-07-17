#!/usr/bin/env python3
"""WRO 2026 path-planner build script.

Reads ``tools/path_planner/field_data.py``, validates it, builds the
graph, runs Dijkstra all-pairs next-hop, emits ``routes.h`` +
``routes.cpp`` into ``WRO2026/src/nav/`` (or a user-chosen output
directory), and renders the visual map.

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
from typing import List, Mapping, Optional

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
from tools.path_planner.graph_builder import EDGE_LINE, build_graph  # noqa: E402
from tools.path_planner.header_writer import (  # noqa: E402
    DEFAULT_HEADER_PATH,
    DEFAULT_SOURCE_PATH,
    FIELD_DATA_JSON_PATH,
    emit_routes,
)
from tools.path_planner.pipeline_config import (  # noqa: E402
    PIPELINE_CONFIG,  # noqa: F401 -- baked into routes.cpp by header_writer
    LineFollowMode,
)
from tools.path_planner.planner import (  # noqa: E402
    BlindDecayParams,
    build_pivot_turns,
    compute_all_pairs_next_hop,
    parse_track_cm_from_config,
)
from tools.path_planner.validator import FieldDataError, validate  # noqa: E402
from tools.path_planner.visualizer import (  # noqa: E402
    DEFAULT_OUTPUT as DEFAULT_MAP_PNG,
)
from tools.path_planner.visualizer import render_map  # noqa: E402

# The two LineFollowMode members that actually bias the follow (issue #156);
# MIDDLE is the centered no-op and DEFAULT (== absence) never reaches a graph
# edge's ``mode`` attribute -- the validator rejects an explicit
# ``mode: DEFAULT`` on a LINE edge, so a graph edge's resolved mode is always
# one of MIDDLE/LEFT/RIGHT. Only LEFT/RIGHT are worth warning about below.
_BIASED_LINE_FOLLOW_MODE_NAMES = (LineFollowMode.LEFT.name, LineFollowMode.RIGHT.name)


def _resolve_line_follow_mode_name(mode_tag) -> Optional[str]:
    """Best-effort resolve an authored LINE-edge ``mode`` tag to its member name.

    ``field_data.EDGES[i].mode`` is forwarded onto the directed graph edge
    verbatim by :func:`graph_builder._extract_directional_attrs` -- it may be
    a :class:`LineFollowMode` instance (programmatic/Python authoring) or the
    bare member-name string (JSON authoring, e.g. ``"LEFT"``). This mirrors
    the tolerance of :func:`header_writer._override_enum_value`: an absent,
    unresolvable, or wrong-type tag resolves to ``None`` rather than raising.
    The validator (``validator.py``) already owns strict rejection of a bad
    ``mode`` value before the graph is ever built, so in practice this always
    resolves cleanly for a validated field -- the ``None`` fallback only
    guards a lint pass invoked directly against an unvalidated/hand-built
    graph in a test.

    :param mode_tag: the raw value stored under a graph edge's ``"mode"``
        attribute (``None`` when the edge carries no authored override).
    :return: the resolved member name (``"MIDDLE"`` / ``"LEFT"`` /
        ``"RIGHT"``), or ``None`` when absent/unresolvable.
    """
    if isinstance(mode_tag, LineFollowMode):
        return mode_tag.name
    if isinstance(mode_tag, str):
        try:
            return LineFollowMode[mode_tag].name
        except KeyError:
            return None
    return None


def find_unpaired_biased_line_reversals(
    graph,
    next_hop: Mapping[str, Mapping[str, Optional[str]]],
) -> List[str]:
    """Return one warning message per used-but-unpaired biased LINE reversal.

    LINE ``mode`` (``LEFT`` / ``RIGHT``) is directional (issue #156 / T-graph):
    authoring it on the directed edge ``u -> v`` does NOT mirror it onto the
    reverse edge ``v -> u`` -- a companion ``EDGES`` entry authored the other
    way round is required to bias the reverse hop too (see
    :func:`graph_builder._extract_directional_attrs`). If the planner's
    precomputed ``NEXT_HOP`` table ever actually drives ``v -> u`` (i.e. some
    destination resolves that hop) and no companion entry authored a ``mode``
    on ``v -> u``, that hop silently falls back to centered (``MIDDLE``)
    follow at runtime -- an easy authoring gap to miss, since nothing rejects
    it at validation time (a lone directional override is legal by design).

    This is a lint, not a validation rule: the caller is expected to print
    the returned messages as loud warnings and let the build continue --
    the biased edge and its (absent) companion are both individually legal;
    only the *combination* of "used in reverse" + "no reverse override" is
    the authoring smell being surfaced.

    :param graph: the directed graph from :func:`graph_builder.build_graph`.
        Every edge carries a ``"type"`` attribute (:data:`EDGE_LINE` /
        ``graph_builder.EDGE_BLIND``) and, on an authored biased LINE edge,
        a ``"mode"`` attribute (verbatim, forward-only per T-graph).
    :param next_hop: the all-pairs next-hop table from
        :func:`planner.compute_all_pairs_next_hop` -- ``next_hop[v][t]`` is
        the node the router steps to from ``v`` when the final destination is
        ``t`` (or ``None``/``v`` itself when unreachable/at-goal).
    :return: one human-readable warning string per offending ``(u, v)`` pair,
        in graph edge-iteration order; empty when the field has no such gap
        (the common case -- e.g. the current test/placeholder field, which
        authors no LINE ``mode`` overrides at all).
    """
    messages: List[str] = []
    for u, v, data in graph.edges(data=True):
        if data.get("type") != EDGE_LINE:
            continue
        mode_name = _resolve_line_follow_mode_name(data.get("mode"))
        if mode_name not in _BIASED_LINE_FOLLOW_MODE_NAMES:
            continue

        # Companion check: does the reverse directed edge v -> u carry its
        # OWN authored `mode`? LINE `mode` never mirrors (T-graph), so only
        # a SEPARATE EDGES entry authored u/v-swapped puts one there.
        reverse_data = graph.get_edge_data(v, u)
        if reverse_data is not None and reverse_data.get("mode") is not None:
            continue

        # Usage check: does the planner's routing ever actually step from v
        # to u -- i.e. is u among the resolved next-hops out of v, for ANY
        # possible final destination a mission might goTo()?
        reverse_row = next_hop.get(v)
        if not reverse_row or u not in reverse_row.values():
            continue

        messages.append(
            f"LINE edge '{u}' -> '{v}' is authored mode={mode_name}, but its "
            f"reverse traversal '{v}' -> '{u}' is driven by a NEXT_HOP route "
            "with no companion authored entry -- that hop will silently "
            "follow centered (MIDDLE). Author a companion EDGES entry "
            f"(type=LINE, u='{v}', v='{u}', mode=...) to bias it too."
        )
    return messages


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
            "routes.cpp (WRO2026/src/nav/ in repo). The two files are "
            "written as <out-dir>/routes.h and <out-dir>/routes.cpp."
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
    """Test-friendly entry point so tests can call without arg parsing.

    Mirrors the argparse layer in :func:`main` but accepts already-parsed
    parameters. The two layers are intentionally separated so the test
    suite can drive the orchestrator directly, without subprocess
    overhead and without monkeypatching ``sys.argv``.

    :param out_dir: override the output directory for ``routes.h`` /
        ``routes.cpp``. ``None`` falls back to the default header/source
        paths imported into this module from
        :mod:`tools.path_planner.header_writer` at module import time.
        Tests that need to override those defaults should patch the
        corresponding names in :mod:`tools.path_planner.build_routes`.
    :param map_out: override the visualization output path. ``None``
        falls back to the default visualization output path imported
        into this module from :mod:`tools.path_planner.visualizer` at
        module import time.
    :param no_viz: when True, skip visualization entirely.
    :param verbose: when True, write per-step progress to stderr.
    :return: process exit code (see module docstring).
    """

    def log(msg: str) -> None:
        if verbose:
            print(msg, file=sys.stderr)

    # 0. Single-source the chassis track from the firmware header. This is
    #    the one true value the robot drives with; parsing it here (instead
    #    of mirroring a Python literal) keeps the planner's pivot
    #    compensation from drifting out of sync with Config::TRACK_MM
    #    (issue #44). A missing / unparseable header is a fatal data error
    #    -- there is no stale-literal fallback -- so it shares the exit-code
    #    2 path with field-data validation failures.
    try:
        track_cm = parse_track_cm_from_config()
    except (OSError, ValueError) as exc:
        print(f"ERROR: cannot read chassis track: {exc}", file=sys.stderr)
        return 2
    log(f"chassis track: {track_cm:.3f} cm (from Config::TRACK_MM)")

    # 1. Validate the field-data module before doing any geometry work.
    #    Validation failure is fatal: routes.h must NOT be written.
    log(
        f"pipeline_config: speed={PIPELINE_CONFIG.speed.name}, "
        f"blind={PIPELINE_CONFIG.blind.name}, "
        f"turn={PIPELINE_CONFIG.turn.name}"
    )
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

    # 3. Compute the all-pairs next-hop table (pivot-agnostic since T2
    #    decoupled pivot derivation into build_pivot_turns below).
    log("computing all-pairs next-hop table ...")
    next_hop = compute_all_pairs_next_hop(
        graph, BlindDecayParams.from_field_data(field_data)
    )

    # 3b. Derive per-turn pivot records from the authored (u,v,w) triples.
    #     Uses the same track_cm resolved from Config.h above; graph is
    #     read-only so both this call and the emitter below see un-mutated
    #     edge distances.
    log("building pivot-turn records ...")
    pivot_records = build_pivot_turns(graph, track_cm)

    # 3c. Lint: warn (never fail the build) about a biased LINE edge whose
    #     reverse traversal is actually driven by NEXT_HOP with no companion
    #     authored entry (issue #156 / AC12) -- an authoring smell, since
    #     that hop would silently follow centered (MIDDLE) on the real robot.
    log("checking for unpaired biased LINE reversals ...")
    for msg in find_unpaired_biased_line_reversals(graph, next_hop):
        print(f"WARN: {msg}", file=sys.stderr)

    # 4. Resolve output paths. The DEFAULT_* names below are this
    #    module's own bindings (imported at module load time). Tests
    #    redirect output by monkeypatching
    #    ``tools.path_planner.build_routes.DEFAULT_HEADER_PATH`` etc. --
    #    patching the source module has no effect here.
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
        pivot_records=pivot_records,
        header_path=header_path,
        source_path=source_path,
        # Geometry lives in field_data.json (loaded by field_data.py); fold it
        # into the banner stamp so geometry edits change the Source SHA256.
        extra_source_paths=(FIELD_DATA_JSON_PATH,),
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
