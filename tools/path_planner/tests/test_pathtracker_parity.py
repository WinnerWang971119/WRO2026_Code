"""Live sim<->firmware FSM parity diff (issue #108, T8).

This is the integration crux of the PathTracker sim<->firmware parity plan
(`docs/plans/2026-06-28-pathtracker-sim-parity.md`). It:

1. invokes the C++ parity-DUMP binary (``test_path_tracker_parity_dump``) built
   by the dedicated ``parity`` CI job over the feature-complete
   ``routes_fixture``, and parses its versioned NDJSON via
   :func:`~tools.path_planner.sim.parity.load_cpp_ndjson`;
2. builds the Python :class:`~tools.path_planner.sim.runtime.PathTrackerSim`
   from the SAME single-sourced ``field_fixture`` map the dump's
   ``routes_fixture.cpp`` was generated from, and runs it over the IDENTICAL
   case set (all-pairs + the same ``CURATED_PARITY_CASES``);
3. normalizes both streams into the shared :class:`TraceRecord` schema
   (:func:`~tools.path_planner.sim.parity.normalize_sim_stream`);
4. integrates pose for BOTH streams through the ONE shared ``VirtualChassis``
   (:func:`~tools.path_planner.sim.parity.integrate_pose`); and
5. diffs the two per the THREE comparison buckets (EXACT decision fields,
   TOLERANT-NUMERIC distance/angular, POSE-TOLERANT), honoring the
   self-auditing divergence allowlist.

The standing anti-vacuous guards (AC5) are each a real, separate assertion:

* **case-count parity + non-zero** — the C++ and Python case sets must be the
  identical non-empty set (:func:`test_case_count_parity_and_nonzero`);
* **AC8 coverage guard** — every per-hop record kind must appear >=1x across
  BOTH sweeps (:func:`test_coverage_every_record_kind_present`);
* **stale-allowlist guard (TC7)** — an allowlist entry that matched zero
  mismatches fails (:func:`test_no_stale_allowlist_entries`); and
* **every-field-bucketed** — every schema field maps to exactly one comparison
  bucket (:func:`test_every_trace_field_is_bucketed`), so a field added later
  cannot silently escape comparison.

AC6 (an injected divergence makes this FAIL at the exact case/record/field) is a
one-time MANUAL demonstration in the PR description, deliberately NOT a committed
permanently-failing test.

Toolchain policy (Error Handling): the dedicated ``parity`` CI job builds the
dump binary first, so its absence in CI is a HARD error; a LOCAL run with no
binary :func:`pytest.skip`\\s with a clear build hint (never a silent pass in
CI — CI is detected via the ``CI`` env var). Build it locally with::

    cmake -S WRO2026/test -B build/parity -G "<your generator>"
    cmake --build build/parity --config Debug --target test_path_tracker_parity_dump
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import warnings
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import pytest

# ---------------------------------------------------------------------------
# Cross-tree import shim (pinned). The parity test lives under
# tools/path_planner/tests/ but must import `field_fixture` from
# WRO2026/test/fixtures/ -- mirroring how regenerate_routes_fixture.py threads
# `_REPO_ROOT` for the reverse direction. Insert the repo root (so the
# `tools.path_planner.*` imports below resolve when collected by any rootdir)
# AND the fixtures dir (so the sibling `field_fixture` module is importable),
# both BEFORE importing field_fixture.
# ---------------------------------------------------------------------------
_REPO_ROOT = Path(__file__).resolve().parents[3]  # tests -> path_planner -> tools -> repo
_FIXTURES_DIR = _REPO_ROOT / "WRO2026" / "test" / "fixtures"
for _p in (str(_REPO_ROOT), str(_FIXTURES_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import field_fixture  # noqa: E402  (cross-tree fixture module, single-sourced with routes_fixture)

from tools.path_planner.graph_builder import build_graph  # noqa: E402
from tools.path_planner.planner import (  # noqa: E402
    BlindDecayParams,
    compute_all_pairs_next_hop,
)
from tools.path_planner.sim.chassis import VirtualChassis, _normalize  # noqa: E402
from tools.path_planner.sim.parity import (  # noqa: E402
    PARITY_SCHEMA_VERSION,
    PER_HOP_RECORD_KINDS,
    TRACE_RECORD_FIELDS,
    Allowlist,
    AllowlistEntry,
    CppCase,
    ParityCase,
    ParitySchemaError,
    Pose,
    TraceRecord,
    _record_from_obj,
    integrate_pose,
    load_allowlist,
    load_cpp_ndjson,
    normalize_sim_stream,
)
from tools.path_planner.sim.runtime import (  # noqa: E402
    PathTrackerSim,
    walk_next_hop,
)

# ---------------------------------------------------------------------------
# Comparison tolerances (the three-bucket constants — owned here in T8, not in
# parity.py). Distance/coord in cm; angular fields compared circularly (wrap-
# aware) in deg; pose position looser than commanded distance because pose is
# accumulated. A later task can still extend `_EXACT_FIELDS` directly (as
# ADR-052's runtime/parity task did for `pass_kind`, ahead of ADR-052's own
# dedicated sim-test task) without moving ownership out of this file.
# ---------------------------------------------------------------------------
DIST_TOL: float = 1e-4       # cm — TOLERANT-NUMERIC (distance) bucket
HEAD_TOL: float = 1e-4       # deg — TOLERANT-NUMERIC (angular) bucket, circular
POSE_POS_TOL: float = 1e-3   # cm — POSE-TOLERANT position (accumulated)

#: Comparison bucket tags.
_BUCKET_EXACT: str = "EXACT"
_BUCKET_DIST: str = "DIST"
_BUCKET_HEAD: str = "HEAD"

#: Fields compared by exact string/enum/bool/int equality (zero tolerance). The
#: enums (`primitive`/`drive_mode`/`speed`/`follow_mode`/`wheel`/`mode`), bools
#: (`reverse`/`pass_through`/`brake_on_gate`), node names
#: (`node`/`start`/`goal`/`final`), `reanchor`, `axis_idx`, and the framing
#: metadata. Any continuous field falls through to the suffix rule below.
_EXACT_FIELDS: frozenset = frozenset(
    {
        # enums
        "primitive",
        "drive_mode",
        # `speed` is the parity v3 rename of the historically-misnamed
        # `line_mode` (it always held the resolved SpeedTier tier name), on
        # both line_follow and projection_follow records.
        "speed",
        # `follow_mode` is the parity v3 LINE lateral-bias tag (issue #156):
        # the resolved LineFollowMode member NAME (MIDDLE|LEFT|RIGHT), on
        # both line_follow and projection_follow records.
        "follow_mode",
        "wheel",
        "mode",
        "reanchor",
        # ADR-052 line-arrival heading reset pass-kind tag ("proj" / "junction")
        "pass_kind",
        # bools
        "reverse",
        "pass_through",
        "brake_on_gate",
        # ADR-059 blind-chain run-membership flag
        "chain",
        # node names
        "node",
        "start",
        "goal",
        "final",
        # integer index
        "axis_idx",
        # ADR-059 resolved velocity-plan endpoints (signed int16 PWM, exact).
        # NOT distance/angular — the `_pwm` suffix falls through to the default
        # (unbucketed) rule, so they are pinned to the EXACT bucket here.
        "start_pwm",
        "end_pwm",
        # goal_start framing metadata
        "case_id",
        "route_dir",
        "arrival_dir",
    }
)


def _bucket_for_field(name: str) -> Optional[str]:
    """Map a trace-record field name to exactly one comparison bucket.

    The classifier realizes the spec's bucketing rule so a field added later
    cannot silently escape comparison: an explicit EXACT field is exact; any
    ``*_cm`` / ``*_coord`` defaults to the distance bucket; any ``*_deg`` /
    heading field to the angular bucket; ANYTHING ELSE is unassigned and
    returns ``None`` (which the comparator treats as a hard failure).

    :param name: the field name (e.g. ``"advance_cm"``, ``"body_heading_deg"``).
    :return: one of :data:`_BUCKET_EXACT` / :data:`_BUCKET_DIST` /
        :data:`_BUCKET_HEAD`, or ``None`` when the field is unbucketed.
    """
    if name in _EXACT_FIELDS:
        return _BUCKET_EXACT
    if name.endswith("_cm") or name.endswith("_coord"):
        return _BUCKET_DIST
    if name.endswith("_deg") or "heading" in name:
        return _BUCKET_HEAD
    return None


def _circular_delta_deg(a: float, b: float) -> float:
    """Return the minimal absolute angular difference (deg) between @p a and @p b.

    Wrap-aware so ``-180`` and ``180`` compare equal (the firmware may emit a
    return heading as ``-180.0`` while the sim carries ``180.0``).
    """
    return abs(((float(a) - float(b) + 180.0) % 360.0) - 180.0)


def _values_equal(bucket: str, cpp_value: Any, sim_value: Any) -> bool:
    """Compare one field's C++ vs sim value under its assigned bucket."""
    if bucket == _BUCKET_EXACT:
        return cpp_value == sim_value
    if bucket == _BUCKET_DIST:
        return abs(float(cpp_value) - float(sim_value)) <= DIST_TOL
    # _BUCKET_HEAD
    return _circular_delta_deg(cpp_value, sim_value) <= HEAD_TOL


# ---------------------------------------------------------------------------
# The curated pinned cases — MUST replicate T6's CURATED_PARITY_CASES table
# byte-for-byte (same order, same tags). The 4th element is the pinned operator-
# offset MAGNITUDE pushed along world heading 0 (matching the C++ `seatAt(start,
# 0); pushOperatorOffset(mag, 0)`), or None for a clean special. A count or tag
# mismatch vs the dump is caught by `test_case_count_parity_and_nonzero`.
# ---------------------------------------------------------------------------
_CURATED_PARITY_CASES: Tuple[Tuple[str, str, str, Optional[float]], ...] = (
    ("position1", "Wall1", "reset_wall1", None),
    ("final", "Wall2", "reset_wall2", None),
    ("Inter3", "Corner", "pivot_line", None),
    ("Corner", "Zed", "pivot_sensor", None),
    ("Inter2", "Inter4", "pass_lll", None),
    ("Inter1", "Inter2", "pass_lpl", None),
    ("Inter1", "Spur", "rear", None),
    ("Inter2", "Inter3", "off_n14_h0", 14.0),
    ("Zed", "ProjB", "off_n6_h0", 6.0),
)


# ---------------------------------------------------------------------------
# Dump-binary discovery + invocation.
# ---------------------------------------------------------------------------


def _dump_exe_name() -> str:
    """Return the platform-specific dump executable file name."""
    base = "test_path_tracker_parity_dump"
    return base + ".exe" if sys.platform.startswith("win") else base


def _find_dump_exe() -> Optional[Path]:
    """Locate the built parity-dump binary, generator- and build-dir-agnostic.

    An explicit ``PARITY_DUMP_EXE`` env var wins (the CI job points it at the
    binary it just built). Otherwise the repo's ``build/`` tree is searched
    recursively for the platform exe name (covering single-config generators at
    the build root and multi-config generators nesting under ``Debug/`` /
    ``Release/`` in any build subdir), returning the newest match so a stale
    binary from a previous generator never shadows a fresh build.

    :return: the most-recently-modified matching executable, or ``None`` when
        no build output exists yet.
    """
    override = os.environ.get("PARITY_DUMP_EXE")
    if override:
        candidate = Path(override)
        return candidate if candidate.is_file() else None

    build_root = _REPO_ROOT / "build"
    if not build_root.is_dir():
        return None
    matches = [p for p in build_root.rglob(_dump_exe_name()) if p.is_file()]
    if not matches:
        return None
    return max(matches, key=lambda p: p.stat().st_mtime)


def _run_dump(exe: Path) -> str:
    """Run the dump binary and return its pure-NDJSON stdout.

    :param exe: the discovered dump executable.
    :return: captured stdout (one JSON object per line; diagnostics go to
        stderr, so stdout is clean NDJSON).
    :raises RuntimeError: if the binary exits non-zero (a stuck FSM or a bad
        curated pin); the message carries its stderr verbatim.
    """
    completed = subprocess.run(
        [str(exe)],
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or "(no stderr)"
        raise RuntimeError(
            f"parity dump {exe.name} exited with code {completed.returncode}:\n"
            f"{stderr}"
        )
    return completed.stdout


# ---------------------------------------------------------------------------
# Fixture build + case enumeration (the Python side of the single-sourced map).
# ---------------------------------------------------------------------------


def _build_fixture() -> Tuple[Any, Any, Any]:
    """Build the (field, graph, next_hop) triple from the single-sourced fixture.

    Mirrors ``regenerate_routes_fixture.py`` (the SAME source the dump's
    ``routes_fixture.cpp`` was generated from) so the two sides cannot diverge
    on the map or the routing: ``build_field_data`` -> ``build_graph`` ->
    ``compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))``.
    The pivot table is resolved by :class:`PathTrackerSim` itself on
    construction (against the same ``Config.h`` track width the emitter used).

    :return: ``(field_data_namespace, networkx_graph, next_hop_table)``.
    """
    fd = field_fixture.build_field_data()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")  # incidental build_graph orphan-node warnings
        graph = build_graph(fd)
        next_hop = compute_all_pairs_next_hop(
            graph, BlindDecayParams.from_field_data(fd)
        )
    return fd, graph, next_hop


def _enumerate_cases(field: Any, next_hop: Any) -> List[ParityCase]:
    """Enumerate the identical case set the C++ dump emits (all-pairs + curated).

    All-pairs: every ordered ``(start, goal)`` node pair with ``start != goal``
    and a reachable route, sorted by ``(start_name, goal_name)`` (Python's
    default name sort matches the dump's strcmp ASCII order; the join is by
    ``case_id`` so order is for determinism only). Then the curated pins in the
    SAME order/tags as :data:`_CURATED_PARITY_CASES`.

    :param field: the fixture field-data namespace.
    :param next_hop: the all-pairs next-hop table.
    :return: the ordered :class:`ParityCase` list.
    """
    cases: List[ParityCase] = []
    names = sorted(field.NODES)
    for start in names:
        for goal in names:
            if start == goal:
                continue
            if next_hop[start][goal] is None:
                continue  # unreachable — the dump skips it too (deterministic)
            cases.append(
                ParityCase(case_id=f"{start}__{goal}__clean", start=start, goal=goal)
            )
    for start, goal, tag, offset_mag in _CURATED_PARITY_CASES:
        offset = None if offset_mag is None else (float(offset_mag), 0.0, 0.0)
        cases.append(
            ParityCase(
                case_id=f"{start}__{goal}__{tag}",
                start=start,
                goal=goal,
                offset=offset,
            )
        )
    return cases


def _run_sim_case(
    field: Any,
    graph: Any,
    next_hop: Any,
    case: ParityCase,
    realized_offset_cm: float = 0.0,
) -> List[TraceRecord]:
    """Seat + drive the sim for one case and return its normalized record stream.

    Seats a fresh sim EXACTLY as the dump's ``seatAt`` does: localization on the
    case start, world frame bound at ``START_HEADING_DEG`` (app-yaw 0), and the
    chassis at the start node coordinates. For an offset case it then deposits
    the operator offset along world heading 0 (the sim counterpart of the C++
    ``pushOperatorOffset(mag, 0)``), so the return-to-node prologue fires. The
    deposited magnitude is the C++-REALIZED (encoder-quantized) value parsed from
    the dump's ``goal_start.realized_offset_cm`` — NOT the nominal table
    magnitude — so ``return_move.dist_cm`` is compared honestly (finding 1) rather
    than masked by an allowlist entry. Reaching into the documented-private
    localization node setter mirrors the established ``sim._field_data`` /
    ``sim._graph`` access pattern in parity.py.

    :param field: the fixture field-data namespace.
    :param graph: the fixture graph.
    :param next_hop: the all-pairs next-hop table.
    :param case: the case to run.
    :param realized_offset_cm: the C++-realized operator-offset magnitude to seed
        an offset case with (``0.0`` for clean cases, where ``case.offset`` is
        ``None`` and no offset is deposited).
    :return: the per-hop :class:`TraceRecord` list (no goal_start/goal_done).
    """
    chassis = VirtualChassis.from_field_data(field)
    sim = PathTrackerSim(field, graph, next_hop, chassis)

    # Seat at the case start (mirror of the dump's seatAt(start) / seatAt(start, 0)).
    sim.localization._current_node = case.start  # noqa: SLF001 — read-only sim access (cf. parity.py)
    sim.localization.zero_heading_from_current()
    start_x, start_y = field.NODES[case.start]
    chassis.x_cm = float(start_x)
    chassis.y_cm = float(start_y)
    chassis.heading_deg = _normalize(float(field.START_HEADING_DEG))

    # Offset cases: deposit the C++-realized operator offset along world heading 0.
    # The believed heading stays 0 (a translation push), so operator_move
    # decomposes the magnitude purely onto +x, matching pushOperatorOffset(mag, 0).
    # The nominal `case.offset` still asserts it is a pure +x push; the MAGNITUDE
    # deposited is the realized (encoder-quantized) value from the C++ stream, so
    # the return distance collapses to a match instead of being allowlisted.
    if case.offset is not None:
        dx, dy, dtheta = case.offset
        assert dy == 0.0 and dtheta == 0.0, (
            "the pinned parity offsets are pure +x pushes at world heading 0; "
            f"got dy={dy} dtheta={dtheta} for {case.case_id!r}"
        )
        sim.operator_move(realized_offset_cm)

    return normalize_sim_stream(sim, case)


# ---------------------------------------------------------------------------
# Diff result types.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _Mismatch:
    """One field-level decision divergence (case / record-index / field)."""

    case_id: str
    record_index: int
    event_kind: str
    field_path: str
    cpp_value: Any
    sim_value: Any

    def describe(self) -> str:
        return (
            f"{self.case_id} [rec #{self.record_index} {self.event_kind}] "
            f"{self.field_path}: cpp={self.cpp_value!r} sim={self.sim_value!r}"
        )


@dataclass
class _ParityRun:
    """The whole-sweep diff result (computed once by the module fixture)."""

    cpp_ids: Set[str]
    py_ids: Set[str]
    structural_failures: List[str] = dataclass_field(default_factory=list)
    unmatched_mismatches: List[_Mismatch] = dataclass_field(default_factory=list)
    allowlisted: List[Tuple[_Mismatch, AllowlistEntry]] = dataclass_field(default_factory=list)
    unbucketed_fields: Set[Tuple[str, str]] = dataclass_field(default_factory=set)
    pose_failures: List[str] = dataclass_field(default_factory=list)
    stale_entries: List[AllowlistEntry] = dataclass_field(default_factory=list)
    cpp_kinds_seen: Set[str] = dataclass_field(default_factory=set)
    sim_kinds_seen: Set[str] = dataclass_field(default_factory=set)
    n_cases_diffed: int = 0


def _diff_records(
    case: ParityCase,
    cpp_recs: List[TraceRecord],
    sim_recs: List[TraceRecord],
    allowlist: Allowlist,
    run: _ParityRun,
) -> bool:
    """Diff one case's record streams, recording structural + field divergences.

    EXACT first: the record-kind sequence (length + order) must match — a
    structural divergence is a hard failure that is NOT allowlistable (the
    allowlist is field-scoped). Then each field is mapped to its bucket and
    compared; an unbucketed field is recorded (a hard failure), and a bucket
    mismatch is checked against the allowlist (matched -> recorded as
    allowlisted, unmatched -> a real failure).

    :return: ``True`` iff the kind sequences matched (so the caller may run the
        pose cross-check on aligned streams).
    """
    cpp_kinds = [r.kind for r in cpp_recs]
    sim_kinds = [r.kind for r in sim_recs]
    if cpp_kinds != sim_kinds:
        run.structural_failures.append(
            f"{case.case_id}: record-kind sequence differs\n"
            f"    cpp ({len(cpp_kinds)}): {cpp_kinds}\n"
            f"    sim ({len(sim_kinds)}): {sim_kinds}"
        )
        return False

    for idx, (cpp_rec, sim_rec) in enumerate(zip(cpp_recs, sim_recs)):
        for fname in TRACE_RECORD_FIELDS[cpp_rec.kind]:
            bucket = _bucket_for_field(fname)
            if bucket is None:
                run.unbucketed_fields.add((cpp_rec.kind, fname))
                continue
            cpp_value = cpp_rec.fields[fname]
            sim_value = sim_rec.fields[fname]
            if _values_equal(bucket, cpp_value, sim_value):
                continue
            mismatch = _Mismatch(
                case_id=case.case_id,
                record_index=idx,
                event_kind=cpp_rec.kind,
                field_path=fname,
                cpp_value=cpp_value,
                sim_value=sim_value,
            )
            entry = allowlist.match(case.case_id, cpp_rec.kind, fname)
            if entry is None:
                run.unmatched_mismatches.append(mismatch)
            else:
                run.allowlisted.append((mismatch, entry))
    return True


def _diff_pose(
    case: ParityCase,
    cpp_recs: List[TraceRecord],
    sim_recs: List[TraceRecord],
    field: Any,
    next_hop: Any,
    run: _ParityRun,
) -> None:
    """Pose cross-check: integrate BOTH streams through the ONE chassis and diff.

    Both sides run the SAME :func:`integrate_pose` over the SAME node sequence
    (``walk_next_hop(next_hop, start, goal)[1:]`` — the per-hop destination
    nodes) seated at the case start, so there is no cross-language float drift.
    Position is compared at :data:`POSE_POS_TOL`, heading circularly at
    :data:`HEAD_TOL`. Run only on cases whose kind sequences already aligned.
    """
    node_sequence = walk_next_hop(next_hop, case.start, case.goal)[1:]
    cpp_poses = integrate_pose(
        cpp_recs, field, node_sequence,
        start_node=case.start, start_heading_deg=float(field.START_HEADING_DEG),
    )
    sim_poses = integrate_pose(
        sim_recs, field, node_sequence,
        start_node=case.start, start_heading_deg=float(field.START_HEADING_DEG),
    )
    for idx, (cpp_pose, sim_pose) in enumerate(zip(cpp_poses, sim_poses)):
        if (
            abs(cpp_pose.x - sim_pose.x) > POSE_POS_TOL
            or abs(cpp_pose.y - sim_pose.y) > POSE_POS_TOL
            or _circular_delta_deg(cpp_pose.heading, sim_pose.heading) > HEAD_TOL
        ):
            run.pose_failures.append(
                f"{case.case_id} [pose #{idx} after {cpp_recs[idx].kind}]: "
                f"cpp=({cpp_pose.x:.4f}, {cpp_pose.y:.4f}, {cpp_pose.heading:.4f}) "
                f"sim=({sim_pose.x:.4f}, {sim_pose.y:.4f}, {sim_pose.heading:.4f})"
            )


def _allowlist_path() -> Path:
    """Return the committed divergence allowlist path."""
    return Path(__file__).resolve().parent / "fixtures" / "parity_allowlist.json"


# ---------------------------------------------------------------------------
# The module-scoped run: locate + invoke the dump, run the sim, diff once.
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def parity_run() -> _ParityRun:
    """Run the whole sweep ONCE: invoke the dump, drive the sim, diff + pose.

    Locating the dump binary is the only place that can skip: a LOCAL run with
    no binary skips with a build hint, but in CI (``CI`` env var set) the
    dedicated ``parity`` job must have built it, so absence is a hard failure
    (never a silent green). Every other failure surfaces through the individual
    standing-guard test functions reading this result.
    """
    exe = _find_dump_exe()
    if exe is None:
        message = (
            "parity dump binary not found under build/. Build it with:\n"
            "  cmake -S WRO2026/test -B build/parity -G \"<your generator>\"\n"
            "  cmake --build build/parity --config Debug "
            "--target test_path_tracker_parity_dump\n"
            "(or point PARITY_DUMP_EXE at an existing build)."
        )
        if os.environ.get("CI"):
            pytest.fail(message)  # the parity CI job builds it first — absence is a bug.
        pytest.skip(message)

    cpp_cases = load_cpp_ndjson(_run_dump(exe))  # loud on any schema desync (no silent skip)

    field, graph, next_hop = _build_fixture()
    cases = _enumerate_cases(field, next_hop)
    allowlist = load_allowlist(_allowlist_path())

    run = _ParityRun(
        cpp_ids=set(cpp_cases),
        py_ids={c.case_id for c in cases},
    )

    shared_ids = run.cpp_ids & run.py_ids
    for case in cases:
        if case.case_id not in shared_ids:
            continue  # a one-sided case is reported by the case-count guard
        cpp_case = cpp_cases[case.case_id]
        cpp_recs = cpp_case.records
        sim_recs = _run_sim_case(
            field, graph, next_hop, case,
            realized_offset_cm=cpp_case.realized_offset_cm,
        )
        run.cpp_kinds_seen.update(r.kind for r in cpp_recs)
        run.sim_kinds_seen.update(r.kind for r in sim_recs)
        run.n_cases_diffed += 1
        aligned = _diff_records(case, cpp_recs, sim_recs, allowlist, run)
        if aligned:
            _diff_pose(case, cpp_recs, sim_recs, field, next_hop, run)

    run.stale_entries = allowlist.stale_entries()
    return run


# ---------------------------------------------------------------------------
# Standing guards (each a real, separate assertion — AC5).
# ---------------------------------------------------------------------------


def test_case_count_parity_and_nonzero(parity_run: _ParityRun) -> None:
    """The C++ and Python case sets must be the IDENTICAL, non-empty set (AC5).

    A case present on only one side is a hard fail (a desynced enumeration or a
    fixture drift); an empty intersection would be a vacuous pass.
    """
    assert parity_run.py_ids, "Python enumerated zero parity cases (vacuous)."
    assert parity_run.cpp_ids, "the C++ dump emitted zero cases (vacuous)."

    only_cpp = sorted(parity_run.cpp_ids - parity_run.py_ids)
    only_py = sorted(parity_run.py_ids - parity_run.cpp_ids)
    assert not only_cpp and not only_py, (
        "case-set mismatch between the C++ dump and the Python sweep:\n"
        f"  only in C++ ({len(only_cpp)}): {only_cpp}\n"
        f"  only in Python ({len(only_py)}): {only_py}"
    )
    assert parity_run.n_cases_diffed == len(parity_run.cpp_ids)


def test_no_structural_divergences(parity_run: _ParityRun) -> None:
    """Every case's record-kind sequence (length + order) must match exactly."""
    assert not parity_run.structural_failures, (
        f"{len(parity_run.structural_failures)} structural (kind-sequence) "
        "divergence(s):\n" + "\n".join(parity_run.structural_failures[:20])
    )


def test_every_trace_field_is_bucketed() -> None:
    """Every schema field maps to exactly one comparison bucket (AC5).

    A pure check over the whole :data:`TRACE_RECORD_FIELDS` schema (no binary
    needed): a field added later that the classifier cannot place fails HERE,
    so it can never silently escape comparison.
    """
    unbucketed = [
        f"{kind}.{fname}"
        for kind, spec in TRACE_RECORD_FIELDS.items()
        for fname in spec
        if _bucket_for_field(fname) is None
    ]
    assert not unbucketed, (
        "trace-record field(s) map to no comparison bucket (would escape the "
        f"diff): {unbucketed}"
    )


def test_no_unbucketed_fields_observed(parity_run: _ParityRun) -> None:
    """No field actually present in the live streams escaped bucketing."""
    assert not parity_run.unbucketed_fields, (
        "unbucketed field(s) observed in the live trace streams: "
        f"{sorted(parity_run.unbucketed_fields)}"
    )


def test_decision_streams_match_modulo_allowlist(parity_run: _ParityRun) -> None:
    """Decision records are identical between C++ and sim, modulo the allowlist (TC2).

    Any field-level divergence not matched by exactly one allowlist entry fails
    here with the exact case / record-index / field / cpp-value / sim-value.
    """
    assert not parity_run.unmatched_mismatches, (
        f"{len(parity_run.unmatched_mismatches)} non-allowlisted decision "
        "divergence(s):\n"
        + "\n".join(m.describe() for m in parity_run.unmatched_mismatches[:40])
    )


def test_return_offset_divergence_is_collapsed_not_allowlisted(
    parity_run: _ParityRun,
) -> None:
    """The return-offset divergence is collapsed by the realized seed, not allowlisted.

    Finding 1 (issue #116) seeds the sim with the C++-REALIZED (encoder-quantized)
    operator offset emitted on each ``goal_start``, so ``return_move.dist_cm`` now
    matches within ``DIST_TOL`` instead of being masked by the deleted
    ``return-move-offset-encoder-quantization`` allowlist entry. This guard
    preserves the original test's anti-vacuous intent — it proves the ``off_*``
    comparison actually RAN — while asserting the divergence is genuinely gone,
    not silently re-hidden:

    * (a) ``return_move`` appears in BOTH the C++ and sim record streams (the
      return-to-node prologue fired and was normalized on both sides);
    * (b) no allowlisted divergence carries the deleted entry's id; and
    * (c) no unmatched ``return_move.dist_cm`` mismatch remains.
    """
    # (a) Anti-vacuous: the off_* return-to-node prologue actually ran on both sides.
    assert "return_move" in parity_run.cpp_kinds_seen, (
        "the C++ sweep never emitted a return_move record — the off_* "
        "return-to-node cases did not run (a vacuous pass)."
    )
    assert "return_move" in parity_run.sim_kinds_seen, (
        "the sim sweep never emitted a return_move record — the off_* "
        "return-to-node cases did not run (a vacuous pass)."
    )

    # (b) The deleted entry must not fire (it is gone from the allowlist file).
    revived = [
        m.describe()
        for (m, entry) in parity_run.allowlisted
        if entry.id == "return-move-offset-encoder-quantization"
    ]
    assert not revived, (
        "the return-move-offset-encoder-quantization allowlist entry matched a "
        "divergence — it should be deleted now the realized-offset seed collapses "
        "the return distance:\n" + "\n".join(revived)
    )

    # (c) No residual return_move.dist_cm divergence (it collapsed within DIST_TOL).
    residual = [
        m.describe()
        for m in parity_run.unmatched_mismatches
        if m.event_kind == "return_move" and m.field_path == "dist_cm"
    ]
    assert not residual, (
        "return_move.dist_cm still diverges after seeding the realized offset "
        "(expected collapse within DIST_TOL — do NOT re-add an allowlist entry; "
        "see the plan's Error Handling):\n" + "\n".join(residual)
    )


def test_pose_cross_check(parity_run: _ParityRun) -> None:
    """Per-record pose matches between C++ and sim within tolerance (TC3)."""
    assert not parity_run.pose_failures, (
        f"{len(parity_run.pose_failures)} pose divergence(s):\n"
        + "\n".join(parity_run.pose_failures[:20])
    )


def test_coverage_every_record_kind_present(parity_run: _ParityRun) -> None:
    """Every per-hop record kind appears >=1x across BOTH sweeps (AC8/TC6).

    The pinned-offset curated cases make ``return_turn``/``return_move``
    reachable; the reset cases make ``wall_*`` reachable. A missing kind fails
    loudly rather than passing vacuously.
    """
    missing_cpp = sorted(PER_HOP_RECORD_KINDS - parity_run.cpp_kinds_seen)
    missing_sim = sorted(PER_HOP_RECORD_KINDS - parity_run.sim_kinds_seen)
    assert not missing_cpp, f"C++ sweep never exercised record kind(s): {missing_cpp}"
    assert not missing_sim, f"sim sweep never exercised record kind(s): {missing_sim}"


def test_reverse_pivot_is_exercised_with_flipped_wheel() -> None:
    """The swept reverse-pivot cluster drives a REAR pivot with a flipped wheel (#54, T5).

    Anti-vacuous companion to the whole-sweep guards: a PURE check (no dump
    binary) that ``field_fixture``'s isolated ``RevA..RevD`` cluster actually
    puts a genuine reverse pivot into the parity sweep -- so a future edit that
    drops or defangs it fails HERE rather than passing vacuously (the reverse
    pivot shares record KINDS with the forward pivots, so
    :func:`test_coverage_every_record_kind_present` alone cannot catch it).

    Drives the ``RevB -> RevC -> RevD`` route through the SAME sim path
    :func:`_run_sim_case` feeds the diff and asserts the pivot fired with:

    * the **REAR-flipped** wheel -- the stored forward wheel for the shallow
      ``+36.87 deg`` turn is ``PIVOT_LEFT``; a REAR traversal L<->R flips it to
      ``PIVOT_RIGHT`` via ``resolve_pivot_wheel`` (the flip the parity normalizer
      must apply so the sim's turn label matches the firmware dump's); and
    * a **REAR** leg (``reverse == True``) -- force-SAME runs both pivot legs in
      the approach (REAR) direction.

    Runs in the pytest-only CI job (no C++ toolchain) since it needs only the
    Python sim. The ``RevB``/``RevC``/``RevD`` names are pinned by this test.
    """
    field, graph, next_hop = _build_fixture()

    # The cluster route that takes the declared pivot (RevB, RevC, RevD).
    assert walk_next_hop(next_hop, "RevB", "RevD") == ["RevB", "RevC", "RevD"], (
        "the reverse-pivot cluster route RevB->RevC->RevD is not the resolved "
        "NEXT_HOP path -- field_fixture's reverse pivot is not being exercised"
    )

    case = ParityCase(
        case_id="RevB__RevD__reverse_pivot", start="RevB", goal="RevD"
    )
    records = _run_sim_case(field, graph, next_hop, case)
    kinds = [r.kind for r in records]

    turns = [r for r in records if r.kind == "turn"]
    assert turns, f"reverse-pivot route emitted no pivot turn record; got {kinds}"
    assert any(t.fields["wheel"] == "PIVOT_RIGHT" for t in turns), (
        "reverse pivot did not flip the wheel to PIVOT_RIGHT (stored forward "
        f"wheel is PIVOT_LEFT): got {[t.fields['wheel'] for t in turns]}"
    )
    assert any(r.fields.get("reverse") is True for r in records), (
        "reverse-pivot route drove no REAR leg (reverse=True absent) -- "
        f"force-SAME should run both legs REAR; got kinds {kinds}"
    )


def test_no_stale_allowlist_entries(parity_run: _ParityRun) -> None:
    """An allowlist entry that matched zero mismatches is stale and fails (TC7)."""
    assert not parity_run.stale_entries, (
        "stale allowlist entr(y/ies) matched nothing this run (remove them): "
        f"{[e.id for e in parity_run.stale_entries]}"
    )


def test_allowlist_match_is_case_sensitive_glob() -> None:
    """`Allowlist.match` globs the case id case-sensitively on every OS (TC5).

    Pure unit test (no dump binary): build a small allowlist directly and assert
    that a same-case selector matches while a case-mismatched selector does NOT.
    This proves `Allowlist.match` uses `fnmatchcase` rather than `fnmatch` —
    `fnmatch` normcases the path, so it is case-insensitive on Windows and the
    lowercase selector below would wrongly match there. `fnmatchcase` keeps the
    behavior identical (case-sensitive) on Windows and POSIX alike.
    """
    matching_entry = AllowlistEntry(
        id="case-sensitive-match-probe",
        case_selector="Inter2__*",
        event_kind="return_move",
        field_path="dist_cm",
        side="cpp",
        reason="unit-test fixture: same-case selector should match",
        tracking_issue="#116",
    )
    matching_allowlist = Allowlist([matching_entry])
    assert (
        matching_allowlist.match("Inter2__Inter3__clean", "return_move", "dist_cm")
        is matching_entry
    ), "a same-case selector should match its case id"

    mismatched_entry = AllowlistEntry(
        id="case-mismatch-no-match-probe",
        case_selector="inter2__*",
        event_kind="return_move",
        field_path="dist_cm",
        side="cpp",
        reason="unit-test fixture: lowercase selector should NOT match",
        tracking_issue="#116",
    )
    mismatched_allowlist = Allowlist([mismatched_entry])
    assert (
        mismatched_allowlist.match("Inter2__Inter3__clean", "return_move", "dist_cm")
        is None
    ), "a case-mismatched selector must not match (case-sensitive on every OS)"


def _goal_start_obj(
    case_id: str,
    start: str,
    goal: str,
    realized_offset_cm: float,
) -> Dict[str, Any]:
    """Build a complete, schema-v3 ``goal_start`` JSON object (dict) for a test."""
    return {
        "schema_version": PARITY_SCHEMA_VERSION,
        "kind": "goal_start",
        "case_id": case_id,
        "start": start,
        "goal": goal,
        "route_dir": "AUTO",
        "arrival_dir": "AUTO",
        "realized_offset_cm": realized_offset_cm,
    }


def test_goal_start_requires_realized_offset_cm() -> None:
    """A ``goal_start`` record must carry ``realized_offset_cm`` (TC6, schema v3).

    Pure unit test (no dump binary): :func:`_record_from_obj` validates the
    ``goal_start`` field set for EXACT equality, so the realized-offset field
    finding 1 added is REQUIRED, not optional. A ``goal_start`` missing it raises
    :class:`ParitySchemaError`; one carrying it (with ``schema_version == 3`` and
    every other ``goal_start`` field) parses into a :class:`TraceRecord`. The
    schema is now v3 (issue #156 added ``follow_mode`` + renamed ``line_mode`` ->
    ``speed`` on the follow records); ``goal_start``'s own field set is unchanged.
    """
    assert PARITY_SCHEMA_VERSION == 3, (
        "issue #156 bumps the parity schema to v3 (follow_mode + line_mode->speed)"
    )

    full = _goal_start_obj("Inter2__Inter3__off_n14_h0", "Inter2", "Inter3", 14.0)

    # Missing realized_offset_cm -> strict field-set mismatch -> loud failure.
    without_field = {k: v for k, v in full.items() if k != "realized_offset_cm"}
    with pytest.raises(ParitySchemaError):
        _record_from_obj(without_field)

    # With the field (+ schema_version 3 + every other goal_start field) -> parses.
    record = _record_from_obj(full)
    assert record.kind == "goal_start"
    assert record.fields["realized_offset_cm"] == pytest.approx(14.0)


def test_load_cpp_ndjson_surfaces_realized_offset() -> None:
    """``load_cpp_ndjson`` returns a :class:`CppCase` carrying the realized offset (TC6).

    Pure unit test: a minimal NDJSON stream (one offset case with a per-hop body,
    one clean case) parses into :class:`CppCase` values whose ``records`` is the
    per-hop body and whose ``realized_offset_cm`` is the value read off each
    case's ``goal_start`` — the return-shape finding 1 introduced.
    """
    blind = json.dumps(
        {
            "kind": "blind_dead_reckon",
            "reverse": False,
            "drive_mode": "GYRO_TSHAPE",
            "dist_cm": 13.999999,
            # ADR-059 field-set bump: a plain (non-chained, cold-start) hop —
            # start = TSHAPE_START_SPEED (50), braked end 0, chain false.
            "start_pwm": 50,
            "end_pwm": 0,
            "chain": False,
            "reanchor": "ON_DEAD_RECKONED_ADVANCE",
        }
    )
    goal_done = json.dumps({"kind": "goal_done", "final": "ProjB"})
    text = "\n".join(
        [
            json.dumps(_goal_start_obj("Zed__ProjB__off_n6_h0", "Zed", "ProjB", 5.999998)),
            blind,
            goal_done,
            json.dumps(_goal_start_obj("Zed__ProjB__clean", "Zed", "ProjB", 0.0)),
            goal_done,
        ]
    )

    cases = load_cpp_ndjson(text)
    assert set(cases) == {"Zed__ProjB__off_n6_h0", "Zed__ProjB__clean"}

    offset_case = cases["Zed__ProjB__off_n6_h0"]
    assert isinstance(offset_case, CppCase)
    assert offset_case.realized_offset_cm == pytest.approx(5.999998)
    assert [r.kind for r in offset_case.records] == ["blind_dead_reckon"]

    clean_case = cases["Zed__ProjB__clean"]
    assert clean_case.realized_offset_cm == 0.0
    assert clean_case.records == []
