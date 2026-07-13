"""Generate ``edge_dispatch_truth_table.csv`` -- the canonical edge-dispatch truth table.

This module is the THIRD, INDEPENDENT transcription of the terminal one-hop
edge-dispatch classification. It does NOT call
``tools.path_planner.sim.runtime.resolve_edge_dispatch`` (the Python resolver)
nor the C++ ``EdgeDispatchPolicy.h::resolveEdgeDispatch`` (the firmware
resolver), and it does NOT call the composed pure policies
(``resolve_pivot_trim`` / ``resolve_sensor_overshoot_trim``). It hand-encodes
the rule list itself -- including the trim-clamp arithmetic -- so the fixture it
emits can catch a bug in BOTH language resolvers at once: if either resolver
drifts from the spec, its parity test fails against this table, and if THIS
table drifts, both parity tests fail together (a louder signal than a
single-language bug). This mirrors ``gen_stop_at_line_truth_table.py`` exactly.

The emitted CSV is the normative single source consumed by:

* TC1 (Python) -- ``test_runtime_sim.py`` drives every row through
  ``resolve_edge_dispatch`` and asserts the seven decision fields.
* The C++ parity test -- ``test_edge_dispatch_truth_table.cpp`` drives every row
  through ``resolveEdgeDispatch`` and asserts the same seven fields.

Column schema (named header, consumed by ``csv.DictReader`` / ``wro_csv``)::

    INPUTS (the EdgeDispatchQuery fields):
      edgeType            LINE | BLIND
      destNodeType        LINE | PROJECTION  (the dest NodeType; for a BLIND row
                          this is a fixed sentinel -- see the prune rules)
      direction           FRONT | REAR
      rawDistanceCm       float -- edge.distance_cm
      lineMode            NORMAL | SLOW | FAST (the resolved speed tier; carried in
                          the query but NEVER read by the decision logic. The CSV
                          column keeps its historical `lineMode` name -- the
                          SpeedTier member spellings are byte-identical, ADR-064)
      seek                true | false -- the resolveStopAtLine seek result
                          (BLIND only; sentinel `false` on a LINE row)
      motionMode          SYNC_TSHAPE | GYRO_TSHAPE | RAW_FLAT -- the
                          resolveStopAtLine motion bundle (BLIND only; sentinel
                          `SYNC_TSHAPE` on a LINE row)
      projPassThrough     true | false -- FSM pass-through lookahead (LINE ->
                          projection only; sentinel `false` on a BLIND row)
      departingTrimCm     float -- outgoing pivot trim, or 0
      approachingTrimCm   float -- approach pivot trim, or 0
      axleOffsetCm        float -- direction-resolved sensor-to-axle offset
                          (FRONT 12.02 / REAR 7.68); read only for the seek
      minDriveCm          float -- PIVOT_TRIM_MIN_DRIVE_CM underflow floor (0.5)

    OUTPUTS (the EdgeDispatchDecision fields):
      expect_primitive            LINE_FOLLOW_INTERSECTION | PROJECTION_FOLLOW |
                                  STOP_AT_LINE_SEEK | BLIND_DEAD_RECKON
      expect_reverse              true | false
      expect_driveMode            DEFAULT (line) | SYNC_TSHAPE | GYRO_TSHAPE |
                                  RAW_FLAT (blind)
      expect_commandedDistanceCm  float -- POST-clamp commanded value (NOT the
                                  physical displacement)
      expect_overshootCm          float -- seek overshoot channel (0 otherwise)
      expect_reanchor             ON_INTERSECTION | ON_DEAD_RECKONED_ADVANCE
      expect_brakeOnGate          true | false

Prune rules (a finite, meaningful grid -- impossible combos are dropped, and a
dimension a given edge type never reads is pinned to a documented sentinel so the
schema stays one uniform 19-column table):

* The grid is the UNION of a LINE block and a BLIND block. The two blocks
  partition cleanly because ``edgeType`` selects which inputs the resolver reads.
* LINE block: iterates the dimensions a LINE hop reads -- ``destNodeType``
  (LINE -> follow-to-intersection vs PROJECTION -> distance-gated follow),
  ``direction`` (reverse), ``rawDistanceCm`` / ``departingTrimCm`` /
  ``approachingTrimCm`` (the projection trim-clamp), ``projPassThrough`` (the
  projection brake-on-gate), and ``lineMode`` (proves it is NEVER read: the
  decision is invariant across all three values). The blind-only inputs are
  pinned: ``seek=false``, ``motionMode=SYNC_TSHAPE``, ``axleOffsetCm=12.02``.
* BLIND block: iterates the dimensions a BLIND hop reads -- ``direction``,
  ``seek`` (seek vs dead-reckon), ``motionMode`` (the drive bundle),
  ``rawDistanceCm`` / ``departingTrimCm`` / ``approachingTrimCm`` (the
  dead-reckon trim-clamp), and ``axleOffsetCm`` (the seek overshoot, two values
  to exercise both the FRONT and REAR offsets). The line-only inputs are pinned:
  ``destNodeType=LINE`` (a non-PROJECTION sentinel the resolver ignores for
  BLIND), ``projPassThrough=false``, ``lineMode=NORMAL``.
* ``minDriveCm`` is a single Config constant (0.5), pinned everywhere but still
  emitted as a column so the C++ harness is fully data-driven (it never reads a
  Config symbol).
* Clamp coverage is built in: the smallest ``rawDistanceCm`` (0.6) with non-zero
  trims drives ``raw - departing - approaching`` below the 0.5 floor, and the
  largest ``approachingTrimCm`` (12.5) drives ``axleOffset - approaching`` below
  the floor for the seek overshoot.

Value spellings mirror the enum MEMBER NAMES exactly so each test's
string->enum map consumes them without translation (the C++ token spellings of
``EdgeType`` / ``NodeType`` / ``Direction`` / ``SpeedTier`` / ``BlindMode`` /
``DispatchPrimitive`` / ``ReanchorKind``). Floats are emitted with a fixed
6-decimal format -- both readers parse them back with ``float`` and compare with
a tiny tolerance, never as strings.

The table is written DETERMINISTICALLY: nested loops in a fixed dimension order,
no date stamp, no randomness. Re-running the generator on an unchanged spec
reproduces a byte-identical CSV.

Run with::

    uv run --extra dev python tools/path_planner/tests/fixtures/gen_edge_dispatch_truth_table.py
"""

from __future__ import annotations

import csv
import pathlib
from typing import List, Tuple

# ---------------------------------------------------------------------------
# Domain spellings (hand-mirrored enum member names -- NOT imported, so this
# transcription stays independent of the resolver and its enum modules).
# ---------------------------------------------------------------------------

# Routes::EdgeType member names that are routed (NONE is never dispatched here).
EDGE_TYPES: Tuple[str, ...] = ("LINE", "BLIND")

# Routes::NodeType member names a routed edge can target. The resolver only ever
# tests `== PROJECTION`, so LINE stands in for "any non-projection dest".
DEST_NODE_TYPES: Tuple[str, ...] = ("LINE", "PROJECTION")

# Direction member names (post-resolveDirection; never AUTO).
DIRECTIONS: Tuple[str, ...] = ("FRONT", "REAR")

# Routes::SpeedTier member names (resolved; never DEFAULT). Carried by the query
# but never read by the decision -- iterated on the LINE block to prove that. The
# CSV column keeps its historical `lineMode` spelling (byte-identical data); only
# the C++ query field it feeds was renamed to `speed:SpeedTier` (ADR-064).
LINE_MODES: Tuple[str, ...] = ("NORMAL", "SLOW", "FAST")

# Routes::BlindMode motion bundles a resolveStopAtLine result can carry -- never
# DEFAULT and never STOP_AT_LINE (the resolver collapses those upstream).
MOTION_MODES: Tuple[str, ...] = ("SYNC_TSHAPE", "GYRO_TSHAPE", "RAW_FLAT")

# resolveStopAtLine seek result (BLIND block).
SEEK_FLAGS: Tuple[bool, ...] = (False, True)

# Projection pass-through lookahead (LINE block).
PASS_THROUGH_FLAGS: Tuple[bool, ...] = (False, True)

# Raw edge length: a tiny value that clamps under the trims, plus two realistic
# lengths that never clamp.
RAW_DISTANCES_CM: Tuple[float, ...] = (0.6, 40.0, 150.0)

# Departing pivot trim: none, and a value that participates in the clamp.
DEPARTING_TRIMS_CM: Tuple[float, ...] = (0.0, 1.5)

# Approaching pivot trim: none, a mid value, and a value large enough to drive
# the seek overshoot (axleOffset - approaching) below the floor.
APPROACHING_TRIMS_CM: Tuple[float, ...] = (0.0, 2.5, 12.5)

# Direction-resolved sensor-to-axle offsets: FRONT (LINE_SENSOR_AXLE_OFFSET_CM)
# and REAR (LINE_SENSOR_AXLE_OFFSET_REAR_CM). Read only by the seek overshoot.
AXLE_OFFSETS_CM: Tuple[float, ...] = (12.02, 7.68)

# PIVOT_TRIM_MIN_DRIVE_CM underflow floor -- a single Config constant.
MIN_DRIVE_CM: float = 0.5

# --- Sentinels for inputs a given edge type never reads. -------------------
# LINE rows pin the blind-only inputs; BLIND rows pin the line-only inputs.
_LINE_SENTINEL_SEEK = False
_LINE_SENTINEL_MOTION = "SYNC_TSHAPE"
_LINE_SENTINEL_AXLE_CM = 12.02

_BLIND_SENTINEL_DEST = "LINE"           # any non-PROJECTION NodeType; ignored for BLIND
_BLIND_SENTINEL_PASS_THROUGH = False
_BLIND_SENTINEL_LINE_MODE = "NORMAL"

# CSV header -- exact column order (inputs then expected outputs).
HEADER: Tuple[str, ...] = (
    # inputs
    "edgeType",
    "destNodeType",
    "direction",
    "rawDistanceCm",
    "lineMode",
    "seek",
    "motionMode",
    "projPassThrough",
    "departingTrimCm",
    "approachingTrimCm",
    "axleOffsetCm",
    "minDriveCm",
    # expected outputs
    "expect_primitive",
    "expect_reverse",
    "expect_driveMode",
    "expect_commandedDistanceCm",
    "expect_overshootCm",
    "expect_reanchor",
    "expect_brakeOnGate",
)


def _bool_str(value: bool) -> str:
    """Render a Python bool as the lowercase CSV token the consumers expect."""
    return "true" if value else "false"


def _fmt_float(value: float) -> str:
    """Render a float in the fixed 6-decimal text format both readers parse.

    Deterministic and platform-independent -- the same value always produces the
    same string, so re-running the generator yields a byte-identical CSV.
    """
    return f"{value:.6f}"


def _clamp_floor(value: float, floor: float) -> float:
    """Clamp @p value up to @p floor -- the trim-clamp `max(value, floor)`.

    Hand-written here (NOT delegated to ``resolve_pivot_trim``) so this table is
    an INDEPENDENT transcription of the trim-clamp arithmetic, not a tautology.
    """
    return value if value >= floor else floor


def _compute_row(
    edge_type: str,
    dest_node_type: str,
    direction: str,
    raw_distance_cm: float,
    line_mode: str,
    seek: bool,
    motion_mode: str,
    proj_pass_through: bool,
    departing_trim_cm: float,
    approaching_trim_cm: float,
    axle_offset_cm: float,
    min_drive_cm: float,
) -> Tuple[str, ...]:
    """Hand-encode one truth-table row from the edge-dispatch rule.

    This is the INDEPENDENT transcription: it reproduces the spec's rule list
    directly, never calling either resolver or the composed pure policies. Keep
    it step-for-step with ``resolveEdgeDispatch`` / ``resolve_edge_dispatch`` so a
    reviewer can diff the three by eye.

    :return: a tuple of CSV cells in :data:`HEADER` order.
    """
    reverse = direction == "REAR"

    if edge_type == "LINE":
        if dest_node_type == "PROJECTION":
            # LINE -> projection: distance-gated follow over the trim-clamped
            # edge length; brake on the gate unless the FSM collapsed a
            # line-proj-line pass-through.
            primitive = "PROJECTION_FOLLOW"
            commanded_distance_cm = _clamp_floor(
                raw_distance_cm - departing_trim_cm - approaching_trim_cm,
                min_drive_cm,
            )
            overshoot_cm = 0.0
            reanchor = "ON_DEAD_RECKONED_ADVANCE"
            brake_on_gate = not proj_pass_through
            drive_mode = "DEFAULT"
        else:
            # LINE -> line (junction): sensor-terminated. The LINE_OVERSHOOT
            # advance is owned by the FSM, NOT this classifier, so the commanded
            # distance and overshoot are both zero here.
            primitive = "LINE_FOLLOW_INTERSECTION"
            commanded_distance_cm = 0.0
            overshoot_cm = 0.0
            reanchor = "ON_INTERSECTION"
            brake_on_gate = True
            drive_mode = "DEFAULT"
    else:    # BLIND
        if seek:
            # BLIND + seek: the crossing terminates the drive, so the commanded
            # distance stays the FULL raw value; the approaching trim folds into
            # the separate overshoot channel (axleOffset - approaching, clamped).
            primitive = "STOP_AT_LINE_SEEK"
            commanded_distance_cm = raw_distance_cm
            overshoot_cm = _clamp_floor(
                axle_offset_cm - approaching_trim_cm,
                min_drive_cm,
            )
            reanchor = "ON_INTERSECTION"
            brake_on_gate = True
            drive_mode = motion_mode
        else:
            # BLIND dead-reckon: distance-terminated, so the commanded distance
            # is the two-trim subtract-and-clamp (the same core the projection
            # hop uses).
            primitive = "BLIND_DEAD_RECKON"
            commanded_distance_cm = _clamp_floor(
                raw_distance_cm - departing_trim_cm - approaching_trim_cm,
                min_drive_cm,
            )
            overshoot_cm = 0.0
            reanchor = "ON_DEAD_RECKONED_ADVANCE"
            brake_on_gate = True
            drive_mode = motion_mode

    return (
        # inputs
        edge_type,
        dest_node_type,
        direction,
        _fmt_float(raw_distance_cm),
        line_mode,
        _bool_str(seek),
        motion_mode,
        _bool_str(proj_pass_through),
        _fmt_float(departing_trim_cm),
        _fmt_float(approaching_trim_cm),
        _fmt_float(axle_offset_cm),
        _fmt_float(min_drive_cm),
        # expected outputs
        primitive,
        _bool_str(reverse),
        drive_mode,
        _fmt_float(commanded_distance_cm),
        _fmt_float(overshoot_cm),
        reanchor,
        _bool_str(brake_on_gate),
    )


def _build_line_rows() -> List[Tuple[str, ...]]:
    """Build the LINE block: full cross-product over the dimensions a LINE hop reads.

    Dimension order (outer -> inner) is fixed for deterministic output:
    destNodeType, direction, rawDistanceCm, lineMode, projPassThrough,
    departingTrimCm, approachingTrimCm. The blind-only inputs (seek, motionMode,
    axleOffsetCm) are pinned to sentinels the LINE branch never reads.

    Count = 2 (destNodeType) x 2 (direction) x 3 (rawDistanceCm) x 3 (lineMode)
          x 2 (projPassThrough) x 2 (departingTrimCm) x 3 (approachingTrimCm) = 432.
    """
    rows: List[Tuple[str, ...]] = []
    for dest_node_type in DEST_NODE_TYPES:
        for direction in DIRECTIONS:
            for raw_distance_cm in RAW_DISTANCES_CM:
                for line_mode in LINE_MODES:
                    for proj_pass_through in PASS_THROUGH_FLAGS:
                        for departing_trim_cm in DEPARTING_TRIMS_CM:
                            for approaching_trim_cm in APPROACHING_TRIMS_CM:
                                rows.append(
                                    _compute_row(
                                        edge_type="LINE",
                                        dest_node_type=dest_node_type,
                                        direction=direction,
                                        raw_distance_cm=raw_distance_cm,
                                        line_mode=line_mode,
                                        seek=_LINE_SENTINEL_SEEK,
                                        motion_mode=_LINE_SENTINEL_MOTION,
                                        proj_pass_through=proj_pass_through,
                                        departing_trim_cm=departing_trim_cm,
                                        approaching_trim_cm=approaching_trim_cm,
                                        axle_offset_cm=_LINE_SENTINEL_AXLE_CM,
                                        min_drive_cm=MIN_DRIVE_CM,
                                    )
                                )
    return rows


def _build_blind_rows() -> List[Tuple[str, ...]]:
    """Build the BLIND block: full cross-product over the dimensions a BLIND hop reads.

    Dimension order (outer -> inner) is fixed for deterministic output:
    direction, seek, motionMode, rawDistanceCm, departingTrimCm,
    approachingTrimCm, axleOffsetCm. The line-only inputs (destNodeType,
    projPassThrough, lineMode) are pinned to sentinels the BLIND branch never
    reads.

    Count = 2 (direction) x 2 (seek) x 3 (motionMode) x 3 (rawDistanceCm)
          x 2 (departingTrimCm) x 3 (approachingTrimCm) x 2 (axleOffsetCm) = 432.
    """
    rows: List[Tuple[str, ...]] = []
    for direction in DIRECTIONS:
        for seek in SEEK_FLAGS:
            for motion_mode in MOTION_MODES:
                for raw_distance_cm in RAW_DISTANCES_CM:
                    for departing_trim_cm in DEPARTING_TRIMS_CM:
                        for approaching_trim_cm in APPROACHING_TRIMS_CM:
                            for axle_offset_cm in AXLE_OFFSETS_CM:
                                rows.append(
                                    _compute_row(
                                        edge_type="BLIND",
                                        dest_node_type=_BLIND_SENTINEL_DEST,
                                        direction=direction,
                                        raw_distance_cm=raw_distance_cm,
                                        line_mode=_BLIND_SENTINEL_LINE_MODE,
                                        seek=seek,
                                        motion_mode=motion_mode,
                                        proj_pass_through=_BLIND_SENTINEL_PASS_THROUGH,
                                        departing_trim_cm=departing_trim_cm,
                                        approaching_trim_cm=approaching_trim_cm,
                                        axle_offset_cm=axle_offset_cm,
                                        min_drive_cm=MIN_DRIVE_CM,
                                    )
                                )
    return rows


def build_rows() -> List[Tuple[str, ...]]:
    """Assemble the full truth table (LINE block + BLIND block) with self-checks.

    Returns the ordered row list. Raises ``AssertionError`` if any invariant
    fails -- the generator refuses to emit a malformed table.
    """
    line_rows = _build_line_rows()
    blind_rows = _build_blind_rows()

    expected_line = (
        len(DEST_NODE_TYPES)
        * len(DIRECTIONS)
        * len(RAW_DISTANCES_CM)
        * len(LINE_MODES)
        * len(PASS_THROUGH_FLAGS)
        * len(DEPARTING_TRIMS_CM)
        * len(APPROACHING_TRIMS_CM)
    )
    assert len(line_rows) == expected_line, (
        f"LINE block must be exactly {expected_line} rows, got {len(line_rows)}"
    )

    expected_blind = (
        len(DIRECTIONS)
        * len(SEEK_FLAGS)
        * len(MOTION_MODES)
        * len(RAW_DISTANCES_CM)
        * len(DEPARTING_TRIMS_CM)
        * len(APPROACHING_TRIMS_CM)
        * len(AXLE_OFFSETS_CM)
    )
    assert len(blind_rows) == expected_blind, (
        f"BLIND block must be exactly {expected_blind} rows, got {len(blind_rows)}"
    )

    all_rows = line_rows + blind_rows
    assert len(all_rows) == len(line_rows) + len(blind_rows), (
        f"TOTAL row count {len(all_rows)} != LINE {len(line_rows)} "
        f"+ BLIND {len(blind_rows)}"
    )

    return all_rows


def expected_row_count() -> int:
    """Return the total row count derived from the dimension products.

    The parity tests guard against a truncated CSV by asserting the data-row
    count equals this number; it is computed, never hardcoded.
    """
    line = (
        len(DEST_NODE_TYPES)
        * len(DIRECTIONS)
        * len(RAW_DISTANCES_CM)
        * len(LINE_MODES)
        * len(PASS_THROUGH_FLAGS)
        * len(DEPARTING_TRIMS_CM)
        * len(APPROACHING_TRIMS_CM)
    )
    blind = (
        len(DIRECTIONS)
        * len(SEEK_FLAGS)
        * len(MOTION_MODES)
        * len(RAW_DISTANCES_CM)
        * len(DEPARTING_TRIMS_CM)
        * len(APPROACHING_TRIMS_CM)
        * len(AXLE_OFFSETS_CM)
    )
    return line + blind


def write_csv(path: pathlib.Path) -> Tuple[int, int, int]:
    """Build and write the truth table to ``path``.

    :return: ``(line_count, blind_count, total_count)``.
    """
    all_rows = build_rows()

    # ``newline=""`` so the csv module owns line endings (no doubled CR on
    # Windows); deterministic output independent of host platform.
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(HEADER)
        writer.writerows(all_rows)

    line_count = (
        len(DEST_NODE_TYPES)
        * len(DIRECTIONS)
        * len(RAW_DISTANCES_CM)
        * len(LINE_MODES)
        * len(PASS_THROUGH_FLAGS)
        * len(DEPARTING_TRIMS_CM)
        * len(APPROACHING_TRIMS_CM)
    )
    blind_count = (
        len(DIRECTIONS)
        * len(SEEK_FLAGS)
        * len(MOTION_MODES)
        * len(RAW_DISTANCES_CM)
        * len(DEPARTING_TRIMS_CM)
        * len(APPROACHING_TRIMS_CM)
        * len(AXLE_OFFSETS_CM)
    )
    return line_count, blind_count, len(all_rows)


def main() -> None:
    """Regenerate ``edge_dispatch_truth_table.csv`` next to this script."""
    out_path = pathlib.Path(__file__).resolve().parent / "edge_dispatch_truth_table.csv"
    line_count, blind_count, total_count = write_csv(out_path)
    print(f"wrote {out_path}")
    print(f"  line  = {line_count}")
    print(f"  blind = {blind_count}")
    print(f"  total = {total_count}")


if __name__ == "__main__":
    main()
