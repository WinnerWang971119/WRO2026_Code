"""Generate ``stop_at_line_truth_table.csv`` -- the canonical stop-at-line truth table.

This module is the THIRD, INDEPENDENT transcription of the stop-at-line
precedence ladder. It does NOT call
``tools.path_planner.sim.runtime.resolve_stop_at_line`` (the Python resolver)
nor the C++ ``StopAtLinePolicy.h::resolveStopAtLine`` (the firmware resolver).
It encodes the rule list itself, by hand, so the fixture it emits can catch a
bug in BOTH language resolvers at once: if either resolver drifts from the
spec, its TCxx parity test fails against this table, and if THIS table drifts,
both parity tests fail together (a louder signal than a single-language bug).

The emitted CSV is the normative single source consumed by:

* TC13 (Python) -- ``test_runtime_sim.py`` drives every row through
  ``resolve_stop_at_line`` and asserts ``(seek, motion_mode)``.
* The C++ parity test -- drives every row through ``resolveStopAtLine``.

Column order (named header, consumed by ``csv.DictReader``)::

    callFlag,edgeOverride,edgeMode,pipelineBlind,direction,lineBoardCount,
    autoRuleActive,expect_seek,expect_motionMode

Value spellings mirror the enum MEMBER NAMES exactly so the test's
string->enum maps consume them without translation:

* ``callFlag``      -- ``StopAtLine`` names: AUTO / ON / OFF.
* ``edgeOverride``  -- ``StopAtLineMode`` names: DEFAULT / ON / OFF.
* ``edgeMode``,
  ``pipelineBlind``,
  ``expect_motionMode`` -- ``BlindMode`` names: SYNC_TSHAPE / GYRO_TSHAPE /
  RAW_FLAT / STOP_AT_LINE. (Note the ``_TSHAPE`` / ``_FLAT`` suffixes -- the
  task's shorthand "GYRO"/"RAW" are these full member names.)
* ``direction``     -- ``Direction`` names: FRONT / REAR.
* ``lineBoardCount``-- integer 1 or 2.
* ``autoRuleActive``, ``expect_seek`` -- lowercase ``true`` / ``false``
  (matching the legacy CSV's boolean spelling).

The table is written DETERMINISTICALLY: nested loops in a fixed dimension
order, no date stamp, no randomness. Re-running the generator on an unchanged
spec reproduces a byte-identical CSV.

Run with::

    uv run python tools/path_planner/tests/fixtures/gen_stop_at_line_truth_table.py
"""

from __future__ import annotations

import csv
import pathlib
from typing import List, Tuple

# ---------------------------------------------------------------------------
# Domain spellings (hand-mirrored enum member names -- NOT imported, so this
# transcription stays independent of the resolver and its enum module).
# ---------------------------------------------------------------------------

# StopAtLine (call-site verb) member names.
CALL_FLAGS: Tuple[str, ...] = ("AUTO", "ON", "OFF")

# StopAtLineMode (per-edge override) member names.
EDGE_OVERRIDES: Tuple[str, ...] = ("DEFAULT", "ON", "OFF")

# BlindMode member names. ``edgeMode`` spans all four (the resolved per-edge
# blind mode is never DEFAULT, but CAN be STOP_AT_LINE -- the legacy tag).
EDGE_MODES: Tuple[str, ...] = ("SYNC_TSHAPE", "GYRO_TSHAPE", "RAW_FLAT", "STOP_AT_LINE")

# ``pipelineBlind`` in the BASE block spans only the three non-STOP_AT_LINE,
# non-DEFAULT motion modes -- the mission-wide default is a concrete velocity
# profile, never the bare STOP_AT_LINE seek tag. (A STOP_AT_LINE pipeline
# default is exercised only in the targeted supplement, below.)
PIPELINE_BLINDS_BASE: Tuple[str, ...] = ("SYNC_TSHAPE", "GYRO_TSHAPE", "RAW_FLAT")

# Direction member names.
DIRECTIONS: Tuple[str, ...] = ("FRONT", "REAR")

# Board counts: 1-board (front-only) vs 2-board (bidirectional-capable).
LINE_BOARD_COUNTS: Tuple[int, ...] = (1, 2)

# Auto-rule active flag: the LOCAL Blind->Projection-node trigger
# (BLIND_PROJECTION_AUTO_SEEK and the edge's OWN destination is a projection
# node, issue #64 -- no goal-relative next-hop peek). The resolver only ever
# sees this resolved bool, so the truth table is unchanged by the move from the
# old goal-relative phrasing to the node-property one.
AUTO_RULE_FLAGS: Tuple[bool, ...] = (False, True)

# The bare seek tag whose presence in edgeMode forces a SYNC_TSHAPE motion.
_STOP_AT_LINE = "STOP_AT_LINE"
_SYNC_TSHAPE = "SYNC_TSHAPE"

# CSV header -- exact column order required by the DictReader consumers.
HEADER: Tuple[str, ...] = (
    "callFlag",
    "edgeOverride",
    "edgeMode",
    "pipelineBlind",
    "direction",
    "lineBoardCount",
    "autoRuleActive",
    "expect_seek",
    "expect_motionMode",
)


def _bool_str(value: bool) -> str:
    """Render a Python bool as the lowercase CSV token the consumers expect."""
    return "true" if value else "false"


def _compute_row(
    call_flag: str,
    edge_override: str,
    edge_mode: str,
    pipeline_blind: str,
    direction: str,
    line_board_count: int,
    auto_rule_active: bool,
) -> Tuple[str, ...]:
    """Hand-encode one truth-table row from the precedence + motion rules.

    This is the INDEPENDENT transcription: it reproduces the spec's rule list
    directly, never calling either resolver. Keep it step-for-step with the
    plan's rule list so a reviewer can diff the two by eye.

    :return: a tuple of CSV cells in :data:`HEADER` order.
    """
    # --- Precedence ladder: first match decides ``requested``. -------------
    requested = False
    if call_flag == "ON":
        requested = True
    elif call_flag == "OFF":
        requested = False
    elif edge_override == "ON":
        requested = True
    elif edge_override == "OFF":
        requested = False
    elif edge_mode == _STOP_AT_LINE:
        requested = True
    elif auto_rule_active:
        requested = True
    else:
        requested = False

    # --- A line board must face the travel direction. ----------------------
    # Front always faces forward; a rear seek needs a second board.
    board_ok = (direction == "FRONT") or (line_board_count >= 2)

    # --- The seek runs only when requested AND physically possible. --------
    expect_seek = requested and board_ok

    # --- Motion mode. ------------------------------------------------------
    if expect_seek:
        # Seeking: keep the edge's typed mode; a bare STOP_AT_LINE edge has no
        # velocity profile of its own, so it seeks under SYNC_TSHAPE.
        expect_motion_mode = _SYNC_TSHAPE if edge_mode == _STOP_AT_LINE else edge_mode
    else:
        # Dead-reckon fallback: a STOP_AT_LINE edge falls back to the pipeline
        # blind mode; any residual STOP_AT_LINE collapses to SYNC_TSHAPE so the
        # emitted mode is never STOP_AT_LINE.
        base = pipeline_blind if edge_mode == _STOP_AT_LINE else edge_mode
        expect_motion_mode = _SYNC_TSHAPE if base == _STOP_AT_LINE else base

    return (
        call_flag,
        edge_override,
        edge_mode,
        pipeline_blind,
        direction,
        str(line_board_count),
        _bool_str(auto_rule_active),
        _bool_str(expect_seek),
        expect_motion_mode,
    )


def _build_base_rows() -> List[Tuple[str, ...]]:
    """Build the BASE block: full cross-product over the seven base dimensions.

    Dimension order (outer -> inner) is fixed for deterministic output:
    callFlag, edgeOverride, edgeMode, pipelineBlind(base 3), direction,
    lineBoardCount, autoRuleActive.

    Count = 3 (callFlag) x 3 (edgeOverride) x 4 (edgeMode) x 3 (pipelineBlind)
          x 2 (direction) x 2 (lineBoardCount) x 2 (autoRuleActive) = 864.
    """
    rows: List[Tuple[str, ...]] = []
    for call_flag in CALL_FLAGS:
        for edge_override in EDGE_OVERRIDES:
            for edge_mode in EDGE_MODES:
                for pipeline_blind in PIPELINE_BLINDS_BASE:
                    for direction in DIRECTIONS:
                        for line_board_count in LINE_BOARD_COUNTS:
                            for auto_rule_active in AUTO_RULE_FLAGS:
                                rows.append(
                                    _compute_row(
                                        call_flag,
                                        edge_override,
                                        edge_mode,
                                        pipeline_blind,
                                        direction,
                                        line_board_count,
                                        auto_rule_active,
                                    )
                                )
    return rows


def _build_supplemental_rows() -> List[Tuple[str, ...]]:
    """Build the TARGETED SUPPLEMENT: ``pipelineBlind=STOP_AT_LINE`` rows that change output.

    ``pipelineBlind`` only feeds the output through the dead-reckon ``base``
    leg, and only when ``edgeMode == STOP_AT_LINE`` (otherwise ``base =
    edgeMode`` and pipelineBlind is ignored entirely). So the ONLY cells where a
    STOP_AT_LINE pipeline default can differ from the base's three pipelineBlind
    values are the non-seeking ``edgeMode == STOP_AT_LINE`` hops, where
    ``base = pipelineBlind = STOP_AT_LINE`` collapses to SYNC_TSHAPE.

    We therefore pin ``edgeMode = pipelineBlind = STOP_AT_LINE`` and iterate the
    other five dimensions (callFlag, edgeOverride, direction, lineBoardCount,
    autoRuleActive). Rows where the hop actually SEEKS are still emitted -- they
    are harmless duplicates of base behavior in the seek leg (pipelineBlind
    unused) but keeping the full five-dim sweep makes the supplement a clean,
    self-contained block rather than a conditionally-pruned one. The count is
    computed from the loop below, never hardcoded.

    Count = 3 (callFlag) x 3 (edgeOverride) x 2 (direction)
          x 2 (lineBoardCount) x 2 (autoRuleActive) = 72.
    """
    rows: List[Tuple[str, ...]] = []
    for call_flag in CALL_FLAGS:
        for edge_override in EDGE_OVERRIDES:
            for direction in DIRECTIONS:
                for line_board_count in LINE_BOARD_COUNTS:
                    for auto_rule_active in AUTO_RULE_FLAGS:
                        rows.append(
                            _compute_row(
                                call_flag,
                                edge_override,
                                _STOP_AT_LINE,   # edgeMode pinned
                                _STOP_AT_LINE,   # pipelineBlind pinned (the new value)
                                direction,
                                line_board_count,
                                auto_rule_active,
                            )
                        )
    return rows


def build_rows() -> List[Tuple[str, ...]]:
    """Assemble the full truth table (base + targeted supplement) with self-checks.

    Returns the ordered row list. Raises ``AssertionError`` if any invariant in
    the spec fails -- the generator refuses to emit a malformed table.
    """
    base_rows = _build_base_rows()
    supplemental_rows = _build_supplemental_rows()

    # --- Self-check: BASE block is exactly 864 rows. -----------------------
    # 3 callFlag x 3 edgeOverride x 4 edgeMode x 3 pipelineBlind(base)
    #   x 2 direction x 2 lineBoardCount x 2 autoRuleActive = 864.
    assert len(base_rows) == 864, (
        f"BASE block must be exactly 864 rows, got {len(base_rows)}"
    )

    # --- Self-check: supplemental size derived from the loop, documented. --
    # 3 callFlag x 3 edgeOverride x 2 direction x 2 lineBoardCount
    #   x 2 autoRuleActive = 72. (edgeMode + pipelineBlind are pinned.)
    expected_supplemental = (
        len(CALL_FLAGS)
        * len(EDGE_OVERRIDES)
        * len(DIRECTIONS)
        * len(LINE_BOARD_COUNTS)
        * len(AUTO_RULE_FLAGS)
    )
    assert len(supplemental_rows) == expected_supplemental, (
        f"SUPPLEMENT must be {expected_supplemental} rows, "
        f"got {len(supplemental_rows)}"
    )

    all_rows = base_rows + supplemental_rows

    # --- Self-check: TOTAL == base + supplemental. -------------------------
    # Total = 864 + 72 = 936.
    assert len(all_rows) == len(base_rows) + len(supplemental_rows), (
        f"TOTAL row count {len(all_rows)} != base {len(base_rows)} "
        f"+ supplemental {len(supplemental_rows)}"
    )

    # --- Self-check: legacy 5-input slice is exactly 144 rows. -------------
    # The pre-feature resolver had 5 inputs (no edgeOverride, no autoRuleActive):
    # 3 callFlag x 4 edgeMode x 3 pipelineBlind(base) x 2 direction
    #   x 2 lineBoardCount = 144. Restricting the BASE block to
    # edgeOverride==DEFAULT and autoRuleActive==False (and the base pipelineBlind
    # values, which the supplement is excluded from) must reproduce that domain.
    _IDX_EDGE_OVERRIDE = HEADER.index("edgeOverride")
    _IDX_AUTO_RULE = HEADER.index("autoRuleActive")
    legacy_slice = [
        row
        for row in base_rows
        if row[_IDX_EDGE_OVERRIDE] == "DEFAULT"
        and row[_IDX_AUTO_RULE] == "false"
    ]
    assert len(legacy_slice) == 144, (
        f"legacy (edgeOverride==DEFAULT, autoRuleActive==False) slice must be "
        f"exactly 144 rows, got {len(legacy_slice)}"
    )

    return all_rows


def write_csv(path: pathlib.Path) -> Tuple[int, int, int]:
    """Build and write the truth table to ``path``.

    :return: ``(base_count, supplemental_count, total_count)``.
    """
    all_rows = build_rows()

    # ``newline=""`` so the csv module owns line endings (no doubled CR on
    # Windows); deterministic output independent of host platform.
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(HEADER)
        writer.writerows(all_rows)

    # Derive counts from the same dimension products that build_rows() asserts
    # use -- no need to call the builders a second time.
    base_count = (
        len(CALL_FLAGS)
        * len(EDGE_OVERRIDES)
        * len(EDGE_MODES)
        * len(PIPELINE_BLINDS_BASE)
        * len(DIRECTIONS)
        * len(LINE_BOARD_COUNTS)
        * len(AUTO_RULE_FLAGS)
    )
    supplemental_count = (
        len(CALL_FLAGS)
        * len(EDGE_OVERRIDES)
        * len(DIRECTIONS)
        * len(LINE_BOARD_COUNTS)
        * len(AUTO_RULE_FLAGS)
    )
    return base_count, supplemental_count, len(all_rows)


def main() -> None:
    """Regenerate ``stop_at_line_truth_table.csv`` next to this script."""
    out_path = pathlib.Path(__file__).resolve().parent / "stop_at_line_truth_table.csv"
    base_count, supplemental_count, total_count = write_csv(out_path)
    print(f"wrote {out_path}")
    print(f"  base        = {base_count}")
    print(f"  supplemental= {supplemental_count}")
    print(f"  total       = {total_count}")


if __name__ == "__main__":
    main()
