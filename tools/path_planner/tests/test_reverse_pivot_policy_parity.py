"""Cross-language parity golden for the reverse-pivot policy (ADR-071, #54).

``WRO2026/src/nav/ReversePivotPolicy.h`` and its Python twins in
``sim/runtime.py`` (``resolve_pivot_wheel`` / ``pivot_overshoot_budget_cm``)
claim to stay "step-for-step identical" so the firmware, the offline sim, the
validator, and CTPS all flip a reverse pivot the same way. This module closes the
gap the way ``test_direction_parity.py`` does for the direction resolver: a SHARED
input vector is run through the Python twin and asserted against a FROZEN record
of what the C++ ``resolvePivotWheel`` / ``pivotOvershootBudgetCm`` return. The
pytest job carries no C++ toolchain, so the C++ side is captured as a table, not
executed. A wheel-flip or budget-select divergence between the two languages now
fails CI (AC10 / TC1).

----------------------------------------------------------------------------
GOLDEN PROVENANCE — READ BEFORE EDITING
----------------------------------------------------------------------------
``_EXPECTED_WHEEL`` below is the *recorded C++ behavior* of ``resolvePivotWheel``.
Each row was hand-derived by tracing the C++ algorithm in
``WRO2026/src/nav/ReversePivotPolicy.h`` (the source of truth):

  1. ``dir == Direction::REAR`` and wheel is ``PIVOT_LEFT``  -> ``PIVOT_RIGHT``
  2. ``dir == Direction::REAR`` and wheel is ``PIVOT_RIGHT`` -> ``PIVOT_LEFT``
  3. ``dir == Direction::REAR`` and a non-pivot wheel        -> unchanged
  4. ``dir`` in {FRONT, AUTO}                                -> unchanged

The L<->R flip is the crux (derived in the header from ADR-036): a REAR traversal
rolls the driven wheel backward, so the same body ``theta`` plants the OPPOSITE
wheel. ``delta`` is direction-invariant, so only the wheel changes.

The wheel golden is FROZEN. Do NOT regenerate it to "make the test pass" if the
Python output changes — a Python change that breaks parity is exactly the bug this
test exists to catch. It may be re-baselined ONLY via an authorized change to
``ReversePivotPolicy.h`` (the C++ source of truth itself moving), and the new
values must again be hand-derived from the C++ algorithm above.

The overshoot budgets are the *symbolic* ``Config`` offsets, not literals:
``pivotOvershootBudgetCm(REAR)`` returns ``Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM``
and everything else returns ``Config::LINE_SENSOR_AXLE_OFFSET_CM``. Those offsets
are unmeasured placeholders pending the wired rear board (ConfigDrive.h TODO), so
this test reads the two C++ header values through the validator's parsers — the
true "C++ output" — and asserts the Python twin returns the matching one. A bench
remeasure of either offset moves the parsed value and the twin's constant in
lockstep; this test stays green without a re-baseline (symbolic-budget rule).
"""

from __future__ import annotations

import pytest

from tools.path_planner.pipeline_config import TurnMode
from tools.path_planner.sim.runtime import (
    Direction,
    LINE_SENSOR_AXLE_OFFSET_CM,
    LINE_SENSOR_AXLE_OFFSET_REAR_CM,
    pivot_overshoot_budget_cm,
    resolve_pivot_wheel,
)
from tools.path_planner.validator import (
    parse_line_sensor_axle_offset_cm,
    parse_line_sensor_rear_axle_offset_cm,
)

# Shorthands to keep the table readable.
_L = TurnMode.PIVOT_LEFT
_R = TurnMode.PIVOT_RIGHT
_DEFAULT = TurnMode.DEFAULT
_COUNTER = TurnMode.COUNTER_ROT
_AUTO = Direction.AUTO
_FRONT = Direction.FRONT
_REAR = Direction.REAR


# A single wheel-parity row: mirrors ``resolve_pivot_wheel``'s signature 1:1 (and
# the C++ ``resolvePivotWheel`` argument order):
#
#   (stored_forward_wheel, direction, expected_cpp_wheel, case_id)
#
# ``expected_cpp_wheel`` is what the C++ ``resolvePivotWheel`` returns for the row
# — the FROZEN golden (see module docstring for the hand-derivation rules).
_Row = tuple[TurnMode, Direction, TurnMode, str]


# ---------------------------------------------------------------------------
# FROZEN GOLDEN TABLE — recorded C++ resolvePivotWheel behavior.
# ---------------------------------------------------------------------------
# Covers every (wheel in {PIVOT_LEFT, PIVOT_RIGHT}, dir in {FRONT, REAR}) input
# the spec requires, plus the AUTO passthrough and the non-pivot passthrough
# branches the header also guarantees.
_EXPECTED_WHEEL: list[_Row] = [
    # ----- REAR flips L<->R (the crux; derived in ReversePivotPolicy.h) -----
    (_L, _REAR, _R, "left_rear_flips_to_right"),
    (_R, _REAR, _L, "right_rear_flips_to_left"),

    # ----- FRONT keeps the stored forward wheel -----
    (_L, _FRONT, _L, "left_front_unchanged"),
    (_R, _FRONT, _R, "right_front_unchanged"),

    # ----- AUTO keeps the stored forward wheel (never flips) -----
    (_L, _AUTO, _L, "left_auto_unchanged"),
    (_R, _AUTO, _R, "right_auto_unchanged"),

    # ----- Non-pivot TurnMode returned untouched, regardless of direction -----
    (_DEFAULT, _REAR, _DEFAULT, "default_rear_unchanged"),
    (_COUNTER, _REAR, _COUNTER, "counter_rot_rear_unchanged"),
    (_DEFAULT, _FRONT, _DEFAULT, "default_front_unchanged"),
    (_COUNTER, _AUTO, _COUNTER, "counter_rot_auto_unchanged"),
]


@pytest.mark.parametrize(
    ("stored_forward_wheel", "direction", "expected"),
    [row[:-1] for row in _EXPECTED_WHEEL],
    ids=[row[-1] for row in _EXPECTED_WHEEL],
)
def test_python_resolve_pivot_wheel_matches_cpp_golden(
    stored_forward_wheel: TurnMode,
    direction: Direction,
    expected: TurnMode,
) -> None:
    """Python ``resolve_pivot_wheel`` must equal the frozen C++ golden per row.

    Feeds the SAME input through the Python twin and asserts the result equals
    the recorded C++ ``resolvePivotWheel`` output. A flip divergence between the
    two languages fails here with the offending input echoed in the message.
    """
    result = resolve_pivot_wheel(stored_forward_wheel, direction)
    assert result == expected, (
        "Python resolve_pivot_wheel diverged from the frozen C++ resolvePivotWheel "
        "golden.\n"
        f"  stored_forward_wheel={stored_forward_wheel.name} "
        f"direction={direction.name}\n"
        f"  expected (C++ golden)={expected.name}  got (Python)={result.name}"
    )


def test_pivot_overshoot_budget_matches_config_offsets() -> None:
    """The budget twin selects the REAR offset for REAR and the FRONT one else.

    The C++ ``pivotOvershootBudgetCm`` returns ``Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM``
    for REAR and ``Config::LINE_SENSOR_AXLE_OFFSET_CM`` for FRONT/AUTO. Reads
    those two values straight from the C++ header (the true "C++ output") via the
    validator parsers and asserts the Python twin returns the matching one — which
    also proves the Python mirror constants have not drifted from Config.h.
    """
    front_offset = parse_line_sensor_axle_offset_cm()
    rear_offset = parse_line_sensor_rear_axle_offset_cm()

    assert pivot_overshoot_budget_cm(_REAR) == rear_offset, (
        "pivot_overshoot_budget_cm(REAR) must return the REAR offset "
        f"Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM ({rear_offset}); "
        f"got {pivot_overshoot_budget_cm(_REAR)}."
    )
    assert pivot_overshoot_budget_cm(_FRONT) == front_offset, (
        "pivot_overshoot_budget_cm(FRONT) must return the FRONT offset "
        f"Config::LINE_SENSOR_AXLE_OFFSET_CM ({front_offset}); "
        f"got {pivot_overshoot_budget_cm(_FRONT)}."
    )
    assert pivot_overshoot_budget_cm(_AUTO) == front_offset, (
        "pivot_overshoot_budget_cm(AUTO) must fall back to the FRONT offset "
        f"Config::LINE_SENSOR_AXLE_OFFSET_CM ({front_offset}); "
        f"got {pivot_overshoot_budget_cm(_AUTO)}."
    )


def test_budget_twin_mirrors_runtime_constants() -> None:
    """The twin returns the module's own mirror constants (no hidden literal).

    Guards that ``pivot_overshoot_budget_cm`` reads the ``sim.runtime`` mirror
    constants rather than a hard-coded value, so the symbolic-budget rule holds
    and a future retune reddens only the four known drive-constant surfaces.
    """
    assert pivot_overshoot_budget_cm(_REAR) == LINE_SENSOR_AXLE_OFFSET_REAR_CM
    assert pivot_overshoot_budget_cm(_FRONT) == LINE_SENSOR_AXLE_OFFSET_CM
    assert pivot_overshoot_budget_cm(_AUTO) == LINE_SENSOR_AXLE_OFFSET_CM


def test_wheel_golden_covers_flip_and_passthrough() -> None:
    """Sanity: the golden exercises both flips, both passthroughs, non-pivot modes.

    Cheap guard that a future edit does not trim the table down to one trivial
    branch — both REAR flips, a FRONT/AUTO passthrough for each pivot wheel, and a
    non-pivot passthrough must stay represented.
    """
    triples = {(row[0], row[1], row[2]) for row in _EXPECTED_WHEEL}

    # Both REAR flips present and genuinely swapped.
    assert (_L, _REAR, _R) in triples
    assert (_R, _REAR, _L) in triples

    # FRONT and AUTO passthrough present for each pivot wheel.
    assert (_L, _FRONT, _L) in triples
    assert (_R, _AUTO, _R) in triples

    # Non-pivot modes are passed through even on REAR.
    assert (_DEFAULT, _REAR, _DEFAULT) in triples
    assert (_COUNTER, _REAR, _COUNTER) in triples


def test_offsets_differ_so_budget_select_is_discriminated() -> None:
    """Document that the budget select is meaningfully tested only while the two
    offsets differ.

    The FRONT/REAR overshoot offsets are unmeasured placeholders; today they
    differ (12.02 vs 7.68), so ``test_pivot_overshoot_budget_matches_config_offsets``
    genuinely distinguishes the REAR branch from the FRONT branch. If a future
    placeholder sets them equal the direction select becomes behaviorally moot and
    that test can no longer catch a REAR->FRONT mis-select; this xfails (does not
    hard-fail) to flag that state without blocking a legitimate placeholder.
    """
    if LINE_SENSOR_AXLE_OFFSET_REAR_CM == LINE_SENSOR_AXLE_OFFSET_CM:
        pytest.xfail(
            "REAR and FRONT overshoot offsets are currently equal; the budget "
            "direction-select is no longer discriminated by the parity test."
        )
    assert LINE_SENSOR_AXLE_OFFSET_REAR_CM != LINE_SENSOR_AXLE_OFFSET_CM
