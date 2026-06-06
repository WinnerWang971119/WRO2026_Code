"""Cross-language parity golden for the bidirectional direction resolver.

Finding #34 (docs/PR35_TRACE_FINDINGS.md): both ``DirectionPolicy.h`` and
``sim/runtime.py`` claim the two ``resolveDirection`` / ``resolve_direction``
implementations stay "step-for-step identical", but until now no test fed the
*same* input vector through both and asserted equal output. The Python tests
exercised only the Python helper; the host-compile C++ test exercised only the
inline C++ helper; the TC14a/b "parity" tests merely regex-grep the header for
the threshold (90.0) and the AUTO flag — not the algorithm. A divergence in the
wrap routine (C++ ``wrapDeltaDeg`` unbounded ``while`` loop vs Python
``compiler.normalize_turn`` modulo) or in the float/double threshold comparison
would pass CI.

This module closes that gap: a SHARED input vector is run through the Python
``resolve_direction`` and asserted against a FROZEN golden table that records
exactly what the C++ ``resolveDirection`` in ``WRO2026/DirectionPolicy.h`` would
return for each input. A wrap or threshold divergence between the two languages
now fails CI.

----------------------------------------------------------------------------
GOLDEN PROVENANCE — READ BEFORE EDITING
----------------------------------------------------------------------------
``_EXPECTED`` below is the *recorded C++ behavior*. Each row was hand-derived by
tracing the C++ algorithm in ``WRO2026/DirectionPolicy.h`` (the source of truth)
step by step:

  1. edgeOverride   FORCE_FORWARD -> FRONT, FORCE_REVERSE -> REAR
  2. pipelineDefault FORCE_FORWARD -> FRONT, FORCE_REVERSE -> REAR
  3. isLastHop && builderArrival != AUTO -> builderArrival
  4. builderRoute != AUTO -> builderRoute
  5. auto:
       !bidirectionalAutoEnabled -> FRONT
       lineBoardCount < 2        -> FRONT
       delta = wrapDeltaDeg(edgeHeading - currentHeading)   # (-180, 180]
       fabsf(delta) > 90.0 -> REAR  else FRONT               # tie (==90) -> FRONT

where ``wrapDeltaDeg`` (WRO2026/NavTypes.h) is:

    while (d > 180.0f)  d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;

The golden is FROZEN. Do NOT regenerate it to "make the test pass" if Python
output changes — a Python change that breaks parity is exactly the bug this
test exists to catch. The golden may be re-baselined ONLY via an authorized
change to ``DirectionPolicy.h`` (i.e. the C++ source of truth itself moved), and
the new expected values must again be hand-derived from the C++ algorithm above,
not copied from whatever Python currently emits.
"""

from __future__ import annotations

import pytest

from tools.path_planner.pipeline_config import DirectionMode
from tools.path_planner.sim.runtime import (
    BIDIRECTIONAL_TURN_THRESHOLD_DEG,
    Direction,
    resolve_direction,
)

# Guard the golden's central assumption: the FRONT/REAR decision threshold the
# golden was hand-derived against is exactly 90.0 degrees (matching the C++
# ``Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG = 90.0f``). If this constant moves,
# the golden's boundary rows (89/90/91, 269/270/271, 449/450/451 ...) are stale
# and the table must be re-derived — fail loudly rather than silently mis-assert.
_THRESHOLD_AT_GOLDEN_TIME: float = 90.0


# A single parity row.
#
# Fields mirror ``resolve_direction``'s signature 1:1 (and the C++
# ``resolveDirection`` argument order):
#
#   (edge_override, pipeline_default, builder_arrival, builder_route,
#    current_heading, edge_heading, is_last_hop, bidirectional_enabled,
#    line_board_count, expected_cpp_direction, case_id)
#
# ``expected_cpp_direction`` is what the C++ ``resolveDirection`` returns for the
# row — the FROZEN golden (see module docstring for the hand-derivation rules).
_Row = tuple[
    DirectionMode,  # edge_override
    DirectionMode,  # pipeline_default
    Direction,      # builder_arrival
    Direction,      # builder_route
    float,          # current_heading
    float,          # edge_heading
    bool,           # is_last_hop
    bool,           # bidirectional_enabled
    int,            # line_board_count
    Direction,      # expected (recorded C++ output)
    str,            # case id
]

# Shorthands to keep the table readable.
_DEF = DirectionMode.DEFAULT
_FF = DirectionMode.FORCE_FORWARD
_FR = DirectionMode.FORCE_REVERSE
_AUTO = Direction.AUTO
_FRONT = Direction.FRONT
_REAR = Direction.REAR


# ---------------------------------------------------------------------------
# FROZEN GOLDEN TABLE — recorded C++ resolveDirection behavior.
# ---------------------------------------------------------------------------
# Stresses every branch where the two implementations could diverge:
#   * explicit-override precedence (steps 1-4),
#   * the FRONT/REAR auto threshold at exactly 90.0 and just above/below,
#   * heading deltas that require wrap-around (near 180, 360, >360, negatives),
#   * the kill-switch and 1-board short-circuits.
_EXPECTED: list[_Row] = [
    # ----- Step 1: per-edge override is binding (beats everything else) -----
    # Force flags win even with a heading delta that auto would call REAR and a
    # contradicting builder override, on any hop.
    (_FF, _FR, _REAR, _REAR, 0.0, 179.0, True, True, 2, _FRONT, "edge_FORCE_FORWARD_beats_all"),
    (_FR, _FF, _FRONT, _FRONT, 0.0, 0.0, True, True, 2, _REAR, "edge_FORCE_REVERSE_beats_all"),

    # ----- Step 2: pipeline default binds when not DEFAULT (edge is DEFAULT) -----
    (_DEF, _FF, _REAR, _REAR, 0.0, 179.0, True, True, 2, _FRONT, "pipeline_FORCE_FORWARD"),
    (_DEF, _FR, _FRONT, _FRONT, 0.0, 0.0, False, True, 2, _REAR, "pipeline_FORCE_REVERSE"),

    # ----- Step 3: builder arrival wins on the LAST hop only -----
    (_DEF, _DEF, _REAR, _AUTO, 0.0, 0.0, True, True, 2, _REAR, "arrival_REAR_on_last_hop"),
    (_DEF, _DEF, _FRONT, _AUTO, 0.0, 179.0, True, True, 2, _FRONT, "arrival_FRONT_on_last_hop_overrides_auto_rear"),
    # Arrival override is IGNORED when not the last hop -> falls through to auto.
    (_DEF, _DEF, _REAR, _AUTO, 0.0, 0.0, False, True, 2, _FRONT, "arrival_REAR_ignored_not_last_hop_auto_front"),

    # ----- Step 4: builder route wins on EVERY hop (when arrival did not fire) -----
    (_DEF, _DEF, _AUTO, _REAR, 0.0, 0.0, False, True, 2, _REAR, "route_REAR_every_hop"),
    (_DEF, _DEF, _AUTO, _FRONT, 0.0, 179.0, False, True, 2, _FRONT, "route_FRONT_overrides_auto_rear"),
    # Route override applies even on the last hop when arrival is AUTO.
    (_DEF, _DEF, _AUTO, _REAR, 0.0, 0.0, True, True, 2, _REAR, "route_REAR_on_last_hop_arrival_auto"),

    # ----- Step 5 short-circuits: kill-switch off / fewer than 2 boards -----
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 179.0, False, False, 2, _FRONT, "auto_killswitch_off_forces_front"),
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 179.0, False, True, 1, _FRONT, "auto_one_board_forces_front"),
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 179.0, False, False, 1, _FRONT, "auto_killswitch_off_and_one_board"),

    # ----- Step 5 auto: threshold boundary at exactly 90.0 (tie -> FRONT) -----
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 0.0, False, True, 2, _FRONT, "auto_delta_0_front"),
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 89.0, False, True, 2, _FRONT, "auto_delta_89_front"),
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 90.0, False, True, 2, _FRONT, "auto_delta_90_tie_front"),
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 91.0, False, True, 2, _REAR, "auto_delta_91_rear"),
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 179.5, False, True, 2, _REAR, "auto_delta_179p5_rear"),

    # ----- Step 5 auto: wrap-around at +180 (boundary maps to +180 -> REAR) -----
    # wrapDeltaDeg(180) = 180 (180 is NOT > 180) -> |180| > 90 -> REAR.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 180.0, False, True, 2, _REAR, "auto_delta_180_rear"),
    # wrapDeltaDeg(-180) = +180 (-180 <= -180 so +360) -> REAR.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, -180.0, False, True, 2, _REAR, "auto_delta_neg180_wraps_to_180_rear"),

    # ----- Step 5 auto: negative deltas (current_heading > edge_heading) -----
    # delta = edge - current.
    (_DEF, _DEF, _AUTO, _AUTO, 90.0, 0.0, False, True, 2, _FRONT, "auto_neg90_tie_front"),    # delta -90
    (_DEF, _DEF, _AUTO, _AUTO, 91.0, 0.0, False, True, 2, _REAR, "auto_neg91_rear"),          # delta -91
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, -91.0, False, True, 2, _REAR, "auto_edge_neg91_rear"),    # delta -91

    # ----- Step 5 auto: wrap-around past 360 (>360 deltas) -----
    # wrapDeltaDeg(270) = 270 - 360 = -90 -> tie -> FRONT.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 270.0, False, True, 2, _FRONT, "auto_delta_270_wraps_to_neg90_front"),
    # wrapDeltaDeg(269) = -91 -> REAR.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 269.0, False, True, 2, _REAR, "auto_delta_269_wraps_to_neg91_rear"),
    # wrapDeltaDeg(271) = -89 -> FRONT.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 271.0, False, True, 2, _FRONT, "auto_delta_271_wraps_to_neg89_front"),
    # wrapDeltaDeg(360) = 0 -> FRONT.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 360.0, False, True, 2, _FRONT, "auto_delta_360_wraps_to_0_front"),
    # wrapDeltaDeg(450) = 90 -> tie -> FRONT.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 450.0, False, True, 2, _FRONT, "auto_delta_450_wraps_to_90_tie_front"),
    # wrapDeltaDeg(451) = 91 -> REAR.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 451.0, False, True, 2, _REAR, "auto_delta_451_wraps_to_91_rear"),
    # wrapDeltaDeg(540) = 540-360=180 (180 not > 180) -> REAR.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 540.0, False, True, 2, _REAR, "auto_delta_540_wraps_to_180_rear"),
    # wrapDeltaDeg(720) = 0 -> FRONT.
    (_DEF, _DEF, _AUTO, _AUTO, 0.0, 720.0, False, True, 2, _FRONT, "auto_delta_720_wraps_to_0_front"),
    # Negative beyond -360: wrapDeltaDeg(-270) = -270+360 = 90 -> tie -> FRONT.
    (_DEF, _DEF, _AUTO, _AUTO, 270.0, 0.0, False, True, 2, _FRONT, "auto_delta_neg270_wraps_to_90_tie_front"),
    # wrapDeltaDeg(-269) = 91 -> REAR.
    (_DEF, _DEF, _AUTO, _AUTO, 269.0, 0.0, False, True, 2, _REAR, "auto_delta_neg269_wraps_to_91_rear"),
]


def test_threshold_constant_matches_golden_assumption() -> None:
    """The frozen golden boundary rows assume the threshold is exactly 90.0.

    The hand-derived ``_EXPECTED`` table places its FRONT/REAR transitions at
    |delta| == 90.0 (the C++ ``Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG``). If
    the Python-side mirror of that constant drifts from 90.0, every boundary row
    is silently stale, so fail explicitly and demand a re-derivation.
    """
    assert BIDIRECTIONAL_TURN_THRESHOLD_DEG == _THRESHOLD_AT_GOLDEN_TIME, (
        "BIDIRECTIONAL_TURN_THRESHOLD_DEG moved from the value the parity golden "
        f"was derived against ({_THRESHOLD_AT_GOLDEN_TIME}); re-derive _EXPECTED "
        "from WRO2026/DirectionPolicy.h before updating this constant."
    )


@pytest.mark.parametrize(
    (
        "edge_override",
        "pipeline_default",
        "builder_arrival",
        "builder_route",
        "current_heading",
        "edge_heading",
        "is_last_hop",
        "bidirectional_enabled",
        "line_board_count",
        "expected",
    ),
    [row[:-1] for row in _EXPECTED],
    ids=[row[-1] for row in _EXPECTED],
)
def test_python_resolve_direction_matches_cpp_golden(
    edge_override: DirectionMode,
    pipeline_default: DirectionMode,
    builder_arrival: Direction,
    builder_route: Direction,
    current_heading: float,
    edge_heading: float,
    is_last_hop: bool,
    bidirectional_enabled: bool,
    line_board_count: int,
    expected: Direction,
) -> None:
    """Python ``resolve_direction`` must equal the frozen C++ golden per row.

    Feeds the SAME input through the Python ``resolve_direction`` and asserts the
    result equals the recorded C++ ``resolveDirection`` output. A wrap or
    threshold divergence between the two languages fails here with the offending
    input echoed in the message.
    """
    result = resolve_direction(
        edge_dir_override=edge_override,
        pipeline_default=pipeline_default,
        builder_arrival=builder_arrival,
        builder_route=builder_route,
        current_heading=current_heading,
        edge_heading=edge_heading,
        is_last_hop=is_last_hop,
        bidirectional_enabled=bidirectional_enabled,
        line_board_count=line_board_count,
    )
    assert result == expected, (
        "Python resolve_direction diverged from the frozen C++ resolveDirection "
        "golden.\n"
        f"  edge_override={edge_override.name} "
        f"pipeline_default={pipeline_default.name}\n"
        f"  builder_arrival={builder_arrival.name} "
        f"builder_route={builder_route.name}\n"
        f"  current_heading={current_heading} edge_heading={edge_heading} "
        f"is_last_hop={is_last_hop}\n"
        f"  bidirectional_enabled={bidirectional_enabled} "
        f"line_board_count={line_board_count}\n"
        f"  expected (C++ golden)={expected.name}  got (Python)={result.name}"
    )


def test_every_branch_and_both_outputs_covered() -> None:
    """Sanity: the golden vector covers all overrides and both FRONT and REAR.

    Cheap guard that a future edit does not accidentally trim the table down to
    one trivial branch — both output values and every override family must stay
    represented.
    """
    expecteds = {row[9] for row in _EXPECTED}
    assert expecteds == {Direction.FRONT, Direction.REAR}

    edge_overrides = {row[0] for row in _EXPECTED}
    assert {DirectionMode.FORCE_FORWARD, DirectionMode.FORCE_REVERSE} <= edge_overrides

    builder_routes = {row[3] for row in _EXPECTED}
    assert {Direction.FRONT, Direction.REAR} <= builder_routes

    # Both the auto and explicit-override paths are exercised.
    has_auto_path = any(
        row[0] == DirectionMode.DEFAULT
        and row[1] == DirectionMode.DEFAULT
        and row[2] == Direction.AUTO
        and row[3] == Direction.AUTO
        for row in _EXPECTED
    )
    assert has_auto_path
