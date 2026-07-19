#ifndef REVERSE_PIVOT_POLICY_H
#define REVERSE_PIVOT_POLICY_H

#include "../../Config.h"
#include "NavTypes.h"
#include "routes.h"

/**
 * @file ReversePivotPolicy.h
 * @brief Single source of truth for the reverse (REAR-traversal) correction of a
 *        single-wheel pivot (ADR-073, issue #54).
 *
 * The generated `PivotTurn[]` table stores the FORWARD-sense pivot wheel and a
 * direction-invariant over-travel magnitude `delta` (ADR-036). When a pivot is
 * traversed in REVERSE the stored wheel and the FRONT overshoot budget are both
 * wrong for the rear board, so this policy maps
 *
 *     (stored forward wheel, resolved pivot direction)
 *         -> (reverse-corrected pivot wheel, direction-resolved overshoot budget)
 *
 * Firmware (`PathTracker`), the Python sim (`sim/runtime.py`), the offline
 * validator, and CTPS all route through this one policy so the flip cannot drift
 * across the four surfaces. Header-only `inline` free functions, matching the
 * sibling `DirectionPolicy.h` / `StopAtLinePolicy.h` / `PivotTrimPolicy.h`
 * pattern (PR #34), so a host test links the SAME symbol the firmware does.
 *
 * -------------------------------------------------------------------------
 * WHY THE WHEEL FLIPS ON REVERSE -- derived, not guessed (ADR-036 -> ADR-073)
 * -------------------------------------------------------------------------
 * ADR-036 fixes the FORWARD wheel from the body rotation `theta`:
 *   1. `theta = wrap(h_out - h_in)` is the body turn realised at the pivot node.
 *   2. A left (CCW, `theta > 0`) turn brakes the LEFT wheel and drives the RIGHT
 *      -> `PIVOT_LEFT`; a right (CW, `theta < 0`) turn brakes the RIGHT wheel and
 *      drives the LEFT -> `PIVOT_RIGHT` (the nose swings toward the braked side).
 *   3. `delta = (T/2)*tan(|theta|/2)` is the drive-wheel forward over-travel.
 *
 * Under force-SAME both legs of a REAR pivot are traversed REAR, so the chassis
 * arrives at body heading `h_in + 180` and leaves at `h_out + 180`: the body
 * rotation is still `theta` -- SAME magnitude AND sign, because the two `+180`
 * offsets cancel in `h_out - h_in`. But the driven wheel now rolls BACKWARD, so
 * achieving that same `theta` plants the OPPOSITE wheel (braking the right and
 * rolling the left backward turns the body the same way forward-driving the right
 * about a braked left does). The wheel therefore FLIPS: `PIVOT_LEFT` <->
 * `PIVOT_RIGHT`. `delta` is direction-invariant (unchanged); in the world/travel
 * frame the planted wheel ends up on the SAME side as the forward pivot. FRONT
 * and AUTO traversals keep the stored forward wheel; a non-pivot `TurnMode`
 * (`DEFAULT` / `COUNTER_ROT`) is never flipped.
 *
 * Cross-language parity: keep step-for-step identical with the Python twins
 * `tools/path_planner/sim/runtime.py::resolve_pivot_wheel` /
 * `::pivot_overshoot_budget_cm`; the frozen-table parity test
 * `tools/path_planner/tests/test_reverse_pivot_policy_parity.py` (TC1, AC10)
 * fails CI if either language drifts.
 *
 * @note The stored table -- not the live commanded delta -- is the wheel
 *       authority. Deriving the wheel from the runtime delta reintroduces
 *       ADR-036 Blocker-1; the commanded-delta check is only a tripwire (TC16).
 */

/**
 * @brief Reverse-correct a single-wheel pivot's stored forward wheel.
 *
 * @param storedForwardWheel The FORWARD-sense wheel latched in `PivotTurn::wheel`
 *                           (`Routes::TurnMode::PIVOT_LEFT` / `PIVOT_RIGHT`).
 * @param dir                The direction the pivot's approach hop resolved to.
 * @return `PIVOT_LEFT` <-> `PIVOT_RIGHT` flipped when `dir == Direction::REAR`;
 *         `storedForwardWheel` unchanged for `Direction::FRONT` / `Direction::AUTO`,
 *         or for any non-pivot `TurnMode` (returned untouched regardless of `dir`).
 *
 * @note The flip is a pure L<->R swap of ADR-036's forward mapping (see the file
 *       header derivation): a REAR traversal rolls the driven wheel backward, so
 *       the same body `theta` plants the opposite wheel. `delta` is unchanged.
 */
inline Routes::TurnMode resolvePivotWheel(Routes::TurnMode storedForwardWheel, Direction dir)
{
    if (dir == Direction::REAR)
    {
        if (storedForwardWheel == Routes::TurnMode::PIVOT_LEFT)
        {
            return Routes::TurnMode::PIVOT_RIGHT;
        }
        if (storedForwardWheel == Routes::TurnMode::PIVOT_RIGHT)
        {
            return Routes::TurnMode::PIVOT_LEFT;
        }
    }
    return storedForwardWheel;
}

/**
 * @brief Select a pivot approach's post-detect overshoot budget by traversal
 *        direction.
 *
 * A sensor-anchored approach into a pivot rolls the axle forward from the crossing
 * line by the leading board's sensor-to-axle offset; a REAR approach leads with the
 * rear board, whose offset differs from the front board's. This selects the offset
 * against which the validator budgets `delta` and the runtime shortens the roll.
 *
 * @param dir The direction the pivot's approach hop resolved to.
 * @return `Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM` when `dir == Direction::REAR`;
 *         `Config::LINE_SENSOR_AXLE_OFFSET_CM` otherwise (`FRONT` / `AUTO`).
 *
 * @note Replaces the three inline direction-selects previously duplicated at the
 *       LINE-overshoot and seek dispatch sites (PathTracker.cpp) and in the sim
 *       (`runtime.py`) so every budget select shares one source.
 */
inline float pivotOvershootBudgetCm(Direction dir)
{
    return (dir == Direction::REAR) ? Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM
                                    : Config::LINE_SENSOR_AXLE_OFFSET_CM;
}

#endif    // REVERSE_PIVOT_POLICY_H
