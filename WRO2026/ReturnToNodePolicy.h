#ifndef RETURN_TO_NODE_POLICY_H
#define RETURN_TO_NODE_POLICY_H

#include <stdint.h>

#include "Config.h"

/**
 * @file ReturnToNodePolicy.h
 * @brief Single source of truth for the return-to-node gate decision (issue #36,
 *        PR1b).
 *
 * `resolveReturnAction(...)` answers one question at every `goTo()` start: given
 * the accumulated operator-offset magnitude and the configured trigger / cap /
 * off-map policy, should PathTracker (a) plan immediately as if on-node, (b)
 * drive an inverse return onto the anchored node first, or (c) refuse / best-
 * effort because the excursion is past the trust cap? The rule, mapping the three
 * Config knobs from `ConfigLocalization.h`:
 *
 *   1. mag <= triggerCm                 -> NONE            (on-node; plan as today)
 *   2. triggerCm < mag <= capCm         -> RETURN          (drive the inverse first)
 *   3. mag > capCm AND gate == FAULT    -> OFF_MAP_FAULT   (refuse to plan)
 *   4. mag > capCm AND gate == BEST_EFFORT -> OFF_MAP_PROCEED (log + return anyway)
 *
 * This is the riskiest decision in T8 (live off-map motion + fault gating under
 * ADR-001), so it is promoted to a header-only `inline` free function — pure
 * float/enum logic with no Arduino / Drive / Localization dependency — exactly
 * like `DirectionPolicy.h::resolveDirection`,
 * `StopAtLinePolicy.h::resolveStopAtLine`, and
 * `PivotTrimPolicy.h::resolvePivotTrim`. Production and the host-compile test
 * (T10) link the SAME symbol via include, so the gate boundaries cannot drift
 * from their test coverage.
 *
 * The branch order matters: the cap test runs BEFORE deciding RETURN, so an
 * over-cap offset never silently falls into a plain return. The trigger test runs
 * first so the common offset-around-zero case short-circuits to NONE with no
 * further work — this is the AC5 true no-op path.
 *
 * @note The Config static_asserts in `ConfigLocalization.h` already guarantee
 *       `0 < triggerCm <= capCm`, so the four cases above are exhaustive and
 *       non-overlapping for any finite non-negative @p offsetMagCm.
 */

/**
 * @brief Outcome of the return-to-node gate decision for one `goTo()` start.
 *
 * - `NONE`            — offset is within trigger; behave exactly as today (no
 *                       return move, straight to planning).
 * - `RETURN`          — offset is over trigger but within the cap; drive the
 *                       approximate inverse of the offset back onto the node.
 * - `OFF_MAP_FAULT`   — offset is over the cap and `OFF_MAP_GATE == FAULT`;
 *                       refuse to plan and latch `Fault::OFF_NODE`.
 * - `OFF_MAP_PROCEED` — offset is over the cap and `OFF_MAP_GATE == BEST_EFFORT`;
 *                       log a warning and attempt the (capped) return anyway.
 */
enum class ReturnAction : uint8_t
{
    NONE,
    RETURN,
    OFF_MAP_FAULT,
    OFF_MAP_PROCEED
};

/**
 * @brief Resolve whether a `goTo()` returns to the anchored node first, and how.
 *
 * @param offsetMagCm The accumulated operator-offset translation magnitude
 *                    (`sqrt(off.x^2 + off.y^2)`), in centimetres. Expected
 *                    finite and non-negative.
 * @param triggerCm   `Config::RETURN_TRIGGER_THRESHOLD_CM` — at or below this the
 *                    chassis is treated as on-node (NONE).
 * @param capCm       `Config::RETURN_DISTANCE_CAP_CM` — above this the off-map
 *                    gate fires; at or below it a return is dispatched.
 * @param gate        `Config::OFF_MAP_GATE` — selects FAULT vs BEST_EFFORT when
 *                    the offset exceeds @p capCm.
 * @return The `ReturnAction` per the four-case rule documented in the file
 *         docblock.
 */
inline ReturnAction resolveReturnAction(
    float              offsetMagCm,
    float              triggerCm,
    float              capCm,
    Config::OffMapGate gate)
{
    // Case 1: within trigger — on-node, plan as today. Checked first so the
    // offset-around-zero case (AC5) short-circuits with no further work.
    if (offsetMagCm <= triggerCm)
    {
        return ReturnAction::NONE;
    }

    // Case 3/4: over the trust cap — the off-map gate decides. Checked BEFORE
    // returning RETURN so an over-cap offset can never fall into a plain return.
    if (offsetMagCm > capCm)
    {
        return (gate == Config::OffMapGate::FAULT) ? ReturnAction::OFF_MAP_FAULT
                                                   : ReturnAction::OFF_MAP_PROCEED;
    }

    // Case 2: trigger < mag <= cap — drive the inverse return.
    return ReturnAction::RETURN;
}

#endif    // RETURN_TO_NODE_POLICY_H
