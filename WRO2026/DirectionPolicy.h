#ifndef DIRECTION_POLICY_H
#define DIRECTION_POLICY_H

#include <math.h>
#include <stdint.h>

#include "Config.h"
#include "NavTypes.h"
#include "routes.h"

/**
 * @file DirectionPolicy.h
 * @brief Single source of truth for the bidirectional direction-resolution
 *        algorithm.
 *
 * `resolveDirection(...)` is the C++ side of ADR-024's precedence rule:
 *
 *   1. Per-edge `direction_override` (FORCE_FORWARD / FORCE_REVERSE) is binding.
 *   2. Pipeline default (`PipelineConfig.direction`) is binding when set.
 *   3. Builder arrival override wins on the final hop only.
 *   4. Builder route override wins on every hop.
 *   5. Auto: REAR iff `BIDIRECTIONAL_AUTO_ENABLED` AND `lineBoardCount >= 2`
 *      AND the wrap-normalized body delta exceeds
 *      `BIDIRECTIONAL_TURN_THRESHOLD_DEG`. Ties go FRONT.
 *
 * Previously this function lived in PathTracker.cpp's anonymous namespace,
 * which forced the host-compile test suite to maintain a byte-for-byte
 * `replica_resolveDirection`. Promoting it to a header-only `inline` free
 * function eliminates that drift risk: production and tests link the same
 * symbol via include. The Python sim already does this — its
 * `resolve_direction` is a module-level public helper called by both the
 * runtime FSM and the offline compiler. See PR #34 review for the full
 * rationale.
 *
 * Cross-language parity: keep this function step-for-step identical with
 * `tools/path_planner/sim/runtime.py::resolve_direction`. Enforced by the
 * shared-vector golden in
 * `tools/path_planner/tests/test_direction_parity.py`, which feeds one input
 * set through the Python helper and asserts each result against a frozen table
 * of this C++ algorithm's outputs (so a wrap/threshold divergence fails CI).
 * TC18 only checks compiler<->sim *primitive-stream* parity, not the
 * cross-language resolver algorithm.
 */
inline Direction resolveDirection(
    Routes::DirectionMode edgeOverride,
    Routes::DirectionMode pipelineDefault,
    Direction             builderArrival,
    Direction             builderRoute,
    float                 currentHeading,
    float                 edgeHeading,
    bool                  isLastHop,
    bool                  bidirectionalAutoEnabled,
    uint8_t               lineBoardCount)
{
    // Step 1: per-edge override is binding.
    if (edgeOverride == Routes::DirectionMode::FORCE_FORWARD) return Direction::FRONT;
    if (edgeOverride == Routes::DirectionMode::FORCE_REVERSE) return Direction::REAR;

    // Step 2: pipeline default is binding when not DEFAULT.
    if (pipelineDefault == Routes::DirectionMode::FORCE_FORWARD) return Direction::FRONT;
    if (pipelineDefault == Routes::DirectionMode::FORCE_REVERSE) return Direction::REAR;

    // Step 3: arrival override wins on the final hop.
    if (isLastHop && builderArrival != Direction::AUTO) return builderArrival;

    // Step 4: route override wins on all hops.
    if (builderRoute != Direction::AUTO) return builderRoute;

    // Step 5: auto — only if 2-board chassis AND kill-switch on.
    if (!bidirectionalAutoEnabled) return Direction::FRONT;
    if (lineBoardCount < 2)        return Direction::FRONT;

    // Wrap delta into (-180, +180] via the shared NavTypes.h helper (same wrap the
    // PathTracker pass-through / turn-needed checks use). Keep this behaviourally
    // identical to the sim's `normalize_turn` in sim/runtime.py (parity golden:
    // test_direction_parity.py).
    const float delta = wrapDeltaDeg(edgeHeading - currentHeading);
    if (fabsf(delta) > Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG) return Direction::REAR;
    return Direction::FRONT;
}

/**
 * @brief Whether a resolved REAR hop is unservable on the current chassis.
 *
 * REAR LINE / PROJECTION needs a rear line sensor (>= 2 boards). BLIND REAR is
 * permitted on a 1-board chassis (dead-reckoned reverse needs only negative
 * PWM, no sensor), so the check is scoped to LINE / PROJECTION. Promoted from
 * PathTracker.cpp's two guard sites so the host-compile test links the same
 * predicate instead of a hand-copied replica (trace #77; same rationale as
 * resolveDirection above).
 */
inline bool reverseUnavailable(Direction resolved, Routes::EdgeType edgeType,
                               uint8_t lineBoardCount)
{
    return resolved == Direction::REAR && lineBoardCount < 2 &&
           (edgeType == Routes::EdgeType::LINE ||
            edgeType == Routes::EdgeType::PROJECTION);
}

#endif    // DIRECTION_POLICY_H
