#ifndef TURN_PROFILE_POLICY_H
#define TURN_PROFILE_POLICY_H

#include "TurnPDPolicy.h"

#include <math.h>
#include <stdint.h>

/**
 * @file TurnProfilePolicy.h
 * @brief Single source of truth for the in-place turn's gated speed profile
 *        (breakaway -> CRUISE -> CREEP -> SETTLE), replacing the per-tick PD
 *        law (ADR-078, supersedes the PD clause of ADR-068 and ADR-077).
 *
 * Three pure free functions answer "which phase, how hard, and what does
 * SETTLE command" for one dispatch tick of an in-place turn:
 *
 *   1. `turnPhaseFor(absErrDeg, creepWindowDeg, toleranceDeg)` maps the
 *      current absolute progress-frame error to the phase that error alone
 *      would naturally occupy. The FSM in `Drive.cpp` latches the phase
 *      monotonically forward via `_turnPhase = max(_turnPhase,
 *      turnPhaseFor(...))` — this function never resolves that latch itself,
 *      it only answers the un-latched, instantaneous question.
 *   2. `turnProfileSpeed(phase, cruisePwm, creepPwm)` returns the fixed
 *      UNSIGNED magnitude for the two driving phases (CRUISE, CREEP); SETTLE
 *      returns 0 because its command comes from `turnSettleNudge` instead.
 *   3. `turnSettleNudge(errProg, toleranceDeg, settleKp, creepPwm)` computes
 *      the SETTLE phase's terminal command: a light proportional nudge back
 *      toward zero error, clamped to the creep ceiling so the settle stage
 *      can never out-drive CREEP.
 *
 * `TurnWrap180` is not needed here — the caller (`Drive.cpp`) folds and signs
 * the error once, upstream, via `TurnPDPolicy.h::turnWrap180` and
 * `TurnSignPolicy.h`, and passes an already-progress-framed scalar into every
 * function below. `turnSettleStep` / `turnSettled` (kept in `TurnPDPolicy.h`)
 * still own the N-consecutive-in-band-samples exit debounce; this header only
 * decides phase and per-phase command, not the exit predicate.
 *
 * This header follows `TurnPDPolicy.h` / `TurnSignPolicy.h`: pure,
 * header-only `inline` free functions with zero Arduino / `Config.h`
 * surface, so a host test (`test_turn_profile_policy`) links the SAME
 * symbols production uses, and every tunable arrives as a parameter instead
 * of a global constant.
 *
 * ## Sign frame
 *
 * `turnSettleNudge` operates on the PROGRESS-frame scalar error, exactly like
 * `TurnPDPolicy.h`'s functions:
 *
 * ```
 * errProgress = _turnDeltaSign * turnWrap180(target - yaw);
 * ```
 *
 * Positive means "still short of the target"; negative means "overshot and
 * must reverse". `_turnDeltaSign` is frozen at dispatch and re-applied
 * exactly once by `TurnSignPolicy.h::turnWheelPattern(sign, cmdProgress,
 * mode)` for CRUISE/CREEP/breakaway commands (the caller applies
 * `copysignf` to the unsigned `turnProfileSpeed` magnitude before that).
 * `turnSettleNudge` is the one function in this profile that returns an
 * already-SIGNED command directly, because its magnitude is itself a
 * function of the signed error (`settleKp * errProg`) — there is no separate
 * unsigned-magnitude step to sign afterward.
 *
 * ## `turnPhaseFor` boundaries (inclusive toward the finer phase)
 *
 * ```
 * absErr >  creepWindowDeg                 -> CRUISE
 * toleranceDeg < absErr <= creepWindowDeg   -> CREEP
 * absErr <= toleranceDeg                    -> SETTLE
 * ```
 *
 * Landing exactly ON the window edge already means "no longer far" and
 * exactly ON the tolerance edge already means "in band" — both edges round
 * INTO the finer-grained phase, not the coarser one.
 *
 * ## Why `turnSettleNudge`'s in-band test is STRICT `<`, not `<=`
 *
 * `turnPhaseFor` enters SETTLE at `absErr <= toleranceDeg` (boundary
 * inclusive), but the kept exit debounce `turnSettleStep` (`TurnPDPolicy.h`)
 * counts a sample in-band only at `absErr < toleranceDeg` (strict). If
 * `turnSettleNudge` used `<=` it would freeze (command 0) on a sample that
 * `turnSettleStep` simultaneously refuses to count toward settling —
 * deadlocking a turn that lands exactly on the tolerance boundary. Matching
 * `turnSettleStep`'s strict `<` instead means that exact-boundary sample
 * gets a small nudge INTO the strictly-in-band region, where the next tick's
 * `turnSettleStep` call does count it. See ADR-078 and plan TC4.
 */

/**
 * @brief The three gated speed-profile phases of an in-place turn.
 *
 * @note Ordering is LOAD-BEARING: `CRUISE(0) < CREEP(1) < SETTLE(2)`. The
 *       dispatch FSM in `Drive.cpp` latches the phase monotonically forward
 *       every tick via `_turnPhase = max(_turnPhase, turnPhaseFor(...))`, so
 *       a `<` comparison between two `TurnPhase` values must agree with
 *       "farther from done". Do NOT reorder, renumber, or insert a phase
 *       without re-deriving every call site that relies on this ordering.
 */
enum class TurnPhase : uint8_t
{
    CRUISE = 0,    ///< Far from target: full authority toward the shortest-path direction.
    CREEP  = 1,    ///< Inside the creep window: reduced, kinetically-sustained authority.
    SETTLE = 2     ///< In tolerance band: terminal proportional nudge, then release.
};

/**
 * @brief Natural (un-latched) phase for an absolute progress-frame error.
 *
 * Answers "what phase does this error alone imply", independent of any prior
 * tick's state. The FSM applies the monotonic latch itself:
 * `_turnPhase = max(_turnPhase, turnPhaseFor(absErr, window, tol))` — this
 * function is deliberately unaware of the latch so it stays trivially pure
 * and testable in isolation.
 *
 * @param absErrDeg      Absolute progress-frame heading error (deg), i.e.
 *                        `fabsf(errProgress)`. Any non-negative finite value.
 * @param creepWindowDeg CRUISE -> CREEP handoff threshold (deg). Must be
 *                        greater than `toleranceDeg` (enforced by
 *                        `ConfigChecks.h`, not by this function).
 * @param toleranceDeg   CREEP -> SETTLE handoff threshold (deg).
 * @return `TurnPhase::CRUISE` when `absErrDeg > creepWindowDeg`;
 *         `TurnPhase::CREEP` when `toleranceDeg < absErrDeg <= creepWindowDeg`;
 *         `TurnPhase::SETTLE` when `absErrDeg <= toleranceDeg`.
 */
inline TurnPhase turnPhaseFor(float absErrDeg, float creepWindowDeg, float toleranceDeg)
{
    if (absErrDeg > creepWindowDeg)
    {
        return TurnPhase::CRUISE;
    }
    if (absErrDeg > toleranceDeg)
    {
        return TurnPhase::CREEP;
    }
    return TurnPhase::SETTLE;
}

/**
 * @brief Unsigned drive magnitude for the two driving phases.
 *
 * SETTLE has no fixed magnitude of its own — its command comes from
 * `turnSettleNudge` instead, so this function returns 0 for SETTLE as a
 * defensive default rather than a value a caller could mistakenly dispatch.
 *
 * @param phase     Current (already-latched) turn phase.
 * @param cruisePwm CRUISE magnitude (PWM). Callers pass the power-override
 *                   `_turnCruisePwm`, not the raw `Config::TURN_CRUISE_PWM`,
 *                   so `.setPower()` is honored.
 * @param creepPwm  CREEP magnitude (PWM), the kinetic-sustain floor.
 * @return `cruisePwm` for `CRUISE`, `creepPwm` for `CREEP`, `0` for `SETTLE`.
 */
inline int16_t turnProfileSpeed(TurnPhase phase, int16_t cruisePwm, int16_t creepPwm)
{
    switch (phase)
    {
    case TurnPhase::CRUISE:
        return cruisePwm;
    case TurnPhase::CREEP:
        return creepPwm;
    case TurnPhase::SETTLE:
    default:
        return 0;
    }
}

/**
 * @brief SETTLE phase's terminal proportional nudge (signed, progress frame).
 *
 * Strict in-band test, matching `turnSettleStep`'s strict `<` (see the file
 * docblock for why): a sample strictly inside tolerance commands exactly 0
 * so the settle debounce can count it and the turn can release cleanly. A
 * sample AT or outside tolerance gets a small signed nudge back toward zero
 * error, clamped to `+/-creepPwm` so SETTLE never out-drives CREEP.
 *
 * @param errProg      Signed progress-frame heading error (deg); positive
 *                       means still short of target, negative means overshot.
 * @param toleranceDeg In-band threshold (deg). Strictly less-than: exactly
 *                       AT tolerance is treated as out-of-band here, same as
 *                       `turnSettleStep`.
 * @param settleKp     Proportional gain (PWM per deg) applied to `errProg`.
 * @param creepPwm     Clamp ceiling (PWM); the nudge magnitude never exceeds
 *                       the CREEP floor it is nested inside.
 * @return `0.0f` when `fabsf(errProg) < toleranceDeg`; otherwise
 *         `clamp(settleKp * errProg, -creepPwm, +creepPwm)`, sign preserved.
 */
inline float turnSettleNudge(float errProg, float toleranceDeg, float settleKp, int16_t creepPwm)
{
    if (fabsf(errProg) < toleranceDeg)
    {
        return 0.0f;
    }

    float raw   = settleKp * errProg;
    float limit = (float) creepPwm;
    if (raw > limit)
    {
        return limit;
    }
    if (raw < -limit)
    {
        return -limit;
    }
    return raw;
}

#endif    // TURN_PROFILE_POLICY_H
