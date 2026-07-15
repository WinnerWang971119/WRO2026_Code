#ifndef TURN_PD_POLICY_H
#define TURN_PD_POLICY_H

#include <math.h>
#include <stdint.h>

/**
 * @file TurnPDPolicy.h
 * @brief Single source of truth for the per-tick turn PD control law (ADR-068).
 *
 * `Drive.cpp` IS host-compiled — integration targets link the real translation
 * unit against the `hal_mocks/` recording surfaces (issue #106 / ADR-030), so
 * the dispatched PWM is observable end-to-end. But pure decision math still
 * belongs in a header-only policy seam: it is the only way to exercise the
 * control law in isolation, with zero mocks and every tunable injected, over
 * the edge cases a stateful FSM test cannot cheaply reach (dt <= 0, exact-zero
 * commands, the +/-180 fold). This header follows `TurnSignPolicy.h` /
 * `DirectionPolicy.h` / `PivotTrimPolicy.h`: pure, header-only `inline` free
 * functions with zero Arduino / `Config.h` surface, so a host test
 * (`test_turn_pd_policy`) links the SAME symbols production uses, and every
 * tunable arrives as a parameter instead of a global constant.
 *
 * Six functions answer "how hard, which way, and are we done" for one PD
 * step of an in-place turn:
 *
 *   1. `turnWrap180(delta)` folds a raw heading delta into `(-180, +180]` so
 *      the shortest signed error is always used (float twin of
 *      `TurnSignPolicy.h::normalizeTurnDelta`, which is `int32_t`-only).
 *   2. `turnPdStep(errNew, errOld, dtSec, kp, kd)` computes one PD command
 *      with a TIME-NORMALIZED derivative. The tick this runs on is a
 *      MEASURED ~8 ms that jitters to ~11.5 ms, so `kd` must carry real
 *      units of PWM per (deg/s) — dividing by the measured `dtSec` — rather
 *      than the mentor's raw per-iteration gain, which would silently
 *      re-scale every time the tick jitters.
 *   3. `turnSlewLimit(cmd, prevCmd, slewPerTick)` bounds how fast the
 *      command can GROW in the previous command's direction, so a heavy
 *      chassis cannot break traction on the initial kick.
 *   4. `turnCreepFloor(cmd, threshPwm, creepPwm)` snaps a small non-zero
 *      command up to the KINETIC creep floor (the chassis is already
 *      rotating by the turn's tail, so the static breakaway floor no longer
 *      applies).
 *   5. `turnSettleStep(absErr, toleranceDeg, count)` advances the
 *      N-consecutive-in-band-samples exit debounce; a single out-of-band
 *      sample resets the count to 0.
 *   6. `turnSettled(count, samplesRequired)` is the exit predicate over that
 *      count.
 *
 * ## Operator order (contract — apply identically in Drive.cpp and CTPS)
 *
 * ```
 * pdStep -> clamp to +/-power -> slewLimit -> creepFloor -> emit via turnWheelPattern
 * ```
 *
 * Clamp-then-slew bounds the ramp-up to the authority ceiling; slew-then-
 * creep lets the creep floor apply to the post-slew value instead of being
 * erased by the slew limiter on the first tick out of rest. Reordering
 * changes behavior — do not reorder.
 *
 * ## Sign frame
 *
 * All six functions operate on the PROGRESS-frame scalar error, not a
 * world-frame heading error:
 *
 * ```
 * errProgress = _turnDeltaSign * turnWrap180(target - yaw);
 * ```
 *
 * Positive means "still short of the target"; negative means "overshot and
 * must reverse". `_turnDeltaSign` is frozen at dispatch and re-applied
 * exactly once by `TurnSignPolicy.h::turnWheelPattern(sign, cmdProgress,
 * mode)` — these functions must therefore treat a negative command as a
 * legitimate reversal, not an error state.
 *
 * ## Why the creep floor is not `deadbandSnap`
 *
 * `deadbandSnap` (Drive.cpp) snaps a small command UP to the STATIC
 * breakaway floor (`PWM_MIN_MAGNITUDE`), which is the PWM needed to start
 * motion from a dead stop. By the turn's tail the chassis is already
 * rotating, so the lower KINETIC floor (the mentor's creep of 15) is the
 * correct one — snapping to the static floor there was the single largest
 * contributor to the overshoot this header exists to fix. `turnCreepFloor`
 * must never call `deadbandSnap` or reuse its threshold.
 *
 * ## Why the slew limiter is ramp-up-only
 *
 * A naive `constrain(cmd, prev-slew, prev+slew)` deadlocks: with `prev`
 * holding an emitted (post-creep) `+15`, an overshoot wants a negative
 * command; the naive constrain bounds it to `+15-slew`, and `turnCreepFloor`
 * snaps that straight back to `+15` (copysign preserves the wrong sign) —
 * the command can never cross zero and latches in the wrong direction until
 * the turn times out, on every overshoot. The rule enforced by
 * `turnSlewLimit`: the limiter binds ONLY when `|cmd|` is growing AND `cmd`
 * shares the previous command's sign (or the previous command was exactly
 * zero, i.e. starting from rest). A sign change or a shrinking magnitude
 * passes through unlimited — deceleration and reversal must never be
 * rate-limited. Ramp-down needs no code: the P term already shrinks with the
 * error.
 */

/**
 * @brief Fold a raw heading delta into the half-open range `(-180, +180]`.
 *
 * Picks the shortest signed rotation: `+181` folds to `-179`, `-181` folds to
 * `+179`, and `+180` stays `+180` (the range is closed at the top, open at
 * the bottom; `-180` folds to `+180`). Float twin of
 * `TurnSignPolicy.h::normalizeTurnDelta`, needed because the turn path is
 * widened to `float` end-to-end (kills the `int16_t` yaw-truncation bug).
 *
 * @param deltaDeg Raw signed degree delta (`target - current`), any FINITE
 *                  magnitude. A non-finite input is out of contract: `+/-INF`
 *                  would spin the fold loop forever (`inf - 360 == inf`). The
 *                  turn path cannot produce one — both operands are bounded
 *                  headings, so `deltaDeg` is within `[-360, +360]` and the
 *                  fold runs at most one iteration.
 * @return The equivalent delta folded into `(-180, +180]`.
 */
inline float turnWrap180(float deltaDeg)
{
    while (deltaDeg > 180.0f)
    {
        deltaDeg -= 360.0f;
    }
    while (deltaDeg <= -180.0f)
    {
        deltaDeg += 360.0f;
    }
    return deltaDeg;
}

/**
 * @brief One PD step with a TIME-NORMALIZED derivative.
 *
 * Computes `kp*errNew + kd*rate`, where `rate = (errNew - errOld) / dtSec`
 * is the measured rate of change of the error in deg/s. The mentor's
 * verified source computes a raw per-ITERATION difference (`errNew -
 * errOld`) with no division, so his effective damping is implicitly `Kd *
 * omega * loop_period` — correct only at his fixed loop rate. Our tick is a
 * MEASURED ~8 ms that jitters to ~11.5 ms, so porting that raw gain would
 * silently re-scale with the tick. Dividing by the measured `dtSec` makes
 * `kd` carry real units of PWM per (deg/s), independent of loop rate.
 *
 * @param errNew Current tick's progress-frame error (deg).
 * @param errOld Previous tick's progress-frame error (deg).
 * @param dtSec  Measured elapsed time since the previous step, in seconds.
 *               Guard: if `dtSec <= 0` the D term contributes exactly 0 —
 *               never divide by zero on a same-millis re-entry.
 * @param kp     Proportional gain (PWM per deg).
 * @param kd     TIME-NORMALIZED derivative gain (PWM per (deg/s)) — NOT the
 *               mentor's raw per-iteration gain.
 * @return Unclamped PD command. The caller clamps to +/-power next.
 */
inline float turnPdStep(float errNew, float errOld, float dtSec, float kp, float kd)
{
    float rate = 0.0f;
    if (dtSec > 0.0f)
    {
        rate = (errNew - errOld) / dtSec;
    }
    return kp * errNew + kd * rate;
}

/**
 * @brief Limit how fast `|cmd|` may grow relative to `prevCmd`, ramp-up only.
 *
 * Binds ONLY when the magnitude is growing AND `cmd` shares `prevCmd`'s sign
 * (a `prevCmd` of exactly zero — starting from rest — counts as compatible
 * with growth in either direction). A sign change or a shrinking magnitude
 * passes straight through, unlimited, so overshoot reversal and deceleration
 * are never rate-limited (see the file docblock's "why ramp-up-only"
 * section for the deadlock this avoids). Applied AFTER the power clamp,
 * BEFORE the creep floor.
 *
 * @param cmd         Candidate command for this tick (post power-clamp).
 * @param prevCmd     Previous tick's EMITTED (post-creep) command.
 * @param slewPerTick Maximum allowed magnitude growth per tick (PWM).
 * @return `cmd`, or the growth-limited value when the ramp-up rule binds.
 */
inline float turnSlewLimit(float cmd, float prevCmd, float slewPerTick)
{
    bool oppositeSigns = (cmd > 0.0f && prevCmd < 0.0f) || (cmd < 0.0f && prevCmd > 0.0f);
    bool growing       = fabsf(cmd) > fabsf(prevCmd);
    if (oppositeSigns || !growing)
    {
        return cmd;    // reversal, deceleration, or no growth: pass through unlimited.
    }

    if (cmd >= 0.0f)
    {
        float maxCmd = prevCmd + slewPerTick;
        return (cmd > maxCmd) ? maxCmd : cmd;
    }
    else
    {
        float minCmd = prevCmd - slewPerTick;
        return (cmd < minCmd) ? minCmd : cmd;
    }
}

/**
 * @brief Snap a small non-zero command up to the KINETIC creep floor.
 *
 * `|cmd| < threshPwm` snaps to `copysign(creepPwm, cmd)`; `cmd == 0` stays
 * EXACTLY 0 (a commanded stop is a stop — only non-zero small commands
 * creep). This is the kinetic floor, not the static breakaway floor: by the
 * turn's tail the chassis is already rotating, so it needs less PWM to keep
 * turning than it needed to start. Do not call `deadbandSnap` here (see the
 * file docblock).
 *
 * @param cmd       Candidate command for this tick (post slew-limit).
 * @param threshPwm Below this magnitude the PD is considered to have lost
 *                   authority.
 * @param creepPwm  Fixed kinetic-floor magnitude to emit instead.
 * @return `cmd` unchanged if `cmd == 0` or `|cmd| >= threshPwm`; otherwise
 *         `copysign(creepPwm, cmd)`.
 */
inline float turnCreepFloor(float cmd, int16_t threshPwm, int16_t creepPwm)
{
    if (cmd == 0.0f)
    {
        return 0.0f;
    }
    if (fabsf(cmd) < (float) threshPwm)
    {
        return copysignf((float) creepPwm, cmd);
    }
    return cmd;
}

/**
 * @brief One step of the N-consecutive-in-band-samples exit debounce.
 *
 * @param absErr       Absolute progress-frame error for this tick (deg).
 * @param toleranceDeg In-band threshold (deg). Strictly less-than: a sample
 *                      exactly AT the tolerance boundary is OUT of band.
 * @param count        Consecutive in-band count carried in from the previous
 *                      tick.
 * @return `count + 1` (saturating at `UINT8_MAX`) when `absErr <
 *         toleranceDeg`; `0` on any out-of-band sample.
 */
inline uint8_t turnSettleStep(float absErr, float toleranceDeg, uint8_t count)
{
    if (absErr < toleranceDeg)
    {
        return (count < UINT8_MAX) ? (uint8_t) (count + 1) : count;
    }
    return 0;
}

/**
 * @brief Exit predicate for the settle debounce.
 *
 * @param count           Consecutive in-band count from `turnSettleStep`.
 * @param samplesRequired Number of consecutive in-band samples required to
 *                         declare the turn settled.
 * @return `true` once `count` has reached `samplesRequired`.
 */
inline bool turnSettled(uint8_t count, uint8_t samplesRequired)
{
    return count >= samplesRequired;
}

#endif    // TURN_PD_POLICY_H
