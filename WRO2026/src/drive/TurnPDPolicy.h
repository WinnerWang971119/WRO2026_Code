#ifndef TURN_PD_POLICY_H
#define TURN_PD_POLICY_H

#include <math.h>
#include <stdint.h>

/**
 * @file TurnPDPolicy.h
 * @brief Shared turn kernel: the float heading fold, the time-normalized PD
 *        kernel the straight-line GYRO heading hold reuses, and the settle
 *        debounce the in-place turn profile reuses.
 *
 * The in-place turn no longer runs a per-tick PD law — it moved to the gated
 * speed profile (breakaway -> CRUISE -> CREEP -> SETTLE) in
 * `TurnProfilePolicy.h` (ADR-078, superseding the PD clause of ADR-068 and
 * ADR-077). What remains in this header is the set of pure helpers that BOTH
 * the profile and the straight-line hold still share; the deleted turn PD law
 * (`turnPdStepRaw`), its ramp-up slew limiter (`turnSlewLimit`), and its
 * kinetic creep floor (`turnCreepFloor`) left with that controller.
 *
 * This header follows `TurnSignPolicy.h` / `DirectionPolicy.h` /
 * `PivotTrimPolicy.h`: pure, header-only `inline` free functions with zero
 * Arduino / `Config.h` surface, so a host test links the SAME symbols
 * production uses, and every tunable arrives as a parameter instead of a
 * global constant.
 *
 * Four functions remain:
 *
 *   1. `turnWrap180(delta)` folds a raw heading delta into `(-180, +180]` so
 *      the shortest signed error is always used (float twin of
 *      `TurnSignPolicy.h::normalizeTurnDelta`, which is `int32_t`-only). Used
 *      by both the turn profile and the straight-line hold.
 *   2. `turnPdStep(errNew, errOld, dtSec, kp, kd)` computes one PD command
 *      with a TIME-NORMALIZED derivative (`kd * (errNew - errOld) / dtSec`).
 *      This is NOT the in-place turn's law — it is retained SOLELY as the
 *      kernel the straight-line heading hold reuses (`GyroHoldPolicy.h`,
 *      ADR-075), where the caller synthesizes `errOld` so the derivative is a
 *      pure measured yaw-rate on a clean (non-quantized) signal.
 *   3. `turnSettleStep(absErr, toleranceDeg, count)` advances the
 *      N-consecutive-in-band-samples exit debounce; a single out-of-band
 *      sample resets the count to 0. Reused by the turn profile's SETTLE exit.
 *   4. `turnSettled(count, samplesRequired)` is the exit predicate over that
 *      count.
 *
 * ## Sign frame
 *
 * The settle debounce operates on the PROGRESS-frame scalar error, not a
 * world-frame heading error:
 *
 * ```
 * errProgress = _turnDeltaSign * turnWrap180(target - yaw);
 * ```
 *
 * Positive means "still short of the target"; negative means "overshot and
 * must reverse". `_turnDeltaSign` is frozen at dispatch and re-applied exactly
 * once by `TurnSignPolicy.h::turnWheelPattern(sign, cmdProgress, mode)`, so a
 * negative command is a legitimate reversal, not an error state.
 * `turnSettleStep` takes the ABSOLUTE error, so a still-overshot turn (a
 * negative error whose magnitude is below tolerance) cannot be mistaken for
 * settled.
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
 * @brief One PD step with a TIME-NORMALIZED derivative — the kernel the
 *        STRAIGHT-LINE heading hold reuses (the in-place turn runs the phase
 *        profile in `TurnProfilePolicy.h`, not a PD law).
 *
 * Computes `kp*errNew + kd*rate`, where `rate = (errNew - errOld) / dtSec`
 * is the measured rate of change of the error in deg/s. This is the right
 * model for a controller riding a CLEAN, continuous signal — the straight-
 * line hold (`GyroHoldPolicy.h`, ADR-075) synthesizes `errOld` so `rate`
 * becomes a pure measured yaw-rate. The in-place turn does NOT use it: it
 * reads a whole-degree quantized gyro on a jittery ~8-11.5 ms cooperative
 * tick, where dividing a 1-degree quantization step by a small, varying dt
 * would inject a timing-dependent D spike, so the turn runs the loop-rate-
 * immune speed profile (`TurnProfilePolicy.h`, ADR-078) instead of any PD law.
 *
 * @param errNew Current tick's progress-frame error (deg).
 * @param errOld Previous tick's progress-frame error (deg).
 * @param dtSec  Measured elapsed time since the previous step, in seconds.
 *               Guard: if `dtSec <= 0` the D term contributes exactly 0 —
 *               never divide by zero on a same-millis re-entry.
 * @param kp     Proportional gain (PWM per deg).
 * @param kd     TIME-NORMALIZED derivative gain (PWM per (deg/s)).
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
