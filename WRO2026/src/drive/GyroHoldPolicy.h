#ifndef GYRO_HOLD_POLICY_H
#define GYRO_HOLD_POLICY_H

#include "TurnPDPolicy.h"      // turnWrap180, turnPdStep — reused verbatim (also pulls in <stdint.h>/<math.h>).
#include "TurnSignPolicy.h"    // turnWheelPattern, TurnWheels — chassis-handedness source of truth.

/**
 * @file GyroHoldPolicy.h
 * @brief Single source of truth for the R4-side GYRO straight-line heading-hold
 *        PD control law (ADR-072).
 *
 * `Drive.cpp` IS host-compiled — integration targets link the real translation
 * unit against the `hal_mocks/` recording surfaces (issue #106 / ADR-030), so
 * the dispatched `Move(L, R)` is observable end-to-end. But pure decision math
 * still belongs in a header-only policy seam: it is the only way to exercise the
 * control law in isolation, with zero mocks and every tunable injected, over the
 * edge cases a stateful FSM test cannot cheaply reach (dt <= 0, the +/-180 fold,
 * the dynamic never-reverse clamp, negative base). This header follows
 * `TurnPDPolicy.h` / `TurnSignPolicy.h`: pure, header-only `inline` free
 * functions with zero Arduino / `Config.h` surface, so a host test
 * (`test_gyro_hold_policy`) links the SAME symbols production uses, and every
 * tunable arrives as a parameter instead of a global constant.
 *
 * Two functions answer "how hard to correct, and how does that land on the
 * wheels" for one PD step of a straight-line heading hold:
 *
 *   1. `gyroHoldCorrection(errNew, errOld, dtSec, kp, kd, corrLimitPwm)` runs
 *      one clamped PD step. It REUSES `turnPdStep` verbatim, then clamps the
 *      result to `+/-corrLimitPwm`. It mints no new control law — the hold and
 *      the turn share the identical PD kernel.
 *   2. `gyroHoldWheels(base, corr, pwmMax)` mixes the symmetric drive base and
 *      the signed heading correction into a left/right PWM pair, each capped to
 *      `+/-pwmMax`.
 *
 * ## Derivative-on-measurement (fold-safe) — why the caller synthesizes errOld
 *
 * `gyroHoldCorrection` does NOT compute the error history itself. The caller
 * passes `errOld = errNew + turnWrap180(yaw - prevYaw)`, so `turnPdStep`'s
 * derivative `(errNew - errOld) / dtSec` collapses to
 * `-turnWrap180(yaw - prevYaw) / dtSec` — a PURE measured yaw-rate, independent
 * of the commanded heading. A heading step between chained GYRO segments
 * therefore injects ZERO derivative kick (the target cancels out of the D
 * term), which removes the segment-boundary problem structurally, with no
 * begin/end seeding hooks. And because the yaw delta is wrapped ONCE
 * (`turnWrap180(yaw - prevYaw)`) rather than differenced from two separately
 * wrapped errors, an error near +/-180 cannot straddle the fold into a spurious
 * -360 derivative jerk. Fold-safety is the caller's contract; this function
 * just runs the kernel and clamps.
 *
 * ## Never-reverse dynamic clamp — why the limit is passed in, not a constant
 *
 * `corrLimitPwm` is a DYNAMIC ceiling the caller computes as
 * `min(GYRO_HOLD_MAX_CORR_PWM, |base|)`. Because the correction magnitude is
 * held at or below `|base|`, `base - corr` and `base + corr` can never cross
 * zero: no wheel ever reverses. A STATIC-only clamp would flip the inner wheel
 * exactly when the chassis is slowest — `base` shrinks through the decel tail
 * below any fixed knob, so the limit has to track `|base|` tick by tick. This
 * is a documented deviation from the mentor's unclamped `gyro_move`.
 *
 * ## PD + clamp only — no slew, no creep, no integrator (unlike the turn)
 *
 * `TurnPDPolicy.h` adds a slew limiter, a kinetic creep floor, and a settle
 * debounce on top of its PD step. This hold deliberately adds NONE of them. The
 * turn starts from rest and scrubs in place, so it needs slew (to keep traction
 * on the initial kick) and creep (to keep rotating at the tail). The hold rides
 * an already-moving base and its goal is `corr -> 0`, so those stages have no
 * purpose here. There is also no integrator: a constant motor imbalance would
 * leave a steady-state heading error of `disturbance_PWM / kp`, and the named
 * remedy is to apply the existing base feedforward (or an I term) at the bench
 * — not to complicate this pure kernel (ADR-072).
 *
 * ## Wheel-mix polarity — reuse the turn's chassis handedness, never a new sign
 *
 * The correction-to-wheel polarity is a known footgun (project memory
 * `gotcha-gyro-inverted-motor-mapping`): the mentor's literal
 * `Move(power + corr, power - corr)` is INVERTED versus this repo's documented
 * `Move(30, 0) curves forward-right` contract. So `gyroHoldWheels` does NOT
 * mint a sign and does NOT copy meat.ino's per-motor reverse flags. It reuses
 * `TurnSignPolicy.h::turnWheelPattern(+1, corr, COUNTER_ROT)` — the same
 * bench-verified chirality the in-place turn uses — applied on top of the
 * straight base. A CCW (`+1`) counter-rotation puts the correction on the
 * wheels as `{left = -corr, right = +corr}`, so a positive `corr` (target CCW
 * of the current yaw) yields `right > left` and the chassis yaws CCW toward the
 * target. A host test cross-asserts `gyroHoldWheels(base, corr, pwmMax)` against
 * `base + turnWheelPattern(+1, corr, 0)` so the two policies can never drift.
 */

/**
 * @brief One clamped PD step for the straight-line heading hold.
 *
 * Runs `turnPdStep(errNew, errOld, dtSec, kp, kd)` — the SAME PD kernel the
 * in-place turn uses — and clamps the result to the symmetric dynamic limit
 * `[-corrLimitPwm, +corrLimitPwm]`. No slew, no creep, no integrator (see the
 * file docblock).
 *
 * @param errNew       Current tick's app-frame heading error in degrees,
 *                      `turnWrap180(headingApp - yaw)`. Positive means the
 *                      target heading is CCW of the current yaw.
 * @param errOld       FOLD-SAFE synthesized previous error that the CALLER
 *                      computes as `errNew + turnWrap180(yaw - prevYaw)`. This
 *                      makes `turnPdStep`'s derivative a pure, single-wrap
 *                      measured yaw-rate (derivative-on-measurement),
 *                      independent of the target — so a heading step between
 *                      chained segments produces no derivative kick.
 * @param dtSec        Measured elapsed time since `prevYaw`'s sample, in
 *                      seconds. `turnPdStep` guards `dtSec <= 0` by contributing
 *                      exactly 0 D — never a same-millis divide-by-zero.
 * @param kp           Proportional gain (PWM per deg).
 * @param kd           TIME-NORMALIZED derivative gain (PWM per (deg/s)) — the
 *                      tick jitters, so `kd` must carry rate units (ADR-068).
 * @param corrLimitPwm DYNAMIC symmetric clamp, `min(GYRO_HOLD_MAX_CORR_PWM,
 *                      |base|)`, computed by the caller. Keeping `|corr|` at or
 *                      below `|base|` is what guarantees `base +/- corr` never
 *                      reverses a wheel. Expected non-negative.
 * @return The PD command clamped to `[-corrLimitPwm, +corrLimitPwm]`.
 */
inline float gyroHoldCorrection(float errNew, float errOld, float dtSec, float kp, float kd,
                                int16_t corrLimitPwm)
{
    float cmd   = turnPdStep(errNew, errOld, dtSec, kp, kd);
    float limit = (float) corrLimitPwm;
    if (cmd > limit)
    {
        return limit;
    }
    if (cmd < -limit)
    {
        return -limit;
    }
    return cmd;
}

/**
 * @brief Result of the wheel-mix decision: a left/right PWM pair.
 *
 * @note These are final dispatch values — the caller drives them straight to
 *       `Move(left, right)`. NO per-wheel `deadbandSnap` is applied: snapping
 *       each wheel independently would destroy the differential. The base is
 *       deadband-snapped by the caller BEFORE it enters this mix, not after.
 */
struct GyroHoldWheels
{
    int16_t left;     ///< left-wheel PWM (signed; negative = backward).
    int16_t right;    ///< right-wheel PWM (signed; negative = backward).
};

/**
 * @brief Mix a symmetric drive base and a signed heading correction into a
 *        left/right PWM pair.
 *
 * Computes `left = base - corr`, `right = base + corr`, each capped to
 * `[-pwmMax, +pwmMax]`. The correction is applied through
 * `TurnSignPolicy.h::turnWheelPattern(+1, corr, COUNTER_ROT)` — which yields
 * `{left = -corr, right = +corr}` — added to the base, so the chassis handedness
 * comes from the bench-verified turn convention rather than a fresh sign (the
 * inverted-motor-mapping footgun; see the file docblock). A positive `corr`
 * (target CCW of the current yaw) therefore gives `right > left`, yawing the
 * chassis CCW. This mix EQUALS `base + turnWheelPattern(+1, corr, 0)`, and a
 * host test asserts that cross-consistency.
 *
 * The signed `corr` is deliberately passed as `turnWheelPattern`'s magnitude
 * argument so the same `-sign*mag` / `+sign*mag` arithmetic carries its sign
 * through — a negative `corr` correctly biases the opposite wheel.
 *
 * @param base   Symmetric drive PWM (signed; negative = reverse), already
 *               `deadbandSnap`-floored by the caller.
 * @param corr   Signed heading correction from `gyroHoldCorrection`, already
 *               clamped to `+/-min(GYRO_HOLD_MAX_CORR_PWM, |base|)`.
 * @param pwmMax Legal PWM ceiling; each wheel is capped to `[-pwmMax, +pwmMax]`.
 * @return `{left, right}` signed PWM pair ready for `Move(left, right)`.
 * @note No wheel reverses PROVIDED the caller passed `|corr| <= |base|` (the
 *       dynamic clamp `gyroHoldCorrection` enforces): both `base - corr` and
 *       `base + corr` then keep `base`'s sign or land on zero, and the `pwmMax`
 *       cap only shrinks magnitude — it can never flip a sign.
 */
inline GyroHoldWheels gyroHoldWheels(int16_t base, int16_t corr, int16_t pwmMax)
{
    // Chirality from the turn convention only — never a new sign, never
    // meat.ino's per-motor reverse flags. CCW (+1) counter-rotation lays the
    // correction on the wheels as {left = -corr, right = +corr}; the straight
    // base rides on top. int32 intermediates so base +/- corr cannot overflow
    // before the pwmMax cap.
    TurnWheels mix = turnWheelPattern((int8_t) +1, corr, (uint8_t) 0 /* COUNTER_ROT */);

    int32_t left  = (int32_t) base + (int32_t) mix.left;
    int32_t right = (int32_t) base + (int32_t) mix.right;

    if (left > pwmMax)
    {
        left = pwmMax;
    }
    else if (left < -pwmMax)
    {
        left = -pwmMax;
    }

    if (right > pwmMax)
    {
        right = pwmMax;
    }
    else if (right < -pwmMax)
    {
        right = -pwmMax;
    }

    GyroHoldWheels wheels;
    wheels.left  = (int16_t) left;
    wheels.right = (int16_t) right;
    return wheels;
}

#endif    // GYRO_HOLD_POLICY_H
