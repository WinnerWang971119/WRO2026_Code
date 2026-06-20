#ifndef TURN_SIGN_POLICY_H
#define TURN_SIGN_POLICY_H

#include <stdint.h>

/**
 * @file TurnSignPolicy.h
 * @brief Single source of truth for the turn-direction sign + wheel-pattern math
 *        (ADR-045, IMU yaw sign normalization).
 *
 * Three pure free functions answer the "which way, how hard, on which wheels"
 * question for an in-place turn:
 *
 *   1. `normalizeTurnDelta(delta)` folds a signed degree delta into the half-open
 *      range `(-180, +180]` so the shortest rotation is always chosen.
 *   2. `turnSign(foldedDelta)` captures the rotation sign at dispatch: `+1` means
 *      yaw must INCREASE (CCW / left turn in the corrected app frame), `-1` means
 *      yaw must DECREASE (CW / right turn).
 *   3. `turnWheelPattern(sign, pwmMag, mode)` maps that sign + a magnitude + a
 *      motor mode (0=COUNTER_ROT, 1=PIVOT_LEFT, 2=PIVOT_RIGHT) to a left/right
 *      PWM pair. Per Drive's setSpeed semantics a CCW (`+1`) counter-rotation is
 *      `L = -sign*pwm < 0`, `R = +sign*pwm > 0`; CW (`-1`) is the mirror.
 *
 * These were previously inlined in Drive.cpp: the fold appeared verbatim in
 * `_execTurnDeg`, `_execTurnToYaw`, and `_updateTurnProfile`; the sign capture in
 * the two `_exec*` dispatchers; and the wheel-pattern switch in
 * `_updateTurnProfile`. Drive.cpp is NOT host-compiled (heavy MatrixMiniR4
 * deps), so none of that logic had automated coverage. Promoting it to
 * header-only `inline` free functions lets production (Drive.cpp) and a host test
 * (`test_turn_sign_policy`) link the SAME symbols — the established
 * `DirectionPolicy.h` / `PivotTrimPolicy.h` pattern, which removes the drift risk
 * of hand-copied test replicas.
 *
 * This header is deliberately self-contained (only `<stdint.h>`, zero Arduino /
 * Config.h surface) so the test target needs no mocks, exactly like
 * `test_pivot_trim_policy`. `deadbandSnap` is intentionally NOT applied here — it
 * stays in Drive.cpp, which snaps each returned PWM after this function returns.
 */

/**
 * @brief Result of the wheel-pattern decision: a left/right PWM pair.
 *
 * @note These are pre-deadband-snap values. The caller (Drive.cpp) applies
 *       `deadbandSnap` to each before driving the motors.
 */
struct TurnWheels
{
    int16_t left;     ///< left-wheel PWM (signed; negative = backward).
    int16_t right;    ///< right-wheel PWM (signed; negative = backward).
};

/**
 * @brief Fold a signed degree delta into the half-open range `(-180, +180]`.
 *
 * Picks the shortest signed rotation between two headings: a delta of `+270`
 * folds to `-90`, `-270` folds to `+90`, and `+180` stays `+180` (the range is
 * closed at the top, open at the bottom). Verbatim from Drive.cpp's three fold
 * sites (`_execTurnDeg`, `_execTurnToYaw`, `_updateTurnProfile`).
 *
 * @param delta Raw signed degree delta (`target - current`), any magnitude.
 * @return The equivalent delta folded into `(-180, +180]`.
 */
inline int32_t normalizeTurnDelta(int32_t delta)
{
    while (delta > 180)
    {
        delta -= 360;
    }
    while (delta <= -180)
    {
        delta += 360;
    }
    return delta;
}

/**
 * @brief Rotation sign captured at dispatch from an already-folded delta.
 *
 * `+1` means yaw must INCREASE (CCW / left turn in the corrected app frame);
 * `-1` means yaw must DECREASE (CW / right turn). A zero delta resolves to `-1`,
 * matching Drive.cpp's `(deltaNorm > 0) ? +1 : -1` ternary (a true zero delta is
 * filtered out upstream by the turn-tolerance check before this is reached).
 *
 * @param foldedDelta Delta already folded via `normalizeTurnDelta`.
 * @return `+1` if `foldedDelta > 0` (CCW), otherwise `-1` (CW).
 */
inline int8_t turnSign(int32_t foldedDelta)
{
    return (foldedDelta > 0) ? (int8_t) +1 : (int8_t) -1;
}

/**
 * @brief Map a rotation sign + magnitude + motor mode to a left/right PWM pair.
 *
 * Verbatim from Drive.cpp's `_updateTurnProfile` switch. Mode encoding:
 *   - 0 COUNTER_ROT — both wheels spin opposite directions about chassis center.
 *   - 1 PIVOT_LEFT  — left wheel stationary, right wheel drives the rotation.
 *   - 2 PIVOT_RIGHT — right wheel stationary, left wheel drives the rotation.
 *   - default — unknown mode falls back to counter-rotation rather than refusing
 *     to turn.
 *
 * For a CCW turn (`sign == +1`) counter-rotation yields `L = -pwmMag < 0`,
 * `R = +pwmMag > 0`; CW (`sign == -1`) is the mirror. The returned PWMs are NOT
 * deadband-snapped — the caller snaps after.
 *
 * @param sign   Rotation sign from `turnSign` (`+1` CCW, `-1` CW).
 * @param pwmMag Unsigned PWM magnitude to apply on the driving wheel(s).
 * @param mode   Motor mode (0=COUNTER_ROT, 1=PIVOT_LEFT, 2=PIVOT_RIGHT).
 * @return `{left, right}` signed PWM pair (pre-deadband-snap).
 */
inline TurnWheels turnWheelPattern(int8_t sign, int16_t pwmMag, uint8_t mode)
{
    int16_t L = 0;
    int16_t R = 0;
    switch (mode)
    {
    case 0:    // COUNTER_ROT — both wheels spin opposite directions.
        L = (int16_t) (-sign * pwmMag);
        R = (int16_t) (+sign * pwmMag);
        break;
    case 1:    // PIVOT_LEFT — left wheel stationary, right wheel drives the rotation.
        L = 0;
        R = (int16_t) (+sign * pwmMag);
        break;
    case 2:    // PIVOT_RIGHT — right wheel stationary, left wheel drives the rotation.
        L = (int16_t) (-sign * pwmMag);
        R = 0;
        break;
    default:    // Unknown mode — fall back to counter-rotation rather than refusing to turn.
        L = (int16_t) (-sign * pwmMag);
        R = (int16_t) (+sign * pwmMag);
        break;
    }

    TurnWheels wheels;
    wheels.left  = L;
    wheels.right = R;
    return wheels;
}

#endif    // TURN_SIGN_POLICY_H
