/**
 * @file WROMath.h
 * @brief Header-only collection of small math helpers shared across modules.
 *
 * @details
 * Three rules govern what lives here:
 *   1. The helper is pure (no side effects on global state).
 *   2. The helper is called from at least two modules — duplication has been
 *      observed, not anticipated.
 *   3. Implementation fits comfortably as `inline` (no template metaprogramming,
 *      no heavy stdlib pulls).
 *
 * Anything that doesn't meet all three stays as a private member of its owning
 * class. Don't speculatively add helpers here — wait until the second caller
 * materialises, then factor out.
 *
 * Currently included:
 *   - `deadbandSnap`  — PWM saturation cap + motor-deadband floor (Drive + LineFollower).
 *   - `presetToPwm`   — `Config::SpeedPreset` → PWM (Drive builders + LineFollower).
 *   - `cmToEncDeg`    — wheel-distance to encoder-degree conversion (Drive distance moves).
 *   - `encDegToCm`    — inverse of `cmToEncDeg` (LineFollower's tShape ramp tracking).
 *
 * @note Free functions in the global namespace, intentionally not in `namespace WROMath`.
 *       The names are specific enough that collision risk is low, and keeping callers free
 *       of `WROMath::` prefixes matches the existing inline-helper style on Arduino.
 */

#ifndef WRO_MATH_H
#define WRO_MATH_H

#include <Arduino.h>
#include <stdint.h>

#include "Config.h"

// =============================================================================================
// PWM clamping
// =============================================================================================

/**
 * @brief Clamp a raw PWM command into {0} U [PWM_MIN_MAGNITUDE, PWM_MAX]
 *        (with mirrored negative range), snapping non-zero small magnitudes
 *        UP to the deadband floor.
 *
 * Apply at every boundary where a computed PWM hits the lower MCU:
 *   * `LineFollower::update()`'s P+D mix (so small line-correction commands
 *     are honored instead of stalling silently in the dead zone).
 *   * `Drive::setSpeed()` after `_applyBias` (so any caller-issued continuous
 *     command is also deadband-corrected).
 *   * `_updateProfile()`'s speed-emission per tick (so ramp setpoints below
 *     the floor still move the chassis).
 *
 * @param pwm Raw signed PWM in any range.
 * @return Clamped/snapped PWM:
 *           - `pwm == 0` -> returns `0` unchanged (intentional stop)
 *           - `0 < |pwm| < PWM_MIN_MAGNITUDE` -> returns `+/-PWM_MIN_MAGNITUDE` (sign preserved)
 *           - `|pwm| > PWM_MAX` -> returns `+/-PWM_MAX` (sign preserved)
 *           - else -> returns `pwm` unchanged
 */
inline int16_t deadbandSnap(int16_t pwm)
{
    if (pwm == 0)
    {
        return 0;
    }
    if (pwm > 0)
    {
        if (pwm < Config::PWM_MIN_MAGNITUDE) return Config::PWM_MIN_MAGNITUDE;
        if (pwm > Config::PWM_MAX)           return Config::PWM_MAX;
        return pwm;
    }
    // pwm < 0
    if (pwm > -Config::PWM_MIN_MAGNITUDE) return -Config::PWM_MIN_MAGNITUDE;
    if (pwm < -Config::PWM_MAX)           return -Config::PWM_MAX;
    return pwm;
}

// =============================================================================================
// Speed-preset resolution
// =============================================================================================

/**
 * @brief Map a `Config::SpeedPreset` to its associated PWM cruise target.
 *
 * Single source of truth for the SLOW / NORMAL / FAST → PWM mapping. Both Drive's fluent
 * builders and `LineFollower::update()` resolve cruise via this function, so changing the
 * mapping in one place propagates everywhere.
 *
 * @param preset Preset to resolve.
 * @return `Config::SPEED_SLOW`, `SPEED_NORMAL`, or `SPEED_FAST` depending on input;
 *         `Config::SPEED_NORMAL` for any unrecognised value (defensive default — calms the
 *         compiler on the no-default switch and survives a future enum extension that
 *         forgets to update this function).
 */
inline int16_t presetToPwm(Config::SpeedPreset preset)
{
    switch (preset)
    {
    case Config::SpeedPreset::SLOW:
        return Config::SPEED_SLOW;
    case Config::SpeedPreset::NORMAL:
        return Config::SPEED_NORMAL;
    case Config::SpeedPreset::FAST:
        return Config::SPEED_FAST;
    }
    return Config::SPEED_NORMAL;
}

// =============================================================================================
// Wheel-distance / encoder-degree conversions
// =============================================================================================

/**
 * @brief Convert a wheel-travel distance in centimetres to encoder degrees.
 *
 * Formula: `revs = (cm * 10) / (PI * WHEEL_DIAM_MM)`, then `degs = revs * 360`.
 * The `* 10` converts cm to mm so the wheel diameter (also mm) cancels cleanly.
 *
 * @param cm Distance in centimetres. Negative inputs are clamped to 0 — converting a
 *           negative float to `uint16_t` is undefined behavior in C++ when the truncated
 *           value is out of range. Call sites pass non-negative `cm` and use a separate
 *           sign on `power` for reverse moves; the clamp is defence-in-depth.
 * @return Encoder degrees the wheel must rotate to cover that distance.
 *
 * @note Source of truth for this conversion — both `_execDriveDistRaw/Sync/Gyro` and
 *       `_execProfileMove` (distance variant) call this. Changing the formula in this
 *       one place propagates to every distance-bounded move.
 */
inline uint16_t cmToEncDeg(float cm)
{
    if (cm < 0.0f)
    {
        cm = 0.0f;
    }
    const float revs = (cm * 10.0f) / ((float) PI * Config::WHEEL_DIAM_MM);
    return (uint16_t) (revs * 360.0f);
}

/**
 * @brief Convert encoder degrees back to a wheel-travel distance in centimetres.
 *
 * Algebraic inverse of `cmToEncDeg`. Used by `LineFollower::update()` to convert the
 * delta of `Drive.getEncL()` since enable into the cm of travel needed to determine
 * whether the tShape cruise ramp has been consumed.
 *
 * @param encDeg Encoder degrees (signed — negative direction is fine).
 * @return Travel distance in centimetres (signed; same sign as input).
 *
 * @note `cmToEncDeg(encDegToCm(x)) ≈ x` (modulo float→uint16 truncation on the forward
 *       direction). Don't rely on bit-exact round-trip — the conversion is for ratio
 *       comparisons (`traveledCm >= rampCm`), not exact arithmetic.
 */
inline float encDegToCm(int32_t encDeg)
{
    return (float) encDeg / 360.0f * ((float) PI * Config::WHEEL_DIAM_MM / 10.0f);
}

#endif    // WRO_MATH_H
