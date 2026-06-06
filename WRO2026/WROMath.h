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
 *   - `stopAtLineArmed` — arm-window predicate for `Drive.stopAtLine()` (Drive seek + host test).
 *   - `debounceStep`  — one streak-counter step for an event debounce (Drive seek + host test).
 *   - `stopAtLineSeekMaxedOut` — max-distance fault predicate for the seek (Drive seek + host test).
 *   - `stopAtLineProgressed`   — progress-detector for the seek watchdog (Drive seek + host test).
 *   - `stopAtLineStalled`      — no-progress watchdog timeout for the seek (Drive seek + host test).
 *
 * @note The stopAtLine predicates are extracted from `Drive::_updateStopAtLine()` purely so the
 *       arm-window, detection-debounce, max-distance, and stall-watchdog DECISIONS are
 *       unit-testable on the host (the stateful seek itself is bench-verified, not CI-verified —
 *       see the 2026-05-30 stop-at-line plan, TC11, and the 2026-06-02 plan, TC16/TC17). They
 *       follow the rule above because the Drive state machine is one caller and the host test is
 *       the second.
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
 *           Results that would exceed UINT16_MAX are likewise clamped to UINT16_MAX —
 *           float→uint16_t is UB above the representable range (wraps in practice).
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
    const float degs = revs * 360.0f;
    if (degs > (float) UINT16_MAX)
    {
        return UINT16_MAX;  // large cm: clamp to ceiling — float→uint16_t is UB above range
    }
    return (uint16_t) degs;
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

// =============================================================================================
// stopAtLine() seek decisions — pure, host-testable (Drive._updateStopAtLine + TC11)
// =============================================================================================

/**
 * @brief Whether `Drive.stopAtLine()` detection should be ARMED at the given travel.
 *
 * The seek drives blind until the chassis is within `armWithin` cm of the expected endpoint,
 * THEN arms its `T_OR_CROSS` detector — so a line crossed earlier en route cannot trigger a
 * premature stop. The arm threshold is `expectedCm - armWithin`, clamped at 0 so the small-hop
 * / "already near the line" case (`armWithin >= expectedCm`) arms from the first tick instead of
 * computing a negative threshold.
 *
 * @param traveledCm Distance travelled so far along the seek direction, in centimetres
 *                   (always non-negative — the caller measures `abs(encDelta)`).
 * @param expectedCm Nominal distance to the expected crossing line, in centimetres.
 * @param armWithin  Arm-window half-length in centimetres (`Config::STOP_AT_LINE_ARM_WINDOW_CM`
 *                   by default, overridable per call).
 * @return `true` once `traveledCm >= max(0, expectedCm - armWithin)`; `false` while still
 *         outside the arm window.
 *
 * @note Source of truth for the arm-window DECISION — `Drive::_updateStopAtLine()` calls this so
 *       the SEEK → ARMED transition and the host test (TC11) share one implementation.
 */
inline bool stopAtLineArmed(float traveledCm, float expectedCm, float armWithin)
{
    float threshold = expectedCm - armWithin;
    if (threshold < 0.0f)
    {
        threshold = 0.0f;
    }
    return traveledCm >= threshold;
}

/**
 * @brief Advance an N-of-N consecutive-sample debounce counter by one observation.
 *
 * A streak counter increments while the watched event is present and resets to 0 the moment a
 * sample misses, so only `requiredTicks` consecutive hits confirm. Saturates at `requiredTicks`
 * to avoid unbounded growth while the event stays present.
 *
 * @param streak   Current consecutive-hit count.
 * @param eventNow `true` if the watched event is present on this sample.
 * @param requiredTicks Consecutive hits needed to confirm (the saturation cap).
 * @return The updated streak: `0` when `eventNow` is false; otherwise `min(streak + 1,
 *         requiredTicks)`.
 *
 * @note Source of truth for the detection-debounce DECISION — `Drive::_updateStopAtLine()`
 *       calls this each ARMED tick (with `eventNow = getJunctionType() == T_OR_CROSS` and
 *       `requiredTicks = Config::STOP_AT_LINE_DETECT_TICKS`) so the confirm logic and the host
 *       test (TC11) share one implementation. Detection is confirmed when the return value
 *       reaches `requiredTicks`.
 */
inline uint8_t debounceStep(uint8_t streak, bool eventNow, uint8_t requiredTicks)
{
    if (!eventNow)
    {
        return 0;
    }
    if (streak >= requiredTicks)
    {
        return requiredTicks;
    }
    return (uint8_t) (streak + 1);
}

/**
 * @brief Whether a `Drive.stopAtLine()` seek has reached or exceeded its absolute max travel.
 *
 * The seek faults `Fault::LINE_NOT_FOUND` once it has driven `maxCm` cm without confirming a
 * crossing — both the early SEEK phase and the ARMED phase enforce this so a seek that arms late
 * (tight `setMaxDistance`) cannot overshoot its cap. The comparison is `>=` so the boundary value
 * itself counts as maxed-out.
 *
 * @param traveledCm Distance travelled so far along the seek direction, in centimetres (always
 *                   non-negative — the caller measures `abs(encDelta)`).
 * @param maxCm      Absolute max travel before faulting, in centimetres.
 * @return `true` once `traveledCm >= maxCm`; `false` while still under the cap.
 *
 * @note Source of truth for the max-distance DECISION — `Drive::_updateStopAtLine()` calls this in
 *       both the SEEK and ARMED phases, and the host test (TC16) drives it directly so the boundary
 *       behaviour is CI-verified.
 */
inline bool stopAtLineSeekMaxedOut(float traveledCm, float maxCm)
{
    return traveledCm >= maxCm;
}

/**
 * @brief Whether a `Drive.stopAtLine()` seek has advanced enough since its last progress mark to
 *        count as making progress (resetting the no-progress watchdog).
 *
 * The stall watchdog faults a seek whose `traveled` stops advancing (zero cruise / jammed wheel).
 * To avoid tripping on float noise or encoder jitter, only an advance of at least `epsCm` since the
 * last recorded progress baseline counts. The comparison is `>=` so an exact `epsCm` step counts.
 *
 * @param traveledCm     Distance travelled so far along the seek direction, in centimetres.
 * @param lastProgressCm Travelled value at the last recorded progress (the watchdog baseline).
 * @param epsCm          Minimum advance (cm) that counts as progress.
 * @return `true` when `(traveledCm - lastProgressCm) >= epsCm`; `false` otherwise.
 *
 * @note Source of truth for the progress-detection DECISION — `Drive::_updateStopAtLine()` calls
 *       this each active tick and, when it returns `true`, advances `_stopLastProgressCm` /
 *       `_stopLastProgressMs`. The host test (TC17) drives it directly so the "slow-but-moving
 *       never stalls" case is CI-verified.
 */
inline bool stopAtLineProgressed(float traveledCm, float lastProgressCm, float epsCm)
{
    return (traveledCm - lastProgressCm) >= epsCm;
}

/**
 * @brief Whether a `Drive.stopAtLine()` seek has gone too long without progress (stall watchdog).
 *
 * Faults `Fault::LINE_NOT_FOUND` once `stallTimeoutMs` has elapsed since the last recorded progress
 * mark — closing the unbounded-async hole where a never-advancing seek (zero cruise / stalled
 * wheel) would hang the FSM forever. The subtraction is computed in unsigned arithmetic so it stays
 * correct across a `millis()` rollover (matching the `_drainBlocking` deadline idiom in Drive.cpp).
 *
 * @param nowMs          Current `millis()` value.
 * @param lastProgressMs `millis()` at the last recorded progress (the watchdog reset point).
 * @param stallTimeoutMs Timeout (ms) without progress before the seek is considered stalled.
 * @return `true` when `(nowMs - lastProgressMs) >= stallTimeoutMs` (rollover-safe); `false`
 *         otherwise.
 *
 * @note Source of truth for the stall-watchdog DECISION — `Drive::_updateStopAtLine()` calls this
 *       each active tick, and the host test (TC17) drives it directly so the stalled-trip and
 *       moving-no-trip cases are both CI-verified.
 */
inline bool stopAtLineStalled(uint32_t nowMs, uint32_t lastProgressMs, uint32_t stallTimeoutMs)
{
    return (uint32_t) (nowMs - lastProgressMs) >= stallTimeoutMs;
}

#endif    // WRO_MATH_H
