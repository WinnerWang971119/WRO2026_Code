#ifndef GYRO_DECODE_POLICY_H
#define GYRO_DECODE_POLICY_H

#include <math.h>
#include <stdint.h>

/**
 * @file GyroDecodePolicy.h
 * @brief Single source of truth for the external gyro's two-byte wire decode,
 *        degree wrap, and read-plausibility gate (ADR-070).
 *
 * Six pure free functions answer "what heading did the board just report, and
 * should we trust it":
 *
 *   1. `gyroRawBiased(hiG, loR)` reconstructs the board's signed 16-bit raw
 *      count from its two bytes -- UNWRAPPED, unscaled, un-signed(app-sign).
 *   2. `gyroWrap180(deg)` folds any degree value into the half-open range
 *      `(-180, +180]`.
 *   3. `decodeGyroYawDeg(hiG, loR, scale, sign)` is the full read: raw bytes
 *      -> app-frame heading, built directly on `gyroRawBiased` so production
 *      and the calibration sketch can never decode differently.
 *   4. `gyroBytesDead(hiG, loR)` flags the vendor's dead-bus pattern.
 *   5. `gyroShortestDelta(aDeg, bDeg)` is the shortest signed angular
 *      difference between two headings, itself built on `gyroWrap180`.
 *   6. `gyroReadPlausible(newDeg, lastDeg, elapsedMs, maxRateDegPerS)` is a
 *      time-normalized rate gate that rejects torn reads without latching out
 *      after a legitimate gap in reading.
 *
 * `scale` and `sign` are passed as **parameters**, never read from `Config` --
 * the same reasoning ADR-045 used to make `TurnSignPolicy.h` host-verifiable.
 * This header includes only `<stdint.h>` and `<math.h>`: no `Config.h`, no
 * `<Arduino.h>`, no `<MatrixMiniR4.h>`. That restriction is the whole point --
 * it lets the host test link this with zero mocks and lets the calibration
 * sketch (`inoTest/gyro_calib/`) reuse the exact production math instead of
 * duplicating it.
 *
 * Two rules here are deliberate and non-obvious enough that a future reader
 * could "simplify" them into bugs:
 *
 *   - **Sign is folded into the same multiply as scale, before the wrap**
 *     (`decodeGyroYawDeg`). `sign * gyroWrap180(x)` can emit `-180` (outside
 *     the canonical `(-180, +180]` range) when `gyroWrap180(x) == 180`,
 *     whereas `gyroWrap180(sign * x)` is always canonical. Negating after the
 *     wrap is therefore not an equivalent rewrite.
 *   - **`gyroReadPlausible` is time-normalized**, not a flat per-read bound.
 *     The bound scales with `elapsedMs` because a documented read gap (the
 *     vendor's 100 ms `WaitData` stall, or an operator hand-rotating the
 *     robot) leaves the last-good heading stale; a fixed bound would then
 *     reject every later read forever. Normalizing by elapsed time lets the
 *     gate recover on its own once enough time has passed.
 */

/**
 * @brief Reconstruct the board's biased 16-bit raw count from its two bytes.
 *
 * UNWRAPPED and unscaled -- the calibration sketch accumulates THIS directly
 * across N*360 degrees of turning to derive `Config::GYRO_SCALE`, so it must
 * stay a separate, reusable step rather than being folded only into
 * `decodeGyroYawDeg`.
 *
 * @param hiG High byte of the reading (the board's `G` register).
 * @param loR Low byte of the reading (the board's `R` register).
 * @return Signed biased count in `[-32768, 32767]`, units = degrees.
 */
inline int32_t gyroRawBiased(uint8_t hiG, uint8_t loR)
{
    uint16_t raw16 = (uint16_t) hiG * 256u + (uint16_t) loR;
    return (int32_t) raw16 - 32768;
}

/**
 * @brief Wrap a degree value into the half-open range `(-180, +180]`.
 *
 * Uses C's truncated `fmodf`, which yields a result in `(-360, 360)` with the
 * same sign as `deg`; this then folds that result into the canonical range.
 * Exactly `-180` maps to `+180` (the range is closed at the top, open at the
 * bottom), matching `TurnSignPolicy.h`'s `normalizeTurnDelta` convention.
 *
 * @param deg Any signed degree value, any magnitude.
 * @return The equivalent angle folded into `(-180, +180]`.
 */
inline float gyroWrap180(float deg)
{
    float wrapped = fmodf(deg, 360.0f);
    if (wrapped < -180.0f || wrapped == -180.0f)
    {
        wrapped += 360.0f;
    }
    else if (wrapped > 180.0f)
    {
        wrapped -= 360.0f;
    }
    return wrapped;
}

/**
 * @brief Decode the board's two bytes into an app-frame heading in
 *        `(-180, +180]`.
 *
 * Built directly on `gyroRawBiased()` -- the calibration sketch and
 * production share that one decode, so the two can never drift apart.
 * `sign` is applied in the SAME multiply as `scale`, BEFORE the wrap: see the
 * file-level note above for why that ordering is load-bearing and not a
 * cosmetic simplification.
 *
 * @param hiG   High byte of the reading (the board's `G` register).
 * @param loR   Low byte of the reading (the board's `R` register).
 * @param scale Positive scale calibration (`Config::GYRO_SCALE` at the call
 *              site).
 * @param sign  Exactly `+1.0f` or `-1.0f` (`Config::GYRO_YAW_SIGN` at the call
 *              site).
 * @return App-frame heading in `(-180, +180]`.
 */
inline float decodeGyroYawDeg(uint8_t hiG, uint8_t loR, float scale, float sign)
{
    int32_t biased = gyroRawBiased(hiG, loR);
    float   appDeg = sign * scale * (float) biased;
    return gyroWrap180(appDeg);
}

/**
 * @brief True when both bytes of a reading are zero.
 *
 * The vendor read has no error channel of its own, so an all-zero pair is the
 * only distinguishable "dead bus" signature.
 *
 * @param hiG High byte of the reading (the board's `G` register).
 * @param loR Low byte of the reading (the board's `R` register).
 * @return `true` if `hiG == 0 && loR == 0`.
 *
 * @note This aliases `gyroRawBiased(hiG, loR) == -32768`, a value that is
 *       legitimate in principle (roughly 91 continuous CW turns of
 *       accumulation) but unreachable in a normal run given the hardware
 *       zero applied at `begin()`. This aliasing is accepted as documented
 *       behavior, not a defect to fix.
 */
inline bool gyroBytesDead(uint8_t hiG, uint8_t loR)
{
    return hiG == 0 && loR == 0;
}

/**
 * @brief Shortest signed angular difference `aDeg - bDeg`, in `(-180, +180]`.
 *
 * Thin wrapper over `gyroWrap180` so the wrap logic is not hand-copied here.
 * A legitimate wrap such as `(179, -179)` yields `-2`, not `358`.
 *
 * @param aDeg Minuend heading, in degrees.
 * @param bDeg Subtrahend heading, in degrees.
 * @return `gyroWrap180(aDeg - bDeg)`, in `(-180, +180]`.
 */
inline float gyroShortestDelta(float aDeg, float bDeg)
{
    return gyroWrap180(aDeg - bDeg);
}

/**
 * @brief Time-normalized plausibility gate: rejects a step faster than
 *        `maxRateDegPerS`.
 *
 * The bound is `maxRateDegPerS * elapsedMs / 1000.0f` -- scaled by the time
 * since the last GOOD read, not a flat per-read constant. See the file-level
 * note above for why a fixed bound is wrong: it would latch out permanently
 * after any read gap (a documented 100 ms vendor stall, or an operator
 * hand-rotating the robot), because the last-good heading goes stale and
 * every later read then exceeds a flat bound forever. Normalizing by
 * `elapsedMs` lets the same absolute step become plausible once enough time
 * has actually elapsed.
 *
 * @param newDeg          Newly read heading, in degrees.
 * @param lastDeg         Last known-good heading, in degrees.
 * @param elapsedMs       Milliseconds since the last good read. A caller
 *                        passing `0` is treated as "no bound" (accept) rather
 *                        than dividing by zero.
 * @param maxRateDegPerS  Maximum plausible rate of heading change, in
 *                        degrees per second (`Config::GYRO_MAX_STEP_DEG_PER_S`
 *                        at the call site).
 * @return `true` if the shortest angular step from `lastDeg` to `newDeg` is
 *         within the time-normalized bound (or `elapsedMs == 0`).
 */
inline bool gyroReadPlausible(float newDeg, float lastDeg, uint32_t elapsedMs, float maxRateDegPerS)
{
    if (elapsedMs == 0)
    {
        return true;
    }
    float maxStepDeg    = maxRateDegPerS * (float) elapsedMs / 1000.0f;
    float actualStepDeg = fabsf(gyroShortestDelta(newDeg, lastDeg));
    return actualStepDeg <= maxStepDeg;
}

#endif    // GYRO_DECODE_POLICY_H
