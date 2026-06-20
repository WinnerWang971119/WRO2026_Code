/**
 * @file YawSource.h
 * @brief Sole adapter between the lower-MCU raw IMU yaw and the app world frame.
 * @author WRO 2026
 *
 * @details
 * `YawSourceClass` is the single owner of all raw lower-MCU IMU yaw I/O. The STM32
 * coprocessor reports yaw in a **clockwise-positive** convention; the application world
 * frame (PathTracker, `Drive.turnToYaw`, `Localization`, and every authored field
 * heading) is **counter-clockwise-positive** (standard mathematical). `YawSource` bridges
 * the two frames by applying the per-chassis `Config::IMU_YAW_SIGN` at exactly two points:
 *
 *  - **STM32 -> app** (`readYawDeg` / `readYawDegOr`): `appYaw = rawYaw * IMU_YAW_SIGN`.
 *  - **app -> STM32** (`toHwHeading`): `hwHeading = appHeading * IMU_YAW_SIGN`, used to
 *    convert an app-frame target into the raw-frame heading a `MoveGyro*` gyro-PID
 *    command expects.
 *
 * Single-owner rule (per ADR-044): no other module reads `mmL.GetIMUEuler` or applies
 * `Config::IMU_YAW_SIGN`. Every consumer works exclusively in the app CCW-positive frame
 * and routes its raw yaw I/O through this class. A second application of the sign anywhere
 * would double-negate the conversion and reverse all heading math.
 *
 * This header is intentionally LIGHT — it pulls in only `<stdint.h>` so dependents
 * (`Drive.h`, `Localization.cpp`) can include it cheaply without dragging in MatrixMiniR4
 * or `Config.h`. The lower-MCU and Config dependencies live entirely in `YawSource.cpp`.
 */

#ifndef YAW_SOURCE_H
#define YAW_SOURCE_H

#include <stdint.h>

/**
 * @brief Sole adapter over the lower-MCU IMU yaw — applies `Config::IMU_YAW_SIGN` to
 *        bridge the hardware CW-positive raw frame and the app CCW-positive world frame.
 *
 * @note Single global instance `YawSource` (declared `extern` below, defined in the .cpp).
 *       Construct nothing else; all consumers share this instance.
 * @note No other module performs raw IMU yaw I/O or applies the yaw sign — see the file
 *       header and ADR-044.
 */
class YawSourceClass
{
public:
    /**
     * @brief Read the corrected (app-frame) IMU yaw, surfacing the lower-MCU result.
     *
     * Reads yaw via `mmL.GetIMUEuler` (retry-once after a short backoff on a transient
     * non-OK result, per the MMLower-caller convention), then multiplies by
     * `Config::IMU_YAW_SIGN` to convert the raw CW-positive yaw into the app CCW-positive
     * frame. The corrected yaw is written to @p yawOut **only** on a successful read.
     *
     * @param[out] yawOut Corrected app-frame yaw in degrees. Left UNTOUCHED on failure —
     *                    the caller's prior value is preserved so a transient miss never
     *                    injects a `0.0`-derived garbage heading.
     * @return `true` if the lower-MCU read returned OK (possibly after one retry);
     *         `false` on a hard failure (both attempts non-OK), with @p yawOut untouched.
     * @note The corrected value is computed in floating point BEFORE any int cast so
     *       a caller that needs an integer can cast exactly once with no intermediate
     *       truncation.
     */
    bool readYawDeg(double& yawOut) const;

    /**
     * @brief Fire-and-forget convenience wrapper around `readYawDeg`.
     *
     * @param fallback Value returned when the read fails (both attempts non-OK).
     * @return The corrected app-frame yaw (degrees) on success, else @p fallback.
     * @note Use on non-critical / telemetry paths where a transient miss should degrade
     *       to a sensible default rather than force the caller to branch on a `bool`.
     */
    double readYawDegOr(double fallback) const;

    /**
     * @brief Convert an app-frame heading into the STM32 raw-frame heading for a
     *        `MoveGyro*` gyro-PID target.
     *
     * A pure sign flip: app CCW-positive heading -> STM32 raw CW-positive gyro-PID target.
     * The app and raw frames differ by sign alone (the `Localization` offset is already
     * folded into the app-frame value reaching here), so there is no offset term.
     *
     * @param appHeadingDeg App-frame (CCW-positive) heading in degrees.
     * @return The raw-frame (CW-positive) heading: `IMU_YAW_SIGN * appHeadingDeg`,
     *         computed as a float multiply before the cast back to `int16_t`.
     * @note `static` — depends only on `Config::IMU_YAW_SIGN`, not on instance state.
     */
    static int16_t toHwHeading(int16_t appHeadingDeg);
};

/// Single global instance — the sole owner of lower-MCU IMU yaw I/O (defined in the .cpp).
extern YawSourceClass YawSource;

#endif    // YAW_SOURCE_H
