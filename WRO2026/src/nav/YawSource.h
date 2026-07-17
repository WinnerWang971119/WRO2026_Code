/**
 * @file YawSource.h
 * @brief Sole adapter between the external gyro's raw heading and the app world frame, and the
 *        app->STM32 heading converter for the unchanged gyro-PID write path.
 * @author WRO 2026
 *
 * @details
 * `YawSourceClass` is the single owner of the robot's yaw READ. The heading is now sourced from
 * an external MXColor-protocol gyro board (I2C addr 0x22 on `Wire1`) rather than the on-board
 * STM32 IMU (ADR-070). That board packs a signed 16-bit heading into two colour registers;
 * `YawSource` owns the I2C transport, a tick-scoped cache, and a per-read dead-byte gate, and
 * hands every consumer an app-frame (CCW-positive), wrapped-to-`(-180, 180]` heading through the
 * UNCHANGED `readYawDeg` / `readYawDegOr` contract. Reads are gated ONLY by dead-byte detection
 * (an all-zero pair = dead bus); a plausibility gate — a torn-read guard the known-good reference
 * omits — is deferred to bench verification.
 *
 * Two PHYSICALLY INDEPENDENT frame-crossing signs live at this boundary, each applied exactly
 * once (ADR-070, extending ADR-045's one-place rule to per-frame):
 *
 *  - **external gyro -> app** (`readYawDeg` / `readYawDegOr`): the read path decodes the raw
 *    bytes via `GyroDecodePolicy.h::decodeGyroYawDeg`, which folds `Config::GYRO_YAW_SIGN` into
 *    the same multiply as the scale, before the wrap. That is the SOLE application of
 *    `GYRO_YAW_SIGN`.
 *  - **app -> STM32 raw** (`toHwHeading`): still `appHeading * Config::IMU_YAW_SIGN`, used to
 *    convert an app-frame target into the raw-frame heading a `MoveGyro*` gyro-PID command
 *    expects. UNCHANGED — the STM32 IMU still backs the move/write path. That is the SOLE
 *    application of `IMU_YAW_SIGN`.
 *
 * The gyro and the STM32 coprocessor are two separate devices with independent handedness, so
 * `GYRO_YAW_SIGN` (read) and `IMU_YAW_SIGN` (write) are distinct constants — never collapse
 * them, and never apply either sign a SECOND time anywhere. A double application double-negates
 * the conversion and reverses all heading math (the exact bug ADR-045 exists to prevent).
 *
 * Single-owner rule: no other module reads the gyro board or applies either sign. Every
 * consumer works exclusively in the app CCW-positive frame and routes its yaw I/O through this
 * class.
 *
 * This header is intentionally LIGHT — it pulls in only `<stdint.h>` so dependents (`Drive.h`,
 * `Localization.cpp`) can include it cheaply without dragging in MatrixMiniR4 or `Config.h`. The
 * `MatrixColor` device instance, the cache, and the health flag all live as file-statics in
 * `YawSource.cpp`.
 */

#ifndef YAW_SOURCE_H
#define YAW_SOURCE_H

#include <stdint.h>

/**
 * @brief Sole adapter over the external gyro yaw read (applies `Config::GYRO_YAW_SIGN` via the
 *        decode policy) plus the app->STM32 write converter (applies `Config::IMU_YAW_SIGN`).
 *
 * @note Single global instance `YawSource` (declared `extern` below, defined in the .cpp).
 *       Construct nothing else; all consumers share this instance.
 * @note No other module performs gyro yaw I/O or applies either yaw sign — see the file header
 *       and ADR-070 / ADR-045.
 */
class YawSourceClass
{
public:
    /**
     * @brief Wire the gyro board onto its I2C bus + mux channel, run a liveness pre-flight, and
     *        seed the cache. Call ONCE in `setup()` before any yaw consumer begins.
     *
     * Sets the `MatrixColor` transport (`Wire1`, mux channel `Config::GYRO_I2C_PORT - 1` — a
     * port LABEL is not a mux channel) and runs a TWO-PATH pre-flight: the vendor
     * `MatrixColor::begin()` is attempted but its result is treated as INCONCLUSIVE, and the
     * AUTHORITATIVE liveness check is a single `getColor(G)`+`getColor(R)` pair — `begin()` fails
     * only when that pair is all-zero (a dead bus / no device answering on the channel, AC-6). On
     * success it zeroes the hardware once (`zeroHardware()`) and seeds the cache from a first read.
     *
     * @return `true` if a device answered with a non-all-zero heading pair; `false` on a dead bus.
     * @note Whether the vendor `MatrixColor::begin()` / its CONFIG-reset is SAFE on this specific
     *       gyro board is UNVERIFIED pending bench — the mentor's flow never calls it, and it
     *       ID-checks `0x43` and writes a reset the board may not implement. This is exactly why
     *       `begin()` does NOT hard-depend on the vendor result and falls through to a raw read.
     */
    bool begin();

    /**
     * @brief Refresh the tick-scoped yaw cache with one gated device read + `millis()` timestamp.
     *
     * Intended to run FIRST in the cooperative tick so every consumer that calls `readYawDeg`
     * later in the same tick serves the freshly-cached value from ONE device transaction (AC-5).
     * `update()` is the sole STEADY-STATE writer of the cache (`begin()` seeds it once at boot;
     * `readYawDeg` never writes it): a gated-out read leaves the prior good value in place until it
     * ages past `Config::GYRO_CACHE_MAX_AGE_MS`, so a transient miss never overwrites a good cache
     * with a rejected/garbage sample.
     */
    void update();

    /**
     * @brief Trigger the gyro board's hardware zero (a `getColor(K)` read-with-side-effect).
     *
     * Register `K` (Device_BLACK, 10) zeroes the board's accumulated heading; the returned byte
     * carries no data and is discarded. Called once by `begin()`, and public so the calibration
     * sketch / menu can re-zero on demand.
     *
     * @note `Localization`'s software offset remains the app-level zero — this is the DEVICE zero,
     *       a separate concern.
     */
    void zeroHardware();

    /**
     * @brief Whether the most recent device read passed the dead-byte gate (was not all-zero).
     *
     * @return `false` after a dead (all-zero) read, `true` after the next non-dead read. Serving
     *         a fresh cached value does not change this — it reflects the last DEVICE read's
     *         result.
     */
    bool isHealthy() const;

    /**
     * @brief Read the corrected (app-frame) gyro yaw, serving the tick cache when it is fresh.
     *
     * Serves the `update()`-populated cache when it is present and no older than
     * `Config::GYRO_CACHE_MAX_AGE_MS`; otherwise refreshes INLINE with one gated device read
     * (`getColor(G)` high byte, then `getColor(R)` low byte, back-to-back — nothing may touch the
     * bus between them) decoded via `GyroDecodePolicy.h::decodeGyroYawDeg`. The read is gated
     * ONLY by dead-byte detection (a plausibility gate is deferred to bench verification). The
     * inline refresh deliberately does NOT persist to the cache — `update()` is the sole cache
     * writer — so a consumer that polls without a driving `update()` (or under a frozen clock)
     * always observes a live value rather than one pinned by its own first read.
     *
     * @param[out] yawOut Corrected app-frame yaw in degrees, wrapped to `(-180, 180]`. Left
     *                    UNTOUCHED on failure — the caller's prior value is preserved so a
     *                    transient miss never injects a `0.0`-derived garbage heading.
     * @return `true` on a served cache hit or an accepted device read (with @p yawOut written);
     *         `false` on a dead-bus read (with @p yawOut untouched).
     * @note `const` — it serves / refreshes file-static cache and health state, not instance
     *       members, so the signature the 32 host targets and both consumers bind to is unchanged.
     */
    bool readYawDeg(double& yawOut) const;

    /**
     * @brief Fire-and-forget convenience wrapper around `readYawDeg`.
     *
     * @param fallback Value returned when the read fails (dead bus).
     * @return The corrected app-frame yaw (degrees) on success, else @p fallback.
     * @note Use on non-critical / telemetry paths where a transient miss should degrade to a
     *       sensible default rather than force the caller to branch on a `bool`.
     */
    double readYawDegOr(double fallback) const;

    /**
     * @brief Convert an app-frame heading into the STM32 raw-frame heading for a `MoveGyro*`
     *        gyro-PID target.
     *
     * A pure sign flip: app CCW-positive heading -> STM32 raw CW-positive gyro-PID target. The
     * app and raw frames differ by sign alone (the `Localization` offset is already folded into
     * the app-frame value reaching here), so there is no offset term.
     *
     * @param appHeadingDeg App-frame (CCW-positive) heading in degrees.
     * @return The raw-frame (CW-positive) heading: `IMU_YAW_SIGN * appHeadingDeg`, computed as a
     *         float multiply before the cast back to `int16_t`.
     * @note `static` — depends only on `Config::IMU_YAW_SIGN`, not on instance state. This is the
     *       UNCHANGED app->STM32 write path (the gyro read migration, ADR-070, does not touch it);
     *       replacing this write path is tracked separately (issue #169).
     */
    static int16_t toHwHeading(int16_t appHeadingDeg);
};

/// Single global instance — the sole owner of gyro yaw I/O (defined in the .cpp).
extern YawSourceClass YawSource;

#endif    // YAW_SOURCE_H
