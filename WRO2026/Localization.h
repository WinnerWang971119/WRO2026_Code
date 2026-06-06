/**
 * @file Localization.h
 * @brief Single source of truth for `currentNode` and the world-frame yaw offset.
 * @author WRO 2026
 *
 * @details
 * `LocalizationClass` is the singleton that holds the robot's discrete position on the
 * field graph (`currentNode`, an id into `Routes::NODES`) and the runtime offset that
 * bridges Routes:: world-frame headings (CCW positive, `0 deg = +x` East, range
 * `[0, 360)`) to the IMU's local yaw frame.
 *
 * Single-writer rule (per ARCHITECTURE.md):
 *  - `_currentNode` is written ONLY by this class. PathTracker calls `onIntersection(...)`
 *    or `onDeadReckonedAdvance(...)` to advance the node id after a successful edge.
 *  - `_yawOffsetDeg` is written ONLY by `zeroHeadingFromCurrent()`. PathTracker NEVER
 *    writes heading state — every turn dispatches `Drive.turnToYaw(...)` against the
 *    IMU as the source of truth, so cumulative-relative drift is never accumulated.
 *
 * Operator-relative reference frame (key convention):
 *  - The yaw offset is captured at the **start-button press**, not at `begin()`. The
 *    chassis is rarely at the right pose when `MiniR4.begin()` runs, so a `begin()`-time
 *    snapshot would bind the wrong frame. Power-on flow:
 *      1. `setup()` runs `Localization.begin()` -> `_zeroed = false`.
 *      2. Operator places the robot on `Routes::START_NODE` facing `START_HEADING_DEG`.
 *      3. Operator presses BTN_UP -> `.ino` calls `zeroHeadingFromCurrent()` -> reads the
 *         IMU yaw NOW, computes the offset, sets `_zeroed = true`.
 *      4. Subsequent goTo() calls succeed; goTo() before zero faults loud.
 *
 * Lifecycle:
 *  - `begin()` once in `setup()` after every `MiniR4.begin()` and the Phase 1 modules'
 *    `begin()`. The order is `Drive` -> `LineFollower` -> `Intersection` -> `Manipulator`
 *    -> `Localization` -> `PathTracker` -> `EventTrace`.
 *  - No `update()` — Localization is a pure state holder + accessor. `getHeadingDeg()`
 *    reads the IMU live; on a transient lower-MCU read failure it falls back to the last
 *    good heading instead of a `0.0`-derived garbage value (see `getHeadingDeg`).
 *  - `getLastHeadingDeg()` returns that cached value with NO I/O — use it on telemetry /
 *    non-blocking paths that must not issue a bus transaction.
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <Arduino.h>
#include <stdint.h>

#include "FieldMap.h"
#include "routes.h"

/**
 * @brief Singleton that holds the discrete current node + IMU-frame yaw offset.
 *
 * Heading is treated as a derived quantity — `getHeadingDeg()` reads the IMU live and
 * returns the world-frame value via `normalize(IMU_yaw - _yawOffsetDeg)`. Every OK read
 * also refreshes `_lastGoodHeadingDeg`; on a non-OK lower-MCU read `getHeadingDeg()`
 * returns that cached value rather than a `0.0`-derived garbage heading, and
 * `getLastHeadingDeg()` exposes the same cache for callers that must not issue I/O.
 */
class LocalizationClass
{
public:
    /**
     * @brief Seed `_currentNode = Routes::START_NODE`; clear `_zeroed` and offset.
     *
     * Must be called from `setup()` after every Phase 1 module's `begin()`. Does NOT
     * touch the IMU — `_yawOffsetDeg` stays `0.0f` until `zeroHeadingFromCurrent()` runs.
     *
     * @return `true` always (defensive — kept for symmetry with the other modules' bool
     *         return convention so `.ino` can collect a single `_setupFailed` flag).
     */
    bool begin();

    /**
     * @brief Capture the IMU yaw NOW and bind the world-frame offset.
     *
     * Reads the IMU yaw through the result-surfacing lower-MCU call (`mmL.GetIMUEuler`). On
     * an OK read computes `_yawOffsetDeg = imu_yaw - Routes::START_HEADING_DEG` and sets
     * `_zeroed = true`. Must be called once on the start-button press AFTER the operator has
     * physically placed the chassis on `Routes::START_NODE` facing the start heading.
     *
     * @return `true` on a successful read. Returns `false` on a non-OK IMU read (transient
     *         STM32 comms timeout) WITHOUT touching `_yawOffsetDeg` or `_zeroed`, so the
     *         `.ino` BTN_UP handler's retry path engages instead of latching a corrupt
     *         world-frame offset.
     * @note Calling this a second time is allowed — it re-binds the offset to whatever
     *       the chassis is currently pointing at. Useful for re-zeroing during testing
     *       between bench runs.
     */
    bool zeroHeadingFromCurrent();

    /**
     * @brief Whether `zeroHeadingFromCurrent()` has been called at least once.
     *
     * @return `true` once the operator has zero'd; `false` otherwise.
     * @note `PathTrackerClass::goTo(...)`'s builder destructor checks this and faults
     *       `Fault::NOT_ZEROED` on a `goTo` issued before zeroing.
     */
    bool isZeroed() const;

    /**
     * @brief Read the current node id (single source of truth).
     *
     * @return Node id in `[0, FieldMap::N_NODES)`. Initialised to `Routes::START_NODE`
     *         by `begin()`.
     */
    uint8_t getCurrentNode() const;

    /**
     * @brief Read the current world-frame heading in degrees (live IMU read).
     *
     * Issues a single lower-MCU IMU read (`mmL.GetIMUEuler`). On an OK read computes
     * `normalize(imu_yaw - _yawOffsetDeg)` to `[0, 360)` and refreshes the last-good cache.
     * On a NON-OK read (transient STM32 comms timeout) the underlying out-param is left
     * untouched by the library, so this method returns the LAST GOOD heading instead of a
     * `0.0`-derived garbage value. Returns `Routes::START_HEADING_DEG` if `!isZeroed()`
     * (defensive — caller should check `isZeroed()` before relying on this).
     *
     * @return Heading in `[0, 360)` degrees, CCW positive, `0 deg = +x` (East).
     * @note Performs I/O. Telemetry / non-blocking paths must use `getLastHeadingDeg()`.
     */
    float getHeadingDeg() const;

    /**
     * @brief Read the cached world-frame heading WITHOUT issuing any I/O.
     *
     * Returns the value refreshed by the most recent successful `getHeadingDeg()` read, or
     * `Routes::START_HEADING_DEG` if no successful read has occurred yet. This accessor
     * never touches the IMU / I2C bus, so it is safe on paths whose contract forbids
     * blocking on hardware (e.g. the EventTrace heartbeat).
     *
     * @return Last good heading in `[0, 360)` degrees, CCW positive, `0 deg = +x` (East).
     */
    float getLastHeadingDeg() const;

    /**
     * @brief `millis()` of the last successful live heading read; `0` if none yet.
     *
     * Lets a no-I/O consumer distinguish a freshly-read heading from a stale cached one.
     * `getLastHeadingDeg()` keeps returning a plausible value even when the IMU has been
     * failing or no navigation code has read the heading recently, so a telemetry path
     * (e.g. the EventTrace heartbeat) compares `millis() - getLastHeadingUpdateMs()`
     * against its own staleness budget to flag the difference.
     *
     * @return `millis()` captured at the last OK `getHeadingDeg()` read; `0` if no OK read
     *         has happened since `begin()` (which resets it).
     */
    uint32_t getLastHeadingUpdateMs() const;

    /**
     * @brief Convert a world-frame heading to the IMU-frame yaw value `Drive.turnToYaw`
     *        expects.
     *
     * Result = `normalize(worldHeadingDeg + _yawOffsetDeg)` rounded to `int16_t`. The
     * `Drive` layer further normalises to `(-360, +360)` via `_normalizeYaw` before
     * dispatching to the lower MCU.
     *
     * @param worldHeadingDeg  World-frame heading in `[0, 360)`. Out-of-range values are
     *                         normalised before the offset is applied.
     * @return IMU-frame yaw target as an `int16_t`.
     */
    int16_t toImuYaw(float worldHeadingDeg) const;

    /**
     * @brief Edge-arrival hook called by PathTracker after a LINE edge intersection event.
     *
     * @param advanceTo  Node id to set as the new `_currentNode`.
     * @note Does not validate @p advanceTo — caller (PathTracker) has just resolved it
     *       from `FieldMap::getNextHop(...)` so it is by construction a valid node id.
     */
    void onIntersection(uint8_t advanceTo);

    /**
     * @brief Edge-arrival hook called by PathTracker after a PROJECTION or BLIND edge.
     *
     * @param advanceTo  Node id to set as the new `_currentNode`.
     * @note Semantically distinct from `onIntersection(...)` because no physical
     *       re-localisation event fired — this is dead-reckoned advance. Future
     *       calibration nodes (Phase 2.5) may differentiate behaviour here.
     */
    void onDeadReckonedAdvance(uint8_t advanceTo);

private:
    /**
     * @brief Wrap a degree value into `[0, 360)`.
     *
     * Handles negative inputs (e.g. raw IMU yaw can be `-180`) by adding `360.0` until
     * the value lands in range. Returns `fmodf(deg + 360, 360)` semantics without the
     * IEEE float subtleties of `fmodf`.
     *
     * @param deg  Any finite float.
     * @return Equivalent value in `[0, 360)`.
     */
    static float _normalize(float deg);

    /**
     * @brief Read the raw IMU yaw through the result-surfacing lower-MCU call.
     *
     * Wraps `mmL.GetIMUEuler(roll, pitch, yaw)` (which `MiniR4Motion::getEuler` calls but
     * whose `MMLower::RESULT` it discards) so callers can distinguish a real read from a
     * silent comms timeout. On a non-OK read `yawOut` is left untouched — the library does
     * not write the out-params on the timeout / read-error paths.
     *
     * @param yawOut  Set to the raw IMU yaw in degrees ONLY when the read returns OK.
     * @return `true` if the lower-MCU read returned `MMLower::RESULT::OK`; `false` otherwise.
     */
    static bool _readImuYaw(float& yawOut);

    uint8_t _currentNode  = 0;        ///< Single source of truth for graph position.
    float   _yawOffsetDeg = 0.0f;     ///< Captured at zeroHeadingFromCurrent(); world = IMU - offset.
    bool    _zeroed       = false;    ///< Latched true on first zeroHeadingFromCurrent() call.

    /**
     * @brief World-frame heading from the last successful `getHeadingDeg()` read.
     *
     * Seeded to `Routes::START_HEADING_DEG` so a read before the first OK IMU sample still
     * returns a sane value. Refreshed on every OK read; served verbatim (no I/O) by
     * `getLastHeadingDeg()` and returned by `getHeadingDeg()` when a read fails. `begin()`
     * re-seeds it so a re-zero between bench runs does not serve the prior run's heading.
     */
    mutable float _lastGoodHeadingDeg = (float) Routes::START_HEADING_DEG;

    /// `millis()` of the last OK `getHeadingDeg()` read; `0` = none since `begin()`.
    mutable uint32_t _lastGoodHeadingMs = 0;
};

/**
 * @brief Single global instance — defined in Localization.cpp.
 */
extern LocalizationClass Localization;

#endif    // LOCALIZATION_H
