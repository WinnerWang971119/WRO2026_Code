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
 *  - No `update()` — Localization is a pure state holder + accessor. Heading is read
 *    live from the IMU on every `getHeadingDeg()` call.
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
 * returns the world-frame value via `normalize(IMU_yaw - _yawOffsetDeg)`. The class never
 * caches a heading sample, so there is no possibility of stale state between PathTracker's
 * dispatch tick and the read tick.
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
     * Reads `MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw)`, computes
     * `_yawOffsetDeg = imu_yaw - Routes::START_HEADING_DEG`, and sets `_zeroed = true`.
     * Must be called once on the start-button press AFTER the operator has physically
     * placed the chassis on `Routes::START_NODE` facing the start heading.
     *
     * @return `true` on success. Returns `false` only if a future defensive check (none
     *         today) finds `MiniR4.Motion` un-ready; current implementation always
     *         returns `true`.
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
     * @brief Read the current world-frame heading in degrees.
     *
     * Computes `normalize(MiniR4.Motion.getEuler(Yaw) - _yawOffsetDeg)` to `[0, 360)`.
     * No state is mutated. Returns `Routes::START_HEADING_DEG` if `!isZeroed()`
     * (defensive — caller should check `isZeroed()` before relying on this).
     *
     * @return Heading in `[0, 360)` degrees, CCW positive, `0 deg = +x` (East).
     */
    float getHeadingDeg() const;

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

    uint8_t _currentNode  = 0;        ///< Single source of truth for graph position.
    float   _yawOffsetDeg = 0.0f;     ///< Captured at zeroHeadingFromCurrent(); world = IMU - offset.
    bool    _zeroed       = false;    ///< Latched true on first zeroHeadingFromCurrent() call.
};

/**
 * @brief Single global instance — defined in Localization.cpp.
 */
extern LocalizationClass Localization;

#endif    // LOCALIZATION_H
