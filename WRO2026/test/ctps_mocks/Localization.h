/**
 * @file Localization.h (CTPS HOST MOCK)
 * @brief No-op recording mock of the Localization singleton for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `Localization.h` (quote-included; force-included by the CTPS
 * cmake wiring). The include guard is the REAL one (`LOCALIZATION_H`) so the real
 * header no-ops if reached afterwards.
 *
 * `Mission.cpp` calls `Localization.declarePosition(id)` from its immediate
 * `declarePosition(name)` recovery verb, and task bodies may call the name overload
 * directly. The CTPS dump records the COMMAND STREAM (drive ops / waits / beeps /
 * leds), not localization state, so both re-anchor forms are no-ops here. Mirroring
 * the precedent set by the `Drive` / `PathTracker` / `EventTrace` mocks, this keeps
 * the real `Localization.cpp` (and its Drive / MatrixMiniR4 / IMU chain) out of the
 * dump link.
 *
 * Only the surface `Mission.cpp` touches is provided. If a later Mission change
 * references more of `Localization`, extend this shadow rather than dragging in
 * the real translation unit.
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <stdint.h>

/**
 * @brief No-op recording mock of `LocalizationClass`.
 *
 * Exposes only `declarePosition` — the single method `Mission.cpp` invokes on the
 * global. The dump captures no localization state, so the call does nothing.
 */
class LocalizationClass
{
public:
    /** @brief No-op re-anchor — the dump does not model graph position. */
    void declarePosition(uint8_t /*nodeId*/) {}

    /** @brief No-op name-based re-anchor — the dump does not model graph position. */
    bool declarePosition(const char* nodeName) { return nodeName != nullptr; }

    /**
     * @brief World->IMU yaw conversion — IDENTITY in the CTPS world frame.
     *
     * The real `toImuYaw` maps a world heading into the gyro frame via the live
     * yaw offset. The CTPS dump has no physical gyro and records turns as absolute
     * WORLD headings (`ctps_mocks/Drive.h`'s `TurnToYawBuilder` re-emits the target
     * unchanged), so the correct shadow is the identity: a task body's
     * `Drive.turnToYaw(Localization.toImuYaw(180))` records a turn to WORLD 180.
     */
    int16_t toImuYaw(float worldHeadingDeg) const { return (int16_t) worldHeadingDeg; }
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern LocalizationClass Localization;

#endif    // LOCALIZATION_H
