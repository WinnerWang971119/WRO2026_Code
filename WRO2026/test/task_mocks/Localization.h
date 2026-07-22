/**
 * @file Localization.h (TASK MOCK)
 * @brief Thin `LocalizationClass` shadow for the `test_tasks_dispatch` harness.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `Localization.h` (quote-included; force-included FIRST by
 * `add_task_mock_force_includes`). The include guard is the REAL one
 * (`LOCALIZATION_H`) so the real header no-ops if reached afterwards, keeping the
 * real `Localization.cpp` (and its Drive / MatrixMiniR4 / IMU chain) out of the
 * `test_tasks_dispatch` link — the same trick as the sibling PathTracker / Drive /
 * Manipulator task mocks.
 *
 * Only the surface a `Tasks.cpp` task body touches is provided — currently just
 * `toImuYaw`. If a later task references more of `Localization`, extend this shadow
 * rather than dragging in the real translation unit.
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <stdint.h>

/**
 * @brief Thin no-op mock of `LocalizationClass`.
 */
class LocalizationClass
{
public:
    /**
     * @brief World->IMU yaw conversion — identity shadow.
     *
     * Heading math is not modelled (the tasks-dispatch tests assert registry /
     * dispatch behaviour, not heading values). Present only so a task body that
     * calls `Localization.toImuYaw(...)` links here instead of pulling in the real
     * translation unit.
     */
    int16_t toImuYaw(float worldHeadingDeg) const { return (int16_t) worldHeadingDeg; }

    /**
     * @brief Snap the current node — no-op shadow (numeric id overload).
     *
     * A task body may declare a known position mid-routine (ADR-045 / #189). The
     * tasks-dispatch tests assert registry / dispatch behaviour, not localization
     * state, so this shadow only needs to exist for the link.
     */
    void declarePosition(uint8_t /*nodeId*/) {}

    /**
     * @brief Snap the current node by name — no-op shadow (string overload).
     *
     * Mirrors the real `bool declarePosition(const char*)`; the return value is
     * unused by the tasks-dispatch tests.
     */
    bool declarePosition(const char* /*nodeName*/) { return true; }
};

/** @brief Single global instance — `inline` so no extra TU is linked. */
inline LocalizationClass Localization;

#endif    // LOCALIZATION_H
