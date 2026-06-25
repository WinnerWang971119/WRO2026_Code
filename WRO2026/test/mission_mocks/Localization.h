/**
 * @file Localization.h (MISSION MOCK — recording variant)
 * @brief Recording no-op `LocalizationClass` mock for the Mission host harness.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `Localization.h` (quote-included; force-included by the
 * `test_mission_dispatch` cmake wiring). The include guard is the REAL one
 * (`LOCALIZATION_H`) so the real header no-ops if reached afterwards.
 *
 * `Mission.cpp` calls `Localization.declarePosition(id)` from its immediate
 * `declarePosition(name)` recovery verb. This mock records that call so the test
 * can assert the correct node id was passed (TC16/TC18).
 *
 * Only the surface `Mission.cpp` touches is provided. If a later Mission change
 * references more of `Localization`, extend this shadow rather than dragging in
 * the real translation unit (which binds Drive / MatrixMiniR4 / IMU chain).
 *
 * @note Fidelity — records ONLY `declarePosition(uint8_t)`; models no graph
 * position, no yaw offset, and no operator-offset delta. This is sufficient for
 * the `declarePosition` success and fault-clearing assertions.
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <stdint.h>

/**
 * @brief Recording no-op mock of `LocalizationClass`.
 *
 * Exposes only `declarePosition` — the single method `Mission.cpp` invokes on the
 * global. All other Localization state is irrelevant to Mission logic and is not
 * modelled here.
 *
 * @note Recording surface: `lastDeclaredNode` and `declareCount` are set by
 * `declarePosition(uint8_t)`; call `mockReset()` between test cases to zero them.
 */
class LocalizationClass
{
public:
    // --- recording fields (public; test-asserted directly) ---

    /** @brief Last node id passed to `declarePosition`. Default 0xFF (sentinel = not called). */
    uint8_t lastDeclaredNode = 0xFF;
    /** @brief Number of times `declarePosition` has been called. */
    int     declareCount     = 0;

    /** @brief Reset all recording fields to their initial values. */
    void mockReset()
    {
        lastDeclaredNode = 0xFF;
        declareCount     = 0;
    }

    /**
     * @brief Record a re-anchor at the given node.
     * @param nodeId  Resolved node index passed by `Mission::declarePosition(name)`.
     * @note The mock records the call but does not model graph position.
     */
    void declarePosition(uint8_t nodeId)
    {
        lastDeclaredNode = nodeId;
        declareCount++;
    }
};

// Single global instance — `inline` (force-included only into `test_mission_dispatch`;
// no .cpp is needed for this target). If a future PR defines Localization in
// hal_mocks.cpp, switch to extern to avoid a multiply-defined symbol.
inline LocalizationClass Localization;

#endif    // LOCALIZATION_H
