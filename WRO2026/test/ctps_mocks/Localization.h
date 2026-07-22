/**
 * @file Localization.h (CTPS HOST MOCK)
 * @brief Recording mock of the Localization singleton for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `Localization.h` (quote-included; force-included by the CTPS
 * cmake wiring). The include guard is the REAL one (`LOCALIZATION_H`) so the real
 * header no-ops if reached afterwards.
 *
 * `Mission.cpp` calls `Localization.declarePosition(id)` from its immediate
 * `declarePosition(name)` recovery verb, and task bodies may call the production
 * `declarePosition(name)` overload directly. This mock is the SINGLE recording
 * seam for that re-anchor: the `uint8_t` overload resolves the id back to a node
 * name via `FieldMap` and appends a `declare_position` op to the shared CTPS log
 * (mirroring how `goto` is recorded by the `PathTracker.goTo` mock only, not by a
 * `Mission.cpp` hook). The `const char*` overload resolves the name to an id and
 * delegates to the funnel, so a caller using either overload is captured exactly
 * once. An out-of-range id or an unknown name records nothing.
 *
 * `toImuYaw` is also shadowed (identity in the CTPS world frame) for task bodies
 * that convert a world heading before a turn. Mirroring the precedent set by the
 * `Drive` / `PathTracker` / `EventTrace` mocks, this keeps the real
 * `Localization.cpp` (and its Drive / MatrixMiniR4 / IMU chain) out of the dump
 * link. If a later change references more of `Localization`, extend this shadow
 * rather than dragging in the real translation unit.
 */

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <stdint.h>

#include "ctps_record.h"
#include "src/nav/FieldMap.h"

/**
 * @brief Recording mock of `LocalizationClass`.
 *
 * Exposes both `declarePosition` overloads the production class does. The
 * `uint8_t` overload is the single recording funnel: it resolves @p nodeId to
 * a node name via `FieldMap::getNode` and appends one `declare_position` op to
 * the shared CTPS log, but only when @p nodeId is in range — an out-of-range
 * id records nothing, matching the production class's silent no-op there.
 */
class LocalizationClass
{
public:
    /**
     * @brief Record a re-anchor to @p nodeId as a `declare_position` op.
     * @param nodeId Target node index. Recorded only when `nodeId < Routes::N_NODES`;
     *               an out-of-range id is a silent no-op, same as production.
     */
    void declarePosition(uint8_t nodeId)
    {
        if (nodeId < Routes::N_NODES)
        {
            ctps::recordDeclarePosition(FieldMap::getNode(nodeId).name);
        }
    }

    /**
     * @brief Resolve @p nodeName and delegate to the `uint8_t` recording funnel.
     * @param nodeName Target node name.
     * @return `true` if the name resolved and the declare was recorded; `false`
     *         on an unknown name (nothing is recorded).
     */
    bool declarePosition(const char* nodeName)
    {
        const uint8_t id = FieldMap::findNodeByName(nodeName);
        if (id == FieldMap::NODE_NONE)
        {
            return false;
        }
        declarePosition(id);
        return true;
    }

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
