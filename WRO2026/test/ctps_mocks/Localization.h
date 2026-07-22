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
 * `declarePosition(name)` recovery verb, on the success path only (after the
 * PathTracker-busy and unknown-node guards have both passed). This mock is the
 * SINGLE recording seam for that re-anchor: the `uint8_t` overload resolves the
 * id back to a node name via `FieldMap` and appends a `declare_position` op to
 * the shared CTPS log (mirroring how `goto` is recorded by the `PathTracker.goTo`
 * mock only, not by a `Mission.cpp` hook). The `const char*` overload delegates
 * to it after resolving the name to an id, so a caller using either overload is
 * captured exactly once. An out-of-range id or an unknown name records nothing.
 *
 * Mirroring the precedent set by the `Drive` / `PathTracker` / `EventTrace`
 * mocks, this keeps the real `Localization.cpp` (and its Drive / MatrixMiniR4 /
 * IMU chain) out of the dump link.
 *
 * `Mission.cpp` only calls the `uint8_t` overload; the `const char*` overload is
 * a test-only convenience (it lets `test_ctps_dump` drive the name path directly)
 * and does not mirror a production `Localization` method. If a later Mission change
 * references more of `Localization`, extend this shadow rather than dragging in
 * the real translation unit.
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
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern LocalizationClass Localization;

#endif    // LOCALIZATION_H
