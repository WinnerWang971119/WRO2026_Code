/**
 * @file PathTracker.h (CTPS HOST MOCK)
 * @brief Recording mock of the PathTracker FSM facade for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `PathTracker.h` (quote-included; force-included by the T4
 * cmake wiring). The include guard is the REAL one (`PATHTRACKER_H`) so the
 * real header no-ops if reached afterwards.
 *
 * This is the SINGLE seam that records `goto` ops (Mission.cpp's TASK/GOTO hook
 * must NOT also record goto, or it would double-count). The `goTo(goalId)` verb
 * resolves the goal index to a node name via the REAL `FieldMap` / `routes` data
 * (linked, not mocked) and records one `goto` op on builder construction — the
 * call site is a discarded temporary, so recording in the constructor is the
 * chaining-proof seam.
 *
 * All "is-done?" queries report instant completion (`isIdle()` → true,
 * `hasFault()` → false) so Mission's `goTo` step advances on the next tick and
 * the dump never hangs.
 */

#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include <stdint.h>

#include "src/nav/FieldMap.h"     // real — resolve goal index → node name
#include "src/nav/NavTypes.h"     // real — Direction / StopAtLine enums for the chain setters
#include "ctps_record.h"
#include "src/nav/routes.h"       // real — Routes::NODES / N_NODES

class PathTrackerClass;

/**
 * @brief Recording mock of `PathTrackerGoToBuilder`.
 *
 * Records the `goto` op on construction with the resolved node name, then
 * exposes the same move-only `[[nodiscard]]` chain surface as the real builder
 * (all chain setters are no-ops returning `*this`).
 */
class [[nodiscard]] PathTrackerGoToBuilder
{
public:
    /**
     * @brief Construct + record the hop immediately, resolving @p goalId to a name.
     *
     * Out-of-range ids resolve to the empty string via `FieldMap::getNode`'s
     * `noNode()` sentinel (whose `name` is `""`), so a bad goal never reads out
     * of bounds.
     */
    PathTrackerGoToBuilder(PathTrackerClass& /*owner*/, uint8_t goalId)
    {
        ctps::recordGoto(FieldMap::getNode(goalId).name);
    }

    PathTrackerGoToBuilder(const PathTrackerGoToBuilder&)            = delete;
    PathTrackerGoToBuilder& operator=(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder(PathTrackerGoToBuilder&&)                 = delete;
    PathTrackerGoToBuilder& operator=(PathTrackerGoToBuilder&&)      = delete;

    // Chain setters — no-op, return *this so Mission's chain compiles.
    PathTrackerGoToBuilder& async() { return *this; }
    PathTrackerGoToBuilder& setRouteDirection(Direction) { return *this; }
    PathTrackerGoToBuilder& setArrivalDirection(Direction) { return *this; }
    PathTrackerGoToBuilder& setStopAtLine(StopAtLine) { return *this; }
};

/**
 * @brief Recording mock of the PathTracker facade.
 */
class PathTrackerClass
{
public:
    /** @brief Fault enum mirror (always NONE in the dump). */
    enum class Fault : uint8_t
    {
        NONE                = 0,
        NOT_ZEROED          = 1,
        UNREACHABLE         = 2,
        DRIVE_FAULT         = 3,
        BAD_NODE            = 4,
        DRAIN_TIMEOUT       = 5,
        REVERSE_UNAVAILABLE = 6
    };

    /** @brief Captured fault details mirror (always NONE). */
    struct FaultInfo
    {
        Fault   kind     = Fault::NONE;
        uint8_t fromNode = FieldMap::NODE_NONE;
        uint8_t destNode = FieldMap::NODE_NONE;
    };

    /** @brief No-op bring-up; always reports success. */
    bool begin() { return true; }

    /** @brief No-op cooperative tick. */
    void update() {}

    /** @brief No-op tick-callback registration. */
    void setTickCallback(void (*)()) {}

    /** @brief Always idle — Mission's goTo step advances on the next tick. */
    bool isIdle() const { return true; }

    /** @brief Records `goto` (resolving id → name) and returns a recording builder. */
    [[nodiscard]] PathTrackerGoToBuilder goTo(uint8_t goalId)
    {
        return PathTrackerGoToBuilder(*this, goalId);
    }

    /** @brief Name overload — resolve to an id first, then delegate. */
    [[nodiscard]] PathTrackerGoToBuilder goTo(const char* goalName)
    {
        return PathTrackerGoToBuilder(*this, FieldMap::findNodeByName(goalName));
    }

    // --- Fault state — never faulted. ---
    bool      hasFault() const { return false; }
    FaultInfo getFault() const { return FaultInfo{}; }
    void      clearFault() {}

    /** @brief No-op hard-cancel. */
    void abort() {}

    /**
     * @brief Short mnemonic for a Fault enum value (matches the real signature).
     */
    static const char* faultMnemonic(Fault kind)
    {
        switch (kind)
        {
            case Fault::NOT_ZEROED:          return "NZ";
            case Fault::UNREACHABLE:         return "UNR";
            case Fault::DRIVE_FAULT:         return "DRV";
            case Fault::BAD_NODE:            return "BAD";
            case Fault::DRAIN_TIMEOUT:       return "DTO";
            case Fault::REVERSE_UNAVAILABLE: return "REV";
            case Fault::NONE:                return "OK";
        }
        return "OK";
    }
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
