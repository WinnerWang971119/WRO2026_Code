/**
 * @file PathTracker.h (HOST MOCK)
 * @brief Self-contained no-op `PathTrackerClass` mock for the Tasks host harness.
 *
 * Shadows the production `WRO2026/PathTracker.h` (same `PATHTRACKER_H` include
 * guard, force-included by the `test_tasks_dispatch` cmake wiring) so a task
 * body that calls `PathTracker.goTo(...)` links against this stub instead of the
 * real facade — whose `goTo` / builder dtor / global live in `PathTracker.cpp`
 * and drag in Localization / FieldMap / routes / Drive / the `MiniR4` global.
 *
 * Unlike the recording `ctps_mocks/PathTracker.h`, this mock is DELIBERATELY
 * self-contained: it resolves no node names (no `FieldMap` / `routes` link) and
 * records nothing. Every verb is a no-op; every state query reports
 * idle / fault-free so a blocking task body advances without a tick loop.
 *
 * The `goTo` builder mirrors the real immovable (copy- and move-deleted),
 * `[[nodiscard]]` chain surface (all setters no-op, returning `*this`) so an
 * arbitrarily chained call
 * (`PathTracker.goTo("N").async().setStopAtLine(...)`) compiles unchanged.
 */

// NOTE: SAME include guard (`PATHTRACKER_H`) as the production header so the
// real one no-ops if reached transitively after this is force-included.
#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include <stdint.h>

#include "src/nav/NavTypes.h"    // real, header-only — Direction / StopAtLine for the chain setters

/**
 * @brief No-op mock of `PathTrackerGoToBuilder`.
 *
 * Non-copyable AND non-movable (both deleted below) and `[[nodiscard]]`, matching
 * the real builder's call-as-temporary contract — `goTo()` returns it by value via
 * guaranteed copy elision (C++17), not a move. Construction and destruction do
 * nothing — there is no queue.
 */
class [[nodiscard]] PathTrackerGoToBuilder
{
public:
    PathTrackerGoToBuilder() = default;

    PathTrackerGoToBuilder(const PathTrackerGoToBuilder&)            = delete;
    PathTrackerGoToBuilder& operator=(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder(PathTrackerGoToBuilder&&)                 = delete;
    PathTrackerGoToBuilder& operator=(PathTrackerGoToBuilder&&)      = delete;

    // Chain setters — no-op, return *this so a chained call compiles.
    PathTrackerGoToBuilder& async() { return *this; }
    PathTrackerGoToBuilder& setRouteDirection(Direction) { return *this; }
    PathTrackerGoToBuilder& setArrivalDirection(Direction) { return *this; }
    PathTrackerGoToBuilder& setStopAtLine(StopAtLine) { return *this; }
};

/**
 * @brief No-op mock of the PathTracker FSM facade.
 */
class PathTrackerClass
{
public:
    /** @brief Fault enum mirror of the real facade (mock is always NONE). */
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

    /** @brief Captured fault details mirror (mock reports no fault). */
    struct FaultInfo
    {
        Fault   kind     = Fault::NONE;
        uint8_t fromNode = 0xFF;    // real uses FieldMap::NODE_NONE; literal here keeps the mock self-contained
        uint8_t destNode = 0xFF;
    };

    // --- lifecycle ---
    bool begin() { return true; }
    void update() {}
    void setTickCallback(void (*)()) {}

    // --- state queries ---
    bool isIdle() const { return true; }

    // --- control verbs ---
    [[nodiscard]] PathTrackerGoToBuilder goTo(uint8_t /*goalId*/) { return PathTrackerGoToBuilder{}; }
    [[nodiscard]] PathTrackerGoToBuilder goTo(const char* /*goalName*/) { return PathTrackerGoToBuilder{}; }

    // --- fault state — never faulted ---
    bool      hasFault() const { return false; }
    FaultInfo getFault() const { return FaultInfo{}; }
    void      clearFault() {}

    // --- hard cancel ---
    void abort() {}

    /** @brief Short mnemonic for a Fault value (matches the real signature). */
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

// Single global instance. `inline` (not `extern` + a mocks.cpp definition) because
// this mock is force-included ONLY into `test_tasks_dispatch`; defining it in the
// shared `mocks.cpp` is impossible without an ODR clash against the real facade
// globals that the sibling mirror tests link. `inline` dedups across the target's TUs.
inline PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
