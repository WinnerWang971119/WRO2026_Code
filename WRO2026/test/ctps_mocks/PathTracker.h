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
 * Records the `goto` op from an `_executed`-guarded DESTRUCTOR (the same seam
 * the Drive / LineFollower recording mocks use), NOT the constructor: recording
 * at destruction is what lets the chained direction setters
 * (`.setArrivalDirection(...)` / `.setRouteDirection(...)`) run and be captured
 * before the single emit. `goto` is authored one-per-statement, so the discarded
 * temporary destructs at the `;` in authored order. The `async` /
 * `setStopAtLine` setters stay no-ops (the sim has no async / call-site
 * stop-at-line surface); the direction setters store into the builder.
 */
class [[nodiscard]] PathTrackerGoToBuilder
{
public:
    /**
     * @brief Capture the goal id; the `goto` op is recorded at destruction.
     *
     * The name is resolved (and the record emitted) in the destructor. Out-of-range
     * ids resolve to the empty string via `FieldMap::getNode`'s `noNode()` sentinel
     * (whose `name` is `""`), so a bad goal never reads out of bounds.
     */
    PathTrackerGoToBuilder(PathTrackerClass& /*owner*/, uint8_t goalId) : _goalId(goalId) {}

    /**
     * @brief Emit the single `goto` record, serializing any forced arrival/route
     *        direction as a "front"/"rear" tag ("" == AUTO, suppressed by the
     *        serializer). The `_executed` sentinel keeps the single-emit invariant.
     */
    ~PathTrackerGoToBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;
        ctps::recordGoto(FieldMap::getNode(_goalId).name, _dirTag(_arrival), _dirTag(_route));
    }

    PathTrackerGoToBuilder(const PathTrackerGoToBuilder&)            = delete;
    PathTrackerGoToBuilder& operator=(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder(PathTrackerGoToBuilder&&)                 = delete;
    PathTrackerGoToBuilder& operator=(PathTrackerGoToBuilder&&)      = delete;

    // Chain setters — the direction knobs are captured (recorded at destruction);
    // the rest stay no-ops returning *this so Mission's chain compiles.
    PathTrackerGoToBuilder& async() { return *this; }
    PathTrackerGoToBuilder& setRouteDirection(Direction d)
    {
        _route = d;
        return *this;
    }
    PathTrackerGoToBuilder& setArrivalDirection(Direction d)
    {
        _arrival = d;
        return *this;
    }
    PathTrackerGoToBuilder& setStopAtLine(StopAtLine) { return *this; }

private:
    /** @brief "front"/"rear" for a forced direction; "" for AUTO (serializer omits it). */
    static const char* _dirTag(Direction d)
    {
        if (d == Direction::FRONT)
        {
            return "front";
        }
        if (d == Direction::REAR)
        {
            return "rear";
        }
        return "";
    }

    uint8_t   _goalId;
    Direction _arrival  = Direction::AUTO;
    Direction _route    = Direction::AUTO;
    bool      _executed = false;
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
