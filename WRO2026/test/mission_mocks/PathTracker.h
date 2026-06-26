/**
 * @file PathTracker.h (HOST MOCK — mission_mocks/)
 * @brief Controllable, recording `PathTrackerClass` mock for the Mission host harness.
 *
 * Shadows the production `WRO2026/PathTracker.h` (same `PATHTRACKER_H` include
 * guard, force-included by the `test_mission_dispatch` cmake wiring) so the REAL
 * `Mission.cpp` links without dragging in `PathTracker.cpp` — whose `goTo` /
 * builder dtor / global live there and pull in Drive / Localization / FieldMap /
 * routes / the `MiniR4` global.
 *
 * Unlike the no-op `task_mocks/PathTracker.h`, this mock is BOTH:
 *   - controllable — the test sets idle and fault state independently
 *     (`mockSetIdle` is orthogonal to `mockSetFault`), so Mission's fault FSM
 *     and the `declarePosition` "busy" guard (`!isIdle() && !hasFault()`) are
 *     reachable; and
 *   - recording — `goTo` / `abort` / `clearFault` calls are captured for the
 *     test to assert on.
 *
 * The `Fault` enum and `faultMnemonic` mirror the REAL production facade EXACTLY
 * (10 values `NONE`..`PIVOT_TRIM_UNDERFLOW`; mnemonics `OK/NZ/UNR/DRV/BAD/DRN/
 * REV/OFN/WSQ/PTU` with a `"??"` post-switch fallback) — NOT the stale 7-value
 * stub + wrong `"DTO"` mnemonic in `task_mocks/` / `ctps_mocks/`. Mission emits
 * the mnemonic in its `NAV FAULT <m>` events, so any drift here would silently
 * assert a fabricated event string.
 *
 * Fidelity — where this mock can lie:
 *   - It does NOT model multi-tick FSM evolution. `isIdle()` / fault state are
 *     test-set knobs, never auto-advanced over `update()` ticks (per the
 *     host-mock-thin standard: record + test-set fields, no behavioral model).
 *   - It resolves no node names — there is no `FieldMap` / `routes` link here.
 *     `goTo(const char*)` therefore CANNOT record a real goal id (it only bumps
 *     the call count; `lastGoToGoal` is left untouched on that overload).
 *   - `mockSetFault`'s `fromNode` / `destNode` are whatever the test passes;
 *     this mock never derives them from a planned route.
 *
 * The `goTo` builder mirrors the real immovable (copy- and move-deleted),
 * `[[nodiscard]]` chain surface so an arbitrarily chained call
 * (`PathTracker.goTo(g).setRouteDirection(...).setArrivalDirection(...)
 * .setStopAtLine(...).async()`) compiles unchanged. Unlike the real builder (which
 * defers its apply to the destructor), it holds a `PathTrackerClass&` back-reference
 * and RECORDS each chain setter's arg into the owner ON CALL (`lastRouteDirection` /
 * `lastArrivalDirection` / `lastStopAtLine`), so a test asserts exactly what Mission
 * forwarded to `PathTracker.goTo(...)` at GOTO execute. `async()` stays a no-op.
 */

// NOTE: SAME include guard (`PATHTRACKER_H`) as the production header so the
// real one no-ops if reached transitively after this is force-included.
#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include <stdint.h>

#include "NavTypes.h"    // real, header-only — Direction / StopAtLine for the chain setters

// Forward declaration so the builder can hold a reference to its owner type.
class PathTrackerClass;

/**
 * @brief Recording mock of `PathTrackerGoToBuilder`.
 *
 * Non-copyable AND non-movable (both deleted below) and `[[nodiscard]]`, matching
 * the real builder's call-as-temporary contract — `goTo()` returns it by value via
 * guaranteed copy elision (C++17), not a move. Unlike the real builder (which defers
 * its apply to the destructor), this mock holds a `PathTrackerClass&` back-reference
 * and RECORDS each chain setter's arg into the owner ON CALL, so a test can assert
 * exactly what Mission forwarded at GOTO execute. `async()` stays a no-op.
 */
class [[nodiscard]] PathTrackerGoToBuilder
{
public:
    /**
     * @brief Construct the builder bound to its owner.
     * @param owner The owning `PathTrackerClass` singleton; outlives this stack temporary
     *              (trivially true for a temporary returned from a singleton method).
     */
    explicit PathTrackerGoToBuilder(PathTrackerClass& owner) : _owner(owner) {}

    PathTrackerGoToBuilder(const PathTrackerGoToBuilder&)            = delete;
    PathTrackerGoToBuilder& operator=(const PathTrackerGoToBuilder&) = delete;
    PathTrackerGoToBuilder(PathTrackerGoToBuilder&&)                 = delete;
    PathTrackerGoToBuilder& operator=(PathTrackerGoToBuilder&&)      = delete;

    /** @brief No-op; returns `*this` so a chained call compiles. */
    PathTrackerGoToBuilder& async() { return *this; }
    /** @brief Record the per-route direction override into the owner. @return `*this`. */
    PathTrackerGoToBuilder& setRouteDirection(Direction d);
    /** @brief Record the final-hop arrival direction override into the owner. @return `*this`. */
    PathTrackerGoToBuilder& setArrivalDirection(Direction d);
    /** @brief Record the call-site stop-at-line override into the owner. @return `*this`. */
    PathTrackerGoToBuilder& setStopAtLine(StopAtLine s);

private:
    PathTrackerClass& _owner;    ///< Owner whose capture fields each setter records into.
};

/**
 * @brief Controllable, recording mock of the PathTracker FSM facade.
 *
 * @note Thin by design — see the file docblock's "Fidelity" list. Idle and fault
 *       are independent test-set knobs; nothing here auto-advances.
 */
class PathTrackerClass
{
public:
    /**
     * @brief Fault enum — mirrors production `PathTracker.h` EXACTLY (10 values).
     * @note  Do NOT shorten to the stale 7-value `task_mocks/` stub; Mission emits
     *        `faultMnemonic(kind)` and the test asserts the exact string.
     */
    enum class Fault : uint8_t
    {
        NONE                 = 0,
        NOT_ZEROED           = 1,
        UNREACHABLE          = 2,
        DRIVE_FAULT          = 3,
        BAD_NODE             = 4,
        DRAIN_TIMEOUT        = 5,
        REVERSE_UNAVAILABLE  = 6,
        OFF_NODE             = 7,
        WALL_NOT_SQUARE      = 8,
        PIVOT_TRIM_UNDERFLOW = 9
    };

    /**
     * @brief Snapshot of the latched fault, returned by `getFault()`.
     * @note  `fromNode` / `destNode` default to `0xFF` (the real facade uses
     *        `FieldMap::NODE_NONE`, which is `0xFF`; the literal keeps this mock
     *        self-contained with no `FieldMap` / `routes` link).
     */
    struct FaultInfo
    {
        Fault   kind     = Fault::NONE;
        uint8_t fromNode = 0xFF;
        uint8_t destNode = 0xFF;
    };

    // --- lifecycle (no-op) ---

    /** @brief No-op lifecycle hook. @return always true. */
    bool begin() { return true; }
    /** @brief No-op — this mock never auto-advances its FSM. */
    void update() {}
    /** @brief No-op; the mock runs no cooperative spins. @param cb ignored. */
    void setTickCallback(void (*cb)()) { (void)cb; }

    // --- control verbs (recording) ---

    /**
     * @brief Mission's call: dispatch a goal by node id.
     * @param goalId The destination node id Mission resolved.
     * @return A no-op chainable builder (immovable, `[[nodiscard]]`).
     * @note  Records the goal: `lastGoToGoal = goalId` and bumps `goToCallCount`.
     */
    [[nodiscard]] PathTrackerGoToBuilder goTo(uint8_t goalId)
    {
        lastGoToGoal = goalId;
        ++goToCallCount;
        return PathTrackerGoToBuilder{*this};
    }

    /**
     * @brief Name overload — REQUIRED so the linked real `Tasks.cpp`
     *        (`testOne` / `test1` bodies call `goTo(const char*)`) compiles.
     * @param goalName Ignored — this mock resolves no node names.
     * @return A no-op chainable builder.
     * @note  Bumps `goToCallCount` only. It CANNOT resolve a name to an id
     *        without `FieldMap`, so it deliberately does NOT touch `lastGoToGoal`.
     */
    [[nodiscard]] PathTrackerGoToBuilder goTo(const char* goalName)
    {
        (void)goalName;
        ++goToCallCount;
        return PathTrackerGoToBuilder{*this};
    }

    // --- state queries ---

    /** @brief @return the test-set idle knob (default true). */
    bool isIdle() const { return _mockIdle; }

    // --- fault state ---

    /** @brief @return true when a non-NONE fault is latched. */
    bool hasFault() const { return _mockFault.kind != Fault::NONE; }
    /** @brief @return the latched fault snapshot. */
    FaultInfo getFault() const { return _mockFault; }
    /** @brief Clear the latched fault. @note Records via `clearFaultCallCount`. */
    void clearFault()
    {
        ++clearFaultCallCount;
        _mockFault = FaultInfo{};
    }

    // --- hard cancel ---

    /**
     * @brief Hard-cancel hook.
     * @note  Records via `abortCallCount` ONLY. It deliberately does NOT also
     *        clear the fault or flip idle — the knobs stay independent so a test
     *        can assert `abortCallCount` without the mock mutating other state.
     */
    void abort() { ++abortCallCount; }

    /**
     * @brief Short mnemonic ("OK", "NZ", "UNR", ...) for a Fault value.
     * @param kind The fault to name.
     * @return The exact production string; `"??"` for any unmapped value.
     * @note  EXHAUSTIVE switch (one case per enum value, no `default` inside it)
     *        so g++ `-Wall -Wextra` / MSVC `/Wswitch` stay quiet — the `"??"`
     *        fallback is a `return` AFTER the switch, matching the real facade.
     *        `static` because Mission calls `PathTrackerClass::faultMnemonic(...)`
     *        via the class name.
     */
    static const char* faultMnemonic(Fault kind)
    {
        switch (kind)
        {
            case Fault::NONE:                 return "OK";
            case Fault::NOT_ZEROED:           return "NZ";
            case Fault::UNREACHABLE:          return "UNR";
            case Fault::DRIVE_FAULT:          return "DRV";
            case Fault::BAD_NODE:             return "BAD";
            case Fault::DRAIN_TIMEOUT:        return "DRN";
            case Fault::REVERSE_UNAVAILABLE:  return "REV";
            case Fault::OFF_NODE:             return "OFN";
            case Fault::WALL_NOT_SQUARE:      return "WSQ";
            case Fault::PIVOT_TRIM_UNDERFLOW: return "PTU";
        }
        return "??";
    }

    // =====================================================================
    // Test surface — control knobs + capture (read these from the test body)
    // =====================================================================

    /**
     * @brief Set the idle knob. INDEPENDENT of fault state.
     * @param v true → `isIdle()` reports idle; false → busy (holds a GOTO step).
     */
    void mockSetIdle(bool v) { _mockIdle = v; }

    /**
     * @brief Latch a fault. INDEPENDENT of the idle knob.
     * @param kind The fault kind to report.
     * @param from From-node id (default 0xFF).
     * @param dest Destination-node id (default 0xFF).
     */
    void mockSetFault(Fault kind, uint8_t from = 0xFF, uint8_t dest = 0xFF)
    {
        _mockFault = FaultInfo{kind, from, dest};
    }

    /** @brief Reset ALL knobs + captures to their defaults (per-test entry). */
    void mockReset()
    {
        _mockIdle           = true;
        _mockFault          = FaultInfo{};
        lastGoToGoal         = 0xFF;
        goToCallCount        = 0;
        abortCallCount       = 0;
        clearFaultCallCount  = 0;
        lastRouteDirection   = Direction::AUTO;
        lastArrivalDirection = Direction::AUTO;
        lastStopAtLine       = StopAtLine::AUTO;
    }

    // --- public capture fields (T3 asserts on these EXACT names) ---

    uint8_t  lastGoToGoal        = 0xFF;    ///< Last id from `goTo(uint8_t)` (not the name form).
    uint16_t goToCallCount       = 0;       ///< Total `goTo(...)` calls (both overloads).
    uint16_t abortCallCount      = 0;       ///< Total `abort()` calls.
    uint16_t clearFaultCallCount = 0;       ///< Total `clearFault()` calls.

    // Builder-forwarded overrides, recorded ON CALL by the chain setters (default AUTO when the
    // GOTO step carried no modifier). These observe what Mission forwarded at GOTO execute.
    Direction  lastRouteDirection   = Direction::AUTO;    ///< Last `setRouteDirection(...)` arg.
    Direction  lastArrivalDirection = Direction::AUTO;    ///< Last `setArrivalDirection(...)` arg.
    StopAtLine lastStopAtLine       = StopAtLine::AUTO;   ///< Last `setStopAtLine(...)` arg.

private:
    bool      _mockIdle  = true;       ///< Test-set idle knob (default idle).
    FaultInfo _mockFault = FaultInfo{}; ///< Test-set latched fault (default NONE).
};

// --- PathTrackerGoToBuilder setter definitions (after the owner is complete) ---

inline PathTrackerGoToBuilder& PathTrackerGoToBuilder::setRouteDirection(Direction d)
{
    _owner.lastRouteDirection = d;
    return *this;
}

inline PathTrackerGoToBuilder& PathTrackerGoToBuilder::setArrivalDirection(Direction d)
{
    _owner.lastArrivalDirection = d;
    return *this;
}

inline PathTrackerGoToBuilder& PathTrackerGoToBuilder::setStopAtLine(StopAtLine s)
{
    _owner.lastStopAtLine = s;
    return *this;
}

// Single global instance. `inline` (not `extern` + a .cpp definition) because this
// mock is force-included ONLY into `test_mission_dispatch`; `inline` dedups across
// the target's TUs without a separate translation unit.
//
// inline global — if a future PR defines PathTracker in hal_mocks.cpp, switch this
// to extern to avoid a multiply-defined symbol.
inline PathTrackerClass PathTracker;

#endif    // PATHTRACKER_H
