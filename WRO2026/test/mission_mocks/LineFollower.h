/**
 * @file LineFollower.h (MISSION MOCK — recording variant)
 * @brief Recording no-op `LineFollowerClass` mock for the Mission host harness.
 *
 * Shadows the production `WRO2026/LineFollower.h` (same `LINEFOLLOWER_H` guard,
 * force-included by the `test_mission_dispatch` wiring) so `Mission.cpp` and
 * `Tasks.cpp` link against this stub instead of the real `LineFollower.cpp` (which
 * binds Drive / WROLineArray / Intersection and the `MiniR4` global).
 *
 * Every verb is a no-op; every query returns a benign default. The `setEnabled`
 * builder mirrors the real immovable (copy- and move-deleted), `[[nodiscard]]`
 * chain surface so a chained
 * `LineFollower.setEnabled(true).tShape().setPreset(...)` compiles unchanged.
 *
 * @note Fidelity — records ONLY `setEnabled`; every other verb is a no-op. This
 * is sufficient for `abort()` assertions (TC15) and does not need to model the
 * real LineFollower enable-state FSM.
 *
 * @note This is the MISSION facade mock. It must NOT be force-included into any
 * target that compiles the real `LineFollower.cpp` (the overshoot / passthrough /
 * bidirectional mirror tests). It is wired only into `test_mission_dispatch` via
 * `add_mission_mock_force_includes`.
 */

// NOTE: SAME include guard (`LINEFOLLOWER_H`) as the production header.
#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <stdint.h>

#include "Config.h"    // real — Config::SpeedPreset for the preset setters

class WROLineArrayClass;    // forward decl — getArray() returns a pointer only

/**
 * @brief No-op mock of `LineFollowerEnableBuilder`.
 *
 * Non-copyable AND non-movable (both deleted below) and `[[nodiscard]]` to match
 * the real builder — `setEnabled()` returns it by value via guaranteed copy
 * elision (C++17), not a move. Construction and destruction do nothing (the real
 * dtor applies the enable state at end-of-statement).
 */
class [[nodiscard]] LineFollowerEnableBuilder
{
public:
    LineFollowerEnableBuilder() = default;

    LineFollowerEnableBuilder(const LineFollowerEnableBuilder&)            = delete;
    LineFollowerEnableBuilder& operator=(const LineFollowerEnableBuilder&) = delete;
    LineFollowerEnableBuilder(LineFollowerEnableBuilder&&)                 = delete;
    LineFollowerEnableBuilder& operator=(LineFollowerEnableBuilder&&)      = delete;

    // Chain setters — no-op, return *this.
    LineFollowerEnableBuilder& setBrake(bool) { return *this; }
    LineFollowerEnableBuilder& setPreset(Config::SpeedPreset) { return *this; }
    LineFollowerEnableBuilder& tShape() { return *this; }
    LineFollowerEnableBuilder& setTShapeRampUp(bool) { return *this; }
    LineFollowerEnableBuilder& setTShapeRampDown(bool) { return *this; }
    LineFollowerEnableBuilder& flat() { return *this; }
    LineFollowerEnableBuilder& setAccel(float) { return *this; }
    LineFollowerEnableBuilder& setReverse(bool) { return *this; }
    LineFollowerEnableBuilder& setDistanceTarget(float) { return *this; }
    LineFollowerEnableBuilder& setOvershootMode(bool) { return *this; }
    LineFollowerEnableBuilder& setBrakeOnGate(bool) { return *this; }
};

/**
 * @brief Recording no-op mock of the LineFollower controller facade.
 *
 * @note Recording surface: `lastEnabled` and `setEnabledCount` are set by
 * `setEnabled(bool)`; call `mockReset()` between test cases to zero them.
 */
class LineFollowerClass
{
public:
    // --- recording fields (public; test-asserted directly) ---

    /** @brief Last `on` value passed to `setEnabled`. Default false. */
    bool lastEnabled     = false;
    /** @brief Number of times `setEnabled` has been called. */
    int  setEnabledCount = 0;

    /** @brief Reset all recording fields to their initial values. */
    void mockReset()
    {
        lastEnabled     = false;
        setEnabledCount = 0;
    }

    // --- lifecycle ---
    bool begin() { return true; }
    void update() {}
    void setTickCallback(void (*)()) {}

    // --- fluent enable verb (records then returns the no-op builder) ---

    /**
     * @brief Enable or disable the line follower.
     * @param on  True to enable, false to disable.
     * @return A no-op `LineFollowerEnableBuilder` for chaining.
     * @note Records the call in `lastEnabled` and `setEnabledCount` before
     * returning the builder so `abort()`'s `LineFollower.setEnabled(false)` is
     * assertable (TC15).
     */
    [[nodiscard]] LineFollowerEnableBuilder setEnabled(bool on)
    {
        lastEnabled = on;
        setEnabledCount++;
        return LineFollowerEnableBuilder{};
    }

    // --- direct preset / direction ---
    void                setPreset(Config::SpeedPreset preset) { _preset = preset; }
    Config::SpeedPreset getPreset() const { return _preset; }
    void                setReverse(bool reverse) { _reverse = reverse; }
    bool                getReverse() const { return _reverse; }

    // --- state queries ---
    bool  isEnabled() const { return false; }
    bool  isLineLost() const { return false; }
    bool  isPidFrozen() const { return false; }
    float getRawError() const { return 0.0f; }
    bool  isDistanceReached() { return false; }

    // --- shared sensor handle (never dereferenced in the mock) ---
    WROLineArrayClass* getArray() { return nullptr; }

    // --- sync-blocking helpers (instant success on host) ---
    bool followDistance(float /*cm*/) { return true; }
    bool waitUntilDistanceReached() { return true; }
    bool followUntilIntersection() { return true; }
    bool waitUntilIntersection() { return true; }

private:
    Config::SpeedPreset _preset  = Config::SpeedPreset::NORMAL;
    bool                _reverse = false;
};

// Single global instance — `inline` (force-included only into `test_mission_dispatch`;
// a shared `.cpp` definition would ODR-clash with any other target that links a
// LineFollower global). See `task_mocks/PathTracker.h` for the rationale.
// NOTE: inline global — if a future PR defines LineFollower in hal_mocks.cpp,
// switch to extern.
inline LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
