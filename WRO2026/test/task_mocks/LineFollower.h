/**
 * @file LineFollower.h (HOST MOCK)
 * @brief Self-contained no-op `LineFollowerClass` mock for the Tasks host harness.
 *
 * Shadows the production `WRO2026/LineFollower.h` (same `LINEFOLLOWER_H` guard,
 * force-included by the `test_tasks_dispatch` wiring) so a task body that drives
 * the line follower links against this stub instead of the real `LineFollower.cpp`
 * (which binds Drive / WROLineArray / Intersection and the `MiniR4` global).
 *
 * Every verb is a no-op; every query returns a benign default. The `setEnabled`
 * builder mirrors the real immovable (copy- and move-deleted), `[[nodiscard]]`
 * chain surface so a chained
 * `LineFollower.setEnabled(true).tShape().setPreset(...)` compiles unchanged.
 *
 * NOTE: this is the FOLLOWER facade mock. It must NOT be force-included into the
 * targets that compile the real `LineFollower.cpp` (the overshoot / passthrough /
 * bidirectional mirror tests) — those want the real declarations. It is wired
 * only into `test_tasks_dispatch` via `add_task_mock_force_includes`.
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
 * @brief No-op mock of the LineFollower controller facade.
 */
class LineFollowerClass
{
public:
    // --- lifecycle ---
    bool begin() { return true; }
    void update() {}
    void setTickCallback(void (*)()) {}

    // --- fluent enable verb ---
    [[nodiscard]] LineFollowerEnableBuilder setEnabled(bool /*on*/) { return LineFollowerEnableBuilder{}; }

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

// Single global instance — `inline` (force-included only into `test_tasks_dispatch`;
// a shared `mocks.cpp` definition would ODR-clash with the real `LineFollower`
// global that the mirror tests link). See `task_mocks/PathTracker.h` for the rationale.
inline LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
