/**
 * @file LineFollower.h (PATH HOST MOCK — recording variant)
 * @brief Recording `LineFollowerClass` mock for the PathTracker + Drive host-link targets
 *        (issue #106). Captures the arm cascade so PathTracker's arming is assertable.
 * @author WRO 2026
 *
 * @details
 * Shadows the production `WRO2026/LineFollower.h` (same `LINEFOLLOWER_H` guard,
 * force-included by `add_path_force_includes`) so the five PathTracker host tests link this
 * stub instead of the real `LineFollower.cpp` (which binds Drive / WROLineArray /
 * Intersection and the `MiniR4` global). It mocks the outer P+D controller that the REAL
 * `PathTracker.cpp` and REAL `Drive.cpp` only *command* — neither production `.cpp`'s link
 * needs the controller's PID math, only its enable/arm verbs and read-and-clear gates.
 *
 * Verified production call sites this surface satisfies:
 *   - `Drive.cpp:1607-1609`   `if (LineFollower.isEnabled()) LineFollower.setEnabled(false).setBrake(false);`
 *                             (the `_dispatchNext` race-guard — the `[[nodiscard]]` builder
 *                             return MUST support `.setBrake(...)` or real `Drive.cpp` will
 *                             not compile).
 *   - `PathTracker.cpp:300`   `LineFollower.setEnabled(false);`                              (bare)
 *   - `PathTracker.cpp:822`   `setEnabled(true).setTShapeRampUp(false).setTShapeRampDown(!_passThrough)`
 *                             `.setOvershootMode(true).setReverse(...).setBrakeOnGate(...)`
 *                             `.setDistanceTarget(...)`
 *   - `PathTracker.cpp:858`   `if (LineFollower.isDistanceReached())`
 *   - `PathTracker.cpp:887`   `if (LineFollower.isDistanceReached())`
 *   - `PathTracker.cpp:918`   `LineFollower.setEnabled(false);`                              (bare)
 *   - `PathTracker.cpp:1109`  `LineFollower.setEnabled(false);`                              (bare)
 *   - `PathTracker.cpp:1637-1639` `if (LineFollower.isEnabled()) LineFollower.setEnabled(false).setBrake(false);`
 *   - `PathTracker.cpp:1924`  `setEnabled(true).setPreset(...).setReverse(...).setOvershootMode(false)`
 *                             `.setBrakeOnGate(...).setDistanceTarget(...)`
 *   - `PathTracker.cpp:1941`  `setEnabled(true).setPreset(...).setReverse(...).setOvershootMode(false)`
 *   - `PathTracker.cpp:2021`  `LineFollower.setEnabled(false).setBrake(false);`
 *   - `PathTracker.cpp:2119`  `presetToPwm(LineFollower.getPreset());`
 *   - `PathTracker.cpp:2120`  `LineFollower.setEnabled(false).setBrake(false);`
 *
 * The `setEnabled` builder mirrors the real immovable (copy- and move-deleted),
 * `[[nodiscard]]` chain surface so every chained statement above compiles unchanged. Each
 * builder setter records its key arg straight back into the owner (record-on-call, NOT
 * deferred to a dtor-apply) so a test can assert `lastEnabled` / `lastBrakeOnGate` /
 * `lastTShapeRampDown` / `lastDistanceTarget` immediately after the statement.
 *
 * ---------------------------------------------------------------------------
 * Fidelity — where this mock can lie vs real hardware
 * ---------------------------------------------------------------------------
 * THIN record-and-test-set per ADR-049. It models nothing the host cannot observe:
 *   - No P+D controller, no `WROLineArrayClass` sampling, no `Drive.setSpeed` writes, no
 *     trapezoidal ramp, no encoder distance gate. `update()` / `begin()` /
 *     `setTickCallback()` are no-ops.
 *   - State queries (`isEnabled()` / `isDistanceReached()` / `isLineLost()`) return
 *     test-set fields — they do NOT derive from any scripted move. The test arms the gate trip
 *     via `mockSetDistanceReached(...)`; `isDistanceReached()` is READ-AND-CLEAR and the
 *     enable/disable verb also disarms the latch, exactly like the real one-shot gate
 *     (LineFollower.cpp:717/742/789-794), so a trip cannot leak across hops.
 *   - The arm-cascade capture fields record the LAST value passed; a multi-call sequence
 *     overwrites (last-write-wins), matching the simplest scripted-sensor pattern. The
 *     count fields (`setEnabledCount`) let a test assert how many arm transitions ran.
 *   - `getArray()` returns `nullptr` — the mock never owns a line array; the path mocks'
 *     `LineArray` global is reached directly by `Drive.cpp`, not through this handle.
 */

// NOTE: This mock uses the SAME include guard (`LINEFOLLOWER_H`) as the production
// `WRO2026/LineFollower.h`. See `path_mocks/WROLineArray.h` for the full guard-shadow
// rationale.
#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <stdint.h>

#include "Config.h"          // real — Config::SpeedPreset for the preset setters/queries.
#include "WROLineArray.h"    // For the WROLineArrayClass type behind getArray().

// Forward declaration so the builder can hold a reference to the owner type.
class LineFollowerClass;

/**
 * @brief Recording mock of `LineFollowerEnableBuilder`.
 *
 * @details
 * Non-copyable AND non-movable (both deleted below) and `[[nodiscard]]` to match the real
 * builder — `setEnabled()` returns it by value via guaranteed copy elision (C++17), not a
 * move. Unlike the real builder (which defers its apply to the destructor), this mock holds
 * a reference to the owner and records each setter's key arg into the owner ON CALL, so the
 * captured value is observable the instant the chained statement ends — no dtor-apply timing
 * to reason about in a test. Every setter returns `*this` for fluent chaining.
 */
class [[nodiscard]] LineFollowerEnableBuilder
{
public:
    /**
     * @brief Construct the builder bound to its owner.
     * @param owner Reference to the owning `LineFollowerClass` (the singleton). Must outlive
     *              `this`, trivially true for a stack temporary from a singleton method call.
     * @note The owner's `lastEnabled` / enabled state is recorded by `setEnabled(...)` BEFORE
     *       this builder is constructed, not here, so a bare `setEnabled(false);` records too.
     */
    explicit LineFollowerEnableBuilder(LineFollowerClass& owner) : _owner(owner) {}

    LineFollowerEnableBuilder(const LineFollowerEnableBuilder&)            = delete;
    LineFollowerEnableBuilder& operator=(const LineFollowerEnableBuilder&) = delete;
    LineFollowerEnableBuilder(LineFollowerEnableBuilder&&)                 = delete;
    LineFollowerEnableBuilder& operator=(LineFollowerEnableBuilder&&)      = delete;

    /** @brief Record the brake-on-disable override. @return `*this`. */
    LineFollowerEnableBuilder& setBrake(bool brake);

    /** @brief Record the cruise-preset override. @return `*this`. */
    LineFollowerEnableBuilder& setPreset(Config::SpeedPreset preset);

    /** @brief Record the ramp-up enable. @return `*this`. */
    LineFollowerEnableBuilder& tShape();

    /** @brief Record the ramp-up opt-out flag. @return `*this`. */
    LineFollowerEnableBuilder& setTShapeRampUp(bool on);

    /** @brief Record the ramp-down opt-out flag (`lastTShapeRampDown`). @return `*this`. */
    LineFollowerEnableBuilder& setTShapeRampDown(bool on);

    /** @brief Record the "run flat" opt-out. @return `*this`. */
    LineFollowerEnableBuilder& flat();

    /** @brief Record the per-cm acceleration override. @return `*this`. */
    LineFollowerEnableBuilder& setAccel(float pwmPerCm);

    /** @brief Record the reverse-direction override. @return `*this`. */
    LineFollowerEnableBuilder& setReverse(bool reverse);

    /** @brief Record the distance-gate target (`lastDistanceTarget`). @return `*this`. */
    LineFollowerEnableBuilder& setDistanceTarget(float cm);

    /** @brief Record the overshoot-mode override. @return `*this`. */
    LineFollowerEnableBuilder& setOvershootMode(bool on);

    /** @brief Record the brake-on-gate override (`lastBrakeOnGate`). @return `*this`. */
    LineFollowerEnableBuilder& setBrakeOnGate(bool on);

private:
    LineFollowerClass& _owner;    ///< Owner whose capture fields each setter records into.
};

/**
 * @brief Recording mock of the LineFollower controller facade.
 *
 * @details
 * Records the arm cascade (`lastEnabled`, `lastBrake`, `lastBrakeOnGate`,
 * `lastTShapeRampUp`, `lastTShapeRampDown`, `lastDistanceTarget`, `lastPreset`,
 * `lastReverse`, `lastOvershootMode`) plus a `setEnabledCount`, and exposes test-set state
 * queries (`isEnabled` / `isDistanceReached` / `isLineLost`). See the file-level Fidelity
 * block for what this mock deliberately does not model. The recording fields are public so
 * tests assert them directly.
 */
class LineFollowerClass
{
public:
    // --- recording fields (public; test-asserted directly) ---

    /** @brief Last `on` value passed to `setEnabled(...)`. Default `false`. */
    bool lastEnabled = false;
    /** @brief Number of `setEnabled(...)` calls since the last `mockReset()`. */
    int setEnabledCount = 0;
    /** @brief Last `brake` value passed to the builder's `setBrake(...)`. Default `true`. */
    bool lastBrake = true;
    /** @brief Last `on` value passed to `setBrakeOnGate(...)`. Default `true` (owner default). */
    bool lastBrakeOnGate = true;
    /** @brief Last `on` value passed to `setTShapeRampUp(...)`. Default `true`. */
    bool lastTShapeRampUp = true;
    /** @brief Last `on` value passed to `setTShapeRampDown(...)`. Default `true`. */
    bool lastTShapeRampDown = true;
    /** @brief Last `cm` value passed to `setDistanceTarget(...)`. Default `0.0f`. */
    float lastDistanceTarget = 0.0f;
    /** @brief Last preset passed to `setPreset(...)`. Default `NORMAL`. */
    Config::SpeedPreset lastPreset = Config::SpeedPreset::NORMAL;
    /** @brief Last `reverse` value passed to `setReverse(...)`. Default `false`. */
    bool lastReverse = false;
    /** @brief Last `on` value passed to `setOvershootMode(...)`. Default `false`. */
    bool lastOvershootMode = false;

    /**
     * @brief Reset all recording + test-set fields to their initial values.
     * @note Call between test cases so a prior case's arm cascade does not leak into the next.
     */
    void mockReset()
    {
        lastEnabled        = false;
        setEnabledCount    = 0;
        lastBrake          = true;
        lastBrakeOnGate    = true;
        lastTShapeRampUp   = true;
        lastTShapeRampDown = true;
        lastDistanceTarget = 0.0f;
        lastPreset         = Config::SpeedPreset::NORMAL;
        lastReverse        = false;
        lastOvershootMode  = false;
        _enabled           = false;
        _distanceReached   = false;
        _lineLost          = false;
        _preset            = Config::SpeedPreset::NORMAL;
        _reverse           = false;
    }

    // --- lifecycle (no-op) ---

    /** @brief Bind / reset hook — no-op on host. @return Always `true`. */
    bool begin() { return true; }
    /** @brief Cooperative tick — no-op on host (no P+D controller). */
    void update() {}
    /** @brief Register the cooperative-tick callback — no-op on host. */
    void setTickCallback(void (*)()) {}

    // --- fluent enable verb (records then returns the recording builder) ---

    /**
     * @brief Enable or disable the line follower.
     * @param on `true` to arm following, `false` to disable.
     * @return A `[[nodiscard]]` `LineFollowerEnableBuilder` bound to this owner for chaining.
     * @note Records `lastEnabled`, bumps `setEnabledCount`, and sets the test-set `_enabled`
     *       state (so `isEnabled()` reflects the transition) BEFORE returning the builder —
     *       so a bare `setEnabled(false);` and the chained `setEnabled(false).setBrake(false)`
     *       race-guard both record. The real `Drive.cpp`/`PathTracker.cpp` discard the
     *       returned builder at end-of-statement; the `[[nodiscard]]` is satisfied by their
     *       chained `.setBrake(...)` / void-context use.
     * @note Clears `_distanceReached` on every enable/disable, matching the real controller —
     *       which disarms the latch both when disabled (LineFollower.cpp:742) and when a fresh
     *       distance target arms (LineFollower.cpp:717). Without this a trip set for one hop would
     *       survive the next hop's re-arm and false-pass a regression that re-armed without a new
     *       gate trip.
     */
    [[nodiscard]] LineFollowerEnableBuilder setEnabled(bool on)
    {
        lastEnabled = on;
        setEnabledCount++;
        _enabled         = on;
        _distanceReached = false;    // re-arm / disable clears the one-shot latch (matches production)
        return LineFollowerEnableBuilder{*this};
    }

    // --- direct preset / direction ---

    /** @brief Direct cruise-preset setter (records `lastPreset`). */
    void setPreset(Config::SpeedPreset preset)
    {
        _preset    = preset;
        lastPreset = preset;
    }
    /** @brief Read the active cruise preset. @return The last set preset. */
    Config::SpeedPreset getPreset() const { return _preset; }
    /** @brief Direct reverse-direction setter (records `lastReverse`). */
    void setReverse(bool reverse)
    {
        _reverse    = reverse;
        lastReverse = reverse;
    }
    /** @brief Read the active line-follow direction. @return `true` if reversing. */
    bool getReverse() const { return _reverse; }

    // --- state queries (test-set) ---

    /**
     * @brief Whether following is currently armed (test-set + set by `setEnabled`).
     * @return The `_enabled` field — set by the last `setEnabled(...)` or `mockSetEnabled(...)`.
     * @note The real `Drive.cpp`/`PathTracker.cpp` race-guards branch on this before
     *       `setEnabled(false).setBrake(false)`.
     */
    bool isEnabled() const { return _enabled; }

    /**
     * @brief Whether the line was lost on the most recent (mock) tick (test-set).
     * @return The `_lineLost` field.
     */
    bool isLineLost() const { return _lineLost; }

    /** @brief Whether the P+D mix was frozen by a junction — always `false` on host. */
    bool isPidFrozen() const { return false; }

    /** @brief Most recent weighted error — `0.0f` on host (no controller). */
    float getRawError() const { return 0.0f; }

    /**
     * @brief Whether the distance gate has tripped — READ-AND-CLEAR, matching production.
     * @return The latched `_distanceReached` value; the latch is cleared as a side effect.
     * @note Mirrors the real one-shot latch (`LineFollowerClass::isDistanceReached`,
     *       LineFollower.cpp:789-794): the first read returns the trip and consumes it. The test
     *       arms the trip with `mockSetDistanceReached(true)` just before the single FSM poll; the
     *       FSM reads it once and transitions in the same tick (PathTracker.cpp:858/887), so the
     *       consume-on-read never starves the FSM. A sticky latch here would false-pass a
     *       regression that stopped polling the gate or ended a hop one gate early.
     */
    bool isDistanceReached()
    {
        const bool latched = _distanceReached;
        _distanceReached   = false;
        return latched;
    }

    // --- shared sensor handle (never dereferenced in the mock) ---

    /** @brief Pointer to the shared line array — `nullptr` on host. */
    WROLineArrayClass* getArray() { return nullptr; }

    // --- sync-blocking helpers (instant success on host) ---

    /** @brief Sync-blocking follow-distance helper — instant success on host. */
    bool followDistance(float /*cm*/) { return true; }
    /** @brief Block until the distance gate trips — instant success on host. */
    bool waitUntilDistanceReached() { return true; }
    /** @brief Sync-blocking follow-until-intersection helper — instant success on host. */
    bool followUntilIntersection() { return true; }
    /** @brief Block until the next intersection edge — instant success on host. */
    bool waitUntilIntersection() { return true; }

    // --- test setters ---

    /** @brief Set the `isEnabled()` state directly (bypasses the arm-cascade record). */
    void mockSetEnabled(bool on) { _enabled = on; }
    /** @brief Set the `isDistanceReached()` gate-trip flag. */
    void mockSetDistanceReached(bool on) { _distanceReached = on; }
    /** @brief Set the `isLineLost()` flag. */
    void mockSetLineLost(bool on) { _lineLost = on; }

private:
    Config::SpeedPreset _preset          = Config::SpeedPreset::NORMAL;  ///< Active preset.
    bool                _reverse         = false;                        ///< Active direction.
    bool                _enabled         = false;                        ///< Test-set enabled state.
    bool                _distanceReached = false;                        ///< Test-set gate-trip flag.
    bool                _lineLost        = false;                        ///< Test-set lost-line flag.
};

// --- LineFollowerEnableBuilder setter definitions (after the owner is complete) ---

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setBrake(bool brake)
{
    _owner.lastBrake = brake;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setPreset(Config::SpeedPreset preset)
{
    _owner.lastPreset = preset;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::tShape()
{
    _owner.lastTShapeRampUp = true;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setTShapeRampUp(bool on)
{
    _owner.lastTShapeRampUp = on;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setTShapeRampDown(bool on)
{
    _owner.lastTShapeRampDown = on;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::flat()
{
    _owner.lastTShapeRampUp   = false;
    _owner.lastTShapeRampDown = false;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setAccel(float /*pwmPerCm*/)
{
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setReverse(bool reverse)
{
    _owner.lastReverse = reverse;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setDistanceTarget(float cm)
{
    _owner.lastDistanceTarget = cm;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setOvershootMode(bool on)
{
    _owner.lastOvershootMode = on;
    return *this;
}

inline LineFollowerEnableBuilder& LineFollowerEnableBuilder::setBrakeOnGate(bool on)
{
    _owner.lastBrakeOnGate = on;
    return *this;
}

/**
 * @brief Single global instance — `inline` so exactly ONE definition exists across the
 *        target's TUs without anything in `path_mocks.cpp`.
 *
 * @note The `mocks/` / `task_mocks/` / `mmL` idiom: declaring the global `inline` in-header
 *       (C++17 inline variable) means the facade-globals TU defines no peripheral global,
 *       decoupling it from this file.
 */
inline LineFollowerClass LineFollower;

#endif    // LINEFOLLOWER_H
