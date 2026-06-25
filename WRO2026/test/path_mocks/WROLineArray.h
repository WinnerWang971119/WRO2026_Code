/**
 * @file WROLineArray.h (PATH HOST MOCK)
 * @brief Thin record-and-test-set `WROLineArrayClass` mock for the PathTracker + Drive
 *        host-link targets (issue #106).
 * @author WRO 2026
 *
 * @details
 * Shadows the production `WRO2026/WROLineArray.h` (same `WROLINEARRAY_H` guard,
 * force-included FIRST among the `path_mocks/` peripherals by `add_path_force_includes`)
 * so the five PathTracker host tests link this stub instead of the real
 * `WROLineArray.cpp` (which binds the `MatrixColor` MXColor driver and the `MiniR4`
 * global). It mocks the line-sensor leaf that the REAL `Drive.cpp` only *commands*:
 * `Drive.cpp`'s seek FSM reads `LineArray.getJunctionType()` to debounce a crossing and
 * calls `LineArray.setReverseActive(cmd.reverse)` to face the travel direction. The real
 * `Intersection`/`LineFollower` path mocks reference its `JunctionType` enum.
 *
 * Verified production call sites this surface satisfies:
 *   - `Drive.cpp:1301`  `LineArray.getJunctionType() == WROLineArrayClass::JUNCTION_T_OR_CROSS`
 *   - `Drive.cpp:1364`  `LineArray.setReverseActive(false)`
 *   - `Drive.cpp:1535`  `LineArray.setReverseActive(false)`
 *   - `Drive.cpp:1742`  `LineArray.setReverseActive(cmd.reverse)`
 *   - `path_mocks/Intersection.h` + the production `JunctionType` enum (declared HERE,
 *     mirroring `Intersection.h`'s production type so every TU agrees).
 *
 * Each getter returns a test-settable scalar. Multi-value sequences are driven by calling
 * `mockSet*` between cooperative ticks — the simplest possible "scripted sensor" pattern
 * (no queue, just last-write-wins).
 *
 * ---------------------------------------------------------------------------
 * Fidelity — where this mock can lie vs real hardware
 * ---------------------------------------------------------------------------
 * THIN record-and-test-set per ADR-049. It models nothing the host cannot observe:
 *   - No I2C, no MXColor board, no CMYK byte unpacking, no two-layer calibration. `begin()`
 *     and `calibrate()` are no-ops; `getError()` / `isOnline()` / `getJunctionType()` are
 *     pure backing-field reads, never resampled from a bus.
 *   - No multi-tick state evolution: the junction value the test sets via
 *     `mockSetJunctionType(...)` stays put until the next `mockSet*` — `update()` does NOT
 *     "eventually" change it, and `getJunctionType()` does NOT clear it. Scripting the
 *     crossing sequence is the test body's job, not this mock's.
 *   - `setReverseActive(...)` records the flag and a call count; a real 1-board config
 *     would no-op it. The mock always records so a test can assert the seek's board flip.
 *   - `getReverseActive()` echoes the last recorded flag — no 1-board clamp.
 */

// NOTE: This mock uses the SAME include guard (`WROLINEARRAY_H`) as the production
// `WRO2026/WROLineArray.h`. It is force-included BEFORE any test TU pulls
// `LineFollower.h`/`Intersection.h`/`Drive.cpp`, so when those later resolve
// `#include "WROLineArray.h"` the production header is short-circuited by the
// already-defined guard. See `WRO2026/test/CMakeLists.txt::add_path_force_includes`.
#ifndef WROLINEARRAY_H
#define WROLINEARRAY_H

#include <stdint.h>

/**
 * @brief Mirror of the production `JunctionType` enum (`WRO2026/Intersection.h`).
 *
 * @details
 * Underlying values match the library `0..4` convention EXACTLY so a plain
 * `static_cast<JunctionType>(raw)` round-trips against the `uint8_t` returned by
 * `getJunctionType()` — the same contract the real `Drive.cpp`, `Intersection.cpp`, and
 * `LineFollower.cpp` rely on.
 *
 * Declared HERE (not in `path_mocks/Intersection.h`) because the real `Drive.cpp` casts
 * `WROLineArrayClass::JUNCTION_T_OR_CROSS` against the raw `getJunctionType()` return, and
 * both `Intersection.h` and the test TUs include this header — so the enum must live where
 * every caller sees it, exactly as the production module is shaped.
 *
 * @note `UNKNOWN` (4) is never emitted by the production line-array backend, but the value
 *       is carried for type-identity with production so a `switch` or `static_cast` over the
 *       enum stays exhaustive.
 */
enum class JunctionType : uint8_t
{
    NONE       = 0,    ///< No junction under the sensor.
    LEFT       = 1,    ///< Left branch detected.
    RIGHT      = 2,    ///< Right branch detected.
    T_OR_CROSS = 3,    ///< T-junction or 4-way crossroads.
    UNKNOWN    = 4     ///< Sensor reported a junction but could not classify it.
};

/**
 * @brief Thin record-and-test-set mock of the 8-channel reflectance line array.
 *
 * @details
 * Exposes the exact public surface the real `Drive.cpp` reaches plus the legacy getters
 * (`getError()` / `isOnline()`) so nothing the production line-sensor contract advertises is
 * missing. Every method is either a pure backing-field read or a recording no-op; see the
 * file-level Fidelity block for what this mock deliberately does not model.
 */
class WROLineArrayClass
{
public:
    WROLineArrayClass() = default;

    /**
     * @brief Raw `getJunctionType()` return value for a full-bar T-junction / 4-way crossing.
     *
     * @details
     * Mirrors the production `static constexpr` so the real `Drive.cpp` seek FSM can compare
     * `getJunctionType() == WROLineArrayClass::JUNCTION_T_OR_CROSS` against this mock unchanged.
     * Equals `JunctionType::T_OR_CROSS` (`3`).
     */
    static constexpr uint8_t JUNCTION_T_OR_CROSS = 3;

    /**
     * @brief Reset every recording/test-set field to its construction default.
     *
     * @note Call between test cases so a prior case's scripted junction value or
     *       reverse-active flag does not leak into the next.
     */
    void mockReset()
    {
        _error               = 0.0f;
        _isOnline            = true;
        _junctionType        = (uint8_t) JunctionType::NONE;
        _reverse             = false;
        _setReverseCallCount = 0;
        _updateCallCount     = 0;
    }

    // ------------------------------------------------------------------ lifecycle (no-op)

    /**
     * @brief Bring-up hook — no-op on host (no MXColor board to initialise).
     * @return Always `true` (the real front-board success path).
     */
    bool begin() { return true; }

    /**
     * @brief Per-tick refresh hook — records the call, performs no I2C.
     * @note The getters already return whatever the test set, so `update()` only bumps a
     *       counter a test may assert (the real blocking-wait loops call it). It does NOT
     *       evolve the junction value — that is the test body's job (ADR-049).
     */
    void update() { _updateCallCount++; }

    /**
     * @brief Software per-channel calibration routine — no-op on host.
     * @note The real routine is a sync-blocking ~5 s operator sweep; the host mock skips it.
     */
    void calibrate() {}

    // ------------------------------------------------------------------ surface real Drive.cpp / legacy callers reach

    /**
     * @brief Weighted-center error for the active board (test-set).
     * @return The last value set via `mockSetError(...)`; `0.0f` until set.
     * @note Carried for surface-completeness with the production line-sensor contract; the
     *       PathTracker host tests drive nav through `getJunctionType()`, not this.
     */
    float getError() { return _error; }

    /**
     * @brief Whether any channel is on-line (test-set).
     * @return The last value set via `mockSetOnline(...)`; `true` until set.
     */
    bool isOnline() { return _isOnline; }

    /**
     * @brief Junction classification for the active board (test-set).
     * @return The raw `uint8_t` (`0..4`) last set via `mockSetJunctionType(...)` /
     *         `mockSetJunctionTypeRaw(...)`; `NONE` (0) until set.
     * @note The real `Drive.cpp` seek FSM compares this against `JUNCTION_T_OR_CROSS`. A test
     *       drives a crossing by setting `T_OR_CROSS` for the tick the chassis straddles the
     *       marker, then back to `NONE`.
     */
    uint8_t getJunctionType() { return _junctionType; }

    /**
     * @brief Flip the active board to follow the chassis reverse state (record).
     * @param reverse `true` selects the rear board, `false` the front board.
     * @note The real `Drive.cpp` calls this from its seek dispatch (`setReverseActive(false)`
     *       on entry/restore, `setReverseActive(cmd.reverse)` for a reverse seek). The mock
     *       records the flag and bumps a call count so a test can assert the board flip.
     */
    void setReverseActive(bool reverse)
    {
        _reverse = reverse;
        _setReverseCallCount++;
    }

    /**
     * @brief Read the last recorded active-board direction.
     * @return `true` if the rear board was last selected; `false` for the front board.
     * @note No 1-board clamp — echoes whatever `setReverseActive(...)` last recorded.
     */
    bool getReverseActive() const { return _reverse; }

    // ------------------------------------------------------------------ test setters

    /** @brief Set the weighted-center error the next `getError()` returns. */
    void mockSetError(float e) { _error = e; }

    /** @brief Set the on-line predicate the next `isOnline()` returns. */
    void mockSetOnline(bool on) { _isOnline = on; }

    /** @brief Set the junction classification the next `getJunctionType()` returns. */
    void mockSetJunctionType(JunctionType j) { _junctionType = (uint8_t) j; }

    /** @brief Set the junction classification as a raw `uint8_t` (`0..4`). */
    void mockSetJunctionTypeRaw(uint8_t v) { _junctionType = v; }

    /** @brief Number of `setReverseActive(...)` calls since the last `mockReset()`. */
    uint32_t mockSetReverseCallCount() const { return _setReverseCallCount; }

    /** @brief Number of `update()` calls since the last `mockReset()`. */
    uint32_t mockUpdateCallCount() const { return _updateCallCount; }

private:
    float    _error               = 0.0f;                          ///< Test-set weighted error.
    bool     _isOnline            = true;                          ///< Test-set on-line predicate.
    uint8_t  _junctionType        = (uint8_t) JunctionType::NONE;  ///< Test-set junction value.
    bool     _reverse             = false;                         ///< Last recorded active-board flag.
    uint32_t _setReverseCallCount = 0;                             ///< `setReverseActive` call count.
    uint32_t _updateCallCount     = 0;                             ///< `update` call count.
};

/**
 * @brief Single global instance — `inline` so exactly ONE definition exists across the
 *        target's TUs without anything in `path_mocks.cpp`.
 *
 * @note The `mocks/` / `task_mocks/` / `mmL` idiom: declaring the global `inline` in-header
 *       (C++17 inline variable) means the facade-globals TU defines no peripheral global,
 *       decoupling it from this file. A `path_mocks.cpp` definition would ODR-clash with the
 *       real `LineArray` only if production `WROLineArray.cpp` were linked — it is not, but
 *       the `inline` keeps the contract self-contained either way.
 */
inline WROLineArrayClass LineArray;

#endif    // WROLINEARRAY_H
