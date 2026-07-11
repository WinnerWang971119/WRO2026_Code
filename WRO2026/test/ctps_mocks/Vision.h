/**
 * @file Vision.h (CTPS HOST MOCK)
 * @brief Recording + test-set mock of the Vision mosaic-colour facade for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `WRO2026/src/perception/Vision.h` (quote-included; force-included
 * by the T7 cmake wiring into the `ctps_dump` target ONLY). The include guard is the
 * REAL one (`VISION_H`) so the real header no-ops if reached afterwards — the same
 * shadow pattern the sibling `ctps_mocks/` headers use (Drive / Manipulator / …).
 *
 * **Why this mock exists.** `ctps_dump` compiles the WHOLE of `Tasks.cpp` (and any
 * future mission) as one TU. A Phase-6 vision-consuming task that calls the `Vision`
 * facade would break the dump build unless this mock stands in for the real,
 * `Serial1`-bound `VisionClass`. It is landed and CI-proven NOW (via the `ctps_dump`
 * self-test synthesis) so it is verified before any real consumer exists (plan T4/T6).
 *
 * **Thin mock — record + test-set only (ADR-049).** The mock carries NO cone / FOV
 * geometry: the cone-visibility prediction is a Python/JS-side concern (the player HUD).
 * The mock's `readMosaic()` return value is intentionally DECOUPLED from any geometry —
 * it returns the injected result (true, or false at a configured fail-at call) and
 * records a `read_mosaic` op on every call. `getColor()` returns the injected colour.
 *
 * **Recording seam.** Every `readMosaic()` appends one `read_mosaic` op to the shared
 * `g_ctpsLog` through the free recorder `ctps::recordReadMosaic(bool success)` (added by
 * task T5 alongside `Op::READ_MOSAIC`), matching the free-function record pattern the
 * other mocks use (`ctps::recordManip`, `ctps::recordDriveMove`, …). The op carries the
 * success flag only — the rolled colours are the top-level `mosaic` echo, not a per-op
 * copy.
 *
 * **Test-set hooks (used by the T6 `ctps_dump` argv wiring).** `_setInjectedColors(...)`
 * injects the 12 cell colours (from `--mosaic`); `_setFailAt(n)` makes the Nth
 * `readMosaic()` return false (0/absent = never fails). Both are named with a leading
 * underscore + `_set…` so they read unambiguously as fixture-only hooks (they do NOT
 * exist on the production `VisionClass` facade).
 *
 * The `Fault` / `FaultInfo` types mirror the real facade so any call site that queries
 * the fault surface compiles unchanged; the mock never latches a fault (a thin,
 * always-succeeding-unless-told-otherwise stand-in).
 */

#ifndef VISION_H
#define VISION_H

#include <stdint.h>

// The REAL colour enum, single-sourced. `Config::Color` is declared in
// `WRO2026/config/ConfigVision.h` as `{ UNKNOWN=0, YELLOW, BLUE, GREEN, WHITE }`,
// wrapped in a `#pragma push_macro("WHITE") / #undef WHITE / … / #pragma pop_macro`
// guard (MiniR4OLED.h `#define`s WHITE). Resolved via the `${SKETCH_DIR}` include path
// the `ctps_dump` target puts on `-I` (same path that resolves the sibling Drive mock's
// `#include "Config.h"`). Include the real header for the enum rather than redeclaring
// it — a hand-copied list would drift from `Config::Color`.
#include "config/ConfigVision.h"

#include "ctps_record.h"

/**
 * @brief Recording + test-set mock of the Vision mosaic-colour facade.
 *
 * Mirrors the subset of the production `VisionClass` surface a mission calls:
 * lifecycle (`begin` / `start`), the blocking command (`readMosaic`), the pure
 * getters (`getColor` / `getRawHue`), the fault surface
 * (`hasFault` / `getFault` / `clearFault`), and `setTickCallback`. The two `_set…`
 * hooks are FIXTURE-ONLY injection points for the dump self-test / argv wiring.
 */
class VisionClass
{
public:
    // -----------------------------------------------------------------------------------------
    // Fault types — mirror the real facade so a fault-querying call site compiles unchanged.
    // -----------------------------------------------------------------------------------------

    /** @brief Source of a Vision-layer fault (mirrors the production enum). */
    enum class Fault : uint8_t
    {
        NONE         = 0,   ///< No fault — normal operation.
        READ_TIMEOUT = 1,   ///< UART read did not complete within the timeout.
        BAD_CHECKSUM = 2,   ///< Received frame failed its integrity check.
        BAD_CELL     = 3    ///< An active mosaic cell classified as UNKNOWN.
    };

    /** @brief Captured details of the last Vision fault (mirrors the production struct). */
    struct FaultInfo
    {
        Fault   kind         = Fault::NONE;   ///< Which failure occurred.
        uint8_t firstBadCell = 0xFF;          ///< First out-of-range cell index (`BAD_CELL` only).
        uint8_t rawValue     = 0;             ///< Raw byte that caused the fault.
    };

    // -----------------------------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------------------------

    /** @brief No-op UART bring-up; always reports success (matches the real `begin()` bool). */
    bool begin() { return true; }

    /** @brief No-op camera wake (pure TX on the real facade — nothing to do in the mock). */
    void start() {}

    // -----------------------------------------------------------------------------------------
    // Command
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Record one `read_mosaic` op and report the injected result.
     *
     * Increments the internal call counter, then returns `false` iff a fail-at call
     * index was configured via `_setFailAt(n)` and this is the Nth call (1-based);
     * otherwise returns `true`. The colour latch is the injected colours regardless
     * (a false read does NOT clear them — the mock is a thin stand-in, not a model of
     * the firmware's no-overwrite-on-failure invariant).
     *
     * A `read_mosaic` op is recorded on EVERY call (both success and failure) so the
     * op pipeline is exercised whatever the outcome.
     *
     * @return `true` normally; `false` on the configured fail-at Nth call.
     */
    bool readMosaic()
    {
        ++_callCount;
        const bool success = !(_failAt > 0 && _callCount == _failAt);
        ctps::recordReadMosaic(success);
        return success;
    }

    // -----------------------------------------------------------------------------------------
    // Getters (pure, safe-sentinel — mirror the real bounds-checking contract)
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Injected colour for mosaic cell @p cell.
     * @param cell Zero-based cell index in `[0, Config::VISION_CELL_COUNT)`.
     * @return The injected `Config::Color` for the cell, or `Config::Color::UNKNOWN`
     *         if @p cell is out of range (bounds-checking mirrors the real getter — an
     *         out-of-range index never traps, it returns the safe sentinel).
     */
    Config::Color getColor(uint8_t cell) const
    {
        if (cell >= Config::VISION_CELL_COUNT)
        {
            return Config::Color::UNKNOWN;
        }
        return _colors[cell];
    }

    /**
     * @brief Raw hue for mosaic cell @p cell — the mock carries no hue data, so `0`.
     * @param cell Zero-based cell index (ignored beyond the bounds guard).
     * @return `0` always (the thin mock injects colours only, never raw hues; `0` is the
     *         real getter's safe sentinel).
     */
    uint8_t getRawHue(uint8_t /*cell*/) const { return 0; }

    // -----------------------------------------------------------------------------------------
    // Fault surface — the mock never latches a fault.
    // -----------------------------------------------------------------------------------------

    /** @brief Whether a fault is currently latched (the mock never faults → `false`). */
    bool hasFault() const { return false; }

    /** @brief Snapshot of the latched fault details (always the NONE default). */
    FaultInfo getFault() const { return FaultInfo{}; }

    /** @brief Clear the latched fault — no-op (the mock never latches one). */
    void clearFault() {}

    // -----------------------------------------------------------------------------------------
    // Callback
    // -----------------------------------------------------------------------------------------

    /** @brief No-op tick-callback registration (matches the real facade surface). */
    void setTickCallback(void (*)()) {}

    // -----------------------------------------------------------------------------------------
    // Fixture-only test-set hooks (NOT on the production facade; used by T6 ctps_dump).
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Inject the 12 cell colours returned by `getColor()` (fixture-only hook).
     *
     * Copies `Config::VISION_CELL_COUNT` colours into the mock's colour latch. Called by
     * the `ctps_dump --mosaic <letters>` argv wiring (T6) after parsing each letter to a
     * `Config::Color`.
     *
     * @param colors Source array of exactly `Config::VISION_CELL_COUNT` colours.
     */
    void _setInjectedColors(const Config::Color colors[Config::VISION_CELL_COUNT])
    {
        for (uint8_t i = 0; i < Config::VISION_CELL_COUNT; ++i)
        {
            _colors[i] = colors[i];
        }
    }

    /**
     * @brief Configure which `readMosaic()` call returns `false` (fixture-only hook).
     *
     * @param nthCall 1-based call index that should fail (the Nth `readMosaic()` returns
     *                `false`). Pass `0` (the default) to never fail. A negative value is
     *                treated as "never fail" too (clamped to 0), so a caller that maps an
     *                absent `--mosaic-fail-at` to `0`/`-1` gets the same "never" behaviour.
     */
    void _setFailAt(int nthCall) { _failAt = (nthCall > 0) ? static_cast<uint32_t>(nthCall) : 0; }

private:
    Config::Color _colors[Config::VISION_CELL_COUNT] = {};   ///< Injected per-cell colour latch (UNKNOWN until injected).
    uint32_t      _failAt    = 0;                             ///< 1-based fail-at call index; 0 = never fail.
    uint32_t      _callCount = 0;                             ///< Number of `readMosaic()` calls so far.
};

/**
 * @brief Single global instance — defined in ctps_record.cpp (alongside the other mock
 *        singletons; declared `extern` here to match the real facade's export).
 */
extern VisionClass Vision;

#endif    // VISION_H
