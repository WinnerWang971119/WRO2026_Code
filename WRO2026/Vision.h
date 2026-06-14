/**
 * @file Vision.h
 * @brief Mosaic-colour-identification singleton wrapping the HUE12 UART camera.
 *
 * @details
 * `VisionClass` is the single entry point mission code uses to capture and query the
 * WRO 2026 game-mosaic colour map. It owns the UART link to the HUE12 camera, issues
 * a one-shot read, and exposes per-cell accessors for the resulting colour classifications
 * and raw hue values.
 *
 * **Usage pattern.** Call `readMosaic()` exactly once per observation site (the chassis
 * must be fully stopped — see the `@note` on `readMosaic`). On success, query each of the
 * `Config::VISION_CELL_COUNT` cells with `getColor()` / `getRawHue()`. On failure, the
 * latch retains its prior contents (safe-default: all UNKNOWN before any successful read)
 * and `hasFault()` returns true.
 *
 * **Fault model.** `hasFault()` / `getFault()` / `clearFault()` mirror the pattern used
 * by `Drive` and `PathTracker`. `readMosaic()` returns `false` on failure **and** latches a
 * `FaultInfo` with the specific cause. The latch is NOT updated on failure (per ADR-037:
 * a false return from a command-time blocking call is authoritative; stale latch contents
 * do not become current). Call `clearFault()` then retry, or abort the mission step.
 *
 * **No `update()` method.** Vision does no async work and is intentionally absent from
 * `cooperativeTick()`. Every observation is a deliberate blocking call issued by mission
 * code at a known idle point (ADR-037 sanctioned blocking site); there is nothing to advance
 * between observations, so a per-tick cooperative hook would be dead code. (ADR-037 sanctions
 * exactly this pattern: a driver that blocks at command time and is otherwise silent.)
 *
 * A single global instance `Vision` is exported so callers do not need to manage
 * construction or pass pointers around.
 */

#ifndef VISION_H
#define VISION_H

#include <stdint.h>

#include "ConfigVision.h"

/**
 * @brief Mosaic-colour-identification facade over the HUE12 UART camera.
 *
 * @details
 * Owns the per-cell colour and raw-hue latch arrays, the fault state, and the registered
 * cooperative-tick callback used during the blocking `readMosaic()` spin. See the file
 * docblock for the overall usage pattern, the fault model, and the rationale for the
 * absent `update()` method.
 *
 * The constructor zeroes the latch arrays and marks the module as not-yet-read, so every
 * getter is safe to call before the first successful `readMosaic()` — it returns the
 * documented safe sentinel.
 */
class VisionClass
{
public:
    // -----------------------------------------------------------------------------------------
    // Fault types
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Source of a Vision-layer fault.
     */
    enum class Fault : uint8_t
    {
        NONE         = 0,   ///< No fault — normal operation.
        READ_TIMEOUT = 1,   ///< UART read did not complete within `Config::VISION_READ_TIMEOUT_MS`.
        BAD_CHECKSUM = 2,   ///< Received frame failed its integrity check.
        BAD_CELL     = 3    ///< An active mosaic cell classified as UNKNOWN (device NONE/Black or uncalibrated band).
    };

    /**
     * @brief Captured details of the last Vision fault.
     */
    struct FaultInfo
    {
        Fault   kind         = Fault::NONE;   ///< Which failure occurred.
        uint8_t firstBadCell = 0xFF;          ///< First out-of-range cell index (`BAD_CELL` only; 0xFF if unused).
        uint8_t rawValue     = 0;             ///< Raw byte that caused the fault (BAD_CHECKSUM / BAD_CELL).
    };

    // -----------------------------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Construct the facade with the latch uninitialised.
     *
     * Zeroes both latch arrays and marks `_hasRead = false` so getters return safe sentinels
     * before the first successful `readMosaic()`. Does NO hardware work.
     */
    VisionClass();

    /**
     * @brief Initialise the HUE12 UART link and clear all latched state.
     *
     * Opens `Serial1` at `Config::VISION_UART_BAUD`, resets the colour and hue latches to
     * their default values, clears `_hasRead`, and clears any latched fault. Must be called
     * exactly once from `setup()` after `MiniR4.begin()` has returned.
     *
     * @return `true` unconditionally (UART open is always successful on this platform).
     *         Declared `bool` to match the `begin()` convention of sibling modules
     *         (`Drive`, `Manipulator`) so the call site can guard on the return value if
     *         the platform is ever changed.
     * @note Does NOT flush any pending UART bytes — the camera may already be streaming.
     *       `readMosaic()` performs its own flush-and-discard sequence before latching.
     */
    bool begin();

    // -----------------------------------------------------------------------------------------
    // Command
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Capture one mosaic frame from the HUE12 camera and latch the results.
     *
     * Flushes stale UART bytes, discards `Config::VISION_FLUSH_DISCARD_FRAMES` frames to
     * let the camera settle on the current field of view, then reads and decodes one complete
     * frame into the internal colour and hue latch arrays.
     *
     * @return `true` if the frame was received and decoded without error; `false` if any
     *         fault occurred. On `false` the latch contents are **not** updated — they
     *         retain the values from the most recent successful read (or the zero-defaults if
     *         no successful read has occurred). The fault is recorded in `_fault` and
     *         survives until `clearFault()` is called.
     *
     * @note **The chassis MUST be fully idle before calling this method.** The HUE12
     *       camera streams continuously; a frame captured while the robot is moving contains
     *       motion-time colour data that does not represent the robot's current field
     *       position. Always ensure `Drive.isIdle()` returns `true` before calling
     *       `readMosaic()`. Calling from inside a moving `cooperativeTick()` chain produces
     *       unreliable results.
     *
     * @note **This is a command-time BLOCKING call** (ADR-003 / ADR-037 sanctioned blocking
     *       site). It spins on the registered tick callback (see `setTickCallback()`) while
     *       waiting for UART bytes. The spin duration is bounded by
     *       `Config::VISION_READ_TIMEOUT_MS`; if that deadline is exceeded the method returns
     *       `false` with `Fault::READ_TIMEOUT`. Do NOT call from inside any module's
     *       `update()` method.
     */
    bool readMosaic();

    // -----------------------------------------------------------------------------------------
    // Getters (pure, safe-sentinel)
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Classified colour for mosaic cell @p cell.
     *
     * Returns the colour latched by the most recent successful `readMosaic()` call.
     *
     * @param cell Zero-based cell index in `[0, Config::VISION_CELL_COUNT)`.
     * @return The `Config::Color` for the requested cell. Returns `Config::Color::UNKNOWN`
     *         if @p cell is out of range **or** if no successful `readMosaic()` has occurred
     *         yet (ADR-028 bounds-checking precedent — out-of-range never traps; it returns
     *         a defined safe sentinel).
     * @note Pure — does not affect any state, the fault latch, or the UART link.
     */
    Config::Color getColor(uint8_t cell) const;

    /**
     * @brief Raw hue byte for mosaic cell @p cell, as decoded from the HUE12 frame.
     *
     * Returns the hue value latched by the most recent successful `readMosaic()` call.
     * The hue axis is 0..239 for chromatic colours; device-side achromatic White is
     * reported as 240..245.
     *
     * @param cell Zero-based cell index in `[0, Config::VISION_CELL_COUNT)`.
     * @return The raw hue byte for the requested cell. Returns `0` if @p cell is out of
     *         range **or** if no successful `readMosaic()` has occurred yet (ADR-028
     *         bounds-checking precedent — out-of-range never traps; it returns a defined
     *         safe sentinel).
     * @note Pure — does not affect any state, the fault latch, or the UART link.
     */
    uint8_t getRawHue(uint8_t cell) const;

    // -----------------------------------------------------------------------------------------
    // Fault surface (mirrors Drive / PathTracker)
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Whether a fault is currently latched.
     * @return `true` if `_fault.kind != Fault::NONE`.
     * @note The fault persists until `clearFault()` is called. A faulted module does not
     *       retry automatically; the caller is responsible for deciding how to recover.
     */
    bool hasFault() const;

    /**
     * @brief Snapshot of the latched fault details.
     *
     * @return The current `FaultInfo`. Fields other than `kind` are only meaningful when
     *         `hasFault()` returns `true`; their values when `kind == Fault::NONE` are
     *         the zero-defaults from `FaultInfo`'s in-class initialiser.
     * @note Does NOT clear the fault — call `clearFault()` explicitly when ready to retry.
     */
    FaultInfo getFault() const;

    /**
     * @brief Clear the latched fault so the module is ready for the next `readMosaic()`.
     *
     * @note Does NOT modify the colour / hue latch — the last successfully-read values
     *       remain accessible after clearing a fault. Only the fault record is reset.
     * @note Does NOT restart any operation; the caller is responsible for re-issuing
     *       `readMosaic()` once the fault condition has been addressed.
     */
    void clearFault();

    // -----------------------------------------------------------------------------------------
    // Callback
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Register a function to call each iteration of `readMosaic()`'s blocking spin.
     *
     * `readMosaic()` spins waiting for UART bytes; registering `cooperativeTick()` (from the
     * sketch) here keeps every other module alive during that wait — otherwise `EventTrace`
     * stops rendering, etc. The callback is invoked once per spin iteration for the duration
     * of the UART read.
     *
     * Call this exactly once from `setup()`, after every module's `begin()`.
     *
     * @param cb Pointer to the cooperative-tick function. Pass `nullptr` to clear.
     * @note If no callback is registered, the blocking spin calls nothing except the UART
     *       poll — other modules freeze for up to `Config::VISION_READ_TIMEOUT_MS`. Register
     *       the callback to avoid this degradation.
     * @warning The registered callback's call chain MUST NOT call `readMosaic()`. Doing so
     *          would re-enter the blocking spin from inside itself and recurse indefinitely.
     *          Same contract as `Drive::setTickCallback` (ADR-016). Today no `update()`
     *          does this; any future tick that needs a vision read must defer it to the next
     *          mission step.
     */
    void setTickCallback(void (*cb)());

private:
    Config::Color _colors[Config::VISION_CELL_COUNT];   ///< Per-cell colour latch. Zeroed by ctor; updated only on a successful read.
    uint8_t       _rawHue[Config::VISION_CELL_COUNT];   ///< Per-cell raw hue latch. Zeroed by ctor; updated only on a successful read.
    bool          _hasRead      = false;                 ///< True once at least one successful `readMosaic()` has returned. Guards safe-sentinel paths in getters.
    FaultInfo     _fault;                                ///< Latched fault from the most recent failed `readMosaic()`. Cleared by `clearFault()`.
    void (*_tickCallback)() = nullptr;                   ///< Optional cooperative-tick callback invoked from the `readMosaic()` spin. `nullptr` until `setTickCallback()` is called.
};

/**
 * @brief Single global instance — defined in Vision.cpp.
 */
extern VisionClass Vision;

#endif    // VISION_H
