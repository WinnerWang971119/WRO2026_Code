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

#include "../../config/ConfigVision.h"

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
     * @brief Construct the facade with the latch cleared to safe defaults (no read yet).
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
     * @note Sends the HUE12 wake command via `start()` at the end of `begin()` so a
     *       cold-booted camera begins emitting frames.
     */
    bool begin();

    /**
     * @brief Send the wake/start command to the HUE12 over Serial1.
     *
     * Writes `Config::VISION_START_CMD` to `Serial1`, kicking a camera that has not yet begun
     * emitting frames. The HUE12 belongs to the same device family as the vendor
     * `MiniR4SmartCamReader`, which writes the same command when it has received no valid data.
     *
     * Called once by `begin()`; also callable any time the camera needs re-kicking (e.g. before a
     * `readMosaic()` after a long idle).
     *
     * @note Pure TX — does NOT read, flush, or touch the colour / hue latch or the fault record.
     *       Safe to call repeatedly; an already-streaming camera ignores a redundant wake.
     */
    void start();

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
     *       waiting for UART bytes. Each read attempt is bounded by
     *       `Config::VISION_READ_TIMEOUT_MS`; if that deadline is exceeded the attempt fails with
     *       `Fault::READ_TIMEOUT`. Because a transport fault retries ONCE, the worst-case
     *       blocking time is the FULL retry budget — up to the maximum attempt count multiplied by
     *       `Config::VISION_READ_TIMEOUT_MS` (2 × 120 ms = 240 ms at today's config), not a single
     *       timeout. Do NOT call from inside any module's `update()` method.
     */
    bool readMosaic();

    /**
     * @brief DIAGNOSTIC read: latch every cell's classification, never failing on UNKNOWN cells.
     *
     * Same blocking UART read as `readMosaic()`, but with NO strict guard and NO mask cap — every
     * cell is classified and latched, an unreadable cell simply latching as `Config::Color::UNKNOWN`
     * (reported truthfully by `getColor()`). It fails ONLY on a transport fault (READ_TIMEOUT /
     * BAD_CHECKSUM), never `BAD_CELL`. This is what the operator console's VISION VIEW / VISION TEST
     * screens use so they can SHOW which cells read UNKNOWN — the strict `readMosaic()` throws the
     * whole frame away on the first UNKNOWN and never latches per-cell colours (ADR-037), which is
     * correct for the mission but useless for a diagnostic that must display the bad cell.
     *
     * @return `true` on a valid latched frame (even with UNKNOWN cells); `false` on a transport
     *         fault, with the matching `Fault` recorded. Same idle-chassis / blocking-call notes as
     *         `readMosaic()` apply.
     */
    bool readMosaicRaw();

    // -----------------------------------------------------------------------------------------
    // Injection (bench-test / camera-failure fallback seam)
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Latch a caller-supplied mosaic as if `readMosaic()` had succeeded — NO camera read.
     *
     * Copies @p colors into the colour latch, fills the raw-hue latch with a representative hue per
     * colour (the midpoint of the matching `Config::HUE_BANDS` entry; colours with no chromatic band
     * — device-achromatic `WHITE`, the bench-TODO `RED`, and `UNKNOWN` — latch
     * `Config::VISION_RAW_HUE_NONE`), marks the module read (`_hasRead = true`), and clears any
     * latched fault. `getColor()` / `getRawHue()` then return the injected frame exactly as they
     * would after a real read, so the whole downstream mission (mosaic sort, …) runs unchanged
     * without ever driving to an observation site or reading the HUE12.
     *
     * Two intended uses:
     *   1. **Bench testing** — skip the drive-to-spot + real read; hand the mission a canned mosaic
     *      (`Vision.fakeRead({RED, BLUE, GREEN, ...})`).
     *   2. **Camera-failure fallback** — inject a mosaic read off the field by eye when the HUE12 is
     *      dead, so a colour-consuming mission can still run.
     *
     * @param colors The `Config::VISION_CELL_COUNT` cell colours to latch, cell-indexed (0..N-1).
     *               A braced temporary binds directly (`fakeRead({...})`); a named array works too.
     * @return `true` always — an explicit injection cannot "fail". The caller vouches for @p colors,
     *         so this does NOT apply the strict UNKNOWN guard `readMosaic()` uses: an injected
     *         `UNKNOWN` cell is latched and reported truthfully rather than faulting.
     *
     * @note **Explicit-only — never an automatic fallback.** Nothing in `readMosaic()` calls this;
     *       a real read still faults loudly on failure (ADR-037). Silently substituting fake colour
     *       would let the robot sort with false confidence, so the substitution stays a deliberate
     *       operator call.
     * @note Unlike `readMosaic()` this touches NO UART and does not block. There is no camera frame
     *       to be corrupted by motion, so the idle-chassis precondition does not apply.
     */
    bool fakeRead(const Config::Color (&colors)[Config::VISION_CELL_COUNT]);

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
     * @return The raw hue byte for the requested cell. Returns `Config::VISION_RAW_HUE_NONE`
     *         (`0xFF`) if @p cell is out of range **or** if no successful `readMosaic()` has
     *         occurred yet (ADR-028 bounds-checking precedent — out-of-range never traps; it
     *         returns a defined safe sentinel). This sentinel never collides with a genuinely
     *         latched hue — a raw hue of `255` (the device's own "no detection" code) always
     *         classifies as `Color::UNKNOWN`, which rejects the whole candidate frame before
     *         `_rawHue` is ever committed.
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
    // -----------------------------------------------------------------------------------------
    // Private read-loop helpers (decomposed out of `readMosaic()` for readability — behaviour is
    // identical to the original monolithic loop). These are hardware-facing UART glue; the pure,
    // host-testable parse / unpack / classify logic lives in `VisionFrame.h` (ADR-030).
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Drain every currently-available byte from `Serial1`'s RX queue (read-and-discard).
     *
     * Called at the start of each read attempt to flush the streamed backlog the HUE12 emitted
     * while the robot was moving, so the latched frame begins strictly after the flush.
     *
     * @note Pure RX side effect — does not touch the latch, the fault record, or the deadline.
     */
    void _flushRx();

    /**
     * @brief One iteration of the bounded wait between RX polls.
     *
     * Invokes the registered cooperative-tick callback if one is set; otherwise emits the
     * one-time "no tick callback" warning. In BOTH cases it then sleeps `kWaitGranularityMs`
     * via `delay()` (LOAD-BEARING: on the host mock `millis()` advances only inside `delay()`,
     * and on hardware the tick re-entry keeps the OLED / heartbeat alive).
     *
     * @note Owns the one-shot `noCallbackWarned` static guard for the no-callback warning.
     */
    void _waitTick();

    /**
     * @brief Map → classify → strict-guard a candidate frame's words into LOCAL cell arrays.
     *
     * Unpacks @p words into per-slot hue / sta, then for every cell maps `VISION_CELL_TO_SLOT[c]`
     * and classifies it into @p outColors / @p outHue. Applies the strict guard: the FIRST cell
     * that classifies as `Config::Color::UNKNOWN` aborts the scan, records its index in
     * @p firstBadCell and the raw mapped-slot hue in @p badRaw, and returns `false` WITHOUT a
     * complete local result. The caller (`readMosaic()`) commits the latch only on `true`, so the
     * no-overwrite-on-failure invariant stays visible at the call site.
     *
     * @param words        The `Config::VISION_FRAME_WORDS` decoded words from `parseFrame()`.
     * @param outColors    Receives the per-cell classified colours (only fully written on `true`).
     * @param outHue       Receives the per-cell raw hues (only fully written on `true`).
     * @param firstBadCell Set to the first offending CELL index on `false`; untouched on `true`.
     * @param badRaw       Set to the offending mapped-slot raw hue on `false`; untouched on `true`.
     * @param tolerant     When `false`, the strict guard applies (first UNKNOWN aborts). When
     *                     `true`, UNKNOWN cells are latched honestly as UNKNOWN and the scan only
     *                     fails if MORE than @p maxUnknown cells are UNKNOWN.
     * @param maxUnknown   Maximum UNKNOWN cells tolerated in the tolerant path (ignored when
     *                     `tolerant` is `false`).
     * @return `true` if the frame classified within budget; `false` on the strict-guard first
     *         UNKNOWN, or on the tolerant path when the UNKNOWN count exceeds @p maxUnknown.
     * @note Pure with respect to member state — writes only the out-parameters, never the latch.
     */
    bool _classifyCandidate(
        const uint16_t words[Config::VISION_FRAME_WORDS],
        Config::Color  outColors[Config::VISION_CELL_COUNT],
        uint8_t        outHue[Config::VISION_CELL_COUNT],
        uint8_t&       firstBadCell,
        uint8_t&       badRaw,
        bool           tolerant,
        uint8_t        maxUnknown) const;

    /**
     * @brief Shared read loop behind `readMosaic()` (strict/mask) and `readMosaicRaw()` (diagnostic).
     * @param diagnostic `true` for `readMosaicRaw()` — always tolerant, no UNKNOWN cap. `false` for
     *                   `readMosaic()` — tolerance follows `Config::VISION_MASK_UNKNOWN_ENABLED` and
     *                   the `Config::VISION_MASK_MAX_UNKNOWN_CELLS` cap.
     * @return As the public wrappers document.
     */
    bool _readMosaic(bool diagnostic);

    /**
     * @brief Representative raw hue for a fake-injected cell of colour @p color (used by `fakeRead`).
     *
     * Returns the midpoint of the matching `Config::HUE_BANDS` entry so a fake cell's `getRawHue()`
     * stays consistent with `VisionFrame::classifyHue()` for the banded colours (YELLOW / GREEN /
     * BLUE). Colours with no chromatic band — device-achromatic `WHITE`, the bench-TODO `RED`, and
     * `UNKNOWN` — have no representative hue and return `Config::VISION_RAW_HUE_NONE`.
     *
     * @param color The injected cell colour.
     * @return A band-midpoint hue for a banded colour, else `Config::VISION_RAW_HUE_NONE`.
     * @note Pure and stateless — declared `static`.
     */
    static uint8_t _canonicalHue(Config::Color color);

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
