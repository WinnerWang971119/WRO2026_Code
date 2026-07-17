/**
 * @file EventTrace.h
 * @brief Singleton ring-buffer of recent event strings rendered on the on-board OLED.
 * @author WRO 2026
 *
 * @details
 * `EventTraceClass` is a tiny telemetry sink that owns a fixed-capacity ring buffer of short
 * status strings and renders the most recent entries on the MATRIX Mini R4's 128x32 OLED. It
 * exists so any subsystem can produce a one-line "I just transitioned" record (e.g. line lost,
 * turn finished) without having to know about the OLED, fonts, or layout.
 *
 * Design rules:
 *  - **Bounded ring buffer.** Exactly `Config::EVENT_TRACE_CAPACITY` slots; older events drop
 *    off. The capacity is sized to fit the OLED screen at text size 1 (8-px-tall lines, 4 lines
 *    of 128x32 = 4 entries) — see the per-member docblocks for the geometry rationale.
 *  - **Edge-triggered polling of source modules.** `update()` polls Drive and LineFollower
 *    for *transitions* in their public state, not for steady-state values. This keeps the OLED
 *    log free of duplicate spam while a condition persists.
 *      - LineFollower: level signal `isLineLost()` → derive rising/falling edges locally.
 *      - Drive:        level signal `isTurnDone()` → derive a rising edge locally so a single
 *                      `TURN END` is logged when an async turn completes.
 *      - Intersection: NOT traced. EventTrace does not poll `Intersection.getEdgeCount()`, so
 *                      junction crossings never reach the OLED ring. The navigation layer owns
 *                      the read-and-clear intersection API; surfacing it here is intentionally
 *                      out of scope.
 *  - **Dirty-flag rendering.** The OLED is repainted only when `_dirty == true`, i.e. when a
 *    new event has actually been pushed (or `clear()` was called). This avoids burning I2C
 *    bandwidth on every loop tick.
 *  - **Non-blocking.** `update()` contains NO `delay()` and never spins on I/O — it is safe to
 *    call once per main-loop iteration after the source modules have already updated.
 *
 * Lifecycle (see WRO2026.ino):
 *  - `EventTrace.begin()` once in `setup()` after `MiniR4.begin()` and after each source
 *    module's own `begin()` has been called.
 *  - `EventTrace.update()` last in the main `loop()`, AFTER Drive/LineFollower have updated
 *    their state for this tick.
 *
 * @note OLED is 128x32. Adafruit GFX text size 1 = 8-px-tall lines = 4 lines fit.
 *       `EVENT_TRACE_CAPACITY = 4` is sized for this — do not increase it without also
 *       changing the line-height accounting in `_render()`.
 */

#ifndef EVENTTRACE_H
#define EVENTTRACE_H

#include <Arduino.h>

#include "../../Config.h"

/**
 * @brief Singleton OLED event log with a fixed-size ring buffer.
 *
 * Owns the buffer storage, the head/count indices, and the previous-state cache used for edge
 * detection on the source modules. All public methods are non-blocking and free of `delay()`.
 */
class EventTraceClass
{
public:
    /** @brief Width budget per OLED line in characters, including the trailing NUL. */
    static constexpr uint8_t MAX_LINE_LEN = 20;

    /** @brief Pixel height of one text-size-1 line on the Adafruit GFX font. */
    static constexpr uint8_t LINE_HEIGHT_PX = 8;

    /**
     * @brief Initialise the ring buffer and edge-detection cache.
     *
     * @return `true` on success. Currently always succeeds — the method returns `bool` for
     *         symmetry with every other `*::begin()` in this codebase so a future failure path
     *         (e.g. defensive OLED handshake) can be added without touching call sites.
     * @note Zeroes out the ring buffer storage, resets `_head` and `_count`, and primes the
     *       edge-detection cache so the first `update()` does not synthesise spurious events
     *       from uninitialised state. Must be called from `setup()` after `MiniR4.begin()`.
     */
    bool begin();

    /**
     * @brief Cooperative tick — call once per main-loop iteration after source modules update.
     *
     * Polls each source module for state transitions and pushes a short label for every edge
     * detected, then repaints the OLED if anything new arrived.
     *
     * @note Non-blocking. Polls LineFollower (level -> edge) and Drive (level -> edge).
     *       Intersection is NOT polled — junctions are not traced. Renders only when `_dirty`.
     * @note Must be called AFTER Drive/LineFollower `update()` have run for this tick, so the
     *       levels we sample are this tick's truth.
     */
    void update();

    /**
     * @brief Push an arbitrary message into the ring buffer (ad-hoc events from any caller).
     *
     * @param msg NUL-terminated C string. `nullptr` and the empty string are silently ignored.
     *            Strings longer than `MAX_LINE_LEN - 1` characters are truncated and explicitly
     *            NUL-terminated.
     * @note Bounded ring buffer of `Config::EVENT_TRACE_CAPACITY` entries. Older events drop
     *       off. Marks the trace as dirty so the next `update()` repaints the OLED.
     */
    void pushEvent(const char* msg);

    /**
     * @brief Wipe every stored event and clear the OLED on the next render.
     *
     * @note Resets `_head` and `_count`, zeroes the buffer storage, and sets the dirty flag so
     *       `update()` repaints (a now-empty) screen. Does NOT touch the edge-detection cache —
     *       a line that was already lost stays "lost" from the detector's point of view, so the
     *       next genuine recovery still fires `LINE OK`.
     */
    void clear();

    /**
     * @brief Enable or disable the periodic Serial heartbeat line.
     *
     * @param enabled `true` (the default) emits the `HB t=... yaw=... node=...` liveness line
     *        every `Config::MISSION_HEARTBEAT_MS`; `false` suppresses it. Event-ring pushes and
     *        OLED rendering are unaffected — only the Serial heartbeat is gated.
     * @note For bench/diagnostic runs whose own Serial output would otherwise be interleaved with
     *       heartbeat lines. Runtime-only, not persisted: a reset restores the default (enabled).
     */
    void setHeartbeatEnabled(bool enabled) { _heartbeatEnabled = enabled; }

private:
    /**
     * @brief Ring buffer of fixed-width NUL-terminated event strings.
     *
     * Indexed `_buf[slot][char]`. Each slot stores up to `MAX_LINE_LEN - 1` characters plus the
     * trailing NUL. Newest event is at `_buf[(_head - 1) mod CAPACITY]`.
     */
    char _buf[Config::EVENT_TRACE_CAPACITY][MAX_LINE_LEN];

    uint8_t _head     = 0;      ///< Next write index into `_buf`.
    uint8_t _count    = 0;      ///< Number of valid entries (0..EVENT_TRACE_CAPACITY).
    bool    _dirty    = true;   ///< Set by `pushEvent` / `clear`; cleared by render in `update()`.

    // -------------------------------------------------------------------------------------
    // Edge-detection cache for source modules (level -> edge).
    // -------------------------------------------------------------------------------------

    /** @brief Previous-tick value of `LineFollower.isLineLost()`. */
    bool _prevLineLost = false;

    /**
     * @brief Previous-tick value of `Drive.isTurnDone()`.
     *
     * Initialised `true` because the drivebase boots idle (no turn pending), so the first
     * `update()` does not falsely synthesise a `TURN END` event.
     */
    bool _prevTurnDone = true;

    /**
     * @brief Last millis() at which a heartbeat status line was emitted to Serial.
     *
     * PR3 ships a periodic `HB t=... yaw=... node=...` line so the
     * operator's serial console always shows liveness even when no
     * state transition has fired. Period: `Config::MISSION_HEARTBEAT_MS`.
     */
    uint32_t _lastHeartbeat_ms = 0;

    /** @brief When false, `_emitHeartbeat()` suppresses the periodic Serial line. Default true. */
    bool _heartbeatEnabled = true;

    /**
     * @brief Last offset magnitude (whole cm) pushed to the OLED ring, used for change-throttled
     *        OLED events.
     *
     * Initialised to `0` — matching the zero offset the chassis has at boot while it sits on its
     * anchored start node. This means `OFF 0` is never pushed at boot (the rounded magnitude
     * equals the sentinel, so no event fires). The first OLED event fires only when the chassis
     * is hand-driven at least 1 cm away from its anchored node, which is the desired behaviour
     * for bench telemetry. `int16_t` accommodates the full field diagonal (~400 cm) while still
     * fitting in a negative sentinel if the design ever needs one.
     */
    int16_t _lastShownOffsetCm = 0;

    /** @brief Emit the heartbeat line to Serial if the period has elapsed. */
    void _emitHeartbeat();

    /**
     * @brief Repaint the OLED with the current ring-buffer contents (newest at top).
     *
     * @note Walks the ring backwards from `_head` so the freshest event lands on line 0 and
     *       older events fall toward the bottom. Only rendered when `_dirty` is set — see
     *       `update()`.
     */
    void _render();
};

/**
 * @brief Single global instance — defined in EventTrace.cpp.
 */
extern EventTraceClass EventTrace;

#endif    // EVENTTRACE_H
