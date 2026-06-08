/**
 * @file EventTrace.h (CTPS HOST MOCK)
 * @brief No-op mock of the EventTrace OLED ring buffer for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `EventTrace.h` (quote-included; force-included by the T4
 * cmake wiring). The include guard is the REAL one (`EVENTTRACE_H`) so the real
 * header no-ops if reached afterwards.
 *
 * EventTrace is a telemetry sink, NOT a recording seam — its events are OLED
 * status strings, not command-stream entries — so every method is a no-op. Only
 * the surface Mission.cpp references is provided:
 *   - `begin()` / `pushEvent(const char*)` / `update()` / `clear()`
 *   - the `EventTraceClass::MAX_LINE_LEN` constant (Mission.cpp sizes
 *     `char buf[EventTraceClass::MAX_LINE_LEN]` from it).
 *
 * `MAX_LINE_LEN` matches the real value (20) so any buffer sized from it has the
 * same length as on the firmware target.
 */

#ifndef EVENTTRACE_H
#define EVENTTRACE_H

#include <stdint.h>

/**
 * @brief No-op mock of the OLED event-trace sink.
 */
class EventTraceClass
{
public:
    /** @brief Width budget per OLED line in characters (matches the real value). */
    static constexpr uint8_t MAX_LINE_LEN = 20;

    /** @brief Pixel height of one text-size-1 line (matches the real value). */
    static constexpr uint8_t LINE_HEIGHT_PX = 8;

    /** @brief No-op bring-up; always reports success. */
    bool begin() { return true; }

    /** @brief No-op cooperative tick. */
    void update() {}

    /** @brief No-op event push (events are OLED telemetry, not command-stream entries). */
    void pushEvent(const char* /*msg*/) {}

    /** @brief No-op clear. */
    void clear() {}
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern EventTraceClass EventTrace;

#endif    // EVENTTRACE_H
