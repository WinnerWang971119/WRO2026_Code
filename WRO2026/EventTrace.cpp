/**
 * @file EventTrace.cpp
 * @brief Implementation of the OLED event-trace singleton.
 * @author WRO 2026
 */

#include "EventTrace.h"

#include <MatrixMiniR4.h>    // MiniR4.OLED + WHITE color macro
#include <string.h>          // strncpy, memset

#include "Drive.h"
#include "FieldMap.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "routes.h"

/**
 * @brief Single global instance, declared `extern` in EventTrace.h.
 */
EventTraceClass EventTrace;

bool EventTraceClass::begin()
{
    _head  = 0;
    _count = 0;
    _dirty = true;

    // Zero the ring storage so a render before any pushEvent paints empty lines, not garbage.
    memset(_buf, 0, sizeof(_buf));

    // Prime the edge-detection cache so the first update() does not synthesise spurious events.
    _prevLineLost     = false;
    _prevTurnDone     = true;
    _prevIntEdgeCount = Intersection.getEdgeCount();
    _lastHeartbeat_ms = millis();

    return true;
}

void EventTraceClass::pushEvent(const char* msg)
{
    if (msg == nullptr || msg[0] == '\0')
    {
        return;
    }

    // Copy with truncation; force NUL terminator at the last writable byte to guarantee a valid
    // C string even if `msg` exactly fills (or overflows) the slot.
    strncpy(_buf[_head], msg, MAX_LINE_LEN - 1);
    _buf[_head][MAX_LINE_LEN - 1] = '\0';

    _head = (_head + 1) % Config::EVENT_TRACE_CAPACITY;

    if (_count < Config::EVENT_TRACE_CAPACITY)
    {
        _count++;
    }

    _dirty = true;
}

void EventTraceClass::clear()
{
    _head  = 0;
    _count = 0;
    memset(_buf, 0, sizeof(_buf));
    _dirty = true;
}

void EventTraceClass::update()
{
    // -----------------------------------------------------------------------------------------
    // Source poll 1 — LineFollower: level signal, derive edges locally.
    // -----------------------------------------------------------------------------------------
    const bool lostNow = LineFollower.isLineLost();
    if (lostNow != _prevLineLost)
    {
        pushEvent(lostNow ? "LINE LOST" : "LINE OK");
        _prevLineLost = lostNow;
    }

    // -----------------------------------------------------------------------------------------
    // Source poll 3 — Drive: level signal, fire on rising edge of `isTurnDone()`.
    // -----------------------------------------------------------------------------------------
    const bool doneNow = Drive.isTurnDone();
    if (doneNow && !_prevTurnDone)
    {
        pushEvent("TURN END");
    }
    _prevTurnDone = doneNow;

    // -----------------------------------------------------------------------------------------
    // Heartbeat: periodic status line on Serial so the operator always sees liveness.
    // -----------------------------------------------------------------------------------------
    _emitHeartbeat();

    // -----------------------------------------------------------------------------------------
    // Render only when something new arrived this tick.
    // -----------------------------------------------------------------------------------------
    if (_dirty)
    {
        _render();
        _dirty = false;
    }
}

void EventTraceClass::_emitHeartbeat()
{
    const uint32_t now = millis();
    if (now - _lastHeartbeat_ms < Config::MISSION_HEARTBEAT_MS)
    {
        return;
    }
    _lastHeartbeat_ms = now;

    const uint8_t node = Localization.getCurrentNode();
    const char*   name = (node < Routes::N_NODES) ? Routes::NODES[node].name : "?";

    Serial.print(F("HB t="));
    Serial.print(now);
    Serial.print(F(" yaw="));
    Serial.print(Localization.getHeadingDeg(), 1);
    Serial.print(F(" node="));
    Serial.println(name);
}

void EventTraceClass::_render()
{
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setTextSize(1);
    MiniR4.OLED.setTextColor(WHITE);

    // Walk the ring backwards so the newest event lands on line 0 (top of the screen) and older
    // events fall toward the bottom.
    for (uint8_t i = 0; i < _count; i++)
    {
        const uint8_t idx =
            (_head + Config::EVENT_TRACE_CAPACITY - 1 - i) % Config::EVENT_TRACE_CAPACITY;
        MiniR4.OLED.setCursor(0, i * LINE_HEIGHT_PX);
        MiniR4.OLED.println(_buf[idx]);
    }

    MiniR4.OLED.display();
}
