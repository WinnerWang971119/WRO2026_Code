/**
 * @file EventTrace.cpp
 * @brief Implementation of the OLED event-trace singleton.
 * @author WRO 2026
 */

#include "EventTrace.h"

#include <MatrixMiniR4.h>    // MiniR4.OLED + WHITE color macro
#include <math.h>            // sqrtf
#include <stdio.h>           // snprintf
#include <string.h>          // strncpy, memset

#include "../drive/Drive.h"
#include "../nav/FieldMap.h"
#include "../drive/LineFollower.h"
#include "../nav/Localization.h"
#include "../nav/routes.h"

namespace
{
/// Heartbeat heading is treated as stale if no successful live read has refreshed the
/// Localization cache within this window (~3 heartbeats at the 500 ms cadence). A stale
/// value is flagged so a frozen-but-plausible cached heading (idle, between hops, or a
/// failing IMU) is not read as a current measurement.
constexpr uint32_t HEADING_STALE_MS = 1500;
}    // namespace

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
    // NOTE: EventTrace does NOT trace intersections — `update()` polls only LineFollower and
    // Drive. There is no Intersection edge cache to prime.
    _prevLineLost     = false;
    _prevTurnDone     = true;
    _lastHeartbeat_ms = millis();

    // Reset the change-throttle sentinel so a re-begin does not carry the prior run's last-shown
    // offset magnitude — which would defeat the documented boot-suppression of the `OFF 0` event.
    _lastShownOffsetCm = 0;

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
    // Offset magnitude — change-throttled OLED event (does NOT push on every tick).
    //
    // Rounds the 2-D Euclidean magnitude to whole centimetres and compares against the last
    // value shown. An event is pushed ONLY when the rounded value differs, so a stationary
    // chassis never floods the 4-slot ring. The `sqrtf` + cast to `int16_t` is the only
    // math; `getOffsetFromNode()` is a pure accessor with no I/O.
    //
    // Display granularity: 1 cm. Fine-grained enough to show a hand-push on the bench;
    // coarse enough that vibration / sensor noise does not spam the ring. Intentionally NOT
    // a Config knob — this is a presentation detail, not a tunable control parameter.
    // -----------------------------------------------------------------------------------------
    {
        const Offset2D  o      = Localization.getOffsetFromNode();
        const float     mag    = sqrtf(o.x * o.x + o.y * o.y);
        const int16_t   magCm  = static_cast<int16_t>(mag);    // truncate to whole cm
        if (magCm != _lastShownOffsetCm)
        {
            char label[MAX_LINE_LEN];
            snprintf(label, sizeof(label), "OFF %d", static_cast<int>(magCm));
            pushEvent(label);
            _lastShownOffsetCm = magCm;
        }
    }

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
    // Cached read: the heartbeat must not issue a live IMU/I2C transaction (EventTrace's
    // contract is non-blocking / no I/O). Navigation refreshes this cache via getHeadingDeg().
    Serial.print(Localization.getLastHeadingDeg(), 1);
    // Flag a stale cache (idle, between hops, or a failing IMU) so a frozen-but-plausible
    // heading is not read as current. No I/O — compares the cache's last-update stamp.
    const uint32_t headingUpdateMs = Localization.getLastHeadingUpdateMs();
    if (headingUpdateMs == 0 || (now - headingUpdateMs) > HEADING_STALE_MS)
    {
        Serial.print(F("?"));
    }
    // Pure accessor — no I/O. Offset magnitude in cm (one decimal place) surfaces how far
    // the chassis has been hand-driven from its last anchored node.
    const Offset2D off    = Localization.getOffsetFromNode();
    const float    offMag = sqrtf(off.x * off.x + off.y * off.y);
    Serial.print(F(" off="));
    Serial.print(offMag, 1);
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
