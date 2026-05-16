/**
 * @file WRO2026.ino
 * @brief WRO 2026 Mosaic Masters — PR3 entry sketch.
 * @author WRO 2026
 *
 * @details
 * PR3 deliverable: human-typed mission DSL on top of PR2's PathTracker.
 *
 * Operator flow:
 *   1. Power on. Module begin()s run; OLED shows `BOOT OK` (or `BOOT FAIL`).
 *   2. Place chassis on `Routes::START_NODE` (`A`) facing `START_HEADING_DEG`.
 *   3. Press `BTN_UP` once — this zeros the IMU yaw offset, then calls
 *      `Mission.start()`. The mission runs asynchronously from `loop()`.
 *   4. Serial prints state-change events + periodic `HB t=... yaw=... node=...`
 *      heartbeats. OLED scrolls the same events.
 *   5. On PathTracker fault (UNREACHABLE / DRIVE_FAULT / ...): the chassis
 *      stops, the buzzer emits 5 × 500 ms tones at 1 kHz, the RGB LEDs
 *      flash red. Press `BTN_DOWN` to acknowledge and reset.
 *
 * Author the mission inline below — one `Mission.<verb>(...)` per line.
 * Available verbs: `goTo`, `wait`, `print`, `beep`, `led`. See Mission.h.
 *
 * @note No business logic in `loop()`. Every module's `update()` is
 *       non-blocking; the loop just dispatches them via `cooperativeTick()`.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "FieldMap.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "Manipulator.h"
#include "Mission.h"
#include "PathTracker.h"

namespace
{
    bool _setupFailed = false;
    bool _missionArmed = false;
    bool _btnUpPrev    = false;
}    // namespace

/**
 * @brief Cooperative tick — every module's `update()` exactly once.
 *
 * Used as the body of `loop()` and registered with both
 * `Drive.setTickCallback()` and `PathTracker.setTickCallback()` so any
 * sync-blocking drain spin keeps everyone alive.
 *
 * Order: Drive -> LineFollower -> Intersection -> Manipulator
 *     -> PathTracker -> Mission -> EventTrace.
 */
void cooperativeTick()
{
    Drive.update();
    LineFollower.update();
    Intersection.update();
    Manipulator.update();
    PathTracker.update();
    Mission.update();
    EventTrace.update();
}

/**
 * @brief Hand-typed PR3 mission. Edit this function to change tomorrow's run.
 *
 * Every `Mission.<verb>(...)` enqueues a step. `Mission.start()` arms the
 * queue and returns immediately; `loop()` drains it via `Mission.update()`.
 */
void buildMission()
{
    Mission
        .print("MISSION GO")
        .led(0, 0, 255, 200)        // brief blue flash on arm
        .goTo("B")
        .beep(1000, 150)
        .print("AT B")
        .wait(500)
        .goTo("G")                  // blind hop from B to G
        .beep(1500, 300)
        .led(0, 255, 0, 500)
        .print("AT G")
        .wait(800)
        .goTo("B")                  // back to the line
        .goTo("C")
        .beep(800, 150)
        .print("AT C")
        .wait(500)
        .goTo("D")
        .print("AT D")
        .wait(500)
        .goTo("A")
        .led(255, 255, 255, 1000)
        .print("DONE");
    Mission.start();
}

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    const bool driveOk = Drive.begin();
    const bool lineOk  = LineFollower.begin();
    const bool intOk   = Intersection.begin();
    const bool manipOk = Manipulator.begin();
    const bool locOk   = Localization.begin();
    const bool pathOk  = PathTracker.begin();
    const bool missOk  = Mission.begin();
    const bool traceOk = EventTrace.begin();

    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);

    _setupFailed = !(driveOk && lineOk && intOk && manipOk && locOk && pathOk && missOk && traceOk);

    if (_setupFailed)
    {
        Serial.println(F("WRO2026 PR3 BOOT FAIL — module begin() returned false:"));
        if (!driveOk)  { Serial.println(F("  - Drive"));        EventTrace.pushEvent("FAIL DRIVE"); }
        if (!lineOk)   { Serial.println(F("  - LineFollower")); EventTrace.pushEvent("FAIL LINE"); }
        if (!intOk)    { Serial.println(F("  - Intersection")); EventTrace.pushEvent("FAIL INT"); }
        if (!manipOk)  { Serial.println(F("  - Manipulator"));  EventTrace.pushEvent("FAIL MANIP"); }
        if (!locOk)    { Serial.println(F("  - Localization")); EventTrace.pushEvent("FAIL LOC"); }
        if (!pathOk)   { Serial.println(F("  - PathTracker"));  EventTrace.pushEvent("FAIL PATH"); }
        if (!missOk)   { Serial.println(F("  - Mission"));      EventTrace.pushEvent("FAIL MIS"); }
        if (!traceOk)  { Serial.println(F("  - EventTrace"));   EventTrace.pushEvent("FAIL TRACE"); }
        EventTrace.pushEvent("BOOT FAIL");
    }
    else
    {
        EventTrace.pushEvent("BOOT OK");
        Serial.println(F("WRO2026 PR3 ready. Place at A, then press BTN_UP."));
    }
}

void loop()
{
    cooperativeTick();

    if (_setupFailed)
    {
        return;
    }

    // BTN_UP rising edge arms the mission exactly once. The press captures
    // the IMU yaw NOW (operator has placed the chassis on `Routes::START_NODE`
    // facing `START_HEADING_DEG`); subsequent presses do nothing.
    //
    // `_missionArmed` is set ONLY on a successful zero so a transient
    // failure does not permanently lock the operator out of retrying.
    const bool btnUpNow = MiniR4.BTN_UP.getState();
    if (btnUpNow && !_btnUpPrev && !_missionArmed)
    {
        if (!Localization.zeroHeadingFromCurrent())
        {
            EventTrace.pushEvent("ZERO FAIL");
            Serial.println(F("[main] zeroHeadingFromCurrent() failed; press BTN_UP again to retry"));
        }
        else
        {
            _missionArmed = true;
            EventTrace.pushEvent("NAV ZERO");
            buildMission();
        }
    }
    _btnUpPrev = btnUpNow;
}
