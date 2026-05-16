/**
 * @file WRO2026.ino
 * @brief WRO 2026 Mosaic Masters — Phase 2 PR2 entry sketch.
 * @author WRO 2026
 *
 * @details
 * Phase 2 PR2 deliverable: a thin entry that wires every module together. `setup()` follows
 * the documented MatrixMiniR4 initialisation order; `loop()` delegates to
 * `cooperativeTick()` — a clean tick across
 * Drive -> LineFollower -> Intersection -> Manipulator -> PathTracker -> EventTrace,
 * with NO business logic. The same `cooperativeTick()` is registered with both
 * `Drive.setTickCallback(...)` and `PathTracker.setTickCallback(...)` so that during a
 * sync-blocking spin (Drive's `_drainBlocking` or PathTracker's goTo() drain), every other
 * module continues to run.
 *
 * Define `NAV_DEMO_MODE` at compile time (e.g. via the IDE custom-build flags or by
 * uncommenting the line below) to enable the Phase 2 navigation demo: the operator
 * presses BTN_UP once, which captures the IMU yaw offset and runs the hardcoded sequence
 * `goTo("C")` then `goTo("G")`. The Phase 1 `DEMO_MODE` smoke (drive 30 cm + 90 deg turn)
 * is replaced — every primitive it exercised is exercised by the nav demo through
 * PathTracker -> Drive / LineFollower / Intersection.
 *
 * @note No business logic in `loop()`. Per ARCHITECTURE.md iron rule 1, every module's
 *       `update()` is non-blocking; the loop just dispatches them via `cooperativeTick()`.
 * @note The cooperative tick order matters — see `cooperativeTick()` doxygen. PathTracker
 *       is AFTER Intersection so its read-and-clear edge consumption sees this tick's event.
 */

// #define NAV_DEMO_MODE   // Uncomment to enable the BTN_UP nav demo.

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "FieldMap.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "Manipulator.h"
#include "PathTracker.h"

namespace
{
    /**
     * @brief Latches `true` if any module's `begin()` returned `false` during `setup()`.
     *        When set, `loop()` suppresses every motion-issuing path so a partially-
     *        initialised robot does not move with stale tuning, an unbound tracer, or a
     *        never-OLED'd display.
     *
     * @note The cooperative tick still runs so the OLED keeps rendering the `BOOT FAIL`
     *       trace lines — the operator needs to see what went wrong.
     */
    bool _setupFailed = false;
}    // namespace

#ifdef NAV_DEMO_MODE
namespace
{
    bool _navArmed = false;    ///< Latches the one-shot demo so re-presses do nothing.
    bool _btnPrev  = false;    ///< Previous BTN_UP sample, for rising-edge detection.
}    // namespace
#endif    // NAV_DEMO_MODE

/**
 * @brief Cooperative tick — calls every module's `update()` exactly once.
 *
 * Used as the body of `loop()` AND as the registered tick callback for both
 * `Drive.setTickCallback()` and `PathTracker.setTickCallback()` so every other module
 * continues to tick during either sync-blocking spin.
 *
 * Order matters:
 *  - `Drive.update()`        — flushes any completed bounded-task flags first so downstream
 *                              modules see fresh state.
 *  - `LineFollower.update()` — reads the tracer once and computes the corr.
 *  - `Intersection.update()` — reuses the same tracer read on the same tick.
 *  - `Manipulator.update()`  — currently a stub; kept in the chain so Phase 3 just fills it
 *                              in without touching this file.
 *  - `PathTracker.update()`  — advances the per-edge FSM. AFTER Intersection so its
 *                              `isAtIntersection()` consumes this tick's edge.
 *  - `EventTrace.update()`   — samples this tick's truth from each module above.
 */
void cooperativeTick()
{
    Drive.update();
    LineFollower.update();
    Intersection.update();
    Manipulator.update();
    PathTracker.update();
    EventTrace.update();
}

/**
 * @brief Arduino entry: bring up the lower MCU and every WRO module in dependency order.
 *
 * Order is fixed and must not be reshuffled:
 *   1. `MiniR4.begin()`                 — lower MCU, IMU, motors, servos, OLED.
 *   2. `Serial.begin(115200)`           — telemetry console.
 *   3. `MiniR4.PWR.setBattCell(...)`    — must match the physical pack.
 *   4. `Drive.begin()`                  — drivebase + PIDs + reverse flags.
 *   5. `LineFollower.begin()`           — MXLineTracer + outer-loop PID.
 *   6. `Intersection.begin()`           — pattern detector (reuses tracer).
 *   7. `Manipulator.begin()`            — Phase-3 stub kept in the chain.
 *   8. `Localization.begin()`           — currentNode, yawOffsetDeg unzeroed.
 *   9. `PathTracker.begin()`            — FSM + queue idle.
 *  10. `EventTrace.begin()`             — OLED ring buffer; pushed last so the boot
 *                                          event is the first line on screen.
 */
void setup()
{
    // 1. Bring up the lower MCU, IMU, motors, servos, OLED, and the on-board LEDs.
    MiniR4.begin();

    // 2. Open the serial console for any ad-hoc telemetry.
    Serial.begin(115200);

    // 3. Tell the power module how many cells are physically installed (must match the pack).
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // 4. Initialise our modules in dependency order. Every `begin()` returns a `bool`
    //    success indicator — accumulate them here so a partial init never reports
    //    "ready" and motion is gated for the rest of the run.
    const bool driveOk = Drive.begin();
    const bool lineOk  = LineFollower.begin();
    const bool intOk   = Intersection.begin();
    const bool manipOk = Manipulator.begin();
    const bool locOk   = Localization.begin();
    const bool pathOk  = PathTracker.begin();
    const bool traceOk = EventTrace.begin();

    // Wire up the tick callbacks so any sync-blocking spin keeps every other module alive.
    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);

    _setupFailed = !(driveOk && lineOk && intOk && manipOk && locOk && pathOk && traceOk);

    // 5. Surface a boot marker. On failure, name the offending modules so the operator
    //    can diagnose without re-reading every begin() body.
    if (_setupFailed)
    {
        Serial.println(F("WRO2026 Phase 2 BOOT FAIL — module begin() returned false:"));
        if (!driveOk)  { Serial.println(F("  - Drive"));         EventTrace.pushEvent("FAIL DRIVE"); }
        if (!lineOk)   { Serial.println(F("  - LineFollower"));  EventTrace.pushEvent("FAIL LINE"); }
        if (!intOk)    { Serial.println(F("  - Intersection"));  EventTrace.pushEvent("FAIL INT"); }
        if (!manipOk)  { Serial.println(F("  - Manipulator"));   EventTrace.pushEvent("FAIL MANIP"); }
        if (!locOk)    { Serial.println(F("  - Localization"));  EventTrace.pushEvent("FAIL LOC"); }
        if (!pathOk)   { Serial.println(F("  - PathTracker"));   EventTrace.pushEvent("FAIL PATH"); }
        if (!traceOk)  { Serial.println(F("  - EventTrace"));    EventTrace.pushEvent("FAIL TRACE"); }
        EventTrace.pushEvent("BOOT FAIL");
    }
    else
    {
        EventTrace.pushEvent("BOOT OK");
        Serial.println(F("WRO2026 Phase 2 ready."));
    }
}

/**
 * @brief Arduino main loop: one cooperative tick across every module.
 *
 * The body is just `cooperativeTick();` outside `NAV_DEMO_MODE`. With `NAV_DEMO_MODE`
 * defined, an additional BTN_UP rising-edge handler captures the IMU yaw offset and runs
 * a hardcoded `goTo("C"); goTo("G");` sequence. Both PathTracker `goTo` calls are
 * sync-blocking by default — the registered cooperative tick callback keeps every other
 * module alive during their drain.
 */
void loop()
{
    cooperativeTick();

#ifdef NAV_DEMO_MODE
    // Motion gate: if any module's begin() failed, refuse to issue any motion. The OLED +
    // Serial trail already explained which module bailed; surfacing a stale-tuning move on
    // top would just hide the failure.
    if (_setupFailed)
    {
        return;
    }

    // One-shot per power-cycle on the BTN_UP rising edge. Capture the IMU yaw offset NOW
    // (operator has just placed the chassis on Routes::START_NODE facing the start
    // heading), push a marker, then chain the hardcoded nav demo. Each goTo() drains
    // its own queue at end-of-statement, so the second call only starts after the first
    // has arrived.
    const bool btnNow = MiniR4.BTN_UP.getState();
    if (btnNow && !_btnPrev && !_navArmed)
    {
        _navArmed = true;

        const bool zeroed = Localization.zeroHeadingFromCurrent();
        if (!zeroed)
        {
            EventTrace.pushEvent("NAV ZERO FAIL");
            return;
        }
        EventTrace.pushEvent("NAV ZERO");

        PathTracker.goTo("C");    // sync-blocks until C reached
        if (PathTracker.hasFault())
        {
            EventTrace.pushEvent("NAV FAULT");
            return;
        }
        EventTrace.pushEvent("NAV ARR C");

        PathTracker.goTo("G");    // sync-blocks until G reached + blind moves done
        if (PathTracker.hasFault())
        {
            EventTrace.pushEvent("NAV FAULT");
            return;
        }
        EventTrace.pushEvent("NAV ARR G");
        EventTrace.pushEvent("NAV DONE");
    }
    _btnPrev = btnNow;
#endif    // NAV_DEMO_MODE
}
