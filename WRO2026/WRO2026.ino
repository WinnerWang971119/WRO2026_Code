/**
 * @file WRO2026.ino
 * @brief WRO 2026 Mosaic Masters — PR3 entry sketch.
 * @author WRO 2026
 *
 * @details
 * PR3 deliverable: human-typed mission DSL on top of PR2's PathTracker.
 *
 * Operator flow:
 *   1. Power on. The pack voltage is read and every module begin()s run; OLED
 *      shows `BOOT OK` (or `BOOT FAIL`). A pack below `Config::BATT_MIN_VOLTAGE_V`
 *      fails the boot just like a failed module begin(). On success the buzzer
 *      chirps 3x and the RGB LEDs turn green; on failure it emits one long low
 *      tone and the LEDs turn red.
 *   2. Place chassis on `Routes::START_NODE` (`A`) facing `START_HEADING_DEG`.
 *   2a. Optional pre-mission calibration: hold BTN_DOWN for >= 2 s to
 *       open the OLED CALIBRATION menu. BTN_DOWN short-presses cycle
 *       the cursor (LINE -> MOTOR -> EXIT). BTN_UP confirms the selection:
 *         - LINE:  runs LineArray.calibrate() (sequential per-board
 *                  sweep with BTN_UP gate between phases).
 *         - MOTOR: runs Drive.calibrateMotorBias() (~2 s wheels-up spin;
 *                  LIFT THE CHASSIS before selecting this!).
 *         - EXIT:  returns to idle.
 *       After dispatch the menu closes; the operator can re-enter it
 *       any time before the mission is armed.
 *   3. Press `BTN_UP` once — this zeros the IMU yaw offset, then calls
 *      `Mission.start()`. The mission runs asynchronously from `loop()`.
 *   4. Serial prints state-change events + periodic `HB t=... yaw=... node=...`
 *      heartbeats. OLED scrolls the same events.
 *   5. On PathTracker fault (UNREACHABLE / DRIVE_FAULT / ...) or a
 *      `goTo` to an unknown node name: the chassis stops, the queue is
 *      dropped, the buzzer emits 5 × 500 ms tones at 1 kHz via
 *      `MiniR4.Buzzer.beep`, and the on-board RGB LEDs flash red in
 *      sync with the beep (solid red while awaiting the ack). Press
 *      `BTN_DOWN` to acknowledge and reset.
 *
 * Author the mission inline below — one `Mission.<verb>(...)` per line.
 * Available verbs: `goTo`, `wait`, `print`, `beep`, `led`. See Mission.h.
 *
 * @note No business logic in `loop()`. Every module's `update()` is
 *       non-blocking; the loop just dispatches them via `cooperativeTick()`.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "src/drive/Drive.h"
#include "src/util/EventTrace.h"
#include "src/nav/FieldMap.h"
#include "src/drive/Intersection.h"
#include "src/drive/LineFollower.h"
#include "src/nav/Localization.h"
#include "src/manip/Manipulator.h"
#include "src/mission/Mission.h"
#include "MissionScript.h"
#include "src/mission/PathTracker.h"
#include "Tasks.h"
#include "src/perception/Vision.h"
#include "src/perception/WROLineArray.h"

namespace
{
    /**
     * @brief Calibration menu top-level state. IDLE = normal pre-mission flow;
     *        OPEN = operator is currently navigating the on-OLED menu and the
     *        BTN_UP mission-arm path is suppressed.
     */
    enum class CalMenuState : uint8_t
    {
        IDLE,
        OPEN
    };

    /**
     * @brief Calibration menu items, indexed contiguously so BTN_DOWN
     *        short-presses can cycle by integer modulo.
     */
    enum class CalMenuItem : uint8_t
    {
        LINE  = 0,
        MOTOR = 1,
        EXIT  = 2
    };

    constexpr uint8_t CAL_MENU_ITEM_COUNT = 3;

    bool         _setupFailed         = false;
    bool         _missionArmed        = false;
    bool         _btnUpPrev           = false;
    bool         _btnDownPrev         = false;
    uint32_t     _btnDownPressStartMs = 0;   // 0 = not currently pressed; nonzero = press start timestamp
    CalMenuState _calMenuState        = CalMenuState::IDLE;
    CalMenuItem  _calMenuSel          = CalMenuItem::LINE;
}    // namespace

/**
 * @brief Draw the CALIBRATION menu to the OLED.
 *
 * The 128x32 panel at text size 1 fits ~4 rows of 8 px text. We use row 0 as
 * the title and rows 1..3 for the three menu items. The cursor `> ` precedes
 * the currently-selected item; non-selected rows get two spaces so labels
 * align vertically.
 *
 * Called once on menu entry and once per BTN_DOWN cursor cycle. Other
 * modules' `cooperativeTick()` OLED writes may transiently overwrite the
 * menu; that is acceptable — the next cursor move repaints.
 */
void renderCalMenu()
{
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setTextSize(1);
    MiniR4.OLED.setTextColor(WHITE);
    MiniR4.OLED.setCursor(0, 0);
    MiniR4.OLED.println(F("CALIBRATION"));

    static const char* labels[CAL_MENU_ITEM_COUNT] = { "LINE", "MOTOR", "EXIT" };
    for (uint8_t i = 0; i < CAL_MENU_ITEM_COUNT; ++i)
    {
        MiniR4.OLED.setCursor(0, 8 + i * 8);
        MiniR4.OLED.print((i == static_cast<uint8_t>(_calMenuSel)) ? F("> ") : F("  "));
        MiniR4.OLED.println(labels[i]);
    }
    MiniR4.OLED.display();
}

/**
 * @brief Whether this tick must run the line-array perception updates.
 *
 * The three line consumers — `LineArray.update()`, `LineFollower.update()` and
 * `Intersection.update()` — only do useful work when the follower is engaged or
 * a `stopAtLine` seek is in flight. On a pure GYRO/RAW/SYNC move or at
 * pre-mission idle nothing reads the line cache, so the slow Wire1 I2C read is
 * wasted budget. `cooperativeTick()` gates those three calls on this predicate.
 *
 * @return `LineFollower.isEnabled() || Drive.isSeekingLine()` when the gate is
 *         enabled; always `true` when `Config::LINE_ARRAY_TICK_GATE_ENABLED` is
 *         `false` (the `constexpr` early-out folds the gate away, restoring the
 *         legacy unconditional three-update path).
 * @note Keys off the `stopAtLine` seek FSM via `Drive.isSeekingLine()`; a
 *       `wallStop` push does NOT open the gate — it uses the encoder stall
 *       watchdog, not the line sensor.
 */
static bool lineArrayNeededThisTick()
{
    if (!Config::LINE_ARRAY_TICK_GATE_ENABLED)
    {
        return true;
    }
    return LineFollower.isEnabled() || Drive.isSeekingLine();
}

/**
 * @brief Cooperative tick — every module's `update()` once per tick, with the
 *        three line-array consumers gated on demand.
 *
 * Used as the body of `loop()` and registered with `Drive.setTickCallback()`,
 * `PathTracker.setTickCallback()`, and `LineFollower.setTickCallback()` so any
 * sync-blocking drain or follow-wait keeps everyone alive.
 *
 * Line-array gate (issue #164): `LineArray.update()`, `LineFollower.update()`
 * and `Intersection.update()` run ONLY when `lineArrayNeededThisTick()` is true
 * (the follower is enabled or a `stopAtLine` seek is in flight). On a pure
 * GYRO/RAW/SYNC move or at idle nothing reads the line cache, so the slow Wire1
 * I2C read (~11 ms in cache mode) is skipped. `Config::LINE_ARRAY_TICK_GATE_ENABLED`
 * folds the gate away when `false`. The predicate is resolved ONCE per tick into
 * `lineNeeded` so the three calls cannot disagree within a tick.
 *
 * Freshness is structural, not an extra refresh: `LineArray.update()` runs FIRST
 * among the gated three and every consumer after it, so on any needed tick the
 * cache is refreshed at the top before any read. The dispatch-tick case relies
 * on a LOAD-BEARING order inside `Drive::update()`: `_updateStopAtLine()` runs
 * BEFORE `_dispatchNext()`, so a queued `stopAtLine` popped this tick arms the
 * seek AFTER the seek FSM already ran — it consumes no line sample until the
 * next tick, whose tick-top predicate is true and refreshes first. Do NOT
 * reorder `_dispatchNext()` ahead of the sub-FSM updates or this guarantee
 * breaks (host-tested).
 *
 * `Drive.update()` (between LineArray and the follower) and everything from
 * `Localization.update()` onward stay UNCONDITIONAL.
 *
 * Reopen drain: the gate's closed->open edge arms a countdown of
 * `Config::INTERSECTION_DEBOUNCE_TICKS` open ticks; on each, after
 * `Intersection.update()` produced that tick's fresh sample, a read-and-clear
 * discards any departure-junction latch so PathTracker cannot consume it as a
 * phantom arrival. A window (not a one-shot) is required because the debounced
 * edge fires up to `INTERSECTION_DEBOUNCE_TICKS` ticks AFTER reopen (Error
 * Handling -> phantom junction on gate reopen).
 *
 * Order: [LineArray] -> Drive -> [LineFollower -> Intersection] -> Manipulator
 *     -> Tasks -> PathTracker -> Localization -> Mission -> EventTrace
 *     (bracketed steps gated on `lineNeeded`).
 */
void cooperativeTick()
{
    // NOTE: if you add/remove a module tick here, mirror it in
    // inoTest/tick_bench/tick_bench.ino (the addSegment() + benchTick() lists
    // AND the SEGMENT_COUNT constant) so the loop-time benchmark stays
    // representative (issue #123).

    // Line-array gate (issue #164). Resolve the predicate ONCE so the three
    // gated consumer updates below cannot disagree within a tick.
    const bool lineNeeded = lineArrayNeededThisTick();

    // Reopen-drain + gate observable state (see the doxygen block above).
    static bool     s_prevLineNeeded   = true;   // prev-tick gate state; init true so a disabled gate never drains
    static uint8_t  s_reopenDrainTicks = 0;      // open ticks left in the reopen-drain window
    static uint32_t s_tickCount        = 0;      // total cooperativeTick() calls
    static uint32_t s_lineUpdateCount  = 0;      // ticks LineArray.update() ran
    static uint32_t s_lastGateHbMs     = 0;      // last gate-heartbeat timestamp
    constexpr uint32_t GATE_HEARTBEAT_INTERVAL_MS = 1000;

    ++s_tickCount;

    if (lineNeeded)
    {
        // LineArray.update() refreshes the sensor cache for the rest of the tick
        // WHEN Config::LINE_ARRAY_TICK_CACHE_ENABLED is true. When false, this is
        // a no-op and the getters refresh independently (current PR #20 behaviour).
        // See WROLineArray.h::update(). Runs FIRST among the gated three so the
        // cache is fresh before any consumer reads it this tick.
        LineArray.update();
        ++s_lineUpdateCount;
    }

    // Unconditional: Drive drives the profile/seek/wallStop FSMs and the queue.
    // Its `_updateStopAtLine()` sub-FSM reads the (now-fresh) cache and runs
    // BEFORE `_dispatchNext()` — the load-bearing order the dispatch-tick
    // freshness proof depends on. Do NOT reorder it past LineArray above.
    Drive.update();

    if (lineNeeded)
    {
        LineFollower.update();
        Intersection.update();

        // Reopen drain (Error Handling -> phantom junction on gate reopen).
        // While the gate was closed over a departure junction, Intersection's
        // _validStreak was frozen below Config::INTERSECTION_DEBOUNCE_TICKS, so
        // the rising edge for that junction does NOT latch on the reopen tick —
        // Intersection.update() latches it only once the streak reaches the
        // threshold, up to INTERSECTION_DEBOUNCE_TICKS open ticks after reopen.
        // A one-shot drain on the reopen tick alone therefore fired one debounce
        // window too early and missed the phantom. Instead, arm a countdown on
        // the closed->open edge and drain (read-and-clear) on EACH of the first
        // INTERSECTION_DEBOUNCE_TICKS open ticks, after Intersection.update()
        // produced that tick's fresh sample — the latest possible departure-
        // marker edge lands on the last drained tick. A genuine NEXT-node
        // arrival cannot physically occur within one debounce window of follow
        // start, so no real edge is ever clipped.
        if (!s_prevLineNeeded)
        {
            s_reopenDrainTicks = Config::INTERSECTION_DEBOUNCE_TICKS;
        }
        if (s_reopenDrainTicks > 0)
        {
            (void) Intersection.isAtIntersection();
            --s_reopenDrainTicks;
        }
    }

    s_prevLineNeeded = lineNeeded;

    Manipulator.update();
    Tasks.update();          // T8: housekeeping; no FSM in this plan
    PathTracker.update();

    // Operator-offset integrator. MUST run unconditionally every tick (not behind
    // the cal-menu guard below): Drive ticked above so its encoder reads are fresh,
    // and PathTracker just pushed its nav-active flag (setNavActive) so the gate
    // resolves on this same tick. While the gate is closed it issues no IMU read; it
    // re-baselines once on the closed->open edge, so the operator's first hand-drive
    // after a hop (or after zeroing) does not register a cold-start phantom.
    Localization.update();

    Mission.update();

    // Gate observable (issue #164, AC1): cumulative LineArray.update() runs vs
    // total ticks, printed on a millis-gated cadence (no delay()). During a pure
    // GYRO/RAW/SYNC move the numerator holds while the denominator climbs — the
    // evidence the gate actually closes.
    const uint32_t nowMs = millis();
    if ((nowMs - s_lastGateHbMs) >= GATE_HEARTBEAT_INTERVAL_MS)
    {
        s_lastGateHbMs = nowMs;
        Serial.print(F("[gate] la="));
        Serial.print(s_lineUpdateCount);
        Serial.print('/');
        Serial.println(s_tickCount);
    }

    // The calibration menu owns the OLED while open (FINDING #4): EventTrace's
    // dirty-flag repaint would clearDisplay() and paint the event ring over the
    // menu within ~one tick. Suppress its render while OPEN. The pending _dirty
    // flag persists, so the first update() after the menu closes repaints the
    // accumulated events.
    if (_calMenuState != CalMenuState::OPEN)
    {
        EventTrace.update();
    }
}

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);

    // Issue #152: self-silencing boot warning — prints once, only when the bench
    // mission re-arm switch (Config::MISSION_ALLOW_REARM, ConfigMission.h) is left on.
    // It ships `true` for bench convenience; this is the loud reminder to flip it to
    // `false` before the competition run so a stray BTN_UP cannot restart the route
    // mid-field. When the constant is already `false` this block prints nothing.
    if (Config::MISSION_ALLOW_REARM)
    {
        Serial.println(F("[WARN] MISSION_ALLOW_REARM=true (bench re-arm ON) — flip to false before competition!"));
    }

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // Boot-time battery gate. getBattVoltage() returns the measured pack voltage,
    // or 0 on an I2C read miss — retry once (the retry-once idiom used throughout
    // MMLower callers) before treating the pack as under-voltage. A read that is
    // still 0 (< the floor) fails the boot, which is the fail-safe outcome: never
    // start a run when the pack voltage cannot be confirmed above the floor.
    float battVoltage = MiniR4.PWR.getBattVoltage();
    if (battVoltage <= 0.0f)
    {
        delay(50);
        battVoltage = MiniR4.PWR.getBattVoltage();
    }
    const bool battOk = (battVoltage >= Config::BATT_MIN_VOLTAGE_V);

    const bool arrayOk = LineArray.begin();
    const bool driveOk = Drive.begin();
    const bool lineOk  = LineFollower.begin();
    const bool intOk   = Intersection.begin();
    const bool manipOk = Manipulator.begin();
    const bool locOk   = Localization.begin();
    const bool pathOk  = PathTracker.begin();
    const bool missOk  = Mission.begin();
    const bool tasksOk  = Tasks.begin();
    const bool visionOk = Vision.begin();
    const bool traceOk  = EventTrace.begin();

    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);
    LineFollower.setTickCallback(cooperativeTick);
    Manipulator.setTickCallback(cooperativeTick);
    Tasks.setTickCallback(cooperativeTick);
    Vision.setTickCallback(cooperativeTick);

    _setupFailed = !(battOk && arrayOk && driveOk && lineOk && intOk && manipOk && locOk && pathOk && missOk && tasksOk && visionOk && traceOk);

    if (_setupFailed)
    {
        Serial.println(F("WRO2026 PR3 BOOT FAIL:"));
        if (!battOk)
        {
            Serial.print(F("  - Battery low/unreadable: "));
            Serial.print(battVoltage);
            Serial.print(F(" V < "));
            Serial.print(Config::BATT_MIN_VOLTAGE_V);
            Serial.println(F(" V"));
            EventTrace.pushEvent("BATT LOW");
        }
        if (!arrayOk)  { Serial.println(F("  - LineArray"));    EventTrace.pushEvent("FAIL ARR"); }
        if (!driveOk)  { Serial.println(F("  - Drive"));        EventTrace.pushEvent("FAIL DRIVE"); }
        if (!lineOk)   { Serial.println(F("  - LineFollower")); EventTrace.pushEvent("FAIL LINE"); }
        if (!intOk)    { Serial.println(F("  - Intersection")); EventTrace.pushEvent("FAIL INT"); }
        if (!manipOk)  { Serial.println(F("  - Manipulator"));  EventTrace.pushEvent("FAIL MANIP"); }
        if (!locOk)    { Serial.println(F("  - Localization")); EventTrace.pushEvent("FAIL LOC"); }
        if (!pathOk)   { Serial.println(F("  - PathTracker"));  EventTrace.pushEvent("FAIL PATH"); }
        if (!missOk)   { Serial.println(F("  - Mission"));      EventTrace.pushEvent("FAIL MIS"); }
        if (!tasksOk)  { Serial.println(F("  - Tasks"));        EventTrace.pushEvent("FAIL TASKS"); }
        if (!visionOk) { Serial.println(F("  - Vision"));       EventTrace.pushEvent("FAIL VIS"); }
        if (!traceOk)  { Serial.println(F("  - EventTrace"));   EventTrace.pushEvent("FAIL TRACE"); }
        EventTrace.pushEvent("BOOT FAIL");

        // Boot-fail signal: solid red on both on-board RGB LEDs + one long low
        // tone. Tone() is non-blocking; the tone plays out via the hardware timer
        // while loop() idles in its _setupFailed early-return.
        MiniR4.LED.setColor(1, 255, 0, 0);
        MiniR4.LED.setColor(2, 255, 0, 0);
        MiniR4.Buzzer.Tone(Config::BOOT_FAIL_BEEP_HZ, Config::BOOT_FAIL_BEEP_MS);
    }
    else
    {
        EventTrace.pushEvent("BOOT OK");
        Serial.println(F("WRO2026 PR3 ready. Place at A, then press BTN_UP."));

        // Boot-OK signal: solid green on both on-board RGB LEDs + BOOT_OK_BEEPS
        // short chirps. Tone() is non-blocking, so delay each chirp's on-time plus
        // a gap to keep the chirps audibly separate (safe — setup() is blocking).
        MiniR4.LED.setColor(1, 0, 255, 0);
        MiniR4.LED.setColor(2, 0, 255, 0);
        for (uint8_t i = 0; i < Config::BOOT_OK_BEEPS; ++i)
        {
            MiniR4.Buzzer.Tone(Config::BOOT_OK_BEEP_HZ, Config::BOOT_OK_BEEP_MS);
            delay(Config::BOOT_OK_BEEP_MS + Config::BOOT_OK_BEEP_GAP_MS);
        }
    }
}

void loop()
{
    cooperativeTick();

    if (_setupFailed)
    {
        return;
    }

    // -----------------------------------------------------------------------
    // Calibration menu state machine. While OPEN: BTN_DOWN rising edge cycles
    // the cursor, BTN_UP rising edge confirms the selection and closes the
    // menu. The IDLE-state BTN_UP mission-arm path is suppressed for the rest
    // of this loop pass via the early `return`.
    // -----------------------------------------------------------------------
    if (_calMenuState == CalMenuState::OPEN)
    {
        const bool btnDownNow = MiniR4.BTN_DOWN.getState();
        const bool btnUpNow   = MiniR4.BTN_UP.getState();

        // BTN_DOWN short-press rising edge → cycle cursor.
        if (btnDownNow && !_btnDownPrev)
        {
            _calMenuSel = static_cast<CalMenuItem>(
                (static_cast<uint8_t>(_calMenuSel) + 1) % CAL_MENU_ITEM_COUNT);
            renderCalMenu();
        }

        // BTN_UP short-press rising edge → confirm + dispatch + exit menu.
        if (btnUpNow && !_btnUpPrev)
        {
            switch (_calMenuSel)
            {
                case CalMenuItem::LINE:
                    EventTrace.pushEvent("CAL LINE");
                    Serial.println(F("[main] dispatching LineArray.calibrate()..."));
                    LineArray.calibrate();
                    break;
                case CalMenuItem::MOTOR:
                    EventTrace.pushEvent("CAL MOTOR");
                    Serial.println(F("[main] dispatching Drive.calibrateMotorBias()..."));
                    Drive.calibrateMotorBias(int16_t(70));
                    break;
                case CalMenuItem::EXIT:
                    EventTrace.pushEvent("CAL EXIT");
                    Serial.println(F("[main] menu exit."));
                    break;
            }
            _calMenuState = CalMenuState::IDLE;

            // Force a release-poll so the BTN_UP press that confirmed doesn't
            // immediately re-trigger the mission-arm path in the same loop
            // pass, and so a still-held BTN_DOWN doesn't cycle the cursor of
            // a re-opened menu the next time we enter OPEN.
            while (MiniR4.BTN_UP.getState() || MiniR4.BTN_DOWN.getState())
            {
                delay(20);
            }
            _btnDownPrev = false;
            _btnUpPrev   = false;
            return;
        }

        _btnDownPrev = btnDownNow;
        _btnUpPrev   = btnUpNow;
        return;   // skip IDLE-state BTN_UP arm logic while menu is open
    }

    // IDLE state: BTN_DOWN held >= 2 s opens the OLED calibration menu. Only
    // available before the mission has been armed (so we don't intercept the
    // BTN_DOWN fault-acknowledge path that activates after a Mission fault).
    if (!_missionArmed)
    {
        const bool btnDownNow = MiniR4.BTN_DOWN.getState();
        if (btnDownNow)
        {
            if (_btnDownPressStartMs == 0)
            {
                _btnDownPressStartMs = millis();
            }
            else if ((millis() - _btnDownPressStartMs) >= 2000)
            {
                EventTrace.pushEvent("CAL MENU");
                Serial.println(F("[main] entering calibration menu (BTN_DOWN held >= 2s)"));
                _calMenuState        = CalMenuState::OPEN;
                _calMenuSel          = CalMenuItem::LINE;
                _btnDownPressStartMs = 0;
                renderCalMenu();

                // Wait for BOTH buttons to release so neither a still-held
                // BTN_DOWN counts as a short-press cursor cycle nor a still-held
                // BTN_UP counts as a confirm rising edge on the very next loop
                // tick (FINDING #5). Mirrors the confirm-exit release loop.
                while (MiniR4.BTN_DOWN.getState() || MiniR4.BTN_UP.getState())
                {
                    delay(20);
                }
                _btnDownPrev = false;
                _btnUpPrev   = false;
                return;   // bail; next loop tick handles menu navigation
            }
        }
        else
        {
            // Released before the 2 s threshold — reset start so a tap doesn't
            // accumulate toward an unintended trigger.
            _btnDownPressStartMs = 0;
        }
    }

    // BTN_UP rising edge arms the mission. The press re-anchors the chassis to
    // `Routes::START_NODE` and captures the IMU yaw NOW (the operator has just
    // placed it there, facing `START_HEADING_DEG`) — both halves of that pose,
    // not the heading alone. In production (`Config::MISSION_ALLOW_REARM` false)
    // it arms exactly once per power cycle; subsequent presses do nothing.
    //
    // `_missionArmed` is set ONLY on a successful zero so a transient
    // failure does not permanently lock the operator out of retrying.
    //
    // Bench-test re-arm: when `Config::MISSION_ALLOW_REARM` is true, a fresh
    // BTN_UP press also re-fires once the mission has returned to idle — a
    // clean finish or a cleared fault, never mid-run (Mission.isIdle() is
    // false while RUNNING or faulting). `_missionArmed` stays latched, so the
    // BTN_DOWN calibration-menu path stays suppressed after the first arm. The
    // re-arm rebinds the frame from scratch via `Localization.begin()` below —
    // the previous run left `_currentNode` on ITS final node, and re-planning
    // from there is what made a re-armed route no-op or take a wrong path.
    //
    // Guard: while BTN_DOWN is being held toward the 2-s menu-entry threshold,
    // ignore BTN_UP so that accidental thumb-on-both-buttons cannot spuriously
    // arm the mission when the operator believes they are entering the cal menu.
    const bool btnUpNow = MiniR4.BTN_UP.getState();
    const bool armable =
        !_missionArmed || (Config::MISSION_ALLOW_REARM && Mission.isIdle());
    if (btnUpNow && !_btnUpPrev && armable && _btnDownPressStartMs == 0)
    {
        // Rebind the WHOLE world frame, not just heading. BTN_UP asserts "the chassis is
        // physically on `Routes::START_NODE` facing `START_HEADING_DEG`", and the NODE half of
        // that contract needs its own write: `zeroHeadingFromCurrent()` binds `_yawOffsetDeg`
        // and drops `_offset`, but deliberately never touches `_currentNode`. `begin()` is the
        // only verb that re-seeds `_currentNode = Routes::START_NODE` (plus the offset,
        // integrator baselines and nav gate), so a re-arm without it leaves `_currentNode`
        // latched on the PREVIOUS run's final node — `PathTracker::_onPlanNextHop` then sees
        // `cur == _destinationNode`, reports an instant arrival, and the chassis never moves.
        //
        // Unconditional (NOT gated on `_missionArmed`): on the first arm this is a no-op —
        // nothing can have moved `_currentNode` or grown `_offset` before the first zero (the
        // integrate gate needs `isZeroed()`, and a pre-zero `goTo` faults NOT_ZEROED).
        //
        // Order matters: `begin()` runs FIRST so the zero below binds against clean state. It
        // clears `_zeroed`, so a failed zero leaves the chassis un-zeroed with the mission
        // un-started — recoverable, not a lockout: `armable` still holds, so BTN_UP retries. It also
        // re-seeds the heading cache to `START_HEADING_DEG`, which is exactly the truth at this
        // moment (`zeroHeadingFromCurrent()` does not refresh that cache itself), instead of
        // serving the prior run's final heading to no-I/O readers until the next live read.
        Localization.begin();
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
