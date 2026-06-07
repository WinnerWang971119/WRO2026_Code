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
 *      On success the buzzer chirps 3x and the RGB LEDs turn green; on failure
 *      it emits one long low tone and the LEDs turn red.
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
#include "Drive.h"
#include "EventTrace.h"
#include "FieldMap.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "Manipulator.h"
#include "Mission.h"
#include "PathTracker.h"
#include "Tasks.h"
#include "WROLineArray.h"

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
 * @brief Cooperative tick — every module's `update()` exactly once.
 *
 * Used as the body of `loop()` and registered with `Drive.setTickCallback()`,
 * `PathTracker.setTickCallback()`, and `LineFollower.setTickCallback()` so any
 * sync-blocking drain or follow-wait keeps everyone alive — in particular
 * `LineArray.update()` runs every iteration when `LINE_ARRAY_TICK_CACHE_ENABLED`
 * is flipped on, instead of freezing to whatever was cached on entry.
 *
 * Order: LineArray -> Drive -> LineFollower -> Intersection -> Manipulator
 *     -> PathTracker -> Mission -> EventTrace.
 */
void cooperativeTick()
{
    // LineArray.update() refreshes the sensor cache for the rest of the tick
    // WHEN Config::LINE_ARRAY_TICK_CACHE_ENABLED is true. When false, this is
    // a no-op and the getters refresh independently (current PR #20 behaviour).
    // See WROLineArray.h::update().
    LineArray.update();

    Drive.update();
    LineFollower.update();
    Intersection.update();
    Manipulator.update();
    Tasks.update();          // T8: housekeeping; no FSM in this plan
    PathTracker.update();
    Mission.update();

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

/**
 * @brief Hand-typed PR3 mission. Edit this function to change tomorrow's run.
 *
 * Every `Mission.<verb>(...)` enqueues a step. `Mission.start()` arms the
 * queue and returns immediately; `loop()` drains it via `Mission.update()`.
 */
void buildMission()
{
    Mission
        .task("testChassis", 10);
    Mission.start();
}

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    const bool arrayOk = LineArray.begin();
    const bool driveOk = Drive.begin();
    const bool lineOk  = LineFollower.begin();
    const bool intOk   = Intersection.begin();
    const bool manipOk = Manipulator.begin();
    const bool locOk   = Localization.begin();
    const bool pathOk  = PathTracker.begin();
    const bool missOk  = Mission.begin();
    const bool tasksOk = Tasks.begin();
    const bool traceOk = EventTrace.begin();

    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);
    LineFollower.setTickCallback(cooperativeTick);
    Manipulator.setTickCallback(cooperativeTick);

    _setupFailed = !(arrayOk && driveOk && lineOk && intOk && manipOk && locOk && pathOk && missOk && tasksOk && traceOk);

    if (_setupFailed)
    {
        Serial.println(F("WRO2026 PR3 BOOT FAIL — module begin() returned false:"));
        if (!arrayOk)  { Serial.println(F("  - LineArray"));    EventTrace.pushEvent("FAIL ARR"); }
        if (!driveOk)  { Serial.println(F("  - Drive"));        EventTrace.pushEvent("FAIL DRIVE"); }
        if (!lineOk)   { Serial.println(F("  - LineFollower")); EventTrace.pushEvent("FAIL LINE"); }
        if (!intOk)    { Serial.println(F("  - Intersection")); EventTrace.pushEvent("FAIL INT"); }
        if (!manipOk)  { Serial.println(F("  - Manipulator"));  EventTrace.pushEvent("FAIL MANIP"); }
        if (!locOk)    { Serial.println(F("  - Localization")); EventTrace.pushEvent("FAIL LOC"); }
        if (!pathOk)   { Serial.println(F("  - PathTracker"));  EventTrace.pushEvent("FAIL PATH"); }
        if (!missOk)   { Serial.println(F("  - Mission"));      EventTrace.pushEvent("FAIL MIS"); }
        if (!tasksOk)  { Serial.println(F("  - Tasks"));        EventTrace.pushEvent("FAIL TASKS"); }
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

    // BTN_UP rising edge arms the mission. The press captures the IMU yaw NOW
    // (operator has placed the chassis on `Routes::START_NODE` facing
    // `START_HEADING_DEG`). In production (`Config::MISSION_ALLOW_REARM` false)
    // it arms exactly once per power cycle; subsequent presses do nothing.
    //
    // `_missionArmed` is set ONLY on a successful zero so a transient
    // failure does not permanently lock the operator out of retrying.
    //
    // Bench-test re-arm: when `Config::MISSION_ALLOW_REARM` is true, a fresh
    // BTN_UP press also re-fires once the mission has returned to idle — a
    // clean finish or a cleared fault, never mid-run (Mission.isIdle() is
    // false while RUNNING or faulting). `_missionArmed` stays latched, so the
    // BTN_DOWN calibration-menu path stays suppressed after the first arm.
    //
    // Guard: while BTN_DOWN is being held toward the 2-s menu-entry threshold,
    // ignore BTN_UP so that accidental thumb-on-both-buttons cannot spuriously
    // arm the mission when the operator believes they are entering the cal menu.
    const bool btnUpNow = MiniR4.BTN_UP.getState();
    const bool armable =
        !_missionArmed || (Config::MISSION_ALLOW_REARM && Mission.isIdle());
    if (btnUpNow && !_btnUpPrev && armable && _btnDownPressStartMs == 0)
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
