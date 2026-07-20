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
 *   2a. Optional pre-mission diagnostics: hold BTN_DOWN for >=
 *       `Config::UI_MENU_LONGPRESS_MS` to open the OLED `DIAGNOSTICS`
 *       console (ADR-076). BTN_DOWN short-presses scroll the 9-entry
 *       list, BTN_UP confirms the highlighted entry:
 *         SELF TEST / BATT / LINE VIEW / VISION VIEW / GYRO VIEW /
 *         GYRO ZERO / CAL LINE / CAL MOTOR / EXIT.
 *       Every screen the console owns is per-tick and non-blocking (the
 *       two dispatched calibration routines are the pre-existing
 *       exceptions). The console owns the OLED exclusively while open and
 *       hands it back to `EventTrace` on close. It is available any time
 *       before the mission is armed — and, unlike the mission-arm path,
 *       ALSO after a failed boot, which is exactly when the operator needs
 *       the BATT screen's boot report.
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
#include "src/operator/OperatorPanel.h"
#include "src/mission/PathTracker.h"
#include "Tasks.h"
#include "src/perception/Vision.h"
#include "src/perception/WROLineArray.h"
#include "src/nav/YawSource.h"

namespace
{
    /// Latched for the whole power cycle once `setup()`'s health checks fail. Nothing — no
    /// console entry, no button — ever clears it, so `_missionArmed` can never become true
    /// after a failed boot (AC23). It no longer short-circuits `loop()` wholesale: the
    /// `DIAGNOSTICS` console stays reachable so the operator can read the boot report about
    /// the very failure that just occurred.
    bool _setupFailed = false;

    bool _missionArmed = false;
    bool _btnUpPrev    = false;

    /// 0 = BTN_DOWN not currently pressed; nonzero = the `millis()` stamp the press began, used
    /// to time the `Config::UI_MENU_LONGPRESS_MS` console-entry hold.
    uint32_t _btnDownPressStartMs = 0;
}    // namespace

/**
 * @brief Whether this tick must run the line-array perception updates.
 *
 * The three line consumers — `LineArray.update()`, `LineFollower.update()` and
 * `Intersection.update()` — only do useful work when the follower is engaged, a
 * `stopAtLine` seek is in flight, or the diagnostics console is showing live line
 * data. On a pure GYRO/RAW/SYNC move or at pre-mission idle nothing reads the line
 * cache, so the slow Wire1 I2C read is wasted budget. `cooperativeTick()` gates
 * those three calls on this predicate.
 *
 * @return `LineFollower.isEnabled() || Drive.isSeekingLine() ||
 *         OperatorPanel.wantsLineData()` when the gate is enabled; always `true`
 *         when `Config::LINE_ARRAY_TICK_GATE_ENABLED` is `false` (the `constexpr`
 *         early-out folds the gate away, restoring the legacy unconditional
 *         three-update path).
 * @note Keys off the `stopAtLine` seek FSM via `Drive.isSeekingLine()`; a
 *       `wallStop` push does NOT open the gate — it uses the encoder stall
 *       watchdog, not the line sensor.
 * @note The console term (AC5) is what makes LINE VIEW and the SELF TEST line
 *       check live: the cached getters they read (`getJunctionType()`,
 *       `isOnline()`, `getError()`) do NOT self-refresh while
 *       `Config::LINE_ARRAY_TICK_CACHE_ENABLED` is on, so without forcing the
 *       gate open they would serve whatever the last mission tick left behind.
 *       `OperatorConsoleClass::wantsLineData()` is itself `false` whenever the
 *       console is closed or `boot.lineArray == false` (AC25), so this term costs
 *       one inlined state comparison per tick during a mission.
 */
static bool lineArrayNeededThisTick()
{
    if (!Config::LINE_ARRAY_TICK_GATE_ENABLED)
    {
        return true;
    }
    return LineFollower.isEnabled() || Drive.isSeekingLine() || OperatorPanel.wantsLineData();
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
 * `YawSource.update()` runs FIRST of all and UNCONDITIONALLY (issue #163): one
 * gated gyro read refreshes the tick-scoped yaw cache so every later `readYawDeg`
 * consumer (Drive, Localization) serves the same fresh value from a single device
 * transaction. `Drive.update()` (between LineArray and the follower) and
 * everything from `Localization.update()` onward stay UNCONDITIONAL.
 *
 * Reopen drain: the gate's closed->open edge arms a state-keyed drain that
 * read-and-clears any departure-junction latch every open tick (after
 * `Intersection.update()` produced that tick's fresh sample) until the marker
 * has stably RELEASED (`Intersection.isJunctionActive()` == false), so
 * PathTracker cannot consume it as a phantom arrival. Keying off the debounced
 * marker state -- rather than a fixed tick count -- closes the flicker residual
 * where sensor noise (e.g. NONE, T, T) delays the debounced edge past a
 * fixed-count window. A settle floor of `Config::INTERSECTION_DEBOUNCE_TICKS`
 * bridges a tick-1 dropout and bounds the no-marker case (an axle-offset arrival
 * whose sensor sits past the node never sees the marker) so the drain still ends
 * promptly (Error Handling -> phantom junction on gate reopen).
 *
 * Order: YawSource -> [LineArray] -> Drive -> [LineFollower -> Intersection] -> Manipulator
 *     -> Tasks -> PathTracker -> Localization -> Mission -> EventTrace
 *     (bracketed steps gated on `lineNeeded`).
 */
void cooperativeTick()
{
    // NOTE: if you add/remove a module tick here, mirror it in
    // inoTest/tick_bench/tick_bench.ino (the addSegment() + benchTick() lists
    // AND the SEGMENT_COUNT constant) so the loop-time benchmark stays
    // representative (issue #123).

    // YawSource.update() refreshes the tick-scoped gyro yaw cache FIRST, with one
    // gated device read, so every consumer that calls readYawDeg later in this same
    // tick (Drive, Localization) serves the freshly-cached value from a single
    // device transaction instead of each triggering its own inline re-read (AC-5).
    YawSource.update();

    // Line-array gate (issue #164). Resolve the predicate ONCE so the three
    // gated consumer updates below cannot disagree within a tick. LineArray.update()
    // itself now runs inside the `if (lineNeeded)` block below.
    const bool lineNeeded = lineArrayNeededThisTick();

    // Reopen-drain state (see the doxygen block above).
    static bool     s_prevLineNeeded   = true;   // prev-tick gate state; init true so a disabled gate never drains
    static bool     s_reopenDraining   = false;  // reopen-drain state machine active (drains until marker releases)
    static uint8_t  s_reopenSettle     = 0;      // settle-floor ticks left before a released drain may end

    if (lineNeeded)
    {
        // LineArray.update() refreshes the sensor cache for the rest of the tick
        // WHEN Config::LINE_ARRAY_TICK_CACHE_ENABLED is true. When false, this is
        // a no-op and the getters refresh independently (current PR #20 behaviour).
        // See WROLineArray.h::update(). Runs FIRST among the gated three so the
        // cache is fresh before any consumer reads it this tick.
        LineArray.update();
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
        // threshold, and sensor flicker (e.g. NONE, T, T over the first open
        // ticks) can push that latch past any fixed-count window. Instead of
        // guessing a tick count, drive a state machine off the debounced marker:
        // on the closed->open edge, arm a drain that read-and-clears the latch
        // (after Intersection.update() produced this tick's fresh sample) EVERY
        // open tick until the departure marker has stably RELEASED
        // (isJunctionActive() == false). A settle floor of
        // INTERSECTION_DEBOUNCE_TICKS keeps the drain armed across a leading
        // tick-1 dropout before the marker asserts AND bounds the no-marker case
        // (an axle-offset arrival whose sensor sits past the node never sees the
        // marker) so the drain still ends promptly. A genuine NEXT-node arrival
        // cannot physically occur within one debounce window of follow start, so
        // no real edge is ever clipped.
        if (!s_prevLineNeeded)
        {
            s_reopenDraining = true;
            s_reopenSettle   = Config::INTERSECTION_DEBOUNCE_TICKS;
        }
        if (s_reopenDraining)
        {
            (void) Intersection.isAtIntersection();   // read-and-clear any departure latch
            if (s_reopenSettle > 0)
            {
                --s_reopenSettle;
            }
            // End the drain only once past the settle floor AND the marker has
            // released; until then keep draining so a leading dropout before the
            // marker asserts is still covered.
            if (s_reopenSettle == 0 && !Intersection.isJunctionActive())
            {
                s_reopenDraining = false;
            }
        }
    }

    s_prevLineNeeded = lineNeeded;

    Manipulator.update();
    Tasks.update();          // T8: housekeeping; no FSM in this plan
    PathTracker.update();

    // Operator-offset integrator. MUST run unconditionally every tick (not behind
    // the console's OLED-ownership gating below): Drive ticked above so its encoder reads are fresh,
    // and PathTracker just pushed its nav-active flag (setNavActive) so the gate
    // resolves on this same tick. While the gate is closed it issues no IMU read; it
    // re-baselines once on the closed->open edge, so the operator's first hand-drive
    // after a hop (or after zeroing) does not register a cold-start phantom.
    Localization.update();

    Mission.update();

    // The diagnostics console owns the OLED exclusively while open (AC17): EventTrace's
    // dirty-flag repaint would clearDisplay() and paint the event ring over the console
    // frame within ~one tick. Suppress ONLY the render — never the whole update(), which
    // is what the pre-console menu did and which silently killed both the edge-triggered
    // event polling and the 500 ms Serial heartbeat along with it. EventTrace keeps
    // polling and heartbeating here while the console owns the display; its pending
    // _dirty flag is left set, and setRenderEnabled() forces one repaint on the
    // false->true edge, so the accumulated events come back in a single pass the first
    // tick after the console closes — no bookkeeping owed here.
    EventTrace.setRenderEnabled(!OperatorPanel.isOpen());
    EventTrace.update();
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
    const bool yawOk   = YawSource.begin();    // MUST run before Drive/Localization — both read yaw through YawSource
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

    // Boot health capture (AC10/AC23/AC24/AC25). These verdicts — the 12 module begin()
    // results above plus the battOk VOLTAGE verdict (MiniR4.PWR itself comes up inside
    // MiniR4.begin(), so `batt` is not an init result) — used to die here as setup()
    // locals the instant this function returned. The BATT screen, the console's
    // degraded-mode availability rules and the SELF TEST sequencer's per-subsystem skips
    // all read them, so they are captured once into a BootReport the console owns for the
    // rest of the power cycle. Value-initialised first so a field added to the struct
    // later can never be read uninitialised if this list is not updated with it.
    Console::BootReport bootReport{};
    bootReport.batt         = battOk;
    bootReport.lineArray    = arrayOk;
    bootReport.yaw          = yawOk;
    bootReport.drive        = driveOk;
    bootReport.lineFollower = lineOk;
    bootReport.intersection = intOk;
    bootReport.manipulator  = manipOk;
    bootReport.localization = locOk;
    bootReport.pathTracker  = pathOk;
    bootReport.mission      = missOk;
    bootReport.tasks        = tasksOk;
    bootReport.vision       = visionOk;
    bootReport.trace        = traceOk;

    // The measured voltage itself, not just the verdict — the BATT screen prints it as the
    // pack reading at boot, which is what the operator compares a later live read against.
    bootReport.battVoltageAtBoot = battVoltage;

    // Order matters: setBootReport() BEFORE begin(), per OperatorPanel.h — and both AFTER
    // every module the panel samples has had its own begin() called. OperatorPanel.begin()
    // does no hardware I/O (it is pure state reset, and OperatorConsole.begin() likewise),
    // so it cannot itself fail the boot and is deliberately absent from the verdict below.
    OperatorPanel.setBootReport(bootReport);
    OperatorPanel.begin();

    // BootReport::clean() is the AND of the same 13 flags this line used to spell out by
    // hand — one definition of "healthy boot", shared with the console.
    _setupFailed = !bootReport.clean();

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
        if (!yawOk)    { Serial.println(F("  - YawSource"));    EventTrace.pushEvent("FAIL YAW"); }
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
        // while loop() keeps running the full cooperative tick — there is no
        // _setupFailed early-return any more (AC23), so the diagnostics console
        // stays reachable for inspecting exactly this failed boot.
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

    // -----------------------------------------------------------------------
    // Diagnostics console entry: BTN_DOWN held >= Config::UI_MENU_LONGPRESS_MS.
    //
    // Deliberately NOT behind `_setupFailed` (AC23). The old unconditional
    // `if (_setupFailed) return;` sat right above here and made the menu
    // unreachable after exactly the boot the operator most needs to inspect —
    // the console is read-only, its BATT screen carries the boot report, and
    // every entry that writes hardware is already non-dispatchable while the
    // report is unclean (AC24). The arm path below stays guarded instead.
    //
    // Still gated on `!_missionArmed` so we never intercept the BTN_DOWN
    // fault-acknowledge path that activates after a Mission fault (AC18), and
    // on `!OperatorPanel.isOpen()` so a held BTN_DOWN inside the console — the
    // cursor-scroll button — cannot re-trigger entry underneath itself.
    // -----------------------------------------------------------------------
    if (!_missionArmed && !OperatorPanel.isOpen())
    {
        if (MiniR4.BTN_DOWN.getState())
        {
            if (_btnDownPressStartMs == 0)
            {
                _btnDownPressStartMs = millis();
            }
            else if ((millis() - _btnDownPressStartMs) >= Config::UI_MENU_LONGPRESS_MS)
            {
                Serial.println(F("[main] opening DIAGNOSTICS console (BTN_DOWN long-press)"));

                // Cleared BEFORE open() so the arm path's `_btnDownPressStartMs == 0`
                // guard is satisfied again the moment the console closes, and so the
                // hold does not keep accumulating toward a second trigger.
                _btnDownPressStartMs = 0;

                // OperatorPanel.open() pushes "DIAG MENU" to EventTrace itself, on the
                // open transition only — do not duplicate the push here. There is no
                // release-spin: the console's own WAIT_RELEASE state absorbs the still-
                // held button on its opening tick, so the cooperative loop keeps running
                // at full cadence while the operator's thumb is down (AC4).
                OperatorPanel.open();
            }
        }
        else
        {
            // Released before the threshold — reset the stamp so a tap doesn't
            // accumulate toward an unintended trigger.
            _btnDownPressStartMs = 0;
        }
    }

    // Push the armed flag down every pass (single source of truth stays here — no
    // second global), then tick the console. `update()` is a single `isOpen()` call
    // and an immediate return while the console is closed (AC21), so the mission's
    // tick cadence is unchanged by its presence.
    //
    // In `loop()`, NOT in `cooperativeTick()`: that hook is re-entered from inside
    // `Drive::_drainBlocking()` and `Vision::readMosaic()`, and ticking the panel
    // there would re-enter the console in the middle of dispatching its own request.
    OperatorPanel.setMissionArmed(_missionArmed);

    // Sampled BEFORE the panel ticks — the sample point is load-bearing, do not move
    // it below update(). Confirming EXIT inside update() calls _close() directly (it
    // is the one confirm path that skips the WAIT_RELEASE leg), so a naive live
    // isOpen() read in the arm condition below already sees CLOSED on the very pass —
    // and the very BTN_UP press — that closed the console, and would arm the mission
    // off the operator's "leave diagnostics" press. Holding the pre-update state for
    // this pass means the closing press can never satisfy the arm guard; the still-
    // held button then latches _btnUpPrev at the bottom of loop(), so arming next
    // requires a genuine release followed by a fresh press.
    const bool consoleWasOpen = OperatorPanel.isOpen();
    OperatorPanel.update();

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
    // BTN_DOWN diagnostics-console path stays suppressed after the first arm. The
    // re-arm rebinds the frame from scratch via `Localization.begin()` below —
    // the previous run left `_currentNode` on ITS final node, and re-planning
    // from there is what made a re-armed route no-op or take a wrong path.
    //
    // Guard: while BTN_DOWN is being held toward the console long-press threshold,
    // ignore BTN_UP so that accidental thumb-on-both-buttons cannot spuriously
    // arm the mission when the operator believes they are opening the console.
    //
    // `_setupFailed` guard: latched for the whole power cycle. After a failed boot
    // this block still runs on every pass — it is the `!_setupFailed` term inside the
    // condition that short-circuits it — so `_missionArmed` can never become true
    // (AC23) while the console above stays reachable.
    //
    // Console guards: BTN_UP is the console's CONFIRM button, so arming must not
    // shadow it. Two terms enforce that. `!consoleWasOpen` (sampled above, BEFORE
    // OperatorPanel.update()) is the one that actually covers the EXIT confirm:
    // _close() runs inside update(), so on that pass a live isOpen() read is already
    // false again microseconds after the same physical press — the pre-update sample
    // is the only state that still remembers the console owned that press.
    // `!OperatorPanel.isOpen()` stays alongside it as the live check, so the guard
    // holds even if a future panel path ever opens or closes the console outside
    // update(). The exclusion is load-bearing beyond this file:
    // `OperatorConsole::_close()` discards a pending CLEAR_VISION_FAULT without
    // emitting it, which would strand a latched vision fault for the mission; that
    // discard is harmless only because `missionArmed` provably cannot flip to true
    // on any pass that saw the console open. Do not relax either term without
    // revisiting that.
    //
    // The read and `_btnUpPrev` write below stay OUTSIDE both guards, on every pass:
    // a level held across a guarded window must not surface as a phantom rising edge
    // the moment the guard lifts (release the button inside the console, and the
    // stale `_btnUpPrev == true` would otherwise arm on the next genuine press only
    // — or worse, a still-held button would arm the instant the console closed).
    const bool btnUpNow = MiniR4.BTN_UP.getState();
    const bool armable =
        !_missionArmed || (Config::MISSION_ALLOW_REARM && Mission.isIdle());
    if (btnUpNow && !_btnUpPrev && armable && _btnDownPressStartMs == 0 && !_setupFailed &&
        !consoleWasOpen && !OperatorPanel.isOpen())
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
