/**
 * @file OperatorPanel.h
 * @brief Hardware adapter for the operator diagnostics console (ADR-076).
 * @author WRO 2026
 *
 * @details
 * `OperatorPanelClass` is the thin, hardware-facing half of the `DIAGNOSTICS` console. It owns
 * every peripheral touch the headless `OperatorConsoleClass` core is forbidden from making:
 * `MiniR4.BTN_UP` / `BTN_DOWN`, the OLED, `MiniR4.PWR`, `YawSource`, `WROLineArray` (`LineArray`),
 * `VisionClass` (`Vision`), `Drive` (calibration only) and `MiniR4.LED` / `MiniR4.Buzzer`. It
 * holds no FSM state of its own — `open()`, `isOpen()` and `wantsLineData()` all delegate
 * straight to `OperatorConsole` — and its only persistent state is the hardware sample
 * (`Console::DiagSample`), the boot report, the armed flag, the single in-flight-request slot, and
 * the timing state for its three independent cadences (PUSHED-group refresh, OLED repaint,
 * verdict-signal sequencer).
 *
 * **AC21 is the load-bearing behaviour of this class.** `update()` early-returns the instant
 * `!OperatorConsole.isOpen()`, and that gate is the FIRST statement of the method: no button read,
 * no sampling, no `readBoard()` / `probeBoard()`, no `YawSource` / `Vision` / `MiniR4.PWR` read,
 * no OLED write, no LED/buzzer drive. The button poll sits BELOW the gate deliberately —
 * `MiniR4.BTN_*.getState()` is not a GPIO read but an `mmL` SoftwareSerial round-trip to the
 * STM32 (a `WaitData(..., 100)` busy-spin worst case), and `update()` runs from `loop()` on every
 * iteration including mid-mission, so two such transactions per tick — both discarded two lines
 * later — are not noise against the measured ~8 ms / ~124 Hz tick. Skipping them owes nothing:
 * the core seeds its own `_btnUpPrev` / `_btnDownPrev` from `in.btnUp` / `in.btnDown` on the
 * opening tick, so the adapter carries no cross-tick button continuity while closed.
 *
 * A core-side-only guard (checking `missionArmed` only inside `OperatorConsole::update()`) would
 * sample first and gate second, injecting I2C traffic onto the Wire1 bus the line array and the
 * external gyro share during a competition run — which is why the gate lives here.
 *
 * This class's own `_missionArmed` check is NOT an independent second opinion:
 * `Console::ConsoleInput::missionArmed` is the very same field, so both gates fail together if
 * the sketch never calls `setMissionArmed()`. Its value is ORDERING — it skips SAMPLING (while
 * still calling `OperatorConsole::update()`, so the core's own force-close can run) on the
 * one-tick window where `OperatorConsole::isOpen()` reports true because a just-latched `open()`
 * request has not yet been consumed and force-closed. Unreachable today (T7 only calls `open()`
 * pre-arm) but cheap to close.
 *
 * **PUSHED vs PULLED (see `ConsoleTypes.h`'s `DiagSample` docs for the full contract).** This class
 * refreshes the PUSHED group (`battVolts`/`battPercent`/`battReadOk`/`battRetryPending`,
 * `gyroYawDeg`/`gyroHealthy`/`gyroReadOk`, `junctionType`/`lineOnline`/`lineError`) autonomously on
 * a cadence. Every PULLED field (`lineRaw`/`lineBoardPresent`/`lineBoardOk`/`lineBoardCount`,
 * `visionCell`/`visionFaultKind`) is written ONLY while executing the matching
 * `Console::ActionRequest` the core emitted in a prior frame — `readBoard()`, `probeBoard()` and
 * `Vision::readMosaic()` are NEVER called from the PUSHED-group cadence.
 *
 * **Adapter tick order (pinned invariant — see the design plan's "Contracts & Interfaces"):**
 * if closed, return -> poll buttons -> refresh the PUSHED group if `Config::UI_VIEW_SAMPLE_MS`
 * elapsed (bump `seq`) -> if a request pended from the previous tick, execute it, write its
 * PULLED fields, bump `seq`, then call `notifyReadDone()` / `notifyActionDone()` -> re-read
 * `millis()` (a dispatched `LineArray.calibrate()` blocks for tens of seconds, so the pre-dispatch
 * capture is stale by then) -> `OperatorConsole.update(in, frame)` -> render if `frame.dirty` and
 * `Config::UI_REPAINT_INTERVAL_MS` elapsed, then `notifyFrameRendered()` -> drive `frame.signal`,
 * `notifySignalDone()` when the pattern completes -> latch `frame.action` as this tick's pending
 * request. At most one request is ever in flight, and the tick after a `notify*` call already sees
 * that request's result.
 *
 * **No `delay()` anywhere.** The battery read retry (`Config::UI_BATT_RETRY_MS` /
 * `UI_BATT_RETRY_ATTEMPTS`) and the verdict LED/buzzer sequencer are both `millis()`-deadline
 * state machines — the boot-time chirp loop in `WRO2026.ino` uses `delay()` between chirps and is
 * sanctioned only because `setup()` blocks; this class runs from an `update()` path, where iron
 * rule 1 forbids it.
 *
 * @note This translation unit's `.cpp` DOES include `<MatrixMiniR4.h>` and calls
 *       `setTextColor(WHITE)` — that is correct there. This HEADER stays free of `<Arduino.h>` /
 *       `<MatrixMiniR4.h>` because none of its members need a hardware type; only `<stdint.h>` and
 *       `ConsoleTypes.h` are required.
 * @note NOT host-linked (it owns the OLED and every hardware peripheral). `arduino-cli compile`
 *       and hardware bring-up are the only things that will catch a mistake here.
 */

#ifndef OPERATORPANEL_H
#define OPERATORPANEL_H

#include <stdint.h>

#include "ConsoleTypes.h"

/**
 * @brief Hardware adapter driving `OperatorConsoleClass` from real peripherals.
 *
 * @details
 * Singleton (`extern OperatorPanelClass OperatorPanel;`, defined in `OperatorPanel.cpp`) per iron
 * rule 2. Holds no FSM state — every state query delegates to `OperatorConsole` — and every
 * hardware-facing method is decomposed into small, named helpers (`_servicePushedGroup`,
 * `_serviceRequestCompletion`, `_renderFrame`, `_serviceSignal`) rather than one monolithic
 * `update()`.
 */
class OperatorPanelClass
{
public:
    /**
     * @brief Reset all adapter-owned state and initialise the console core.
     *
     * Calls `OperatorConsole.begin()`, clears the armed flag and the single in-flight-request
     * slot, resets `Console::DiagSample` to its all-sentinel defaults (every PUSHED field at its
     * documented "read failed" value, every PULLED field zeroed), and primes every cadence timer
     * (`_lastPushMs`, `_battLastAttemptMs`, `_lastRenderMs`) to `0` so the very first `update()`
     * after opening refreshes and repaints immediately rather than waiting out a full cadence
     * period.
     *
     * @return `true` unconditionally — declared `bool` for parity with every other `*::begin()`
     *         in this codebase (`Drive`, `Manipulator`, `Vision`, ...) so a future failure path
     *         can be added without touching the call site.
     * @note Call once from `setup()`, after `setBootReport()` (see that method) and after every
     *       hardware module this class samples has already had its own `begin()` called.
     * @note Does NO hardware I/O itself — `OperatorConsole.begin()` is likewise pure state reset.
     */
    bool begin();

    /**
     * @brief Trigger the console to open. Delegates entirely to `OperatorConsole.open()`.
     *
     * @note Holds no state of its own — see the class-level docs. Also pushes the `"DIAG MENU"`
     *       event to `EventTrace` on the OPEN TRANSITION only (mirrors the pre-console menu's
     *       `"CAL MENU"` push). `OperatorConsole::open()` no-ops when the console is already open,
     *       so the transition is sampled via `isOpen()` BEFORE delegating; a repeat call must not
     *       spam the 4-slot event ring.
     */
    void open();

    /**
     * @brief Advance the adapter one tick: sample hardware, drive the console core, render.
     *
     * See the class-level docs for the full pinned tick order. `if (!OperatorConsole.isOpen())
     * return;` is the FIRST statement (AC21), so a closed console costs one `isOpen()` call and
     * nothing else — not even the two button reads, which are `mmL` round-trips (class docs).
     *
     * @note Call from `loop()`, NEVER from `cooperativeTick()` — that hook is re-entered from
     *       inside `Drive::_drainBlocking()` and `Vision::readMosaic()`'s blocking spins, and
     *       placing the panel there would re-enter the console mid-dispatch.
     */
    void update();

    /**
     * @brief Whether the console currently owns the panel. Delegates to `OperatorConsole.isOpen()`.
     * @return `true` while the console FSM is anywhere other than `Console::State::CLOSED`, or a
     *         latched `open()` request is still awaiting its consuming `update()`.
     */
    bool isOpen() const;

    /**
     * @brief Whether the line array must be ticked this cycle. Delegates to
     *        `OperatorConsole.wantsLineData()`.
     * @return `true` if the sketch's `lineArrayNeededThisTick()` gate must be forced open.
     */
    bool wantsLineData() const;

    /**
     * @brief Publish the 13 boot health flags captured once in `setup()`.
     *
     * @param boot The boot report to store. Consulted every tick to gate PUSHED sampling (AC25 —
     *             `YawSource` skipped when `!boot.yaw`, line telemetry skipped whenever
     *             `!wantsLineData()`, which itself folds in `boot.lineArray`) and to defensively
     *             veto executing an `ActionRequest` for a subsystem whose `begin()` failed.
     * @note Call once from `setup()`, before `begin()`. `OperatorConsole` receives the SAME report
     *       every tick via `Console::ConsoleInput::boot` — this class is just the holder.
     */
    void setBootReport(const Console::BootReport& boot);

    /**
     * @brief Publish the sketch's mission-armed flag every tick.
     *
     * @param armed The sketch's `_missionArmed` global. Fed into `Console::ConsoleInput::
     *              missionArmed` every `update()` so `OperatorConsole` can force-close, and
     *              consulted directly by this class's own belt-and-braces sampling guard (see the
     *              class-level docs).
     * @note No second global — `WRO2026.ino` (T7) is the single source of truth for the armed
     *       flag; this method just mirrors it here.
     */
    void setMissionArmed(bool armed);

private:
    // -----------------------------------------------------------------------------------------
    // PUSHED-group sampling (cadence: Config::UI_VIEW_SAMPLE_MS)
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Refresh the PUSHED group's three sub-groups on their respective cadences.
     *
     * Gyro and line-derived telemetry refresh together every `Config::UI_VIEW_SAMPLE_MS`.
     * Battery refreshes on the SAME cadence normally, but narrows to `Config::UI_BATT_RETRY_MS`
     * while a retry is pending (`Console::DiagSample::battRetryPending`), so the non-blocking
     * retry lands quickly instead of waiting out a full group period.
     *
     * @param now `millis()` sampled at adapter-tick entry, i.e. when the sample was taken.
     */
    void _servicePushedGroup(uint32_t now);

    /** @brief Refresh `gyroYawDeg` / `gyroReadOk` / `gyroHealthy`. No-op if `!_boot.yaw` (AC25). */
    void _refreshGyro();

    /**
     * @brief Refresh `junctionType` / `lineOnline` / `lineError` from `LineArray`'s cache readers.
     *
     * Gated on `OperatorConsole.wantsLineData()`, which is itself always `false` when
     * `!_boot.lineArray` (AC25) — a single gate covers both "is a view/check asking for this" and
     * "did the module even come up".
     */
    void _refreshLineDerived();

    /**
     * @brief Non-blocking battery read with a `millis()`-deadline retry.
     *
     * On a `<= 0.0f` read, publishes `battReadOk = false`, `battVolts = 0.0f`, `battPercent = 0`
     * and sets `battRetryPending = true` until `Config::UI_BATT_RETRY_ATTEMPTS` total attempts
     * have been made, at which point it latches `battRetryPending = false` (giving up). Runs
     * regardless of `_boot.batt` — a boot-time low-voltage verdict never disables the live read,
     * since the pack may have recovered (Decisions: "`boot.batt` is a voltage verdict, not an init
     * verdict").
     */
    void _refreshBattery();

    /** @brief Bump `Console::DiagSample::seq` — called after every PUSHED or PULLED refresh. */
    void _bumpSeq();

    // -----------------------------------------------------------------------------------------
    // PULLED execution — the single in-flight ActionRequest slot
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Execute the request latched from the PREVIOUS tick's frame, if any.
     *
     * Dispatches on `_pendingAction`, defensively re-checking dispatchability against `_boot`
     * before touching any hardware (`Console::isDispatchable()` for the three menu-item actions;
     * the owning boot flag directly for the two reads) — the core should never emit a request for
     * a non-dispatchable entry, but this is the last line of defence before a real transaction.
     * A rejected request executes nothing, yet is still REPORTED back through `_refuseRequest()`
     * (see that method) — a silent drop would wedge the core.
     *
     * @note Clears `_pendingAction` unconditionally at entry — the slot is consumed whether or
     *       not the request turns out to be legitimate.
     */
    void _serviceRequestCompletion();

    /** @brief Execute `ActionRequest::READ_MOSAIC`: `Vision.readMosaic()` + latch the 12 cells. */
    void _executeReadMosaic();

    /** @brief Execute `ActionRequest::READ_LINE_BOARDS`: `probeBoard()`+`readBoard()` per board. */
    void _executeReadLineBoards();

    /**
     * @brief Report an `ActionRequest` this adapter refuses to execute, and log it to Serial.
     *
     * The refusal MUST be reported, never swallowed. For `CAL_LINE` / `CAL_MOTOR` / `GYRO_ZERO`
     * the core has already entered `Console::State::WAIT_RELEASE` with its pending-action flag
     * set, and that wait has no deadline and no button escape — it releases only on the adapter's
     * report. A silent drop would wedge the console there for the rest of the power cycle, with
     * the mission unarmable behind it. Side effects therefore report
     * `notifyActionDone(action, false)` (raising a visible `Console::Signal::FAIL`); the two reads
     * report `notifyReadDone(action)` for symmetry — their absence would merely time out
     * fail-closed in the SELF TEST freshness gate, but silence is the wrong shape. Also pushes
     * `"REFUSED"` to `EventTrace` alongside the `Serial.print` (same reasoning as the `"ST PASS"` /
     * `"ST FAIL"` pushes: the event ring is what the operator reads when something has gone wrong,
     * and a refusal driving the FAIL LEDs/tone with no matching ring entry reads as an
     * indistinguishable, dropped genuine failure).
     *
     * @param action The rejected request — decides which `notify*` channel reports it.
     * @param which  Short diagnostic name of the rejected request, for the Serial line.
     */
    void _refuseRequest(Console::ActionRequest action, const char* which);

    // -----------------------------------------------------------------------------------------
    // Rendering + verdict signal
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Repaint the OLED from @p frame if `frame.dirty` and the repaint cadence has elapsed.
     * @param frame This tick's rendered frame.
     * @param now   `millis()` re-read after the request-dispatch half of the tick, so a repaint
     *              following a blocking `CAL LINE` is not timestamped tens of seconds in the past.
     * @note Calls `OperatorConsole.notifyFrameRendered()` only on an actual repaint — `dirty` is
     *       level-triggered, so a repaint suppressed by the cadence is retried next tick.
     */
    void _renderFrame(const Console::ConsoleFrame& frame, uint32_t now);

    /**
     * @brief Timestamp-driven LED + buzzer verdict sequencer.
     *
     * Reuses the boot-OK (green + `Config::BOOT_OK_BEEPS` chirps) / boot-FAIL (red + one long
     * tone) patterns from `setup()`, converted from `delay()`-spaced chirps to a `millis()`-driven
     * schedule. Calls `OperatorConsole.notifySignalDone()` exactly once, the tick the pattern
     * finishes playing.
     *
     * @param signal This tick's `Console::ConsoleFrame::signal` (level-triggered until acked).
     * @param now    `millis()` after the request-dispatch half of the tick.
     * @note The LED/buzzer drive is unconditional — `Console::Signal` is a GENERIC verdict
     *       channel, and a failed `CAL MOTOR` earns the same red-and-tone as a failed SELF TEST.
     *       The `"ST PASS"` / `"ST FAIL"` `EventTrace` pushes are NOT: they are gated on the
     *       actually originating from the SELF TEST sequencer (`OperatorConsole.state() ==
     *       Console::State::SELF_TEST`), so a successful motor-bias calibration never reads back
     *       as a passed self test on the OLED trace or the Serial heartbeat.
     * @note **Known limitation — a same-kind verdict arriving mid-pattern is silently absorbed.**
     *       The start block above is gated on `!_signalActive || _signalKind != signal`, so a
     *       SECOND verdict of the SAME `Console::Signal` kind arriving while the first pattern is
     *       still playing repaints no LEDs and reschedules no chirps — and because
     *       `_signalChirpsFired` is already at its max and `elapsed` already past `totalMs`,
     *       `notifySignalDone()` fires on the very next tick, acking a verdict that was never
     *       actually played. Reachable path: SELF TEST reaches PASS -> operator aborts with
     *       BTN_DOWN -> confirms GYRO ZERO, all inside the 750 ms PASS pattern. Consequence is one
     *       missing confirmation chirp, not a wedge — deliberately deferred rather than fixed.
     *       The correct fix is a monotonic signal counter the core writes on every `_signal`
     *       write, with this adapter restarting whenever that counter changes: a `_signalKind`
     *       comparison alone provably cannot distinguish "same verdict, still playing" from "new
     *       verdict, same kind". That ripples through `ConsoleTypes.h`, the core, this adapter,
     *       and the host tests — not worth it for a missed beep.
     */
    void _serviceSignal(Console::Signal signal, uint32_t now);

    /**
     * @brief Push the `EventTrace` transition events ("SELF TEST" / "DIAG EXIT") for a state edge.
     * @param before `OperatorConsole.state()` before this tick's `update()` call.
     * @param after  `OperatorConsole.state()` after it.
     */
    void _traceStateTransition(Console::State before, Console::State after);

    // -----------------------------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------------------------

    Console::BootReport _boot{};    ///< Published by `setBootReport()`; all-false until then.
    bool _missionArmed = false;     ///< Published by `setMissionArmed()` every tick.

    /// Adapter-owned hardware sample. PUSHED fields refresh on cadence; PULLED fields are written
    /// ONLY inside `_executeReadMosaic()` / `_executeReadLineBoards()`. Persists byte-identical
    /// across ticks otherwise — see the class-level PUSHED/PULLED docs.
    Console::DiagSample _sample{};

    /// The request latched from the frame `OperatorConsole.update()` produced last tick; executed
    /// at the top of THIS tick by `_serviceRequestCompletion()`, then cleared. `NONE` means the
    /// single request slot is free.
    Console::ActionRequest _pendingAction = Console::ActionRequest::NONE;

    uint32_t _lastPushMs        = 0;   ///< `nowMs` of the last gyro/line-derived PUSHED refresh.
    uint32_t _battLastAttemptMs = 0;   ///< `nowMs` of the last battery read attempt.
    uint8_t  _battAttemptsMade  = 0;   ///< Attempts made in the current battery retry sequence.

    uint32_t _lastRenderMs = 0;   ///< `nowMs` of the last actual OLED repaint.

    bool            _signalActive      = false;                ///< A verdict pattern is playing.
    Console::Signal _signalKind        = Console::Signal::NONE; ///< Which pattern is playing.
    uint32_t        _signalStartMs     = 0;                     ///< `nowMs` the pattern started.
    uint8_t         _signalChirpsFired = 0;                     ///< PASS chirps fired so far.
};

/**
 * @brief Single global instance — defined in OperatorPanel.cpp.
 */
extern OperatorPanelClass OperatorPanel;

#endif    // OPERATORPANEL_H
