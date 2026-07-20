/**
 * @file OperatorConsole.h
 * @brief Headless core of the operator diagnostics console — FSM, item list and live views.
 * @author WRO 2026
 *
 * @details
 * `OperatorConsoleClass` is the OLED-free half of the `DIAGNOSTICS` console (ADR-076). It owns
 * the nine-entry item table, the scroll/confirm FSM, button edge detection, the `WAIT_RELEASE`
 * state, the four live-view states and their row formatting, the wrap-correct gyro-drift watch,
 * and the pull cadence that asks the adapter for the expensive reads. It consumes a pushed-in
 * `Console::ConsoleInput` sample and emits a `Console::ConsoleFrame` — four bounded text rows
 * plus a repaint flag, an action request and a verdict signal. Everything hardware-facing
 * (OLED, buttons, I2C, LEDs, buzzer) lives one layer up in `OperatorPanelClass`.
 *
 * **Purity is the point.** This translation unit includes `<stdint.h>`, `ConsoleTypes.h` and the
 * two `<stdint.h>`-only config sub-headers — never `<Arduino.h>`, never `<MatrixMiniR4.h>`,
 * never the `Config.h` umbrella (which transitively pulls `<Arduino.h>`). It calls no `millis()`,
 * performs no I/O, allocates nothing, and holds no mutable state outside the singleton instance.
 * `ConsoleInput::nowMs` is the ONLY time source, and every comparison against it is written
 * rollover-safe as `(uint32_t)(now - then) >= interval`. That is what lets the core compile and
 * link into the host ctest harness with zero mocks (AC19); a single stray include destroys it.
 *
 * Data flow (the hybrid push/pull contract — see `ConsoleTypes.h` for the field-level split):
 *  - The adapter PUSHES the cheap cached state (battery, gyro, line-derived telemetry) on its own
 *    cadence, bumping `DiagSample::seq` on every refresh.
 *  - The core PULLS everything expensive or latching by emitting a `Console::ActionRequest` in
 *    the frame; the adapter executes it, writes the matching PULLED fields, and reports back via
 *    `notifyReadDone()` (reads) or `notifyActionDone()` (side effects). At most ONE request is
 *    ever in flight.
 *
 * Lifecycle:
 *  - `begin()` once from `setup()` (via the adapter), before any `update()`.
 *  - `open()` latches a one-shot open request; the NEXT `update()` consumes it.
 *  - `update(in, out)` once per adapter tick while open. Non-blocking, returns within one tick.
 *  - `notifyFrameRendered()` / `notifySignalDone()` acknowledge the level-triggered `dirty` and
 *    `signal` outputs, so a repaint suppressed by the adapter's repaint cadence is never lost.
 *
 * @note The `Console::State::SELF_TEST` sequencer runs strictly on FRESH data: every check
 *       latches `DiagSample::seq` (plus, for a check that dispatches a read, that read's
 *       `notifyReadDone()`) on entry and judges nothing until both advance, failing closed on
 *       `Config::UI_SELFTEST_CHECK_TIMEOUT_MS`. That is what stops the vision check PASSing an
 *       unplugged camera off the boot state, which `Vision::begin()` leaves fault-free.
 */

#ifndef OPERATORCONSOLE_H
#define OPERATORCONSOLE_H

#include <stdint.h>

#include "ConsoleTypes.h"

/**
 * @brief Pure, host-testable core of the operator diagnostics console.
 *
 * @details
 * Holds the whole console FSM and rendering surface. The class is a singleton
 * (`extern OperatorConsoleClass OperatorConsole;`, defined once in `OperatorConsole.cpp`) per
 * iron rule 2, and every member is plain data — no pointers, no allocation, no I/O.
 *
 * State model:
 *  - `CLOSED` — inert. `update()` writes a blank frame and consumes nothing but the open latch.
 *  - `LIST` — the nine-entry scrolling list; BTN_DOWN advances (wrapping 8 -> 0), BTN_UP
 *    confirms.
 *  - `VIEW_LINE` / `VIEW_VISION` / `VIEW_GYRO` / `VIEW_BATT` — live screens. BTN_DOWN returns to
 *    `LIST` with the cursor preserved. **No view has a timeout.**
 *  - `SELF_TEST` — the four-check sequencer (`BATT` -> `GYRO` -> `VISION` -> `LINE`), halting on
 *    the first failure. BTN_DOWN aborts back to `LIST`.
 *  - `WAIT_RELEASE` — two distinct waits sharing one state, discriminated by an explicit
 *    pending-action flag: an *await-release* (both buttons were held, so no edge may be trusted
 *    until both read released for one tick) and an *await-completion* (a side-effect action is in
 *    flight and only its `notifyActionDone()` may release it). The two are never conflated.
 *
 * Leaving `WAIT_RELEASE` takes one of two paths, and the difference is load-bearing:
 *  - a **resume** (`_resumeState()`) returns to the very view the chord was pressed in. The
 *    console never left it, so NO entry effect runs — the `VIEW_GYRO` drift baseline, the
 *    `VIEW_LINE` / `VIEW_VISION` pull throttles and any in-flight read all survive untouched.
 *  - a **transition** (`_enterState()`) arrives somewhere new — every await-completion wait, and
 *    every list confirm — and DOES run entry effects.
 * `_beginWait()` classifies the wait once, at the moment it is entered; `_updateWaitRelease()`
 * merely dispatches on that verdict.
 */
class OperatorConsoleClass
{
public:
    // -----------------------------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Reset the console to `State::CLOSED` with a clean cursor, frame and latch set.
     *
     * Clears the open latch, the cursor and scroll window, the button edge history, every
     * in-flight/pending request, the repaint and signal levels, the gyro drift baseline and the
     * cached frame contents. Must be called once from `setup()` (through
     * `OperatorPanelClass::begin()`) before the first `update()`.
     *
     * @note Does NO hardware work — there is none to do. The core never touches a peripheral.
     */
    void begin();

    /**
     * @brief Latch a one-shot request to open the console; the NEXT `update()` consumes it.
     *
     * On consumption the console goes `CLOSED` -> `State::LIST` with the cursor reset to
     * `Console::Item::SELF_TEST` and the scroll window re-homed. The latch is cleared both on
     * consumption and on a `missionArmed` force-close, so a held button or a repeated call can
     * never re-open the console or reset a live view that is already on screen.
     *
     * @note No-op when the console is already open (`isOpen()`), which includes the window
     *       between a latching `open()` and the `update()` that consumes it.
     * @note If `ConsoleInput::missionArmed` is `true` when the latch is consumed, the latch is
     *       discarded WITHOUT opening — the console is unavailable once the mission is armed.
     */
    void open();

    // -----------------------------------------------------------------------------------------
    // Per-tick entry point
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Advance the console one tick and render the resulting frame.
     *
     * Consumes the open latch, force-closes on `in.missionArmed`, detects button edges against
     * the previous tick's levels, runs the active state's handler, services the view pull cadence
     * and formats the four text rows. Always writes every field of @p out.
     *
     * @param in  This tick's button levels, time base, armed flag, hardware sample and boot
     *            report. `in.nowMs` is the console's ONLY time source.
     * @param out Receives the rendered frame: bounded NUL-terminated rows, cursor row, the
     *            level-triggered `dirty` flag, at most one `ActionRequest`, and the
     *            level-triggered `signal`.
     *
     * @note **Pure and non-blocking.** No I/O, no `millis()`, no `delay()`, no statics outside
     *       this instance; returns within one tick from every state.
     * @note At most one request is emitted per frame, and never a second read while a previous
     *       one is unacknowledged.
     */
    void update(const Console::ConsoleInput& in, Console::ConsoleFrame& out);

    // -----------------------------------------------------------------------------------------
    // Adapter callbacks
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Report completion of a dispatched SIDE-EFFECT action.
     *
     * Valid only for `ActionRequest::CAL_LINE`, `CAL_MOTOR`, `GYRO_ZERO` and
     * `CLEAR_VISION_FAULT`; any other value (in particular the two reads, which carry no verdict
     * and must use `notifyReadDone()`) is ignored. A report that does not match the action
     * currently in flight is ignored.
     *
     * Completing the action the FSM is waiting on releases the `WAIT_RELEASE` await-completion
     * wait; the await-release half still applies, so a still-held button cannot immediately
     * re-trigger. For an operator-confirmed action @p ok is published as a `Signal::PASS` /
     * `Signal::FAIL` verdict for the adapter's LED + buzzer sequencer.
     *
     * @param which The action that completed.
     * @param ok    `true` if it succeeded — the SOLE authority on success for a side-effect
     *              action (the sample carries no verdict for one).
     */
    void notifyActionDone(Console::ActionRequest which, bool ok);

    /**
     * @brief Report completion of a dispatched READ.
     *
     * Valid only for `ActionRequest::READ_MOSAIC` and `READ_LINE_BOARDS`; any other value is
     * ignored. Carries no verdict by design — the refreshed PULLED fields of `DiagSample` are
     * the sole authority on what was read, and the adapter must have refreshed them (bumping
     * `DiagSample::seq`) BEFORE calling this.
     *
     * @param which The read that completed. Clears the in-flight slot so the next cadence tick
     *              may issue a fresh request.
     */
    void notifyReadDone(Console::ActionRequest which);

    /**
     * @brief Acknowledge that the frame just emitted was painted; clears `ConsoleFrame::dirty`.
     *
     * `dirty` is LEVEL-triggered: it is raised whenever the rendered rows differ from the last
     * acknowledged ones and stays raised across ticks until this call arrives, so a repaint the
     * adapter suppressed for `Config::UI_REPAINT_INTERVAL_MS` is never lost.
     */
    void notifyFrameRendered();

    /**
     * @brief Acknowledge that the pending verdict signal finished playing; clears
     *        `ConsoleFrame::signal`.
     *
     * `signal` is LEVEL-triggered for the same reason as `dirty`: the adapter's LED + buzzer
     * pattern spans many ticks, so the request must survive until the pattern completes.
     */
    void notifySignalDone();

    // -----------------------------------------------------------------------------------------
    // Queries
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Whether the console currently owns the panel.
     *
     * @return `true` while the FSM is in any state other than `Console::State::CLOSED`, AND
     *         during the window between a latching `open()` and the `update()` that consumes
     *         the latch.
     * @note The latch is deliberately included: `OperatorPanelClass::update()` early-returns
     *       while the console is closed, so a latch excluded here would never be handed the
     *       `update()` that consumes it.
     */
    bool isOpen() const;

    /**
     * @brief Whether the line array must be ticked and its derived telemetry sampled this tick.
     *
     * `true` in `State::VIEW_LINE`, and inside SELF TEST at `Console::Check::LINE` from the tick
     * `ActionRequest::READ_LINE_BOARDS` is emitted until the line verdict is recorded.
     *
     * @return `true` if the sketch's line-array tick gate must be forced open.
     * @note ALWAYS `false` when the most recent `update()` saw `boot.lineArray == false` (AC25 —
     *       a module whose `begin()` failed is never sampled), and `false` in `CLOSED` and
     *       `VIEW_GYRO`. Gates the PUSHED line-derived fields and `LineArray` ticking only; it
     *       never authorises a raw `readBoard()` — that is exclusively `READ_LINE_BOARDS` work.
     */
    bool wantsLineData() const;

    /**
     * @brief The current FSM state.
     * @return The active `Console::State`.
     */
    Console::State state() const;

private:
    // -----------------------------------------------------------------------------------------
    // FSM helpers
    // -----------------------------------------------------------------------------------------

    /**
     * @brief Close the console and drop every transient: latch, waits, requests, levels, frame.
     *
     * Used by the `missionArmed` force-close and by confirming `Console::Item::EXIT`, so a later
     * re-open always starts from a clean list rather than inheriting a half-finished view.
     */
    void _close();

    /**
     * @brief Consume the one-shot open latch if one is pending and the console is closed.
     *
     * @param in This tick's input; its button levels seed the edge history so the long-press
     *           that opened the console cannot also be read as a list edge on the opening tick.
     * @return `true` if the console opened on this tick (the caller then skips button handling
     *         for this tick).
     */
    bool _consumeOpenLatch(const Console::ConsoleInput& in);

    /**
     * @brief TRANSITION into @p next: run that state's entry effects, then become @p next.
     *
     * This is the *transition* half of the two-way split with `_resumeState()`. Call it only
     * when the console is genuinely arriving at a view it was not already showing — every entry
     * effect below is destructive to the departing view's accumulated context.
     *
     * Entry effects: `VIEW_GYRO` drops the drift baseline (so re-entry re-baselines to zero
     * drift), `VIEW_LINE` / `VIEW_VISION` disarm their pull throttles (so the first tick in the
     * view issues its read immediately), and `SELF_TEST` re-homes the sequencer to
     * `Console::Check::BATT`. Independently of @p next, an in-flight read whose owning view is
     * not @p next is dropped (see `_stateOwnsRead`), so a `notifyReadDone()` the adapter never
     * delivered cannot poison the single read slot for the rest of the session.
     *
     * @param next The state to enter.
     * @note Latching a `CLEAR_VISION_FAULT` on leaving `VIEW_VISION` is NOT an entry effect —
     *       it belongs to the BTN_DOWN handler (`_handleViewButtons`) and to the SELF TEST vision
     *       check (`_selfTestAdvance()`), so that a chord *resume* back into `VIEW_VISION` never
     *       owes a spurious clear.
     */
    void _enterState(Console::State next);

    /**
     * @brief RESUME @p next: become @p next WITHOUT running any entry effect.
     *
     * The counterpart to `_enterState()`. Used on exactly one path: releasing an await-RELEASE
     * `WAIT_RELEASE` that was entered from @p next itself (the both-button chord). The console
     * never left that view, so re-running its entry effects would silently re-baseline the
     * `VIEW_GYRO` drift watch (AC8 measures drift since the view OPENED, not since the last
     * thumb-brush) and disarm the `VIEW_VISION` pull throttle into an out-of-cadence
     * `READ_MOSAIC`. The in-flight read is likewise preserved — the owning view is still on
     * screen.
     *
     * @param next The state to resume; always equal to the state the wait was entered from.
     */
    void _resumeState(Console::State next);

    /**
     * @brief Enter `State::WAIT_RELEASE`, recording what the wait is for and where it leads.
     *
     * Also classifies the wait as a *resume* or a *transition* and stores the verdict in
     * `_waitIsResume`, which decides whether the release runs entry effects. The test is
     * evaluated against the CURRENT state before `_state` is overwritten: a plain await-release
     * wait whose @p next is the state we are standing in is a resume; everything else — in
     * particular every await-completion wait, which targets `State::LIST` from `State::LIST` —
     * is a transition and must run entry effects.
     *
     * @param next          The state to enter once the wait clears.
     * @param pendingAction `true` for an await-COMPLETION wait (a side-effect action is in
     *                      flight and only `notifyActionDone()` may release it); `false` for a
     *                      plain await-RELEASE wait. The two are never conflated — an
     *                      await-completion wait additionally serves the release gate afterwards.
     */
    void _beginWait(Console::State next, bool pendingAction);

    /**
     * @brief Service `State::WAIT_RELEASE`: the completion gate first, then the release gate.
     *
     * On release, dispatches on `_waitIsResume`: a resume goes through `_resumeState()` (no
     * entry effects, in-flight read preserved), a transition through `_enterState()`.
     *
     * @param in This tick's input (button levels).
     */
    void _updateWaitRelease(const Console::ConsoleInput& in);

    /**
     * @brief Handle button edges while in `State::LIST` (cursor advance + confirm).
     *
     * @param in       This tick's input.
     * @param upEdge   BTN_UP rising edge — confirm the highlighted entry.
     * @param downEdge BTN_DOWN rising edge — advance the cursor, wrapping 8 -> 0.
     */
    void _handleListButtons(const Console::ConsoleInput& in, bool upEdge, bool downEdge);

    /**
     * @brief Handle button edges while a live view (or the SELF TEST screen) is on show.
     *
     * BTN_DOWN leaves for `State::LIST` with the cursor preserved. Nothing has a timeout.
     *
     * @param in       This tick's input.
     * @param downEdge BTN_DOWN rising edge — exit to the list.
     */
    void _handleViewButtons(const Console::ConsoleInput& in, bool downEdge);

    /**
     * @brief Confirm the entry under the cursor (`State::LIST`, BTN_UP edge).
     *
     * A non-dispatchable entry (see `Console::isDispatchable`) is a no-op that emits
     * `ActionRequest::NONE` — the `"! "` row prefix is the operator's feedback.
     *
     * @param in This tick's input; `in.boot` decides dispatchability.
     */
    void _confirmItem(const Console::ConsoleInput& in);

    /**
     * @brief Dispatch a side-effect action and park the FSM until the adapter reports back.
     *
     * Emits @p action into this frame's action slot, records it as in flight, and enters the
     * await-completion `WAIT_RELEASE`. Does nothing if the slot is already taken.
     *
     * @param action One of `CAL_LINE`, `CAL_MOTOR` or `GYRO_ZERO`.
     */
    void _dispatchAction(Console::ActionRequest action);

    /**
     * @brief Re-home the scroll window so the cursor stays visible at both ends of the list.
     *
     * Cursor 0 shows items 0-2; cursor `ITEM_COUNT - 1` shows the last
     * `Console::LIST_VISIBLE_ROWS` items.
     */
    void _scrollWindowTop();

    /**
     * @brief Latch the `VIEW_GYRO` drift baseline from the first good sample after entry.
     * @param in This tick's input; a failed gyro read never (re-)baselines.
     */
    void _updateGyroBaseline(const Console::ConsoleInput& in);

    /**
     * @brief Render the active state into @p out and publish the level-triggered outputs.
     *
     * Blanks the frame, dispatches to the per-state renderer, then raises `dirty` if the rows
     * differ from the last acknowledged frame and copies the tick's action + signal levels out.
     *
     * @param in  This tick's input.
     * @param out Receives the finished frame.
     */
    void _publishFrame(const Console::ConsoleInput& in, Console::ConsoleFrame& out);

    /**
     * @brief SELF TEST sequencer step: latch -> await-fresh -> judge -> dwell, per check.
     *
     * Runs the four checks in the order `Console::Check::BATT` -> `GYRO` -> `VISION` -> `LINE`,
     * halting on the first FAIL (AC11) with `_check` left ON the failing check so the screen can
     * name it. Judges only a sample proven fresh since the active check began (AC22), and fails
     * closed on `Config::UI_SELFTEST_CHECK_TIMEOUT_MS`.
     *
     * @param in This tick's input.
     * @note Never advances `_check` through `_enterState(State::SELF_TEST)` — that
     *       self-transition re-homes `_check` to `Check::BATT` and would loop BATT forever.
     */
    void _updateSelfTest(const Console::ConsoleInput& in);

    /**
     * @brief SELF TEST **latch** phase: baseline the active check and emit its read, if any.
     *
     * Consults `in.boot` BEFORE the sample (AC25) — a subsystem whose `begin()` failed is failed
     * straight from the boot report and is never read, sampled or commanded. Otherwise records
     * `DiagSample::seq` and `nowMs` as the freshness / deadline baseline and dispatches the
     * check's read (`READ_MOSAIC` for `Check::VISION`, `READ_LINE_BOARDS` for `Check::LINE`).
     *
     * @param in This tick's input.
     * @return `true` if the check is latched; `false` if this frame's single action slot was
     *         already taken, in which case NOTHING was mutated and the caller retries next tick
     *         (a check latched without its read would only time out).
     */
    bool _selfTestBeginCheck(const Console::ConsoleInput& in);

    /**
     * @brief SELF TEST **await + judge** phase for the active check.
     *
     * Judges nothing until `_selfTestDataFresh()` holds. On expiry of
     * `Config::UI_SELFTEST_CHECK_TIMEOUT_MS` measured from the check's entry `nowMs`, records a
     * fail-closed timeout verdict rather than PASSing on stale data or stalling forever.
     *
     * @param in This tick's input.
     */
    void _selfTestJudgeCheck(const Console::ConsoleInput& in);

    /**
     * @brief Record the active check's verdict and start its `Config::UI_SELFTEST_STEP_MS` dwell.
     *
     * @param in       This tick's input; `in.nowMs` becomes the dwell base.
     * @param passed   The verdict.
     * @param timedOut `true` when @p passed is `false` because the freshness deadline expired
     *                 (the screen then reads `<SUBSYS> NO DATA` rather than `<SUBSYS> FAIL`).
     */
    void _selfTestRecord(const Console::ConsoleInput& in, bool passed, bool timedOut);

    /**
     * @brief SELF TEST **dwell + advance** phase; called only once a verdict is recorded.
     *
     * Holds the verdict on screen for at least `Config::UI_SELFTEST_STEP_MS` — a readability
     * minimum that runs AFTER the verdict, never in place of it — then either halts on a failure
     * (leaving `_check` on the failing check) or moves to the next check, publishing
     * `Console::Signal::PASS` when the last check passes.
     *
     * @param in This tick's input.
     */
    void _selfTestAdvance(const Console::ConsoleInput& in);

    /**
     * @brief Whether the active check's data is provably fresher than its entry baseline (AC22).
     *
     * Requires `DiagSample::seq` to have advanced past the value latched at check entry, AND —
     * for a check that dispatched a read — the matching `notifyReadDone()` to have arrived. The
     * second half is what stops `Check::VISION` PASSing on the boot state: `Vision::begin()`
     * clears the latched fault, so a `Fault::NONE` never preceded by a completed read would
     * otherwise pass an unplugged camera.
     *
     * @param in This tick's input.
     * @return `true` if the sample may be judged.
     */
    bool _selfTestDataFresh(const Console::ConsoleInput& in) const;

    /**
     * @brief Emit at most one request per frame into the tick's action slot.
     *
     * @param action The request to emit.
     * @return `true` if the slot was free and @p action was taken; `false` if a request was
     *         already emitted this frame (the caller must retry on a later tick).
     */
    bool _requestAction(Console::ActionRequest action);

    /**
     * @brief Whether @p state is allowed to have @p read outstanding — the read's owning view.
     *
     * `READ_LINE_BOARDS` is owned by `State::VIEW_LINE` and by `State::SELF_TEST`
     * (`Check::LINE`); `READ_MOSAIC` by `State::VIEW_VISION` and `State::SELF_TEST`
     * (`Check::VISION`). Anything else — including `ActionRequest::NONE`, which is the free
     * slot and needs no owner — answers `false`.
     *
     * @param state The state to test.
     * @param read  The read currently in flight.
     * @return `true` if @p state may keep @p read outstanding.
     */
    static bool _stateOwnsRead(Console::State state, Console::ActionRequest read);

    /**
     * @brief Issue the view pull cadence: `READ_LINE_BOARDS` in `VIEW_LINE`, `READ_MOSAIC` in
     *        `VIEW_VISION`.
     *
     * Throttled to `Config::UI_VIEW_SAMPLE_MS` and `Config::UI_VISION_SAMPLE_MS` respectively
     * with rollover-safe arithmetic, gated on the owning module's boot flag, and suppressed
     * entirely while any read is still in flight.
     *
     * @param in This tick's input (supplies `nowMs` and the boot report).
     */
    void _servicePullCadence(const Console::ConsoleInput& in);

    // -----------------------------------------------------------------------------------------
    // Rendering helpers — every one is bounded and always NUL-terminates
    // -----------------------------------------------------------------------------------------

    /** @brief Blank all four rows and clear the cursor row of @p out. */
    static void _clearFrame(Console::ConsoleFrame& out);

    /** @brief Render `State::LIST`: the title row plus the three-row scroll window. */
    void _renderList(const Console::ConsoleInput& in, Console::ConsoleFrame& out) const;

    /** @brief Render `State::VIEW_LINE`: per-board raw channels plus derived line telemetry. */
    void _renderLineView(const Console::ConsoleInput& in, Console::ConsoleFrame& out) const;

    /** @brief Render `State::VIEW_VISION`: the twelve mosaic cells plus the fault kind. */
    void _renderVisionView(const Console::ConsoleInput& in, Console::ConsoleFrame& out) const;

    /** @brief Render `State::VIEW_GYRO`: yaw, health and wrap-correct drift since entry. */
    void _renderGyroView(const Console::ConsoleInput& in, Console::ConsoleFrame& out) const;

    /** @brief Render `State::VIEW_BATT`: pack voltage, percentage and the boot failure list. */
    void _renderBattView(const Console::ConsoleInput& in, Console::ConsoleFrame& out) const;

    /** @brief Render `State::WAIT_RELEASE`: which of the two waits is holding the console. */
    void _renderWaitRelease(Console::ConsoleFrame& out) const;

    /** @brief Render `State::SELF_TEST`: active check, its verdict or `RUNNING...`, abort hint. */
    void _renderSelfTest(Console::ConsoleFrame& out) const;

    /**
     * @brief Append the failed-module names from @p boot across rows 2 and 3 of @p out.
     * @param boot The boot report to summarise.
     * @param out  The frame whose rows 2-3 receive the list.
     */
    static void _renderBootFailures(const Console::BootReport& boot, Console::ConsoleFrame& out);

    /**
     * @brief Wrap-correct signed difference @p yawDeg - @p baselineDeg, folded to (-180, +180].
     *
     * Baseline 179 deg with a -179 deg sample yields +2 deg, never -358 deg.
     *
     * @param yawDeg      The current yaw reading in degrees.
     * @param baselineDeg The baseline latched on view entry, in degrees.
     * @return The signed drift in degrees.
     */
    static double _wrapDeltaDeg(double yawDeg, double baselineDeg);

    /** @brief Length of a NUL-terminated string, without pulling in `<string.h>`. */
    static uint8_t _strLen(const char* text);

    /**
     * @brief Append @p text at @p pos, truncating at `Console::TEXT_COLS`.
     * @param row  Destination row buffer (`Console::FRAME_COLS` wide).
     * @param pos  Current write position.
     * @param text Text to append.
     * @return The new write position; the row is always NUL-terminated there.
     */
    static uint8_t _rowAppend(char* row, uint8_t pos, const char* text);

    /** @brief Append a single character, truncating at `Console::TEXT_COLS`. */
    static uint8_t _rowAppendChar(char* row, uint8_t pos, char c);

    /** @brief Append a signed integer in decimal, truncating at `Console::TEXT_COLS`. */
    static uint8_t _rowAppendInt(char* row, uint8_t pos, int32_t value);

    /**
     * @brief Append a fixed-point value with exactly one decimal place.
     * @param row   Destination row buffer.
     * @param pos   Current write position.
     * @param value Value to append; magnitudes above 99999 are clamped.
     * @return The new write position; the row is always NUL-terminated there.
     */
    static uint8_t _rowAppendFixed1(char* row, uint8_t pos, double value);

    /** @brief Overwrite a whole row with @p text, truncated and NUL-terminated. */
    static void _rowSet(char* row, const char* text);

    // -----------------------------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------------------------

    Console::State _state       = Console::State::CLOSED;  ///< Active FSM state.
    uint8_t        _cursor      = 0;      ///< Highlighted `Console::Item` index.
    uint8_t        _windowTop   = 0;      ///< First item index in the visible scroll window.
    bool           _openLatch   = false;  ///< One-shot `open()` request awaiting consumption.
    bool           _btnUpPrev   = false;  ///< BTN_UP level last tick (edge detection).
    bool           _btnDownPrev = false;  ///< BTN_DOWN level last tick (edge detection).

    Console::State _waitNextState     = Console::State::LIST;  ///< Where `WAIT_RELEASE` leads.
    bool           _waitPendingAction = false;   ///< `WAIT_RELEASE` is awaiting a dispatched
                                                 ///< action's completion, not a button release.
    bool           _waitIsResume      = false;   ///< `_waitNextState` is the state the wait was
                                                 ///< entered FROM, so releasing it resumes that
                                                 ///< view (`_resumeState`) instead of re-entering
                                                 ///< it (`_enterState`). Set in `_beginWait`.

    /// Side-effect action awaiting `notifyActionDone()`; `NONE` when none is in flight.
    Console::ActionRequest _actionInFlight = Console::ActionRequest::NONE;
    /// A matching `notifyActionDone()` arrived and the await-completion wait may release.
    bool _actionDoneLatch = false;

    /// Read awaiting `notifyReadDone()`; `NONE` means the single read slot is free. Cleared by
    /// the matching `notifyReadDone()`, by `_close()`, and by `_enterState()` when the console
    /// transitions away from the read's owning view (a dropped acknowledgement must not poison
    /// the slot for the rest of the session).
    Console::ActionRequest _readInFlight = Console::ActionRequest::NONE;

    /// A `CLEAR_VISION_FAULT` is owed because the console left `VIEW_VISION`.
    bool _clearVisionPending = false;

    uint32_t _lastLineReqMs   = 0;      ///< `nowMs` of the last `READ_LINE_BOARDS`.
    bool     _lineReqIssued   = false;  ///< `_lastLineReqMs` is meaningful.
    uint32_t _lastVisionReqMs = 0;      ///< `nowMs` of the last `READ_MOSAIC`.
    bool     _visionReqIssued = false;  ///< `_lastVisionReqMs` is meaningful.

    double _gyroBaselineDeg = 0.0;    ///< Yaw latched on `VIEW_GYRO` entry.
    bool   _gyroBaselineSet = false;  ///< Baseline captured from a good sample.

    bool            _dirty  = false;                 ///< Level-triggered repaint request.
    Console::Signal _signal = Console::Signal::NONE;  ///< Level-triggered verdict signal.

    /// This tick's single action slot — at most one request is emitted per frame.
    Console::ActionRequest _frameAction = Console::ActionRequest::NONE;

    /// `boot.lineArray` as seen by the most recent `update()`; gates `wantsLineData()`.
    bool _bootLineArrayOk = false;

    /// Rows emitted this tick. Default-initialised to all-NUL so a STACK-allocated instance
    /// (the host tests construct one) never compares `_shownRows` against indeterminate bytes
    /// before `begin()` runs; the `extern` singleton is zero-initialised anyway.
    char _lastRows[Console::FRAME_ROWS][Console::FRAME_COLS] = {};
    /// Rows last acknowledged by `notifyFrameRendered()`. Default-initialised for the same
    /// reason as `_lastRows`.
    char _shownRows[Console::FRAME_ROWS][Console::FRAME_COLS] = {};

    // --- SELF TEST sequencer ---------------------------------------------------------------

    /// Current SELF TEST check. Homed to `Check::BATT` on entry by `_enterState()`; advanced by
    /// `_selfTestAdvance()`, which leaves it ON a failing check so the screen can name it.
    Console::Check _check = Console::Check::BATT;

    /// Raised by `_selfTestBeginCheck()` on the tick it emits `READ_LINE_BOARDS`, and lowered by
    /// `_selfTestRecord()` when the line verdict lands; read by `wantsLineData()`.
    bool _selfTestLineDataWanted = false;

    /**
     * @brief Per-check working memory of the SELF TEST sequencer.
     *
     * The sequencer's three phases are memory-bound and cannot be re-derived from `_check`
     * alone: **latch** needs the entry `seq` and `nowMs` baselines, **await** needs to know a
     * read was dispatched and is still outstanding, and the post-verdict **dwell** needs the
     * verdict's own timestamp (which is not the entry timestamp — the dwell runs AFTER the
     * verdict, never in place of it).
     *
     * Re-initialised at every point that homes `_check` to `Console::Check::BATT` — `begin()`,
     * `_close()` and `_enterState(State::SELF_TEST)` — so a re-entered SELF TEST never inherits
     * the previous run's freshness or dwell baselines.
     */
    struct SelfTestState
    {
        uint32_t entrySeq  = 0;      ///< `DiagSample::seq` latched when the active check began.
        uint32_t entryMs   = 0;      ///< `nowMs` at check entry — base of the fail-closed
                                     ///< `Config::UI_SELFTEST_CHECK_TIMEOUT_MS` deadline.
        uint32_t verdictMs = 0;      ///< `nowMs` the verdict landed — base of the
                                     ///< `Config::UI_SELFTEST_STEP_MS` readability dwell.
        bool     latched   = false;  ///< The latch phase has run for the active check.
        bool     readSent  = false;  ///< The active check dispatched a read, so freshness also
                                     ///< requires its `notifyReadDone()`.
        bool     judged    = false;  ///< A verdict is recorded and the dwell is running.
        bool     passed    = false;  ///< The recorded verdict for the active check.
        bool     timedOut  = false;  ///< The recorded failure was the freshness timeout.
    };

    /// SELF TEST sequencer working state; see `SelfTestState`.
    SelfTestState _selfTest = {};
};

/**
 * @brief Single global instance — defined in OperatorConsole.cpp.
 */
extern OperatorConsoleClass OperatorConsole;

#endif    // OPERATORCONSOLE_H
