#ifndef CONSOLETYPES_H
#define CONSOLETYPES_H

#include <stdint.h>

/**
 * @file ConsoleTypes.h
 * @brief Shared plain-old-data (POD) types and enums for the operator diagnostics
 *        console (`OperatorConsoleClass` / `OperatorPanelClass`).
 *
 * `<stdint.h>` only. This header MUST NOT include `<Arduino.h>`, `<MatrixMiniR4.h>`,
 * `Config.h`, or any other `config/`/`src/` header. It is the seam that lets the
 * headless console core (`OperatorConsole.cpp`) compile and link in the host ctest
 * harness with zero Arduino surface — a single stray include here would destroy that.
 *
 * @note Frame geometry (`FRAME_ROWS`, `TEXT_COLS`, `FRAME_COLS`, `LIST_VISIBLE_ROWS`)
 *       is a structural property of `ConsoleFrame` and lives solely here, never as a
 *       `Config::UI_*` tunable, because `config/` must never include a `src/` header
 *       and `Config.h` is transitively included by every module.
 */
namespace Console
{
    /**
     * @brief The nine flat, scrollable `DIAGNOSTICS` menu entries, in display order.
     */
    enum class Item : uint8_t
    {
        SELF_TEST   = 0,  ///< Runs the BATT -> GYRO -> VISION -> LINE self-test sequence.
        BATT        = 1,  ///< Shows pack voltage, percentage and the boot health summary.
        LINE_VIEW   = 2,  ///< Live per-board raw channel view.
        VISION_VIEW = 3,  ///< Live 12-cell mosaic colour view.
        GYRO_VIEW   = 4,  ///< Live yaw, health, and drift-since-entry view.
        GYRO_ZERO   = 5,  ///< Dispatches YawSource.zeroHardware().
        CAL_LINE    = 6,  ///< Dispatches WROLineArray::calibrate() (blocking).
        CAL_MOTOR   = 7,  ///< Dispatches Drive::calibrateMotorBias() (blocking).
        EXIT        = 8,  ///< Closes the console back to State::CLOSED.
    };

    /** @brief Number of entries in `Item` — the item table's fixed length. */
    constexpr uint8_t ITEM_COUNT = 9;

    /**
     * @brief Console FSM states.
     */
    enum class State : uint8_t
    {
        CLOSED       = 0,  ///< Not open; `OperatorPanelClass::update()` early-returns.
        LIST         = 1,  ///< Scrolling item list, cursor visible.
        VIEW_LINE    = 2,  ///< `Item::LINE_VIEW` live screen.
        VIEW_VISION  = 3,  ///< `Item::VISION_VIEW` live screen.
        VIEW_GYRO    = 4,  ///< `Item::GYRO_VIEW` live screen.
        VIEW_BATT    = 5,  ///< `Item::BATT` screen.
        SELF_TEST    = 6,  ///< `Item::SELF_TEST` sequencer running.
        WAIT_RELEASE = 7,  ///< Awaiting button release, or a dispatched action's completion.
    };

    /**
     * @brief Side-effect actions (reported via `notifyActionDone`) and reads
     *        (reported via `notifyReadDone`) the core can request of the adapter.
     */
    enum class ActionRequest : uint8_t
    {
        NONE               = 0,  ///< No request pending.
        CAL_LINE           = 1,  ///< Side effect: `WROLineArray::calibrate()` (blocking).
        CAL_MOTOR          = 2,  ///< Side effect: `Drive::calibrateMotorBias()` (blocking).
        GYRO_ZERO          = 3,  ///< Side effect: `YawSource.zeroHardware()`.
        READ_MOSAIC        = 4,  ///< Read: `Vision.readMosaic()`; PULLED fields via
                                  ///< `notifyReadDone`.
        READ_LINE_BOARDS   = 5,  ///< Read: `probeBoard()`+`readBoard()` per board; PULLED via
                                  ///< `notifyReadDone`.
        CLEAR_VISION_FAULT = 6,  ///< Side effect: clears `Vision`'s latched fault.
    };

    /**
     * @brief Level-triggered verdict signal for LEDs + buzzer, held until
     *        `notifySignalDone()` acknowledges it.
     */
    enum class Signal : uint8_t
    {
        NONE = 0,  ///< No verdict pending.
        PASS = 1,  ///< All-clear verdict (e.g. SELF TEST all-pass).
        FAIL = 2,  ///< Failure verdict.
    };

    /**
     * @brief SELF TEST subsystem sequence, in fixed run order. `DONE` is the
     *        terminal state once the last check (or the halting check) has settled.
     */
    enum class Check : uint8_t
    {
        BATT   = 0,  ///< Battery voltage check (runs first).
        GYRO   = 1,  ///< Gyro read + health check.
        VISION = 2,  ///< Vision link-health check (dispatches `READ_MOSAIC`).
        LINE   = 3,  ///< Line-board presence check (dispatches `READ_LINE_BOARDS`).
        DONE   = 4,  ///< Sequence finished — all checks passed, or halted on a failure.
    };

    /**
     * @brief The boot-time health verdicts captured once in `setup()`, before they
     *        would otherwise die as locals.
     *
     * 12 module `begin()` results plus the battery voltage verdict. `clean()` is the
     * AND of all 13 flags and is what `isDispatchable()` and the SELF TEST sequencer's
     * per-subsystem degraded-mode skip consult to decide what may be sampled,
     * dispatched, or displayed while `_setupFailed` is latched.
     */
    struct BootReport
    {
        bool batt;  ///< Voltage verdict at boot, NOT an init verdict (`MiniR4.PWR` comes up
                    ///< inside `MiniR4.begin()`); a low pack alone never disables the BATT
                    ///< screen.
        bool lineArray;    ///< `WROLineArray::begin()` result.
        bool yaw;          ///< `YawSource::begin()` result.
        bool drive;        ///< `Drive::begin()` result.
        bool lineFollower; ///< `LineFollower::begin()` result.
        bool intersection; ///< `Intersection::begin()` result.
        bool manipulator;  ///< `Manipulator::begin()` result.
        bool localization; ///< `Localization::begin()` result.
        bool pathTracker;  ///< `PathTracker::begin()` result.
        bool mission;      ///< `Mission::begin()` result.
        bool tasks;        ///< `Tasks::begin()` result.
        bool vision;       ///< `Vision::begin()` result.
        bool trace;        ///< `EventTrace::begin()` result.

        float battVoltageAtBoot;  ///< Pack voltage sampled at boot, in volts.

        /**
         * @brief Whether every one of the 13 boot health flags passed.
         * @return The logical AND of `batt`, `lineArray`, `yaw`, `drive`,
         *         `lineFollower`, `intersection`, `manipulator`, `localization`,
         *         `pathTracker`, `mission`, `tasks`, `vision` and `trace`.
         */
        bool clean() const
        {
            return batt && lineArray && yaw && drive && lineFollower && intersection &&
                   manipulator && localization && pathTracker && mission && tasks && vision &&
                   trace;
        }
    };

    /**
     * @brief Hardware state pushed to the core every tick.
     *
     * Every field belongs to exactly one of two provenance groups, each with a
     * single initiator and a single authoritative channel:
     *
     * - **PUSHED** fields are refreshed autonomously by the adapter every
     *   `Config::UI_VIEW_SAMPLE_MS`, independent of any console request.
     * - **PULLED** fields are written ONLY as the completion of the matching
     *   `ActionRequest` (`READ_LINE_BOARDS` or `READ_MOSAIC`), immediately before the
     *   adapter calls `notifyReadDone()`. Between two such completions a PULLED
     *   field is otherwise carried forward byte-identical — the adapter must NEVER
     *   refresh a PULLED field on its own, no matter how cheap the underlying read.
     *
     * `seq` is bumped by the adapter on every refresh of either group (a PUSHED
     * cadence tick or a PULLED request completion); it is how the core tells a
     * fresh sample from a stale one (the SELF TEST sequencer's fail-closed
     * freshness gate judges only a sample whose `seq` advanced after the check
     * began).
     */
    struct DiagSample
    {
        uint32_t seq;  ///< Monotonic; bumped on every hardware refresh (PUSHED or PULLED).

        // --- PUSHED: refreshed autonomously every Config::UI_VIEW_SAMPLE_MS ---
        float   battVolts;         ///< Pack voltage in volts; 0.0f => read failed.
        uint8_t battPercent;       ///< Pack charge percentage, derived from `battVolts`.
        bool    battReadOk;        ///< false while the read has failed and no retry is pending.
        bool    battRetryPending;  ///< A non-blocking retry is in flight; batt verdict must wait.
        double  gyroYawDeg;        ///< Current external-gyro yaw, in degrees.
        bool    gyroHealthy;       ///< `YawSource::isHealthy()` at last refresh.
        bool    gyroReadOk;        ///< false if the yaw read itself failed.
        uint8_t junctionType;      ///< Cached `Intersection::JunctionType`; gated on
                                   ///< `wantsLineData()`.
        bool    lineOnline;        ///< Cached `WROLineArray::isOnline()`; gated on
                                   ///< `wantsLineData()`.
        float   lineError;         ///< Cached `LineFollower` centroid error; gated on
                                   ///< `wantsLineData()`.

        // --- PULLED by ActionRequest::READ_LINE_BOARDS ---
        uint8_t lineRaw[2][8];        ///< Raw per-channel reading (0..9) per board.
        bool    lineBoardPresent[2];  ///< `probeBoard()` I2C address-ACK result — THE liveness
                                       ///< verdict for the corresponding board.
        bool    lineBoardOk[2];       ///< `readBoard()` return — raw-row validity for DISPLAY
                                       ///< ONLY. A `true` value does NOT prove the board
                                       ///< answered; only `lineBoardPresent` is authoritative
                                       ///< for liveness.
        uint8_t lineBoardCount;       ///< Number of configured boards
                                       ///< (`Config::LINE_BOARD_COUNT`).

        // --- PULLED by ActionRequest::READ_MOSAIC ---
        uint8_t visionCell[12];   ///< `static_cast<uint8_t>(Config::Color)` per mosaic cell.
        uint8_t visionFaultKind;  ///< `static_cast<uint8_t>(VisionClass::Fault)` from the read.
    };

    /**
     * @brief Per-tick input to `OperatorConsoleClass::update()`.
     *
     * @note Carries no open-request field. Opening the console is an out-of-band
     *       latched call on the core (`OperatorConsoleClass::open()`) — the same
     *       shape as `notifyActionDone()` / `notifyReadDone()` — never a field read
     *       here.
     */
    struct ConsoleInput
    {
        bool       btnUp;         ///< Current BTN_UP level (already debounced by the adapter).
        bool       btnDown;       ///< Current BTN_DOWN level (already debounced by the adapter).
        uint32_t   nowMs;         ///< The console's ONLY time source; comparisons must be
                                   ///< rollover-safe.
        bool       missionArmed;  ///< true force-closes the console to `State::CLOSED`.
        DiagSample sample;        ///< Latest pushed/pulled hardware sample.
        BootReport boot;          ///< Boot-time health verdicts (degraded-mode availability).
    };

    /** @brief OLED text rows per frame (128x32 @ textSize 1: title row + 3 body rows). */
    constexpr uint8_t FRAME_ROWS = 4;
    /** @brief Printable characters per row, excluding the terminating NUL. */
    constexpr uint8_t TEXT_COLS = 21;
    /** @brief Row buffer width: `TEXT_COLS` printable chars plus one terminating NUL. */
    constexpr uint8_t FRAME_COLS = TEXT_COLS + 1;
    /** @brief Scrollable list rows remaining after the fixed title row. */
    constexpr uint8_t LIST_VISIBLE_ROWS = FRAME_ROWS - 1;

    static_assert(FRAME_COLS == TEXT_COLS + 1, "row buffer must hold TEXT_COLS chars + NUL");
    static_assert(ITEM_COUNT >= LIST_VISIBLE_ROWS, "scroll window cannot exceed the item table");

    /**
     * @brief One rendered console frame: `FRAME_ROWS` fixed-width NUL-terminated
     *        text rows plus the cursor, repaint, action-dispatch and
     *        verdict-signal state.
     */
    struct ConsoleFrame
    {
        char rows[FRAME_ROWS][FRAME_COLS];  ///< Bounded, always NUL-terminated text rows.
        uint8_t       cursorRow;  ///< Row index of the cursor within `rows`; 0xFF = no cursor.
        bool          dirty;      ///< Level-triggered repaint request until
                                   ///< `notifyFrameRendered()`.
        ActionRequest action;     ///< Requested action for this frame; `NONE` if nothing to
                                   ///< dispatch.
        Signal        signal;     ///< Level-triggered verdict signal until `notifySignalDone()`.
    };

    /**
     * @brief AC24 availability predicate: whether `item` may be confirmed and
     *        dispatched right now, given the current boot health.
     *
     * Pure; host-tested. Declaration only — the definition belongs to a different
     * task, in `OperatorConsole.cpp`.
     *
     * @param item The candidate menu entry.
     * @param boot The current boot health report.
     * @return `true` iff `item` is currently dispatchable.
     */
    bool isDispatchable(Item item, const BootReport& boot);
}    // namespace Console

#endif    // CONSOLETYPES_H
