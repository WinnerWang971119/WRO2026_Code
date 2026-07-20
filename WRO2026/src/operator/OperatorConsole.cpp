/**
 * @file OperatorConsole.cpp
 * @brief Implementation of the headless operator-diagnostics console core.
 *
 * @details
 * See `OperatorConsole.h` for the design contract. Two rules govern every line of this file:
 *
 *  1. **Purity.** The only includes are `ConsoleTypes.h` and the two `<stdint.h>`-only config
 *     sub-headers. NOT `Config.h` — the umbrella pulls `<Arduino.h>`, which would drag the whole
 *     Arduino/MatrixMiniR4 surface (and the `WHITE` macro trap) into a translation unit that must
 *     link into the host ctest harness with zero mocks. `Vision.h` sets the same
 *     direct-sub-header precedent. There is no `millis()`, no I/O and no mutable state outside
 *     the singleton instance anywhere below.
 *  2. **Rollover safety.** Every elapsed-time test is `(uint32_t)(now - then) >= interval`, never
 *     `now >= then + interval`, because the caller's clock wraps at ~49.7 days.
 */

#include "OperatorConsole.h"

#include "../../config/ConfigGlobal.h"
#include "../../config/ConfigUi.h"

namespace
{
    /**
     * @brief The nine menu labels, indexed by `Console::Item`, in display order.
     *
     * Immutable const data, not state — the purity rule forbids mutable statics, not a read-only
     * literal table.
     */
    const char* const kItemLabels[Console::ITEM_COUNT] = {
        "SELF TEST",    // Item::SELF_TEST
        "BATT",         // Item::BATT
        "LINE VIEW",    // Item::LINE_VIEW
        "VISION VIEW",  // Item::VISION_VIEW
        "VISION TEST",  // Item::VISION_TEST
        "GYRO VIEW",    // Item::GYRO_VIEW
        "GYRO ZERO",    // Item::GYRO_ZERO
        "CAL LINE",     // Item::CAL_LINE
        "CAL MOTOR",    // Item::CAL_MOTOR
        "EXIT",         // Item::EXIT
    };

    /** @brief Title row shown by the list and by `WAIT_RELEASE`. */
    const char* const kTitle = "DIAGNOSTICS";

    /** @brief Row prefix for the highlighted, dispatchable entry. */
    const char* const kPrefixCursor = "> ";
    /** @brief Row prefix for an ordinary, dispatchable entry. */
    const char* const kPrefixPlain = "  ";
    /** @brief Row prefix for a non-dispatchable entry (AC24). */
    const char* const kPrefixBlocked = "! ";

    /**
     * @brief Mosaic colour letters indexed by `static_cast<uint8_t>(Config::Color)`.
     *
     * Order is pinned to the enum: UNKNOWN, YELLOW, BLUE, GREEN, WHITE, RED.
     */
    const char* const kColorLetters = "?YBGWR";

    /** @brief Number of colour letters in `kColorLetters`. */
    const uint8_t kColorLetterCount = 6;

    /** @brief Mosaic cells carried by `Console::DiagSample::visionCell`. */
    const uint8_t kVisionCellCount = 12;

    /**
     * @brief The operator's EXPECTED mosaic colour code — the card placed on the mat for VISION
     *        TEST. **EDIT THIS to match your card.**
     *
     * One `static_cast<uint8_t>(Config::Color)` per cell, row-major over the 4x3 mosaic (cell =
     * row*4 + col), same indexing as `Console::DiagSample::visionCell` and `kColorLetters`
     * (0=?/UNKNOWN, 1=Y, 2=B, 3=G, 4=W, 5=R). OperatorPanel.cpp static_asserts these codes against
     * `Config::Color`, so a code drift is a compile error, not a silent mis-grade.
     */
    const uint8_t kExpectedMosaic[kVisionCellCount] = {
        4, 3, 1, 2,    // W G Y B
        3, 4, 2, 2,    // G W B B
        1, 1, 4, 3,    // Y Y W G
    };

    /** @brief Line-array channels per board carried by `Console::DiagSample::lineRaw`. */
    const uint8_t kLineChannelCount = 8;

    /** @brief Maximum line boards carried by `Console::DiagSample`. */
    const uint8_t kLineBoardMax = 2;

    /** @brief Boot-report entries summarised by the BATT screen. */
    const uint8_t kBootFlagCount = 13;

    /**
     * @brief `VisionClass::Fault::NONE` as carried by `DiagSample::visionFaultKind`.
     *
     * Mirrored as a plain code rather than the enum itself: `Vision.h` is an Arduino-facing
     * header and including it here would destroy this translation unit's host-linkability. The
     * four codes below are therefore a COPY of the `VisionClass::Fault` contract, which is
     * precisely why `judgeVision()` refuses to guess at a value it does not find here.
     */
    const uint8_t kVisionFaultNone = 0;

    /** @brief `VisionClass::Fault::READ_TIMEOUT` as carried by `DiagSample::visionFaultKind`. */
    const uint8_t kVisionFaultReadTimeout = 1;

    /** @brief `VisionClass::Fault::BAD_CHECKSUM` as carried by `DiagSample::visionFaultKind`. */
    const uint8_t kVisionFaultBadChecksum = 2;

    /** @brief `VisionClass::Fault::BAD_CELL` as carried by `DiagSample::visionFaultKind`. */
    const uint8_t kVisionFaultBadCell = 3;

    /**
     * @brief Outcome of judging one SELF TEST check against the current sample.
     *
     * `PENDING` is NOT "no data yet" — freshness is decided before a judge is even attempted.
     * It means the check saw fresh data that is legitimately inconclusive (a battery read whose
     * non-blocking retry is still in flight) and must keep waiting, bounded by the check's
     * `Config::UI_SELFTEST_CHECK_TIMEOUT_MS` deadline like any other wait.
     */
    enum class CheckVerdict : uint8_t
    {
        PENDING = 0,  ///< Fresh but inconclusive; keep waiting until the deadline.
        PASS    = 1,  ///< The subsystem is fit to run.
        FAIL    = 2,  ///< The subsystem failed; the sequence halts here.
    };

    /**
     * @brief Whether @p check may be run against live hardware at all (AC25).
     *
     * Consulted BEFORE the sample: a module whose `begin()` failed is reported straight from the
     * boot report, with no `ActionRequest` ever issued against it.
     *
     * @param check The check about to be entered.
     * @param boot  The boot health report.
     * @return `false` if the owning module did not come up, so @p check FAILs without reading.
     * @note `Check::BATT` answers `true` even when `boot.batt` is `false`. That flag is a
     *       VOLTAGE verdict sampled once at boot, not an init verdict — `MiniR4.PWR` comes up
     *       inside `MiniR4.begin()` — and a pack that was low at boot may have been swapped or
     *       recovered since, so the battery check always takes the live reading.
     */
    bool selfTestBootAllows(Console::Check check, const Console::BootReport& boot)
    {
        switch (check)
        {
        case Console::Check::BATT:
            return true;
        case Console::Check::GYRO:
            return boot.yaw;
        case Console::Check::VISION:
            return boot.vision;
        case Console::Check::LINE:
            return boot.lineArray;
        case Console::Check::DONE:
            return true;
        }
        return true;
    }

    /**
     * @brief The read @p check must dispatch on entry, if it has one.
     * @param check The check being entered.
     * @return `READ_MOSAIC` for `Check::VISION`, `READ_LINE_BOARDS` for `Check::LINE`,
     *         `ActionRequest::NONE` for the checks served by the PUSHED sample.
     */
    Console::ActionRequest selfTestRead(Console::Check check)
    {
        switch (check)
        {
        case Console::Check::VISION:
            return Console::ActionRequest::READ_MOSAIC;
        case Console::Check::LINE:
            return Console::ActionRequest::READ_LINE_BOARDS;
        default:
            return Console::ActionRequest::NONE;
        }
    }

    /**
     * @brief The check that follows @p check in the fixed BATT -> GYRO -> VISION -> LINE order.
     * @param check The check that just passed.
     * @return The next check, or `Check::DONE` once `Check::LINE` has passed.
     */
    Console::Check selfTestNext(Console::Check check)
    {
        switch (check)
        {
        case Console::Check::BATT:
            return Console::Check::GYRO;
        case Console::Check::GYRO:
            return Console::Check::VISION;
        case Console::Check::VISION:
            return Console::Check::LINE;
        case Console::Check::LINE:
        case Console::Check::DONE:
            return Console::Check::DONE;
        }
        return Console::Check::DONE;
    }

    /**
     * @brief Judge the battery check against a fresh sample.
     * @param sample The fresh sample.
     * @return `PENDING` while a non-blocking retry is in flight, `FAIL` on a flat or unreadable
     *         pack, else `PASS`.
     * @note `0.0f` is the documented read-failed sentinel for `battVolts`, so it is tested
     *       explicitly even though it also trips the threshold — the verdict must read as
     *       "unreadable", not merely "low".
     */
    CheckVerdict judgeBattery(const Console::DiagSample& sample)
    {
        if (sample.battRetryPending)
        {
            return CheckVerdict::PENDING;
        }
        if (sample.battVolts == 0.0f || sample.battVolts < Config::BATT_MIN_VOLTAGE_V)
        {
            return CheckVerdict::FAIL;
        }
        return CheckVerdict::PASS;
    }

    /**
     * @brief Judge the gyro check against a fresh sample.
     * @param sample The fresh sample.
     * @return `FAIL` if the yaw read failed or the sensor reports itself unhealthy, else `PASS`.
     */
    CheckVerdict judgeGyro(const Console::DiagSample& sample)
    {
        if (!sample.gyroReadOk || !sample.gyroHealthy)
        {
            return CheckVerdict::FAIL;
        }
        return CheckVerdict::PASS;
    }

    /**
     * @brief Judge the vision check against a sample refreshed by a COMPLETED `READ_MOSAIC`.
     *
     * Link health only (AC14): `READ_TIMEOUT` / `BAD_CHECKSUM` mean the camera did not answer
     * intelligibly and FAIL. `BAD_CELL` PASSes — it fires when a cell classifies `UNKNOWN`,
     * which is the expected pre-run state with no game elements placed; judging cell CONTENT
     * would repeat the mistake already vetoed for the line channels.
     *
     * @param sample The fresh sample.
     * @return `PASS` only for `NONE` and `BAD_CELL`; `FAIL` for every other code.
     * @note The caller is responsible for only ever handing this a post-read sample. `NONE` is a
     *       PASS solely because a completed read produced it; `Vision::begin()` clears the fault,
     *       so the BOOT-state `NONE` would pass an unplugged camera.
     */
    CheckVerdict judgeVision(const Console::DiagSample& sample)
    {
        // ALLOW-LIST, and deliberately so. Only the two codes proven compatible with a working
        // camera link PASS; everything else FAILs, INCLUDING codes that do not exist today. The
        // fault codes above are a hand-mirrored copy of `VisionClass::Fault`, so an unrecognised
        // value means that contract has changed underneath the console — and the safe reading of
        // "I do not recognise this" is "not fit to run", never "fine". Written as a deny-list it
        // would silently PASS every future fault kind, in an advisory that gates a competition
        // run; every other judge in this sequencer fails closed, and so does this one.
        switch (sample.visionFaultKind)
        {
        case kVisionFaultNone:
        case kVisionFaultBadCell:
            return CheckVerdict::PASS;

        case kVisionFaultReadTimeout:
        case kVisionFaultBadChecksum:
            return CheckVerdict::FAIL;

        default:
            return CheckVerdict::FAIL;
        }
    }

    /**
     * @brief Judge the line check against a sample refreshed by a completed `READ_LINE_BOARDS`.
     *
     * PASSes iff every CONFIGURED board answered its `probeBoard()` I2C address ACK.
     *
     * @param sample The fresh sample.
     * @return `PASS` iff `lineBoardPresent[b]` holds for every `b < lineBoardCount`.
     * @note Deliberately inspects NEITHER `lineRaw` NOR `lineBoardOk`. `readBoard()` has no error
     *       return and accepts an all-zero dead-bus read at face value, so `lineBoardOk` proves
     *       nothing about liveness (it is display validity only), and channel values false-fail a
     *       perfectly live bar held off the mat. The address ACK is the sole liveness verdict.
     */
    CheckVerdict judgeLineBoards(const Console::DiagSample& sample)
    {
        uint8_t boards = sample.lineBoardCount;
        if (boards > kLineBoardMax)
        {
            boards = kLineBoardMax;
        }

        for (uint8_t b = 0; b < boards; ++b)
        {
            if (!sample.lineBoardPresent[b])
            {
                return CheckVerdict::FAIL;
            }
        }
        return CheckVerdict::PASS;
    }

    /**
     * @brief Dispatch @p check to its judge.
     * @param check  The active check.
     * @param sample The sample, already proven fresh by the caller.
     * @return The check's verdict; `PASS` for `Check::DONE`, which has nothing to judge.
     */
    CheckVerdict selfTestJudge(Console::Check check, const Console::DiagSample& sample)
    {
        switch (check)
        {
        case Console::Check::BATT:
            return judgeBattery(sample);
        case Console::Check::GYRO:
            return judgeGyro(sample);
        case Console::Check::VISION:
            return judgeVision(sample);
        case Console::Check::LINE:
            return judgeLineBoards(sample);
        case Console::Check::DONE:
            break;
        }
        return CheckVerdict::PASS;
    }

    /**
     * @brief Short display name for a `VisionClass::Fault` value.
     * @param kind `DiagSample::visionFaultKind`.
     * @return A stable short name; `"?"` for an unknown value.
     */
    const char* visionFaultName(uint8_t kind)
    {
        switch (kind)
        {
        case kVisionFaultNone:
            return "NONE";
        case kVisionFaultReadTimeout:
            return "TIMEOUT";
        case kVisionFaultBadChecksum:
            return "CHKSUM";
        case kVisionFaultBadCell:
            return "BADCELL";
        default:
            return "?";
        }
    }

    /**
     * @brief Short display name for a `Console::Check` value.
     * @param check The SELF TEST check to name.
     * @return A stable short name.
     */
    const char* checkName(Console::Check check)
    {
        switch (check)
        {
        case Console::Check::BATT:
            return "BATT";
        case Console::Check::GYRO:
            return "GYRO";
        case Console::Check::VISION:
            return "VISION";
        case Console::Check::LINE:
            return "LINE";
        case Console::Check::DONE:
            return "DONE";
        }
        return "?";
    }

    /**
     * @brief Short display name for a dispatched side-effect action.
     * @param action The action in flight.
     * @return A stable short name; `"-"` for `NONE` / a read.
     */
    const char* actionName(Console::ActionRequest action)
    {
        switch (action)
        {
        case Console::ActionRequest::CAL_LINE:
            return "CAL LINE";
        case Console::ActionRequest::CAL_MOTOR:
            return "CAL MOTOR";
        case Console::ActionRequest::GYRO_ZERO:
            return "GYRO ZERO";
        case Console::ActionRequest::CLEAR_VISION_FAULT:
            return "CLR FAULT";
        default:
            return "-";
        }
    }
}    // namespace

// =============================================================================================
// Console::isDispatchable  (AC24 availability predicate — declared in ConsoleTypes.h)
// =============================================================================================

bool Console::isDispatchable(Console::Item item, const Console::BootReport& boot)
{
    switch (item)
    {
    // Always available: the operator needs these MOST in degraded mode. `boot.batt` is a
    // voltage verdict, not an init verdict, so a low pack never disables a read-only entry.
    case Console::Item::SELF_TEST:
    case Console::Item::BATT:
    case Console::Item::EXIT:
        return true;

    // Read-only views: available iff the module they read came up.
    case Console::Item::LINE_VIEW:
        return boot.lineArray;
    case Console::Item::VISION_VIEW:
    case Console::Item::VISION_TEST:
        return boot.vision;
    case Console::Item::GYRO_VIEW:
        return boot.yaw;

    // Hardware-writing entries: need their own module AND a clean boot report.
    case Console::Item::GYRO_ZERO:
        return boot.yaw && boot.clean();
    case Console::Item::CAL_LINE:
        return boot.lineArray && boot.clean();
    case Console::Item::CAL_MOTOR:
        return boot.drive && boot.clean();
    }

    return false;
}

// =============================================================================================
// Lifecycle
// =============================================================================================

void OperatorConsoleClass::begin()
{
    _state       = Console::State::CLOSED;
    _cursor      = 0;
    _windowTop   = 0;
    _openLatch   = false;
    _btnUpPrev   = false;
    _btnDownPrev = false;

    _waitNextState     = Console::State::LIST;
    _waitPendingAction = false;
    _waitIsResume      = false;

    _actionInFlight  = Console::ActionRequest::NONE;
    _actionDoneLatch = false;
    _readInFlight    = Console::ActionRequest::NONE;
    _frameAction     = Console::ActionRequest::NONE;

    _clearVisionPending = false;

    _lastLineReqMs   = 0;
    _lineReqIssued   = false;
    _lastVisionReqMs = 0;
    _visionReqIssued = false;

    _visionTestReqIssued = false;
    _visionTestJudged    = false;
    _visionTestPass      = false;

    _gyroBaselineDeg = 0.0;
    _gyroBaselineSet = false;

    _dirty  = false;
    _signal = Console::Signal::NONE;

    _bootLineArrayOk = false;

    _check                 = Console::Check::BATT;
    _selfTestLineDataWanted = false;
    _selfTest               = SelfTestState();

    for (uint8_t r = 0; r < Console::FRAME_ROWS; ++r)
    {
        for (uint8_t c = 0; c < Console::FRAME_COLS; ++c)
        {
            _lastRows[r][c]  = '\0';
            _shownRows[r][c] = '\0';
        }
    }
}

void OperatorConsoleClass::open()
{
    if (isOpen())
    {
        // Already open (or already latched): a held button or a repeated call must never
        // re-open the console or reset a live view that is on screen.
        return;
    }
    _openLatch = true;
}

// =============================================================================================
// Per-tick entry point
// =============================================================================================

void OperatorConsoleClass::update(const Console::ConsoleInput& in, Console::ConsoleFrame& out)
{
    _frameAction     = Console::ActionRequest::NONE;
    _bootLineArrayOk = in.boot.lineArray;

    // (1) Armed force-close outranks everything, from any state, and discards the open latch.
    if (in.missionArmed)
    {
        _close();
        _btnUpPrev   = in.btnUp;
        _btnDownPrev = in.btnDown;
        _publishFrame(in, out);
        return;
    }

    // (2) Consume the one-shot open latch. The opening tick takes no button action: the
    //     long-press that opened the console is still held, and reading it as a list edge
    //     would advance the cursor the instant the list appeared.
    const bool openedThisTick = _consumeOpenLatch(in);

    if (_state == Console::State::CLOSED)
    {
        _btnUpPrev   = in.btnUp;
        _btnDownPrev = in.btnDown;
        _publishFrame(in, out);
        return;
    }

    // (3) Edge detection: a level held across N ticks yields exactly one action.
    if (!openedThisTick)
    {
        const bool upEdge   = in.btnUp && !_btnUpPrev;
        const bool downEdge = in.btnDown && !_btnDownPrev;

        switch (_state)
        {
        case Console::State::LIST:
            if (in.btnUp && in.btnDown)
            {
                // Ambiguous chord — take no action until both read released for one tick.
                _beginWait(_state, false);
            }
            else
            {
                _handleListButtons(in, upEdge, downEdge);
            }
            break;

        case Console::State::VIEW_LINE:
        case Console::State::VIEW_VISION:
        case Console::State::VIEW_GYRO:
        case Console::State::VIEW_BATT:
            if (in.btnUp && in.btnDown)
            {
                _beginWait(_state, false);
            }
            else
            {
                _handleViewButtons(in, downEdge);
            }
            break;

        case Console::State::SELF_TEST:
            _handleViewButtons(in, downEdge);
            if (_state == Console::State::SELF_TEST)
            {
                _updateSelfTest(in);
            }
            break;

        case Console::State::VISION_TEST:
            if (in.btnUp && in.btnDown)
            {
                _beginWait(_state, false);
            }
            else
            {
                _handleViewButtons(in, downEdge);
            }
            // Grade once the one-shot read lands — only while still in the screen (a BTN press
            // above may have exited it, exactly like the SELF TEST abort path).
            if (_state == Console::State::VISION_TEST)
            {
                _updateVisionTest(in);
            }
            break;

        case Console::State::WAIT_RELEASE:
            _updateWaitRelease(in);
            break;

        case Console::State::CLOSED:
            break;
        }
    }

    // (4) Post-transition per-state servicing.
    if (_state == Console::State::VIEW_GYRO)
    {
        _updateGyroBaseline(in);
    }

    // An advisory diagnostic must never leave a latched vision fault behind for the mission,
    // so the clear is latched on leaving VIEW_VISION and drained as soon as the single
    // per-frame action slot is free.
    if (_clearVisionPending && _requestAction(Console::ActionRequest::CLEAR_VISION_FAULT))
    {
        _clearVisionPending = false;
    }

    _servicePullCadence(in);

    // (5) Render and latch the button levels for the next tick's edge detection.
    _btnUpPrev   = in.btnUp;
    _btnDownPrev = in.btnDown;
    _publishFrame(in, out);
}

// =============================================================================================
// Adapter callbacks
// =============================================================================================

void OperatorConsoleClass::notifyActionDone(Console::ActionRequest which, bool ok)
{
    switch (which)
    {
    case Console::ActionRequest::CAL_LINE:
    case Console::ActionRequest::CAL_MOTOR:
    case Console::ActionRequest::GYRO_ZERO:
    case Console::ActionRequest::CLEAR_VISION_FAULT:
        break;
    default:
        // Reads carry no verdict — they must be reported through notifyReadDone().
        return;
    }

    if (_actionInFlight != which)
    {
        return;
    }
    _actionInFlight = Console::ActionRequest::NONE;

    if (_waitPendingAction)
    {
        // Only an operator-confirmed action publishes a verdict signal; a housekeeping
        // CLEAR_VISION_FAULT is never dispatched through the pending-action path, so it can
        // never clobber the SELF TEST verdict.
        _actionDoneLatch = true;
        _signal          = ok ? Console::Signal::PASS : Console::Signal::FAIL;
    }
}

void OperatorConsoleClass::notifyReadDone(Console::ActionRequest which)
{
    if (which != Console::ActionRequest::READ_MOSAIC &&
        which != Console::ActionRequest::READ_LINE_BOARDS)
    {
        return;
    }
    if (_readInFlight == which)
    {
        _readInFlight = Console::ActionRequest::NONE;
    }
}

void OperatorConsoleClass::notifyFrameRendered()
{
    for (uint8_t r = 0; r < Console::FRAME_ROWS; ++r)
    {
        for (uint8_t c = 0; c < Console::FRAME_COLS; ++c)
        {
            _shownRows[r][c] = _lastRows[r][c];
        }
    }
    _dirty = false;
}

void OperatorConsoleClass::notifySignalDone()
{
    _signal = Console::Signal::NONE;
}

// =============================================================================================
// Queries
// =============================================================================================

bool OperatorConsoleClass::isOpen() const
{
    return _state != Console::State::CLOSED || _openLatch;
}

bool OperatorConsoleClass::wantsLineData() const
{
    if (!_bootLineArrayOk)
    {
        return false;
    }
    if (_state == Console::State::VIEW_LINE)
    {
        return true;
    }
    return _state == Console::State::SELF_TEST && _check == Console::Check::LINE &&
           _selfTestLineDataWanted;
}

Console::State OperatorConsoleClass::state() const
{
    return _state;
}

// =============================================================================================
// FSM helpers
// =============================================================================================

void OperatorConsoleClass::_close()
{
    _state             = Console::State::CLOSED;
    _openLatch         = false;
    _waitNextState     = Console::State::LIST;
    _waitPendingAction = false;
    _waitIsResume      = false;
    _actionInFlight    = Console::ActionRequest::NONE;
    _actionDoneLatch   = false;
    _readInFlight      = Console::ActionRequest::NONE;
    _clearVisionPending = false;
    _lineReqIssued     = false;
    _visionReqIssued   = false;
    _gyroBaselineSet   = false;
    _dirty             = false;
    _signal            = Console::Signal::NONE;

    _check                 = Console::Check::BATT;
    _selfTestLineDataWanted = false;
    _selfTest               = SelfTestState();

    for (uint8_t r = 0; r < Console::FRAME_ROWS; ++r)
    {
        for (uint8_t c = 0; c < Console::FRAME_COLS; ++c)
        {
            _lastRows[r][c]  = '\0';
            _shownRows[r][c] = '\0';
        }
    }
}

bool OperatorConsoleClass::_consumeOpenLatch(const Console::ConsoleInput& in)
{
    if (!_openLatch)
    {
        return false;
    }
    _openLatch = false;

    if (_state != Console::State::CLOSED)
    {
        return false;
    }

    _cursor = static_cast<uint8_t>(Console::Item::SELF_TEST);
    _scrollWindowTop();
    _enterState(Console::State::LIST);

    // Seed the edge history from THIS tick's levels so the still-held long-press cannot be
    // mistaken for a fresh list edge.
    _btnUpPrev   = in.btnUp;
    _btnDownPrev = in.btnDown;
    return true;
}

bool OperatorConsoleClass::_stateOwnsRead(Console::State state, Console::ActionRequest read)
{
    switch (read)
    {
    case Console::ActionRequest::READ_LINE_BOARDS:
        // VIEW_LINE's pull cadence, and the SELF TEST sequencer's Check::LINE.
        return state == Console::State::VIEW_LINE || state == Console::State::SELF_TEST;

    case Console::ActionRequest::READ_MOSAIC:
        // VIEW_VISION's pull cadence, the SELF TEST sequencer's Check::VISION, and the VISION TEST
        // one-shot read.
        return state == Console::State::VIEW_VISION || state == Console::State::SELF_TEST ||
               state == Console::State::VISION_TEST;

    default:
        // ActionRequest::NONE is the free slot and every side-effect action is tracked by
        // _actionInFlight instead; neither has a read owner.
        return false;
    }
}

void OperatorConsoleClass::_enterState(Console::State next)
{
    // A read is owned by the view that issued it. If that view is gone, so is anyone who could
    // still make sense of the answer, so the slot is released here rather than waiting on a
    // notifyReadDone() that a non-conforming adapter may never deliver. Without this, ONE
    // dropped acknowledgement would suppress every subsequent read — in every view, for the
    // whole console session — and the only recovery would be closing the console.
    //
    // Deliberately keyed on the DESTINATION, not the departing state: a state that cannot own
    // the read is by definition not the owning view. The one-in-flight invariant WITHIN a view
    // is untouched (no _enterState call happens there), and a chord *resume* never reaches this
    // code at all — it goes through _resumeState(), because the owning view never left the
    // screen.
    if (!_stateOwnsRead(next, _readInFlight))
    {
        _readInFlight = Console::ActionRequest::NONE;
    }

    _state = next;

    switch (next)
    {
    case Console::State::VIEW_GYRO:
        // Re-entering the view re-baselines, so drift always reads zero at the entry sample.
        _gyroBaselineSet = false;
        break;

    case Console::State::VIEW_LINE:
        // Disarm the throttle so the first tick in the view issues its read immediately.
        _lineReqIssued = false;
        break;

    case Console::State::VIEW_VISION:
        _visionReqIssued = false;
        break;

    case Console::State::VISION_TEST:
        // Fresh grade every entry: re-request the one-shot read and drop the prior verdict.
        _visionTestReqIssued = false;
        _visionTestJudged    = false;
        _visionTestPass      = false;
        break;

    case Console::State::SELF_TEST:
        _check                 = Console::Check::BATT;
        _selfTestLineDataWanted = false;
        // A re-entered SELF TEST must never inherit the previous run's freshness or dwell
        // baselines — a stale `entryMs` would time the first check out on its very first tick.
        _selfTest = SelfTestState();
        break;

    default:
        break;
    }
}

void OperatorConsoleClass::_resumeState(Console::State next)
{
    // NO entry effects, and no read is dropped. The console is returning to the view it was
    // already showing when the both-button chord parked it, so every scrap of that view's
    // accumulated context must survive: the VIEW_GYRO drift baseline (AC8 measures drift since
    // the view OPENED — silently re-baselining on a thumb-brush would hand the operator a
    // number they trust and that is wrong), the VIEW_LINE / VIEW_VISION pull throttles (a
    // disarmed throttle fires an out-of-cadence read on the resume tick, defeating
    // Config::UI_VISION_SAMPLE_MS and paying its blocking cost again), and any read still in
    // flight to the view that asked for it.
    _state = next;
}

void OperatorConsoleClass::_beginWait(Console::State next, bool pendingAction)
{
    // Classify the wait BEFORE _state is overwritten — this is the only moment the state we are
    // leaving is still readable. A plain await-release wait that targets the state we are
    // standing in is a resume; every await-completion wait is a transition (it targets LIST from
    // LIST, and confirming an action genuinely leaves the view for the list).
    _waitIsResume      = !pendingAction && (next == _state);
    _waitNextState     = next;
    _waitPendingAction = pendingAction;
    _actionDoneLatch   = false;
    _state             = Console::State::WAIT_RELEASE;
}

void OperatorConsoleClass::_updateWaitRelease(const Console::ConsoleInput& in)
{
    if (_waitPendingAction)
    {
        // Await-COMPLETION: only the matching notifyActionDone() releases this half. The
        // dispatched calibration routines block the whole cooperative loop, so there is no
        // deadline to enforce here — the adapter always reports back.
        if (!_actionDoneLatch)
        {
            return;
        }
        _actionDoneLatch   = false;
        _waitPendingAction = false;
        // Fall through to the release gate: the confirming button may still be held.
    }

    // Await-RELEASE: both buttons must read released for one tick.
    if (!in.btnUp && !in.btnDown)
    {
        const bool resume = _waitIsResume;
        _waitIsResume     = false;

        if (resume)
        {
            _resumeState(_waitNextState);
        }
        else
        {
            _enterState(_waitNextState);
        }
    }
}

void OperatorConsoleClass::_handleListButtons(
    const Console::ConsoleInput& in,
    bool                         upEdge,
    bool                         downEdge)
{
    if (downEdge)
    {
        _cursor = static_cast<uint8_t>((_cursor + 1) % Console::ITEM_COUNT);
        _scrollWindowTop();
        return;
    }
    if (upEdge)
    {
        _confirmItem(in);
    }
}

void OperatorConsoleClass::_handleViewButtons(const Console::ConsoleInput& in, bool downEdge)
{
    (void) in;

    if (!downEdge)
    {
        // No timeout, by design: a view stays up until the operator leaves it.
        return;
    }

    if (_state == Console::State::VIEW_VISION || _state == Console::State::VISION_TEST)
    {
        // An advisory diagnostic must never leave a latched fault for the mission.
        _clearVisionPending = true;
    }
    else if (_state == Console::State::SELF_TEST && _check == Console::Check::VISION &&
             _selfTest.readSent)
    {
        // The SELF TEST vision check owes the same clear, and this is the ONLY place it can be
        // discharged on the abort route: update() runs this handler first and then calls
        // _updateSelfTest() only while `_state` is STILL State::SELF_TEST, so the sequencer body
        // never executes on the aborting tick. Gated on `readSent` so a vision module that never
        // came up (AC25) is never commanded — and so an aborted BATT/GYRO/LINE check, which owes
        // nothing, never emits a spurious clear.
        _clearVisionPending = true;
    }

    // Cursor is preserved — the operator returns to the entry they came from.
    _enterState(Console::State::LIST);
}

void OperatorConsoleClass::_confirmItem(const Console::ConsoleInput& in)
{
    const Console::Item item = static_cast<Console::Item>(_cursor);

    if (!Console::isDispatchable(item, in.boot))
    {
        // AC24: a non-dispatchable entry is a no-op emitting ActionRequest::NONE. The "! "
        // row prefix is the operator's feedback.
        return;
    }

    switch (item)
    {
    case Console::Item::SELF_TEST:
        _enterState(Console::State::SELF_TEST);
        break;
    case Console::Item::BATT:
        _enterState(Console::State::VIEW_BATT);
        break;
    case Console::Item::LINE_VIEW:
        _enterState(Console::State::VIEW_LINE);
        break;
    case Console::Item::VISION_VIEW:
        _enterState(Console::State::VIEW_VISION);
        break;
    case Console::Item::VISION_TEST:
        _enterState(Console::State::VISION_TEST);
        break;
    case Console::Item::GYRO_VIEW:
        _enterState(Console::State::VIEW_GYRO);
        break;
    case Console::Item::GYRO_ZERO:
        _dispatchAction(Console::ActionRequest::GYRO_ZERO);
        break;
    case Console::Item::CAL_LINE:
        _dispatchAction(Console::ActionRequest::CAL_LINE);
        break;
    case Console::Item::CAL_MOTOR:
        _dispatchAction(Console::ActionRequest::CAL_MOTOR);
        break;
    case Console::Item::EXIT:
        _close();
        break;
    }
}

void OperatorConsoleClass::_dispatchAction(Console::ActionRequest action)
{
    if (!_requestAction(action))
    {
        return;
    }
    _actionInFlight = action;
    _beginWait(Console::State::LIST, true);
}

void OperatorConsoleClass::_scrollWindowTop()
{
    if (_cursor < _windowTop)
    {
        _windowTop = _cursor;
    }
    else if (_cursor >= static_cast<uint8_t>(_windowTop + Console::LIST_VISIBLE_ROWS))
    {
        _windowTop = static_cast<uint8_t>(_cursor - Console::LIST_VISIBLE_ROWS + 1);
    }

    const uint8_t maxTop = static_cast<uint8_t>(Console::ITEM_COUNT - Console::LIST_VISIBLE_ROWS);
    if (_windowTop > maxTop)
    {
        _windowTop = maxTop;
    }
}

// =============================================================================================
// SELF TEST sequencer
//
// Every check runs the same three phases, and the order is load-bearing:
//
//   latch  — record `DiagSample::seq` and `nowMs` at check entry and emit the check's read, if
//            it has one. The boot report is consulted BEFORE the sample, so a module that never
//            came up is failed from the report with no request issued against it (AC25).
//   await  — judge NOTHING until the data is provably fresher than that baseline (AC22). This is
//            the whole point of the sequencer: `Vision::begin()` clears the latched fault, so a
//            vision check that judged the sample it was handed on entry would PASS an unplugged
//            camera. A check that never goes fresh FAILs on
//            `Config::UI_SELFTEST_CHECK_TIMEOUT_MS` — fail-closed, never a PASS and never a
//            stall.
//   judge   — verdict, then hold it on screen for at least `Config::UI_SELFTEST_STEP_MS`. The
//   + dwell   dwell is a readability minimum that runs AFTER the verdict, never in place of it.
//
// The sequence halts on the first FAIL with `_check` left ON the failing check, so the screen
// names the subsystem and no later check is ever run (AC11).
// =============================================================================================

void OperatorConsoleClass::_updateSelfTest(const Console::ConsoleInput& in)
{
    if (_check == Console::Check::DONE)
    {
        // Every check passed; the result screen stands until the operator leaves.
        return;
    }
    if (_selfTest.judged && !_selfTest.passed)
    {
        // Halted on the first FAIL (AC11). `_check` deliberately stays on the failing check.
        return;
    }

    if (!_selfTest.latched && !_selfTestBeginCheck(in))
    {
        // The single per-frame action slot was already taken; retry the whole latch next tick.
        return;
    }

    if (!_selfTest.judged)
    {
        _selfTestJudgeCheck(in);
        if (!_selfTest.judged)
        {
            return;    // Still awaiting fresh data, and the deadline has not expired.
        }
    }

    _selfTestAdvance(in);
}

bool OperatorConsoleClass::_selfTestBeginCheck(const Console::ConsoleInput& in)
{
    // AC25: the boot report is consulted BEFORE the sample. A module whose begin() failed is
    // failed straight from the report — never read, never sampled, never commanded.
    const bool                   bootOk = selfTestBootAllows(_check, in.boot);
    const Console::ActionRequest read   = selfTestRead(_check);

    bool sent = false;
    if (bootOk && read != Console::ActionRequest::NONE)
    {
        // Emit BEFORE latching anything. A check latched without the read it depends on would
        // sit in await until it timed out, turning a busy action slot into a spurious FAIL.
        if (!_requestAction(read))
        {
            return false;
        }
        _readInFlight = read;
        sent          = true;
    }

    _selfTest.entrySeq  = in.sample.seq;
    _selfTest.entryMs   = in.nowMs;
    _selfTest.verdictMs = in.nowMs;
    _selfTest.latched   = true;
    _selfTest.readSent  = sent;
    _selfTest.judged    = false;
    _selfTest.passed    = false;
    _selfTest.timedOut  = false;

    if (sent && read == Console::ActionRequest::READ_LINE_BOARDS)
    {
        // Held true from the tick READ_LINE_BOARDS is emitted until the line verdict lands, so
        // the sketch's line-array tick gate stays forced open for the duration (AC5).
        _selfTestLineDataWanted = true;
    }

    if (!bootOk)
    {
        _selfTestRecord(in, false, false);
    }
    return true;
}

void OperatorConsoleClass::_selfTestJudgeCheck(const Console::ConsoleInput& in)
{
    if (_selfTestDataFresh(in))
    {
        const CheckVerdict verdict = selfTestJudge(_check, in.sample);
        if (verdict != CheckVerdict::PENDING)
        {
            _selfTestRecord(in, verdict == CheckVerdict::PASS, false);
            return;
        }
        // PENDING (a battery retry in flight) falls through to the deadline below: an
        // inconclusive check still has to settle inside its timeout budget.
    }

    // Fail-closed. `State::SELF_TEST` owns BOTH reads, so `_enterState`'s destination-keyed slot
    // release never fires while the sequencer is running — this deadline is the only thing
    // standing between a silent subsystem and an unbounded stall.
    if ((uint32_t) (in.nowMs - _selfTest.entryMs) >= Config::UI_SELFTEST_CHECK_TIMEOUT_MS)
    {
        _selfTestRecord(in, false, true);
    }
}

bool OperatorConsoleClass::_selfTestDataFresh(const Console::ConsoleInput& in) const
{
    // Rollover-safe spelling of `seq > entrySeq` for a monotonic counter: after ~4.3e9 refreshes
    // `seq` wraps, and `>` would then reject every subsequent sample forever.
    if (in.sample.seq == _selfTest.entrySeq)
    {
        return false;
    }

    // A check that dispatched a read additionally needs that read's notifyReadDone(). Without
    // this clause the adapter's PUSHED cadence — which bumps `seq` every
    // Config::UI_VIEW_SAMPLE_MS regardless of any read — would make the boot-state sample look
    // "fresh" and let Check::VISION PASS a camera that never answered.
    if (_selfTest.readSent && _readInFlight != Console::ActionRequest::NONE)
    {
        return false;
    }
    return true;
}

void OperatorConsoleClass::_selfTestRecord(
    const Console::ConsoleInput& in,
    bool                         passed,
    bool                         timedOut)
{
    _selfTest.judged    = true;
    _selfTest.passed    = passed;
    _selfTest.timedOut  = timedOut;
    _selfTest.verdictMs = in.nowMs;

    if (_check == Console::Check::LINE)
    {
        // The line verdict is recorded, so the forced-open tick gate is no longer owed.
        _selfTestLineDataWanted = false;
    }

    if (!passed)
    {
        // SECOND writer of `_signal` — `notifyActionDone()` publishes the CAL_LINE / CAL_MOTOR /
        // GYRO_ZERO verdicts. Last-write-wins is deliberate, not accidental: a SELF TEST verdict
        // is always the more recent statement about the robot's fitness, so it must override a
        // stale action verdict the adapter has not finished acknowledging. Published at VERDICT
        // time rather than after the dwell so the LEDs and the buzzer start the moment the
        // failure is known.
        _signal = Console::Signal::FAIL;
    }
}

void OperatorConsoleClass::_selfTestAdvance(const Console::ConsoleInput& in)
{
    // Readability dwell — AFTER the verdict, never instead of one.
    if ((uint32_t) (in.nowMs - _selfTest.verdictMs) < Config::UI_SELFTEST_STEP_MS)
    {
        return;
    }

    if (!_selfTest.passed)
    {
        // Halt on the first FAIL: `_check` stays put so `_renderSelfTest` keeps naming it.
        return;
    }

    if (_check == Console::Check::VISION)
    {
        // Leaving Check::VISION owes a CLEAR_VISION_FAULT — an advisory diagnostic must never
        // hand the mission a latched fault. This covers the PASS route; the BTN_DOWN abort route
        // is hooked in _handleViewButtons(), because update() stops calling this function the
        // moment that handler leaves State::SELF_TEST.
        _clearVisionPending = true;
    }

    const Console::Check next = selfTestNext(_check);
    if (next == Console::Check::DONE)
    {
        // All four checks passed. Same last-write-wins precedence as the FAIL path above.
        //
        // The placement is deliberately ASYMMETRIC with that FAIL: a failure is published the
        // moment the verdict lands, whereas a pass is published only after the final check's
        // dwell has elapsed (this function early-returns until then). A failure is urgent and
        // should light the LEDs immediately; a pass is not, and firing it at verdict time would
        // sound the all-clear while the last check's result was still being read off the screen.
        _signal = Console::Signal::PASS;
    }

    // Assign `_check` DIRECTLY. `_enterState(State::SELF_TEST)` would re-home it to
    // `Check::BATT` — the case has no self-transition guard — looping the sequencer on the
    // battery check forever.
    _check             = next;
    _selfTest.latched  = false;
    _selfTest.readSent = false;
    _selfTest.judged   = false;
    _selfTest.passed   = false;
    _selfTest.timedOut = false;
}

bool OperatorConsoleClass::_requestAction(Console::ActionRequest action)
{
    if (_frameAction != Console::ActionRequest::NONE)
    {
        return false;
    }
    _frameAction = action;
    return true;
}

void OperatorConsoleClass::_servicePullCadence(const Console::ConsoleInput& in)
{
    // Never a second read while one is unacknowledged (TC25), and never a read against a
    // module whose begin() failed (AC25).
    if (_readInFlight != Console::ActionRequest::NONE)
    {
        return;
    }

    if (_state == Console::State::VIEW_LINE)
    {
        if (!in.boot.lineArray)
        {
            return;
        }
        const bool due =
            !_lineReqIssued ||
            (uint32_t) (in.nowMs - _lastLineReqMs) >= Config::UI_VIEW_SAMPLE_MS;
        if (due && _requestAction(Console::ActionRequest::READ_LINE_BOARDS))
        {
            _readInFlight  = Console::ActionRequest::READ_LINE_BOARDS;
            _lastLineReqMs = in.nowMs;
            _lineReqIssued = true;
        }
        return;
    }

    if (_state == Console::State::VIEW_VISION)
    {
        if (!in.boot.vision)
        {
            return;
        }
        const bool due =
            !_visionReqIssued ||
            (uint32_t) (in.nowMs - _lastVisionReqMs) >= Config::UI_VISION_SAMPLE_MS;
        if (due && _requestAction(Console::ActionRequest::READ_MOSAIC))
        {
            _readInFlight    = Console::ActionRequest::READ_MOSAIC;
            _lastVisionReqMs = in.nowMs;
            _visionReqIssued = true;
        }
        return;
    }

    if (_state == Console::State::VISION_TEST)
    {
        // ONE-SHOT (not a cadence): grade a single fresh read per entry. Guarded by
        // `_visionTestReqIssued` so the read fires exactly once; re-entering the item re-tests.
        if (!in.boot.vision)
        {
            return;
        }
        if (!_visionTestReqIssued && _requestAction(Console::ActionRequest::READ_MOSAIC))
        {
            _readInFlight        = Console::ActionRequest::READ_MOSAIC;
            _visionTestReqIssued = true;
        }
    }
}

void OperatorConsoleClass::_updateGyroBaseline(const Console::ConsoleInput& in)
{
    // A failed read never (re-)baselines — the view holds the last baseline and shows "--".
    if (!_gyroBaselineSet && in.sample.gyroReadOk)
    {
        _gyroBaselineDeg = in.sample.gyroYawDeg;
        _gyroBaselineSet = true;
    }
}

void OperatorConsoleClass::_updateVisionTest(const Console::ConsoleInput& in)
{
    if (_visionTestJudged)
    {
        return;    // One-shot: graded already. Re-enter the item to run a fresh test.
    }
    if (!_visionTestReqIssued)
    {
        return;    // The one-shot read has not been dispatched yet (that is `_servicePullCadence`).
    }
    if (_readInFlight == Console::ActionRequest::READ_MOSAIC)
    {
        return;    // Read still in flight — wait for its notifyReadDone before grading.
    }

    // The requested read has completed (notifyReadDone cleared `_readInFlight`) and the adapter has
    // refreshed the pulled `visionCell` / `visionFaultKind`. Grade exactly once and raise the
    // verdict signal so the adapter's LED + buzzer report PASS / FAIL.
    _visionTestJudged = true;

    if (in.sample.visionFaultKind != kVisionFaultNone)
    {
        // A transport fault means the board could not be read at all — cannot confirm a match.
        _visionTestPass = false;
        _signal         = Console::Signal::FAIL;
        return;
    }

    bool allMatch = true;
    for (uint8_t c = 0; c < kVisionCellCount; ++c)
    {
        if (in.sample.visionCell[c] != kExpectedMosaic[c])
        {
            allMatch = false;
            break;
        }
    }
    _visionTestPass = allMatch;
    _signal         = allMatch ? Console::Signal::PASS : Console::Signal::FAIL;
}

// =============================================================================================
// Rendering
// =============================================================================================

void OperatorConsoleClass::_publishFrame(
    const Console::ConsoleInput& in,
    Console::ConsoleFrame&       out)
{
    _clearFrame(out);

    switch (_state)
    {
    case Console::State::LIST:
        _renderList(in, out);
        break;
    case Console::State::VIEW_LINE:
        _renderLineView(in, out);
        break;
    case Console::State::VIEW_VISION:
        _renderVisionView(in, out);
        break;
    case Console::State::VISION_TEST:
        _renderVisionTest(in, out);
        break;
    case Console::State::VIEW_GYRO:
        _renderGyroView(in, out);
        break;
    case Console::State::VIEW_BATT:
        _renderBattView(in, out);
        break;
    case Console::State::SELF_TEST:
        _renderSelfTest(out);
        break;
    case Console::State::WAIT_RELEASE:
        _renderWaitRelease(out);
        break;
    case Console::State::CLOSED:
        // Nothing to show; the blank frame stands.
        break;
    }

    // `dirty` is LEVEL-triggered: raise it whenever the rows differ from the last frame the
    // adapter acknowledged, and never lower it here — only notifyFrameRendered() may do that.
    // A repaint suppressed by the adapter's repaint cadence therefore survives to the next tick.
    for (uint8_t r = 0; r < Console::FRAME_ROWS; ++r)
    {
        for (uint8_t c = 0; c < Console::FRAME_COLS; ++c)
        {
            _lastRows[r][c] = out.rows[r][c];
            if (_shownRows[r][c] != out.rows[r][c])
            {
                _dirty = true;
            }
        }
    }

    out.dirty  = _dirty;
    out.action = _frameAction;
    out.signal = _signal;
}

void OperatorConsoleClass::_clearFrame(Console::ConsoleFrame& out)
{
    for (uint8_t r = 0; r < Console::FRAME_ROWS; ++r)
    {
        for (uint8_t c = 0; c < Console::FRAME_COLS; ++c)
        {
            out.rows[r][c] = '\0';
        }
    }
    out.cursorRow = 0xFF;
}

void OperatorConsoleClass::_renderList(
    const Console::ConsoleInput& in,
    Console::ConsoleFrame&       out) const
{
    _rowSet(out.rows[0], kTitle);

    for (uint8_t k = 0; k < Console::LIST_VISIBLE_ROWS; ++k)
    {
        const uint8_t item = static_cast<uint8_t>(_windowTop + k);
        if (item >= Console::ITEM_COUNT)
        {
            break;
        }

        const bool dispatchable =
            Console::isDispatchable(static_cast<Console::Item>(item), in.boot);
        const bool onCursor = (item == _cursor);

        // A non-dispatchable entry always shows "! " — the operator must see WHY confirming it
        // does nothing, and `out.cursorRow` still carries the highlight for the adapter.
        const char* prefix = kPrefixPlain;
        if (!dispatchable)
        {
            prefix = kPrefixBlocked;
        }
        else if (onCursor)
        {
            prefix = kPrefixCursor;
        }

        char*   row = out.rows[k + 1];
        uint8_t pos = _rowAppend(row, 0, prefix);
        _rowAppend(row, pos, kItemLabels[item]);

        if (onCursor)
        {
            out.cursorRow = static_cast<uint8_t>(k + 1);
        }
    }
}

void OperatorConsoleClass::_renderLineView(
    const Console::ConsoleInput& in,
    Console::ConsoleFrame&       out) const
{
    _rowSet(out.rows[0], "LINE VIEW");

    uint8_t boards = in.sample.lineBoardCount;
    if (boards > kLineBoardMax)
    {
        boards = kLineBoardMax;
    }

    for (uint8_t b = 0; b < kLineBoardMax; ++b)
    {
        char* row = out.rows[b + 1];
        if (b >= boards)
        {
            continue;    // Unconfigured board: leave the row blank.
        }

        uint8_t pos = _rowAppend(row, 0, "BOARD");
        pos         = _rowAppendInt(row, pos, static_cast<int32_t>(b));
        pos         = _rowAppendChar(row, pos, ':');

        if (!in.sample.lineBoardOk[b])
        {
            // Never substitute zeros for a failed read — a zero is a legitimate channel value.
            _rowAppend(row, pos, " NO DATA");
            continue;
        }

        for (uint8_t c = 0; c < kLineChannelCount; ++c)
        {
            const uint8_t v = in.sample.lineRaw[b][c];
            pos = _rowAppendChar(row, pos, (v <= 9) ? static_cast<char>('0' + v) : '?');
        }
    }

    char*   row = out.rows[3];
    uint8_t pos = _rowAppend(row, 0, "J:");
    pos         = _rowAppendInt(row, pos, static_cast<int32_t>(in.sample.junctionType));
    pos         = _rowAppend(row, pos, " ON:");
    pos         = _rowAppendChar(row, pos, in.sample.lineOnline ? '1' : '0');
    pos         = _rowAppend(row, pos, " E:");
    _rowAppendFixed1(row, pos, static_cast<double>(in.sample.lineError));
}

void OperatorConsoleClass::_renderVisionView(
    const Console::ConsoleInput& in,
    Console::ConsoleFrame&       out) const
{
    _rowSet(out.rows[0], "VISION VIEW");

    const uint8_t half = static_cast<uint8_t>(kVisionCellCount / 2);

    uint8_t pos = _rowAppend(out.rows[1], 0, "C1-6:");
    for (uint8_t i = 0; i < half; ++i)
    {
        const uint8_t colour = in.sample.visionCell[i];
        pos = _rowAppendChar(out.rows[1], pos, ' ');
        pos = _rowAppendChar(
            out.rows[1], pos, (colour < kColorLetterCount) ? kColorLetters[colour] : '?');
    }

    pos = _rowAppend(out.rows[2], 0, "C7-12:");
    for (uint8_t i = half; i < kVisionCellCount; ++i)
    {
        const uint8_t colour = in.sample.visionCell[i];
        pos = _rowAppendChar(out.rows[2], pos, ' ');
        pos = _rowAppendChar(
            out.rows[2], pos, (colour < kColorLetterCount) ? kColorLetters[colour] : '?');
    }

    pos = _rowAppend(out.rows[3], 0, "FAULT: ");
    _rowAppend(out.rows[3], pos, visionFaultName(in.sample.visionFaultKind));
}

void OperatorConsoleClass::_renderVisionTest(
    const Console::ConsoleInput& in,
    Console::ConsoleFrame&       out) const
{
    // Row 0: title, plus the verdict tag once the read has been graded.
    uint8_t pos0 = _rowAppend(out.rows[0], 0, "VISION TEST");
    if (_visionTestJudged)
    {
        _rowAppend(out.rows[0], pos0, _visionTestPass ? "  PASS" : "  FAIL");
    }

    // Rows 1-2: detected (GOT) vs expected (EXP) colour letters, aligned column-for-column so the
    // operator eyeballs which cell differs. No spaces between letters — 4 + 12 = 16 <= TEXT_COLS.
    uint8_t p1 = _rowAppend(out.rows[1], 0, "GOT ");
    uint8_t p2 = _rowAppend(out.rows[2], 0, "EXP ");
    for (uint8_t c = 0; c < kVisionCellCount; ++c)
    {
        const uint8_t got = in.sample.visionCell[c];
        const uint8_t exp = kExpectedMosaic[c];
        p1 = _rowAppendChar(out.rows[1], p1, (got < kColorLetterCount) ? kColorLetters[got] : '?');
        p2 = _rowAppendChar(out.rows[2], p2, (exp < kColorLetterCount) ? kColorLetters[exp] : '?');
    }

    // Row 3: status line — reading, read fault, all-match, or the first mismatch spelled out.
    if (!_visionTestJudged)
    {
        _rowSet(out.rows[3], "reading...");
        return;
    }
    if (in.sample.visionFaultKind != kVisionFaultNone)
    {
        _rowSet(out.rows[3], "READ FAULT");
        return;
    }

    uint8_t wrong    = 0;
    uint8_t firstBad = 0xFF;
    for (uint8_t c = 0; c < kVisionCellCount; ++c)
    {
        if (in.sample.visionCell[c] != kExpectedMosaic[c])
        {
            ++wrong;
            if (firstBad == 0xFF)
            {
                firstBad = c;
            }
        }
    }

    if (wrong == 0)
    {
        _rowSet(out.rows[3], "all 12 match");
        return;
    }

    // "<n> bad c<i> <E>-><G>" — e.g. "1 bad c4 G->?".
    char*         row = out.rows[3];
    uint8_t       pos = _rowAppendInt(row, 0, static_cast<int32_t>(wrong));
    pos               = _rowAppend(row, pos, " bad c");
    pos               = _rowAppendInt(row, pos, static_cast<int32_t>(firstBad));
    pos               = _rowAppendChar(row, pos, ' ');
    const uint8_t e   = kExpectedMosaic[firstBad];
    const uint8_t g   = in.sample.visionCell[firstBad];
    pos               = _rowAppendChar(row, pos, (e < kColorLetterCount) ? kColorLetters[e] : '?');
    pos               = _rowAppend(row, pos, "->");
    _rowAppendChar(row, pos, (g < kColorLetterCount) ? kColorLetters[g] : '?');
}

void OperatorConsoleClass::_renderGyroView(
    const Console::ConsoleInput& in,
    Console::ConsoleFrame&       out) const
{
    _rowSet(out.rows[0], "GYRO VIEW");

    uint8_t pos = _rowAppend(out.rows[1], 0, "YAW: ");
    if (in.sample.gyroReadOk)
    {
        _rowAppendFixed1(out.rows[1], pos, in.sample.gyroYawDeg);
    }
    else
    {
        _rowAppend(out.rows[1], pos, "--");
    }

    pos = _rowAppend(out.rows[2], 0, "HEALTH: ");
    _rowAppend(out.rows[2], pos, in.sample.gyroHealthy ? "OK" : "BAD");

    pos = _rowAppend(out.rows[3], 0, "DRIFT: ");
    if (!_gyroBaselineSet || !in.sample.gyroReadOk)
    {
        _rowAppend(out.rows[3], pos, "--");
        return;
    }

    const double drift = _wrapDeltaDeg(in.sample.gyroYawDeg, _gyroBaselineDeg);
    if (drift > 0.0)
    {
        pos = _rowAppendChar(out.rows[3], pos, '+');
    }
    pos = _rowAppendFixed1(out.rows[3], pos, drift);

    const double magnitude = (drift < 0.0) ? -drift : drift;
    if (magnitude >= static_cast<double>(Config::UI_GYRO_DRIFT_WARN_DEG))
    {
        _rowAppend(out.rows[3], pos, " WARN");
    }
}

void OperatorConsoleClass::_renderBattView(
    const Console::ConsoleInput& in,
    Console::ConsoleFrame&       out) const
{
    _rowSet(out.rows[0], "BATT");

    if (!in.sample.battReadOk || in.sample.battVolts == 0.0f)
    {
        _rowSet(out.rows[1], in.sample.battRetryPending ? "--.- V RETRY" : "--.- V");
    }
    else
    {
        uint8_t pos = _rowAppendFixed1(out.rows[1], 0, static_cast<double>(in.sample.battVolts));
        pos         = _rowAppend(out.rows[1], pos, "V ");
        pos         = _rowAppendInt(out.rows[1], pos, static_cast<int32_t>(in.sample.battPercent));
        _rowAppendChar(out.rows[1], pos, '%');
    }

    _renderBootFailures(in.boot, out);
}

void OperatorConsoleClass::_renderBootFailures(
    const Console::BootReport& boot,
    Console::ConsoleFrame&     out)
{
    const char* const names[kBootFlagCount] = {
        "BATT", "LINE", "YAW", "DRV", "LF",  "INT", "MAN",
        "LOC",  "PT",   "MIS", "TSK", "VIS", "TRC",
    };
    const bool ok[kBootFlagCount] = {
        boot.batt,         boot.lineArray,    boot.yaw,     boot.drive, boot.lineFollower,
        boot.intersection, boot.manipulator,  boot.localization, boot.pathTracker,
        boot.mission,      boot.tasks,        boot.vision,  boot.trace,
    };

    // The list flows across rows 2 and 3. The last row reserves its final column for a '+'
    // truncation marker, so a boot report with more failures than fit is never silently
    // shortened into a report that reads as complete.
    const uint8_t lastRow   = static_cast<uint8_t>(Console::FRAME_ROWS - 1);
    const uint8_t lastLimit = static_cast<uint8_t>(Console::TEXT_COLS - 1);

    uint8_t row       = 2;
    uint8_t pos       = _rowAppend(out.rows[2], 0, "FAIL:");
    bool    anyBad    = false;
    bool    truncated = false;

    for (uint8_t i = 0; i < kBootFlagCount; ++i)
    {
        if (ok[i])
        {
            continue;
        }
        anyBad = true;

        const uint8_t need  = static_cast<uint8_t>(_strLen(names[i]) + 1);
        uint8_t       limit = (row >= lastRow) ? lastLimit : Console::TEXT_COLS;

        if (static_cast<uint16_t>(pos) + need > limit)
        {
            if (row >= lastRow)
            {
                truncated = true;
                break;
            }
            row   = lastRow;
            pos   = 0;
            limit = lastLimit;
            if (need > limit)
            {
                truncated = true;
                break;
            }
        }

        if (pos > 0)
        {
            pos = _rowAppendChar(out.rows[row], pos, ' ');
        }
        pos = _rowAppend(out.rows[row], pos, names[i]);
    }

    if (truncated)
    {
        _rowAppendChar(out.rows[lastRow], pos, '+');
    }

    if (!anyBad)
    {
        _rowSet(out.rows[2], "ALL MODULES OK");
    }
}

void OperatorConsoleClass::_renderWaitRelease(Console::ConsoleFrame& out) const
{
    _rowSet(out.rows[0], kTitle);

    if (_waitPendingAction)
    {
        _rowSet(out.rows[1], "WORKING...");
        _rowSet(out.rows[2], actionName(_actionInFlight));

        // CAL MOTOR only. `Drive::calibrateMotorBias()` derives each wheel's bias from a
        // DIFFERENTIAL comparison of the two encoders, and on this chassis the M2/M3 encoder
        // cables are crossed — so it attributes each wheel's result to the other one. The entry
        // is kept (an operator may still want the raw numbers), but nobody should paste them
        // into Config.h believing they are trustworthy, and this is the one screen guaranteed to
        // be in front of them while it runs. Not shown for the other actions, which are sound.
        if (_actionInFlight == Console::ActionRequest::CAL_MOTOR)
        {
            _rowSet(out.rows[3], "! ENC CABLES XED");
        }
        return;
    }
    _rowSet(out.rows[1], "RELEASE BUTTONS");
}

void OperatorConsoleClass::_renderSelfTest(Console::ConsoleFrame& out) const
{
    _rowSet(out.rows[0], "SELF TEST");

    uint8_t pos = _rowAppend(out.rows[1], 0, "CHECK: ");
    _rowAppend(out.rows[1], pos, checkName(_check));

    if (_check == Console::Check::DONE)
    {
        _rowSet(out.rows[2], "ALL PASS");
    }
    else if (!_selfTest.judged)
    {
        _rowSet(out.rows[2], "RUNNING...");
    }
    else
    {
        // The failing check is still `_check` (the sequence halts on it), so naming it here
        // names the failed subsystem (AC11). A timeout is called out separately from a real
        // verdict: "NO DATA" tells the operator the subsystem never answered at all.
        pos = _rowAppend(out.rows[2], 0, checkName(_check));
        if (_selfTest.timedOut)
        {
            _rowAppend(out.rows[2], pos, " NO DATA");
        }
        else
        {
            _rowAppend(out.rows[2], pos, _selfTest.passed ? " PASS" : " FAIL");
        }
    }

    _rowSet(out.rows[3], "DOWN = ABORT");
}

// =============================================================================================
// Bounded row formatting — every helper truncates at Console::TEXT_COLS and NUL-terminates
// =============================================================================================

double OperatorConsoleClass::_wrapDeltaDeg(double yawDeg, double baselineDeg)
{
    double delta = yawDeg - baselineDeg;
    while (delta > 180.0)
    {
        delta -= 360.0;
    }
    while (delta <= -180.0)
    {
        delta += 360.0;
    }
    return delta;
}

uint8_t OperatorConsoleClass::_strLen(const char* text)
{
    if (text == nullptr)
    {
        return 0;
    }
    uint8_t n = 0;
    while (text[n] != '\0' && n < 0xFF)
    {
        ++n;
    }
    return n;
}

uint8_t OperatorConsoleClass::_rowAppend(char* row, uint8_t pos, const char* text)
{
    if (text == nullptr)
    {
        return pos;
    }
    while (*text != '\0' && pos < Console::TEXT_COLS)
    {
        row[pos] = *text;
        ++pos;
        ++text;
    }
    row[pos] = '\0';
    return pos;
}

uint8_t OperatorConsoleClass::_rowAppendChar(char* row, uint8_t pos, char c)
{
    if (pos < Console::TEXT_COLS)
    {
        row[pos] = c;
        ++pos;
    }
    row[pos] = '\0';
    return pos;
}

uint8_t OperatorConsoleClass::_rowAppendInt(char* row, uint8_t pos, int32_t value)
{
    if (value < 0)
    {
        pos = _rowAppendChar(row, pos, '-');
    }

    // Magnitude built into a fixed buffer: int32_t needs at most 10 digits.
    char     digits[10];
    uint8_t  count     = 0;
    uint32_t magnitude = (value < 0) ? (uint32_t) (-(int64_t) value) : (uint32_t) value;

    do
    {
        digits[count] = static_cast<char>('0' + (magnitude % 10u));
        ++count;
        magnitude /= 10u;
    } while (magnitude != 0u && count < 10);

    while (count > 0)
    {
        --count;
        pos = _rowAppendChar(row, pos, digits[count]);
    }
    return pos;
}

uint8_t OperatorConsoleClass::_rowAppendFixed1(char* row, uint8_t pos, double value)
{
    const bool negative = (value < 0.0);
    double     magnitude = negative ? -value : value;
    if (!(magnitude <= 99999.0))
    {
        magnitude = 99999.0;    // Also catches NaN, which fails every comparison.
    }

    const uint32_t scaled = static_cast<uint32_t>(magnitude * 10.0 + 0.5);
    if (negative && scaled != 0u)
    {
        pos = _rowAppendChar(row, pos, '-');
    }
    pos = _rowAppendInt(row, pos, static_cast<int32_t>(scaled / 10u));
    pos = _rowAppendChar(row, pos, '.');
    pos = _rowAppendChar(row, pos, static_cast<char>('0' + (scaled % 10u)));
    return pos;
}

void OperatorConsoleClass::_rowSet(char* row, const char* text)
{
    _rowAppend(row, 0, text);
}

/**
 * @brief Single global instance (iron rule 2).
 */
OperatorConsoleClass OperatorConsole;
