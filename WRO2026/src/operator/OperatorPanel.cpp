/**
 * @file OperatorPanel.cpp
 * @brief Out-of-line members of `OperatorPanelClass` — the console's hardware adapter.
 * @author WRO 2026
 *
 * @details
 * See `OperatorPanel.h` for the class-level contract (AC21's early-return, the PUSHED/PULLED
 * split, and the pinned adapter tick order). This file is the ONLY place in the console domain
 * that touches `MiniR4.BTN_UP` / `BTN_DOWN`, the OLED, `MiniR4.PWR`, `YawSource`, `LineArray`,
 * `Vision`, `Drive` (calibration only) and `MiniR4.LED` / `MiniR4.Buzzer`. `OperatorConsole.cpp`
 * (the headless core) never includes `<MatrixMiniR4.h>` — this TU does, per the `WHITE` macro
 * note below.
 *
 * @note **The `WHITE` macro.** `MiniR4OLED.h` `#define`s bare `WHITE` / `BLACK` / `INVERSE`. This
 *       file includes `<MatrixMiniR4.h>`, so `MiniR4.OLED.setTextColor(WHITE)` resolves to the
 *       macro and is correct here. Never name `Config::Color::WHITE` in this translation unit.
 * @note **`probeBoard()` leaves the PCA9548A mux selected on the probed channel** (it does not
 *       restore the prior selection). That is safe here for two SEPARATE reasons, and neither
 *       generalises:
 *       - The muxed sensor accessors each select their channel before their own transaction
 *         (`MatrixColor::getColor()` calls `i2cMUXSelect()`; `probeBoard()` mirrors it inline —
 *         `i2cMUXSelect()` is private to `MatrixColor`, so `WROLineArray.cpp` open-codes the same
 *         `ADDR_PCA954X` write plus the 300 us settle), so neither inherits a stale channel.
 *       - The OLED is immune by TOPOLOGY, not by self-selection: `MiniR4OLED.h` is a verbatim
 *         `Adafruit_SSD1306` with no mux logic whatsoever, and `MatrixMiniR4.cpp` constructs it as
 *         `Adafruit_SSD1306(128, 32, &Wire1, -1)` — it hangs off the Wire1 TRUNK, upstream of the
 *         PCA9548A, so whichever channel is selected is irrelevant to it.
 *       Hand-rolled `Wire1` code that talks to a device BEHIND the mux gets neither guarantee and
 *       must select its own channel; do not assume a persistent mux channel across calls here.
 */

#include "OperatorPanel.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>

#include "../../Config.h"
#include "../drive/Drive.h"
#include "../nav/YawSource.h"
#include "../perception/Vision.h"
#include "../perception/WROLineArray.h"
#include "../util/EventTrace.h"
#include "ConsoleTypes.h"
#include "OperatorConsole.h"

/// @brief Single global instance declared `extern` in OperatorPanel.h.
OperatorPanelClass OperatorPanel;

// ---------------------------------------------------------------------------------------------
// Cross-module tripwire: the console core's mirrored VisionClass::Fault codes
// ---------------------------------------------------------------------------------------------
//
// `Console::DiagSample::visionFaultKind` carries a VisionClass::Fault as a raw uint8_t, because
// ConsoleTypes.h must stay free of every src/ header (it is the seam that lets the headless core
// compile in the host ctest harness). OperatorConsole.cpp therefore RE-DECLARES the numeric codes
// as local kVisionFault* constants, and its SELF TEST VISION check decides PASS/FAIL from them.
// Nothing else enforces that mirror: inserting a member above READ_TIMEOUT would renumber the
// enum, silently turn a dead camera into a PASS, and pass every host test (they drive the PODs and
// never see the real enum). This TU is the one place that legitimately includes BOTH Vision.h and
// ConsoleTypes.h, so the mirror is pinned here. If one of these fires, fix the kVisionFault*
// constants in OperatorConsole.cpp to match — never the other way round.
static_assert(
    static_cast<uint8_t>(VisionClass::Fault::NONE) == 0,
    "VisionClass::Fault::NONE must stay 0 — OperatorConsole.cpp mirrors it as kVisionFaultNone");
static_assert(
    static_cast<uint8_t>(VisionClass::Fault::READ_TIMEOUT) == 1,
    "VisionClass::Fault::READ_TIMEOUT must stay 1 — mirrored as kVisionFaultReadTimeout");
static_assert(
    static_cast<uint8_t>(VisionClass::Fault::BAD_CHECKSUM) == 2,
    "VisionClass::Fault::BAD_CHECKSUM must stay 2 — mirrored as kVisionFaultBadChecksum");
static_assert(
    static_cast<uint8_t>(VisionClass::Fault::BAD_CELL) == 3,
    "VisionClass::Fault::BAD_CELL must stay 3 — mirrored as kVisionFaultBadCell");

namespace
{
    /**
     * @brief `DiagSample::visionFaultKind` poison written when a `READ_MOSAIC` is REFUSED.
     *
     * Deliberately outside the mirrored `VisionClass::Fault` contract pinned above. The core's
     * `judgeVision()` is an ALLOW-LIST (only `NONE` and `BAD_CELL` PASS; every unrecognised code
     * FAILs), so this value makes a refused read fail closed instead of leaving the previous —
     * at boot, all-zero and therefore PASSing — sample in place. See `_refuseRequest()`.
     */
    constexpr uint8_t kVisionFaultRefused = 0xFF;
}

// -----------------------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------------------

bool OperatorPanelClass::begin()
{
    OperatorConsole.begin();

    _missionArmed  = false;
    _pendingAction = Console::ActionRequest::NONE;

    // Value-initialise the sample to its all-sentinel defaults: seq=0, every PUSHED float/bool
    // at its documented "read failed" value (battVolts=0.0f, battReadOk=false, gyroReadOk=false,
    // gyroHealthy=false, junctionType=0/NONE, lineOnline=false, lineError=0.0f), every PULLED
    // array zeroed (visionCell all UNKNOWN==0, lineRaw all 0, lineBoardPresent/Ok all false).
    _sample = Console::DiagSample();

    // Prime every cadence timer to 0 so the FIRST update() after opening refreshes and repaints
    // immediately rather than waiting out a full Config::UI_VIEW_SAMPLE_MS / UI_REPAINT_INTERVAL_MS
    // period — mirrors EventTrace's `_lastHeartbeat_ms = 0` idiom.
    _lastPushMs        = 0;
    _battLastAttemptMs = 0;
    _battAttemptsMade  = 0;
    _lastRenderMs      = 0;

    _signalActive      = false;
    _signalKind        = Console::Signal::NONE;
    _signalStartMs     = 0;
    _signalChirpsFired = 0;

    // Always true on this platform; declared `bool` for parity with Drive/Manipulator/Vision
    // begin() so the call site can guard on the return value if a future failure path is added.
    return true;
}

void OperatorPanelClass::open()
{
    // Sample the transition BEFORE delegating: OperatorConsole::open() no-ops when the console is
    // already open, so pushing unconditionally would spam the 4-slot event ring on a repeat call.
    const bool wasOpen = OperatorConsole.isOpen();

    OperatorConsole.open();

    if (!wasOpen)
    {
        EventTrace.pushEvent("DIAG MENU");
    }
}

// -----------------------------------------------------------------------------------------
// Delegating queries — hold no state of their own (class-level docs)
// -----------------------------------------------------------------------------------------

bool OperatorPanelClass::isOpen() const
{
    return OperatorConsole.isOpen();
}

bool OperatorPanelClass::wantsLineData() const
{
    return OperatorConsole.wantsLineData();
}

void OperatorPanelClass::setBootReport(const Console::BootReport& boot)
{
    _boot = boot;
}

void OperatorPanelClass::setMissionArmed(bool armed)
{
    _missionArmed = armed;
}

// -----------------------------------------------------------------------------------------
// Per-tick entry point — the pinned adapter tick order (see OperatorPanel.h class docs)
// -----------------------------------------------------------------------------------------

void OperatorPanelClass::update()
{
    // Step 1 — AC21 outer gate, FIRST statement. While the console is closed this method costs
    // one isOpen() call and nothing else: no button poll, no sampling, no readBoard()/probeBoard(),
    // no YawSource/Vision/MiniR4.PWR read, no OLED write, no LED/buzzer drive. The button reads
    // sit BELOW the gate because MiniR4.BTN_*.getState() is an mmL SoftwareSerial round-trip to
    // the STM32 (WaitData busy-spin worst case), not a GPIO read, and this runs from loop() every
    // iteration including mid-mission. Nothing is lost: the core seeds its own _btnUpPrev/
    // _btnDownPrev from in.btnUp/in.btnDown on the opening tick, so no cross-tick button
    // continuity is owed while closed.
    if (!OperatorConsole.isOpen())
    {
        return;
    }

    // Step 2 — Poll buttons. A `mmL` (SoftwareSerial) transaction, NOT I2C.
    const bool btnUp   = MiniR4.BTN_UP.getState();
    const bool btnDown = MiniR4.BTN_DOWN.getState();

    uint32_t now = millis();

    // Belt-and-braces (see class docs): OperatorConsole::isOpen() deliberately reports true
    // while a just-latched open() request is still pending, so if open() were ever called while
    // armed, the gate above would still let this tick through for one pass. Skip SAMPLING in that
    // case, but still call OperatorConsole.update() below so the core's OWN missionArmed
    // force-close (the inner gate) can run and clear its latch/FSM. Unreachable today — T7 only
    // triggers open() pre-arm — but cheap to close and strictly cheaper than doing nothing here.
    if (!_missionArmed)
    {
        // Step 3 — Refresh the PUSHED group on its cadence.
        _servicePushedGroup(now);

        // Step 4 — Execute a request pended from the PREVIOUS tick, write its PULLED fields,
        // bump seq, then notify. At most one request is ever in flight.
        _serviceRequestCompletion();

        // A dispatched CAL_LINE blocks inside LineArray.calibrate() for tens of seconds, so the
        // capture above is stale by the time we get here. Re-read the clock: every timestamp
        // written from this point on (the core's own time source, the repaint cadence, the signal
        // sequencer's start stamp) must reflect the real present, not the pre-dispatch instant.
        now = millis();
    }

    Console::ConsoleInput in;
    in.btnUp        = btnUp;
    in.btnDown      = btnDown;
    in.nowMs        = now;
    in.missionArmed = _missionArmed;
    in.sample       = _sample;
    in.boot         = _boot;

    const Console::State stateBefore = OperatorConsole.state();

    // Step 5 — Advance the console core and render its frame. Value-initialised: the core fills
    // every field on every call, but this adapter must not depend on an invariant it cannot
    // enforce, and ~88 bytes of stack zeroing at this cadence is free.
    Console::ConsoleFrame frame{};
    OperatorConsole.update(in, frame);

    const Console::State stateAfter = OperatorConsole.state();
    _traceStateTransition(stateBefore, stateAfter);

    if (!OperatorConsole.isOpen())
    {
        // Just closed (EXIT confirmed, or the missionArmed force-close). Nothing left to render
        // or signal, and no request emitted from a frame belonging to a closed console can be
        // legitimate — drop it rather than latching it for a tick that will never execute it.
        _pendingAction = Console::ActionRequest::NONE;

        // Abandon any half-played verdict pattern too. _close() already cleared the core's
        // `_signal`, and this branch returns before _serviceSignal() can observe that, so leaving
        // _signalActive raised would strand a stale _signalKind/_signalStartMs/_signalChirpsFired.
        // The next verdict of the SAME kind would then skip the start block entirely (LEDs never
        // repainted) and measure `elapsed` against the stale start — firing every remaining chirp
        // in one tick, i.e. three Tone() calls microseconds apart merging into a single beep.
        _signalActive = false;
        _signalKind   = Console::Signal::NONE;
        return;
    }

    // Step 6 — Repaint if dirty and due.
    _renderFrame(frame, now);

    // Step 7 — Drive the verdict signal; notifySignalDone() when the pattern completes.
    _serviceSignal(frame.signal, now);

    // Step 8 — Latch this frame's request for execution at the top of the NEXT tick.
    _pendingAction = frame.action;
}

// -----------------------------------------------------------------------------------------
// PUSHED-group sampling
// -----------------------------------------------------------------------------------------

void OperatorPanelClass::_servicePushedGroup(uint32_t now)
{
    // Gyro + line-derived telemetry: fixed Config::UI_VIEW_SAMPLE_MS cadence.
    if ((uint32_t) (now - _lastPushMs) >= Config::UI_VIEW_SAMPLE_MS)
    {
        _refreshGyro();
        _refreshLineDerived();
        _lastPushMs = now;
        _bumpSeq();
    }

    // Battery: same cadence normally, but narrows to Config::UI_BATT_RETRY_MS while a retry is
    // pending so the non-blocking retry lands quickly instead of waiting out a full group period.
    const uint32_t battInterval =
        _sample.battRetryPending ? Config::UI_BATT_RETRY_MS : Config::UI_VIEW_SAMPLE_MS;
    if ((uint32_t) (now - _battLastAttemptMs) >= battInterval)
    {
        _refreshBattery();
        _battLastAttemptMs = now;
        _bumpSeq();
    }
}

void OperatorPanelClass::_refreshGyro()
{
    if (!_boot.yaw)
    {
        // AC25 — never sampled once YawSource::begin() failed; the sentinels begin() latched
        // (gyroReadOk=false, gyroHealthy=false) stay in place for the rest of the power cycle.
        return;
    }

    double yaw = 0.0;
    if (YawSource.readYawDeg(yaw))
    {
        _sample.gyroYawDeg = yaw;
        _sample.gyroReadOk = true;
    }
    else
    {
        // readYawDeg() leaves `yaw` untouched on a dead-bus read; mirror that by leaving
        // gyroYawDeg at its prior value and only flipping the read-ok flag.
        _sample.gyroReadOk = false;
    }
    _sample.gyroHealthy = YawSource.isHealthy();
}

void OperatorPanelClass::_refreshLineDerived()
{
    // wantsLineData() is always false when !_boot.lineArray (AC25), so this single gate covers
    // both "is a view/check currently asking for this" and "did the module even come up".
    if (!OperatorConsole.wantsLineData())
    {
        return;
    }

    _sample.junctionType = LineArray.getJunctionType();
    _sample.lineOnline   = LineArray.isOnline();
    _sample.lineError    = LineArray.getError();
}

void OperatorPanelClass::_refreshBattery()
{
    // Runs regardless of _boot.batt — a boot-time low-voltage verdict never disables the live
    // read; the pack may have recovered (Decisions: "boot.batt is a voltage verdict, not an init
    // verdict").
    const float volts = MiniR4.PWR.getBattVoltage();

    if (volts <= 0.0f)
    {
        ++_battAttemptsMade;
        _sample.battVolts   = 0.0f;
        _sample.battPercent = 0;
        _sample.battReadOk  = false;

        if (_battAttemptsMade >= Config::UI_BATT_RETRY_ATTEMPTS)
        {
            // Exhausted the retry budget — give up until the next fresh attempt cycle.
            _sample.battRetryPending = false;
            _battAttemptsMade        = 0;
        }
        else
        {
            _sample.battRetryPending = true;
        }
        return;
    }

    float pct = MiniR4.PWR.getBattPercentage();
    if (pct < 0.0f)
    {
        pct = 0.0f;
    }
    else if (pct > 100.0f)
    {
        pct = 100.0f;
    }

    _sample.battVolts        = volts;
    _sample.battPercent      = static_cast<uint8_t>(pct);
    _sample.battReadOk       = true;
    _sample.battRetryPending = false;
    _battAttemptsMade        = 0;
}

void OperatorPanelClass::_bumpSeq()
{
    ++_sample.seq;
}

// -----------------------------------------------------------------------------------------
// PULLED execution — the single in-flight ActionRequest slot
// -----------------------------------------------------------------------------------------

void OperatorPanelClass::_serviceRequestCompletion()
{
    const Console::ActionRequest action = _pendingAction;
    if (action == Console::ActionRequest::NONE)
    {
        return;
    }

    // Consume the slot unconditionally — whether or not the request below turns out to be
    // legitimate, nothing may retry it next tick under a different guise.
    _pendingAction = Console::ActionRequest::NONE;

    switch (action)
    {
        case Console::ActionRequest::CAL_LINE:
        {
            if (!Console::isDispatchable(Console::Item::CAL_LINE, _boot))
            {
                _refuseRequest(action, "CAL_LINE");
                break;
            }
            EventTrace.pushEvent("CAL LINE");
            LineArray.calibrate();
            // calibrate() is void — it handles/reports per-board failures internally (Serial +
            // safe-default rows) and has no boolean outcome to report; the confirmed action
            // itself always "succeeds" from the console's point of view.
            OperatorConsole.notifyActionDone(action, true);
            break;
        }

        case Console::ActionRequest::CAL_MOTOR:
        {
            if (!Console::isDispatchable(Console::Item::CAL_MOTOR, _boot))
            {
                _refuseRequest(action, "CAL_MOTOR");
                break;
            }
            EventTrace.pushEvent("CAL MOTOR");
            // The 70 is deliberate, and is NOT `Config::MOTOR_BIAS_CALIB_POWER` (50, the
            // parameter's default). Motor bias is PWM-dependent, so calibrating at a different
            // power samples a different operating point; the pre-console calibration menu this
            // dispatch replaced called `calibrateMotorBias(70)`. Passing it explicitly keeps
            // this a pure relocation of that behaviour — moving the operating point is a tuning
            // decision, not a side effect of the console rewrite.
            const uint8_t result = Drive.calibrateMotorBias(int16_t(70));
            OperatorConsole.notifyActionDone(action, result == 0x00);
            break;
        }

        case Console::ActionRequest::GYRO_ZERO:
        {
            if (!Console::isDispatchable(Console::Item::GYRO_ZERO, _boot))
            {
                _refuseRequest(action, "GYRO_ZERO");
                break;
            }
            EventTrace.pushEvent("GYRO ZERO");
            YawSource.zeroHardware();
            OperatorConsole.notifyActionDone(action, true);
            break;
        }

        case Console::ActionRequest::READ_MOSAIC:
        {
            if (!_boot.vision)
            {
                // AC25 — SELF TEST must never emit this against a subsystem that failed to
                // begin(); this is the last line of defence, not the primary gate.
                _refuseRequest(action, "READ_MOSAIC");
                break;
            }
            _executeReadMosaic();
            OperatorConsole.notifyReadDone(action);
            break;
        }

        case Console::ActionRequest::READ_LINE_BOARDS:
        {
            if (!_boot.lineArray)
            {
                _refuseRequest(action, "READ_LINE_BOARDS");
                break;
            }
            _executeReadLineBoards();
            OperatorConsole.notifyReadDone(action);
            break;
        }

        case Console::ActionRequest::CLEAR_VISION_FAULT:
        {
            // Not item-dispatch-gated (it is the console's own exit-cleanup action, not a menu
            // entry) and Vision::clearFault() is a pure state reset with no hardware I/O and no
            // failure mode, so no defensive veto is needed here.
            Vision.clearFault();
            OperatorConsole.notifyActionDone(action, true);
            break;
        }

        case Console::ActionRequest::NONE:
            // Unreachable — the early return at the top of this method already filters NONE.
            // Kept as its own case rather than folded into `default` below.
            break;

        default:
            // Reachable only if a future enumerator is added to Console::ActionRequest without a
            // matching case above — exactly the change that would otherwise reintroduce the
            // no-deadline WAIT_RELEASE wedge _refuseRequest()'s docs warn about. Refuse it through
            // the same reported path as every other rejected request rather than a silent break.
            _refuseRequest(action, "UNKNOWN");
            break;
    }
}

void OperatorPanelClass::_executeReadMosaic()
{
    const bool ok = Vision.readMosaic();

    // Vision.getColor()/getFault() retain the prior latch on a failed read (ADR-037), so it is
    // always safe to re-copy every cell regardless of `ok` — a failure simply re-latches the
    // same values the sample already held.
    for (uint8_t c = 0; c < Config::VISION_CELL_COUNT; ++c)
    {
        _sample.visionCell[c] = static_cast<uint8_t>(Vision.getColor(c));
    }
    _sample.visionFaultKind = ok ? static_cast<uint8_t>(VisionClass::Fault::NONE)
                                 : static_cast<uint8_t>(Vision.getFault().kind);

    _bumpSeq();
}

void OperatorPanelClass::_executeReadLineBoards()
{
    _sample.lineBoardCount = Config::LINE_BOARD_COUNT;

    for (uint8_t b = 0; b < 2; ++b)
    {
        if (b >= Config::LINE_BOARD_COUNT)
        {
            _sample.lineBoardPresent[b] = false;
            _sample.lineBoardOk[b]      = false;
            continue;
        }

        // probeBoard() is THE liveness verdict — an address ACK, never a value judgement.
        _sample.lineBoardPresent[b] = LineArray.probeBoard(b);

        // readBoard() is authoritative for raw-row DISPLAY validity only; its `true` return does
        // NOT prove the board answered (see WROLineArray.h). On failure it leaves `raw`
        // untouched, so lineRaw[b] keeps its last-known-good contents rather than zeros.
        uint8_t raw[8];
        if (LineArray.readBoard(b, raw))
        {
            _sample.lineBoardOk[b] = true;
            for (uint8_t c = 0; c < 8; ++c)
            {
                _sample.lineRaw[b][c] = raw[c];
            }
        }
        else
        {
            _sample.lineBoardOk[b] = false;
        }
    }

    _bumpSeq();
}

void OperatorPanelClass::_refuseRequest(Console::ActionRequest action, const char* which)
{
    Serial.print(F("[OperatorPanel] refusing non-dispatchable ActionRequest: "));
    Serial.println(which);

    // Also push to the 4-slot OLED/Serial-heartbeat event ring — the same surface the "ST PASS" /
    // "ST FAIL" pushes use. Without this, a refusal still drives the red LEDs and the long FAIL
    // tone (below) but leaves no trace entry behind it, which reads as a genuine calibration
    // failure whose event got dropped. `which` doesn't fit here (event ring lines are 20 bytes
    // including NUL, shared across every request kind); the Serial line above already carries it.
    EventTrace.pushEvent("REFUSED");

    // Refuse, but never in silence. For the three side-effect actions the core is already sitting
    // in State::WAIT_RELEASE with its pending-action flag set, and that wait has no deadline and
    // no button escape — it releases only when the adapter reports back. Dropping the request
    // would wedge the console there for the rest of the power cycle, mission unarmable behind it.
    // Report a FAILED completion instead: the wait unwinds and a Signal::FAIL surfaces on the
    // LEDs/buzzer where the operator can actually see the refusal.
    //
    // The two reads are reported too — but POISON FIRST, THEN ACK. A bare `notifyReadDone()`
    // would be fail-OPEN: its documented precondition is that the adapter has already refreshed
    // the PULLED fields and bumped `seq`, and the SELF TEST freshness gate stops waiting the
    // instant the ack lands (it does NOT run out its timeout). Acking without a refresh therefore
    // hands the judge the PREVIOUS sample, which at boot is the all-zero one — where
    // `visionFaultKind == NONE` is a VISION *PASS* and `lineBoardCount == 0` makes the LINE
    // verdict vacuously true. Writing PULLED values no healthy read can produce makes the refusal
    // fail closed on the very next judge, like every other path in this sequencer.
    if (action == Console::ActionRequest::READ_MOSAIC)
    {
        _sample.visionFaultKind = kVisionFaultRefused;
        _bumpSeq();
        OperatorConsole.notifyReadDone(action);
    }
    else if (action == Console::ActionRequest::READ_LINE_BOARDS)
    {
        // `judgeLineBoards()` iterates `lineBoardCount` boards and FAILs on the first that did
        // not answer, so the count must be the REAL configured one — a zeroed count would leave
        // it nothing to inspect and PASS vacuously. `lineBoardOk` follows `lineBoardPresent`
        // down so LINE VIEW cannot keep presenting the stale channel values as valid.
        _sample.lineBoardCount = Config::LINE_BOARD_COUNT;
        for (uint8_t b = 0; b < 2; ++b)
        {
            _sample.lineBoardPresent[b] = false;
            _sample.lineBoardOk[b]      = false;
        }
        _bumpSeq();
        OperatorConsole.notifyReadDone(action);
    }
    else
    {
        OperatorConsole.notifyActionDone(action, false);
    }
}

// -----------------------------------------------------------------------------------------
// Rendering
// -----------------------------------------------------------------------------------------

void OperatorPanelClass::_renderFrame(const Console::ConsoleFrame& frame, uint32_t now)
{
    if (!frame.dirty)
    {
        return;
    }
    if ((uint32_t) (now - _lastRenderMs) < Config::UI_REPAINT_INTERVAL_MS)
    {
        // Suppressed this tick. `dirty` is level-triggered on the core side, so it stays raised
        // and this same repaint is retried on a later tick — nothing is lost.
        return;
    }

    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setTextSize(1);
    MiniR4.OLED.setTextColor(WHITE);
    for (uint8_t r = 0; r < Console::FRAME_ROWS; ++r)
    {
        MiniR4.OLED.setCursor(0, r * EventTraceClass::LINE_HEIGHT_PX);
        MiniR4.OLED.println(frame.rows[r]);
    }
    MiniR4.OLED.display();

    _lastRenderMs = now;
    OperatorConsole.notifyFrameRendered();
}

// -----------------------------------------------------------------------------------------
// Verdict signal sequencer
// -----------------------------------------------------------------------------------------

void OperatorPanelClass::_serviceSignal(Console::Signal signal, uint32_t now)
{
    if (signal == Console::Signal::NONE)
    {
        _signalActive = false;
        return;
    }

    if (!_signalActive || _signalKind != signal)
    {
        // A new verdict just arrived (signal is level-triggered, so this fires exactly once per
        // verdict) — (re)start the pattern: set the LEDs once and, for FAIL, fire the single long
        // tone once. The PASS pattern's chirps are scheduled below on every tick instead, since
        // there are several of them spread over time.
        _signalActive      = true;
        _signalKind        = signal;
        _signalStartMs     = now;
        _signalChirpsFired = 0;

        // Console::Signal is a GENERIC verdict channel: notifyActionDone() raises it for ANY
        // confirmed side-effect action, so GYRO ZERO / CAL LINE / CAL MOTOR land here too. The
        // LED + buzzer drive below is unconditional (correct for every verdict), but the "ST *"
        // EventTrace pushes are gated on the verdict actually coming from the SELF TEST
        // sequencer — otherwise a successful motor-bias calibration would read back as a passed
        // self test on the OLED trace and the Serial heartbeat, which is exactly the surface the
        // operator reads when something has gone wrong at a venue.
        const bool fromSelfTest = (OperatorConsole.state() == Console::State::SELF_TEST);

        if (signal == Console::Signal::PASS)
        {
            MiniR4.LED.setColor(1, 0, 255, 0);
            MiniR4.LED.setColor(2, 0, 255, 0);
            if (fromSelfTest)
            {
                EventTrace.pushEvent("ST PASS");
            }
        }
        else
        {
            MiniR4.LED.setColor(1, 255, 0, 0);
            MiniR4.LED.setColor(2, 255, 0, 0);
            MiniR4.Buzzer.Tone(Config::BOOT_FAIL_BEEP_HZ, Config::BOOT_FAIL_BEEP_MS);
            if (fromSelfTest)
            {
                EventTrace.pushEvent("ST FAIL");
            }
        }
    }

    const uint32_t elapsed = now - _signalStartMs;

    if (signal == Console::Signal::PASS)
    {
        constexpr uint32_t kChirpPeriodMs = Config::BOOT_OK_BEEP_MS + Config::BOOT_OK_BEEP_GAP_MS;

        while (_signalChirpsFired < Config::BOOT_OK_BEEPS &&
               elapsed >= static_cast<uint32_t>(_signalChirpsFired) * kChirpPeriodMs)
        {
            MiniR4.Buzzer.Tone(Config::BOOT_OK_BEEP_HZ, Config::BOOT_OK_BEEP_MS);
            ++_signalChirpsFired;
        }

        const uint32_t totalMs = static_cast<uint32_t>(Config::BOOT_OK_BEEPS) * kChirpPeriodMs;
        if (_signalChirpsFired >= Config::BOOT_OK_BEEPS && elapsed >= totalMs)
        {
            OperatorConsole.notifySignalDone();
            _signalActive = false;
        }
    }
    else    // Signal::FAIL
    {
        if (elapsed >= Config::BOOT_FAIL_BEEP_MS)
        {
            OperatorConsole.notifySignalDone();
            _signalActive = false;
        }
    }
}

// -----------------------------------------------------------------------------------------
// EventTrace state-transition tracing
// -----------------------------------------------------------------------------------------

void OperatorPanelClass::_traceStateTransition(Console::State before, Console::State after)
{
    if (before != Console::State::SELF_TEST && after == Console::State::SELF_TEST)
    {
        EventTrace.pushEvent("SELF TEST");
    }
    if (after == Console::State::CLOSED && before != Console::State::CLOSED)
    {
        EventTrace.pushEvent("DIAG EXIT");
    }
}
