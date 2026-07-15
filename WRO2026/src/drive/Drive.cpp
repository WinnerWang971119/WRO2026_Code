/**
 * @file Drive.cpp
 * @brief Implementation of `DriveClass` — the movement-primitive singleton.
 * @author WRO 2026
 *
 * @note Build-time invariant: the wheel diameter must be positive — otherwise
 *       `cmToEncDeg` (WROMath.h) would divide by zero or invert the travel direction silently.
 *
 * @note The verb entry points (`driveDist`, `driveTime`, `turnDeg`, `turnToYaw`) are
 *       defined inline in `Drive.h` because they construct fluent builders that themselves
 *       live in the header. The heavy `_execXxx` dispatch methods live here.
 */

#include "Drive.h"

#include "TurnSignPolicy.h"    // Turn-sign + counter-rotation wheel math (host-tested; ADR-045).
#include "TShapeProfilePolicy.h"    // Distance-domain T-shape ramp/triangle/half-cap + speed-at
                                    // math (host-tested + golden-pinned; issue #130).
#include "../nav/YawSource.h"         // Single owner of corrected (app CCW-positive) IMU yaw I/O (ADR-045).

#include "LineFollower.h"    // Race-guard: _dispatchNext() auto-disables LineFollower so
                              // its tick loop does not write setSpeed concurrently with a
                              // queued bounded/profiled move. LineFollower.h does NOT pull
                              // Drive.h transitively, so the include is non-circular.

#include "../perception/WROLineArray.h"    // ADR-027: _updateStopAtLine() reads `LineArray.getJunctionType()`
                              // directly (NOT `Intersection::isAtIntersection()`, which is
                              // read-and-clear and owned by PathTracker's nav consumer). The
                              // include lives in the .cpp ONLY — Drive.h needs none of
                              // WROLineArray's definition, so including it here keeps
                              // MatrixColor out of every TU that pulls in Drive.h.

static_assert(Config::WHEEL_DIAM_MM > 0.0f, "WHEEL_DIAM_MM must be positive");

/// @brief Single global instance.
DriveClass Drive;

// -----------------------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------------------

bool DriveClass::begin()
{
    // Reverse-flag and PPR/RPM apply to the per-motor objects directly. The MatrixMiniR4
    // facade does not expose runtime indexing into MiniR4.M<N>, so we hardcode M2/M3 to match
    // the WRO chassis convention (Config::MOTOR_LEFT_ID == 2, Config::MOTOR_RIGHT_ID == 3).
    MiniR4.M2.setReverse(Config::MOTOR_LEFT_REVERSE);
    MiniR4.M3.setReverse(Config::MOTOR_RIGHT_REVERSE);
    MiniR4.M2.setPPR_RPM(Config::MOTOR_PPR, Config::MOTOR_RPM);
    MiniR4.M3.setPPR_RPM(Config::MOTOR_PPR, Config::MOTOR_RPM);

    const uint8_t driveBeginRc = MiniR4.DriveDC.begin(Config::MOTOR_LEFT_ID,
                                                     Config::MOTOR_RIGHT_ID,
                                                     Config::MOTOR_LEFT_REVERSE,
                                                     Config::MOTOR_RIGHT_REVERSE);
    if (driveBeginRc != 0x00)
    {
        return false;
    }

    // Best-effort PID push — the library already retries once internally on failure.
    MiniR4.DriveDC.setMoveSyncPID(Config::MOVESYNC_KP,
                                  Config::MOVESYNC_KI,
                                  Config::MOVESYNC_KD);
    MiniR4.DriveDC.setMoveGyroPID(Config::MOVEGYRO_KP,
                                  Config::MOVEGYRO_KI,
                                  Config::MOVEGYRO_KD);
    MiniR4.DriveDC.setTurnGyroPID(Config::TURNGYRO_KP,
                                  Config::TURNGYRO_KI,
                                  Config::TURNGYRO_KD);

    _heldYaw  = 0;
    _moveDone = true;
    _turnDone = true;

    // Apply the Config-defined motor-bias scalars (Strategy B; see DECISIONS.md). Defaults
    // to (1.0, 1.0) — identity — until `Drive.calibrateMotorBias()` produces values that
    // Diego hand-edits into Config::MOTOR_BIAS_L / MOTOR_BIAS_R.
    setMotorBias(Config::MOTOR_BIAS_L, Config::MOTOR_BIAS_R);

    return true;
}

void DriveClass::setTickCallback(void (*cb)())
{
    _tickCallback                = cb;
    _tickCallbackWarningEmitted  = false;    // reset so re-register clears the warned state
}

void DriveClass::update()
{
    // -------------------------------------------------------------------------------------
    // Stage 1 — drain any in-flight bounded task. When a profile is in flight, the profile
    // owns `_moveDone` — the lower-MCU task-done bit refers to a *prior* bounded MMLower
    // task and would falsely flip _moveDone to true mid-cruise (W1 fix). Skip the drain in
    // that case; _updateProfile will set _moveDone when the profile itself completes.
    //
    // Same guard applies when the custom TurnGyro profile is in flight: `_updateTurnProfile`
    // owns `_turnDone` (and the collapsed `_moveDone`) and the lower-MCU task-done bit would
    // refer to a *prior* bounded MMLower task, falsely flipping _turnDone mid-turn.
    //
    // And again for an in-flight stopAtLine seek: `_updateStopAtLine` owns `_moveDone` for the
    // duration of the continuous, event-terminated seek. The seek commands MoveSync/MoveGyro/Move
    // continuously (which do NOT raise the lower-MCU task-done bit), but a *prior* bounded task's
    // stale done bit could still flip `_moveDone` mid-seek — skip the drain while seeking (W1).
    //
    // Same guard for an in-flight wallStop push: `_updateWallStop` owns `_moveDone` for the
    // duration of the continuous, stall-terminated push (which also commands MoveSync/MoveGyro/Move
    // continuously) — skip the drain while pushing (W1).
    // -------------------------------------------------------------------------------------
    if (_profileState == ProfileState::IDLE
        && _turnProfileState == TurnProfileState::IDLE
        && _stopState == StopAtLineState::IDLE
        && _wallState == WallStopState::IDLE
        && (!_moveDone || !_turnDone))
    {
        // The lower MCU reports a single task-done bit, so move-done and turn-done collapse
        // into one signal here. Application code MUST NOT issue a bounded move and a turn
        // concurrently — they would race for this flag.
        if (MiniR4.DriveDC.isPrevTaskDone())
        {
            _moveDone = true;
            _turnDone = true;
        }
    }

    // -------------------------------------------------------------------------------------
    // Stage 2 — advance the profile state machine. Idempotent when IDLE, so the call is
    // cheap on every tick. Order matters: profile drain MUST run before stage 3 so that a
    // just-completed profile sets `_moveDone` here, allowing `_dispatchNext()` to fire on
    // the same tick (no inter-segment gap on chained queue moves).
    // -------------------------------------------------------------------------------------
    _updateProfile();
    _updateTurnProfile();    // NEW — cooperative tick parallel to _updateProfile, owns
                              // the custom CRUISE → SLOW → BRAKE TurnGyro state machine.
    _updateStopAtLine();     // ADR-027 — cooperative tick parallel to the two profiles, owns
                              // the continuous, event-terminated SEEK → ARMED → OVERSHOOT → DONE
                              // drive-until-line seek. Reads LineArray.getJunctionType() directly.
    _updateWallStop();       // AC8 — cooperative tick parallel to the seek, owns the continuous,
                              // stall-terminated PUSH → BACK_OFF → DONE wall re-zero push. REUSES
                              // the stop-at-line no-progress watchdog with the stall meaning
                              // inverted to success.

    // -------------------------------------------------------------------------------------
    // Stage 3 — if the chassis is fully idle and a queued command is waiting, dispatch it.
    // -------------------------------------------------------------------------------------
    if (_isIdle() && _qCount > 0)
    {
        _dispatchNext();
    }
}

// -----------------------------------------------------------------------------------------
// Continuous primitives
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::setSpeed(int16_t L, int16_t R, int16_t floorMag)
{
    _applyBias(L, R);
    L = deadbandSnap(L, floorMag);
    R = deadbandSnap(R, floorMag);
    return MiniR4.DriveDC.Move(L, R);
}

uint8_t DriveClass::setSpeedSync(int16_t L, int16_t R)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md. Snap each wheel through the
    // shared PWM cap/deadband floor so this continuous path clamps uniformly with the RAW path
    // and the profile emitter (finding #14) — the library documents the power range as -100..100.
    return MiniR4.DriveDC.MoveSync(deadbandSnap(L), deadbandSnap(R));
}

uint8_t DriveClass::forwardGyro(int16_t power)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md. Snap the power through the
    // shared PWM cap/deadband floor for uniform clamping (finding #14).
    // GYRO precondition (ADR-045): _heldYaw is an app-frame (CCW-positive) heading; toHwHeading
    // converts it to the STM32 raw frame. NEVER pass a raw heading here — it would double-correct.
    return MiniR4.DriveDC.MoveGyro(deadbandSnap(power), YawSource.toHwHeading(_heldYaw));
}

uint8_t DriveClass::stop(bool brake)
{
    MiniR4.DriveDC.Move(0, 0);
    const uint8_t brakeRc = MiniR4.DriveDC.brake(brake);

    _moveDone         = true;
    _turnDone         = true;
    _profileState     = ProfileState::IDLE;        // also abort any in-flight profile
    _turnProfileState = TurnProfileState::IDLE;    // also abort any in-flight turn profile

    // Abort any in-flight stopAtLine seek too (ADR-027) — restore the front board so LineFollower
    // and any later read sample the forward-facing sensor. The shared teardown re-issues the same
    // `Move(0, 0)` + `brake(brake)` already done above with this `brake`, which is harmless.
    if (_stopState != StopAtLineState::IDLE)
    {
        _endStopAtLine(brake);
    }

    // Abort any in-flight wallStop push too (AC8). The shared teardown re-issues the same
    // `Move(0, 0)` + `brake(brake)` already done above with this `brake`, which is harmless.
    if (_wallState != WallStopState::IDLE)
    {
        _endWallStop(brake);
    }

    // Abort semantics — pending queued work is dropped. Without this, a `Drive.stop()` call
    // intended to halt the chassis would be silently undone on the next `update()` tick when
    // the queue dispatched the next command.
    _clearQueue();

    return brakeRc;
}

// -----------------------------------------------------------------------------------------
// Distance-bounded executors (called from DriveDistBuilder::~DriveDistBuilder)
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::_execDriveDistRaw(int16_t power, float cm, bool brake, bool blocking)
{
    // Apply motor-bias scalars by treating the single `power` as (L=power, R=power) and
    // letting `_applyBias` produce per-wheel PWM.
    int16_t powerL = power;
    int16_t powerR = power;
    _applyBias(powerL, powerR);

    const uint16_t encDeg = cmToEncDeg(cm);
    // The library convention: the last argument is `async`. Blocking == sync == async=false.
    const uint8_t rc = MiniR4.DriveDC.MoveDegs(powerL, powerR, encDeg, brake, !blocking);
    // W3 guard: only set `_moveDone = false` when the lower MCU actually accepted the task.
    // Without this, a failed dispatch leaves the flag stuck at `false` forever and
    // `_drainBlocking()` infinite-loops waiting for a task that never started.
    if (!blocking && rc == 0x00)
    {
        _moveDone = false;
    }
    return rc;
}

uint8_t DriveClass::_execDriveDistSync(int16_t power, float cm, bool brake, bool blocking)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md. Snap the caller's power through
    // the shared PWM cap/deadband floor so this executor clamps uniformly with the RAW/profile
    // paths (finding #14) — the library documents the power range as -100..100.
    const int16_t  snapped = deadbandSnap(power);
    const uint16_t encDeg  = cmToEncDeg(cm);
    const uint8_t  rc =
        MiniR4.DriveDC.MoveSyncDegs(snapped, snapped, encDeg, brake, !blocking);
    if (!blocking && rc == 0x00)    // W3 guard
    {
        _moveDone = false;
    }
    return rc;
}

uint8_t DriveClass::_execDriveDistGyro(int16_t power,
                                       float   cm,
                                       int16_t yaw,
                                       bool    brake,
                                       bool    blocking)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md. Snap the caller's power through
    // the shared PWM cap/deadband floor for uniform clamping with the RAW/profile paths (finding
    // #14). The yaw target is a heading, NOT a PWM — leave it unsnapped.
    const uint16_t encDeg = cmToEncDeg(cm);
    // GYRO precondition (ADR-045): `yaw` is an app-frame heading; toHwHeading converts to raw.
    const uint8_t  rc =
        MiniR4.DriveDC.MoveGyroDegs(deadbandSnap(power), YawSource.toHwHeading(yaw), encDeg, brake, !blocking);
    if (!blocking && rc == 0x00)    // W3 guard
    {
        _moveDone = false;
    }
    return rc;
}

// -----------------------------------------------------------------------------------------
// Time-bounded executors (called from DriveTimeBuilder::~DriveTimeBuilder)
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::_execDriveTimeRaw(int16_t power, float sec, bool brake, bool blocking)
{
    // Apply motor-bias scalars by treating the single `power` as (L=power, R=power) and
    // letting `_applyBias` produce per-wheel PWM.
    int16_t powerL = power;
    int16_t powerR = power;
    _applyBias(powerL, powerR);

    const uint8_t rc = MiniR4.DriveDC.MoveTime(powerL, powerR, sec, brake, !blocking);
    if (!blocking && rc == 0x00)    // W3 guard — see _execDriveDistRaw
    {
        _moveDone = false;
    }
    return rc;
}

uint8_t DriveClass::_execDriveTimeSync(int16_t power, float sec, bool brake, bool blocking)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md. Snap the caller's power through
    // the shared PWM cap/deadband floor for uniform clamping with the RAW/profile paths (finding
    // #14) — the library documents the power range as -100..100.
    const int16_t snapped = deadbandSnap(power);
    const uint8_t rc = MiniR4.DriveDC.MoveSyncTime(snapped, snapped, sec, brake, !blocking);
    if (!blocking && rc == 0x00)    // W3 guard
    {
        _moveDone = false;
    }
    return rc;
}

uint8_t DriveClass::_execDriveTimeGyro(int16_t power,
                                       float   sec,
                                       int16_t yaw,
                                       bool    brake,
                                       bool    blocking)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md. Snap the caller's power through
    // the shared PWM cap/deadband floor for uniform clamping with the RAW/profile paths (finding
    // #14). The yaw target is a heading, NOT a PWM — leave it unsnapped.
    // GYRO precondition (ADR-045): `yaw` is an app-frame heading; toHwHeading converts to raw.
    const uint8_t rc = MiniR4.DriveDC.MoveGyroTime(deadbandSnap(power), YawSource.toHwHeading(yaw), sec, brake, !blocking);
    if (!blocking && rc == 0x00)    // W3 guard
    {
        _moveDone = false;
    }
    return rc;
}

// -----------------------------------------------------------------------------------------
// Turn executors (called from TurnDegBuilder / TurnToYawBuilder destructors)
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::_execTurnDeg(int16_t deg,
                                 int16_t power,
                                 uint8_t mode,
                                 bool    brake,
                                 bool    blocking)
{
    // No _applyBias here — wheel-pattern dispatch happens in _updateTurnProfile().
    //
    // `power` is honored as the cruise-PWM magnitude override: callers can chain
    // `Drive.turnDeg(deg).setPower(80)` for a faster turn. Slow-zone PWM stays at
    // Config::TURN_SLOW_SPEED_PWM (the final-approach must stay gentle enough to
    // stop within TURN_TOLERANCE_DEG). Non-positive overrides fall back to the
    // Config default to keep `_turnDeltaSign` as the sole source of rotation sign.
    //
    // Mode encoding (NEW — repurposed from MiniR4.DriveDC.TurnGyro's 0/1 bits):
    //   0 = COUNTER_ROT (both wheels spin opposite directions)
    //   1 = PIVOT_LEFT  (right wheel only — left wheel acts as pivot)
    //   2 = PIVOT_RIGHT (left wheel only — right wheel acts as pivot)

    // Read the corrected (app CCW-positive) IMU yaw through YawSource (ADR-045). Best-effort
    // fallback 0.0 preserves the prior dispatch-time behavior of acting on the vendor's 0.0.
    const int16_t cur    = (int16_t) YawSource.readYawDegOr(0.0);
    const int16_t target = _normalizeYaw((int32_t) cur + (int32_t) deg);

    // Capture the initial signed delta normalized into (-180, +180] so the BRAKE-edge
    // tick cannot oscillate the spin direction when the IMU briefly straddles target.
    const int32_t deltaNorm = normalizeTurnDelta((int32_t) target - (int32_t) cur);

    // Short-circuit on no-op turns. When the normalized delta is already within
    // the heading tolerance, arming the profile would issue one spurious CRUISE
    // tick (sign defaults to -1 in the ternary below) before BRAKE catches up.
    // Just mark the turn done and keep _heldYaw at the target so subsequent
    // yaw-locked moves stay aligned.
    if (abs(deltaNorm) <= Config::TURN_TOLERANCE_DEG)
    {
        _heldYaw          = target;
        _turnTargetYaw    = target;
        _turnProfileState = TurnProfileState::IDLE;
        _turnDone         = true;
        _moveDone         = true;    // single-task-done bit collapse (matches BRAKE branch).
        (void) power;
        (void) mode;
        (void) brake;
        (void) blocking;
        return 0x00;
    }

    const int8_t sign = turnSign(deltaNorm);

    _turnTargetYaw          = target;
    _turnDeltaSign          = sign;
    _turnMotorMode          = mode;
    _turnCruisePwm          = (power > 0) ? power : Config::TURN_CRUISE_PWM;
    _turnProfileBrakeOnDone = brake;
    _turnProfileState       = TurnProfileState::CRUISE;
    _turnDone               = false;
    _heldYaw                = target;
    _turnLastGoodReadMs     = millis();    // seed the persistent-IMU-fault watchdog at dispatch.

    // Blocking path: TurnDegBuilder's destructor already calls _drainBlocking(); nothing
    // extra here. We always return 0x00 — the new state machine cannot fail at dispatch
    // (no lower-MCU round trip), so there is no rc to surface.
    (void) blocking;
    return 0x00;
}

uint8_t DriveClass::_execTurnToYaw(int16_t yaw,
                                   int16_t power,
                                   uint8_t mode,
                                   bool    brake,
                                   bool    blocking)
{
    // No _applyBias here — wheel-pattern dispatch happens in _updateTurnProfile().
    //
    // `power` is honored as the cruise-PWM magnitude override; see _execTurnDeg
    // for the full rationale. Slow-zone PWM stays at Config::TURN_SLOW_SPEED_PWM.
    // Non-positive overrides fall back to the Config default.
    //
    // Mode encoding identical to _execTurnDeg (0=COUNTER_ROT, 1=PIVOT_LEFT, 2=PIVOT_RIGHT).

    // Read the corrected (app CCW-positive) IMU yaw through YawSource (ADR-045); best-effort 0.0.
    const int16_t cur    = (int16_t) YawSource.readYawDegOr(0.0);
    const int16_t target = _normalizeYaw((int32_t) yaw);

    const int32_t deltaNorm = normalizeTurnDelta((int32_t) target - (int32_t) cur);

    // Short-circuit on no-op turns. When the normalized delta is already within
    // the heading tolerance, arming the profile would issue one spurious CRUISE
    // tick (sign defaults to -1 in the ternary below) before BRAKE catches up.
    // Just mark the turn done and keep _heldYaw at the target so subsequent
    // yaw-locked moves stay aligned.
    if (abs(deltaNorm) <= Config::TURN_TOLERANCE_DEG)
    {
        _heldYaw          = target;
        _turnTargetYaw    = target;
        _turnProfileState = TurnProfileState::IDLE;
        _turnDone         = true;
        _moveDone         = true;    // single-task-done bit collapse (matches BRAKE branch).
        (void) power;
        (void) mode;
        (void) brake;
        (void) blocking;
        return 0x00;
    }

    const int8_t sign = turnSign(deltaNorm);

    _turnTargetYaw          = target;
    _turnDeltaSign          = sign;
    _turnMotorMode          = mode;
    _turnCruisePwm          = (power > 0) ? power : Config::TURN_CRUISE_PWM;
    _turnProfileBrakeOnDone = brake;
    _turnProfileState       = TurnProfileState::CRUISE;
    _turnDone               = false;
    _heldYaw                = target;
    _turnLastGoodReadMs     = millis();    // seed the persistent-IMU-fault watchdog at dispatch.

    (void) blocking;
    return 0x00;
}

// -----------------------------------------------------------------------------------------
// State accessors
// -----------------------------------------------------------------------------------------

bool DriveClass::isMoveDone() const
{
    return _moveDone;
}

bool DriveClass::isTurnDone() const
{
    return _turnDone;
}

bool DriveClass::isIdle() const
{
    return _isIdle() && _qCount == 0;
}

bool DriveClass::isLineFound() const
{
    return _isLineFound;
}

float DriveClass::getStopOvershoot() const
{
    return _stopOvershoot;
}

bool DriveClass::wallStopTimedOut() const
{
    return _wallStopTimedOut;
}

void DriveClass::waitUntilIdle()
{
    _drainBlocking();
}

int32_t DriveClass::getEncL() const
{
    // M2/M3 hardcoded to match Config::MOTOR_LEFT_ID/RIGHT_ID == 2/3. Editing those config
    // constants requires changing the motor objects referenced here too.
    return MiniR4.M2.getDegrees();
}

int32_t DriveClass::getEncR() const
{
    return MiniR4.M3.getDegrees();
}

void DriveClass::resetEnc()
{
    MiniR4.DriveDC.resetCounter();
}

double DriveClass::getYaw() const
{
    // Corrected app-frame yaw via the single owner (ADR-045); best-effort 0.0 on read failure.
    return YawSource.readYawDegOr(0.0);
}

void DriveClass::holdYaw()
{
    // Latch the corrected app-frame yaw (ADR-045); best-effort 0.0 on read failure.
    _heldYaw = (int16_t) YawSource.readYawDegOr(0.0);
}

// -----------------------------------------------------------------------------------------
// Motor-bias calibration (Strategy B — see DECISIONS.md)
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::calibrateMotorBias(int16_t power, uint32_t durationMs)
{
    // Reset both encoders to zero so the post-spin readings ARE the per-wheel deltas.
    MiniR4.DriveDC.resetCounter();

    // Save the existing bias and force identity for the duration of the spin so the
    // measurement reflects raw motor behaviour, not pre-existing correction.
    const float savedBiasL = _biasL;
    const float savedBiasR = _biasR;
    _biasL                 = 1.0f;
    _biasR                 = 1.0f;

    // Drive the lower-MCU directly with raw `Move` to skip the Drive.setSpeed wrapper.
    // We are already in identity-bias state, so the result would be the same either way,
    // but going direct removes one layer of indirection that could mask a future bug.
    const uint8_t spinRc = MiniR4.DriveDC.Move(power, power);
    if (spinRc != 0x00)
    {
        // Restore bias and bail — we never spun, so encoder readings would be meaningless.
        _biasL = savedBiasL;
        _biasR = savedBiasR;
        Serial.println(F("[calibrateMotorBias] ERROR: Move() failed to start the spin."));
        return spinRc;
    }

    // Explicit blocking delay — the Doxygen on this method calls out that this is the ONE
    // place in the codebase where a `delay()` is acceptable (manual one-shot, off the
    // cooperative tick).
    delay(durationMs);

    // Active-brake stop. Reuse `stop(true)` so the move/turn done flags get cleared too.
    stop(true);

    // Read final encoder degrees on each side (post-direction-correction by setReverse).
    const int32_t  encL = MiniR4.M2.getDegrees();
    const int32_t  encR = MiniR4.M3.getDegrees();
    const uint32_t aL   = (uint32_t) abs(encL);
    const uint32_t aR   = (uint32_t) abs(encR);

    // Sanity check: zero on either side means the motor never spun (stuck wheel, broken
    // encoder, or we tried to calibrate at too low a power). Don't divide by zero, don't
    // mutate bias.
    if (aL == 0 || aR == 0)
    {
        _biasL = savedBiasL;
        _biasR = savedBiasR;
        Serial.print(F("[calibrateMotorBias] ERROR: zero encoder delta. encL="));
        Serial.print(encL);
        Serial.print(F(" encR="));
        Serial.println(encR);
        return 0x01;
    }

    // Direction sanity check: after `setReverse` is honoured, both encoders should have the
    // same sign when both motors are commanded with the same positive PWM. Opposite signs
    // mean a wiring error or a wrong `MOTOR_*_REVERSE` flag in Config.h — silently absing
    // would hide the bug and yield a "successful" calibration that drives the chassis the
    // wrong way.
    if ((encL > 0) != (encR > 0))
    {
        _biasL = savedBiasL;
        _biasR = savedBiasR;
        Serial.print(F("[calibrateMotorBias] ERROR: wheels spun opposite directions. encL="));
        Serial.print(encL);
        Serial.print(F(" encR="));
        Serial.println(encR);
        Serial.println(F("Check MOTOR_LEFT_REVERSE / MOTOR_RIGHT_REVERSE in Config.h."));
        return 0x02;
    }

    // The target is the mean of the two observed rotations — both wheels should reach this
    // common value when the bias scalars are applied next time.
    const float target   = ((float) aL + (float) aR) / 2.0f;
    const float newBiasL = target / (float) aL;
    const float newBiasR = target / (float) aR;

    // Restore the original bias — calibration NEVER auto-applies. Diego copies the printed
    // values into Config.h and re-flashes; persistence is via source-edit only.
    _biasL = savedBiasL;
    _biasR = savedBiasR;

    Serial.print(F("[calibrateMotorBias] result (power="));
    Serial.print(power);
    Serial.print(F(", dur="));
    Serial.print(durationMs);
    Serial.println(F("ms):"));
    Serial.print(F("  MOTOR_BIAS_L = "));
    Serial.print(newBiasL, 4);
    Serial.println(F("f;"));
    Serial.print(F("  MOTOR_BIAS_R = "));
    Serial.print(newBiasR, 4);
    Serial.println(F("f;"));
    Serial.println(F("Copy these into Config.h, then re-flash."));

    return 0x00;
}

void DriveClass::setMotorBias(float biasL, float biasR)
{
    _biasL = biasL;
    _biasR = biasR;
}

void DriveClass::resetMotorBias()
{
    setMotorBias(1.0f, 1.0f);
}

float DriveClass::getBiasL() const
{
    return _biasL;
}

float DriveClass::getBiasR() const
{
    return _biasR;
}

// -----------------------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------------------

int16_t DriveClass::_normalizeYaw(int32_t deg)
{
    // C++11+ truncating modulo: the result is always in (-360, +360) for any int32 input,
    // so a single `% 360` is sufficient — no further clamping required.
    return (int16_t) (deg % 360);
}

// `_presetToPwm` and `_cmToEncDeg` moved to free functions in WROMath.h once LineFollower
// also needed them. Internal callers below now invoke `presetToPwm(...)` / `cmToEncDeg(...)`
// directly.

void DriveClass::_applyBias(int16_t& L, int16_t& R) const
{
    L = (int16_t) ((float) L * _biasL);
    R = (int16_t) ((float) R * _biasR);
}

// -----------------------------------------------------------------------------------------
// Trapezoidal-profile state machine (`.tShape()` cooperative driver)
// -----------------------------------------------------------------------------------------

void DriveClass::_emitContinuous(DriveMode mode, int16_t mag, int16_t heading, int16_t floorMag)
{
    // Shared continuous-emit path for both `_updateProfile` and `_updateStopAtLine`. Reads NO
    // instance state — every input is a parameter — so the two callers cannot cross-contaminate
    // through it. RAW applies the motor bias + deadband snap (open loop); SYNC / GYRO do NOT — the
    // STM32 closes the loop and a feedforward scalar would fight its inner PID (see DECISIONS.md).
    switch (mode)
    {
    case DriveMode::RAW:
    {
        int16_t L = mag;
        int16_t R = mag;
        _applyBias(L, R);
        L = deadbandSnap(L, floorMag);
        R = deadbandSnap(R, floorMag);
        MiniR4.DriveDC.Move(L, R);
        break;
    }
    case DriveMode::SYNC:
        // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
        MiniR4.DriveDC.MoveSync(deadbandSnap(mag, floorMag), deadbandSnap(mag, floorMag));
        break;
    case DriveMode::GYRO:
        // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
        // GYRO precondition (ADR-045): `heading` is app-frame (CCW-positive); toHwHeading
        // converts to raw. All three GYRO channels feeding this (_profileHeading / _stopHeading
        // / _wallHeading) carry app-frame values, so this single wrap covers straight-line,
        // stop-at-line, and wall-stop alike.
        MiniR4.DriveDC.MoveGyro(deadbandSnap(mag, floorMag), YawSource.toHwHeading(heading));
        break;
    }
}

void DriveClass::_updateProfile()
{
    if (_profileState == ProfileState::IDLE)
    {
        return;
    }

    // Compute traveled fraction in [0, 1].
    float frac;
    if (_profileIsTime)
    {
        const uint32_t elapsed = millis() - _profileStartMs;
        frac = (_profileTotalMs == 0) ? 1.0f : (float) elapsed / (float) _profileTotalMs;
    }
    else
    {
        // Use the left motor (M2) as the canonical encoder, matching `getEncL()`. We compare
        // unsigned magnitudes everywhere so negative-direction (reverse) moves work without
        // any sign juggling: `_profileTotalEnc` is stored as a positive magnitude, and the
        // encoder delta's absolute value is compared against it.
        const int32_t  now      = MiniR4.M2.getDegrees();
        const int32_t  delta    = now - _profileStartEnc;
        const uint32_t traveled = (uint32_t) abs(delta);
        if (_profileTotalEnc == 0)
        {
            frac = 1.0f;
        }
        else
        {
            frac = (float) traveled / (float) _profileTotalEnc;
        }
    }

    if (frac >= 1.0f)
    {
        // Profile complete — finalise. One tick of latency vs. immediate completion is
        // acceptable per spec.
        _profileState = ProfileState::DONE;
        if (_profileEndSpd == 0)
        {
            // Bounded move ending at rest. We INLINE the equivalent of `stop(_profileBrake)`
            // here rather than calling it: `stop()` calls `_clearQueue()` which would wipe
            // every queued follow-up command — that's fine for a top-level abort but wrong
            // for the queue dispatcher's "this move just completed normally, hand off to the
            // next" path. The canonical pattern
            //
            //     Drive.driveDist(30).tShape();   // default endSpeed=0
            //     Drive.driveDist(20).tShape();   // would be wiped if we called stop()
            //
            // depends on this distinction. W1 (queue review).
            MiniR4.DriveDC.Move(0, 0);
            MiniR4.DriveDC.brake(_profileBrake);
            _moveDone = true;
            _turnDone = true;
        }
        else if (_qCount > 0)
        {
            // Chained move WITH a follow-up queued: caller asked us to leave the chassis at
            // endSpeed (typically cruise) so the next queued move can pick up without a
            // zero-velocity gap. The last setpoint issued by `_profileSpeedAt(near 1.0)` was
            // already at endSpeed — leave it. No motor command, no brake; the next move's first
            // `_dispatchNext` tick overwrites the setpoint.
            _moveDone = true;
            _turnDone = true;
        }
        else
        {
            // Chained move ending at endSpeed but NOTHING is queued (`_qCount == 0`, the same
            // queue-empty predicate `_dispatchNext` uses). The seamless-handoff optimization
            // above assumed "the next queued move overwrites the cruise setpoint" — with an empty
            // queue no such move exists, so latching cruise would leave the wheels driving at the
            // endSpeed setpoint forever while every state accessor reports idle. Fall through to
            // the brake/stop path instead. W1 (queue review) — finding #13.
            MiniR4.DriveDC.Move(0, 0);
            MiniR4.DriveDC.brake(_profileBrake);
            _moveDone = true;
            _turnDone = true;
        }
        _profileState = ProfileState::IDLE;
        return;
    }

    // Compute speed setpoint and issue the appropriate continuous primitive through the shared
    // emitter (the heading is consulted only in GYRO mode).
    const int16_t spd = _profileSpeedAt(frac);
    _emitContinuous(_profileMode, spd, _profileHeading, _decelTailFloorMag(frac));
}

int16_t DriveClass::_profileSpeedAt(float frac) const
{
    // The trapezoid integral matches the requested distance/duration because:
    //   integral = (start + peak) / 2 * accel + peak * (total - accel - decel)
    //                                          + (peak + end) / 2 * decel
    // For the full-trapezoid case peak = cruise; the triangle-fallback branch in
    // `_execProfileMove` reduces peak so the same identity still holds.
    if (_profileIsTime)
    {
        const uint32_t elapsedMs = (uint32_t) (frac * (float) _profileTotalMs);
        if (elapsedMs < _profileAccelMs)
        {
            // Ramp up: linear interp from start → peak.
            const float r =
                _profileAccelMs == 0 ? 1.0f : (float) elapsedMs / (float) _profileAccelMs;
            return (int16_t) (_profileStartSpd
                              + (int16_t) ((_profilePeakSpd - _profileStartSpd) * r));
        }
        else if (_profileTotalMs > _profileDecelMs
                 && elapsedMs >= (_profileTotalMs - _profileDecelMs))
        {
            // Ramp down: linear interp from peak → end.
            const uint32_t into = elapsedMs - (_profileTotalMs - _profileDecelMs);
            const float    r =
                _profileDecelMs == 0 ? 1.0f : (float) into / (float) _profileDecelMs;
            return (int16_t) (_profilePeakSpd + (int16_t) ((_profileEndSpd - _profilePeakSpd) * r));
        }
        return _profilePeakSpd;
    }

    // Distance variant — same shape, units in encoder degrees. The interpolation is delegated
    // to `tShapeSpeedAt` (TShapeProfilePolicy.h), which is unit-agnostic: we feed it the
    // encoder-degree magnitudes cast to float. Bit-identical to the previous inline int32
    // branch — every value here is an exact integer ≤ UINT16_MAX (`cmToEncDeg` returns
    // uint16_t), far below float's 2^24 exact-integer ceiling, so the float compares /
    // subtractions / divides see exactly the operands the int32 code saw.
    const int32_t traveled = (int32_t) (frac * (float) _profileTotalEnc);

    TShapeRamps ramps;
    ramps.accelCm = (float) _profileAccelEnc;
    ramps.decelCm = (float) _profileDecelEnc;
    ramps.peakPwm = _profilePeakSpd;
    return tShapeSpeedAt(
        (float) traveled, (float) _profileTotalEnc, ramps, _profileStartSpd, _profileEndSpd);
}

int16_t DriveClass::_decelTailFloorMag(float frac) const
{
    // Only a stop/near-stop (endSpeed below the static floor) gets the kinetic tail. A move that
    // ends at or above the static floor never dips into the softened band, so keep it static.
    if (abs(_profileEndSpd) >= Config::PWM_MIN_MAGNITUDE)
    {
        return Config::PWM_MIN_MAGNITUDE;
    }

    bool inDecel;
    if (_profileIsTime)
    {
        const uint32_t elapsedMs = (uint32_t) (frac * (float) _profileTotalMs);
        inDecel = (_profileTotalMs > _profileDecelMs)
                  && (elapsedMs >= (_profileTotalMs - _profileDecelMs));
    }
    else
    {
        const int32_t traveled = (int32_t) (frac * (float) _profileTotalEnc);
        inDecel = (traveled >= (_profileTotalEnc - _profileDecelEnc));
    }
    return inDecel ? Config::PWM_MOVING_MIN : Config::PWM_MIN_MAGNITUDE;
}

uint8_t DriveClass::_execProfileMove(DriveMode mode,
                                     bool      isTime,
                                     float     totalCm,
                                     float     totalSec,
                                     int16_t   cruise,
                                     int16_t   startSpeed,
                                     int16_t   endSpeed,
                                     float     accel,
                                     bool      brake,
                                     int16_t   heading)
{
    // Defensive: an invalid acceleration would either divide-by-zero (0) or invert the ramp
    // direction (negative). Surface a Serial warning and bail out — the caller is then free
    // to fall back to a fire-and-forget non-profiled move on its own.
    if (accel <= 0.0f)
    {
        Serial.println(F("[Drive] tShape: invalid accel"));
        _profileState = ProfileState::IDLE;
        return 0x01;
    }

    // Defensive non-negative clamp on the move bound. A negative totalCm/totalSec would later
    // underflow into cmToEncDeg's uint16_t cast (or produce a negative ramp duration), so we
    // floor at zero up-front. WARNING-2 fix.
    if (totalCm < 0.0f)
    {
        totalCm = 0.0f;
    }
    if (totalSec < 0.0f)
    {
        totalSec = 0.0f;
    }

    // Cancel any in-flight profile (silent — caller-discipline issue). We deliberately do NOT
    // stop the motors here: the new move is about to issue its own setpoints from the next
    // _updateProfile tick.
    if (_profileState != ProfileState::IDLE)
    {
        _profileState = ProfileState::IDLE;
    }

    _profileMode     = mode;
    _profileIsTime   = isTime;
    _profileBrake    = brake;
    _profileHeading  = heading;
    _profileCruise   = cruise;
    _profileStartSpd = startSpeed;
    _profileEndSpd   = endSpeed;

    if (isTime)
    {
        // ---------------------------------------------------------------------------------
        // Time variant — convert the cm-based ramp distances into ms via the cruise-speed
        // approximation: rampMs ≈ rampCm / |cruise| * 1000. This is a units-coherent
        // reinterpretation of the same `Config::TSHAPE_*` knobs and is documented in
        // `DriveTimeBuilder::tShape()`.
        // ---------------------------------------------------------------------------------
        const float total = totalSec * 1000.0f;    // ms
        auto        cmToMs = [&](float cm) -> float
        {
            // No-ramp short-circuit MUST come before the cruise==0 fallback so that a chained
            // move (resolveTShapeRampCm returned 0.0f) honours the documented no-ramp invariant
            // — even when cruise itself is 0 (degenerate stand-still call). CRITICAL-1 fix.
            if (cm == 0.0f)
            {
                return 0.0f;
            }
            // Defensive: cruise == 0 means a degenerate "stand still" call — half the total
            // is the safest default ramp duration since the speed table is flat.
            if (cruise == 0)
            {
                return total * 0.5f;
            }
            return cm / (float) abs(cruise) * 1000.0f;
        };

        // Per-side ramp distances (cm) resolved by the shared policy — no-ramp epsilon + the
        // [MIN, MAX] clamps (`resolveTShapeRampCm`, TShapeProfilePolicy.h; returns 0.0f on the
        // no-ramp path for a chained move already at cruise) — then converted to ms.
        float accelMs = cmToMs(resolveTShapeRampCm(cruise,
                                                   startSpeed,
                                                   accel,
                                                   Config::TSHAPE_RAMP_MIN_CM,
                                                   Config::TSHAPE_RAMP_MAX_CM,
                                                   Config::TSHAPE_NO_RAMP_EPSILON_PWM));
        float decelMs = cmToMs(resolveTShapeRampCm(cruise,
                                                   endSpeed,
                                                   accel,
                                                   Config::TSHAPE_RAMP_MIN_CM,
                                                   Config::TSHAPE_RAMP_MAX_CM,
                                                   Config::TSHAPE_NO_RAMP_EPSILON_PWM));

        // Triangle fallback: if the two FULL-peak ramps don't fit in the move, collapse the cruise
        // plateau and reduce the peak so the speed-vs-time integral still equals the cruise * total
        // requested. The simple averaged-peak heuristic keeps the math easy and bounded:
        // peak <= cruise always (clamp). The triangle test MUST run on the UN-capped ramps —
        // applying the half-of-total cap first floors each ramp at total*0.5, so their sum could
        // never exceed `total` and the branch was unreachable (finding #15). After this branch the
        // ramps are rescaled to sum to `total`, so the half-cap below is a no-op on this path; it
        // only guards the non-triangle case where the ramps fit overall but one side alone would
        // still over-run half the move.
        if (accelMs + decelMs > total)
        {
            int16_t peak = (int16_t) (((int32_t) startSpeed + (int32_t) endSpeed
                                       + (int32_t) cruise)
                                      / 3);
            // Peak must NEVER exceed the requested cruise.
            if (peak > cruise && cruise >= 0)
            {
                peak = cruise;
            }
            if (peak < cruise && cruise < 0)
            {
                peak = cruise;
            }
            _profilePeakSpd = peak;

            if (accelMs + decelMs > 0.0f)
            {
                const float scale = total / (accelMs + decelMs);
                accelMs *= scale;
                decelMs *= scale;
            }
        }
        else
        {
            _profilePeakSpd = cruise;
        }

        // Half-of-total cap — must not exceed half the move duration on either side. Runs AFTER the
        // triangle test (finding #15). In the triangle branch the ramps were rescaled to sum to
        // `total`, but an asymmetric pair can still leave one side above total*0.5 (e.g. 2.25/3.75
        // for a total of 6), so this cap may still trim the larger ramp; that only reshapes the apex
        // (a small plateau at the reduced peak instead of a pure triangle) and is harmless because
        // move completion is bound-gated on the time total, not on the velocity integral. In the
        // full-trapezoid branch it bounds a lone over-long ramp (e.g. a large start-speed delta
        // paired with a zero decel) so a plateau still exists.
        if (accelMs > total * 0.5f)
        {
            accelMs = total * 0.5f;
        }
        if (decelMs > total * 0.5f)
        {
            decelMs = total * 0.5f;
        }

        _profileAccelMs = (uint32_t) accelMs;
        _profileDecelMs = (uint32_t) decelMs;
        _profileTotalMs = (uint32_t) total;
        _profileStartMs = millis();
    }
    else
    {
        // ---------------------------------------------------------------------------------
        // Distance variant — encoder-based ramps in cm, converted to encoder degrees via the
        // shared `cmToEncDeg` helper (WROMath.h). The ramp resolution + triangle fallback +
        // half-of-total cap moved VERBATIM into `resolveTShapeRamps` (TShapeProfilePolicy.h)
        // so the math is host-tested + golden-pinned (issue #130); this block only converts
        // the resolved cm to encoder degrees and seeds the profile state. Negative `cruise`
        // (backwards) is handled implicitly: we store `_profileTotalEnc` / `_profileAccelEnc`
        // / `_profileDecelEnc` as positive magnitudes and `_updateProfile` compares them
        // against `abs(encDelta)`. The PWM setpoints (`_profileStartSpd` / `_profilePeakSpd`
        // / `_profileEndSpd`) carry the sign and are dispatched directly to the motors.
        // ---------------------------------------------------------------------------------
        const TShapeRamps ramps = resolveTShapeRamps(totalCm,
                                                     cruise,
                                                     startSpeed,
                                                     endSpeed,
                                                     accel,
                                                     Config::TSHAPE_RAMP_MIN_CM,
                                                     Config::TSHAPE_RAMP_MAX_CM,
                                                     Config::TSHAPE_NO_RAMP_EPSILON_PWM);
        _profilePeakSpd = ramps.peakPwm;

        // Store ramp + total distances as positive magnitudes — `_updateProfile` and
        // `_profileSpeedAt` compare them against `abs(encDelta)` so the same code path
        // handles both forward and reverse moves transparently.
        _profileTotalEnc = (int32_t) cmToEncDeg(totalCm);
        _profileAccelEnc = (int32_t) cmToEncDeg(ramps.accelCm < 0.0f ? 0.0f : ramps.accelCm);
        _profileDecelEnc = (int32_t) cmToEncDeg(ramps.decelCm < 0.0f ? 0.0f : ramps.decelCm);
        _profileStartEnc = MiniR4.M2.getDegrees();
    }

    // Initial state — RAMP_UP, even if accelDist == 0; the first `_updateProfile` tick will
    // observe `traveled >= _profileAccelEnc` (or its time-variant equivalent) and fall
    // straight through to the cruise plateau.
    _profileState = ProfileState::RAMP_UP;
    _moveDone     = false;
    return 0x00;
}

// -----------------------------------------------------------------------------------------
// Custom TurnGyro state machine (replaces MiniR4.DriveDC.TurnGyro)
//
// Two-stage profile: CRUISE at TURN_CRUISE_PWM until within TURN_SLOW_ZONE_DEG of target,
// then SLOW at TURN_SLOW_SPEED_PWM until within TURN_TOLERANCE_DEG, then BRAKE.
// Both wheels' PWM magnitude/sign depends on _turnDeltaSign (captured at dispatch) and
// _turnMotorMode (0=COUNTER_ROT, 1=PIVOT_LEFT, 2=PIVOT_RIGHT — repurposed from cmd.turnMode).
// -----------------------------------------------------------------------------------------

void DriveClass::_updateTurnProfile()
{
    if (_turnProfileState == TurnProfileState::IDLE)
    {
        return;
    }

    // Read current IMU yaw through YawSource (corrected app CCW-positive frame, ADR-045). On a
    // transient lower-MCU read failure SKIP this tick: holding the previous Move(L,R) one more
    // cycle is safer than acting on a fabricated 0.0 yaw, which mid-turn could flip the spin
    // direction. This is a deliberate behavior change vs the old "act on the vendor's 0.0".
    //
    // A SUSTAINED read failure is different. An async turn (`.async()`) has no `_drainBlocking`
    // deadline behind it, so skipping every tick would hold the counter-rotation Move(L,R) latched
    // forever — an unbounded spin with no fault path. The persistent-fault watchdog (mirrors the
    // stopAtLine / wallStop stall watchdog) escalates once no good read has landed for
    // `Config::TURN_IMU_STALL_TIMEOUT_MS` since dispatch / the last good read: hard-stop, drop the
    // queue, and latch `Fault::IMU_READ_FAILED`. The decision predicate `turnImuReadStalled` is
    // host-tested and rollover-safe.
    double curYaw = 0.0;
    if (!YawSource.readYawDeg(curYaw))
    {
        if (turnImuReadStalled(millis(), _turnLastGoodReadMs, Config::TURN_IMU_STALL_TIMEOUT_MS))
        {
            Serial.println(F("[Drive] turn: IMU yaw read failed past watchdog — IMU_READ_FAILED"));
            MiniR4.DriveDC.Move(0, 0);
            MiniR4.DriveDC.brake(true);    // emergency active-brake, like the _drainBlocking abort.
            _fault.kind       = Fault::IMU_READ_FAILED;
            _fault.rc         = 0x00;
            _fault.cmdKind    = DriveCommand::Kind::TURN_DEG;    // representative — the profile is
                                                                 // shared by TURN_DEG / TURN_TO_YAW.
            _turnProfileState = TurnProfileState::IDLE;
            _turnDone         = true;
            _moveDone         = true;    // single-task-done bit collapse (matches BRAKE branch).
            _clearQueue();
        }
        return;
    }

    // A good read landed — reset the persistent-fault watchdog window.
    _turnLastGoodReadMs    = millis();

    const int16_t cur      = (int16_t) curYaw;
    const int32_t rawDelta = normalizeTurnDelta((int32_t) _turnTargetYaw - (int32_t) cur);
    // rawDelta is in (-180, +180] so a plain abs() is sufficient — no risk of int overflow.
    // Consistent with the `abs(delta)` / `abs(encL)` pattern used elsewhere in this file.
    const float absDelta = (float) abs(rawDelta);

    // Final stop band — tightest check first, then slow-zone entry.
    if (absDelta <= Config::TURN_TOLERANCE_DEG)
    {
        _turnProfileState = TurnProfileState::BRAKE;
    }
    else if (absDelta <= Config::TURN_SLOW_ZONE_DEG
             && _turnProfileState == TurnProfileState::CRUISE)
    {
        _turnProfileState = TurnProfileState::SLOW;
    }

    // Choose PWM magnitude by state.
    int16_t pwmMag = 0;
    switch (_turnProfileState)
    {
    case TurnProfileState::CRUISE:
        pwmMag = _turnCruisePwm;
        break;
    case TurnProfileState::SLOW:
        pwmMag = Config::TURN_SLOW_SPEED_PWM;
        break;
    case TurnProfileState::BRAKE:
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(_turnProfileBrakeOnDone);
        _turnProfileState = TurnProfileState::IDLE;
        _turnDone         = true;
        _moveDone         = true;    // single-task-done bit collapse (matches _updateProfile)
        return;
    default:
        // IDLE handled above; no other state values exist.
        return;
    }

    // Apply rotation direction + wheel pattern via TurnSignPolicy (host-tested; ADR-045).
    // `_turnDeltaSign` was captured at dispatch from the CORRECTED (app CCW-positive) yaw:
    //   +1 → yaw must increase (CCW / left turn), -1 → yaw must decrease (CW / right turn).
    // The CCW counter-rotation pattern is L = -sign*pwm, R = +sign*pwm. The app frame is made
    // CCW-positive by Config::IMU_YAW_SIGN inside YawSource (the raw IMU is CW-positive), so this
    // pattern is correct WITHOUT flipping signs here. If a chassis turns the wrong way, change
    // Config::IMU_YAW_SIGN, not this block.
    const TurnWheels wheels = turnWheelPattern(_turnDeltaSign, pwmMag, _turnMotorMode);

    // Apply the same deadband snap / PWM saturation used by other drive outputs so
    // profiled turns neither stall below the minimum effective magnitude nor exceed
    // the configured PWM range at the actuator boundary.
    const int16_t L = (int16_t) deadbandSnap(wheels.left);
    const int16_t R = (int16_t) deadbandSnap(wheels.right);

    MiniR4.DriveDC.Move(L, R);
}

// -----------------------------------------------------------------------------------------
// Drive-until-line seek state machine (`stopAtLine()` cooperative driver) — ADR-027
//
// CONTINUOUS, event-terminated seek. Unlike `_updateProfile()` there is no known total distance
// to plan a ramp-down against — the stop point is unknown until a `T_OR_CROSS` line is detected.
// Each tick this commands a continuous primitive (Move / MoveSync / MoveGyro) per `_stopMode` and
// walks SEEK → ARMED → OVERSHOOT → DONE:
//   - SEEK      cruise flat; arm detection once within the arm window of the expected endpoint.
//   - ARMED     keep cruising; debounce `LineArray.getJunctionType() == T_OR_CROSS`. On confirm,
//               latch `_isLineFound`, capture the detect encoder, enter OVERSHOOT. On reaching the
//               max distance first, fault `LINE_NOT_FOUND` (stop + clear queue + restore board).
//   - OVERSHOOT roll the known overshoot distance, ramping cruise → `Config::PWM_MIN_MAGNITUDE`,
//               then enter DONE.
//   - DONE      issue the final stop (active-brake per `_stopBrake`), restore the front board,
//               set `_moveDone`, return to IDLE.
//
// Reads `LineArray.getJunctionType()` directly — NOT `Intersection::isAtIntersection()`, which is
// read-and-clear and owned by PathTracker's nav consumer (ADR-027). The reverse (rear-board) path
// is `@untested on hardware` until the rear board is wired (ADR-024).
// -----------------------------------------------------------------------------------------

void DriveClass::_updateStopAtLine()
{
    if (_stopState == StopAtLineState::IDLE)
    {
        return;
    }

    // DONE neither reads the encoder nor commands continuous motion — it only finalises. Handle it
    // first so the per-tick STM32 encoder round-trip (`MiniR4.M2.getDegrees()`) and `encDegToCm`
    // below are skipped entirely in that state.
    if (_stopState == StopAtLineState::DONE)
    {
        // Final stop. Active-brake (or coast) per `_stopBrake`, restore the front board, mark the
        // seek complete so `_dispatchNext` can hand off to the next queued command on this tick.
        // DONE is a normal completion — the queue is intentionally NOT cleared.
        _endStopAtLine(_stopBrake);
        return;
    }

    // Travel measured as an unsigned magnitude from the per-side baseline (M2, the canonical
    // encoder), matching `_updateProfile`. Forward and reverse both accumulate positive `traveled`
    // so the arm-window / max-distance comparisons need no sign juggling.
    const int32_t nowEnc   = MiniR4.M2.getDegrees();
    const float   traveled = encDegToCm((int32_t) abs(nowEnc - _stopStartEnc));

    // No-progress watchdog (ADR-027): whenever `traveled` advances by at least
    // `Config::STOP_AT_LINE_PROGRESS_EPS_CM` since the last mark, reset the watchdog (record the new
    // baseline + timestamp). A seek whose `traveled` stops advancing (zero cruise / jammed wheel)
    // never resets it, so `stopAtLineStalled(...)` faults `LINE_NOT_FOUND` once the timeout elapses.
    // The progress decision lives in `stopAtLineProgressed` so the slow-but-moving "never stalls"
    // case is host-testable (TC17).
    if (stopAtLineProgressed(traveled, _stopLastProgressCm, Config::STOP_AT_LINE_PROGRESS_EPS_CM))
    {
        _stopLastProgressCm = traveled;
        _stopLastProgressMs = millis();
    }
    const bool stalled =
        stopAtLineStalled(millis(), _stopLastProgressMs, Config::STOP_AT_LINE_STALL_TIMEOUT_MS);

    // Emit a continuous PWM at the given positive magnitude through the shared `_emitContinuous`,
    // applying the seek's travel direction (backward when `_stopReverse`). `_stopCruise` and every
    // magnitude passed here are positive — the builder carries direction in `_stopReverse`, never in
    // the power sign.
    auto driveContinuous = [&](int16_t mag)
    {
        const int16_t pwm = (int16_t) (_stopReverse ? -mag : mag);
        _emitContinuous(_stopMode, pwm, _stopHeading);
    };

    switch (_stopState)
    {
    case StopAtLineState::SEEK:
    {
        // No line by the max search distance, or no progress for the stall timeout → fail fast. The
        // SEEK phase enforces the max-distance cap too (not only ARMED) so a seek that never arms
        // (tight `setMaxDistance`) cannot overshoot, and a never-advancing seek cannot hang the
        // async FSM forever. Same `LINE_NOT_FOUND` teardown as the ARMED no-line path.
        if (stopAtLineSeekMaxedOut(traveled, _stopMaxCm) || stalled)
        {
            Serial.println(F("[Drive] stopAtLine: no line within max distance — LINE_NOT_FOUND"));
            _fault.kind    = Fault::LINE_NOT_FOUND;
            _fault.rc      = 0x00;
            _fault.cmdKind = DriveCommand::Kind::STOP_AT_LINE;
            _isLineFound   = false;
            _clearQueue();
            _endStopAtLine(true);
            break;
        }

        // Cruise flat toward the endpoint; arm detection once inside the window. The arm threshold
        // (`expectedCm - armWithin`, clamped at 0) lives in `stopAtLineArmed` so the SEEK → ARMED
        // decision is host-testable (TC11). `_stopDetectStreak` is already zeroed at dispatch and
        // SEEK never writes it — ARMED is its only writer — so no reset is needed on this transition.
        driveContinuous(_stopCruise);
        if (stopAtLineArmed(traveled, _stopExpectedCm, _stopArmWithin))
        {
            _stopState = StopAtLineState::ARMED;
        }
        break;
    }

    case StopAtLineState::ARMED:
    {
        // No line by the max search distance, or no progress for the stall timeout → fail fast. Stop,
        // latch `LINE_NOT_FOUND`, clear the queue, restore the front board. No silent dead-reckon
        // fallback (spec / AC7). Both gates run before the detection read so a maxed/stalled seek
        // cannot also confirm on the same tick.
        if (stopAtLineSeekMaxedOut(traveled, _stopMaxCm) || stalled)
        {
            Serial.println(F("[Drive] stopAtLine: no line within max distance — LINE_NOT_FOUND"));
            _fault.kind    = Fault::LINE_NOT_FOUND;
            _fault.rc      = 0x00;
            _fault.cmdKind = DriveCommand::Kind::STOP_AT_LINE;
            _isLineFound   = false;
            _clearQueue();
            _endStopAtLine(true);
            break;
        }

        // Keep cruising while watching for the crossing. Read the junction classification DIRECTLY
        // from the line array (NOT the read-and-clear `Intersection::isAtIntersection()`), and step
        // the consecutive-sample debounce. Detection confirms when the streak reaches
        // `Config::STOP_AT_LINE_DETECT_TICKS`.
        driveContinuous(_stopCruise);

        const bool crossNow =
            (LineArray.getJunctionType() == WROLineArrayClass::JUNCTION_T_OR_CROSS);
        _stopDetectStreak =
            debounceStep(_stopDetectStreak, crossNow, Config::STOP_AT_LINE_DETECT_TICKS);

        if (_stopDetectStreak >= Config::STOP_AT_LINE_DETECT_TICKS)
        {
            // Confirmed line. Latch the result, anchor the overshoot roll on the current encoder,
            // and hand off to OVERSHOOT.
            _isLineFound   = true;
            _stopDetectEnc = nowEnc;
            _stopState     = StopAtLineState::OVERSHOOT;
        }
        break;
    }

    case StopAtLineState::OVERSHOOT:
    {
        // Roll the known overshoot distance so the wheel axle lands on the line, ramping the cruise
        // magnitude down to the deadband floor over the roll. The overshoot distance IS known, so
        // (unlike the seek) this phase can ramp down. `frac` in [0, 1] is the fraction of the roll
        // completed, measured from the detect encoder.
        const float rolled = encDegToCm((int32_t) abs(nowEnc - _stopDetectEnc));

        if (_stopOvershoot <= 0.0f || rolled >= _stopOvershoot)
        {
            // Overshoot consumed (or none requested) — finalise on the next state.
            _stopState = StopAtLineState::DONE;
            break;
        }

        // Linear ramp of the (positive) cruise magnitude down to the deadband floor over the roll;
        // `driveContinuous` reapplies the seek direction. Clamp at the floor so the motor never
        // stalls mid-roll. If cruise was already below the floor, hold it flat at cruise.
        const float   frac     = rolled / _stopOvershoot;
        const int16_t floorPwm = Config::PWM_MIN_MAGNITUDE;
        int16_t       rampMag  = _stopCruise;
        if (_stopCruise > floorPwm)
        {
            rampMag = (int16_t) (_stopCruise - (int16_t) ((_stopCruise - floorPwm) * frac));
            if (rampMag < floorPwm)
            {
                rampMag = floorPwm;
            }
        }
        driveContinuous(rampMag);
        break;
    }

    default:
        // IDLE and DONE are handled above the encoder read; no other state values exist.
        break;
    }
}

void DriveClass::_endStopAtLine(bool brake)
{
    // Shared seek teardown (ADR-027). Used by the DONE state (brake = `_stopBrake`), the no-line /
    // max-distance / stall fault paths (brake = true), and `stop()` / `_drainBlocking()` when an
    // in-flight seek is aborted. Restores the front board — a reverse seek may have left the rear
    // board active via `setReverseActive(true)`; no-op in 1-board configs. Does NOT clear the queue:
    // the normal DONE hand-off keeps it, and fault paths call `_clearQueue()` themselves first.
    MiniR4.DriveDC.Move(0, 0);
    MiniR4.DriveDC.brake(brake);
    LineArray.setReverseActive(false);
    _stopState = StopAtLineState::IDLE;
    _moveDone  = true;
    _turnDone  = true;
}

// -----------------------------------------------------------------------------------------
// Wall-stop push state machine (`wallStop()` cooperative driver) — wall-zero reset nodes (AC8)
//
// CONTINUOUS, STALL-terminated push. Unlike `_updateProfile()` there is no known total distance to
// plan a ramp-down against — the stop point is unknown until the wheels stall against the wall. Each
// tick this commands a continuous primitive (Move / MoveSync / MoveGyro) per `_wallMode` in the push
// direction (`_wallReverse`) and walks PUSH → BACK_OFF → DONE:
//   - PUSH      drive flat at the low push PWM; REUSE the stop-at-line no-progress watchdog
//               (`stopAtLineProgressed` / `stopAtLineStalled` over STOP_AT_LINE_STALL_TIMEOUT_MS).
//               A stall here is SUCCESS (wall reached) — the INVERSE of the seek, where a stall is
//               the `LINE_NOT_FOUND` fault. On stall, capture the back-off encoder + enter BACK_OFF.
//               If the outer advisory timeout elapses first WITHOUT a stall, record the
//               NON-LATCHING `_wallStopTimedOut` advisory (NOT a hard fault) + enter DONE.
//   - BACK_OFF  roll the configured back-off distance in the reverse-of-push direction, then DONE.
//   - DONE      issue the final stop (active-brake per `_wallBrake`), restore the front board, set
//               `_moveDone`, return to IDLE.
//
// The stall watchdog is the SAME mechanism `_updateStopAtLine` uses — the only inversion is the
// meaning (success vs fault), so the predicates and Config constants are shared, not duplicated.
// -----------------------------------------------------------------------------------------

void DriveClass::_updateWallStop()
{
    if (_wallState == WallStopState::IDLE)
    {
        return;
    }

    // DONE neither reads the encoder nor commands continuous motion — it only finalises. Handle it
    // first so the per-tick STM32 encoder round-trip below is skipped entirely in that state.
    if (_wallState == WallStopState::DONE)
    {
        // Final stop. Active-brake (or coast) per `_wallBrake`, restore the front board, mark the
        // push complete so `_dispatchNext` can hand off to the next queued command on this tick.
        _endWallStop(_wallBrake);
        return;
    }

    // SQUARE precedes the push (#1): the in-place turn to the wall normal is driven by
    // `_updateTurnProfile()` (it ran earlier this tick); here we only wait for it to idle, then seed
    // the push watchdogs at the TRUE start of the push and enter PUSH. Seeding here (not at dispatch)
    // ensures a long square-up never eats into the push's stall window. The turn does not command the
    // push primitive, so SQUARE issues no continuous motion of its own.
    if (_wallState == WallStopState::SQUARE)
    {
        if (_turnProfileState != TurnProfileState::IDLE)
        {
            return;    // still squaring — let `_updateTurnProfile` drive the in-place turn.
        }
        // Squared. Seed BOTH watchdogs (AC8) now: the reused stop-at-line no-progress watchdog
        // (baseline at zero travel, clock from now) AND the outer advisory timeout (clock from now).
        // The inner stall watchdog REUSES `STOP_AT_LINE_STALL_TIMEOUT_MS` / `STOP_AT_LINE_PROGRESS_EPS_CM`
        // via `stopAtLineProgressed` / `stopAtLineStalled` — a stall here means SUCCESS (wall reached),
        // the inverse of the seek's `LINE_NOT_FOUND`.
        _wallStartEnc       = MiniR4.M2.getDegrees();    // canonical encoder, matches `_updateProfile`
        _wallBackOffEnc     = _wallStartEnc;
        _wallLastProgressCm = 0.0f;
        _wallLastProgressMs = millis();
        _wallStartMs        = millis();
        _moveDone           = false;    // the turn's BRAKE set this true; the push re-owns it now.
        _wallState          = WallStopState::PUSH;
        return;            // begin pushing on the next tick.
    }

    // Travel measured as an unsigned magnitude from the per-side baseline (M2, the canonical
    // encoder), matching `_updateProfile` / `_updateStopAtLine`. Forward and reverse both accumulate
    // positive `traveled`, so the back-off comparison needs no sign juggling.
    const int32_t nowEnc   = MiniR4.M2.getDegrees();
    const float   traveled = encDegToCm((int32_t) abs(nowEnc - _wallStartEnc));

    // Emit a continuous PWM at the given positive magnitude through the shared `_emitContinuous`,
    // applying the push direction (backward when `_wallReverse`). Every magnitude passed here is
    // positive — the builder carries direction in `_wallReverse`, never in the power sign.
    auto driveContinuous = [&](int16_t mag)
    {
        const int16_t pwm = (int16_t) (_wallReverse ? -mag : mag);
        _emitContinuous(_wallMode, pwm, _wallHeading);
    };

    switch (_wallState)
    {
    case WallStopState::PUSH:
    {
        // Reused no-progress watchdog (AC8): whenever `traveled` advances by at least
        // `Config::STOP_AT_LINE_PROGRESS_EPS_CM` since the last mark, reset the watchdog (record the
        // new baseline + timestamp). A push that stops advancing (wheels stalled against the wall)
        // never resets it, so `stopAtLineStalled(...)` fires once the timeout elapses — and HERE that
        // is SUCCESS, the inverse of the stop-at-line seek. The progress decision lives in
        // `stopAtLineProgressed` (host-tested via TC17) and is shared verbatim.
        if (stopAtLineProgressed(traveled, _wallLastProgressCm, Config::STOP_AT_LINE_PROGRESS_EPS_CM))
        {
            _wallLastProgressCm = traveled;
            _wallLastProgressMs = millis();
        }
        const uint32_t now     = millis();
        const bool     stalled =
            stopAtLineStalled(now, _wallLastProgressMs, Config::STOP_AT_LINE_STALL_TIMEOUT_MS);

        if (stalled)
        {
            // Wall reached — SUCCESS. Anchor the back-off roll on the current encoder and hand off to
            // BACK_OFF. `_wallStopTimedOut` stays false (it was cleared at dispatch).
            _wallBackOffEnc = nowEnc;
            _wallState      = WallStopState::BACK_OFF;
            break;
        }

        // Outer advisory timeout — never stalled (e.g. never reached a wall, wheels kept turning
        // freely). Record the NON-LATCHING advisory flag (NOT a hard `_fault` — `hasFault()` stays
        // false and the queue is NOT frozen) and finalise via DONE. The caller assumes-at-wall and
        // continues. Checked AFTER the stall gate so a press that stalls on the very tick the outer
        // timeout would also fire still reports success, never a spurious advisory timeout.
        if ((uint32_t) (now - _wallStartMs) >= _wallTimeoutMs)
        {
            Serial.println(F("[Drive] wallStop: no stall within timeout — advisory (assume at wall)"));
            _wallStopTimedOut = true;
            _wallState        = WallStopState::DONE;
            break;
        }

        // Still pushing — hold the flat low push PWM toward the wall.
        driveContinuous(_wallPushPwm);
        break;
    }

    case WallStopState::BACK_OFF:
    {
        // Roll the known back-off distance in the reverse-of-push direction so the chassis is no
        // longer pressing the wall. The distance IS known, so this phase could ramp — but the roll
        // is short (a few cm) and the watchdog is disarmed here, so a flat low-PWM roll is simplest
        // and reliable. `rolled` is the magnitude travelled since the stall was confirmed.
        const float rolled = encDegToCm((int32_t) abs(nowEnc - _wallBackOffEnc));

        if (_wallBackOffCm <= 0.0f || rolled >= _wallBackOffCm)
        {
            // Back-off consumed (or none requested) — finalise on the next state.
            _wallState = WallStopState::DONE;
            break;
        }

        // Drive at the deadband floor in the REVERSE-of-push direction. `_wallReverse` already
        // selected the push direction; negate it here so back-off opposes the push regardless of
        // whether the push itself was forward or rearward.
        const int16_t floorPwm   = Config::PWM_MIN_MAGNITUDE;
        const int16_t backOffMag = (int16_t) (_wallReverse ? floorPwm : -floorPwm);
        _emitContinuous(_wallMode, backOffMag, _wallHeading);
        break;
    }

    default:
        // IDLE and DONE are handled above the encoder read; no other state values exist.
        break;
    }
}

void DriveClass::_endWallStop(bool brake)
{
    // Shared push teardown (AC8). Used by the DONE state (brake = `_wallBrake`) and `stop()` /
    // `_drainBlocking()` when an in-flight push is aborted. Restores the front board for symmetry
    // with `_endStopAtLine` (wall-stop never selects the rear board, so this is a harmless no-op,
    // but it keeps the active-board invariant explicit). Does NOT clear the queue (the normal DONE
    // hand-off keeps it) and does NOT touch `_wallStopTimedOut` (owned by the PUSH-phase outcome)
    // or `_fault` (the wall-stop timeout is non-latching by design).
    MiniR4.DriveDC.Move(0, 0);
    MiniR4.DriveDC.brake(brake);
    LineArray.setReverseActive(false);
    _wallState = WallStopState::IDLE;
    _moveDone  = true;
    _turnDone  = true;
}

// -----------------------------------------------------------------------------------------
// Fault state accessors (code review F5).
// -----------------------------------------------------------------------------------------

bool DriveClass::hasFault() const
{
    return _fault.kind != Fault::NONE;
}

DriveClass::FaultInfo DriveClass::getFault() const
{
    return _fault;
}

void DriveClass::clearFault()
{
    _fault = FaultInfo{};
}

// -----------------------------------------------------------------------------------------
// Command-queue plumbing — invoked from update() and from builder destructors.
// -----------------------------------------------------------------------------------------

bool DriveClass::_enqueue(const DriveCommand& cmd)
{
    if (_qCount >= Config::DRIVE_QUEUE_CAPACITY)
    {
        // Caller bug — the path planner is issuing more bounded moves than the queue can
        // hold without an occasional `_drainBlocking()`. Drop the new command rather than
        // silently overwriting an in-flight slot, and surface the issue on Serial.
        Serial.println(F("[Drive] queue overflow — dropping command"));
        return false;
    }
    _queue[_qTail] = cmd;
    _qTail         = (uint8_t) ((_qTail + 1) % Config::DRIVE_QUEUE_CAPACITY);
    _qCount++;
    return true;
}

void DriveClass::_dispatchNext()
{
    if (_qCount == 0)
    {
        return;
    }

    // Fault gate (code review F5): once a fault is latched, the queue is frozen until
    // higher-level recovery code calls `clearFault()`. Without this, `update()` would
    // happily dispatch the next queued command on the very tick after a failed one — and
    // the mission planner's pose model would diverge from physical reality.
    if (hasFault())
    {
        return;
    }

    // Pop.
    const DriveCommand cmd = _queue[_qHead];
    _qHead                 = (uint8_t) ((_qHead + 1) % Config::DRIVE_QUEUE_CAPACITY);
    _qCount--;

    // Race guard: a Drive bounded/profiled move owns the motors exclusively for its
    // duration. If the line follower is enabled, its update() would race us by writing
    // setSpeed every tick. Auto-disable with brake=false so the follower stops cleanly
    // without us issuing a redundant brake (the queued move will set its own motor
    // setpoints on its first tick). Caller re-enables the follower after the queued
    // sequence completes.
    if (LineFollower.isEnabled())
    {
        LineFollower.setEnabled(false).setBrake(false);
    }

    // Dispatch by kind. All paths use blocking=false — the queue is the asynchronous
    // execution chain; blocking-variant builders go through `_drainBlocking()` to achieve
    // synchronous semantics on top of this.
    //
    // Capture the rc and latch a fault on non-zero (code review F5). The W3 guard inside
    // each executor already prevents a failed dispatch from leaving `_moveDone = false`
    // (which would have hung `_drainBlocking` forever); the fault gate above stops the
    // follow-up queued commands from racing ahead of the lower MCU's actual state.
    uint8_t rc = 0x00;
    switch (cmd.kind)
    {
    case DriveCommand::Kind::DRIVE_DIST:
        switch (cmd.mode)
        {
        case DriveMode::RAW:
            rc = _execDriveDistRaw(cmd.power, cmd.cm, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::SYNC:
            rc = _execDriveDistSync(cmd.power, cmd.cm, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::GYRO:
            rc = _execDriveDistGyro(cmd.power, cmd.cm, cmd.heading, cmd.brake, /*blocking*/ false);
            break;
        }
        break;

    case DriveCommand::Kind::DRIVE_TIME:
        switch (cmd.mode)
        {
        case DriveMode::RAW:
            rc = _execDriveTimeRaw(cmd.power, cmd.sec, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::SYNC:
            rc = _execDriveTimeSync(cmd.power, cmd.sec, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::GYRO:
            rc = _execDriveTimeGyro(cmd.power, cmd.sec, cmd.heading, cmd.brake, /*blocking*/ false);
            break;
        }
        break;

    case DriveCommand::Kind::TURN_DEG:
        rc = _execTurnDeg(cmd.deg, cmd.power, cmd.turnMode, cmd.brake, /*blocking*/ false);
        break;

    case DriveCommand::Kind::TURN_TO_YAW:
        rc = _execTurnToYaw(cmd.yaw, cmd.power, cmd.turnMode, cmd.brake, /*blocking*/ false);
        break;

    case DriveCommand::Kind::PROFILE_DIST:
        rc = _execProfileMove(cmd.mode,
                              /*isTime*/ false,
                              cmd.cm,
                              /*totalSec*/ 0.0f,
                              cmd.power,
                              cmd.startSpeed,
                              cmd.endSpeed,
                              cmd.accel,
                              cmd.brake,
                              cmd.heading);
        break;

    case DriveCommand::Kind::PROFILE_TIME:
        rc = _execProfileMove(cmd.mode,
                              /*isTime*/ true,
                              /*totalCm*/ 0.0f,
                              cmd.sec,
                              cmd.power,
                              cmd.startSpeed,
                              cmd.endSpeed,
                              cmd.accel,
                              cmd.brake,
                              cmd.heading);
        break;

    case DriveCommand::Kind::STOP_AT_LINE:
    {
        // Seed the continuous, event-terminated seek state machine (ADR-027). This does NOT go
        // through `_execProfileMove` — the stop point is unknown until the line is detected, so
        // there is no known-distance trapezoid to plan. `_updateStopAtLine()` owns the wheels
        // from here until DONE/fault. LineFollower was already auto-disabled above.
        //
        // Reject reverse on a 1-board chassis at dispatch: the rear board does not exist, so
        // `setReverseActive(true)` is a no-op and the FRONT board would read while the chassis
        // backs up — a wrong-facing sample. Fail fast with `LINE_NOT_FOUND` rather than seek
        // against the wrong sensor. (PathTracker's REVERSE_UNAVAILABLE guard already prevents the
        // map path from emitting a reverse hop here — this is defense-in-depth for the bare verb.)
        if (cmd.reverse && Config::LINE_BOARD_COUNT < 2)
        {
            Serial.println(F("[Drive] stopAtLine: reverse requires LINE_BOARD_COUNT==2 — rejected"));
            _fault.kind    = Fault::LINE_NOT_FOUND;
            _fault.rc      = 0x00;
            _fault.cmdKind = cmd.kind;
            _isLineFound   = false;

            MiniR4.DriveDC.Move(0, 0);
            MiniR4.DriveDC.brake(true);
            _moveDone = true;
            _turnDone = true;
            _clearQueue();
            return;
        }

        // Capture seek parameters. Per-side encoder baseline is M2 (the canonical encoder used by
        // `_updateProfile`); `traveled` is measured as `abs(now - _stopStartEnc)` each tick so the
        // forward/reverse direction needs no sign juggling here.
        _stopMode        = cmd.mode;
        _stopCruise      = cmd.power;
        _stopHeading     = cmd.heading;
        _stopExpectedCm  = cmd.expectedCm;
        _stopArmWithin   = cmd.armWithin;
        _stopMaxCm       = cmd.maxCm;
        _stopOvershoot   = cmd.overshoot;
        _stopReverse     = cmd.reverse;
        _stopBrake       = cmd.brake;
        _stopStartEnc    = MiniR4.M2.getDegrees();
        _stopDetectEnc   = _stopStartEnc;
        _stopDetectStreak = 0;
        _isLineFound     = false;    // cleared on dispatch; latched true only on a confirmed line.

        // Seed the no-progress watchdog: baseline at zero travel, clock from now (ADR-027). The
        // watchdog faults `LINE_NOT_FOUND` if `traveled` does not advance for
        // `Config::STOP_AT_LINE_STALL_TIMEOUT_MS`.
        _stopLastProgressCm = 0.0f;
        _stopLastProgressMs = millis();

        // Select the rear board for a reverse seek so `getJunctionType()` samples the board that
        // faces the travel direction. No-op in 1-board configs (already rejected above), restored
        // to the front board at DONE/fault. LineFollower is disabled for the seek, so Drive briefly
        // owning the active-board selection is safe.
        LineArray.setReverseActive(cmd.reverse);

        _stopState = StopAtLineState::SEEK;
        _moveDone  = false;          // the seek owns _moveDone until DONE/fault. `rc` stays 0x00 —
                                     // seeding cannot fail at dispatch (no lower-MCU round trip).
        break;
    }

    case DriveCommand::Kind::WALL_STOP:
    {
        // Seed the continuous, stall-terminated wall push (AC8). Like STOP_AT_LINE this does NOT go
        // through `_execProfileMove` — the stop point is unknown until the wheels stall against the
        // wall, so there is no known-distance trapezoid. `_updateWallStop()` owns the wheels from
        // here until DONE. LineFollower was already auto-disabled above.
        _wallMode     = cmd.mode;
        _wallPushPwm  = cmd.power;
        _wallHeading  = cmd.heading;
        _wallBackOffCm = cmd.backOffCm;
        _wallTimeoutMs = cmd.timeoutMs;
        _wallReverse  = cmd.reverse;
        _wallBrake    = cmd.brake;
        _wallStopTimedOut = false;    // cleared on dispatch; set true only on the no-stall timeout.

        // (#1) SQUARE FIRST. Turn in place to the wall normal (`_wallHeading`) BEFORE pushing, so the
        // press is square regardless of the arrival heading — a misaligned arrival no longer arcs into
        // the wall (the old GYRO push steered while driving). Reuse the existing `_updateTurnProfile`
        // turn machine via `_execTurnToYaw`; COUNTER_ROT (mode 0) rotates about the chassis center so
        // the axle stays on the node. The push watchdogs are seeded later, at the SQUARE→PUSH
        // transition in `_updateWallStop`, NOT here — a long square-up must not eat the push's stall
        // window. A no-op turn (already squared) short-circuits inside `_execTurnToYaw`, so the very
        // next `_updateWallStop` tick advances straight to PUSH.
        (void) _execTurnToYaw(_wallHeading, /*power*/ 0, /*mode COUNTER_ROT*/ 0,
                              /*brake*/ true, /*blocking*/ false);

        _wallState = WallStopState::SQUARE;
        _moveDone  = false;          // the interlude owns _moveDone until DONE. `rc` stays 0x00 —
                                     // seeding cannot fail at dispatch (no lower-MCU round trip).
        break;
    }
    }

    if (rc != 0x00)
    {
        // Lower MCU rejected the command. Freeze the queue, stop the chassis, latch the
        // fault. Higher-level recovery is responsible for `clearFault()` + re-issue.
        Serial.print(F("[Drive] _dispatchNext: dispatch failed, rc=0x"));
        Serial.println(rc, HEX);
        _fault.kind    = Fault::DISPATCH_FAILED;
        _fault.rc      = rc;
        _fault.cmdKind = cmd.kind;

        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
        _moveDone         = true;
        _turnDone         = true;
        _profileState     = ProfileState::IDLE;
        _turnProfileState = TurnProfileState::IDLE;
        _clearQueue();
    }
}

bool DriveClass::_isIdle() const
{
    return _moveDone && _turnDone && _profileState == ProfileState::IDLE
           && _turnProfileState == TurnProfileState::IDLE
           && _stopState == StopAtLineState::IDLE
           && _wallState == WallStopState::IDLE;
}

void DriveClass::_drainBlocking()
{
    // Allowed `delay()` site #3. The blocking caller is by definition off the cooperative
    // tick chain (synchronous-orchestration path), so this spin doesn't violate the iron
    // rule — which targets non-blocking `update()` chains, not orchestration.
    //
    // Sync-by-default builders need EVERY module to keep ticking during the spin —
    // otherwise EventTrace doesn't render, LineFollower stops polling the tracer, etc.
    // The sketch registers a `cooperativeTick()`-style function via `setTickCallback`;
    // we call it here. If no callback was registered we degrade to ticking only
    // `Drive.update()` and warn once.
    //
    // Bounded-spin guard (code review F3): without a deadline a stalled wheel, lost
    // STM32 task-done bit, or unreachable profile target would trap the entire mission.
    // Budget is `(qCount + 1) * DRIVE_DRAIN_TIMEOUT_MS_PER_CMD` — the +1 covers the
    // command currently in flight (which is no longer in `_queue`). On expiry we hard-
    // stop the chassis, drop the queue, and emit a Serial warning so higher layers see
    // the failure. Mission-level recovery hooks land with F5's fault struct.
    const uint32_t budget   = ((uint32_t) _qCount + 1U)
                              * (uint32_t) Config::DRIVE_DRAIN_TIMEOUT_MS_PER_CMD;
    const uint32_t deadline = millis() + budget;

    while (!_isIdle() || _qCount > 0)
    {
        // Fault early-out (issue #150): a fault latched BEFORE this verb's command was
        // enqueued freezes `_dispatchNext` forever (its own fault gate refuses to pop the
        // queue while `hasFault()`), so a sync verb issued while faulted would otherwise sit
        // here spinning for the ENTIRE budget below before the timeout branch fires — and
        // that branch unconditionally overwrites `_fault.kind` with `DRAIN_TIMEOUT`, losing
        // whatever fault (e.g. `DISPATCH_FAILED`, `LINE_NOT_FOUND`, `IMU_READ_FAILED`) was
        // actually latched. Bail out immediately instead: drop the just-enqueued command
        // (it can never dispatch anyway) and return with the ORIGINAL `_fault.kind` intact,
        // so higher-level recovery sees the true cause, not a fabricated timeout.
        if (hasFault())
        {
            _clearQueue();
            return;
        }

        if ((int32_t) (millis() - deadline) >= 0)
        {
            Serial.println(F("[Drive] _drainBlocking: timeout — aborting queue"));
            // Hard-stop sequence — mirrors `stop(true)` minus the public-API bookkeeping
            // (we already own the state we need to clear). Latch a fault so higher-level
            // recovery (mission planner) sees the abort via `hasFault()` (F5).
            MiniR4.DriveDC.Move(0, 0);
            MiniR4.DriveDC.brake(true);
            _fault.kind = Fault::DRAIN_TIMEOUT;
            _fault.rc   = 0x00;
            // _fault.cmdKind left at its default — the in-flight command's kind is no longer
            // tracked once dispatched.
            _moveDone         = true;
            _turnDone         = true;
            _profileState     = ProfileState::IDLE;
            _turnProfileState = TurnProfileState::IDLE;
            // Abort an in-flight stopAtLine seek too and restore the front board (ADR-027) — a
            // reverse seek may have left the rear board active. No-op in 1-board configs. Active-brake
            // to match the hard-stop above. (The shared teardown re-issues `Move(0, 0)` +
            // `brake(true)`, which is harmless after the hard-stop.)
            if (_stopState != StopAtLineState::IDLE)
            {
                _endStopAtLine(true);
            }
            // Abort an in-flight wallStop push too (AC8) — active-brake to match the hard-stop. The
            // shared teardown re-issues `Move(0, 0)` + `brake(true)`, harmless after the hard-stop.
            if (_wallState != WallStopState::IDLE)
            {
                _endWallStop(true);
            }
            _clearQueue();
            return;
        }

        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!_tickCallbackWarningEmitted)
            {
                Serial.println(F("[Drive] _drainBlocking: no tick callback set — only "
                                  "Drive.update() will tick during blocks. Call "
                                  "Drive.setTickCallback(...) in setup()."));
                _tickCallbackWarningEmitted = true;
            }
            update();
        }
        delay(2);
    }
}

void DriveClass::_clearQueue()
{
    // Reset the FIFO without touching its storage — slots are no longer referenced.
    _qHead  = 0;
    _qTail  = 0;
    _qCount = 0;
}
