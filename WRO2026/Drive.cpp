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

#include "LineFollower.h"    // Race-guard: _dispatchNext() auto-disables LineFollower so
                              // its tick loop does not write setSpeed concurrently with a
                              // queued bounded/profiled move. LineFollower.h does NOT pull
                              // Drive.h transitively, so the include is non-circular.

#include "WROLineArray.h"    // ADR-027: _updateStopAtLine() reads `LineArray.getJunctionType()`
                              // directly (NOT `Intersection::isAtIntersection()`, which is
                              // read-and-clear and owned by PathTracker's nav consumer). The
                              // include lives in the .cpp ONLY — Drive.h needs none of
                              // WROLineArray's definition, so including it here keeps
                              // MatrixColor out of every TU that pulls in Drive.h.

static_assert(Config::WHEEL_DIAM_MM > 0.0f, "WHEEL_DIAM_MM must be positive");

/// @brief Single global instance.
DriveClass Drive;

/**
 * @brief Raw `getJunctionType()` value meaning "full-bar crossing" (T-junction / 4-way).
 *
 * Mirrors `JunctionType::T_OR_CROSS == 3` from Intersection.h, which itself mirrors the
 * `WROLineArrayClass::getJunctionType()` 0..3 contract. Defined locally so Drive.cpp does not
 * pull in `Intersection.h` (and the `Intersection` consumer it owns) just for one enumerator —
 * the seek reads the sensor classification directly per ADR-027. If the junction-type encoding
 * ever changes, both this constant and `Intersection.h`'s enum move together.
 */
static constexpr uint8_t STOP_AT_LINE_T_OR_CROSS_RAW = 3;

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
    // -------------------------------------------------------------------------------------
    if (_profileState == ProfileState::IDLE
        && _turnProfileState == TurnProfileState::IDLE
        && _stopState == StopAtLineState::IDLE
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

uint8_t DriveClass::setSpeed(int16_t L, int16_t R)
{
    _applyBias(L, R);
    L = deadbandSnap(L);
    R = deadbandSnap(R);
    return MiniR4.DriveDC.Move(L, R);
}

uint8_t DriveClass::setSpeedSync(int16_t L, int16_t R)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    return MiniR4.DriveDC.MoveSync(L, R);
}

uint8_t DriveClass::forwardGyro(int16_t power)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    return MiniR4.DriveDC.MoveGyro(power, _heldYaw);
}

uint8_t DriveClass::stop(bool brake)
{
    MiniR4.DriveDC.Move(0, 0);
    const uint8_t brakeRc = MiniR4.DriveDC.brake(brake);

    _moveDone         = true;
    _turnDone         = true;
    _profileState     = ProfileState::IDLE;        // also abort any in-flight profile
    _turnProfileState = TurnProfileState::IDLE;    // also abort any in-flight turn profile

    // Abort any in-flight stopAtLine seek too (ADR-027). A reverse seek may have left the rear
    // board active via `setReverseActive(true)` — restore the front board so LineFollower and
    // any later read sample the forward-facing sensor. No-op in 1-board configs.
    if (_stopState != StopAtLineState::IDLE)
    {
        _stopState = StopAtLineState::IDLE;
        LineArray.setReverseActive(false);
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
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    const uint16_t encDeg = cmToEncDeg(cm);
    const uint8_t  rc =
        MiniR4.DriveDC.MoveSyncDegs(power, power, encDeg, brake, !blocking);
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
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    const uint16_t encDeg = cmToEncDeg(cm);
    const uint8_t  rc =
        MiniR4.DriveDC.MoveGyroDegs(power, yaw, encDeg, brake, !blocking);
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
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    const uint8_t rc = MiniR4.DriveDC.MoveSyncTime(power, power, sec, brake, !blocking);
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
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    const uint8_t rc = MiniR4.DriveDC.MoveGyroTime(power, yaw, sec, brake, !blocking);
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

    const int16_t cur    = (int16_t) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    const int16_t target = _normalizeYaw((int32_t) cur + (int32_t) deg);

    // Capture the initial signed delta normalized into (-180, +180] so the BRAKE-edge
    // tick cannot oscillate the spin direction when the IMU briefly straddles target.
    int32_t deltaNorm = (int32_t) target - (int32_t) cur;
    while (deltaNorm > 180)
    {
        deltaNorm -= 360;
    }
    while (deltaNorm <= -180)
    {
        deltaNorm += 360;
    }

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

    const int8_t sign = (deltaNorm > 0) ? (int8_t) +1 : (int8_t) -1;

    _turnTargetYaw          = target;
    _turnDeltaSign          = sign;
    _turnMotorMode          = mode;
    _turnCruisePwm          = (power > 0) ? power : Config::TURN_CRUISE_PWM;
    _turnProfileBrakeOnDone = brake;
    _turnProfileState       = TurnProfileState::CRUISE;
    _turnDone               = false;
    _heldYaw                = target;

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

    const int16_t cur    = (int16_t) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    const int16_t target = _normalizeYaw((int32_t) yaw);

    int32_t deltaNorm = (int32_t) target - (int32_t) cur;
    while (deltaNorm > 180)
    {
        deltaNorm -= 360;
    }
    while (deltaNorm <= -180)
    {
        deltaNorm += 360;
    }

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

    const int8_t sign = (deltaNorm > 0) ? (int8_t) +1 : (int8_t) -1;

    _turnTargetYaw          = target;
    _turnDeltaSign          = sign;
    _turnMotorMode          = mode;
    _turnCruisePwm          = (power > 0) ? power : Config::TURN_CRUISE_PWM;
    _turnProfileBrakeOnDone = brake;
    _turnProfileState       = TurnProfileState::CRUISE;
    _turnDone               = false;
    _heldYaw                = target;

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
    return MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
}

void DriveClass::holdYaw()
{
    _heldYaw = (int16_t) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
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
        else
        {
            // Chained move: caller asked us to leave the chassis at endSpeed (typically
            // cruise) so the next queued move can pick up without a zero-velocity gap. The
            // last setpoint issued by `_profileSpeedAt(near 1.0)` was already at endSpeed —
            // leave it. No motor command, no brake.
            _moveDone = true;
            _turnDone = true;
        }
        _profileState = ProfileState::IDLE;
        return;
    }

    // Compute speed setpoint and issue the appropriate continuous primitive.
    const int16_t spd = _profileSpeedAt(frac);

    switch (_profileMode)
    {
    case DriveMode::RAW:
    {
        int16_t L = spd;
        int16_t R = spd;
        _applyBias(L, R);
        L = deadbandSnap(L);
        R = deadbandSnap(R);
        MiniR4.DriveDC.Move(L, R);
        break;
    }
    case DriveMode::SYNC:
        // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
        MiniR4.DriveDC.MoveSync(deadbandSnap(spd), deadbandSnap(spd));
        break;
    case DriveMode::GYRO:
        // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
        MiniR4.DriveDC.MoveGyro(deadbandSnap(spd), _profileHeading);
        break;
    }
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

    // Distance variant — same shape, units in encoder degrees.
    const int32_t traveled = (int32_t) (frac * (float) _profileTotalEnc);
    if (traveled < _profileAccelEnc)
    {
        const float r =
            _profileAccelEnc == 0 ? 1.0f : (float) traveled / (float) _profileAccelEnc;
        return (int16_t) (_profileStartSpd + (int16_t) ((_profilePeakSpd - _profileStartSpd) * r));
    }
    else if (_profileTotalEnc > _profileDecelEnc
             && traveled >= (_profileTotalEnc - _profileDecelEnc))
    {
        const int32_t into = traveled - (_profileTotalEnc - _profileDecelEnc);
        const float   r =
            _profileDecelEnc == 0 ? 1.0f : (float) into / (float) _profileDecelEnc;
        return (int16_t) (_profilePeakSpd + (int16_t) ((_profileEndSpd - _profilePeakSpd) * r));
    }
    return _profilePeakSpd;
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

    // Resolve per-side ramp distances (cm) with the no-ramp epsilon AND the [MIN, MAX] clamps.
    // Returns 0.0f on the no-ramp path (chained move already at cruise).
    auto resolveRamp = [&](int16_t endpoint) -> float
    {
        if (abs(cruise - endpoint) < Config::TSHAPE_NO_RAMP_EPSILON_PWM)
        {
            return 0.0f;
        }
        float raw = (float) abs(cruise - endpoint) / accel;
        if (raw < Config::TSHAPE_RAMP_MIN_CM)
        {
            raw = Config::TSHAPE_RAMP_MIN_CM;
        }
        if (raw > Config::TSHAPE_RAMP_MAX_CM)
        {
            raw = Config::TSHAPE_RAMP_MAX_CM;
        }
        return raw;
    };

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
            // move (resolveRamp returned 0.0f) honours the documented no-ramp invariant — even
            // when cruise itself is 0 (degenerate stand-still call). CRITICAL-1 fix.
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

        float accelMs = cmToMs(resolveRamp(startSpeed));
        float decelMs = cmToMs(resolveRamp(endSpeed));

        // Half-of-total cap — must not exceed half the move duration on either side.
        if (accelMs > total * 0.5f)
        {
            accelMs = total * 0.5f;
        }
        if (decelMs > total * 0.5f)
        {
            decelMs = total * 0.5f;
        }

        // Triangle fallback: if the two ramps don't fit, collapse the cruise plateau and
        // reduce the peak so the speed-vs-time integral still equals the cruise * total
        // requested. The simple averaged-peak heuristic keeps the math easy and bounded:
        // peak <= cruise always (clamp).
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

        _profileAccelMs = (uint32_t) accelMs;
        _profileDecelMs = (uint32_t) decelMs;
        _profileTotalMs = (uint32_t) total;
        _profileStartMs = millis();
    }
    else
    {
        // ---------------------------------------------------------------------------------
        // Distance variant — encoder-based ramps in cm, converted to encoder degrees via the
        // shared `cmToEncDeg` helper (WROMath.h). Negative `cruise` (backwards) is handled implicitly:
        // we store `_profileTotalEnc` / `_profileAccelEnc` / `_profileDecelEnc` as positive
        // magnitudes and `_updateProfile` compares them against `abs(encDelta)`. The PWM
        // setpoints (`_profileStartSpd` / `_profilePeakSpd` / `_profileEndSpd`) carry the
        // sign and are dispatched directly to the motors.
        // ---------------------------------------------------------------------------------
        float accelCm = resolveRamp(startSpeed);
        float decelCm = resolveRamp(endSpeed);

        if (accelCm > totalCm * 0.5f)
        {
            accelCm = totalCm * 0.5f;
        }
        if (decelCm > totalCm * 0.5f)
        {
            decelCm = totalCm * 0.5f;
        }

        if (accelCm + decelCm > totalCm)
        {
            int16_t peak = (int16_t) (((int32_t) startSpeed + (int32_t) endSpeed
                                       + (int32_t) cruise)
                                      / 3);
            if (peak > cruise && cruise >= 0)
            {
                peak = cruise;
            }
            if (peak < cruise && cruise < 0)
            {
                peak = cruise;
            }
            _profilePeakSpd = peak;

            if (accelCm + decelCm > 0.0f)
            {
                const float scale = totalCm / (accelCm + decelCm);
                accelCm *= scale;
                decelCm *= scale;
            }
        }
        else
        {
            _profilePeakSpd = cruise;
        }

        // Store ramp + total distances as positive magnitudes — `_updateProfile` and
        // `_profileSpeedAt` compare them against `abs(encDelta)` so the same code path
        // handles both forward and reverse moves transparently.
        _profileTotalEnc = (int32_t) cmToEncDeg(totalCm);
        _profileAccelEnc = (int32_t) cmToEncDeg(accelCm < 0.0f ? 0.0f : accelCm);
        _profileDecelEnc = (int32_t) cmToEncDeg(decelCm < 0.0f ? 0.0f : decelCm);
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

    // Read current IMU yaw + compute signed normalized delta into (-180, +180].
    const int16_t cur      = (int16_t) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    int32_t       rawDelta = (int32_t) _turnTargetYaw - (int32_t) cur;
    while (rawDelta > 180)
    {
        rawDelta -= 360;
    }
    while (rawDelta <= -180)
    {
        rawDelta += 360;
    }
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

    // Apply rotation direction. The captured `_turnDeltaSign` tells us which way to spin:
    //   +1 → yaw must increase (CCW / left turn).
    //   -1 → yaw must decrease (CW / right turn).
    //
    // Sign convention for the wheel pattern (per Drive's existing setSpeed semantics):
    // positive PWM on the left wheel drives the left side forward, which rotates the nose
    // RIGHT (clockwise body). To rotate the body CCW (yaw increases on this IMU) the left
    // wheel must spin BACKWARD and the right wheel FORWARD — hence the L = -sign*pwm,
    // R = +sign*pwm pattern for counter-rotation. If hardware testing reveals the IMU yaw
    // axis is inverted relative to this assumption, flip the signs below and update this
    // comment.
    int16_t L = 0;
    int16_t R = 0;
    switch (_turnMotorMode)
    {
    case 0:    // COUNTER_ROT — both wheels spin opposite directions.
        L = (int16_t) (-_turnDeltaSign * pwmMag);
        R = (int16_t) (+_turnDeltaSign * pwmMag);
        break;
    case 1:    // PIVOT_LEFT — left wheel stationary, right wheel drives the rotation.
        L = 0;
        R = (int16_t) (+_turnDeltaSign * pwmMag);
        break;
    case 2:    // PIVOT_RIGHT — right wheel stationary, left wheel drives the rotation.
        L = (int16_t) (-_turnDeltaSign * pwmMag);
        R = 0;
        break;
    default:
        // Unknown mode — fall back to counter-rotation rather than refusing to turn.
        L = (int16_t) (-_turnDeltaSign * pwmMag);
        R = (int16_t) (+_turnDeltaSign * pwmMag);
        break;
    }

    // Apply the same deadband snap / PWM saturation used by other drive outputs so
    // profiled turns neither stall below the minimum effective magnitude nor exceed
    // the configured PWM range at the actuator boundary.
    L = (int16_t) deadbandSnap(L);
    R = (int16_t) deadbandSnap(R);

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

    // Travel measured as an unsigned magnitude from the per-side baseline (M2, the canonical
    // encoder), matching `_updateProfile`. Forward and reverse both accumulate positive `traveled`
    // so the arm-window / max-distance comparisons need no sign juggling.
    const int32_t nowEnc   = MiniR4.M2.getDegrees();
    const float   traveled = encDegToCm((int32_t) abs(nowEnc - _stopStartEnc));

    // Emit a continuous PWM at the given positive magnitude through the mode-correct primitive,
    // applying the seek's travel direction (backward when `_stopReverse`). `_stopCruise` and every
    // magnitude passed here are positive — the builder carries direction in `_stopReverse`, never in
    // the power sign. RAW applies the motor bias + deadband snap (open loop); SYNC / GYRO do NOT —
    // the STM32 closes the loop and a feedforward scalar would fight its inner PID (mirrors
    // `_updateProfile`; see DECISIONS.md).
    auto driveContinuous = [&](int16_t mag)
    {
        const int16_t pwm = (int16_t) (_stopReverse ? -mag : mag);
        switch (_stopMode)
        {
        case DriveMode::RAW:
        {
            int16_t L = pwm;
            int16_t R = pwm;
            _applyBias(L, R);
            L = deadbandSnap(L);
            R = deadbandSnap(R);
            MiniR4.DriveDC.Move(L, R);
            break;
        }
        case DriveMode::SYNC:
            MiniR4.DriveDC.MoveSync(deadbandSnap(pwm), deadbandSnap(pwm));
            break;
        case DriveMode::GYRO:
            MiniR4.DriveDC.MoveGyro(deadbandSnap(pwm), _stopHeading);
            break;
        }
    };

    switch (_stopState)
    {
    case StopAtLineState::SEEK:
    {
        // Cruise flat toward the endpoint; arm detection once inside the window. The arm threshold
        // (`expectedCm - armWithin`, clamped at 0) lives in `stopAtLineArmed` so the SEEK → ARMED
        // decision is host-testable (TC11).
        driveContinuous(_stopCruise);
        if (stopAtLineArmed(traveled, _stopExpectedCm, _stopArmWithin))
        {
            _stopDetectStreak = 0;
            _stopState        = StopAtLineState::ARMED;
        }
        break;
    }

    case StopAtLineState::ARMED:
    {
        // Keep cruising while watching for the crossing. Read the junction classification DIRECTLY
        // from the line array (NOT the read-and-clear `Intersection::isAtIntersection()`), and step
        // the consecutive-sample debounce. Detection confirms when the streak reaches
        // `Config::STOP_AT_LINE_DETECT_TICKS`.
        driveContinuous(_stopCruise);

        const bool crossNow =
            (LineArray.getJunctionType() == STOP_AT_LINE_T_OR_CROSS_RAW);
        _stopDetectStreak =
            debounceStep(_stopDetectStreak, crossNow, Config::STOP_AT_LINE_DETECT_TICKS);

        if (_stopDetectStreak >= Config::STOP_AT_LINE_DETECT_TICKS)
        {
            // Confirmed line. Latch the result, anchor the overshoot roll on the current encoder,
            // and hand off to OVERSHOOT.
            _isLineFound   = true;
            _stopDetectEnc = nowEnc;
            _stopState     = StopAtLineState::OVERSHOOT;
            break;
        }

        // No line by the max search distance → fail fast. Stop, latch `LINE_NOT_FOUND`, clear the
        // queue, restore the front board. No silent dead-reckon fallback (spec / AC7).
        if (traveled >= _stopMaxCm)
        {
            Serial.println(F("[Drive] stopAtLine: no line within max distance — LINE_NOT_FOUND"));
            MiniR4.DriveDC.Move(0, 0);
            MiniR4.DriveDC.brake(true);

            _fault.kind    = Fault::LINE_NOT_FOUND;
            _fault.rc      = 0x00;
            _fault.cmdKind = DriveCommand::Kind::STOP_AT_LINE;
            _isLineFound   = false;

            LineArray.setReverseActive(false);    // restore front board
            _stopState = StopAtLineState::IDLE;
            _moveDone  = true;
            _turnDone  = true;
            _clearQueue();
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

    case StopAtLineState::DONE:
    {
        // Final stop. Active-brake (or coast) per `_stopBrake`, restore the front board, mark the
        // seek complete so `_dispatchNext` can hand off to the next queued command on this tick.
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(_stopBrake);

        LineArray.setReverseActive(false);    // restore front board (no-op in 1-board configs)
        _stopState = StopAtLineState::IDLE;
        _moveDone  = true;
        _turnDone  = true;
        break;
    }

    default:
        // IDLE handled above; no other state values exist.
        break;
    }
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

        // Select the rear board for a reverse seek so `getJunctionType()` samples the board that
        // faces the travel direction. No-op in 1-board configs (already rejected above), restored
        // to the front board at DONE/fault. LineFollower is disabled for the seek, so Drive briefly
        // owning the active-board selection is safe.
        LineArray.setReverseActive(cmd.reverse);

        _stopState = StopAtLineState::SEEK;
        _moveDone  = false;          // the seek owns _moveDone until DONE/fault.
        rc         = 0x00;           // seeding cannot fail at dispatch (no lower-MCU round trip).
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
           && _stopState == StopAtLineState::IDLE;
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
            // reverse seek may have left the rear board active. No-op in 1-board configs.
            if (_stopState != StopAtLineState::IDLE)
            {
                _stopState = StopAtLineState::IDLE;
                LineArray.setReverseActive(false);
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
