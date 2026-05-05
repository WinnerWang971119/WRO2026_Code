/**
 * @file Drive.cpp
 * @brief Implementation of `DriveClass` — the movement-primitive singleton.
 * @author WRO 2026
 *
 * @note Build-time invariant: the wheel diameter must be positive — otherwise
 *       `_cmToEncDeg` would divide by zero or invert the travel direction silently.
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

void DriveClass::update()
{
    // -------------------------------------------------------------------------------------
    // Stage 1 — drain any in-flight bounded task. When a profile is in flight, the profile
    // owns `_moveDone` — the lower-MCU task-done bit refers to a *prior* bounded MMLower
    // task and would falsely flip _moveDone to true mid-cruise (W1 fix). Skip the drain in
    // that case; _updateProfile will set _moveDone when the profile itself completes.
    // -------------------------------------------------------------------------------------
    if (_profileState == ProfileState::IDLE && (!_moveDone || !_turnDone))
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

    _moveDone     = true;
    _turnDone     = true;
    _profileState = ProfileState::IDLE;    // also abort any in-flight profile

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

    const uint16_t encDeg = _cmToEncDeg(cm);
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
    const uint16_t encDeg = _cmToEncDeg(cm);
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
    const uint16_t encDeg = _cmToEncDeg(cm);
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
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    const int16_t cur          = (int16_t) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    const int16_t target       = _normalizeYaw((int32_t) cur + (int32_t) deg);
    const bool    wasIdleAtTry = _turnDone;

    // The library's `async` argument is the inverse of our `blocking` flag.
    const uint8_t rc =
        MiniR4.DriveDC.TurnGyro(power, target, mode, brake, !blocking);

    if (rc == 0x00)
    {
        _heldYaw = target;
        // Async path: clear `_turnDone` so callers can poll. Blocking path: the library
        // already spun until the lower MCU reported done, so leave `_turnDone = true`.
        _turnDone = blocking ? true : false;
    }
    else
    {
        // Turn never started — leave _heldYaw at the previous target so forwardGyro/*Gyro
        // primitives keep their existing yaw lock. C1 fix: unblock any caller polling
        // isTurnDone() — but ONLY if no prior bounded task was pending at entry. If a
        // prior task is still in flight, we must not lie about its completion (W1).
        if (wasIdleAtTry)
        {
            _turnDone = true;
        }
        Serial.print(F("[Drive] _execTurnDeg failed, rc=0x"));
        Serial.println(rc, HEX);
    }
    return rc;
}

uint8_t DriveClass::_execTurnToYaw(int16_t yaw,
                                   int16_t power,
                                   uint8_t mode,
                                   bool    brake,
                                   bool    blocking)
{
    // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
    const int16_t target       = _normalizeYaw((int32_t) yaw);
    const bool    wasIdleAtTry = _turnDone;

    const uint8_t rc =
        MiniR4.DriveDC.TurnGyro(power, target, mode, brake, !blocking);

    if (rc == 0x00)
    {
        _heldYaw  = target;
        _turnDone = blocking ? true : false;
    }
    else
    {
        // Turn never started. C1 fix: unblock the caller — but only if the slot was
        // actually idle at entry (W1: don't clobber a pending prior task's flag).
        if (wasIdleAtTry)
        {
            _turnDone = true;
        }
        Serial.print(F("[Drive] _execTurnToYaw failed, rc=0x"));
        Serial.println(rc, HEX);
    }
    return rc;
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

int16_t DriveClass::_presetToPwm(Config::SpeedPreset preset) const
{
    switch (preset)
    {
    case Config::SpeedPreset::SLOW:
        return Config::SPEED_SLOW;
    case Config::SpeedPreset::NORMAL:
        return Config::SPEED_NORMAL;
    case Config::SpeedPreset::FAST:
        return Config::SPEED_FAST;
    }
    // Defensive fallback — `enum class` should keep the switch exhaustive at compile time.
    return Config::SPEED_NORMAL;
}

int16_t DriveClass::_normalizeYaw(int32_t deg)
{
    // C++11+ truncating modulo: the result is always in (-360, +360) for any int32 input,
    // so a single `% 360` is sufficient — no further clamping required.
    return (int16_t) (deg % 360);
}

uint16_t DriveClass::_cmToEncDeg(float cm)
{
    // cm * 10 -> mm; divide by wheel circumference (PI * diameter_mm); multiply by 360 deg.
    const float revs = (cm * 10.0f) / ((float) PI * Config::WHEEL_DIAM_MM);
    return (uint16_t) (revs * 360.0f);
}

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
        MiniR4.DriveDC.Move(L, R);
        break;
    }
    case DriveMode::SYNC:
        // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
        MiniR4.DriveDC.MoveSync(spd, spd);
        break;
    case DriveMode::GYRO:
        // No _applyBias here — STM32 closes the loop. See DECISIONS.md.
        MiniR4.DriveDC.MoveGyro(spd, _profileHeading);
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
    // underflow into _cmToEncDeg's uint16_t cast (or produce a negative ramp duration), so we
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
        // shared `_cmToEncDeg` helper. Negative `cruise` (backwards) is handled implicitly:
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
        _profileTotalEnc = (int32_t) _cmToEncDeg(totalCm);
        _profileAccelEnc = (int32_t) _cmToEncDeg(accelCm < 0.0f ? 0.0f : accelCm);
        _profileDecelEnc = (int32_t) _cmToEncDeg(decelCm < 0.0f ? 0.0f : decelCm);
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
    switch (cmd.kind)
    {
    case DriveCommand::Kind::DRIVE_DIST:
        switch (cmd.mode)
        {
        case DriveMode::RAW:
            _execDriveDistRaw(cmd.power, cmd.cm, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::SYNC:
            _execDriveDistSync(cmd.power, cmd.cm, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::GYRO:
            _execDriveDistGyro(cmd.power, cmd.cm, cmd.heading, cmd.brake, /*blocking*/ false);
            break;
        }
        break;

    case DriveCommand::Kind::DRIVE_TIME:
        switch (cmd.mode)
        {
        case DriveMode::RAW:
            _execDriveTimeRaw(cmd.power, cmd.sec, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::SYNC:
            _execDriveTimeSync(cmd.power, cmd.sec, cmd.brake, /*blocking*/ false);
            break;
        case DriveMode::GYRO:
            _execDriveTimeGyro(cmd.power, cmd.sec, cmd.heading, cmd.brake, /*blocking*/ false);
            break;
        }
        break;

    case DriveCommand::Kind::TURN_DEG:
        _execTurnDeg(cmd.deg, cmd.power, cmd.turnMode, cmd.brake, /*blocking*/ false);
        break;

    case DriveCommand::Kind::TURN_TO_YAW:
        _execTurnToYaw(cmd.yaw, cmd.power, cmd.turnMode, cmd.brake, /*blocking*/ false);
        break;

    case DriveCommand::Kind::PROFILE_DIST:
        _execProfileMove(cmd.mode,
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
        _execProfileMove(cmd.mode,
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
    }
}

bool DriveClass::_isIdle() const
{
    return _moveDone && _turnDone && _profileState == ProfileState::IDLE;
}

void DriveClass::_drainBlocking()
{
    // Allowed `delay()` site #3. The blocking caller is by definition off the cooperative
    // tick chain (synchronous-orchestration path), so this spin doesn't violate the iron
    // rule — which targets non-blocking `update()` chains, not orchestration.
    while (!_isIdle() || _qCount > 0)
    {
        update();
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
