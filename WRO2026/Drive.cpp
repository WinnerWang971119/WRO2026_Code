/**
 * @file Drive.cpp
 * @brief Implementation of `DriveClass` — the movement-primitive singleton.
 * @author WRO 2026
 *
 * @note Build-time invariant: the wheel diameter must be positive — otherwise
 *       `_cmToEncDeg` would divide by zero or invert the travel direction silently.
 */

#include "Drive.h"

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

    return true;
}

void DriveClass::update()
{
    if (!_moveDone || !_turnDone)
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
}

// -----------------------------------------------------------------------------------------
// Continuous primitives
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::setSpeed(int16_t L, int16_t R)
{
    return MiniR4.DriveDC.Move(L, R);
}

uint8_t DriveClass::setSpeedSync(int16_t L, int16_t R)
{
    return MiniR4.DriveDC.MoveSync(L, R);
}

uint8_t DriveClass::forwardGyro(int16_t power)
{
    return MiniR4.DriveDC.MoveGyro(power, _heldYaw);
}

uint8_t DriveClass::stop(bool brake)
{
    MiniR4.DriveDC.Move(0, 0);
    const uint8_t brakeRc = MiniR4.DriveDC.brake(brake);

    _moveDone = true;
    _turnDone = true;

    return brakeRc;
}

// -----------------------------------------------------------------------------------------
// Distance-bounded primitives
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::driveDist(int16_t power, float cm)
{
    const uint16_t encDeg = _cmToEncDeg(cm);
    const uint8_t  rc     = MiniR4.DriveDC.MoveDegs(power, power, encDeg, Config::TURN_BRAKE, true);
    _moveDone             = false;
    return rc;
}

uint8_t DriveClass::driveDistSync(int16_t power, float cm)
{
    const uint16_t encDeg = _cmToEncDeg(cm);
    const uint8_t  rc =
        MiniR4.DriveDC.MoveSyncDegs(power, power, encDeg, Config::TURN_BRAKE, true);
    _moveDone = false;
    return rc;
}

uint8_t DriveClass::driveDistGyro(int16_t power, float cm)
{
    const uint16_t encDeg = _cmToEncDeg(cm);
    const uint8_t  rc =
        MiniR4.DriveDC.MoveGyroDegs(power, _heldYaw, encDeg, Config::TURN_BRAKE, true);
    _moveDone = false;
    return rc;
}

// -----------------------------------------------------------------------------------------
// Time-bounded primitives
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::driveTime(int16_t power, float sec)
{
    const uint8_t rc = MiniR4.DriveDC.MoveTime(power, power, sec, Config::TURN_BRAKE, true);
    _moveDone        = false;
    return rc;
}

uint8_t DriveClass::driveTimeSync(int16_t L, int16_t R, float sec)
{
    const uint8_t rc = MiniR4.DriveDC.MoveSyncTime(L, R, sec, Config::TURN_BRAKE, true);
    _moveDone        = false;
    return rc;
}

uint8_t DriveClass::driveTimeGyro(int16_t power, float sec)
{
    const uint8_t rc =
        MiniR4.DriveDC.MoveGyroTime(power, _heldYaw, sec, Config::TURN_BRAKE, true);
    _moveDone = false;
    return rc;
}

// -----------------------------------------------------------------------------------------
// Turns
// -----------------------------------------------------------------------------------------

uint8_t DriveClass::turnDeg(int16_t deg)
{
    const int16_t cur    = (int16_t) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    const int16_t target = _normalizeYaw((int32_t) cur + (int32_t) deg);

    const uint8_t rc =
        MiniR4.DriveDC.TurnGyro(Config::TURN_POWER, target, Config::TURN_MODE, Config::TURN_BRAKE, true);

    _heldYaw  = target;
    _turnDone = false;
    return rc;
}

uint8_t DriveClass::turnDegBlocking(int16_t deg)
{
    const int16_t cur    = (int16_t) MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
    const int16_t target = _normalizeYaw((int32_t) cur + (int32_t) deg);

    const uint8_t rc =
        MiniR4.DriveDC.TurnGyro(Config::TURN_POWER, target, Config::TURN_MODE, Config::TURN_BRAKE, false);

    _heldYaw  = target;
    _turnDone = true;    // library already blocked until the lower MCU reported done.
    return rc;
}

uint8_t DriveClass::turnToYaw(int16_t yaw)
{
    const int16_t target = _normalizeYaw((int32_t) yaw);

    const uint8_t rc =
        MiniR4.DriveDC.TurnGyro(Config::TURN_POWER, target, Config::TURN_MODE, Config::TURN_BRAKE, true);

    _heldYaw  = target;
    _turnDone = false;
    return rc;
}

uint8_t DriveClass::turnToYawBlocking(int16_t yaw)
{
    const int16_t target = _normalizeYaw((int32_t) yaw);

    const uint8_t rc =
        MiniR4.DriveDC.TurnGyro(Config::TURN_POWER, target, Config::TURN_MODE, Config::TURN_BRAKE, false);

    _heldYaw  = target;
    _turnDone = true;
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
// Private helpers
// -----------------------------------------------------------------------------------------

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
