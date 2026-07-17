/**
 * @file MatrixMiniR4.h (HOST HAL MOCK)
 * @brief Unified host-test mock of the MatrixMiniR4 vendor facade — the single
 *        `MiniR4` global the host-test harness links production `.cpp` against.
 * @author WRO 2026
 *
 * @details
 * This is the shared host-test HAL facade (epic #107 / issue #105). It shadows the
 * real `<MatrixMiniR4.h>` (angle-included, resolved by include-path order — the only
 * include dir of the `test_mock_hal` target is `hal_mocks/`). The include guard is the
 * REAL production guard (`MATRIXMINIR4_H`), so if the vendor header is ever force-included
 * first, this mock no-ops it (the guard-shadow trick, mirroring `ctps_mocks/`).
 *
 * The facade exposes EVERY actuator/sensor family the production `Drive.cpp` and
 * `Mission.cpp` reach through `MiniR4.*`, with vendor-faithful signatures verified
 * against the `MatrixMiniR4/src/Modules/` headers:
 *   - `DriveDC` (`MiniR4DriveDC<1>` → non-template `MockDriveDC`): the full Move/MoveSync/
 *     MoveGyro/MoveDegs/MoveSyncDegs/MoveGyroDegs/MoveTime/MoveSyncTime/MoveGyroTime/brake/
 *     begin/setMove*PID/setTurnGyroPID/isPrevTaskDone/resetCounter/getDegrees surface.
 *   - `M1`–`M4` (`MiniR4DC<ID>` → `MockDC`): getDegrees/setReverse/setPPR_RPM/... .
 *   - `RC1`–`RC4` (`MiniR4RC<ID>` → `MockRC`): begin/setHWDir/setAngle.
 *   - `LED` (`MiniR4LED` → `MockLED`): setColor(idx,r,g,b) + packed setColor(idx,rgb) + setBrightness.
 *   - `Buzzer` (`MiniR4BUZZER` → `MockBuzzer`): Tone(freq,ms) + NoTone().
 *   - `BTN_UP`/`BTN_DOWN` (`MiniR4BTN<ID>` → `MockBTN<ID>`): getState().
 *   - `OLED` (`Adafruit_SSD1306` → `MockOLED`): the Adafruit text surface, all no-ops.
 *   - `PWR` (`MiniR4Power` → `MockPower`): setBattCell(cell).
 *   - `mmL` (lower-MCU IMU `MMLower` → `MockMMLower`) + `namespace MMLower { enum class RESULT }`.
 *
 * SUPERSET-CARRY INVARIANT (T1 ↔ T7):
 *   `MockDC`, `MockRC`, `MockMMLower`, and `namespace MMLower` are carried TYPE-IDENTICALLY
 *   from the legacy `mocks/MatrixMiniR4.h` (same class names, member names, method signatures),
 *   so this `MatrixMiniR4` facade is a STRICT SUPERSET of the old `MockMatrixMiniR4`. A later
 *   task replaces `mocks/MatrixMiniR4.h` with a forwarding shim over this file, and every
 *   existing facade-consuming test (manipulator, localization, yaw_source) must stay green.
 *   Do NOT change any carried member's name or signature.
 *
 * ---------------------------------------------------------------------------
 * Fidelity — where this mock can lie vs real hardware
 * ---------------------------------------------------------------------------
 * This is a THIN record-and-test-set mock. It deliberately models nothing the host
 * cannot observe; treat the following as places it diverges from a real MiniR4:
 *   - Faults it never raises on its own: every method reports success
 *     (`begin()`→0x00/true, `setColor`→true, `setBattCell`→true, PID setters→true).
 *     The real lower-MCU calls can fail, retry, and time out; this mock cannot — a
 *     fixture must inject any failure it wants to test via the `mock*` setters.
 *   - Timing it ignores: the real blocking moves (`MoveDegs/...Time` with `async==false`)
 *     spin on an STM32 done-bit and may add a brake delay; this mock returns IMMEDIATELY
 *     and performs NO multi-tick completion / done-bit evolution (explicitly rejected —
 *     `isPrevTaskDone()` returns a plain test-settable field, it never "eventually" flips).
 *   - Encoder/IMU state is whatever a test set: `getDegrees()` returns the backing field;
 *     `mmL.GetIMUEuler` returns `mockYawDeg`. No physics, no integration, no drift.
 *   - Single-call vendor CONTRACTS it DOES honor (observation, not logic):
 *       * `MockDriveDC::resetCounter()` and `MockDC::resetCounter()` ZERO their degrees
 *         backing, exactly as the real encoder reset does.
 *       * `MockMMLower::GetIMUEuler` leaves its out-params UNTOUCHED on a non-OK result,
 *         matching the vendor contract `Localization::_readImuYaw` depends on.
 */

#ifndef MATRIXMINIR4_H
#define MATRIXMINIR4_H

#include <stdint.h>
#include <math.h>    // lround() for the MatrixColor mmL.mockYawDeg -> gyro-bytes bridge (T3)

// ===========================================================================
// MockRC — single RC servo channel. Mirror of `MiniR4RC<ID>` (vendor MiniR4RC.h).
// CARRIED VERBATIM from `mocks/MatrixMiniR4.h` (superset-carry invariant).
// ===========================================================================

/**
 * @brief Host mock for a single RC servo channel.
 *
 * Not a template — the vendor `MiniR4RC<ID>` carries its `ID` only to address the
 * lower MCU, which the host mock never touches. A plain class with per-instance
 * recording fields is sufficient and keeps the mock memory-light.
 */
class MockRC
{
public:
    MockRC() = default;

    /** @brief Mirrors `MiniR4RC::begin()`; records the call, always succeeds. */
    bool begin()
    {
        beginCallCount++;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4RC::setHWDir(bool)`; records the hardware-direction flag.
     * @param dir Hardware direction flag (true = forward, false = reverse).
     * @return Always true on host.
     */
    bool setHWDir(bool dir)
    {
        setHWDirCallCount++;
        lastHWDir = dir;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4RC::setAngle(uint16_t)`; records the commanded angle.
     * @param angle Commanded servo angle (degrees).
     * @return Always true on host.
     */
    bool setAngle(uint16_t angle)
    {
        setAngleCallCount++;
        lastAngle = angle;
        return true;
    }

    /** @brief Clear all recorded state. Called by `mockResetMiniR4()`. */
    void mockReset()
    {
        beginCallCount    = 0;
        setHWDirCallCount = 0;
        setAngleCallCount = 0;
        lastHWDir         = false;
        lastAngle         = 0;
    }

    // ---- Test-visible recorded state ----------------------------------------
    uint32_t beginCallCount    = 0;
    uint32_t setHWDirCallCount = 0;
    uint32_t setAngleCallCount = 0;
    bool     lastHWDir         = false;
    uint16_t lastAngle         = 0;
};

// ===========================================================================
// MockDC — single DC motor channel with encoder. Mirror of `MiniR4DC<ID>`
// (vendor MiniR4DC.h). CARRIED VERBATIM from `mocks/MatrixMiniR4.h`
// (superset-carry invariant). `M2`/`M3.getDegrees()` is the canonical encoder
// `Drive.cpp` reads every tick.
// ===========================================================================

/**
 * @brief Host mock for a single DC motor channel with encoder.
 *
 * Not a template, for the same reason as `MockRC`: the vendor `MiniR4DC<ID>` uses its
 * `ID` only to address the lower MCU. The mock backs `getDegrees()` with a settable
 * field so a fixture can script belt position, and `resetCounter()` zeroes it exactly
 * as the real `SetEncoderResetCounter` would.
 */
class MockDC
{
public:
    MockDC() = default;

    /** @brief Mirrors `MiniR4DC::begin()`; records the call, always succeeds. */
    bool begin()
    {
        beginCallCount++;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DC::setPPR_RPM(uint16_t,uint16_t)`; records both args.
     * @param motorPPR    Encoder pulses per revolution.
     * @param motorMaxRPM Motor maximum RPM.
     * @return Always true on host.
     */
    bool setPPR_RPM(uint16_t motorPPR, uint16_t motorMaxRPM)
    {
        setPPR_RPMCallCount++;
        lastPPR    = motorPPR;
        lastMaxRPM = motorMaxRPM;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DC::setReverse(bool)`; records the direction flag.
     * @param dir true = reverse, false = forward.
     * @return Always true on host.
     */
    bool setReverse(bool dir)
    {
        setReverseCallCount++;
        lastReverse = dir;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DC::setSpeed(int16_t)`; records the last speed.
     * @param speed Commanded speed (-100..100).
     * @return Always true on host.
     */
    bool setSpeed(int16_t speed)
    {
        setSpeedCallCount++;
        lastSpeed = speed;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DC::rotateFor(int16_t,uint16_t)`; records speed+degree.
     * @param speed  Rotation speed.
     * @param degree Rotation amount (degrees).
     * @return Always true on host.
     */
    bool rotateFor(int16_t speed, uint16_t degree)
    {
        rotateForCallCount++;
        lastRotateSpeed  = speed;
        lastRotateDegree = degree;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DC::getDegrees()`; returns the test-settable backing value.
     * @return The current mock encoder degrees.
     */
    int32_t getDegrees()
    {
        getDegreesCallCount++;
        return _degrees;
    }

    /**
     * @brief Mirrors `MiniR4DC::resetCounter()`; ZEROES the `getDegrees()` backing value.
     * @return Always true on host.
     */
    bool resetCounter()
    {
        resetCounterCallCount++;
        _degrees = 0;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DC::setBrake(bool)`; records the brake flag.
     * @param brake true = brake, false = coast.
     * @return Always true on host.
     */
    bool setBrake(bool brake)
    {
        setBrakeCallCount++;
        lastBrake = brake;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DC::ChkRotateEnd(bool&)`; writes `mockRotateEnded` out.
     * @param isEnd Out-param set to the test-controlled rotate-end flag.
     * @return Always true on host (the underlying read "succeeded").
     */
    bool ChkRotateEnd(bool& isEnd)
    {
        chkRotateEndCallCount++;
        isEnd = mockRotateEnded;
        return true;
    }

    // ---- Test setters --------------------------------------------------------

    /// Set the value `getDegrees()` returns (until `resetCounter()` zeroes it).
    void mockSetDegrees(int32_t degs) { _degrees = degs; }

    /// Advance the mock encoder by a signed delta (simulate belt travel between ticks).
    void mockAdvanceDegrees(int32_t delta) { _degrees += delta; }

    /** @brief Clear all recorded state + reset the encoder backing + rotate-end flag. */
    void mockReset()
    {
        beginCallCount        = 0;
        setPPR_RPMCallCount   = 0;
        setReverseCallCount   = 0;
        setSpeedCallCount     = 0;
        rotateForCallCount    = 0;
        getDegreesCallCount   = 0;
        resetCounterCallCount = 0;
        setBrakeCallCount     = 0;
        chkRotateEndCallCount = 0;
        lastPPR          = 0;
        lastMaxRPM       = 0;
        lastReverse      = false;
        lastSpeed        = 0;
        lastRotateSpeed  = 0;
        lastRotateDegree = 0;
        lastBrake        = false;
        _degrees         = 0;
        mockRotateEnded  = false;
    }

    // ---- Test-visible recorded state ----------------------------------------
    uint32_t beginCallCount        = 0;
    uint32_t setPPR_RPMCallCount   = 0;
    uint32_t setReverseCallCount   = 0;
    uint32_t setSpeedCallCount     = 0;
    uint32_t rotateForCallCount    = 0;
    uint32_t getDegreesCallCount   = 0;
    uint32_t resetCounterCallCount = 0;
    uint32_t setBrakeCallCount     = 0;
    uint32_t chkRotateEndCallCount = 0;

    uint16_t lastPPR          = 0;
    uint16_t lastMaxRPM       = 0;
    bool     lastReverse      = false;
    int16_t  lastSpeed        = 0;
    int16_t  lastRotateSpeed  = 0;
    uint16_t lastRotateDegree = 0;
    bool     lastBrake        = false;

    /// Value `ChkRotateEnd(bool&)` writes into its out-param. Default false (rotating).
    bool mockRotateEnded = false;

private:
    /// Backing value for `getDegrees()`; zeroed by `resetCounter()`.
    int32_t _degrees = 0;
};

// ===========================================================================
// MockDriveDC — differential-drive base. Mirror of `MiniR4DriveDC<1>`
// (vendor MiniR4DriveDC.h). NON-TEMPLATE: the vendor's `<1>` only addresses the
// lower MCU, which the host never touches — drop it, exactly as MockRC/MockDC do.
//
// Vendor-faithful return types (verified against MiniR4DriveDC.h):
//   begin           -> uint8_t (0x00 OK)
//   setMove*PID      -> bool
//   Move/MoveSync/MoveGyro/Move*Degs/Move*Time/brake/resetCounter -> uint8_t (0x00 OK)
//   isPrevTaskDone   -> bool
//   getDegrees       -> int32_t
// The `async` arg keeps the vendor's `= false` default so a call that omits it resolves.
//
// THIN: records args into test-visible fields; completion / done-bit / encoder are
// PLAIN TEST-SETTABLE FIELDS — NO behavioral modeling (rejected by debate).
// ===========================================================================

/**
 * @brief Host mock for the differential-drive base (`MiniR4.DriveDC`).
 *
 * Records the last-commanded arguments of every move verb and bumps per-method call
 * counters. `isPrevTaskDone()` returns a test-settable flag (default `true`, matching the
 * common "task already finished" host case); `getDegrees()` returns a test-settable
 * backing the `resetCounter()` single-call contract zeroes. Nothing here evolves on its
 * own across ticks — a fixture scripts every observable.
 */
class MockDriveDC
{
public:
    MockDriveDC() = default;

    /**
     * @brief Mirrors `MiniR4DriveDC::begin(uint8_t,uint8_t,bool,bool)`.
     * @param leftMotorID   Left drive-motor ID.
     * @param rightMotorID  Right drive-motor ID.
     * @param isLeftReverse  Left encoder reverse flag.
     * @param isRightReverse Right encoder reverse flag.
     * @return 0x00 (OK) on host.
     */
    uint8_t begin(uint8_t leftMotorID, uint8_t rightMotorID, bool isLeftReverse,
                  bool isRightReverse)
    {
        beginCallCount++;
        lastBeginLeftID      = leftMotorID;
        lastBeginRightID     = rightMotorID;
        lastBeginLeftReverse  = isLeftReverse;
        lastBeginRightReverse = isRightReverse;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::setMoveSyncPID(float,float,float)`; records the gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @return Always true on host.
     */
    bool setMoveSyncPID(float kp, float ki, float kd)
    {
        setMoveSyncPIDCallCount++;
        lastMoveSyncKp = kp;
        lastMoveSyncKi = ki;
        lastMoveSyncKd = kd;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::setMoveGyroPID(float,float,float)`; records the gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @return Always true on host.
     */
    bool setMoveGyroPID(float kp, float ki, float kd)
    {
        setMoveGyroPIDCallCount++;
        lastMoveGyroKp = kp;
        lastMoveGyroKi = ki;
        lastMoveGyroKd = kd;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::setTurnGyroPID(float,float,float)`; records the gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @return Always true on host.
     */
    bool setTurnGyroPID(float kp, float ki, float kd)
    {
        setTurnGyroPIDCallCount++;
        lastTurnGyroKp = kp;
        lastTurnGyroKi = ki;
        lastTurnGyroKd = kd;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::Move(int16_t,int16_t)`; records the L/R power.
     * @param power_left  Left motor power (-100..100).
     * @param power_right Right motor power (-100..100).
     * @return 0x00 (OK) on host.
     */
    uint8_t Move(int16_t power_left, int16_t power_right)
    {
        moveCallCount++;
        lastMoveL = power_left;
        lastMoveR = power_right;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveSync(int16_t,int16_t)`; records the L/R power.
     * @param power_left  Left motor power (-100..100).
     * @param power_right Right motor power (-100..100).
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveSync(int16_t power_left, int16_t power_right)
    {
        moveSyncCallCount++;
        lastMoveSyncL = power_left;
        lastMoveSyncR = power_right;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveGyro(int16_t,int16_t)`; records power + target.
     * @param power  Drive power (-100..100).
     * @param Target Target IMU yaw (degrees).
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveGyro(int16_t power, int16_t Target)
    {
        moveGyroCallCount++;
        lastMoveGyroPower  = power;
        lastMoveGyroTarget = Target;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveDegs(int16_t,int16_t,uint16_t,bool,bool)`.
     * @param power_left  Left motor power (-100..100).
     * @param power_right Right motor power (-100..100).
     * @param degree      Target encoder degrees.
     * @param brake       true = brake, false = coast.
     * @param async       true = non-blocking, false = blocking.
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveDegs(int16_t power_left, int16_t power_right, uint16_t degree, bool brake,
                     bool async = false)
    {
        moveDegsCallCount++;
        lastMoveDegsL     = power_left;
        lastMoveDegsR     = power_right;
        lastMoveDegsDeg   = degree;
        lastMoveDegsBrake = brake;
        lastMoveDegsAsync = async;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveSyncDegs(int16_t,int16_t,uint16_t,bool,bool)`.
     * @param power_left  Left motor power (-100..100).
     * @param power_right Right motor power (-100..100).
     * @param degree      Target encoder degrees.
     * @param brake       true = brake, false = coast.
     * @param async       true = non-blocking, false = blocking.
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveSyncDegs(int16_t power_left, int16_t power_right, uint16_t degree, bool brake,
                         bool async = false)
    {
        moveSyncDegsCallCount++;
        lastMoveSyncDegsL     = power_left;
        lastMoveSyncDegsR     = power_right;
        lastMoveSyncDegsDeg   = degree;
        lastMoveSyncDegsBrake = brake;
        lastMoveSyncDegsAsync = async;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveGyroDegs(int16_t,int16_t,uint16_t,bool,bool)`.
     * @param power_X  Drive power (-100..100).
     * @param Target_D Target IMU yaw (degrees).
     * @param degree   Target encoder degrees.
     * @param brake    true = brake, false = coast.
     * @param async    true = non-blocking, false = blocking.
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveGyroDegs(int16_t power_X, int16_t Target_D, uint16_t degree, bool brake,
                         bool async = false)
    {
        moveGyroDegsCallCount++;
        lastMoveGyroDegsPower  = power_X;
        lastMoveGyroDegsTarget = Target_D;
        lastMoveGyroDegsDeg    = degree;
        lastMoveGyroDegsBrake  = brake;
        lastMoveGyroDegsAsync  = async;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveTime(int16_t,int16_t,float,bool,bool)`.
     * @param power_left  Left motor power (-100..100).
     * @param power_right Right motor power (-100..100).
     * @param Time_S      Run duration (seconds).
     * @param brake       true = brake, false = coast.
     * @param async       true = non-blocking, false = blocking.
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveTime(int16_t power_left, int16_t power_right, float Time_S, bool brake,
                     bool async = false)
    {
        moveTimeCallCount++;
        lastMoveTimeL     = power_left;
        lastMoveTimeR     = power_right;
        lastMoveTimeSec   = Time_S;
        lastMoveTimeBrake = brake;
        lastMoveTimeAsync = async;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveSyncTime(int16_t,int16_t,float,bool,bool)`.
     * @param power_left  Left motor power (-100..100).
     * @param power_right Right motor power (-100..100).
     * @param Time_S      Run duration (seconds).
     * @param brake       true = brake, false = coast.
     * @param async       true = non-blocking, false = blocking.
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveSyncTime(int16_t power_left, int16_t power_right, float Time_S, bool brake,
                         bool async = false)
    {
        moveSyncTimeCallCount++;
        lastMoveSyncTimeL     = power_left;
        lastMoveSyncTimeR     = power_right;
        lastMoveSyncTimeSec   = Time_S;
        lastMoveSyncTimeBrake = brake;
        lastMoveSyncTimeAsync = async;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::MoveGyroTime(int16_t,int16_t,float,bool,bool)`.
     * @param power  Drive power (-100..100).
     * @param Target Target IMU yaw (degrees).
     * @param Time_S Run duration (seconds).
     * @param brake  true = brake, false = coast.
     * @param async  true = non-blocking, false = blocking.
     * @return 0x00 (OK) on host.
     */
    uint8_t MoveGyroTime(int16_t power, int16_t Target, float Time_S, bool brake,
                         bool async = false)
    {
        moveGyroTimeCallCount++;
        lastMoveGyroTimePower  = power;
        lastMoveGyroTimeTarget = Target;
        lastMoveGyroTimeSec    = Time_S;
        lastMoveGyroTimeBrake  = brake;
        lastMoveGyroTimeAsync  = async;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::brake(bool)`; records the brake type.
     * @param brakeType true = brake, false = coast.
     * @return 0x00 (OK) on host.
     */
    uint8_t brake(bool brakeType)
    {
        brakeCallCount++;
        lastBrake = brakeType;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::resetCounter()`; ZEROES the `getDegrees()` backing.
     *
     * Single-call vendor contract honored: a real encoder reset zeroes the degrees, so
     * the mock does too (observation, not behavioral modeling).
     *
     * @return 0x00 (OK) on host.
     */
    uint8_t resetCounter()
    {
        resetCounterCallCount++;
        _degrees = 0;
        return 0x00;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::isPrevTaskDone()`; returns the test-settable flag.
     *
     * Defaults to `true`. This NEVER evolves on its own — multi-tick done-bit modeling was
     * rejected; a fixture sets it via `mockSetPrevTaskDone()`.
     *
     * @return `mockPrevTaskDone` (default `true`).
     */
    bool isPrevTaskDone()
    {
        isPrevTaskDoneCallCount++;
        return mockPrevTaskDone;
    }

    /**
     * @brief Mirrors `MiniR4DriveDC::getDegrees()`; returns the test-settable backing value.
     * @return The current mock drive-base encoder degrees.
     */
    int32_t getDegrees()
    {
        getDegreesCallCount++;
        return _degrees;
    }

    // ---- Test setters --------------------------------------------------------

    /// Set the flag `isPrevTaskDone()` returns (default `true`).
    void mockSetPrevTaskDone(bool done) { mockPrevTaskDone = done; }

    /// Set the value `getDegrees()` returns (until `resetCounter()` zeroes it).
    void mockSetDegrees(int32_t degs) { _degrees = degs; }

    /** @brief Restore every recorded field, call counter, and test-settable backing. */
    void mockReset()
    {
        beginCallCount          = 0;
        setMoveSyncPIDCallCount = 0;
        setMoveGyroPIDCallCount = 0;
        setTurnGyroPIDCallCount = 0;
        moveCallCount           = 0;
        moveSyncCallCount       = 0;
        moveGyroCallCount       = 0;
        moveDegsCallCount       = 0;
        moveSyncDegsCallCount   = 0;
        moveGyroDegsCallCount   = 0;
        moveTimeCallCount       = 0;
        moveSyncTimeCallCount   = 0;
        moveGyroTimeCallCount   = 0;
        brakeCallCount          = 0;
        resetCounterCallCount   = 0;
        isPrevTaskDoneCallCount = 0;
        getDegreesCallCount     = 0;

        lastBeginLeftID       = 0;
        lastBeginRightID      = 0;
        lastBeginLeftReverse  = false;
        lastBeginRightReverse = false;

        lastMoveSyncKp = 0.0f;
        lastMoveSyncKi = 0.0f;
        lastMoveSyncKd = 0.0f;
        lastMoveGyroKp = 0.0f;
        lastMoveGyroKi = 0.0f;
        lastMoveGyroKd = 0.0f;
        lastTurnGyroKp = 0.0f;
        lastTurnGyroKi = 0.0f;
        lastTurnGyroKd = 0.0f;

        lastMoveL          = 0;
        lastMoveR          = 0;
        lastMoveSyncL      = 0;
        lastMoveSyncR      = 0;
        lastMoveGyroPower  = 0;
        lastMoveGyroTarget = 0;

        lastMoveDegsL     = 0;
        lastMoveDegsR     = 0;
        lastMoveDegsDeg   = 0;
        lastMoveDegsBrake = false;
        lastMoveDegsAsync = false;

        lastMoveSyncDegsL     = 0;
        lastMoveSyncDegsR     = 0;
        lastMoveSyncDegsDeg   = 0;
        lastMoveSyncDegsBrake = false;
        lastMoveSyncDegsAsync = false;

        lastMoveGyroDegsPower  = 0;
        lastMoveGyroDegsTarget = 0;
        lastMoveGyroDegsDeg    = 0;
        lastMoveGyroDegsBrake  = false;
        lastMoveGyroDegsAsync  = false;

        lastMoveTimeL     = 0;
        lastMoveTimeR     = 0;
        lastMoveTimeSec   = 0.0f;
        lastMoveTimeBrake = false;
        lastMoveTimeAsync = false;

        lastMoveSyncTimeL     = 0;
        lastMoveSyncTimeR     = 0;
        lastMoveSyncTimeSec   = 0.0f;
        lastMoveSyncTimeBrake = false;
        lastMoveSyncTimeAsync = false;

        lastMoveGyroTimePower  = 0;
        lastMoveGyroTimeTarget = 0;
        lastMoveGyroTimeSec    = 0.0f;
        lastMoveGyroTimeBrake  = false;
        lastMoveGyroTimeAsync  = false;

        lastBrake        = false;
        _degrees         = 0;
        mockPrevTaskDone = true;
    }

    // ---- Test-visible call counters -----------------------------------------
    uint32_t beginCallCount          = 0;
    uint32_t setMoveSyncPIDCallCount = 0;
    uint32_t setMoveGyroPIDCallCount = 0;
    uint32_t setTurnGyroPIDCallCount = 0;
    uint32_t moveCallCount           = 0;
    uint32_t moveSyncCallCount       = 0;
    uint32_t moveGyroCallCount       = 0;
    uint32_t moveDegsCallCount       = 0;
    uint32_t moveSyncDegsCallCount   = 0;
    uint32_t moveGyroDegsCallCount   = 0;
    uint32_t moveTimeCallCount       = 0;
    uint32_t moveSyncTimeCallCount   = 0;
    uint32_t moveGyroTimeCallCount   = 0;
    uint32_t brakeCallCount          = 0;
    uint32_t resetCounterCallCount   = 0;
    uint32_t isPrevTaskDoneCallCount = 0;
    uint32_t getDegreesCallCount     = 0;

    // ---- Test-visible last-arg captures -------------------------------------
    uint8_t lastBeginLeftID       = 0;
    uint8_t lastBeginRightID      = 0;
    bool    lastBeginLeftReverse  = false;
    bool    lastBeginRightReverse = false;

    float lastMoveSyncKp = 0.0f;
    float lastMoveSyncKi = 0.0f;
    float lastMoveSyncKd = 0.0f;
    float lastMoveGyroKp = 0.0f;
    float lastMoveGyroKi = 0.0f;
    float lastMoveGyroKd = 0.0f;
    float lastTurnGyroKp = 0.0f;
    float lastTurnGyroKi = 0.0f;
    float lastTurnGyroKd = 0.0f;

    int16_t lastMoveL          = 0;
    int16_t lastMoveR          = 0;
    int16_t lastMoveSyncL      = 0;
    int16_t lastMoveSyncR      = 0;
    int16_t lastMoveGyroPower  = 0;
    int16_t lastMoveGyroTarget = 0;

    int16_t  lastMoveDegsL     = 0;
    int16_t  lastMoveDegsR     = 0;
    uint16_t lastMoveDegsDeg   = 0;
    bool     lastMoveDegsBrake = false;
    bool     lastMoveDegsAsync = false;

    int16_t  lastMoveSyncDegsL     = 0;
    int16_t  lastMoveSyncDegsR     = 0;
    uint16_t lastMoveSyncDegsDeg   = 0;
    bool     lastMoveSyncDegsBrake = false;
    bool     lastMoveSyncDegsAsync = false;

    int16_t  lastMoveGyroDegsPower  = 0;
    int16_t  lastMoveGyroDegsTarget = 0;
    uint16_t lastMoveGyroDegsDeg    = 0;
    bool     lastMoveGyroDegsBrake  = false;
    bool     lastMoveGyroDegsAsync  = false;

    int16_t lastMoveTimeL     = 0;
    int16_t lastMoveTimeR     = 0;
    float   lastMoveTimeSec   = 0.0f;
    bool    lastMoveTimeBrake = false;
    bool    lastMoveTimeAsync = false;

    int16_t lastMoveSyncTimeL     = 0;
    int16_t lastMoveSyncTimeR     = 0;
    float   lastMoveSyncTimeSec   = 0.0f;
    bool    lastMoveSyncTimeBrake = false;
    bool    lastMoveSyncTimeAsync = false;

    int16_t lastMoveGyroTimePower  = 0;
    int16_t lastMoveGyroTimeTarget = 0;
    float   lastMoveGyroTimeSec    = 0.0f;
    bool    lastMoveGyroTimeBrake  = false;
    bool    lastMoveGyroTimeAsync  = false;

    bool lastBrake = false;

    /// Flag `isPrevTaskDone()` returns. Default true (task already finished).
    bool mockPrevTaskDone = true;

private:
    /// Backing value for `getDegrees()`; zeroed by `resetCounter()`.
    int32_t _degrees = 0;
};

// ===========================================================================
// MockLED — on-board RGB LED pair. Mirror of `MiniR4LED` (vendor MiniR4LED.h).
// Records the last color + counts (the CTPS variant was a pure no-op; the HAL
// records so a fixture can assert on Mission status-LED dispatch).
// ===========================================================================

/**
 * @brief Host mock for the on-board RGB LED pair (`MiniR4.LED`).
 *
 * Both `setColor` overloads mirror the vendor signatures and return type. The packed
 * overload stores the 0x00RRGGBB value; the separate-channel overload packs `r/g/b`
 * the same way the vendor does, so a test can assert on `lastRgb` regardless of which
 * overload the caller used.
 */
class MockLED
{
public:
    MockLED() = default;

    /**
     * @brief Mirrors `MiniR4LED::setColor(uint8_t,uint8_t,uint8_t,uint8_t)`.
     * @param idx LED index (1 or 2).
     * @param r   Red channel (0-255).
     * @param g   Green channel (0-255).
     * @param b   Blue channel (0-255).
     * @return Always true on host.
     */
    bool setColor(uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
    {
        const uint32_t rgb = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) |
                             static_cast<uint32_t>(b);
        return setColor(idx, rgb);
    }

    /**
     * @brief Mirrors `MiniR4LED::setColor(uint8_t,uint32_t)`; records idx + packed color.
     * @param idx LED index (1 or 2).
     * @param rgb Packed 0x00RRGGBB color.
     * @return Always true on host.
     */
    bool setColor(uint8_t idx, uint32_t rgb)
    {
        setColorCallCount++;
        lastIdx = idx;
        lastRgb = rgb;
        return true;
    }

    /**
     * @brief Mirrors `MiniR4LED::setBrightness(uint8_t,uint8_t)`; records idx + level.
     * @param idx        LED index (1 or 2).
     * @param brightness Brightness level (0-255).
     */
    void setBrightness(uint8_t idx, uint8_t brightness)
    {
        setBrightnessCallCount++;
        lastBrightnessIdx = idx;
        lastBrightness    = brightness;
    }

    /** @brief Clear all recorded state. */
    void mockReset()
    {
        setColorCallCount     = 0;
        setBrightnessCallCount = 0;
        lastIdx               = 0;
        lastRgb               = 0;
        lastBrightnessIdx     = 0;
        lastBrightness        = 0;
    }

    // ---- Test-visible recorded state ----------------------------------------
    uint32_t setColorCallCount      = 0;
    uint32_t setBrightnessCallCount = 0;
    uint8_t  lastIdx                = 0;
    uint32_t lastRgb                = 0;
    uint8_t  lastBrightnessIdx      = 0;
    uint8_t  lastBrightness         = 0;
};

// ===========================================================================
// MockBuzzer — on-board buzzer. Mirror of `MiniR4BUZZER` (vendor MiniR4Buzzer.h).
// ===========================================================================

/**
 * @brief Host mock for the on-board buzzer (`MiniR4.Buzzer`).
 *
 * Records the last tone frequency + duration and a NoTone count so a fixture can assert
 * on Mission beep dispatch.
 */
class MockBuzzer
{
public:
    MockBuzzer() = default;

    /**
     * @brief Mirrors `MiniR4BUZZER::Tone(uint16_t,uint32_t)`; records freq + duration.
     * @param frequency Tone frequency (Hz).
     * @param duration  Tone duration (ms).
     */
    void Tone(uint16_t frequency, uint32_t duration)
    {
        toneCallCount++;
        lastFrequency = frequency;
        lastDuration  = duration;
    }

    /** @brief Mirrors `MiniR4BUZZER::NoTone()`; records the stop-tone call. */
    void NoTone(void) { noToneCallCount++; }

    /** @brief Clear all recorded state. */
    void mockReset()
    {
        toneCallCount   = 0;
        noToneCallCount = 0;
        lastFrequency   = 0;
        lastDuration    = 0;
    }

    // ---- Test-visible recorded state ----------------------------------------
    uint32_t toneCallCount   = 0;
    uint32_t noToneCallCount = 0;
    uint16_t lastFrequency   = 0;
    uint32_t lastDuration    = 0;
};

// ===========================================================================
// MockBTN<ID> — on-board button. Mirror of `MiniR4BTN<ID>` (vendor MiniR4BTN.h).
// Kept templated on `ID` so the member shapes match (`BTN_UP` is <2>, `BTN_DOWN` <1>).
// ===========================================================================

/**
 * @brief Host mock for an on-board button (`MiniR4.BTN_UP` / `MiniR4.BTN_DOWN`).
 *
 * `getState()` returns a test-settable `mockPressed` (default `false` = not pressed), so a
 * fixture can script the operator pressing reset during a Mission fault wait.
 *
 * @tparam ID The button ID (matches the vendor template parameter; host never uses it).
 */
template <uint8_t ID>
class MockBTN
{
public:
    MockBTN() = default;

    /**
     * @brief Mirrors `MiniR4BTN<ID>::getState()`; returns the test-settable press flag.
     * @return `mockPressed` (default `false`).
     */
    bool getState(void)
    {
        getStateCallCount++;
        return mockPressed;
    }

    /// Set the flag `getState()` returns (default `false`).
    void mockSetPressed(bool pressed) { mockPressed = pressed; }

    /** @brief Clear all recorded state + restore the not-pressed default. */
    void mockReset()
    {
        getStateCallCount = 0;
        mockPressed       = false;
    }

    // ---- Test-visible recorded state ----------------------------------------
    uint32_t getStateCallCount = 0;

    /// Value `getState()` returns. Default false (not pressed).
    bool mockPressed = false;
};

// ===========================================================================
// MockOLED — on-board OLED. Mirror of the `Adafruit_SSD1306` text surface
// (vendor MiniR4OLED.h). All no-ops: the OLED is rarely touched in logic tests.
// ===========================================================================

/**
 * @brief Host mock for the on-board OLED (`MiniR4.OLED`).
 *
 * Covers the Adafruit_SSD1306-style text surface EventTrace / Mission could touch. All
 * methods are no-ops — display output is invisible in a host test, so nothing is recorded;
 * these exist only so any OLED call site compiles.
 */
class MockOLED
{
public:
    MockOLED() = default;

    /** @brief No-op clear-display. */
    void clearDisplay() {}

    /** @brief No-op flush-to-display. */
    void display() {}

    /** @brief No-op text-size set. @param size Text scale factor. */
    void setTextSize(uint8_t /*size*/) {}

    /** @brief No-op text-color set. @param color Foreground color. */
    void setTextColor(uint16_t /*color*/) {}

    /** @brief No-op cursor move. @param x Column. @param y Row. */
    void setCursor(int16_t /*x*/, int16_t /*y*/) {}

    /** @brief No-op print. @param s C-string to print. */
    void print(const char* /*s*/) {}

    /** @brief No-op println. @param s C-string to print with a newline. */
    void println(const char* /*s*/) {}

    /** @brief No-op newline-only println. */
    void println() {}
};

// ===========================================================================
// MockPower — battery management. Mirror of `MiniR4Power` (vendor MiniR4Power.h).
// ===========================================================================

/**
 * @brief Host mock for the power-management facade (`MiniR4.PWR`).
 *
 * Records the last cell count + a call counter and always reports success (the real call
 * can fail on the lower MCU; the host never does).
 */
class MockPower
{
public:
    MockPower() = default;

    /**
     * @brief Mirrors `MiniR4Power::setBattCell(uint8_t)`; records the cell count.
     * @param cell Number of 18650 cells (2-6).
     * @return Always true on host.
     */
    bool setBattCell(uint8_t cell)
    {
        setBattCellCallCount++;
        lastCell = cell;
        return true;
    }

    /** @brief Clear all recorded state. */
    void mockReset()
    {
        setBattCellCallCount = 0;
        lastCell             = 0;
    }

    // ---- Test-visible recorded state ----------------------------------------
    uint32_t setBattCellCallCount = 0;
    uint8_t  lastCell             = 0;
};

// ===========================================================================
// MMLower — lower-MCU surface. CARRIED VERBATIM from `mocks/MatrixMiniR4.h`
// (superset-carry invariant). `Localization.cpp` reads the IMU via
// `mmL.GetIMUEuler(roll,pitch,yaw)` and inspects the returned `MMLower::RESULT`.
// ===========================================================================

/**
 * @brief Result enum mirrors of the vendor `MMLower::RESULT` / `MMLower::Drive_RESULT`.
 *
 * `RESULT` — the two states `Localization.cpp` / `EncoderSource.cpp` compare against: `OK`
 * (read succeeded, out-params written) and `ERROR` (any non-OK; the test scripts a transient
 * timeout with it). `Drive_RESULT` — the vendor's separate drive-command result type,
 * modelled with the same two states so `EncoderSource::isPrevTaskDone()` can gate on it.
 * The underlying type matches the vendor's `uint8_t`.
 */
namespace MMLower
{
    enum class RESULT : uint8_t
    {
        OK    = 0,
        ERROR = 1
    };

    enum class Drive_RESULT : uint8_t
    {
        OK    = 0,
        ERROR = 1
    };
}    // namespace MMLower

/**
 * @brief Host mock for the lower-MCU IMU read used by `Localization`.
 *
 * Backs `mmL.GetIMUEuler(double&,double&,double&)` with a test-settable yaw and a
 * test-settable result code so a fixture can script the heading the integrator sees
 * AND a transient read failure. Faithfully reproduces the one behaviour Localization
 * depends on: on a NON-OK read the out-params are left UNTOUCHED (the real lib returns
 * before writing yaw on a wait/read timeout — see `Localization::_readImuYaw`).
 */
class MockMMLower
{
public:
    MockMMLower() = default;

    /**
     * @brief Mirrors `mmL.GetIMUEuler(double&,double&,double&)`.
     *
     * Writes `yaw` (and zeroes `roll`/`pitch`) ONLY on an `OK` result; on a non-OK
     * result returns immediately WITHOUT touching the out-params, matching the vendor
     * contract `Localization::_readImuYaw` relies on.
     *
     * @param roll  Out-param set to `0.0` only on OK.
     * @param pitch Out-param set to `0.0` only on OK.
     * @param yaw   Out-param set to `mockYawDeg` only on OK; untouched otherwise.
     * @return `mockResult` (defaults to `OK`).
     */
    MMLower::RESULT GetIMUEuler(double& roll, double& pitch, double& yaw)
    {
        getImuEulerCallCount++;
        if (mockResult != MMLower::RESULT::OK)
        {
            return mockResult;
        }
        roll  = 0.0;
        pitch = 0.0;
        yaw   = mockYawDeg;
        return MMLower::RESULT::OK;
    }

    /**
     * @brief Mirrors `mmL.GetEncoderDegrees(uint8_t,int32_t&)` — the RESULT-surfacing encoder
     *        read `EncoderSource` uses in place of the RESULT-discarding `MiniR4DC::getDegrees`.
     *
     * DELEGATES the degrees to the corresponding `MiniR4.M<num>` backing so the 51-usage
     * `M2/M3/M1.mockSetDegrees / mockAdvanceDegrees` test surface stays the single source of
     * truth (real hardware reads the SAME encoder either way). Gated on `mockEncoderResult`:
     * a fixture sets it to `ERROR` to script a timeout, and on a non-OK result the out-param
     * is left UNTOUCHED, matching the vendor contract `EncoderSource::readDegrees` depends on.
     * Defined out-of-class below (after `MiniR4` is declared) so the delegation resolves.
     *
     * @param num     M-bus channel `1..4`.
     * @param enDeges Out-param set to `MiniR4.M<num>.getDegrees()` only on OK; untouched otherwise.
     * @return `mockEncoderResult` (defaults to `OK`).
     */
    MMLower::RESULT GetEncoderDegrees(uint8_t num, int32_t& enDeges);

    /**
     * @brief Mirrors `mmL.Get_Drive_isTaskDone(uint8_t,bool*)` — the RESULT-surfacing task-done
     *        read `EncoderSource::isPrevTaskDone` uses.
     *
     * DELEGATES the done-bit to `MiniR4.DriveDC.mockPrevTaskDone` (the existing test-settable
     * flag) using the vendor convention `isEnd[0] == true` means "still running": a done task
     * writes `isEnd[0] = false`. Gated on `mockTaskDoneResult`; on a non-OK result `isEnd` is
     * left untouched. Defined out-of-class below (after `MiniR4` is declared).
     *
     * @param num   Drive instance id (`EncoderSource::DRIVE_INSTANCE_ID == 1`).
     * @param isEnd Two-element buffer; `isEnd[0]` receives the running flag only on OK.
     * @return `mockTaskDoneResult` (defaults to `OK`).
     */
    MMLower::Drive_RESULT Get_Drive_isTaskDone(uint8_t num, bool* isEnd);

    /** @brief Restore defaults: yaw 0, encoder/task-done OK, all call counters cleared. */
    void mockReset()
    {
        mockYawDeg                 = 0.0;
        mockResult                 = MMLower::RESULT::OK;
        getImuEulerCallCount       = 0;
        mockEncoderResult          = MMLower::RESULT::OK;
        getEncoderDegreesCallCount = 0;
        lastEncoderNum             = 0;
        mockTaskDoneResult         = MMLower::Drive_RESULT::OK;
        getDriveTaskDoneCallCount  = 0;
        lastTaskDoneNum            = 0;
    }

    // ---- Test setters / recorded state --------------------------------------

    /// Yaw (degrees) returned by the next OK `GetIMUEuler`.
    double mockYawDeg = 0.0;

    /// Result returned by `GetIMUEuler` (set to `ERROR` to script a transient timeout).
    MMLower::RESULT mockResult = MMLower::RESULT::OK;

    /// Number of `GetIMUEuler` calls since the last `mockReset()`.
    uint32_t getImuEulerCallCount = 0;

    /// Result returned by `GetEncoderDegrees` (set to `ERROR` to script an encoder timeout).
    MMLower::RESULT mockEncoderResult = MMLower::RESULT::OK;

    /// Number of `GetEncoderDegrees` calls since the last `mockReset()`.
    uint32_t getEncoderDegreesCallCount = 0;

    /// Motor id passed to the most recent `GetEncoderDegrees` call.
    uint8_t lastEncoderNum = 0;

    /// Result returned by `Get_Drive_isTaskDone` (set to `ERROR` to script a task-done timeout).
    MMLower::Drive_RESULT mockTaskDoneResult = MMLower::Drive_RESULT::OK;

    /// Number of `Get_Drive_isTaskDone` calls since the last `mockReset()`.
    uint32_t getDriveTaskDoneCallCount = 0;

    /// Drive instance id passed to the most recent `Get_Drive_isTaskDone` call.
    uint8_t lastTaskDoneNum = 0;
};

/// Single global instance. A C++17 `inline` variable so it has ONE definition across
/// every TU that includes this header without a `hal_mocks.cpp` edit (mirrors the
/// vendor's `extern MMLower mmL;`, but inline so no mock source file changes). Carried
/// verbatim from `mocks/MatrixMiniR4.h`.
inline MockMMLower mmL;

// ===========================================================================
// External gyro board (MatrixColor) — host mock + `mmL.mockYawDeg` bridge (T3, ADR-070).
//
// T4 re-points `YawSource::readYawDeg` off the STM32 IMU onto an external MXColor-protocol
// gyro read through the vendor `MatrixColor` class (a real board at I2C addr 0x22). The host
// mock world has no such class, no `TwoWire`, and no `Wire1`, so this block adds exactly the
// surface T4 (and its test T7) bind to — nothing more. It models NO board behaviour, drift, or
// physics: a mock that evolves its own state produces false greens (the repo's THIN-mock rule).
// Scripted bytes + a `mmL.mockYawDeg` bridge, full stop. The vendor `begin()` really ID-checks
// `Device_ID == 0x43` and writes a CONFIG reset; this mock deliberately does NOT simulate that —
// it just returns `mockBeginOk`.
//
// !! NAMESPACE HAZARD: the vendor `ColorType` enum (carried byte-for-byte below) injects the
//    UNSCOPED single-letter names `R, G, B, C, M, Y, K` into the global namespace of EVERY host
//    target that includes this header — the same collision class as the known `WHITE`-macro-vs-
//    enum gotcha. They cannot be renamed (production code and the vendor header use them). If a
//    target suddenly fails to compile on one of those identifiers, this is why.
// ===========================================================================

/**
 * @brief Host mock of the Arduino-core `TwoWire` I2C bus object.
 *
 * The real `MatrixColor::_pWire` points at a `TwoWire`; T4 does `_pWire = &Wire1`. Only
 * `begin()` is part of the surface the SUT touches, and it is a no-op here (the host has no
 * physical bus). Added because `TwoWire`/`Wire1` exist nowhere else in the mock world.
 */
class TwoWire
{
public:
    /** @brief No-op bus init — the host has no physical I2C. */
    void begin() {}
};

/// The `Wire1` bus instance T4 binds `MatrixColor::_pWire` to. C++17 `inline` so it has ONE
/// definition across every TU that includes this header without any `.cpp` edit (mirrors
/// `inline MockMMLower mmL;`).
inline TwoWire Wire1;

/// Vendor `ColorType` enum, carried byte-for-byte from
/// `MatrixMiniR4/src/Modules/Sensors/MiniR4ColorSensorExt.h`. See the NAMESPACE HAZARD note above.
typedef enum __ColorType
{
    R = 4,
    G,
    B,
    C,
    M,
    Y,
    K
} ColorType;

/**
 * @brief Host mock of the vendor `MatrixColor` external-gyro driver (I2C addr 0x22).
 *
 * THIN record-and-script mock. `getColor` either returns test-injected raw bytes
 * (`mockUseRawBytes == true`, used by TC14/TC15 to inject dead/torn reads) or BRIDGES from
 * `mmL.mockYawDeg` (the default) so the 17 legacy suites that drive yaw through `mmL.mockYawDeg`
 * survive T4's read-path migration untouched. See the class-level HAZARD + bridge notes.
 *
 * @note THE BRIDGE (load-bearing). Because `mmL` is an inline global in this same header,
 *       `getColor` synthesizes the board's two bytes from `mmL.mockYawDeg` so that the
 *       production decode `GYRO_YAW_SIGN * GYRO_SCALE * (raw16 - 32768)` (wrapped) yields
 *       exactly `mmL.mockYawDeg * IMU_YAW_SIGN` — what `test_path_tracker_harness.h` documents.
 *       The scale/sign cancel by construction, so the bridge stays correct for any pinned
 *       constants.
 * @note ACCEPTED RESOLUTION LIMIT. The bridge quantizes to whole degrees (1 raw count ~= 1 deg,
 *       the device's real resolution), so a parked non-integer heading round-trips within
 *       +/-0.5 deg. If a suite ever proves tighter than that, RAISE that suite's tolerance —
 *       never fake sub-degree precision in the mock.
 */
class MatrixColor
{
public:
    /// Mux channel (`Config::GYRO_I2C_PORT - 1`); T4 records it, the host never acts on it.
    uint8_t  _ch    = 0;
    /// Bus pointer T4 sets to `&Wire1`; recorded, never dereferenced on the host.
    TwoWire* _pWire = nullptr;

    // ---- test-settable state (static inline: no .cpp edit, no duplicate symbols) ---------
    /// Value `begin()` returns.
    static inline bool     mockBeginOk     = true;
    /// false => `getColor` bridges from `mmL.mockYawDeg`; true => returns `mockG`/`mockR` raw.
    static inline bool     mockUseRawBytes = false;
    /// Raw high byte returned for `G` when `mockUseRawBytes` (TC14/TC15 dead/torn injection).
    static inline uint8_t  mockG           = 0;
    /// Raw low byte returned for `R` when `mockUseRawBytes`.
    static inline uint8_t  mockR           = 0;
    /// Count of `getColor(K)` calls — proves `zeroHardware()` fired (AC-7 / TC20).
    static inline uint32_t mockKReadCount  = 0;
    /// Logical device-sample count. Incremented ONCE PER G-REGISTER READ (see `getColor`), NOT
    /// once per call: a sample is a G+R pair, so per-call counting would read 2 and break TC17's
    /// `mockDeviceReads == 1` across 3 consumer reads in one tick. CROSS-TASK SEAM (AC-5).
    static inline uint32_t mockDeviceReads = 0;

    /// Restore every `mock*` field above to its constructed default. Call in test setup.
    static inline void resetMock();

    /** @brief Mirrors `MatrixColor::begin()`; returns the scripted flag — no ID-check, no reset. */
    bool    begin() { return mockBeginOk; }

    /** @brief Mirrors `MatrixColor::setLight(bool,bool,uint8_t)`; a no-op on the host. */
    void    setLight(bool, bool, uint8_t) {}

    /**
     * @brief Mirrors `MatrixColor::getColor(ColorType)`.
     * @param color Register to read (`G` = high byte, `R` = low byte, `K` = zero side-effect).
     * @return The scripted or bridged byte (0 for `K` and for channels the gyro path never asks).
     */
    uint8_t getColor(ColorType color);

private:
    // ---- Bridge scale/sign ---------------------------------------------------------------
    // These MUST equal the constants YawSource.cpp's decode uses, or the synthesized bytes will
    // not round-trip to `mmL.mockYawDeg * IMU_YAW_SIGN`. We read the Config `*_DEFAULT` macro
    // hooks DIRECTLY rather than `#include "Config.h"`: dragging project Config into the HAL mock
    // would invert the HAL<-project layering and pull all of Config.h (+ its ConfigChecks
    // static_asserts) into all 47 mock-including TUs. Reading the same `#ifndef X_DEFAULT` hooks
    // Config uses keeps the mock HONEST — a target-wide `target_compile_definitions(... X=v)`
    // reaches this mock and Config.h identically. T3b pins `GYRO_SCALE_DEFAULT=1.0f` and
    // `GYRO_YAW_SIGN_DEFAULT=<IMU_YAW_SIGN value>` on the 32 YawSource targets, making the
    // round-trip exact there.
    //
    // COUPLING: the fallback literals below are kept BYTE-IDENTICAL to `config/ConfigNav.h`'s own
    // `#ifndef` fallbacks (IMU_YAW_SIGN -1.0f, GYRO_YAW_SIGN -1.0f, GYRO_SCALE 0.997f) so an
    // UNpinned target still agrees with production. Change ConfigNav.h's defaults => change these.
#if defined(IMU_YAW_SIGN_DEFAULT)
    static constexpr float kBridgeImuSign  = IMU_YAW_SIGN_DEFAULT;
#else
    static constexpr float kBridgeImuSign  = -1.0f;
#endif
#if defined(GYRO_YAW_SIGN_DEFAULT)
    static constexpr float kBridgeGyroSign = GYRO_YAW_SIGN_DEFAULT;
#else
    static constexpr float kBridgeGyroSign = -1.0f;
#endif
#if defined(GYRO_SCALE_DEFAULT)
    static constexpr float kBridgeScale    = GYRO_SCALE_DEFAULT;
#else
    static constexpr float kBridgeScale    = 0.997f;
#endif
};

inline void MatrixColor::resetMock()
{
    mockBeginOk     = true;
    mockUseRawBytes = false;
    mockG           = 0;
    mockR           = 0;
    mockKReadCount  = 0;
    mockDeviceReads = 0;
}

inline uint8_t MatrixColor::getColor(ColorType color)
{
    // K is the gyro's zero-trigger side-effect read: count it, return a discarded byte, and do
    // NOT count it as a device sample.
    if (color == K)
    {
        mockKReadCount++;
        return 0;
    }

    // One logical device sample == a G(high) + R(low) pair. Count once, on the G read, so a full
    // sample registers as exactly ONE device read regardless of how many consumers serve from it.
    if (color == G)
    {
        mockDeviceReads++;
    }

    if (mockUseRawBytes)
    {
        if (color == G) return mockG;
        if (color == R) return mockR;
        return 0;    // the gyro read path only ever asks for G / R / K
    }

    // Bridge from mmL.mockYawDeg (see the class @note). `biased` reverses the production decode
    // so that decode(G, R) == mmL.mockYawDeg * IMU_YAW_SIGN. Quantizes to whole degrees.
    const long     biased = lround((double)mmL.mockYawDeg * (double)kBridgeImuSign /
                                   ((double)kBridgeGyroSign * (double)kBridgeScale));
    const uint16_t raw16  = (uint16_t)(biased + 32768);
    if (color == G) return (uint8_t)(raw16 >> 8);
    if (color == R) return (uint8_t)(raw16 & 0xFF);
    return 0;
}

// ===========================================================================
// MatrixMiniR4 — top-level facade. Mirror of the vendor `MatrixMiniR4`.
// Strict SUPERSET of the legacy `MockMatrixMiniR4` (carries M1..M4, RC1..RC4)
// plus the DriveDC / LED / Buzzer / BTN / OLED / PWR families.
// ===========================================================================

/**
 * @brief Host mock for the top-level `MiniR4` vendor facade.
 *
 * Exposes every actuator/sensor family the production `Drive.cpp` + `Mission.cpp` reach.
 * `begin()` mirrors the vendor facade's top-level init and reports success WITHOUT
 * auto-beginning the children (each consumer begins the channels it owns explicitly).
 */
class MatrixMiniR4
{
public:
    MatrixMiniR4() = default;

    /** @brief Mirrors `MatrixMiniR4::begin()`; records the call, always succeeds. */
    bool begin()
    {
        beginCallCount++;
        return true;
    }

    /// Clear every child mock's recorded state and the facade's own counter.
    void mockReset()
    {
        beginCallCount = 0;
        DriveDC.mockReset();
        LED.mockReset();
        Buzzer.mockReset();
        BTN_UP.mockReset();
        BTN_DOWN.mockReset();
        PWR.mockReset();
        RC1.mockReset();
        RC2.mockReset();
        RC3.mockReset();
        RC4.mockReset();
        M1.mockReset();
        M2.mockReset();
        M3.mockReset();
        M4.mockReset();
    }

    // ---- Actuator / sensor families -----------------------------------------

    /// Differential-drive base (`MiniR4DriveDC<1>`, template param dropped).
    MockDriveDC DriveDC;

    /// On-board RGB LED pair.
    MockLED LED;

    /// On-board buzzer.
    MockBuzzer Buzzer;

    /// On-board OLED display.
    MockOLED OLED;

    /// Up button (vendor ID 2).
    MockBTN<2> BTN_UP;

    /// Down button (vendor ID 1).
    MockBTN<1> BTN_DOWN;

    /// Power / battery management.
    MockPower PWR;

    // Servo channels (RC bus).
    MockRC RC1;
    MockRC RC2;
    MockRC RC3;
    MockRC RC4;

    // DC motor channels (M bus). M2/M3 are the WRO chassis drive encoders.
    MockDC M1;
    MockDC M2;
    MockDC M3;
    MockDC M4;

    // ---- Facade-level recorded state ----------------------------------------
    uint32_t beginCallCount = 0;
};

/// Single global instance — defined in `hal_mocks/hal_mocks.cpp` (mirrors the vendor's
/// `extern MatrixMiniR4 MiniR4;` so `#include <MatrixMiniR4.h>` resolves to this mock via
/// the `-I hal_mocks/` include path).
extern MatrixMiniR4 MiniR4;

// ---------------------------------------------------------------------------------------
// MockMMLower encoder / task-done reads — defined OUT-OF-CLASS here because they DELEGATE
// to `MiniR4` (the `M<num>` encoder backing / `DriveDC` done flag), which is only declared
// above this point. `inline` so they satisfy the ODR across every TU that includes this
// header (mirroring the header's other inline definitions). See the in-class Doxygen.
// ---------------------------------------------------------------------------------------

inline MMLower::RESULT MockMMLower::GetEncoderDegrees(uint8_t num, int32_t& enDeges)
{
    getEncoderDegreesCallCount++;
    lastEncoderNum = num;
    if (mockEncoderResult != MMLower::RESULT::OK)
    {
        return mockEncoderResult;    // out-param untouched, matching the vendor timeout contract
    }
    switch (num)
    {
        case 1: enDeges = MiniR4.M1.getDegrees(); break;
        case 2: enDeges = MiniR4.M2.getDegrees(); break;
        case 3: enDeges = MiniR4.M3.getDegrees(); break;
        case 4: enDeges = MiniR4.M4.getDegrees(); break;
        default: return MMLower::RESULT::ERROR;    // unaddressable channel
    }
    return MMLower::RESULT::OK;
}

inline MMLower::Drive_RESULT MockMMLower::Get_Drive_isTaskDone(uint8_t num, bool* isEnd)
{
    getDriveTaskDoneCallCount++;
    lastTaskDoneNum = num;
    if (mockTaskDoneResult != MMLower::Drive_RESULT::OK)
    {
        return mockTaskDoneResult;    // isEnd untouched
    }
    // Vendor convention: isEnd[0] == true means "still running". Mirror the DriveDC done flag.
    if (isEnd != nullptr)
    {
        isEnd[0] = !MiniR4.DriveDC.mockPrevTaskDone;
        isEnd[1] = false;
    }
    return MMLower::Drive_RESULT::OK;
}

/**
 * @brief Reset every recorded field on the `MiniR4` mock (facade + all child mocks).
 *
 * Tests call this at the top of each scenario, paralleling `Drive.mockReset()` /
 * `LineArray.mockReset()`. Defined inline so any TU that includes this header can use it
 * without linking an extra object.
 */
inline void mockResetMiniR4()
{
    MiniR4.mockReset();
}

#endif    // MATRIXMINIR4_H
