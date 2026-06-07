/**
 * @file MatrixMiniR4.h (HOST MOCK)
 * @brief Host-compile mock of the MatrixMiniR4 vendor facade — just enough of the
 *        `MiniR4` global to let the real `Manipulator.cpp` link on a host toolchain.
 *
 * Background — why this used to be empty:
 *   `LineFollower.h` and `Intersection.h` include `<MatrixMiniR4.h>` only so the
 *   `#include` resolves; they never touch the `MiniR4` global. The original mock
 *   was therefore an intentionally empty shim.
 *
 * What changed (T8, supports T9):
 *   T9 links the production `Manipulator.cpp` against this mock. The real
 *   manipulator drives RC servos (`MiniR4.RC1..RC4`) and the conveyor-belt DC motor
 *   (`MiniR4.M1..M4`), so the mock now exposes those two actuator families with the
 *   exact public method signatures the vendor headers declare:
 *     - `MiniR4RC<ID>`  -> `MiniR4RC.h`  (`begin`, `setHWDir`, `setAngle`).
 *     - `MiniR4DC<ID>`  -> `MiniR4DC.h`  (`begin`, `setPPR_RPM`, `setReverse`,
 *                                         `setSpeed`, `rotateFor`, `getDegrees`,
 *                                         `resetCounter`, `setBrake`, `ChkRotateEnd`).
 *
 *   Every call is recorded into test-visible fields so a future fixture can assert on
 *   dispatch (last commanded angle, last rotate speed/degree, call counts, ...).
 *   Encoder degrees and the rotate-end flag are test-settable so a fixture can script
 *   belt motion / homing completion deterministically.
 *
 * ADDITIVE contract:
 *   These additions are purely additive. The existing consumers
 *   (`LineFollower.h` / `Intersection.h`) only require the `#include` to succeed, and
 *   they still see no symbols that conflict with their own. The mock introduces NO
 *   `MatrixColor` driver (the mock `WROLineArray.h` has no `MatrixColor` members, so
 *   it is not needed) and NO Arduino-core symbols beyond what `Arduino.h` already
 *   provides.
 *
 * The `MiniR4` global is declared `extern` here and defined once in `mocks.cpp`,
 * mirroring the vendor's `extern MatrixMiniR4 MiniR4;` so `#include <MatrixMiniR4.h>`
 * (angle brackets) resolves to this mock via the `-I mocks/` include path.
 */

#ifndef WRO_TEST_MOCK_MATRIXMINIR4_H
#define WRO_TEST_MOCK_MATRIXMINIR4_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Mock RC servo channel — mirrors `MiniR4RC<ID>` (vendor `MiniR4RC.h`).
//
// Public surface kept to exactly the three methods the manipulator calls:
//   - `begin()`        -> vendor sets the 0..180 angle range; mock returns true.
//   - `setHWDir(bool)` -> records the hardware-direction flag.
//   - `setAngle(uint16_t)` -> records the last commanded angle + bumps a counter.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Mock DC motor channel — mirrors `MiniR4DC<ID>` (vendor `MiniR4DC.h`).
//
// Public surface kept to exactly the methods the manipulator (belt) calls, with
// the vendor signatures preserved:
//   - `begin()`                          -> records, always succeeds.
//   - `setPPR_RPM(uint16_t,uint16_t)`    -> records both args.
//   - `setReverse(bool)`                 -> records the flag.
//   - `setSpeed(int16_t)`                -> records last speed.
//   - `rotateFor(int16_t,uint16_t)`      -> records speed+degree, bumps counter.
//   - `getDegrees()`                     -> returns the test-settable backing value.
//   - `resetCounter()`                   -> ZEROES that backing value (mirrors the
//                                            vendor encoder reset).
//   - `setBrake(bool)`                   -> records the flag.
//   - `ChkRotateEnd(bool&)`              -> writes the test-settable `mockRotateEnded`
//                                            into the out-param, returns true.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Mock MiniR4 facade — exposes the RC servo + DC motor families the manipulator
// uses. Member names mirror the vendor `MatrixMiniR4` (`RC1..RC4`, `M1..M4`).
// ---------------------------------------------------------------------------

/**
 * @brief Host mock for the `MiniR4` vendor facade.
 *
 * Exposes only the actuator families exercised by the manipulator. `begin()` mirrors
 * the vendor facade's top-level init and reports success without auto-beginning the
 * children (the manipulator begins the channels it owns explicitly).
 */
class MockMatrixMiniR4
{
public:
    MockMatrixMiniR4() = default;

    /** @brief Mirrors `MatrixMiniR4::begin()`; records the call, always succeeds. */
    bool begin()
    {
        beginCallCount++;
        return true;
    }

    /// Clear every channel's recorded state and the facade's own counter.
    void mockReset()
    {
        beginCallCount = 0;
        RC1.mockReset();
        RC2.mockReset();
        RC3.mockReset();
        RC4.mockReset();
        M1.mockReset();
        M2.mockReset();
        M3.mockReset();
        M4.mockReset();
    }

    // Servo channels (RC bus).
    MockRC RC1;
    MockRC RC2;
    MockRC RC3;
    MockRC RC4;

    // DC motor channels (M bus).
    MockDC M1;
    MockDC M2;
    MockDC M3;
    MockDC M4;

    // Facade-level recorded state.
    uint32_t beginCallCount = 0;
};

/// Single global instance — defined in `mocks/mocks.cpp` (mirrors the vendor's
/// `extern MatrixMiniR4 MiniR4;`).
extern MockMatrixMiniR4 MiniR4;

/**
 * @brief Reset every recorded field on the `MiniR4` mock (facade + all RC/M channels).
 *
 * Tests call this at the top of each scenario, paralleling `Drive.mockReset()` /
 * `LineArray.mockReset()`. Defined inline so any TU that includes this header can use
 * it without linking an extra object.
 */
inline void mockResetMiniR4()
{
    MiniR4.mockReset();
}

#endif    // WRO_TEST_MOCK_MATRIXMINIR4_H
