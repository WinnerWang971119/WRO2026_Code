/**
 * @file Drive.h (HOST MOCK)
 * @brief Minimal `DriveClass` mock — only the surface called by `LineFollower.cpp`.
 *
 * `LineFollower.cpp` calls exactly these `Drive` methods:
 *   - `Drive.setSpeed(L, R)`   (continuous PWM emit)
 *   - `Drive.stop(brake)`      (brake/coast)
 *   - `Drive.getEncL()`        (left encoder, post-direction)
 *   - `Drive.getEncR()`        (right encoder, post-direction)
 *   - `Drive.isIdle()`         (used only by the sync-blocking helpers)
 *
 * The mock records each call's arguments and exposes them via public counters
 * + last-call accessors so the tests can assert on dispatch behavior.
 *
 * Encoder values are settable so a test can simulate forward chassis motion by
 * incrementing the encoders manually between `LineFollower.update()` ticks.
 */

// NOTE: This mock uses the SAME include guard (`DRIVE_H`) as the production
// `WRO2026/Drive.h`. See `mocks/WROLineArray.h` for the full rationale.
#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

// Production `LineFollower.cpp` relies on `Drive.h` transitively pulling in
// `WROMath.h` (which holds the free functions `deadbandSnap`, `presetToPwm`,
// `encDegToCm`). We mirror that include order here so the controller TU finds
// those symbols when the mock Drive.h shadows the real one.
#include "WROMath.h"
#include "Config.h"    // real — Config::SpeedPreset for the builder preset setters

/**
 * @brief Drive control mode — mirror of the production `DriveMode`.
 */
enum class DriveMode : uint8_t
{
    RAW,     ///< Open-loop PWM.
    SYNC,    ///< Encoder-rate sync PID.
    GYRO     ///< Yaw-lock.
};

// ---------------------------------------------------------------------------
// No-op move-only `[[nodiscard]]` builders mirroring the production fluent
// verbs. Task bodies use them as discarded temporaries
// (`Drive.driveDist(30).tShape();`), so construction / destruction do nothing
// and every chain setter returns `*this`. The mock builders carry no owner
// reference (the real ones do) because they perform no work — this also avoids
// the facade/builder definition-order cycle.
// ---------------------------------------------------------------------------

/// @brief Shared macro emitting a move-only `[[nodiscard]]` builder body with the
///        given chain setters. Keeps the five near-identical mock builders terse.
#define WRO_MOCK_BUILDER_BOILERPLATE(NAME)                                  \
    NAME() = default;                                                       \
    NAME(NAME&&) noexcept            = default;                             \
    NAME(const NAME&)                = delete;                              \
    NAME& operator=(const NAME&)     = delete;                             \
    NAME& operator=(NAME&&)          = delete;

class [[nodiscard]] DriveDistBuilder
{
public:
    WRO_MOCK_BUILDER_BOILERPLATE(DriveDistBuilder)
    DriveDistBuilder& setPower(int16_t) { return *this; }
    DriveDistBuilder& setPreset(Config::SpeedPreset) { return *this; }
    DriveDistBuilder& setMode(DriveMode) { return *this; }
    DriveDistBuilder& setBrake(bool) { return *this; }
    DriveDistBuilder& setBlocking(bool) { return *this; }
    DriveDistBuilder& async() { return *this; }
    DriveDistBuilder& setHeading(int16_t) { return *this; }
    DriveDistBuilder& tShape() { return *this; }
    DriveDistBuilder& flat() { return *this; }
    DriveDistBuilder& setAccel(float) { return *this; }
    DriveDistBuilder& setStartSpeed(int16_t) { return *this; }
    DriveDistBuilder& setEndSpeed(int16_t) { return *this; }
};

class [[nodiscard]] DriveTimeBuilder
{
public:
    WRO_MOCK_BUILDER_BOILERPLATE(DriveTimeBuilder)
    DriveTimeBuilder& setPower(int16_t) { return *this; }
    DriveTimeBuilder& setPreset(Config::SpeedPreset) { return *this; }
    DriveTimeBuilder& setMode(DriveMode) { return *this; }
    DriveTimeBuilder& setBrake(bool) { return *this; }
    DriveTimeBuilder& setBlocking(bool) { return *this; }
    DriveTimeBuilder& async() { return *this; }
    DriveTimeBuilder& setHeading(int16_t) { return *this; }
    DriveTimeBuilder& tShape() { return *this; }
    DriveTimeBuilder& flat() { return *this; }
    DriveTimeBuilder& setAccel(float) { return *this; }
    DriveTimeBuilder& setStartSpeed(int16_t) { return *this; }
    DriveTimeBuilder& setEndSpeed(int16_t) { return *this; }
};

class [[nodiscard]] TurnDegBuilder
{
public:
    WRO_MOCK_BUILDER_BOILERPLATE(TurnDegBuilder)
    TurnDegBuilder& setPower(int16_t) { return *this; }
    TurnDegBuilder& setPreset(Config::SpeedPreset) { return *this; }
    TurnDegBuilder& setMode(uint8_t) { return *this; }
    TurnDegBuilder& setBrake(bool) { return *this; }
    TurnDegBuilder& setBlocking(bool) { return *this; }
    TurnDegBuilder& async() { return *this; }
};

class [[nodiscard]] TurnToYawBuilder
{
public:
    WRO_MOCK_BUILDER_BOILERPLATE(TurnToYawBuilder)
    TurnToYawBuilder& setPower(int16_t) { return *this; }
    TurnToYawBuilder& setPreset(Config::SpeedPreset) { return *this; }
    TurnToYawBuilder& setMode(uint8_t) { return *this; }
    TurnToYawBuilder& setBrake(bool) { return *this; }
    TurnToYawBuilder& setBlocking(bool) { return *this; }
    TurnToYawBuilder& async() { return *this; }
};

class [[nodiscard]] StopAtLineBuilder
{
public:
    WRO_MOCK_BUILDER_BOILERPLATE(StopAtLineBuilder)
    StopAtLineBuilder& setPower(int16_t) { return *this; }
    StopAtLineBuilder& setPreset(Config::SpeedPreset) { return *this; }
    StopAtLineBuilder& setMode(DriveMode) { return *this; }
    StopAtLineBuilder& setHeading(int16_t) { return *this; }
    StopAtLineBuilder& setArmWithin(float) { return *this; }
    StopAtLineBuilder& setMaxDistance(float) { return *this; }
    StopAtLineBuilder& setOvershoot(float) { return *this; }
    StopAtLineBuilder& setReverse(bool) { return *this; }
    StopAtLineBuilder& setBrake(bool) { return *this; }
    StopAtLineBuilder& setBlocking(bool) { return *this; }
    StopAtLineBuilder& async() { return *this; }
};

#undef WRO_MOCK_BUILDER_BOILERPLATE

/**
 * @brief Captured details of the most recent `Drive` method call.
 */
struct DriveCallRecord
{
    // setSpeed counters
    uint32_t setSpeedCallCount = 0;
    int16_t  lastSetSpeedL     = 0;
    int16_t  lastSetSpeedR     = 0;

    // stop counters
    uint32_t stopCallCount       = 0;
    bool     lastStopBrake       = false;
    uint32_t stopBrakeTrueCount  = 0;
    uint32_t stopBrakeFalseCount = 0;
};

class DriveClass
{
public:
    // ------------------------------------------------------------------ ctor / reset
    DriveClass() = default;

    /// Reset all recorded state + encoder counters. Tests call this at the top
    /// of each scenario.
    void mockReset()
    {
        _record    = DriveCallRecord{};
        _encL      = 0;
        _encR      = 0;
        _isIdle    = true;
        _hasFault  = false;
    }

    // ------------------------------------------------------------------ surface used by LineFollower

    uint8_t setSpeed(int16_t L, int16_t R)
    {
        _record.setSpeedCallCount++;
        _record.lastSetSpeedL = L;
        _record.lastSetSpeedR = R;
        return 0;
    }

    uint8_t stop(bool brake = true)
    {
        _record.stopCallCount++;
        _record.lastStopBrake = brake;
        if (brake) { _record.stopBrakeTrueCount++; }
        else       { _record.stopBrakeFalseCount++; }
        return 0;
    }

    int32_t getEncL() const { return _encL; }
    int32_t getEncR() const { return _encR; }

    bool isIdle() const { return _isIdle; }

    // ------------------------------------------------------------------ test setters

    void mockSetEncL(int32_t v) { _encL = v; }
    void mockSetEncR(int32_t v) { _encR = v; }
    void mockAdvanceEncoders(int32_t deltaL, int32_t deltaR) { _encL += deltaL; _encR += deltaR; }
    void mockSetIdle(bool idle) { _isIdle = idle; }

    const DriveCallRecord& mockRecord() const { return _record; }

    // ------------------------------------------------------------------ task-body surface
    // Lifecycle / state / fluent verbs used by Mission + task bodies. All no-op;
    // every "done?" query reports idle so a blocking verb advances on host.

    /** @brief Fault enum mirror of the production facade (mock is always NONE). */
    enum class Fault : uint8_t
    {
        NONE            = 0,
        DRAIN_TIMEOUT   = 1,
        DISPATCH_FAILED = 2,
        LINE_NOT_FOUND  = 3
    };

    /** @brief Captured fault details mirror (mock reports no fault). */
    struct FaultInfo
    {
        Fault   kind = Fault::NONE;
        uint8_t rc   = 0x00;
    };

    // lifecycle
    bool begin() { return true; }
    void update() {}
    void setTickCallback(void (*)()) {}

    // continuous primitives (setSpeed / stop are above, in the LineFollower surface)
    uint8_t setSpeedSync(int16_t /*L*/, int16_t /*R*/) { return 0; }
    uint8_t forwardGyro(int16_t /*power*/) { return 0; }

    // fluent verbs
    [[nodiscard]] DriveDistBuilder  driveDist(float /*cm*/) { return DriveDistBuilder{}; }
    [[nodiscard]] DriveTimeBuilder  driveTime(float /*sec*/) { return DriveTimeBuilder{}; }
    [[nodiscard]] TurnDegBuilder    turnDeg(int16_t /*deg*/) { return TurnDegBuilder{}; }
    [[nodiscard]] TurnToYawBuilder  turnToYaw(int16_t /*yaw*/) { return TurnToYawBuilder{}; }
    [[nodiscard]] StopAtLineBuilder stopAtLine(float /*expectedCm*/) { return StopAtLineBuilder{}; }

    // state accessors
    bool isMoveDone() const { return true; }
    bool isTurnDone() const { return true; }
    bool isLineFound() const { return false; }
    void waitUntilIdle() {}

    // encoder / yaw
    void   resetEnc() { _encL = 0; _encR = 0; }
    double getYaw() const { return 0.0; }
    void   holdYaw() {}

    // fault state — never faulted
    bool      hasFault() const { return _hasFault; }
    FaultInfo getFault() const { return FaultInfo{}; }
    void      clearFault() { _hasFault = false; }

private:
    DriveCallRecord _record;
    int32_t         _encL     = 0;
    int32_t         _encR     = 0;
    bool            _isIdle   = true;
    bool            _hasFault = false;
};

/// Single global instance — defined in `mocks/Drive.cpp`.
extern DriveClass Drive;

#endif    // DRIVE_H
