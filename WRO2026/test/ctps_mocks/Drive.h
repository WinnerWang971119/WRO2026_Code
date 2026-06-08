/**
 * @file Drive.h (CTPS HOST MOCK)
 * @brief Recording mock of the Drive movement-primitive facade for the CTPS dump.
 * @author WRO 2026
 *
 * @details
 * Shadows the real `Drive.h` (quote-included; force-included by the T4 cmake
 * wiring). The include guard is the REAL one (`DRIVE_H`) so the real header
 * no-ops if reached afterwards.
 *
 * Captures task-internal RAW moves. Each of the five fluent builders records
 * exactly one `drive` op into the shared `g_ctpsLog` AT CONSTRUCTION — the verb
 * call site (`Drive.driveDist(15);`) is a discarded temporary, so recording in
 * the constructor (rather than the destructor, as the real builders do) is the
 * single, chaining-proof seam: the verb constructs one builder, the chain
 * setters all return `*this`, and the temporary is discarded.
 *
 * The builders mirror the real move-only `[[nodiscard]]` shape (deleted copy
 * ctor / copy-assign) so discarded-temporary call sites compile exactly as they
 * do against the real header. All "is-done?" queries report instant completion
 * (`isIdle()` → true, `hasFault()` → false) so task bodies' fake-blocking drain
 * exits in one iteration and the dump never hangs.
 *
 * `goto` is NOT recorded here — it is captured by the PathTracker.goTo mock.
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

#include "ctps_record.h"

class DriveClass;

/**
 * @brief Mode selector mirroring the real `DriveMode` so `.setMode(...)` chains compile.
 */
enum class DriveMode : uint8_t
{
    RAW,
    SYNC,
    GYRO
};

/**
 * @brief Recording mock of `DriveDistBuilder`. Records `driveDist` on construction.
 */
class [[nodiscard]] DriveDistBuilder
{
public:
    /** @brief Construct + record the move immediately (call site is a discarded temporary). */
    DriveDistBuilder(DriveClass& /*drive*/, float cm) { ctps::recordDriveDist(cm); }

    DriveDistBuilder(const DriveDistBuilder&)            = delete;
    DriveDistBuilder& operator=(const DriveDistBuilder&) = delete;
    DriveDistBuilder& operator=(DriveDistBuilder&&)      = delete;

    // Chain setters — no-op, return *this so any chained call site compiles.
    DriveDistBuilder& setPower(int16_t) { return *this; }
    DriveDistBuilder& setMode(DriveMode) { return *this; }
    DriveDistBuilder& setBrake(bool) { return *this; }
    DriveDistBuilder& setBlocking(bool) { return *this; }
    DriveDistBuilder& async() { return *this; }
    DriveDistBuilder& setHeading(int16_t) { return *this; }
    DriveDistBuilder& tShape() { return *this; }
    DriveDistBuilder& setAccel(float) { return *this; }
    DriveDistBuilder& setStartSpeed(int16_t) { return *this; }
    DriveDistBuilder& setEndSpeed(int16_t) { return *this; }
};

/**
 * @brief Recording mock of `DriveTimeBuilder`.
 *
 * A time-bounded move carries no distance/angle the CTPS geometry layer can use,
 * so it is recorded as a non-geometry `drive` entry (`method == "stop"`) — this
 * preserves execution order without injecting a bogus distance the loader would
 * misinterpret. `driveTime` is reproduced mainly so its call site compiles;
 * authored task bodies use `driveDist` / `turnDeg` / `turnToYaw` for geometry.
 */
class [[nodiscard]] DriveTimeBuilder
{
public:
    DriveTimeBuilder(DriveClass& /*drive*/, float /*sec*/) { ctps::recordDriveStop(); }

    DriveTimeBuilder(const DriveTimeBuilder&)            = delete;
    DriveTimeBuilder& operator=(const DriveTimeBuilder&) = delete;
    DriveTimeBuilder& operator=(DriveTimeBuilder&&)      = delete;

    DriveTimeBuilder& setPower(int16_t) { return *this; }
    DriveTimeBuilder& setMode(DriveMode) { return *this; }
    DriveTimeBuilder& setBrake(bool) { return *this; }
    DriveTimeBuilder& setBlocking(bool) { return *this; }
    DriveTimeBuilder& async() { return *this; }
    DriveTimeBuilder& setHeading(int16_t) { return *this; }
    DriveTimeBuilder& tShape() { return *this; }
    DriveTimeBuilder& setAccel(float) { return *this; }
    DriveTimeBuilder& setStartSpeed(int16_t) { return *this; }
    DriveTimeBuilder& setEndSpeed(int16_t) { return *this; }
};

/**
 * @brief Recording mock of `TurnDegBuilder`. Records `turnDeg` on construction.
 */
class [[nodiscard]] TurnDegBuilder
{
public:
    TurnDegBuilder(DriveClass& /*drive*/, int16_t deg)
    {
        ctps::recordTurnDeg(static_cast<float>(deg));
    }

    TurnDegBuilder(const TurnDegBuilder&)            = delete;
    TurnDegBuilder& operator=(const TurnDegBuilder&) = delete;
    TurnDegBuilder& operator=(TurnDegBuilder&&)      = delete;

    TurnDegBuilder& setPower(int16_t) { return *this; }
    TurnDegBuilder& setMode(uint8_t) { return *this; }
    TurnDegBuilder& setBrake(bool) { return *this; }
    TurnDegBuilder& setBlocking(bool) { return *this; }
    TurnDegBuilder& async() { return *this; }
};

/**
 * @brief Recording mock of `TurnToYawBuilder`. Records `turnToYaw` on construction.
 */
class [[nodiscard]] TurnToYawBuilder
{
public:
    TurnToYawBuilder(DriveClass& /*drive*/, int16_t yaw)
    {
        ctps::recordTurnToYaw(static_cast<float>(yaw));
    }

    TurnToYawBuilder(const TurnToYawBuilder&)            = delete;
    TurnToYawBuilder& operator=(const TurnToYawBuilder&) = delete;
    TurnToYawBuilder& operator=(TurnToYawBuilder&&)      = delete;

    TurnToYawBuilder& setPower(int16_t) { return *this; }
    TurnToYawBuilder& setMode(uint8_t) { return *this; }
    TurnToYawBuilder& setBrake(bool) { return *this; }
    TurnToYawBuilder& setBlocking(bool) { return *this; }
    TurnToYawBuilder& async() { return *this; }
};

/**
 * @brief Recording mock of `StopAtLineBuilder`. Records `stopAtLine` on construction.
 */
class [[nodiscard]] StopAtLineBuilder
{
public:
    StopAtLineBuilder(DriveClass& /*drive*/, float expectedCm)
    {
        ctps::recordStopAtLine(expectedCm);
    }

    StopAtLineBuilder(const StopAtLineBuilder&)            = delete;
    StopAtLineBuilder& operator=(const StopAtLineBuilder&) = delete;
    StopAtLineBuilder& operator=(StopAtLineBuilder&&)      = delete;

    StopAtLineBuilder& setPower(int16_t) { return *this; }
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

/**
 * @brief Recording mock of the Drive facade.
 *
 * The fluent verbs return the recording builders by value; the builders record
 * at construction. The continuous primitives + state accessors are no-ops that
 * report an instantly-idle, fault-free chassis.
 */
class DriveClass
{
public:
    /** @brief No-op bring-up; always reports success. */
    bool begin() { return true; }

    /** @brief No-op cooperative tick. */
    void update() {}

    /** @brief No-op tick-callback registration. */
    void setTickCallback(void (*)()) {}

    // --- Fluent verbs — return recording builders (which record on construction). ---
    [[nodiscard]] DriveDistBuilder driveDist(float cm) { return DriveDistBuilder(*this, cm); }
    [[nodiscard]] DriveTimeBuilder driveTime(float sec) { return DriveTimeBuilder(*this, sec); }
    [[nodiscard]] TurnDegBuilder   turnDeg(int16_t deg) { return TurnDegBuilder(*this, deg); }
    [[nodiscard]] TurnToYawBuilder turnToYaw(int16_t yaw) { return TurnToYawBuilder(*this, yaw); }
    [[nodiscard]] StopAtLineBuilder stopAtLine(float expectedCm)
    {
        return StopAtLineBuilder(*this, expectedCm);
    }

    // --- Continuous primitives — no-op. ---
    uint8_t setSpeed(int16_t, int16_t) { return 0x00; }
    uint8_t setSpeedSync(int16_t, int16_t) { return 0x00; }
    uint8_t forwardGyro(int16_t) { return 0x00; }

    /**
     * @brief Records a `stop` drive op so a task-body `Drive.stop()` stays in
     *        execution order, then reports success.
     *
     * Mission.cpp's safety-stop path (`abort()` / `_enterFaultBeeping`) also
     * calls this; those are mission-control housekeeping rather than authored
     * motion, but recording them is harmless — the loader treats `stop` as a
     * non-geometry primitive that does not move the chassis.
     */
    uint8_t stop(bool /*brake*/ = true)
    {
        ctps::recordDriveStop();
        return 0x00;
    }

    // --- State accessors — report instantly-idle, fault-free. ---
    bool isMoveDone() const { return true; }
    bool isTurnDone() const { return true; }
    bool isIdle() const { return true; }
    bool isLineFound() const { return true; }
    void waitUntilIdle() {}

    int32_t getEncL() const { return 0; }
    int32_t getEncR() const { return 0; }
    void    resetEnc() {}
    double  getYaw() const { return 0.0; }
    void    holdYaw() {}

    // --- Motor-bias calibration — no-op. ---
    uint8_t calibrateMotorBias(int16_t = 0, uint32_t = 0) { return 0x00; }
    void    setMotorBias(float, float) {}
    void    resetMotorBias() {}
    float   getBiasL() const { return 1.0f; }
    float   getBiasR() const { return 1.0f; }

    // --- Fault state — never faulted. ---
    enum class Fault : uint8_t
    {
        NONE            = 0,
        DRAIN_TIMEOUT   = 1,
        DISPATCH_FAILED = 2,
        LINE_NOT_FOUND  = 3
    };

    /** @brief Captured fault details mirror (always NONE). */
    struct FaultInfo
    {
        Fault   kind = Fault::NONE;
        uint8_t rc   = 0x00;
    };

    bool      hasFault() const { return false; }
    FaultInfo getFault() const { return FaultInfo{}; }
    void      clearFault() {}
};

/** @brief Single global instance — defined in ctps_record.cpp. */
extern DriveClass Drive;

#endif    // DRIVE_H
