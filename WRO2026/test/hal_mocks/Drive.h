/**
 * @file Drive.h (HAL HOST MOCK)
 * @brief Full host-test `DriveClass` mock — the surface `PathTracker.cpp` /
 *        `Mission.cpp` / task bodies call on the `Drive` facade.
 *
 * This is the shared `hal_mocks/` Drive surface (ADR — host-test mock HAL). It is a
 * COPY of `test/mocks/Drive.h` (which already provides the LineFollower surface plus
 * the task-body fluent verbs) extended with the #105 T2 delta:
 *   - a **test-settable** fault (`mockSetFault`) backed by the FULL production
 *     `Fault` / `FaultInfo` shape (5-value enum incl. `IMU_READ_FAILED`; 3-field
 *     struct incl. `cmdKind`), so #106's real `PathTracker.cpp` (which reads
 *     `getFault().kind` / `getFault().cmdKind`) links against a faithful type;
 *   - `mockSetLineFound(bool)` → `isLineFound()`;
 *   - **builder argument capture** — each fluent verb records its verb + key arg
 *     into a `DriveCallRecord` AT CONSTRUCTION (the C2027-safe free-function seam);
 *   - `wallStopTimedOut()` (test-settable).
 *
 * `LineFollower.cpp` (when this header shadows the real one) calls exactly:
 *   - `Drive.setSpeed(L, R)` / `Drive.stop(brake)` / `Drive.getEncL()` /
 *     `Drive.getEncR()` / `Drive.isIdle()`.
 * `PathTracker.cpp` / `Mission.cpp` additionally reach the fluent verbs, the
 * fault accessors, `isLineFound()`, `wallStopTimedOut()`, and `waitUntilIdle()`.
 *
 * ---------------------------------------------------------------------------
 * Fidelity — where this mock can lie vs the real `DriveClass`
 * ---------------------------------------------------------------------------
 *   - **NO completion modeling.** Every "done?" query is test-settable or reports
 *     instant idle: `isMoveDone()` / `isTurnDone()` are always `true`, and
 *     `isIdle()` returns `mockSetIdle(...)` (default `true`). The real class drains
 *     a queue against the STM32 `isPrevTaskDone()` bit across many ticks — the host
 *     cannot observe that coprocessor signal, so modeling it would invent timing
 *     that drifts from hardware (debate-rejected). A blocking verb therefore appears
 *     to complete in zero ticks here.
 *   - **NO motion is executed.** The fluent builders record their verb + argument
 *     and do nothing else; encoders only move when a test calls `mockAdvanceEncoders`.
 *   - **Faults are never raised on their own.** `hasFault()` is `false` until a test
 *     calls `mockSetFault(...)`; the real class latches `DRAIN_TIMEOUT` /
 *     `DISPATCH_FAILED` / `LINE_NOT_FOUND` / `IMU_READ_FAILED` from real lower-MCU
 *     failures the host can't reproduce.
 *   - **`isLineFound()` / `wallStopTimedOut()` are pure test-set state** — they do
 *     not follow from any seek/push the mock ran (it ran none).
 *   - **Builder argument capture is faithful for the KEY arg only** (the verb's
 *     leading distance/time/angle/yaw). Chain-setter values (`.setPower`, `.async`,
 *     etc.) are accepted and discarded — no consumer asserts on them here.
 */

// NOTE: This mock uses the SAME include guard (`DRIVE_H`) as the production
// `WRO2026/Drive.h`. Force-included first by the `test_mock_hal` CMake wiring so the
// real header no-ops if reached afterward. `mocks/Drive.h` and this header are two
// different `class DriveClass` definitions sharing this guard — they must never be
// reachable by the same target (ODR mutual-exclusion invariant, #105).
#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

// Production `LineFollower.cpp` relies on `Drive.h` transitively pulling in
// `WROMath.h` (which holds the free functions `deadbandSnap`, `presetToPwm`,
// `encDegToCm`). We mirror that include order here so a controller TU that links
// this mock finds those symbols.
#include "src/util/WROMath.h"
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

/**
 * @brief Discriminated-command shape mirror — carries only the `Kind` enum the
 *        production `FaultInfo::cmdKind` field is typed on.
 *
 * The real `DriveCommand` (in `WRO2026/Drive.h`) is a rich queued-command struct;
 * the mock only needs its nested `Kind` enum to resolve so `FaultInfo` below mirrors
 * the production type byte-for-byte. The enumerators match the production set exactly
 * (#106's `PathTracker.cpp` reads `getFault().cmdKind`).
 */
struct DriveCommand
{
    /**
     * @brief Discriminator for the offending queued command (mirror of production).
     */
    enum class Kind : uint8_t
    {
        DRIVE_DIST,      ///< Non-profiled distance move (mode-driven).
        DRIVE_TIME,      ///< Non-profiled time move (mode-driven).
        TURN_DEG,        ///< Relative turn.
        TURN_TO_YAW,     ///< Absolute-yaw turn.
        PROFILE_DIST,    ///< Profiled (tShape) distance move.
        PROFILE_TIME,    ///< Profiled (tShape) time move.
        STOP_AT_LINE,    ///< Drive-until-line move.
        WALL_STOP        ///< Push-until-encoder-stall move.
    };
};

// Forward declaration so the builders can name `DriveClass&` before it is defined.
// The builders MUST NOT dereference it inline (MSVC `C2027: use of undefined type`);
// they call the free `_recordDrive*` functions below, which are DECLARED here and
// DEFINED out-of-line after `DriveClass` is complete.
class DriveClass;

// ---------------------------------------------------------------------------
// Free builder-capture recording functions — the C2027-safe seam.
//
// Each fluent verb constructs exactly one builder as a discarded temporary
// (`Drive.stopAtLine(30).setReverse(true).async();`). The builder ctor records the
// verb + key arg by calling one of these free functions, which write to the
// test-inspectable `DriveCallRecord` on the facade. Declared before the builders
// (so the ctors can call them) and defined after `DriveClass` (so they can touch its
// record without an incomplete-type dereference).
// ---------------------------------------------------------------------------
void _recordDriveDist(DriveClass& d, float cm);
void _recordDriveTime(DriveClass& d, float sec);
void _recordTurnDeg(DriveClass& d, int16_t deg);
void _recordTurnToYaw(DriveClass& d, int16_t yaw);
void _recordStopAtLine(DriveClass& d, float expectedCm);
void _recordWallStop(DriveClass& d, float cm);

// ---------------------------------------------------------------------------
// Move-only builders mirroring the production fluent verbs. Task
// bodies use them as discarded temporaries (`Drive.driveDist(30).tShape();`), so
// every chain setter returns `*this`. UNLIKE the production builders (and the
// `mocks/Drive.h` no-op builders), these record their verb + key arg AT
// CONSTRUCTION via the free `_recordDrive*` functions — the single chaining-proof
// capture seam (the verb constructs one builder; the chain setters all return
// `*this`; the temporary is discarded). The owner reference is forward-declared and
// touched ONLY through the free functions, never inline (the MSVC C2027 trap).
// ---------------------------------------------------------------------------

/// @brief Shared macro emitting the move-only boilerplate (no default
///        ctor — every builder is constructed with its owner + key arg).
#define WRO_HAL_BUILDER_BOILERPLATE(NAME)                                   \
    NAME(NAME&&) noexcept            = default;                             \
    NAME(const NAME&)                = delete;                              \
    NAME& operator=(const NAME&)     = delete;                             \
    NAME& operator=(NAME&&)          = delete;

/**
 * @brief Recording mock of `DriveDistBuilder`. Records `driveDist(cm)` on construction.
 */
class DriveDistBuilder
{
public:
    /** @brief Construct + record the move immediately (call site is a discarded temporary). */
    DriveDistBuilder(DriveClass& drive, float cm) { _recordDriveDist(drive, cm); }
    WRO_HAL_BUILDER_BOILERPLATE(DriveDistBuilder)

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

/**
 * @brief Recording mock of `DriveTimeBuilder`. Records `driveTime(sec)` on construction.
 */
class DriveTimeBuilder
{
public:
    /** @brief Construct + record the move immediately (call site is a discarded temporary). */
    DriveTimeBuilder(DriveClass& drive, float sec) { _recordDriveTime(drive, sec); }
    WRO_HAL_BUILDER_BOILERPLATE(DriveTimeBuilder)

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

/**
 * @brief Recording mock of `TurnDegBuilder`. Records `turnDeg(deg)` on construction.
 */
class TurnDegBuilder
{
public:
    /** @brief Construct + record the turn immediately (call site is a discarded temporary). */
    TurnDegBuilder(DriveClass& drive, int16_t deg) { _recordTurnDeg(drive, deg); }
    WRO_HAL_BUILDER_BOILERPLATE(TurnDegBuilder)

    TurnDegBuilder& setPower(int16_t) { return *this; }
    TurnDegBuilder& setPreset(Config::SpeedPreset) { return *this; }
    TurnDegBuilder& setMode(uint8_t) { return *this; }
    TurnDegBuilder& setBrake(bool) { return *this; }
    TurnDegBuilder& setBlocking(bool) { return *this; }
    TurnDegBuilder& async() { return *this; }
};

/**
 * @brief Recording mock of `TurnToYawBuilder`. Records `turnToYaw(yaw)` on construction.
 */
class TurnToYawBuilder
{
public:
    /** @brief Construct + record the turn immediately (call site is a discarded temporary). */
    TurnToYawBuilder(DriveClass& drive, int16_t yaw) { _recordTurnToYaw(drive, yaw); }
    WRO_HAL_BUILDER_BOILERPLATE(TurnToYawBuilder)

    TurnToYawBuilder& setPower(int16_t) { return *this; }
    TurnToYawBuilder& setPreset(Config::SpeedPreset) { return *this; }
    TurnToYawBuilder& setMode(uint8_t) { return *this; }
    TurnToYawBuilder& setBrake(bool) { return *this; }
    TurnToYawBuilder& setBlocking(bool) { return *this; }
    TurnToYawBuilder& async() { return *this; }
};

/**
 * @brief Recording mock of `StopAtLineBuilder`. Records `stopAtLine(expectedCm)` on construction.
 */
class StopAtLineBuilder
{
public:
    /** @brief Construct + record the seek immediately (call site is a discarded temporary). */
    StopAtLineBuilder(DriveClass& drive, float expectedCm) { _recordStopAtLine(drive, expectedCm); }
    WRO_HAL_BUILDER_BOILERPLATE(StopAtLineBuilder)

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

/**
 * @brief Recording mock of `WallStopBuilder`. Records `wallStop(cm)` on construction.
 */
class WallStopBuilder
{
public:
    /** @brief Construct + record the push immediately (call site is a discarded temporary). */
    WallStopBuilder(DriveClass& drive, float cm) { _recordWallStop(drive, cm); }
    WRO_HAL_BUILDER_BOILERPLATE(WallStopBuilder)

    WallStopBuilder& setPower(int16_t) { return *this; }
    WallStopBuilder& setPreset(Config::SpeedPreset) { return *this; }
    WallStopBuilder& setMode(DriveMode) { return *this; }
    WallStopBuilder& setHeading(int16_t) { return *this; }
    WallStopBuilder& setReverse(bool) { return *this; }
    WallStopBuilder& setBackOff(float) { return *this; }
    WallStopBuilder& setTimeout(uint32_t) { return *this; }
    WallStopBuilder& setBrake(bool) { return *this; }
    WallStopBuilder& setBlocking(bool) { return *this; }
    WallStopBuilder& async() { return *this; }
};

#undef WRO_HAL_BUILDER_BOILERPLATE

/**
 * @brief Which fluent verb constructed the most recently captured builder.
 *
 * Lets a test assert *which* verb was issued without inspecting per-verb fields. `NONE`
 * is the post-`mockReset()` default (no builder constructed yet).
 */
enum class DriveVerb : uint8_t
{
    NONE,
    DRIVE_DIST,
    DRIVE_TIME,
    TURN_DEG,
    TURN_TO_YAW,
    STOP_AT_LINE,
    WALL_STOP
};

/**
 * @brief Captured details of recent `Drive` method calls.
 *
 * Holds the continuous-primitive counters (`setSpeed` / `stop`, used by the
 * `LineFollower` surface) plus the fluent-builder argument capture (recorded at
 * builder construction via the free `_recordDrive*` functions). Each builder bumps a
 * per-verb call count and stashes its key argument, and updates `lastVerb` so a test
 * can assert verb ORDER/identity as well as the captured value.
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

    // -------- fluent builder argument capture (recorded at construction) --------
    DriveVerb lastVerb = DriveVerb::NONE;    ///< Verb of the most recent builder.

    uint32_t driveDistCallCount  = 0;    ///< `driveDist(cm)` constructions.
    uint32_t driveTimeCallCount  = 0;    ///< `driveTime(sec)` constructions.
    uint32_t turnDegCallCount    = 0;    ///< `turnDeg(deg)` constructions.
    uint32_t turnToYawCallCount  = 0;    ///< `turnToYaw(yaw)` constructions.
    uint32_t stopAtLineCallCount = 0;    ///< `stopAtLine(expectedCm)` constructions.
    uint32_t wallStopCallCount   = 0;    ///< `wallStop(cm)` constructions.

    float   lastDriveDistCm  = 0.0f;    ///< Key arg of the last `driveDist`.
    float   lastDriveTimeSec = 0.0f;    ///< Key arg of the last `driveTime`.
    int16_t lastTurnDegDeg   = 0;       ///< Key arg of the last `turnDeg`.
    int16_t lastTurnToYawDeg = 0;       ///< Key arg of the last `turnToYaw`.
    float   lastStopAtLineCm = 0.0f;    ///< Key arg of the last `stopAtLine`.
    float   lastWallStopCm   = 0.0f;    ///< Key arg of the last `wallStop`.
};

/**
 * @brief Full host-test mock of the Drive movement-primitive facade.
 *
 * Record-and-test-set: continuous primitives + the `LineFollower` surface record
 * their args; the fluent verbs return recording builders; completion / fault /
 * line-found / wall-timeout state is plain test-settable state. See the file-level
 * Fidelity docblock for what this mock deliberately does NOT model.
 */
class DriveClass
{
public:
    // ------------------------------------------------------------------ ctor / reset
    DriveClass() = default;

    /**
     * @brief Reset all recorded state, encoder counters, and test-set flags.
     *
     * Tests call this at the top of each scenario. Restores the fault-free,
     * instantly-idle, no-line-found, no-wall-timeout defaults.
     */
    void mockReset()
    {
        _record           = DriveCallRecord{};
        _encL             = 0;
        _encR             = 0;
        _isIdle           = true;
        _fault            = FaultInfo{};
        _lineFound        = false;
        _wallStopTimedOut = false;
    }

    // ------------------------------------------------------------------ surface used by LineFollower

    /**
     * @brief Record a continuous raw-PWM command. No motion is executed.
     * @param L Left-motor power in `[-100, 100]`.
     * @param R Right-motor power in `[-100, 100]`.
     * @return Always `0x00` (success).
     */
    uint8_t setSpeed(int16_t L, int16_t R, int16_t /*floorMag*/ = Config::PWM_MIN_MAGNITUDE)
    {
        _record.setSpeedCallCount++;
        _record.lastSetSpeedL = L;
        _record.lastSetSpeedR = R;
        return 0;
    }

    /**
     * @brief Record a stop command and the requested brake/coast mode.
     * @param brake `true` (default) = active brake; `false` = coast.
     * @return Always `0x00` (success).
     */
    uint8_t stop(bool brake = true)
    {
        _record.stopCallCount++;
        _record.lastStopBrake = brake;
        if (brake) { _record.stopBrakeTrueCount++; }
        else       { _record.stopBrakeFalseCount++; }
        return 0;
    }

    /** @brief Left encoder reading in degrees (test-controlled via `mockSetEncL`). */
    int32_t getEncL() const { return _encL; }
    /** @brief Right encoder reading in degrees (test-controlled via `mockSetEncR`). */
    int32_t getEncR() const { return _encR; }

    /** @brief Whether the chassis (and queue) is idle. Test-settable; default `true`. */
    bool isIdle() const { return _isIdle; }

    // ------------------------------------------------------------------ test setters

    /** @brief Set the left encoder reading (degrees). */
    void mockSetEncL(int32_t v) { _encL = v; }
    /** @brief Set the right encoder reading (degrees). */
    void mockSetEncR(int32_t v) { _encR = v; }
    /** @brief Advance both encoders to simulate forward chassis motion between ticks. */
    void mockAdvanceEncoders(int32_t deltaL, int32_t deltaR) { _encL += deltaL; _encR += deltaR; }
    /** @brief Set the value `isIdle()` reports. */
    void mockSetIdle(bool idle) { _isIdle = idle; }

    /** @brief Read-only access to the captured call record. */
    const DriveCallRecord& mockRecord() const { return _record; }

    // ------------------------------------------------------------------ task-body surface
    // Lifecycle / state / fluent verbs used by Mission + PathTracker + task bodies.
    // Completion queries report idle so a blocking verb advances instantly on host.

    /**
     * @brief Source of a Drive-layer fault — mirror of the production enum (5 values).
     */
    enum class Fault : uint8_t
    {
        NONE            = 0,    ///< No fault — normal operation.
        DRAIN_TIMEOUT   = 1,    ///< `_drainBlocking` deadline exceeded.
        DISPATCH_FAILED = 2,    ///< `_dispatchNext` saw a non-zero rc from the lower MCU.
        LINE_NOT_FOUND  = 3,    ///< `stopAtLine` ran the full max distance with no crossing found.
        IMU_READ_FAILED = 4     ///< Custom TurnGyro profile lost the IMU yaw read past its watchdog.
    };

    /**
     * @brief Captured details of the last Drive fault — mirror of the production struct (3 fields).
     */
    struct FaultInfo
    {
        Fault              kind    = Fault::NONE;
        uint8_t            rc      = 0x00;                              ///< Lower-MCU error code (`DISPATCH_FAILED` only).
        DriveCommand::Kind cmdKind = DriveCommand::Kind::DRIVE_DIST;    ///< Kind of the offending command.
    };

    // lifecycle
    /** @brief No-op bring-up; always reports success. */
    bool begin() { return true; }
    /** @brief No-op cooperative tick. */
    void update() {}
    /** @brief No-op tick-callback registration. */
    void setTickCallback(void (*)()) {}

    // continuous primitives (setSpeed / stop are above, in the LineFollower surface)
    /** @brief Record-free no-op MoveSync continuous primitive. @return `0x00`. */
    uint8_t setSpeedSync(int16_t /*L*/, int16_t /*R*/) { return 0; }
    /** @brief Record-free no-op MoveGyro continuous primitive. @return `0x00`. */
    uint8_t forwardGyro(int16_t /*power*/) { return 0; }

    // fluent verbs — return recording builders (which capture verb + key arg on construction)
    /** @brief Build a distance-bounded move; records `driveDist(cm)` immediately. */
    DriveDistBuilder driveDist(float cm) { return DriveDistBuilder(*this, cm); }
    /** @brief Build a time-bounded move; records `driveTime(sec)` immediately. */
    DriveTimeBuilder driveTime(float sec) { return DriveTimeBuilder(*this, sec); }
    /** @brief Build a relative turn; records `turnDeg(deg)` immediately. */
    TurnDegBuilder turnDeg(int16_t deg) { return TurnDegBuilder(*this, deg); }
    /** @brief Build an absolute-yaw turn; records `turnToYaw(yaw)` immediately. */
    TurnToYawBuilder turnToYaw(int16_t yaw) { return TurnToYawBuilder(*this, yaw); }
    /** @brief Build a drive-until-line seek; records `stopAtLine(expectedCm)` immediately. */
    StopAtLineBuilder stopAtLine(float expectedCm)
    {
        return StopAtLineBuilder(*this, expectedCm);
    }
    /** @brief Build a push-until-wall move; records `wallStop(cm)` immediately. */
    WallStopBuilder wallStop(float cm) { return WallStopBuilder(*this, cm); }

    // state accessors
    /** @brief Whether the last bounded move finished. Always `true` (no completion modeling). */
    bool isMoveDone() const { return true; }
    /** @brief Whether the last async turn finished. Always `true` (no completion modeling). */
    bool isTurnDone() const { return true; }
    /** @brief Latched line-found result of the last `stopAtLine`. Test-settable; default `false`. */
    bool isLineFound() const { return _lineFound; }
    /**
     * @brief Advisory result of the last `wallStop`: did it time out without stalling?
     *        Test-settable; default `false`. NON-LATCHING — independent of `hasFault()`.
     */
    bool wallStopTimedOut() const { return _wallStopTimedOut; }
    /** @brief No-op spin-until-idle (the mock is always idle). */
    void waitUntilIdle() {}

    // encoder / yaw
    /** @brief Reset both encoder counters to zero. */
    void   resetEnc() { _encL = 0; _encR = 0; }
    /** @brief Current IMU yaw in the corrected app frame. Always `0.0` (no IMU on host). */
    double getYaw() const { return 0.0; }
    /** @brief Snapshot the current yaw as the held target. No-op on host. */
    void   holdYaw() {}

    // fault state — test-settable
    /** @brief Whether a fault is currently latched. @return `true` if `getFault().kind != NONE`. */
    bool      hasFault() const { return _fault.kind != Fault::NONE; }
    /** @brief Snapshot of the latched fault (test-set via `mockSetFault`). */
    FaultInfo getFault() const { return _fault; }
    /** @brief Clear the latched fault so a consumer's re-arm path proceeds. */
    void      clearFault() { _fault = FaultInfo{}; }

    // ------------------------------------------------------------------ test setters (delta)

    /**
     * @brief Latch a fault so `hasFault()` returns `true` and `getFault().kind` returns @p kind.
     *
     * The `rc` / `cmdKind` fields keep their production defaults — #105's self-test does not
     * exercise them, but the fields are present so #106's `PathTracker.cpp` (which reads
     * `getFault().kind` / `.cmdKind`) compiles. Use the richer overload to set them.
     *
     * @param kind Fault kind to latch (`NONE` effectively clears the fault).
     */
    void mockSetFault(Fault kind)
    {
        _fault.kind = kind;
    }

    /**
     * @brief Latch a fully-specified fault (kind + lower-MCU rc + offending command kind).
     * @param kind    Fault kind to latch.
     * @param rc      Lower-MCU error code to report (meaningful for `DISPATCH_FAILED`).
     * @param cmdKind Kind of the offending queued command.
     */
    void mockSetFault(Fault kind, uint8_t rc, DriveCommand::Kind cmdKind)
    {
        _fault.kind    = kind;
        _fault.rc      = rc;
        _fault.cmdKind = cmdKind;
    }

    /** @brief Set the value `isLineFound()` reports. */
    void mockSetLineFound(bool found) { _lineFound = found; }

    /** @brief Set the value `wallStopTimedOut()` reports. */
    void mockSetWallStopTimedOut(bool timedOut) { _wallStopTimedOut = timedOut; }

    // ------------------------------------------------------------------ builder-capture sinks
    // Called by the free `_recordDrive*` functions (defined out-of-line below) so the builders
    // never dereference this (then-incomplete) type inline — the MSVC C2027 trap.

    /** @brief Record a `driveDist(cm)` builder construction. */
    void mockCaptureDriveDist(float cm)
    {
        _record.driveDistCallCount++;
        _record.lastDriveDistCm = cm;
        _record.lastVerb        = DriveVerb::DRIVE_DIST;
    }
    /** @brief Record a `driveTime(sec)` builder construction. */
    void mockCaptureDriveTime(float sec)
    {
        _record.driveTimeCallCount++;
        _record.lastDriveTimeSec = sec;
        _record.lastVerb         = DriveVerb::DRIVE_TIME;
    }
    /** @brief Record a `turnDeg(deg)` builder construction. */
    void mockCaptureTurnDeg(int16_t deg)
    {
        _record.turnDegCallCount++;
        _record.lastTurnDegDeg = deg;
        _record.lastVerb       = DriveVerb::TURN_DEG;
    }
    /** @brief Record a `turnToYaw(yaw)` builder construction. */
    void mockCaptureTurnToYaw(int16_t yaw)
    {
        _record.turnToYawCallCount++;
        _record.lastTurnToYawDeg = yaw;
        _record.lastVerb         = DriveVerb::TURN_TO_YAW;
    }
    /** @brief Record a `stopAtLine(expectedCm)` builder construction. */
    void mockCaptureStopAtLine(float expectedCm)
    {
        _record.stopAtLineCallCount++;
        _record.lastStopAtLineCm = expectedCm;
        _record.lastVerb         = DriveVerb::STOP_AT_LINE;
    }
    /** @brief Record a `wallStop(cm)` builder construction. */
    void mockCaptureWallStop(float cm)
    {
        _record.wallStopCallCount++;
        _record.lastWallStopCm = cm;
        _record.lastVerb       = DriveVerb::WALL_STOP;
    }

private:
    DriveCallRecord _record;
    int32_t         _encL             = 0;
    int32_t         _encR             = 0;
    bool            _isIdle           = true;
    FaultInfo       _fault;
    bool            _lineFound        = false;
    bool            _wallStopTimedOut = false;
};

// ---------------------------------------------------------------------------
// Out-of-line definitions of the free builder-capture functions. Defined HERE,
// after `DriveClass` is complete, so they can call its `mockCapture*` members
// without the incomplete-type dereference an inline builder ctor would trigger
// (MSVC `C2027`). Marked `inline` because this header is included by multiple TUs.
// ---------------------------------------------------------------------------
inline void _recordDriveDist(DriveClass& d, float cm) { d.mockCaptureDriveDist(cm); }
inline void _recordDriveTime(DriveClass& d, float sec) { d.mockCaptureDriveTime(sec); }
inline void _recordTurnDeg(DriveClass& d, int16_t deg) { d.mockCaptureTurnDeg(deg); }
inline void _recordTurnToYaw(DriveClass& d, int16_t yaw) { d.mockCaptureTurnToYaw(yaw); }
inline void _recordStopAtLine(DriveClass& d, float expectedCm) { d.mockCaptureStopAtLine(expectedCm); }
inline void _recordWallStop(DriveClass& d, float cm) { d.mockCaptureWallStop(cm); }

/// Single global instance — defined in `hal_mocks/hal_mocks.cpp` (T4).
extern DriveClass Drive;

#endif    // DRIVE_H
