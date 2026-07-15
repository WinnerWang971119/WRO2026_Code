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
 *
 * @note (ADR-059 / T10) `DriveDistBuilder` is the ONE builder among the six
 *       promoted from a pure no-op to a RECORDING RAII builder: its destructor
 *       writes the resolved `cm` / `setStartSpeed` / `setEndSpeed` / `setBrake` /
 *       `tShape` knobs into `DriveClass::mockRecord()` (mirroring the REAL
 *       `DriveDistBuilder`'s forward-declare-then-define-after-`DriveClass`
 *       ordering below, so the destructor body sees a complete `DriveClass`).
 *       This is a superset carry: nothing in this file's ONLY current consumer
 *       (`test_line_follower_overshoot`, which never calls `driveDist(...)`)
 *       observes the change, but a no-op setter here is a documented
 *       false-green for any future test asserting on it (memory
 *       `gotcha-ctps-mock-noop-setter-drops-builder-knob`, ADR-049). The other
 *       five builders (`DriveTimeBuilder`/`TurnDegBuilder`/`TurnToYawBuilder`/
 *       `StopAtLineBuilder`/`WallStopBuilder`) stay untouched no-ops — out of
 *       scope for the blind-chain assertions this task adds.
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

// Forward declaration so `DriveDistBuilder` can name-friend / reference `DriveClass`
// before it is defined below — the SAME ordering trick the real `Drive.h` uses
// (forward-declare the builders, define the facade, define the builder bodies that
// need a complete `DriveClass` AFTER it). Only `DriveDistBuilder` needs this; the
// other five builders stay pure no-ops with no owner reference.
class DriveClass;

// ---------------------------------------------------------------------------
// No-op move-only builders mirroring the production fluent
// verbs. Task bodies use them as discarded temporaries
// (`Drive.driveDist(30).tShape();`), so construction / destruction do nothing
// and every chain setter returns `*this`. The mock builders carry no owner
// reference (the real ones do) because they perform no work — this also avoids
// the facade/builder definition-order cycle.
//
// EXCEPTION: `DriveDistBuilder` (below) is a RECORDING builder — it DOES carry a
// `DriveClass&` and its destructor writes into `DriveClass::mockRecord()`. See the
// file-level ADR-059/T10 note above.
// ---------------------------------------------------------------------------

/// @brief Shared macro emitting a move-only builder body with the
///        given chain setters. Keeps the five near-identical no-op mock builders terse.
#define WRO_MOCK_BUILDER_BOILERPLATE(NAME)                                  \
    NAME() = default;                                                       \
    NAME(NAME&&) noexcept            = default;                             \
    NAME(const NAME&)                = delete;                              \
    NAME& operator=(const NAME&)     = delete;                             \
    NAME& operator=(NAME&&)          = delete;

/**
 * @brief RECORDING `driveDist(...)` builder (ADR-059 / T10).
 *
 * Unlike its five no-op siblings, this builder is bound to the owning
 * `DriveClass&` at construction (mirroring the REAL `DriveDistBuilder`) and its
 * destructor writes the resolved knobs into `DriveClass::mockRecord()` — the
 * blind-chain assertions reduce to "the builder recorded `setStartSpeed` /
 * `setEndSpeed` / `setBrake(false)` and NOT a `Drive.stop()` between hops", so a
 * no-op setter here would silently drop exactly the knobs under test (memory
 * `gotcha-ctps-mock-noop-setter-drops-builder-knob`).
 *
 * Move-only RAII, single discarded-temporary usage (memory
 * `gotcha_drive_builder_raii`): the destructor fires exactly once, when the
 * chain statement ends, moving through zero or more `DriveDistBuilder&&` hops
 * along the way (mandatory copy elision in C++17, or the move ctor pre-17).
 */
class DriveDistBuilder
{
public:
    /// @brief Bind to the owning mock `Drive` and the requested distance.
    DriveDistBuilder(DriveClass& drive, float cm) : _drive(drive), _cm(cm)
    {
    }

    DriveDistBuilder(const DriveDistBuilder&)            = delete;
    DriveDistBuilder& operator=(const DriveDistBuilder&) = delete;
    DriveDistBuilder& operator=(DriveDistBuilder&&)      = delete;

    /// @brief Move ctor — transfers recording responsibility; the moved-from
    ///        instance is marked `_recorded` so its destructor is a no-op.
    DriveDistBuilder(DriveDistBuilder&& other) noexcept
        : _drive(other._drive),
          _cm(other._cm),
          _brake(other._brake),
          _tShape(other._tShape),
          _startSpeed(other._startSpeed),
          _startSpeedSet(other._startSpeedSet),
          _endSpeed(other._endSpeed),
          _endSpeedSet(other._endSpeedSet),
          _recorded(other._recorded)
    {
        other._recorded = true;
    }

    /// @brief Records into `DriveClass::mockRecord()` — defined AFTER `DriveClass`
    ///        below (needs the complete type; see the file-level ordering note).
    ~DriveDistBuilder();

    DriveDistBuilder& setPower(int16_t) { return *this; }
    DriveDistBuilder& setPreset(Config::SpeedPreset) { return *this; }
    DriveDistBuilder& setMode(DriveMode) { return *this; }
    DriveDistBuilder& setBrake(bool b) { _brake = b; return *this; }
    DriveDistBuilder& setBlocking(bool) { return *this; }
    DriveDistBuilder& async() { return *this; }
    DriveDistBuilder& setHeading(int16_t) { return *this; }
    DriveDistBuilder& tShape() { _tShape = true; return *this; }
    DriveDistBuilder& flat() { _tShape = false; return *this; }
    DriveDistBuilder& setAccel(float) { return *this; }
    DriveDistBuilder& setStartSpeed(int16_t s) { _startSpeed = s; _startSpeedSet = true; return *this; }
    DriveDistBuilder& setEndSpeed(int16_t e) { _endSpeed = e; _endSpeedSet = true; return *this; }

private:
    DriveClass& _drive;
    float       _cm            = 0.0f;
    bool        _brake         = true;
    bool        _tShape        = false;
    int16_t     _startSpeed    = 0;
    bool        _startSpeedSet = false;
    int16_t     _endSpeed      = 0;
    bool        _endSpeedSet   = false;
    bool        _recorded      = false;    ///< Set on the moved-from instance so its dtor no-ops.
};

class DriveTimeBuilder
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

class TurnDegBuilder
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

class TurnToYawBuilder
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

class StopAtLineBuilder
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

class WallStopBuilder
{
public:
    WRO_MOCK_BUILDER_BOILERPLATE(WallStopBuilder)
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

    // driveDist(...) builder-knob counters (ADR-059 / ADR-049 — the chain-speed
    // dispatch reduces to "the builder recorded setStartSpeed/setEndSpeed/setBrake";
    // memory `gotcha-ctps-mock-noop-setter-drops-builder-knob` warns a no-op setter
    // here is a documented false-green for that assertion).
    uint32_t driveDistCallCount   = 0;
    float    lastDriveDistCm      = 0.0f;
    int16_t  lastDriveDistStart   = 0;
    bool     lastDriveDistStartSet = false;
    int16_t  lastDriveDistEnd     = 0;
    bool     lastDriveDistEndSet  = false;
    bool     lastDriveDistBrake   = true;
    bool     lastDriveDistTShape  = false;
};

class DriveClass
{
    // `DriveDistBuilder`'s destructor writes into `_record` (ADR-059 / T10 recording
    // builder) — friend it the same way the real `DriveClass` friends its builders.
    friend class DriveDistBuilder;

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

    // floorMag mirrors the real Drive::setSpeed signature (T4 kinetic decel floor). This mock
    // records the PRE-snap value LineFollower passed, so the deadband floor is irrelevant here —
    // the arg is accepted (defaulted) only so the 3-arg call site compiles against this regime.
    uint8_t setSpeed(int16_t L, int16_t R, int16_t /*floorMag*/ = Config::PWM_MIN_MAGNITUDE)
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
    DriveDistBuilder  driveDist(float cm) { return DriveDistBuilder{*this, cm}; }
    DriveTimeBuilder  driveTime(float /*sec*/) { return DriveTimeBuilder{}; }
    TurnDegBuilder    turnDeg(int16_t /*deg*/) { return TurnDegBuilder{}; }
    TurnToYawBuilder  turnToYaw(int16_t /*yaw*/) { return TurnToYawBuilder{}; }
    StopAtLineBuilder stopAtLine(float /*expectedCm*/) { return StopAtLineBuilder{}; }
    WallStopBuilder   wallStop(float /*cm*/) { return WallStopBuilder{}; }

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

// ---------------------------------------------------------------------------
// `DriveDistBuilder::~DriveDistBuilder()` — defined here, AFTER `DriveClass`, the
// SAME ordering the real `Drive.h`/`Drive.cpp` use (the destructor needs the
// complete `DriveClass` type to reach the friended private `_record`; MSVC will
// not accept an inline definition against an incomplete type — memory
// `gotcha_msvc_inline_incomplete_type`). Records unconditionally into
// `DriveClass::mockRecord()` — no queue/dispatch semantics, no multi-tick FSM
// modeling (ADR-049 / memory `gotcha-host-mock-thin-not-behavioral`): this mock
// is record-and-test-set only, exactly like every other mock in this file.
// ---------------------------------------------------------------------------
inline DriveDistBuilder::~DriveDistBuilder()
{
    if (_recorded)
    {
        return;    // moved-from instance — the new instance will record instead.
    }
    _recorded = true;

    DriveCallRecord& rec        = _drive._record;
    rec.driveDistCallCount++;
    rec.lastDriveDistCm       = _cm;
    rec.lastDriveDistStart    = _startSpeed;
    rec.lastDriveDistStartSet = _startSpeedSet;
    rec.lastDriveDistEnd      = _endSpeed;
    rec.lastDriveDistEndSet   = _endSpeedSet;
    rec.lastDriveDistBrake    = _brake;
    rec.lastDriveDistTShape   = _tShape;
}

/// Single global instance — defined in `mocks/Drive.cpp`.
extern DriveClass Drive;

#endif    // DRIVE_H
