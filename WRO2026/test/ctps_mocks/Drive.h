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
 * Captures task-internal RAW moves. Each of the six fluent builders records
 * exactly one `drive` op into the shared `g_ctpsLog` from an `_executed`-guarded
 * DESTRUCTOR — the same recording seam the real builders use. Recording at
 * destruction (rather than at construction) is what lets a chained setter run
 * (`.setMode(GYRO).setHeading(0).setReverse(true)`) so every knob is captured
 * BEFORE the single emit: the verb constructs one builder, each chain setter
 * mutates it and returns `*this`, and the discarded temporary destructs at the
 * statement's `;`, emitting once with the fully-resolved `mode` / `heading` /
 * `reverse`. Because authored mission/task code is one-verb-per-statement, that
 * end-of-statement emit lands in exactly the execution order the older
 * construction-time recording produced.
 *
 * The `_executed` sentinel (set on the first destruction) keeps the single-emit
 * robust even against an accidental double-run. The builders stay NON-MOVABLE
 * and NON-COPYABLE (deleted copy ctor + copy/move-assign, no move ctor): a
 * user-declared destructor already suppresses the implicit move ctor, and C++17
 * guaranteed copy elision means the by-value `return XxxBuilder(...)` from each
 * facade verb needs no move. A file-scope
 * `static_assert(!is_copy_constructible && !is_move_constructible)` after each
 * builder documents and enforces that invariant.
 *
 * The builders mirror the real move-only `[[nodiscard]]` shape so discarded-
 * temporary call sites compile exactly as they do against the real header. All
 * "is-done?" queries report instant completion (`isIdle()` -> true,
 * `hasFault()` -> false) so task bodies' fake-blocking drain exits in one
 * iteration and the dump never hangs.
 *
 * `goto` is NOT recorded here — it is captured by the PathTracker.goTo mock.
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include <type_traits>

#include "Config.h"    // Config::SpeedPreset for the setPreset(...) chain setters.
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
 * @brief Map a `DriveMode` to its command-stream mode string.
 * @param m The resolved drive mode.
 * @return `"raw"` / `"sync"` / `"gyro"`.
 * @note These three strings ARE the cross-language `drive.method` contract
 *       (recorder <-> `ctps/trace_loader.py` <-> `ctps/track_builder.py`), not
 *       magic numbers; keep them byte-identical to the loader's mode whitelist.
 */
inline const char* driveModeToStr(DriveMode m)
{
    switch (m)
    {
        case DriveMode::RAW:
            return "raw";
        case DriveMode::SYNC:
            return "sync";
        case DriveMode::GYRO:
            return "gyro";
    }
    return "";    // Unreachable: every DriveMode value is handled — satisfies -Wreturn-type.
}

/**
 * @brief Recording mock of `DriveDistBuilder`. Records `driveDist` on destruction.
 *
 * Captures the explicitly-set `mode` (via `setMode`) and `heading` (via
 * `setHeading`) and emits them through `ctps::recordDriveMove`. The real
 * `DriveDistBuilder` has NO `setReverse`, so the emitted `reverse` flag is
 * always `false` here.
 */
class [[nodiscard]] DriveDistBuilder
{
public:
    /** @brief Store the distance; the record is emitted once from the destructor. */
    DriveDistBuilder(DriveClass& /*drive*/, float cm) : _cm(cm) {}

    DriveDistBuilder(const DriveDistBuilder&)            = delete;
    DriveDistBuilder& operator=(const DriveDistBuilder&) = delete;
    DriveDistBuilder& operator=(DriveDistBuilder&&)      = delete;

    // Chain setters — capture the CTPS-relevant knobs, no-op the rest; all return *this.
    DriveDistBuilder& setPower(int16_t) { return *this; }
    DriveDistBuilder& setPreset(Config::SpeedPreset) { return *this; }
    DriveDistBuilder& setMode(DriveMode m)
    {
        _mode = driveModeToStr(m);
        return *this;
    }
    DriveDistBuilder& setBrake(bool) { return *this; }
    DriveDistBuilder& setBlocking(bool) { return *this; }
    DriveDistBuilder& async() { return *this; }
    DriveDistBuilder& setHeading(int16_t yaw)
    {
        _heading    = static_cast<float>(yaw);
        _hasHeading = true;
        return *this;
    }
    DriveDistBuilder& tShape() { return *this; }
    DriveDistBuilder& flat() { return *this; }
    DriveDistBuilder& setAccel(float) { return *this; }
    DriveDistBuilder& setStartSpeed(int16_t) { return *this; }
    DriveDistBuilder& setEndSpeed(int16_t) { return *this; }

    /** @brief Emit the captured `driveDist` move exactly once (guarded by `_executed`). */
    ~DriveDistBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;
        const ctps::DriveMoveParams params{_cm, _mode, _hasHeading, _heading, false};
        ctps::recordDriveMove("driveDist", params);
    }

private:
    float       _cm;
    const char* _mode       = nullptr;    ///< Set by setMode(); nullptr = mode unset.
    bool        _hasHeading = false;      ///< True iff setHeading() was called.
    float       _heading    = 0.0f;       ///< Held heading (degrees); valid iff _hasHeading.
    bool        _executed   = false;      ///< Single-emit guard (idempotent destructor).
};
static_assert(
    !std::is_copy_constructible<DriveDistBuilder>::value
        && !std::is_move_constructible<DriveDistBuilder>::value,
    "CTPS DriveDistBuilder must stay non-copyable/non-movable (relies on guaranteed elision)");

/**
 * @brief Recording mock of `DriveTimeBuilder`.
 *
 * A time-bounded move carries no distance/angle the CTPS geometry layer can use,
 * so it is recorded as a non-geometry `drive` entry (`method == "stop"`) from the
 * destructor — this preserves execution order without injecting a bogus distance
 * the loader would misinterpret. No `mode` / `heading` is captured: `driveTime`
 * stays encoded as `stop`. `driveTime` is reproduced mainly so its call site
 * compiles; authored task bodies use `driveDist` / `turnDeg` / `turnToYaw` for
 * geometry.
 */
class [[nodiscard]] DriveTimeBuilder
{
public:
    DriveTimeBuilder(DriveClass& /*drive*/, float /*sec*/) {}

    DriveTimeBuilder(const DriveTimeBuilder&)            = delete;
    DriveTimeBuilder& operator=(const DriveTimeBuilder&) = delete;
    DriveTimeBuilder& operator=(DriveTimeBuilder&&)      = delete;

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

    /** @brief Emit a non-geometry `stop` drive op exactly once (guarded by `_executed`). */
    ~DriveTimeBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;
        ctps::recordDriveStop();
    }

private:
    bool _executed = false;    ///< Single-emit guard (idempotent destructor).
};
static_assert(
    !std::is_copy_constructible<DriveTimeBuilder>::value
        && !std::is_move_constructible<DriveTimeBuilder>::value,
    "CTPS DriveTimeBuilder must stay non-copyable/non-movable (relies on guaranteed elision)");

/**
 * @brief Recording mock of `TurnDegBuilder`. Records `turnDeg` on destruction.
 *
 * A turn encodes its heading change as the turn itself, so no `mode` / `heading`
 * is captured — the destructor re-emits the relative delta unchanged.
 */
class [[nodiscard]] TurnDegBuilder
{
public:
    TurnDegBuilder(DriveClass& /*drive*/, int16_t deg) : _deg(deg) {}

    TurnDegBuilder(const TurnDegBuilder&)            = delete;
    TurnDegBuilder& operator=(const TurnDegBuilder&) = delete;
    TurnDegBuilder& operator=(TurnDegBuilder&&)      = delete;

    TurnDegBuilder& setPower(int16_t) { return *this; }
    TurnDegBuilder& setPreset(Config::SpeedPreset) { return *this; }
    TurnDegBuilder& setMode(uint8_t) { return *this; }
    TurnDegBuilder& setBrake(bool) { return *this; }
    TurnDegBuilder& setBlocking(bool) { return *this; }
    TurnDegBuilder& async() { return *this; }

    /** @brief Emit the captured `turnDeg` exactly once (guarded by `_executed`). */
    ~TurnDegBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;
        ctps::recordTurnDeg(static_cast<float>(_deg));
    }

private:
    int16_t _deg;
    bool    _executed = false;    ///< Single-emit guard (idempotent destructor).
};
static_assert(
    !std::is_copy_constructible<TurnDegBuilder>::value
        && !std::is_move_constructible<TurnDegBuilder>::value,
    "CTPS TurnDegBuilder must stay non-copyable/non-movable (relies on guaranteed elision)");

/**
 * @brief Recording mock of `TurnToYawBuilder`. Records `turnToYaw` on destruction.
 *
 * Like `TurnDegBuilder`, a turn encodes its heading change as the turn itself, so
 * no `mode` / `heading` is captured — the destructor re-emits the absolute target
 * unchanged.
 */
class [[nodiscard]] TurnToYawBuilder
{
public:
    TurnToYawBuilder(DriveClass& /*drive*/, int16_t yaw) : _yaw(yaw) {}

    TurnToYawBuilder(const TurnToYawBuilder&)            = delete;
    TurnToYawBuilder& operator=(const TurnToYawBuilder&) = delete;
    TurnToYawBuilder& operator=(TurnToYawBuilder&&)      = delete;

    TurnToYawBuilder& setPower(int16_t) { return *this; }
    TurnToYawBuilder& setPreset(Config::SpeedPreset) { return *this; }
    TurnToYawBuilder& setMode(uint8_t) { return *this; }
    TurnToYawBuilder& setBrake(bool) { return *this; }
    TurnToYawBuilder& setBlocking(bool) { return *this; }
    TurnToYawBuilder& async() { return *this; }

    /** @brief Emit the captured `turnToYaw` exactly once (guarded by `_executed`). */
    ~TurnToYawBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;
        ctps::recordTurnToYaw(static_cast<float>(_yaw));
    }

private:
    int16_t _yaw;
    bool    _executed = false;    ///< Single-emit guard (idempotent destructor).
};
static_assert(
    !std::is_copy_constructible<TurnToYawBuilder>::value
        && !std::is_move_constructible<TurnToYawBuilder>::value,
    "CTPS TurnToYawBuilder must stay non-copyable/non-movable (relies on guaranteed elision)");

/**
 * @brief Recording mock of `StopAtLineBuilder`. Records `stopAtLine` on destruction.
 *
 * Captures the explicitly-set `mode` (via `setMode`), `heading` (via
 * `setHeading`), and `reverse` (via `setReverse`) and emits them through
 * `ctps::recordDriveMove`.
 */
class [[nodiscard]] StopAtLineBuilder
{
public:
    StopAtLineBuilder(DriveClass& /*drive*/, float expectedCm) : _expectedCm(expectedCm) {}

    StopAtLineBuilder(const StopAtLineBuilder&)            = delete;
    StopAtLineBuilder& operator=(const StopAtLineBuilder&) = delete;
    StopAtLineBuilder& operator=(StopAtLineBuilder&&)      = delete;

    StopAtLineBuilder& setPower(int16_t) { return *this; }
    StopAtLineBuilder& setPreset(Config::SpeedPreset) { return *this; }
    StopAtLineBuilder& setMode(DriveMode m)
    {
        _mode = driveModeToStr(m);
        return *this;
    }
    StopAtLineBuilder& setHeading(int16_t yaw)
    {
        _heading    = static_cast<float>(yaw);
        _hasHeading = true;
        return *this;
    }
    StopAtLineBuilder& setArmWithin(float) { return *this; }
    StopAtLineBuilder& setMaxDistance(float) { return *this; }
    StopAtLineBuilder& setOvershoot(float) { return *this; }
    StopAtLineBuilder& setReverse(bool r)
    {
        _reverse = r;
        return *this;
    }
    StopAtLineBuilder& setBrake(bool) { return *this; }
    StopAtLineBuilder& setBlocking(bool) { return *this; }
    StopAtLineBuilder& async() { return *this; }

    /** @brief Emit the captured `stopAtLine` move exactly once (guarded by `_executed`). */
    ~StopAtLineBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;
        const ctps::DriveMoveParams params{_expectedCm, _mode, _hasHeading, _heading, _reverse};
        ctps::recordDriveMove("stopAtLine", params);
    }

private:
    float       _expectedCm;
    const char* _mode       = nullptr;    ///< Set by setMode(); nullptr = mode unset.
    bool        _hasHeading = false;      ///< True iff setHeading() was called.
    float       _heading    = 0.0f;       ///< Held heading (degrees); valid iff _hasHeading.
    bool        _reverse    = false;      ///< Set by setReverse(); reverse-direction flag.
    bool        _executed   = false;      ///< Single-emit guard (idempotent destructor).
};
static_assert(
    !std::is_copy_constructible<StopAtLineBuilder>::value
        && !std::is_move_constructible<StopAtLineBuilder>::value,
    "CTPS StopAtLineBuilder must stay non-copyable/non-movable (relies on guaranteed elision)");

/**
 * @brief Recording mock of `WallStopBuilder`.
 *
 * CTPS replays a wall-stop as a nominal forward (or, when `setReverse(true)`,
 * backward) push by the advisory `cm` argument; the real firmware terminates on
 * stall and may stop slightly short or long, but the nominal push is the least-
 * surprising host approximation. Captures the explicitly-set `mode` / `heading` /
 * `reverse` and emits them through `ctps::recordDriveMove`.
 *
 * @note The mock records mode-absent when `setMode` was never called, even though
 *       the real `wallStop` defaults its mode to `GYRO`. Modelling that firmware
 *       default here would make the mock behavioural; a thin mock captures only
 *       the caller's explicit setter calls.
 */
class [[nodiscard]] WallStopBuilder
{
public:
    WallStopBuilder(DriveClass& /*drive*/, float expectedCm) : _expectedCm(expectedCm) {}

    WallStopBuilder(const WallStopBuilder&)            = delete;
    WallStopBuilder& operator=(const WallStopBuilder&) = delete;
    WallStopBuilder& operator=(WallStopBuilder&&)      = delete;

    WallStopBuilder& setPower(int16_t) { return *this; }
    WallStopBuilder& setPreset(Config::SpeedPreset) { return *this; }
    WallStopBuilder& setMode(DriveMode m)
    {
        _mode = driveModeToStr(m);
        return *this;
    }
    WallStopBuilder& setHeading(int16_t yaw)
    {
        _heading    = static_cast<float>(yaw);
        _hasHeading = true;
        return *this;
    }
    WallStopBuilder& setBackOff(float) { return *this; }
    WallStopBuilder& setTimeout(uint32_t) { return *this; }
    WallStopBuilder& setReverse(bool r)
    {
        _reverse = r;
        return *this;
    }
    WallStopBuilder& setBrake(bool) { return *this; }
    WallStopBuilder& setBlocking(bool) { return *this; }
    WallStopBuilder& async() { return *this; }

    /** @brief Emit the captured `wallStop` move exactly once (guarded by `_executed`). */
    ~WallStopBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;
        const ctps::DriveMoveParams params{_expectedCm, _mode, _hasHeading, _heading, _reverse};
        ctps::recordDriveMove("wallStop", params);
    }

private:
    float       _expectedCm;
    const char* _mode       = nullptr;    ///< Set by setMode(); nullptr = mode unset.
    bool        _hasHeading = false;      ///< True iff setHeading() was called.
    float       _heading    = 0.0f;       ///< Held heading (degrees); valid iff _hasHeading.
    bool        _reverse    = false;      ///< Set by setReverse(); reverse-direction flag.
    bool        _executed   = false;      ///< Single-emit guard (idempotent destructor).
};
static_assert(
    !std::is_copy_constructible<WallStopBuilder>::value
        && !std::is_move_constructible<WallStopBuilder>::value,
    "CTPS WallStopBuilder must stay non-copyable/non-movable (relies on guaranteed elision)");

/**
 * @brief Recording mock of the Drive facade.
 *
 * The fluent verbs return the recording builders by value; each builder records
 * once from its `_executed`-guarded destructor. The continuous primitives + state
 * accessors are no-ops that report an instantly-idle, fault-free chassis.
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

    // --- Fluent verbs — return recording builders (which record on destruction). ---
    [[nodiscard]] DriveDistBuilder driveDist(float cm) { return DriveDistBuilder(*this, cm); }
    [[nodiscard]] DriveTimeBuilder driveTime(float sec) { return DriveTimeBuilder(*this, sec); }
    [[nodiscard]] TurnDegBuilder   turnDeg(int16_t deg) { return TurnDegBuilder(*this, deg); }
    [[nodiscard]] TurnToYawBuilder turnToYaw(int16_t yaw) { return TurnToYawBuilder(*this, yaw); }
    [[nodiscard]] StopAtLineBuilder stopAtLine(float expectedCm)
    {
        return StopAtLineBuilder(*this, expectedCm);
    }
    [[nodiscard]] WallStopBuilder wallStop(float expectedCm)
    {
        return WallStopBuilder(*this, expectedCm);
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
