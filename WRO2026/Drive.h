/**
 * @file Drive.h
 * @brief Movement-primitive singleton wrapping `MiniR4.DriveDC` and `MiniR4.Motion`.
 *
 * @details
 * `DriveClass` owns the differential-drive chassis. It is the single entry point every
 * higher-level module (LineFollower, Intersection, EventTrace, mission state machine) uses
 * to actuate the wheels and to read encoder / yaw state. A single global instance `Drive` is
 * exported so callers do not need to manage construction or pass pointers around.
 *
 * Method families:
 *  - **Continuous** (`setSpeed`, `setSpeedSync`, `forwardGyro`, `stop`) — the underlying
 *    motors run until the next continuous command is issued or `stop()` is called. These do
 *    NOT touch the bounded-task done flags. Kept as direct (non-fluent) calls because they
 *    take 0-2 trivial arguments — fluent there would be ceremony. Of these, ONLY `setSpeed`
 *    applies the motor-bias scalar (Strategy B); `setSpeedSync` and `forwardGyro` do not,
 *    because the STM32 closes the loop on those paths and a feedforward scalar would fight
 *    the inner PID. See DECISIONS.md.
 *  - **Distance-bounded** (`driveDist(cm)`) — fluent builder; defaults to non-blocking, raw
 *    PWM, brake-on-stop. Set `.setMode(DriveMode::SYNC|GYRO)`, `.setBlocking(true)`, etc. on
 *    the returned builder; the move executes when the temporary destructs at end-of-statement.
 *  - **Time-bounded** (`driveTime(sec)`) — same fluent shape as `driveDist`.
 *  - **Turns** (`turnDeg(deg)` / `turnToYaw(yaw)`) — fluent builders; defaults to
 *    non-blocking, mode `Config::TURN_MODE`, brake `Config::TURN_BRAKE`, power
 *    `Config::TURN_POWER`. Both relative (`turnDeg`) and absolute (`turnToYaw`) targets are
 *    supported.
 *
 * @note The underlying `MiniR4.DriveDC` lower-MCU API exposes ONE task-done bit, so this
 *       class collapses move-done and turn-done into a single shared "task done" signal
 *       inside `update()`. Application code MUST NOT issue a bounded move and a turn
 *       concurrently — they would race for the same flag. On a failed turn issue
 *       (non-zero rc from TurnGyro): if the slot was idle at entry, `_turnDone` is set
 *       `true` so the caller is unblocked (C1 fix). If a prior task was already in
 *       flight, `_turnDone` is left untouched — the prior task still owns the slot
 *       (W1 guard).
 *
 * @warning The `*Builder::setBlocking(true)` variant spins inside the underlying library's
 *          `delay(50)`-ish poll loop until the lower MCU reports the task is done. They MUST
 *          NOT be called from inside any module's `update()` method — `update()` must remain
 *          non-blocking and cooperative. Blocking entry points are reserved for command-time
 *          orchestration (mission step transitions, calibration routines, etc.).
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <MatrixMiniR4.h>

#include "Config.h"

// Forward declarations so DriveClass can name-friend the builders before they are defined.
class DriveDistBuilder;
class DriveTimeBuilder;
class TurnDegBuilder;
class TurnToYawBuilder;

/**
 * @brief Mode selector for distance/time-bounded straight-line moves.
 *
 * Selects which lower-MCU primitive a `driveDist` / `driveTime` builder dispatches to. The
 * choice determines whether the `_applyBias` correction is applied (RAW only — Sync/Gyro
 * close the loop on the STM32, so layering a feedforward scalar would fight their inner PID).
 */
enum class DriveMode : uint8_t
{
    RAW,     ///< Open-loop PWM (`Move` / `MoveDegs` / `MoveTime`). Bias-compensated via `_applyBias`.
    SYNC,    ///< `MoveSync*` — STM32 encoder-rate sync PID closes the loop. No bias applied.
    GYRO     ///< `MoveGyro*` — STM32 yaw-lock PID closes the loop. No bias applied.
};

/**
 * @brief Rich movement-primitive facade over `MiniR4.DriveDC` + `MiniR4.Motion`.
 *
 * Holds the persistent yaw-hold target (`_heldYaw`), the move/turn done flags, and exposes a
 * uniform return convention: `0x00` = OK, non-zero = lower-MCU error code propagated from the
 * underlying library. See class-level docblock for method-family semantics and threading
 * caveats.
 */
class DriveClass
{
    // The fluent builders need access to the private `_execXxx` dispatch methods and the
    // `_presetToPwm` helper. Friending keeps the public surface small and prevents callers
    // from invoking the legacy wide-signature paths directly.
    friend class DriveDistBuilder;
    friend class DriveTimeBuilder;
    friend class TurnDegBuilder;
    friend class TurnToYawBuilder;

public:
    /**
     * @brief Configure the drivebase, register reverse-flags, and push every PID profile.
     *
     * Must be called exactly once from `setup()` after `MiniR4.begin()` has returned. Sources
     * every value from `Config::*` — never hardcode tuning here.
     *
     * @return `true` if every lower-MCU call succeeded; `false` if `MiniR4.DriveDC.begin`
     *         returned a non-zero error.
     * @note PID-set calls return `bool` and are best-effort: a single retry already happens
     *       inside the library, so we do not surface a partial-PID failure here.
     */
    bool begin();

    /**
     * @brief Cooperative tick — must be called every iteration of the main `loop()`.
     *
     * Polls `MiniR4.DriveDC.isPrevTaskDone()` and clears `_moveDone` / `_turnDone` together
     * once the lower MCU signals completion.
     * @note Contains NO `delay()`. Safe to call at any cadence.
     * @note Because the lower MCU reports a single task-done bit, this method clears BOTH
     *       move and turn flags simultaneously — see class docblock.
     */
    void update();

    // -------------------------------------------------------------------------------------
    // Continuous primitives (kept direct — 0-2 args, fluent would be ceremony)
    // -------------------------------------------------------------------------------------

    /**
     * @brief Continuously drive both wheels at raw PWM power.
     *
     * Routes to `MiniR4.DriveDC.Move` (raw PWM, no inner PID). This is the variant the
     * line-follower outer loop calls every tick — using `MoveSync` here would stack a PID on
     * top of the line-follower's PID and oscillate.
     *
     * @param L Left-motor power in `[-100, 100]`.
     * @param R Right-motor power in `[-100, 100]`.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Continuous; does not touch the move/turn done flags.
     * @note Bias-compensated via `_applyBias` (raw-PWM path).
     */
    uint8_t setSpeed(int16_t L, int16_t R);

    /**
     * @brief Continuously drive both wheels with the lower-MCU's MoveSync regulator.
     *
     * Routes to `MiniR4.DriveDC.MoveSync` — encoder-feedback synchronisation between the two
     * wheels. Use only when the higher-level controller does NOT itself add a wheel-balance
     * loop on top.
     *
     * @param L Left-motor power in `[-100, 100]`.
     * @param R Right-motor power in `[-100, 100]`.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Continuous; does not touch the move/turn done flags.
     * @note No bias applied — STM32 closes the loop.
     */
    uint8_t setSpeedSync(int16_t L, int16_t R);

    /**
     * @brief Continuously drive forward holding the most recent yaw target.
     *
     * Routes to `MiniR4.DriveDC.MoveGyro(power, _heldYaw)`. Update the held yaw with
     * `holdYaw()` (snapshot the IMU now) or by issuing a turn (turns update `_heldYaw` to the
     * post-turn target).
     *
     * @param power Drive power in `[-100, 100]`. Negative drives backwards while still
     *              locking heading.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Continuous; does not touch the move/turn done flags.
     * @note No bias applied — STM32 closes the loop.
     */
    uint8_t forwardGyro(int16_t power);

    /**
     * @brief Stop both wheels and engage (or release) the brake.
     *
     * Issues `Move(0, 0)` then `brake(brake)`. Marks both bounded-task done flags `true`
     * because the chassis is now idle.
     *
     * @param brake `true` (default) = active brake; `false` = coast.
     * @return `0x00` on success, lower-MCU error code from `brake()` otherwise. The `Move`
     *         result is intentionally not surfaced — a brake error is the more actionable
     *         failure to report.
     */
    uint8_t stop(bool brake = true);

    // -------------------------------------------------------------------------------------
    // Fluent verb entry points
    // -------------------------------------------------------------------------------------

    /**
     * @brief Build a distance-bounded straight-line move.
     *
     * Returns a `DriveDistBuilder` that executes the move when the temporary destructs at
     * end-of-statement. Defaults are: non-blocking, `DriveMode::RAW`, power
     * `Config::SPEED_NORMAL`, brake `true`, heading = current `_heldYaw` (only consulted in
     * `DriveMode::GYRO`).
     *
     * Example:
     * @code
     * Drive.driveDist(30);                                                    // defaults
     * Drive.driveDist(30).setPower(50);                                       // override
     * Drive.driveDist(30).setPreset(Config::SpeedPreset::FAST).setMode(DriveMode::GYRO);
     * @endcode
     *
     * @param cm Distance in centimetres (always non-negative — sign is carried by `power`).
     * @return A fluent builder that auto-executes on destruction.
     * @note `[[nodiscard]]` — discarding the builder without using it almost certainly means
     *       you forgot the chain.
     */
    [[nodiscard]] DriveDistBuilder driveDist(float cm);

    /**
     * @brief Build a time-bounded straight-line move.
     *
     * Returns a `DriveTimeBuilder` that executes the move when the temporary destructs.
     * Defaults match `driveDist`.
     *
     * @param sec Duration in seconds.
     * @return A fluent builder that auto-executes on destruction.
     * @note `[[nodiscard]]`.
     */
    [[nodiscard]] DriveTimeBuilder driveTime(float sec);

    /**
     * @brief Build a relative-yaw-delta turn.
     *
     * Returns a `TurnDegBuilder` that executes the turn when the temporary destructs.
     * Defaults are: non-blocking, mode `Config::TURN_MODE`, brake `Config::TURN_BRAKE`,
     * power `Config::TURN_POWER`.
     *
     * @param deg Relative yaw delta in degrees. Positive = whichever direction the lower MCU
     *            interprets as positive yaw (typically counter-clockwise).
     * @return A fluent builder that auto-executes on destruction.
     * @note `[[nodiscard]]`.
     */
    [[nodiscard]] TurnDegBuilder turnDeg(int16_t deg);

    /**
     * @brief Build an absolute-yaw turn.
     *
     * Returns a `TurnToYawBuilder` that executes the turn when the temporary destructs.
     * Defaults match `turnDeg`.
     *
     * @param yaw Absolute yaw target in degrees (normalised through `_normalizeYaw` to
     *            `(-360, +360)` before being handed to `TurnGyro`).
     * @return A fluent builder that auto-executes on destruction.
     * @note `[[nodiscard]]`.
     */
    [[nodiscard]] TurnToYawBuilder turnToYaw(int16_t yaw);

    // -------------------------------------------------------------------------------------
    // State accessors
    // -------------------------------------------------------------------------------------

    /**
     * @brief Whether the most recent bounded move (`driveDist*` / `driveTime*`) has finished.
     * @return `true` if no bounded move is pending.
     * @note Continuous primitives (`setSpeed`, etc.) do not affect this flag.
     */
    bool isMoveDone() const;

    /**
     * @brief Whether the most recent async turn has finished.
     * @return `true` if no async turn is pending. Blocking turns leave this `true` on return.
     */
    bool isTurnDone() const;

    /**
     * @brief Left-wheel encoder reading in degrees (post-direction-correction).
     * @return Cumulative degrees rotated by the left motor since the last `resetEnc()`.
     * @note Reads `MiniR4.M2` directly — assumes the M2/M3 left/right convention from
     *       Config. Swapping `Config::MOTOR_LEFT_ID` away from `2` requires editing this
     *       method (no runtime template indexing is possible).
     */
    int32_t getEncL() const;

    /**
     * @brief Right-wheel encoder reading in degrees (post-direction-correction).
     * @return Cumulative degrees rotated by the right motor since the last `resetEnc()`.
     * @note Reads `MiniR4.M3` directly — same M2/M3 convention caveat as `getEncL`.
     */
    int32_t getEncR() const;

    /**
     * @brief Reset the drivebase encoder counters to zero.
     */
    void resetEnc();

    /**
     * @brief Current IMU yaw in degrees.
     * @return Yaw value sampled from `MiniR4.Motion.getEuler(AxisType::Yaw)`.
     */
    double getYaw() const;

    /**
     * @brief Snapshot the current IMU yaw as the new `_heldYaw` for `forwardGyro` / `*Gyro`
     *        primitives.
     * @note The snapshot is truncated to `int16_t` to match the `TurnGyro` / `MoveGyro` API.
     */
    void holdYaw();

    // -------------------------------------------------------------------------------------
    // Motor-bias calibration (Strategy B — see DECISIONS.md)
    // -------------------------------------------------------------------------------------

    /**
     * @brief Run a wheels-free calibration routine and print bias scalars.
     *
     * Spins both motors at @p power for @p durationMs using raw `Move` (no Sync, no
     * Gyro), reads final encoder degrees on each, and computes scalars that, when
     * multiplied through future `setSpeed` / `driveDist` / `driveTime` calls,
     * equalise the two wheels' rotation per unit PWM.
     *
     * **Wheels MUST be free of the ground.** Routine spins for ~2 s.
     *
     * Prints the resulting scalars to Serial in a copy-paste-ready format. Diego
     * then hand-edits `Config::MOTOR_BIAS_L` / `MOTOR_BIAS_R` and re-flashes.
     *
     * @param power      PWM target during calibration (default `Config::MOTOR_BIAS_CALIB_POWER`).
     * @param durationMs Spin duration in milliseconds (default `Config::MOTOR_BIAS_CALIB_DURATION_MS`).
     * @return `0x00` on success; `0x01` on zero encoder delta (stuck wheel / broken
     *         encoder / power too low); `0x02` on opposite-sign deltas (wiring or
     *         `MOTOR_*_REVERSE` flag mismatch); MMLower error code otherwise.
     *
     * @note Strategy B (Phase 1.5). See DECISIONS.md for full rationale.
     * @note This is the ONE place in the codebase where a `delay()` is acceptable —
     *       the routine is a manual one-shot, not part of any cooperative tick.
     */
    uint8_t calibrateMotorBias(int16_t  power      = Config::MOTOR_BIAS_CALIB_POWER,
                               uint32_t durationMs = Config::MOTOR_BIAS_CALIB_DURATION_MS);

    /**
     * @brief Manually set the motor-bias scalars at runtime.
     *
     * Used by `begin()` to apply the Config defaults, and available for live
     * override during testing.
     *
     * @param biasL Scalar for the left motor's raw-PWM commands.
     * @param biasR Scalar for the right motor's raw-PWM commands.
     *
     * @note Strategy B. See DECISIONS.md.
     */
    void setMotorBias(float biasL, float biasR);

    /**
     * @brief Reset both bias scalars to 1.0f (identity — no correction).
     *
     * Equivalent to `setMotorBias(1.0f, 1.0f)` but reads more clearly at call sites
     * where the intent is explicitly "I don't trust the calibration, give me raw".
     *
     * @note Strategy B. See DECISIONS.md.
     */
    void resetMotorBias();

    /**
     * @brief Read current left-motor bias scalar (telemetry).
     * @return Current `_biasL`.
     */
    float getBiasL() const;

    /**
     * @brief Read current right-motor bias scalar (telemetry).
     * @return Current `_biasR`.
     */
    float getBiasR() const;

private:
    int16_t _heldYaw  = 0;       ///< Current yaw target used by `forwardGyro` and `*Gyro`.
    bool    _moveDone = true;    ///< Cleared on bounded move issue, set by `update()`.
    bool    _turnDone = true;    ///< Cleared on async turn issue, set by `update()`.
    float   _biasL    = 1.0f;    ///< Static manufacturer-bias scalar applied to raw-PWM L paths. See DECISIONS.md.
    float   _biasR    = 1.0f;    ///< Static manufacturer-bias scalar applied to raw-PWM R paths. See DECISIONS.md.

    // -------------------------------------------------------------------------------------
    // Builder dispatch targets — each fluent builder destructor calls into one of these.
    // -------------------------------------------------------------------------------------

    /**
     * @brief Open-loop PWM distance move (`MoveDegs`). Bias-compensated.
     * @param power    PWM target in `[-100, 100]`.
     * @param cm       Distance in centimetres.
     * @param brake    Apply brake at end of move.
     * @param blocking If `true`, block until the lower MCU reports done; if `false`, return
     *                 immediately and clear `_moveDone`.
     * @return `0x00` on success, lower-MCU error code otherwise.
     */
    uint8_t _execDriveDistRaw(int16_t power, float cm, bool brake, bool blocking);

    /**
     * @brief MoveSync distance move (`MoveSyncDegs`). No bias applied.
     * @param power    PWM target in `[-100, 100]`.
     * @param cm       Distance in centimetres.
     * @param brake    Apply brake at end of move.
     * @param blocking Block / async semantics — see `_execDriveDistRaw`.
     * @return `0x00` on success, lower-MCU error code otherwise.
     */
    uint8_t _execDriveDistSync(int16_t power, float cm, bool brake, bool blocking);

    /**
     * @brief MoveGyro distance move (`MoveGyroDegs`). Heading-locked. No bias applied.
     * @param power    PWM target in `[-100, 100]`.
     * @param cm       Distance in centimetres.
     * @param yaw      Heading to hold during the move (typically the current `_heldYaw`).
     * @param brake    Apply brake at end of move.
     * @param blocking Block / async semantics — see `_execDriveDistRaw`.
     * @return `0x00` on success, lower-MCU error code otherwise.
     */
    uint8_t _execDriveDistGyro(int16_t power, float cm, int16_t yaw, bool brake, bool blocking);

    /**
     * @brief Open-loop PWM time move (`MoveTime`). Bias-compensated.
     * @param power    PWM target in `[-100, 100]`.
     * @param sec      Duration in seconds.
     * @param brake    Apply brake at end of move.
     * @param blocking Block / async semantics.
     * @return `0x00` on success, lower-MCU error code otherwise.
     */
    uint8_t _execDriveTimeRaw(int16_t power, float sec, bool brake, bool blocking);

    /**
     * @brief MoveSync time move (`MoveSyncTime`). No bias applied.
     * @param power    PWM target in `[-100, 100]`.
     * @param sec      Duration in seconds.
     * @param brake    Apply brake at end of move.
     * @param blocking Block / async semantics.
     * @return `0x00` on success, lower-MCU error code otherwise.
     */
    uint8_t _execDriveTimeSync(int16_t power, float sec, bool brake, bool blocking);

    /**
     * @brief MoveGyro time move (`MoveGyroTime`). Heading-locked. No bias applied.
     * @param power    PWM target in `[-100, 100]`.
     * @param sec      Duration in seconds.
     * @param yaw      Heading to hold during the move.
     * @param brake    Apply brake at end of move.
     * @param blocking Block / async semantics.
     * @return `0x00` on success, lower-MCU error code otherwise.
     */
    uint8_t _execDriveTimeGyro(int16_t power, float sec, int16_t yaw, bool brake, bool blocking);

    /**
     * @brief Relative-yaw-delta turn (`TurnGyro` with target = current yaw + deg).
     * @param deg      Relative yaw delta in degrees.
     * @param power    PWM target for the turn.
     * @param mode     Turn motor mode (0 = pivot, 1 = counter-rotate).
     * @param brake    Apply brake at end of turn.
     * @param blocking Block until done (`async = false`) or fire-and-forget (`async = true`).
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note On non-zero rc, `_turnDone` is left `true` and `_heldYaw` is NOT updated — the
     *       chassis never moved, so the previous yaw lock must be preserved.
     */
    uint8_t _execTurnDeg(int16_t deg, int16_t power, uint8_t mode, bool brake, bool blocking);

    /**
     * @brief Absolute-yaw turn (`TurnGyro` with target = `_normalizeYaw(yaw)`).
     * @param yaw      Absolute yaw target in degrees.
     * @param power    PWM target for the turn.
     * @param mode     Turn motor mode (0 = pivot, 1 = counter-rotate).
     * @param brake    Apply brake at end of turn.
     * @param blocking Block until done or async.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Same C1-fix semantics as `_execTurnDeg`.
     */
    uint8_t _execTurnToYaw(int16_t yaw, int16_t power, uint8_t mode, bool brake, bool blocking);

    /**
     * @brief Resolve a `Config::SpeedPreset` to its corresponding PWM constant.
     *
     * @param preset Coarse speed selector.
     * @return PWM in `[-100, 100]`. `SLOW` -> `Config::SPEED_SLOW`,
     *         `NORMAL` -> `Config::SPEED_NORMAL`, `FAST` -> `Config::SPEED_FAST`.
     * @note Falls back to `Config::SPEED_NORMAL` for any unrecognised value (defensive — the
     *       `enum class` should keep the switch exhaustive at compile time).
     */
    int16_t _presetToPwm(Config::SpeedPreset preset) const;

    /**
     * @brief Normalise an absolute yaw target into the range that
     *        `MiniR4.DriveDC.TurnGyro` accepts.
     *
     * Uses `deg % 360` only. By the C++11+ truncating modulo rule, the
     * result is always in `(-360, +360)` for any `int32_t` input, which is
     * inside the `[-360, +360]` band the lower-MCU `TurnGyro` API documents.
     *
     * @param deg Raw target degrees (may be any int32 produced by `cur + delta`).
     * @return Normalised target in `(-360, +360)`.
     */
    static int16_t _normalizeYaw(int32_t deg);

    /**
     * @brief Convert a linear distance in centimetres to wheel rotation in degrees.
     *
     * Formula: `(cm * 10 / (PI * WHEEL_DIAM_MM)) * 360`. The `cm * 10` converts to mm so the
     * units cancel against `WHEEL_DIAM_MM`.
     *
     * @param cm Linear distance in centimetres (always non-negative — sign of travel is
     *           carried by the `power` argument of the calling primitive).
     * @return Wheel rotation in degrees, cast to `uint16_t` (the type accepted by every
     *         `*Degs` lower-MCU API).
     */
    static uint16_t _cmToEncDeg(float cm);

    /**
     * @brief Multiply (L, R) in-place by the bias scalars before forwarding to
     *        any raw-PWM MatrixMiniR4 call (`Move`, `MoveDegs`, `MoveTime`).
     *
     * Sync/Gyro paths must NOT call this — they close the loop on the STM32 and
     * applying a feedforward scalar on top would fight the inner PID.
     *
     * @note Strategy B. See DECISIONS.md.
     */
    void _applyBias(int16_t& L, int16_t& R) const;
};

/**
 * @brief Single global instance — defined in Drive.cpp.
 */
extern DriveClass Drive;

// =========================================================================================
// Fluent builders — header-only, inline. Each is RAII: the destructor executes the move
// with whatever knobs were set. `_executed` guards against accidental double-execution if
// a builder is moved out of (we delete copy and move below to keep it simple, but the flag
// also makes the destructor safe even if a future change re-enables move).
//
// `[[nodiscard]]` on the class itself means a stack-stored builder warns if unused.
// =========================================================================================

/**
 * @brief Fluent builder for `Drive.driveDist(cm)` — distance-bounded straight-line move.
 *
 * Auto-executes when the temporary destructs at end-of-statement. Setters return `*this` by
 * reference for chaining. Defaults: `DriveMode::RAW`, power `Config::SPEED_NORMAL`, brake
 * `true`, blocking `false`, heading = `Drive`'s current `_heldYaw` (only consulted in
 * `DriveMode::GYRO`).
 */
class [[nodiscard]] DriveDistBuilder
{
public:
    /**
     * @brief Construct a builder bound to the singleton `Drive` for distance @p cm.
     * @param drive Reference to the singleton `Drive` (only `Drive.driveDist` should call this).
     * @param cm    Distance in centimetres.
     */
    DriveDistBuilder(DriveClass& drive, float cm) : _drive(drive), _cm(cm)
    {
    }

    DriveDistBuilder(const DriveDistBuilder&)            = delete;
    DriveDistBuilder& operator=(const DriveDistBuilder&) = delete;
    DriveDistBuilder& operator=(DriveDistBuilder&&)      = delete;

    /**
     * @brief Move constructor — transfers execution responsibility to the new instance.
     *
     * Required so the verb methods (`Drive.driveDist(cm)`) can return a builder by value on
     * compilers that do not perform mandatory copy elision (pre-C++17). The moved-from
     * instance is marked `_executed` so its destructor is a no-op.
     */
    DriveDistBuilder(DriveDistBuilder&& other) noexcept
        : _drive(other._drive),
          _cm(other._cm),
          _power(other._power),
          _powerExplicit(other._powerExplicit),
          _preset(other._preset),
          _presetSet(other._presetSet),
          _mode(other._mode),
          _brake(other._brake),
          _blocking(other._blocking),
          _heading(other._heading),
          _headingSet(other._headingSet),
          _executed(other._executed)
    {
        other._executed = true;
    }

    /**
     * @brief Override the PWM power explicitly. Wins over any preset.
     * @param p PWM target in `[-100, 100]`.
     * @return `*this` for chaining.
     * @note Calling both `setPower` and `setPreset` on the same builder is allowed; the
     *       last `setPower` (whether before or after `setPreset`) wins.
     */
    DriveDistBuilder& setPower(int16_t p)
    {
        _power       = p;
        _powerExplicit = true;
        return *this;
    }

    /**
     * @brief Use a `Config::SpeedPreset` for the PWM power.
     * @param p Preset selector — resolved via `Drive._presetToPwm` at execute-time.
     * @return `*this` for chaining.
     * @note Ignored if `setPower` was also called on the same builder.
     */
    DriveDistBuilder& setPreset(Config::SpeedPreset p)
    {
        _preset       = p;
        _presetSet    = true;
        return *this;
    }

    /**
     * @brief Choose which underlying lower-MCU primitive executes the move.
     * @param m `RAW` (open-loop), `SYNC` (encoder-rate sync PID), or `GYRO` (yaw-lock).
     * @return `*this` for chaining.
     * @note `_applyBias` is applied only for `RAW`; the other modes are closed-loop on the
     *       STM32.
     */
    DriveDistBuilder& setMode(DriveMode m)
    {
        _mode = m;
        return *this;
    }

    /**
     * @brief Apply (or release) the active brake at end-of-move.
     * @param b `true` = brake; `false` = coast.
     * @return `*this` for chaining.
     */
    DriveDistBuilder& setBrake(bool b)
    {
        _brake = b;
        return *this;
    }

    /**
     * @brief Block the caller until the move completes.
     * @param b `true` = sync (library spins on `delay`); `false` = async (default).
     * @return `*this` for chaining.
     * @warning Blocking variants MUST NOT be called from inside any module's `update()`.
     */
    DriveDistBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Override the heading held during a `DriveMode::GYRO` move.
     * @param yaw Absolute yaw target in degrees.
     * @return `*this` for chaining.
     * @note Only consulted when the resolved mode is `GYRO`. If never set, the move uses the
     *       current `Drive` `_heldYaw` (snapshot via `holdYaw()` or set by a prior turn).
     */
    DriveDistBuilder& setHeading(int16_t yaw)
    {
        _heading       = yaw;
        _headingSet    = true;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-execute. Resolves the final knobs and dispatches to the
     *        appropriate `DriveClass::_execDriveDist*` method.
     * @note Idempotent: the `_executed` guard prevents a second dispatch if a future change
     *       re-enables move semantics.
     */
    ~DriveDistBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;

        const int16_t resolvedPower = _resolvePower();
        switch (_mode)
        {
        case DriveMode::RAW:
            _drive._execDriveDistRaw(resolvedPower, _cm, _brake, _blocking);
            break;
        case DriveMode::SYNC:
            _drive._execDriveDistSync(resolvedPower, _cm, _brake, _blocking);
            break;
        case DriveMode::GYRO:
        {
            const int16_t yaw = _headingSet ? _heading : _drive._heldYaw;
            _drive._execDriveDistGyro(resolvedPower, _cm, yaw, _brake, _blocking);
            break;
        }
        }
    }

private:
    /**
     * @brief Resolve the final PWM power: explicit `setPower` wins, then `setPreset`, then
     *        `Config::SPEED_NORMAL` as the documented default.
     */
    int16_t _resolvePower() const
    {
        if (_powerExplicit)
        {
            return _power;
        }
        if (_presetSet)
        {
            return _drive._presetToPwm(_preset);
        }
        return Config::SPEED_NORMAL;
    }

    DriveClass& _drive;
    float       _cm;
    int16_t     _power          = Config::SPEED_NORMAL;
    bool        _powerExplicit  = false;
    Config::SpeedPreset _preset = Config::SpeedPreset::NORMAL;
    bool        _presetSet      = false;
    DriveMode   _mode           = DriveMode::RAW;
    bool        _brake          = true;
    bool        _blocking       = false;
    int16_t     _heading        = 0;
    bool        _headingSet     = false;
    bool        _executed       = false;
};

/**
 * @brief Fluent builder for `Drive.driveTime(sec)` — time-bounded straight-line move.
 *
 * Auto-executes on destruction. Setters mirror `DriveDistBuilder`; only the time vs. distance
 * verb differs.
 */
class [[nodiscard]] DriveTimeBuilder
{
public:
    /**
     * @brief Construct a builder bound to the singleton `Drive` for duration @p sec.
     * @param drive Reference to the singleton `Drive`.
     * @param sec   Duration in seconds.
     */
    DriveTimeBuilder(DriveClass& drive, float sec) : _drive(drive), _sec(sec)
    {
    }

    DriveTimeBuilder(const DriveTimeBuilder&)            = delete;
    DriveTimeBuilder& operator=(const DriveTimeBuilder&) = delete;
    DriveTimeBuilder& operator=(DriveTimeBuilder&&)      = delete;

    /**
     * @brief Move constructor — transfers execution responsibility to the new instance.
     *
     * Required so the verb methods can return a builder by value on compilers without
     * mandatory copy elision. The moved-from instance is marked `_executed` so its
     * destructor is a no-op.
     */
    DriveTimeBuilder(DriveTimeBuilder&& other) noexcept
        : _drive(other._drive),
          _sec(other._sec),
          _power(other._power),
          _powerExplicit(other._powerExplicit),
          _preset(other._preset),
          _presetSet(other._presetSet),
          _mode(other._mode),
          _brake(other._brake),
          _blocking(other._blocking),
          _heading(other._heading),
          _headingSet(other._headingSet),
          _executed(other._executed)
    {
        other._executed = true;
    }

    /**
     * @brief Override the PWM power explicitly. Wins over any preset.
     * @param p PWM target in `[-100, 100]`.
     * @return `*this` for chaining.
     */
    DriveTimeBuilder& setPower(int16_t p)
    {
        _power         = p;
        _powerExplicit = true;
        return *this;
    }

    /**
     * @brief Use a `Config::SpeedPreset` for the PWM power.
     * @param p Preset selector.
     * @return `*this` for chaining.
     */
    DriveTimeBuilder& setPreset(Config::SpeedPreset p)
    {
        _preset    = p;
        _presetSet = true;
        return *this;
    }

    /**
     * @brief Choose which underlying lower-MCU primitive executes the move.
     * @param m `RAW`, `SYNC`, or `GYRO`.
     * @return `*this` for chaining.
     */
    DriveTimeBuilder& setMode(DriveMode m)
    {
        _mode = m;
        return *this;
    }

    /**
     * @brief Apply (or release) the active brake at end-of-move.
     * @param b `true` = brake; `false` = coast.
     * @return `*this` for chaining.
     */
    DriveTimeBuilder& setBrake(bool b)
    {
        _brake = b;
        return *this;
    }

    /**
     * @brief Block the caller until the move completes.
     * @param b `true` = sync; `false` = async.
     * @return `*this` for chaining.
     */
    DriveTimeBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Override the heading held during a `DriveMode::GYRO` move.
     * @param yaw Absolute yaw target in degrees.
     * @return `*this` for chaining.
     * @note Only consulted when the resolved mode is `GYRO`.
     */
    DriveTimeBuilder& setHeading(int16_t yaw)
    {
        _heading    = yaw;
        _headingSet = true;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-execute. Dispatches to the correct
     *        `DriveClass::_execDriveTime*` method.
     */
    ~DriveTimeBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;

        const int16_t resolvedPower = _resolvePower();
        switch (_mode)
        {
        case DriveMode::RAW:
            _drive._execDriveTimeRaw(resolvedPower, _sec, _brake, _blocking);
            break;
        case DriveMode::SYNC:
            _drive._execDriveTimeSync(resolvedPower, _sec, _brake, _blocking);
            break;
        case DriveMode::GYRO:
        {
            const int16_t yaw = _headingSet ? _heading : _drive._heldYaw;
            _drive._execDriveTimeGyro(resolvedPower, _sec, yaw, _brake, _blocking);
            break;
        }
        }
    }

private:
    /**
     * @brief Resolve the final PWM power: explicit `setPower` wins, then `setPreset`, then
     *        `Config::SPEED_NORMAL`.
     */
    int16_t _resolvePower() const
    {
        if (_powerExplicit)
        {
            return _power;
        }
        if (_presetSet)
        {
            return _drive._presetToPwm(_preset);
        }
        return Config::SPEED_NORMAL;
    }

    DriveClass& _drive;
    float       _sec;
    int16_t     _power          = Config::SPEED_NORMAL;
    bool        _powerExplicit  = false;
    Config::SpeedPreset _preset = Config::SpeedPreset::NORMAL;
    bool        _presetSet      = false;
    DriveMode   _mode           = DriveMode::RAW;
    bool        _brake          = true;
    bool        _blocking       = false;
    int16_t     _heading        = 0;
    bool        _headingSet     = false;
    bool        _executed       = false;
};

/**
 * @brief Fluent builder for `Drive.turnDeg(deg)` — relative-yaw-delta turn.
 *
 * Auto-executes on destruction. Defaults: power `Config::TURN_POWER`, mode `Config::TURN_MODE`,
 * brake `Config::TURN_BRAKE`, blocking `false`.
 */
class [[nodiscard]] TurnDegBuilder
{
public:
    /**
     * @brief Construct a builder bound to the singleton `Drive` for relative delta @p deg.
     * @param drive Reference to the singleton `Drive`.
     * @param deg   Relative yaw delta in degrees.
     */
    TurnDegBuilder(DriveClass& drive, int16_t deg) : _drive(drive), _deg(deg)
    {
    }

    TurnDegBuilder(const TurnDegBuilder&)            = delete;
    TurnDegBuilder& operator=(const TurnDegBuilder&) = delete;
    TurnDegBuilder& operator=(TurnDegBuilder&&)      = delete;

    /**
     * @brief Move constructor — transfers execution responsibility to the new instance.
     *
     * Required so the verb methods can return a builder by value on compilers without
     * mandatory copy elision. The moved-from instance is marked `_executed`.
     */
    TurnDegBuilder(TurnDegBuilder&& other) noexcept
        : _drive(other._drive),
          _deg(other._deg),
          _power(other._power),
          _powerExplicit(other._powerExplicit),
          _preset(other._preset),
          _presetSet(other._presetSet),
          _mode(other._mode),
          _brake(other._brake),
          _blocking(other._blocking),
          _executed(other._executed)
    {
        other._executed = true;
    }

    /**
     * @brief Override the turn PWM power explicitly. Wins over any preset.
     * @param p PWM target in `[-100, 100]`.
     * @return `*this` for chaining.
     */
    TurnDegBuilder& setPower(int16_t p)
    {
        _power         = p;
        _powerExplicit = true;
        return *this;
    }

    /**
     * @brief Use a `Config::SpeedPreset` for the turn power.
     * @param p Preset selector.
     * @return `*this` for chaining.
     */
    TurnDegBuilder& setPreset(Config::SpeedPreset p)
    {
        _preset    = p;
        _presetSet = true;
        return *this;
    }

    /**
     * @brief Set the lower-MCU turn motor mode.
     * @param m `0` = single-motor pivot (one wheel braked), `1` = counter-rotation.
     * @return `*this` for chaining.
     */
    TurnDegBuilder& setMode(uint8_t m)
    {
        _mode = m;
        return *this;
    }

    /**
     * @brief Apply (or release) the active brake at end-of-turn.
     * @param b `true` = brake; `false` = coast.
     * @return `*this` for chaining.
     */
    TurnDegBuilder& setBrake(bool b)
    {
        _brake = b;
        return *this;
    }

    /**
     * @brief Block the caller until the turn completes.
     * @param b `true` = sync (library spins); `false` = async (default).
     * @return `*this` for chaining.
     * @warning Blocking variants MUST NOT be called from inside any module's `update()`.
     */
    TurnDegBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-execute. Dispatches to `DriveClass::_execTurnDeg`.
     */
    ~TurnDegBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed                   = true;
        const int16_t resolvedPower = _resolvePower();
        _drive._execTurnDeg(_deg, resolvedPower, _mode, _brake, _blocking);
    }

private:
    /**
     * @brief Resolve the final PWM power: explicit `setPower` wins, then `setPreset`, then
     *        `Config::TURN_POWER` as the documented default.
     */
    int16_t _resolvePower() const
    {
        if (_powerExplicit)
        {
            return _power;
        }
        if (_presetSet)
        {
            return _drive._presetToPwm(_preset);
        }
        return Config::TURN_POWER;
    }

    DriveClass&         _drive;
    int16_t             _deg;
    int16_t             _power         = Config::TURN_POWER;
    bool                _powerExplicit = false;
    Config::SpeedPreset _preset        = Config::SpeedPreset::NORMAL;
    bool                _presetSet     = false;
    uint8_t             _mode          = Config::TURN_MODE;
    bool                _brake         = Config::TURN_BRAKE;
    bool                _blocking      = false;
    bool                _executed      = false;
};

/**
 * @brief Fluent builder for `Drive.turnToYaw(yaw)` — absolute-yaw turn.
 *
 * Auto-executes on destruction. Defaults match `TurnDegBuilder`.
 */
class [[nodiscard]] TurnToYawBuilder
{
public:
    /**
     * @brief Construct a builder bound to the singleton `Drive` for absolute target @p yaw.
     * @param drive Reference to the singleton `Drive`.
     * @param yaw   Absolute yaw target in degrees.
     */
    TurnToYawBuilder(DriveClass& drive, int16_t yaw) : _drive(drive), _yaw(yaw)
    {
    }

    TurnToYawBuilder(const TurnToYawBuilder&)            = delete;
    TurnToYawBuilder& operator=(const TurnToYawBuilder&) = delete;
    TurnToYawBuilder& operator=(TurnToYawBuilder&&)      = delete;

    /**
     * @brief Move constructor — transfers execution responsibility to the new instance.
     *
     * Required so the verb methods can return a builder by value on compilers without
     * mandatory copy elision. The moved-from instance is marked `_executed`.
     */
    TurnToYawBuilder(TurnToYawBuilder&& other) noexcept
        : _drive(other._drive),
          _yaw(other._yaw),
          _power(other._power),
          _powerExplicit(other._powerExplicit),
          _preset(other._preset),
          _presetSet(other._presetSet),
          _mode(other._mode),
          _brake(other._brake),
          _blocking(other._blocking),
          _executed(other._executed)
    {
        other._executed = true;
    }

    /**
     * @brief Override the turn PWM power explicitly. Wins over any preset.
     * @param p PWM target in `[-100, 100]`.
     * @return `*this` for chaining.
     */
    TurnToYawBuilder& setPower(int16_t p)
    {
        _power         = p;
        _powerExplicit = true;
        return *this;
    }

    /**
     * @brief Use a `Config::SpeedPreset` for the turn power.
     * @param p Preset selector.
     * @return `*this` for chaining.
     */
    TurnToYawBuilder& setPreset(Config::SpeedPreset p)
    {
        _preset    = p;
        _presetSet = true;
        return *this;
    }

    /**
     * @brief Set the lower-MCU turn motor mode.
     * @param m `0` = pivot, `1` = counter-rotate.
     * @return `*this` for chaining.
     */
    TurnToYawBuilder& setMode(uint8_t m)
    {
        _mode = m;
        return *this;
    }

    /**
     * @brief Apply (or release) the active brake at end-of-turn.
     * @param b `true` = brake; `false` = coast.
     * @return `*this` for chaining.
     */
    TurnToYawBuilder& setBrake(bool b)
    {
        _brake = b;
        return *this;
    }

    /**
     * @brief Block the caller until the turn completes.
     * @param b `true` = sync; `false` = async.
     * @return `*this` for chaining.
     * @warning Blocking variants MUST NOT be called from inside any module's `update()`.
     */
    TurnToYawBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-execute. Dispatches to `DriveClass::_execTurnToYaw`.
     */
    ~TurnToYawBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed                   = true;
        const int16_t resolvedPower = _resolvePower();
        _drive._execTurnToYaw(_yaw, resolvedPower, _mode, _brake, _blocking);
    }

private:
    /**
     * @brief Resolve the final PWM power: explicit `setPower` wins, then `setPreset`, then
     *        `Config::TURN_POWER`.
     */
    int16_t _resolvePower() const
    {
        if (_powerExplicit)
        {
            return _power;
        }
        if (_presetSet)
        {
            return _drive._presetToPwm(_preset);
        }
        return Config::TURN_POWER;
    }

    DriveClass&         _drive;
    int16_t             _yaw;
    int16_t             _power         = Config::TURN_POWER;
    bool                _powerExplicit = false;
    Config::SpeedPreset _preset        = Config::SpeedPreset::NORMAL;
    bool                _presetSet     = false;
    uint8_t             _mode          = Config::TURN_MODE;
    bool                _brake         = Config::TURN_BRAKE;
    bool                _blocking      = false;
    bool                _executed      = false;
};

// =========================================================================================
// Inline definitions of the verb entry points — these MUST come after the builder classes
// are fully defined, since they construct and return them by value.
// =========================================================================================

inline DriveDistBuilder DriveClass::driveDist(float cm)
{
    return DriveDistBuilder(*this, cm);
}

inline DriveTimeBuilder DriveClass::driveTime(float sec)
{
    return DriveTimeBuilder(*this, sec);
}

inline TurnDegBuilder DriveClass::turnDeg(int16_t deg)
{
    return TurnDegBuilder(*this, deg);
}

inline TurnToYawBuilder DriveClass::turnToYaw(int16_t yaw)
{
    return TurnToYawBuilder(*this, yaw);
}

#endif    // DRIVE_H
