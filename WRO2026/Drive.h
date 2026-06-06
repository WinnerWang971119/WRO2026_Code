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
 *  - **Distance-bounded** (`driveDist(cm)`) — fluent builder; **sync-blocking by default**,
 *    raw PWM, brake-on-stop. Each verb call drains the queue at end-of-statement so mission
 *    code reads top-down like Python. Chain `.async()` to opt out (auto-path planner case
 *    where the caller ticks `loop()` and polls `Drive.isIdle()`). Other knobs:
 *    `.setMode(DriveMode::SYNC|GYRO)`, `.setPower(...)`, etc.
 *  - **Time-bounded** (`driveTime(sec)`) — same fluent shape as `driveDist`.
 *  - **Turns** (`turnDeg(deg)` / `turnToYaw(yaw)`) — fluent builders; **sync-blocking by
 *    default**, mode `Config::TURN_MODE`, brake `Config::TURN_BRAKE`, power
 *    `Config::TURN_CRUISE_PWM`. Both relative (`turnDeg`) and absolute (`turnToYaw`) targets are
 *    supported. Chain `.async()` to opt out.
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
#include "WROMath.h"

// Forward declarations so DriveClass can name-friend the builders before they are defined.
class DriveDistBuilder;
class DriveTimeBuilder;
class TurnDegBuilder;
class TurnToYawBuilder;
class StopAtLineBuilder;

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

// Math helpers used by Drive and its builders live in `WROMath.h` (free functions).
// Notably `deadbandSnap`, `presetToPwm`, `cmToEncDeg` — all originally private to this
// class, factored out once a second caller (LineFollower) needed them.

/**
 * @brief Captured parameters of a queued Drive command.
 *
 * Built by a fluent builder's destructor and pushed onto `DriveClass`'s internal
 * FIFO. Dispatched by `Drive.update()` when the previous command completes. Holds
 * a discriminated-union shape — `kind` selects which of the bounded/profiled
 * subtypes the other fields apply to.
 */
struct DriveCommand
{
    /**
     * @brief Discriminator for the queued command's variant.
     */
    enum class Kind : uint8_t
    {
        DRIVE_DIST,        ///< Non-profiled distance move (mode-driven).
        DRIVE_TIME,        ///< Non-profiled time move (mode-driven).
        TURN_DEG,          ///< Relative turn.
        TURN_TO_YAW,       ///< Absolute-yaw turn.
        PROFILE_DIST,      ///< Profiled (tShape) distance move.
        PROFILE_TIME,      ///< Profiled (tShape) time move.
        STOP_AT_LINE       ///< Drive-until-line move; stops on T_OR_CROSS near the endpoint.
    };

    Kind        kind;
    DriveMode   mode      = DriveMode::SYNC;    ///< For DRIVE_DIST/TIME and PROFILE_DIST/TIME.
                                                ///< SYNC is the project-wide default — see DECISIONS.md.

    // Bounds.
    float       cm        = 0.0f;
    float       sec       = 0.0f;
    int16_t     deg       = 0;                  ///< For TURN_DEG.
    int16_t     yaw       = 0;                  ///< For TURN_TO_YAW.

    // Common.
    int16_t     power     = 0;
    int16_t     heading   = 0;                  ///< Used by GYRO mode and PROFILE_*.
    bool        brake     = true;
    uint8_t     turnMode  = 0;                  ///< For TURN_*.

    // Profile-only.
    int16_t     startSpeed = 0;
    int16_t     endSpeed   = 0;
    float       accel      = 0.0f;

    // STOP_AT_LINE-only. Dispatched into `_updateStopAtLine()` (a continuous, event-terminated
    // machine), NOT `_updateProfile()` — the stop point is unknown until the line is detected,
    // so the trapezoid's known-distance ramp-down does not apply. `mode`, `power`, `heading`,
    // and `brake` above are reused; these add the seek-specific bounds.
    float       expectedCm = 0.0f;     ///< Nominal distance to the expected crossing line.
    float       armWithin  = 0.0f;     ///< Arm detection within this many cm of `expectedCm`.
    float       maxCm      = 0.0f;     ///< Fault `LINE_NOT_FOUND` if no line detected by here.
    float       overshoot  = 0.0f;     ///< Post-detect roll so the wheel axle lands on the line.
    bool        reverse    = false;    ///< Back into the line via the rear board (needs 2 boards).
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
    // The fluent builders need access to the private `_execXxx` dispatch methods. Friending
    // keeps the public surface small and prevents callers from invoking the legacy
    // wide-signature paths directly. (`presetToPwm` is now a free function in `WROMath.h`,
    // so builders no longer need friend access for it.)
    friend class DriveDistBuilder;
    friend class DriveTimeBuilder;
    friend class TurnDegBuilder;
    friend class TurnToYawBuilder;
    friend class StopAtLineBuilder;

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
     * Three-stage tick, executed strictly in this order each call:
     *   1. **Drain** — if no profile is in flight and a bounded task was issued, poll
     *      `MiniR4.DriveDC.isPrevTaskDone()`. When the lower MCU reports completion, clear
     *      `_moveDone` and `_turnDone` together (the MCU exposes a single task-done bit).
     *   2. **Profile** — advance the trapezoidal `.tShape()` state machine via
     *      `_updateProfile()` (idempotent when IDLE). On the tick that the profile finishes,
     *      this stage flips `_moveDone` to `true` so stage 3 sees an idle chassis on the same
     *      tick — chained queue moves get dispatched without an extra-tick gap.
     *   3. **Dispatch** — if `_isIdle()` and `_qCount > 0`, pop the next `DriveCommand` and
     *      hand it to `_dispatchNext()`. The dispatcher also auto-disables `LineFollower`
     *      (with `brake=false`) if it was enabled — a queued bounded/profiled move owns the
     *      motors exclusively and must not race the follower's tick loop.
     *
     * @note Contains NO `delay()`. Safe to call at any cadence.
     * @note Because the lower MCU reports a single task-done bit, this method clears BOTH
     *       move and turn flags simultaneously — see class docblock.
     * @note A queued blocking-variant builder bypasses this method's normal flow: it calls
     *       `_drainBlocking()` (which spins on the registered tick callback + `delay(2)`,
     *       falling back to `update()` alone if no callback is registered) to honour
     *       synchronous semantics on top of the queue infrastructure. This means `update()`
     *       may be re-entered indirectly via `setTickCallback`'s registered cooperative tick
     *       during a sync block.
     */
    void update();

    /**
     * @brief Register a function to call each iteration of the blocking-spin loop.
     *
     * `_drainBlocking()` (used by sync-by-default builders) needs to keep ALL
     * cooperative subsystems alive during its spin — otherwise `EventTrace` doesn't
     * render, `LineFollower` doesn't poll the tracer, etc. Register a function that
     * ticks every module (typically `cooperativeTick()` defined in the sketch) and
     * `_drainBlocking` will invoke it instead of just calling `Drive.update()`.
     *
     * Call this exactly once from `setup()`, after every module's `begin()`.
     *
     * @param cb Pointer to the cooperative-tick function. Pass `nullptr` to clear.
     *
     * @note If no callback is registered, `_drainBlocking` falls back to ticking
     *       only `Drive.update()` and emits a one-time Serial warning at first use.
     *       This is a safe degradation but means other modules freeze during a sync
     *       block — register the callback to avoid this.
     * @warning The registered callback's call chain MUST NOT issue sync-blocking Drive
     *          commands (`driveDist` / `driveTime` / `turnDeg` / `turnToYaw` without
     *          `.async()`). Doing so would re-enter `_drainBlocking()` from inside its
     *          own spin, recursing indefinitely. Today no `update()` does this; if a
     *          future module's tick needs to issue Drive moves, it must use `.async()`.
     */
    void setTickCallback(void (*cb)());

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
     * end-of-statement. Defaults are: sync-blocking, `DriveMode::SYNC`, trapezoidal profile
     * (`tShape`) on, power `Config::SPEED_NORMAL`, brake `true`, heading = current `_heldYaw`
     * (only consulted in `DriveMode::GYRO`). The SYNC + tShape default reflects the auto-path
     * planner being the dominant consumer — see DECISIONS.md "Sync + T-shape as default
     * move profile". Opt out per call: `.setMode(DriveMode::RAW)`, `.tShape(false)` is not
     * exposed (set acceleration absurdly high or chain `.setStartSpeed(cruise).setEndSpeed(cruise)`
     * if a flat profile is wanted on a one-off — usually just stay with the default).
     *
     * Example:
     * @code
     * Drive.driveDist(30);                                                    // SYNC + tShape, sync-block
     * Drive.driveDist(30).setPower(50);                                       // override power
     * Drive.driveDist(30).setMode(DriveMode::GYRO);                           // straight-line under yaw lock
     * Drive.driveDist(30).setMode(DriveMode::RAW);                            // open-loop fallback
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
     * Defaults match `driveDist` (sync-blocking, `DriveMode::SYNC`, `tShape` on, power
     * `Config::SPEED_NORMAL`, brake `true`).
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
     * power `Config::TURN_CRUISE_PWM`.
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

    /**
     * @brief Build a drive-until-line move that stops on an expected perpendicular crossing.
     *
     * Returns a `StopAtLineBuilder` that executes when the temporary destructs at
     * end-of-statement. The move drives straight toward a line expected near @p expectedCm,
     * **arms** a `T_OR_CROSS` detector only once the chassis is within `armWithin` cm of the
     * expected endpoint (so an earlier line crossed en route cannot trigger), rolls a
     * configurable `overshoot` past the detected line so the wheel axle lands on it, then
     * active-brakes. If no line is seen by the resolved max distance the move faults
     * `Fault::LINE_NOT_FOUND` (no silent dead-reckon fallback) — query `isLineFound()`.
     *
     * Defaults are: sync-blocking, `DriveMode::SYNC`, power `Config::SPEED_NORMAL`, brake
     * `true`, heading = current `_heldYaw` (only consulted in `DriveMode::GYRO`),
     * `armWithin = Config::STOP_AT_LINE_ARM_WINDOW_CM`,
     * `maxDistance = expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM`,
     * `overshoot = Config::LINE_SENSOR_AXLE_OFFSET_CM`, forward (front board). Chain
     * `.async()` for the auto-path-planner opt-out (tick `loop()`, poll `Drive.isIdle()`).
     *
     * Unlike `driveDist`, this is a CONTINUOUS event-terminated move: it runs the bespoke
     * `_updateStopAtLine()` machine, NOT the bounded `_updateProfile()` trapezoid — the stop
     * point is unknown until detection, so only the known-length overshoot can ramp down.
     *
     * Example:
     * @code
     * Drive.stopAtLine(40);                                    // SYNC, arm at 30 cm, axle-overshoot, sync-block
     * Drive.stopAtLine(40).setMode(DriveMode::GYRO).async();   // heading-locked, pollable
     * Drive.stopAtLine(40).setReverse(true);                   // back into the line via rear board
     * @endcode
     *
     * @param expectedCm Nominal distance to the expected crossing line, in centimetres
     *                   (always non-negative — reverse direction is selected via `.setReverse`).
     * @return A fluent builder that auto-executes on destruction.
     * @note `[[nodiscard]]` — discarding the builder without using it almost certainly means
     *       you forgot the chain.
     */
    [[nodiscard]] StopAtLineBuilder stopAtLine(float expectedCm);

    // -------------------------------------------------------------------------------------
    // State accessors
    // -------------------------------------------------------------------------------------

    /**
     * @brief Whether the most recent bounded move (`driveDist*` / `driveTime*`) has finished.
     * @return `true` if no bounded move is pending.
     * @note Continuous primitives (`setSpeed`, etc.) do not affect this flag.
     * @warning **Queue-blind.** Returns `true` between queued segments only if a dispatch
     *          failed; the happy path keeps it `false` from the first move's start until
     *          the last queued move's end. For "is everything (current + queue) done?",
     *          use `isIdle()` instead.
     */
    bool isMoveDone() const;

    /**
     * @brief Whether the most recent async turn has finished.
     * @return `true` if no async turn is pending. Blocking turns leave this `true` on return.
     * @warning **Queue-blind** — same caveat as `isMoveDone()`. Use `isIdle()` to query
     *          chassis-and-queue state together.
     */
    bool isTurnDone() const;

    /**
     * @brief Whether the chassis is fully idle: no bounded move, no async turn, no profile
     *        in flight, AND the command queue is empty.
     *
     * This is the right query for the auto-path planner's "is it safe to issue the next
     * mission step?" tick — it's queue-aware unlike `isMoveDone()` / `isTurnDone()`.
     *
     * @return `true` iff `_moveDone && _turnDone && _profileState == IDLE &&
     *         _stopState == IDLE && _qCount == 0`.
     */
    bool isIdle() const;

    /**
     * @brief Latched result of the most recent `stopAtLine` move.
     *
     * Set `true` the tick the seeker confirms a `T_OR_CROSS` crossing; set `false` when a
     * `stopAtLine` is dispatched and left `false` if the move runs to its max distance
     * without a detection (which also latches `Fault::LINE_NOT_FOUND`). Other move/turn
     * verbs do not touch this flag.
     *
     * @return `true` if the last `stopAtLine` found its line; `false` if it faulted/maxed or
     *         none has run yet.
     * @note On a no-line outcome `hasFault() == true` and `getFault().kind ==
     *       Fault::LINE_NOT_FOUND` — this query and the fault state agree.
     */
    bool isLineFound() const;

    /**
     * @brief Block (spin) until the chassis is fully idle and the queue is empty.
     *
     * Thin wrapper that delegates to `_drainBlocking()` — see that method's doxygen for
     * the spin contract, the registered tick-callback, and the allowed-`delay()`
     * rationale. Exposed for command-time orchestration where the next decision genuinely
     * needs every queued move to be complete (mission-step transitions, calibration
     * sequences, end-of-run cleanup).
     *
     * @warning **Do NOT call from inside any module's `update()`.** Reserved for
     *          top-level orchestration only.
     * @note Mostly redundant since builders are sync-by-default — a hand-written mission
     *       script doesn't need `waitUntilIdle()` between calls. Useful when mixing
     *       `.async()` calls with subsequent code that needs them all to land first.
     */
    void waitUntilIdle();

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

    // -------------------------------------------------------------------------------------
    // Fault state (code review F5 + F3)
    //
    // The queue and the sync-blocking drain are both fail-fast: when the lower MCU rejects
    // a queued dispatch (`_dispatchNext` non-zero rc) or `_drainBlocking` exceeds its
    // bounded budget, motors are stopped, the queue is cleared, and the fault is recorded
    // here. Higher-level mission code is expected to query `hasFault()` between mission
    // steps; on fault, recover (or abort), then call `clearFault()` to re-arm the queue.
    // -------------------------------------------------------------------------------------

    /**
     * @brief Source of a Drive-layer fault.
     */
    enum class Fault : uint8_t
    {
        NONE             = 0,   ///< No fault — normal operation.
        DRAIN_TIMEOUT    = 1,   ///< `_drainBlocking` deadline exceeded (F3).
        DISPATCH_FAILED  = 2,   ///< `_dispatchNext` saw a non-zero rc from the lower MCU (F5).
        LINE_NOT_FOUND   = 3    ///< `stopAtLine` ran the full max distance with no crossing found.
    };

    /**
     * @brief Captured details of the last Drive fault.
     */
    struct FaultInfo
    {
        Fault              kind    = Fault::NONE;
        uint8_t            rc      = 0x00;                              ///< Lower-MCU error code (`DISPATCH_FAILED` only).
        DriveCommand::Kind cmdKind = DriveCommand::Kind::DRIVE_DIST;    ///< Kind of the offending command.
    };

    /**
     * @brief Whether a fault is currently latched.
     * @return `true` if `_fault.kind != Fault::NONE`.
     */
    bool hasFault() const;

    /**
     * @brief Snapshot of the latched fault. Undefined fields when `!hasFault()`.
     */
    FaultInfo getFault() const;

    /**
     * @brief Clear the latched fault so the queue is re-armed.
     *
     * @note Does NOT restart any motion — the queue was emptied at fault time. The
     *       caller is responsible for re-issuing whatever follow-up moves it wants.
     */
    void clearFault();

private:
    int16_t _heldYaw  = 0;       ///< Current yaw target used by `forwardGyro` and `*Gyro`.
    bool    _moveDone = true;    ///< Cleared on bounded move issue, set by `update()`.
    bool    _turnDone = true;    ///< Cleared on async turn issue, set by `update()`.
    float   _biasL    = 1.0f;    ///< Static manufacturer-bias scalar applied to raw-PWM L paths. See DECISIONS.md.
    float   _biasR    = 1.0f;    ///< Static manufacturer-bias scalar applied to raw-PWM R paths. See DECISIONS.md.

    /// @brief Optional cooperative-tick callback invoked from `_drainBlocking` so other
    ///        modules keep running during a sync-blocking Drive command. Registered via
    ///        `setTickCallback()`. `nullptr` until set; falls back to `update()` alone with a
    ///        one-time Serial warning.
    void (*_tickCallback)() = nullptr;
    /// @brief One-shot guard so the missing-callback warning is printed only once per
    ///        registration cycle (cleared by `setTickCallback`).
    bool _tickCallbackWarningEmitted = false;

    // -----------------------------------------------------------------------------------------
    // Command queue (sequential dispatch from update())
    // -----------------------------------------------------------------------------------------

    DriveCommand _queue[Config::DRIVE_QUEUE_CAPACITY];
    uint8_t      _qHead  = 0;   ///< Index of next command to dispatch.
    uint8_t      _qTail  = 0;   ///< Index of next free slot to enqueue into.
    uint8_t      _qCount = 0;   ///< Current queue depth.

    /**
     * @brief Latched fault from the queue dispatcher or the blocking drain.
     *
     * Cleared by `clearFault()`. While set, `_dispatchNext` becomes a no-op so a stale
     * queue cannot fire after a recorded failure.
     */
    FaultInfo _fault;

    // -------------------------------------------------------------------------------------
    // Trapezoidal-profile state machine (`.tShape()`-driven moves)
    // -------------------------------------------------------------------------------------
    /**
     * @brief State of the active profile move, if any.
     *
     * `IDLE` means no profile is in flight; `Drive.update()` skips the profile branch entirely.
     * Other states represent the three phases of the trapezoid plus the brief DONE tick during
     * which `update()` issues the final stop and clears `_moveDone`.
     */
    enum class ProfileState : uint8_t
    {
        IDLE,
        RAMP_UP,
        CRUISE,
        RAMP_DOWN,
        DONE
    };

    /// Mode of the active profile move (RAW / SYNC / GYRO). Mirrors `DriveMode`.
    DriveMode    _profileMode   = DriveMode::RAW;
    ProfileState _profileState  = ProfileState::IDLE;
    bool         _profileIsTime = false;    ///< true → time-bounded (driveTime); false → distance.

    // Bounds — distance variant uses encoder-degrees; time variant uses millis().
    int32_t  _profileStartEnc = 0;    ///< Encoder reading at move start (degrees).
    int32_t  _profileTotalEnc = 0;    ///< Total target travel in encoder degrees.
    uint32_t _profileStartMs  = 0;    ///< millis() at move start (time variant only).
    uint32_t _profileTotalMs  = 0;    ///< Total target duration in ms (time variant only).

    // Profile shape parameters (cached at move-start time after resolving overrides).
    int16_t  _profileCruise   = 0;    ///< Resolved cruise PWM.
    int16_t  _profileStartSpd = 0;    ///< Resolved start speed (PWM).
    int16_t  _profileEndSpd   = 0;    ///< Resolved end speed (PWM).
    int16_t  _profilePeakSpd  = 0;    ///< Peak achievable (== cruise unless triangle fallback).
    int32_t  _profileAccelEnc = 0;    ///< Per-side ramp-up distance in encoder degrees.
    int32_t  _profileDecelEnc = 0;    ///< Per-side ramp-down distance in encoder degrees.
    uint32_t _profileAccelMs  = 0;    ///< Per-side ramp-up duration in ms (time variant).
    uint32_t _profileDecelMs  = 0;    ///< Per-side ramp-down duration in ms (time variant).
    bool     _profileBrake    = true; ///< Apply brake at end?
    int16_t  _profileHeading  = 0;    ///< Heading target for GYRO mode.

    // -------------------------------------------------------------------------------------
    // Custom TurnGyro profile state machine (replaces MiniR4.DriveDC.TurnGyro)
    //
    // Parallel to the trapezoidal move profile above, but for in-place turns. The lower-MCU
    // `TurnGyro` primitive overshoots noticeably; we replace it with a cooperative two-phase
    // profile (CRUISE → SLOW → BRAKE) driven from `_updateTurnProfile()`, which is hooked
    // into `Drive.update()` parallel to `_updateProfile()`.
    // -------------------------------------------------------------------------------------
    /**
     * @brief State of the in-flight custom TurnGyro profile (parallel to ProfileState).
     *
     * `CRUISE` = fast phase at `Config::TURN_CRUISE_PWM`; `SLOW` = ramped-down phase at
     * `Config::TURN_SLOW_SPEED_PWM` once the heading error enters
     * `Config::TURN_SLOW_ZONE_DEG`; `BRAKE` = final stop after the error falls within
     * `Config::TURN_TOLERANCE_DEG`. `IDLE` means no turn is in flight; `_updateTurnProfile()`
     * short-circuits in that state.
     */
    enum class TurnProfileState : uint8_t
    {
        IDLE,
        CRUISE,
        SLOW,
        BRAKE
    };

    TurnProfileState _turnProfileState = TurnProfileState::IDLE;
    ///< @brief Current TurnGyro phase. `IDLE` = no turn in flight; non-IDLE values are
    ///<        driven by `_updateTurnProfile()`.

    int16_t _turnTargetYaw = 0;
    ///< @brief Absolute IMU-frame yaw target for the in-flight turn, normalized to
    ///<        `[-180, 180]`. Captured at dispatch time by `_execTurnDeg` /
    ///<        `_execTurnToYaw` (after `_normalizeYaw`) and read every tick by
    ///<        `_updateTurnProfile()`.

    int8_t _turnDeltaSign = 0;
    ///< @brief Sign of the initial yaw delta (`+1` = CCW / left turn, `-1` = CW / right
    ///<        turn). Captured ONCE at dispatch so the BRAKE-boundary tick does not
    ///<        oscillate sign when the IMU briefly flips across the target. `0` only in
    ///<        the IDLE state.

    /**
     * @brief Repurposed cmd.turnMode byte selecting the wheel pattern for the custom
     *        TurnGyro profile.
     *
     * @note SEMANTIC REMAP — the existing `DriveCommand::turnMode` field used to mirror
     *       the MiniR4 lower-MCU `TurnGyro` API where:
     *           - `0` = pivot (one wheel stopped)
     *           - `1` = counter-rotate (wheels in opposite directions)
     *       We REUSE the same byte but redefine its values for the custom profile:
     *           - `0` = COUNTER_ROT (both wheels, opposite directions — was old `1`)
     *           - `1` = PIVOT_LEFT  (only left wheel drives; right wheel held)
     *           - `2` = PIVOT_RIGHT (only right wheel drives; left wheel held)
     *       The remap is intentional: the new profile owns wheel dispatch directly via
     *       `Move(L, R)`, so the new encoding describes which wheels move rather than
     *       which lower-MCU `TurnGyro` mode to ask for. `_execTurnDeg` /
     *       `_execTurnToYaw` translate `cmd.turnMode` into this field at dispatch.
     */
    uint8_t _turnMotorMode = 0;

    int16_t _turnCruisePwm = Config::TURN_CRUISE_PWM;
    ///< @brief Cruise PWM magnitude for the in-flight turn profile. Captured from
    ///<        `cmd.power` at dispatch time by `_execTurnDeg`/`_execTurnToYaw`,
    ///<        defaulting to `Config::TURN_CRUISE_PWM` when the caller did not
    ///<        explicitly override via `.setPower(...)` on the builder. Non-positive
    ///<        values fall back to the Config default to keep the sign-direction
    ///<        logic in `_updateTurnProfile` clean (sign comes from `_turnDeltaSign`,
    ///<        magnitude from this field). Slow-zone PWM is always
    ///<        `Config::TURN_SLOW_SPEED_PWM` — not overridable, since the final
    ///<        approach must stay gentle enough to stop within `TURN_TOLERANCE_DEG`.

    bool _turnProfileBrakeOnDone = true;
    ///< @brief Whether to engage the active brake at completion of the turn profile.
    ///<        Mirrors `DriveCommand::brake` captured at dispatch time. Read by
    ///<        `_updateTurnProfile()` in the `BRAKE` transition.

    // -------------------------------------------------------------------------------------
    // Drive-until-line state machine (`stopAtLine()`-driven moves) — ADR-027
    //
    // Parallel to the trapezoidal move profile and the custom TurnGyro profile above, but for
    // a CONTINUOUS event-terminated seek. `_updateStopAtLine()` commands continuous motion each
    // tick per `.setMode` (it does NOT run `_updateProfile` — the stop point is unknown until a
    // line is detected, so the trapezoid's known-distance ramp-down does not apply). It reads
    // the line sensor's junction classification (via `WROLineArray`, included only in Drive.cpp)
    // and stops on a debounced `T_OR_CROSS` near the expected endpoint or faults
    // `Fault::LINE_NOT_FOUND` at the max distance. Hooked into `Drive.update()` parallel to
    // `_updateProfile()` and included in `_isIdle()`.
    // -------------------------------------------------------------------------------------
    /**
     * @brief State of the in-flight `stopAtLine` seek (parallel to ProfileState).
     *
     * `IDLE` means no seek is in flight; `_updateStopAtLine()` short-circuits in that state.
     * `SEEK` = driving blind toward the endpoint, detection disarmed. `ARMED` = within the arm
     * window, watching for a debounced `T_OR_CROSS`. `OVERSHOOT` = line detected, rolling the
     * known overshoot distance with a ramp-to-`Config::PWM_MIN_MAGNITUDE` then active-brake.
     * `DONE` is the brief tick during which `update()` issues the final stop, restores the front
     * board, and clears `_moveDone`.
     */
    enum class StopAtLineState : uint8_t
    {
        IDLE,
        SEEK,
        ARMED,
        OVERSHOOT,
        DONE
    };

    StopAtLineState _stopState = StopAtLineState::IDLE;
    ///< @brief Current seek phase. `IDLE` = no seek in flight; non-IDLE values are driven by
    ///<        `_updateStopAtLine()`.

    DriveMode _stopMode = DriveMode::SYNC;
    ///< @brief Mode of the active seek (RAW / SYNC / GYRO). Selects which continuous primitive
    ///<        `_updateStopAtLine()` commands each tick. Mirrors `DriveMode`.

    int32_t _stopStartEnc = 0;
    ///< @brief Per-side encoder reading (degrees) captured at seek dispatch; `traveled` is
    ///<        measured relative to this each tick.

    int16_t _stopCruise = Config::SPEED_NORMAL;
    ///< @brief Resolved cruise PWM magnitude held during SEEK/ARMED. Captured from `cmd.power`
    ///<        at dispatch.

    int16_t _stopHeading = 0;
    ///< @brief Yaw target held during a `DriveMode::GYRO` seek (ignored otherwise). Captured
    ///<        from `cmd.heading` at dispatch.

    float _stopExpectedCm = 0.0f;
    ///< @brief Nominal distance to the expected crossing line. Captured from `cmd.expectedCm`.

    float _stopArmWithin = Config::STOP_AT_LINE_ARM_WINDOW_CM;
    ///< @brief Arm detection within this many cm of `_stopExpectedCm` (clamped so the arm
    ///<        threshold never goes below 0). Captured from `cmd.armWithin`.

    float _stopMaxCm = 0.0f;
    ///< @brief Absolute max travel before faulting `LINE_NOT_FOUND`. Captured from `cmd.maxCm`
    ///<        (default `expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM`).

    float _stopOvershoot = Config::LINE_SENSOR_AXLE_OFFSET_CM;
    ///< @brief Post-detect roll distance so the wheel axle lands on the line. Captured from
    ///<        `cmd.overshoot`.

    int32_t _stopDetectEnc = 0;
    ///< @brief Per-side encoder reading (degrees) captured the tick the line is confirmed; the
    ///<        OVERSHOOT phase rolls `_stopOvershoot` cm relative to this.

    uint8_t _stopDetectStreak = 0;
    ///< @brief Consecutive `T_OR_CROSS` samples seen in the ARMED phase. Confirms detection once
    ///<        it reaches `Config::STOP_AT_LINE_DETECT_TICKS`; reset by a non-crossing sample.

    uint32_t _stopLastProgressMs = 0;
    ///< @brief `millis()` at the last detected progress — the no-progress watchdog reset point.
    ///<        Seeded at dispatch and advanced each tick `stopAtLineProgressed(...)` is true; the
    ///<        watchdog faults `LINE_NOT_FOUND` once `now - this >= STOP_AT_LINE_STALL_TIMEOUT_MS`.

    float _stopLastProgressCm = 0.0f;
    ///< @brief `traveled` (cm) at the last detected progress — the progress-detector baseline. Seeded
    ///<        at dispatch and advanced (with `_stopLastProgressMs`) whenever `traveled` gains at
    ///<        least `Config::STOP_AT_LINE_PROGRESS_EPS_CM` since this value.

    bool _stopReverse = false;
    ///< @brief Whether this seek backs into the line via the rear board (needs
    ///<        `Config::LINE_BOARD_COUNT == 2`). Captured from `cmd.reverse`; in reverse,
    ///<        `expectedCm`, `armWithin`, and `overshoot` are all measured along backward travel.

    bool _stopBrake = true;
    ///< @brief Whether to engage the active brake at the end of the OVERSHOOT roll. Mirrors
    ///<        `DriveCommand::brake` captured at dispatch time.

    bool _isLineFound = false;
    ///< @brief Latched result of the last `stopAtLine` (see `isLineFound()`): `true` on a
    ///<        confirmed crossing, `false` while seeking or after a `LINE_NOT_FOUND` fault.

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

    // `presetToPwm` and `cmToEncDeg` were previously private members of this class.
    // They moved to free functions in `WROMath.h` once `LineFollower` started needing
    // both — see WROMath.h's preamble for the policy on what lives there.

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

    /**
     * @brief Emit one continuous-PWM command in the given mode at the given magnitude/heading.
     *
     * The single shared continuous-emit path for both the trapezoidal profile
     * (`_updateProfile`) and the `stopAtLine` seek (`_updateStopAtLine`). Dispatches the
     * mode-correct lower-MCU primitive: `RAW` applies `_applyBias` then `deadbandSnap` on each
     * wheel and calls `Move(L, R)`; `SYNC` calls `MoveSync(deadbandSnap(mag), deadbandSnap(mag))`;
     * `GYRO` calls `MoveGyro(deadbandSnap(mag), heading)`. RAW is the only mode that applies the
     * motor-bias scalar — SYNC / GYRO close the loop on the STM32 and a feedforward scalar would
     * fight the inner PID (see DECISIONS.md).
     *
     * @param mode    Continuous primitive to dispatch (`RAW` / `SYNC` / `GYRO`).
     * @param mag     Signed PWM magnitude (caller carries direction in the sign).
     * @param heading Yaw target — only consulted in `GYRO` mode; ignored otherwise.
     *
     * @note Reads NO instance state — every input is a parameter, so the profile
     *       (`_profileMode` / `_profileHeading`) and the seek (`_stopMode` / `_stopHeading`)
     *       cannot cross-contaminate through it.
     */
    void _emitContinuous(DriveMode mode, int16_t mag, int16_t heading);

    /**
     * @brief Advance the profile state machine one tick. Called from `update()` when active.
     *
     * Reads encoder (distance variant) or millis() (time variant), computes the traveled
     * fraction in `[0, 1]`, calls `_profileSpeedAt()` for the current setpoint, and dispatches
     * the correct continuous MMLower primitive (`Move` / `MoveSync` / `MoveGyro`) for the
     * configured `_profileMode`. RAW mode applies `_applyBias`; SYNC and GYRO do NOT (the
     * STM32 closes the loop — see DECISIONS.md). When `frac >= 1.0`, transitions to `DONE`,
     * marks the done flags, and returns to `IDLE`. The DONE branch SPLITS by endSpeed:
     *   - `_profileEndSpd == 0` → inline `Move(0,0) + brake(_profileBrake)` (does NOT call
     *     `stop()` because that would `_clearQueue()` and wipe queued follow-ups; W1 fix).
     *   - `_profileEndSpd != 0` → no motor command at all; the prior tick's setpoint (already
     *     at endSpeed) holds, and the next queued move's first tick overwrites it. This is
     *     how chained moves preserve cruise across boundaries.
     *
     * @note Idempotent when `_profileState == IDLE` — short-circuits immediately.
     */
    void _updateProfile();

    /**
     * @brief Cooperative tick for the custom TurnGyro state machine (replaces
     *        `MiniR4.DriveDC.TurnGyro`). Reads the IMU yaw, computes the normalized
     *        delta to `_turnTargetYaw`, dispatches the appropriate `Move(L, R)` wheel
     *        pattern per `_turnMotorMode`, and transitions through
     *        `CRUISE` → `SLOW` → `BRAKE` based on `Config::TURN_SLOW_ZONE_DEG` and
     *        `Config::TURN_TOLERANCE_DEG`. Idempotent / no-op when
     *        `_turnProfileState == IDLE`.
     *
     * @note Hook ordering inside `Drive.update()`: drain-task-done bit →
     *       `_updateProfile()` → `_updateTurnProfile()` → `_dispatchNext`. The
     *       drain-task-done bit must NOT flip `_turnDone` while a turn profile is in
     *       flight (same W1 guard as the move profile).
     */
    void _updateTurnProfile();

    /**
     * @brief Cooperative tick for the `stopAtLine` seek state machine (ADR-027).
     *
     * Drives a CONTINUOUS event-terminated seek that does NOT use `_updateProfile()` (the stop
     * point is unknown until the line is detected, so the trapezoid's known-distance ramp-down
     * does not apply). Each tick it measures `traveled` from `_stopStartEnc`, commands the
     * continuous primitive selected by `_stopMode` (`Move` / `MoveSync` / `MoveGyro`), and steps
     * the state machine:
     *   - `SEEK` — cruise flat at `_stopCruise` until `traveled >= _stopExpectedCm -
     *     _stopArmWithin` (arm threshold clamped at 0), then transition to `ARMED`.
     *   - `ARMED` — keep cruising; read `WROLineArray::getJunctionType()` and count consecutive
     *     `T_OR_CROSS` samples. On `Config::STOP_AT_LINE_DETECT_TICKS` in a row, latch
     *     `_isLineFound = true`, capture `_stopDetectEnc`, and transition to `OVERSHOOT`. If
     *     `traveled >= _stopMaxCm` first, fault `Fault::LINE_NOT_FOUND` (stop + clear queue).
     *   - `OVERSHOOT` — roll the known `_stopOvershoot` distance, ramping cruise →
     *     `Config::PWM_MIN_MAGNITUDE`, then transition to `DONE`.
     *   - `DONE` — issue the final stop (active-brake per `_stopBrake`), restore the front board
     *     (`setReverseActive(false)`), set `_moveDone`, and return to `IDLE`.
     *
     * @note Idempotent / no-op when `_stopState == IDLE` — short-circuits immediately.
     * @note Hook ordering inside `Drive.update()`: drain-task-done bit → `_updateProfile()` →
     *       `_updateTurnProfile()` → `_updateStopAtLine()` → `_dispatchNext`. The drain-task-done
     *       bit must NOT flip `_moveDone` while a seek is in flight (same W1 guard as the move
     *       and turn profiles).
     * @note Reads `WROLineArray::getJunctionType()` directly (NOT `Intersection::isAtIntersection`,
     *       which is read-and-clear and owned by PathTracker's nav consumer). The
     *       `WROLineArray.h` include lives in Drive.cpp only — the header needs none of its
     *       definition.
     * @note `@untested on hardware` for the reverse (rear-board) path until the rear board is
     *       wired (ADR-024).
     */
    void _updateStopAtLine();

    /**
     * @brief Tear down an in-flight `stopAtLine` seek: stop, brake, restore the front board, and
     *        return the seek state machine to IDLE.
     *
     * The single shared seek-teardown path. Issues `Move(0, 0)` + `brake(brake)`, restores the
     * front board (`LineArray.setReverseActive(false)` — a reverse seek may have selected the rear
     * board; no-op in 1-board configs), sets `_stopState = IDLE`, and marks `_moveDone` /
     * `_turnDone` so `_dispatchNext` can hand off on the same tick. Does NOT clear the queue — the
     * caller decides queue semantics (normal DONE hand-off keeps the queue; a fault path calls
     * `_clearQueue()` itself before this).
     *
     * @param brake `true` = engage the active brake; `false` = coast.
     *
     * @note Called from the DONE state (brake = `_stopBrake`), the no-line / max-distance / stall
     *       fault paths (brake = `true`), and from `stop()` / `_drainBlocking()` when an in-flight
     *       seek is aborted.
     */
    void _endStopAtLine(bool brake);

    /**
     * @brief Compute the current speed setpoint based on traveled fraction (0.0 → 1.0).
     *
     * Implements the trapezoidal velocity profile:
     *  - Below the accel threshold: linear interp from `_profileStartSpd` → `_profilePeakSpd`.
     *  - Above the decel threshold: linear interp from `_profilePeakSpd` → `_profileEndSpd`.
     *  - In between: hold at `_profilePeakSpd`.
     *
     * In the triangle-fallback case, `_profilePeakSpd` is below `_profileCruise` and the
     * accel/decel intervals abut at the midpoint — the integral of the profile still equals
     * the requested distance/duration.
     *
     * @param traveledFrac Fraction of total move completed in [0.0, 1.0].
     * @return Current speed setpoint as PWM in `[-100, 100]`.
     */
    int16_t _profileSpeedAt(float traveledFrac) const;

    /**
     * @brief Resolve all profile parameters and seed the state machine.
     *
     * Called from a builder's destructor. Computes per-side ramp distances with all clamps
     * (MIN_CM / MAX_CM / half-of-total) applied, detects the triangle-fallback case and
     * computes the reduced peak, decides whether to skip a ramp entirely (no-ramp epsilon
     * for chained moves), and transitions the state machine to RAMP_UP. If a profile is
     * already in flight, it is cancelled silently (its motors keep running until this new
     * call's first `_updateProfile` tick overwrites them).
     *
     * @param mode        RAW / SYNC / GYRO.
     * @param isTime      `true` for `driveTime`, `false` for `driveDist`.
     * @param totalCm     Total travel in cm (distance variant). Ignored for time variant.
     * @param totalSec    Total duration in seconds (time variant). Ignored for distance variant.
     * @param cruise      Target cruise speed (PWM).
     * @param startSpeed  Starting speed at t=0 (PWM).
     * @param endSpeed    Ending speed at completion (PWM).
     * @param accel       PWM per cm. For time variant, interpreted via the cmToMs scaling
     *                    documented in the body — see `tShape()` doxygen on the time builder.
     * @param brake       Apply brake at end.
     * @param heading     Yaw target for GYRO mode (ignored otherwise).
     * @return `0x00` on successful seed; non-zero if pre-conditions failed
     *         (currently only `0x01` when `accel <= 0` — caller should fall back to a
     *         fire-and-forget non-profiled move).
     */
    uint8_t _execProfileMove(DriveMode mode,
                             bool      isTime,
                             float     totalCm,
                             float     totalSec,
                             int16_t   cruise,
                             int16_t   startSpeed,
                             int16_t   endSpeed,
                             float     accel,
                             bool      brake,
                             int16_t   heading);

    // -------------------------------------------------------------------------------------
    // Command-queue plumbing — invoked from update() and from builder destructors.
    // -------------------------------------------------------------------------------------

    /**
     * @brief Push a command onto the queue. Drops + warns on overflow.
     *
     * The queue is a fixed-size FIFO with `Config::DRIVE_QUEUE_CAPACITY` slots; if it is
     * already full, the new command is dropped and a warning is printed to Serial. This is
     * a caller bug — the path planner should not be issuing more bounded moves than the
     * queue can hold without occasional `_drainBlocking()` calls.
     *
     * @param cmd Fully-populated command POD. Copy semantics — caller's instance is no
     *            longer needed after the call returns.
     * @return `true` if enqueued; `false` if dropped due to overflow.
     */
    bool _enqueue(const DriveCommand& cmd);

    /**
     * @brief Pop the front command if any, dispatch it. Called from `update()` when idle.
     *
     * Also auto-disables `LineFollower` (with `brake=false`) before dispatching, so the
     * follower's tick loop does not race the new bounded/profiled move on motor PWM. The
     * caller must re-enable the follower explicitly after their queued move sequence completes.
     *
     * @note No-ops if the queue is empty — safe to call unconditionally.
     */
    void _dispatchNext();

    /**
     * @brief `true` when no current task is in flight AND no profile is active.
     *
     * Used by `update()` to decide whether to dispatch the next queued command and by
     * blocking-variant destructors (via `_drainBlocking`) to know when their drain phase
     * is complete.
     *
     * @return `true` iff `_moveDone && _turnDone && _profileState == ProfileState::IDLE &&
     *         _stopState == StopAtLineState::IDLE`.
     */
    bool _isIdle() const;

    /**
     * @brief Drain queue and current task synchronously. Used by blocking variants.
     *
     * Spins, calling `update()` and `delay(2)`, until `_isIdle()` returns true AND the queue
     * is empty.
     *
     * @note This is the THIRD allowed `delay()` site in the codebase (after
     *       `calibrateMotorBias` and the historical per-builder spin which is now folded into
     *       this helper). The blocking caller is by definition off the cooperative tick chain,
     *       so this spin doesn't violate the iron rule — which targets non-blocking `update()`
     *       chains, not synchronous-orchestration paths.
     */
    void _drainBlocking();

    /**
     * @brief Empty the queue without dispatching. Called from `stop()`.
     *
     * Resets head, tail, and count to zero — the storage is left untouched but is no longer
     * referenced. Used to implement `Drive.stop()`'s abort semantics: pending commands are
     * discarded, not just the in-flight one.
     */
    void _clearQueue();
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
 * **Sync-blocking by default; chain `.async()` to opt out.** Auto-executes when the temporary
 * destructs at end-of-statement. Setters return `*this` by reference for chaining. Defaults:
 * `DriveMode::SYNC`, `tShape` on, power `Config::SPEED_NORMAL`, brake `true`, blocking `true`
 * (the destructor drains the queue before returning), heading = `Drive`'s current `_heldYaw`
 * (only consulted in `DriveMode::GYRO`). The SYNC + tShape default reflects the auto-path
 * planner being the dominant consumer — see DECISIONS.md "Sync + T-shape as default move
 * profile". Mission scripts read top-down like Python — each verb call blocks until done. The
 * auto-path planner opts out per call via `.async()` so it can poll `Drive.isIdle()` instead.
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
          _tShape(other._tShape),
          _accelExplicit(other._accelExplicit),
          _accel(other._accel),
          _startSpeedSet(other._startSpeedSet),
          _startSpeed(other._startSpeed),
          _endSpeedSet(other._endSpeedSet),
          _endSpeed(other._endSpeed),
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
     * @param p Preset selector — resolved via `presetToPwm` (WROMath.h) at execute-time.
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
     * @param b `true` = sync (default — library spins on `delay`); `false` = async.
     * @return `*this` for chaining.
     * @warning Blocking variants MUST NOT be called from inside any module's `update()`.
     * @note The default is now `true` (sync). Prefer `.async()` for the auto-path-planner
     *       opt-out — it documents intent more clearly than `.setBlocking(false)`.
     */
    DriveDistBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Opt this command out of the default sync-blocking behaviour.
     *
     * Equivalent to `.setBlocking(false)`. The builder's destructor will enqueue the
     * command and return immediately, without draining the queue. Use for the auto-path
     * planner case where the caller ticks `loop()` and polls `Drive.isIdle()` to know
     * when to issue the next command — the planner explicitly does NOT want each verb
     * call to block its own tick.
     *
     * @return Reference to this builder for chaining.
     * @note The default is sync (`_blocking = true`). Hand-written mission scripts should
     *       almost never need `.async()` — it's a tool for the auto-path planner.
     */
    DriveDistBuilder& async()
    {
        _blocking = false;
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
     * @brief Enable trapezoidal velocity profile for this move.
     *
     * @note **`tShape` is on by default** — this setter is idempotent and exists for
     *       explicit-readability and parity with `setStartSpeed`/`setEndSpeed`/`setAccel`.
     *       To opt OUT of the profile on a one-off, chain
     *       `.setStartSpeed(cruise).setEndSpeed(cruise)` so the ramp formula short-circuits
     *       (the no-ramp epsilon `Config::TSHAPE_NO_RAMP_EPSILON_PWM` then fires on both
     *       sides). Defaulting it on is intentional — see DECISIONS.md "Sync + T-shape as
     *       default move profile".
     *
     * Speed ramps from `startSpeed` (default 0) up to the resolved cruise, plateaus, then
     * ramps down to `endSpeed` (default 0, but defaults to cruise when `setBrake(false)` is
     * also chained — assumes a no-brake move is chaining into a follow-up).
     *
     * Acceleration is `Config::TSHAPE_ACCEL_PWM_PER_CM` unless overridden via `.setAccel()`.
     * Per-side ramp distance is implicit: `(cruise - endpointSpeed) / accel`, clamped to
     * `[Config::TSHAPE_RAMP_MIN_CM, Config::TSHAPE_RAMP_MAX_CM]` and to half of the move
     * length. If accel + decel distance > total, profile becomes a triangle (peak speed
     * reduced).
     *
     * @return Reference to this builder for chaining.
     * @note Profile runs cooperatively from `Drive.update()` — issuing a new bounded move
     *       while a profile is in flight cancels the in-flight profile silently. Callers
     *       must poll `Drive.isMoveDone()` if sequencing matters.
     */
    DriveDistBuilder& tShape()
    {
        _tShape = true;
        return *this;
    }

    /**
     * @brief Override the per-PWM-per-cm acceleration rate for this profiled move.
     *
     * Implies `tShape()`. If `tShape()` was not also chained, this setter has no effect.
     *
     * @param pwmPerCm PWM units gained per cm of travel during ramps.
     * @return Reference to this builder for chaining.
     */
    DriveDistBuilder& setAccel(float pwmPerCm)
    {
        _accel         = pwmPerCm;
        _accelExplicit = true;
        _tShape        = true;
        return *this;
    }

    /**
     * @brief Set the starting speed of the profiled move (default 0 = at rest).
     *
     * Used for chained moves: pass the previous move's exit speed so the new move starts
     * smoothly without a step in PWM. Implies `tShape()`.
     *
     * @param startPwm Starting speed in `[-100, 100]`.
     * @return Reference to this builder for chaining.
     */
    DriveDistBuilder& setStartSpeed(int16_t startPwm)
    {
        _startSpeed    = startPwm;
        _startSpeedSet = true;
        _tShape        = true;
        return *this;
    }

    /**
     * @brief Set the ending speed of the profiled move (default 0 = stop at rest).
     *
     * For chained moves, pass cruise to skip the ramp-down at this segment's end. Implies
     * `tShape()`. Note: when `setBrake(false)` is chained, the default automatically
     * resolves to `cruise` instead of 0 — explicit `setEndSpeed(0)` overrides that.
     *
     * @param endPwm Ending speed in `[-100, 100]`.
     * @return Reference to this builder for chaining.
     */
    DriveDistBuilder& setEndSpeed(int16_t endPwm)
    {
        _endSpeed    = endPwm;
        _endSpeedSet = true;
        _tShape      = true;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-enqueue. Resolves the final knobs and pushes a
     *        `DriveCommand` onto the Drive queue.
     *
     * The actual dispatch happens later, from `Drive.update()`, when the previous queued
     * command (if any) has finished. This makes source-order sequencing of multiple moves
     * work without caller-side polling — see `Config::DRIVE_QUEUE_CAPACITY` rationale.
     *
     * Endpoint-default rule: when `_brake == false` and `setEndSpeed()` was never called,
     * `endSpeed` resolves to the cruise PWM (the assumption is that a no-brake move is
     * chaining into a follow-up segment). Override with an explicit `setEndSpeed(0)` if a
     * coast-to-stop is intended.
     *
     * Min-move threshold: when `_tShape` is set but `_cm < Config::TSHAPE_MIN_MOVE_CM`, the
     * profile is silently skipped — the command is enqueued as `Kind::DRIVE_DIST` instead of
     * `Kind::PROFILE_DIST`. The half-of-total clamp on per-side ramp distances would force
     * the effective acceleration steeper than `TSHAPE_ACCEL_PWM_PER_CM` on short moves; a
     * non-profiled constant-cruise move is the cleaner fallback.
     *
     * @note Idempotent: the `_executed` guard prevents a second enqueue if a future change
     *       re-enables move semantics.
     * @note When `_blocking == true`, the destructor calls `_drainBlocking()` before AND
     *       after the enqueue: drain anything already in the queue plus the in-flight task,
     *       enqueue this command, drain again. Result: synchronous semantics on top of the
     *       queue infrastructure. `_drainBlocking()` is the THIRD allowed `delay()` site —
     *       see its doxygen.
     */
    ~DriveDistBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;

        // Reject negative distance at the public-API boundary (code review F4). The
        // contract is: distance is non-negative, reverse direction is expressed via
        // negative power. A typo like `Drive.driveDist(-3)` would otherwise reach
        // `cmToEncDeg`, casting a negative float into a huge `uint16_t` encoder target.
        // The profile path already clamps as a defensive WARNING-2 — this is the
        // matching front-door check.
        if (_cm < 0.0f)
        {
            Serial.println(F("[Drive] driveDist: negative cm rejected"));
            return;
        }

        // Resolve cruise — explicit setPower wins, then setPreset, then SPEED_NORMAL.
        const int16_t cruise = _resolvePower();

        // Build the command POD.
        DriveCommand cmd;
        cmd.cm      = _cm;
        cmd.power   = cruise;
        cmd.mode    = _mode;
        cmd.brake   = _brake;
        cmd.heading = _headingSet ? _heading : _drive._heldYaw;

        // Reject a non-positive acceleration at the public-API boundary (finding #16). A bad
        // `.setAccel(0)` / negative would otherwise enqueue a PROFILE_DIST whose `_execProfileMove`
        // rejects it with rc 0x01 only when it reaches the FRONT of the queue — and `_dispatchNext`
        // treats any non-zero rc as a hard fault (stop + DISPATCH_FAILED + _clearQueue), discarding
        // every other valid queued command. Fail fast here by falling back to a non-profiled move,
        // mirroring the min-move-threshold fallback below.
        const bool accelValid = _accel > 0.0f;
        if (_tShape && !accelValid)
        {
            Serial.println(F("[Drive] driveDist: non-positive accel — falling back to non-profiled move"));
        }

        // Decide profile vs non-profile, with the min-move-threshold no-op for tShape. Below
        // the threshold we silently fall back to a non-profiled constant-cruise move so the
        // constant-acceleration contract isn't violated on short hops.
        if (_tShape && accelValid && _cm >= Config::TSHAPE_MIN_MOVE_CM)
        {
            cmd.kind       = DriveCommand::Kind::PROFILE_DIST;
            // Resolve start/end speeds. The no-brake → endSpeed=cruise rule kicks in here:
            // a `.setBrake(false).tShape()` move is presumed to be chaining into a follow-up,
            // so ramping down to zero would defeat the purpose. Explicit `.setEndSpeed(0)`
            // still wins because `_endSpeedSet` becomes true.
            //
            // Sign-by-cruise on the default start speed: a reverse move (cruise<0) must ramp
            // from a negative-direction PWM, otherwise the trapezoid initially drives the
            // chassis FORWARD before crossing zero into the requested reverse direction
            // (see code review F1). Explicit `.setStartSpeed(...)` still wins.
            cmd.startSpeed = _startSpeedSet
                               ? _startSpeed
                               : (cruise >= 0
                                      ? Config::TSHAPE_START_SPEED
                                      : (int16_t) -Config::TSHAPE_START_SPEED);
            cmd.endSpeed   = _endSpeedSet
                               ? _endSpeed
                               : (_brake ? Config::TSHAPE_END_SPEED : cruise);
            cmd.accel      = _accel;
        }
        else
        {
            cmd.kind = DriveCommand::Kind::DRIVE_DIST;
            // Profile-only fields are unused; leave defaulted.
        }

        if (_blocking)
        {
            // Blocking variant: drain anything already in the queue plus the current move,
            // then enqueue this one and drain again. Synchronous semantics on top of the
            // queue infrastructure. The drain helper is the third allowed `delay()` site.
            _drive._drainBlocking();
            _drive._enqueue(cmd);
            _drive._drainBlocking();
        }
        else
        {
            _drive._enqueue(cmd);
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
            return presetToPwm(_preset);
        }
        return Config::SPEED_NORMAL;
    }

    DriveClass& _drive;
    float       _cm;
    int16_t     _power          = Config::SPEED_NORMAL;
    bool        _powerExplicit  = false;
    Config::SpeedPreset _preset = Config::SpeedPreset::NORMAL;
    bool        _presetSet      = false;
    DriveMode   _mode           = DriveMode::SYNC;   ///< default SYNC — see DECISIONS.md.
    bool        _brake          = true;
    bool        _blocking       = true;    ///< default sync — destructor blocks until command completes.
                                           ///< Auto-planner opts out per call via `.async()`.
    int16_t     _heading        = 0;
    bool        _headingSet     = false;
    bool        _tShape         = true;    ///< default trapezoidal profile — see DECISIONS.md.
    bool        _accelExplicit  = false;
    float       _accel          = Config::TSHAPE_ACCEL_PWM_PER_CM;
    bool        _startSpeedSet  = false;
    int16_t     _startSpeed     = Config::TSHAPE_START_SPEED;
    bool        _endSpeedSet    = false;
    int16_t     _endSpeed       = Config::TSHAPE_END_SPEED;
    bool        _executed       = false;
};

/**
 * @brief Fluent builder for `Drive.driveTime(sec)` — time-bounded straight-line move.
 *
 * **Sync-blocking by default; chain `.async()` to opt out.** Auto-executes on destruction.
 * Setters mirror `DriveDistBuilder`; only the time vs. distance verb differs. Defaults:
 * `DriveMode::SYNC`, `tShape` on (see DECISIONS.md "Sync + T-shape as default move profile").
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
          _tShape(other._tShape),
          _accelExplicit(other._accelExplicit),
          _accel(other._accel),
          _startSpeedSet(other._startSpeedSet),
          _startSpeed(other._startSpeed),
          _endSpeedSet(other._endSpeedSet),
          _endSpeed(other._endSpeed),
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
     * @param b `true` = sync (default); `false` = async.
     * @return `*this` for chaining.
     * @note The default is now `true` (sync). Prefer `.async()` for the auto-path-planner
     *       opt-out — it documents intent more clearly than `.setBlocking(false)`.
     */
    DriveTimeBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Opt this command out of the default sync-blocking behaviour.
     *
     * Equivalent to `.setBlocking(false)`. The builder's destructor will enqueue the
     * command and return immediately, without draining the queue. Use for the auto-path
     * planner case where the caller ticks `loop()` and polls `Drive.isIdle()` to know
     * when to issue the next command — the planner explicitly does NOT want each verb
     * call to block its own tick.
     *
     * @return Reference to this builder for chaining.
     * @note The default is sync (`_blocking = true`). Hand-written mission scripts should
     *       almost never need `.async()` — it's a tool for the auto-path planner.
     */
    DriveTimeBuilder& async()
    {
        _blocking = false;
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
     * @brief Enable trapezoidal velocity profile for this time-bounded move.
     *
     * @note **`tShape` is on by default** — this setter is idempotent. To opt OUT, chain
     *       `.setStartSpeed(cruise).setEndSpeed(cruise)`; see `DriveDistBuilder::tShape()`
     *       for the full rationale and DECISIONS.md "Sync + T-shape as default move profile".
     *
     * Speed ramps from `startSpeed` (default 0) up to the resolved cruise, plateaus, then
     * ramps back down to `endSpeed` (default 0, but defaults to cruise when
     * `setBrake(false)` is also chained).
     *
     * Acceleration is `Config::TSHAPE_ACCEL_PWM_PER_CM`. Because a time-bounded move has no
     * a-priori distance, ramp DURATIONS are derived via the approximation
     * `rampMs ≈ rampCm / |cruise| * 1000` — the units-coherent equivalent ramp time at the
     * target cruise speed. This is documented as an approximation; it is not as physically
     * meaningful as the distance-variant formula but matches the same `Config::TSHAPE_*`
     * tuning constants. Per-side ramp duration is then clamped to half of `totalSec`. If the
     * accel + decel durations together exceed `totalSec`, the profile collapses to a triangle
     * with reduced peak.
     *
     * @return Reference to this builder for chaining.
     * @note Profile runs cooperatively from `Drive.update()` — issuing a new bounded move
     *       while a profile is in flight cancels the in-flight profile silently.
     */
    DriveTimeBuilder& tShape()
    {
        _tShape = true;
        return *this;
    }

    /**
     * @brief Override the per-PWM-per-cm acceleration rate for this profiled move.
     *
     * Implies `tShape()`. For the time variant the rate is reinterpreted as ramp duration
     * via the cm/cruise scaling — see `tShape()` doxygen.
     *
     * @param pwmPerCm PWM units gained per cm of travel during ramps.
     * @return Reference to this builder for chaining.
     */
    DriveTimeBuilder& setAccel(float pwmPerCm)
    {
        _accel         = pwmPerCm;
        _accelExplicit = true;
        _tShape        = true;
        return *this;
    }

    /**
     * @brief Set the starting speed of the profiled move (default 0 = at rest).
     *
     * Used for chained moves: pass the previous move's exit speed so the new move starts
     * smoothly without a step in PWM. Implies `tShape()`.
     *
     * @param startPwm Starting speed in `[-100, 100]`.
     * @return Reference to this builder for chaining.
     */
    DriveTimeBuilder& setStartSpeed(int16_t startPwm)
    {
        _startSpeed    = startPwm;
        _startSpeedSet = true;
        _tShape        = true;
        return *this;
    }

    /**
     * @brief Set the ending speed of the profiled move (default 0 = stop at rest).
     *
     * For chained moves, pass cruise to skip the ramp-down at this segment's end. Implies
     * `tShape()`. Note: when `setBrake(false)` is chained, the default automatically
     * resolves to `cruise` instead of 0 — explicit `setEndSpeed(0)` overrides that.
     *
     * @param endPwm Ending speed in `[-100, 100]`.
     * @return Reference to this builder for chaining.
     */
    DriveTimeBuilder& setEndSpeed(int16_t endPwm)
    {
        _endSpeed    = endPwm;
        _endSpeedSet = true;
        _tShape      = true;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-enqueue. Resolves the final knobs and pushes a
     *        `DriveCommand` onto the Drive queue.
     *
     * The actual dispatch happens later, from `Drive.update()`, when the previous queued
     * command (if any) has finished. Endpoint-default rule mirrors `DriveDistBuilder`:
     * when `_brake == false` and `setEndSpeed()` was never called, `endSpeed` resolves to
     * cruise (chained-move presumption).
     *
     * Min-move threshold: when `_tShape` is set but `_sec < Config::TSHAPE_MIN_MOVE_SEC`,
     * the profile is silently skipped — the command is enqueued as `Kind::DRIVE_TIME`
     * instead of `Kind::PROFILE_TIME`.
     *
     * @note When `_blocking == true`, the destructor calls `_drainBlocking()` before AND
     *       after the enqueue (third allowed `delay()` site — see its doxygen).
     */
    ~DriveTimeBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;

        // Reject negative duration — see DriveDistBuilder::~ for rationale (F4).
        if (_sec < 0.0f)
        {
            Serial.println(F("[Drive] driveTime: negative sec rejected"));
            return;
        }

        // Resolve cruise — explicit setPower wins, then setPreset, then SPEED_NORMAL.
        const int16_t cruise = _resolvePower();

        DriveCommand cmd;
        cmd.sec     = _sec;
        cmd.power   = cruise;
        cmd.mode    = _mode;
        cmd.brake   = _brake;
        cmd.heading = _headingSet ? _heading : _drive._heldYaw;

        // Reject a non-positive acceleration at the public-API boundary (finding #16) — see
        // DriveDistBuilder for the full rationale (a bad accel would otherwise poison the whole
        // queue at dispatch time). Fall back to a non-profiled move, mirroring the min-move guard.
        const bool accelValid = _accel > 0.0f;
        if (_tShape && !accelValid)
        {
            Serial.println(F("[Drive] driveTime: non-positive accel — falling back to non-profiled move"));
        }

        if (_tShape && accelValid && _sec >= Config::TSHAPE_MIN_MOVE_SEC)
        {
            cmd.kind       = DriveCommand::Kind::PROFILE_TIME;
            // Sign-by-cruise on the default start speed — see DriveDistBuilder for rationale (F1).
            cmd.startSpeed = _startSpeedSet
                               ? _startSpeed
                               : (cruise >= 0
                                      ? Config::TSHAPE_START_SPEED
                                      : (int16_t) -Config::TSHAPE_START_SPEED);
            cmd.endSpeed   = _endSpeedSet
                               ? _endSpeed
                               : (_brake ? Config::TSHAPE_END_SPEED : cruise);
            cmd.accel      = _accel;
        }
        else
        {
            cmd.kind = DriveCommand::Kind::DRIVE_TIME;
        }

        if (_blocking)
        {
            _drive._drainBlocking();
            _drive._enqueue(cmd);
            _drive._drainBlocking();
        }
        else
        {
            _drive._enqueue(cmd);
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
            return presetToPwm(_preset);
        }
        return Config::SPEED_NORMAL;
    }

    DriveClass& _drive;
    float       _sec;
    int16_t     _power          = Config::SPEED_NORMAL;
    bool        _powerExplicit  = false;
    Config::SpeedPreset _preset = Config::SpeedPreset::NORMAL;
    bool        _presetSet      = false;
    DriveMode   _mode           = DriveMode::SYNC;   ///< default SYNC — see DECISIONS.md.
    bool        _brake          = true;
    bool        _blocking       = true;    ///< default sync — destructor blocks until command completes.
                                           ///< Auto-planner opts out per call via `.async()`.
    int16_t     _heading        = 0;
    bool        _headingSet     = false;
    bool        _tShape         = true;    ///< default trapezoidal profile — see DECISIONS.md.
    bool        _accelExplicit  = false;
    float       _accel          = Config::TSHAPE_ACCEL_PWM_PER_CM;
    bool        _startSpeedSet  = false;
    int16_t     _startSpeed     = Config::TSHAPE_START_SPEED;
    bool        _endSpeedSet    = false;
    int16_t     _endSpeed       = Config::TSHAPE_END_SPEED;
    bool        _executed       = false;
};

/**
 * @brief Fluent builder for `Drive.turnDeg(deg)` — relative-yaw-delta turn.
 *
 * **Sync-blocking by default; chain `.async()` to opt out.** Auto-executes on destruction.
 * Defaults: power `Config::TURN_CRUISE_PWM`, mode `Config::TURN_MODE`, brake `Config::TURN_BRAKE`,
 * blocking `true`.
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
     * @brief Select the wheel pattern used by the custom TurnGyro state machine
     *        (`_updateTurnProfile` in Drive.cpp; replaces MiniR4.DriveDC.TurnGyro).
     *
     * @param m Wheel-pattern byte. Encoding:
     *          - `0` = COUNTER_ROT — both wheels spin opposite directions, chassis pivots around its center.
     *          - `1` = PIVOT_LEFT — left wheel braked; right wheel drives forward; chassis pivots around the LEFT wheel.
     *          - `2` = PIVOT_RIGHT — right wheel braked; left wheel drives forward; chassis pivots around the RIGHT wheel.
     *
     * @note This is the NEW encoding (introduced by the custom TurnGyro rewrite — see DECISIONS.md / the
     *       2026-05-17 plan). It is intentionally NOT the legacy `MiniR4.DriveDC.TurnGyro` encoding
     *       (`0=pivot, 1=counter-rot`); the new state machine owns wheel dispatch directly via `Move(L, R)`
     *       and needed a third value for the pivot-side selection. PathTracker maps `Routes::TurnMode::*`
     *       to this byte via `_turnModeToByte` in PathTracker.cpp.
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
     * @param b `true` = sync (default — library spins); `false` = async.
     * @return `*this` for chaining.
     * @warning Blocking variants MUST NOT be called from inside any module's `update()`.
     * @note The default is now `true` (sync). Prefer `.async()` for the auto-path-planner
     *       opt-out — it documents intent more clearly than `.setBlocking(false)`.
     */
    TurnDegBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Opt this command out of the default sync-blocking behaviour.
     *
     * Equivalent to `.setBlocking(false)`. The builder's destructor will enqueue the
     * command and return immediately, without draining the queue. Use for the auto-path
     * planner case where the caller ticks `loop()` and polls `Drive.isIdle()` to know
     * when to issue the next command — the planner explicitly does NOT want each verb
     * call to block its own tick.
     *
     * @return Reference to this builder for chaining.
     * @note The default is sync (`_blocking = true`). Hand-written mission scripts should
     *       almost never need `.async()` — it's a tool for the auto-path planner.
     */
    TurnDegBuilder& async()
    {
        _blocking = false;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-enqueue. Pushes a `Kind::TURN_DEG` command onto the
     *        Drive queue.
     *
     * The turn dispatches later from `Drive.update()`. Blocking variant drains the queue
     * before and after enqueue (third allowed `delay()` site).
     */
    ~TurnDegBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;

        const int16_t resolvedPower = _resolvePower();

        DriveCommand cmd;
        cmd.kind     = DriveCommand::Kind::TURN_DEG;
        cmd.deg      = _deg;
        cmd.power    = resolvedPower;
        cmd.turnMode = _mode;
        cmd.brake    = _brake;

        if (_blocking)
        {
            _drive._drainBlocking();
            _drive._enqueue(cmd);
            _drive._drainBlocking();
        }
        else
        {
            _drive._enqueue(cmd);
        }
    }

private:
    /**
     * @brief Resolve the final PWM power: explicit `setPower` wins, then `setPreset`, then
     *        `Config::TURN_CRUISE_PWM` as the documented default.
     */
    int16_t _resolvePower() const
    {
        if (_powerExplicit)
        {
            return _power;
        }
        if (_presetSet)
        {
            return presetToPwm(_preset);
        }
        return Config::TURN_CRUISE_PWM;
    }

    DriveClass&         _drive;
    int16_t             _deg;
    int16_t             _power         = Config::TURN_CRUISE_PWM;
    bool                _powerExplicit = false;
    Config::SpeedPreset _preset        = Config::SpeedPreset::NORMAL;
    bool                _presetSet     = false;
    uint8_t             _mode          = Config::TURN_MODE;
    bool                _brake         = Config::TURN_BRAKE;
    bool                _blocking      = true;    ///< default sync — destructor blocks until command completes.
                                                  ///< Auto-planner opts out per call via `.async()`.
    bool                _executed      = false;
};

/**
 * @brief Fluent builder for `Drive.turnToYaw(yaw)` — absolute-yaw turn.
 *
 * **Sync-blocking by default; chain `.async()` to opt out.** Auto-executes on destruction.
 * Defaults match `TurnDegBuilder`.
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
     * @brief Select the wheel pattern used by the custom TurnGyro state machine
     *        (`_updateTurnProfile` in Drive.cpp; replaces MiniR4.DriveDC.TurnGyro).
     *
     * @param m Wheel-pattern byte. Encoding:
     *          - `0` = COUNTER_ROT — both wheels spin opposite directions, chassis pivots around its center.
     *          - `1` = PIVOT_LEFT — left wheel braked; right wheel drives forward; chassis pivots around the LEFT wheel.
     *          - `2` = PIVOT_RIGHT — right wheel braked; left wheel drives forward; chassis pivots around the RIGHT wheel.
     *
     * @note This is the NEW encoding (introduced by the custom TurnGyro rewrite — see DECISIONS.md / the
     *       2026-05-17 plan). It is intentionally NOT the legacy `MiniR4.DriveDC.TurnGyro` encoding
     *       (`0=pivot, 1=counter-rot`); the new state machine owns wheel dispatch directly via `Move(L, R)`
     *       and needed a third value for the pivot-side selection. PathTracker maps `Routes::TurnMode::*`
     *       to this byte via `_turnModeToByte` in PathTracker.cpp.
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
     * @param b `true` = sync (default); `false` = async.
     * @return `*this` for chaining.
     * @warning Blocking variants MUST NOT be called from inside any module's `update()`.
     * @note The default is now `true` (sync). Prefer `.async()` for the auto-path-planner
     *       opt-out — it documents intent more clearly than `.setBlocking(false)`.
     */
    TurnToYawBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Opt this command out of the default sync-blocking behaviour.
     *
     * Equivalent to `.setBlocking(false)`. The builder's destructor will enqueue the
     * command and return immediately, without draining the queue. Use for the auto-path
     * planner case where the caller ticks `loop()` and polls `Drive.isIdle()` to know
     * when to issue the next command — the planner explicitly does NOT want each verb
     * call to block its own tick.
     *
     * @return Reference to this builder for chaining.
     * @note The default is sync (`_blocking = true`). Hand-written mission scripts should
     *       almost never need `.async()` — it's a tool for the auto-path planner.
     */
    TurnToYawBuilder& async()
    {
        _blocking = false;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-enqueue. Pushes a `Kind::TURN_TO_YAW` command onto the
     *        Drive queue.
     *
     * The turn dispatches later from `Drive.update()`. Blocking variant drains the queue
     * before and after enqueue (third allowed `delay()` site).
     */
    ~TurnToYawBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;

        const int16_t resolvedPower = _resolvePower();

        DriveCommand cmd;
        cmd.kind     = DriveCommand::Kind::TURN_TO_YAW;
        cmd.yaw      = _yaw;
        cmd.power    = resolvedPower;
        cmd.turnMode = _mode;
        cmd.brake    = _brake;

        if (_blocking)
        {
            _drive._drainBlocking();
            _drive._enqueue(cmd);
            _drive._drainBlocking();
        }
        else
        {
            _drive._enqueue(cmd);
        }
    }

private:
    /**
     * @brief Resolve the final PWM power: explicit `setPower` wins, then `setPreset`, then
     *        `Config::TURN_CRUISE_PWM`.
     */
    int16_t _resolvePower() const
    {
        if (_powerExplicit)
        {
            return _power;
        }
        if (_presetSet)
        {
            return presetToPwm(_preset);
        }
        return Config::TURN_CRUISE_PWM;
    }

    DriveClass&         _drive;
    int16_t             _yaw;
    int16_t             _power         = Config::TURN_CRUISE_PWM;
    bool                _powerExplicit = false;
    Config::SpeedPreset _preset        = Config::SpeedPreset::NORMAL;
    bool                _presetSet     = false;
    uint8_t             _mode          = Config::TURN_MODE;
    bool                _brake         = Config::TURN_BRAKE;
    bool                _blocking      = true;    ///< default sync — destructor blocks until command completes.
                                                  ///< Auto-planner opts out per call via `.async()`.
    bool                _executed      = false;
};

/**
 * @brief Fluent builder for `Drive.stopAtLine(expectedCm)` — drive-until-line seek (ADR-027).
 *
 * **Sync-blocking by default; chain `.async()` to opt out.** Auto-executes on destruction.
 * Drives straight toward a line expected near `expectedCm`, arms a `T_OR_CROSS` detector only
 * once within `armWithin` cm of the endpoint, overshoots so the wheel axle lands on the line,
 * then active-brakes — or faults `Fault::LINE_NOT_FOUND` if no line is seen by the max distance
 * (query `Drive.isLineFound()`). Defaults: `DriveMode::SYNC`, power `Config::SPEED_NORMAL`,
 * brake `true`, blocking `true`, forward (front board), heading = `Drive`'s current `_heldYaw`
 * (only consulted in `DriveMode::GYRO`), `armWithin = Config::STOP_AT_LINE_ARM_WINDOW_CM`,
 * `maxDistance = expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM`,
 * `overshoot = Config::LINE_SENSOR_AXLE_OFFSET_CM`. The SYNC default mirrors the project-wide
 * move default (see DECISIONS.md). Unlike `driveDist` there is NO `tShape` knob — the seek is a
 * continuous event-terminated move whose stop point is unknown until detection.
 */
class [[nodiscard]] StopAtLineBuilder
{
public:
    /**
     * @brief Construct a builder bound to the singleton `Drive` for endpoint @p expectedCm.
     * @param drive      Reference to the singleton `Drive` (only `Drive.stopAtLine` calls this).
     * @param expectedCm Nominal distance to the expected crossing line, in centimetres.
     */
    StopAtLineBuilder(DriveClass& drive, float expectedCm) : _drive(drive), _expectedCm(expectedCm)
    {
    }

    StopAtLineBuilder(const StopAtLineBuilder&)            = delete;
    StopAtLineBuilder& operator=(const StopAtLineBuilder&) = delete;
    StopAtLineBuilder& operator=(StopAtLineBuilder&&)      = delete;

    /**
     * @brief Move constructor — transfers execution responsibility to the new instance.
     *
     * Required so the verb method (`Drive.stopAtLine(expectedCm)`) can return a builder by value
     * on compilers without mandatory copy elision. The moved-from instance is marked `_executed`
     * so its destructor is a no-op.
     */
    StopAtLineBuilder(StopAtLineBuilder&& other) noexcept
        : _drive(other._drive),
          _expectedCm(other._expectedCm),
          _power(other._power),
          _powerExplicit(other._powerExplicit),
          _preset(other._preset),
          _presetSet(other._presetSet),
          _mode(other._mode),
          _brake(other._brake),
          _blocking(other._blocking),
          _heading(other._heading),
          _headingSet(other._headingSet),
          _armWithin(other._armWithin),
          _maxDistance(other._maxDistance),
          _maxDistanceSet(other._maxDistanceSet),
          _overshoot(other._overshoot),
          _reverse(other._reverse),
          _executed(other._executed)
    {
        other._executed = true;
    }

    /**
     * @brief Override the PWM power explicitly. Wins over any preset.
     * @param p PWM target in `[-100, 100]`.
     * @return `*this` for chaining.
     * @note The seek carries direction via `.setReverse()`, not the power sign — pass a positive
     *       magnitude; the state machine applies the travel direction.
     */
    StopAtLineBuilder& setPower(int16_t p)
    {
        _power         = p;
        _powerExplicit = true;
        return *this;
    }

    /**
     * @brief Use a `Config::SpeedPreset` for the seek cruise PWM.
     * @param p Preset selector — resolved via `presetToPwm` (WROMath.h) at execute-time.
     * @return `*this` for chaining.
     * @note Ignored if `setPower` was also called on the same builder.
     */
    StopAtLineBuilder& setPreset(Config::SpeedPreset p)
    {
        _preset    = p;
        _presetSet = true;
        return *this;
    }

    /**
     * @brief Choose which underlying continuous primitive drives the seek each tick.
     * @param m `RAW` (open-loop), `SYNC` (encoder-rate sync PID), or `GYRO` (yaw-lock).
     * @return `*this` for chaining.
     * @note Default `SYNC`. `GYRO` holds `setHeading(...)` (or the current `_heldYaw`).
     */
    StopAtLineBuilder& setMode(DriveMode m)
    {
        _mode = m;
        return *this;
    }

    /**
     * @brief Override the heading held during a `DriveMode::GYRO` seek.
     * @param yaw Absolute yaw target in degrees.
     * @return `*this` for chaining.
     * @note Only consulted when the resolved mode is `GYRO`. If never set, the seek uses the
     *       current `Drive` `_heldYaw`.
     */
    StopAtLineBuilder& setHeading(int16_t yaw)
    {
        _heading    = yaw;
        _headingSet = true;
        return *this;
    }

    /**
     * @brief Override the arm-window half-length: detection arms within this many cm of the
     *        expected endpoint.
     * @param cm Arm window in centimetres. Default `Config::STOP_AT_LINE_ARM_WINDOW_CM`.
     * @return `*this` for chaining.
     * @note Size this LARGER than the worst-case dead-reckoning under-travel over the hop — if
     *       the encoder under-estimates by more than this, the chassis crosses the real line
     *       while still disarmed and misses it (→ runs to max → `LINE_NOT_FOUND`). Generous, not
     *       tight. See `Config::STOP_AT_LINE_ARM_WINDOW_CM` and the spec.
     */
    StopAtLineBuilder& setArmWithin(float cm)
    {
        _armWithin = cm;
        return *this;
    }

    /**
     * @brief Override the absolute max travel before faulting `LINE_NOT_FOUND`.
     * @param cm Max search distance in centimetres. Default
     *           `expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM`.
     * @return `*this` for chaining.
     */
    StopAtLineBuilder& setMaxDistance(float cm)
    {
        _maxDistance    = cm;
        _maxDistanceSet = true;
        return *this;
    }

    /**
     * @brief Override the post-detect roll so the wheel axle lands on the line.
     * @param cm Overshoot in centimetres. Default `Config::LINE_SENSOR_AXLE_OFFSET_CM`.
     * @return `*this` for chaining.
     */
    StopAtLineBuilder& setOvershoot(float cm)
    {
        _overshoot = cm;
        return *this;
    }

    /**
     * @brief Back into the line via the rear board instead of driving forward.
     * @param r `true` = reverse (rear board); `false` (default) = forward (front board).
     * @return `*this` for chaining.
     * @note Requires `Config::LINE_BOARD_COUNT == 2`. On a 1-board chassis the reverse verb is
     *       rejected at dispatch (Drive faults `LINE_NOT_FOUND`) rather than reading the
     *       wrong-facing board. In reverse, `expectedCm`, `armWithin`, and `overshoot` are all
     *       measured along the backward travel direction.
     * @note `@untested on hardware` until the rear board is wired (ADR-024).
     */
    StopAtLineBuilder& setReverse(bool r)
    {
        _reverse = r;
        return *this;
    }

    /**
     * @brief Apply (or release) the active brake at the end of the overshoot roll.
     * @param b `true` = brake; `false` = coast.
     * @return `*this` for chaining.
     */
    StopAtLineBuilder& setBrake(bool b)
    {
        _brake = b;
        return *this;
    }

    /**
     * @brief Block the caller until the seek completes (detection + overshoot, or fault).
     * @param b `true` = sync (default — spins on the registered tick callback); `false` = async.
     * @return `*this` for chaining.
     * @warning Blocking variants MUST NOT be called from inside any module's `update()`.
     * @note Prefer `.async()` for the auto-path-planner opt-out — it documents intent more
     *       clearly than `.setBlocking(false)`.
     */
    StopAtLineBuilder& setBlocking(bool b)
    {
        _blocking = b;
        return *this;
    }

    /**
     * @brief Opt this seek out of the default sync-blocking behaviour.
     *
     * Equivalent to `.setBlocking(false)`. The builder's destructor enqueues the command and
     * returns immediately. Use for the auto-path planner / PathTracker case where the caller
     * ticks `loop()` and polls `Drive.isIdle()` / `Drive.isLineFound()` to know the outcome.
     *
     * @return Reference to this builder for chaining.
     * @note The default is sync (`_blocking = true`).
     */
    StopAtLineBuilder& async()
    {
        _blocking = false;
        return *this;
    }

    /**
     * @brief Destructor — RAII auto-enqueue. Resolves the final knobs and pushes a
     *        `Kind::STOP_AT_LINE` command onto the Drive queue.
     *
     * The seek dispatches later from `Drive.update()` into `_updateStopAtLine()` (NOT
     * `_updateProfile()` — see the verb doxygen). Unset knobs resolve to their Config defaults:
     * `armWithin = Config::STOP_AT_LINE_ARM_WINDOW_CM`,
     * `maxCm = _expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM`,
     * `overshoot = Config::LINE_SENSOR_AXLE_OFFSET_CM`.
     *
     * @note Negative `expectedCm` is rejected at this boundary (reverse direction is selected
     *       via `.setReverse()`, not a negative distance) — mirrors `DriveDistBuilder` (F4).
     * @note When `_blocking == true`, the destructor drains the queue before AND after the
     *       enqueue (third allowed `delay()` site — see `_drainBlocking` doxygen).
     */
    ~StopAtLineBuilder()
    {
        if (_executed)
        {
            return;
        }
        _executed = true;

        // Reject negative distance at the public-API boundary — reverse direction is expressed
        // via `.setReverse()`, never a negative distance. Mirrors DriveDistBuilder (F4).
        if (_expectedCm < 0.0f)
        {
            Serial.println(F("[Drive] stopAtLine: negative expectedCm rejected"));
            return;
        }

        // Resolve cruise — explicit setPower wins, then setPreset, then SPEED_NORMAL.
        const int16_t cruise = _resolvePower();

        DriveCommand cmd;
        cmd.kind       = DriveCommand::Kind::STOP_AT_LINE;
        cmd.mode       = _mode;
        cmd.power      = cruise;
        cmd.brake      = _brake;
        cmd.heading    = _headingSet ? _heading : _drive._heldYaw;
        cmd.expectedCm = _expectedCm;
        cmd.armWithin  = _armWithin;
        cmd.maxCm      = _maxDistanceSet
                           ? _maxDistance
                           : (_expectedCm + Config::STOP_AT_LINE_MAX_MARGIN_CM);
        cmd.overshoot  = _overshoot;
        cmd.reverse    = _reverse;

        if (_blocking)
        {
            _drive._drainBlocking();
            _drive._enqueue(cmd);
            _drive._drainBlocking();
        }
        else
        {
            _drive._enqueue(cmd);
        }
    }

private:
    /**
     * @brief Resolve the final cruise PWM: explicit `setPower` wins, then `setPreset`, then
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
            return presetToPwm(_preset);
        }
        return Config::SPEED_NORMAL;
    }

    DriveClass&         _drive;
    float               _expectedCm;
    int16_t             _power          = Config::SPEED_NORMAL;
    bool                _powerExplicit  = false;
    Config::SpeedPreset _preset         = Config::SpeedPreset::NORMAL;
    bool                _presetSet      = false;
    DriveMode           _mode           = DriveMode::SYNC;   ///< default SYNC — see DECISIONS.md.
    bool                _brake          = true;
    bool                _blocking       = true;    ///< default sync — destructor blocks until command completes.
                                                   ///< Auto-planner / PathTracker opt out per call via `.async()`.
    int16_t             _heading        = 0;
    bool                _headingSet     = false;
    float               _armWithin      = Config::STOP_AT_LINE_ARM_WINDOW_CM;
    float               _maxDistance    = 0.0f;    ///< Resolved to `expectedCm + margin` if unset.
    bool                _maxDistanceSet = false;
    float               _overshoot      = Config::LINE_SENSOR_AXLE_OFFSET_CM;
    bool                _reverse        = false;
    bool                _executed       = false;
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

inline StopAtLineBuilder DriveClass::stopAtLine(float expectedCm)
{
    return StopAtLineBuilder(*this, expectedCm);
}

#endif    // DRIVE_H
