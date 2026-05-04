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
 *    NOT touch the bounded-task done flags.
 *  - **Distance-bounded** (`driveDist`, `driveDistSync`, `driveDistGyro`) — non-blocking,
 *    measured in cm. Sets `_moveDone = false`; cleared by `update()` once the underlying
 *    library reports the task done.
 *  - **Time-bounded** (`driveTime`, `driveTimeSync`, `driveTimeGyro`) — non-blocking,
 *    measured in seconds. Same done-flag semantics as distance-bounded.
 *  - **Turns** (`turnDeg`, `turnDegBlocking`, `turnToYaw`, `turnToYawBlocking`) — both
 *    relative and absolute. Async variants set `_turnDone = false`; blocking variants return
 *    only once the lower MCU reports task done and leave `_turnDone = true`.
 *
 * @note The underlying `MiniR4.DriveDC` lower-MCU API exposes ONE task-done bit, so this
 *       class collapses move-done and turn-done into a single shared "task done" signal
 *       inside `update()`. Application code MUST NOT issue a bounded move and a turn
 *       concurrently — they would race for the same flag. On a failed turn issue (non-zero
 *       rc from TurnGyro), `_turnDone` is set/left `true` immediately so the caller is
 *       unblocked even though no task is in flight on the lower MCU.
 *
 * @warning The `*Blocking` variants spin inside the underlying library's `delay(50)`-ish
 *          poll loop until the lower MCU reports the task is done. They MUST NOT be called
 *          from inside any module's `update()` method — `update()` must remain non-blocking
 *          and cooperative. Blocking entry points are reserved for command-time orchestration
 *          (mission step transitions, calibration routines, etc.).
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <MatrixMiniR4.h>

#include "Config.h"

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
    // Continuous primitives
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
    // Distance-bounded primitives (non-blocking)
    // -------------------------------------------------------------------------------------

    /**
     * @brief Drive a fixed distance at raw PWM power, non-blocking.
     *
     * @param power Drive power applied to BOTH wheels in `[-100, 100]`.
     * @param cm    Distance in centimetres (always positive — sign is carried by `power`).
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_moveDone = false`. Poll `isMoveDone()` (or rely on `update()` to clear the
     *       flag) before issuing the next bounded command.
     */
    uint8_t driveDist(int16_t power, float cm);

    /**
     * @brief Drive a fixed distance with MoveSync wheel synchronisation, non-blocking.
     *
     * @param power Drive power in `[-100, 100]`.
     * @param cm    Distance in centimetres.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_moveDone = false`.
     */
    uint8_t driveDistSync(int16_t power, float cm);

    /**
     * @brief Drive a fixed distance holding the held-yaw target, non-blocking.
     *
     * @param power Drive power in `[-100, 100]`.
     * @param cm    Distance in centimetres.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_moveDone = false`. Heading is locked to the current `_heldYaw`.
     */
    uint8_t driveDistGyro(int16_t power, float cm);

    // -------------------------------------------------------------------------------------
    // Time-bounded primitives (non-blocking)
    // -------------------------------------------------------------------------------------

    /**
     * @brief Drive at raw PWM power for a fixed time, non-blocking.
     *
     * @param power Drive power applied to BOTH wheels in `[-100, 100]`.
     * @param sec   Duration in seconds.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_moveDone = false`.
     */
    uint8_t driveTime(int16_t power, float sec);

    /**
     * @brief Drive with MoveSync at independent L/R powers for a fixed time, non-blocking.
     *
     * @param L   Left-motor power in `[-100, 100]`.
     * @param R   Right-motor power in `[-100, 100]`.
     * @param sec Duration in seconds.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_moveDone = false`.
     */
    uint8_t driveTimeSync(int16_t L, int16_t R, float sec);

    /**
     * @brief Drive holding the held-yaw target for a fixed time, non-blocking.
     *
     * @param power Drive power in `[-100, 100]`.
     * @param sec   Duration in seconds.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_moveDone = false`.
     */
    uint8_t driveTimeGyro(int16_t power, float sec);

    // -------------------------------------------------------------------------------------
    // Turns
    // -------------------------------------------------------------------------------------

    /**
     * @brief Turn by a relative yaw delta, non-blocking.
     *
     * Computes the absolute target as `current_yaw + deg`, normalises through
     * `_normalizeYaw`, and issues `MiniR4.DriveDC.TurnGyro` with `async = true`. Updates
     * `_heldYaw` to the post-turn target so subsequent `forwardGyro` / `*Gyro` primitives
     * lock onto the new heading.
     *
     * @param deg Relative yaw delta in degrees. Positive = whichever direction the lower MCU
     *            interprets as positive yaw (typically counter-clockwise).
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_turnDone = false`. Updates `_heldYaw` to the post-turn target.
     *       Yaw wrap-around: `_normalizeYaw` keeps targets in `(-360, +360)`; do not
     *       pre-modulo to `[-180, 180]` — the example sketch shows ±270 working fine.
     * @note If `TurnGyro` returns non-zero (turn failed to start), `_turnDone` is left/set
     *       to `true` and `_heldYaw` is NOT updated — the caller is unblocked and the
     *       previous yaw lock is preserved.
     */
    uint8_t turnDeg(int16_t deg);

    /**
     * @brief Turn by a relative yaw delta, blocking until complete.
     *
     * Same target math as `turnDeg`, but `async = false` so the underlying library spins on
     * `delay(50)` until the lower MCU signals completion. Leaves `_turnDone = true` on
     * return.
     *
     * @param deg Relative yaw delta in degrees.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Returns with `_turnDone = true` (the underlying library already blocked until the
     *       lower MCU reported done). Updates `_heldYaw` to the post-turn target.
     * @note If `TurnGyro` returns non-zero (turn failed to start), `_turnDone` is left/set
     *       to `true` and `_heldYaw` is NOT updated — the caller is unblocked and the
     *       previous yaw lock is preserved.
     * @warning DO NOT call from inside any `update()` — see class-level warning.
     */
    uint8_t turnDegBlocking(int16_t deg);

    /**
     * @brief Turn to an absolute yaw heading, non-blocking.
     *
     * @param yaw Absolute yaw target in degrees, normalised through `_normalizeYaw` to the
     *            `(-360, +360)` range that `TurnGyro` accepts.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Sets `_turnDone = false`. Updates `_heldYaw` to the post-turn target.
     *       Yaw wrap-around: `_normalizeYaw` keeps targets in `(-360, +360)`; do not
     *       pre-modulo to `[-180, 180]` — the example sketch shows ±270 working fine.
     * @note If `TurnGyro` returns non-zero (turn failed to start), `_turnDone` is left/set
     *       to `true` and `_heldYaw` is NOT updated — the caller is unblocked and the
     *       previous yaw lock is preserved.
     */
    uint8_t turnToYaw(int16_t yaw);

    /**
     * @brief Turn to an absolute yaw heading, blocking until complete.
     *
     * @param yaw Absolute yaw target in degrees.
     * @return `0x00` on success, lower-MCU error code otherwise.
     * @note Returns with `_turnDone = true`. Updates `_heldYaw` to the post-turn target.
     * @note If `TurnGyro` returns non-zero (turn failed to start), `_turnDone` is left/set
     *       to `true` and `_heldYaw` is NOT updated — the caller is unblocked and the
     *       previous yaw lock is preserved.
     * @warning DO NOT call from inside any `update()` — see class-level warning.
     */
    uint8_t turnToYawBlocking(int16_t yaw);

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

private:
    int16_t _heldYaw  = 0;       ///< Current yaw target used by `forwardGyro` and `*Gyro`.
    bool    _moveDone = true;    ///< Cleared on bounded move issue, set by `update()`.
    bool    _turnDone = true;    ///< Cleared on async turn issue, set by `update()`.

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
};

/**
 * @brief Single global instance — defined in Drive.cpp.
 */
extern DriveClass Drive;

#endif    // DRIVE_H
