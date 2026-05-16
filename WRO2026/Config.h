#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Single source of truth for every tunable constant; modules MUST source values from
 *        here, never hardcode.
 *
 * Every downstream subsystem (Drive, Manipulator, LineFollower, Intersection, EventTrace, etc.)
 * pulls its tuning, hardware-mapping and behavioural constants from this namespace. Changing a
 * value here is intended to be the only edit required to retune the robot — never duplicate any
 * of these constants in a `.cpp`/`.ino` or as a function-local literal.
 *
 * @note PID gains and motor specs target TT-motors (the chassis we are building). Values are
 *       sourced from MatrixMiniR4's `MiniR4DriveDC.h` TT defaults (`setMoveSyncPID` /
 *       `setMoveGyroPID` / `setTurnGyroPID` docstrings). If we ever switch motor families,
 *       re-tune in this section, not via a compile-time flag.
 */
namespace Config
{
    // ---------------------------------------------------------------------------------------
    // Battery
    // ---------------------------------------------------------------------------------------

    /** @brief Battery cell count passed to `MiniR4.PWR.setBattCell()`. MUST match the pack. */
    constexpr uint8_t BATT_CELLS = 2;

    // ---------------------------------------------------------------------------------------
    // Motor IDs and direction
    // ---------------------------------------------------------------------------------------

    /** @brief Motor controller ID for the left drive wheel (M2 per WRO chassis convention). */
    constexpr uint8_t MOTOR_LEFT_ID = 2;     // M2

    /** @brief Motor controller ID for the right drive wheel (M3 per WRO chassis convention). */
    constexpr uint8_t MOTOR_RIGHT_ID = 3;    // M3

    /** @brief Reverse the left motor's positive-power direction at the driver level. */
    constexpr bool MOTOR_LEFT_REVERSE = true;

    /** @brief Reverse the right motor's positive-power direction at the driver level. */
    constexpr bool MOTOR_RIGHT_REVERSE = false;

    // ---------------------------------------------------------------------------------------
    // Motor profile + PID gains (TT-motor)
    // ---------------------------------------------------------------------------------------
    //
    // PID gains taken from MatrixMiniR4 `MiniR4DriveDC.h` TT defaults:
    //   - setMoveSyncPID  TT: Kp=0.02,  Ki=0.00, Kd=0.04
    //   - setMoveGyroPID  TT: Kp=6.01,  Ki=0.00, Kd=2.15
    //   - setTurnGyroPID  TT: Kp=22.75, Ki=0.08, Kd=0.25
    //

    /** @brief Encoder pulses per output-shaft revolution for the TT motor. */
    constexpr uint16_t MOTOR_PPR = 545;

    /** @brief Nominal no-load RPM for the TT motor at full power. */
    constexpr uint16_t MOTOR_RPM = 200;

    /** @brief MoveSync (left/right encoder match) Kp. */
    constexpr float MOVESYNC_KP = 0.02f;
    /** @brief MoveSync Ki. */
    constexpr float MOVESYNC_KI = 0.0f;
    /** @brief MoveSync Kd. */
    constexpr float MOVESYNC_KD = 0.04f;

    /** @brief MoveGyro (straight-line yaw lock) Kp. */
    constexpr float MOVEGYRO_KP = 6.01f;
    /** @brief MoveGyro Ki. */
    constexpr float MOVEGYRO_KI = 0.0f;
    /** @brief MoveGyro Kd. */
    constexpr float MOVEGYRO_KD = 2.15f;

    /** @brief TurnGyro (in-place rotation) Kp. */
    constexpr float TURNGYRO_KP = 22.75f;
    /** @brief TurnGyro Ki. */
    constexpr float TURNGYRO_KI = 0.08f;
    /** @brief TurnGyro Kd. */
    constexpr float TURNGYRO_KD = 0.25f;

    /** @brief Line-follower outer-loop Kp. @note TODO: tune on real strip. */
    constexpr float LINE_KP = 8.0f;
    /** @brief Line-follower outer-loop Ki. @note TODO: tune on real strip. */
    constexpr float LINE_KI = 0.0f;
    /** @brief Line-follower outer-loop Kd. @note TODO: tune on real strip. */
    constexpr float LINE_KD = 4.0f;

    // ---------------------------------------------------------------------------------------
    // Geometry (TODO: measure on real chassis)
    // ---------------------------------------------------------------------------------------

    /** @brief Drive-wheel diameter in millimetres. @note TODO: measure on real chassis. */
    constexpr float WHEEL_DIAM_MM = 60.0f;

    /** @brief Wheel-to-wheel track width in millimetres. @note TODO: measure on real chassis. */
    constexpr float TRACK_MM = 150.0f;

    // ---------------------------------------------------------------------------------------
    // Speed presets (consumed by the fluent Drive / LineFollower APIs)
    // ---------------------------------------------------------------------------------------
    //
    // The fluent chassis API takes `SpeedPreset preset = SpeedPreset::NORMAL` as its default
    // power source. Each preset maps to a PWM in the [-100, 100] DriveDC range below. Callers
    // that need a non-preset value can still override via `.setPower(int16_t)` on the builder.
    //

    /**
     * @brief Coarse speed selector for chassis movement primitives.
     *
     * Used by every fluent verb that takes a power: Drive.driveDist / driveTime /
     * forwardGyro / turnDeg / turnToYaw, plus LineFollower.setPreset for cruise. Default is
     * `NORMAL`; pass `SLOW` for intersection approaches and tight corners, `FAST` for long
     * straights.
     */
    enum class SpeedPreset : uint8_t
    {
        SLOW,
        NORMAL,
        FAST
    };

    /** @brief PWM target for `SpeedPreset::SLOW` (intersections, tight corners). */
    constexpr int16_t SPEED_SLOW = 30;

    /** @brief PWM target for `SpeedPreset::NORMAL` (default cruise / general moves). */
    constexpr int16_t SPEED_NORMAL = 50;

    /** @brief PWM target for `SpeedPreset::FAST` (long straights, low-precision moves). */
    constexpr int16_t SPEED_FAST = 80;

    // =============================================================================
    // PWM bounds (saturation cap + motor deadband floor).
    //
    // Two physical properties of the TT motors:
    //   * Saturation: PWM above ~95% gives no headroom for closed-loop corrections
    //     — anything beyond gets clipped by the lower MCU.
    //   * Deadband: PWM below ~20% can't break static friction; the motor stays
    //     still even though a non-zero command was issued. Any P+D correction in
    //     the (0, 20) range would be silently lost without compensation.
    //
    // `deadbandSnap()` (defined in WROMath.h) takes a raw PWM and returns a value
    // in {0} U [PWM_MIN_MAGNITUDE, PWM_MAX] (with mirrored negative range): zero
    // stays zero, non-zero values below the floor snap UP to the floor magnitude,
    // values above the cap clamp DOWN to the cap. Apply this at every actuator
    // boundary where a computed PWM hits the lower MCU.
    // =============================================================================

    /** @brief Saturation cap — any |PWM| above this gets clipped to this magnitude. */
    constexpr int16_t PWM_MAX = 95;

    /**
     * @brief Motor deadband floor — non-zero |PWM| below this can't break stiction.
     *
     * `deadbandSnap()` snaps non-zero commands UP to this floor (preserving sign)
     * so P+D corrections always result in motion. Zero commands pass through —
     * the caller intended to stop, deadband is irrelevant.
     */
    constexpr int16_t PWM_MIN_MAGNITUDE = 20;

    // ---------------------------------------------------------------------------------------
    // Cruise / line following
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Forward cruise power for line following, in the [-100, 100] DriveDC range.
     *
     * Kept as a named constant for back-compat with code that still references it directly;
     * new callers should prefer `LineFollower.setPreset(SpeedPreset::NORMAL)` which resolves
     * to `SPEED_NORMAL` (currently equal to this value).
     */
    constexpr int16_t CRUISE_SPEED = SPEED_NORMAL;

    /** @brief Maximum left/right correction added to / subtracted from cruise speed. */
    constexpr int16_t LINE_MAX_CORR = 40;

    /** @brief Threshold passed to `MXLineTracer.setThreshold()` (0..100). */
    constexpr uint8_t LINE_THRESHOLD = 60;

    // ---------------------------------------------------------------------------------------
    // MXLineTracer wiring
    // ---------------------------------------------------------------------------------------

    /** @brief External I2C port index (I2C1 = first external port on Wire1). */
    constexpr uint8_t LINE_I2C_PORT = 1;

    // ---------------------------------------------------------------------------------------
    // Turn defaults
    // ---------------------------------------------------------------------------------------

    /** @brief Default rotation power for `DriveDC.TurnGyro` calls, in [-100, 100]. */
    constexpr int16_t TURN_POWER = 50;

    /** @brief Turn motor mode: 0 = single-motor pivot, 1 = two-motor counter-rotation. */
    constexpr uint8_t TURN_MODE = 0;

    /** @brief Apply active brake at end of turn (vs. coast). */
    constexpr bool TURN_BRAKE = true;

    // ---------------------------------------------------------------------------------------
    // Lost-line behavior
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Action taken by the LineFollower when the strip is lost beneath every channel.
     *
     * - `STOP`  — actively brake to zero and surface the lost-line event.
     * - `COAST` — release power and let the chassis roll to a stop.
     * - `CRAWL` — keep cruising at a reduced speed in the last-known direction to try to reacquire.
     */
    enum class LostLineAction : uint8_t
    {
        STOP,
        COAST,
        CRAWL
    };

    /** @brief Default lost-line action used by the LineFollower. */
    constexpr LostLineAction LOST_LINE_ACTION = LostLineAction::STOP;

    /**
     * @brief Hard ceiling for the blocking `LineFollower.waitUntilDistanceReached()` helper.
     *
     * The new sync helper is intentionally a thin orchestration wrapper around repeated
     * `LineFollower.update()` ticks. Without a deadline, a bad sensor read, wrong target, or
     * a caller accidentally handing motor ownership back to a bounded `Drive` move would hang
     * the sketch forever. On expiry the helper disables the follower with active brake and
     * returns `false`.
     */
    constexpr uint32_t LINE_FOLLOW_WAIT_TIMEOUT_MS = 10000;

    // ---------------------------------------------------------------------------------------
    // OLED event trace
    // ---------------------------------------------------------------------------------------

    /** @brief Number of recent events kept in the EventTrace ring buffer for the OLED. */
    constexpr uint8_t EVENT_TRACE_CAPACITY = 4;

    // =========================================================================================
    // Motor-bias calibration (Strategy B from Phase 1.5 chassis-API research).
    //
    // TT-motors of the same model can vary 10-20% in RPM at the same PWM. Our
    // raw-PWM Drive paths (setSpeed, driveDist, driveTime — all routed through
    // MatrixMiniR4 Move/MoveDegs/MoveTime, which are open-loop) are the ones
    // affected; Sync/Gyro variants close the loop on the STM32 and don't need it.
    //
    // Workflow: at first hardware bring-up, call Drive.calibrateMotorBias()
    // once with the wheels free of the ground. The routine prints two scalars
    // to Serial; copy them into MOTOR_BIAS_L / MOTOR_BIAS_R below. From then on,
    // Drive.begin() applies them automatically via setMotorBias().
    //
    // 1.0f = no correction (default; safe before calibration).
    //
    // See DECISIONS.md "Motor-bias calibration (Strategy B)" for full rationale,
    // including the on-shelf Strategy C (R4-side encoder-rate PID) we'd add if
    // B alone is not enough on the actual track.
    // =========================================================================================

    /** @brief Static manufacturer-bias scalar applied to the LEFT motor in raw-PWM paths. */
    static constexpr float MOTOR_BIAS_L = 1.0f;

    /** @brief Static manufacturer-bias scalar applied to the RIGHT motor in raw-PWM paths. */
    static constexpr float MOTOR_BIAS_R = 1.0f;

    // Calibration-routine defaults. The routine spins both motors at this power
    // for this duration, then computes scalars from the resulting encoder deltas.
    // 50% power for 2 s gives ~1-2k encoder counts on TT motors — plenty of signal,
    // not enough to spin the chassis off a bench in wheels-up mode.

    /** @brief PWM power used by `Drive.calibrateMotorBias()` during the wheels-free spin. */
    static constexpr int16_t MOTOR_BIAS_CALIB_POWER = 50;

    /** @brief Spin duration in milliseconds used by `Drive.calibrateMotorBias()`. */
    static constexpr uint32_t MOTOR_BIAS_CALIB_DURATION_MS = 2000;

    // =============================================================================
    // Trapezoidal velocity profile (`.tShape()` modifier on Drive / LineFollower).
    //
    // Acceleration-based: speed ramps at TSHAPE_ACCEL_PWM_PER_CM (PWM units gained
    // per cm of encoder travel) until the cruise target is reached, then plateaus,
    // then ramps back down at the same rate. Per-side ramp distance per move is
    // implicit: (cruise - endpointSpeed) / accel, clamped to [MIN_CM, MAX_CM] and
    // to half of the total move length. If accel- and decel-distances together
    // exceed total, the cruise plateau collapses (triangle fallback) and the peak
    // speed is reduced so the integral still matches the requested distance.
    //
    // Endpoint-default convention: when a move is configured `.setBrake(false)`,
    // the default `endSpeed` resolves to `cruise` (not 0) — the assumption is that
    // a no-brake move is chaining into a follow-up. Caller can override with an
    // explicit `.setEndSpeed(0)` if they really want a coast-to-stop.
    //
    // See DECISIONS.md "Trapezoidal velocity profile" for full rationale.
    // =============================================================================

    /** @brief PWM units gained per cm of travel during the ramp phase. */
    constexpr float TSHAPE_ACCEL_PWM_PER_CM = 5.0f;

    /** @brief Floor on the per-side ramp distance — never shorter than this (cm). */
    constexpr float TSHAPE_RAMP_MIN_CM = 2.0f;

    /** @brief Ceiling on the per-side ramp distance — never longer than this (cm). */
    constexpr float TSHAPE_RAMP_MAX_CM = 20.0f;

    /**
     * @brief Default starting speed for a profiled move (raw PWM).
     *
     * Set to `PWM_MIN_MAGNITUDE` (the motor deadband floor) rather than 0 so the
     * very first ramp tick emits a PWM the motor can actually act on. Starting at
     * 0 would waste the first ~20% of the ramp distance with the chassis sitting
     * still while the speed setpoint climbs through the dead zone.
     *
     * Override per-call with `.setStartSpeed(0)` if a true rest-start is desired
     * (e.g. for very long ramps where the lost first centimetres don't matter).
     */
    constexpr int16_t TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE;

    /**
     * @brief Default ending speed for a profiled move (raw PWM). 0 = end at rest.
     *
     * Stays at 0 (no symmetric deadband floor) by design: the motor naturally
     * stalls in the last bit when the ramp passes through the dead zone, then
     * `stop()` engages the brake immediately — the chassis comes to rest cleanly
     * without needing a ramp-down floor.
     */
    constexpr int16_t TSHAPE_END_SPEED = 0;

    /**
     * @brief Hysteresis: if `|cruise - endpointSpeed| < TSHAPE_NO_RAMP_EPSILON_PWM`,
     *        skip the ramp on that side entirely.
     *
     * Prevents the MIN_CM floor from forcing a tiny pointless ramp on chained moves
     * where startSpeed or endSpeed is already at (or very near) cruise.
     */
    constexpr int16_t TSHAPE_NO_RAMP_EPSILON_PWM = 2;

    /** @brief Tick-rate target for the profile state machine — only used as a comment marker. */
    // Profile is driven from Drive.update() at whatever rate the main loop ticks (~50-200 Hz typical).

    /**
     * @brief Minimum move length below which `.tShape()` silently no-ops.
     *
     * Below this threshold, the half-of-total clamp on per-side ramp distances would force
     * the effective acceleration steeper than `TSHAPE_ACCEL_PWM_PER_CM`, breaking the
     * constant-acceleration contract. Cleaner to fall back to a non-profiled constant-cruise
     * move below the threshold than to deliver an inconsistent ramp.
     *
     * @note When tShape no-ops, the move still runs — just without the ramp. Caller does not
     *       need to special-case short moves.
     */
    constexpr float TSHAPE_MIN_MOVE_CM = 5.0f;

    /** @brief Same threshold as TSHAPE_MIN_MOVE_CM but for time-bounded moves (seconds). */
    constexpr float TSHAPE_MIN_MOVE_SEC = 0.4f;

    // =============================================================================
    // Drive command queue (sequential dispatch).
    //
    // Drive's fluent builder destructors enqueue commands instead of dispatching
    // immediately. `Drive.update()` pops the next command when the current bounded
    // move + profile state machine are both idle. This makes source-order sequencing
    // of multiple moves work without caller-side polling — the future auto-path
    // planner can issue dozens of moves and they execute in order:
    //
    //   Drive.driveDist(30).tShape();
    //   Drive.driveDist(20).tShape();
    //   Drive.turnDeg(90);
    //
    // Blocking variants (`.setBlocking(true)`) bypass the queue: they drain existing
    // entries first, then dispatch synchronously.
    //
    // `Drive.stop()` clears the queue (abort semantics).
    //
    // See DECISIONS.md "Drive command queue" for full rationale.
    // =============================================================================

    /**
     * @brief Maximum number of pending commands the Drive queue can hold.
     *
     * 8 is comfortably more than a typical multi-segment maneuver and keeps the
     * struct array under 1 KB even with the largest command POD. Overflow drops
     * the new command and prints a Serial warning — that's a caller bug.
     */
    constexpr uint8_t DRIVE_QUEUE_CAPACITY = 8;

    /**
     * @brief Per-command worst-case ceiling used by `_drainBlocking` to bound its
     *        synchronous spin (code review F3).
     *
     * A typical 30 cm @ 50 PWM move completes in ~1 s; 10 s is a 10× safety factor
     * that still fits the 2-minute WRO run clock even if a full queue (8 cmds plus
     * the in-flight one) all hit the ceiling. The drain budget is computed as
     * `(qCount + 1) * DRIVE_DRAIN_TIMEOUT_MS_PER_CMD`. On expiry the drain stops the
     * chassis, clears the queue, and surfaces a Serial warning.
     */
    constexpr uint32_t DRIVE_DRAIN_TIMEOUT_MS_PER_CMD = 10000;

    // =============================================================================
    // Intersection debounce (code review F7).
    //
    // The MX line tracer can flicker between NONE, UNKNOWN, and a valid junction
    // type at the leading/trailing edges of a junction marker, under lighting
    // changes, or during fast motion. Without debounce a single physical
    // intersection can fire multiple software edges. With these knobs we require
    // a stable streak of valid samples to ARM and a stable streak of NONE samples
    // to RELEASE — a single noisy sample no longer crosses the threshold.
    //
    // At ~50-100 Hz cooperative tick, 2 ticks ≈ 20-40 ms ≈ ~0.5-1 cm at typical
    // line-follow speeds (15-30 cm/s). Junction markers are ~5-10 cm wide so 2
    // ticks is well within the dwell time but rejects single-sample flicker.
    // =============================================================================

    /**
     * @brief Consecutive valid-junction samples required to arm a rising edge.
     */
    constexpr uint8_t INTERSECTION_DEBOUNCE_TICKS = 2;

    /**
     * @brief Consecutive NONE samples required to release before re-arming.
     */
    constexpr uint8_t INTERSECTION_RELEASE_TICKS = 2;

    // =========================================================================================
    // PathTracker queue + drain timeout + heading tolerance (Phase 2 PR2).
    //
    // PathTracker.goTo() returns a fluent builder that auto-enqueues a goal on the FIFO. The
    // destructor sync-blocks by default (drains the queue via the registered cooperative tick
    // callback) so source-order chains like
    //
    //   PathTracker.goTo("C");
    //   PathTracker.goTo("G");
    //
    // execute one-after-the-other with no caller polling. Chain `.async()` on the builder to
    // opt out (auto-planner case where the caller ticks loop() and polls PathTracker.isIdle()).
    //
    // See docs/plans/2026-05-06-phase2-pr2-path-tracker-runtime.md and
    // user_fluent_api_preference.md for the rationale.
    // =========================================================================================

    /**
     * @brief Maximum number of pending goTo() entries the PathTracker queue can hold.
     *
     * Sized to comfortably exceed a typical multi-step demo or mission segment. Overflow
     * drops the new entry and prints a Serial warning — that is a caller bug.
     */
    constexpr uint8_t PATH_QUEUE_CAPACITY = 8;

    /**
     * @brief Per-goal worst-case ceiling used by `PathTracker._drainBlocking` to bound
     *        the synchronous spin (mirrors `DRIVE_DRAIN_TIMEOUT_MS_PER_CMD`).
     *
     * 60 s per goal accommodates a worst-case multi-edge traverse on the toy graph at slow
     * cruise. Drain budget is `(qCount + 1) * PATH_DRAIN_TIMEOUT_MS_PER_GOAL`. On expiry,
     * the chassis is stopped, the queue cleared, and a fault is latched.
     */
    constexpr uint32_t PATH_DRAIN_TIMEOUT_MS_PER_GOAL = 60000;

    /**
     * @brief Tolerance (degrees) below which `PathTracker` skips an absolute turn dispatch.
     *
     * Avoids a sub-degree twitch at every edge join when consecutive edges happen to be
     * colinear (e.g. mid-segment projection-node hand-off). 0.5 deg is well below the
     * `Drive.turnToYaw` resolvable range.
     */
    constexpr float NAV_TURN_TOLERANCE_DEG = 0.5f;

    // =========================================================================================
    // Mission DSL (PR3).
    //
    // Mission is a thin, fully-manual scheduler the operator hand-writes in
    // WRO2026.ino. Each `Mission.<verb>(...)` call enqueues one step; the
    // queue advances entirely from `Mission.update()` in the cooperative
    // tick. No automatic planning — that is intentional for PR3.
    //
    // On PathTracker fault, Mission halts the chassis, beeps five long
    // tones, flashes the RGB LEDs red, and waits for BTN_DOWN to clear.
    // =========================================================================================

    /** @brief Maximum number of pending Mission steps (no dynamic alloc). */
    constexpr uint8_t MAX_MISSION_STEPS = 64;

    /** @brief Tone frequency used for the halt-on-fault beep sequence. */
    constexpr uint16_t MISSION_FAULT_BEEP_HZ = 1000;

    /** @brief Duration of each on/off pulse in the halt-on-fault beep sequence. */
    constexpr uint16_t MISSION_FAULT_BEEP_MS = 500;

    /** @brief Number of beeps emitted before Mission awaits BTN_DOWN to clear. */
    constexpr uint8_t MISSION_FAULT_BEEPS = 5;

    /** @brief Interval at which EventTrace emits its `HB ...` status heartbeat. */
    constexpr uint32_t MISSION_HEARTBEAT_MS = 500;
}    // namespace Config

#endif    // CONFIG_H
