#ifndef CONFIG_DRIVE_H
#define CONFIG_DRIVE_H

#include <stdint.h>

#include "ConfigGlobal.h"

/**
 * @brief Drivetrain configuration — reopens `namespace Config` per ADR-031 / Issue #39.
 *
 * Holds the motor IDs / direction flags, the per-chassis motor profile + PID gains
 * (dispatched via `ActiveMotor`), chassis geometry, speed presets, the custom TurnGyro
 * state-machine knobs, the stop-at-line seek defaults, the trapezoidal velocity profile,
 * the Drive command queue, and the motor-bias calibration scalars.
 *
 * Depends on `ConfigGlobal.h` for `PWM_MIN_MAGNITUDE` (`TSHAPE_START_SPEED` forwards it).
 * The drive-pair / CRAWL bounds static_asserts live in `ConfigChecks.h` (included last).
 *
 * @note PID gains and encoder specs are dispatched via `Config::ActiveMotor` — see the two
 *       structs `MatrixTT` and `LegoEV3` (each with `static constexpr` members) below.
 *       Values are sourced from MatrixMiniR4's `MiniR4DriveDC.h` PID docstrings
 *       (`setMoveSyncPID` / `setMoveGyroPID` / `setTurnGyroPID`) and the vendor example
 *       `MiniR4_DriveDC_Test.ino`. To swap chassis, edit only the `using ActiveMotor = ...;`
 *       line; the flat constants below forward to the active profile. See ADR-022.
 */
namespace Config
{
    // ---------------------------------------------------------------------------------------
    // Motor IDs and direction
    // ---------------------------------------------------------------------------------------
    //
    // ⚠️  DO NOT EDIT `MOTOR_LEFT_ID` / `MOTOR_RIGHT_ID` IN ISOLATION  ⚠️
    //
    // These two constants LOOK like config knobs but they are NOT independently tunable.
    // They are consumed in exactly one place — `MiniR4.DriveDC.begin(left_id, right_id, ...)`
    // at `Drive.cpp:40-41` — while EVERY OTHER reference to the drive pair in `Drive.cpp`
    // hardcodes `MiniR4.M2` and `MiniR4.M3` directly. Specifically:
    //
    //   - `Drive.cpp:35-38`   setReverse / setPPR_RPM are called on M2 / M3 by name.
    //   - `Drive.cpp:428-435` getEncL / getEncR read M2.getDegrees() / M3.getDegrees().
    //   - `Drive.cpp:491-492, 620, 944`  profile-state and bias-calibration encoder sampling
    //                                    also read M2 / M3 directly.
    //   - `Drive.h:397-406`   doxygen explicitly calls out the M2/M3 assumption.
    //
    // Changing only these two constants therefore puts the system into a contradictory
    // state: `DriveDC` would steer a different motor pair than the one the R4-side code
    // configures and reads. The "left encoder" would read a motor `DriveDC` doesn't own,
    // motor-bias calibration would target the wrong wheel, and reverse flags would be
    // applied to the wrong motor. The bug is silent — the chassis still moves, just
    // wrong, and distance gates drift unpredictably.
    //
    // If you actually need to re-wire the drive pair to different ports, you MUST also
    // edit every `MiniR4.M2` / `MiniR4.M3` reference in `Drive.cpp` AND update the M2/M3
    // doxygen in `Drive.h`. See PR #20 codex P1 review for the full failure analysis.
    // A cleaner long-term fix is a `motorById<ID>()` constexpr template helper that
    // dispatches on the ID — but that refactor is intentionally NOT done here; the
    // assumption that "the drive pair is M2/M3" is baked deep enough that the
    // safest current state is "this convention is the ground truth, enforced by
    // static_assert below."
    //
    // The static_assert lines (in ConfigChecks.h) are the load-bearing safety net: they
    // will FAIL THE BUILD if either constant is edited without the Drive.cpp / Drive.h
    // surgery above. Do not "fix" them by editing the assert — that defeats the entire point.
    // ---------------------------------------------------------------------------------------

    /** @brief Motor controller ID for the left drive wheel (M2 per WRO chassis convention).
     *  @warning See block comment above — do NOT edit without auditing Drive.cpp / Drive.h. */
    constexpr uint8_t MOTOR_LEFT_ID = 2;     // M2

    /** @brief Motor controller ID for the right drive wheel (M3 per WRO chassis convention).
     *  @warning See block comment above — do NOT edit without auditing Drive.cpp / Drive.h. */
    constexpr uint8_t MOTOR_RIGHT_ID = 3;    // M3

    /** @brief Reverse the left motor's positive-power direction at the driver level.
     *  @note Per-chassis MOUNTING fact, NOT tied to the motor model — `ActiveMotor` does not
     *        carry it. Contract (vendor direction check): `M2.setPower(+50)` must spin the
     *        left wheel TOWARD THE FRONT and `DriveDC.Move(30, 0)` must curve forward-right.
     *        Wrong flags leave SYNC/RAW straight moves looking fine but mirror the STM32
     *        gyro correction: GYRO moves veer in a runaway arc and heading holds lock 180°
     *        off target. Re-run the direction check after ANY motor re-wire or chassis swap.
     *  @warning (true, false) = Lofaix/EV3 mounting (matches vendor `setupLEGOMotors()`).
     *           The TT test rover uses (false, true). */
    constexpr bool MOTOR_LEFT_REVERSE = true;

    /** @brief Reverse the right motor's positive-power direction at the driver level.
     *  @note See `MOTOR_LEFT_REVERSE` — the two flags describe one mounting and change
     *        together as a pair. */
    constexpr bool MOTOR_RIGHT_REVERSE = false;

    // ---------------------------------------------------------------------------------------
    // Motor profile + PID gains (selected via ActiveMotor)
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Vendor PID + encoder defaults for the Matrix TT motor.
     * @note Source: MatrixMiniR4 MiniR4DriveDC.h PID docstrings (setMoveSyncPID, setMoveGyroPID,
     *       setTurnGyroPID) + examples/4-DriveDC Functions/MiniR4_DriveDC_Test/MiniR4_DriveDC_Test.ino:64-74.
     */
    struct MatrixTT
    {
        static constexpr uint16_t PPR = 545;
        static constexpr uint16_t RPM = 200;
        static constexpr float    MOVESYNC_KP = 0.02f;
        static constexpr float    MOVESYNC_KI = 0.0f;
        static constexpr float    MOVESYNC_KD = 0.04f;
        static constexpr float    TURNGYRO_KP = 22.75f;
        static constexpr float    TURNGYRO_KI = 0.08f;
        static constexpr float    TURNGYRO_KD = 0.25f;
    };

    /**
     * @brief Vendor PID + encoder defaults for the LEGO EV3 motor (LE-F TurnGyro variant).
     * @note Source: MiniR4DriveDC.h PID docstrings + MiniR4_DriveDC_Test.ino:52-62.
     *       LE-F = "LEGO EV3 Fast" Kp (22.75). LE-S variant uses Kp=5.25 — not used here.
     */
    struct LegoEV3
    {
        static constexpr uint16_t PPR = 360;
        static constexpr uint16_t RPM = 300;
        static constexpr float    MOVESYNC_KP = 0.05f;
        static constexpr float    MOVESYNC_KI = 0.0f;
        static constexpr float    MOVESYNC_KD = 0.025f;
        static constexpr float    TURNGYRO_KP = 22.75f;   // LE-F variant
        static constexpr float    TURNGYRO_KI = 0.02f;
        static constexpr float    TURNGYRO_KD = 1.54f;
    };

    /**
     * @brief Single edit point to swap the chassis motor model.
     *
     * Default is LegoEV3 (Lofaix competition robot). Switch to MatrixTT for the
     * test rover. The flat constants below (MOTOR_PPR, MOVESYNC_K*, …) forward
     * to ActiveMotor::PPR / ActiveMotor::MOVESYNC_KP / etc. so call sites do
     * not change.
     *
     * @warning Swapping motors does NOT recalibrate MOTOR_BIAS_L/R — those are
     *          per-physical-chassis measurements and MUST be re-captured via
     *          Drive.calibrateMotorBias() after a swap. See ADR-011 + ADR-022.
     */
    using ActiveMotor = LegoEV3;

    constexpr uint16_t MOTOR_PPR    = ActiveMotor::PPR;
    constexpr uint16_t MOTOR_RPM    = ActiveMotor::RPM;
    constexpr float    MOVESYNC_KP  = ActiveMotor::MOVESYNC_KP;
    constexpr float    MOVESYNC_KI  = ActiveMotor::MOVESYNC_KI;
    constexpr float    MOVESYNC_KD  = ActiveMotor::MOVESYNC_KD;
    constexpr float    TURNGYRO_KP  = ActiveMotor::TURNGYRO_KP;
    constexpr float    TURNGYRO_KI  = ActiveMotor::TURNGYRO_KI;
    constexpr float    TURNGYRO_KD  = ActiveMotor::TURNGYRO_KD;

    // ---------------------------------------------------------------------------------------
    // Geometry (TODO: measure on real chassis)
    // ---------------------------------------------------------------------------------------

    /** @brief Drive-wheel diameter in millimetres. @note TODO: measure on real chassis. */
    constexpr float WHEEL_DIAM_MM = 63.0f;

    /** @brief Wheel-to-wheel track width in millimetres. @note TODO: measure on real chassis. */
    constexpr float TRACK_MM = 180.0f;

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
     * turnDeg / turnToYaw, plus LineFollower.setPreset for cruise. Default is
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
    constexpr int16_t SPEED_SLOW = 60;

    /** @brief PWM target for `SpeedPreset::NORMAL` (default cruise / general moves). */
    constexpr int16_t SPEED_NORMAL = 70;

    /** @brief PWM target for `SpeedPreset::FAST` (long straights, low-precision moves). */
    constexpr int16_t SPEED_FAST = 90;

    // ---------------------------------------------------------------------------------------
    // Turn defaults
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Default wheel-pattern selector for the custom TurnGyro state machine.
     *
     * Encoding (NEW — set by the 2026-05-17 PR4 plan; intentionally different from
     * the legacy `MiniR4.DriveDC.TurnGyro` byte values):
     *   - `0` = COUNTER_ROT — both wheels spin opposite directions, chassis pivots
     *                         around its center. Tightest in-place rotation.
     *   - `1` = PIVOT_LEFT  — left wheel braked, right wheel drives; chassis pivots
     *                         around the LEFT wheel.
     *   - `2` = PIVOT_RIGHT — right wheel braked, left wheel drives; chassis pivots
     *                         around the RIGHT wheel.
     *
     * @note `TurnDegBuilder::setMode` / `TurnToYawBuilder::setMode` accept this byte;
     *       PathTracker maps `Routes::TurnMode::*` to it via `_turnModeToByte`. See
     *       `Drive.h::TurnDegBuilder::setMode` for the full encoding contract.
     */
    constexpr uint8_t TURN_MODE = 0;

    /** @brief Apply active brake at end of turn (vs. coast). */
    constexpr bool TURN_BRAKE = true;

    // ---------------------------------------------------------------------------------------
    // Line-sensor → axle geometric offset (intersection overshoot compensation)
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Distance from the FRONT line-sensor board's centerline to the wheel-axle
     *        line, in cm.
     *
     * Positive = sensor is in front of (ahead of) the axle.
     *
     * Used by the PathTracker LINE_OVERSHOOT state: the intersection-detect event fires
     * when the SENSOR is over the junction marker, but we want the chassis CENTER (the
     * axle midpoint) to land on the node. After detecting the intersection, PathTracker
     * re-arms LineFollower with the axle offset of the board that led the hop (front when
     * FRONT, `LINE_SENSOR_AXLE_OFFSET_REAR_CM` when REAR) to roll forward by exactly that
     * amount before stopping.
     *
     * Also doubles as the single-wheel PIVOT overshoot budget (ADR-036/044): the
     * offline validator bounds a pivot's sensor-terminated approach `delta` against
     * this same constant. Pre-#54 that budget was implicitly FRONT-only (pivots only
     * ever ran FRONT); post-#54 (ADR-073) the budget is direction-resolved between
     * this constant and `LINE_SENSOR_AXLE_OFFSET_REAR_CM`, mirroring the
     * LINE_OVERSHOOT selection above -- see `ReversePivotPolicy::pivotOvershootBudgetCm`.
     *
     * @note TODO: measure on real chassis.
     */
    constexpr float LINE_SENSOR_AXLE_OFFSET_CM = 12.02f;

    /**
     * @brief Distance from the REAR line-sensor board's centerline to the wheel-axle line,
     *        in cm, used when a hop is followed in REVERSE (rear board leading).
     *
     * On a bidirectional chassis the rear board need not sit the same distance from the
     * axle as the front, so a REAR hop must dead-reckon with its own offset. Reusing the
     * front offset lands every reverse hop off-node and feeds that error into the next
     * dead-reckoned hop. PathTracker selects this when the resolved direction is
     * `Direction::REAR`.
     *
     * @note TODO: measure on real chassis. Defaults to the front offset until the rear
     *       board is wired (T5); override once the two distances are known to differ.
     */
    constexpr float LINE_SENSOR_AXLE_OFFSET_REAR_CM = 7.68f;

    /**
     * @brief Floor (cm) for a pivot-trimmed dead-reckon drive / LINE overshoot advance (ADR-036).
     *
     * PathTracker subtracts the resolved pivot over-travel δ from a dead-reckon
     * drive (and from the LINE overshoot advance) to compensate a single-wheel
     * pivot. The offline validator already guarantees `δ < edge length` on every
     * dead-reckon side and `δ ≤ overshoot budget` on every LINE-incoming side, so
     * a correctly-built routes table never drives the effective distance to (or
     * below) zero. This is a DEFENSIVE floor only: if a future bad table or a
     * validator/config mismatch ever produced a non-positive effective distance,
     * `Drive.driveDist` would reject a negative cm and silently no-op the hop,
     * stalling the FSM. Clamping to a tiny positive advance keeps motion forward
     * (with a Serial trace) instead of dropping the drive into UB / a stall.
     *
     * @note Single-sourced here so the offline validator / sim
     *       (`parse_pivot_trim_min_drive_cm_from_config`) read the same value the
     *       firmware uses (TC26 parity).
     */
    constexpr float PIVOT_TRIM_MIN_DRIVE_CM = 0.5f;

    /**
     * @brief Minimum authored pivot turn angle, in degrees (offline validator guard, ADR-036).
     *
     * The offline validator rejects any authored pivot turn `(u, v, w)` whose
     * geometric angle `|θ|` is below this floor. A pivot only fires at runtime
     * when the pre-turn maneuver dispatches (heading delta `> NAV_TURN_TOLERANCE_DEG`,
     * PathTracker), so a turn near that gate would silently no-op the pivot while
     * its δ trim is still subtracted — under-driving the adjacent hop. Keeping a
     * real pivot comfortably above the gate makes a too-shallow "pivot" a
     * build-time authoring error instead of a silent runtime mis-position.
     *
     * @note This is a TUNABLE knob, not a derived value: set it to the smallest
     *       pivot the chassis should be allowed to author. It must stay above
     *       `NAV_TURN_TOLERANCE_DEG` (the firmware dispatch gate). NOT read by the
     *       firmware at runtime — the validator/sim single-source it from here
     *       (`parse_pivot_min_turn_deg_from_config`) so a retune cannot desync the
     *       offline check (TC26 parity).
     */
    constexpr float PIVOT_MIN_TURN_DEG = 5.0f;

    // ---------------------------------------------------------------------------------------
    // Drive.stopAtLine() — drive-until-line seek defaults (ADR-027)
    //
    // `Drive.stopAtLine(expectedCm)` drives straight toward an expected perpendicular crossing
    // line, arms a T_OR_CROSS detector only once the chassis is near the expected endpoint,
    // overshoots so the axle lands on the line, then active-brakes. PathTracker uses it on
    // flagged BLIND hops to re-anchor `Localization` on the physical line instead of dead
    // reckoning. The post-detect overshoot default reuses `LINE_SENSOR_AXLE_OFFSET_CM` above.
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Arm-window half-length (cm): detection arms only once the chassis is within this
     *        many cm of the expected endpoint.
     *
     * The seeker drives blind until `traveled >= expectedCm - STOP_AT_LINE_ARM_WINDOW_CM`, then
     * arms its `T_OR_CROSS` detector. This prevents stopping on a line crossed earlier en route.
     *
     * @note Tuning constraint (load-bearing): the window only fires on the true line if the
     *       encoder estimate of `traveled` is within this many cm of reality. If dead reckoning
     *       UNDER-travels by more than this, the chassis crosses the real line while still
     *       disarmed and misses it (→ runs to max → `LINE_NOT_FOUND`). Size this LARGER than the
     *       worst-case dead-reckoning under-travel over a hop — generous, not tight. That drift
     *       is exactly what stop-at-line exists to correct. Verified on hardware (bench B3.7).
     * @note Overridable per call via `StopAtLineBuilder::setArmWithin(float)`.
     */
    constexpr float STOP_AT_LINE_ARM_WINDOW_CM = 10.0f;

    /**
     * @brief Extra search margin (cm) past the expected endpoint before failing.
     *
     * If no `T_OR_CROSS` is detected by `traveled >= expectedCm + STOP_AT_LINE_MAX_MARGIN_CM`,
     * `Drive` fail-fast faults `LINE_NOT_FOUND` (stop + clear queue) — there is no silent
     * dead-reckon fallback. Surfaced to PathTracker as `DRIVE_FAULT`.
     *
     * @note Overridable per call via `StopAtLineBuilder::setMaxDistance(float)`, which sets the
     *       absolute max distance (default `expectedCm + STOP_AT_LINE_MAX_MARGIN_CM`).
     */
    constexpr float STOP_AT_LINE_MAX_MARGIN_CM = 15.0f;

    /**
     * @brief Consecutive `T_OR_CROSS` samples required to confirm a line detection.
     *
     * The seeker debounces the crossing with its own rising-edge counter — a single noisy
     * `getJunctionType()` sample no longer triggers the stop. Mirrors the streak-confirm idiom
     * used by `INTERSECTION_DEBOUNCE_TICKS`. At ~50-100 Hz cooperative tick, 2 ticks is well
     * within a ~5-10 cm junction marker's dwell time while rejecting single-sample flicker.
     */
    constexpr uint8_t STOP_AT_LINE_DETECT_TICKS = 2;

    /**
     * @brief No-progress watchdog timeout (ms) for the unbounded stopAtLine seek.
     *
     * The cooperative tick runs every ~10-30 ms (LineArray I2C refresh + the 2 ms drain delay),
     * so a moving seek advances every tick; 1500 ms (~50-150 idle ticks) trips only a truly
     * stalled / zero-power seek, never a slow-but-moving one. On a no-progress timeout the seek
     * fail-fasts Fault::LINE_NOT_FOUND instead of hanging the async nav FSM (which has no drain
     * timeout). Bench-tunable.
     */
    constexpr uint32_t STOP_AT_LINE_STALL_TIMEOUT_MS = 1500;
    // SHARED-WINDOW FLOOR (load-bearing): this no-progress window is reused VERBATIM as the
    // wall-stop SUCCESS window in `Drive::_updateWallStop` (AC8) — but with the meaning INVERTED.
    // There a no-progress span means "stalled against the wall = AT WALL" (success), not "seek
    // failed". A retune that shortens this to fail line seeks faster ALSO shrinks the wall-success
    // window in the opposite direction: too SHORT and a low-PWM GYRO wall press reads as "stalled"
    // before the wheels overcome static friction — a false at-wall on thin air, then BACK_OFF +
    // re-anchor against nothing. Keep it well above motor spin-up; the floor below guards the press.
    static_assert(STOP_AT_LINE_STALL_TIMEOUT_MS >= 500,
                  "Config::STOP_AT_LINE_STALL_TIMEOUT_MS doubles as the wall-stop press SUCCESS "
                  "window (Drive::_updateWallStop, inverted meaning). Below ~500 ms a low-PWM wall "
                  "push can false-stall before the wheels move (re-anchoring against thin air). "
                  "Keep it above motor spin-up — retune both primitives' needs together.");

    /**
     * @brief Minimum traveled advance (cm) per check that counts as forward progress for the
     *        stall watchdog (guards float noise / encoder jitter from masking a real stall).
     */
    constexpr float STOP_AT_LINE_PROGRESS_EPS_CM = 0.2f;

    // ---------------------------------------------------------------------------------------
    // Anticipatory line-approach ramp (LineFollower junction hops + Drive.stopAtLine() seek)
    //
    // Both line-detection primitives predict WHEN the crossing line should arrive (from the
    // authored edge length / expected distance) and ramp cruise DOWN to the arrival floor
    // (`Config::PWM_MOVING_MIN`) BEFORE that predicted point, instead of reacting only after
    // the sensor sees the line. This lets the chassis already be slow when an early line
    // arrives, and still lets a late line be found via a bounded floor-speed search before
    // declaring a hard fault. Toggling the feature off restores today's flat-cruise-until-
    // detected behavior byte-for-byte.
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Master toggle for the anticipatory line-approach ramp.
     *
     * `true` arms the shared ramp-down schedule on both consumers (`LineFollower`'s junction
     * hops via `setApproachTarget`, `Drive.stopAtLine()`'s seek). `false` disables the
     * schedule everywhere — both paths fall back to today's flat cruise until the line is
     * detected, byte-identical to pre-feature behavior.
     *
     * @note Global kill switch, not a per-call knob: flip this to `false` to instantly revert
     *       the feature without touching any call site.
     * @note **Compile-time override:** guarded by `#ifndef LINE_APPROACH_RAMP_ENABLED_DEFAULT`
     *       so a host test can pin it target-wide via `-DLINE_APPROACH_RAMP_ENABLED_DEFAULT=false`
     *       on the compiler command line without editing this file — same idiom as
     *       `PATH_PASS_THROUGH_ENABLED_DEFAULT` (`ConfigNav.h`). The public
     *       `Config::LINE_APPROACH_RAMP_ENABLED` symbol is unchanged; only its initializer is
     *       overridable.
     */
#ifndef LINE_APPROACH_RAMP_ENABLED_DEFAULT
#define LINE_APPROACH_RAMP_ENABLED_DEFAULT true
#endif
    constexpr bool LINE_APPROACH_RAMP_ENABLED = LINE_APPROACH_RAMP_ENABLED_DEFAULT;

    /**
     * @brief Distance (cm) BEFORE the predicted line crossing where the ramp bottoms out at
     *        the arrival floor (`Config::PWM_MOVING_MIN`).
     *
     * The schedule reaches floor speed this many cm early so an early-arriving line (encoder
     * under-estimate, curvy-follow path-length bias) still meets an already-slow chassis
     * instead of one still ramping down. Larger values trade a longer floor-speed crawl for
     * more margin against prediction error.
     *
     * @note Must be non-negative — enforced by a `ConfigChecks.h` static_assert.
     */
    constexpr float LINE_APPROACH_MARGIN_CM = 4.0f;

    /**
     * @brief Overrun bound (cm) PAST the predicted line crossing before the bounded
     *        floor-speed search gives up and hard-faults.
     *
     * Mirrors `STOP_AT_LINE_MAX_MARGIN_CM` (ADR-039: a missed line is a hard fault, not a
     * silent crawl). If no line is detected by `predicted + LINE_APPROACH_MAX_MARGIN_CM`, the
     * caller (`LineFollower` / `Drive.stopAtLine()`) surfaces `LINE_NOT_FOUND` so PathTracker
     * stops the chassis rather than crawling forever.
     *
     * @note Must be strictly positive — enforced by a `ConfigChecks.h` static_assert.
     */
    constexpr float LINE_APPROACH_MAX_MARGIN_CM = 15.0f;

    // ---------------------------------------------------------------------------------------
    // Wall-stop re-zero primitive (`Drive.wallStop(cm)`) — wall-zero reset nodes (AC8)
    //
    // `Drive.wallStop(cm)` pushes the chassis toward a physical wall at low PWM and re-zeros
    // against it via encoder-STALL — the SAME no-progress watchdog `stopAtLine` already uses
    // (`stopAtLineProgressed` / `stopAtLineStalled` over `STOP_AT_LINE_STALL_TIMEOUT_MS` and
    // `STOP_AT_LINE_PROGRESS_EPS_CM` above). The ONLY inversion is the meaning of a stall: in
    // `stopAtLine` a stall is a `LINE_NOT_FOUND` fault; here a stall is SUCCESS (wall reached),
    // and the chassis then backs off `WALL_STOP_BACK_OFF_CM` and goes idle. The wall-stop verb
    // deliberately reuses those two stall constants rather than adding parallel ones, so a single
    // tune of the stall window governs both primitives — only the three knobs unique to wall-stop
    // are added here.
    //
    // COUPLED CALL SITES for the wall-stop stall window (NOT redefined here — listed so a future
    // editor who retunes them knows wall-stop rides on the same values):
    //   - `Config::STOP_AT_LINE_STALL_TIMEOUT_MS`   — no-progress timeout that signals "at wall"
    //                                                  (INVERTED meaning vs the line seek; a
    //                                                  spin-up floor static_assert above guards it).
    //   - `Config::STOP_AT_LINE_PROGRESS_EPS_CM`    — minimum advance that resets the watchdog.
    //   - `Drive::_updateWallStop()` (Drive.cpp)    — sole consumer of the three constants below.
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Low push PWM the wall-stop primitive drives at toward the wall, in [-100, 100].
     *
     * Sized AT the motor deadband floor (`PWM_MIN_MAGNITUDE`) on purpose: `deadbandSnap()` floors
     * any non-zero command up to `PWM_MIN_MAGNITUDE`, so a softer crawl is not physically
     * achievable on this chassis — the effective push is always `max(pwm, PWM_MIN_MAGNITUDE)`. A
     * floor-level push is the gentlest reliable wall press while still carrying enough torque to
     * stall cleanly against the wall (which is exactly the success condition).
     *
     * @warning Load-bearing: pushing below `PWM_MIN_MAGNITUDE` is silently floored UP to it by
     *          `deadbandSnap`, so a "20-PWM crawl" is fiction — the wheels would still press at 50.
     *          Pushing far ABOVE the floor risks a hard impact that bounces the chassis off the
     *          wall before the watchdog confirms the stall. Keep this at or just above the floor.
     *          The static_assert below pins the value in the snap-safe band.
     */
    constexpr int16_t WALL_STOP_PUSH_PWM = PWM_MIN_MAGNITUDE;
    static_assert(WALL_STOP_PUSH_PWM >= PWM_MIN_MAGNITUDE && WALL_STOP_PUSH_PWM <= PWM_MAX,
                  "Config::WALL_STOP_PUSH_PWM must be in [PWM_MIN_MAGNITUDE, PWM_MAX] so the wall "
                  "push is neither silently floored by the motor deadband nor capped by the "
                  "saturation limit — a low-but-real push is the whole point of the primitive.");

    /**
     * @brief Distance (cm) the chassis backs off the wall after a confirmed stall, before idling.
     *
     * On stall (= wall reached) the wall-stop primitive rolls this far in the reverse-of-push
     * direction so the chassis is no longer pressing the wall when control returns to the caller —
     * leaving it free to turn / drive away without scrubbing against the wall. Tuned small: just
     * enough to break contact, not a full retreat. Bench-tunable.
     *
     * @note Overridable per call via `WallStopBuilder::setBackOff(float)`.
     */
    constexpr float WALL_STOP_BACK_OFF_CM = 2.0f;

    /**
     * @brief Outer advisory timeout (ms) before the wall-stop primitive gives up waiting to stall.
     *
     * If the chassis pushes for this long and NEVER stalls (e.g. it never reached a wall and the
     * wheels kept turning freely), the primitive records a NON-LATCHING advisory "stall not
     * detected" outcome (queryable via `Drive.wallStopTimedOut()`), stops the motors, and goes
     * idle — it does NOT set the hard `Drive::Fault` that `hasFault()` latches on, and does NOT
     * halt the FSM. The caller (PathTracker) then assumes-at-wall and continues. Sized generously
     * above `STOP_AT_LINE_STALL_TIMEOUT_MS`: the inner stall watchdog should normally fire first on
     * a real wall press, so this outer cap is the never-reached-a-wall safety net, not the primary
     * success path. Bench-tunable.
     *
     * @note Overridable per call via `WallStopBuilder::setTimeout(uint32_t)`.
     * @warning Load-bearing ordering: keep this STRICTLY GREATER than
     *          `STOP_AT_LINE_STALL_TIMEOUT_MS` so a genuine wall press trips the inner
     *          stall-success path BEFORE this outer advisory cap fires. If this were smaller, every
     *          wall press would report the advisory timeout instead of success. The static_assert
     *          below enforces the ordering.
     */
    constexpr uint32_t WALL_STOP_TIMEOUT_MS = 3000;
    static_assert(WALL_STOP_TIMEOUT_MS > STOP_AT_LINE_STALL_TIMEOUT_MS,
                  "Config::WALL_STOP_TIMEOUT_MS must exceed STOP_AT_LINE_STALL_TIMEOUT_MS so a real "
                  "wall press trips the inner stall-success watchdog before the outer advisory "
                  "timeout — otherwise every successful press would mis-report as a timeout.");

    // ---------------------------------------------------------------------------------------
    // Turn PD controller (ADR-068) — per-tick PD + kinetic creep floor + slew-limited ramp-up.
    //
    // Replaces `MiniR4.DriveDC.TurnGyro` AND the earlier two-stage cruise/slow-zone TurnGyro
    // state machine (2026-05-17 PR4 plan, now superseded). Ported from the mentor's
    // competition-verified `M_Turn_angle` (`Meat_code/wro20260714_大通道gyro+16路2ms_RT.mbr4`),
    // with one addition the mentor's light TT chassis does not need (the slew limiter, for the
    // heavier Lofaix chassis). Pure decision math lives in `TurnPDPolicy.h`; these constants are
    // the tunables `Drive::_updateTurnProfile` feeds into it. See ADR-068 for the full rationale.
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Cruise PWM for `Drive.turnDeg` / `Drive.turnToYaw`, in [-100, 100].
     *
     * Two roles:
     *   1. Default value of `TurnDegBuilder::_power` / `TurnToYawBuilder::_power`;
     *      callers can override per-call via `.setPower(...)` on the builder.
     *   2. Authority clamp for the per-tick turn PD controller (ADR-068): the raw
     *      `Kp*err + Kd*d(err)/dt` command is clamped to `±power` (this constant,
     *      absent a per-call override) BEFORE the slew limiter and creep floor are
     *      applied. A clamped P term already IS the deceleration ramp — there is no
     *      separate slow zone or terminal stage.
     *
     * @note **Compile-time override:** guarded by `#ifndef TURN_CRUISE_PWM_DEFAULT` so a
     *       host test can pin it target-wide via `-DTURN_CRUISE_PWM_DEFAULT=...` without
     *       editing this file — same idiom as `HOP_SETTLE_MS_DEFAULT` (`ConfigNav.h`).
     *       Guarded by the turn-ladder invariant in `ConfigChecks.h` (issue #152).
     * @note TODO: tune on real chassis.
     */
#ifndef TURN_CRUISE_PWM_DEFAULT
#define TURN_CRUISE_PWM_DEFAULT 50
#endif
    constexpr int16_t TURN_CRUISE_PWM = TURN_CRUISE_PWM_DEFAULT;

    /**
     * @brief Proportional gain for the per-tick turn PD controller (ADR-068), in PWM per degree.
     *
     * Ported from the mentor's competition-verified `M_Turn_angle`: `Kp = 2` (the 2026-07
     * screenshot revision the mentor confirmed working on THIS chassis; an earlier revision used
     * `Kp = 1`). Kp maps a degree error straight to a PWM, so it ports DIRECTLY — no loop-rate
     * conversion, unlike TURN_KD below. `TurnPDPolicy.h::turnPdStep` computes `Kp * errNew` before
     * the D term, clamp, slew, and creep floor.
     *
     * @note Verify on the robot via `inoTest/turn_tune` (T11) before committing.
     */
#ifndef TURN_KP_DEFAULT
#define TURN_KP_DEFAULT 2.0f
#endif
    constexpr float TURN_KP = TURN_KP_DEFAULT;

    /**
     * @brief TIME-NORMALIZED derivative gain for the turn PD controller (ADR-068), in PWM per
     *        (deg/s) — NOT the mentor's raw per-iteration gain.
     *
     * The mentor's `M_Turn_angle` computes a raw per-ITERATION difference (`errNew - errOld`,
     * no division), so his continuous damping is `Kd_raw * loop_period` — correct only at his
     * fixed ~6 ms loop. Our cooperative tick is a MEASURED ~8 ms that jitters to ~11.5 ms
     * (`inoTest/tick_bench`, 2026-07-16), so porting the raw gain verbatim would silently re-scale
     * every time the tick jitters. `TurnPDPolicy.h::turnPdStep` instead divides by the MEASURED
     * per-step `dtSec`, so this gain carries real units of PWM per (deg/s) and reproduces the
     * mentor's CONTINUOUS damping at any loop rate: `TURN_KD = Kd_raw * mentor_loop_period`.
     *
     * The mentor's 2026-07 revision uses `Kd_raw = 10` (an earlier revision used 3), so
     * `0.06 ~= 10 * 0.006 s`. Sanity check: at omega=90 deg/s our D term is `0.06 * 90 = 5.4`,
     * matching his per-iteration `10 * (90 * 0.006) = 5.4`.
     *
     * @note Verify on the robot via `inoTest/turn_tune` (T11) before committing.
     */
#ifndef TURN_KD_DEFAULT
#define TURN_KD_DEFAULT 0.06f
#endif
    constexpr float TURN_KD = TURN_KD_DEFAULT;

    /**
     * @brief Threshold (PWM magnitude) below which the turn PD stops modulating and emits the
     *        fixed kinetic creep floor instead (ADR-068, `TurnPDPolicy.h::turnCreepFloor`).
     *
     * `TURN_CREEP_PWM` plus a 5-PWM headroom margin, so a command handing off to the creep floor
     * has a small band to land in instead of chattering across the boundary tick-to-tick.
     *
     * Robot-verified 2026-07: the bench-measured 43 (over a 38 creep) collapsed the whole final
     * ~21 deg of a turn to a flat, fast creep and overshot a 90 deg turn to ~120. Lowered to the
     * mentor's proven tail (creep 20 + 5) so the PD keeps modulating far closer to the target.
     */
#ifndef TURN_CREEP_THRESH_PWM_DEFAULT
#define TURN_CREEP_THRESH_PWM_DEFAULT 25
#endif
    constexpr int16_t TURN_CREEP_THRESH_PWM = TURN_CREEP_THRESH_PWM_DEFAULT;

    /**
     * @brief KINETIC creep floor (PWM magnitude) the turn PD emits once it loses authority
     *        (`|cmd| < TURN_CREEP_THRESH_PWM`), ADR-068.
     *
     * The lowest PWM that reliably SUSTAINS rotation once the chassis is already turning — NOT the
     * static breakaway floor (`TURN_BREAKAWAY_PWM`, the higher PWM needed to START an in-place spin
     * from rest, measured HIGHER than this because a spin scrubs the tires sideways). Do not reuse
     * `deadbandSnap` / `PWM_MIN_MAGNITUDE` here; see `TurnPDPolicy.h`'s file docblock ("Why the
     * creep floor is not deadbandSnap").
     *
     * Robot-verified 2026-07: the bench sweep's 38 approached the target too fast and coasted a
     * 90 deg turn out to ~120. Set to the mentor's competition-proven kinetic tail floor of 20,
     * which lands on target on this chassis.
     */
#ifndef TURN_CREEP_PWM_DEFAULT
#define TURN_CREEP_PWM_DEFAULT 20
#endif
    constexpr int16_t TURN_CREEP_PWM = TURN_CREEP_PWM_DEFAULT;

    /**
     * @brief STATIC breakaway floor (PWM magnitude) the turn PD emits from REST until the chassis
     *        has demonstrably started rotating (ADR-068).
     *
     * Bench-measured on Lofaix (T1, 2026-07-16, phase a1): the lowest PWM that reliably STARTS an
     * in-place spin from a standstill, plus a small margin. HIGHER than the kinetic
     * `TURN_CREEP_PWM` that sustains an already-moving spin (44 vs 38 measured) because starting
     * from rest must break STATIC scrub friction across every contact patch at once. Applied as
     * the creep floor only while `!_turnBrokeAway`; once `_updateTurnProfile` sees the yaw move
     * past `TURN_BREAKAWAY_MOTION_DEG` it drops to the efficient kinetic `TURN_CREEP_PWM`. Without
     * it, a small in-place turn (whose PD command never exceeds the creep floor) would command
     * only `TURN_CREEP_PWM`, which is below the static breakaway, and never start — hunting to
     * `TURN_TIMEOUT_MS`.
     *
     * @note TODO: bench-tune (T11) — raise if a cold/heavy chassis or a sagging pack fails to
     *       break away on the first ticks.
     */
#ifndef TURN_BREAKAWAY_PWM_DEFAULT
#define TURN_BREAKAWAY_PWM_DEFAULT 46
#endif
    constexpr int16_t TURN_BREAKAWAY_PWM = TURN_BREAKAWAY_PWM_DEFAULT;

    /**
     * @brief Rotation from the turn's start yaw (degrees) after which the PD hands off from the
     *        static `TURN_BREAKAWAY_PWM` kick to the kinetic `TURN_CREEP_PWM` floor (ADR-068).
     *
     * Chosen well above the bench IMU noise spread (T1: 0.100 deg over 100 stationary reads) so a
     * stationary chassis's read jitter cannot falsely latch "moving" and drop it to a creep floor
     * that cannot start it. Latched once reached — see `_updateTurnProfile`.
     *
     * @note TODO: bench-tune (T11) — lower to reach the efficient creep sooner, but stay clear of
     *       the IMU noise floor.
     */
#ifndef TURN_BREAKAWAY_MOTION_DEG_DEFAULT
#define TURN_BREAKAWAY_MOTION_DEG_DEFAULT 1.0f
#endif
    constexpr float TURN_BREAKAWAY_MOTION_DEG = TURN_BREAKAWAY_MOTION_DEG_DEFAULT;

    /**
     * @brief Maximum PWM growth per tick for the turn PD's ramp-up limiter (ADR-068,
     *        `TurnPDPolicy.h::turnSlewLimit`).
     *
     * NOT part of the mentor's TT-verified law — his light rover does not need it. Added
     * because at large heading error `Kp*err` saturates instantly (0 -> power in one tick),
     * which risks a traction break on the heavier Lofaix chassis (rotates about the gripping
     * wheel and translates the axle off the node, invisible to a heading-only loop). Binds
     * ONLY while `|cmd|` is growing in the previous command's direction; ramp-down needs no
     * limiter since P already shrinks with the error.
     *
     * @note TODO: bench-tune (T11) — down-tune until the initial kick no longer slips on Lofaix.
     */
#ifndef TURN_SLEW_PWM_PER_TICK_DEFAULT
#define TURN_SLEW_PWM_PER_TICK_DEFAULT 10
#endif
    constexpr int16_t TURN_SLEW_PWM_PER_TICK = TURN_SLEW_PWM_PER_TICK_DEFAULT;

    /**
     * @brief Consecutive in-band samples required to exit the turn PD's settle debounce
     *        (ADR-068, `TurnPDPolicy.h::turnSettleStep` / `turnSettled`).
     *
     * A single out-of-band sample resets the count to 0. Set to 2 (operator decision) rather
     * than the mentor's 3, so the debounce stays satisfiable at the bench-measured
     * `TURN_MIN_RESOLVABLE_DEG` step size and the widened `TURN_TOLERANCE_DEG` below — see the
     * debounce-coupling `static_assert` in `ConfigChecks.h`, which turns a future desync of any
     * of the three into a build failure instead of every turn silently hunting to timeout.
     *
     * @note TODO: bench-tune (T11).
     */
#ifndef TURN_SETTLE_SAMPLES_DEFAULT
#define TURN_SETTLE_SAMPLES_DEFAULT 2
#endif
    constexpr uint8_t TURN_SETTLE_SAMPLES = TURN_SETTLE_SAMPLES_DEFAULT;

    /**
     * @brief Timeout (ms) for the turn PD controller before it gives up converging (ADR-068).
     *
     * On expiry the profile brakes, prints the residual heading error, and marks the turn done
     * WITHOUT latching a fault or clearing the queue — a 1-2 deg miss must not end a scored
     * run; PathTracker re-anchors heading at line arrivals (ADR-052) regardless. Mentor-verified
     * starting value (his `M_Turn_angle` timeout is also 3000 ms).
     *
     * @note TODO: bench-tune (T11).
     */
#ifndef TURN_TIMEOUT_MS_DEFAULT
#define TURN_TIMEOUT_MS_DEFAULT 3000
#endif
    constexpr uint32_t TURN_TIMEOUT_MS = TURN_TIMEOUT_MS_DEFAULT;

    /**
     * @brief Final heading tolerance in degrees for the turn PD controller (ADR-068).
     *
     * When |heading error| < this value for `TURN_SETTLE_SAMPLES` consecutive ticks, the turn
     * brakes and declares itself complete (`TurnPDPolicy.h::turnSettleStep` / `turnSettled`).
     * Strictly less-than: a sample exactly AT the boundary is OUT of band.
     *
     * @warning COUPLED to `TURN_MIN_RESOLVABLE_DEG` and `TURN_SETTLE_SAMPLES` — the settle band
     *          must be wide enough that at least `TURN_SETTLE_SAMPLES` physical steps land
     *          inside it per crossing, or the debounce can never be satisfied and every turn
     *          silently hunts to `TURN_TIMEOUT_MS` (non-faulting by design). Widened from the
     *          vendor-inherited 0.5 deg to 0.75 deg (operator decision, 2026-07-16) specifically
     *          to keep this satisfiable at the bench-measured 0.4423 deg/tick resolution:
     *          `(2 * 0.75) / 0.4423 ~= 3.39` against a required 2 samples — a 1.70x margin. See
     *          the debounce-coupling `static_assert` in `ConfigChecks.h`, which converts a
     *          future desync into a build failure instead of a field mystery.
     *
     * @note TODO: bench-tune (T11) if B3.2 wants it tighter or looser.
     */
    constexpr float TURN_TOLERANCE_DEG = 0.75f;

    /**
     * @brief Smallest physically resolvable heading step at the turn PD's creep floor, in
     *        degrees (ADR-068).
     *
     * Bench-measured (T1, Lofaix, 2026-07-16): `omega_at_creep * measured_tick_period` =
     * 19.52 deg/s * ~22.66 ms = 0.4423 — the angular rate sustained at `TURN_CREEP_PWM` times the
     * measured ~22.66 ms TURN tick (T1 phase b: the tick a real PD turn actually runs at, higher
     * than the ~8 ms idle tick because a turn adds a live yaw read plus a Move write per tick).
     * NOT a placeholder: T1's bench run is the source of truth this value is pinned to. Consumed
     * only by the `static_assert`s in `ConfigChecks.h` (never read at runtime) — it exists so a
     * future retune of `TURN_TOLERANCE_DEG` / `TURN_SETTLE_SAMPLES` / `TURN_CREEP_PWM` cannot
     * silently desync from the physical floor.
     *
     * @note TODO: re-measure (T11) if `TURN_CREEP_PWM` is retuned — this value is derived from
     *       it and goes stale if the creep floor moves without a re-bench.
     */
#ifndef TURN_MIN_RESOLVABLE_DEG_DEFAULT
#define TURN_MIN_RESOLVABLE_DEG_DEFAULT 0.4423f
#endif
    constexpr float TURN_MIN_RESOLVABLE_DEG = TURN_MIN_RESOLVABLE_DEG_DEFAULT;

    /**
     * @brief Persistent-IMU-fault watchdog timeout for the custom TurnGyro profile, in ms.
     *
     * `Drive::_updateTurnProfile()` reads the IMU yaw every tick. A single transient read miss
     * skips the tick (holds the previous `Move(L,R)`) rather than act on a fabricated yaw. But a
     * SUSTAINED failure must not spin forever — an async turn has no `_drainBlocking` deadline
     * behind it. Once no good yaw read has landed for this long since the turn started (or since
     * the last good read), the profile hard-stops and latches `DriveClass::Fault::IMU_READ_FAILED`.
     *
     * Sized well above a couple of ticks (`readYawDeg` already retries once internally) so a brief
     * I2C hiccup never false-trips, yet short enough that a real fault stops the in-place spin
     * promptly.
     *
     * @note TODO: tune on real chassis.
     */
    constexpr uint32_t TURN_IMU_STALL_TIMEOUT_MS = 500;
    static_assert(TURN_IMU_STALL_TIMEOUT_MS >= 100,
                  "Config::TURN_IMU_STALL_TIMEOUT_MS must leave room for transient retries — a near-"
                  "zero timeout would fault the turn on the first IMU read miss.");

    // ---------------------------------------------------------------------------------------
    // GYRO straight heading-hold PD (ADR-075, issue #169) — per-tick PD over Move(L,R) driven
    // by the external gyro (`YawSource.readYawDeg`), replacing the STM32 `MoveGyro*` write path.
    //
    // Ported from the mentor's proven `gyro_move` (`speed = Kp*err + Kd*delta`,
    // `Move(base +- speed)`), mirroring the shipped turn PD (ADR-068): pure decision math lives
    // in `GyroHoldPolicy.h`; these constants are the tunables `Drive::_emitGyroHold` feeds into
    // it. Unlike the turn PD this is PD + a never-reverse clamp ONLY — no creep floor, no slew
    // limiter, no integrator (a hold rides an already-moving base and drives corr -> 0; it does
    // not start from rest). Gains ship UNVERIFIED pending hardware bench — see ADR-075.
    // ---------------------------------------------------------------------------------------

    /**
     * @brief Proportional gain for the R4-side GYRO straight heading-hold PD (ADR-075), in PWM
     *        per degree of heading error.
     *
     * Ported from the mentor's proven `gyro_move` (`Kp = 1.5`), which maps a degree error
     * straight to a PWM correction, so it ports DIRECTLY — no loop-rate conversion, unlike
     * `GYRO_HOLD_KD` below. `GyroHoldPolicy.h::gyroHoldCorrection` computes `Kp * errNew` before
     * the D term and the never-reverse clamp.
     *
     * @warning **UNVERIFIED — pending hardware bench (AC9).** Carried over as a starting
     *          placeholder from the mentor's `gyro_move` reference — same UNVERIFIED status as
     *          #172's `GYRO_SCALE` before its `gyro_calib` bench run. Bench-tune by editing this
     *          constant and reflashing; no dedicated tuning sketch exists for the GYRO hold.
     * @note Consumer: `Drive::_emitGyroHold`, via `GyroHoldPolicy.h::gyroHoldCorrection`.
     */
#ifndef GYRO_HOLD_KP_DEFAULT
#define GYRO_HOLD_KP_DEFAULT 1.5f
#endif
    constexpr float GYRO_HOLD_KP = GYRO_HOLD_KP_DEFAULT;

    /**
     * @brief TIME-NORMALIZED derivative gain for the GYRO straight heading-hold PD (ADR-075), in
     *        PWM per (deg/s) — NOT the mentor's raw per-iteration gain.
     *
     * The mentor's `gyro_move` computes a raw per-ITERATION difference (no division by loop
     * period), so his continuous damping is `Kd_raw * loop_period` — correct only at his fixed
     * ~6 ms loop. Our cooperative tick is a MEASURED ~8 ms that jitters (see `TURN_KD` above and
     * the ~8-11.5 ms vs ~22.66 ms timing-doc inconsistency noted on `TURN_MIN_RESOLVABLE_DEG`),
     * so porting the raw gain verbatim would silently re-scale every time the tick jitters.
     * `GyroHoldPolicy.h::gyroHoldCorrection` (via `TurnPDPolicy.h::turnPdStep`) instead divides by
     * the MEASURED per-step `dtSec`, so this gain carries real units of PWM per (deg/s):
     * `GYRO_HOLD_KD = Kd_raw * mentor_loop_period ~= 8 * 0.006 s ~= 0.048`.
     *
     * @warning **UNVERIFIED — pending hardware bench (AC9).** Same placeholder status as
     *          `GYRO_HOLD_KP` above; bench-tune via Config edit + reflash.
     * @note Consumer: `Drive::_emitGyroHold`, via `GyroHoldPolicy.h::gyroHoldCorrection`.
     */
#ifndef GYRO_HOLD_KD_DEFAULT
#define GYRO_HOLD_KD_DEFAULT 0.048f
#endif
    constexpr float GYRO_HOLD_KD = GYRO_HOLD_KD_DEFAULT;

    /**
     * @brief Static ceiling (PWM magnitude) on the GYRO heading-hold's per-tick correction
     *        (ADR-075).
     *
     * The correction actually applied is dynamically clamped to
     * `min(GYRO_HOLD_MAX_CORR_PWM, |base|)` — this constant is only the STATIC half of that
     * clamp. The dynamic `|base|` half matters because `base` shrinks through the profile's
     * decel tail below any static knob; a static-only clamp would let the correction exceed the
     * base and flip the inner wheel's direction exactly when the chassis is slowest to recover.
     * See `GyroHoldPolicy.h::gyroHoldWheels` for the never-reverse wheel mix this feeds.
     *
     * Deviates from the mentor's `gyro_move`, which runs unclamped. Set generous (60) so the
     * clamp rarely binds pending bench data on how large a correction the chassis actually needs.
     *
     * @note TODO: bench-tune (AC9) once hardware is in hand.
     * @note Consumer: `Drive::_emitGyroHold`, via `GyroHoldPolicy.h::gyroHoldCorrection`.
     */
#ifndef GYRO_HOLD_MAX_CORR_PWM_DEFAULT
#define GYRO_HOLD_MAX_CORR_PWM_DEFAULT 60
#endif
    constexpr int16_t GYRO_HOLD_MAX_CORR_PWM = GYRO_HOLD_MAX_CORR_PWM_DEFAULT;

    /**
     * @brief Staleness threshold (ms) for the GYRO heading-hold's previous-yaw sample (ADR-075).
     *
     * `Drive::_emitGyroHold` computes its fold-safe derivative-on-measurement term from the
     * previously stored yaw (`_gyroHoldPrevYaw` / `_gyroHoldPrevMs`). If that sample is older
     * than this threshold — the first tick of a hold (`_gyroHoldHasPrev == false`), or a gap left
     * by one or more `readYawDeg` failures — the derivative term is forced to 0 for that tick
     * (P-only command) instead of computing a rate over a stale, possibly multi-tick gap.
     *
     * Sized comfortably above the measured ~8-11.5 ms cooperative tick so a single ordinary tick
     * never falsely trips staleness, while still catching a genuine multi-tick gyro dropout.
     *
     * @note TODO: bench-tune (AC9) once hardware is in hand.
     * @note Consumer: `Drive::_emitGyroHold`.
     */
#ifndef GYRO_HOLD_STALE_MS_DEFAULT
#define GYRO_HOLD_STALE_MS_DEFAULT 50
#endif
    constexpr uint16_t GYRO_HOLD_STALE_MS = GYRO_HOLD_STALE_MS_DEFAULT;

    // =========================================================================================
    // Motor-bias calibration (Strategy B from Phase 1.5 chassis-API research).
    //
    // DC drive motors of the same model can vary 10-20% in RPM at the same PWM (this is true for
    // both `MatrixTT` and `LegoEV3`). Our
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
    // 50% power for 2 s gives a few thousand encoder counts on either profile (PPR 360 for
    // LegoEV3, 545 for MatrixTT) — plenty of signal, not enough to spin the chassis off a
    // bench in wheels-up mode.

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
    constexpr float TSHAPE_ACCEL_PWM_PER_CM = 4.0f;

    /** @brief Floor on the per-side ramp distance — never shorter than this (cm). */
    constexpr float TSHAPE_RAMP_MIN_CM = 5.0f;

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
    constexpr int16_t TSHAPE_NO_RAMP_EPSILON_PWM = 5;

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
    constexpr float TSHAPE_MIN_MOVE_SEC = 0.2f;

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
}    // namespace Config

#endif    // CONFIG_DRIVE_H
