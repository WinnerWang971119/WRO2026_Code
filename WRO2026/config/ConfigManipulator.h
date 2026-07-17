#ifndef CONFIG_MANIPULATOR_H
#define CONFIG_MANIPULATOR_H

#include <stdint.h>

/**
 * @brief Per-actuator manipulator configuration, carved out of `Config.h` per
 *        ADR-031 / Issue #39.
 *
 * This header reopens `namespace Config` and holds the channel IDs, servo
 * calibration, sorter-bin angles and conveyor-belt station table for the
 * manipulator subsystem. It is self-contained (only `<stdint.h>`) and is
 * pulled in by `Config.h` (a later task wires the `#include`), so downstream
 * modules continue to source these symbols from the single `Config::` namespace
 * exactly as before the split.
 *
 * @note **Every value here is a bring-up placeholder.** Each constant carries a
 *       `TODO: measure at bring-up` (abbreviated `TODO ...`) annotation — do not
 *       treat any of these numbers as final until they have been measured on the
 *       physical robot.
 * @note Collision safety (servo-vs-servo on the RC bus, belt-vs-drive-pair on the
 *       M bus) is enforced separately in `ConfigChecks.h`; this header only
 *       declares the IDs.
 */
namespace Config
{
    // Channel IDs. Servos are on the RC bus, the belt on the M bus — the RC and M id
    // spaces are INDEPENDENT, so a servo on RC1 and the belt on M1 do NOT collide. Only
    // servo-vs-servo (RC) and belt-vs-drive-pair (M2/M3) collisions matter (enforced in
    // ConfigChecks.h). The *_DEFAULT hooks let the compile-fail ctest force a collision
    // via -D without macro-corrupting a constexpr (mirrors Config.h:926/942).

    /** @brief RC-bus channel for the grabber servo. TODO confirm wiring at bring-up. */
    constexpr uint8_t GRABBER_RC_ID = 2;   // TODO confirm wiring at bring-up

    /**
     * @brief RC-bus channel for the sorter servo.
     * @note Override-hook: the default initializer is guarded by
     *       `#ifndef SORTER_RC_ID_DEFAULT` so the compile-fail ctest can force a
     *       servo-vs-servo ID collision via `-DSORTER_RC_ID_DEFAULT=...` without
     *       macro-corrupting the `constexpr` (mirrors `Config.h`'s
     *       `PATH_PASS_THROUGH_ENABLED_DEFAULT` idiom).
     */
#ifndef SORTER_RC_ID_DEFAULT
#define SORTER_RC_ID_DEFAULT 4
#endif
    constexpr uint8_t SORTER_RC_ID = SORTER_RC_ID_DEFAULT;

    /** @brief RC-bus channel for the dumper servo. TODO confirm wiring at bring-up. */
    constexpr uint8_t DUMPER_RC_ID = 1;   // TODO confirm wiring at bring-up

    /**
     * @brief M-bus channel for the conveyor-belt motor.
     * @note Override-hook: guarded by `#ifndef BELT_M_ID_DEFAULT` so the
     *       compile-fail ctest can force a belt-vs-drive-pair collision via
     *       `-DBELT_M_ID_DEFAULT=...` (same idiom as `SORTER_RC_ID`). Must differ
     *       from `MOTOR_LEFT_ID(2)` / `MOTOR_RIGHT_ID(3)`.
     */
#ifndef BELT_M_ID_DEFAULT
#define BELT_M_ID_DEFAULT 1
#endif
    constexpr uint8_t BELT_M_ID = BELT_M_ID_DEFAULT;   // must differ from MOTOR_LEFT_ID(2)/MOTOR_RIGHT_ID(3)

    /**
     * @brief Per-servo calibration: channel, key poses, slew rate and settle time.
     * @note `reverse` is a HARDWARE direction flag forwarded to
     *       `MiniR4RC::setHWDir(reverse)`; it is NOT a software angle negation.
     */
    struct ServoCfg
    {
        uint8_t  rcId;       ///< RC-bus channel this servo is wired to.
        uint16_t homeDeg;    ///< Rest / start angle (degrees).
        uint16_t openDeg;    ///< "Open" pose angle (degrees).
        uint16_t closeDeg;   ///< "Close" pose angle (degrees).
        float    msPerDeg;   ///< Slew estimate (ms per degree) for move-time budgeting.
        uint16_t settleMs;   ///< Extra dwell after the slew completes (ms).
        bool     reverse;    ///< HARDWARE direction flag -> MiniR4RC::setHWDir(reverse); NOT software.
    };

    /** @brief Grabber servo calibration. TODO calibrate. */
    constexpr ServoCfg GRABBER = { GRABBER_RC_ID, 125, 5, 105, 7.0f, 80, true };  
    /** @brief Dumper servo calibration. TODO calibrate. */
    constexpr ServoCfg DUMPER  = { DUMPER_RC_ID,  78, 33, 18, 5.0f, 80, false };  
    /** @brief Sorter servo calibration; open/close poses unused (bins drive the angle). TODO. */
    constexpr ServoCfg SORTER  = { SORTER_RC_ID,  90,  0,   0, 5.2f, 95, false };  // open/close unused; 

    /** @brief Number of discrete sorter bins. TODO finalize. */
    constexpr uint8_t  SORTER_BIN_COUNT = 4;                          
    /** @brief Sorter servo angle (degrees) for each bin index. TODO calibrate. */
    constexpr uint16_t SORTER_BIN_ANGLE[SORTER_BIN_COUNT] = { 55, 76, 102, 126};  

    // Belt sign convention: +speed drives toward FAR_END, -speed drives toward HOME.

    /**
     * @brief Belt motor direction sign (`+1` or `-1`). Applied ONCE at the `ManipBelt`
     *        driver boundary — the `_setSpeed` / `_rotateFor` speed argument and the
     *        `_getDegrees` read — so the sign convention above holds no matter how the
     *        belt motor is wired (ADR-045 style single-point sign, like `IMU_YAW_SIGN`).
     * @note  This is the LOGICAL-direction knob (which physical end is FAR_END). It is a
     *        different axis from `BELT_MOTOR_REVERSE`, which fixes motor/encoder AGREEMENT so
     *        the closed-loop `rotateFor` can complete. On this build both come out such that
     *        `BELT_DIR_SIGN = +1` and `BELT_MOTOR_REVERSE = true` home the belt correctly.
     */
    constexpr int8_t   BELT_DIR_SIGN        = +1;    // logical +speed = FAR_END on this build

    /**
     * @brief Reverse the belt motor's power direction (`setReverse` / `SetDCMotorDir`).
     *
     * The belt encoder counts OPPOSITE to the motor's power on this build, so the lower MCU's
     * closed-loop `rotateFor` (POSITION moves) saw the count run away from target and hung to
     * the `BELT_MOVE_TIMEOUT_MS` watchdog. Reversing the MOTOR (the working `SetDCMotorDir`
     * half of `setReverse`) brings motor + encoder back into agreement so `rotateFor`
     * completes — verified on hardware via `inoTest/belt_encoder_probe`. `SetEncoderDir`
     * (flip the encoder instead) does NOT work: the STM32 rotate controller ignores it, which
     * is why the vendor left it commented out of `setReverse`.
     * @note `true` on the current belt (measured: +motor command drove the count negative).
     */
    constexpr bool     BELT_MOTOR_REVERSE   = true;    // flip motor to agree with the encoder

    /** @brief Belt encoder pulses per revolution (LEGO EV3). TODO confirm. */
    constexpr uint16_t BELT_PPR             = 360;    // LEGO EV3; TODO confirm
    /** @brief Belt free-run speed spec (RPM). TODO confirm. */
    constexpr uint16_t BELT_RPM             = 300;    // TODO confirm
    /** @brief Magnitude of a POSITION-mode belt move (PWM). TODO. */
    constexpr int16_t  BELT_MOVE_SPEED      = 55;     // POSITION-move magnitude; TODO
    /** @brief Magnitude of the stall-home drive toward HOME (PWM). TODO. */
    constexpr int16_t  BELT_STALL_SPEED      = 15;     // stall-home magnitude (toward HOME); TODO
    /** @brief Timed run length of the stall-home maneuver (ms). TODO. */
    constexpr uint32_t BELT_RESET_MS         = 2000;   // stall-home timed run; TODO
    /**
     * @brief Wall-clock ceiling for the servo settle-home poll in `ManipulatorClass::begin()`
     *        (grabber/sorter/dumper). NOT the belt home ceiling: `ManipBelt::begin()` bounds its
     *        full stall+reset+settle+MID home with `BELT_HOME_TIMEOUT_MS`. Sized off the belt
     *        reset timing (+1 s margin) and reused here as a generic boot-home backstop.
     */
    constexpr uint32_t BELT_RESET_TIMEOUT_MS = BELT_RESET_MS + 1000;  // servo settle-home ceiling
    /** @brief Watchdog ceiling for a POSITION-mode belt move (ms). TODO. */
    constexpr uint32_t BELT_MOVE_TIMEOUT_MS = 4000;   // POSITION-move watchdog; TODO
    /**
     * @brief Settle dwell between the home stall (which is braked to a stop) and the chained
     *        MID move. Gives the lower MCU a clean tick gap to switch from the stall's
     *        `setSpeed()` drive into the `rotateFor()` position move — mirroring how ordinary
     *        back-to-back moves are separated across ticks. TODO tune at bring-up.
     */
    constexpr uint32_t BELT_HOME_SETTLE_MS = 200;   // dwell after the stall before the MID rotate; TODO
    /**
     * @brief Wall-clock ceiling for the FULL `home()` sequence: stall into the LEFT/HOME
     *        end-stop, re-zero the encoder, settle, then drive to the MID rest position.
     * @note Must exceed `BELT_RESET_MS` (stall) + `BELT_HOME_SETTLE_MS` (settle) +
     *       `BELT_MOVE_TIMEOUT_MS` (the MID POSITION move's own watchdog), since `home()`
     *       chains all three. Used by `ManipBelt::begin()`'s boot poll and `Manipulator::_drain()`.
     */
    constexpr uint32_t BELT_HOME_TIMEOUT_MS =
        BELT_RESET_MS + BELT_HOME_SETTLE_MS + BELT_MOVE_TIMEOUT_MS + 1000;  // stall + settle + MID move + margin

    /**
     * @brief Backstop ceiling for a servo-only blocking drain (no belt in the mask).
     *
     * Sized to comfortably exceed the largest single servo sweep
     * (max msPerDeg * 180 + settleMs ≈ 1.1 s). Belt-masked drains use the longer
     * BELT_HOME_TIMEOUT_MS instead, since the belt's home() chains stall+reset+MID.
     * @note `grab()` drains a TWO-stop stroke (grabber home->close via
     *       `ManipServo::homeThenClose`), so this must exceed TWICE the grabber's largest sweep
     *       (worst case ≈ 2 * (GRABBER close-vs-home * msPerDeg + settleMs) ≈ 1.7 s today).
     *       Keep that margin when tuning `GRABBER.msPerDeg` / `settleMs` at bring-up.
     */
    constexpr uint32_t MANIP_SERVO_DRAIN_TIMEOUT_MS = 2000;

    /** @brief How a belt station is reached: by absolute encoder target or by a timed run. */
    enum class BeltMode : uint8_t { POSITION, TIMED };

    /** @brief Per-station belt command: mode plus the target/duration/speed it consumes. */
    struct BeltStationCfg
    {
        BeltMode mode;    ///< POSITION (absolute encoder target) or TIMED (run duration).
        int32_t  deg;     ///< POSITION: ABSOLUTE encoder target vs home zero (toStation reads getDegrees, issues signed relative rotateFor).
        uint32_t ms;      ///< TIMED: run duration (ms).
        int16_t  speed;   ///< signed: +toward FAR_END, -toward HOME (magnitude = PWM).
    };

    /**
     * @brief Logical belt stations, indexing `BELT_STATIONS`. TODO finalize.
     * @note `LEFT` is the near/reset end-stop (the HOME end where `home()` stall-zeroes
     *       the encoder); `MID` is the operating rest position; `FAR_END` is the opposite
     *       hard stop. `LEFT` is named for its physical side — it sits at the belt's HOME
     *       end, so its row drives toward HOME (`-BELT_STALL_SPEED`).
     */
    enum class BeltStation : uint8_t { LEFT = 0, MID = 1, FAR_END = 2 , REAR_BOX_OPEN = 3, REAR_BOX_CLOSE = 4};  // TODO finalize
    /** @brief Number of belt stations (length of `BELT_STATIONS`). */
    constexpr uint8_t BELT_STATION_COUNT = 5;
    /** @brief Per-station belt command table, indexed by `BeltStation`. TODO. */
    constexpr BeltStationCfg BELT_STATIONS[BELT_STATION_COUNT] = {
        { BeltMode::TIMED,    0,   BELT_RESET_MS, -BELT_STALL_SPEED },  // LEFT  (timed into near/HOME end); TODO
        { BeltMode::POSITION, 700, 0,             BELT_MOVE_SPEED },  // MID   (absolute 400); TODO
        { BeltMode::TIMED,    0,   1500,          BELT_STALL_SPEED },  // FAR_END (stalls -> timed); TODO
        { BeltMode::POSITION,    200,   0,          BELT_MOVE_SPEED },  // REAR_BOX_OPEN (pos); TODO
        { BeltMode::POSITION,    100,   0,          BELT_MOVE_SPEED },  // REAR_BOX_CLOSE (pos); TODO
    };
}    // namespace Config

#endif    // CONFIG_MANIPULATOR_H
