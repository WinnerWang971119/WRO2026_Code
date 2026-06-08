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
    constexpr uint8_t GRABBER_RC_ID = 1;   // TODO confirm wiring at bring-up

    /**
     * @brief RC-bus channel for the sorter servo.
     * @note Override-hook: the default initializer is guarded by
     *       `#ifndef SORTER_RC_ID_DEFAULT` so the compile-fail ctest can force a
     *       servo-vs-servo ID collision via `-DSORTER_RC_ID_DEFAULT=...` without
     *       macro-corrupting the `constexpr` (mirrors `Config.h`'s
     *       `PATH_PASS_THROUGH_ENABLED_DEFAULT` idiom).
     */
#ifndef SORTER_RC_ID_DEFAULT
#define SORTER_RC_ID_DEFAULT 2
#endif
    constexpr uint8_t SORTER_RC_ID = SORTER_RC_ID_DEFAULT;

    /** @brief RC-bus channel for the dumper servo. TODO confirm wiring at bring-up. */
    constexpr uint8_t DUMPER_RC_ID = 3;   // TODO confirm wiring at bring-up

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
    constexpr ServoCfg GRABBER = { GRABBER_RC_ID, 90, 30, 150, 2.0f, 80, false };  // TODO calibrate
    /** @brief Dumper servo calibration. TODO calibrate. */
    constexpr ServoCfg DUMPER  = { DUMPER_RC_ID,  90, 90, 150, 2.0f, 80, false };  // TODO calibrate
    /** @brief Sorter servo calibration; open/close poses unused (bins drive the angle). TODO. */
    constexpr ServoCfg SORTER  = { SORTER_RC_ID,  90,  0,   0, 2.0f, 80, false };  // open/close unused; TODO

    /** @brief Number of discrete sorter bins. TODO finalize. */
    constexpr uint8_t  SORTER_BIN_COUNT = 3;                          // TODO finalize
    /** @brief Sorter servo angle (degrees) for each bin index. TODO calibrate. */
    constexpr uint16_t SORTER_BIN_ANGLE[SORTER_BIN_COUNT] = { 30, 90, 150 };  // TODO calibrate

    // Belt sign convention: +speed drives toward FAR_END, -speed drives toward HOME.

    /** @brief Belt encoder pulses per revolution (LEGO EV3). TODO confirm. */
    constexpr uint16_t BELT_PPR             = 360;    // LEGO EV3; TODO confirm
    /** @brief Belt free-run speed spec (RPM). TODO confirm. */
    constexpr uint16_t BELT_RPM             = 300;    // TODO confirm
    /** @brief Magnitude of a POSITION-mode belt move (PWM). TODO. */
    constexpr int16_t  BELT_MOVE_SPEED      = 40;     // POSITION-move magnitude; TODO
    /** @brief Magnitude of the stall-home drive toward HOME (PWM). TODO. */
    constexpr int16_t  BELT_HOME_SPEED      = 25;     // stall-home magnitude (toward HOME); TODO
    /** @brief Timed run length of the stall-home maneuver (ms). TODO. */
    constexpr uint32_t BELT_HOME_MS         = 1500;   // stall-home timed run; TODO
    /** @brief Wall-clock ceiling for the `begin()` homing routine (ms). */
    constexpr uint32_t BELT_HOME_TIMEOUT_MS = BELT_HOME_MS + 1000;  // wall-clock ceiling for begin() home
    /** @brief Watchdog ceiling for a POSITION-mode belt move (ms). TODO. */
    constexpr uint32_t BELT_MOVE_TIMEOUT_MS = 4000;   // POSITION-move watchdog; TODO

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

    /** @brief Logical belt stations, indexing `BELT_STATIONS`. TODO finalize. */
    enum class BeltStation : uint8_t { HOME = 0, MID = 1, FAR_END = 2 };  // TODO finalize
    /** @brief Number of belt stations (length of `BELT_STATIONS`). */
    constexpr uint8_t BELT_STATION_COUNT = 3;
    /** @brief Per-station belt command table, indexed by `BeltStation`. TODO. */
    constexpr BeltStationCfg BELT_STATIONS[BELT_STATION_COUNT] = {
        { BeltMode::TIMED,    0,   BELT_HOME_MS, -BELT_HOME_SPEED },  // HOME  (timed into near end); TODO
        { BeltMode::POSITION, 400, 0,             BELT_MOVE_SPEED },  // MID   (absolute 400); TODO
        { BeltMode::TIMED,    0,   1500,          BELT_MOVE_SPEED },  // FAR_END (stalls -> timed); TODO
    };
}    // namespace Config

#endif    // CONFIG_MANIPULATOR_H
