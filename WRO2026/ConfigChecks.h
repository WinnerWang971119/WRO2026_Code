#ifndef CONFIG_CHECKS_H
#define CONFIG_CHECKS_H

#ifndef CONFIG_H
#error "ConfigChecks.h must be included via Config.h (it references Config drivetrain symbols)."
#endif

/**
 * @brief Compile-time validation for the manipulator config (ConfigManipulator.h).
 *
 * Carved out of `Config.h` per ADR-031 / Issue #39 ("datacheck in another file").
 * Holds ONLY the NEW manipulator `static_assert`s. It is included LAST by `Config.h`
 * so it sees both the manipulator symbols (GRABBER_RC_ID, BELT_M_ID, the ServoCfg
 * tables, …) and the drivetrain symbols (MOTOR_LEFT_ID / MOTOR_RIGHT_ID from
 * `Config.h`'s body). The drive-pair asserts stay in `Config.h`; do not move or
 * duplicate them here.
 */
namespace Config
{
    // Servo RC IDs must address real RC channels.
    static_assert(GRABBER_RC_ID >= 1 && GRABBER_RC_ID <= 4 &&
                  SORTER_RC_ID  >= 1 && SORTER_RC_ID  <= 4 &&
                  DUMPER_RC_ID  >= 1 && DUMPER_RC_ID  <= 4,
                  "Manipulator servo RC IDs must be in [1,4].");
    // Two servos sharing an RC channel would fight; guard against it.
    static_assert(GRABBER_RC_ID != SORTER_RC_ID && GRABBER_RC_ID != DUMPER_RC_ID &&
                  SORTER_RC_ID  != DUMPER_RC_ID,
                  "Manipulator servo RC IDs must be pairwise distinct.");
    // Belt motor must address a real M channel...
    static_assert(BELT_M_ID >= 1 && BELT_M_ID <= 4, "BELT_M_ID must be in [1,4].");
    // ...and must NOT collide with the differential-drive pair (M2/M3).
    static_assert(BELT_M_ID != MOTOR_LEFT_ID && BELT_M_ID != MOTOR_RIGHT_ID,
                  "BELT_M_ID collides with a drive motor (MOTOR_LEFT_ID/MOTOR_RIGHT_ID).");
    // Table sizes must be non-empty.
    static_assert(SORTER_BIN_COUNT >= 1, "SORTER_BIN_COUNT must be >= 1.");
    static_assert(BELT_STATION_COUNT >= 1, "BELT_STATION_COUNT must be >= 1.");
    // Servo angles must be within the 0..180 RC range.
    static_assert(GRABBER.openDeg <= 180 && GRABBER.closeDeg <= 180 && GRABBER.homeDeg <= 180 &&
                  DUMPER.openDeg  <= 180 && DUMPER.closeDeg  <= 180 && DUMPER.homeDeg  <= 180 &&
                  SORTER.homeDeg  <= 180,
                  "Manipulator servo angles must be <= 180.");
}    // namespace Config

#endif    // CONFIG_CHECKS_H
