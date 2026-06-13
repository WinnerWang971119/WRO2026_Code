#ifndef CONFIG_CHECKS_H
#define CONFIG_CHECKS_H

#ifndef CONFIG_H
#error "ConfigChecks.h must be included via Config.h (it references Config drivetrain symbols)."
#endif

/**
 * @brief Compile-time validation for all per-domain Config sub-headers.
 *
 * Carved out of `Config.h` per ADR-031 / Issue #39 ("datacheck in another file").
 * Included LAST by `Config.h` so it sees the drivetrain symbols (MOTOR_LEFT_ID /
 * MOTOR_RIGHT_ID), the manipulator symbols (from ConfigManipulator.h), and the
 * vision symbols (from ConfigVision.h). The drive-pair asserts stay in `Config.h`;
 * do not move or duplicate them here.
 */
namespace Config
{
    // -----------------------------------------------------------------------------------------
    // Manipulator checks (ConfigManipulator.h)
    // -----------------------------------------------------------------------------------------

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

    // -----------------------------------------------------------------------------------------
    // Vision checks (ConfigVision.h)
    // -----------------------------------------------------------------------------------------

    // HUE_BANDS: each band must have lo <= hi and hi <= 239 (chromatic range is 0..239).
    static_assert(HUE_BANDS[0].lo <= HUE_BANDS[0].hi && HUE_BANDS[0].hi <= 239 &&
                  HUE_BANDS[1].lo <= HUE_BANDS[1].hi && HUE_BANDS[1].hi <= 239 &&
                  HUE_BANDS[2].lo <= HUE_BANDS[2].hi && HUE_BANDS[2].hi <= 239,
                  "Config::HUE_BANDS: every band must satisfy lo <= hi and hi <= 239 (chromatic range).");

    // VISION_CELL_COUNT must not exceed the number of hardware slots.
    static_assert(VISION_CELL_COUNT <= VISION_SLOT_COUNT,
                  "Config::VISION_CELL_COUNT must be <= VISION_SLOT_COUNT.");

    // VISION_WHITE_STA_MAX must be representable in the sta field range [0..15].
    static_assert(VISION_WHITE_STA_MAX <= 15,
                  "Config::VISION_WHITE_STA_MAX must be <= 15 (sta field is 0..15).");

    // Every cell-to-slot mapping must reference a real hardware slot.
    static_assert(VISION_CELL_TO_SLOT[0]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[1]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[2]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[3]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[4]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[5]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[6]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[7]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[8]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[9]  < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[10] < VISION_SLOT_COUNT &&
                  VISION_CELL_TO_SLOT[11] < VISION_SLOT_COUNT,
                  "Config::VISION_CELL_TO_SLOT: every slot index must be < VISION_SLOT_COUNT.");

    /**
     * @brief Constexpr helper: returns true when all VISION_CELL_COUNT entries in
     *        VISION_CELL_TO_SLOT are pairwise distinct.
     *
     * Placed here (not in ConfigVision.h) to stay beside the static_assert that
     * consumes it — matching how compile-time helpers sit next to their validators
     * in this file.
     */
    constexpr bool _visionSlotsUnique()
    {
        for (uint8_t i = 0; i < VISION_CELL_COUNT; ++i)
            for (uint8_t j = i + 1; j < VISION_CELL_COUNT; ++j)
                if (VISION_CELL_TO_SLOT[i] == VISION_CELL_TO_SLOT[j])
                    return false;
        return true;
    }
    static_assert(_visionSlotsUnique(),
                  "Config::VISION_CELL_TO_SLOT entries must be unique (no two cells share a slot).");

}    // namespace Config

#endif    // CONFIG_CHECKS_H
