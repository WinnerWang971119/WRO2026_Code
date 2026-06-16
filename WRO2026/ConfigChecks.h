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

    /**
     * @brief Constexpr helper: returns true when every entry in HUE_BANDS satisfies
     *        `lo <= hi && hi <= 239` (the chromatic range is 0..239).
     *
     * Loops `sizeof(HUE_BANDS)/sizeof(HUE_BANDS[0])` so adding or removing a band at bench T8
     * needs no edit here — replaces the hand-unrolled `HUE_BANDS[0..2]` check. Placed beside its
     * static_assert, matching how the other compile-time helpers sit next to their validators.
     */
    constexpr bool _visionBandsValid()
    {
        for (size_t b = 0; b < sizeof(HUE_BANDS) / sizeof(HUE_BANDS[0]); ++b)
            if (!(HUE_BANDS[b].lo <= HUE_BANDS[b].hi && HUE_BANDS[b].hi <= 239))
                return false;
        return true;
    }
    static_assert(_visionBandsValid(),
                  "Config::HUE_BANDS: every band must satisfy lo <= hi and hi <= 239 (chromatic range).");

    // VISION_CELL_COUNT must not exceed the number of hardware slots.
    static_assert(VISION_CELL_COUNT <= VISION_SLOT_COUNT,
                  "Config::VISION_CELL_COUNT must be <= VISION_SLOT_COUNT.");

    // VISION_WHITE_STA_MAX must be representable in the sta field range [0..15].
    static_assert(VISION_WHITE_STA_MAX <= 15,
                  "Config::VISION_WHITE_STA_MAX must be <= 15 (sta field is 0..15).");

    // -----------------------------------------------------------------------------------------
    // Vision device-shape coupling guards (defence-in-depth).
    //
    // The HUE12 is fixed at 12 detection slots, and the firmware leans on three relationships
    // between VISION_CELL_COUNT, VISION_SLOT_COUNT, and VISION_FRAME_WORDS. Each is consumed at a
    // call site that would silently mis-behave (OOB read, wrong packing) if the constants drifted,
    // so pin them here with a static_assert tripwire next to the list of coupled sites.
    // -----------------------------------------------------------------------------------------

    // COUPLED SITES: Vision.cpp's readMosaic() / _classifyCandidate() size the local hue/sta/
    // colour arrays by VISION_CELL_COUNT but index them through VISION_CELL_TO_SLOT[c], whose
    // entries are bounded by VISION_SLOT_COUNT. If the two counts diverge, a slot index could
    // land past the CELL-sized arrays. Pinning them equal closes that hue[slot]/sta[slot] OOB.
    static_assert(VISION_CELL_COUNT == VISION_SLOT_COUNT,
                  "Config::VISION_CELL_COUNT must equal VISION_SLOT_COUNT: Vision.cpp sizes "
                  "per-cell arrays by CELL_COUNT but indexes them by slot (bounded by SLOT_COUNT).");

    // COUPLED SITE: VisionFrame.h's unpackFrame() packs two hues per word (low byte = even cell,
    // high byte = odd cell), so it assumes an even cell count — an odd count would leave the final
    // hue's high byte unwritten / off-by-one.
    static_assert(VISION_CELL_COUNT % 2 == 0,
                  "Config::VISION_CELL_COUNT must be even: VisionFrame.h::unpackFrame packs two "
                  "hues per word (low/high byte).");

    // COUPLED SITE: VisionFrame.h's unpackFrame() reads words[] using indices derived from
    // VISION_CELL_COUNT — hue words = ceil(cells/2), sta words = ceil(cells/4). The wire framing
    // count VISION_FRAME_WORDS MUST equal their sum or unpackFrame would read past words[] (or
    // leave trailing cells unfilled). For 12 cells that is 6 + 3 = 9.
    static_assert(VISION_FRAME_WORDS == (VISION_CELL_COUNT + 1) / 2 + (VISION_CELL_COUNT + 3) / 4,
                  "Config::VISION_FRAME_WORDS must equal ceil(CELL_COUNT/2) + ceil(CELL_COUNT/4): "
                  "VisionFrame.h::unpackFrame indexes words[] by counts derived from CELL_COUNT.");

    /**
     * @brief Constexpr helper: returns true when every cell-to-slot mapping references a real
     *        hardware slot (`VISION_CELL_TO_SLOT[c] < VISION_SLOT_COUNT` for all cells).
     *
     * Loops `0..VISION_CELL_COUNT` — replaces the hand-unrolled `VISION_CELL_TO_SLOT[0..11]`
     * range check. Placed beside its static_assert, matching the other helpers in this file.
     */
    constexpr bool _visionSlotsInRange()
    {
        for (uint8_t c = 0; c < VISION_CELL_COUNT; ++c)
            if (!(VISION_CELL_TO_SLOT[c] < VISION_SLOT_COUNT))
                return false;
        return true;
    }
    static_assert(_visionSlotsInRange(),
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
