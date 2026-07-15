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
 * vision symbols (from ConfigVision.h). The drive-pair + line asserts now live
 * here too (the `Config.h` umbrella body is empty after the per-domain split).
 */
namespace Config
{
    // -----------------------------------------------------------------------------------------
    // Drivetrain / line checks (ConfigDrive.h / ConfigLine.h)
    //
    // Relocated here from the old monolithic Config.h body. Their explanatory block comments /
    // @warning lines stay beside the constants they guard (in ConfigDrive.h / ConfigLine.h);
    // only the static_assert statements live here, since this header is included LAST and so
    // sees every per-domain symbol.
    // -----------------------------------------------------------------------------------------

    static_assert(MOTOR_LEFT_ID == 2,
                  "MOTOR_LEFT_ID is hardcoded as M2 throughout Drive.cpp/Drive.h. "
                  "Changing this constant alone produces a silently-broken chassis. "
                  "See the block comment above this assert and PR #20 codex P1 review.");
    static_assert(MOTOR_RIGHT_ID == 3,
                  "MOTOR_RIGHT_ID is hardcoded as M3 throughout Drive.cpp/Drive.h. "
                  "Changing this constant alone produces a silently-broken chassis. "
                  "See the block comment above this assert and PR #20 codex P1 review.");

    static_assert(LINE_BOARD_COUNT == 1 || LINE_BOARD_COUNT == 2,
                  "Config::LINE_BOARD_COUNT must be 1 or 2 — WROLineArrayClass has fixed [2][8] "
                  "storage and front/rear semantics. Update both this assertion AND the array "
                  "dimensions if a third board is ever added.");

    static_assert(LINE_FRONT_I2C_PORT >= 1 && LINE_FRONT_I2C_PORT <= 4,
                  "Config::LINE_FRONT_I2C_PORT must be in [1, 4] — matches the "
                  "physical I2C1..I2C4 port labels on the MiniR4 board. The "
                  "WROLineArrayClass converts to the 0-based PCA9548A mux channel "
                  "internally; do NOT pre-convert in Config.");

    static_assert(LINE_REAR_I2C_PORT >= 1 && LINE_REAR_I2C_PORT <= 4,
                  "Config::LINE_REAR_I2C_PORT must be in [1, 4] — matches the "
                  "physical I2C1..I2C4 port labels on the MiniR4 board. The "
                  "WROLineArrayClass converts to the 0-based PCA9548A mux channel "
                  "internally; do NOT pre-convert in Config.");

    static_assert(CRAWL_PWM >= PWM_MIN_MAGNITUDE && CRAWL_PWM <= PWM_MAX,
                  "Config::CRAWL_PWM must be in [PWM_MIN_MAGNITUDE, PWM_MAX] so the CRAWL "
                  "recovery branch never hits the deadband floor or saturation cap silently.");

    // PWM_MOVING_MIN is the kinetic decel-tail floor; it must never exceed the static breakaway
    // floor (PWM_MIN_MAGNITUDE) it substitutes for. Equal to PWM_MIN_MAGNITUDE disables the
    // feature (identity passthrough) — see the @warning on PWM_MOVING_MIN in ConfigGlobal.h.
    static_assert(PWM_MOVING_MIN > 0 && PWM_MOVING_MIN <= PWM_MIN_MAGNITUDE,
                  "PWM_MOVING_MIN must be in (0, PWM_MIN_MAGNITUDE]");

    // -----------------------------------------------------------------------------------------
    // Line-junction predicate checks (ConfigLine.h, issue #152).
    //
    // COUPLED SITE: WROLineArrayClass::getJunctionType() classifies against these three knobs
    // over the fixed 8-channel line array (valid channel indices 0..7). An out-of-range knob
    // would either reference a channel that does not exist or make a junction predicate
    // permanently unsatisfiable / trivially always-true. Each knob got an `#ifndef *_DEFAULT`
    // override hook in ConfigLine.h precisely so a compile-fail probe can trip these asserts.
    // -----------------------------------------------------------------------------------------

    static_assert(LINE_JUNCTION_LEFT_HI <= 7,
                  "Config::LINE_JUNCTION_LEFT_HI must be <= 7 -- it is an inclusive upper channel "
                  "index into the fixed 8-channel line array (valid indices run 0 through 7); a "
                  "higher value would test a channel that does not exist.");

    static_assert(LINE_JUNCTION_RIGHT_LO >= 1 && LINE_JUNCTION_RIGHT_LO <= 7,
                  "Config::LINE_JUNCTION_RIGHT_LO must be between 1 and 7 inclusive -- it is a "
                  "lower channel index into the fixed 8-channel line array, and the RIGHT "
                  "junction predicate treats ch[0..RIGHT_LO-1] as the required OFF-LINE side, so "
                  "0 would leave no off-line channel to check and a value above 7 would index "
                  "past the array.");

    static_assert(LINE_JUNCTION_T_MIN_ON >= 1 && LINE_JUNCTION_T_MIN_ON <= 8,
                  "Config::LINE_JUNCTION_T_MIN_ON must be between 1 and 8 inclusive -- it is a "
                  "channel-lit COUNT over the fixed 8-channel line array; 0 would declare a "
                  "T_OR_CROSS junction with zero channels lit and a count above 8 could never be "
                  "satisfied by an 8-channel array.");

    // -----------------------------------------------------------------------------------------
    // Turn-ladder invariant (ConfigDrive.h, issue #152).
    //
    // COUPLED SITE: Drive::_updateTurnProfile() drives at TURN_CRUISE_PWM until the heading
    // error falls within TURN_SLOW_ZONE_DEG of the target, ramps down to TURN_SLOW_SPEED_PWM,
    // and brakes once the error is within TURN_TOLERANCE_DEG. TURN_SLOW_SPEED_PWM is
    // INTENTIONALLY set below the motor deadband floor (PWM_MIN_MAGNITUDE) -- `deadbandSnap`
    // floors it back up to PWM_MIN_MAGNITUDE at emit time (see the corrected doc comment on
    // TURN_SLOW_SPEED_PWM in ConfigDrive.h) -- so this invariant deliberately does NOT assert
    // `TURN_SLOW_SPEED_PWM >= PWM_MIN_MAGNITUDE` (that would be false today: 20 < 35). Each
    // constant referenced below got an `#ifndef *_DEFAULT` override hook in ConfigDrive.h so a
    // compile-fail probe can trip each clause independently.
    // -----------------------------------------------------------------------------------------

    static_assert(TURN_SLOW_SPEED_PWM > 0 && TURN_SLOW_SPEED_PWM <= TURN_CRUISE_PWM &&
                  TURN_CRUISE_PWM <= PWM_MAX,
                  "Config::TURN_SLOW_SPEED_PWM/TURN_CRUISE_PWM/PWM_MAX turn-ladder ordering "
                  "violated -- require 0 < TURN_SLOW_SPEED_PWM <= TURN_CRUISE_PWM <= PWM_MAX so "
                  "the slow-zone ramp is a reduction from cruise, never an increase, and cruise "
                  "never exceeds the saturation cap.");

    static_assert(TURN_CRUISE_PWM >= PWM_MIN_MAGNITUDE,
                  "Config::TURN_CRUISE_PWM must be >= PWM_MIN_MAGNITUDE -- the fast turn phase "
                  "must clear the motor deadband floor or the chassis would never break static "
                  "friction during the cruise phase of a turn.");

    static_assert(TURN_TOLERANCE_DEG < TURN_SLOW_ZONE_DEG,
                  "Config::TURN_TOLERANCE_DEG must be < TURN_SLOW_ZONE_DEG -- the slow zone must "
                  "be strictly wider than the final tolerance gate or the TurnGyro state machine "
                  "would brake at TURN_CRUISE_PWM without ever ramping down through the slow "
                  "zone.");

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

    // -----------------------------------------------------------------------------------------
    // Localization / IMU checks (ConfigNav.h)
    //
    // COUPLED SITES: Config::IMU_YAW_SIGN is the per-chassis CW->CCW yaw sign applied by
    // YawSource::readYawDeg (raw STM32 read -> app frame) and YawSource::toHwHeading (app
    // heading -> STM32 gyro-PID target). It is a PURE SIGN, never a scale factor: every read
    // and every gyro-heading write multiplies by it exactly once, so any magnitude other than
    // +/-1 would silently scale all heading math. Pin it here next to the coupled call sites.
    // -----------------------------------------------------------------------------------------
    static_assert(IMU_YAW_SIGN == 1.0f || IMU_YAW_SIGN == -1.0f,
                  "Config::IMU_YAW_SIGN must be exactly 1.0f or -1.0f -- it is the pure "
                  "CW-to-CCW sign applied by YawSource::readYawDeg and YawSource::toHwHeading, "
                  "not a scale factor. See config/ConfigNav.h.");

    // -----------------------------------------------------------------------------------------
    // Line-arrival heading reset checks (ConfigNav.h, ADR-052)
    // -----------------------------------------------------------------------------------------
    static_assert(LINE_HEADING_RESET_TOLERANCE_DEG > 0.0f && LINE_HEADING_RESET_TOLERANCE_DEG < 180.0f,
                  "Config::LINE_HEADING_RESET_TOLERANCE_DEG must be in (0, 180) -- a zero or "
                  "negative tolerance would reject EVERY bind when the strict check is enabled "
                  "(no live IMU read lands bit-exact on the pre-bind belief), and the wrap-around "
                  "angular difference is bounded by 180 deg, so a tolerance >= 180 can never be "
                  "exceeded and the strict check would accept any heading, defeating it. See "
                  "config/ConfigNav.h.");

    // -----------------------------------------------------------------------------------------
    // Blind-chain continuous motion checks (ConfigNav.h, ADR-059)
    //
    // COUPLED SITES: PATH_BLIND_CHAIN_MAX_CM and MAX_CHAIN_HOPS both cap the same run seen by
    // PathTracker's walk-ahead chain builder and ChainSpeedPolicy.h's two-pass resolver; the
    // Python sim (tools/path_planner/sim/runtime.py) hand-mirrors all three constants.
    // PATH_BLIND_CHAIN_MAX_CM is a LOOSE runaway sanity guard only (owner decision, 2026-07) --
    // it is no longer tied to STOP_AT_LINE_ARM_WINDOW_CM. MAX_CHAIN_HOPS is the practical bound
    // on how long a chain can run; overflowing either cap ends the chain at an ordinary braked
    // stop (not a fault), so raising MAX_CHAIN_HOPS still needs the RAM check its own docstring
    // describes.
    // -----------------------------------------------------------------------------------------

    static_assert(PATH_BLIND_CHAIN_MAX_CM > 0.0f,
                  "Config::PATH_BLIND_CHAIN_MAX_CM must be positive — a zero or negative cap "
                  "would make every blind chain ineligible (or nonsensical), defeating the "
                  "ADR-059 continuous-motion feature entirely.");

    static_assert(PATH_BLIND_CHAIN_MAX_CM <= 10000.0f,
                  "Config::PATH_BLIND_CHAIN_MAX_CM must be <= 10000.0f cm -- this is a loose "
                  "runaway sanity bound, not a physical or arm-window-derived constraint (that "
                  "coupling was deliberately dropped; MAX_CHAIN_HOPS is the practical bound on "
                  "how far a chain can run). A cap this large or larger indicates a config typo "
                  "rather than an intentional value. See config/ConfigNav.h.");

    static_assert(MAX_CHAIN_HOPS >= 2 && MAX_CHAIN_HOPS <= 32,
                  "Config::MAX_CHAIN_HOPS must be in [2, 32] -- below 2 a \"chain\" can never "
                  "span more than one hop (the feature is unreachable), and this constant sizes "
                  "a FIXED firmware run buffer (no heap) inside PathTracker, so an upper bound "
                  "keeps its per-instance RAM footprint bounded on an Arduino Uno R4. Raising "
                  "past 32 requires re-checking available RAM, not just bumping this literal. "
                  "See config/ConfigNav.h.");

    // -----------------------------------------------------------------------------------------
    // Hop-settle wait checks (ConfigNav.h, ADR-061)
    //
    // COUPLED SITE: Config::HOP_SETTLE_MS is held once per braked hop (and again per cold
    // turn->move seam) by PathTracker's non-blocking State::SETTLE before the FSM dispatches
    // the next maneuver. Every settle dwell on a goal's braked hops adds against the 60 s
    // PATH_DRAIN_TIMEOUT_MS_PER_GOAL drain budget that bounds PathTracker::_drainBlocking and
    // Mission's isIdle() poll -- a settle value large enough to threaten that budget on a
    // multi-hop goal is a config typo, not an intentional tuning value.
    // -----------------------------------------------------------------------------------------

    static_assert(HOP_SETTLE_MS <= 2000,
                  "Config::HOP_SETTLE_MS must be <= 2000 ms -- a settle dwell longer than "
                  "~2 s per braked hop would eat into the 60 s PATH_DRAIN_TIMEOUT_MS_PER_GOAL "
                  "drain budget on a multi-hop goal and indicates a config typo rather than an "
                  "intentional value. See config/ConfigNav.h.");

}    // namespace Config

#endif    // CONFIG_CHECKS_H
