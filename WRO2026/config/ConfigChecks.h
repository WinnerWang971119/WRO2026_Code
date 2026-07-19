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

    static_assert(LINE_FOLLOW_OFFSET_ERROR >= 0.0f && LINE_FOLLOW_OFFSET_ERROR <= 2.5f,
                  "Config::LINE_FOLLOW_OFFSET_ERROR must stay in [0,2.5] error units (>=1 unit "
                  "margin before the outer channel; LINE_KP*offset stays under LINE_MAX_CORR=40).");

    // PWM_MOVING_MIN is the kinetic decel-tail floor; it must never exceed the static breakaway
    // floor (PWM_MIN_MAGNITUDE) it substitutes for. Equal to PWM_MIN_MAGNITUDE disables the
    // feature (identity passthrough) — see the @warning on PWM_MOVING_MIN in ConfigGlobal.h.
    static_assert(PWM_MOVING_MIN > 0 && PWM_MOVING_MIN <= PWM_MIN_MAGNITUDE,
                  "PWM_MOVING_MIN must be in (0, PWM_MIN_MAGNITUDE]");

    // -----------------------------------------------------------------------------------------
    // Turn PD controller checks (ConfigDrive.h, ADR-068)
    //
    // `Drive::_updateTurnProfile` (via `TurnPDPolicy.h`) is a per-tick PD + kinetic creep floor
    // + slew-limited ramp-up. These eight asserts pin the physical/dynamical relationships
    // between the PD knobs so a future retune of any ONE of them cannot silently desync from
    // the others and produce a turn that hunts to its timeout on every dispatch (the plan's
    // single sharpest failure mode — see docs/plans/2026-07-16-turn-pd-controller.md).
    // -----------------------------------------------------------------------------------------

    // (a) The settle band must be at least one physical step wide, or NO sample can ever land
    // in-band and the turn can never exit via the debounce (only via TURN_TIMEOUT_MS).
    static_assert(TURN_TOLERANCE_DEG >= TURN_MIN_RESOLVABLE_DEG,
                  "Config::TURN_TOLERANCE_DEG must be >= TURN_MIN_RESOLVABLE_DEG — a settle band "
                  "narrower than the bench-measured physical step size (omega_at_creep * tick) "
                  "can never contain a sample, so the turn would always hunt to TURN_TIMEOUT_MS.");

    // (b) LOAD-BEARING: enough physical steps must land in-band per crossing for the
    // TURN_SETTLE_SAMPLES-consecutive-sample debounce to ever be satisfiable. At
    // TOLERANCE=0.75, MIN_RESOLVABLE=0.4423, SETTLE=2 this is (2*0.75)/0.4423 ~= 3.39 > 2, a
    // 1.70x margin. Converts the plan's sharpest failure mode (every turn silently hunting to
    // timeout, since timeout is non-faulting by design) into a build error.
    static_assert((2.0f * TURN_TOLERANCE_DEG) / TURN_MIN_RESOLVABLE_DEG > TURN_SETTLE_SAMPLES,
                  "Config::TURN_TOLERANCE_DEG / TURN_MIN_RESOLVABLE_DEG / TURN_SETTLE_SAMPLES are "
                  "desynced: (2 * TURN_TOLERANCE_DEG) / TURN_MIN_RESOLVABLE_DEG must exceed "
                  "TURN_SETTLE_SAMPLES, or fewer physical steps land in-band per crossing than "
                  "the debounce requires and EVERY turn silently hunts to TURN_TIMEOUT_MS. Widen "
                  "TURN_TOLERANCE_DEG, lower TURN_SETTLE_SAMPLES, or re-bench a lower creep PWM.");

    // (c)/(d) The creep floor must sit strictly below its own hand-off threshold, and must be a
    // real, non-zero push — a zero or negative creep floor could never sustain rotation.
    static_assert(TURN_CREEP_PWM < TURN_CREEP_THRESH_PWM,
                  "Config::TURN_CREEP_PWM must be < TURN_CREEP_THRESH_PWM — otherwise the PD "
                  "would hand off to a creep command it could already command unfloored, making "
                  "TURN_CREEP_THRESH_PWM's hand-off band meaningless.");
    static_assert(TURN_CREEP_PWM > 0,
                  "Config::TURN_CREEP_PWM must be > 0 — a zero or negative creep floor could "
                  "never sustain in-place rotation once the PD's authority drops below "
                  "TURN_CREEP_THRESH_PWM.");

    // (e) At least one in-band sample must be required, or the debounce is a no-op and the turn
    // exits on the very first (possibly noisy) in-band tick.
    static_assert(TURN_SETTLE_SAMPLES >= 1,
                  "Config::TURN_SETTLE_SAMPLES must be >= 1 — zero would make the settle "
                  "debounce a no-op, exiting on the first in-band sample with no confirmation.");

    // (f)/(g) The slew step must be strictly positive (or the ramp-up limiter never lets the
    // command grow at all), and it must be able to cross the creep-to-threshold hand-off gap in
    // a single tick, or a command growing off the creep floor lands back inside the gap and gets
    // re-floored to the SAME creep value every tick — the "ramp" never actually grows.
    static_assert(TURN_SLEW_PWM_PER_TICK > 0,
                  "Config::TURN_SLEW_PWM_PER_TICK must be > 0 — zero would freeze the command at "
                  "its seeded start value forever, since growth is never permitted.");
    static_assert(TURN_SLEW_PWM_PER_TICK >= (TURN_CREEP_THRESH_PWM - TURN_CREEP_PWM),
                  "Config::TURN_SLEW_PWM_PER_TICK must be able to cross the "
                  "(TURN_CREEP_THRESH_PWM - TURN_CREEP_PWM) hand-off gap in a single tick. "
                  "NOTE: this deviates from the turn-PD plan's literal AC8(g) text "
                  "(`TURN_SLEW_PWM_PER_TICK >= TURN_CREEP_PWM`), which fails to compile even "
                  "with the plan's OWN placeholder values (slew=10, creep=15) and with the "
                  "bench-measured values (slew=10, creep=38) -- flagged for review in the T3 "
                  "handoff, not silently resolved. A slew step smaller than this gap means a "
                  "command growing off the creep floor lands back inside "
                  "[TURN_CREEP_PWM, TURN_CREEP_THRESH_PWM) and turnCreepFloor snaps it straight "
                  "back to TURN_CREEP_PWM every tick -- the ramp-up limit would be a lie.");

    // (h) The turn's own timeout must fire before Drive's blocking-drain ceiling, or a hunting
    // turn (non-faulting by design) could stall the drain's 10 s-per-command budget instead of
    // resolving on its own.
    static_assert(TURN_TIMEOUT_MS < DRIVE_DRAIN_TIMEOUT_MS_PER_CMD,
                  "Config::TURN_TIMEOUT_MS must be < DRIVE_DRAIN_TIMEOUT_MS_PER_CMD — the turn's "
                  "own non-faulting timeout must resolve before the blocking drain's per-command "
                  "ceiling, or a hunting turn eats the whole drain budget instead of the turn's "
                  "own bounded give-up.");

    // (i) The static breakaway floor must sit ABOVE the kinetic creep floor — starting an in-place
    // spin from rest breaks more (scrub) friction than sustaining one, so a breakaway at or below
    // the creep would defeat its own purpose (bench: 46 vs 38 on Lofaix).
    static_assert(TURN_BREAKAWAY_PWM > TURN_CREEP_PWM,
                  "Config::TURN_BREAKAWAY_PWM must be > TURN_CREEP_PWM — the from-rest static "
                  "breakaway floor is higher than the kinetic floor that sustains an already-"
                  "moving spin. A breakaway <= creep cannot start a turn the creep floor could "
                  "not already start.");

    // (j) The breakaway kick must be reachable within the default authority ceiling, or the ±power
    // clamp (which runs before the floor) caps the command below the floor the chassis needs.
    static_assert(TURN_BREAKAWAY_PWM <= TURN_CRUISE_PWM,
                  "Config::TURN_BREAKAWAY_PWM must be <= TURN_CRUISE_PWM — the ±power clamp runs "
                  "before the floor, so a breakaway above the default authority ceiling could "
                  "never be commanded on a default-power turn.");

    // (k) The breakaway->creep hand-off distance must be at least one physically resolvable step,
    // or a genuinely-rotating chassis could never accumulate enough measured motion to cross it.
    static_assert(TURN_BREAKAWAY_MOTION_DEG >= TURN_MIN_RESOLVABLE_DEG,
                  "Config::TURN_BREAKAWAY_MOTION_DEG must be >= TURN_MIN_RESOLVABLE_DEG — the "
                  "breakaway->creep hand-off distance must exceed one per-tick step, else the turn "
                  "never latches 'moving' and kicks at the breakaway floor for its whole travel.");

    // -----------------------------------------------------------------------------------------
    // GYRO straight heading-hold PD checks (ConfigDrive.h, ADR-075, issue #169)
    //
    // `Drive::_emitGyroHold` (via `GyroHoldPolicy.h`) is a per-tick PD + never-reverse clamp
    // driven by the external gyro. These four asserts pin the basic sign/range sanity of the
    // hold's knobs — the gains themselves ship UNVERIFIED (mentor `gyro_move` placeholders,
    // AC9 hardware-gated), so this is a structural gate, not a value pin.
    // -----------------------------------------------------------------------------------------

    static_assert(GYRO_HOLD_KP > 0.0f,
                  "Config::GYRO_HOLD_KP must be > 0 — a zero or negative proportional gain would "
                  "either apply no heading correction at all or steer AWAY from the target, and "
                  "Drive::_emitGyroHold has no other authority source to hold a GYRO straight to "
                  "heading.");

    static_assert(GYRO_HOLD_KD >= 0.0f,
                  "Config::GYRO_HOLD_KD must be >= 0 — a negative derivative gain would invert "
                  "the damping term into positive feedback on yaw rate, amplifying oscillation "
                  "instead of damping it.");

    static_assert(GYRO_HOLD_MAX_CORR_PWM > 0 && GYRO_HOLD_MAX_CORR_PWM <= PWM_MAX,
                  "Config::GYRO_HOLD_MAX_CORR_PWM must be in (0, PWM_MAX] — zero or negative "
                  "disables all heading correction (a zero-width or inverted clamp), and a value "
                  "above PWM_MAX could command a per-wheel PWM beyond the legal range before the "
                  "final ±PWM_MAX cap in GyroHoldPolicy.h::gyroHoldWheels.");

    static_assert(GYRO_HOLD_STALE_MS > 0,
                  "Config::GYRO_HOLD_STALE_MS must be > 0 — zero would force the derivative term "
                  "to 0 on every tick (even back-to-back ones), silently degrading the GYRO "
                  "heading hold to P-only.");

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

    // Anticipatory line-approach ramp checks (ConfigDrive.h)
    // -----------------------------------------------------------------------------------------

    static_assert(LINE_APPROACH_MARGIN_CM >= 0.0f,
                  "Config::LINE_APPROACH_MARGIN_CM must be >= 0.0f -- a negative pre-prediction "
                  "margin would bottom out the ramp AFTER the predicted crossing instead of "
                  "before it, defeating the anticipatory approach ramp entirely.");

    static_assert(LINE_APPROACH_MAX_MARGIN_CM > 0.0f,
                  "Config::LINE_APPROACH_MAX_MARGIN_CM must be > 0.0f -- a zero or negative "
                  "overrun bound would fault LINE_NOT_FOUND at (or before) the predicted "
                  "crossing itself, leaving no bounded floor-speed search window for a late "
                  "line.");

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
    // Belt encoder read sign is applied in ManipBelt::_getDegrees(); must be exactly +/-1.
    static_assert(BELT_DIR_SIGN == 1 || BELT_DIR_SIGN == -1,
                  "BELT_DIR_SIGN must be +1 or -1.");
    // The POSITION loop's creep phase is what lets a move stop inside tolerance. A creep speed
    // at or above the coarse speed collapses the two phases into single-speed bang-bang, which
    // overshoots the tolerance band every tick and dithers until the watchdog fires.
    static_assert(BELT_POS_CREEP_SPEED > 0 && BELT_POS_CREEP_SPEED < BELT_MOVE_SPEED,
                  "BELT_POS_CREEP_SPEED must be positive and below BELT_MOVE_SPEED, or a "
                  "POSITION move cannot settle inside BELT_POS_TOL_DEG.");
    // The slow zone must be wider than the tolerance band, otherwise the loop is still at
    // coarse speed when it enters tolerance and overshoots straight back out.
    static_assert(BELT_POS_SLOW_ZONE_DEG > BELT_POS_TOL_DEG,
                  "BELT_POS_SLOW_ZONE_DEG must exceed BELT_POS_TOL_DEG.");
    // Arrival tolerance must exceed the stall detector's motion epsilon, or the loop can sit
    // inside tolerance while the stall detector still counts the residual creep as motion.
    static_assert(BELT_POS_TOL_DEG > BELT_POS_MOTION_EPS_DEG,
                  "BELT_POS_TOL_DEG must exceed BELT_POS_MOTION_EPS_DEG.");
    static_assert(BELT_POS_MOTION_EPS_DEG > 0, "BELT_POS_MOTION_EPS_DEG must be positive.");
    // A stall must be detectable before the wall-clock watchdog, or it never fires.
    static_assert(BELT_POS_STALL_MS > 0 && BELT_POS_STALL_MS < BELT_MOVE_TIMEOUT_MS,
                  "BELT_POS_STALL_MS must be positive and below BELT_MOVE_TIMEOUT_MS.");
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
    // Config::IMU_YAW_SIGN and its tripwire were RETIRED in issue #169 (ADR-075): the STM32
    // gyro-PID write path (YawSource::toHwHeading) is gone, so nothing applies an app->STM32 yaw
    // sign anymore. The surviving read-path sign is Config::GYRO_YAW_SIGN, checked below.

    // -----------------------------------------------------------------------------------------
    // External gyro checks (ConfigNav.h, ADR-074)
    //
    // COUPLED SITES: Config::GYRO_YAW_SIGN and Config::GYRO_SCALE are applied together, exactly
    // once, inside GyroDecodePolicy.h::decodeGyroYawDeg (called from YawSource) to convert the
    // external gyro's raw decoded heading into the app CCW-positive frame. GYRO_YAW_SIGN is a
    // PURE SIGN (mirrors the IMU_YAW_SIGN assert above); GYRO_SCALE is a magnitude only sane
    // near 1.0 -- a value outside that band indicates a units/decode error, not a real
    // calibration. GYRO_I2C_PORT is a physical mux-port label that must not collide with either
    // line board's port, else the gyro read would silently address a line sensor instead.
    // -----------------------------------------------------------------------------------------

    static_assert(GYRO_YAW_SIGN == 1.0f || GYRO_YAW_SIGN == -1.0f,
                  "Config::GYRO_YAW_SIGN must be exactly 1.0f or -1.0f -- it is the pure "
                  "raw-gyro-to-app CCW sign applied by GyroDecodePolicy.h::decodeGyroYawDeg, "
                  "not a scale factor. See config/ConfigNav.h.");

    static_assert(GYRO_SCALE > 0.9f && GYRO_SCALE < 1.1f,
                  "Config::GYRO_SCALE must be in (0.9, 1.1) -- this is a sane-band sanity check "
                  "on the raw-count-to-degree calibration, not a value pin (unlike a tuned drive "
                  "constant, GYRO_SCALE is MEANT to be overwritten by the gyro_calib bench "
                  "output). A value this far from 1.0 indicates a units or decode error rather "
                  "than a real per-board calibration. See config/ConfigNav.h.");

    static_assert(GYRO_I2C_PORT >= 1 && GYRO_I2C_PORT <= 4,
                  "Config::GYRO_I2C_PORT must be in [1, 4] -- matches the physical I2C1..I2C4 "
                  "port labels on the MiniR4 board. YawSource converts to the 0-based PCA9548A "
                  "mux channel internally; do NOT pre-convert in Config.");

    static_assert(GYRO_I2C_PORT != LINE_FRONT_I2C_PORT,
                  "Config::GYRO_I2C_PORT must not collide with Config::LINE_FRONT_I2C_PORT -- a "
                  "shared port would make the gyro read silently address the front line board "
                  "(or vice versa) even though both pass their own runtime ID check.");

    static_assert(LINE_BOARD_COUNT != 2 || GYRO_I2C_PORT != LINE_REAR_I2C_PORT,
                  "Config::GYRO_I2C_PORT must not collide with Config::LINE_REAR_I2C_PORT when "
                  "Config::LINE_BOARD_COUNT == 2 -- a shared port would make the gyro read "
                  "silently address the rear line board (or vice versa) even though both pass "
                  "their own runtime ID check.");

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
