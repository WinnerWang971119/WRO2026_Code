/**
 * @file test_manipulator.cpp
 * @brief Host-compile tests for the Manipulator facade + ManipServo + ManipBelt (T9).
 *
 * Links the real production `Manipulator.cpp` (and the header-only `ManipServo.h` /
 * `ManipBelt.h` it owns) against the host mocks under `WRO2026/test/mocks/`:
 *   - `<Arduino.h>`       -> mock clock (`g_mockMillis`); `delay(ms)` advances it.
 *   - `<MatrixMiniR4.h>`  -> mock `MiniR4` facade with `RC1..RC4` (MockRC) +
 *                            `M1..M4` (MockDC), each recording call counts / last
 *                            args, with test-settable encoder degrees + rotate-end
 *                            flag (`mockSetDegrees`, `mockRotateEnded`).
 * The global `Manipulator` is defined in `Manipulator.cpp` (linked by CMake, T11).
 *
 * Channel wiring (from ConfigManipulator.h):
 *   - grabber -> RC2   (GRABBER_RC_ID = 2, home 0 / open 30 / close 150)
 *   - sorter  -> RC4   (SORTER_RC_ID  = 4, bins {55, 76, 104, 126})
 *   - dumper  -> RC1   (DUMPER_RC_ID  = 1, home 80 / open 90 / close 150)
 *   - belt    -> M1    (BELT_M_ID     = 1)
 *
 * Channel assertions go through `rcFor(Config::<servo>.rcId)` rather than a hardcoded
 * `MiniR4.RCn`, so they follow the configured wiring (the RC IDs are bring-up TODOs and
 * have already been re-assigned once).
 *
 * Test cases: TC1..TC12 + TC15. TC16 is OMITTED — see the comment block above
 * `test_TC15_begin_homes_all_and_zeroes_belt_once()` for the host-untriggerability
 * rationale (the mock returns true unconditionally and `delay()` advances the clock,
 * so begin()'s wall-clock-bounded home always completes; the timeout-false branch is
 * verified by inspection, and the bounded watchdog is covered by TC9).
 *
 * Each test resets the mock world (MiniR4 channels + clock) and re-`begin()`s the
 * Manipulator at scope entry. The clock is NOT re-zeroed after `begin()` — it sits at
 * the post-boot baseline (~1580 ms, advanced by begin()'s settle + belt home), at which
 * every actuator already reports done. Per-test timing math is RELATIVE: each test that
 * inspects timing re-`setAngle()`s the servo (re-stamping its start to the current clock)
 * or issues a belt move, then advances via `mockAdvanceMillis()` deltas — so the nonzero
 * baseline is harmless. See `resetWorld()` for the full rationale.
 */

#include "TestAssert.h"

#include "Arduino.h"
#include "MatrixMiniR4.h"

// Production code under test (pulls Config.h / ConfigManipulator.h / ManipServo.h /
// ManipBelt.h transitively):
#include "src/manip/Manipulator.h"

// ---------------------------------------------------------------------------
// Shared helpers.
// ---------------------------------------------------------------------------

/// Reset every mock + the clock, then re-init the Manipulator for a fresh scenario.
///
/// `Manipulator` is a global already constructed at static-init, so we cannot
/// reconstruct it. Instead we wipe the mock recording state, zero the clock, and
/// call `Manipulator.begin()` so the servos seed to home and the belt stall-homes —
/// the actuators all start in their booted, done state and the mock counters are
/// clean.
///
/// Post-`begin()` baseline: the clock sits at ~6300 ms — begin()'s internal settle plus the
/// full belt home (TIMED stall ~2000 ms, the SETTLE dwell ~150 ms, then the MID move
/// which, with ChkRotateEnd held false in the mock, completes via its ~4000 ms watchdog), all
/// of which advance `g_mockMillis` via the `delay()`-driven poll loops. We do NOT re-zero the
/// clock afterward: each servo's
/// `_startMs` was stamped at ~0 during begin() and its `_estMs` is the slew+settle
/// budget (~80 ms for a home no-op), so with the clock now well past that budget every
/// servo's `isDone()` (`millis() - _startMs >= _estMs`) is true. The belt finished its
/// home and is IDLE, so `belt.isDone()` is true. Hence the facade is homed and
/// `Manipulator.isDone()` is true at the end of this function — the correct post-boot
/// baseline. (Re-zeroing the clock here would make `millis() - _startMs == 0 < _estMs`
/// and falsely report the servos NOT done; that was the TC5 bug this fixes.)
///
/// Every per-test scenario that inspects timing first re-`setAngle()`s the servo (which
/// re-stamps `_startMs` to the current clock) or issues a belt move (stamping its start),
/// then uses `mockAdvanceMillis()` RELATIVE deltas — so a nonzero baseline clock is fine.
///
/// We re-wipe the mock after `begin()` so the begin()-time writes/counters do not leak
/// into per-test call-count assertions; the belt encoder is re-zeroed by `begin()`'s
/// home `resetCounter()` either way. We also clear any registered tick callback so a
/// test that registered one (TC10) cannot leak its callback into a later test (S1).
static void resetWorld()
{
    mockResetMiniR4();
    mockResetMillis();

    Manipulator.begin();    // home servos + stall-home belt (consumes mock millis)

    // Wipe begin()-time recording so per-test call-count deltas start from zero. Do NOT
    // re-zero the clock: clearing the mock does not touch the Manipulator's internal
    // timer/state fields (servos done because their settle budget elapsed against the
    // advanced clock, belt IDLE), so the facade stays in its homed, isDone()==true state.
    mockResetMiniR4();

    // Clear any sticky tick callback so the TC10 callback cannot leak across tests.
    Manipulator.setTickCallback(nullptr);
}

/// Clear the registered tick callback so tests that do NOT exercise the callback path
/// are independent of one that did (the callback pointer is a sticky facade member).
///
/// `resetWorld()` already clears the callback at the start of every test (S1), so these
/// explicit calls are redundant but harmless — they are kept for local readability and
/// create no ordering dependency (calling `setTickCallback(nullptr)` twice is idempotent).
static void clearTickCallback()
{
    Manipulator.setTickCallback(nullptr);
}

/// Map a `Config::ServoCfg::rcId` to the matching mock RC channel, so channel assertions
/// track the configured wiring instead of hardcoding a specific `MiniR4.RCn`. Keeps the
/// test green when the RC IDs are re-assigned in `ConfigManipulator.h` (the manipulator
/// channels are bring-up TODOs and have already moved once).
static MockRC& rcFor(uint8_t rcId)
{
    switch (rcId)
    {
    case 1:  return MiniR4.RC1;
    case 2:  return MiniR4.RC2;
    case 3:  return MiniR4.RC3;
    case 4:  return MiniR4.RC4;
    default:
        // A configured rcId outside 1..4 is a wiring/config bug. Fail loudly instead of
        // silently aliasing to RC1, which would assert against the wrong channel and let
        // the bad Config::<servo>.rcId slip through green.
        std::fprintf(stderr,
                     "FATAL: rcFor: invalid rcId %u (Config::<servo>.rcId must be 1..4)\n",
                     (unsigned) rcId);
        std::abort();
    }
}

// ===========================================================================
// TC1 — setAngle clamps to [0,180] and writes the clamped angle (no SW reverse).
// ===========================================================================

static void test_TC1_grabber_setangle_clamps_to_180()
{
    TEST_SCOPE("TC1: grabber.setAngle(200) clamps to 180; grabber channel records 180");

    resetWorld();
    clearTickCallback();

    // Force the instant path (stepMs = 0) so this clamp check is independent of the grabber's
    // configured default ramp rate — a stepMs > 0 default would defer the write to update().
    Manipulator.grabber.setAngle(200, 0);

    // Commanded shadow is the clamped value (no software reverse transform).
    ASSERT_EQ(Manipulator.grabber.commandedAngle(), (uint16_t) 180);
    // The mock RC channel wired to the grabber recorded the clamped angle verbatim.
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) 180);
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).setAngleCallCount, (uint32_t) 1);
}

static void test_TC1b_dumper_open_for_white_uses_dedicated_pose()
{
    TEST_SCOPE("TC1b: dumper.openForWhite() commands its dedicated configured pose");

    resetWorld();
    clearTickCallback();

    Manipulator.dumper.openForWhite();

    ASSERT_EQ(Manipulator.dumper.commandedAngle(), (uint16_t) Config::DUMPER.openForWhiteDeg);
    ASSERT_EQ(rcFor(Config::DUMPER.rcId).lastAngle,
              (uint16_t) Config::DUMPER.openForWhiteDeg);
    ASSERT_EQ(rcFor(Config::DUMPER.rcId).setAngleCallCount, (uint32_t) 1);
}

// ===========================================================================
// TC2 / TC3 — INSTANT-move timer-based completion crosses the slew+settle budget.
//
// These pin the msPerDeg completion estimate for an instant (stepMs = 0) move, so they call the
// explicit-rate setAngle(deg, 0) overload — independent of the grabber's configured default rate.
// Estimate from cfg: |target - prevShadow| * msPerDeg + settleMs.
// grabber boot shadow = GRABBER.closeDeg (post-begin — begin() boots the grabber CLOSED).
// setAngle(180, 0):
//   est = |180 - closeDeg| * GRABBER.msPerDeg + GRABBER.settleMs.
// ===========================================================================

// Slew+settle budget for grabber boot pose (close) -> 180, computed from Config (drift-proof).
static uint32_t grabberSetAngle180Est()
{
    // Mirror production ManipServo::setAngle() exactly: fabs() of the delta, so a misconfigured
    // closeDeg > 180 can't drive the cast negative and underflow into a huge uint32_t.
    return (uint32_t) (fabs(180.0f - (float) Config::GRABBER.closeDeg) * Config::GRABBER.msPerDeg)
           + Config::GRABBER.settleMs;
}

static void test_TC2_grabber_not_done_below_estimate()
{
    TEST_SCOPE("TC2: grabber not done just below slew+settle estimate");

    resetWorld();
    clearTickCallback();

    Manipulator.grabber.setAngle(180, 0);    // instant: shadow homeDeg -> 180

    const uint32_t est = grabberSetAngle180Est();
    // Advance to just BELOW the estimate.
    mockAdvanceMillis(est - 1);
    ASSERT_FALSE(Manipulator.grabber.isDone());
}

static void test_TC3_grabber_done_at_estimate()
{
    TEST_SCOPE("TC3: grabber done once estimate elapses");

    resetWorld();
    clearTickCallback();

    Manipulator.grabber.setAngle(180, 0);    // instant: shadow homeDeg -> 180

    const uint32_t est = grabberSetAngle180Est();
    // Advance to exactly the estimate (>= path).
    mockAdvanceMillis(est);
    ASSERT_TRUE(Manipulator.grabber.isDone());
}

// ===========================================================================
// TC4 — sortTo(valid bin).async() commands the sorter to SORTER_BIN_ANGLE[bin].
// ===========================================================================

static void test_TC4_sortto_valid_bin_commands_sorter()
{
    TEST_SCOPE("TC4: sortTo(1).async() drives the sorter channel to SORTER_BIN_ANGLE[1]");

    resetWorld();
    clearTickCallback();

    Manipulator.sortTo(1).async();

    // bin 1 angle is SORTER_BIN_ANGLE[1]; assert on the sorter's configured channel.
    ASSERT_EQ(rcFor(Config::SORTER.rcId).lastAngle, (uint16_t) Config::SORTER_BIN_ANGLE[1]);
    ASSERT_EQ(rcFor(Config::SORTER.rcId).setAngleCallCount, (uint32_t) 1);
}

// ===========================================================================
// TC5 — sortTo(out-of-range) performs NO sorter write and completes immediately.
//
// bin == SORTER_BIN_COUNT is the first OOB index. The guard returns an empty-mask
// action (drain is a no-op) and issues no servo write — so a read of
// SORTER_BIN_ANGLE[bin] never happens (an OOB read would trap under ASan/UBSan, T11).
// ===========================================================================

static void test_TC5_sortto_out_of_range_no_write_and_done()
{
    TEST_SCOPE("TC5: sortTo(SORTER_BIN_COUNT).async() — no sorter write, immediately done");

    resetWorld();
    clearTickCallback();

    const uint32_t writesBefore = rcFor(Config::SORTER.rcId).setAngleCallCount;

    Manipulator.sortTo(Config::SORTER_BIN_COUNT).async();

    // No sorter write happened (call count unchanged) — the OOB SORTER_BIN_ANGLE read
    // was avoided by the bounds guard.
    ASSERT_EQ(rcFor(Config::SORTER.rcId).setAngleCallCount, writesBefore);
    // Empty-mask action drains as an immediate no-op; the facade is still all-done.
    ASSERT_TRUE(Manipulator.isDone());
}

// ===========================================================================
// Belt POSITION helpers.
//
// The POSITION move is closed on the R4 (ManipBelt::update()), not by the lower MCU's
// rotateFor — that primitive delivers only a fraction of the commanded degrees on this
// hardware and never reports done. So a POSITION test scripts the encoder itself:
// set a position, call update(), assert on the speed the loop chose.
//
// mockSetDegrees seeds the RAW encoder; _getDegrees() applies Config::BELT_DIR_SIGN to
// produce the logical frame. Seed through the same sign so the intended LOGICAL value
// lands, whatever the sign is configured to.
// ===========================================================================

/// @brief Put the belt's logical position at @p logicalDeg (belt is on M1).
static void seedBeltLogicalDeg(int32_t logicalDeg)
{
    MiniR4.M1.mockSetDegrees((int32_t) (Config::BELT_DIR_SIGN * logicalDeg));
}

// ===========================================================================
// TC6 — belt POSITION move to MID: the R4 loop drives coarse, creeps, then stops.
//
// Seeds the belt well below MID so the move starts outside the slow zone, then walks it
// in: coarse speed while far, creep speed inside BELT_POS_SLOW_ZONE_DEG, stop + brake
// once inside BELT_POS_TOL_DEG. Asserts the sign, both speed phases, and that no
// rotateFor is issued at all.
// ===========================================================================

static void test_TC6_belt_position_move_to_mid()
{
    TEST_SCOPE("TC6: belt.toStation(MID) drives the R4 position loop coarse->creep->stop");

    resetWorld();
    clearTickCallback();

    // MID's absolute target comes from config (the operator tunes it). Start a fixed distance
    // BELOW it, outside the slow zone, so the coarse phase is exercised regardless of the
    // configured MID value.
    const int32_t midDeg = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::MID].deg;
    const int32_t farBelow = midDeg - (Config::BELT_POS_SLOW_ZONE_DEG + 300);
    seedBeltLogicalDeg(farBelow);

    Manipulator.belt.toStation(Config::BeltStation::MID);    // POSITION, abs target midDeg

    // Arming issues no drive command — update() owns every write.
    ASSERT_EQ(MiniR4.M1.rotateForCallCount, (uint32_t) 0);
    ASSERT_FALSE(Manipulator.belt.isDone());

    // First tick: far from target -> COARSE, positive (target is above current).
    Manipulator.belt.update();
    ASSERT_EQ(MiniR4.M1.lastSpeed, (int16_t) Config::BELT_MOVE_SPEED);
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Walk into the slow zone -> CREEP, still positive, still in flight.
    seedBeltLogicalDeg(midDeg - (Config::BELT_POS_SLOW_ZONE_DEG / 2));
    mockAdvanceMillis(20);
    Manipulator.belt.update();
    ASSERT_EQ(MiniR4.M1.lastSpeed, (int16_t) Config::BELT_POS_CREEP_SPEED);
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Arrive inside tolerance -> stop, brake, IDLE.
    seedBeltLogicalDeg(midDeg);
    mockAdvanceMillis(20);
    Manipulator.belt.update();
    ASSERT_EQ(MiniR4.M1.lastSpeed, (int16_t) 0);
    ASSERT_TRUE(MiniR4.M1.lastBrake);
    ASSERT_TRUE(Manipulator.belt.isDone());

    // The lower MCU's position primitive is never used.
    ASSERT_EQ(MiniR4.M1.rotateForCallCount, (uint32_t) 0);
}

// ===========================================================================
// TC6b — a POSITION move that starts inside tolerance is a no-op.
//
// Guards the early-out in beltTo(): no drive command, immediately done, so a blocking
// drain on an already-positioned belt returns at once.
// ===========================================================================

static void test_TC6b_belt_position_already_there_is_noop()
{
    TEST_SCOPE("TC6b: beltTo() inside BELT_POS_TOL_DEG is an immediate no-op");

    resetWorld();
    clearTickCallback();

    const int32_t midDeg = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::MID].deg;
    // Just inside tolerance — must NOT start a move.
    seedBeltLogicalDeg(midDeg - (Config::BELT_POS_TOL_DEG - 1));

    const uint32_t speedCallsBefore = MiniR4.M1.setSpeedCallCount;
    Manipulator.belt.toStation(Config::BeltStation::MID);

    ASSERT_TRUE(Manipulator.belt.isDone());
    ASSERT_EQ(MiniR4.M1.setSpeedCallCount, speedCallsBefore);
    ASSERT_EQ(MiniR4.M1.rotateForCallCount, (uint32_t) 0);
}

// ===========================================================================
// TC6c — the loop drives NEGATIVE when the target is below the current position.
//
// The direction is re-derived from `target - current` every tick, so a target on the
// other side must flip the commanded sign. This is the property that makes the loop
// self-correcting on overshoot.
// ===========================================================================

static void test_TC6c_belt_position_drives_negative_when_target_below()
{
    TEST_SCOPE("TC6c: POSITION move below the current position commands negative speed");

    resetWorld();
    clearTickCallback();

    const int32_t midDeg = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::MID].deg;
    // Start ABOVE the target, outside the slow zone.
    seedBeltLogicalDeg(midDeg + Config::BELT_POS_SLOW_ZONE_DEG + 300);

    Manipulator.belt.toStation(Config::BeltStation::MID);
    Manipulator.belt.update();

    ASSERT_EQ(MiniR4.M1.lastSpeed, (int16_t) (-Config::BELT_MOVE_SPEED));
    ASSERT_FALSE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC6d — a jammed POSITION move ends on the stall detector, not the watchdog.
//
// The belt is held still (encoder never changes) while the clock advances past
// BELT_POS_STALL_MS. The move must brake and report done — a jam must never hang a
// blocking drain — and it must do so well before BELT_MOVE_TIMEOUT_MS.
// ===========================================================================

static void test_TC6d_belt_position_stall_ends_move()
{
    TEST_SCOPE("TC6d: a POSITION move whose encoder never moves ends on BELT_POS_STALL_MS");

    resetWorld();
    clearTickCallback();

    const int32_t midDeg = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::MID].deg;
    seedBeltLogicalDeg(midDeg - (Config::BELT_POS_SLOW_ZONE_DEG + 300));

    Manipulator.belt.toStation(Config::BeltStation::MID);
    Manipulator.belt.update();
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Hold the encoder dead still across the stall window. The position never changes, so the
    // stall reference never advances.
    mockAdvanceMillis((uint32_t) Config::BELT_POS_STALL_MS);
    Manipulator.belt.update();

    ASSERT_TRUE(Manipulator.belt.isDone());
    ASSERT_EQ(MiniR4.M1.lastSpeed, (int16_t) 0);
    ASSERT_TRUE(MiniR4.M1.lastBrake);
    // Ended on the stall path, long before the wall-clock watchdog would have fired.
    ASSERT_GT((uint32_t) Config::BELT_MOVE_TIMEOUT_MS, (uint32_t) Config::BELT_POS_STALL_MS);
}

// ===========================================================================
// TC6e — a slow creep is NOT mistaken for a stall.
//
// Stall detection measures against a reference that only advances on confirmed motion,
// never against the previous tick. At creep speed the belt moves less than
// BELT_POS_MOTION_EPS_DEG in a single tick, so a per-tick comparison would read a healthy
// creep as a jam. This walks the belt in sub-epsilon steps across more than the stall
// window and asserts the move survives.
// ===========================================================================

static void test_TC6e_belt_position_slow_creep_is_not_a_stall()
{
    TEST_SCOPE("TC6e: sub-epsilon per-tick motion accumulates and does not trip the stall");

    resetWorld();
    clearTickCallback();

    const int32_t midDeg = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::MID].deg;
    int32_t       cur    = midDeg - (Config::BELT_POS_SLOW_ZONE_DEG + 300);
    seedBeltLogicalDeg(cur);

    Manipulator.belt.toStation(Config::BeltStation::MID);
    Manipulator.belt.update();

    // Advance in steps SMALLER than the motion epsilon, for longer than the stall window.
    // Real motion accumulates past the epsilon, so the reference keeps advancing.
    const uint32_t stepMs = 20;
    const uint32_t steps  = ((uint32_t) Config::BELT_POS_STALL_MS / stepMs) * 3;
    for (uint32_t i = 0; i < steps; ++i)
    {
        cur += 1;    // below BELT_POS_MOTION_EPS_DEG per tick, but it adds up
        seedBeltLogicalDeg(cur);
        mockAdvanceMillis(stepMs);
        Manipulator.belt.update();
        if (Manipulator.belt.isDone()) { break; }
    }

    // Still driving: real progress was made, so neither the stall detector nor an early
    // tolerance match retired the move.
    ASSERT_FALSE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC7 — belt TIMED move to FAR_END: setSpeed issued, done only after the timer.
//
// FAR_END is TIMED with ms == 1500 (BELT_STATIONS[FAR_END].ms). The belt runs at a
// fixed setSpeed and completes purely on the timer (no ChkRotateEnd).
// ===========================================================================

static void test_TC7_belt_timed_move_to_far_end()
{
    TEST_SCOPE("TC7: belt.toStation(FAR_END) is TIMED — setSpeed, done after the run ms");

    resetWorld();
    clearTickCallback();

    Manipulator.belt.toStation(Config::BeltStation::FAR_END);    // TIMED, 1500 ms

    // setSpeed was issued to drive the belt (M1).
    ASSERT_GE(MiniR4.M1.setSpeedCallCount, (uint32_t) 1);
    // Not done immediately (the run timer has not elapsed).
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Advance past the station's run duration; update() then brakes + idles.
    const uint32_t runMs = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::FAR_END].ms;
    mockAdvanceMillis(runMs);
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC16 — beltTo(station) verb delegates to the belt like belt.toStation().
//
// The facade verb is the belt analogue of sortTo(bin): it fires belt.toStation(station)
// and returns a belt-masked, blocking-by-default action. Assert the .async() form issues
// the same TIMED move as the primitive (FAR_END) and returns with the belt still in flight.
// ===========================================================================

static void test_TC16_beltto_verb_delegates_to_belt()
{
    TEST_SCOPE("TC16: beltTo(FAR_END).async() issues the belt's TIMED move, in flight");

    resetWorld();
    clearTickCallback();

    Manipulator.beltTo(Config::BeltStation::FAR_END).async();    // TIMED, 1500 ms

    // The verb drove the belt (M1) via toStation, so a setSpeed run was issued.
    ASSERT_GE(MiniR4.M1.setSpeedCallCount, (uint32_t) 1);
    // Blocking-by-default action was .async()'d, so control returned with the belt still moving.
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Advance past the run duration; update() brakes + idles -> done.
    const uint32_t runMs = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::FAR_END].ms;
    mockAdvanceMillis(runMs);
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC8 — belt home() is a three-phase sequence: reset() TIMED stall toward LEFT/HOME (braked
//       + resetCounter zeroes the encoder) -> SETTLE dwell -> chained toStation(MID).
//
// home() now splits into reset() + a chained station move: on stall completion update()
// brakes + resetCounter()s and enters SETTLE (NOT a move yet). After BELT_HOME_SETTLE_MS the
// MID move is issued via toStation(MID) -> beltTo(midDeg). Because the encoder was just zeroed
// (getDegrees()==0) and MID's absolute target is midDeg, beltTo computes delta = midDeg - 0 =
// midDeg and issues rotateFor(+speed, midDeg) — so the externally observable rotate
// (rotateForCallCount==1, lastRotateDegree==midDeg, lastRotateSpeed>0) is identical to the
// old relative-rotate path. isDone() stays false until the MID move ends (ChkRotateEnd /
// watchdog).
// ===========================================================================

static void test_TC8_belt_home_resets_settles_then_moves_to_mid()
{
    TEST_SCOPE("TC8: belt.home() -> reset+stall -> settle -> toStation(MID) rotate -> done");

    resetWorld();
    clearTickCallback();

    // Park the encoder at a nonzero position so the reset is observable.
    MiniR4.M1.mockSetDegrees(250);
    MiniR4.M1.mockRotateEnded = false;    // hold the MID move in flight until we signal it

    Manipulator.belt.home();    // phase 1: reset() TIMED toward LEFT/HOME for BELT_RESET_MS
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Advance past the stall duration; the completing update() brakes + zeroes and enters
    // SETTLE (the MID move is deferred to give the lower MCU a clean tick gap).
    mockAdvanceMillis(Config::BELT_RESET_MS);
    Manipulator.belt.update();

    // The stall re-zeroed the encoder exactly once and braked — but NO MID rotate yet.
    ASSERT_EQ(MiniR4.M1.resetCounterCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.M1.getDegrees(), (int32_t) 0);
    ASSERT_GE(MiniR4.M1.setBrakeCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.M1.rotateForCallCount, (uint32_t) 0);
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Advance past the settle dwell; update() now issues the MID move via toStation(MID) ->
    // beltTo(midDeg). The encoder reads 0 (just zeroed), so beltTo's delta == midDeg and it
    // emits rotateFor(+speed, midDeg) — observably identical to the prior relative rotate.
    mockAdvanceMillis(Config::BELT_HOME_SETTLE_MS);
    Manipulator.belt.update();

    const int32_t midDeg = Config::BELT_STATIONS[(uint8_t) Config::BeltStation::MID].deg;
    // The chained MID move is now ARMED but not yet driving: beltTo() issues no command, so
    // update() still shows the stall's speed until the next tick runs the POSITION loop.
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Next tick drives toward FAR_END (positive, since the freshly zeroed encoder sits below
    // MID's positive target).
    mockAdvanceMillis(20);
    Manipulator.belt.update();
    ASSERT_GT(MiniR4.M1.lastSpeed, (int16_t) 0);
    ASSERT_FALSE(Manipulator.belt.isDone());
    // The lower MCU's position primitive is never used.
    ASSERT_EQ(MiniR4.M1.rotateForCallCount, (uint32_t) 0);

    // Deliver the belt to MID; a single update() sees the error inside tolerance and retires
    // the whole home sequence.
    seedBeltLogicalDeg(midDeg);
    mockAdvanceMillis(20);
    Manipulator.belt.update();
    ASSERT_EQ(MiniR4.M1.lastSpeed, (int16_t) 0);
    ASSERT_TRUE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC9 (AC10) — POSITION-move watchdog forces completion at BELT_MOVE_TIMEOUT_MS.
//
// The watchdog and the stall detector are different escapes. A belt that never MOVES ends
// on the stall detector (TC6d), long before the watchdog. This covers the other case: a
// belt that keeps moving but never arrives — a slipping belt, or a target it cannot reach.
// The encoder is advanced every tick so the stall detector never fires, and the move must
// survive until millis()-start >= BELT_MOVE_TIMEOUT_MS, then be forced to IDLE.
// ===========================================================================

static void test_TC9_belt_position_watchdog_fires()
{
    TEST_SCOPE("TC9: a moving-but-never-arriving POSITION move hits BELT_MOVE_TIMEOUT_MS (AC10)");

    resetWorld();
    clearTickCallback();

    seedBeltLogicalDeg(0);
    Manipulator.belt.beltTo(800);    // |800 - 0| = 800 -> POS_MOVE in flight
    ASSERT_FALSE(Manipulator.belt.isDone());

    // Walk the clock to just below the ceiling, moving the belt AWAY from the target each
    // tick so it never arrives and the stall detector never trips.
    const uint32_t stepMs = 100;
    int32_t        cur    = 0;
    uint32_t       spent  = 0;
    while (spent + stepMs < (uint32_t) Config::BELT_MOVE_TIMEOUT_MS)
    {
        cur -= 50;    // real motion, wrong direction — never reaches +800
        seedBeltLogicalDeg(cur);
        mockAdvanceMillis(stepMs);
        spent += stepMs;
        Manipulator.belt.update();
        // Must still be driving: neither arrival nor stall applies.
        ASSERT_FALSE(Manipulator.belt.isDone());
    }

    // Cross the ceiling: the watchdog fires and forces completion.
    mockAdvanceMillis((uint32_t) Config::BELT_MOVE_TIMEOUT_MS - spent);
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
    ASSERT_EQ(MiniR4.M1.lastSpeed, (int16_t) 0);
    ASSERT_TRUE(MiniR4.M1.lastBrake);
}

// ===========================================================================
// TC10 (AC7) — blocking verb drains until the actuator settles.
//
// With a tick callback registered, _drain spins: callback() + yield() each iteration.
// yield() is a host no-op (it does NOT advance the mock clock — production relies on
// real millis() advancing while the callback ticks the modules), so the callback itself
// must model wall-clock passing AND tick the modules. grab() now runs a home->close two-stop
// stroke: the queued close is fired by Manipulator.update() once home settles, so the callback
// must advance the clock AND call update() (exactly what the sketch's cooperativeTick does) or
// the sequence would never progress past home. We register such a callback so the blocking
// grab() statement returns only once the WHOLE stroke settles (grabber.isDone()).
// ===========================================================================

static void test_TC10_blocking_grab_drains_until_done()
{
    TEST_SCOPE("TC10: blocking grab() drains until grabber.isDone() (AC7)");

    resetWorld();

    // Register a tick callback that advances the mock clock AND ticks the manipulator, mirroring
    // production's cooperativeTick. _drain calls yield() (a host no-op) each spin, so the callback
    // is the sole source of both wall-clock progress and the update() that fires the queued close.
    Manipulator.setTickCallback([]() { mockAdvanceMillis(2); Manipulator.update(); });

    // Blocking verb (no .async()) as a full statement — the temporary ManipAction's
    // destructor drains at end-of-statement.
    Manipulator.grab();

    // After the blocking drain returns, the grabber must be done — the whole home->close stroke,
    // including the queued close, has settled.
    ASSERT_TRUE(Manipulator.grabber.isDone());
    // The final commanded pose is close (the second stop of the stroke).
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) Config::GRABBER.closeDeg);

    clearTickCallback();
}

// ===========================================================================
// TC11 (AC7) — async grab() fires the open pose first, then update() stages the rest.
//
// grab() now runs a THREE-stop capture stroke: jump to openForWhiteDeg INSTANTLY, ramp up to
// homeDeg (the queued slow lift), then snap to closeDeg. async() gates the wait, not the firing,
// so right after the async returns the grabber channel holds the open pose and isDone() is false
// (follow-up stops are queued). Driving the clock + update() advances the lift and the close; the
// stroke is done only once the FINAL close settles.
// ===========================================================================

static void test_TC11_async_grab_sequences_open_lift_close()
{
    TEST_SCOPE("TC11: grab().async() fires the open pose first, then update() advances to close (AC7)");

    resetWorld();
    clearTickCallback();

    Manipulator.grab().async();    // fires the open pose now, queues lift+close; does NOT drain

    // Async gates the wait, not the firing: the FIRST stop (openForWhiteDeg) is issued immediately,
    // and it is an instant stop so the channel holds it in one write.
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) Config::GRABBER.openForWhiteDeg);
    // Not done — the lift + close stops are queued, so isDone() is false regardless of the timer.
    ASSERT_FALSE(Manipulator.grabber.isDone());

    // Drive the whole stroke to completion: the slow lift crawls one degree per step, then the
    // instant close lands. The loop advances the clock and ticks the servo until it settles.
    for (int i = 0; i < 100000 && !Manipulator.grabber.isDone(); ++i)
    {
        mockAdvanceMillis(2);
        Manipulator.update();
    }

    // The final stop (close) is the last commanded pose, and the whole stroke is now done.
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) Config::GRABBER.closeDeg);
    ASSERT_TRUE(Manipulator.grabber.isDone());
}

// ===========================================================================
// TC12 — reset() homes all three servos and runs the belt home path.
// ===========================================================================

static void test_TC12_reset_homes_all_servos_and_belt()
{
    TEST_SCOPE("TC12: reset().async() homes all three servos + invokes belt.home()");

    resetWorld();
    clearTickCallback();

    // Move every servo OFF home first so the home writes are observable. Force the instant path
    // (stepMs = 0) so a servo with a slow default rate actually lands at 10 here rather than
    // deferring the write to update().
    Manipulator.grabber.setAngle(10, 0);
    Manipulator.sorter.setAngle(10, 0);
    Manipulator.dumper.setAngle(10, 0);

    const uint32_t beltResetsBefore = MiniR4.M1.resetCounterCallCount;

    Manipulator.reset().async();    // homes 3 servos + belt.home(); async so no drain

    // The grabber homes via a ramp (its configured stepMs > 0 and it was moved far off home), so
    // its home write is deferred to update(); sorter/dumper are instant. reset() used .async(), so
    // advance the grabber ramp by hand before asserting its home pose. The belt is driven
    // separately below, so tick only the grabber here.
    for (int i = 0; i < 100000 && !Manipulator.grabber.isDone(); ++i)
    {
        mockAdvanceMillis(2);
        Manipulator.grabber.update();
    }

    // Each servo commanded back to its configured home angle (raw, no SW reverse), asserted
    // on each servo's configured channel.
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) Config::GRABBER.homeDeg);
    ASSERT_EQ(rcFor(Config::SORTER.rcId).lastAngle, (uint16_t) Config::SORTER.homeDeg);
    ASSERT_EQ(rcFor(Config::DUMPER.rcId).lastAngle, (uint16_t) Config::DUMPER.homeDeg);

    // belt.home() ran: reset()'s TIMED stall toward LEFT/HOME was kicked off (setSpeed issued,
    // belt now in flight). Drive it through the full reset -> settle -> MID sequence.
    ASSERT_FALSE(Manipulator.belt.isDone());
    mockAdvanceMillis(Config::BELT_RESET_MS);
    Manipulator.belt.update();            // stall done -> reset(), enter SETTLE
    // The completing stall re-zeroed the encoder via resetCounter().
    ASSERT_EQ(MiniR4.M1.resetCounterCallCount, beltResetsBefore + 1);
    ASSERT_FALSE(Manipulator.belt.isDone());    // settling
    mockAdvanceMillis(Config::BELT_HOME_SETTLE_MS);
    Manipulator.belt.update();            // settle done -> toStation(MID) issued (still in flight)
    ASSERT_FALSE(Manipulator.belt.isDone());
    // Deliver the belt to MID to finish the chained move.
    seedBeltLogicalDeg(Config::BELT_STATIONS[(uint8_t) Config::BeltStation::MID].deg);
    mockAdvanceMillis(20);
    Manipulator.belt.update();
    ASSERT_TRUE(Manipulator.belt.isDone());
}

// ===========================================================================
// TC15 (AC8) — begin() drives every actuator to its boot pose (grabber CLOSED,
// sorter/dumper home, belt homed) and zeroes the belt encoder exactly once.
//
// This test deliberately does NOT use resetWorld() (which calls begin() then wipes the
// mock + clock); it drives begin() directly off a freshly-wiped mock/clock so the
// begin()-time recording is preserved for assertion.
//
// --- TC16 OMITTED (host-untriggerable) -------------------------------------
// TC16 would force begin()'s belt home to exceed BELT_HOME_TIMEOUT_MS (the
// timeout-returns-false branch in ManipBelt::begin). It is NOT host-triggerable:
//   1. The T8 MockDC exposes NO fault-injection hook — every vendor call
//      (setSpeed/rotateFor/setBrake/resetCounter/setPPR_RPM/setReverse/ChkRotateEnd)
//      returns true unconditionally, and there is no hook to hold the home incomplete.
//   2. home() is a three-phase sequence: a TIMED stall (completes at >= BELT_RESET_MS =
//      2000 ms), a SETTLE dwell (BELT_HOME_SETTLE_MS), then a MID POSITION move. The
//      mock holds ChkRotateEnd false, so the MID phase completes via its own POSITION
//      watchdog (BELT_MOVE_TIMEOUT_MS = 4000 ms after it starts). begin()'s poll loop
//      advances the clock with delay(2) each spin, bounded by `millis()-start <
//      BELT_HOME_TIMEOUT_MS` (= BELT_RESET_MS + BELT_HOME_SETTLE_MS + BELT_MOVE_TIMEOUT_MS +
//      1000 ms). Since the whole sequence finishes well inside that ceiling, the home ALWAYS
//      completes before it on host — by construction.
// The timeout-returns-false branch is therefore verified by inspection
// (ManipBelt::begin lines: `if (!isDone()) { _setBrake(true); ... return false; }`),
// and the bounded-watchdog behavior it guards against is covered by TC9. Faking a
// failure here (e.g. a speculative mock hook that does not exist) would not exercise
// real production behavior, so TC16 is intentionally not implemented.
// ===========================================================================

static void test_TC15_begin_homes_all_and_zeroes_belt_once()
{
    TEST_SCOPE("TC15: begin() returns true, all done, belt resetCounter once (AC8)");

    mockResetMiniR4();
    mockResetMillis();

    const bool ok = Manipulator.begin();

    ASSERT_TRUE(ok);
    // All four actuators report done once begin() returns.
    ASSERT_TRUE(Manipulator.grabber.isDone());
    ASSERT_TRUE(Manipulator.sorter.isDone());
    ASSERT_TRUE(Manipulator.dumper.isDone());
    ASSERT_TRUE(Manipulator.belt.isDone());
    ASSERT_TRUE(Manipulator.isDone());
    // Boot poses: grabber CLOSED, sorter/dumper at home (raw angles, no SW reverse).
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) Config::GRABBER.closeDeg);
    ASSERT_EQ(rcFor(Config::SORTER.rcId).lastAngle, (uint16_t) Config::SORTER.homeDeg);
    ASSERT_EQ(rcFor(Config::DUMPER.rcId).lastAngle, (uint16_t) Config::DUMPER.homeDeg);
    // The belt home zeroed the encoder exactly once during begin().
    ASSERT_EQ(MiniR4.M1.resetCounterCallCount, (uint32_t) 1);

    // Leave the world reset for any later test ordering hygiene.
    mockResetMiniR4();
    mockResetMillis();
}

// ===========================================================================
// TCR1 / TCR2 — stepped-ramp speed control (stepMs > 0).
//
// Exercise the ramp on a LOCAL ServoCfg fixture with stepMs > 0, so the behaviour is asserted
// directly and independent of whatever rate the production servos ship with. The fixture is wired
// to a real RC channel (id 2) so _writeAngle() dispatches through the mock; assertions read the
// crawling shadow via commandedAngle().
// ===========================================================================

// home 90 / close 180 / msPerDeg 5 / settle 20 / stepMs 10 (10 ms per 1° step).
static constexpr Config::ServoCfg RAMP_CFG = { /*rcId*/ 2,     /*home*/ 90,  /*open*/ 0,
                                               /*openWhite*/ 0, /*close*/ 180, /*msPerDeg*/ 5.0f,
                                               /*settle*/ 20,   /*reverse*/ false, /*stepMs*/ 10 };

static void test_TCR1_ramp_steps_one_degree_per_interval()
{
    TEST_SCOPE("TCR1: a stepMs>0 servo crawls one degree per step interval, done after settle");

    resetWorld();
    clearTickCallback();

    ManipServo s(RAMP_CFG);
    s.begin();    // homes to 90 (delta 0 -> instant)
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 90);

    s.setAngle(95);    // arm a ramp 90 -> 95 at the default rate (10 ms/deg)

    // The ramp is deferred to update(): nothing is written yet, and it is NOT done.
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 90);
    ASSERT_FALSE(s.isDone());

    // Each 10 ms interval advances the commanded angle by exactly one degree.
    for (uint16_t expected = 91; expected <= 95; ++expected)
    {
        mockAdvanceMillis(10);
        s.update();
        ASSERT_EQ(s.commandedAngle(), expected);
    }

    // Target reached, but still settling — not done until the settle dwell elapses.
    ASSERT_FALSE(s.isDone());
    mockAdvanceMillis(RAMP_CFG.settleMs);
    ASSERT_TRUE(s.isDone());
}

static void test_TCR2_setangle_override_rate()
{
    TEST_SCOPE("TCR2: setAngle(deg, stepMs) overrides the per-move step rate (0 = instant)");

    resetWorld();
    clearTickCallback();

    ManipServo s(RAMP_CFG);
    s.begin();    // at 90

    // Explicit instant override: the full target is written immediately, no crawl.
    s.setAngle(120, 0);
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 120);

    // Explicit slow override for the next move: deferred, then one degree per 25 ms.
    s.setAngle(130, 25);
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 120);    // deferred to update()
    mockAdvanceMillis(25);
    s.update();
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 121);
    mockAdvanceMillis(25);
    s.update();
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 122);

    // A subsequent default-rate move restores the servo's configured rate (one-shot override).
    s.setAngle(122);    // delta 0 from current shadow -> instant, settles on the timer
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 122);
}

static void test_TCR3_movewaypoints_mixes_per_stop_rates()
{
    TEST_SCOPE("TCR3: moveWaypoints runs each stop at its own rate (instant / slow / instant)");

    resetWorld();
    clearTickCallback();

    ManipServo s(RAMP_CFG);
    s.begin();    // homes to 90

    // Stroke: 90 -> 100 INSTANT, 100 -> 130 SLOW (15 ms/deg), 130 -> 120 INSTANT.
    const ManipServo::Waypoint stroke[3] = { { 100, 0 }, { 130, 15 }, { 120, 0 } };
    s.moveWaypoints(stroke, 3);

    // Stop 1 is instant: it lands in one write. Stops 2,3 are still queued, so not done.
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 100);
    ASSERT_FALSE(s.isDone());

    // Drive until the servo moves off stop 1 — i.e. stop 2 (the slow ramp) has begun. The first
    // move off 100 must be a SINGLE degree toward 130 (a crawl), not a jump to 130.
    int guard = 0;
    while (s.commandedAngle() == 100 && guard++ < 100000)
    {
        mockAdvanceMillis(2);
        s.update();
    }
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 101);

    // Drive the rest to completion; the final stop (120) is instant.
    while (!s.isDone() && guard++ < 300000)
    {
        mockAdvanceMillis(2);
        s.update();
    }
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 120);
    ASSERT_TRUE(s.isDone());
}

static void test_TCR4_grab_stages_instant_open_slow_lift_instant_close()
{
    TEST_SCOPE("TCR4: grab() jumps to the open pose instantly, lifts to home slowly, closes instantly");

    resetWorld();
    clearTickCallback();

    Manipulator.grab().async();    // fire the stroke, don't drain

    // Stop 1 (openForWhiteDeg) is instant: the channel holds the open pose right away.
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) Config::GRABBER.openForWhiteDeg);
    ASSERT_FALSE(Manipulator.grabber.isDone());    // lift + close still queued

    // Drive until the servo moves off the open pose — the lift (stop 2) has begun. The lift is a
    // SLOW ramp, so the first move off the open pose is a single degree toward home (homeDeg >
    // openForWhiteDeg here), NOT a jump straight to home.
    int guard = 0;
    while (Manipulator.grabber.commandedAngle() == Config::GRABBER.openForWhiteDeg && guard++ < 100000)
    {
        mockAdvanceMillis(2);
        Manipulator.update();
    }
    ASSERT_EQ(Manipulator.grabber.commandedAngle(), (uint16_t) (Config::GRABBER.openForWhiteDeg + 1));

    // Drive to completion; the final close is instant and the stroke ends done.
    while (!Manipulator.grabber.isDone() && guard++ < 300000)
    {
        mockAdvanceMillis(2);
        Manipulator.update();
    }
    ASSERT_EQ(rcFor(Config::GRABBER.rcId).lastAngle, (uint16_t) Config::GRABBER.closeDeg);
    ASSERT_TRUE(Manipulator.grabber.isDone());
}

// ===========================================================================
// TCR5 / TCR6 — a rate override applied AFTER an instant-default verb already wrote its target.
//
// This is the bug where `.rampMs()`/`.instant()` behaved like `.async()`: an instant-default verb
// writes the target eagerly (shadow == target), so a naive re-arm sees delta 0 and collapses the
// budget to settle-only (and never ramps). setStepMs() must rewind to the move's start.
// ===========================================================================

// Instant-default fixture (stepMs = 0): models a verb whose servo default writes eagerly.
static constexpr Config::ServoCfg INSTANT_CFG = { /*rcId*/ 2,     /*home*/ 90,  /*open*/ 0,
                                                  /*openWhite*/ 0, /*close*/ 150, /*msPerDeg*/ 5.0f,
                                                  /*settle*/ 20,   /*reverse*/ false, /*stepMs*/ 0 };

static void test_TCR5_rampms_override_after_instant_write_crawls()
{
    TEST_SCOPE("TCR5: .rampMs() override after an instant write rewinds and actually ramps");

    resetWorld();
    clearTickCallback();

    ManipServo s(INSTANT_CFG);
    s.begin();    // at 90

    // Instant default: the move to 150 is written immediately (shadow == target).
    s.setAngle(150);
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 150);

    // Override to a ramp: it must rewind to the move's start (90) and crawl — NOT no-op because the
    // shadow is already at the target.
    s.setStepMs(10);
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 90);    // rewound to the start
    ASSERT_FALSE(s.isDone());
    mockAdvanceMillis(10);
    s.update();
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 91);    // crawling toward 150

    int guard = 0;
    while (!s.isDone() && guard++ < 100000)
    {
        mockAdvanceMillis(2);
        s.update();
    }
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 150);
    ASSERT_TRUE(s.isDone());
}

static void test_TCR6_instant_override_keeps_full_slew_budget()
{
    TEST_SCOPE("TCR6: .instant() after an instant write keeps the full slew budget (not settle-only)");

    resetWorld();
    clearTickCallback();

    ManipServo s(INSTANT_CFG);
    s.begin();    // at 90

    // Instant move 90 -> 150; the completion budget is |150-90| * msPerDeg + settle = 320 ms.
    s.setAngle(150);
    // .instant() override. The bug collapsed this to settle-only (20 ms) — so a blocking drain
    // barely waited and looked like .async(). The rewind must preserve the full 320 ms slew budget.
    s.setStepMs(0);
    ASSERT_EQ(s.commandedAngle(), (uint16_t) 150);    // still commands the target in one write

    const uint32_t budget = (uint32_t) (60 * INSTANT_CFG.msPerDeg) + INSTANT_CFG.settleMs;    // 320
    mockAdvanceMillis(budget - 1);
    ASSERT_FALSE(s.isDone());    // NOT done just under the full slew budget (would fail with the bug)
    mockAdvanceMillis(1);
    ASSERT_TRUE(s.isDone());     // done only once the full budget elapses
}

// ===========================================================================
// Test entry point.
// ===========================================================================

int main()
{
    test_TC1_grabber_setangle_clamps_to_180();
    test_TC1b_dumper_open_for_white_uses_dedicated_pose();
    test_TC2_grabber_not_done_below_estimate();
    test_TC3_grabber_done_at_estimate();
    test_TC4_sortto_valid_bin_commands_sorter();
    test_TC5_sortto_out_of_range_no_write_and_done();
    test_TC6_belt_position_move_to_mid();
    test_TC6b_belt_position_already_there_is_noop();
    test_TC6c_belt_position_drives_negative_when_target_below();
    test_TC6d_belt_position_stall_ends_move();
    test_TC6e_belt_position_slow_creep_is_not_a_stall();
    test_TC7_belt_timed_move_to_far_end();
    test_TC8_belt_home_resets_settles_then_moves_to_mid();
    test_TC9_belt_position_watchdog_fires();
    test_TC10_blocking_grab_drains_until_done();
    test_TC11_async_grab_sequences_open_lift_close();
    test_TC12_reset_homes_all_servos_and_belt();
    test_TC15_begin_homes_all_and_zeroes_belt_once();
    test_TC16_beltto_verb_delegates_to_belt();
    test_TCR1_ramp_steps_one_degree_per_interval();
    test_TCR2_setangle_override_rate();
    test_TCR3_movewaypoints_mixes_per_stop_rates();
    test_TCR4_grab_stages_instant_open_slow_lift_instant_close();
    test_TCR5_rampms_override_after_instant_write_crawls();
    test_TCR6_instant_override_keeps_full_slew_budget();
    TEST_REPORT();
}
