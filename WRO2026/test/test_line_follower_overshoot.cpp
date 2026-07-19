/**
 * @file test_line_follower_overshoot.cpp
 * @brief Host-compile tests for the LineFollower overshoot-mode + brake-on-gate
 *        + junction-freeze interactions (T10a / T10b / T10d).
 *
 * Built against the mock Arduino / Drive / WROLineArray / Intersection surface
 * under `WRO2026/test/mocks/`. The real `WRO2026/LineFollower.{h,cpp}` and
 * `WRO2026/WROMath.h` are compiled unchanged — the test exercises the actual
 * production state machine.
 *
 * Test cases:
 *   - T10a: setOvershootMode(true) dead-reckons symmetric cruise on isOnline()==false
 *           (no LOST_LINE_ACTION dispatch). With (false), today's LOST_LINE_ACTION
 *           fires unchanged. The encoder distance gate continues to tick across
 *           the offline window, so the gate trips at the target distance.
 *   - T10b: setBrakeOnGate(false) — gate trip latches isDistanceReached() but
 *           does NOT self-disable + does NOT call Drive.stop. Inverse case
 *           (default brake=true) keeps today's behavior.
 *   - T10d: setOvershootMode(true) + junction classifier returning T_OR_CROSS:
 *           junction-freeze branch (`_pidFrozen`) fires; symmetric cruise is
 *           commanded.
 *
 * Each test resets the mock world (Drive encoders/records, LineArray scripted
 * state) at scope entry so scenarios are fully independent.
 */

#include "TestAssert.h"

#include "Arduino.h"
#include "Drive.h"
#include "WROLineArray.h"
#include "Intersection.h"

// Production code under test:
#include "src/drive/LineFollower.h"
#include "Config.h"
#include "src/util/WROMath.h"

// ---------------------------------------------------------------------------
// Shared helpers.
// ---------------------------------------------------------------------------

/// Reset every mock and re-init the LineFollower for a fresh scenario.
///
/// The production `LineFollower::begin()` does NOT touch `_brakeOnGate`,
/// `_overshootMode`, `_distanceTargetActive`, or the distance-gate latch — on
/// the real robot `begin()` runs once at boot so cross-call hygiene is not
/// needed. For tests we run multiple scenarios in one process, so we issue a
/// `setEnabled(false)` after `begin()` — the disable path in
/// `_applyEnableBuilder` explicitly resets every cross-tick field to its
/// documented default. This matches the boot-time invariants the production
/// code assumes.
static void resetWorld()
{
    mockResetMillis();
    Drive.mockReset();
    LineArray.mockReset();
    Intersection.mockReset();
    LineFollower.begin();
    // Force a clean disable to reset cross-tick fields (`_brakeOnGate`,
    // `_overshootMode`, `_distanceTargetActive`, `_distanceReached`,
    // `_pidFrozen`) to their documented defaults — see disable branch in
    // `LineFollower::_applyEnableBuilder`.
    LineFollower.setEnabled(false);
    // Clear out any Drive.stop() the disable path issued so the per-test
    // assertions start from zero stop-count.
    Drive.mockReset();
}

/// Convert a cm of travel to encoder degrees (matches WROMath::cmToEncDeg).
/// Tests use this to feed the mock encoders just past the 5 cm gate so the
/// distance-gate path actually trips.
static int32_t encDegsForCm(float cm)
{
    return (int32_t) cmToEncDeg(cm);
}

// ===========================================================================
// T10a — Overshoot mode dead-reckon vs. legacy LOST_LINE_ACTION.
// ===========================================================================

static void test_T10a_overshoot_mode_dead_reckons_symmetric_cruise()
{
    TEST_SCOPE("T10a: setOvershootMode(true) dead-reckons on isOnline()==false");

    resetWorld();

    // Arm: enable + overshoot-mode + 5 cm distance target. Disable ramp-up
    // so the cruise plateau is immediately full-preset (simpler to assert).
    LineArray.mockSetOnline(true);
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setOvershootMode(true)
                .setDistanceTarget(5.0f);

    ASSERT_TRUE(LineFollower.isEnabled());

    // Pre-condition: no stop, no setSpeed yet.
    ASSERT_EQ(Drive.mockRecord().stopCallCount,     (uint32_t) 0);
    ASSERT_EQ(Drive.mockRecord().setSpeedCallCount, (uint32_t) 0);

    // Drop the line — overshoot-mode should dead-reckon symmetric cruise.
    LineArray.mockSetOnline(false);

    const uint32_t setSpeedBefore = Drive.mockRecord().setSpeedCallCount;
    const uint32_t stopBefore     = Drive.mockRecord().stopCallCount;

    // Tick the controller — offline branch with overshoot-mode on.
    LineFollower.update();

    // Expected: ONE new setSpeed call with symmetric L == R.
    ASSERT_EQ(Drive.mockRecord().setSpeedCallCount, setSpeedBefore + 1);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, Drive.mockRecord().lastSetSpeedR);

    // PWM magnitude is the deadband-snapped preset cruise (NORMAL).
    const int16_t expected = deadbandSnap(presetToPwm(LineFollower.getPreset()));
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expected);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expected);

    // No Drive.stop() — that would mean LOST_LINE_ACTION::STOP fired.
    ASSERT_EQ(Drive.mockRecord().stopCallCount, stopBefore);

    // Controller stayed enabled — overshoot-mode does NOT self-disable.
    ASSERT_TRUE(LineFollower.isEnabled());

    // Line-lost state is still true (overshoot just suppresses the dispatch).
    ASSERT_TRUE(LineFollower.isLineLost());

    // Now advance the encoders past the 5 cm target so the distance gate trips
    // even though the line is still offline.
    Drive.mockAdvanceEncoders(encDegsForCm(6.0f), encDegsForCm(6.0f));

    // Default _brakeOnGate is `true` since we did NOT chain setBrakeOnGate(false)
    // in this scenario — so the gate trip self-disables with brake.
    LineFollower.update();

    ASSERT_TRUE(LineFollower.isDistanceReached());    // one-shot latch, clears
    ASSERT_FALSE(LineFollower.isEnabled());           // self-disabled
    // Either the most recent stop call was brake=true (preferred) OR at least one
    // brake-true stop fired during the gate handler.
    ASSERT_GE(Drive.mockRecord().stopBrakeTrueCount, (uint32_t) 1);
}

static void test_T10a_inverse_legacy_lost_line_action_dispatches_stop()
{
    TEST_SCOPE("T10a inverse: setOvershootMode(false) preserves LOST_LINE_ACTION");

    resetWorld();

    // Arm: enable WITHOUT overshoot-mode (default false). Default
    // Config::LOST_LINE_ACTION is STOP, which calls Drive.stop(true).
    LineArray.mockSetOnline(true);
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setOvershootMode(false)
                .setDistanceTarget(5.0f);

    ASSERT_TRUE(LineFollower.isEnabled());

    // Drop the line — legacy LOST_LINE_ACTION dispatch.
    LineArray.mockSetOnline(false);

    const uint32_t setSpeedBefore = Drive.mockRecord().setSpeedCallCount;
    const uint32_t stopBefore     = Drive.mockRecord().stopCallCount;

    LineFollower.update();

    // STOP action: Drive.stop(true) was called exactly once; no extra setSpeed.
    ASSERT_EQ(Drive.mockRecord().setSpeedCallCount, setSpeedBefore);
    ASSERT_EQ(Drive.mockRecord().stopCallCount,     stopBefore + 1);
    ASSERT_TRUE(Drive.mockRecord().lastStopBrake);

    // Distance gate did NOT run (early-return on the legacy path).
    // The controller stayed enabled (LOST_LINE_ACTION::STOP does not self-disable;
    // it just brakes and waits for the line to come back).
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_TRUE(LineFollower.isLineLost());
}

// ===========================================================================
// T10b — setBrakeOnGate(false) keeps controller enabled across gate trip.
// ===========================================================================

static void test_T10b_brake_on_gate_false_keeps_controller_enabled()
{
    TEST_SCOPE("T10b: setBrakeOnGate(false) — gate trip latches but no self-disable");

    resetWorld();

    // Arm: line ONLINE the whole time (so the P+D path runs and the gate evaluates).
    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setOvershootMode(true)
                .setBrakeOnGate(false)
                .setDistanceTarget(5.0f);

    ASSERT_TRUE(LineFollower.isEnabled());

    const uint32_t stopBefore = Drive.mockRecord().stopCallCount;

    // Tick once below the gate — should command setSpeed at cruise, no stop.
    LineFollower.update();
    ASSERT_GE(Drive.mockRecord().setSpeedCallCount, (uint32_t) 1);
    ASSERT_EQ(Drive.mockRecord().stopCallCount,     stopBefore);
    ASSERT_TRUE(LineFollower.isEnabled());

    // Advance encoders past the gate.
    Drive.mockAdvanceEncoders(encDegsForCm(6.0f), encDegsForCm(6.0f));

    const uint32_t setSpeedBefore = Drive.mockRecord().setSpeedCallCount;
    LineFollower.update();

    // Distance gate fired — latch set.
    ASSERT_TRUE(LineFollower.isDistanceReached());
    // No new stop call — pass-through mode keeps motors live.
    ASSERT_EQ(Drive.mockRecord().stopCallCount, stopBefore);
    // Controller still enabled — PathTracker keeps polling.
    ASSERT_TRUE(LineFollower.isEnabled());
    // setSpeed was called on this tick (controller still drives the wheels).
    ASSERT_GT(Drive.mockRecord().setSpeedCallCount, setSpeedBefore);
}

static void test_T10b_inverse_brake_on_gate_true_self_disables()
{
    TEST_SCOPE("T10b inverse: default brake-on-gate=true self-disables + brakes");

    resetWorld();

    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setOvershootMode(true)    // overshoot has no effect when line is online
                // brake-on-gate defaults to true — do NOT chain setBrakeOnGate.
                .setDistanceTarget(5.0f);

    ASSERT_TRUE(LineFollower.isEnabled());

    // Advance past the gate.
    Drive.mockAdvanceEncoders(encDegsForCm(6.0f), encDegsForCm(6.0f));

    const uint32_t stopBefore = Drive.mockRecord().stopCallCount;
    LineFollower.update();

    ASSERT_TRUE(LineFollower.isDistanceReached());
    ASSERT_FALSE(LineFollower.isEnabled());                                   // self-disabled
    ASSERT_GE(Drive.mockRecord().stopBrakeTrueCount, (uint32_t) 1);           // active brake
    ASSERT_GT(Drive.mockRecord().stopCallCount, stopBefore);
}

// ===========================================================================
// T10d — Overshoot mode + junction-freeze (AC9).
// ===========================================================================

static void test_T10d_junction_freeze_fires_while_overshoot_mode_enabled()
{
    TEST_SCOPE("T10d: junction-freeze fires while overshoot mode enabled (AC9)");

    resetWorld();

    // Arm: enable overshoot, line ONLINE, classifier reports T_OR_CROSS.
    // Junction-freeze runs ONLY on the line-online path — that's the AC9 claim.
    LineArray.mockSetOnline(true);
    LineArray.mockSetJunctionType(JunctionType::T_OR_CROSS);
    LineArray.mockSetError(2.0f);    // Non-zero error to prove freeze ignores it.

    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setOvershootMode(true)
                .setDistanceTarget(5.0f);

    ASSERT_TRUE(LineFollower.isEnabled());

    LineFollower.update();

    // Junction-freeze branch ran (line online + non-NONE/non-UNKNOWN type).
    ASSERT_TRUE(LineFollower.isPidFrozen());

    // P+D was bypassed → symmetric cruise was commanded (L == R, both at the
    // deadband-snapped preset PWM).
    const int16_t expected = deadbandSnap(presetToPwm(LineFollower.getPreset()));
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expected);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expected);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, Drive.mockRecord().lastSetSpeedR);

    // Line is online — `_lostLine` flag is cleared.
    ASSERT_FALSE(LineFollower.isLineLost());
}

// ===========================================================================
// T-rampup — tShape ramp-up STARTS at the deadband floor (TSHAPE_START_SPEED),
// not 0. Regression for the start-speed bug: the ramp-up seeded
// `_tShapeStartPwm = 0`, so on a fresh off->on enable the first tick commands
// cruise 0 (on a centered line, P+D corr is 0 → setSpeed(0, 0)), the chassis
// does not move, and the distance-gated ramp cannot advance. The fix seeds the
// start from `Config::TSHAPE_START_SPEED` (== PWM_MIN_MAGNITUDE, the motor
// deadband floor), sign-corrected for reverse — mirroring the ramp-DOWN floor
// and Drive's sibling trapezoid. See Config.h TSHAPE_START_SPEED doc + ADR-018.
//
// These tests deliberately leave ramp-up ON (every other test in this file
// disables it via setTShapeRampUp(false), which is why the bug slipped past CI).
// ===========================================================================

static void test_rampup_starts_at_deadband_floor_forward()
{
    TEST_SCOPE("ramp-up: first tick commands TSHAPE_START_SPEED, not 0 (forward)");

    resetWorld();

    // Line online, centered (err 0), no junction → the P+D path runs with
    // corr == 0, so L == R == cruise. Encoders are 0 after resetWorld(), so the
    // ramp-up traveled fraction is 0 and the first-tick cruise == the ramp START
    // pwm. Distance target armed (mirrors a PROJECTION_FOLLOW segment) but the
    // 50 cm gate is far away, so it cannot trip on tick 1.
    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    LineFollower.setEnabled(true).setDistanceTarget(50.0f);    // ramp-up default ON
    ASSERT_TRUE(LineFollower.isEnabled());

    LineFollower.update();

    const int16_t expected = deadbandSnap(Config::TSHAPE_START_SPEED);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expected);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expected);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, Drive.mockRecord().lastSetSpeedR);
}

static void test_rampup_starts_at_deadband_floor_reverse()
{
    TEST_SCOPE("ramp-up: first tick commands -TSHAPE_START_SPEED (reverse)");

    resetWorld();

    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    // Fresh enable in reverse with ramp-up ON. The start pwm must be NEGATED so
    // the ramp stays on the negative-direction side from tick 1 (a positive
    // start would drive the chassis briefly FORWARD before reversing).
    LineFollower.setEnabled(true).setReverse(true).setDistanceTarget(50.0f);
    ASSERT_TRUE(LineFollower.isEnabled());

    LineFollower.update();

    const int16_t expected = deadbandSnap((int16_t) -Config::TSHAPE_START_SPEED);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expected);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expected);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, Drive.mockRecord().lastSetSpeedR);
}

// ===========================================================================
// TC3 — Anticipatory approach-ramp monotonic detect handoff (AC3, 2026-07-17
// line-approach-ramp plan, T11).
//
// Mirrors PathTracker's real dispatch pair over the LineFollower fluent surface
// directly (T8's `LINE_FOLLOW_INTERSECTION` arm, then the `LINE_FOLLOW` ->
// `LINE_OVERSHOOT` re-arm at PathTracker.cpp:1027-1033) so the REAL
// `LineFollower::_applyEnableBuilder` / `update()` drive the handoff, exactly as
// the T10a/T10b/T10d tests above drive the real controller directly rather than
// mirroring it. Reuses the file's `deadbandSnap(...)` exact-value assertion
// mechanism (see T10a at line ~117 and T10d at line ~292).
//
// T11 FINDING: writing this test surfaced a real defect in the already-merged T6
// ramp-down "cruise plateau" branch (the `else` arm of the `remainingCm <=
// rampDownCm` check) — it fell through to the top-of-tick `presetToPwm(_preset)`
// default instead of holding the AC3-seeded `_tShapeRampDownStart`, so a hop
// whose approach ramp reached FLOOR_HOLD before detection (the common,
// by-design case — AC1) jolted the commanded PWM back UP to full cruise for the
// plateau portion of the LINE_OVERSHOOT window before ramping back down. Fixed
// in `LineFollower.cpp`'s ramp-down block (holds `_tShapeRampDownStart` in the
// plateau when ramp-up is not concurrently in flight); verified byte-identical
// for the pre-feature / ramp-up-concurrent paths by the full existing suite.
// ===========================================================================

static void test_TC3_approach_ramp_floor_hold_monotonic_detect_handoff()
{
    TEST_SCOPE("TC3: approach-ramp FLOOR_HOLD magnitude is non-increasing into LINE_OVERSHOOT (AC3)");

    resetWorld();

    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    // Arm the approach ramp directly (mirrors PathTracker's LINE_FOLLOW_INTERSECTION
    // dispatch, T8): disable both tShape ramps so the approach branch alone shapes
    // cruise. predictedCm=100 is comfortably above TSHAPE_MIN_MOVE_CM/the ramp length,
    // and — because LINE_APPROACH_MARGIN_CM > 0 and LINE_APPROACH_MAX_MARGIN_CM > 0 are
    // ConfigChecks-enforced invariants — advancing to `traveled == predictedCm` ALWAYS
    // lands strictly inside [floorStartCm, overrunLimitCm) -> FLOOR_HOLD, regardless of
    // the exact tuned margin/accel/ramp values.
    constexpr float predictedCm = 100.0f;
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setApproachTarget(predictedCm);
    ASSERT_TRUE(LineFollower.isEnabled());

    Drive.mockAdvanceEncoders(encDegsForCm(predictedCm), encDegsForCm(predictedCm));
    LineFollower.update();

    ASSERT_TRUE(LineFollower.approachPhase() == ApproachPhase::FLOOR_HOLD);

    // The dispatched magnitude at the floor is the KINETIC floor (PWM_MOVING_MIN), not
    // the static deadband floor: past CRUISE the approach branch lowers `stopFloorMag`
    // to PWM_MOVING_MIN so `deadbandSnap` does not re-snap the ramped-down magnitude back
    // up to the static floor (see the ApproachRampPolicy.h / update() docs).
    const int16_t approachFloorMag = Drive.mockRecord().lastSetSpeedL;
    ASSERT_EQ(approachFloorMag, deadbandSnap(Config::PWM_MOVING_MIN, Config::PWM_MOVING_MIN));
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, Drive.mockRecord().lastSetSpeedR);

    // Detect tick: mirror PathTracker's LINE_OVERSHOOT re-arm (PathTracker.cpp:1027-1033)
    // — overshoot-mode, ramp-up off, ramp-DOWN on, a fresh distance target. This does NOT
    // chain `.setApproachTarget(...)`, so the schedule disarms (the LINE_OVERSHOOT
    // re-arm contract) — the sentinel reads CRUISE — and the T6 AC3 seed reads the
    // just-disarmed `_approachLastMag` (== the floor magnitude above) to seed the
    // ramp-down, instead of jolting back to `presetToPwm(_preset)`.
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(true)
                .setOvershootMode(true)
                .setDistanceTarget(Config::LINE_SENSOR_AXLE_OFFSET_CM);
    ASSERT_TRUE(LineFollower.approachPhase() == ApproachPhase::CRUISE);

    LineFollower.update();

    // THE MONOTONIC ASSERT (AC3 / F2): the first LINE_OVERSHOOT tick's commanded
    // magnitude must be <= the last approach-leg magnitude — never a jolt back toward
    // full preset cruise on the detect tick.
    const int16_t overshootFirstMag = Drive.mockRecord().lastSetSpeedL;
    ASSERT_TRUE(overshootFirstMag <= approachFloorMag);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, Drive.mockRecord().lastSetSpeedR);

    // Non-regression: a second overshoot tick (no further travel) stays flat at the
    // floor too — no delayed jolt once the plateau/ramp math re-evaluates.
    LineFollower.update();
    ASSERT_TRUE(Drive.mockRecord().lastSetSpeedL <= approachFloorMag);
}

// ===========================================================================
// TC8 (part 1, AC13 regression companion) — with the approach ramp NEVER armed
// (no `.setApproachTarget(...)` chained on the first enable — the
// `Config::LINE_APPROACH_RAMP_ENABLED == false` / pass-through shape), the
// LINE_OVERSHOOT re-arm seeds from `presetToPwm(_preset)` exactly as before this
// feature existed — the fix above must not perturb this path (byte-identical).
// ===========================================================================

static void test_TC8_no_approach_ramp_overshoot_seed_is_byte_identical_to_preset()
{
    TEST_SCOPE("TC8: no approach ramp armed -> LINE_OVERSHOOT still seeds from presetToPwm(_preset)");

    resetWorld();

    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    // Plain LINE_FOLLOW arm — no `.setApproachTarget(...)` chained, matching a
    // pass-through hop or `LINE_APPROACH_RAMP_ENABLED == false`.
    LineFollower.setEnabled(true).setTShapeRampUp(false).setTShapeRampDown(false);
    ASSERT_TRUE(LineFollower.approachPhase() == ApproachPhase::CRUISE);

    Drive.mockAdvanceEncoders(encDegsForCm(20.0f), encDegsForCm(20.0f));
    LineFollower.update();

    const int16_t expectedFullCruise = deadbandSnap(presetToPwm(LineFollower.getPreset()));
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expectedFullCruise);

    // Detect tick: LINE_OVERSHOOT re-arm, exactly as production dispatches it.
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(true)
                .setOvershootMode(true)
                .setDistanceTarget(Config::LINE_SENSOR_AXLE_OFFSET_CM);
    LineFollower.update();

    // Byte-identical to the pre-feature seed: the first overshoot tick still holds full
    // cruise (the legacy "plateau at presetToPwm(_preset)" value) — no regression from
    // the TC3 plateau fix, which only changes behaviour when an approach ramp seeded a
    // DIFFERENT (lower) magnitude.
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expectedFullCruise);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expectedFullCruise);
}

// ===========================================================================
// Test entry point.
// ===========================================================================

int main()
{
    test_T10a_overshoot_mode_dead_reckons_symmetric_cruise();
    test_T10a_inverse_legacy_lost_line_action_dispatches_stop();
    test_T10b_brake_on_gate_false_keeps_controller_enabled();
    test_T10b_inverse_brake_on_gate_true_self_disables();
    test_T10d_junction_freeze_fires_while_overshoot_mode_enabled();
    test_rampup_starts_at_deadband_floor_forward();
    test_rampup_starts_at_deadband_floor_reverse();
    test_TC3_approach_ramp_floor_hold_monotonic_detect_handoff();
    test_TC8_no_approach_ramp_overshoot_seed_is_byte_identical_to_preset();
    TEST_REPORT();
}
