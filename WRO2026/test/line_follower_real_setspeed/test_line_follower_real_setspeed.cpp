/**
 * @file test_line_follower_real_setspeed.cpp
 * @brief T7 host test (docs/plans/2026-07-12-kinetic-decel-tail-floor.md, TC7 / AC4):
 *        proves the kinetic decel-tail floor reaches hardware through the REAL
 *        `Drive::setSpeed`, not a mock that would false-green on the pre-snap value.
 *
 * @details
 * `test_line_follower_overshoot.cpp` (the sibling target in `WRO2026/test/`) links a MOCK
 * `DriveClass` (`mocks/Drive.h`) whose `setSpeed(...)` just records the value LineFollower
 * passed in — the value BEFORE `_applyBias` + `deadbandSnap` run. A test of the kinetic-floor
 * snap written against that mock would false-green: it would never actually execute
 * `Drive::setSpeed`'s real floor-selection/snap logic, so a bug that dropped `floorMag` on the
 * wire (or wired the wrong constant into it) would still "pass".
 *
 * This target closes that gap by linking the REAL `Drive.cpp` (+ real `LineFollower.cpp` + real
 * `YawSource.cpp`) so every tick runs the full production chain:
 *
 *   `LineFollower.update() -> Drive.setSpeed(L, R, stopFloorMag) -> _applyBias (identity;
 *    Config::MOTOR_BIAS_L/R == 1.0) -> deadbandSnap(L, floorMag) / deadbandSnap(R, floorMag)
 *    -> MiniR4.DriveDC.Move(L, R)`
 *
 * Every assertion below reads `MiniR4.DriveDC.lastMoveL` / `lastMoveR` — the DriveDC HAL mock's
 * RECORD of what was actually dispatched to the lower MCU — never a value LineFollower computed
 * internally. See `WRO2026/test/CMakeLists.txt::add_line_follower_real_force_includes` for the
 * link/include recipe rationale (the load-bearing omission is `path_mocks/LineFollower.h`: this
 * target links the REAL `LineFollowerClass` as its subject-under-test, so the mock would
 * collide with it).
 *
 * Test cases (TC7 / AC4):
 *   - TC7a: a distance-gated stop's ramp-down tail (inside the zone where
 *           `remainingCm <= rampDownCm`) reaches the DriveDC mock at exactly
 *           `+/-Config::PWM_MOVING_MIN` (30) — both forward and reverse.
 *   - TC7b: a mid-follow (no ramp-down zone active) P+D correction that mixes a wheel down
 *           below the static floor reaches the DriveDC mock at exactly
 *           `+/-Config::PWM_MIN_MAGNITUDE` (35), proving the kinetic floor does NOT leak onto
 *           the mid-follow path.
 */

#include "TestAssert.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>

#include "WROLineArray.h"    // path mock — LineArray: getError/isOnline/getJunctionType
#include "Intersection.h"    // path mock — Intersection (pulled in transitively by LineFollower.cpp)

// Production code under test — REAL headers (SKETCH_DIR leads the include path, the inversion).
#include "src/drive/Drive.h"
#include "src/drive/LineFollower.h"
#include "Config.h"
#include "src/util/WROMath.h"

#include <cmath>

// ---------------------------------------------------------------------------
// Shared helpers.
// ---------------------------------------------------------------------------

/// Reset every mock and re-init Drive + LineFollower for a fresh scenario. Mirrors the
/// `resetWorld()` idiom shared by every real-Drive-linked host test in this harness family
/// (`test_path_tracker_harness.h::resetWorld`), trimmed to the surfaces this target links.
static void resetWorld()
{
    mockResetMillis();
    mockResetMiniR4();    // resets MiniR4 facade: DriveDC (lastMoveL/R, degrees, ...), M1..M4, ...

    LineArray.mockReset();
    Intersection.mockReset();

    Drive.begin();      // seeds _moveDone/_turnDone, pushes PID, applies MOTOR_BIAS_L/R (1.0/1.0)
    Drive.clearFault();

    LineFollower.begin();
    // Force a clean disable so every cross-tick field (`_useTShape`, `_useTShapeRampDown`,
    // `_distanceTargetActive`, `_overshootMode`, `_brakeOnGate`, ...) starts at its documented
    // default — mirrors `test_line_follower_overshoot.cpp::resetWorld`.
    LineFollower.setEnabled(false);

    // The disable path above issues `Drive.stop(true)` (Move(0,0) + brake), which would leave
    // `lastMoveL/R` at 0 anyway, but reset the DriveDC record again so every per-test assertion
    // starts from a clean call-count baseline.
    mockResetMiniR4();
    LineArray.mockReset();
}

/// Convert a cm of travel to encoder degrees (mirrors `WROMath::cmToEncDeg`, used to script the
/// mock `MiniR4.M2`/`M3` encoders that `Drive::getEncL()/getEncR()` read).
static int32_t encDegsForCm(float cm)
{
    return (int32_t) cmToEncDeg(cm);
}

/// Advance both drive encoders (M2 == left, M3 == right — Config::MOTOR_LEFT_ID/RIGHT_ID == 2/3)
/// to the SAME absolute degree reading, matching how the ramp-down/distance-gate math in
/// `LineFollower::update()` averages `|dL| + |dR|` over two wheels that traveled together.
static void setBothEncodersCm(float cm)
{
    const int32_t degs = encDegsForCm(cm);
    MiniR4.M2.mockSetDegrees(degs);
    MiniR4.M3.mockSetDegrees(degs);
}

// ===========================================================================
// TC7a — Ramp-down tail reaches the DriveDC mock at +/-PWM_MOVING_MIN (35).
// ===========================================================================

static void test_TC7a_ramp_down_tail_reaches_driveDC_at_kinetic_floor_forward()
{
    TEST_SCOPE("TC7a: forward ramp-down tail dispatches at +PWM_MOVING_MIN (35)");

    resetWorld();

    // Arm: SLOW preset (cruise == Config::SPEED_SLOW == 60), no ramp-up (so cruise is
    // immediately the full preset — simpler to reason about), ramp-down ON (default), 30 cm
    // distance-gated stop. Line stays online with zero error, so the P+D mix contributes
    // nothing and the emitted wheel value is exactly the ramp-down-lerped cruise.
    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(true)
                .setPreset(Config::SpeedPreset::SLOW)
                .setDistanceTarget(30.0f);

    ASSERT_TRUE(LineFollower.isEnabled());

    // Baseline: no DriveDC.Move dispatched yet this scenario.
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 0);

    // Drive the encoders to 29.9 cm traveled (remaining == 0.1 cm). With
    // Config::TSHAPE_ACCEL_PWM_PER_CM == 4.0 and a SLOW cruise of 60, the ramp-down geometry
    // is: defaultRampCm = |60-30|/4.0 = 7.5 cm, clamped into [TSHAPE_RAMP_MIN_CM,
    // TSHAPE_RAMP_MAX_CM] = [5,20] -> stays 7.5. remainingCm (0.1) <= rampDownCm (7.5), so this
    // tick is deep in the ramp-down zone; r = 0.1/7.5 ~= 0.0133, and the lerp's inner
    // `(int16_t)((start-floor)*r)` truncates to 0 for any r < 1/25 (remaining < 0.3 cm) — so
    // the lerped cruise is EXACTLY the kinetic floor (35), not merely close to it.
    setBothEncodersCm(29.9f);

    LineFollower.update();

    // Exactly one new Move() dispatched, both wheels at +30 (forward, symmetric — zero line
    // error means zero P+D correction).
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) Config::PWM_MOVING_MIN);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, (int16_t) Config::PWM_MOVING_MIN);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) 35);

    // The distance gate has NOT tripped yet (29.9 < 30 cm target) — controller still enabled,
    // proving this tick's dispatch is a genuine ramp-down sample, not the post-brake Move(0,0).
    ASSERT_TRUE(LineFollower.isEnabled());
}

static void test_TC7a_ramp_down_tail_reaches_driveDC_at_kinetic_floor_reverse()
{
    TEST_SCOPE("TC7a: reverse ramp-down tail dispatches at -PWM_MOVING_MIN (35)");

    resetWorld();

    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);

    // Same geometry as the forward case, mirrored: `.setReverse(true)` negates cruise, and the
    // ramp-down floor sign-corrects to -PWM_MOVING_MIN (LineFollower.cpp's `targetFloor`
    // ternary on `_tShapeRampDownStart < 0`).
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(true)
                .setPreset(Config::SpeedPreset::SLOW)
                .setReverse(true)
                .setDistanceTarget(30.0f);

    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 0);

    setBothEncodersCm(29.9f);

    LineFollower.update();

    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) -Config::PWM_MOVING_MIN);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, (int16_t) -Config::PWM_MOVING_MIN);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) -35);
}

// ===========================================================================
// TC7b — Mid-follow correction is clamped to LINE_MAX_CORR and dispatched as
//        cruise +/- corr. With the current tuning (SLOW cruise 60, LINE_MAX_CORR
//        20) the LOW-side wheel (40) now sits below the retuned PWM_MIN_MAGNITUDE
//        (45), so deadbandSnap floors it up to the floor; the high side (80) is
//        unaffected. The kinetic-floor-leak guard for PROFILED moves lives in
//        test_drive_decel_floor.
// ===========================================================================

static void test_TC7b_mid_follow_correction_clamped_to_line_max_corr()
{
    TEST_SCOPE("TC7b: mid-follow correction clamps to LINE_MAX_CORR, dispatched as cruise +/- corr");

    resetWorld();

    // Arm plain line-following: SLOW preset, no ramp-up (instant full cruise), NO distance
    // target at all — so `_useTShapeRampDown && _distanceTargetActive` is false every tick and
    // `stopFloorMag` never leaves its default `Config::PWM_MIN_MAGNITUDE` (45). This is the
    // cruise/mid-follow path the ramp-down zone is explicitly NOT part of.
    LineArray.mockSetOnline(true);
    LineArray.mockSetJunctionType(JunctionType::NONE);
    LineFollower.setEnabled(true).setTShapeRampUp(false).setPreset(Config::SpeedPreset::SLOW);

    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 0);

    // Inject a large line error. On the first tick after enable, `_lastErr == 0`, so
    // `dErr == err`. With the tuned gains, `Config::LINE_KP * err + Config::LINE_KD * err` for
    // err = 15.0 sits far above `Config::LINE_MAX_CORR`, so `corr` clips to exactly
    // `Config::LINE_MAX_CORR` regardless of the precise err/gain values — a deterministic clamp.
    LineArray.mockSetError(15.0f);

    LineFollower.update();

    // Dispatched pair is cruise (SLOW preset) +/- the clamped correction, THEN deadbandSnap:
    //   L = SPEED_SLOW + LINE_MAX_CORR = 80 (inside [floor, PWM_MAX], no snap).
    //   R = SPEED_SLOW - LINE_MAX_CORR = 40, which is now BELOW the retuned PWM_MIN_MAGNITUDE
    //   (45), so deadbandSnap floors the small-side wheel UP to the floor. The PWM_MIN_MAGNITUDE
    //   35->45 retune (commit 55648f2) crossed this boundary — the earlier note "if a future
    //   retune pushes SPEED_SLOW - LINE_MAX_CORR below PWM_MIN_MAGNITUDE, restore the floor-snap
    //   assertions" now applies, so the floor-snap IS exercised on the line-follow dispatch here.
    const int16_t rawR = (int16_t) (Config::SPEED_SLOW - Config::LINE_MAX_CORR);
    const int16_t expL = (int16_t) (Config::SPEED_SLOW + Config::LINE_MAX_CORR);
    const int16_t expR =
        (rawR < Config::PWM_MIN_MAGNITUDE) ? Config::PWM_MIN_MAGNITUDE : rawR;
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, (uint32_t) 1);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, expL);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, expR);

    // The small-side wheel is genuinely below the floor pre-snap, so the deadbandSnap floor path
    // IS exercised here (unlike the pre-retune tune where 40 >= 35 and no snap engaged).
    ASSERT_TRUE(rawR < Config::PWM_MIN_MAGNITUDE);
    ASSERT_EQ(expR, (int16_t) Config::PWM_MIN_MAGNITUDE);
}

// ===========================================================================
// main
// ===========================================================================

int main()
{
    test_TC7a_ramp_down_tail_reaches_driveDC_at_kinetic_floor_forward();
    test_TC7a_ramp_down_tail_reaches_driveDC_at_kinetic_floor_reverse();
    test_TC7b_mid_follow_correction_clamped_to_line_max_corr();

    TEST_REPORT();
}
