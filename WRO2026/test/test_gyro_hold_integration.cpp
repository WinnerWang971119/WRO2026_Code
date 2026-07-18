/**
 * @file test_gyro_hold_integration.cpp
 * @brief Stateful host-link integration test for the R4-side GYRO heading-hold PD executor
 *        (ADR-072, T7, TC7-TC11 + a mid-segment-hitch companion), asserting the DISPATCHED
 *        `MiniR4.DriveDC.Move(L,R)` differential end-to-end.
 *
 * @details
 * `test_gyro_hold_policy.cpp` (T7) proves the PURE decision math in `GyroHoldPolicy.h` in
 * isolation, with zero mocks. This file is the stateful FSM-level companion — it links the
 * REAL `Drive.cpp` + `YawSource.cpp` per the issue-#106 host-link recipe (see
 * `test_turn_pd_integration.cpp` / `test_drive_decel_floor.cpp` for the canonical write-up)
 * so the DISPATCHED `MiniR4.DriveDC` PWM is observable end-to-end — post-`deadbandSnap`,
 * post-dynamic-clamp, post-wheel-mix — not just the policy's return value. `PathTracker` /
 * `Localization` / `routes.cpp` are DELIBERATELY NOT linked: `Drive.cpp` does not touch any of
 * them, so this target stays the minimal four-file link (`Drive.cpp`, `EncoderSource.cpp`,
 * `YawSource.cpp`, `path_mocks.cpp`), mirroring `test_turn_pd_integration.cpp` /
 * `test_drive_decel_floor.cpp` exactly.
 *
 * ===========================================================================================
 * THE TWO-TICK DISPATCH GOTCHA (read this before adding a case)
 * ===========================================================================================
 * `Drive::update()` runs stage 2 (`_updateProfile()`, which owns `_emitContinuous` /
 * `_emitGyroHold`) BEFORE stage 3 (`_dispatchNext()`). So the `Drive.update()` call that pops a
 * queued `.async()` move off the queue does NOT also run the move's first PD step —
 * `_profileState` is still `IDLE` when stage 2 runs that tick. The FIRST REAL PD STEP (the
 * first `MiniR4.DriveDC.Move(...)`) happens on the NEXT `Drive.update()` call. This is IDENTICAL
 * to `test_turn_pd_integration.cpp`'s documented gotcha for the turn FSM — see that file's
 * docblock for the underlying `Drive.cpp` code citation. Every case below ticks once to
 * dispatch, THEN ticks again to observe the PD.
 *
 * The SAME gotcha applies, doubled, at a CHAINED segment boundary (TC8): the tick that finishes
 * segment 1 ALSO dispatches segment 2 (stage 2 finishes seg1 -> `_moveDone=true` -> stage 3
 * sees `_isIdle() && qCount>0` -> dispatches seg2, same tick, no Move() call from either side)
 * — so segment 2's first REAL PD step is the tick AFTER THAT.
 *
 * ===========================================================================================
 * WHY THE PAIRED-TICK IDIOM ("park at the same yaw twice, read after the second")
 * ===========================================================================================
 * Every time this file introduces a NEW scripted yaw, the FIRST tick at that yaw carries a
 * large transient derivative kick (the yaw "teleported" instantaneously in the mock's eyes,
 * since there is no physics/integration — see the `hal_mocks` bridge docblock). The SECOND tick
 * parked at the SAME yaw has `errOld == errNew` (the yaw did not move between the two reads), so
 * the derivative term is EXACTLY 0 regardless of `kd` or the measured `dt` — a clean, P-only
 * reading. This mirrors `test_turn_pd_integration.cpp`'s overshoot case ("two ticks parked at
 * the SAME still-short-of-target yaw"). Every assertion in this file that pins an EXACT dispatch
 * value reads it after the SECOND tick of a pair, never the first.
 *
 * ===========================================================================================
 * THE mmL.mockYawDeg -> APP-FRAME BRIDGE SIGN (kMockYawSign — deliberately NOT Config::IMU_YAW_SIGN)
 * ===========================================================================================
 * `hal_mocks/MatrixMiniR4.h`'s `MatrixColor::getColor` synthesizes the external-gyro board's two
 * raw bytes from `mmL.mockYawDeg` so that the production decode
 * (`GyroDecodePolicy.h::decodeGyroYawDeg`, driven by `Config::GYRO_YAW_SIGN` /
 * `Config::GYRO_SCALE`) round-trips back to exactly `mmL.mockYawDeg * kBridgeImuSign` — see that
 * header's "THE BRIDGE (load-bearing)" note. With `GYRO_SCALE_DEFAULT=1.0f` and
 * `GYRO_YAW_SIGN_DEFAULT=-1.0f` PINNED on this target (this file's CMake target MUST be added to
 * the existing `GYRO_SCALE_DEFAULT`/`GYRO_YAW_SIGN_DEFAULT` `foreach` pinning block in
 * `CMakeLists.txt` that already covers `test_turn_pd_integration` and its siblings — T8), the
 * round trip collapses to `readYawDeg() == mmL.mockYawDeg * kBridgeImuSign`, where
 * `kBridgeImuSign` falls back to `-1.0f`. `kMockYawSign` below hardcodes that SAME `-1.0f`
 * LOCALLY, rather than reading `Config::IMU_YAW_SIGN` — that constant is the retired
 * STM32-write-path constant this whole issue (#169) is deleting (T2b, a LATER wave than this
 * file), and `hal_mocks`' own bridge is separately being decoupled from it (T6). Hardcoding the
 * value here means this file keeps compiling and behaving identically before and after that
 * retirement lands.
 *
 * The upshot: `parkYaw(appYawDeg)` scripts `mmL.mockYawDeg = appYawDeg * kMockYawSign`, so that
 * `YawSource::readYawDeg()` reads back exactly `appYawDeg` (integer degrees round-trip exactly;
 * the bridge quantizes to whole degrees, so every scripted heading in this file is a whole
 * number).
 *
 * ===========================================================================================
 * READING Config::GYRO_HOLD_* SYMBOLICALLY, NOT AS HARDCODED LITERALS
 * ===========================================================================================
 * `GYRO_HOLD_KP`/`KD`/`MAX_CORR_PWM` ship as UNVERIFIED bench-tuning placeholders (ADR-072).
 * Every exact-value assertion in this file computes its EXPECTED PWM from the live
 * `Config::GYRO_HOLD_KP` (via the `roundPwm` helper below) rather than a hardcoded numeric
 * literal, so a later bench retune does not require hand-editing this file's expected values.
 * Where a case's correctness structurally depends on a numeric relationship holding, that
 * relationship is asserted as an explicit PRECONDITION before the real assertion, so a future
 * Config change that breaks the scenario fails loudly with a clear message.
 */

#include "Config.h"
#include "src/drive/Drive.h"
#include "src/nav/YawSource.h"
#include "src/util/WROMath.h"    // cmToEncDeg — profile-distance-to-encoder-degrees math.

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "WROLineArray.h"
#include "Intersection.h"
#include "LineFollower.h"

#include "TestAssert.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

// ===========================================================================================
// Local integration harness — scoped to Drive + YawSource only (no PathTracker/Localization).
// Mirrors `test_turn_pd_integration.cpp`'s harness idioms (reset, per-tick yaw parking, the
// two-tick dispatch gotcha) plus `test_drive_decel_floor.cpp`'s encoder-driven profile stepping
// (`tickProfileToEnc`): every GYRO hold tick needs BOTH a scripted yaw AND a scripted encoder
// advance to drive `_updateProfile`'s distance-based fraction.
// ===========================================================================================
namespace gyro_hold_harness
{
/// @brief millis advanced per tick — matches `turn_harness::kTickMs`.
constexpr uint32_t kTickMs = 5;

/// Local sign constant for the `mmL.mockYawDeg` -> app-frame bridge — see the file docblock's
/// "THE mmL.mockYawDeg -> APP-FRAME BRIDGE SIGN" section. Deliberately NOT `Config::IMU_YAW_SIGN`
/// (a retiring constant this file must survive the removal of).
constexpr float kMockYawSign = -1.0f;

/// @brief Reset every mock surface and the real `Drive` singleton to a clean per-case world.
inline void resetWorld()
{
    Drive.stop(true);

    mockResetMillis();
    mockResetMiniR4();
    mmL.mockReset();

    LineArray.mockReset();
    Intersection.mockReset();
    LineFollower.mockReset();

    Drive.begin();
    Drive.clearFault();
}

/// @brief Script `mmL.mockYawDeg` so `YawSource::readYawDeg()` reads back exactly @p appYawDeg.
inline void parkYaw(float appYawDeg)
{
    mmL.mockYawDeg = (double) (appYawDeg * kMockYawSign);
}

/**
 * @brief Advance the mock encoders by @p encDelta (both M2 and M3, in lockstep), park the yaw at
 *        @p appYawDeg, run ONE `Drive.update()` tick, then advance the mock clock by @p kTickMs.
 *
 * @note Encoder advance happens BEFORE `Drive.update()` (matching
 *       `test_drive_decel_floor.cpp`'s `tickProfileToEnc`, since `_updateProfile` reads
 *       `MiniR4.M2.getDegrees()` live); the millis advance happens AFTER, so THIS tick's
 *       `_emitGyroHold` sees the dt since the PREVIOUS tick.
 */
inline void tick(float appYawDeg, int32_t encDelta)
{
    if (encDelta != 0)
    {
        MiniR4.M2.mockAdvanceDegrees(encDelta);
        MiniR4.M3.mockAdvanceDegrees(encDelta);
    }
    parkYaw(appYawDeg);
    Drive.update();
    mockAdvanceMillis(kTickMs);
}

/**
 * @brief Park the yaw at @p appYawDeg and tick, ONE encoder-degree at a time, until `MiniR4.M2`'s
 *        advance since @p startEnc reaches (or first exceeds) @p targetEncDelta.
 */
inline void tickProfileToEnc(float appYawDeg, int32_t targetEncDelta, int32_t startEnc, int maxTicks)
{
    for (int i = 0; i < maxTicks; ++i)
    {
        const int32_t now = MiniR4.M2.getDegrees() - startEnc;
        if (now >= targetEncDelta)
        {
            break;
        }
        tick(appYawDeg, 1);
    }
}

/// @brief Round a float PWM command the SAME way `Drive::_emitGyroHold` does
///        (`(int16_t) lroundf(corr)`), so expected values are computed identically to production.
inline int16_t roundPwm(float v)
{
    return (int16_t) std::lround((double) v);
}
}    // namespace gyro_hold_harness

// ===========================================================================================
// TC7 — a GYRO profile move (`.tShape()` default, cruise plateau); mocked yaw scripted to drift
// off the target, then back toward it. Asserts the dispatched `Move(L,R)` differential (a)
// points the correct way (right>left for a +CCW-needed error), (b) GROWS as the scripted error
// grows, (c) SHRINKS again as the scripted error shrinks back, and (d) no `MoveGyro*` vendor
// call ever fires.
// ===========================================================================================
static void test_TC7_dispatched_differential_tracks_error_and_never_uses_vendor_gyro_path()
{
    TEST_SCOPE("TC7: GYRO cruise-plateau hold -- dispatched differential tracks +/- error, "
               "no MoveGyro* call");
    gyro_hold_harness::resetWorld();

    Drive.driveDist(40.0f).setMode(DriveMode::GYRO).setHeading((int16_t) 0).async();
    gyro_hold_harness::tick(0.0f, 0);    // dispatch tick -- no real Move() yet (two-tick gotcha).

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    const int32_t encPerCm        = (int32_t) cmToEncDeg(1.0f);
    ASSERT_GT(encPerCm, (int32_t) 0);

    // Fast-forward well past accelCm, landing solidly in the cruise plateau, yaw held ALIGNED to
    // the target the whole way (error stays 0 throughout the ramp-up).
    gyro_hold_harness::tickProfileToEnc(0.0f, 12L * encPerCm, profileStartEnc, 5000);

    // --- Stage A: aligned (error 0) -- baseline, confirms we ARE in cruise (base == SPEED_NORMAL).
    gyro_hold_harness::tick(0.0f, 1);
    gyro_hold_harness::tick(0.0f, 1);
    const int16_t lA = MiniR4.DriveDC.lastMoveL;
    const int16_t rA = MiniR4.DriveDC.lastMoveR;
    ASSERT_EQ(lA, rA);
    ASSERT_EQ((int16_t) ((lA + rA) / 2), (int16_t) Config::SPEED_NORMAL);
    const int16_t corrA = (int16_t) ((rA - lA) / 2);
    ASSERT_EQ(corrA, (int16_t) 0);

    // --- Stage B: yaw drifts to -4 (error=+4, target CCW of yaw -> right>left correction).
    gyro_hold_harness::tick(-4.0f, 1);
    gyro_hold_harness::tick(-4.0f, 1);    // second read: clean, D==0 (yaw unchanged from the first).
    const int16_t lB = MiniR4.DriveDC.lastMoveL;
    const int16_t rB = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(rB > lB);
    const int16_t corrB         = (int16_t) ((rB - lB) / 2);
    const int16_t expectedCorrB = gyro_hold_harness::roundPwm(Config::GYRO_HOLD_KP * 4.0f);
    ASSERT_EQ(corrB, expectedCorrB);

    // --- Stage C: yaw drifts FURTHER to -8 (error=+8, larger) -- the differential must GROW.
    gyro_hold_harness::tick(-8.0f, 1);
    gyro_hold_harness::tick(-8.0f, 1);
    const int16_t lC = MiniR4.DriveDC.lastMoveL;
    const int16_t rC = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(rC > lC);
    const int16_t corrC         = (int16_t) ((rC - lC) / 2);
    const int16_t expectedCorrC = gyro_hold_harness::roundPwm(Config::GYRO_HOLD_KP * 8.0f);
    ASSERT_EQ(corrC, expectedCorrC);
    ASSERT_TRUE(corrC > corrB);    // GROWS toward re-alignment as the error grows.

    // --- Stage D: yaw comes BACK toward the target, to -2 (error=+2) -- the differential SHRINKS.
    gyro_hold_harness::tick(-2.0f, 1);
    gyro_hold_harness::tick(-2.0f, 1);
    const int16_t lD = MiniR4.DriveDC.lastMoveL;
    const int16_t rD = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(rD > lD);
    const int16_t corrD         = (int16_t) ((rD - lD) / 2);
    const int16_t expectedCorrD = gyro_hold_harness::roundPwm(Config::GYRO_HOLD_KP * 2.0f);
    ASSERT_EQ(corrD, expectedCorrD);
    ASSERT_TRUE(corrD < corrC);    // SHRINKS as the scripted error shrinks.
    ASSERT_TRUE(corrD > 0);        // still actively correcting, not yet re-aligned.

    // No vendor gyro-PID write path was ever touched (AC1).
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, (uint32_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.moveGyroDegsCallCount, (uint32_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.moveGyroTimeCallCount, (uint32_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveGyroTarget, (int16_t) 0);
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC7b — drive the SAME 40cm GYRO profile deep into its decel tail (base shrunk to the kinetic
// floor, `Config::PWM_MOVING_MIN`), then introduce a LARGE live heading error. Asserts the
// dynamic `corrLimitPwm = min(GYRO_HOLD_MAX_CORR_PWM, |base|)` clamp binds at |base| (not the
// static ceiling), landing the inner wheel at EXACTLY 0 -- never negative (never reverses).
// ===========================================================================================
static void test_TC7b_decel_tail_shrunk_base_never_reverses_a_wheel()
{
    TEST_SCOPE("TC7b: decel-tail shrunk base -- dynamic |base| clamp binds, inner wheel == 0, "
               "never reverses");
    gyro_hold_harness::resetWorld();

    Drive.driveDist(40.0f).setMode(DriveMode::GYRO).setHeading((int16_t) 0).async();
    gyro_hold_harness::tick(0.0f, 0);    // dispatch.

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    const int32_t encPerCm        = (int32_t) cmToEncDeg(1.0f);

    // Walk deep into the decel tail (38cm traveled), yaw held ALIGNED so the base genuinely
    // shrinks to the kinetic floor BEFORE any error is introduced.
    gyro_hold_harness::tickProfileToEnc(0.0f, 38L * encPerCm, profileStartEnc, 20000);
    gyro_hold_harness::tick(0.0f, 1);
    gyro_hold_harness::tick(0.0f, 1);    // settle any residual transient D from the fast-forward.

    const int16_t lBase = MiniR4.DriveDC.lastMoveL;
    const int16_t rBase = MiniR4.DriveDC.lastMoveR;
    ASSERT_EQ(lBase, rBase);    // aligned -> zero correction -> both wheels == base.
    ASSERT_EQ(lBase, (int16_t) Config::PWM_MOVING_MIN);    // confirms the shrunk kinetic-floor regime.
    ASSERT_FALSE(Drive.hasFault());

    // Precondition: the introduced error must overwhelm the shrunk base by a comfortable margin
    // so the clamp visibly BINDS rather than coinciding with an unclamped result.
    ASSERT_TRUE(Config::GYRO_HOLD_KP * 100.0f > (float) Config::PWM_MOVING_MIN);

    // Introduce a large live heading error while the base is at its most shrunk.
    gyro_hold_harness::tick(-100.0f, 1);
    gyro_hold_harness::tick(-100.0f, 1);    // second read: clean, D==0.

    const int16_t l = MiniR4.DriveDC.lastMoveL;
    const int16_t r = MiniR4.DriveDC.lastMoveR;

    // corrLimit == min(GYRO_HOLD_MAX_CORR_PWM, |base|) == |base| here (kinetic floor is far below
    // the static ceiling), so corr saturates at EXACTLY |base|: left == base - corrLimit == 0
    // (never negative), right == base + corrLimit == 2*base.
    ASSERT_EQ(l, (int16_t) 0);
    ASSERT_EQ(r, (int16_t) (2 * lBase));
    ASSERT_TRUE(r > l);    // still correcting the right way, even fully clamped.
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC8 — an IMMEDIATE GYRO->GYRO chain (segment 1 no-brake -> seamless cruise handoff -> segment
// 2 dispatches the SAME tick segment 1 completes) with a CHANGED heading target. Because the
// derivative is computed on the MEASUREMENT (yaw), not the target, and the yaw genuinely never
// moves across this boundary, segment 2's FIRST real PD tick must be PURE-P -- no derivative
// spike from the target's discontinuity (0 -> 20).
// ===========================================================================================
static void test_TC8_chained_gyro_segments_no_derivative_spike_on_heading_change()
{
    TEST_SCOPE("TC8: immediate GYRO->GYRO chain, changed heading -- no derivative spike on "
               "segment 2's first tick");
    gyro_hold_harness::resetWorld();

    // Segment 1: 10cm, heading 0, NO BRAKE -> endSpeed resolves to cruise (seamless handoff).
    // Segment 2: 10cm, heading 20, default brake.
    Drive.driveDist(10.0f)
        .setMode(DriveMode::GYRO)
        .setHeading((int16_t) 0)
        .setBrake(false)
        .async();
    Drive.driveDist(10.0f).setMode(DriveMode::GYRO).setHeading((int16_t) 20).async();

    gyro_hold_harness::tick(0.0f, 0);    // dispatch segment 1.
    const int32_t totalEnc1 = (int32_t) cmToEncDeg(10.0f);

    // Drive segment 1 to completion ONE encoder-degree at a time, yaw held ALIGNED to heading 1
    // (0) throughout. The LAST iteration is BOTH segment 1's completion AND segment 2's same-tick
    // dispatch (per the two-tick gotcha, that tick issues no new Move() call from EITHER side).
    for (int32_t i = 0; i < totalEnc1; ++i)
    {
        gyro_hold_harness::tick(0.0f, 1);
    }

    const uint32_t movesBeforeSeg2 = MiniR4.DriveDC.moveCallCount;

    // Segment 2's FIRST real PD tick. The yaw is STILL 0 -- UNCHANGED across the boundary --
    // only the TARGET jumped (0 -> 20). This is the exact scenario the fold-safe
    // derivative-on-measurement design exists for.
    gyro_hold_harness::tick(0.0f, 1);
    ASSERT_TRUE(MiniR4.DriveDC.moveCallCount > movesBeforeSeg2);

    const int16_t l2    = MiniR4.DriveDC.lastMoveL;
    const int16_t r2    = MiniR4.DriveDC.lastMoveR;
    const int16_t base2 = (int16_t) ((l2 + r2) / 2);
    const int16_t corr2 = (int16_t) ((r2 - l2) / 2);

    // Precondition: the pure-P expectation must stay under BOTH the static clamp and segment 2's
    // freshly-ramping base, so a spurious derivative spike would visibly change the result.
    ASSERT_TRUE(Config::GYRO_HOLD_KP * 20.0f < (float) Config::PWM_MIN_MAGNITUDE);

    const int16_t expectedCorr2 = gyro_hold_harness::roundPwm(Config::GYRO_HOLD_KP * 20.0f);
    ASSERT_EQ(corr2, expectedCorr2);
    ASSERT_TRUE(base2 > 0);
    ASSERT_TRUE(r2 > l2);
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, (uint32_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.moveGyroDegsCallCount, (uint32_t) 0);
}

// ===========================================================================================
// TC9 — a TRANSIENT (single-tick) `readYawDeg` failure (scripted via `mmL.mockResult = ERROR`,
// which the `MatrixColor` mock bridge honors by returning dead bytes). Asserts the skip-and-hold
// contract: `Move(base, base)` emitted (symmetric, zero correction) on exactly that one tick, no
// fault, and the move still completes normally once the read recovers.
// ===========================================================================================
static void test_TC9_transient_yaw_read_failure_emits_symmetric_move_and_completes()
{
    TEST_SCOPE("TC9: one-tick yaw read failure -- Move(base,base) emitted, no fault, move completes");
    gyro_hold_harness::resetWorld();

    Drive.driveDist(20.0f).setMode(DriveMode::GYRO).setHeading((int16_t) 0).async();
    gyro_hold_harness::tick(0.0f, 0);    // dispatch.

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    const int32_t encPerCm        = (int32_t) cmToEncDeg(1.0f);

    // Walk partway in with GOOD reads, yaw aligned.
    gyro_hold_harness::tickProfileToEnc(0.0f, 5L * encPerCm, profileStartEnc, 5000);

    const uint32_t movesBefore = MiniR4.DriveDC.moveCallCount;

    // Force exactly ONE bad yaw read.
    mmL.mockResult = MMLower::RESULT::ERROR;
    gyro_hold_harness::tick(0.0f, 1);
    mmL.mockResult = MMLower::RESULT::OK;    // restore immediately -- exactly one tick failed.

    // Exactly one NEW Move() call this tick (the skip-and-hold), symmetric (zero correction),
    // and no new fault (every GYRO move is bounded by a non-IMU terminator).
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, movesBefore + 1);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, MiniR4.DriveDC.lastMoveR);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveL > 0);    // still driving forward, not zeroed out.
    ASSERT_FALSE(Drive.hasFault());

    // Drive the rest of the way to completion with good reads.
    gyro_hold_harness::tickProfileToEnc(
        0.0f, (int32_t) cmToEncDeg(20.0f), profileStartEnc, 20000);

    ASSERT_TRUE(Drive.isIdle());
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, (int16_t) 0);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// Mid-segment tick hitch (large dt). Simulates a delayed cooperative tick (a real elapsed-time
// gap with NO `Drive.update()` call in between) that exceeds `Config::GYRO_HOLD_STALE_MS`,
// followed by a NEW yaw arriving on the next real tick. The staleness guard must degrade the
// hold to PURE-P (`errOld = errNew`) rather than compute a derivative over the wide gap.
// ===========================================================================================
static void test_mid_segment_tick_hitch_stays_bounded_via_staleness_guard()
{
    TEST_SCOPE("mid-segment tick hitch: a stale gap degrades the hold to pure-P, no jerk");
    gyro_hold_harness::resetWorld();

    Drive.driveDist(40.0f).setMode(DriveMode::GYRO).setHeading((int16_t) 0).async();
    gyro_hold_harness::tick(0.0f, 0);    // dispatch.

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    const int32_t encPerCm        = (int32_t) cmToEncDeg(1.0f);
    gyro_hold_harness::tickProfileToEnc(0.0f, 12L * encPerCm, profileStartEnc, 5000);    // cruise.

    // Establish a clean P-only baseline (paired-tick idiom) at a small live error.
    gyro_hold_harness::tick(-4.0f, 1);
    gyro_hold_harness::tick(-4.0f, 1);
    const int16_t lBaseline     = MiniR4.DriveDC.lastMoveL;
    const int16_t rBaseline     = MiniR4.DriveDC.lastMoveR;
    const int16_t corrBaseline  = (int16_t) ((rBaseline - lBaseline) / 2);
    ASSERT_EQ(corrBaseline, gyro_hold_harness::roundPwm(Config::GYRO_HOLD_KP * 4.0f));

    // Preconditions: the post-hitch pure-P expectation must stay under the static clamp (so a
    // staleness-guard regression shows as a WRONG NUMBER) AND leave the OUTER wheel below PWM_MAX
    // at the cruise base (so the exact differential is not silently truncated by the legal-PWM cap
    // in gyroHoldWheels — the never-reverse clamp only protects the INNER wheel).
    ASSERT_TRUE(Config::GYRO_HOLD_KP * 12.0f < (float) Config::GYRO_HOLD_MAX_CORR_PWM);
    ASSERT_TRUE((float) Config::SPEED_NORMAL + Config::GYRO_HOLD_KP * 12.0f <= (float) Config::PWM_MAX);

    // Simulate the hitch: a large elapsed-time gap with NO Drive.update() call in between,
    // exceeding GYRO_HOLD_STALE_MS, then a NEW yaw arrives on the very next real tick.
    mockAdvanceMillis((uint32_t) Config::GYRO_HOLD_STALE_MS + 50);
    gyro_hold_harness::tick(-12.0f, 1);

    const int16_t l    = MiniR4.DriveDC.lastMoveL;
    const int16_t r    = MiniR4.DriveDC.lastMoveR;
    const int16_t corr = (int16_t) ((r - l) / 2);
    const int16_t expectedCorr = gyro_hold_harness::roundPwm(Config::GYRO_HOLD_KP * 12.0f);

    ASSERT_EQ(corr, expectedCorr);    // pure-P -- the staleness guard zeroed the derivative term.
    ASSERT_TRUE(std::abs((int) corr) <= (int) Config::GYRO_HOLD_MAX_CORR_PWM);    // bounded, no jerk.
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC11 — a short (< Config::TSHAPE_MIN_MOVE_CM) GYRO move. The default-tShape() builder falls
// back to `Kind::DRIVE_DIST` below the threshold, which `_dispatchNext`'s dispatcher-level GYRO
// lowering runs as a cooperative FLAT profile (startSpeed==cruise, no ramp-up;
// `accel=Config::TSHAPE_ACCEL_PWM_PER_CM` explicit). Asserts it RUNS the cooperative profile
// (nonzero dispatched PWM) and BRAKES at the commanded distance -- never `MoveGyroDegs`.
// ===========================================================================================
static void test_TC11_short_flat_gyro_move_runs_cooperative_profile_and_brakes_at_distance()
{
    TEST_SCOPE("TC11: short (<TSHAPE_MIN_MOVE_CM) GYRO move -- cooperative profile, brakes at "
               "distance, no MoveGyroDegs");
    gyro_hold_harness::resetWorld();

    ASSERT_TRUE(3.0f < Config::TSHAPE_MIN_MOVE_CM);    // precondition: genuinely below threshold.

    Drive.driveDist(3.0f)
        .setMode(DriveMode::GYRO)
        .setHeading((int16_t) 0)
        .setPower((int16_t) 60)
        .async();

    gyro_hold_harness::tick(0.0f, 0);    // dispatch tick.
    const int32_t profileStartEnc = MiniR4.M2.getDegrees();

    gyro_hold_harness::tick(0.0f, 1);    // first REAL cooperative tick.
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveL != 0 || MiniR4.DriveDC.lastMoveR != 0);

    const int32_t totalEnc = (int32_t) cmToEncDeg(3.0f);
    gyro_hold_harness::tickProfileToEnc(0.0f, totalEnc, profileStartEnc, 5000);

    ASSERT_TRUE(Drive.isIdle());
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, (int16_t) 0);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_EQ(MiniR4.DriveDC.moveGyroDegsCallCount, (uint32_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, (uint32_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.moveGyroTimeCallCount, (uint32_t) 0);
}

// ===========================================================================================
// Entry.
// ===========================================================================================
int main()
{
    std::printf("INFO: GYRO heading-hold PD -- real Drive.cpp FSM integration (ADR-072, T7, "
                "TC7-TC11 + mid-segment-hitch)\n");

    test_TC7_dispatched_differential_tracks_error_and_never_uses_vendor_gyro_path();
    test_TC7b_decel_tail_shrunk_base_never_reverses_a_wheel();
    test_TC8_chained_gyro_segments_no_derivative_spike_on_heading_change();
    test_TC9_transient_yaw_read_failure_emits_symmetric_move_and_completes();
    test_mid_segment_tick_hitch_stays_bounded_via_staleness_guard();
    test_TC11_short_flat_gyro_move_runs_cooperative_profile_and_brakes_at_distance();

    TEST_REPORT();
}
