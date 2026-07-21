/**
 * @file test_drive_decel_floor.cpp
 * @brief Host-link tests for the ADR (kinetic decel-tail floor) `PWM_MOVING_MIN` feature —
 *        driving the REAL `Drive.cpp` `.tShape()` trapezoidal profile through the shared
 *        cooperative-tick idiom (2026-07-12 kinetic-decel-tail-floor plan, T6; TC1-TC6;
 *        AC2, AC3, AC5, AC6).
 *
 * @details
 * `PWM_MOVING_MIN` (Config::PWM_MOVING_MIN, seeded 30) is a lower "kinetic" deadband floor
 * that applies ONLY on a `.tShape()` profile's decel-to-brake tail when the move's resolved
 * endSpeed is below the static breakaway floor (`Config::PWM_MIN_MAGNITUDE`, 35). Every other
 * phase of a profiled move — ramp-up/accel, cruise plateau — keeps the static floor. This
 * target proves the DISPATCHED setpoint (the value the mock DriveDC records, i.e. what
 * actually reaches the motors), not the planned/resolved value, on both sides of that
 * phase-gate:
 *
 *   TC1/TC2 (unit, `deadbandSnap`)     — pure `WROMath.h` free-function floor/cap/zero/sign
 *                                        math, both the defaulted 1-arg call and the explicit
 *                                        `floorMag` override (AC2).
 *   TC3 (integration, real Drive.cpp) — a `.tShape()` stop (endSpeed 0, cruise above the
 *                                        floor) ticked into its DECEL region dispatches
 *                                        `+-PWM_MOVING_MIN`; the CRUISE plateau dispatches
 *                                        `+-cruise` (AC3).
 *   TC4 (identity, AC5)               — the SAME scenario, but this translation unit is
 *                                        compiled into a SEPARATE cmake target with
 *                                        `-DPWM_MOVING_MIN_DEFAULT=PWM_MIN_MAGNITUDE` — the
 *                                        off-switch — so the decel tail dispatches `+-35`
 *                                        (byte-identical to the pre-feature build). See
 *                                        `test_drive_decel_floor_identity.cpp`.
 *   TC5 (phase-gating NEGATIVE, AC6)  — the correctness crux guard: a
 *                                        `.tShape().setStartSpeed(0)` rest-START's early
 *                                        sub-35 ACCEL setpoints dispatch `+-PWM_MIN_MAGNITUDE`
 *                                        (35), NEVER `+-PWM_MOVING_MIN` (30). This is the only
 *                                        test that can catch the kinetic floor leaking onto
 *                                        the accel/breakaway path — `TSHAPE_START_SPEED ==
 *                                        PWM_MIN_MAGNITUDE` by default, so a default-start
 *                                        profile can never exercise this branch.
 *   TC6 (triangle short hop)          — a short hop whose two full-peak ramps don't fit
 *                                        (start 35 / end 0 / cruise 60 / dist 20cm), so the
 *                                        peak collapses below the static floor (~31). Documents
 *                                        + asserts the intended plateau(35)->decel(31->30 crawl)
 *                                        behaviour: the reduced-peak plateau still dispatches at
 *                                        the STATIC floor (peak 31 snaps up to 35, since the
 *                                        plateau is reached via the ACCEL ramp, never flagged
 *                                        `inDecel`), and only the tail end of the decel ramp
 *                                        (once `traveled >= totalEnc - decelEnc`) crawls down to
 *                                        the KINETIC floor.
 *
 * HOW THE SUT IS LINKED (mirrors the issue-#106 PathTracker+Drive real-link recipe — see the
 * block comment above `test_path_tracker_passthrough_on` in CMakeLists.txt — MINUS
 * PathTracker/Localization/YawSource-as-a-consumer/routes, since this target drives `Drive`
 * directly and never touches PathTracker):
 *   - Links the REAL `Drive.cpp` + `YawSource.cpp` (Drive.cpp's GYRO heading-hold reads
 *     `YawSource::readYawDeg` — the symbol must resolve even though this test's SYNC-mode moves
 *     never execute that branch at runtime) + `path_mocks.cpp`
 *     (the facade-globals TU: `MiniR4` / `Serial` / `Serial1` / `EventTrace` / clock globals —
 *     deliberately NOT `Drive`, so it never collides with the real `Drive.cpp` that is the
 *     sole definer of the `Drive` global).
 *   - `${SKETCH_DIR}` leads the include path (the inversion) so the real `Drive.h` /
 *     `YawSource.h` win; `${HAL_MOCKS_DIR}` supplies the angle-bracket `<MatrixMiniR4.h>` /
 *     `<Arduino.h>` (the `MiniR4.M2`/`M3` mock encoders + `MiniR4.DriveDC` mock recorder this
 *     test reads); `${PATH_MOCKS_DIR}` supplies the force-included `WROLineArray.h` /
 *     `Intersection.h` / `LineFollower.h` peripherals `Drive.cpp`'s seek/dispatch machinery
 *     touches (this test never dispatches a `stopAtLine`/`wallStop`, but the real `Drive.cpp`
 *     translation unit still needs those symbols to link); `${HAL_MOCKS_DIR}/EventTrace.h` is
 *     force-included too, mirroring `add_path_force_includes` exactly (unused by `Drive.cpp`
 *     itself, but part of the standard bundle).
 *   - `${MOCKS_DIR}` (the OLDER `mocks/Drive.h` mock-DriveClass world) is OMITTED entirely —
 *     using it here would be trap #1 from the task brief: `mocks/Drive.h`'s `setSpeed` records
 *     the PRE-snap value, which would false-green every assertion in this file. This target
 *     links the REAL `Drive.cpp`; every setpoint we assert on is read from
 *     `MiniR4.DriveDC.lastMoveSyncL/R` — the mock DriveDC's recording of what `Drive.cpp`
 *     actually dispatched to the lower MCU, i.e. POST-snap.
 *
 * TICK IDIOM (mirrors `test_path_tracker_settle.cpp`'s `MiniR4.M2/M3.mockAdvanceDegrees` +
 * `Drive.update()` drivers): `_updateProfile()` reads `MiniR4.M2.getDegrees()` as the sole
 * canonical encoder (matching `getEncL()`) to compute `frac`, so a test walks the profile by
 * calling `MiniR4.M2.mockAdvanceDegrees(delta)` (M3 is not read by the profile fraction, but
 * advanced in lockstep for realism) then `Drive.update()`, and inspects
 * `MiniR4.DriveDC.lastMoveSyncL` / `lastMoveSyncR` (the default `.tShape()` mode is
 * `DriveMode::SYNC`, which dispatches via `MoveSync` — see `_emitContinuous`'s `SYNC` case).
 * No GYRO heading is ever parked because none of these scenarios use `DriveMode::GYRO`.
 */

#include "TestAssert.h"

#include <cstdio>
#include <cstdlib>    // std::abs

#include "src/drive/Drive.h"    // real — DriveClass + the global `Drive` (defined in real Drive.cpp)
#include "MatrixMiniR4.h"    // mock MiniR4.M2/M3/DriveDC — hal_mocks/, resolved via the -I search order.
#include "src/util/WROMath.h"    // deadbandSnap (TC1/TC2) — the shared seam under test.

namespace
{
/// @brief Reset every mock actuator + Drive's own instance state to a clean slate, mirroring
///        `test_path_tracker_settle.cpp`'s `resetWorld()` (minus PathTracker/Localization,
///        which this target never links).
///
/// @note `Drive.begin()` alone does NOT reset in-flight profile/queue state — it only
///       reconfigures the lower-MCU PIDs and bias. A prior test that ticked a `.tShape()`
///       move partway through (e.g. TC5's rest-start, deliberately left mid-ramp) would
///       otherwise leak its `_profileState` into the next test: `Drive.update()` only pops
///       the NEXT queued command once `_isIdle()` is true, so a still-in-flight profile
///       silently swallows a fresh `.async()` dispatch instead of starting it. `Drive.stop()`
///       force-idles the profile/turn/seek/push state machines AND drops the queue
///       (`_clearQueue()`), giving every test a genuinely blank slate.
void resetWorld()
{
    mockResetMiniR4();
    Drive.begin();
    Drive.stop();
}

/// @brief Tick `Drive.update()` one encoder-degree at a time until `MiniR4.M2`'s advance since
///        dispatch reaches (or first exceeds) @p targetEncDelta, or @p maxTicks is exhausted.
///        Fine-grained single-degree stepping lets a test land ON a specific phase boundary
///        (e.g. exactly at the decel threshold) without overshooting it in one large jump.
void tickProfileToEnc(int32_t targetEncDelta, int32_t startEnc, int maxTicks)
{
    for (int i = 0; i < maxTicks; ++i)
    {
        const int32_t now = MiniR4.M2.getDegrees() - startEnc;
        if (now >= targetEncDelta)
        {
            break;
        }
        MiniR4.M2.mockAdvanceDegrees(1);
        MiniR4.M3.mockAdvanceDegrees(1);
        Drive.update();
    }
}
}    // namespace

// ===========================================================================================
// TC1/TC2 — deadbandSnap unit coverage (AC2). Pure WROMath.h free function; no Drive/mock
// surface touched at all.
// ===========================================================================================
void test_TC1_deadbandSnap_default_preserves_callers()
{
    TEST_SCOPE("TC1: deadbandSnap(pwm) default floor == PWM_MIN_MAGNITUDE (35), unchanged callers");

    // The bare 1-arg call must still resolve to the STATIC floor (Config::PWM_MIN_MAGNITUDE),
    // exactly as every pre-feature caller relied on — this is the "byte-identical" half of AC2.
    ASSERT_EQ(deadbandSnap((int16_t) 20), (int16_t) Config::PWM_MIN_MAGNITUDE);
    ASSERT_EQ(deadbandSnap((int16_t) 0), (int16_t) 0);
    ASSERT_EQ(deadbandSnap((int16_t) 90), (int16_t) 90);
    ASSERT_EQ(deadbandSnap((int16_t) -20), (int16_t) -Config::PWM_MIN_MAGNITUDE);
}

void test_TC2_deadbandSnap_explicit_floor_arg()
{
    TEST_SCOPE("TC2: deadbandSnap(pwm, 30) — floor/cap/zero/sign with an explicit kinetic floor");

    // {28(->as 20 per plan)->30, 30->30, 90->90, 0->0, -20->-30} — the plan's TC2 table, floor=30.
    ASSERT_EQ(deadbandSnap((int16_t) 20, (int16_t) 30), (int16_t) 30);      // below floor -> snaps up
    ASSERT_EQ(deadbandSnap((int16_t) 30, (int16_t) 30), (int16_t) 30);      // exactly at floor -> unchanged
    ASSERT_EQ(deadbandSnap((int16_t) 90, (int16_t) 30), (int16_t) 90);      // above floor, below cap -> passthrough
    ASSERT_EQ(deadbandSnap((int16_t) 0, (int16_t) 30), (int16_t) 0);        // zero -> zero (intentional stop)
    ASSERT_EQ(deadbandSnap((int16_t) -20, (int16_t) 30), (int16_t) -30);    // sign-mirrored floor snap

    // Cap stays PWM_MAX regardless of floorMag (the floor only ever raises the FLOOR, never the cap).
    ASSERT_EQ(deadbandSnap((int16_t) (Config::PWM_MAX + 10), (int16_t) 30), (int16_t) Config::PWM_MAX);
}

// ===========================================================================================
// TC3 — real Drive.cpp `.tShape()` stop: cruise plateau dispatches +-cruise, sub-static-floor
// decel tail dispatches +-PWM_MOVING_MIN (AC3).
//
// Scenario: cruise=SPEED_NORMAL(70), startSpeed=default(=PWM_MIN_MAGNITUDE=35), endSpeed=0,
// distance=40cm. With Config::TSHAPE_ACCEL_PWM_PER_CM=4.0:
//   accelCm = |70-35|/4 = 8.75cm (in [TSHAPE_RAMP_MIN_CM=5, TSHAPE_RAMP_MAX_CM=20])
//   decelCm = |70-0|/4  = 17.5cm (in [5,20])
//   accelCm+decelCm = 26.25 < totalCm(40) -> NO triangle collapse; peakPwm == cruise == 70.
// Decel begins once traveled >= totalCm - decelCm = 40 - 17.5 = 22.5cm.
// ===========================================================================================
void test_TC3_braked_stop_cruise_and_decel_tail_floors()
{
    TEST_SCOPE("TC3: tShape stop (cruise 70, end 0) — cruise dispatches +-70, decel tail +-30");

    resetWorld();

    ASSERT_TRUE(Config::PWM_MOVING_MIN < Config::PWM_MIN_MAGNITUDE);    // precondition: feature active

    const int32_t startEnc = MiniR4.M2.getDegrees();
    ASSERT_EQ(startEnc, (int32_t) 0);    // resetWorld -> begin() does not touch encoders; mockReset zeroed them.

    // Dispatch: sync-blocking .tShape() would spin forever with no tick callback registered (no
    // isPrevTaskDone modeling on this mock path), so use .async() — matches every async-builder
    // idiom in the existing real-Drive host tests (e.g. `test_path_tracker_*`, which drive
    // PathTracker's own async dispatch). The FIRST `Drive.update()` after `.async()` pops the
    // queued command and seeds `_profileState = RAMP_UP` / `_profileStartEnc`.
    Drive.driveDist(40.0f).tShape().async();
    Drive.update();    // dispatch tick — profile now RAMP_UP, _profileStartEnc captured here.

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    ASSERT_EQ(profileStartEnc, startEnc);    // no encoder motion yet — dispatch is same-tick as start.

    // --- Cruise plateau: advance well past accelCm (8.75cm) but short of the decel threshold
    //     (22.5cm). 15cm of travel lands cleanly inside the plateau. cmToEncDeg(15) ~= 272.9 deg.
    const int32_t encPerCm = (int32_t) cmToEncDeg(1.0f);
    ASSERT_GT(encPerCm, (int32_t) 0);
    tickProfileToEnc(15L * encPerCm, profileStartEnc, 5000);

    const int16_t cruiseL = MiniR4.DriveDC.lastMoveSyncL;
    const int16_t cruiseR = MiniR4.DriveDC.lastMoveSyncR;
    ASSERT_EQ(cruiseL, (int16_t) Config::SPEED_NORMAL);
    ASSERT_EQ(cruiseR, (int16_t) Config::SPEED_NORMAL);

    // --- Decel tail: advance well past the decel threshold (22.5cm) toward the move end (40cm),
    //     e.g. 38cm traveled -- 2cm from the target, deep in the sub-static-floor decel ramp
    //     (endSpeed 0 < PWM_MIN_MAGNITUDE), so the dispatched setpoint must be pinned at the
    //     KINETIC floor, never re-snapped to the static one.
    tickProfileToEnc(38L * encPerCm, profileStartEnc, 5000);

    const int16_t tailL = MiniR4.DriveDC.lastMoveSyncL;
    const int16_t tailR = MiniR4.DriveDC.lastMoveSyncR;
    ASSERT_EQ(tailL, (int16_t) Config::PWM_MOVING_MIN);
    ASSERT_EQ(tailR, (int16_t) Config::PWM_MOVING_MIN);
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC5 — phase-gating NEGATIVE (AC6, the correctness crux guard). A `.setStartSpeed(0)`
// rest-START's early sub-35 ACCEL setpoints must dispatch +-PWM_MIN_MAGNITUDE (35), NEVER
// +-PWM_MOVING_MIN (30). `_decelTailFloorMag` only returns the kinetic floor while `inDecel`;
// the ramp-UP region must NEVER read as `inDecel` regardless of how low the interpolated
// speed dips near a rest start.
//
// Scenario: cruise=70, startSpeed=0 (forced rest-start via .setStartSpeed(0)), endSpeed=0
// (default), distance=40cm (same shape as TC3, only startSpeed changes).
//   accelCm = |70-0|/4 = 17.5cm (clamped into [5,20] -> 17.5, no clamp needed)
// The very first ramp-up ticks (traveled << accelCm) interpolate a setpoint near 0 -- deep
// below EITHER floor -- so if the kinetic floor ever leaked onto the accel path this would
// silently dispatch +-30 instead of the intended +-35 breakaway kick.
// ===========================================================================================
void test_TC5_rest_start_accel_floor_stays_static()
{
    TEST_SCOPE("TC5: tShape .setStartSpeed(0) rest-start — early accel setpoints dispatch +-35, not +-30");

    resetWorld();

    const int32_t startEnc = MiniR4.M2.getDegrees();

    Drive.driveDist(40.0f).tShape().setStartSpeed(0).async();
    Drive.update();    // dispatch tick.

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    ASSERT_EQ(profileStartEnc, startEnc);

    // One small encoder tick — deep inside the 17.5cm accel ramp, where the raw interpolated
    // speed is far below PWM_MIN_MAGNITUDE (e.g. traveled=1cm -> raw ~= 0 + (70-0)*(1/17.5) ~= 4).
    const int32_t encPerCm = (int32_t) cmToEncDeg(1.0f);
    tickProfileToEnc(1L * encPerCm, profileStartEnc, 5000);

    const int16_t earlyL = MiniR4.DriveDC.lastMoveSyncL;
    const int16_t earlyR = MiniR4.DriveDC.lastMoveSyncR;

    // The dispatched setpoint must be pinned at the STATIC floor, not the kinetic one, and must
    // be strictly positive (the deadband snap never dispatches a raw near-zero accel setpoint).
    ASSERT_EQ(earlyL, (int16_t) Config::PWM_MIN_MAGNITUDE);
    ASSERT_EQ(earlyR, (int16_t) Config::PWM_MIN_MAGNITUDE);
    ASSERT_TRUE(earlyL != (int16_t) Config::PWM_MOVING_MIN);    // the crux negative: NOT the kinetic floor.

    // Walk forward through several more early-ramp ticks — the guard must hold at every one of
    // them, not just the very first tick.
    for (int i = 0; i < 20; ++i)
    {
        MiniR4.M2.mockAdvanceDegrees(1);
        MiniR4.M3.mockAdvanceDegrees(1);
        Drive.update();
        const int32_t traveled = MiniR4.M2.getDegrees() - profileStartEnc;
        if (traveled >= 5L * encPerCm)    // stop well before the 17.5cm accel ramp ends
        {
            break;
        }
        const int16_t l = MiniR4.DriveDC.lastMoveSyncL;
        if (l != 0)    // 0 would mean "not yet dispatched this tick's setpoint" — skip a same-tick no-op
        {
            ASSERT_TRUE(std::abs((int) l) >= (int) Config::PWM_MIN_MAGNITUDE);
        }
    }
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC6 — short hop. start=45 (default TSHAPE_START_SPEED == PWM_MIN_MAGNITUDE), end=0, cruise=60,
//   distance=20cm. The T-shape decel sheds cruise->PWM_MIN floor (not cruise->0); the brake
//   finishes floor->rest.
//   accelCm = |60-45|/4 = 3.75cm   (start -> cruise)
//   decelCm = |60-45|/4 = 3.75cm   (cruise -> PWM_MIN floor)
//   accelCm+decelCm = 7.5cm < totalCm(20cm) -> NO triangle collapse: the profile reaches cruise
//   and holds a plateau from ~3.75cm to ~16.25cm.
// NOTE (retune 55648f2): TSHAPE_START_SPEED was bumped to 45 == PWM_MIN_MAGNITUDE, so the accel
// ramp now starts AT the static floor and never dips below it — the old "collapsed peak below the
// static floor snaps UP to it" plateau condition is no longer reachable (nothing runs below the
// floor). The plateau here is therefore the commanded cruise (60). Only once travel enters the
// decel region (last ~3.75cm) does the setpoint crawl toward the KINETIC floor (35 == PWM_MOVING_MIN)
// as endSpeed(0) < floor pins it there — the kinetic decel-tail floor this file guards.
// ===========================================================================================
void test_TC6_triangle_short_hop_plateau_then_decel_crawl()
{
    TEST_SCOPE("TC6: short hop (45/0/60, 20cm) — cruise plateau(60), decel crawl to kinetic floor(35)");

    resetWorld();

    const int32_t startEnc = MiniR4.M2.getDegrees();

    Drive.driveDist(20.0f).tShape().setEndSpeed(0).setPower((int16_t) 60).async();
    Drive.update();    // dispatch tick.

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    ASSERT_EQ(profileStartEnc, startEnc);

    const int32_t encPerCm = (int32_t) cmToEncDeg(1.0f);

    // --- Plateau: sample mid-cruise (past the accel end ~3.75cm, before the decel region at
    //     ~16.25cm) -- e.g. 8cm traveled. With start(45) == floor the accel ramp never dips below
    //     the floor, so there is no below-floor plateau to snap up: the dispatched setpoint is
    //     simply the commanded cruise (60). (Pre-retune this plateau snapped UP to the static floor
    //     from a collapsed sub-floor peak; that condition is unreachable now — see the header note.)
    tickProfileToEnc(8L * encPerCm, profileStartEnc, 5000);

    const int16_t plateauL = MiniR4.DriveDC.lastMoveSyncL;
    const int16_t plateauR = MiniR4.DriveDC.lastMoveSyncR;
    ASSERT_EQ(plateauL, (int16_t) 60);    // commanded cruise (setPower(60)); no floor-snap on the plateau
    ASSERT_EQ(plateauR, (int16_t) 60);

    // --- Decel crawl: push deep into the decel region (traveled >= ~16.25cm), close to the 20cm
    //     finish -- e.g. 19cm traveled, 1cm from completion. endSpeed(0) < PWM_MIN_MAGNITUDE, so
    //     once `inDecel` this must be pinned at the KINETIC floor.
    tickProfileToEnc(19L * encPerCm, profileStartEnc, 5000);

    const int16_t tailL = MiniR4.DriveDC.lastMoveSyncL;
    const int16_t tailR = MiniR4.DriveDC.lastMoveSyncR;
    ASSERT_EQ(tailL, (int16_t) Config::PWM_MOVING_MIN);
    ASSERT_EQ(tailR, (int16_t) Config::PWM_MOVING_MIN);
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// Entry.
// ===========================================================================================
int main()
{
    std::printf("INFO: Drive.tShape() kinetic decel-tail floor (T6, TC1-TC6) -- real Drive.cpp, "
                "PWM_MIN_MAGNITUDE=%d PWM_MOVING_MIN=%d\n",
                (int) Config::PWM_MIN_MAGNITUDE,
                (int) Config::PWM_MOVING_MIN);

    test_TC1_deadbandSnap_default_preserves_callers();
    test_TC2_deadbandSnap_explicit_floor_arg();
    test_TC3_braked_stop_cruise_and_decel_tail_floors();
    test_TC5_rest_start_accel_floor_stays_static();
    test_TC6_triangle_short_hop_plateau_then_decel_crawl();

    TEST_REPORT();
}
