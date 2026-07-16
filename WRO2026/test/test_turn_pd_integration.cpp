/**
 * @file test_turn_pd_integration.cpp
 * @brief Stateful host-link integration test for the per-tick turn PD executor
 *        (ADR-068, T6, TC9-TC17), plus the two-level breakaway/kinetic creep floor that went
 *        beyond the original plan.
 *
 * @details
 * `test_turn_pd_policy.cpp` (T5) proves the PURE decision math in `TurnPDPolicy.h` in
 * isolation. This file is the FIRST STATEFUL test of the turn FSM itself
 * (`DriveClass::_updateTurnProfile`, Drive.cpp) — today's coverage before this file was pure
 * predicates plus end-to-end yaw-parking (the PathTracker host-link targets), with nothing in
 * between. It links the REAL `Drive.cpp` + `YawSource.cpp` per the issue-#106 host-link recipe
 * (see `test_path_tracker_*` for the canonical write-up of the recipe) so the DISPATCHED
 * `MiniR4.DriveDC` PWM — post power-clamp, post slew-limit, post creep-floor — is observable,
 * not just the policy's return value. `PathTracker` / `Localization` / `routes.cpp` are
 * DELIBERATELY NOT linked: `Drive.cpp` does not touch any of them (grep confirms), so this
 * target stays the minimal three-file link (`Drive.cpp`, `YawSource.cpp`, `path_mocks.cpp`).
 *
 * ===========================================================================================
 * THE TWO-TICK DISPATCH GOTCHA (read this before adding a case)
 * ===========================================================================================
 * `Drive::update()` runs `_updateTurnProfile()` (stage 2) BEFORE `_dispatchNext()` (stage 3).
 * So the `Drive.update()` call that pops a queued `.async()` turn off the queue does NOT also
 * run the turn's first PD step — `_turnProfileState` is still `IDLE` when stage 2 runs that
 * tick. The FIRST REAL PD STEP (the first `MiniR4.DriveDC.Move(...)`) happens on the NEXT
 * `Drive.update()` call. Every case below ticks once to dispatch, THEN ticks again (or more)
 * to observe PD behaviour — see `turn_harness::tickAtYaw` / `enqueueTurnDeg` below.
 *
 * ===========================================================================================
 * lastMoveL/R vs lastMoveSyncL/R — WHICH FIELD TO ASSERT
 * ===========================================================================================
 * The turn FSM (`_updateTurnProfile`) ALWAYS dispatches via the raw `MiniR4.DriveDC.Move(L,
 * R)` primitive (`TurnSignPolicy.h::turnWheelPattern` -> `Move`), regardless of any
 * `DriveMode` (RAW/SYNC/GYRO) — turn dispatch does not go through `_emitContinuous` at all, so
 * every turn-PD assertion in this file reads `MockDriveDC::lastMoveL` / `lastMoveR`. The
 * wallStop PUSH phase (TC15/TC16) is DIFFERENT: it drives through `_emitContinuous`, so this
 * file dispatches it explicitly with `.setMode(DriveMode::SYNC)` and reads
 * `lastMoveSyncL`/`lastMoveSyncR` there instead. Do not swap these two — asserting the wrong
 * field reads a stale/zero value and fails loudly rather than false-greening (the mock never
 * mixes the two fields), but it is still the wrong assertion for the wrong regime.
 *
 * ===========================================================================================
 * THE TWO-LEVEL BREAKAWAY FLOOR (beyond the original plan; committed alongside T4)
 * ===========================================================================================
 * `_updateTurnProfile` floors a sub-threshold PD command to ONE OF TWO fixed values depending
 * on `_turnBrokeAway`: the higher STATIC `Config::TURN_BREAKAWAY_PWM` from rest (bench: a spin
 * scrubs the tires sideways, and static scrub friction beats kinetic), latching DOWN to the
 * lower KINETIC `Config::TURN_CREEP_PWM` the first tick the yaw has moved
 * `Config::TURN_BREAKAWAY_MOTION_DEG` from the turn's dispatch-time yaw. The latch never
 * un-sets. TC10/TC-breakaway-handoff/TC-small-turn below exercise this directly with REAL
 * dispatched values, not the policy in isolation (T5 already covers `turnCreepFloor` itself).
 *
 * Cross-refs: ADR-068, `docs/plans/2026-07-16-turn-pd-controller.md` (Testing Strategy table,
 * TC9-TC17), `TurnPDPolicy.h`, `TurnSignPolicy.h`.
 */

#include "Config.h"
#include "src/drive/Drive.h"
#include "src/nav/YawSource.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "WROLineArray.h"
#include "Intersection.h"
#include "LineFollower.h"

#include "TestAssert.h"

#include <cstdio>

// ===========================================================================================
// Local integration harness — scoped to Drive + YawSource only (no PathTracker/Localization).
// Mirrors the per-tick / per-case idioms established by `test_path_tracker_harness.h`
// (CONTRACT 2's re-park-every-tick rule in particular), but does not #include that header:
// it drives the real `PathTracker`/`Localization` singletons this target does not link.
// ===========================================================================================
namespace turn_harness
{
/// @brief millis advanced per tick — small enough that no watchdog trips between two ticks of
///        a multi-tick handshake, matching `path_harness::kMillisPerTick`.
constexpr uint32_t kTickMs = 5;

/// @brief Generous per-case tick budget so a real-FSM regression that never converges fails as
///        a budget-exhausted assert, never an infinite loop (mirrors
///        `path_harness::kMaxHandshakeTicks`).
constexpr int kMaxTicks = 64;

/// @brief Absolute-value helper for `int16_t` dispatched PWM, avoiding any `std::abs` overload
///        ambiguity across host toolchains.
inline int16_t magOf(int16_t v)
{
    return (int16_t) (v < 0 ? -v : v);
}

/**
 * @brief Reset every mock surface and the real `Drive` singleton to a clean per-case world.
 *
 * @details
 * `Drive.stop(true)` runs FIRST — it force-aborts ANY in-flight profile / turn-profile /
 * stopAtLine / wallStop state machine and drops the queue (`DriveClass::stop`, Drive.cpp),
 * regardless of what a PRIOR test in this same process left in flight. This is load-bearing:
 * `Drive.begin()` alone does NOT reset `_turnProfileState` / `_wallState` / `_qCount` (see its
 * own body — it only reseeds `_heldYaw`/`_moveDone`/`_turnDone` and re-pushes the PID configs),
 * and several cases below (TC10, the slew-ramp case) deliberately return with a turn still
 * mid-flight to observe an early tick, never draining it to IDLE themselves. Calling `stop()`
 * BEFORE `mockResetMiniR4()` lets the mock-reset wipe the call-count/last-arg artifacts `stop()`
 * itself records, so every case starts from an observably pristine mock (`moveCallCount == 0`,
 * etc.) despite the extra `Move(0,0)`/`brake(true)` `stop()` issues internally.
 */
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

/**
 * @brief Convert an APP-frame yaw (any float magnitude, including sub-degree residuals) to the
 *        raw hw-frame value `mmL.mockYawDeg` expects.
 *
 * @details
 * Deliberately NOT `YawSource::toHwHeading` — that helper only takes/returns `int16_t` and
 * would truncate the sub-degree residuals several cases below script (TC11's 0.3 deg, TC11b's
 * exact `Config::TURN_TOLERANCE_DEG` boundary, the breakaway hand-off's fractions of
 * `Config::TURN_BREAKAWAY_MOTION_DEG`). This applies the SAME formula `YawSource` does
 * internally (`hwHeading = appHeading * IMU_YAW_SIGN`) at full `float` precision; since
 * `IMU_YAW_SIGN` is exactly +/-1.0f, the round trip through `YawSource::readYawDeg` (which
 * multiplies back by the same sign) recovers the scripted app value exactly.
 */
inline double hwYawF(float appYawDeg)
{
    return (double) (appYawDeg * Config::IMU_YAW_SIGN);
}

/**
 * @brief Park the mock IMU yaw at an APP-frame heading and run ONE `Drive.update()` tick,
 *        advancing the mock clock by `kTickMs` afterward.
 *
 * @note Re-parks the yaw on EVERY call, matching the path-tracker harness's CONTRACT 2 idiom —
 *       a tick this helper does not drive reads whatever `mmL.mockYawDeg` was last set to, so a
 *       caller that wants a STEADY yaw across several ticks must pass the same value each time.
 */
inline void tickAtYaw(float appYawDeg)
{
    mmL.mockYawDeg = hwYawF(appYawDeg);
    Drive.update();
    mockAdvanceMillis(turn_harness::kTickMs);
}

/**
 * @brief Park the yaw AT @p targetAppYaw and tick until `Drive.isTurnDone()` or the budget
 *        (`kMaxTicks`) is exhausted.
 * @return `true` if the turn settled within budget.
 */
inline bool tickUntilTurnDone(float targetAppYaw, int maxTicks)
{
    for (int i = 0; i < maxTicks; ++i)
    {
        turn_harness::tickAtYaw(targetAppYaw);
        if (Drive.isTurnDone())
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Enqueue an ASYNC `Drive.turnDeg(...)` command (no drain — the caller ticks it).
 *
 * @param deg   Relative yaw delta, degrees.
 * @param mode  Wheel-pattern byte (0=COUNTER_ROT / 1=PIVOT_LEFT / 2=PIVOT_RIGHT).
 * @param power Authority-clamp override; `0` (default) leaves the builder's own default
 *              (`Config::TURN_CRUISE_PWM`) in place — mirrors the `power > 0 ? power :
 *              Config::TURN_CRUISE_PWM` sentinel `_execTurnDeg` itself resolves.
 * @note Move-only RAII (gotcha_drive_builder_raii): built and destructed in ONE statement,
 *       never stored in a local variable.
 */
inline void enqueueTurnDeg(int16_t deg, uint8_t mode = Config::TURN_MODE, int16_t power = 0)
{
    if (power > 0)
    {
        Drive.turnDeg(deg).setMode(mode).setPower(power).async();
    }
    else
    {
        Drive.turnDeg(deg).setMode(mode).async();
    }
}
}    // namespace turn_harness

// ===========================================================================================
// TC9 — settle debounce needs Config::TURN_SETTLE_SAMPLES CONSECUTIVE in-band ticks, not the
// first; final stop is Move(0,0) + brake.
// ===========================================================================================
static void test_TC9_settle_requires_consecutive_in_band_samples()
{
    TEST_SCOPE("TC9: settle exits only after TURN_SETTLE_SAMPLES consecutive in-band ticks");
    turn_harness::resetWorld();

    const float startYaw  = 0.0f;
    const int16_t deg     = 20;
    const float targetYaw = startYaw + (float) deg;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch tick — no PD step yet (see the file docblock).
    ASSERT_FALSE(Drive.isTurnDone());

    // Park EXACTLY at the target for every subsequent tick. The debounce must still require
    // Config::TURN_SETTLE_SAMPLES consecutive in-band ticks, not declare done on the first.
    for (uint8_t i = 0; i + 1 < Config::TURN_SETTLE_SAMPLES; ++i)
    {
        turn_harness::tickAtYaw(targetYaw);
        ASSERT_FALSE(Drive.isTurnDone());
    }
    turn_harness::tickAtYaw(targetYaw);    // the Nth (final) consecutive in-band tick.

    ASSERT_TRUE(Drive.isTurnDone());
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, 0);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
}

// ===========================================================================================
// TC10 (rewritten) — first tick of a turn FROM REST snaps UP to the STATIC breakaway floor,
// not the slew-limited ramp value and not the kinetic creep. (The original plan's TC10
// expected `abs(cmd) <= TURN_SLEW_PWM_PER_TICK` on tick 1 — that assumed `_turnPrevSpeed`
// seeded at 0. It is now seeded at `Config::TURN_BREAKAWAY_PWM`, so a small-error command
// (well under the breakaway threshold) snaps straight to the floor instead of ramping.)
// ===========================================================================================
static void test_TC10_first_tick_from_rest_snaps_to_breakaway_floor()
{
    TEST_SCOPE("TC10 (rewritten): first tick from rest == TURN_BREAKAWAY_PWM, "
               "NOT <= TURN_SLEW_PWM_PER_TICK, NOT TURN_CREEP_PWM");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 20;    // |Kp*err| == 20, well under the breakaway threshold -> floored.

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    ASSERT_FALSE(Drive.isTurnDone());

    turn_harness::tickAtYaw(startYaw);    // FIRST real PD tick — chassis has not moved (from rest).

    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, Config::TURN_BREAKAWAY_PWM);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) -Config::TURN_BREAKAWAY_PWM);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveR > Config::TURN_SLEW_PWM_PER_TICK);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveR != Config::TURN_CREEP_PWM);
}

// -------------------------------------------------------------------------------------------
// Companion to TC10: the slew limiter DOES show up, but on later RAMP ticks of a large-error
// turn whose authority is raised well above `TURN_BREAKAWAY_PWM + TURN_SLEW_PWM_PER_TICK` (a
// `.setPower(...)` override). With the DEFAULT authority (Config::TURN_CRUISE_PWM), the
// breakaway seed (46) is already within one slew step of cruise (50), so a default-power turn
// never needs a multi-tick ramp — this is what forces the elevated power here.
// -------------------------------------------------------------------------------------------
static void test_slew_limit_binds_on_later_ramp_ticks_of_elevated_power_turn()
{
    TEST_SCOPE("slew limiter binds on later ramp ticks when authority is raised well above "
               "breakaway+slew (NOT visible on tick 1 of a default-power turn)");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 150;    // large, near-maximal error — stays saturated at the raised
                                     // clamp on every tick since the yaw never moves in this case.
    const int16_t power    = 90;     // >> TURN_BREAKAWAY_PWM + TURN_SLEW_PWM_PER_TICK.

    turn_harness::enqueueTurnDeg(deg, Config::TURN_MODE, power);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    ASSERT_FALSE(Drive.isTurnDone());

    int16_t prevMag = Config::TURN_BREAKAWAY_PWM;    // the seeded ramp floor.
    for (int i = 0; i < 4; ++i)
    {
        turn_harness::tickAtYaw(startYaw);    // yaw held at start -> the raw PD command stays
                                              // saturated at the raised clamp every tick.
        const int16_t mag = MiniR4.DriveDC.lastMoveR;
        ASSERT_TRUE(mag > prevMag);                                     // still ramping up.
        ASSERT_TRUE(mag - prevMag <= Config::TURN_SLEW_PWM_PER_TICK);    // bounded by the slew rate.
        prevMag = mag;
    }
    // After 4 ramp ticks the dispatched magnitude has NOT yet reached the raised authority
    // ceiling — proof the slew limiter, not the power clamp, governed these ticks.
    ASSERT_TRUE(prevMag < power);
}

// ===========================================================================================
// Breakaway hand-off: every dispatched floor is the STATIC breakaway (46) while the chassis
// has moved < Config::TURN_BREAKAWAY_MOTION_DEG from the turn's start yaw; the FIRST tick the
// cumulative motion reaches that threshold, the floor drops to the KINETIC creep (38) and
// stays there (latched) even if the yaw regresses back under the threshold afterward.
// ===========================================================================================
static void test_breakaway_handoff_latches_static_to_kinetic_floor()
{
    TEST_SCOPE("breakaway hand-off: floor is 46 (static) below TURN_BREAKAWAY_MOTION_DEG of "
               "motion, drops to 38 (kinetic) the FIRST tick motion reaches it, then latches");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 20;    // Kp*err stays well under BOTH floor thresholds throughout,
                                    // so the ONLY thing that can change the dispatched magnitude
                                    // is which floor is selected.

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.

    turn_harness::tickAtYaw(startYaw);    // 0.0x TURN_BREAKAWAY_MOTION_DEG moved.
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG * 0.4f);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG * 0.9f);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    // Cross the breakaway-motion threshold — the check runs BEFORE the PD step, so the SAME
    // tick this is observed must already show the kinetic floor.
    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG * 1.2f);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_CREEP_PWM);

    // Latched: a regression back UNDER the threshold (relative to the ORIGINAL start yaw) must
    // NOT un-set it.
    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG * 0.5f);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_CREEP_PWM);
}

// ===========================================================================================
// Small turn does not stall: a ~20 deg turn keeps dispatching the STATIC breakaway floor (46)
// until the chassis is demonstrably rotating — a single static 38 kick (below the 46 measured
// breakaway) would never start the spin at all. Once moving, hand off to the kinetic floor and
// converge to a clean settle (not a hunt-to-timeout).
// ===========================================================================================
static void test_small_turn_breaks_away_then_settles_cleanly()
{
    TEST_SCOPE("small turn (~20deg): floors at breakaway (>=46) until moving, "
               "then converges via TURN_SETTLE_SAMPLES in-band ticks");
    turn_harness::resetWorld();

    const float   startYaw  = 0.0f;
    const int16_t deg       = 20;
    const float   targetYaw = startYaw + (float) deg;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.

    // From rest: tick 1 MUST already be the breakaway floor (>= 46), never the lower creep.
    turn_harness::tickAtYaw(startYaw);
    ASSERT_TRUE(turn_harness::magOf(MiniR4.DriveDC.lastMoveR) >= Config::TURN_BREAKAWAY_PWM);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    // Still not (yet) demonstrably rotating — the floor stays at the static breakaway value.
    turn_harness::tickAtYaw(startYaw);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    // Now let the chassis actually move past the breakaway-motion threshold.
    turn_harness::tickAtYaw(startYaw + 5.0f);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_CREEP_PWM);

    // Drive the rest of the way to the target and confirm a clean settle (not a hunt).
    ASSERT_TRUE(turn_harness::tickUntilTurnDone(targetYaw, turn_harness::kMaxTicks));
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// Overshoot: script the yaw PAST the target (skipping the in-band window entirely, so no
// premature settle claim is even possible) — the turn must NOT declare settled while
// |err| > tolerance, and the dispatched command must REVERSE sign to correct back.
// ===========================================================================================
static void test_overshoot_never_settles_out_of_band_and_reverses_command()
{
    TEST_SCOPE("overshoot: |err| > TURN_TOLERANCE_DEG blocks settle; dispatched command reverses");
    turn_harness::resetWorld();

    const float   startYaw  = 0.0f;
    const int16_t deg       = 90;
    const float   targetYaw = startYaw + (float) deg;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.

    // Approach: two ticks parked at the SAME still-short-of-target yaw. The FIRST of the two
    // still carries the transient (teleport-from-dispatch) derivative kick from the big
    // instantaneous error drop (dispatch error 90 -> 5 in one tick) — exactly like TC9's first
    // in-band tick — so it is not representative of a steady approach. The SECOND tick has
    // errOld == errNew (yaw unchanged), so the D-term contributes ~0 and the command is a clean
    // forward (positive R) proportional term.
    turn_harness::tickAtYaw(targetYaw - 5.0f);
    turn_harness::tickAtYaw(targetYaw - 5.0f);
    const int16_t forwardR = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(forwardR > 0);

    // Overshoot tick — skip straight PAST the target by more than the tolerance.
    const float overshotYaw = targetYaw + Config::TURN_TOLERANCE_DEG + 0.5f;
    turn_harness::tickAtYaw(overshotYaw);

    // The fabsf() at the settle call must keep a negative, still-out-of-band overshoot error
    // from ever reading as "in band".
    ASSERT_FALSE(Drive.isTurnDone());

    // The dispatched command must have REVERSED sign to correct back toward the target.
    const int16_t overshotR = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(overshotR < 0);
    ASSERT_TRUE((forwardR > 0) != (overshotR > 0));
}

// ===========================================================================================
// TC11 — a small (0.3 deg) residual, deliberately INSIDE the tolerance band (never exactly ON
// the boundary — see TC11b), converges and brakes; the dispatched command never equals the
// STATIC deadband floor PWM_MIN_MAGNITUDE (that is `deadbandSnap`'s floor, not the turn's).
// ===========================================================================================
static void test_TC11_small_in_band_residual_converges_never_uses_static_deadband_floor()
{
    TEST_SCOPE("TC11: small in-band residual converges & brakes; never emits PWM_MIN_MAGNITUDE");
    turn_harness::resetWorld();

    const float   startYaw    = 0.0f;
    const int16_t deg         = 20;
    const float   residualYaw = startYaw + (float) deg - 0.3f;    // 0.3deg short of target.

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.

    bool done = false;
    for (int i = 0; i < turn_harness::kMaxTicks && !done; ++i)
    {
        turn_harness::tickAtYaw(residualYaw);
        ASSERT_TRUE(MiniR4.DriveDC.lastMoveL != Config::PWM_MIN_MAGNITUDE);
        ASSERT_TRUE(MiniR4.DriveDC.lastMoveL != (int16_t) -Config::PWM_MIN_MAGNITUDE);
        ASSERT_TRUE(MiniR4.DriveDC.lastMoveR != Config::PWM_MIN_MAGNITUDE);
        ASSERT_TRUE(MiniR4.DriveDC.lastMoveR != (int16_t) -Config::PWM_MIN_MAGNITUDE);
        done = Drive.isTurnDone();
    }
    ASSERT_TRUE(done);
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
}

// ===========================================================================================
// TC11b — a residual EXACTLY AT Config::TURN_TOLERANCE_DEG is OUT of band (the settle compare
// is strict-<). Pins the boundary at the FSM level so a later </<= slip is caught here too, not
// just in the T5 policy unit test.
// ===========================================================================================
static void test_TC11b_residual_exactly_at_tolerance_boundary_stays_out_of_band()
{
    TEST_SCOPE("TC11b: |err| == TURN_TOLERANCE_DEG is OUT of band (strict-<) — never settles");
    turn_harness::resetWorld();

    const float   startYaw    = 0.0f;
    const int16_t deg         = 20;
    const float   boundaryYaw = startYaw + (float) deg - Config::TURN_TOLERANCE_DEG;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.

    for (int i = 0; i < 5; ++i)
    {
        turn_harness::tickAtYaw(boundaryYaw);
        ASSERT_FALSE(Drive.isTurnDone());
    }
}

// ===========================================================================================
// TC12 — a residual that never closes (5 deg, well outside tolerance) times out at
// Config::TURN_TIMEOUT_MS: brakes, isTurnDone() true, hasFault() FALSE, and the QUEUE is NOT
// cleared (a second queued turn dispatches and starts issuing real Move() traffic afterward).
// ===========================================================================================
static void test_TC12_non_convergence_times_out_without_fault_or_queue_clear()
{
    TEST_SCOPE("TC12: a residual that never closes times out — no fault, queue stays intact");
    turn_harness::resetWorld();

    const float   startYaw  = 0.0f;
    const int16_t deg       = 20;
    const float   stuckYaw  = startYaw + (float) deg - 5.0f;    // 5deg residual, never closes.

    // Deliberately only ONE command queued for the timeout observation below. Pre-queueing a
    // second command here would race the very thing this test checks: the moment the first
    // turn's timeout fires, stage 3 of that SAME `Drive.update()` call would immediately
    // dispatch the second (non-short-circuit) turn and flip `_turnDone` back to `false` before
    // this test ever gets to read it — `isTurnDone()` would NEVER be observed `true` even
    // though the FSM behaved correctly. Enqueue the second command only AFTER independently
    // confirming the first one timed out cleanly (below), which cannot race anything.
    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.

    bool      done         = false;
    const int timeoutTicks = (int) (Config::TURN_TIMEOUT_MS / turn_harness::kTickMs) + 20;
    for (int i = 0; i < timeoutTicks && !done; ++i)
    {
        turn_harness::tickAtYaw(stuckYaw);
        done = Drive.isTurnDone();
    }
    ASSERT_TRUE(done);
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    ASSERT_TRUE(Drive.isIdle());    // qCount == 0 here -- nothing else was ever queued.

    // NOW enqueue a second turn and confirm the queue is genuinely alive: it dispatches (a
    // fresh, non-short-circuit 20deg delta off the current stuck yaw) and starts issuing real
    // PD Move() traffic — proving the FIRST turn's non-fault timeout did NOT clear the queue.
    const uint32_t movesBeforeSecond = MiniR4.DriveDC.moveCallCount;
    turn_harness::enqueueTurnDeg(deg);
    for (int i = 0; i < turn_harness::kMaxTicks; ++i)
    {
        turn_harness::tickAtYaw(stuckYaw);
    }
    ASSERT_TRUE(MiniR4.DriveDC.moveCallCount > movesBeforeSecond);
}

// ===========================================================================================
// TC13 — a SUSTAINED IMU read failure escalates to Fault::IMU_READ_FAILED past
// Config::TURN_IMU_STALL_TIMEOUT_MS, and the queue IS cleared (a second queued turn never
// dispatches, even after the read recovers and the fault is cleared).
// ===========================================================================================
static void test_TC13_sustained_imu_failure_faults_and_clears_queue()
{
    TEST_SCOPE("TC13: sustained IMU read failure -> IMU_READ_FAILED past the stall watchdog; "
               "queue IS cleared");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 20;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::enqueueTurnDeg(deg);    // second queued turn — must NEVER dispatch if the
                                          // fault correctly clears the queue.
    turn_harness::tickAtYaw(startYaw);    // dispatch the FIRST turn with a GOOD read.
    ASSERT_FALSE(Drive.hasFault());

    mmL.mockResult = MMLower::RESULT::ERROR;

    bool      faulted     = false;
    const int budgetTicks = (int) (Config::TURN_IMU_STALL_TIMEOUT_MS / turn_harness::kTickMs) + 40;
    for (int i = 0; i < budgetTicks && !faulted; ++i)
    {
        turn_harness::tickAtYaw(startYaw);
        faulted = Drive.hasFault();
    }
    ASSERT_TRUE(faulted);
    ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::IMU_READ_FAILED);
    ASSERT_TRUE(Drive.isTurnDone());
    ASSERT_TRUE(Drive.isIdle());    // _qCount == 0 -> proves the SECOND queued turn was dropped.

    // Restore the read and clear the fault: if the queue had NOT been cleared, the second
    // queued turn would now dispatch and issue real Move() traffic. Confirm it does NOT.
    mmL.mockResult               = MMLower::RESULT::OK;
    Drive.clearFault();
    const uint32_t movesAtClear = MiniR4.DriveDC.moveCallCount;
    for (int i = 0; i < turn_harness::kMaxTicks; ++i)
    {
        turn_harness::tickAtYaw(startYaw);
    }
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, movesAtClear);
    ASSERT_TRUE(Drive.isIdle());
}

// ===========================================================================================
// TC14 — a TRANSIENT (single-tick) IMU read failure is skipped: the prior Move(L,R) is held
// verbatim (no new Move() call that tick), no fault latches, and the turn still completes once
// the read recovers.
// ===========================================================================================
static void test_TC14_transient_single_tick_imu_failure_holds_prior_move_and_recovers()
{
    TEST_SCOPE("TC14: a ONE-tick IMU read failure is skipped, holding the prior Move(L,R); "
               "no fault; the turn still completes");
    turn_harness::resetWorld();

    const float   startYaw  = 0.0f;
    const int16_t deg       = 20;
    const float   targetYaw = startYaw + (float) deg;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // first real PD tick — records a Move(L,R).

    const int16_t  priorL      = MiniR4.DriveDC.lastMoveL;
    const int16_t  priorR      = MiniR4.DriveDC.lastMoveR;
    const uint32_t movesBefore = MiniR4.DriveDC.moveCallCount;

    // Fail exactly ONE tick.
    mmL.mockResult = MMLower::RESULT::ERROR;
    turn_harness::tickAtYaw(startYaw);

    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, movesBefore);    // no NEW Move() call this tick.
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, priorL);              // prior command held verbatim.
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, priorR);
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_FALSE(Drive.isTurnDone());

    // Recover: restore the read, park the yaw at the target, confirm the turn still completes
    // cleanly with no lingering effect from the transient miss.
    mmL.mockResult = MMLower::RESULT::OK;
    ASSERT_TRUE(turn_harness::tickUntilTurnDone(targetYaw, turn_harness::kMaxTicks));
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC15 — `wallStop` SQUARE turns to a NON-ZERO wall heading (a real, non-trivial in-place
// turn), then PUSH seeds on the tick right after SQUARE settles. The SQUARE dispatch passes
// `power=0` as a sentinel that `_execTurnToYaw` resolves to `Config::TURN_CRUISE_PWM`; this is
// indirectly but concretely proven by the SQUARE turn actually issuing nonzero Move() traffic
// (a literal zero-authority clamp would leave every dispatched command at exactly 0 and the
// turn would hunt to its 3s timeout instead of squaring up).
// ===========================================================================================
static void test_TC15_wallstop_squares_to_nonzero_heading_then_seeds_push()
{
    TEST_SCOPE("TC15: wallStop SQUARE turns to the wall heading, then PUSH seeds the very next tick");
    turn_harness::resetWorld();

    const int16_t wallHeading = 90;
    const float   startYaw    = 0.0f;

    Drive.wallStop(10.0f).setMode(DriveMode::SYNC).setHeading(wallHeading).setBackOff(0.0f).async();
    turn_harness::tickAtYaw(startYaw);    // dispatch -> SQUARE (a real 90deg turn, non-trivial).
    ASSERT_FALSE(Drive.isTurnDone());

    bool moved = false;
    for (int i = 0; i < turn_harness::kMaxTicks && !Drive.isTurnDone(); ++i)
    {
        turn_harness::tickAtYaw((float) wallHeading);
        if (MiniR4.DriveDC.lastMoveL != 0 || MiniR4.DriveDC.lastMoveR != 0)
        {
            moved = true;
        }
    }
    ASSERT_TRUE(moved);            // the power=0 sentinel resolved to a REAL nonzero authority.
    ASSERT_TRUE(Drive.isTurnDone());

    // The tick right after SQUARE settles seeds PUSH's watchdogs and returns WITHOUT commanding
    // motion (`_updateWallStop`'s "begin pushing on the next tick" comment) — so the FIRST real
    // PUSH dispatch is the tick AFTER THAT.
    turn_harness::tickAtYaw((float) wallHeading);
    turn_harness::tickAtYaw((float) wallHeading);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::WALL_STOP_PUSH_PWM);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncR, Config::WALL_STOP_PUSH_PWM);

    // Retire the push: freeze the encoder (stall) and jump the clock past the stall window.
    // The stall confirmation, the PUSH->BACK_OFF transition, the (skipped, `setBackOff(0)`)
    // BACK_OFF->DONE transition, and DONE's own finalisation (`_endWallStop`, which only runs
    // at the TOP of `_updateWallStop` on a LATER tick) each consume a SEPARATE tick, so this
    // is bounded rather than a single fixed-count tick.
    mockAdvanceMillis(Config::STOP_AT_LINE_STALL_TIMEOUT_MS + 1);
    for (int i = 0; i < turn_harness::kMaxTicks && !Drive.isIdle(); ++i)
    {
        turn_harness::tickAtYaw((float) wallHeading);
    }

    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.wallStopTimedOut());
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC16 — `wallStop` dispatched ALREADY facing the wall: the internal SQUARE turn short-circuits
// at dispatch (no turning ticks spent, no turn Move() traffic at all), and PUSH begins on the
// very next tick.
// ===========================================================================================
static void test_TC16_wallstop_already_squared_short_circuits_push_next_tick()
{
    TEST_SCOPE("TC16: wallStop dispatched already facing the wall short-circuits SQUARE; "
               "PUSH begins the very next tick");
    turn_harness::resetWorld();

    const int16_t wallHeading = 45;
    const float   startYaw    = (float) wallHeading;    // already facing the wall at dispatch.

    Drive.wallStop(10.0f).setMode(DriveMode::SYNC).setHeading(wallHeading).setBackOff(0.0f).async();
    turn_harness::tickAtYaw(startYaw);    // dispatch — the internal SQUARE turn short-circuits
                                          // at dispatch (delta ~= 0), so isTurnDone() reads true
                                          // immediately.
    ASSERT_TRUE(Drive.isTurnDone());
    const uint32_t movesAfterDispatch = MiniR4.DriveDC.moveCallCount;

    // The VERY NEXT tick: `_updateWallStop` sees the already-idle turn profile and advances
    // SQUARE -> PUSH immediately, seeding watchdogs WITHOUT commanding motion this tick.
    turn_harness::tickAtYaw(startYaw);
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, movesAfterDispatch);    // no turn Move() traffic at all.

    // And the tick after THAT is PUSH's first real dispatch.
    turn_harness::tickAtYaw(startYaw);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::WALL_STOP_PUSH_PWM);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncR, Config::WALL_STOP_PUSH_PWM);

    // Retire cleanly (stall confirm -> BACK_OFF -> DONE -> finalise are each a separate tick).
    mockAdvanceMillis(Config::STOP_AT_LINE_STALL_TIMEOUT_MS + 1);
    for (int i = 0; i < turn_harness::kMaxTicks && !Drive.isIdle(); ++i)
    {
        turn_harness::tickAtYaw(startYaw);
    }
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.wallStopTimedOut());
}

// ===========================================================================================
// TC17 — a pivot turn (mode 1 = PIVOT_LEFT / mode 2 = PIVOT_RIGHT) commands ONLY the free
// wheel; the anchored wheel's dispatched PWM is EXACTLY 0.
// ===========================================================================================
static void test_TC17_pivot_turn_holds_the_anchored_wheel_at_zero()
{
    TEST_SCOPE("TC17: a pivot turn holds the anchored wheel at exactly 0; only the free wheel moves");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 20;

    // PIVOT_LEFT (mode 1): left wheel anchored at 0, right wheel carries the command.
    turn_harness::enqueueTurnDeg(deg, /*mode PIVOT_LEFT*/ 1);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // first real PD tick.

    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, 0);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveR != 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, Config::TURN_BREAKAWAY_PWM);

    // Mirror — PIVOT_RIGHT (mode 2): right wheel anchored at 0, left wheel carries the command.
    turn_harness::resetWorld();
    turn_harness::enqueueTurnDeg(deg, /*mode PIVOT_RIGHT*/ 2);
    turn_harness::tickAtYaw(startYaw);
    turn_harness::tickAtYaw(startYaw);

    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, 0);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveL != 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) -Config::TURN_BREAKAWAY_PWM);
}

// ===========================================================================================
// Entry.
// ===========================================================================================

int main()
{
    std::printf("INFO: Turn PD controller — real Drive.cpp FSM integration (ADR-068, T6, "
                "TC9-TC17 + breakaway/kinetic floor cases)\n");

    test_TC9_settle_requires_consecutive_in_band_samples();
    test_TC10_first_tick_from_rest_snaps_to_breakaway_floor();
    test_slew_limit_binds_on_later_ramp_ticks_of_elevated_power_turn();
    test_breakaway_handoff_latches_static_to_kinetic_floor();
    test_small_turn_breaks_away_then_settles_cleanly();
    test_overshoot_never_settles_out_of_band_and_reverses_command();
    test_TC11_small_in_band_residual_converges_never_uses_static_deadband_floor();
    test_TC11b_residual_exactly_at_tolerance_boundary_stays_out_of_band();
    test_TC12_non_convergence_times_out_without_fault_or_queue_clear();
    test_TC13_sustained_imu_failure_faults_and_clears_queue();
    test_TC14_transient_single_tick_imu_failure_holds_prior_move_and_recovers();
    test_TC15_wallstop_squares_to_nonzero_heading_then_seeds_push();
    test_TC16_wallstop_already_squared_short_circuits_push_next_tick();
    test_TC17_pivot_turn_holds_the_anchored_wheel_at_zero();

    TEST_REPORT();
}
