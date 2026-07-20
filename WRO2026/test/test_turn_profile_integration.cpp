/**
 * @file test_turn_profile_integration.cpp
 * @brief Stateful host-link integration test for the in-place turn's gated speed profile
 *        (ADR-078, T4, TC9-TC18): breakaway -> CRUISE -> CREEP -> SETTLE.
 *
 * @details
 * `test_turn_profile_policy.cpp` proves the PURE decision math in `TurnProfilePolicy.h` in
 * isolation. This file is the STATEFUL test of the turn FSM itself
 * (`DriveClass::_updateTurnProfile`, Drive.cpp) — it links the REAL `Drive.cpp` + `YawSource.cpp`
 * per the issue-#106 host-link recipe (see `test_path_tracker_*` for the canonical write-up of
 * the recipe) so the DISPATCHED `MiniR4.DriveDC` PWM is observable, not just the policy's return
 * value. `PathTracker` / `Localization` / `routes.cpp` are DELIBERATELY NOT linked: `Drive.cpp`
 * does not touch any of them (grep confirms), so this target stays the minimal three-file link
 * (`Drive.cpp`, `YawSource.cpp`, `path_mocks.cpp`) — same shape as the file this replaces,
 * `test_turn_pd_integration.cpp`.
 *
 * This target intentionally does NOT re-test `wallStop`'s SQUARE->PUSH handoff or the sustained /
 * transient IMU-read-failure escalation the old PD-era file also covered: `wallStop` has its own
 * dedicated coverage (`test_wall_stop_watchdog.cpp`), and the IMU-stall PREDICATE
 * (`turnImuReadStalled`) — unchanged by ADR-078 and kept verbatim above the profile block in
 * `Drive.cpp` — is host-tested in isolation by `test_turn_imu_watchdog.cpp`. This file's scope is
 * exactly the profile's own gated speed-phase dispatch, per the plan's Testing Strategy table.
 *
 * ===========================================================================================
 * THE TWO-TICK DISPATCH GOTCHA (read this before adding a case)
 * ===========================================================================================
 * `Drive::update()` runs `_updateTurnProfile()` (stage 2) BEFORE `_dispatchNext()` (stage 3).
 * So the `Drive.update()` call that pops a queued `.async()` turn off the queue does NOT also
 * run the turn's first profile step — `_turnProfileState` is still `IDLE` when stage 2 runs that
 * tick. The FIRST REAL PROFILE STEP (the first `MiniR4.DriveDC.Move(...)`) happens on the NEXT
 * `Drive.update()` call. Every case below ticks once to dispatch, THEN ticks again (or more) to
 * observe profile behaviour — see `turn_harness::tickAtYaw` / `enqueueTurnDeg` below.
 *
 * ===========================================================================================
 * lastMoveL/R — WHICH FIELD TO ASSERT
 * ===========================================================================================
 * The turn FSM (`_updateTurnProfile`) ALWAYS dispatches via the raw `MiniR4.DriveDC.Move(L, R)`
 * primitive (`TurnSignPolicy.h::turnWheelPattern` -> `Move`), regardless of any `DriveMode`
 * (RAW/SYNC/GYRO) — turn dispatch does not go through `_emitContinuous` at all, so every
 * assertion in this file reads `MockDriveDC::lastMoveL` / `lastMoveR`, the ACTUAL dispatched
 * setpoint — never a plan value. That is the whole point of a stateful integration target: it
 * witnesses real dispatch, not the pure policy math (`test_turn_profile_policy.cpp` already
 * covers that in isolation).
 *
 * ===========================================================================================
 * THE BREAKAWAY GATE OVERRIDES THE PHASE (read this before adding a case)
 * ===========================================================================================
 * `_updateTurnProfile` commands the STATIC `Config::TURN_BREAKAWAY_PWM` kick on EVERY tick until
 * the chassis has demonstrably rotated `Config::TURN_BREAKAWAY_MOTION_DEG` from the turn's
 * dispatch-time yaw — regardless of which phase the error alone would imply (ADR-078; a small
 * turn dispatches straight into CREEP/SETTLE territory and would otherwise command a
 * sub-static-floor speed and never start, ADR-068's third documented failure). So the FIRST real
 * profile tick of every case below reads the breakaway floor, not the phase magnitude; to
 * observe a phase's own magnitude (CRUISE/CREEP/SETTLE), the yaw must first be advanced past
 * `Config::TURN_BREAKAWAY_MOTION_DEG` from the turn's start yaw to latch `_turnBrokeAway`. All
 * scripted yaws in this file are whole-degree values (the mock IMU bridge quantizes to whole
 * degrees, ADR-074), so `Config::TURN_BREAKAWAY_MOTION_DEG` (1.0) is crossed by parking at
 * exactly 1 degree of motion — no fractional-degree scripting needed.
 *
 * Cross-refs: ADR-078, `docs/plans/2026-07-21-turn-profile-controller.md` (Testing Strategy
 * table, TC9-TC18), `TurnProfilePolicy.h`, `TurnPDPolicy.h`, `TurnSignPolicy.h`.
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
 * own body — it only reseeds `_heldYaw`/`_moveDone`/`_turnDone` and re-pushes the PID configs).
 * Calling `stop()` BEFORE `mockResetMiniR4()` lets the mock-reset wipe the call-count/last-arg
 * artifacts `stop()` itself records, so every case starts from an observably pristine mock
 * (`moveCallCount == 0`, etc.) despite the extra `Move(0,0)`/`brake(true)` `stop()` issues
 * internally.
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

/// @brief Test-local yaw sign the mock IMU bridge round-trips through `readYawDeg`, byte-identical
///        to the retired production `Config::IMU_YAW_SIGN` (-1.0f) but INDEPENDENT of it — so this
///        target survives issue #169's removal of `IMU_YAW_SIGN` from Config.
static constexpr float kTurnYawReadSign = -1.0f;

/**
 * @brief Convert an APP-frame yaw (any float magnitude) to the raw-frame value
 *        `mmL.mockYawDeg` expects.
 *
 * @details
 * Applies the mock yaw bridge's fixed sign. This file only scripts whole-degree yaws (see the
 * file docblock), so the mock bridge's own whole-degree quantization never surprises a case
 * here — every scripted value round-trips exactly.
 */
inline double hwYawF(float appYawDeg)
{
    return (double) (appYawDeg * kTurnYawReadSign);
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
// TC9 — a small turn (8deg, well inside the creep window) dispatched from rest still commands
// TURN_BREAKAWAY_PWM on its first real tick, NOT the phase's own (lower) CREEP magnitude — the
// breakaway gate overrides the phase until motion is observed (ADR-068's third failure, retained
// by ADR-078).
// ===========================================================================================
static void test_TC9_small_turn_breaks_away_from_rest_not_creep()
{
    TEST_SCOPE("TC9: small turn (8deg) from rest, tick 1 == TURN_BREAKAWAY_PWM (breaks away, "
               "not creep)");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 8;    // well inside TURN_CREEP_WINDOW_DEG (25) -> phase is CREEP
                                   // from the very first latch, but the chassis has not moved yet.

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch tick — no profile step yet (file docblock).
    ASSERT_FALSE(Drive.isTurnDone());

    turn_harness::tickAtYaw(startYaw);    // first real profile tick, chassis has not moved.

    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);
    ASSERT_TRUE(turn_harness::magOf(MiniR4.DriveDC.lastMoveR) != Config::TURN_CREEP_PWM);
}

// ===========================================================================================
// TC10 — a large turn (90deg, far outside the creep window), once broken away, commands
// `_turnCruisePwm` (the default-power CRUISE magnitude). The FIRST profile tick (chassis still
// at rest) is still the breakaway floor; only the tick where cumulative motion reaches
// TURN_BREAKAWAY_MOTION_DEG shows the CRUISE magnitude — the breakaway check runs BEFORE the
// per-phase command, so the SAME tick that latches the handoff already reflects it.
// ===========================================================================================
static void test_TC10_large_turn_post_breakaway_commands_cruise()
{
    TEST_SCOPE("TC10: large turn (90deg), far from target, post-breakaway == _turnCruisePwm");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 90;    // far outside TURN_CREEP_WINDOW_DEG (25) -> CRUISE the whole way.

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // first real tick — from rest -> breakaway floor.
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    // Cross TURN_BREAKAWAY_MOTION_DEG (1.0) from the start yaw — the SAME tick that latches
    // `_turnBrokeAway` must already dispatch the CRUISE magnitude (still far from the 90deg
    // target, so the phase stays CRUISE).
    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG);
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_CRUISE_PWM);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveR > 0);           // toward the target (errProg still positive).
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) -MiniR4.DriveDC.lastMoveR);    // COUNTER_ROT.
}

// ===========================================================================================
// TC11 — `.setPower(80)` on a large turn, post-breakaway, sets the CRUISE magnitude to the
// override (80), not the raw `Config::TURN_CRUISE_PWM` (50) — `_turnCruisePwm` is seeded from
// the builder override at dispatch and is what `turnProfileSpeed` receives for CRUISE.
// ===========================================================================================
static void test_TC11_power_override_sets_cruise_magnitude()
{
    TEST_SCOPE("TC11: .setPower(80) large turn, post-breakaway == 80 (override honored)");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 90;
    const int16_t power    = 80;

    turn_harness::enqueueTurnDeg(deg, Config::TURN_MODE, power);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // first real tick — breakaway floor, NOT the override.
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG);    // breakaway latches.
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), power);
}

// ===========================================================================================
// TC12 — a turn whose delta (20deg) is already inside the creep window, once broken away,
// commands TURN_CREEP_PWM (38) — the phase latches to CREEP on its very first read (20 <= 25),
// but the breakaway gate still overrides the FIRST tick's command until motion is observed.
// ===========================================================================================
static void test_TC12_creep_window_post_breakaway_commands_creep_pwm()
{
    TEST_SCOPE("TC12: yaw inside the creep window, post-breakaway == TURN_CREEP_PWM");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 20;    // inside TURN_CREEP_WINDOW_DEG (25) -> CREEP from the first latch.

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // first real tick — breakaway floor (chassis at rest).
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_BREAKAWAY_PWM);

    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG);    // breakaway latches.
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_CREEP_PWM);
}

// ===========================================================================================
// TC13 — settle exits only after Config::TURN_SETTLE_SAMPLES CONSECUTIVE strictly-in-band
// ticks, not the first; final stop is Move(0,0) + brake, no fault.
// ===========================================================================================
static void test_TC13_settle_requires_consecutive_in_band_samples()
{
    TEST_SCOPE("TC13: settle exits only after TURN_SETTLE_SAMPLES consecutive in-band ticks");
    turn_harness::resetWorld();

    const float startYaw  = 0.0f;
    const int16_t deg     = 20;
    const float targetYaw = startYaw + (float) deg;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch tick — no profile step yet.
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
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC14 — band-skip overshoot INSIDE CREEP: the yaw is scripted from +4deg short of target
// straight to -4deg past it, SKIPPING the in-band (<TURN_TOLERANCE_DEG) window entirely, so no
// settle sample is ever counted. `copysignf(speed, errProg)` must reverse the dispatched command
// (not merely shrink it), pinning the "every phase is signed by errProg" contract even for
// CREEP, not just SETTLE.
// ===========================================================================================
static void test_TC14_band_skip_overshoot_in_creep_reverses_command()
{
    TEST_SCOPE("TC14: band-skip overshoot in CREEP (+4deg -> -4deg, no in-band sample) reverses "
               "the dispatched command");
    turn_harness::resetWorld();

    const float   startYaw  = 0.0f;
    const int16_t deg       = 20;
    const float   targetYaw = startYaw + (float) deg;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // breakaway floor (at rest).
    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG);    // breakaway latches;
                                                                              // still CREEP (|err|=19).
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_CREEP_PWM);

    // +4deg short of target — still inside the creep window (3 < 4 <= 25), forward command.
    turn_harness::tickAtYaw(targetYaw - 4.0f);
    const int16_t forwardR = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(forwardR > 0);
    ASSERT_EQ(turn_harness::magOf(forwardR), Config::TURN_CREEP_PWM);
    ASSERT_FALSE(Drive.isTurnDone());

    // Skip straight PAST the target to -4deg — still inside the creep window on the other side
    // (|err|=4), but the SIGN has flipped. No sample ever landed inside TURN_TOLERANCE_DEG (3),
    // so the settle debounce never engaged; this is a pure CREEP-phase reversal.
    turn_harness::tickAtYaw(targetYaw + 4.0f);
    const int16_t overshotR = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(overshotR < 0);
    ASSERT_EQ(turn_harness::magOf(overshotR), Config::TURN_CREEP_PWM);
    ASSERT_TRUE((forwardR > 0) != (overshotR > 0));    // the command REVERSED sign.
    ASSERT_FALSE(Drive.isTurnDone());                  // |err| (4) is still > TURN_TOLERANCE_DEG (3).
}

// ===========================================================================================
// TC15 — overshoot AFTER the SETTLE phase has already latched (a sample landed exactly at
// TURN_TOLERANCE_DEG, which naturalizes SETTLE per turnPhaseFor's inclusive boundary but does
// NOT count toward the strict-< settle debounce). A subsequent overshoot must still reverse the
// nudge command and must NOT report done while |err| > tolerance — the monotonic latch keeps the
// phase at SETTLE (never regressing to CREEP) even though the error swung back into the creep
// window's range.
// ===========================================================================================
static void test_TC15_overshoot_after_settle_latched_reverses_nudge()
{
    TEST_SCOPE("TC15: overshoot after SETTLE latched reverses the nudge; not done while "
               "|err| > tol");
    turn_harness::resetWorld();

    const float   startYaw  = 0.0f;
    const int16_t deg       = 20;
    const float   targetYaw = startYaw + (float) deg;

    turn_harness::enqueueTurnDeg(deg);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // breakaway floor (at rest).
    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG);    // breakaway latches;
                                                                              // CREEP (|err|=19).
    ASSERT_EQ(turn_harness::magOf(MiniR4.DriveDC.lastMoveR), Config::TURN_CREEP_PWM);

    // Land EXACTLY at TURN_TOLERANCE_DEG (3deg short of target): turnPhaseFor's inclusive
    // boundary latches SETTLE, but turnSettleStep's strict-< test does NOT count this sample —
    // so the turn is NOT yet settling, only the phase has advanced. The nudge is still signed
    // forward: clamp(TURN_SETTLE_KP * 3, +/-TURN_CREEP_PWM) == clamp(12, +/-38) == +12.
    turn_harness::tickAtYaw(targetYaw - Config::TURN_TOLERANCE_DEG);
    ASSERT_FALSE(Drive.isTurnDone());
    const int16_t nudgeForward = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(nudgeForward > 0);
    ASSERT_EQ(nudgeForward, (int16_t) 12);

    // Overshoot to 5deg PAST the target — well outside tolerance, and inside what would
    // naturally be CREEP's zone in isolation. The monotonic latch keeps the phase at SETTLE
    // (never regresses), so the command still comes from turnSettleNudge, not the fixed CREEP
    // magnitude: clamp(TURN_SETTLE_KP * -5, +/-TURN_CREEP_PWM) == clamp(-20, +/-38) == -20.
    turn_harness::tickAtYaw(targetYaw + 5.0f);
    ASSERT_FALSE(Drive.isTurnDone());    // |err| (5) > TURN_TOLERANCE_DEG (3) -> not done.
    const int16_t nudgeOvershoot = MiniR4.DriveDC.lastMoveR;
    ASSERT_TRUE(nudgeOvershoot < 0);
    ASSERT_EQ(nudgeOvershoot, (int16_t) -20);
    ASSERT_TRUE((nudgeForward > 0) != (nudgeOvershoot > 0));    // the nudge REVERSED sign.
}

// ===========================================================================================
// TC16 — a residual that never closes (5deg, well outside tolerance, parked forever) times out
// at Config::TURN_TIMEOUT_MS: brakes, isTurnDone() true, hasFault() FALSE, and the QUEUE is NOT
// cleared (a second queued turn dispatches and starts issuing real Move() traffic afterward).
// ===========================================================================================
static void test_TC16_non_convergence_times_out_without_fault_or_queue_clear()
{
    TEST_SCOPE("TC16: a residual that never closes times out — no fault, queue stays intact");
    turn_harness::resetWorld();

    const float   startYaw = 0.0f;
    const int16_t deg      = 20;
    const float   stuckYaw = startYaw + (float) deg - 5.0f;    // 5deg residual, never closes.

    // Deliberately only ONE command queued for the timeout observation below — enqueueing a
    // second here would race the timeout itself (the moment it fires, stage 3 of that SAME
    // Drive.update() call would immediately dispatch the second command and flip _turnDone back
    // to false before this test ever observes it true).
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
    // profile Move() traffic — proving the FIRST turn's non-fault timeout did NOT clear the queue.
    const uint32_t movesBeforeSecond = MiniR4.DriveDC.moveCallCount;
    turn_harness::enqueueTurnDeg(deg);
    for (int i = 0; i < turn_harness::kMaxTicks; ++i)
    {
        turn_harness::tickAtYaw(stuckYaw);
    }
    ASSERT_TRUE(MiniR4.DriveDC.moveCallCount > movesBeforeSecond);
}

// ===========================================================================================
// TC17 — a sub-tolerance turn (2deg, under TURN_TOLERANCE_DEG of 3deg) short-circuits at
// dispatch: `_execTurnDeg` marks it done immediately (existing `_exec*` behavior, unchanged by
// ADR-078), and the profile FSM never runs at all — no Move() call is ever issued.
// ===========================================================================================
static void test_TC17_sub_tolerance_turn_short_circuits_with_no_move()
{
    TEST_SCOPE("TC17: sub-tolerance turn (2deg, tol 3deg) short-circuits done, no Move");
    turn_harness::resetWorld();

    const uint32_t movesBefore = MiniR4.DriveDC.moveCallCount;

    turn_harness::enqueueTurnDeg(/*deg*/ 2);
    turn_harness::tickAtYaw(0.0f);    // the DISPATCH tick itself resolves the short-circuit,
                                      // synchronously, inside _execTurnDeg's stage-3 pop.

    ASSERT_TRUE(Drive.isTurnDone());
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_EQ(MiniR4.DriveDC.moveCallCount, movesBefore);    // no Move() call was ever issued.
}

// ===========================================================================================
// TC18 — a pivot turn (mode 1 = PIVOT_LEFT / mode 2 = PIVOT_RIGHT) commands ONLY the free wheel
// through every phase (the anchored wheel's dispatched PWM is EXACTLY 0 in CREEP, per
// `turnWheelPattern`'s pivot mode), and still converges to a clean settle.
// ===========================================================================================
static void test_TC18_pivot_mode_turn_converges_with_only_driven_wheel_nonzero_in_creep()
{
    TEST_SCOPE("TC18: pivot-mode turn converges; only the driven wheel is nonzero in CREEP");

    const float   startYaw  = 0.0f;
    const int16_t deg       = 20;    // inside TURN_CREEP_WINDOW_DEG -> CREEP once broken away.
    const float   targetYaw = startYaw + (float) deg;

    // PIVOT_LEFT (mode 1): left wheel anchored at 0, right wheel carries the command.
    turn_harness::resetWorld();
    turn_harness::enqueueTurnDeg(deg, /*mode PIVOT_LEFT*/ 1);
    turn_harness::tickAtYaw(startYaw);    // dispatch.
    turn_harness::tickAtYaw(startYaw);    // breakaway floor (at rest).
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, Config::TURN_BREAKAWAY_PWM);

    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG);    // breakaway latches; CREEP.
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, Config::TURN_CREEP_PWM);

    ASSERT_TRUE(turn_harness::tickUntilTurnDone(targetYaw, turn_harness::kMaxTicks));
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.hasFault());

    // Mirror — PIVOT_RIGHT (mode 2): right wheel anchored at 0, left wheel carries the command.
    turn_harness::resetWorld();
    turn_harness::enqueueTurnDeg(deg, /*mode PIVOT_RIGHT*/ 2);
    turn_harness::tickAtYaw(startYaw);
    turn_harness::tickAtYaw(startYaw);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) -Config::TURN_BREAKAWAY_PWM);

    turn_harness::tickAtYaw(startYaw + Config::TURN_BREAKAWAY_MOTION_DEG);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveR, 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveL, (int16_t) -Config::TURN_CREEP_PWM);

    ASSERT_TRUE(turn_harness::tickUntilTurnDone(targetYaw, turn_harness::kMaxTicks));
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// Entry.
// ===========================================================================================

int main()
{
    std::printf("INFO: Turn profile controller — real Drive.cpp FSM integration (ADR-078, T4, "
                "TC9-TC18)\n");

    test_TC9_small_turn_breaks_away_from_rest_not_creep();
    test_TC10_large_turn_post_breakaway_commands_cruise();
    test_TC11_power_override_sets_cruise_magnitude();
    test_TC12_creep_window_post_breakaway_commands_creep_pwm();
    test_TC13_settle_requires_consecutive_in_band_samples();
    test_TC14_band_skip_overshoot_in_creep_reverses_command();
    test_TC15_overshoot_after_settle_latched_reverses_nudge();
    test_TC16_non_convergence_times_out_without_fault_or_queue_clear();
    test_TC17_sub_tolerance_turn_short_circuits_with_no_move();
    test_TC18_pivot_mode_turn_converges_with_only_driven_wheel_nonzero_in_creep();

    TEST_REPORT();
}
