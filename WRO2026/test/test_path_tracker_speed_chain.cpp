/**
 * @file test_path_tracker_speed_chain.cpp
 * @brief Host-link tests for the per-edge speed axis (issues #97 + #99, ADR-064),
 *        driving the REAL `PathTracker` + `Drive` + `Localization` FSMs through the
 *        shared cooperative-tick harness over the dedicated `routes_speed_chain`
 *        shadow field.
 *
 * @details
 * The single-sourced `routes_fixture` (field_fixture.py) does not author the
 * per-edge `speed`/`mode`/`stop_at_line` COMBINATIONS these cases need — a
 * same-tier SYNC<->GYRO chain, a tier boundary INSIDE a collinear blind run, a
 * seek edge carrying its own speed, and a FAST line feeding a SLOW blind
 * pass-through. Rather than perturb that shared fixture (whose every re-weight
 * fans out across ~20 FSM targets), these tests link a small HAND-AUTHORED shadow
 * routes table (`routes_speed_chain.{h,cpp}`) force-included ahead of the sketch
 * `routes.h` — the same shadow trick `routes_reset_passthrough_forced` uses for a
 * topology the pipeline cannot emit. Nodes are resolved BY NAME (issue #91).
 *
 * The observables are the SAME lower-MCU facts the ADR-059 blind-chain test reads
 * off the real `Drive.cpp` through the `MiniR4.DriveDC` mock:
 *   - **`lastBrake`** at a hop boundary: a chained mid-hop coast-releases
 *     (`brake(false)`); a chain that ENDED at that hop brakes for real
 *     (`brake(true)`). This is the observable for "did the run chain THROUGH this
 *     hop or stop AT it" (see `test_path_tracker_blind_chain.cpp`'s docblock).
 *   - **`lastMoveSyncL`** for the SYNC cruise magnitude a hop launches / seeks at.
 *
 * TCs (2026-07-14 mode/speed plan, T14host):
 *   - TC10a — a SPEED-tier boundary breaks the blind chain: `Q1 ->(NORMAL) Q2
 *     ->(SLOW) Q3`; seated at Q1 with goal Q3, the walk admits `Q1->Q2` then hits
 *     the tier boundary at `Q2->Q3` (SLOW != NORMAL) and ends the run, so `Q1->Q2`
 *     brakes at Q2 (no chain).
 *   - TC10b — a SAME-tier SYNC<->GYRO run STILL chains: `Q0 ->(NORMAL,SYNC) Q1
 *     ->(NORMAL,GYRO) Q2`; seated at Q0 with goal Q2, both hops share the NORMAL
 *     tier (the mode mix is NOT a break), so `Q0->Q1` coast-releases at Q1
 *     (chained). The a/b contrast isolates the gate as SPEED-only.
 *   - TC10c — a seek edge cruises at its resolved tier: `Sk0 ->(SLOW,
 *     stop_at_line=on) SkL`; the seek's SEEK-phase cruise is dispatched at
 *     `presetToPwm(SLOW)` (60), not the former hardcoded `SPEED_NORMAL` (70).
 *   - TC14 — pass-through entry seed clamp: a FAST line (`Ln0->Ln1`) feeding a
 *     collinear SLOW blind (`Ln1->Lb2`) enters the blind hop at <= the SLOW cruise
 *     (60): the captured inbound line cruise is clamped down to the hop's own
 *     resolved cruise before launch. `Ln0->Ln1` also carries `LineFollowMode::LEFT`
 *     (issue #156), so this case additionally pins that the FAST-line arm threads the
 *     resolved lateral offset into the follower.
 *   - TC16 — lateral offset threading (issue #156): the `LineFollowMode::LEFT`
 *     `Ln0->Ln1` LINE edge arms the follower with the composed forward setpoint
 *     `+Config::LINE_FOLLOW_OFFSET_ERROR` (a FORWARD hop, so no reverse-sign flip).
 *   - TC17 — reverse-hop offset negation (issue #156, AC3): the SAME `LineFollowMode::LEFT`
 *     `Ln0->Ln1` LINE edge taken in REVERSE (a builder `setRouteDirection(REAR)` route
 *     override resolves `decision.reverse == true` on the 2-board default) arms the follower
 *     with the NEGATED setpoint `-Config::LINE_FOLLOW_OFFSET_ERROR`. This pins PathTracker's
 *     reverse-sign composition (`sign = decision.reverse ? -1 : +1`) on the
 *     LINE_FOLLOW_INTERSECTION dispatch THROUGH the real FSM — TC16 covers only the forward
 *     branch. The identical composition on the PROJECTION_FOLLOW dispatch is not reached by
 *     this plain-LINE hop; it is covered only by the local arithmetic re-derivation in TC3
 *     (test_line_follower_offset.cpp), which does not run PathTracker.
 *
 * Config overrides use `target_compile_definitions` (CMakeLists.txt), never an
 * in-file `#define` (`PathTracker.cpp` is a separate TU): `HOP_SETTLE_MS=0` so the
 * dispatch tick sequence is unperturbed by the ADR-061 settle dwell.
 */

#include "test_path_tracker_harness.h"

#include <cstdio>

namespace
{
/// @brief Float near-equality (TestAssert.h has no float assertion; avoids the `abs`/`fabs`
///        macro clash from the mock `Arduino.h`).
bool nearf(float a, float b, float eps = 1.0e-4f)
{
    const float d = a - b;
    return d < eps && d > -eps;
}

/// @brief Tick the real FSM until a Drive move is IN FLIGHT (a blind hop / seek
///        dispatched), bounded so a never-dispatching regression fails as an
///        assert rather than hanging. Parks the mock IMU yaw at the collinear
///        (heading-0) run facing every tick so a GYRO dead-reckon's yaw read stays
///        deterministic (harmless for SYNC hops).
void tickUntilMoveInFlight()
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Drive.isIdle(); ++i)
    {
        mmL.mockYawDeg = 0.0;    // collinear run at heading 0.
        cooperativeTick();
    }
    ASSERT_FALSE(Drive.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

/// @brief Drive whichever blind hop is in flight to completion via a generous
///        encoder overrun + done-bit (covers both the profiled SYNC/GYRO regime,
///        whose `_updateProfile` retires on the encoder gate, and any flat
///        degrade), stopping the INSTANT the discrete node advances so a CHAINED
///        run's next hop cannot also dispatch+retire and clobber `lastBrake`.
///
/// @return The node the hop landed on (the caller asserts it + `lastBrake`).
uint8_t driveBlindHopUntilNodeAdvances()
{
    const uint8_t before = Localization.getCurrentNode();
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && Localization.getCurrentNode() == before && !PathTracker.hasFault();
         ++i)
    {
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        mmL.mockYawDeg = 0.0;    // hold the GYRO hop's heading target.
        cooperativeTick();
    }
    ASSERT_FALSE(PathTracker.hasFault());
    return Localization.getCurrentNode();
}
}    // namespace

// ===========================================================================
// TC10a — a SPEED-tier boundary ends the blind chain (the run brakes at it).
// ===========================================================================

void test_tc10a_tier_boundary_breaks_chain()
{
    TEST_SCOPE("TC10a: Q1->(NORMAL) Q2 ->(SLOW) Q3 -- the tier boundary ends the run at Q2");

    const uint8_t q1 = nodeByName("Q1");
    const uint8_t q2 = nodeByName("Q2");
    const uint8_t q3 = nodeByName("Q3");
    ASSERT_TRUE(q1 != FieldMap::NODE_NONE && q2 != FieldMap::NODE_NONE && q3 != FieldMap::NODE_NONE);

    // Pin the run + the tier data so a shadow-table edit fails LOUDLY rather than
    // silently binding this probe on a different reason.
    ASSERT_EQ(routedFirstHop(q1, q3), q2);
    ASSERT_EQ(routedFirstHop(q2, q3), q3);
    ASSERT_TRUE(FieldMap::getEdge(q1, q2).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdge(q2, q3).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdge(q1, q2).speed_override == Routes::SpeedTier::NORMAL);
    ASSERT_TRUE(FieldMap::getEdge(q2, q3).speed_override == Routes::SpeedTier::SLOW);

    resetWorld();
    Serial.clearCapture();
    seatAt("Q1", 0);    // Q1->Q2 departs heading 0 == the seated facing (no pre-turn).
    PathTracker.goTo(q3).async();
    tickUntilMoveInFlight();

    // No chain forms: the walk-ahead breaks at the Q2->Q3 tier boundary (SLOW != NORMAL),
    // leaving a 1-hop run (== no chain), so Q1->Q2 dispatches on the ordinary per-hop path
    // and BRAKES on arrival at Q2. (Q1->Q2 is a GYRO hop, so its start speed lands in the
    // MoveGyro channel, not MoveSync; the load-bearing observable here is the brake.)
    ASSERT_EQ(driveBlindHopUntilNodeAdvances(), q2);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // chain ended at the tier boundary -- braked stop at Q2.
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC10b — a SAME-tier SYNC<->GYRO run STILL chains (the mode mix is not a break).
// ===========================================================================

void test_tc10b_same_tier_mode_mix_chains()
{
    TEST_SCOPE("TC10b: Q0->(NORMAL,SYNC) Q1 ->(NORMAL,GYRO) Q2 -- same tier still chains");

    const uint8_t q0 = nodeByName("Q0");
    const uint8_t q1 = nodeByName("Q1");
    const uint8_t q2 = nodeByName("Q2");
    ASSERT_TRUE(q0 != FieldMap::NODE_NONE && q1 != FieldMap::NODE_NONE && q2 != FieldMap::NODE_NONE);

    // Pin the run + the tier/mode data: both hops NORMAL (so no tier break), Q0->Q1
    // SYNC (pipeline default) and Q1->Q2 GYRO (an explicit per-edge BlindMode).
    ASSERT_EQ(routedFirstHop(q0, q2), q1);
    ASSERT_EQ(routedFirstHop(q1, q2), q2);
    ASSERT_TRUE(FieldMap::getEdge(q0, q1).speed_override == Routes::SpeedTier::NORMAL);
    ASSERT_TRUE(FieldMap::getEdge(q1, q2).speed_override == Routes::SpeedTier::NORMAL);
    ASSERT_TRUE(FieldMap::getEdge(q0, q1).blind_mode_override == Routes::BlindMode::DEFAULT);
    ASSERT_TRUE(FieldMap::getEdge(q1, q2).blind_mode_override == Routes::BlindMode::GYRO_TSHAPE);

    resetWorld();
    Serial.clearCapture();
    seatAt("Q0", 0);    // Q0->Q1 departs heading 0 (no pre-turn).
    PathTracker.goTo(q2).async();
    tickUntilMoveInFlight();

    // The walk-ahead admits Q1->Q2 (same NORMAL tier; a SYNC<->GYRO mode mix is NOT a
    // break), so a 2-hop chain forms and Q0->Q1 is a chained MID-hop: it coast-releases
    // at Q1 (`brake(false)`), NOT a real braked stop.
    const uint32_t brakeBefore = MiniR4.DriveDC.brakeCallCount;
    ASSERT_EQ(driveBlindHopUntilNodeAdvances(), q1);
    // A brake WAS emitted at the Q1 boundary (rules out a never-braked / hung path -- the
    // same coast-release idiom as test_path_tracker_blind_chain.cpp's TC4)...
    ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeBefore);
    ASSERT_FALSE(MiniR4.DriveDC.lastBrake);    // ...and it was a coast (brake(false)), not a braked stop.
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC10c — a seek edge cruises at its resolved speed tier (not the old SPEED_NORMAL).
// ===========================================================================

void test_tc10c_seek_cruises_at_resolved_tier()
{
    TEST_SCOPE("TC10c: Sk0 ->(SLOW, stop_at_line=on) SkL -- the seek cruises at SPEED_SLOW, not NORMAL");

    const uint8_t sk0 = nodeByName("Sk0");
    const uint8_t skL = nodeByName("SkL");
    ASSERT_TRUE(sk0 != FieldMap::NODE_NONE && skL != FieldMap::NODE_NONE);

    // Pin the seek edge + its SLOW speed override.
    ASSERT_EQ(routedFirstHop(sk0, skL), skL);
    const Routes::EdgeInfo& seekEdge = FieldMap::getEdge(sk0, skL);
    ASSERT_TRUE(seekEdge.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(seekEdge.stop_at_line_override == Routes::StopAtLineMode::ON);
    ASSERT_TRUE(seekEdge.speed_override == Routes::SpeedTier::SLOW);

    resetWorld();
    Serial.clearCapture();
    seatAt("Sk0", 0);    // Sk0->SkL departs heading 0 (no pre-turn).
    PathTracker.goTo(skL).async();

    // Dispatch the seek, then tick (WITHOUT advancing the encoder, so it stays in the
    // SEEK cruise) until the SEEK phase emits its first MoveSync. The SYNC seek cruise
    // is `MiniR4.DriveDC.MoveSync(deadbandSnap(power), ...)`, so `lastMoveSyncL` is the
    // dispatched cruise magnitude.
    for (int i = 0;
         i < path_harness::kMaxHandshakeTicks && MiniR4.DriveDC.moveSyncCallCount == 0
         && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg = 0.0;
        cooperativeTick();
    }
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, (uint32_t) 0);
    ASSERT_FALSE(PathTracker.hasFault());

    // THE TC10c ASSERT: the seek cruises at the per-edge SLOW tier (60), NOT the former
    // hardcoded SPEED_NORMAL (70). `deadbandSnap(60)` is above the PWM floor, so it
    // passes through unchanged.
    const int16_t expectSlowPwm =
        presetToPwm(speedTierToPreset(Routes::SpeedTier::SLOW));
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, expectSlowPwm);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveSyncL != presetToPwm(speedTierToPreset(Routes::SpeedTier::NORMAL)));

    // Complete the seek so the FSM re-anchors on the crossing and returns to IDLE. This
    // leaves NO in-flight Drive seek state to leak into the next case: `resetWorld()`
    // re-begins the singletons but does NOT clear Drive's `_stopState`, so a seek left
    // mid-SEEK would keep Drive non-idle and derail the next test's first dispatch.
    expectStopAtLine(seekEdge.distance_cm, true);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC14 — pass-through entry seed clamp: a FAST line into a SLOW blind enters <= 60.
// ===========================================================================

void test_tc14_passthrough_entry_seed_clamped_to_resolved_cruise()
{
    TEST_SCOPE("TC14: FAST line Ln0->Ln1 -> SLOW blind Ln1->Lb2 -- blind entry clamped to <= SLOW cruise");

    const uint8_t ln0 = nodeByName("Ln0");
    const uint8_t ln1 = nodeByName("Ln1");
    const uint8_t lb2 = nodeByName("Lb2");
    ASSERT_TRUE(ln0 != FieldMap::NODE_NONE && ln1 != FieldMap::NODE_NONE && lb2 != FieldMap::NODE_NONE);

    // Pin the run: a FAST LINE junction feeding a collinear SLOW BLIND hop.
    ASSERT_EQ(routedFirstHop(ln0, lb2), ln1);
    ASSERT_EQ(routedFirstHop(ln1, lb2), lb2);
    ASSERT_TRUE(FieldMap::getEdge(ln0, ln1).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(ln0, ln1).speed_override == Routes::SpeedTier::FAST);
    // issue #156: Ln0->Ln1 is authored LEFT so the same run exercises the biased-leg thread.
    ASSERT_TRUE(FieldMap::getEdge(ln0, ln1).line_follow_mode_override
                == Routes::LineFollowMode::LEFT);
    ASSERT_TRUE(FieldMap::getEdge(ln1, lb2).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdge(ln1, lb2).speed_override == Routes::SpeedTier::SLOW);

    resetWorld();
    Serial.clearCapture();
    EventTrace.mockReset();
    seatAt("Ln0", 0);    // Ln0->Ln1 departs heading 0 (no pre-turn).
    PathTracker.goTo(lb2).async();

    // Drive to LINE_FOLLOW for Ln0->Ln1 (the shared line-follow entry precondition).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == ln0)
        {
            break;
        }
    }
    ASSERT_EQ(Localization.getCurrentNode(), ln0);
    ASSERT_TRUE(LineFollower.isEnabled());
    // The FAST line edge armed the follower at the FAST preset.
    ASSERT_TRUE(LineFollower.lastPreset == Config::SpeedPreset::FAST);
    // issue #156: the LEFT bias on Ln0->Ln1 threaded the composed forward setpoint into the
    // follower arm. Ln0->Ln1 is a FORWARD hop (heading 0 == the seated facing), so the sign is
    // +1 and the resolved offset is +Config::LINE_FOLLOW_OFFSET_ERROR (referenced, never a literal).
    ASSERT_TRUE(nearf(LineFollower.lastLineFollowOffset, Config::LINE_FOLLOW_OFFSET_ERROR));

    // Cross Ln1: the intersection-gated pass-through peek fires (Ln1's onward flank
    // Ln1->Lb2 is collinear BLIND and Ln1 is not the goal), keeping the follower live
    // into the BLIND hop -- the LINE_OVERSHOOT re-arm is where `brakeOnGate` is set false.
    const int armsBefore = LineFollower.setEnabledCount;
    Intersection.mockSetAtIntersection(true);
    cooperativeTick();
    ASSERT_GT(LineFollower.setEnabledCount, armsBefore);
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), ln1);
    ASSERT_FALSE(PathTracker.hasFault());

    // The BLIND dispatch for Ln1->Lb2 consumes _passThrough: it seeds the captured
    // inbound line cruise but CLAMPS it to this hop's own resolved cruise (SLOW = 60)
    // before launch. So the first blind MoveSync enters at <= 60, never the inbound
    // line speed. (In this host mock the LineFollower builder records the preset only
    // in `lastPreset`, so `getPreset()` returns the mock default NORMAL/70 for the
    // capture -- the clamp still fires, 70 -> 60; the FAST line itself is pinned above
    // via `lastPreset == FAST`. The exact FAST(90)->60 magnitude is covered by the sim
    // parity + CTPS surfaces.)
    // Tick until the blind hop emits its first MoveSync (the traveled==0 entry). The
    // LINE follow uses the LineFollower mock's setSpeed (never MiniR4.DriveDC.MoveSync),
    // so the FIRST MoveSync is unambiguously this blind hop's entry -- do NOT advance the
    // encoder, so the hop stays at traveled==0 and `lastMoveSyncL` is the seed magnitude.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && MiniR4.DriveDC.moveSyncCallCount == 0 && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, (uint32_t) 0);

    const int16_t slowCruise = presetToPwm(speedTierToPreset(Routes::SpeedTier::SLOW));
    // THE TC14 ASSERT: entry seed clamped to the SLOW blind cruise (<= 60), NOT the
    // captured inbound line cruise (which exceeds 60). Falsifiable: without the clamp
    // the blind hop would launch at the captured line cruise (> 60). (TestAssert.h has
    // no ASSERT_LE, so the <= is written as `slowCruise >= entry`.)
    ASSERT_GE(slowCruise, MiniR4.DriveDC.lastMoveSyncL);
    ASSERT_GT(MiniR4.DriveDC.lastMoveSyncL, (int16_t) 0);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC16 — the LEFT LINE edge threads the resolved lateral offset into the follower.
// ===========================================================================

void test_tc16_left_bias_threads_offset()
{
    TEST_SCOPE("TC16: LineFollowMode::LEFT Ln0->Ln1 arms the follower with +OFFSET (issue #156)");

    const uint8_t ln0 = nodeByName("Ln0");
    const uint8_t ln1 = nodeByName("Ln1");
    ASSERT_TRUE(ln0 != FieldMap::NODE_NONE && ln1 != FieldMap::NODE_NONE);

    // Pin the biased cell + the direct route so a shadow-table edit fails LOUDLY here rather
    // than silently binding this probe on a different (unbiased) edge.
    ASSERT_EQ(routedFirstHop(ln0, ln1), ln1);
    ASSERT_TRUE(FieldMap::getEdge(ln0, ln1).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(ln0, ln1).line_follow_mode_override
                == Routes::LineFollowMode::LEFT);

    resetWorld();
    Serial.clearCapture();
    seatAt("Ln0", 0);    // Ln0->Ln1 departs heading 0 (no pre-turn), so it is a FORWARD hop.
    PathTracker.goTo(ln1).async();

    // Tick until the LINE follow arms for the Ln0->Ln1 hop (the non-overshoot P+D entry).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == ln0)
        {
            break;
        }
    }
    ASSERT_EQ(Localization.getCurrentNode(), ln0);
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(PathTracker.hasFault());

    // THE TC16 ASSERT: a FORWARD LEFT hop resolves to sign(+1) * lineFollowModeToSetpoint(LEFT) =
    // +Config::LINE_FOLLOW_OFFSET_ERROR (referenced, never a literal, per the retune-fanout rule).
    // Falsifiable: a MIDDLE/absent edge would thread 0.0f; a reverse hop would negate it.
    ASSERT_TRUE(nearf(LineFollower.lastLineFollowOffset, Config::LINE_FOLLOW_OFFSET_ERROR));
    ASSERT_TRUE(LineFollower.lastLineFollowOffset > 0.0f);
}

// ===========================================================================
// TC17 — a REVERSE (rear-board) LEFT LINE hop threads the NEGATED offset (issue #156, AC3).
// ===========================================================================

void test_tc17_reverse_left_bias_negates_offset()
{
    TEST_SCOPE("TC17: a REAR-board LineFollowMode::LEFT Ln0->Ln1 hop threads -OFFSET "
               "(issue #156, AC3 reverse-sign composition)");

    const uint8_t ln0 = nodeByName("Ln0");
    const uint8_t ln1 = nodeByName("Ln1");
    ASSERT_TRUE(ln0 != FieldMap::NODE_NONE && ln1 != FieldMap::NODE_NONE);

    // The rear board is what makes a REAR LINE hop servable — `reverseUnavailable()` faults a REAR
    // LINE hop on a 1-board chassis. The speed_chain target compiles at the ConfigLine.h default
    // `LINE_BOARD_COUNT == 2`, so a forced-REAR LINE hop resolves REAR instead of REVERSE_UNAVAILABLE
    // (same 2-board precondition test_path_tracker_heading_reset.cpp's TC9 pins).
    ASSERT_EQ(Config::LINE_BOARD_COUNT, (uint8_t) 2);

    // Pin the biased cell + the direct route so a shadow-table edit fails LOUDLY here rather than
    // silently binding this probe on a different (unbiased / non-LINE) edge. This is the SAME LEFT
    // cell TC16 drives FORWARD — here it is taken in REVERSE, so the resolved forward setpoint
    // (+OFFSET) is negated to -OFFSET by PathTracker's dispatch-site sign composition.
    ASSERT_EQ(routedFirstHop(ln0, ln1), ln1);
    ASSERT_TRUE(FieldMap::getEdge(ln0, ln1).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(ln0, ln1).line_follow_mode_override
                == Routes::LineFollowMode::LEFT);

    // Drive the REAL PathTracker/Drive/Localization FSM to the LINE-follow entry with a builder
    // route override forcing REAR: `goTo(ln1).setRouteDirection(REAR)` -> resolveDirection Step 4
    // -> Direction::REAR -> `decision.reverse == true`. The shared harness helper seats at Ln0,
    // parks the mock IMU at the body-corrected (heading + 180) REAR target every tick so the
    // pre-turn settles, and returns with the follower armed (non-overshoot) for the reverse hop.
    driveToLineFollowEntry("Ln0", "Ln1", Direction::REAR);
    ASSERT_EQ(Localization.getCurrentNode(), ln0);
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(PathTracker.hasFault());    // no REVERSE_UNAVAILABLE on the 2-board default.

    // The hop is GENUINELY reverse: the LINE_FOLLOW_INTERSECTION dispatch threaded
    // `.setReverse(decision.reverse)` == true. This guards against a regression where REAR
    // resolution silently degrades to a forward hop (which would thread +OFFSET and mask the
    // negation the assert below checks).
    ASSERT_TRUE(LineFollower.lastReverse);

    // THE TC17 ASSERT: a REVERSE LEFT hop composes sign(-1) * lineFollowModeToSetpoint(LEFT) =
    // -Config::LINE_FOLLOW_OFFSET_ERROR (referenced, never a literal, per the retune-fanout rule),
    // so the bias stays left-of-TRAVEL. Falsifiable: dropping the `sign = decision.reverse ? -1 :
    // +1` factor at the LINE_FOLLOW_INTERSECTION dispatch (the site this plain-LINE hop exercises)
    // would thread the un-negated forward +OFFSET here and RED this assert. (The identical
    // PROJECTION_FOLLOW reverse-sign is covered only by TC3's local re-derivation.)
    ASSERT_TRUE(nearf(LineFollower.lastLineFollowOffset, -Config::LINE_FOLLOW_OFFSET_ERROR));
    ASSERT_TRUE(LineFollower.lastLineFollowOffset < 0.0f);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: per-edge speed axis (issues #97 + #99, ADR-064) -- real "
                "PathTracker + Drive + Localization over routes_speed_chain\n");

    test_tc10a_tier_boundary_breaks_chain();
    test_tc10b_same_tier_mode_mix_chains();
    test_tc10c_seek_cruises_at_resolved_tier();
    test_tc14_passthrough_entry_seed_clamped_to_resolved_cruise();
    test_tc16_left_bias_threads_offset();
    test_tc17_reverse_left_bias_negates_offset();

    TEST_REPORT();
}
