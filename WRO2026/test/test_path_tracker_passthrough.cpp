/**
 * @file test_path_tracker_passthrough.cpp
 * @brief Host-link test for the `PATH_PASS_THROUGH_ENABLED` behavioral toggle,
 *        driving the REAL `PathTracker` + `Drive` FSMs through the shared
 *        cooperative-tick harness (issue #106, T5a).
 *
 * @details
 * Rewritten from the original mirror-slice test. The old file directly executed
 * an inline copy of the LineFollower arming sequence (`wrapDelta` /
 * `computePassThrough` / `armLineFollowerForOvershoot`) against a hand-mocked
 * FieldMap fixture — a `// MIRROR:` of `PathTracker.cpp`. That is gone: this file
 * now drives the production `PathTracker` FSM over the REAL fixture graph and
 * asserts the pass-through arm cascade the FSM itself issues.
 *
 * THE HOP (`fixtures/routes_fixture.h`): the collinear `line-line-line` triple
 * `Inter2 -> Inter3 -> Inter4` (all LINE edges at heading 0, all LINE nodes). A
 * `goTo(Inter4)` from `Inter2` walks `NEXT_HOP` through the mid junction
 * `Inter3`. The bypass edge `Inter2 -> Inter4` is `EdgeType::NONE`, so Dijkstra
 * MUST traverse `Inter3` — pinned with the routing guard + `assertNoBypass`.
 *
 * Built TWICE by CMake — once with `-DPATH_PASS_THROUGH_ENABLED_DEFAULT=true`
 * (`test_path_tracker_passthrough_on`) and once with `=false`
 * (`test_path_tracker_passthrough_off`). Both binaries run the SAME hop; the
 * assertions branch on the COMPILED `Config::PATH_PASS_THROUGH_ENABLED`
 * constexpr (resolved from that flag in `ConfigNav.h`):
 *
 *   - TC-PT1 (flag ON):  pass-through FIRES. At the mid junction `Inter3` the
 *     real FSM line-follows past WITHOUT braking — the LINE_OVERSHOOT arm carries
 *     `brakeOnGate == false` (and `tShapeRampDown == false`). `Localization`
 *     advances `Inter2 -> Inter3 -> Inter4`.
 *
 *   - TC-PT2 (flag OFF): pass-through SUPPRESSED. At `Inter3` the FSM brakes +
 *     re-plans (stop-and-go) — the LINE_OVERSHOOT arm carries
 *     `brakeOnGate == true`. Proves the compile flag changes runtime behaviour.
 *
 * A LINE hop has no lower-MCU `MiniR4.DriveDC` move (the LineFollower drives
 * `setSpeed`, absorbed by the path mock), so the representative observable is the
 * LineFollower arm cascade (`LineFollower.lastBrakeOnGate` etc.), asserted
 * directly on the path mock. The lower-MCU `DriveDC` gyro/seek asserts live in the
 * projection / stop_at_line / bidirectional tests, not here.
 */

#include "test_path_tracker_harness.h"

#include <cmath>

namespace
{
// ---------------------------------------------------------------------------
// Drive the real PathTracker from a clean world up to the LINE_FOLLOW state of
// the FIRST hop (`Inter2 -> Inter3`), with the LineFollower armed for the line
// follow — exactly the precondition `expectLineHop()` assumes (harness caveat
// S2). Returns with the FSM in LINE_FOLLOW, the follower freshly enabled (the
// `_dispatchEdgeAfterTurn` LINE -> line arm: overshootMode=false, no brake-on-gate
// yet), and no Drive move in flight (a LINE hop issues none).
//
// Sequence the real FSM runs to get here (PathTracker.cpp):
//   IDLE -> DEQUEUE (pop goal, offset~0 return-gate -> PLAN_NEXT_HOP)
//        -> PLAN_NEXT_HOP (Inter2->Inter3 LINE, heading 0 == current world 0, so
//           `_dispatchTurnIfNeeded` skips the pre-turn) -> _dispatchEdgeAfterTurn
//        -> LINE_FOLLOW (LineFollower armed: setEnabled(true).setPreset(...)
//           .setReverse(false).setOvershootMode(false)).
// No pre-edge turn fires because Inter2->Inter3 departs at heading 0 and
// `seatAt("Inter2")` bound the world frame at app-yaw 0 (harness caveat S3 does
// not bite: the departure heading IS the default heading).
// ---------------------------------------------------------------------------
void driveToFirstLineFollow()
{
    resetWorld();
    seatAt("Inter2");    // bind world frame @ app-yaw 0 + declare Localization at Inter2

    // Enqueue the multi-hop goal; the FINAL node only — PathTracker walks NEXT_HOP
    // hop-by-hop (Inter2 -> Inter3 -> Inter4). `.async()` so the harness drives the
    // cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(nodeByName("Inter4")).async();

    // Tick until the FSM dispatches the first LINE hop and arms the follower for the
    // line follow (the LINE -> line arm: enabled + overshootMode=false). That arm is
    // the LINE_FOLLOW entry; the follower is enabled and no Drive move is pending, so
    // `expectLineHop()` can latch the junction next.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode)
        {
            return;    // LINE_FOLLOW reached, follower armed for the line follow
        }
    }
    ASSERT_TRUE(false);    // budget exhausted: the FSM never reached LINE_FOLLOW
}
}    // namespace

// ===========================================================================
// Routing guard — pin the hop so a future fixture re-weight fails LOUDLY rather
// than silently re-routing the pass-through under test (Fable amendment 2).
// ===========================================================================

void test_routing_is_forced_through_mid_junction()
{
    TEST_SCOPE("routing: goTo(Inter4) from Inter2 MUST traverse Inter3 (no bypass)");

    const uint8_t inter2 = nodeByName("Inter2");
    const uint8_t inter3 = nodeByName("Inter3");
    const uint8_t inter4 = nodeByName("Inter4");

    // The fixture must resolve these three names (the stable-name contract).
    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter4 != FieldMap::NODE_NONE);

    // Dijkstra's first hop Inter2 -> Inter4 is Inter3 ...
    ASSERT_EQ(routedFirstHop(inter2, inter4), inter3);
    // ... and there is NO direct Inter2 -> Inter4 edge, so the only route is THROUGH
    // Inter3 — the pass-through triple is genuinely traversed, not bypassed.
    assertNoBypass(inter2, inter4);

    // Both flanks of the triple are LINE edges (the line-line-line geometry the
    // pass-through collapses).
    ASSERT_TRUE(FieldMap::getEdge(inter2, inter3).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(inter3, inter4).type == Routes::EdgeType::LINE);
}

// ===========================================================================
// Announce the compiled build mode for ctest log clarity.
// ===========================================================================

void test_announce_build_mode()
{
    TEST_SCOPE("build-mode announcement");
    if (Config::PATH_PASS_THROUGH_ENABLED)
    {
        std::printf("INFO: PATH_PASS_THROUGH_ENABLED == true (pass-through build)\n");
    }
    else
    {
        std::printf("INFO: PATH_PASS_THROUGH_ENABLED == false (legacy stop-and-go build)\n");
    }
}

// ===========================================================================
// TC-PT1 / TC-PT2 — the real FSM's mid-junction LINE_OVERSHOOT arm tracks the
// compile flag, and Localization advances across the full triple.
// ===========================================================================

void test_passthrough_arm_and_advance_diverge_by_flag()
{
    if (Config::PATH_PASS_THROUGH_ENABLED)
    {
        TEST_SCOPE("TC-PT1: flag ON — pass-through FIRES at Inter3 (no brake) + advances");
    }
    else
    {
        TEST_SCOPE("TC-PT2: flag OFF — pass-through SUPPRESSED at Inter3 (brakes) + advances");
    }

    const uint8_t inter2 = nodeByName("Inter2");
    const uint8_t inter3 = nodeByName("Inter3");
    const uint8_t inter4 = nodeByName("Inter4");

    driveToFirstLineFollow();

    // Precondition: the FSM is in LINE_FOLLOW for Inter2 -> Inter3, still at Inter2,
    // the follower enabled for the (non-overshoot) line follow.
    ASSERT_EQ(Localization.getCurrentNode(), inter2);
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);

    // T11 TC5 (AC5) — the anticipatory approach ramp tracks the SAME dispatch-time
    // pass-through latch (`_lineFollowPassThrough`, F3) the brake-on-gate/ramp-down knobs
    // above are keyed on: `LINE_FOLLOW_INTERSECTION`'s arm chains `.setApproachTarget(...)`
    // ONLY when `!_lineFollowPassThrough` (PathTracker.cpp:2742). `driveToFirstLineFollow()`
    // just drove through that exact arm for Inter2 -> Inter3, so `LineFollower.lastApproachTarget`
    // — reset to 0.0 by `resetWorld()`'s `LineFollower.mockReset()` and otherwise only ever
    // written by `setApproachTarget(...)` — is the "was it armed" observable:
    //   - flag ON (TC-PT1): Inter3 IS a pass-through node here (Inter4 collinear beyond it),
    //     so the ramp must be SUPPRESSED — full cruise across the node — and the arm must
    //     NOT have chained `.setApproachTarget(...)`: `lastApproachTarget` stays exactly 0.0.
    //   - flag OFF (TC-PT2): pass-through is globally suppressed, so this is treated as a
    //     normal TERMINATING hop and the ramp DOES arm — a positive predicted sensor-crossing
    //     distance (the pivot-trimmed edge length minus the front axle offset; no pivot is
    //     latched on this hop, so it reduces to `edge.distance_cm - LINE_SENSOR_AXLE_OFFSET_CM`).
    if (Config::LINE_APPROACH_RAMP_ENABLED)
    {
        const Routes::EdgeInfo& edge = FieldMap::getEdge(inter2, inter3);
        if (Config::PATH_PASS_THROUGH_ENABLED)
        {
            ASSERT_TRUE(LineFollower.lastApproachTarget == 0.0f);
        }
        else
        {
            ASSERT_TRUE(LineFollower.lastApproachTarget > 0.0f);
            ASSERT_TRUE(std::fabs(LineFollower.lastApproachTarget
                                  - (edge.distance_cm - Config::LINE_SENSOR_AXLE_OFFSET_CM))
                        < 1e-3f);
        }
    }

    // ---- Cross the mid junction Inter3 -------------------------------------
    // `expectLineHop()` latches `Intersection.isAtIntersection(true)` and ticks: the
    // real LINE_FOLLOW case runs the pass-through peek (next-next = Inter4, collinear,
    // non-goal) and re-arms LINE_OVERSHOOT with `setBrakeOnGate(!_passThrough)`. It then
    // trips the overshoot distance gate, advancing Localization onto Inter3.
    const int armsBefore = LineFollower.setEnabledCount;
    expectLineHop();

    // The overshoot arm happened (re-enable on the junction-detect tick) ...
    ASSERT_GT(LineFollower.setEnabledCount, armsBefore);
    // ... it carried the overshoot semantics (project-wide ADR-023 rule: always
    // overshootMode=true at the junction-detect re-arm) ...
    ASSERT_TRUE(LineFollower.lastOvershootMode);
    // ... and the distance gate was set to the axle offset (the overshoot window).
    ASSERT_TRUE(LineFollower.lastDistanceTarget > 0.0f);

    // THE FLAG-DIVERGENT ASSERT: brake-on-gate (and ramp-down) flip with the compile
    // flag. ON => pass-through fires => no brake; OFF => stop-and-go => brake.
    if (Config::PATH_PASS_THROUGH_ENABLED)
    {
        // TC-PT1: pass-through FIRES — line-follow past Inter3 WITHOUT braking.
        ASSERT_FALSE(LineFollower.lastBrakeOnGate);
        ASSERT_FALSE(LineFollower.lastTShapeRampDown);
    }
    else
    {
        // TC-PT2: pass-through SUPPRESSED — brake + re-plan (stop-and-go) at Inter3.
        ASSERT_TRUE(LineFollower.lastBrakeOnGate);
        ASSERT_TRUE(LineFollower.lastTShapeRampDown);
    }

    // The overshoot gate trip advanced the discrete node onto Inter3 (the mid junction)
    // via `Localization.onIntersection(_nextHop)` — the FSM is now back in PLAN_NEXT_HOP.
    ASSERT_EQ(Localization.getCurrentNode(), inter3);

    // ---- Drive the second hop Inter3 -> Inter4 to the goal ------------------
    // PLAN_NEXT_HOP re-plans Inter3 -> Inter4 (LINE, heading 0, no pre-turn) and arms
    // LINE_FOLLOW again. Tick into that LINE_FOLLOW, then cross the final junction.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == inter3)
        {
            break;    // LINE_FOLLOW for the Inter3 -> Inter4 hop
        }
    }
    ASSERT_EQ(Localization.getCurrentNode(), inter3);
    ASSERT_TRUE(LineFollower.isEnabled());

    // Cross Inter4. This is the FINAL hop (Inter4 == goal), so the pass-through peek's
    // not-final guard rejects pass-through here regardless of the flag — the arm brakes
    // on the goal junction. We only assert the node advanced to the destination.
    expectLineHop();
    ASSERT_EQ(Localization.getCurrentNode(), inter4);

    // The whole triple was traversed Inter2 -> Inter3 -> Inter4 by the real FSM, and the
    // goal is reached (no fault). One more tick lets PLAN_NEXT_HOP see cur == goal and
    // settle to IDLE.
    cooperativeTick();
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_announce_build_mode();
    test_routing_is_forced_through_mid_junction();
    test_passthrough_arm_and_advance_diverge_by_flag();
    TEST_REPORT();
}
