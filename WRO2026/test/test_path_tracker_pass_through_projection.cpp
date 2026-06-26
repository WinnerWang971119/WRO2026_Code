/**
 * @file test_path_tracker_pass_through_projection.cpp
 * @brief Host-link test for the firmware `line(A)-projection(B)-line(C)`
 *        distance-gated pass-through, driving the REAL `PathTracker` + `Drive` +
 *        `Localization` FSMs through the shared cooperative-tick harness
 *        (issue #106, T5b; behavioral coverage TC-PP1..TC-PP5).
 *
 * @details
 * Rewritten from the original mirror-slice test. The old file re-implemented the
 * `PathTracker::_evaluateProjectionPassThrough` decision cell-for-cell in three
 * annotated firmware-mirror helpers over a TOY routes graph and asserted only the
 * gate's boolean return. Those mirror slices are deleted: this file now drives the
 * production `PathTracker` FSM over the REAL `fixtures/routes_fixture.h` graph
 * and asserts the OBSERVABLE outcomes the FSM itself produces — the node-advance
 * sequence, the LineFollower brake-on-gate arm, and the re-anchor state.
 *
 * THE HOP (`fixtures/routes_fixture.h`): the collinear `line-projection-line`
 * triple `Inter1 -> ProjL -> Inter2`. `Inter1` (LINE, x=60) and `Inter2` (LINE,
 * x=140) sit on the main line `y=40`; `ProjL` (PROJECTION, x=100) sits ON that
 * line but at NO junction marker. `Inter1 -> ProjL` and `ProjL -> Inter2` are both
 * LINE edges at heading 0 (collinear). A `goTo(Inter2)` from `Inter1` walks
 * `NEXT_HOP` THROUGH `ProjL`; the bypass edge `Inter1 -> Inter2` is
 * `EdgeType::NONE`, so Dijkstra MUST traverse `ProjL` (pinned with the routing
 * guard + `assertNoBypass`).
 *
 * The gate's TRIGGER is the `Inter1 -> ProjL` DISTANCE gate, NOT
 * `Intersection.isAtIntersection()` — a projection node has no junction marker, so
 * an intersection never fires at `ProjL` (PROJECTION_FOLLOW state, PathTracker.cpp).
 *
 * This target does NOT override `PATH_PASS_THROUGH_ENABLED_DEFAULT`, so the
 * production default (`true`) compiles in and TC-PP1's collapse can fire.
 *
 * Five behavioral cases:
 *   - TC-PP1: `goTo(Inter2)` from `Inter1` — the gate FIRES. The PROJECTION_FOLLOW
 *             gate trips on DISTANCE at `ProjL`, the FSM stamps
 *             `onDeadReckonedAdvance(ProjL)` MID-FOLLOW (no stop), keeps the
 *             follower live (`brakeOnGate == false`), and continues to the real
 *             junction `Inter2` where it re-anchors. Node advances
 *             `Inter1 -> ProjL -> Inter2`.
 *   - TC-PP2: `goTo(ProjL)` where `ProjL` IS the goal — the not-final guard (gate
 *             condition 2) rejects the collapse (a collinear LINE flank
 *             `ProjL -> Inter2` genuinely exists, so the ONLY reason it declines is
 *             goal-is-ProjL). The FSM stops AT `ProjL` (`brakeOnGate == true`,
 *             follower disabled).
 *   - TC-PP3: a BLIND-flanked projection node (`ProjB`) — the LINE-projection
 *             pass-through gate is NEVER EVALUATED (`ProjB` is reached by a BLIND
 *             edge, not a LINE edge, so the dispatch takes the BLIND->projection
 *             auto-seek path). The seek re-anchors AT `ProjB`. The "gate not even
 *             reached" case.
 *   - TC-PP4: `goTo(Cend)` over `Inter2 -> ProjC -> Cend` — the gate IS evaluated
 *             (LINE into the projection `ProjC`) and DECLINES on gate condition 3:
 *             the outbound flank `ProjC -> Cend` is a BLIND edge. `ProjC` is NOT
 *             the goal, so the not-final guard is not the decliner — isolating
 *             condition 3. The FSM stops AT `ProjC` (`brakeOnGate == true`), then
 *             dead-reckons the onward BLIND hop to `Cend`.
 *   - TC-PP5: `goTo(Efar)` over `Inter3 -> ProjE -> Efar` — the gate DECLINES on
 *             gate condition 5: both flanks are LINE and `Efar` is a LINE junction
 *             (conditions 3 + 4 PASS), but `ProjE -> Efar` (heading 0) is NOT
 *             collinear with `Inter3 -> ProjE` (heading 270) — a 90 deg bend at the
 *             marker-less node. `ProjE` is NOT the goal, isolating the collinearity
 *             check. The FSM stops AT `ProjE` (`brakeOnGate == true`).
 *
 * TC-PP4 / TC-PP5 (#112) close the two gate-decline branches the earlier rewrite
 * could not reach: the original `routes_fixture` had only ONE LINE-reached
 * projection (`ProjL`), whose flanks are both LINE and collinear, so through the
 * REAL FSM the gate could decline there only via the not-final guard (TC-PP2). The
 * fixture now also carries `Inter2 -> ProjC -> Cend` (BLIND outbound flank) and
 * `Inter3 -> ProjE -> Efar` (non-collinear LINE flank) — two pendants off the main
 * line that change no existing route — so condition 3 and condition 5 are exercised
 * through the real FSM (see `WRO2026/test/fixtures/regenerate_routes_fixture.py`).
 */

#include "test_path_tracker_harness.h"

namespace
{
// ---------------------------------------------------------------------------
// Drive the real PathTracker from a clean world up to the PROJECTION_FOLLOW state of
// the `<start> -> <projection>` hop (the first hop of `goTo(goalName)` seated at
// `startName` facing `startAppYaw`), with the LineFollower armed for the distance-
// gated projection follow. Returns with the FSM in PROJECTION_FOLLOW, the follower
// freshly enabled (the LINE -> projection arm: overshootMode=false), and no Drive
// move in flight (a LINE/projection follow issues no lower-MCU move).
//
// Sequence the real FSM runs to get here (PathTracker.cpp):
//   IDLE -> DEQUEUE (pop goal, offset~0 return-gate -> PLAN_NEXT_HOP)
//        -> PLAN_NEXT_HOP (LINE first hop; if the edge heading == the seated facing,
//           `_dispatchTurnIfNeeded` skips the pre-turn) -> _dispatchEdgeAfterTurn
//        -> (isProjectionNode(proj)) evaluate `_evaluateProjectionPassThrough`,
//           arm LineFollower with brakeOnGate=!_projPassThrough -> PROJECTION_FOLLOW.
//
// @param startName Node to seat Localization at (the LINE->projection hop origin).
// @param goalName  The active goTo destination — drives which onward flank the gate
//                  peeks at (a routed-PAST goal, the projection itself, or a node
//                  beyond a declining flank).
// ---------------------------------------------------------------------------
void driveToProjectionFollowHop(const char* startName, const char* goalName)
{
    resetWorld();
    seatAt(startName);    // bind world frame @ app-yaw 0 + declare Localization at the hop origin

    const uint8_t start    = nodeByName(startName);
    const uint8_t firstHop = routedFirstHop(start, nodeByName(goalName));
    // `seatAt` always binds the facing to START_HEADING_DEG (0). If the inbound edge departs at a
    // non-zero heading the FSM dispatches a FRONT pre-edge turn first (BIDIRECTIONAL_AUTO ties go
    // FRONT, so a 90 deg bend stays FRONT). Park the corrected IMU read at the edge heading each
    // tick so that pre-turn settles immediately on its body target — the driveToSeekInFlight idiom.
    const int16_t hwHeading = hwYaw((int16_t) lroundf(FieldMap::getEdge(start, firstHop).heading_deg));

    // Enqueue the goal; PathTracker walks NEXT_HOP hop-by-hop. `.async()` so the
    // harness drives the cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(nodeByName(goalName)).async();

    // Tick until the FSM dispatches the LINE -> projection hop and arms the follower
    // for the distance-gated projection follow (overshootMode=false, enabled). That
    // arm is the PROJECTION_FOLLOW entry; the follower is enabled and no Drive move is
    // pending, so the projection distance gate can be tripped next.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == start)
        {
            return;    // PROJECTION_FOLLOW reached, follower armed for the projection follow
        }
    }
    ASSERT_TRUE(false);    // budget exhausted: the FSM never reached PROJECTION_FOLLOW
}

// Inter1 -> ProjL convenience wrapper for the collinear triple (TC-PP1 / TC-PP2):
// the Inter1->ProjL edge departs heading 0 == the seated facing, so no pre-turn fires.
void driveToProjectionFollow(const char* goalName)
{
    driveToProjectionFollowHop("Inter1", goalName);
}
}    // namespace

// ===========================================================================
// Routing guard — pin the line-proj-line triple so a future fixture re-weight
// fails LOUDLY rather than silently re-routing the pass-through under test
// (Fable amendment 2).
// ===========================================================================

void test_routing_is_forced_through_projection_node()
{
    TEST_SCOPE("routing: goTo(Inter2) from Inter1 MUST traverse ProjL (no bypass)");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");
    const uint8_t inter2 = nodeByName("Inter2");

    // The fixture must resolve these three names (the stable-name contract).
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(projL != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE);

    // ProjL is a projection node; Inter1 and Inter2 are LINE junction nodes.
    ASSERT_TRUE(FieldMap::isProjectionNode(projL));
    ASSERT_TRUE(FieldMap::getNodeType(inter1) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(inter2) == Routes::NodeType::LINE);

    // Dijkstra's first hop Inter1 -> Inter2 is ProjL ...
    ASSERT_EQ(routedFirstHop(inter1, inter2), projL);
    // ... and there is NO direct Inter1 -> Inter2 edge, so the only route is THROUGH
    // ProjL — the line-proj-line triple is genuinely traversed, not bypassed.
    assertNoBypass(inter1, inter2);

    // Both flanks of the triple are LINE edges into/out of the projection node.
    ASSERT_TRUE(FieldMap::getEdge(inter1, projL).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(projL, inter2).type == Routes::EdgeType::LINE);

    // Collinear: Inter1->ProjL and ProjL->Inter2 share a heading (within the gate's
    // |delta| <= NAV_TURN_TOLERANCE_DEG window).
    const float delta = wrapDeltaDeg(FieldMap::getEdge(projL, inter2).heading_deg
                                     - FieldMap::getEdge(inter1, projL).heading_deg);
    ASSERT_TRUE(fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG);

    // The onward lookahead from ProjL toward Inter2 is the direct hop Inter2 (so the
    // gate's C is the LINE junction Inter2, exercising the real route).
    ASSERT_EQ(routedFirstHop(projL, inter2), inter2);
}

// ===========================================================================
// TC-PP1 — line(Inter1)-projection(ProjL)-line(Inter2) routed PAST ProjL: the
//          distance-gated pass-through FIRES. The follower stays live across the
//          marker-less projection node (brake-on-gate=false), the node advances
//          mid-follow by dead-reckoning at ProjL, and the hop re-anchors at the
//          real junction Inter2.
// ===========================================================================

void test_TC_PP1_distance_gated_pass_through_fires()
{
    TEST_SCOPE("TC-PP1: line-proj-line pass-through FIRES (ProjL mid-follow advance -> Inter2)");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");
    const uint8_t inter2 = nodeByName("Inter2");

    driveToProjectionFollow("Inter2");

    // Precondition: the FSM is in PROJECTION_FOLLOW for Inter1 -> ProjL, still at
    // Inter1, the follower enabled for the (non-overshoot) distance follow.
    ASSERT_EQ(Localization.getCurrentNode(), inter1);
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);

    // THE PASS-THROUGH ARM ASSERT: the gate fired at arm time, so the projection
    // follow was armed with brake-on-gate=false — the follower will NOT self-disable
    // or brake when the distance gate trips, keeping the Inter1->Inter2 follow
    // continuous across the marker-less ProjL. (`_projPassThrough == true` is observed
    // here through the LineFollower arm cascade, the production observable.)
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);

    // ---- Trip the ProjL DISTANCE gate (NOT an intersection) ----------------
    // A projection node has no junction marker, so PROJECTION_FOLLOW waits on
    // `LineFollower.isDistanceReached()`, not `Intersection.isAtIntersection()`. Trip
    // it: the FSM stamps `Localization.onDeadReckonedAdvance(ProjL)` MID-FOLLOW,
    // keeps the follower live, and re-plans the ProjL->Inter2 hop.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();

    // The mid-follow dead-reckon advance landed the node on ProjL (NOT Inter1 still,
    // NOT Inter2 yet) — the one-advance-per-node invariant the pass-through preserves.
    ASSERT_EQ(Localization.getCurrentNode(), projL);
    // The follower was kept live (no disable on the trip) — the collapse continues.
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(PathTracker.hasFault());

    // Clear the stale distance-trip flag (the path mock does not auto-clear on read)
    // so the upcoming LINE_OVERSHOOT does not trip prematurely on a leftover gate.
    LineFollower.mockSetDistanceReached(false);

    // ---- Continue the ProjL -> Inter2 hop to the junction re-anchor ---------
    // PLAN_NEXT_HOP re-plans ProjL -> Inter2 (LINE into a LINE junction, heading 0, no
    // pre-turn). Tick into LINE_FOLLOW for that hop (still at ProjL), then cross the
    // real junction Inter2.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == projL)
        {
            break;    // LINE_FOLLOW for the ProjL -> Inter2 hop
        }
    }
    ASSERT_EQ(Localization.getCurrentNode(), projL);
    ASSERT_TRUE(LineFollower.isEnabled());

    // Cross Inter2 (the real junction). This is the FINAL hop (Inter2 == goal), so the
    // LINE_FOLLOW pass-through peek's not-final guard rejects pass-through here — the
    // arm brakes on the goal junction and `Localization.onIntersection(Inter2)`
    // re-anchors. `expectLineHop()` drives the intersection-detect + overshoot gate.
    expectLineHop();
    ASSERT_EQ(Localization.getCurrentNode(), inter2);

    // The whole triple was traversed Inter1 -> ProjL -> Inter2 by the real FSM, and the
    // goal is reached (no fault). One more tick lets PLAN_NEXT_HOP see cur == goal and
    // settle to IDLE.
    cooperativeTick();
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC-PP2 — goTo(ProjL) where ProjL IS the projection goal: the not-final guard
//          rejects the collapse -> a normal DISTANCE stop AT ProjL. A collinear
//          LINE flank ProjL->Inter2 genuinely exists, so the ONLY reason the gate
//          declines is goal-is-ProjL (NOT a missing flank).
// ===========================================================================

void test_TC_PP2_goto_projection_goal_stops_at_projection()
{
    TEST_SCOPE("TC-PP2: goTo(ProjL) — not-final guard rejects pass-through -> stop at ProjL");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");
    const uint8_t inter2 = nodeByName("Inter2");

    // A collinear LINE flank ProjL->Inter2 genuinely exists beyond the goal, so the
    // gate's flank (condition 3) / node-type (condition 4) / heading (condition 5)
    // checks would all PASS — isolating the not-final guard (condition 2) as the sole
    // reason the gate declines here.
    ASSERT_TRUE(FieldMap::getEdge(projL, inter2).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(inter2) == Routes::NodeType::LINE);

    driveToProjectionFollow("ProjL");

    // Precondition: PROJECTION_FOLLOW for Inter1 -> ProjL, still at Inter1.
    ASSERT_EQ(Localization.getCurrentNode(), inter1);
    ASSERT_TRUE(LineFollower.isEnabled());

    // THE NOT-FINAL ASSERT: ProjL IS the goal, so the gate's not-final guard rejected
    // the collapse — the projection follow was armed with brake-on-gate=TRUE. The
    // follower WILL self-disable + brake when the distance gate trips (a normal stop at
    // the projection node), the opposite of TC-PP1's pass-through arm.
    ASSERT_TRUE(LineFollower.lastBrakeOnGate);

    // ---- Trip the ProjL DISTANCE gate --------------------------------------
    // The FSM disables the follower (active brake) and dead-reckons the advance onto
    // ProjL, then PLAN_NEXT_HOP sees cur == goal == ProjL -> IDLE (stop at ProjL).
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();

    // The node advanced to ProjL and the FSM STOPPED there (the goal) — it did NOT
    // continue to Inter2 (no pass-through). The follower was disabled on the stop.
    ASSERT_EQ(Localization.getCurrentNode(), projL);
    ASSERT_FALSE(LineFollower.isEnabled());
    ASSERT_FALSE(PathTracker.hasFault());

    // Settle: cur == goal == ProjL -> IDLE. The discrete node stays at ProjL and the
    // FSM never advanced toward Inter2 — the pass-through genuinely did not fire.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_EQ(Localization.getCurrentNode(), projL);
}

// ===========================================================================
// TC-PP3 — a BLIND-flanked projection node (`ProjB`): the LINE-projection
//          pass-through gate does NOT fire. ProjB is reached ONLY by a BLIND edge
//          (Corner->ProjB), so `_evaluateProjectionPassThrough` (the LINE->
//          projection dispatch path) is never even evaluated for it. The BLIND ->
//          projection auto-seek path runs instead (BLIND_PROJECTION_AUTO_SEEK), and
//          the seek re-anchors AT ProjB — the robot stops at the projection node.
//
// This is the "gate never evaluated" case (a BLIND-reached projection never takes
// the LINE->projection dispatch path). The two LINE-reached decline branches — a
// BLIND outbound flank (condition 3) and a non-collinear LINE flank (condition 5) —
// are covered by TC-PP4 / TC-PP5 below.
// ===========================================================================

void test_TC_PP3_blind_flanked_projection_node_no_pass_through()
{
    TEST_SCOPE("TC-PP3: BLIND-flanked ProjB -> LINE-projection gate never fires; stop at ProjB");

    const uint8_t corner = nodeByName("Corner");
    const uint8_t projB  = nodeByName("ProjB");

    // The fixture must resolve these names (the stable-name contract).
    ASSERT_TRUE(corner != FieldMap::NODE_NONE);
    ASSERT_TRUE(projB != FieldMap::NODE_NONE);

    // ProjB is a projection node, and its inbound flank from Corner is a BLIND edge —
    // the LINE-projection pass-through gate (which only runs on a LINE-into-projection
    // dispatch) is therefore never evaluated for ProjB. Pin both facts so a future
    // fixture edit that grows a LINE edge into ProjB fails loudly here.
    ASSERT_TRUE(FieldMap::isProjectionNode(projB));
    ASSERT_TRUE(FieldMap::getEdge(corner, projB).type == Routes::EdgeType::BLIND);

    // Route: goTo(ProjB) from Corner is the single BLIND hop Corner -> ProjB.
    ASSERT_EQ(routedFirstHop(corner, projB), projB);

    resetWorld();
    seatAt("Corner");    // bind world frame @ app-yaw 0 + declare Localization at Corner
    PathTracker.goTo(projB).async();

    // Drive the FSM until it dispatches the BLIND -> projection auto-seek. Corner->ProjB
    // departs at heading 0 (== the bound app-yaw), so no pre-turn fires and the FSM goes
    // straight to STOP_AT_LINE (the auto-seek), NEVER to PROJECTION_FOLLOW (that state is
    // the LINE-projection gate path). Wait until a seek is in flight: the path mock's
    // LineArray records the seek's `setReverseActive(...)` board flip.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineArray.mockSetReverseCallCount() > 0)
        {
            break;    // the BLIND auto-seek dispatched (Drive.stopAtLine -> setReverseActive)
        }
    }
    ASSERT_TRUE(LineArray.mockSetReverseCallCount() > 0);

    // The LINE-projection pass-through gate never fired: the follower is NOT armed for a
    // projection follow (the seek path hands the wheels to Drive, not LineFollower), and
    // the node is still Corner (the seek has not completed yet).
    ASSERT_FALSE(LineFollower.isEnabled());
    ASSERT_EQ(Localization.getCurrentNode(), corner);
    ASSERT_FALSE(PathTracker.hasFault());

    // Drive the seek to a SUCCESSFUL crossing: the seek lands on the line, the FSM
    // re-anchors AT ProjB via `onIntersection(ProjB)` — the robot stops at the
    // projection node (ProjB is the goal). `expectStopAtLine` runs the seek
    // success handshake (arm window + debounce + overshoot roll).
    expectStopAtLine(FieldMap::getEdge(corner, projB).distance_cm, /*found*/ true);

    // The seek success advanced the discrete node onto ProjB and the FSM settled — the
    // robot stopped at the projection node, with the LINE-projection pass-through gate
    // never having been reached.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_EQ(Localization.getCurrentNode(), projB);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC-PP4 — LINE->projection with a BLIND outbound flank: the pass-through gate IS
//          evaluated and DECLINES on condition 3 (outbound flank not LINE). Drives
//          `goTo(Cend)` over `Inter2 -> ProjC -> Cend`: ProjC is a projection node
//          reached by a LINE edge (so the gate runs), it is NOT the goal (so the
//          not-final guard is not the decliner), and its onward flank ProjC->Cend is
//          a BLIND edge — so condition 3 rejects the collapse, the FSM stops AT
//          ProjC, then dead-reckons the BLIND hop on to Cend. (#112: a decline
//          branch the collinear ProjL triple could not reach.)
// ===========================================================================

void test_TC_PP4_line_projection_blind_flank_declines()
{
    TEST_SCOPE("TC-PP4: LINE->projection ProjC, BLIND outbound flank -> gate declines (cond 3), stop at ProjC");

    const uint8_t inter2 = nodeByName("Inter2");
    const uint8_t projC  = nodeByName("ProjC");
    const uint8_t cend   = nodeByName("Cend");

    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE);
    ASSERT_TRUE(projC != FieldMap::NODE_NONE);
    ASSERT_TRUE(cend != FieldMap::NODE_NONE);

    // Isolate the decline to condition 3 (outbound flank not LINE):
    //   - the inbound Inter2->ProjC is a LINE edge into a projection node (the gate IS evaluated);
    //   - ProjC is NOT the goal (Cend is, beyond it) -> not the not-final guard (condition 2);
    //   - the onward flank ProjC->Cend is a BLIND edge -> condition 3 rejects.
    ASSERT_TRUE(FieldMap::isProjectionNode(projC));
    ASSERT_TRUE(FieldMap::getEdge(inter2, projC).type == Routes::EdgeType::LINE);
    ASSERT_EQ(routedFirstHop(inter2, cend), projC);
    ASSERT_EQ(routedFirstHop(projC, cend), cend);
    ASSERT_TRUE(FieldMap::getEdge(projC, cend).type == Routes::EdgeType::BLIND);
    assertNoBypass(inter2, cend);    // the only route Inter2->Cend is THROUGH ProjC

    // Inter2->ProjC departs heading 270 (straight down off the y=40 main line); the helper settles
    // the FRONT pre-turn before the LINE->projection follow.
    driveToProjectionFollowHop("Inter2", "Cend");

    // Precondition: PROJECTION_FOLLOW for Inter2->ProjC, still at Inter2, follower armed.
    ASSERT_EQ(Localization.getCurrentNode(), inter2);
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);

    // THE DECLINE ASSERT: the gate evaluated and DECLINED on the BLIND outbound flank, so the
    // projection follow armed with brake-on-gate=TRUE — a normal stop at ProjC, NOT a pass-through.
    ASSERT_TRUE(LineFollower.lastBrakeOnGate);

    // Trip the projection distance gate: the FSM stops, dead-reckons the advance onto ProjC, and
    // disables the follower. ProjC is NOT the goal, so the mission continues.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projC);
    ASSERT_FALSE(LineFollower.isEnabled());
    ASSERT_FALSE(PathTracker.hasFault());

    // The onward ProjC->Cend hop is a plain BLIND dead-reckon (Cend is a blind node, no auto-seek):
    // it pre-turns from the 270-facing to the BLIND heading, then drives forward. Feed both the turn
    // settle (park the IMU at the edge heading) and the move retire (advance the encoder + done-bit)
    // each tick until the node advances to Cend.
    const int16_t cendHwHeading = hwYaw((int16_t) lroundf(FieldMap::getEdge(projC, cend).heading_deg));
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != cend; ++i)
    {
        mmL.mockYawDeg = (double) cendHwHeading;
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), cend);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC-PP5 — LINE->projection with a non-collinear LINE outbound flank: the gate
//          DECLINES on condition 5 (the collinearity / same-direction check). Drives
//          `goTo(Efar)` over `Inter3 -> ProjE -> Efar`: both flanks are LINE and Efar
//          is a LINE junction (conditions 3 + 4 PASS), ProjE is NOT the goal (the
//          not-final guard is not the decliner), but ProjE->Efar (heading 0) is a
//          90 deg bend off the inbound Inter3->ProjE (heading 270) — so ONLY the
//          collinearity check can reject, isolating condition 5. The FSM stops AT
//          ProjE. (#112: the heading-half of the gate, previously a contrived unit
//          probe, now exercised through the real FSM.)
//
// Scope: this asserts the gate DECISION (decline -> stop at ProjE). The onward
// ProjE->Efar LINE hop (a real 90 deg pre-turn + line follow) is out of scope — the
// next test's resetWorld() reclaims the mid-mission FSM.
// ===========================================================================

void test_TC_PP5_line_projection_non_collinear_flank_declines()
{
    TEST_SCOPE("TC-PP5: LINE->projection ProjE, non-collinear LINE flank -> gate declines (cond 5), stop at ProjE");

    const uint8_t inter3 = nodeByName("Inter3");
    const uint8_t projE  = nodeByName("ProjE");
    const uint8_t efar   = nodeByName("Efar");

    ASSERT_TRUE(inter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(projE != FieldMap::NODE_NONE);
    ASSERT_TRUE(efar != FieldMap::NODE_NONE);

    // Isolate the decline to condition 5 (non-collinear), NOT 3 / 4 / not-final:
    //   - inbound Inter3->ProjE is LINE into a projection node (gate evaluated);
    //   - ProjE is NOT the goal (Efar is) -> not the not-final guard (condition 2);
    //   - the onward flank ProjE->Efar is a LINE edge -> condition 3 PASSES;
    //   - Efar is a LINE junction node -> condition 4 PASSES;
    //   - but ProjE->Efar (heading 0) is NOT collinear with Inter3->ProjE (heading 270):
    //     |wrapDelta| == 90 deg > NAV_TURN_TOLERANCE_DEG -> ONLY condition 5 can reject.
    ASSERT_TRUE(FieldMap::isProjectionNode(projE));
    ASSERT_TRUE(FieldMap::getEdge(inter3, projE).type == Routes::EdgeType::LINE);
    ASSERT_EQ(routedFirstHop(inter3, efar), projE);
    ASSERT_EQ(routedFirstHop(projE, efar), efar);
    ASSERT_TRUE(FieldMap::getEdge(projE, efar).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(efar) == Routes::NodeType::LINE);
    const float bend = wrapDeltaDeg(FieldMap::getEdge(projE, efar).heading_deg
                                    - FieldMap::getEdge(inter3, projE).heading_deg);
    ASSERT_TRUE(fabsf(bend) > Config::NAV_TURN_TOLERANCE_DEG);
    assertNoBypass(inter3, efar);    // the only route Inter3->Efar is THROUGH ProjE

    // Inter3->ProjE departs heading 270; the helper settles the FRONT pre-turn before the follow.
    driveToProjectionFollowHop("Inter3", "Efar");

    // Precondition: PROJECTION_FOLLOW for Inter3->ProjE, still at Inter3, follower armed.
    ASSERT_EQ(Localization.getCurrentNode(), inter3);
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);

    // THE DECLINE ASSERT: every other gate condition passes, so brake-on-gate=TRUE here can only be
    // the collinearity check (condition 5) rejecting the 90 deg bend — a normal stop at ProjE.
    ASSERT_TRUE(LineFollower.lastBrakeOnGate);

    // Trip the gate -> stop + dead-reckon advance onto ProjE, follower disabled. ProjE != goal, so
    // the FSM did NOT pass through and is NOT done — it stopped at the projection and will continue.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projE);
    ASSERT_FALSE(LineFollower.isEnabled());
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(PathTracker.isIdle());    // a stop (decline), not a mission-end arrival
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker line(A)-projection(B)-line(C) distance-gated "
                "pass-through (issue #106, T5b, TC-PP1..PP5; #112 decline branches) "
                "-- real PathTracker + Drive + Localization over routes_fixture\n");

    test_routing_is_forced_through_projection_node();
    test_TC_PP1_distance_gated_pass_through_fires();
    test_TC_PP2_goto_projection_goal_stops_at_projection();
    test_TC_PP3_blind_flanked_projection_node_no_pass_through();
    test_TC_PP4_line_projection_blind_flank_declines();
    test_TC_PP5_line_projection_non_collinear_flank_declines();

    TEST_REPORT();
}
