/**
 * @file test_path_tracker_pass_through_projection.cpp
 * @brief Host-link test for the firmware `line(A)-projection(B)-line(C)`
 *        distance-gated pass-through, driving the REAL `PathTracker` + `Drive` +
 *        `Localization` FSMs through the shared cooperative-tick harness
 *        (issue #106, T5b; behavioral coverage TC-PP1 / TC-PP2 / TC-PP3).
 *
 * @details
 * Rewritten from the original mirror-slice test. The old file re-implemented the
 * `PathTracker::_evaluateProjectionPassThrough` decision cell-for-cell in three
 * annotated firmware-mirror helpers over a TOY routes graph and asserted only the
 * gate's boolean return. Those mirror slices are deleted: this file now drives the
 * production `PathTracker` FSM over the REAL 14-node `fixtures/routes_fixture.h`
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
 * Three behavioral cases:
 *   - TC-PP1: `goTo(Inter2)` from `Inter1` — the gate FIRES. The PROJECTION_FOLLOW
 *             gate trips on DISTANCE at `ProjL`, the FSM stamps
 *             `onDeadReckonedAdvance(ProjL)` MID-FOLLOW (no stop), keeps the
 *             follower live (`brakeOnGate == false`), and continues to the real
 *             junction `Inter2` where it re-anchors. Node advances
 *             `Inter1 -> ProjL -> Inter2`.
 *   - TC-PP2: `goTo(ProjL)` where `ProjL` IS the goal — the not-final guard
 *             rejects the collapse (a collinear LINE flank `ProjL -> Inter2`
 *             genuinely exists, so the ONLY reason it declines is goal-is-ProjL).
 *             The FSM stops AT `ProjL` (`brakeOnGate == true`, follower disabled).
 *   - TC-PP3: a BLIND-flanked projection node (`ProjB`) — the LINE-projection
 *             pass-through gate does NOT fire (it is reached by a BLIND edge, never
 *             a LINE edge, so the gate is never even evaluated). The BLIND ->
 *             projection auto-seek path runs instead and re-anchors AT `ProjB`,
 *             stopping at the projection node.
 *
 * FIXTURE-ROUTING DEVIATION (reported): the spec's TC-PP3 asks for a LINE hop into
 * a projection node whose OUTBOUND flank is BLIND / non-collinear so the gate's
 * condition 3/5 declines. The fixture has no such route — `ProjL` is the ONLY
 * LINE-reached projection node and both its onward flanks (`-> Inter1` heading
 * 180, `-> Inter2` heading 0) are LINE edges collinear with whichever inbound hop
 * reaches it (the node sits mid-line). `ProjB` is BLIND-flanked but is reached
 * ONLY by BLIND edges, so the LINE-projection gate is never evaluated for it.
 * Through the REAL FSM the gate can therefore decline at `ProjL` only via the
 * not-final guard (TC-PP2). TC-PP3 is adapted to the genuinely-reachable
 * fixture behavior that matches the spec's intent and node hint ("the fixture's
 * `ProjB` may be the BLIND-flanked one"): drive `goTo(ProjB)` from `Corner` (a
 * BLIND edge into the BLIND-flanked projection node) and assert the LINE-projection
 * pass-through gate never fires — the FSM takes the auto-seek STOP_AT_LINE path and
 * stops AT `ProjB`. This is pinned with a routing + flank guard so a future fixture
 * edit that grows a LINE edge into a projection node fails loudly here.
 */

#include "test_path_tracker_harness.h"

namespace
{
// ---------------------------------------------------------------------------
// Drive the real PathTracker from a clean world up to the PROJECTION_FOLLOW state
// of the `Inter1 -> ProjL` hop, with the LineFollower armed for the distance-gated
// projection follow. Returns with the FSM in PROJECTION_FOLLOW, the follower
// freshly enabled (the LINE -> projection arm: overshootMode=false), and no Drive
// move in flight (a LINE/projection follow issues no lower-MCU move).
//
// Sequence the real FSM runs to get here (PathTracker.cpp):
//   IDLE -> DEQUEUE (pop goal, offset~0 return-gate -> PLAN_NEXT_HOP)
//        -> PLAN_NEXT_HOP (Inter1->ProjL LINE, heading 0 == current world 0, so
//           `_dispatchTurnIfNeeded` skips the pre-turn) -> _dispatchEdgeAfterTurn
//        -> (isProjectionNode(ProjL)) evaluate `_evaluateProjectionPassThrough`,
//           arm LineFollower with brakeOnGate=!_projPassThrough -> PROJECTION_FOLLOW.
// No pre-edge turn fires because Inter1->ProjL departs at heading 0 and
// `seatAt("Inter1")` bound the world frame at app-yaw 0 (harness caveat S3 does
// not bite: the departure heading IS the default heading).
//
// @param goalName The active goTo destination ("Inter2" for the routed-PAST
//                 pass-through case; "ProjL" for the not-final stop case).
// ---------------------------------------------------------------------------
void driveToProjectionFollow(const char* goalName)
{
    resetWorld();
    seatAt("Inter1");    // bind world frame @ app-yaw 0 + declare Localization at Inter1

    // Enqueue the goal; PathTracker walks NEXT_HOP hop-by-hop. `.async()` so the
    // harness drives the cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(nodeByName(goalName)).async();

    // Tick until the FSM dispatches the LINE -> projection hop and arms the follower
    // for the distance-gated projection follow (overshootMode=false, enabled). That
    // arm is the PROJECTION_FOLLOW entry; the follower is enabled and no Drive move is
    // pending, so the projection distance gate can be tripped next.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == nodeByName("Inter1"))
        {
            return;    // PROJECTION_FOLLOW reached, follower armed for the projection follow
        }
    }
    ASSERT_TRUE(false);    // budget exhausted: the FSM never reached PROJECTION_FOLLOW
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
// (Adapted from the spec's "BLIND-flanked / non-collinear projection -> no
// pass-through" intent — see the file-level FIXTURE-ROUTING DEVIATION note: no
// fixture route exists for a LINE hop into a projection node with a BLIND/non-
// collinear outbound flank, so this drives the genuinely-reachable ProjB behavior.)
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
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker line(Inter1)-projection(ProjL)-line(Inter2) "
                "distance-gated pass-through (issue #106, T5b, TC-PP1/PP2/PP3) -- "
                "real PathTracker + Drive + Localization over routes_fixture\n");

    test_routing_is_forced_through_projection_node();
    test_TC_PP1_distance_gated_pass_through_fires();
    test_TC_PP2_goto_projection_goal_stops_at_projection();
    test_TC_PP3_blind_flanked_projection_node_no_pass_through();

    TEST_REPORT();
}
