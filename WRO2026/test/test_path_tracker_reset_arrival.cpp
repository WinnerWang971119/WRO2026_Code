/**
 * @file test_path_tracker_reset_arrival.cpp
 * @brief Host-link test for the PathTracker RESET-node arrival interlude
 *        (`WALL_STOP -> REANCHOR -> WALL_DEPART -> PLAN_NEXT_HOP`), driving the REAL
 *        `PathTracker` + `Drive` + `Localization` + `YawSource` FSMs through the shared
 *        cooperative-tick harness (issue #113, T3). Strict-OFF half.
 *
 * @details
 * Rewritten from the original mirror-slice test. The old file replicated the WALL_STOP ->
 * REANCHOR decision slice inline (a hand-rolled state enum + a replica struct + a node-seeded
 * reset helper), every transition annotated against a cited PathTracker.cpp line. Those FSM-slice
 * replicas are GONE: this file now drives the production `PathTracker` FSM over the REAL issue #91
 * fixture graph and asserts the interlude the FSM itself runs, against the REAL `Localization`
 * re-anchor + offset clear.
 *
 * THE REAL INTERLUDE FIRES ONLY ON A GENUINE ARRIVAL. `_onPlanNextHop` guards the interlude
 * behind `if (!firstPlan && _maybeEnterReset(cur))` (PathTracker.cpp:1411) — the first plan
 * of a goal (`_planIsFirstAfterDequeue`, set at DEQUEUE) is skipped. So every case drives a
 * REAL inbound hop INTO the RESET node first: `goTo(Wall1)` seated at `position1` dispatches
 * one BLIND dead-reckon hop (`position1 -> Wall1`, ~45.45 cm, heading ~140.36 deg off the seated
 * 0), and the SECOND `_onPlanNextHop` (firstPlan == false) fires `_maybeEnterReset(Wall1)` ->
 * `_dispatchWallStop` -> `State::WALL_STOP`.
 *
 * THE HOP (`fixtures/routes_fixture.h`):
 *   - `position1 -> Wall1`: a direct BLIND edge (`StopAtLineMode::DEFAULT`, so a dead-reckon
 *     `driveDist`, NOT a seek — a seek would re-anchor via `onIntersection` and change the
 *     believed-pose math) into a RESET node. The routing guard pins it so a fixture re-weight
 *     fails LOUDLY rather than silently routing the hop elsewhere.
 *
 * WHAT THIS DRIVES (strict-OFF — the shipped `WALL_NORMAL_STRICT_ENABLED == false`, so the
 * live IMU heading does NOT gate the re-anchor; the strict-ON `WALL_NOT_SQUARE` reject is the
 * sibling `test_path_tracker_reset_arrival_strict.cpp`, the `test_reanchor_at_wall[_strict]`
 * precedent):
 *   - TC-RA-route — the inbound routing guard: `position1 -> Wall1` is a BLIND edge into a
 *     RESET node and the dead-reckon advance lands the discrete node on `Wall1`.
 *   - TC-RA1 (advisory) — `expectWallStop(270, timedOut=true)`: the wallStop advisory timeout
 *     does NOT halt. The FSM STILL re-anchors (real `Localization.reanchorAtWall`, accepts under
 *     strict OFF), drives the WALL_DEPART off the wall, clears the offset, and reaches
 *     PLAN_NEXT_HOP with `!PathTracker.hasFault()`. The advisory surfaces only via the no-op
 *     EventTrace mock + Serial, so it is asserted BEHAVIORALLY (the FSM continued, no fault),
 *     never via a mock bool. The INTERMEDIATE post-reanchor / pre-depart belief is the AXLE
 *     (y == 146 = node 139 + the ~7 cm `standoff - WALL_CONTACT_OFFSET_CM` offset); the FINAL
 *     post-clearOffset belief is the NODE (y == 139, NOT the wall 154) — so 139 is reached by
 *     clearing a REAL ~7 cm offset, not by it never existing.
 *   - TC-RA2 (clean stall-success) — identical with `expectWallStop(270, timedOut=false)`: the
 *     non-advisory path, same believed-pose 146 -> 139, no fault.
 *   - TC-RA3 (one-shot latch) — `goTo(Inter3)` routes a goal BEYOND `Wall1` (the Wall1 ->
 *     position1 reverse BLIND edge is the first hop), so after WALL_DEPART -> PLAN_NEXT_HOP the
 *     FSM re-enters `_onPlanNextHop` with `cur == Wall1` and a REAL next hop to plan: the
 *     interlude fires EXACTLY ONCE (`_maybeEnterReset(Wall1)` returns false the second time, the
 *     `_resetNode == cur` latch holds). A single `goTo(Wall1)` would make this vacuous (the goal
 *     is reached, no second plan), so the goal MUST be beyond `Wall1`.
 *
 * @note The `Drive.wallStop` push retire (`expectWallStop`) and the per-hop done-bit / encoder
 *       handshake (`expectBlindHop`) are encapsulated in the shared harness — this body reads as
 *       a sequence of "drive the inbound hop, retire the wall push, assert the believed pose".
 */

#include "test_path_tracker_harness.h"

#include <cmath>
#include <cstdio>

namespace
{
// ===========================================================================
// Reset-record + believed-pose facts the asserts rely on (verified against
// `fixtures/routes_fixture.cpp` + `config/ConfigNav.h`):
//   - `Wall1` (RESET) node y == 139; reset record { axis=y, wall_coord=154, facing=270 }.
//   - standoff           = |154 - 139|            = 15 cm.
//   - Config::WALL_CONTACT_OFFSET_CM               = 8 cm.
//   - departCm           = standoff - contact     = 7 cm (> 0, so WALL_DEPART runs).
//   - INTERMEDIATE belief (post-reanchor, pre-clearOffset): node + offset == 154 - 8 == 146.
//   - FINAL belief        (post-clearOffset)              : node                      == 139.
// ===========================================================================
constexpr float kWall1NodeY        = 139.0f;
constexpr float kWall1AxleBeliefY  = 146.0f;    // node + (standoff - WALL_CONTACT_OFFSET_CM)
constexpr int16_t kWall1Facing     = 270;       // app-frame wall normal the SQUARE turn aligns to

/// @brief Approx-equality for the believed-pose floats (TestAssert has only an integer
///        `ASSERT_EQ`). The believed coordinates are integral (139 / 146) but stored as floats,
///        so a tight 0.01 cm tolerance pins the exact value while absorbing FP round-off.
bool approx(float a, float b)
{
    const float d = a - b;
    return (d < 0.0f ? -d : d) < 0.01f;
}

/// @brief The believed WORLD-frame y of the chassis: the discrete node's y plus the live
///        operator-offset y. After a successful re-anchor the offset carries the axle's standoff
///        from the node; after `clearOffset` it is zero, so this collapses to the node coordinate.
float believedY(uint8_t node)
{
    const Routes::NodeInfo& n   = FieldMap::getNode(node);
    const Offset2D          off = Localization.getOffsetFromNode();
    return n.y_cm + off.y;
}

// ===========================================================================
// Real-FSM inbound driver — shared by every reset case.
//
// Drive the real PathTracker from a clean world THROUGH the `position1 -> Wall1`
// BLIND dead-reckon hop until the discrete node lands on `Wall1` (the genuine
// arrival that makes the SECOND `_onPlanNextHop` fire the interlude). Returns
// with the FSM in PLAN_NEXT_HOP, `_resetNode` un-latched (DEQUEUE re-armed it),
// and `_planIsFirstAfterDequeue` already consumed by the first plan — so the
// caller's next tick (inside `expectWallStop`) dispatches the wallStop push.
//
// Sequence the real FSM runs to get here (PathTracker.cpp):
//   IDLE -> DEQUEUE (pop Wall1, offset~0 return-gate -> PLAN_NEXT_HOP)
//        -> PLAN_NEXT_HOP (firstPlan == true: skip the interlude; resolve the
//           pre-edge turn to ~140.36 deg and dispatch it -> TURNING)
//        -> [TURNING settles] -> _dispatchEdgeAfterTurn (BLIND dead-reckon
//           driveDist -> BLIND_FORWARD) -> [drive retires] ->
//           onDeadReckonedAdvance(Wall1) -> PLAN_NEXT_HOP.
//
// Each tick parks `mmL.mockYawDeg` at the hardware-frame value of the inbound
// edge heading so BOTH the pre-edge turn (custom TurnGyro profile, CONTRACT 2)
// AND a GYRO-leaning blind hop settle, and advances `M2`/`M3` past the profile
// total + holds the done-bit so the dead-reckon move (profiled SYNC_TSHAPE or a
// flat regime) retires (CONTRACT 1). The loop is budgeted so a real-FSM
// regression fails as a budget-exhausted assert, not a hang.
// ===========================================================================
void driveInboundToWall1()
{
    resetWorld();             // clears both Drive + PathTracker faults (order-independent cases)
    seatAt("position1", 0);   // bind world frame @ app-yaw 0 + declare Localization at position1

    const uint8_t position1 = nodeByName("position1");
    const uint8_t wall1      = nodeByName("Wall1");

    // The fixture must resolve both names (the stable-name contract, issue #91).
    ASSERT_TRUE(position1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(wall1 != FieldMap::NODE_NONE);

    // Routing guard — pin the inbound hop so a fixture re-weight fails LOUDLY rather than
    // silently routing `position1 -> Wall1` through a different (e.g. seek) path.
    ASSERT_EQ(routedFirstHop(position1, wall1), wall1);
    const Routes::EdgeInfo& edge = FieldMap::getEdge(position1, wall1);
    ASSERT_TRUE(edge.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getNodeType(wall1) == Routes::NodeType::RESET);

    // Park the corrected IMU read at the inbound edge's forward heading so the pre-edge turn
    // settles on the body target each tick (the `driveToSeekInFlight` idiom, CONTRACT 2).
    const int16_t hwHeading = hwYaw((int16_t) lroundf(edge.heading_deg));

    // Enqueue the FINAL node only; PathTracker walks NEXT_HOP. `.async()` so the harness drives
    // the cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(wall1).async();

    // Tick until the dead-reckon advance lands the discrete node on Wall1. Each tick re-parks the
    // yaw (so the pre-turn cannot stall the loop) and over-advances the encoder + holds the
    // done-bit (so the dead-reckon move retires under EITHER regime — CONTRACT 1).
    for (int i = 0;
         i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != wall1;
         ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }

    ASSERT_EQ(Localization.getCurrentNode(), wall1);    // the inbound dead-reckon advance landed.
    ASSERT_FALSE(PathTracker.hasFault());               // a clean arrival, the interlude is pending.
}

// ===========================================================================
// Shared reset-arrival driver for the advisory / clean-success cases.
//
// From a fresh inbound arrival at `Wall1`, retire the wallStop push per @p timedOut
// (`expectWallStop`), tick the real FSM through REANCHOR (real
// `Localization.reanchorAtWall` over the Wall1 reset record — strict OFF, accepts), and assert:
//   - the INTERMEDIATE post-reanchor / pre-depart belief is the AXLE (y == 146): the real FSM is
//     now in WALL_DEPART with the depart `driveDist` in flight and the ~7 cm offset live;
//   - retiring the depart (`expectBlindHop`) + a settle tick clears the offset, landing the FINAL
//     belief on the NODE (y == 139) — so 139 is reached by clearing a REAL offset;
//   - the advisory / clean-success path is NOT a fault and the FSM continued (reached
//     PLAN_NEXT_HOP / IDLE, `!PathTracker.hasFault()`), asserted BEHAVIORALLY.
//
// Note on the stable INTERMEDIATE 146 belief: during WALL_DEPART `PathTracker.update()` sets
// `Localization.setNavActive(true)` (the FSM is not IDLE/FAULT, PathTracker.cpp:495), which
// FREEZES the offset integrator — so the live ~7 cm offset cannot be mutated by a stray encoder
// tick between the re-anchor and the assert. The belief stays the axle until WALL_DEPART's
// `clearOffset` zeroes it.
// ===========================================================================
void runResetArrivalContinues(bool timedOut)
{
    const uint8_t wall1 = nodeByName("Wall1");

    driveInboundToWall1();

    // Retire the wallStop push (its internal SQUARE turn to facing == 270 settles first, then the
    // PUSH retires via the requested path). Asserts Drive idle, the advisory flag matches, no fault.
    expectWallStop(kWall1Facing, timedOut);

    // WALL_STOP (Drive now idle) -> REANCHOR -> (real reanchor accepts) dispatch the WALL_DEPART
    // `driveDist(7).setMode(GYRO)` -> WALL_DEPART. Tick until that depart drive is in flight,
    // parking the squared facing so the GYRO hold settles. At this point the belief is the AXLE.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Drive.isIdle(); ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
        cooperativeTick();
    }
    ASSERT_FALSE(Drive.isIdle());          // the depart drive was dispatched (REANCHOR succeeded).
    ASSERT_FALSE(PathTracker.hasFault());  // strict OFF: the re-anchor accepted, no WALL_NOT_SQUARE.

    // INTERMEDIATE belief — the axle (node 139 + the ~7 cm standoff offset == 146), proving the
    // re-anchor deposited a REAL offset BEFORE the depart clears it.
    ASSERT_TRUE(approx(believedY(wall1), kWall1AxleBeliefY));

    // Retire the WALL_DEPART drive (a bounded `driveDist().setMode(GYRO)`, retired by the done-bit
    // drain — CONTRACT 1). Park the squared facing for the GYRO hold.
    mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
    expectBlindHop(/*cm*/ kWall1AxleBeliefY - kWall1NodeY);

    // WALL_DEPART (Drive now idle) -> clearOffset() -> PLAN_NEXT_HOP. Tick the settle so the offset
    // is cleared and the FSM continues; a single goTo(Wall1) reaches the goal here (cur == dest).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !approx(believedY(wall1), kWall1NodeY);
         ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
        cooperativeTick();
    }

    // FINAL belief — the node coordinate (139), reached by clearing the real ~7 cm offset (NOT the
    // wall 154, NOT the lingering axle 146).
    ASSERT_TRUE(approx(believedY(wall1), kWall1NodeY));

    // BEHAVIORAL invariant: the advisory / clean-success path did NOT halt — the FSM continued
    // past the interlude without faulting (the advisory surfaces only via EventTrace + Serial, so
    // there is no mock bool to read; the proof is that the FSM ran the depart + clearOffset and
    // stayed fault-free).
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(Drive.hasFault());
}
}    // namespace

// ===========================================================================
// TC-RA-route — the inbound routing guard (BLIND edge into a RESET node).
// ===========================================================================
void test_TC_RA_route_inbound_blind_hop_into_reset_node()
{
    TEST_SCOPE("TC-RA-route: position1 -> Wall1 is a BLIND dead-reckon hop into a RESET node");

    driveInboundToWall1();

    // The dead-reckon advance landed the discrete node on the RESET node, the interlude is now
    // pending on the next plan tick (firstPlan already consumed).
    ASSERT_EQ(Localization.getCurrentNode(), nodeByName("Wall1"));
    ASSERT_TRUE(FieldMap::getNodeType(nodeByName("Wall1")) == Routes::NodeType::RESET);
    ASSERT_TRUE(FieldMap::getResetInfo(nodeByName("Wall1")) != nullptr);
}

// ===========================================================================
// TC-RA1 — advisory-timeout arrival: STILL re-anchors + departs + clears the
// offset, does NOT halt, INTERMEDIATE belief 146 (axle) -> FINAL belief 139 (node).
// ===========================================================================
void test_TC_RA1_advisory_timeout_continues_clears_offset()
{
    TEST_SCOPE("TC-RA1: wallStop advisory timeout re-anchors + departs + clears offset, no fault");

    runResetArrivalContinues(/*timedOut*/ true);
}

// ===========================================================================
// TC-RA2 — clean stall-success arrival: the non-advisory path, same believed-pose
// 146 -> 139 and no fault.
// ===========================================================================
void test_TC_RA2_clean_stall_success_clears_offset()
{
    TEST_SCOPE("TC-RA2: clean wallStop stall-success re-anchors + departs + clears offset, no fault");

    runResetArrivalContinues(/*timedOut*/ false);
}

// ===========================================================================
// TC-RA3 — one-shot latch: a goal BEYOND Wall1 forces a REAL second plan tick on
// `cur == Wall1` after WALL_DEPART; `_maybeEnterReset(Wall1)` returns false the
// second time (the interlude fires EXACTLY once; the `_resetNode == cur` latch holds).
// ===========================================================================
void test_TC_RA3_interlude_fires_exactly_once()
{
    TEST_SCOPE("TC-RA3: goTo beyond Wall1 -> the reset interlude fires exactly once (latch holds)");

    const uint8_t wall1  = nodeByName("Wall1");
    const uint8_t inter3 = nodeByName("Inter3");

    ASSERT_TRUE(inter3 != FieldMap::NODE_NONE);

    // Drive the inbound hop into Wall1 exactly as the other cases (firstPlan consumed, interlude
    // pending), THEN re-queue a goal BEYOND Wall1 so the post-depart plan has a real next hop.
    driveInboundToWall1();

    // The goal MUST be beyond Wall1 (a bare goTo(Wall1) would reach the goal at the post-depart
    // plan with no second hop, making the latch test vacuous). From Wall1 the only outbound edge
    // is the reverse `Wall1 -> position1` BLIND edge, so a goal past position1 routes its first
    // hop back through position1 — a genuine second plan tick on `cur == Wall1`.
    ASSERT_EQ(routedFirstHop(wall1, inter3), nodeByName("position1"));

    // Re-queue the beyond-Wall1 goal on top of the in-flight one. The inbound goTo(Wall1) is
    // already drained to the arrival, so this enqueues the next goal; PathTracker pops it once the
    // current goal's interlude completes and re-plans from Wall1.
    PathTracker.goTo(inter3).async();

    // Retire the wallStop push (advisory path is fine — the latch behaviour is path-independent),
    // then drive the interlude all the way through WALL_DEPART -> clearOffset -> PLAN_NEXT_HOP.
    expectWallStop(kWall1Facing, /*timedOut*/ true);

    // The FSM must run REANCHOR + dispatch the depart (no fault under strict OFF), then retire it.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Drive.isIdle(); ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
        cooperativeTick();
    }
    ASSERT_FALSE(Drive.isIdle());          // depart dispatched (interlude ran its first/only time)
    ASSERT_FALSE(PathTracker.hasFault());

    mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
    expectBlindHop(/*cm*/ kWall1AxleBeliefY - kWall1NodeY);

    // After WALL_DEPART -> PLAN_NEXT_HOP the FSM re-enters `_onPlanNextHop` with `cur == Wall1`
    // and the beyond-Wall1 goal still in flight. The one-shot latch (`_resetNode == Wall1`) must
    // hold: the interlude does NOT re-fire — instead the FSM plans the real `Wall1 -> position1`
    // hop and dispatches it (a pre-edge turn or the dead-reckon drive). Drive the next hop until
    // the chassis leaves Wall1; if the interlude had re-fired it would have re-entered WALL_STOP
    // and dispatched a fresh wallStop SQUARE turn back to facing 270 instead of departing Wall1.
    const uint8_t position1 = nodeByName("position1");
    for (int i = 0;
         i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() == wall1;
         ++i)
    {
        // Park the OUTBOUND edge heading (Wall1 -> position1, ~320.36 deg) so the second hop's
        // pre-turn settles; advance the encoder + done-bit so its dead-reckon move retires.
        const Routes::EdgeInfo& out = FieldMap::getEdge(wall1, position1);
        mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(out.heading_deg));
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }

    // The FSM advanced OFF Wall1 onto the next hop — the interlude did not re-fire (which would
    // have pinned the FSM at Wall1 in a fresh WALL_STOP). The reset latch held one-shot.
    ASSERT_EQ(Localization.getCurrentNode(), position1);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// Entry.
// ===========================================================================
int main()
{
    std::printf("INFO: PathTracker RESET-node arrival interlude — real FSM "
                "(WALL_STOP -> REANCHOR -> WALL_DEPART -> PLAN_NEXT_HOP); strict flag = %s\n",
                Config::WALL_NORMAL_STRICT_ENABLED ? "ON" : "OFF");

    // Preconditions (kept from the original file).
    {
        TEST_SCOPE("precondition: strict-OFF target + Wall1 is a RESET node with a reset record");

        // This target compiles against the shipped default (strict OFF) so the real reanchor
        // proceeds on the advisory / success cases without the live IMU heading gating it. The
        // strict-ON WALL_NOT_SQUARE reject is the sibling test_path_tracker_reset_arrival_strict.
        ASSERT_FALSE(Config::WALL_NORMAL_STRICT_ENABLED);

        // The fixture must actually have "Wall1" tagged RESET (and carry a reset record) for the
        // interlude to be exercised.
        const uint8_t wall1 = nodeByName("Wall1");
        ASSERT_TRUE(wall1 != FieldMap::NODE_NONE);
        ASSERT_TRUE(FieldMap::getNodeType(wall1) == Routes::NodeType::RESET);
        ASSERT_TRUE(FieldMap::getResetInfo(wall1) != nullptr);
    }

    test_TC_RA_route_inbound_blind_hop_into_reset_node();
    test_TC_RA1_advisory_timeout_continues_clears_offset();
    test_TC_RA2_clean_stall_success_clears_offset();
    test_TC_RA3_interlude_fires_exactly_once();

    if (wro_test::failCounter() == 0)
    {
        std::printf("PATH TRACKER RESET ARRIVAL OK\n");
    }
    TEST_REPORT();
}
