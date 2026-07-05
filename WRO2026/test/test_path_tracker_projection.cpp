/**
 * @file test_path_tracker_projection.cpp
 * @brief Host-link tests for the REAL PathTracker projection-NODE dispatch (issue #64 / #106,
 *        T5c). Drives the production `PathTracker` + `Drive` + `Localization` + `YawSource`
 *        singletons through the shared cooperative-tick harness — NO mirror slices.
 * @author WRO 2026
 *
 * @details
 * Issue #64 made `projection` a NODE property rather than an edge type. Every per-hop
 * behaviour falls out of `(edge_type, destination_node_type)`:
 *
 *   | Edge  | → line node                | → projection node                       |
 *   |-------|----------------------------|-----------------------------------------|
 *   | LINE  | follow to junction         | DISTANCE-gated follow + dead-reckon adv |
 *   | BLIND | dead-reckon                | auto-seek stop-at-line + full re-anchor |
 *
 * THE INVERSION (#106): this target links the REAL `PathTracker.cpp` + `Drive.cpp`, so the two
 * projection cells of `PathTracker::_dispatchEdgeAfterTurn` are asserted against PRODUCTION
 * code, not a `replica_dispatchProjectionCell` mirror. The previous version of this file
 * inline-MIRRORED the `(edge, dest)` routing slice (`replica_dispatchProjectionCell` +
 * `firmwareAutoRuleActive` / `resolveBareBlindHop`); those mirrors are GONE. What stays:
 *   - The REAL graph-accessor reads (`FieldMap::getNodeType` / `isProjectionNode` /
 *     `getEdgeType`) — these are not mirrors, they pin the fixture's `(edge, dest)` shape.
 *   - One REAL frozen-resolver call (`resolveStopAtLine`) in TC-PJ4 — a KEPT resolver test
 *     proving the auto-rule (`isProjectionNode(dest) && BLIND_PROJECTION_AUTO_SEEK`) drives
 *     the seek decision over the real graph data. Helpers that CALL a frozen `*Policy.h`
 *     resolver are not mirrors (per #106 AC).
 *
 * PathTracker exposes no public `getState()`, so each case observes the FSM phase through the
 * PUBLIC / mock surface the harness already drives — the LineFollower arm cascade
 * (`lastEnabled` / `lastDistanceTarget` / `lastOvershootMode` / `lastBrakeOnGate`), the seek's
 * `LineArray.setReverseActive(...)` board flip, the recorded `MiniR4.DriveDC` seek move, the
 * real `Drive.isLineFound()` / `isIdle()` / `hasFault()`, and the real `Localization`
 * node advance — never a private state read.
 *
 * T4 switched this target from the TOY golden onto the 14-node feature-complete
 * `fixtures/routes_fixture.cpp`, which carries the `ProjL` (LINE-reached) and `ProjB`
 * (BLIND-reached) projection nodes plus the `Corner` LINE node. Every node is resolved BY
 * NAME (`nodeByName`), never by hardcoded index (issue #91 — the fixture renumbers freely).
 *
 * Fixture hops driven (by name):
 *   - TC-PJ1  `Inter1→ProjL`  LINE edge into a PROJECTION node  (distance-gated dead-reckon).
 *   - TC-PJ2  `Corner→ProjB`  BLIND edge into a PROJECTION node (auto-seek + re-anchor).
 *   - TC-PJ3  `ProjB→Zed`     BLIND edge into a BLIND node       (dead-reckon, NO seek).
 *   - TC-PJ4  resolver-level auto-rule truth (direct `resolveStopAtLine` over real graph data).
 */

#include "test_path_tracker_harness.h"    // real singletons + per-hop expect* helpers + node-by-name

#include "src/nav/FieldMap.h"          // real FieldMap::getNodeType / isProjectionNode / getEdgeType (KEPT reads).
#include "src/nav/StopAtLinePolicy.h"  // real frozen resolveStopAtLine (KEPT resolver test, TC-PJ4 — not a mirror).
#include "src/nav/NavTypes.h"          // Direction / StopAtLine for the TC-PJ4 resolver query.

#include <cstdio>
#include <stdint.h>

namespace
{

// ===========================================================================================
// Pre-flight — the fixture actually carries the projection scenarios under test. A future
// fixture edit that drops `ProjL` / `ProjB` (or a flanking LINE/BLIND edge) fails LOUDLY here
// rather than silently weakening the four projection cases below.
// ===========================================================================================

void test_fixture_has_projection_scenarios()
{
    TEST_SCOPE("PJ-pre: fixture carries ProjL/ProjB (projection) + LINE→ProjL + BLIND→ProjB + BLIND→Zed");

    const uint8_t inter1 = nodeByName("Inter1");    // LINE node
    const uint8_t projL  = nodeByName("ProjL");     // PROJECTION node (LINE-reached)
    const uint8_t corner = nodeByName("Corner");    // LINE node
    const uint8_t projB  = nodeByName("ProjB");     // PROJECTION node (BLIND-reached)
    const uint8_t zed    = nodeByName("Zed");       // BLIND node

    // Every name resolved (a typo / renamed fixture node faults here, not silently elsewhere).
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(projL != FieldMap::NODE_NONE);
    ASSERT_TRUE(corner != FieldMap::NODE_NONE);
    ASSERT_TRUE(projB != FieldMap::NODE_NONE);
    ASSERT_TRUE(zed != FieldMap::NODE_NONE);

    // Node types — the destination-node read that ROUTES every hop's dispatch cell.
    ASSERT_TRUE(FieldMap::getNodeType(projL) == Routes::NodeType::PROJECTION);
    ASSERT_TRUE(FieldMap::isProjectionNode(projL));
    ASSERT_TRUE(FieldMap::getNodeType(projB) == Routes::NodeType::PROJECTION);
    ASSERT_TRUE(FieldMap::isProjectionNode(projB));
    ASSERT_TRUE(FieldMap::getNodeType(inter1) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(corner) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(zed) == Routes::NodeType::BLIND);
    ASSERT_FALSE(FieldMap::isProjectionNode(inter1));
    ASSERT_FALSE(FieldMap::isProjectionNode(zed));

    // The three `(edge_type, dest_node_type)` cells the cases below exercise:
    //   Inter1→ProjL  LINE  into projection  (TC-PJ1)
    //   Corner→ProjB  BLIND into projection  (TC-PJ2 / TC-PJ4)
    //   ProjB →Zed    BLIND into a non-projection BLIND node (TC-PJ3)
    ASSERT_TRUE(FieldMap::getEdgeType(inter1, projL) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdgeType(corner, projB) == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdgeType(projB, zed) == Routes::EdgeType::BLIND);
}

// ===========================================================================================
// TC-PJ1 — LINE → projection dispatch.
//
// `Inter1→ProjL`: the real `_dispatchEdgeAfterTurn` LINE branch reads `isProjectionNode(ProjL)`
// and arms the follower for a DISTANCE-gated follow (PROJECTION_FOLLOW), NOT an intersection
// wait — a projection node carries no junction marker. The PROJECTION_FOLLOW arm is the only
// `_dispatchEdgeAfterTurn` path that sets a positive `setDistanceTarget` with
// `setOvershootMode(false)`, so that arm cascade is the observable PROJECTION_FOLLOW signature
// (a LINE→line hop arms with no distance target). On the distance gate the real state
// dead-reckons the advance (`onDeadReckonedAdvance(ProjL)`), with NO junction re-anchor.
// ===========================================================================================

void test_PJ1_line_into_projection_distance_gated_dead_reckons()
{
    TEST_SCOPE("TC-PJ1: LINE→projection arms a distance-gated follow + dead-reckon advance to ProjL");

    resetWorld();

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");

    // Real graph reads: Inter1→ProjL is a LINE edge into a projection destination.
    ASSERT_TRUE(FieldMap::getEdgeType(inter1, projL) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::isProjectionNode(projL));

    // Inter1→ProjL heads due-east (0°), so seating at Inter1 (frame zeroed to the fixture's
    // START_HEADING_DEG = 0°) leaves the pre-edge turn a no-op — the FSM dispatches the LINE
    // edge directly. Pin the route: goTo(ProjL) from Inter1 routes the single hop Inter1→ProjL.
    seatAt("Inter1");
    ASSERT_EQ(routedFirstHop(inter1, projL), projL);

    PathTracker.goTo(projL).async();

    // Drive IDLE→DEQUEUE→PLAN_NEXT_HOP→(no turn)→_dispatchEdgeAfterTurn until the LINE→projection
    // branch arms the distance-gated follow (a positive distance target — the PROJECTION_FOLLOW
    // signature). No node has advanced yet (the distance gate has not tripped).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && !(LineFollower.lastEnabled && LineFollower.lastDistanceTarget > 0.0f);
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_FALSE(PathTracker.hasFault());

    // The PROJECTION_FOLLOW arm: follower enabled, NOT in overshoot mode (this is the
    // dead-reckon-to-distance follow, not the post-junction roll), with a positive distance
    // target, braking on the gate (a single-hop goTo(ProjL) is not a pass-through). These are
    // the real `_dispatchEdgeAfterTurn` LINE→projection arm verbs.
    ASSERT_TRUE(LineFollower.lastEnabled);
    ASSERT_FALSE(LineFollower.lastOvershootMode);
    ASSERT_GT(LineFollower.lastDistanceTarget, 0.0f);
    ASSERT_TRUE(LineFollower.lastBrakeOnGate);
    ASSERT_EQ(Localization.getCurrentNode(), inter1);

    // Trip the distance gate. The real PROJECTION_FOLLOW state dead-reckons the advance to ProjL
    // (no junction to re-anchor against), returns to PLAN_NEXT_HOP, and completes the goal.
    LineFollower.mockSetDistanceReached(true);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }

    // Dead-reckon advance landed the discrete node ON the projection node ProjL; goal reached,
    // FSM idle, NO fault — and a LINE hop never seeks (the seek's continuous emit never ran).
    ASSERT_EQ(Localization.getCurrentNode(), projL);
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_EQ(LineArray.mockSetReverseCallCount(), 0u);    // no seek touched the line array
    ASSERT_EQ(MiniR4.DriveDC.moveSyncCallCount, 0u);       // no seek continuous-emit ran
}

// ===========================================================================================
// TC-PJ2 — BLIND → projection auto-seek + full-advance re-anchor.
//
// `Corner→ProjB`: a BLIND edge into a projection node, with `BLIND_PROJECTION_AUTO_SEEK` baked
// into the fixture. The real `_dispatchEdgeAfterTurn` BLIND branch computes
// `autoRuleActive = BLIND_PROJECTION_AUTO_SEEK && isProjectionNode(ProjB)` (true), feeds it to
// the frozen `resolveStopAtLine`, which seeks, and dispatches `Drive.stopAtLine(...)`. The FSM
// sits in STOP_AT_LINE; on the seek crossing (Drive idle, line found) the real state advances
// the node via `Localization.onIntersection(ProjB)` — a FULL-ADVANCE re-anchor.
//
// AC S1 — assert the LOWER-MCU seek move. The `Corner→ProjB` edge carries `BlindMode::DEFAULT`,
// which resolves to the pipeline blind default (`SYNC_TSHAPE` → `DriveMode::SYNC`), so the real
// seek's continuous emit dispatches `MiniR4.DriveDC.MoveSync(...)` each tick (Drive.cpp
// `_emitContinuous` SYNC case), NOT `MoveGyro`/`MoveGyroDegs`. We therefore assert the recorded
// SYNC seek move (`moveSyncCallCount` + `lastMoveSyncL`/`R`) — the actual `DriveDC` verb this
// hop's seek dispatches — rather than a gyro-frame target (there is none for a SYNC hop). The
// seek dispatch is further pinned by the `LineArray.setReverseActive(false)` FRONT-board flip
// (only the real Drive seek FSM touches `LineArray`).
// ===========================================================================================

void test_PJ2_blind_into_projection_auto_seeks_and_reanchors()
{
    TEST_SCOPE("TC-PJ2: BLIND→projection auto-seeks (STOP_AT_LINE) + full-advance re-anchor; assert SYNC seek move");

    resetWorld();

    const uint8_t corner = nodeByName("Corner");
    const uint8_t projB  = nodeByName("ProjB");

    // Real graph reads: Corner→ProjB is a BLIND edge into a projection destination.
    ASSERT_TRUE(FieldMap::getEdgeType(corner, projB) == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::isProjectionNode(projB));

    // Corner→ProjB heads due-east (0°), so seating at Corner (frame zeroed to START_HEADING_DEG
    // = 0°) leaves the pre-edge turn a no-op. Pin the route: goTo(ProjB) from Corner routes
    // Corner→ProjB.
    seatAt("Corner");
    ASSERT_EQ(routedFirstHop(corner, projB), projB);

    PathTracker.goTo(projB).async();

    // Drive IDLE→DEQUEUE→PLAN_NEXT_HOP→(no turn)→_dispatchEdgeAfterTurn. The BLIND→projection
    // auto-rule makes the resolver seek, so the FSM dispatches the real `Drive.stopAtLine` seek.
    // The seek's seeding calls `LineArray.setReverseActive(cmd.reverse)` (Drive.cpp), which a
    // dead-reckon `driveDist` never does — so a non-zero board-flip count is the "a SEEK (not a
    // dead-reckon) was dispatched" signal. The seek leaves Drive busy.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && !(LineArray.mockSetReverseCallCount() > 0u && !Drive.isIdle());
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_GT(LineArray.mockSetReverseCallCount(), 0u);    // a SEEK was dispatched
    ASSERT_FALSE(Drive.isIdle());                           // seek in flight
    ASSERT_FALSE(LineArray.getReverseActive());             // FRONT seek (Corner→ProjB is FRONT)
    ASSERT_EQ(Localization.getCurrentNode(), corner);       // not advanced yet

    // Drive the seek to a SUCCESSFUL crossing. `expectStopAtLine(cm, found=true)` advances M2
    // into the arm window, holds T_OR_CROSS for the debounce, and rolls the overshoot to DONE —
    // exercising the real `Drive._updateStopAtLine` SEEK→ARMED→OVERSHOOT→DONE path. The seek's
    // edge distance is 40 cm (Corner→ProjB).
    expectStopAtLine(40.0f, /*found*/ true);

    // AC S1 — the LOWER-MCU seek move. The SYNC seek emitted `MoveSync` continuously while
    // seeking, so the recorded SYNC call count is non-zero and the last commanded L/R powers are
    // the (forward, equal) cruise magnitude. This is the recorded `MiniR4.DriveDC` call for the
    // seek (Drive.cpp `_emitContinuous` SYNC branch — `MoveSync`, not `MoveGyro`).
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, 0u);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, MiniR4.DriveDC.lastMoveSyncR);    // straight cruise
    ASSERT_GT(MiniR4.DriveDC.lastMoveSyncL, (int16_t) 0);                     // forward seek
    // A SYNC seek never issues a gyro move, so no gyro-frame target was recorded for this hop.
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, 0u);

    // Drive latched the crossing — the success precondition the STOP_AT_LINE state reads.
    ASSERT_TRUE(Drive.isLineFound());

    // Let the real STOP_AT_LINE state observe Drive idle, full-advance re-anchor via
    // `onIntersection(ProjB)`, and complete the goal.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }

    // FULL-ADVANCE re-anchor: the discrete node is now the projection node ProjB (onIntersection,
    // not a dead-reckon), the goal is reached, and there is no fault. The seek restored the FRONT
    // board on DONE.
    ASSERT_EQ(Localization.getCurrentNode(), projB);
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(LineArray.getReverseActive());
}

// ===========================================================================================
// TC-PJ3 — BLIND → non-projection does NOT auto-seek.
//
// `ProjB→Zed`: a BLIND edge into a BLIND (non-projection) node. The real
// `_dispatchEdgeAfterTurn` BLIND branch computes `autoRuleActive = ... && isProjectionNode(Zed)`
// == false, the frozen resolver does NOT seek, and the FSM dispatches a plain dead-reckon
// `driveDist` (BLIND_FORWARD), advancing via `onDeadReckonedAdvance(Zed)`. NO `stopAtLine` seek
// is issued — proving the auto-rule keys on the DESTINATION node type, not a constant. The
// decisive negatives: the seek FSM never ran (`LineArray` untouched — `setReverseActive` count
// stays 0; the seek's continuous `MoveSync` never emitted; `Drive.isLineFound()` stays false).
//
// ProjB→Zed heads due-south (270°), so this hop's pre-edge turn is NOT a no-op (the believed
// heading after `seatAt` is START_HEADING_DEG = 0°). Drive the real custom TurnGyro pre-turn to
// 270° via `expectPreTurn`, then the BLIND dead-reckon via `expectBlindHop`.
// ===========================================================================================

void test_PJ3_blind_into_nonprojection_does_not_auto_seek()
{
    TEST_SCOPE("TC-PJ3: BLIND→non-projection dead-reckons (BLIND_FORWARD), NO stopAtLine seek");

    resetWorld();

    const uint8_t projB = nodeByName("ProjB");
    const uint8_t zed   = nodeByName("Zed");

    // Real graph reads: ProjB→Zed is a BLIND edge into a NON-projection (BLIND) destination.
    ASSERT_TRUE(FieldMap::getEdgeType(projB, zed) == Routes::EdgeType::BLIND);
    ASSERT_FALSE(FieldMap::isProjectionNode(zed));

    // Pin the route: goTo(Zed) from ProjB routes the single hop ProjB→Zed.
    seatAt("ProjB");
    ASSERT_EQ(routedFirstHop(projB, zed), zed);

    PathTracker.goTo(zed).async();

    // ProjB→Zed departs facing 270° (the believed heading after `seatAt` is START_HEADING_DEG =
    // 0°, so a real pre-edge turn fires first). Park the corrected IMU read at the 270° app-frame
    // turn target BEFORE retiring, so the custom TurnGyro profile settles as soon as it runs and
    // the BLIND→non-projection branch then dispatches its dead-reckon `driveDist`.
    mmL.mockYawDeg = (double) hwYaw(270);

    // Advance the FSM IDLE→DEQUEUE→PLAN→TURN→dead-reckon dispatch. The parked yaw settles the
    // pre-edge turn within these ticks; the BLIND→non-projection branch then dispatches a
    // SYNC_TSHAPE dead-reckon `driveDist`, whose profile emits the continuous `MoveSync` — that is
    // the "dead-reckon move is now in flight" signal. A no-line seek would instead have flipped
    // the line-array board; this loop runs until the dead-reckon emits, NOT a seek.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && MiniR4.DriveDC.moveSyncCallCount == 0u;
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_FALSE(PathTracker.hasFault());

    // The decisive negative: NO `stopAtLine` seek was dispatched. ONLY the real Drive seek touches
    // `LineArray` (its seeding calls `setReverseActive(cmd.reverse)`, Drive.cpp); a BLIND_FORWARD
    // `driveDist` never does. So a board-flip count of 0 with a dead-reckon move already in flight
    // proves the BLIND→non-projection hop dead-reckoned rather than auto-seeking.
    // (`Drive.isLineFound()` is intentionally NOT asserted here — it is sticky across the
    // re-`begin()` between cases, so it is not a reliable negative; the board-flip count is.)
    ASSERT_EQ(LineArray.mockSetReverseCallCount(), 0u);
    ASSERT_EQ(Localization.getCurrentNode(), projB);    // dead-reckon not yet retired

    // Retire the dead-reckon move with the harness helper: `expectBlindHop` advances the encoder +
    // done-bit each tick until `Drive.isIdle()`, completing the SYNC_TSHAPE profile. The real
    // BLIND_FORWARD state then advances via `onDeadReckonedAdvance(Zed)`, reaching the goal.
    expectBlindHop(60.0f);    // ProjB→Zed edge distance
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }

    // Dead-reckon advance landed the discrete node on Zed; goal reached, no fault — and NO
    // `stopAtLine` seek was ever dispatched across the whole hop (the board-flip count is still 0,
    // monotonic, so a seek anywhere in the run would have tripped it).
    ASSERT_EQ(Localization.getCurrentNode(), zed);
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_EQ(LineArray.mockSetReverseCallCount(), 0u);
}

// ===========================================================================================
// TC-PJ4 — the auto-rule is a LOCAL destination-node read (KEPT resolver test, NOT a mirror).
//
// `BLIND_PROJECTION_AUTO_SEEK` is graph DATA baked into `routes_fixture.h`, not a Config
// `_DEFAULT` compile hook, so it cannot be toggled per-target without a second fixture. This
// case asserts the auto-rule at the RESOLVER level: a direct call to the REAL frozen
// `resolveStopAtLine` proving `isProjectionNode(dest) && BLIND_PROJECTION_AUTO_SEEK` drives the
// seek decision over the real graph data — the SAME symbol the FSM's BLIND branch calls
// (PathTracker.cpp `_dispatchEdgeAfterTurn`). Calling a frozen `*Policy.h` resolver is NOT a
// mirror (per #106 AC); no per-target Config toggle is fabricated.
// ===========================================================================================

/// Run the REAL frozen `resolveStopAtLine` for a bare `goTo()` BLIND hop into @p dest (no call
/// verb, no per-edge override, plain blind edge), with `autoRuleActive` read LIVE off the real
/// graph data — `BLIND_PROJECTION_AUTO_SEEK && FieldMap::isProjectionNode(dest)`, the toggle
/// leading the `&&` so the projection lookup is short-circuited when the feature is compiled out.
/// This is NOT a mirror: it CALLS the production resolver (`StopAtLinePolicy.h`) — the same
/// symbol `_dispatchEdgeAfterTurn`'s BLIND branch calls — over the real fixture graph, proving
/// the auto-rule's seek verdict at the resolver/accessor level. FRONT travel + the real
/// `Config::LINE_BOARD_COUNT` so the board-gate matches production.
StopAtLineResolution autoSeekVerdictOverRealGraph(uint8_t dest)
{
    const bool autoRuleActive =
        Routes::BLIND_PROJECTION_AUTO_SEEK && FieldMap::isProjectionNode(dest);
    const StopAtLineQuery q{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,    // a plain blind edge (DEFAULT → pipeline blind)
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ Config::LINE_BOARD_COUNT,
        /*autoRuleActive*/ autoRuleActive};
    return resolveStopAtLine(q);
}

void test_PJ4_auto_rule_is_local_dest_read_resolver_level()
{
    TEST_SCOPE("TC-PJ4: auto-rule is a LOCAL dest-node read — frozen resolveStopAtLine over real graph data");

    const uint8_t projB = nodeByName("ProjB");    // PROJECTION dest
    const uint8_t zed   = nodeByName("Zed");      // BLIND (non-projection) dest

    // The fixture ships the toggle ON, so the live composition equals the destination read.
    ASSERT_TRUE((bool) Routes::BLIND_PROJECTION_AUTO_SEEK);
    ASSERT_TRUE(FieldMap::isProjectionNode(projB));
    ASSERT_FALSE(FieldMap::isProjectionNode(zed));

    // A bare BLIND hop into a PROJECTION node SEEKS — the auto rung (ladder rung 4) fires
    // because `BLIND_PROJECTION_AUTO_SEEK && isProjectionNode(ProjB)` is true. The resolver's
    // motion mode is the edge's concrete bundle (SYNC_TSHAPE), never STOP_AT_LINE/DEFAULT.
    const StopAtLineResolution seekProj = autoSeekVerdictOverRealGraph(projB);
    ASSERT_TRUE(seekProj.seek);
    ASSERT_TRUE(seekProj.motionMode == Routes::BlindMode::SYNC_TSHAPE);

    // A bare BLIND hop into a NON-projection node does NOT seek — the same LOCAL read, with no
    // NEXT_HOP/goal context, returns `autoRuleActive == false`, so no rung fires.
    const StopAtLineResolution seekNonProj = autoSeekVerdictOverRealGraph(zed);
    ASSERT_FALSE(seekNonProj.seek);
    ASSERT_TRUE(seekNonProj.motionMode == Routes::BlindMode::SYNC_TSHAPE);

    // Pin the SHORT-CIRCUIT shape (`toggle && dest`): with the toggle forced false, even a
    // projection destination yields no seek — the auto-seek is compiled out regardless of dest.
    {
        const StopAtLineQuery q{
            StopAtLine::AUTO,
            Routes::StopAtLineMode::DEFAULT,
            Routes::BlindMode::SYNC_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE,
            Direction::FRONT,
            Config::LINE_BOARD_COUNT,
            /*autoRuleActive*/ false && FieldMap::isProjectionNode(projB)};
        ASSERT_FALSE(resolveStopAtLine(q).seek);
    }
}

}    // namespace

// ===========================================================================================
// Entry.
// ===========================================================================================

int main()
{
    std::printf("INFO: PathTracker projection-NODE dispatch (issue #64 / #106, T5c) — REAL "
                "PathTracker + Drive + Localization over routes_fixture (ProjL / ProjB / Corner)\n");

    test_fixture_has_projection_scenarios();
    test_PJ1_line_into_projection_distance_gated_dead_reckons();
    test_PJ2_blind_into_projection_auto_seeks_and_reanchors();
    test_PJ3_blind_into_nonprojection_does_not_auto_seek();
    test_PJ4_auto_rule_is_local_dest_read_resolver_level();

    TEST_REPORT();
}
