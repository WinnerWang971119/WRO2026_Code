/**
 * @file test_path_tracker_bidirectional.cpp
 * @brief Host-link test for the bidirectional line-following surface, driving the
 *        REAL `PathTracker` + `Drive` FSMs through the shared cooperative-tick
 *        harness (issue #106, T5e).
 *
 * @details
 * Rewritten from the original mirror-slice test. Two things changed:
 *
 *   1. The `mission_replica` namespace (a hand-copied `MissionClass` Step queue +
 *      `setRouteDirection` / `setArrivalDirection` modifier guard) is DELETED.
 *      Mission's modifier behaviour is asserted against the REAL `Mission.cpp` in
 *      `test_mission_dispatch` (TC21-TC23: write-through to `PathTracker.goTo` at
 *      execute, no-op off a non-GOTO step / empty queue, and last-GOTO-only
 *      targeting), so re-mirroring it here is redundant drift risk.
 *
 *   2. The REVERSE_UNAVAILABLE / AUTO-toggle cases that USED to be exercised on a
 *      synthetic `replica_resolveDirection` are now split:
 *        - TC-BD1 (REVERSE_UNAVAILABLE guard) and TC-BD3 (AUTO toggle) call the
 *          REAL frozen `*Policy.h` resolvers DIRECTLY — `reverseUnavailable` and
 *          `resolveDirection` from `DirectionPolicy.h`. These are the SAME symbols
 *          `PathTracker.cpp`'s pre-turn guard (PathTracker.cpp:1569-1573) and
 *          `_onPlanNextHop`'s direction resolution (PathTracker.cpp:1514) call, so
 *          they are NOT mirrors — they exercise production logic by include, with
 *          no replicated decision (closing the drift surface the old replicas had).
 *        - TC-BD2 (rear LINE allowed on a 2-board chassis) drives the REAL
 *          `PathTracker` FSM over the rear-override hop `Inter1 -> Spur`.
 *
 * THE FSM HOP (`fixtures/routes_fixture.h`): `Inter1 -> Spur`. The fixture edge
 * carries `DirectionMode::FORCE_REVERSE` (the `direction="rear"` EDGES override):
 *   `EdgeType::LINE, distance 40 cm, heading 90 deg, direction_override FORCE_REVERSE`.
 * `Spur` is a LINE node (not a projection), so a rear `goTo(Spur)` from `Inter1`:
 *   - resolves `Direction::REAR` (Step 1 of the precedence table binds on the
 *     per-edge FORCE_REVERSE override, regardless of the AUTO kill-switch);
 *   - body-target heading = (90 + 180) mod 360 = 270 deg, so the chassis pre-turns
 *     from its seated app-yaw 0 to face 270 (rear board ends up pointing along the
 *     90 deg edge toward `Spur`);
 *   - on a 2-board chassis (`LINE_BOARD_COUNT == 2`, the default) the rear LINE
 *     dispatch is SERVED — no REVERSE_UNAVAILABLE fault — so `_dispatchEdgeAfterTurn`
 *     enters the LINE branch and arms `LineFollower.setReverse(true)` (PathTracker.cpp:1941).
 *
 * Built TWICE by CMake — once with `-DBIDIRECTIONAL_AUTO_ENABLED_DEFAULT=true`
 * (`test_path_tracker_bidirectional_on`) and once with `=false`
 * (`test_path_tracker_bidirectional_off`). Both binaries run the SAME hop; the
 * AUTO-toggle assertion (TC-BD3) branches on the COMPILED
 * `Config::BIDIRECTIONAL_AUTO_ENABLED` constexpr (resolved from that flag in
 * `ConfigNav.h`). The FSM hop (TC-BD2) behaves identically in both builds — the
 * FORCE_REVERSE override binds before the AUTO branch is consulted — which is the
 * point: an operator-forced rear hop is independent of the kill-switch.
 *
 * REVERSE_UNAVAILABLE on a 1-board chassis (TC-BD1) is asserted at the
 * frozen-resolver level, NOT through a 1-board real-FSM build: `Config::LINE_BOARD_COUNT`
 * is a compile-time `constexpr` (= 2) with no per-target override hook, and adding
 * one would be a production edit. The direct `reverseUnavailable(REAR, LINE, 1)`
 * call exercises the exact predicate the FSM guard calls — strictly test-only.
 */

#include "test_path_tracker_harness.h"

#include "src/nav/DirectionPolicy.h"    // real frozen resolvers: resolveDirection + reverseUnavailable.
#include "src/nav/NavTypes.h"           // Direction enum.
#include "src/mission/PathTracker.h"        // PathTrackerClass::Fault + FaultInfo (the real enum the guard latches).
#include "src/nav/routes.h"             // Routes::DirectionMode / EdgeType / TurnMode.

#include <cstdio>
#include <stdint.h>

// ===========================================================================
// TC-BD1 — REVERSE_UNAVAILABLE guard, asserted at the FROZEN-RESOLVER level.
//
// The load-bearing predicate is the shared `reverseUnavailable(...)` from
// DirectionPolicy.h — the SAME symbol `PathTracker.cpp`'s pre-turn guard
// (PathTracker.cpp:1569-1573) and its defensive post-turn guard
// (PathTracker.cpp:1842) both call. Calling it here is NOT a mirror: there is no
// replicated condition that could drift. The thin wrapper below assembles the
// `FaultInfo` the production code would latch when the guard fires
// (`_enterFault(Fault::REVERSE_UNAVAILABLE, cur, nxt)`), and an empty FaultInfo
// (`kind == Fault::NONE`) when the guard does not fire (the dispatcher would have
// proceeded into the LINE / BLIND branch).
// ===========================================================================

/// Exercises the REVERSE_UNAVAILABLE guard via the REAL frozen
/// `reverseUnavailable(...)` predicate, returning the `FaultInfo` the production
/// dispatcher would latch. The predicate call is production logic (by include);
/// only the FaultInfo assembly is test glue, mirroring how
/// `_enterFault(Fault::REVERSE_UNAVAILABLE, cur, nxt)` latches when the guard
/// fires (issue #64 retired the standalone PROJECTION edge type, so the only
/// reverse-unservable edge type is `LINE`).
static PathTrackerClass::FaultInfo dispatchEdgeAfterTurnGuard(
    Direction        resolvedDirection,
    uint8_t          lineBoardCount,
    Routes::EdgeType edgeType,
    uint8_t          cur,
    uint8_t          nxt)
{
    PathTrackerClass::FaultInfo info{};
    if (reverseUnavailable(resolvedDirection, edgeType, lineBoardCount))
    {
        info.kind     = PathTrackerClass::Fault::REVERSE_UNAVAILABLE;
        info.fromNode = cur;
        info.destNode = nxt;
    }
    return info;
}

static void test_TC_BD1_reverse_unavailable_resolver_truth_table()
{
    TEST_SCOPE("TC-BD1: reverseUnavailable frozen resolver — REAR LINE 1-board faults; all else clears");

    constexpr uint8_t cur = 0;    // representative inbound node (FaultInfo.fromNode)
    constexpr uint8_t nxt = 1;    // representative next-hop node (FaultInfo.destNode)

    // (REAR, LINE, 1 board) → unservable: a REAR line-follow needs the rear board.
    // The FSM guard latches REVERSE_UNAVAILABLE with fromNode = cur, destNode = nxt.
    ASSERT_TRUE(reverseUnavailable(Direction::REAR, Routes::EdgeType::LINE, /*boards=*/1));
    {
        const PathTrackerClass::FaultInfo info = dispatchEdgeAfterTurnGuard(
            Direction::REAR, /*boards=*/1, Routes::EdgeType::LINE, cur, nxt);
        ASSERT_TRUE(info.kind == PathTrackerClass::Fault::REVERSE_UNAVAILABLE);
        ASSERT_EQ(info.fromNode, cur);
        ASSERT_EQ(info.destNode, nxt);
    }

    // (REAR, LINE, 2 boards) → SERVED: the rear board exists, so no fault — the
    // dispatcher falls through to the LINE branch's `setReverse(true)`.
    ASSERT_FALSE(reverseUnavailable(Direction::REAR, Routes::EdgeType::LINE, /*boards=*/2));
    ASSERT_TRUE(dispatchEdgeAfterTurnGuard(
                    Direction::REAR, /*boards=*/2, Routes::EdgeType::LINE, cur, nxt)
                    .kind == PathTrackerClass::Fault::NONE);

    // (REAR, BLIND, 1 board) → PERMITTED: dead-reckoned reverse needs only negative
    // PWM, no rear line sensor (PathTracker.cpp comment "BLIND REAR is permitted on
    // a 1-board chassis"). The guard keys on the LINE/BLIND edge type, not the dest.
    ASSERT_FALSE(reverseUnavailable(Direction::REAR, Routes::EdgeType::BLIND, /*boards=*/1));
    ASSERT_TRUE(dispatchEdgeAfterTurnGuard(
                    Direction::REAR, /*boards=*/1, Routes::EdgeType::BLIND, cur, nxt)
                    .kind == PathTrackerClass::Fault::NONE);

    // (FRONT, LINE, 1 board) → never trips: the guard is scoped to REAR.
    ASSERT_FALSE(reverseUnavailable(Direction::FRONT, Routes::EdgeType::LINE, /*boards=*/1));
    ASSERT_TRUE(dispatchEdgeAfterTurnGuard(
                    Direction::FRONT, /*boards=*/1, Routes::EdgeType::LINE, cur, nxt)
                    .kind == PathTrackerClass::Fault::NONE);
}

// ===========================================================================
// TC-BD2 — rear LINE hop is ALLOWED on a 2-board chassis, driven through the
//          REAL PathTracker + Drive FSMs over the fixture rear-override hop
//          `Inter1 -> Spur` (direction="rear" / FORCE_REVERSE).
// ===========================================================================

namespace
{
// ---------------------------------------------------------------------------
// Drive the real PathTracker from a clean world over `Inter1 -> Spur` until the
// FSM has dispatched the rear LINE hop (LineFollower freshly armed for the line
// follow, `setReverse(true)`), with no fault. Returns with the FSM in LINE_FOLLOW.
//
// Sequence the real FSM runs (PathTracker.cpp):
//   IDLE -> DEQUEUE (pop goal, offset~0 return-gate -> PLAN_NEXT_HOP)
//        -> PLAN_NEXT_HOP: resolveDirection sees the per-edge FORCE_REVERSE override
//           (Step 1 binds) -> REAR; body-target = (90 + 180) mod 360 = 270 deg.
//           The early reverse guard does NOT fire (LINE_BOARD_COUNT == 2). The
//           bodyTarget (270) differs from the seated app-yaw (0) by 90 deg, so
//           `_dispatchTurnIfNeeded` dispatches a pre-edge turn -> TURNING.
//        -> TURNING: `expectPreTurn(270, ...)` parks the corrected IMU read at 270
//           and ticks until `Drive.isIdle()` -> `_dispatchEdgeAfterTurn`.
//        -> _dispatchEdgeAfterTurn: LINE edge, dest `Spur` is a LINE node, so the
//           LINE -> line arm fires with `.setReverse(reverse == true)` -> LINE_FOLLOW.
// ---------------------------------------------------------------------------
void driveToRearLineFollow()
{
    resetWorld();
    seatAt("Inter1");    // bind world frame @ app-yaw 0 + declare Localization at Inter1

    // `Spur` is the FINAL (and only) hop from `Inter1`; `.async()` so the harness
    // drives the cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(nodeByName("Spur")).async();

    // Tick until the pre-edge turn is in flight (Drive busy). The turn cannot
    // auto-complete: the seated yaw (0) is 90 deg off the 270 deg body target, so
    // `_updateTurnProfile` keeps driving until `expectPreTurn` parks the yaw.
    bool turnInFlight = false;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (!Drive.isIdle())
        {
            turnInFlight = true;
            break;
        }
        // Guard against the no-turn path: if the FSM somehow armed the follower
        // without a pre-turn, stop ticking and let the assertions below catch it.
        if (LineFollower.isEnabled())
        {
            break;
        }
    }
    ASSERT_TRUE(turnInFlight);    // the rear hop MUST pre-turn 90 deg before dispatch

    // Drive the pre-edge turn to the 270 deg body target (app frame). The wheel byte
    // is the pipeline default (COUNTER_ROT) — `Inter1` is not a pivot node this hop.
    expectPreTurn(/*appYawTarget=*/270, /*wheelByte=*/(uint8_t) Routes::TurnMode::COUNTER_ROT);

    // After the turn retires, TURNING sees `Drive.isIdle()` and dispatches the edge.
    // Tick until the LINE -> line arm enables the follower (the LINE_FOLLOW entry).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode)
        {
            return;    // LINE_FOLLOW reached, follower armed for the rear line follow
        }
        if (PathTracker.hasFault())
        {
            return;    // a fault aborts the dispatch — the assertions below report it
        }
    }
    ASSERT_TRUE(false);    // budget exhausted: the FSM never reached LINE_FOLLOW
}
}    // namespace

// Routing guard — pin the hop so a future fixture re-weight or a dropped
// rear-override edge fails LOUDLY rather than silently changing what runs.
static void test_TC_BD2_routing_and_override_are_forced()
{
    TEST_SCOPE("TC-BD2 routing: goTo(Spur) from Inter1 is the direct rear-override LINE hop");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t spur   = nodeByName("Spur");

    // The fixture must resolve both names (the stable-name contract).
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(spur != FieldMap::NODE_NONE);

    // Dijkstra's first hop Inter1 -> Spur is the direct edge to Spur (it is the goal).
    ASSERT_EQ(routedFirstHop(inter1, spur), spur);

    // The edge under test is a LINE edge carrying the FORCE_REVERSE (direction="rear")
    // override and the 90 deg heading the body-flip rotates 180 deg from.
    const Routes::EdgeInfo& edge = FieldMap::getEdge(inter1, spur);
    ASSERT_TRUE(edge.type == Routes::EdgeType::LINE);
    ASSERT_TRUE(edge.direction_override == Routes::DirectionMode::FORCE_REVERSE);

    // The destination is a LINE node (not a projection) — the rear hop takes the
    // LINE -> line follow branch, the one this test asserts `setReverse(true)` on.
    ASSERT_TRUE(FieldMap::getNodeType(spur) == Routes::NodeType::LINE);
}

static void test_TC_BD2_rear_line_allowed_on_2board()
{
    TEST_SCOPE("TC-BD2: rear LINE Inter1->Spur dispatches on 2-board — no fault, follower armed reverse");

    const uint8_t inter1 = nodeByName("Inter1");

    // The real FSM runs on the default 2-board config (LINE_BOARD_COUNT == 2), so a
    // REAR LINE hop is SERVED — the early/late reverse guards must NOT fire.
    ASSERT_EQ(Config::LINE_BOARD_COUNT, (uint8_t) 2);

    driveToRearLineFollow();

    // The rear LINE dispatch proceeded: no REVERSE_UNAVAILABLE (nor any other) fault.
    ASSERT_FALSE(PathTracker.hasFault());

    // Still at the origin node — the follow has not yet crossed a junction to advance.
    ASSERT_EQ(Localization.getCurrentNode(), inter1);

    // The LINE -> line arm fired with the rear board selected: the follower is enabled
    // for a NON-overshoot line follow and `setReverse(true)` was applied (the observable
    // proving the rear dispatch path was taken, PathTracker.cpp:1941 `.setReverse(reverse)`).
    // `lastReverse` is the recording field the BUILDER's `setReverse` writes — the FSM
    // arms via the `setEnabled(true)...setReverse(reverse)` chain, not the direct
    // `setReverse(bool)` setter, so `lastReverse` (not `getReverse()`) is the capture point.
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);
    ASSERT_TRUE(LineFollower.lastReverse);
}

// ===========================================================================
// TC-BD3 — the AUTO kill-switch flips the resolved direction on the SAME hop
//          heading, asserted via the REAL frozen `resolveDirection` resolver.
//
// `resolveDirection` is the SAME symbol `_onPlanNextHop` calls (PathTracker.cpp:1514).
// The fixture supplies a real LINE edge whose heading exceeds the turn-saving
// threshold (BIDIRECTIONAL_TURN_THRESHOLD_DEG == 90) and whose direction_override is
// DEFAULT, so the AUTO branch (Step 5) is reached: with the kill-switch ON it picks
// the turn-saving REAR; with it OFF it short-circuits to FRONT. Branching on the
// compiled `Config::BIDIRECTIONAL_AUTO_ENABLED` proves the macro genuinely controls
// the resolver the real FSM runs.
// ===========================================================================

static void test_TC_BD3_macro_propagates_into_runtime_constant()
{
    TEST_SCOPE("TC-BD3: BIDIRECTIONAL_AUTO_ENABLED_DEFAULT macro propagates into the runtime constant");

    // The compile-time override knob MUST land on the public constant the FSM reads.
    ASSERT_EQ(Config::BIDIRECTIONAL_AUTO_ENABLED, (bool) BIDIRECTIONAL_AUTO_ENABLED_DEFAULT);
}

static void test_TC_BD3_auto_toggle_flips_resolved_direction()
{
    if (Config::BIDIRECTIONAL_AUTO_ENABLED)
    {
        TEST_SCOPE("TC-BD3: kill-switch ON — AUTO picks turn-saving REAR on a >90 deg fixture heading");
    }
    else
    {
        TEST_SCOPE("TC-BD3: kill-switch OFF — AUTO defeated, resolves FRONT on the SAME heading");
    }

    // Pull a REAL fixture LINE edge whose heading exceeds the 90 deg turn-saving
    // threshold and whose per-edge direction is DEFAULT, so the AUTO branch (Step 5)
    // of `resolveDirection` is the one reached. `Inter3 -> Inter2` is heading 180 deg,
    // DEFAULT direction (the body delta from a 0 deg current heading is +180 deg, well
    // past the 90 deg threshold).
    const uint8_t inter3 = nodeByName("Inter3");
    const uint8_t inter2 = nodeByName("Inter2");
    ASSERT_TRUE(inter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE);

    const Routes::EdgeInfo& edge = FieldMap::getEdge(inter3, inter2);
    ASSERT_TRUE(edge.type == Routes::EdgeType::LINE);
    ASSERT_TRUE(edge.direction_override == Routes::DirectionMode::DEFAULT);
    // Sanity: the heading delta vs a 0 deg current heading must exceed the threshold,
    // otherwise neither build would resolve REAR and the toggle would not flip.
    ASSERT_TRUE(fabsf(wrapDeltaDeg(edge.heading_deg - 0.0f))
                > Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG);

    // Resolve the direction through the REAL frozen resolver with a bare-goTo() input
    // set (no per-edge override, no pipeline default, AUTO at the builder), exactly as
    // `_onPlanNextHop` feeds it for an un-modified hop. The ONLY varying input is the
    // compiled kill-switch.
    const Direction resolved = resolveDirection(
        /*edgeOverride*/             edge.direction_override,             // DEFAULT
        /*pipelineDefault*/          Routes::PIPELINE_CONFIG.direction,   // DEFAULT (fixture pipeline)
        /*builderArrival*/           Direction::AUTO,
        /*builderRoute*/             Direction::AUTO,
        /*currentHeading*/           0.0f,
        /*edgeHeading*/              edge.heading_deg,                    // 180 deg
        /*isLastHop*/                false,
        /*bidirectionalAutoEnabled*/ Config::BIDIRECTIONAL_AUTO_ENABLED,
        /*lineBoardCount*/           Config::LINE_BOARD_COUNT);           // 2 (auto gate satisfied)

    if (Config::BIDIRECTIONAL_AUTO_ENABLED)
    {
        // Kill-switch ON: a >90 deg body delta on a 2-board chassis → turn-saving REAR.
        ASSERT_TRUE(resolved == Direction::REAR);
    }
    else
    {
        // Kill-switch OFF: the AUTO branch short-circuits to FRONT regardless of geometry.
        ASSERT_TRUE(resolved == Direction::FRONT);
    }
}

// ---------------------------------------------------------------------------
// Announce the compiled build mode for ctest log clarity.
// ---------------------------------------------------------------------------

static void test_announce_build_mode()
{
    TEST_SCOPE("build-mode announcement");
    if (Config::BIDIRECTIONAL_AUTO_ENABLED)
    {
        std::printf("INFO: BIDIRECTIONAL_AUTO_ENABLED == true (kill-switch ON build)\n");
    }
    else
    {
        std::printf("INFO: BIDIRECTIONAL_AUTO_ENABLED == false (kill-switch OFF build)\n");
    }
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_announce_build_mode();

    // TC-BD1 — REVERSE_UNAVAILABLE guard at the frozen-resolver level (KEPT, not a mirror).
    test_TC_BD1_reverse_unavailable_resolver_truth_table();

    // TC-BD2 — rear LINE allowed on a 2-board chassis, through the real FSM.
    test_TC_BD2_routing_and_override_are_forced();
    test_TC_BD2_rear_line_allowed_on_2board();

    // TC-BD3 — AUTO kill-switch flips the resolved direction (real frozen resolver).
    test_TC_BD3_macro_propagates_into_runtime_constant();
    test_TC_BD3_auto_toggle_flips_resolved_direction();

    TEST_REPORT();
}
