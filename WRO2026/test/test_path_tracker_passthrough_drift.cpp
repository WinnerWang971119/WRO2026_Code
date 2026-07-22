/**
 * @file test_path_tracker_passthrough_drift.cpp
 * @brief Host-link regression for ADR-078 — "a LINE pass-through trusts the line, not the
 *        drifted gyro" (the `PATH_PASS_THROUGH_TRUST_LINE_ENABLED` toggle).
 *
 * @details
 * THE BUG (confirmed, root-caused): on a collinear `line(A)-line(B)-line(C)` pass-through the
 * LineFollower corrects lateral position only and never feeds the gyro, so IMU yaw drifts freely
 * over the A->B leg. Pass-through is CLASSIFIED at plan time on EDGE-table headings (correct,
 * latches `_passThrough`), but at dispatch time `PathTracker::_dispatchTurnIfNeeded` re-checks the
 * LIVE IMU heading vs the outbound B->C edge heading and fires a real gyro turn once the
 * accumulated drift exceeds `NAV_TURN_TOLERANCE_DEG` (0.5 deg) — stopping the chassis, disabling
 * the follower, and clearing `_passThrough`. The geometry-classified pass-through is broken into a
 * stop-and-turn at B.
 *
 * THE FIX (ADR-078): when the hop is a LINE pass-through continuation, `_dispatchTurnIfNeeded`
 * suppresses the pre-edge turn regardless of the live delta — the physical line is the heading
 * authority and the LineFollower's P+D washes out the residual over B->C. LINE_LOST (#146) remains
 * the separate safety net if the follower actually leaves the line. Making the guard return false
 * lands the FSM in the EXACT same continue-live state as a normal driftless pass-through.
 *
 * ONE SOURCE, TWO EXECUTABLES (CMakeLists.txt), same recipe as `test_path_tracker_passthrough.cpp`:
 *   - `test_path_tracker_passthrough_drift_on`  — default (`PATH_PASS_THROUGH_TRUST_LINE_ENABLED`
 *     true): THE FIX. A >tolerance drift injected at B does NOT fire a turn; the follower stays
 *     live and the FSM continues the pass-through.
 *   - `test_path_tracker_passthrough_drift_off` — `-DPATH_PASS_THROUGH_TRUST_LINE_ENABLED_DEFAULT
 *     =false`: reproduces THE BUG. The same drift fires a real pre-edge turn — the follower is
 *     disabled and the FSM enters TURNING. This is the proof the toggle + fix are load-bearing.
 *
 * Both binaries run the SAME scenario; the final assertion branches on the COMPILED
 * `Config::PATH_PASS_THROUGH_TRUST_LINE_ENABLED` constexpr (resolved from that flag in
 * `ConfigNav.h`) — the `test_path_tracker_passthrough.cpp` idiom, which also proves the compile
 * def actually reached the separately-compiled `PathTracker.cpp` TU.
 *
 * THE HOP (`fixtures/routes_fixture.h`): the collinear `line-line-line` triple
 * `Inter2 -> Inter3 -> Inter4` (all LINE edges, all LINE nodes, collinear at heading 0). A
 * `goTo(Inter4)` from `Inter2` walks `NEXT_HOP` through the mid junction `Inter3`; the bypass edge
 * `Inter2 -> Inter4` is `EdgeType::NONE`, so Dijkstra MUST traverse `Inter3` (pinned below). Drift
 * is injected AFTER `_passThrough` is latched live through LINE_OVERSHOOT, before the
 * `_onPlanNextHop` tick that runs the Inter3 -> Inter4 pre-edge turn decision.
 *
 * @note `Serial` / `EventTrace` / `mmL` are the capturing mocks force-included target-wide by
 *       CMake (`add_path_force_includes` + the HAL mock) — visible here with no extra `#include`,
 *       same as every sibling PathTracker+Drive real-link test.
 */

#include "test_path_tracker_harness.h"

#include "src/nav/NavTypes.h"    // wrapDeltaDeg — wrap-around-correct signed heading delta.

#include <cmath>

namespace
{
/// @brief Drift injected at B (deg). Strictly ABOVE `NAV_TURN_TOLERANCE_DEG` (10.0 after the
///        2026-07 retune) so the pre-edge yaw check WOULD fire a turn on the drifted gyro, and
///        strictly BELOW `LINE_HEADING_RESET_TOLERANCE_DEG` so no heading-reset gate could have
///        laundered it — the drift genuinely reaches `_dispatchTurnIfNeeded`. Production sets both
///        tolerances to 10.0, which collapses the valid window, so the two CMake targets raise the
///        heading-reset ceiling to 20.0 (`LINE_HEADING_RESET_TOLERANCE_DEG_DEFAULT`); this value is
///        pinned above the turn tolerance and below that raised ceiling. Matches the heading_reset
///        drift template (`test_path_tracker_heading_reset.cpp`).
constexpr float kDriftDeg = Config::NAV_TURN_TOLERANCE_DEG + 3.0f;

// ---------------------------------------------------------------------------
// Drive the real FSM from a clean world through the FIRST hop `Inter2 -> Inter3` of
// `goTo(Inter4)` to the arrival AT `Inter3`, with `_passThrough` latched live through
// LINE_OVERSHOOT (the follower kept enabled, no brake). Mirrors
// `test_path_tracker_heading_reset.cpp::driveToLineJunctionPassThroughArrival` — the SAME
// pass-through arm proof (`LineFollower.lastBrakeOnGate == false` at arm time, the follower still
// enabled after the overshoot trip) TC-PP1 / TC8 use. Leaves the FSM parked at PLAN_NEXT_HOP
// (pending) for the `Inter3 -> Inter4` hop, node at Inter3, heading still ~0.
// ---------------------------------------------------------------------------
void driveToLinePassThroughArrivalAtInter3(uint8_t& outInter2, uint8_t& outInter3,
                                           uint8_t& outInter4)
{
    outInter2 = nodeByName("Inter2");
    outInter3 = nodeByName("Inter3");
    outInter4 = nodeByName("Inter4");
    ASSERT_TRUE(outInter2 != FieldMap::NODE_NONE);
    ASSERT_TRUE(outInter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(outInter4 != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNodeType(outInter3) == Routes::NodeType::LINE);

    // Pin the route so a future fixture re-weight fails LOUDLY instead of silently re-routing the
    // pass-through triple under test (the routing-guard idiom).
    ASSERT_EQ(routedFirstHop(outInter2, outInter4), outInter3);
    assertNoBypass(outInter2, outInter4);

    // Both flanks are LINE edges — the geometry the pass-through collapses.
    ASSERT_TRUE(FieldMap::getEdge(outInter2, outInter3).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(outInter3, outInter4).type == Routes::EdgeType::LINE);

    driveToLineFollowEntry("Inter2", "Inter4");
    ASSERT_EQ(Localization.getCurrentNode(), outInter2);

    // Gate 1 — the intersection crossing at Inter3. THE PASS-THROUGH ARM PROOF: the real
    // LINE_FOLLOW peek sees the collinear outbound Inter3->Inter4 LINE edge and latches
    // `_passThrough == true`, observed via brake-on-gate=false (TC-PP1's precondition).
    Intersection.mockSetAtIntersection(true);
    cooperativeTick();
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);

    // Gate 2 — the axle-overshoot advance onto Inter3. Pass-through keeps the follower live (no
    // self-brake); the FSM parks at PLAN_NEXT_HOP for the Inter3->Inter4 hop.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), outInter3);
    ASSERT_TRUE(LineFollower.isEnabled());
}
}    // namespace

// ===========================================================================
// Announce the compiled build mode for ctest log clarity.
// ===========================================================================
void test_announce_build_mode()
{
    TEST_SCOPE("build-mode announcement");
    if (Config::PATH_PASS_THROUGH_TRUST_LINE_ENABLED)
    {
        std::printf("INFO: PATH_PASS_THROUGH_TRUST_LINE_ENABLED == true (ADR-078 fix build)\n");
    }
    else
    {
        std::printf(
            "INFO: PATH_PASS_THROUGH_TRUST_LINE_ENABLED == false (pre-ADR-078 bug-repro build)\n");
    }
}

// ===========================================================================
// THE FIX / THE BUG — a >tolerance IMU-yaw drift injected at a collinear LINE pass-through either
// is IGNORED (fix, follower stays live) or breaks the pass-through into a stop-and-turn (bug),
// depending solely on the compiled toggle.
// ===========================================================================
void test_line_passthrough_drift_diverges_by_flag()
{
    if (Config::PATH_PASS_THROUGH_TRUST_LINE_ENABLED)
    {
        TEST_SCOPE("ADR-078 ON: drift at B is trusted-away — no turn, follower stays live");
    }
    else
    {
        TEST_SCOPE("ADR-078 OFF: drift at B breaks the pass-through into a stop-and-turn (bug)");
    }

    uint8_t inter2 = 0;
    uint8_t inter3 = 0;
    uint8_t inter4 = 0;
    driveToLinePassThroughArrivalAtInter3(inter2, inter3, inter4);

    const Routes::EdgeInfo& inbound  = FieldMap::getEdge(inter2, inter3);
    const Routes::EdgeInfo& outbound = FieldMap::getEdge(inter3, inter4);

    // Collinear precondition: inbound and outbound headings agree within the turn tolerance, so a
    // DRIFTLESS pass-through would fire no turn here — only the injected drift can.
    ASSERT_TRUE(fabsf(wrapDeltaDeg(outbound.heading_deg - inbound.heading_deg))
                <= Config::NAV_TURN_TOLERANCE_DEG);

    // INJECT DRIFT: park the live IMU yaw kDriftDeg off the inbound edge heading (== the outbound
    // heading; the triple is collinear) — the drift the LineFollower let accumulate over A->B.
    const float driftedApp = inbound.heading_deg + kDriftDeg;
    mmL.mockYawDeg         = (double) hwYaw((int16_t) lroundf(driftedApp));

    // Genuinely drifted BEFORE the plan tick: the live belief sits kDriftDeg off, i.e. PAST the
    // turn tolerance vs the outbound edge the pre-edge check compares against.
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - driftedApp) < 0.5f);
    ASSERT_TRUE(fabsf(wrapDeltaDeg(outbound.heading_deg - Localization.getHeadingDeg()))
                > Config::NAV_TURN_TOLERANCE_DEG);

    // The tick that runs `_onPlanNextHop` for Inter3 -> Inter4 — i.e. the `_dispatchTurnIfNeeded`
    // decision ADR-078 guards.
    cooperativeTick();

    if (Config::PATH_PASS_THROUGH_TRUST_LINE_ENABLED)
    {
        // THE FIX: no pre-edge turn fired. The guard returned false, so the follower was NEVER
        // disabled and the FSM dispatched the LINE_FOLLOW_INTERSECTION continuation (already-
        // enabled fast-path) — the SAME continue-live state a driftless pass-through lands in.
        ASSERT_TRUE(LineFollower.isEnabled());
        ASSERT_TRUE(PathTracker.state() != PathTrackerClass::State::TURNING);
        ASSERT_FALSE(PathTracker.hasFault());
    }
    else
    {
        // THE BUG (repro): the drifted gyro tripped a real pre-edge turn. `_dispatchTurnIfNeeded`
        // disabled the LineFollower (no brake) and the FSM entered TURNING — the pass-through is
        // broken into a stop-and-turn at Inter3.
        ASSERT_FALSE(LineFollower.isEnabled());
        ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::TURNING);
    }
}

// ===========================================================================
// Entry.
// ===========================================================================
int main()
{
    test_announce_build_mode();
    test_line_passthrough_drift_diverges_by_flag();
    TEST_REPORT();
}
