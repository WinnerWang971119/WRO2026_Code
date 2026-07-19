/**
 * @file test_path_tracker_approach_ramp.cpp
 * @brief Host-link tests for the anticipatory line-approach ramp's PathTracker
 *        dispatch surface (2026-07-17 line-approach-ramp plan, T11), driving the
 *        REAL `PathTracker` + `Drive` FSMs through the shared cooperative-tick
 *        harness (`test_path_tracker_harness.h`, issue #106 lineage).
 *
 * @details
 * Covers the two LineFollower-junction-hop cases the sibling files do not:
 *
 *   - TC9 (AC9, direction-aware prediction): a forward LINE hop's
 *     `LINE_FOLLOW_INTERSECTION` arm predicts using the FRONT axle offset
 *     (`Config::LINE_SENSOR_AXLE_OFFSET_CM`, 12.02); a reverse (FORCE_REVERSE)
 *     LINE hop predicts using the REAR axle offset
 *     (`Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM`, 7.68) — asserted via
 *     `LineFollower.lastApproachTarget` (the recording `path_mocks` field
 *     `setApproachTarget(...)` writes), which PathTracker.cpp:2744 computes as
 *     `_pivotTrimDistance(edge) - axleOffsetCm`. Neither fixture hop under test
 *     carries a latched pivot, so `_pivotTrimDistance(edge) == edge.distance_cm`.
 *
 *   - TC6 (AC6): the bounded-overrun fault the LineFollower junction hop lacked
 *     before this feature. `path_mocks/LineFollower.h`'s `mockSetApproachPhase(...)`
 *     scripts `approachPhase() == ApproachPhase::OVERRUN` directly — the mock
 *     never models a real P+D controller, so this is the correct, and only,
 *     lever to drive the phase from a host test. Per the plan's S3 note, the
 *     scenario is deliberately NOT a lost-line scenario: `Intersection` is left
 *     un-latched (no `mockSetAtIntersection(true)`) so the ONLY branch PathTracker's
 *     `LINE_FOLLOW` state can take is the `approachPhase() == OVERRUN` poll — the
 *     path_mocks surface has no `isOnline()`/lost-line concept at all (that logic
 *     is entirely internal to the REAL `LineFollower::update()`, not linked here),
 *     so there is no way for this test to accidentally exercise a different
 *     branch.
 *
 * TC5 (pass-through suppression, AC5) lives in `test_path_tracker_passthrough.cpp`
 * instead — it reuses that file's existing ON/OFF build matrix and collinear
 * `Inter2->Inter3->Inter4` fixture triple rather than duplicating them here.
 */

#include "test_path_tracker_harness.h"

#include <cmath>

// ===========================================================================
// TC9 — direction-aware prediction (AC9): forward uses the FRONT axle offset,
// reverse uses the REAR axle offset, via the `axleOffsetCm` PathTracker already
// resolves for `decision.reverse`.
// ===========================================================================

static void test_TC9_forward_line_hop_predicts_with_front_axle_offset()
{
    TEST_SCOPE("TC9: forward LINE hop predicts distance_cm - LINE_SENSOR_AXLE_OFFSET_CM (front, AC9)");

    // Inter2 -> Inter3, goal == Inter3 (single hop): `_nextHop == _destinationNode`, so the
    // dispatch-time pass-through peek's not-final guard (`_peekNextNextHop`,
    // PathTracker.cpp:2204) returns NODE_NONE regardless of what lies beyond Inter3 — this hop
    // is unconditionally a TERMINATING LINE hop, so the approach ramp arms.
    driveToLineFollowEntry("Inter2", "Inter3");
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);
    ASSERT_FALSE(LineFollower.lastReverse);

    if (!Config::LINE_APPROACH_RAMP_ENABLED)
    {
        // AC8 (LineFollower half): with the feature globally disabled, the arm never chains
        // `.setApproachTarget(...)` — byte-identical to today's flat behavior.
        ASSERT_TRUE(LineFollower.lastApproachTarget == 0.0f);
        return;
    }

    const Routes::EdgeInfo& edge = FieldMap::getEdge(nodeByName("Inter2"), nodeByName("Inter3"));
    const float              expectedPredictedCm =
        edge.distance_cm - Config::LINE_SENSOR_AXLE_OFFSET_CM;

    ASSERT_TRUE(LineFollower.lastApproachTarget > 0.0f);
    ASSERT_TRUE(std::fabs(LineFollower.lastApproachTarget - expectedPredictedCm) < 1e-3f);
}

static void test_TC9_reverse_line_hop_predicts_with_rear_axle_offset()
{
    TEST_SCOPE("TC9: reverse LINE hop predicts distance_cm - LINE_SENSOR_AXLE_OFFSET_REAR_CM (rear, AC9)");

    // Inter1 -> Spur carries the FORCE_REVERSE per-edge override (direction="rear") — the SAME
    // rear-override hop TC-BD2 (`test_path_tracker_bidirectional.cpp`) exercises. `Spur` is the
    // sole (and therefore final) hop from `Inter1`, so pass-through is inapplicable here too.
    // `Direction::REAR` is passed explicitly so `driveToLineFollowEntry`'s OWN body-heading
    // calculation (which +180's only when its `routeDirection` param is REAR) parks the mock
    // IMU yaw at the target the FSM will ACTUALLY resolve (the edge's FORCE_REVERSE override
    // already forces REAR independently — the explicit override is consistent with it, not a
    // conflicting one).
    driveToLineFollowEntry("Inter1", "Spur", Direction::REAR);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);
    ASSERT_TRUE(LineFollower.lastReverse);

    if (!Config::LINE_APPROACH_RAMP_ENABLED)
    {
        ASSERT_TRUE(LineFollower.lastApproachTarget == 0.0f);
        return;
    }

    const Routes::EdgeInfo& edge = FieldMap::getEdge(nodeByName("Inter1"), nodeByName("Spur"));
    ASSERT_TRUE(edge.direction_override == Routes::DirectionMode::FORCE_REVERSE);
    const float expectedPredictedCm = edge.distance_cm - Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM;

    ASSERT_TRUE(LineFollower.lastApproachTarget > 0.0f);
    ASSERT_TRUE(std::fabs(LineFollower.lastApproachTarget - expectedPredictedCm) < 1e-3f);

    // Sanity: the rear prediction is NOT the front one — proves the direction-aware branch
    // actually fired rather than always resolving the front constant.
    ASSERT_TRUE(std::fabs(LineFollower.lastApproachTarget
                          - (edge.distance_cm - Config::LINE_SENSOR_AXLE_OFFSET_CM))
                > 1e-3f);
}

// ===========================================================================
// TC6 — bounded-overrun fault (AC6): `LineFollower.approachPhase() == OVERRUN` ->
// PathTracker's `LINE_FOLLOW` state raises `Fault::DRIVE_FAULT` and halts, rather
// than crawling forever. Per plan note S3, this is NOT a lost-line scenario — the
// path_mocks surface has no `isOnline()`/lost-line concept to accidentally trip;
// `Intersection` is simply never latched, so the marker never classifies.
// ===========================================================================

static void test_TC6_approach_overrun_raises_drive_fault()
{
    TEST_SCOPE("TC6: LineFollower approachPhase()==OVERRUN -> PathTracker DRIVE_FAULT, chassis stopped (AC6)");

    driveToLineFollowEntry("Inter2", "Inter3");
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(LineFollower.isEnabled());

    // The junction marker never classifies (Intersection is left un-latched — the sensor
    // "stays online" in the sense that path_mocks has no offline concept at all) while the
    // approach schedule has crawled past `predicted + LINE_APPROACH_MAX_MARGIN_CM` without a
    // confirmed crossing. Script that terminal phase directly.
    LineFollower.mockSetApproachPhase(ApproachPhase::OVERRUN);

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        cooperativeTick();
    }

    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::DRIVE_FAULT);

    // The chassis is stopped: PathTracker's fault path disables the follower and issues an
    // active `Drive.stop(true)` — `LineFollower.isEnabled()` is the path_mocks-observable half
    // of that (the mock's `setEnabled(false)` bare call still records `lastEnabled`/state).
    ASSERT_FALSE(LineFollower.isEnabled());
}

// ===========================================================================
// TC8 (seek half, AC8) — with `LINE_APPROACH_RAMP_ENABLED == false`, the
// `Drive.stopAtLine` seek holds flat cruise all the way up to (and past) the
// expected crossing distance, instead of ramping toward the floor. This target
// is built TWICE (see CMakeLists.txt): only the toggle-off build's assertion is
// meaningful — the toggle-on build's LineFollower-side coverage already lives in
// TC9 above, and the seek's ENABLED ramp behavior already lives in TC2/TC11/TC12
// (`test_path_tracker_stop_at_line.cpp`), so this function is a no-op there.
// ===========================================================================

static void test_TC8_seek_stays_flat_when_ramp_disabled()
{
    TEST_SCOPE("TC8: with LINE_APPROACH_RAMP_ENABLED==false, the stopAtLine seek holds flat cruise (AC8)");

    if (Config::LINE_APPROACH_RAMP_ENABLED)
    {
        return;    // this build's toggle is ON — nothing to assert here (see TC2/TC11/TC12).
    }

    resetWorld();
    seatAt("Start");

    const uint8_t            start    = nodeByName("Start");
    const uint8_t            inter1   = nodeByName("Inter1");
    const uint8_t            firstHop = routedFirstHop(start, inter1);
    const Routes::EdgeInfo&  edge     = FieldMap::getEdge(start, firstHop);
    const int16_t            hwHeading = hwYaw((int16_t) lroundf(edge.heading_deg));

    PathTracker.goTo(inter1).async();

    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
        if (MiniR4.DriveDC.moveSyncCallCount > 0)
        {
            break;
        }
    }
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, 0u);

    const int16_t cruiseMag = MiniR4.DriveDC.lastMoveSyncL;

    // Advance to right at the expected crossing distance — where the ENABLED build (TC2) would
    // already be deep in FLOOR_HOLD — and confirm cruise never moved.
    const int32_t baseline = MiniR4.M2.getDegrees();
    LineArray.mockSetJunctionType(JunctionType::NONE);
    MiniR4.M2.mockSetDegrees(baseline + (int32_t) cmToEncDeg(edge.distance_cm - 2.0f));
    MiniR4.M3.mockSetDegrees(baseline + (int32_t) cmToEncDeg(edge.distance_cm - 2.0f));
    cooperativeTick();

    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, cruiseMag);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncR, cruiseMag);

    // Teardown: fault the seek out (no crossing scripted) so it does not leak `_stopState`.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.hasFault(); ++i)
    {
        const int32_t faultEnc =
            baseline
            + (int32_t) cmToEncDeg(edge.distance_cm + Config::STOP_AT_LINE_MAX_MARGIN_CM + 5.0f);
        MiniR4.M2.mockSetDegrees(faultEnc);
        MiniR4.M3.mockSetDegrees(faultEnc);
        cooperativeTick();
    }
    ASSERT_TRUE(Drive.hasFault());
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_TC9_forward_line_hop_predicts_with_front_axle_offset();
    test_TC9_reverse_line_hop_predicts_with_rear_axle_offset();
    test_TC6_approach_overrun_raises_drive_fault();
    test_TC8_seek_stays_flat_when_ramp_disabled();
    TEST_REPORT();
}
