/**
 * @file test_path_tracker_reset_arrival_strict.cpp
 * @brief Host-link test for the PathTracker reset-node arrival interlude, STRICT ON —
 *        the ON half of the reset-arrival split (#113, TC-RA-strict / AC11). Drives the
 *        REAL `PathTracker` + `Drive` + `Localization` FSMs through the shared #106
 *        cooperative-tick harness into `Fault::WALL_NOT_SQUARE`.
 *
 * @details
 * This is the strict-ON sibling of `test_path_tracker_reset_arrival.cpp` (strict OFF).
 * Together they de-mirror the last reset-arrival FSM slice end-to-end: the strict-OFF
 * target drives the advisory-timeout + clean-success paths through the real interlude;
 * THIS target drives the genuine `WALL_NOT_SQUARE` reject — the exact split precedent set
 * by `test_reanchor_at_wall.cpp` / `test_reanchor_at_wall_strict.cpp`.
 *
 * Flipping the strict flag is a per-TARGET decision, not a per-test one: the CMake target
 * `test_path_tracker_reset_arrival_strict` is compiled with
 * `-DWALL_NORMAL_STRICT_ENABLED_DEFAULT=true` (wired by #113 T2), which reaches EVERY TU —
 * both the separately-compiled `Localization.cpp` that runs the strict wall-normal check
 * AND this test main that asserts the precondition. The shipped production default in
 * `config/ConfigNav.h` stays `false` (the strict check is a guard, not the nominal flow);
 * the strict-OFF reset-arrival target keeps exercising the success/advisory paths against
 * that shipped default. Keeping the two flag states in two single-purpose targets is why
 * the split exists — one target cannot drive both the accept and the reject of the same
 * `reanchorAtWall` call without flipping a compile-time `constexpr` mid-run.
 *
 * WHAT THIS DRIVES (the real FSM, no inline mirror):
 *   1. The SAME real inbound BLIND hop into the RESET node `Wall1` as the strict-OFF
 *      sibling: `seatAt("position1", 0)` + `goTo(Wall1).async()`. `position1 -> Wall1` is a
 *      direct BLIND fixture edge (~45.45 cm, heading ~140.36 deg) resolving to a dead-reckon
 *      advance under the pipeline `SYNC_TSHAPE` blind default — NOT a seek (a seek would
 *      re-anchor via `onIntersection` instead). The interlude fires on the SECOND
 *      `_onPlanNextHop` (firstPlan=false), so the inbound hop must complete first.
 *   2. The wall-stop push (`expectWallStop(270, false)`, timedOut=false) — the internal SQUARE
 *      turn to facing 270 then the stall-success PUSH — retiring the FSM into REANCHOR.
 *   3. The strict reject: BEFORE the REANCHOR tick reads the IMU, the mock yaw is parked
 *      OFF-NORMAL (40 deg off the facing 270, well outside the 10 deg tolerance), so the
 *      real `Localization.reanchorAtWall(...)` runs its strict wall-normal check and REJECTS
 *      (returns false, NO mutation). The real REANCHOR state then `_enterFault`s
 *      `Fault::WALL_NOT_SQUARE` and dispatches NO depart drive (the reject path faults
 *      BEFORE the `Drive.driveDist(departCm)` of the success path).
 *
 * WHAT IT ASSERTS (TC-RA-strict / AC11):
 *   - The precondition `Config::WALL_NORMAL_STRICT_ENABLED == true` FIRST, so a missing /
 *     typo'd compile-def fails loudly instead of silently testing the OFF behaviour (the
 *     `test_reanchor_at_wall_strict.cpp` / `test_path_tracker_reset_arrival.cpp` precedent).
 *   - `PathTracker.getFault().kind == PathTrackerClass::Fault::WALL_NOT_SQUARE`.
 *   - The one-shot reset latch re-armed: `_enterFault` clears `_resetNode` to `NODE_NONE`
 *     (PathTracker.cpp:1128), so a fresh arrival could re-enter the interlude. Asserted via
 *     the observable `PathTracker.hasFault()` (the FSM is halted in FAULT) — the private
 *     `_resetNode` has no accessor; the latch re-arm is the documented `_enterFault`
 *     post-condition (the boolean reject provenance itself is independently proven by
 *     `test_reanchor_at_wall_strict.cpp`).
 *   - NO depart drive was issued — a CONCRETE lower-MCU observable: the WALL_DEPART depart is
 *     `Drive.driveDist(departCm).setMode(DriveMode::GYRO)`, which dispatches via
 *     `_execDriveDistGyro -> MiniR4.DriveDC.MoveGyroDegs(...)` (Drive.cpp:266) — the ONLY
 *     `MoveGyroDegs` emitter in Drive.cpp. The wall-stop SQUARE turn (custom TurnGyro,
 *     `Move`) and PUSH (RAW, `Move`), the pre-edge turn (`Move`), and the inbound
 *     `SYNC_TSHAPE` profiled hop (`Move`/`MoveSync`) NEVER touch `MoveGyroDegs`. So
 *     `MiniR4.DriveDC.moveGyroDegsCallCount` is 0 across the whole inbound hop + wallStop and
 *     would become non-zero ONLY if the depart dispatched. The test captures it right after
 *     `expectWallStop` (before the REANCHOR reject tick) and asserts it is UNCHANGED after.
 */

#include "test_path_tracker_harness.h"

#include <cmath>
#include <cstdio>

// ===========================================================================
// The field's RESET node + its reset record. Resolve "Wall1"/"position1" BY NAME —
// the frozen fixture's node order renumbers on any field edit, so a literal index
// silently points at the wrong node (issue #91). For `Wall1` the fixture authors
// the record { axis=1 (y), wall_coord=154, facing=270 } at node y=139.
// ===========================================================================

/// The wall normal the reset node squares to (app-frame, CCW-positive). `Wall1.facing`.
static constexpr int16_t kWallFacingDeg = 270;

/// Degrees off the wall normal to park the IMU at the REANCHOR tick — well beyond
/// `Config::WALL_NORMAL_TOLERANCE_DEG` (10), so the strict check rejects. 40 deg is the
/// "stopped against a corner / obstacle, not the wall" failure AC11 guards against.
static constexpr int16_t kOffNormalDeg = 40;

// ===========================================================================
// Routing + precondition guard — pin the inbound hop and the strict flag so a future
// fixture re-weight or a mis-wired compile-def fails LOUDLY at the top, not as a
// confusing mid-run assert against a different path / the OFF behaviour.
// ===========================================================================

static void test_precondition_strict_on_and_reset_node()
{
    TEST_SCOPE("precondition: strict ON + Wall1 is a RESET node with a reset record");

    // The whole point of this target is the strict flag being ON. A build that drops or
    // mistypes the `-DWALL_NORMAL_STRICT_ENABLED_DEFAULT=true` compile-def would silently
    // test the OFF behaviour (the real reanchor would ACCEPT off-normal and no fault would
    // ever fire) — assert it here so that mis-wire fails as a clear precondition failure.
    ASSERT_TRUE(Config::WALL_NORMAL_STRICT_ENABLED == true);
    // The tolerance the off-normal park must exceed (mirrors the shipped default).
    ASSERT_TRUE(std::fabs(Config::WALL_NORMAL_TOLERANCE_DEG - 10.0f) < 1e-4f);
    // The off-normal park (40 deg) must be strictly outside the tolerance so the reject is
    // unambiguous — guard the constant choice against a future tolerance widening.
    ASSERT_GT((float) kOffNormalDeg, Config::WALL_NORMAL_TOLERANCE_DEG);

    const uint8_t wall1     = nodeByName("Wall1");
    const uint8_t position1 = nodeByName("position1");

    // The fixture must resolve these names (the stable-name contract).
    ASSERT_TRUE(wall1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(position1 != FieldMap::NODE_NONE);

    // Wall1 is a RESET node carrying a reset record (else the interlude is never entered).
    ASSERT_TRUE(FieldMap::getNodeType(wall1) == Routes::NodeType::RESET);
    const Routes::ResetNode* info = FieldMap::getResetInfo(wall1);
    ASSERT_TRUE(info != nullptr);
    ASSERT_EQ((int) info->facing, (int) kWallFacingDeg);

    // The inbound hop `position1 -> Wall1` routes DIRECTLY to Wall1 over a BLIND edge (a
    // dead-reckon advance, NOT a seek — a seek would re-anchor via onIntersection and change
    // the believed-pose math the interlude relies on).
    ASSERT_EQ(routedFirstHop(position1, wall1), wall1);
    ASSERT_TRUE(FieldMap::getEdge(position1, wall1).type == Routes::EdgeType::BLIND);
}

// ===========================================================================
// TC-RA-strict — strict reject -> WALL_NOT_SQUARE (AC11), real FSM end-to-end.
//
// Drive the real inbound BLIND hop into Wall1, retire the wall-stop push, then park the
// IMU off-normal so the real `reanchorAtWall` rejects under the strict check. Assert the
// FSM faults WALL_NOT_SQUARE, the reset latch re-arms, and NO depart drive was issued.
// ===========================================================================

static void test_TC_RA_strict_off_normal_reanchor_faults_wall_not_square()
{
    TEST_SCOPE("TC-RA-strict: off-normal REANCHOR -> real reanchorAtWall rejects -> WALL_NOT_SQUARE");

    resetWorld();    // clears both Drive + PathTracker faults — order-independent per case.

    const uint8_t wall1     = nodeByName("Wall1");
    const uint8_t position1 = nodeByName("position1");

    // Seat at the inbound origin, frame zeroed at app-yaw 0 (so `_yawOffsetDeg == 0` and the
    // strict check's live heading == normalize(mockYawDeg * IMU_YAW_SIGN)).
    seatAt("position1", 0);

    // The inbound hop's forward heading — park the corrected IMU read here every tick so the
    // pre-edge turn settles (the `driveToSeekInFlight` / CONTRACT-2 idiom).
    const Routes::EdgeInfo& edge      = FieldMap::getEdge(position1, wall1);
    const int16_t           hwHeading = hwYaw((int16_t) lroundf(edge.heading_deg));

    // Enqueue the FINAL node only; PathTracker walks NEXT_HOP. `.async()` so the harness
    // drives the cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(wall1).async();

    // --- Drive the inbound BLIND hop. Settle the pre-edge turn by parking the edge heading
    // each tick, then retire the dead-reckon advance. The hop completes when the FSM has
    // dead-reckoned onto Wall1 and re-entered _onPlanNextHop (which fires the interlude:
    // _maybeEnterReset dispatches the wallStop and enters WALL_STOP). Detect that transition
    // by Localization landing on Wall1 — the dead-reckon advance sets the current node.
    for (int i = 0;
         i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != wall1;
         ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;    // re-park EVERY tick so the pre-turn settles.
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);    // retire the
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);    // dead-reckon move.
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), wall1);
    // No depart-shaped lower-MCU dispatch has happened yet — the inbound SYNC_TSHAPE hop and
    // the pre-turn drive `Move`/`MoveSync`, never `MoveGyroDegs`.
    ASSERT_EQ(MiniR4.DriveDC.moveGyroDegsCallCount, 0u);

    // --- Retire the wall-stop push to a clean stall-success (no advisory). The interlude is
    // now WALL_STOP; this settles the internal SQUARE turn to facing 270 then confirms the
    // stall, leaving the FSM in WALL_STOP -> (next tick) REANCHOR. NOT timed out: a clean
    // arrival, so the advisory path is not taken (irrelevant to the reject, which faults
    // before any advisory branch matters).
    expectWallStop(kWallFacingDeg, /*timedOut*/ false);

    // CAPTURE the depart observable BEFORE the REANCHOR reject tick. The WALL_DEPART depart is
    // `Drive.driveDist(departCm).setMode(GYRO)` -> `MoveGyroDegs` (Drive.cpp:266, the sole
    // emitter). Nothing in the inbound hop or the wallStop touches it, so this is 0 here — but
    // capture the value rather than assuming, so the post-assert is a true before/after delta.
    const uint32_t moveGyroDegsBeforeReanchor = MiniR4.DriveDC.moveGyroDegsCallCount;

    // --- Park the IMU OFF-NORMAL before the REANCHOR tick reads it. 40 deg off the facing
    // (270) is well outside the 10 deg tolerance, so the real `reanchorAtWall` strict check
    // computes |live - facing| = 40 > 10 and REJECTS (returns false, no mutation). With the
    // frame zeroed at app-yaw 0 (_yawOffsetDeg == 0), the live heading the check sees is
    // normalize(mockYawDeg * IMU_YAW_SIGN), so park the hw-frame value of (270 - 40) = 230.
    mmL.mockYawDeg = (double) hwYaw(kWallFacingDeg - kOffNormalDeg);

    // Tick until the FSM faults (the REANCHOR state runs the real reanchorAtWall, gets false,
    // and `_enterFault(WALL_NOT_SQUARE)`). The interlude is one tick from REANCHOR after
    // WALL_STOP saw Drive idle, so a small bound suffices; the budget guards a regression.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        // Hold the off-normal yaw across the settle ticks so the strict read stays off-normal.
        mmL.mockYawDeg = (double) hwYaw(kWallFacingDeg - kOffNormalDeg);
        cooperativeTick();
    }

    // --- Assert the strict reject routed to WALL_NOT_SQUARE on the real FSM.
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::WALL_NOT_SQUARE);

    // The fault was raised AT Wall1 — `REANCHOR` calls `_enterFault(WALL_NOT_SQUARE, cur, ...)`
    // with `cur == Localization.getCurrentNode() == Wall1` (PathTracker.cpp:1035), so this also
    // pins that the reject happened on the reset node whose interlude was in flight (not some
    // other halt). `_enterFault` then re-arms the one-shot reset latch by clearing `_resetNode`
    // to NODE_NONE (PathTracker.cpp:1128) — the private latch has no accessor, but routing the
    // fault through `_enterFault` (the FSM is halted in FAULT, fromNode == Wall1) is the
    // observable proof the reject took the re-arming path, NOT the success tail (which would have
    // dispatched a depart and left the latch set).
    ASSERT_EQ(PathTracker.getFault().fromNode, wall1);

    // --- NO depart drive was issued: the reject path faults BEFORE `Drive.driveDist(departCm)`.
    // The depart's `MoveGyroDegs` dispatch count is UNCHANGED across the reject tick (the
    // concrete lower-MCU observable the spec pins). A regression that drove the depart before
    // (or instead of) faulting would bump this and fail here.
    ASSERT_EQ(MiniR4.DriveDC.moveGyroDegsCallCount, moveGyroDegsBeforeReanchor);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker reset-arrival STRICT — real FSM into WALL_NOT_SQUARE "
                "(TC-RA-strict); strict flag = %s\n",
                Config::WALL_NORMAL_STRICT_ENABLED ? "ON" : "OFF");

    test_precondition_strict_on_and_reset_node();
    test_TC_RA_strict_off_normal_reanchor_faults_wall_not_square();

    if (wro_test::failCounter() == 0)
    {
        std::printf("PATH TRACKER RESET ARRIVAL STRICT OK\n");
    }
    TEST_REPORT();
}
