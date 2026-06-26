/**
 * @file test_return_to_node_dispatch.cpp
 * @brief Host-link test for the PathTracker RETURN_TO_NODE per-kind dispatch slice,
 *        driving the REAL `PathTracker` + `Drive` + `Localization` FSMs through the
 *        shared cooperative-tick harness (issue #36 PR1b origin, de-mirrored under #113).
 *
 * @details
 * Rewritten from the original mirror-slice test. The old file replicated the
 * RETURN_TO_NODE state body inline (an annotated decision replica plus its private
 * result struct and outcome enum), each mirrored decision tagged against a
 * PathTracker.cpp line for drift detection. Those FSM-slice replicas are GONE: this
 * file now drives the production `PathTracker` FSM over the REAL issue #91 fixture graph
 * (`fixtures/routes_fixture.cpp`) and asserts the re-anchor / fault routing the FSM
 * itself issues, manufacturing a genuine operator offset through the real
 * `Localization` integrator (`pushOperatorOffset`) so the DEQUEUE return prologue
 * fires for real.
 *
 * THE RETURN PROLOGUE (PathTracker.cpp DEQUEUE -> `_evaluateReturnGate` ->
 * `_dispatchReturnMove`): a `goTo(anchor)` with a manufactured offset > the trigger
 * threshold (3 cm) and <= the cap (50 cm) runs the return prologue at DEQUEUE,
 * driving the chassis FORWARD toward `atan2(-off.y, -off.x)` (= the seated push
 * heading + 180 deg) back onto the anchor, THEN completing the goal (cur == dest)
 * right after the re-anchor. `_returnUsesLineSeek = nodeIsLine && (_returnDistCm >=
 * Config::LINE_SENSOR_AXLE_OFFSET_CM)` selects the seek vs the dead-reckon branch:
 *   - TC-RN-line-ok    : LINE anchor (`Inter1`), push 20 cm (>= 12.02) -> GYRO seek
 *                        -> crossing -> `onIntersection` re-anchor (same node), no fault.
 *   - TC-RN-line-noline: LINE anchor, push 20 cm, seek finds no line -> Drive latches
 *                        `LINE_NOT_FOUND` -> under the shipped `OFF_MAP_GATE == FAULT`
 *                        the RETURN_TO_NODE body routes `Fault::OFF_NODE` (NOT
 *                        DRIVE_FAULT), no re-anchor.
 *   - TC-RN4c          : LINE anchor, push 6 cm (3 < 6 < 12.02) -> the distance gate
 *                        clears `_returnUsesLineSeek` -> dead-reckon branch even on a
 *                        LINE anchor (#14): `clearOffset` re-anchor, no seek, no fault.
 *   - TC-RN-blind-ok   : BLIND anchor (`Start`), push 20 cm -> dead-reckon ->
 *                        `clearOffset` re-anchor, node unchanged, no fault.
 *
 * KEPT (direct real-symbol guards, NOT mirrors): TC4 discriminates LINE (`Inter1`)
 * vs BLIND (`Start`) over the REAL `FieldMap::getNodeType` — the same accessor
 * `_dispatchReturnMove` reads for `nodeIsLine` — and TC4c computes the line-seek
 * distance gate against the REAL `Config::LINE_SENSOR_AXLE_OFFSET_CM`. A helper that
 * *calls* the real symbol is not a mirror (plan AC). The feature-complete fixture
 * carries every node kind, so the LINE-vs-non-LINE classification is genuine, not a
 * tautology. Both anchors are resolved BY NAME (`Inter1` / `Start`), so the test is
 * decoupled from the fixture's index order (issue #91).
 *
 * ===========================================================================================
 * HOST-UNREACHABLE BRANCHES — deleted + documented (NOT covered, NO tautological assert)
 * ===========================================================================================
 * The deleted decision replica also covered THREE RETURN_TO_NODE state-body branches that,
 * driven through the REAL FSM, cannot be exercised on host. The host targets link the REAL
 * `Drive.cpp` — there is NO `path_mocks` Drive, so the real `Drive` wins; the only Drive
 * fault any RETURN_TO_NODE drive can latch on host is `DriveClass::Fault::LINE_NOT_FOUND`
 * (the GYRO `stopAtLine` seek's max-distance gate, `_updateStopAtLine`, Drive.cpp). That seek
 * has no IMU stall watchdog that latches a NON-LINE_NOT_FOUND kind, and the dead-reckon
 * branch's SYNC `driveDist` cannot be faulted at all through the mock observables (a profiled
 * move only ever completes or hangs — it never self-faults). No ADDITIVE, non-behavioral mock
 * can inject a generic Drive fault without modeling the STM32 done-bit / fault FSM, which the
 * thin-mock rule forbids ([[project-host-mock-thin-not-behavioral]]). So these three REAL
 * production branches are knowingly UNCOVERED on host (recorded in ADR-030's
 * "Trigger to revisit" as intentional divergence) — and are NOT replaced with a fake/placeholder
 * `ASSERT_EQ(X, X)`:
 *   (1) PathTracker.cpp:611    — line-seek discriminator: `Drive.getFault().kind !=
 *                                LINE_NOT_FOUND` -> `Fault::DRIVE_FAULT`. Unreachable: the real
 *                                seek only ever latches LINE_NOT_FOUND.
 *   (2) PathTracker.cpp:629-635 — BEST_EFFORT accept: `Drive.clearFault()` + `clearOffset()` on a
 *                                LINE_NOT_FOUND under `OFF_MAP_GATE == BEST_EFFORT`. Unreachable:
 *                                the shipped gate is FAULT (no per-target compile-def override is
 *                                added — no production change), so the FAULT branch at :619-625 is
 *                                what TC-RN-line-noline drives instead.
 *   (3) PathTracker.cpp:654-659 — blind dead-reckon failure: `Drive.hasFault()` -> `DRIVE_FAULT`.
 *                                Unreachable: a SYNC `driveDist` cannot be faulted via the mock.
 *
 * NOTE: the pure policy test `test_return_to_node_policy.cpp` covers ONLY the GATE decision
 * (`resolveReturnAction`: RETURN / NONE / OFF_MAP), NOT this state-body routing — so it does NOT
 * recover the three branches above. This file does not claim otherwise.
 *
 * Links the real `PathTracker.cpp` / `Drive.cpp` / `Localization.cpp` / `YawSource.cpp` +
 * `routes_fixture.cpp` + `path_mocks.cpp` via the #106 real-link recipe (SKETCH_DIR-first
 * includes; force-include the fixture routes then the path mocks; NO mock Drive). The pass/fail
 * + final-line conventions mirror `test_path_tracker_stop_at_line.cpp`.
 */

#include "test_path_tracker_harness.h"

#include <cmath>
#include <cstdio>

namespace
{
// The two node kinds this test discriminates, resolved BY NAME (issue #91): a LINE
// anchor (`Inter1`) and a BLIND anchor (`Start`). The fixture is feature-complete and
// its alphabetical index order shifts whenever a node is added, so a hardcoded index
// would silently point at the wrong node. Resolving by name keeps the test decoupled
// from the fixture's shape — it depends only on these two names existing with their kinds.
const uint8_t kLineNode  = FieldMap::findNodeByName("Inter1");    // LINE-backed
const uint8_t kBlindNode = FieldMap::findNodeByName("Start");     // BLIND-only

// ===========================================================================
// Real-FSM return driver — shared by the four RETURN_TO_NODE cases.
//
// Seat `Localization` at `anchorName` (binding the world frame at app-yaw
// `pushHeadingDeg`), manufacture a REAL operator offset of magnitude `pushCm`
// along that heading through the real integrator, then `goTo(anchor).async()` so
// the DEQUEUE return prologue fires. The return drives FORWARD toward
// `atan2(-off.y, -off.x)` == `pushHeadingDeg + 180`, so park that return heading
// every tick (the `driveToSeekInFlight` idiom) until the return DRIVE emits — for a
// LINE-seek that is the GYRO `MoveGyro` continuous emission; for a dead-reckon
// return that is the profiled SYNC `MoveSync` emission. The shared pre-turn runs
// `Move(L,R)` (NOT MoveSync/MoveGyro), so a non-zero `moveSync`/`moveGyro` count
// cleanly distinguishes "return drive in flight" from "still turning".
//
// The caller then retires the in-flight drive with `expectStopAtLine` (seek) or
// `expectBlindHop` (dead-reckon) and asserts the re-anchor / fault outcome.
//
// @param anchorName     LINE or BLIND anchor to seat at and return to (resolved by name).
// @param pushCm         Operator-offset magnitude to manufacture (cm). Must be > the
//                       return trigger threshold (3) and <= the cap (50).
// @param pushHeadingDeg App-frame heading the hand-push runs along (the seated facing).
// @param usesSeek       `true`  -> a LINE-seek return (wait for `moveGyroCallCount`);
//                       `false` -> a dead-reckon return (wait for `moveSyncCallCount`).
// @return The achieved operator-offset magnitude (so the caller can size
//         `expectStopAtLine(mag, ...)` from the same value the dispatch clamped to).
// ===========================================================================
float driveReturnInFlight(const char* anchorName, float pushCm, int16_t pushHeadingDeg,
                          bool usesSeek)
{
    resetWorld();    // clears both Drive + PathTracker faults, so a prior case's
                     // LINE_NOT_FOUND cannot leak into this return.

    // Bind the world frame at the push heading and seat Localization at the anchor.
    seatAt(anchorName, pushHeadingDeg);

    // Manufacture a genuine operator offset along the seated heading through the real
    // integrator (self-asserts the achieved magnitude — a zero-accumulation regression
    // fails inside the helper, not as a misleading green RETURN no-op).
    const float pushedMag = pushOperatorOffset(pushCm, pushHeadingDeg);

    // The return drives FORWARD toward the inverse-offset heading = push heading + 180.
    const int16_t returnAppHeading = (int16_t) (pushHeadingDeg + 180);
    const int16_t hwReturnHeading  = hwYaw(returnAppHeading);

    // Enqueue the SAME anchor only; the return prologue runs at DEQUEUE, then the goal
    // completes (cur == dest) right after the re-anchor. `.async()` so the harness drives
    // the cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(nodeByName(anchorName)).async();

    // Tick until the return DRIVE emits (seek -> MoveGyro; dead-reckon -> MoveSync). Each
    // tick re-parks the IMU yaw at the return heading so the shared pre-turn settles and the
    // loop cannot stall on the in-place turn.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        mmL.mockYawDeg = (double) hwReturnHeading;
        cooperativeTick();
        const bool emitted = usesSeek ? (MiniR4.DriveDC.moveGyroCallCount > 0)
                                      : (MiniR4.DriveDC.moveSyncCallCount > 0);
        if (emitted)
        {
            return pushedMag;    // return drive dispatched + emitting; M2 still at the baseline
        }
    }

    ASSERT_TRUE(false);    // budget exhausted: the FSM never dispatched the return drive
    return pushedMag;
}
}    // namespace

// ===========================================================================
// TC4 — node-kind CLASSIFICATION against the REAL FieldMap::getNodeType (KEPT,
// a direct real-symbol guard, NOT a mirror). The fixture has LINE nodes plus
// BLIND/RESET nodes, so the node-kind HALF of the `_returnUsesLineSeek` predicate
// (`getNodeType(cur) == NodeType::LINE`) discriminates LINE from non-LINE for real.
// The distance HALF (dist >= the axle lead) is covered by TC4c below.
// ===========================================================================

void test_TC4_real_get_node_type_discriminates_line_vs_blind()
{
    TEST_SCOPE("TC4: real FieldMap::getNodeType discriminates LINE (Inter1) vs BLIND (Start)");

    // The fixture must actually contain both kinds for the discrimination to be real —
    // assert that precondition so a future field_data change that drops one kind fails
    // loudly here rather than silently weakening the test.
    bool sawLine  = false;
    bool sawBlind = false;
    for (uint8_t id = 0; id < Routes::N_NODES; ++id)
    {
        const Routes::NodeType t = FieldMap::getNodeType(id);
        if (t == Routes::NodeType::LINE)
        {
            sawLine = true;
        }
        if (t == Routes::NodeType::BLIND)
        {
            sawBlind = true;
        }
    }
    ASSERT_TRUE(sawLine);
    ASSERT_TRUE(sawBlind);

    // Spot-check the two named anchors (resolved by name): a LINE node uses the seek; a
    // BLIND node dead-reckons. No index assumptions — the fixture's shape can change freely
    // as long as Inter1 stays LINE and Start stays BLIND.
    ASSERT_TRUE(kLineNode != FieldMap::NODE_NONE);
    ASSERT_TRUE(kBlindNode != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNodeType(kLineNode) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(kBlindNode) != Routes::NodeType::LINE);
}

// ===========================================================================
// TC4c — line-seek DISTANCE GATE (#14, KEPT — a direct call to the REAL
// `Config::LINE_SENSOR_AXLE_OFFSET_CM`, NOT a mirror). `_returnUsesLineSeek` is true
// ONLY for a LINE anchor whose return distance reaches the front sensor's axle lead.
// A shorter LINE return leaves the sensor already past the node line, so a forward
// seek can never re-cross it — it dead-reckons back like a blind anchor. This
// confirms the gate arithmetic the FSM applies in `_dispatchReturnMove`; the
// end-to-end routing of a short LINE return through the dead-reckon branch is driven
// against the REAL FSM in TC-RN4c below.
// ===========================================================================

void test_TC4c_line_seek_distance_gate_uses_real_axle_offset()
{
    TEST_SCOPE("TC4c: line-seek distance gate keys on the real Config::LINE_SENSOR_AXLE_OFFSET_CM");

    const float axle = Config::LINE_SENSOR_AXLE_OFFSET_CM;

    // The LINE anchor (Inter1) is LINE (asserted in TC4). The gate the FSM computes:
    const bool lineAnchorIsLine = (FieldMap::getNodeType(kLineNode) == Routes::NodeType::LINE);
    ASSERT_TRUE(lineAnchorIsLine);

    const bool shortUsesSeek = lineAnchorIsLine && ((axle - 1.0f) >= axle);     // short return
    const bool atUsesSeek    = lineAnchorIsLine && (axle >= axle);              // exactly the lead
    const bool longUsesSeek  = lineAnchorIsLine && ((axle + 10.0f) >= axle);    // comfortably long
    ASSERT_FALSE(shortUsesSeek);    // below the lead -> dead-reckon (the #14 case)
    ASSERT_TRUE(atUsesSeek);        // exactly at the lead -> seek (>=)
    ASSERT_TRUE(longUsesSeek);      // above the lead -> seek

    // A non-LINE node (the BLIND anchor Start) never uses the seek regardless of distance.
    const bool blindAnchorIsLine = (FieldMap::getNodeType(kBlindNode) == Routes::NodeType::LINE);
    const bool blindLongUsesSeek = blindAnchorIsLine && ((axle + 10.0f) >= axle);
    ASSERT_FALSE(blindLongUsesSeek);
}

// ===========================================================================
// TC-RN-line-ok — LINE-seek return success (real FSM, real GYRO Drive seek).
//
// `Inter1` (LINE) + a 20 cm push (>= the 12.02 axle lead) -> `_returnUsesLineSeek`
// true. The real `_dispatchReturnMoveAfterTurn` issues `Drive.stopAtLine(...)
// .setMode(GYRO).async()`; the harness scripts a debounced crossing + overshoot roll
// so the seek confirms the line and finishes DONE; the RETURN_TO_NODE state then
// re-anchors the SAME node via `Localization.onIntersection(getCurrentNode())` and
// completes the goal. Asserts the node is unchanged (Inter1), the offset is cleared,
// and there is no fault.
// ===========================================================================

void test_TC_RN_line_ok_seek_reanchors_via_on_intersection()
{
    TEST_SCOPE("TC-RN-line-ok: LINE-seek return success re-anchors Inter1 via onIntersection");

    // Routing guard: the anchor is a LINE node (so the seek branch is selected for a
    // >= axle-lead push) — pin it so a fixture kind change fails loudly here.
    ASSERT_TRUE(FieldMap::getNodeType(kLineNode) == Routes::NodeType::LINE);

    const float expectedCm = driveReturnInFlight("Inter1", /*pushCm*/ 20.0f,
                                                 /*pushHeadingDeg*/ 0, /*usesSeek*/ true);

    // The return seek is in flight: the real GYRO seek emitted its continuous drive
    // (the representative lower-MCU seek call — RETURN line-seek is GYRO, not SYNC).
    ASSERT_GT(MiniR4.DriveDC.moveGyroCallCount, 0u);
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the seek to a confirmed crossing: arm the window, hold a debounced
    // T_OR_CROSS, roll the overshoot to DONE. The harness asserts Drive latched the
    // crossing (`isLineFound()`) and did NOT fault.
    expectStopAtLine(expectedCm, /*found*/ true);

    // The RETURN_TO_NODE state saw `Drive.isIdle()` and re-anchored via
    // `onIntersection(getCurrentNode())` — a physical re-localisation that keeps the
    // discrete node on Inter1 (we returned TO it, did not advance), then completed the goal.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && (PathTracker.hasFault() == false && !PathTracker.isIdle());
         ++i)
    {
        cooperativeTick();
    }

    // Re-anchored on the SAME node ...
    ASSERT_EQ(Localization.getCurrentNode(), kLineNode);
    // ... the operator offset was cleared by the re-anchor (onIntersection clears it) ...
    const Offset2D off = Localization.getOffsetFromNode();
    ASSERT_TRUE(sqrtf(off.x * off.x + off.y * off.y) < Config::RETURN_TRIGGER_THRESHOLD_CM);
    // ... and the FSM settled without faulting (Inter1 IS the goal).
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(Drive.isLineFound());
}

// ===========================================================================
// TC-RN-line-noline — LINE-seek return no-line under the shipped FAULT gate.
//
// `Inter1` (LINE) + a 20 cm push -> seek; the seek runs the full max distance with no
// crossing and latches `DriveClass::Fault::LINE_NOT_FOUND`. RETURN_TO_NODE is EXEMPT
// from the generic Drive-fault guard, so its state body routes the LINE_NOT_FOUND
// through the off-map gate: under the SHIPPED `Config::OFF_MAP_GATE == FAULT` this is
// `Fault::OFF_NODE` (NOT DRIVE_FAULT — that discriminator branch at PathTracker.cpp:611
// is host-unreachable, see the file header). No re-anchor.
// ===========================================================================

void test_TC_RN_line_noline_routes_off_node_under_fault_gate()
{
    TEST_SCOPE("TC-RN-line-noline: LINE-seek no-line under FAULT gate -> Fault::OFF_NODE");

    // This case relies on the SHIPPED gate being FAULT — pin it so a config change that
    // flips it to BEST_EFFORT fails loudly here rather than silently testing the wrong route.
    ASSERT_TRUE(Config::OFF_MAP_GATE == Config::OffMapGate::FAULT);

    const float expectedCm = driveReturnInFlight("Inter1", /*pushCm*/ 20.0f,
                                                 /*pushHeadingDeg*/ 0, /*usesSeek*/ true);
    ASSERT_GT(MiniR4.DriveDC.moveGyroCallCount, 0u);

    // Over-advance the encoder past the max margin with the marker held NONE: the real
    // seek's max-distance gate latches LINE_NOT_FOUND (stop + clear queue, so isIdle()).
    expectStopAtLine(expectedCm, /*found*/ false);
    ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::LINE_NOT_FOUND);

    // The next RETURN_TO_NODE tick inspects the Drive fault itself (the state is exempt from
    // the generic guard) and routes the EXPECTED LINE_NOT_FOUND through the off-map gate ->
    // OFF_NODE under FAULT. Tick until the FSM latches its fault.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        cooperativeTick();
    }

    ASSERT_TRUE(PathTracker.hasFault());
    // OFF_NODE, NOT DRIVE_FAULT (the off-map gate route, the whole point of the exemption).
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::OFF_NODE);

    // No re-anchor: the node stays the anchor; the FSM halted before completing the goal.
    ASSERT_EQ(Localization.getCurrentNode(), kLineNode);
}

// ===========================================================================
// TC-RN4c — short LINE return routes through the DEAD-RECKON branch (real FSM).
//
// `Inter1` (LINE) + a 6 cm push (3 < 6 < 12.02 axle lead) -> `_returnUsesLineSeek`
// FALSE even on a LINE anchor (#14): the distance gate routes the return through the
// dead-reckon (`driveDist().setMode(SYNC).tShape()`) branch. RETURN_TO_NODE then
// re-anchors via `clearOffset` (node id unchanged) — proving the distance gate, not
// the node kind alone, selects the branch. Asserts NO seek ran (no GYRO emission),
// the offset was cleared, the node is unchanged, and there is no fault.
// ===========================================================================

void test_TC_RN4c_short_line_return_dead_reckons_via_clear_offset()
{
    TEST_SCOPE("TC-RN4c: short LINE return (6 cm < axle lead) dead-reckons via clearOffset");

    // The anchor is LINE, yet a sub-axle-lead push must route through the dead-reckon branch.
    ASSERT_TRUE(FieldMap::getNodeType(kLineNode) == Routes::NodeType::LINE);
    // Sanity-pin the gate arithmetic for this exact push: 6 cm < the real axle lead.
    ASSERT_TRUE(6.0f < Config::LINE_SENSOR_AXLE_OFFSET_CM);
    ASSERT_TRUE(6.0f > Config::RETURN_TRIGGER_THRESHOLD_CM);

    // usesSeek == false: a short LINE return dead-reckons, emitting the profiled SYNC drive.
    const float expectedCm = driveReturnInFlight("Inter1", /*pushCm*/ 6.0f,
                                                 /*pushHeadingDeg*/ 0, /*usesSeek*/ false);
    (void) expectedCm;

    // It DID NOT seek — the dead-reckon branch issues a SYNC `driveDist`, never a GYRO seek
    // (so no `MoveGyro` and no `LINE_NOT_FOUND` path can fire). This is the load-bearing proof
    // that the distance gate (not the LINE kind) routed the return.
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, 0u);
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, 0u);
    ASSERT_FALSE(Drive.hasFault());

    // Retire the dead-reckon hop, then let RETURN_TO_NODE re-anchor via clearOffset + complete.
    expectBlindHop(expectedCm);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && (PathTracker.hasFault() == false && !PathTracker.isIdle());
         ++i)
    {
        cooperativeTick();
    }

    // clearOffset re-anchor: node UNCHANGED (Inter1), offset cleared, no fault, no seek.
    ASSERT_EQ(Localization.getCurrentNode(), kLineNode);
    const Offset2D off = Localization.getOffsetFromNode();
    ASSERT_TRUE(sqrtf(off.x * off.x + off.y * off.y) < Config::RETURN_TRIGGER_THRESHOLD_CM);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, 0u);    // re-affirm: never seeked
}

// ===========================================================================
// TC-RN-blind-ok — BLIND dead-reckon return success (real FSM).
//
// `Start` (BLIND) + a 20 cm push -> `_returnUsesLineSeek` false (no line to snap onto)
// -> dead-reckon (`driveDist().setMode(SYNC).tShape()`). RETURN_TO_NODE re-anchors via
// `clearOffset` (the dead-reckoned inverse drive IS the re-anchor; node id unchanged)
// and completes the goal. Asserts the node is unchanged (Start), the offset is cleared,
// and there is no fault.
// ===========================================================================

void test_TC_RN_blind_ok_dead_reckon_reanchors_via_clear_offset()
{
    TEST_SCOPE("TC-RN-blind-ok: BLIND dead-reckon return re-anchors Start via clearOffset");

    // Routing guard: the anchor is a BLIND node, so the dead-reckon branch is selected
    // regardless of distance.
    ASSERT_TRUE(FieldMap::getNodeType(kBlindNode) != Routes::NodeType::LINE);

    const float expectedCm = driveReturnInFlight("Start", /*pushCm*/ 20.0f,
                                                 /*pushHeadingDeg*/ 0, /*usesSeek*/ false);
    (void) expectedCm;

    // Dead-reckon: SYNC drive emitted, no seek.
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, 0u);
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, 0u);
    ASSERT_FALSE(Drive.hasFault());

    expectBlindHop(expectedCm);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && (PathTracker.hasFault() == false && !PathTracker.isIdle());
         ++i)
    {
        cooperativeTick();
    }

    // clearOffset re-anchor: node UNCHANGED (Start), offset cleared, no fault.
    ASSERT_EQ(Localization.getCurrentNode(), kBlindNode);
    const Offset2D off = Localization.getOffsetFromNode();
    ASSERT_TRUE(sqrtf(off.x * off.x + off.y * off.y) < Config::RETURN_TRIGGER_THRESHOLD_CM);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker RETURN_TO_NODE dispatch — real FSM per-kind routing "
                "(TC-RN-line-ok/line-noline/RN4c/blind-ok) + real getNodeType / axle-gate guards "
                "(TC4 / TC4c)\n");

    // --- Direct real-symbol classification / distance-gate guards (KEPT, not mirrors) ---
    test_TC4_real_get_node_type_discriminates_line_vs_blind();
    test_TC4c_line_seek_distance_gate_uses_real_axle_offset();

    // --- Real-FSM RETURN_TO_NODE routing ---
    test_TC_RN_line_ok_seek_reanchors_via_on_intersection();
    test_TC_RN_line_noline_routes_off_node_under_fault_gate();
    test_TC_RN4c_short_line_return_dead_reckons_via_clear_offset();
    test_TC_RN_blind_ok_dead_reckon_reanchors_via_clear_offset();

    // Keep the established final-line success marker (CI / human-readable), then report.
    if (wro_test::failCounter() == 0)
    {
        std::printf("RETURN TO NODE DISPATCH OK\n");
    }
    TEST_REPORT();
}
