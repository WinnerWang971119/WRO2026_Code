/**
 * @file test_path_tracker_stop_at_line.cpp
 * @brief Host-link test for the PathTracker STOP_AT_LINE slice, driving the REAL
 *        `PathTracker` + `Drive` + `Localization` FSMs through the shared
 *        cooperative-tick harness (issue #106, T5d).
 *
 * @details
 * Rewritten from the original mirror-slice test. The old file replicated the
 * STOP_AT_LINE state body (`replica_stopAtLineTick`), the dispatch drain ordering
 * (`replica_dispatchDrainOrdering`), and the seek-overshoot selector
 * (`replica_seekOvershoot`) inline — every one annotated `// MIRROR:
 * PathTracker.cpp:<line>`. Those FSM-slice mirrors are GONE: this file now drives
 * the production `PathTracker` FSM over the REAL fixture graph and asserts the
 * seek behaviour the FSM itself issues, plus the lower-MCU `MiniR4.DriveDC` seek
 * call the real `Drive.cpp` seek emits.
 *
 * What stays: the resolver truth-table TCs (TC1-TC7 / TC13 / TC20 over the frozen
 * `resolveStopAtLine`, and the TC4 seek-overshoot cases over the frozen
 * `resolveSensorOvershootTrim`). These are direct calls to the SAME header-only
 * `*Policy.h` symbols production links — they are NOT mirrors (a helper that
 * *calls* the real resolver is fine, per the plan AC), so they are kept verbatim
 * as TC-SL4.
 *
 * THE HOPS (`fixtures/routes_fixture.h`):
 *   - `Start -> Inter1`: a BLIND edge carrying the per-edge `stop_at_line="on"`
 *     override (`StopAtLineMode::ON`). `goTo(Inter1)` from `Start` resolves the
 *     seek (override rung 2) and dispatches `Drive.stopAtLine` under the pipeline
 *     SYNC bundle. Drives TC-SL1 (success re-anchor) and TC-SL2 (DRIVE_FAULT on
 *     no-line).
 *   - `Corner -> ProjB -> Zed`: the SENSOR-regime single-wheel pivot
 *     `(Corner, ProjB, Zed)` (trim 9.0 cm). `Corner -> ProjB` is a BLIND hop into
 *     a PROJECTION node, so `BLIND_PROJECTION_AUTO_SEEK` routes it through the
 *     same seek with the approaching-pivot overshoot trim. Drives TC-SL3.
 *
 * THE SEEK'S LOWER-MCU CALL (AC S1): the real `Drive.cpp` seek is CONTINUOUS, not
 * a single bounded `MoveGyroDegs`. Its `_updateStopAtLine` SEEK/ARMED phases emit
 * `_emitContinuous(mode, ...)` every tick, which for the SYNC bundle these hops
 * resolve to (`PIPELINE_CONFIG.blind == SYNC_TSHAPE` -> `DriveMode::SYNC`) calls
 * `MiniR4.DriveDC.MoveSync(deadbandSnap(mag), deadbandSnap(mag))` — recorded in
 * `MockDriveDC::moveSyncCallCount` / `lastMoveSyncL` / `lastMoveSyncR`. So the
 * representative lower-MCU seek observable for these hops is the recorded
 * `MoveSync` call, asserted alongside the harness's `Drive.isLineFound()` /
 * `Drive.getFault().kind` checks. (A GYRO-mode seek would instead stamp
 * `lastMoveGyroTarget` via `MoveGyro`; neither of these two fixture hops is GYRO.)
 *
 * Done-bit / encoder handshake: `expectStopAtLine(cm, found)` from the harness
 * encapsulates the full seek handshake — success arms the window with a debounced
 * `T_OR_CROSS` then rolls the overshoot to DONE; fault over-advances `M2` past
 * `expected + STOP_AT_LINE_MAX_MARGIN_CM` with the marker NONE so the real seek
 * latches `DriveClass::Fault::LINE_NOT_FOUND`. There is NO `Drive.mockSetIdle`;
 * idle is derived from the real Drive queue + the lower-MCU done-bit.
 */

#include "test_path_tracker_harness.h"

#include "src/nav/StopAtLinePolicy.h"  // resolveStopAtLine + StopAtLineResolution (real frozen resolver).
#include "src/nav/PivotTrimPolicy.h"   // resolveSensorOvershootTrim + SensorOvershootResolution (real frozen helper).

#include <cmath>
#include <cstdio>

namespace
{
// ===========================================================================
// Real-FSM seek driver — shared by TC-SL1 / TC-SL2 / TC-SL3.
//
// Drive the real PathTracker from a clean world until it dispatches the
// `Drive.stopAtLine` seek for the FIRST hop of `goTo(goalName)` (seated at
// `startName`), returning with the seek IN FLIGHT (SEEK phase, `Drive` busy, no
// tick yet advancing `M2` past the dispatch baseline). The caller then runs
// `expectStopAtLine(cm, found)` to retire the seek to a crossing or a no-line
// fault.
//
// Sequence the real FSM runs to get here (PathTracker.cpp):
//   IDLE -> DEQUEUE (pop goal, offset~0 return-gate -> PLAN_NEXT_HOP)
//        -> PLAN_NEXT_HOP (resolves direction + pre-edge turn; if the edge
//           heading differs from the seated facing, dispatches a pre-edge turn
//           and enters TURNING) -> [TURNING settles] -> _dispatchEdgeAfterTurn
//           (resolveStopAtLine().seek -> Drive.stopAtLine(...).async()) ->
//           STOP_AT_LINE.
//
// The pre-edge turn runs the CUSTOM TurnGyro profile (drives `Move(L,R)`), which
// completes only when the corrected IMU yaw lands within tolerance of the body
// heading. We park `mmL.mockYawDeg` at the hardware-frame value of the EDGE
// heading every tick so the turn settles immediately (harness CONTRACT 2). The
// seek's first `MoveSync` emission (the SYNC seek's continuous drive) is the
// signal the seek is in flight — the pre-turn drives `Move`, NOT `MoveSync`, so
// `moveSyncCallCount > 0` cleanly distinguishes "seek running" from "turning".
//
// @return The seek's expected distance (the edge's `distance_cm`) so the caller
//         can size `expectStopAtLine(cm, ...)` from the same source.
// ===========================================================================
float driveToSeekInFlight(const char* startName, const char* goalName)
{
    resetWorld();    // clears both Drive + PathTracker faults, so a prior TC-SL2 LINE_NOT_FOUND
                     // cannot leak into this seek (the order-independence guarantee now lives in
                     // the shared harness, not a per-driver workaround).

    seatAt(startName);    // bind world frame @ app-yaw 0 + declare Localization at the start node

    const uint8_t start    = nodeByName(startName);
    const uint8_t firstHop = routedFirstHop(start, nodeByName(goalName));

    // The seek hop is the FIRST hop; park the corrected IMU read at the edge's
    // forward heading so the pre-edge turn (if any) settles on the body target.
    const Routes::EdgeInfo& edge = FieldMap::getEdge(start, firstHop);
    const int16_t           hwHeading = hwYaw((int16_t) lroundf(edge.heading_deg));

    // Enqueue the FINAL node only; PathTracker walks NEXT_HOP. `.async()` so the
    // harness drives the cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(nodeByName(goalName)).async();

    // Tick until the SYNC seek emits its first `MoveSync` (seek in flight). Each tick
    // re-parks the IMU yaw at the body heading so a pre-edge turn cannot stall the loop.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
        if (MiniR4.DriveDC.moveSyncCallCount > 0)
        {
            return edge.distance_cm;    // seek dispatched + emitting; M2 still at the dispatch baseline
        }
    }

    ASSERT_TRUE(false);    // budget exhausted: the FSM never dispatched the seek
    return edge.distance_cm;
}

// ===========================================================================
// TC4 fault-clear cascade re-dispatch probe (#145 / ADR-067).
//
// Re-issue `goTo(goalName)` from `startName` on the CURRENT world — WITHOUT a
// `resetWorld()`, so nothing clears a latched fault behind the cascade under
// test — and report whether Drive accepts the command (a NEW `MoveSync` beyond
// the pre-`goTo` baseline) WITHOUT the FSM re-tripping a fault.
//
// `seatAt(startName)` re-anchors Localization + zeroes the operator offset (a
// clean PLAN path, no return-to-node prologue); it does NOT touch either fault
// surface, so a Drive fault the cascade FAILED to clear survives into the
// dispatch and the shared Drive-fault guard re-enters `Fault::DRIVE_FAULT` on
// the first non-IDLE tick (Drive's fault gate freezes `_dispatchNext`, so no
// `MoveSync` is ever emitted). With the cascade WORKING, Drive is clean, the
// seek dispatches its continuous SYNC drive, and no fault is raised.
//
// @return `true`  — a fresh `MoveSync` dispatched and PathTracker stayed unfaulted
//                   (the next dispatch did NOT re-trip `DRIVE_FAULT`).
//         `false` — the FSM re-faulted, or no dispatch happened within budget.
// ===========================================================================
bool reGoToDispatchesSeekWithoutFault(const char* startName, const char* goalName)
{
    // Re-anchor + zero offset (does NOT touch Drive/PathTracker fault state).
    seatAt(startName);

    const uint8_t           start     = nodeByName(startName);
    const uint8_t           firstHop  = routedFirstHop(start, nodeByName(goalName));
    const Routes::EdgeInfo& edge      = FieldMap::getEdge(start, firstHop);
    const int16_t           hwHeading = hwYaw((int16_t) lroundf(edge.heading_deg));

    const auto baseline = MiniR4.DriveDC.moveSyncCallCount;

    PathTracker.goTo(nodeByName(goalName)).async();

    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
        if (PathTracker.hasFault())
        {
            return false;    // re-tripped a fault: the cascade left Drive frozen.
        }
        if (MiniR4.DriveDC.moveSyncCallCount > baseline)
        {
            return true;     // a fresh seek dispatched — Drive accepted the command.
        }
    }
    return false;            // budget exhausted: the FSM never re-dispatched the seek.
}

// ===========================================================================
// Latch a real Drive `LINE_NOT_FOUND` fault (and its surfaced PathTracker
// `DRIVE_FAULT`) by running the `Start -> Inter1` seek to its max distance with
// no line — the SAME machinery TC-SL2 uses — leaving the FSM in FAULT with both
// fault surfaces set. Shared by the two TC4 cascade cases.
// ===========================================================================
void latchDriveFaultViaNoLineSeek()
{
    const float expectedCm = driveToSeekInFlight("Start", "Inter1");
    expectStopAtLine(expectedCm, /*found*/ false);    // Drive latches LINE_NOT_FOUND.

    // The shared Drive-fault guard maps it to PathTracker DRIVE_FAULT on the next tick(s).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        cooperativeTick();
    }

    ASSERT_TRUE(Drive.hasFault());
    ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::LINE_NOT_FOUND);
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::DRIVE_FAULT);
}
}    // namespace

// ===========================================================================
// Routing guards — pin the two seek hops so a future fixture re-weight fails
// LOUDLY rather than silently re-routing the hop under test (Fable amendment 2).
// ===========================================================================

void test_routing_start_to_inter1_is_the_stop_at_line_edge()
{
    TEST_SCOPE("routing: goTo(Inter1) from Start MUST take the stop_at_line='on' BLIND edge");

    const uint8_t start  = nodeByName("Start");
    const uint8_t inter1 = nodeByName("Inter1");

    // The fixture must resolve these names (the stable-name contract).
    ASSERT_TRUE(start != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    // Dijkstra's first hop Start -> Inter1 IS Inter1 (the direct seek edge).
    ASSERT_EQ(routedFirstHop(start, inter1), inter1);

    // The edge is a BLIND edge carrying the per-edge stop_at_line override = ON
    // (the rung-2 force-on the resolver keys on).
    const Routes::EdgeInfo& edge = FieldMap::getEdge(start, inter1);
    ASSERT_TRUE(edge.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(edge.stop_at_line_override == Routes::StopAtLineMode::ON);
}

void test_routing_corner_to_projb_is_the_sensor_pivot_seek()
{
    TEST_SCOPE("routing: goTo(Zed) from Corner MUST seek into ProjB (sensor-regime pivot)");

    const uint8_t corner = nodeByName("Corner");
    const uint8_t projb  = nodeByName("ProjB");
    const uint8_t zed    = nodeByName("Zed");

    ASSERT_TRUE(corner != FieldMap::NODE_NONE);
    ASSERT_TRUE(projb != FieldMap::NODE_NONE);
    ASSERT_TRUE(zed != FieldMap::NODE_NONE);

    // First hop Corner -> Zed is ProjB ...
    ASSERT_EQ(routedFirstHop(corner, zed), projb);
    // ... over a BLIND edge into a PROJECTION node (BLIND_PROJECTION_AUTO_SEEK fires) ...
    ASSERT_TRUE(FieldMap::getEdge(corner, projb).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::isProjectionNode(projb));
    ASSERT_TRUE(Routes::BLIND_PROJECTION_AUTO_SEEK);
    // ... and the (Corner, ProjB, Zed) single-wheel pivot exists with the trim the
    // sensor-overshoot resolver subtracts (next-next of ProjB toward Zed is Zed).
    ASSERT_EQ(routedFirstHop(projb, zed), zed);
    const Routes::PivotTurn* pivot = FieldMap::findPivotTurn(corner, projb, zed);
    ASSERT_TRUE(pivot != nullptr);
    ASSERT_TRUE(pivot->trim_cm > 0.0f);
}

// ===========================================================================
// TC-SL1 — STOP_AT_LINE success re-anchor (real FSM, real Drive seek).
//
// `Start -> Inter1`: the real FSM dispatches the seek; the harness scripts a
// debounced crossing + overshoot roll so the seek confirms the line and finishes
// DONE; the STOP_AT_LINE state then re-anchors via `Localization.onIntersection`
// and advances. Asserts the recorded lower-MCU seek call (AC S1) and the
// re-anchor onto the real node.
// ===========================================================================

void test_TC_SL1_success_reanchors_via_on_intersection()
{
    TEST_SCOPE("TC-SL1: STOP_AT_LINE success seeks, re-anchors onto Inter1, no fault");

    const uint8_t inter1 = nodeByName("Inter1");

    const float expectedCm = driveToSeekInFlight("Start", "Inter1");

    // The seek is in flight: the real Drive seek emitted its continuous SYNC drive
    // (the representative lower-MCU seek call — AC S1). `Start -> Inter1` resolves
    // to the pipeline SYNC bundle, so the seek runs `MoveSync`, not `MoveGyro`.
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, 0u);
    // The forward seek selected the FRONT board (reverse=false) at dispatch.
    ASSERT_FALSE(LineArray.getReverseActive());
    // No fault yet — the crossing has not been scripted.
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the seek to a confirmed crossing: arm the window, hold a debounced
    // T_OR_CROSS, roll the overshoot to DONE. The harness asserts Drive latched the
    // crossing (`isLineFound()`) and did NOT fault.
    expectStopAtLine(expectedCm, /*found*/ true);

    // The STOP_AT_LINE state saw `Drive.isIdle()` and re-anchored via
    // `Localization.onIntersection(_nextHop)` — a true physical re-localisation
    // (NOT a dead-reckon advance), landing the discrete node on Inter1.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && Localization.getCurrentNode() != inter1;
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), inter1);

    // Inter1 IS the goal, so the FSM settles without faulting.
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(Drive.isLineFound());
}

// ===========================================================================
// TC-SL2 — STOP_AT_LINE -> DRIVE_FAULT on no-line (real FSM, real Drive seek).
//
// `Start -> Inter1`: the real seek runs the full max distance with the marker
// NONE, latches `DriveClass::Fault::LINE_NOT_FOUND`; the FSM's shared Drive-fault
// guard maps it to `PathTracker::Fault::DRIVE_FAULT` and HALTS.
// ===========================================================================

void test_TC_SL2_no_line_surfaces_drive_fault()
{
    TEST_SCOPE("TC-SL2: STOP_AT_LINE no-line -> Drive LINE_NOT_FOUND -> PathTracker DRIVE_FAULT");

    const float expectedCm = driveToSeekInFlight("Start", "Inter1");
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, 0u);

    // Over-advance the encoder past `expected + STOP_AT_LINE_MAX_MARGIN_CM` with the
    // marker held NONE: the real seek's max-distance gate latches LINE_NOT_FOUND.
    expectStopAtLine(expectedCm, /*found*/ false);

    // Drive surfaced its own seek fault ...
    ASSERT_TRUE(Drive.hasFault());
    ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::LINE_NOT_FOUND);
    ASSERT_FALSE(Drive.isLineFound());

    // ... and the next PathTracker tick's shared Drive-fault guard maps it to
    // DRIVE_FAULT and halts the FSM (no silent dead-reckon fallback, ADR-027/AC7).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::DRIVE_FAULT);
}

// ===========================================================================
// TC-SL3 — seek-overshoot trim on the sensor-regime pivot (real FSM).
//
// `Corner -> ProjB -> Zed`: the BLIND->projection auto-seek into the
// `(Corner, ProjB, Zed)` pivot. The real `_dispatchEdgeAfterTurn` reduces the
// seek's post-detect overshoot by the approaching-pivot trim via the frozen
// `resolveSensorOvershootTrim` (the same symbol this test calls directly below).
// The fixture pivot trim (9.0 cm) is below the FRONT axle offset (12.02 cm), so
// the resolver does NOT clamp -> the real seek dispatches normally (a clamp would
// hard-fault PIVOT_TRIM_UNDERFLOW and emit no seek). Assert the real seek ran
// (its recorded lower-MCU call) AND that the frozen resolver produces the trimmed
// overshoot the dispatch fed it.
// ===========================================================================

void test_TC_SL3_seek_overshoot_trim_on_sensor_pivot()
{
    TEST_SCOPE("TC-SL3: BLIND->projection seek trims overshoot by the approaching pivot");

    const uint8_t corner = nodeByName("Corner");
    const uint8_t projb  = nodeByName("ProjB");
    const uint8_t zed    = nodeByName("Zed");

    // The trim the real FSM reads for this pivot (the approaching pivot at ProjB).
    const Routes::PivotTurn* pivot = FieldMap::findPivotTurn(corner, projb, zed);
    ASSERT_TRUE(pivot != nullptr);

    // The frozen resolver the FSM dispatch calls (KEPT — a direct call to the real
    // policy is not a mirror): a FRONT pivot seek rolls the FRONT axle offset minus
    // the approaching trim, clamped to the floor. For the fixture trim this is well
    // above the floor, so it does NOT clamp and the dispatch can arm the seek.
    const SensorOvershootResolution trimmed = resolveSensorOvershootTrim(
        Config::LINE_SENSOR_AXLE_OFFSET_CM, pivot->trim_cm, Config::PIVOT_TRIM_MIN_DRIVE_CM);
    ASSERT_FALSE(trimmed.clamped);
    // overshoot == offset - trim (the delta-short-of-node landing ADR-044 D1 requires),
    // strictly less than the untrimmed full offset.
    ASSERT_TRUE(std::fabs(trimmed.overshootCm
                          - (Config::LINE_SENSOR_AXLE_OFFSET_CM - pivot->trim_cm))
                < 1e-4f);
    ASSERT_TRUE(trimmed.overshootCm < Config::LINE_SENSOR_AXLE_OFFSET_CM);

    // Drive the REAL FSM seek over Corner -> ProjB. Because the resolver did NOT
    // clamp, the dispatch arms the seek rather than hard-faulting, so the seek is in
    // flight (its continuous SYNC drive emitted) — the lower-MCU evidence the FSM
    // applied the trimmed (non-clamping) overshoot and dispatched.
    const float expectedCm = driveToSeekInFlight("Corner", "Zed");
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, 0u);
    ASSERT_FALSE(PathTracker.hasFault());    // no PIVOT_TRIM_UNDERFLOW: the trim fit the budget

    // THE TRIM ASSERT (closes the TC-SL3 gap): observe the EXACT overshoot the real FSM fed the
    // seek's `Drive.stopAtLine(...).setOvershoot(...)` — Drive captured it into `_stopOvershoot` at
    // dispatch (Drive.cpp:1724), and `getStopOvershoot()` exposes it. It must equal the frozen
    // resolver's offset-minus-trim (the same value asserted above), and be strictly below the
    // untrimmed full axle offset. A regression that fed the full offset, fed 0, or skipped the trim
    // entirely would change this value and fail here — the direct resolver call alone could not.
    ASSERT_TRUE(std::fabs(Drive.getStopOvershoot() - trimmed.overshootCm) < 1e-4f);
    ASSERT_TRUE(Drive.getStopOvershoot() < Config::LINE_SENSOR_AXLE_OFFSET_CM);

    // Retire the seek to a confirmed crossing; the harness rolls the overshoot to
    // DONE and asserts Drive latched the line. The re-anchor lands on ProjB.
    expectStopAtLine(expectedCm, /*found*/ true);
    ASSERT_TRUE(Drive.isLineFound());

    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && Localization.getCurrentNode() != projb;
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), projb);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC4 (#145 / ADR-067) — Drive fault-clear cascade, real FSM.
//
// Latch a real Drive `LINE_NOT_FOUND` fault (surfacing PathTracker `DRIVE_FAULT`),
// then recover through `PathTracker.clearFault()` — the same call the BTN_DOWN
// mission reset and `declarePosition` route through. ADR-067 makes `clearFault()`
// cascade to `Drive.clearFault()`, so BOTH fault surfaces clear and the next
// `goTo()` dispatches instead of re-tripping `DRIVE_FAULT`. Before ADR-067 the
// Drive fault survived, Drive's queue stayed frozen, and the re-`goTo()` re-tripped
// `DRIVE_FAULT` — a reboot-only dead end.
// ===========================================================================

void test_TC4_clearFault_cascade_clears_drive_fault_and_redispatches()
{
    TEST_SCOPE("TC4: PathTracker.clearFault() cascades to Drive; next goTo does not re-trip DRIVE_FAULT");

    latchDriveFaultViaNoLineSeek();

    // The cascade under test (BTN_DOWN mission reset recovers through this exact call).
    PathTracker.clearFault();

    // Both fault surfaces cleared — the Drive half is the ADR-067 cascade (clearFault() alone
    // used to leave Drive latched; Drive.stop() does not clear it either).
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(Drive.hasFault());

    // The next dispatch issues a fresh seek WITHOUT re-entering DRIVE_FAULT — proof the cascade
    // un-froze Drive's command queue.
    ASSERT_TRUE(reGoToDispatchesSeekWithoutFault("Start", "Inter1"));
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the in-flight seek so Drive's `_stopState` returns to IDLE — `resetWorld()`'s
    // `Drive.begin()` does NOT clear sub-FSM state, so a left-running seek would poison the next
    // case (abort() runs `Drive.stop(true)` -> `_endStopAtLine`, restoring IDLE).
    PathTracker.abort();
}

// ===========================================================================
// TC4b (#145 / ADR-067) — the same cascade via the hard-cancel `abort()` path.
//
// `PathTracker.abort()` (which `Mission::abort()` and the safety stop route
// through) also cascades to `Drive.clearFault()`, so a hard-cancel after a
// latched Drive fault leaves BOTH surfaces clean and the next `goTo()` dispatches.
// ===========================================================================

void test_TC4b_abort_cascade_clears_drive_fault_and_redispatches()
{
    TEST_SCOPE("TC4b: PathTracker.abort() cascades to Drive; next goTo does not re-trip DRIVE_FAULT");

    latchDriveFaultViaNoLineSeek();

    // The abort cascade under test.
    PathTracker.abort();

    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(Drive.hasFault());    // ADR-067: abort() cascaded to Drive.clearFault().

    ASSERT_TRUE(reGoToDispatchesSeekWithoutFault("Start", "Inter1"));
    ASSERT_FALSE(Drive.hasFault());
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the in-flight seek so `_stopState` returns to IDLE for case isolation (see TC4).
    PathTracker.abort();
}

// ===========================================================================
// TC-SL4 (resolver truth-table — KEPT, not a mirror).
//
// Direct calls to the real frozen `resolveStopAtLine` (StopAtLinePolicy.h) and
// `resolveSensorOvershootTrim` (PivotTrimPolicy.h) — the SAME symbols production
// links. A helper that *calls* the real resolver is explicitly not a mirror (plan
// AC), so these vectors stay verbatim. Cross-language parity rows live in
// `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv`; the full
// 936-row C++ parity sweep runs in the sibling `test_stop_at_line_truth_table`
// target (`test_stop_at_line_truth_table.cpp`). These vectors are the kept
// representative in-FSM subset.
// ===========================================================================

// ---- resolveStopAtLine: precedence ladder + board degrade + motion bundle ----

static void test_TC1_auto_stop_at_line_edge_front_1board_seeks_sync_tshape()
{
    TEST_SCOPE("TC1: AUTO + STOP_AT_LINE edge, FRONT, 1-board -> seek=true, SYNC_TSHAPE");

    // AUTO defers to edge; edge is STOP_AT_LINE; FRONT board always exists.
    // Seeks under SYNC_TSHAPE (the bare STOP_AT_LINE edge has no velocity
    // profile of its own — Step 4 of the resolver). Legacy slice: no per-edge
    // override, no auto rule — the seek rides the legacy STOP_AT_LINE tag rung.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC2_auto_stop_at_line_edge_rear_2board_seeks_sync_tshape()
{
    TEST_SCOPE("TC2: AUTO + STOP_AT_LINE edge, REAR, 2-board -> seek=true, SYNC_TSHAPE");

    // REAR + 2 boards: boardOK = true (lineBoardCount >= 2).
    // AUTO defers to edge -> seekRequested = true via the legacy STOP_AT_LINE
    // tag rung. Seeks under SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC3_auto_stop_at_line_edge_rear_1board_degrades_to_pipeline_blind()
{
    TEST_SCOPE("TC3: AUTO + STOP_AT_LINE edge, REAR, 1-board -> seek=false, motionMode=pipelineBlind");

    // REAR + only 1 board: boardOK = false -> seek = false even though requested.
    // Dead-reckon fallback: edgeMode == STOP_AT_LINE -> motionMode = pipelineBlind.
    // pipelineBlind is SYNC_TSHAPE (not STOP_AT_LINE) so no second defensive
    // degrade fires.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC4_on_gyro_tshape_edge_front_1board_seeks_gyro_tshape()
{
    TEST_SCOPE("TC4: ON + GYRO_TSHAPE edge, FRONT, 1-board -> seek=true, GYRO_TSHAPE");

    // ON forces a seek even on a plain GYRO_TSHAPE edge. Step 4: edgeMode is
    // not STOP_AT_LINE, so motionMode keeps the edge's typed mode: GYRO_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
}

static void test_TC5_on_raw_flat_edge_rear_2board_seeks_raw_flat()
{
    TEST_SCOPE("TC5: ON + RAW_FLAT edge, REAR, 2-board -> seek=true, RAW_FLAT");

    // ON forces seek; REAR + 2 boards satisfies boardOK. Edge is RAW_FLAT
    // (not STOP_AT_LINE), so Step 4 keeps motionMode = RAW_FLAT.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::RAW_FLAT,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
}

static void test_TC6_off_stop_at_line_edge_front_1board_no_seek()
{
    TEST_SCOPE("TC6: OFF + STOP_AT_LINE edge, FRONT, 1-board -> seek=false, motionMode=pipelineBlind");

    // OFF always suppresses the seek regardless of edge mode or board count.
    // Dead-reckon: edgeMode == STOP_AT_LINE -> base = pipelineBlind (SYNC_TSHAPE).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::OFF,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC6_off_stop_at_line_edge_front_2board_no_seek()
{
    TEST_SCOPE("TC6: OFF + STOP_AT_LINE edge, FRONT, 2-board -> seek=false (OFF wins regardless of board count)");

    // Same as above with 2 boards — OFF still suppresses.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::OFF,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC7_auto_plain_blind_edge_no_seek()
{
    TEST_SCOPE("TC7: AUTO + SYNC_TSHAPE edge, FRONT -> seek=false, SYNC_TSHAPE");

    // AUTO on a non-STOP_AT_LINE edge with no auto rule active: seekRequested =
    // false. Dead-reckon: edgeMode != STOP_AT_LINE -> base = edgeMode =
    // SYNC_TSHAPE (no defensive degrade, base is already a concrete mode).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC13_on_wins_over_every_plain_blind_mode()
{
    TEST_SCOPE("TC13: ON wins (seek=true) over DEFAULT / GYRO_TSHAPE / RAW_FLAT blind edges — FRONT board");

    // ON is unconditional — it forces seek regardless of which typed edge mode
    // the edge resolved to. motionMode keeps the edge's type (not STOP_AT_LINE,
    // so Step 4 leaves it unchanged).
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::ON, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::GYRO_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_TRUE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
    }
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::ON, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::RAW_FLAT,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_TRUE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
    }
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::ON, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::SYNC_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_TRUE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    }
}

static void test_TC13_off_suppresses_regardless_of_edge_mode_or_board_count()
{
    TEST_SCOPE("TC13: OFF suppresses seek for STOP_AT_LINE, SYNC_TSHAPE, GYRO_TSHAPE, RAW_FLAT edges");

    // OFF is unconditional: seek = false for every edge mode, every board count,
    // every direction. Dead-reckon fallback applies (Step 5).
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::OFF, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::STOP_AT_LINE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 2, false});
        ASSERT_FALSE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    }
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::OFF, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::GYRO_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::REAR, 2, false});
        ASSERT_FALSE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
    }
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::OFF, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::RAW_FLAT,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_FALSE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
    }
}

static void test_TC13_on_rear_1board_degrades_seek_but_keeps_edge_motionMode()
{
    TEST_SCOPE("TC13: ON + GYRO_TSHAPE, REAR, 1-board -> seek=false (no rear board), motionMode=GYRO_TSHAPE");

    // ON requests the seek but boardOK = false (REAR, 1 board). seek = false.
    // Dead-reckon: edgeMode = GYRO_TSHAPE (not STOP_AT_LINE) -> base = GYRO_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
}

static void test_TC13_pipeline_stop_at_line_defensively_degrades_to_sync_tshape()
{
    TEST_SCOPE("TC13: board-degrade + pipelineBlind==STOP_AT_LINE -> motionMode defensively SYNC_TSHAPE");

    // Corner case: the pipeline default is itself STOP_AT_LINE. If a STOP_AT_LINE
    // edge does not seek (e.g. REAR + 1-board), Step 5: base = pipelineBlind =
    // STOP_AT_LINE; the defensive second degrade kicks in -> SYNC_TSHAPE so the
    // returned motionMode is never STOP_AT_LINE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::STOP_AT_LINE,    // unusual but must not leak
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

// ---- resolveStopAtLine: TC20 — edgeOverride + autoRuleActive rungs + precedence ----

static void test_TC20_edge_override_on_forces_seek()
{
    TEST_SCOPE("TC20: edgeOverride==ON forces seek on a plain (non-STOP_AT_LINE) edge");

    // No call-site verb (AUTO), no legacy tag (edge is SYNC_TSHAPE), no auto
    // rule — but the per-edge override is ON, so rung 2 forces the seek. The
    // edge is not STOP_AT_LINE, so motionMode keeps its typed mode (SYNC_TSHAPE).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::ON,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC20_edge_override_off_suppresses_seek()
{
    TEST_SCOPE("TC20: edgeOverride==OFF suppresses seek even with auto rule active");

    // edgeOverride OFF (rung 2) beats both the auto rule (rung 4) and would beat
    // the legacy tag (rung 3) — see the dedicated test below. With the seek
    // suppressed and a non-STOP_AT_LINE edge, the hop dead-reckons under the
    // edge's own SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::OFF,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ true});    // auto rule wants to seek, but OFF wins

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC20_edge_override_off_beats_legacy_stop_at_line_tag()
{
    TEST_SCOPE("TC20: edgeOverride==OFF beats the legacy BlindMode::STOP_AT_LINE tag -> seek=false");

    // The historical "STOP_AT_LINE tag means seek" behaviour is rung 3, BELOW
    // the per-edge override at rung 2. A STOP_AT_LINE-tagged edge with an
    // explicit edgeOverride==OFF must NOT seek — the override wins. Dead-reckon:
    // edgeMode == STOP_AT_LINE -> base = pipelineBlind (SYNC_TSHAPE).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::OFF,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC20_call_off_beats_edge_override_on()
{
    TEST_SCOPE("TC20: call OFF (rung 1) beats edgeOverride==ON (rung 2) -> seek=false");

    // Call-site OFF is the top rung — it suppresses the seek even when the
    // per-edge override says ON. Non-STOP_AT_LINE edge dead-reckons under its
    // own SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::OFF,
        /*edgeOverride*/   Routes::StopAtLineMode::ON,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC20_call_on_beats_edge_override_off()
{
    TEST_SCOPE("TC20: call ON (rung 1) beats edgeOverride==OFF (rung 2) -> seek=true");

    // The symmetric case: call-site ON forces the seek even when the per-edge
    // override says OFF. Edge keeps its typed RAW_FLAT mode.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::OFF,
        /*edgeMode*/       Routes::BlindMode::RAW_FLAT,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
}

static void test_TC20_auto_rule_active_front_seeks()
{
    TEST_SCOPE("TC20: autoRuleActive (no higher rung) -> seek=true (FRONT)");

    // The auto rung (rung 4) fires when nothing above it decided: call AUTO,
    // edgeOverride DEFAULT, edge is a plain non-STOP_AT_LINE mode. FRONT board
    // always services the seek -> seek=true. motionMode keeps the edge's GYRO_TSHAPE.
    //
    // Issue #64: `autoRuleActive` here models the LOCAL caller computation
    // `Routes::BLIND_PROJECTION_AUTO_SEEK && isProjectionNode(_nextHop)` — a BLIND
    // edge whose DESTINATION is a projection node. The end-to-end LOCAL read over
    // the real graph is exercised by TC-SL3 above (Corner -> ProjB).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ true});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
}

static void test_TC20_auto_rule_inactive_no_seek()
{
    TEST_SCOPE("TC20: autoRuleActive==false on a plain edge -> no auto seek (seek=false)");

    // With the auto rule off and nothing else requesting, the resolver falls to
    // the terminal "never seek" rung. Plain edge dead-reckons under SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC20_auto_rule_rear_1board_degrades()
{
    TEST_SCOPE("TC20: autoRuleActive + REAR + 1-board -> seek=false (board degrade)");

    // The auto rule wants the seek but a REAR seek needs a second board
    // (lineBoardCount >= 2). With only 1 board, boardOK = false -> seek = false.
    // Dead-reckon under the edge's SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ true});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC20_auto_rule_rear_2board_seeks()
{
    TEST_SCOPE("TC20: autoRuleActive + REAR + 2-board -> seek=true");

    // Same auto-rule request as above, but a 2-board chassis services the REAR
    // seek (boardOK = true). seek = true. motionMode keeps the edge's GYRO_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ true});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
}

static void test_TC20_degenerate_full_domain_pipeline_stop_at_line_no_seek()
{
    TEST_SCOPE("TC20: AUTO, no auto rule, edge!=STOP_AT_LINE, pipeline==STOP_AT_LINE -> seek=false");

    // Full-domain degenerate row (the case that proves there is NO pipeline-
    // default seek rung): callFlag AUTO, edgeOverride DEFAULT, the resolved edge
    // is a concrete non-STOP_AT_LINE mode, the auto rule is off — yet the
    // PIPELINE default happens to be STOP_AT_LINE. No rung requests a seek, so
    // seek=false. Dead-reckon: edgeMode != STOP_AT_LINE -> base = edgeMode
    // (RAW_FLAT); the pipeline STOP_AT_LINE never leaks into motionMode.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::RAW_FLAT,
        /*pipelineBlind*/  Routes::BlindMode::STOP_AT_LINE,    // pipeline default is itself STOP_AT_LINE
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
}

// ---- resolveSensorOvershootTrim: TC4 seek-overshoot helper (frozen, kept) ----

/// Approx-equality for the float overshoot (no float ASSERT macro exists).
/// Matches the 1e-4f tolerance the sibling `test_sensor_overshoot_policy` uses.
static bool seekOvershootApprox(float a, float b)
{
    const float d = a - b;
    return (d < 0.0f ? -d : d) < 1e-4f;
}

static void test_TC4_seek_into_approaching_pivot_trims_overshoot()
{
    TEST_SCOPE("TC4: seek into an approaching pivot reduces overshoot by the trim (offset - trim)");

    // Approaching pivot active with trim=9.0: the seek overshoot becomes
    // offset - trim = 12.02 - 9.0 = 3.02 cm (well above the 0.5 floor, so not
    // clamped). This is the delta-short-of-node landing ADR-044 D1 requires.
    const SensorOvershootResolution r = resolveSensorOvershootTrim(
        Config::LINE_SENSOR_AXLE_OFFSET_CM, /*approachingTrimCm*/ 9.0f,
        Config::PIVOT_TRIM_MIN_DRIVE_CM);

    ASSERT_TRUE(seekOvershootApprox(r.overshootCm, 3.02f));
    ASSERT_FALSE(r.clamped);
}

static void test_TC4_seek_without_approaching_pivot_keeps_full_offset()
{
    TEST_SCOPE("TC4: seek with NO approaching pivot keeps the full default overshoot (AC2)");

    // No approaching pivot -> trim 0 -> overshoot stays the full default
    // LINE_SENSOR_AXLE_OFFSET_CM = 12.02 cm, unchanged from the legacy behaviour
    // (AC2: a departing-into-seek or plain seek hop is untouched).
    const SensorOvershootResolution r = resolveSensorOvershootTrim(
        Config::LINE_SENSOR_AXLE_OFFSET_CM, /*approachingTrimCm*/ 0.0f,
        Config::PIVOT_TRIM_MIN_DRIVE_CM);

    ASSERT_TRUE(seekOvershootApprox(r.overshootCm, Config::LINE_SENSOR_AXLE_OFFSET_CM));
    ASSERT_FALSE(r.clamped);
}

static void test_TC4_seek_rear_uses_rear_offset()
{
    TEST_SCOPE("TC4: a REAR seek rolls the REAR axle offset, not FRONT (#2 direction-resolve)");

    // A REAR 2-board seek resolves its overshoot from the REAR sensor-to-axle
    // offset (7.68 cm), mirroring the LINE overshoot path — not the FRONT 12.02.
    // A REAR seek never carries a pivot (pivots are FRONT-only in v1), so the trim
    // is 0 and the overshoot is the full REAR offset (no reduction, no clamp).
    const SensorOvershootResolution r = resolveSensorOvershootTrim(
        Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM, /*approachingTrimCm*/ 0.0f,
        Config::PIVOT_TRIM_MIN_DRIVE_CM);

    ASSERT_TRUE(seekOvershootApprox(r.overshootCm, Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM));
    ASSERT_FALSE(r.clamped);
}

static void test_TC4_seek_overbudget_trim_clamps_to_floor()
{
    TEST_SCOPE("TC4: over-budget approaching trim clamps the seek overshoot to the floor");

    // Pathological trim larger than the axle offset (15.588 > 12.02): the shared
    // subtract-and-clamp floors the overshoot at PIVOT_TRIM_MIN_DRIVE_CM = 0.5
    // and reports clamped=true so the dispatch can hard-fault (ADR-044 D4) — the
    // helper itself still returns the floored value; the firmware faults on it.
    const SensorOvershootResolution r = resolveSensorOvershootTrim(
        Config::LINE_SENSOR_AXLE_OFFSET_CM, /*approachingTrimCm*/ 15.588f,
        Config::PIVOT_TRIM_MIN_DRIVE_CM);

    ASSERT_TRUE(seekOvershootApprox(r.overshootCm, Config::PIVOT_TRIM_MIN_DRIVE_CM));
    ASSERT_TRUE(r.clamped);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker STOP_AT_LINE — real FSM seek (TC-SL1/SL2/SL3), Drive "
                "fault-clear cascade (TC4/TC4b, #145/ADR-067) + frozen resolver truth table "
                "(TC-SL4: TC1-TC7/TC13/TC20/TC4)\n");

    // --- Real-FSM hops (TC-SL1 / TC-SL2 / TC-SL3) -----------------------------
    test_routing_start_to_inter1_is_the_stop_at_line_edge();
    test_routing_corner_to_projb_is_the_sensor_pivot_seek();
    test_TC_SL1_success_reanchors_via_on_intersection();
    test_TC_SL2_no_line_surfaces_drive_fault();
    test_TC_SL3_seek_overshoot_trim_on_sensor_pivot();

    // --- TC4 (#145 / ADR-067): Drive fault-clear cascade (clearFault + abort) ---
    test_TC4_clearFault_cascade_clears_drive_fault_and_redispatches();
    test_TC4b_abort_cascade_clears_drive_fault_and_redispatches();

    // --- TC-SL4: frozen resolver truth table (KEPT — direct calls, not mirrors) ---
    // resolveStopAtLine: TC1-TC7, the kept representative in-FSM subset (full
    // domain -> the sibling test_stop_at_line_truth_table harness).
    test_TC1_auto_stop_at_line_edge_front_1board_seeks_sync_tshape();
    test_TC2_auto_stop_at_line_edge_rear_2board_seeks_sync_tshape();
    test_TC3_auto_stop_at_line_edge_rear_1board_degrades_to_pipeline_blind();
    test_TC4_on_gyro_tshape_edge_front_1board_seeks_gyro_tshape();
    test_TC5_on_raw_flat_edge_rear_2board_seeks_raw_flat();
    test_TC6_off_stop_at_line_edge_front_1board_no_seek();
    test_TC6_off_stop_at_line_edge_front_2board_no_seek();
    test_TC7_auto_plain_blind_edge_no_seek();

    // resolveStopAtLine: TC13 additional vectors.
    test_TC13_on_wins_over_every_plain_blind_mode();
    test_TC13_off_suppresses_regardless_of_edge_mode_or_board_count();
    test_TC13_on_rear_1board_degrades_seek_but_keeps_edge_motionMode();
    test_TC13_pipeline_stop_at_line_defensively_degrades_to_sync_tshape();

    // resolveStopAtLine: TC20 edgeOverride + autoRuleActive rungs + precedence.
    test_TC20_edge_override_on_forces_seek();
    test_TC20_edge_override_off_suppresses_seek();
    test_TC20_edge_override_off_beats_legacy_stop_at_line_tag();
    test_TC20_call_off_beats_edge_override_on();
    test_TC20_call_on_beats_edge_override_off();
    test_TC20_auto_rule_active_front_seeks();
    test_TC20_auto_rule_inactive_no_seek();
    test_TC20_auto_rule_rear_1board_degrades();
    test_TC20_auto_rule_rear_2board_seeks();
    test_TC20_degenerate_full_domain_pipeline_stop_at_line_no_seek();

    // resolveSensorOvershootTrim: TC4 seek-overshoot trim vectors.
    test_TC4_seek_into_approaching_pivot_trims_overshoot();
    test_TC4_seek_without_approaching_pivot_keeps_full_offset();
    test_TC4_seek_rear_uses_rear_offset();
    test_TC4_seek_overbudget_trim_clamps_to_floor();

    TEST_REPORT();
}
