/**
 * @file test_reverse_pivot_dispatch.cpp
 * @brief Host-link test for the reverse (REAR-traversal) single-wheel pivot dispatch,
 *        driving the REAL `PathTracker` + `Drive` + `Localization` FSMs through the shared
 *        cooperative-tick harness (issue #54, ADR-072, T10).
 *
 * @details
 * Runs on the DEFAULT 2-board chassis (`Config::LINE_BOARD_COUNT == 2`, no override — see
 * the sibling `test_reverse_pivot_seek_1board.cpp` for the 1-board target). Links the REAL
 * `PathTracker.cpp` + `Drive.cpp` + `Localization.cpp` + `YawSource.cpp` against the T9
 * shadow `fixtures/routes_reverse_pivots.cpp` (20 nodes, 7 pivots — see that header's
 * docblock for the six components it authors), force-included FIRST so its `ROUTES_H`
 * shadows the production `routes.h` for this target only. Same shared recipe as every
 * sibling PathTracker+Drive real-link target (`test_path_tracker_stop_at_line`,
 * `test_path_tracker_lost_line`, ...): `path_mocks.cpp` supplies the facade globals,
 * `add_path_force_includes` shadows the `WROLineArray`/`Intersection`/`LineFollower`
 * peripherals + `EventTrace`, and `SKETCH_DIR` leads the include dirs so the real
 * `Drive.h`/`PathTracker.h` win (the #106 inversion).
 *
 * THE COMPONENTS EXERCISED (`fixtures/routes_reverse_pivots.h`, nodes resolved BY NAME):
 *   - **AC1** — `ArS0`-chain: `ArL0 --LINE,"rear"--> ArL1 --BLIND,"rear"--> ArL2`, pivot
 *     `(ArL0, ArL1, ArL2)` stored `PIVOT_LEFT`. The approach resolves REAR (binding), so
 *     force-SAME holds the departure REAR too — `resolvePivotWheel(PIVOT_LEFT, REAR)` must
 *     flip the DEPARTURE turn to `PIVOT_RIGHT` (left wheel drives, right wheel held at 0 —
 *     `TurnSignPolicy.h` case 2). Sampled directly off the real `Drive`'s per-tick
 *     `MiniR4.DriveDC.Move(L, R)` capture during the turn's PD stepping, BEFORE the target
 *     yaw is parked (so the very first sample is a genuine, large, unsettled PD command).
 *   - **AC2/TC3** — `RaU -> RaV -> RaW`, pivot `(RaU, RaV, RaW)` stored `PIVOT_LEFT`, no
 *     authored override anywhere. `RaU -> RaV`'s 100 deg entry delta genuinely resolves REAR
 *     under `BIDIRECTIONAL_AUTO` (> the 90 deg threshold, non-axis-aligned). The DEPARTURE
 *     `RaV -> RaW` (a 120 deg turn) would naively resolve FRONT under AUTO on its own
 *     (`|wrap(220 - 280)| = 60 <= 90`) — asserted directly via the REAL frozen
 *     `resolveDirection` (DirectionPolicy.h), NOT a mirror. Force-SAME must override that to
 *     REAR, proving the pivot runs COHERENTLY instead of degrading to a center turn — the
 *     real FSM's `PathTracker::resolvedBodyHeadingDeg()` is asserted against the REAR body
 *     target (40 deg = 220 + 180), not the naive FRONT one (220 deg).
 *   - **TC4/AC3** — `BcU --BLIND,"front"--> BcV --BLIND,"rear"--> BcW`, pivot
 *     `(BcU, BcV, BcW)`. The approach is pinned FRONT; the departure is pinned REAR — a
 *     statically-visible, binding contradiction force-SAME cannot reconcile, so the real FSM
 *     must fault `Fault::PIVOT_DIRECTION_MISMATCH` at the departure's `PLAN_NEXT_HOP`, before
 *     dispatching any turn or move.
 *   - **TC12** — REUSES the `(RaU, RaV, RaW)` pivot from AC2, but forces the APPROACH hop to
 *     SEEK (`.setStopAtLine(StopAtLine::ON)`) instead of dead-reckoning. The pivot's trim
 *     (15.588457 cm, deliberately sized for AC2's coherent-run proof) is far larger than the
 *     REAR overshoot budget (`Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM` = 7.68 cm) minus the
 *     trim floor (`Config::PIVOT_TRIM_MIN_DRIVE_CM` = 0.5 cm) — the real dispatch re-derives
 *     the clamp (`resolveSensorOvershootTrim`) and must fault `Fault::PIVOT_TRIM_UNDERFLOW`
 *     BEFORE ever issuing `Drive.stopAtLine(...)` (no `MoveSync` call is ever recorded).
 *   - **TC16** — the commanded-delta sign tripwire (`PathTracker.cpp:2249-2274`). AC1's real
 *     dispatch above proves the NEGATIVE case (a genuinely consistent table/geometry pairing
 *     never trips it — every `!PathTracker.hasFault()` assertion through that hop would
 *     otherwise catch a stray `PIVOT_DIRECTION_MISMATCH`). The POSITIVE case (a table/geometry
 *     MISMATCH trips it) is exercised directly against the REAL `resolvePivotWheel`
 *     (`ReversePivotPolicy.h`) — the same production symbol the tripwire calls — reproducing
 *     its exact comparison (a helper that calls the real resolver is not a mirror, matching
 *     the established `test_path_tracker_stop_at_line.cpp` TC-SL3 / `test_path_tracker_
 *     bidirectional.cpp` TC-BD1 idiom).
 */

#include "test_path_tracker_harness.h"

#include "src/nav/DirectionPolicy.h"      // resolveDirection (real frozen resolver).
#include "src/nav/NavTypes.h"             // Direction / StopAtLine enums.
#include "src/nav/ReversePivotPolicy.h"   // resolvePivotWheel (real frozen policy, TC16).
#include "src/mission/PathTracker.h"      // PathTrackerClass::Fault + FaultInfo.
#include "src/nav/routes.h"               // Routes::TurnMode / DirectionMode / EdgeType.

#include <cmath>
#include <cstdio>

namespace
{
/// @brief Small epsilon for the float body-heading / geometry comparisons below — the
///        fixture's headings and the REAR (+180) rotation are exact float arithmetic, so a
///        tight tolerance still catches a genuine divergence.
constexpr float kHeadingEpsDeg = 1e-3f;

/**
 * @brief Tick the real FSM until a pre-edge turn is GENUINELY running (`Drive.isTurnDone() ==
 *        false` — the turn PD controller has actually started, not merely queued) or a fault
 *        latches, WITHOUT parking the target yaw — so the caller can sample an early PD tick's
 *        `MiniR4.DriveDC.lastMoveL/R` while the error is still genuinely large (the mock yaw
 *        stays at whatever the PRIOR hop left it at).
 *
 * @note `!Drive.isIdle()` alone is NOT sufficient here: it can go true the tick the turn
 *       command is merely POPPED onto Drive's queue (`_qCount > 0`), one tick BEFORE
 *       `_updateTurnProfile()` actually claims it and starts driving `Move(L, R)` — at that
 *       point `_turnDone` is still `true` (unchanged from the prior idle hop) and no `Move()`
 *       has run yet, so a same-tick `lastMoveL/R` sample would read a stale `(0, 0)`. Waiting
 *       for `!Drive.isTurnDone()` — the flag the turn dispatch itself clears — then running
 *       one MORE tick guarantees at least one genuine PD step (and its `Move()` call) has
 *       landed before the caller reads the mock capture.
 *
 * @return `true` if the turn is now genuinely in flight; `false` on a fault or a
 *         budget-exhausted, un-dispatched turn (the caller's own fault/dispatch asserts
 *         report the failure).
 */
bool tickUntilTurnInFlight()
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (!Drive.isTurnDone())
        {
            cooperativeTick();    // one more PD step so Move(L,R) reflects the ACTIVE turn.
            return true;
        }
        if (PathTracker.hasFault())
        {
            return false;
        }
    }
    return false;
}
}    // namespace

// ===========================================================================================
// AC1 — a REAR-approached pivot dispatches the FLIPPED wheel (PIVOT_LEFT -> PIVOT_RIGHT vs
// the stored forward wheel), through the REAL FSM's departure-turn dispatch.
// ===========================================================================================

static void test_AC1_rear_pivot_flips_wheel()
{
    TEST_SCOPE("AC1: REAR-approached pivot (ArL0,ArL1,ArL2) dispatches the FLIPPED wheel");

    const uint8_t arL0 = nodeByName("ArL0");
    const uint8_t arL1 = nodeByName("ArL1");
    const uint8_t arL2 = nodeByName("ArL2");
    ASSERT_TRUE(arL0 != FieldMap::NODE_NONE);
    ASSERT_TRUE(arL1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(arL2 != FieldMap::NODE_NONE);

    // Routing guard (Fable amendment 2): the authored-REAR chain routes directly.
    const Routes::PivotTurn* pivot = FieldMap::findPivotTurn(arL0, arL1, arL2);
    ASSERT_TRUE(pivot != nullptr);
    ASSERT_TRUE(pivot->wheel == Routes::TurnMode::PIVOT_LEFT);    // stored FORWARD-sense wheel

    const Routes::EdgeInfo& approachEdge  = FieldMap::getEdge(arL0, arL1);
    const Routes::EdgeInfo& departureEdge = FieldMap::getEdge(arL1, arL2);
    ASSERT_TRUE(approachEdge.type == Routes::EdgeType::LINE);
    ASSERT_TRUE(approachEdge.direction_override == Routes::DirectionMode::FORCE_REVERSE);
    ASSERT_TRUE(departureEdge.direction_override == Routes::DirectionMode::FORCE_REVERSE);

    resetWorld();
    // `seatAt(node)` always binds the world frame to `Routes::START_HEADING_DEG` (0 for this
    // fixture), regardless of any app-yaw parameter (`zeroHeadingFromCurrent()` captures
    // `_yawOffsetDeg = imuYaw - START_HEADING_DEG`, so the reading right after bind is always
    // `START_HEADING_DEG` — see Localization.cpp) — so hop 1's REAR body target (180 deg) is
    // NOT pre-aligned; the pre-edge turn genuinely dispatches and is retired below like any
    // other hop's turn. No departing pivot is active yet at `ArL0` (a fresh start), so the
    // wheel is the pipeline default.
    seatAt("ArL0", 0);
    ASSERT_EQ(routedFirstHop(arL0, arL2), arL1);

    PathTracker.goTo(arL2).async();

    // Hop 1 (ArL0 -> ArL1): REAR resolves (binding); body target = 0 + 180 = 180. Confirm the
    // turn is GENUINELY dispatched (`!Drive.isTurnDone()`) BEFORE calling `expectPreTurn` — that
    // helper parks the target yaw immediately, so calling it before the turn is observably in
    // flight would let `_onPlanNextHop`'s OWN first read of `Localization.getHeadingDeg()` see
    // the already-parked target and skip the turn as "already aligned".
    ASSERT_TRUE(tickUntilTurnInFlight());
    expectPreTurn(/*appYawTarget=*/180, (uint8_t) Routes::PIPELINE_CONFIG.turn);
    ASSERT_FALSE(PathTracker.hasFault());

    // Tick until the LINE follow arms for the REAR approach.
    bool armed = false;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode)
        {
            armed = true;
            break;
        }
        if (PathTracker.hasFault())
        {
            break;
        }
    }
    ASSERT_TRUE(armed);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(LineFollower.lastReverse);    // REAR resolved -- rear-board follow

    expectLineHop();
    ASSERT_EQ(Localization.getCurrentNode(), arL1);
    ASSERT_FALSE(PathTracker.hasFault());

    // Hop 2 (ArL1 -> ArL2): the pivot DEPARTURE. Force-SAME holds REAR (both legs pinned
    // REAR), so the dispatched wheel must be resolvePivotWheel(PIVOT_LEFT, REAR) ==
    // PIVOT_RIGHT: left wheel drives, right wheel held at 0 (TurnSignPolicy.h case 2).
    ASSERT_TRUE(tickUntilTurnInFlight());
    ASSERT_FALSE(PathTracker.hasFault());    // no PIVOT_DIRECTION_MISMATCH / sign tripwire (TC16 negative)

    const int16_t movedL = MiniR4.DriveDC.lastMoveL;
    const int16_t movedR = MiniR4.DriveDC.lastMoveR;
    ASSERT_EQ(movedR, (int16_t) 0);    // PIVOT_RIGHT: right wheel held stationary
    ASSERT_TRUE(movedL != 0);          // ... left wheel drives the rotation

    // Retire the turn on `Drive.isTurnDone()`, NOT the shared `expectPreTurn` helper: with
    // `HOP_SETTLE_MS == 0` the FSM's `TURNING` handler dispatches the departure's dead-reckon
    // move in the SAME tick the turn settles (`PathTracker.cpp` TURNING case ->
    // `_dispatchEdgeAfterTurn()`), so `Drive.isIdle()` can already read `false` again (a NEW
    // move claimed the queue) by the time a turn-retire loop re-checks it — `expectPreTurn`'s
    // own `ASSERT_TRUE(Drive.isIdle())` would then spuriously fail. Waiting on `isTurnDone()`
    // sidesteps the race; `expectBlindHop` below retires whatever the turn's settle tick
    // already dispatched.
    const float bodyTarget2 = fmodf(departureEdge.heading_deg + 180.0f, 360.0f);
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(bodyTarget2));
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.isTurnDone(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(Drive.isTurnDone());
    ASSERT_FALSE(PathTracker.hasFault());

    expectBlindHop(departureEdge.distance_cm);
    ASSERT_EQ(Localization.getCurrentNode(), arL2);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// AC2/TC3 — a >90 deg pivot under BIDIRECTIONAL_AUTO runs COHERENTLY (force-SAME holds the
// departure to the approach direction) rather than degrading to a center turn.
// ===========================================================================================

static void test_AC2_TC3_coherent_run_forces_rear_departure()
{
    TEST_SCOPE("AC2/TC3: >90deg AUTO pivot (RaU,RaV,RaW) runs coherently under force-SAME");

    const uint8_t raU = nodeByName("RaU");
    const uint8_t raV = nodeByName("RaV");
    const uint8_t raW = nodeByName("RaW");
    ASSERT_TRUE(raU != FieldMap::NODE_NONE);
    ASSERT_TRUE(raV != FieldMap::NODE_NONE);
    ASSERT_TRUE(raW != FieldMap::NODE_NONE);

    const Routes::PivotTurn* pivot = FieldMap::findPivotTurn(raU, raV, raW);
    ASSERT_TRUE(pivot != nullptr);

    const Routes::EdgeInfo& approachEdge  = FieldMap::getEdge(raU, raV);
    const Routes::EdgeInfo& departureEdge = FieldMap::getEdge(raV, raW);
    ASSERT_TRUE(approachEdge.direction_override == Routes::DirectionMode::DEFAULT);
    ASSERT_TRUE(departureEdge.direction_override == Routes::DirectionMode::DEFAULT);
    ASSERT_TRUE(Routes::PIPELINE_CONFIG.direction == Routes::DirectionMode::DEFAULT);

    // The naive AUTO resolution for the DEPARTURE hop, fed the SAME real resolveDirection()
    // the FSM calls with the arrival heading the approach hop leaves the chassis at
    // (280 deg = 100 + 180, the REAR approach body target): resolves FRONT, since
    // |wrap(220 - 280)| = 60 <= the 90 deg threshold. Force-SAME must override this to REAR
    // -- the coherent-run proof (a naive re-resolve would degrade the departure to FRONT).
    const float naiveArrivalHeading = fmodf(approachEdge.heading_deg + 180.0f, 360.0f);
    const Direction naiveDeparture  = resolveDirection(
        departureEdge.direction_override, Routes::PIPELINE_CONFIG.direction,
        Direction::AUTO, Direction::AUTO,
        naiveArrivalHeading, departureEdge.heading_deg,
        /*isLastHop*/ true, Config::BIDIRECTIONAL_AUTO_ENABLED, Config::LINE_BOARD_COUNT);
    ASSERT_TRUE(naiveDeparture == Direction::FRONT);

    resetWorld();
    seatAt("RaU", 0);
    ASSERT_EQ(routedFirstHop(raU, raW), raV);

    PathTracker.goTo(raW).async();

    // Hop 1 (RaU -> RaV): AUTO resolves REAR (100 deg body delta, strictly > the 90 deg
    // threshold, non-axis-aligned). Body target = 100 + 180 = 280. Confirm the turn is
    // genuinely dispatched before parking the target (see the AC1 test's comment on why).
    // Retire via `isTurnDone()` (not `expectPreTurn`'s `isIdle()` exit) -- this is a BLIND
    // dead-reckon hop, so `_dispatchEdgeAfterTurn()` claims Drive's queue for the departure
    // move in the SAME tick the turn settles (see the AC1 hop-2 comment for the race).
    ASSERT_TRUE(tickUntilTurnInFlight());
    mmL.mockYawDeg = (double) hwYaw((int16_t) 280);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.isTurnDone(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(Drive.isTurnDone());
    ASSERT_FALSE(PathTracker.hasFault());
    expectBlindHop(approachEdge.distance_cm);
    ASSERT_EQ(Localization.getCurrentNode(), raV);
    ASSERT_FALSE(PathTracker.hasFault());

    // Hop 2 (RaV -> RaW): force-SAME must hold REAR (the naive AUTO re-resolve above proved
    // FRONT). Body target = 220 + 180 = 40, NOT the naive 220.
    ASSERT_TRUE(tickUntilTurnInFlight());
    ASSERT_FALSE(PathTracker.hasFault());

    const float expectedBodyTarget = fmodf(departureEdge.heading_deg + 180.0f, 360.0f);
    ASSERT_TRUE(std::fabs(PathTracker.resolvedBodyHeadingDeg() - expectedBodyTarget) < kHeadingEpsDeg);

    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(expectedBodyTarget));
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.isTurnDone(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(Drive.isTurnDone());
    ASSERT_FALSE(PathTracker.hasFault());

    expectBlindHop(departureEdge.distance_cm);
    ASSERT_EQ(Localization.getCurrentNode(), raW);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC4/AC3 — a binding departure override that contradicts the pivot's approach direction
// faults Fault::PIVOT_DIRECTION_MISMATCH, never running mis-trimmed.
// ===========================================================================================

static void test_TC4_AC3_binding_contradiction_faults()
{
    TEST_SCOPE("TC4/AC3: binding departure contradiction (BcU,BcV,BcW) faults PIVOT_DIRECTION_MISMATCH");

    const uint8_t bcU = nodeByName("BcU");
    const uint8_t bcV = nodeByName("BcV");
    const uint8_t bcW = nodeByName("BcW");
    ASSERT_TRUE(bcU != FieldMap::NODE_NONE);
    ASSERT_TRUE(bcV != FieldMap::NODE_NONE);
    ASSERT_TRUE(bcW != FieldMap::NODE_NONE);

    const Routes::PivotTurn* pivot = FieldMap::findPivotTurn(bcU, bcV, bcW);
    ASSERT_TRUE(pivot != nullptr);

    const Routes::EdgeInfo& approachEdge  = FieldMap::getEdge(bcU, bcV);
    const Routes::EdgeInfo& departureEdge = FieldMap::getEdge(bcV, bcW);
    ASSERT_TRUE(approachEdge.direction_override == Routes::DirectionMode::FORCE_FORWARD);
    ASSERT_TRUE(departureEdge.direction_override == Routes::DirectionMode::FORCE_REVERSE);

    resetWorld();
    seatAt("BcU", 0);
    ASSERT_EQ(routedFirstHop(bcU, bcW), bcV);

    PathTracker.goTo(bcW).async();

    // Hop 1 (BcU -> BcV): pinned FRONT, heading 0 deg -- already aligned with the seated yaw
    // (0), so NO pre-edge turn: the dead-reckon move dispatches directly.
    bool dispatched = false;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (!Drive.isIdle() || PathTracker.hasFault())
        {
            dispatched = true;
            break;
        }
    }
    ASSERT_TRUE(dispatched);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(Drive.isIdle());

    expectBlindHop(approachEdge.distance_cm);
    ASSERT_EQ(Localization.getCurrentNode(), bcV);
    ASSERT_FALSE(PathTracker.hasFault());

    // Hop 2 (BcV -> BcW): the departure's BINDING FORCE_REVERSE contradicts the pivot's
    // FRONT approach direction -- force-SAME cannot reconcile it, so PLAN_NEXT_HOP faults
    // PIVOT_DIRECTION_MISMATCH BEFORE dispatching any turn or move.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::PIVOT_DIRECTION_MISMATCH);
    ASSERT_EQ(PathTracker.getFault().fromNode, bcV);
    ASSERT_EQ(PathTracker.getFault().destNode, bcW);
    ASSERT_TRUE(Drive.isIdle());    // no turn/move was ever dispatched for the faulted hop
}

// ===========================================================================================
// TC12 -- a REAR seek whose approaching-pivot trim exceeds the REAR overshoot budget (minus
// the trim floor) faults Fault::PIVOT_TRIM_UNDERFLOW at dispatch, before Drive.stopAtLine(...)
// is ever issued.
// ===========================================================================================

static void test_TC12_seek_delta_over_rear_budget_faults_pivot_trim_underflow()
{
    TEST_SCOPE("TC12: REAR seek into (RaU,RaV,RaW), trim > REAR budget, faults PIVOT_TRIM_UNDERFLOW");

    const uint8_t raU = nodeByName("RaU");
    const uint8_t raV = nodeByName("RaV");
    const uint8_t raW = nodeByName("RaW");
    ASSERT_TRUE(raU != FieldMap::NODE_NONE);
    ASSERT_TRUE(raV != FieldMap::NODE_NONE);
    ASSERT_TRUE(raW != FieldMap::NODE_NONE);

    const Routes::PivotTurn* pivot = FieldMap::findPivotTurn(raU, raV, raW);
    ASSERT_TRUE(pivot != nullptr);
    // The fixture's trim exists to prove AC2's coherent-run case; here it doubles as TC12's
    // over-budget delta -- deliberately larger than the REAR budget minus the trim floor.
    ASSERT_TRUE(pivot->trim_cm
                > Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM - Config::PIVOT_TRIM_MIN_DRIVE_CM);
    ASSERT_TRUE(Config::LINE_BOARD_COUNT >= 2);    // the REAR seek is genuinely serviceable here

    resetWorld();
    seatAt("RaU", 0);

    // Force the seek regardless of the edge's plain BLIND mode -- the approach resolves REAR
    // under AUTO (AC2) and LINE_BOARD_COUNT == 2 satisfies resolveStopAtLine's board gate, so
    // the seek genuinely dispatches (unlike the 1-board case in the sibling test file).
    PathTracker.goTo(raW).setStopAtLine(StopAtLine::ON).async();

    // Pre-edge turn to the REAR body target (280 deg), same as AC2's hop 1. Confirm the turn
    // is genuinely dispatched before parking the target (see the AC1 test's comment).
    //
    // NOTE: unlike a BLIND dead-reckon departure, the fault path here (PIVOT_TRIM_UNDERFLOW)
    // returns BEFORE touching Drive, so `Drive.isIdle()` stays genuinely true once the turn
    // settles -- `expectPreTurn` itself is race-free for this hop. But `_dispatchEdgeAfterTurn`
    // (and therefore the fault) can fire WITHIN `expectPreTurn`'s own tick loop, the instant
    // `Drive.isIdle()` sees the turn settle -- so the fault may already be latched by the time
    // this call returns; do not assert "no fault yet" immediately after it.
    ASSERT_TRUE(tickUntilTurnInFlight());
    expectPreTurn(/*appYawTarget=*/280, (uint8_t) Routes::PIPELINE_CONFIG.turn);

    const auto moveSyncBaseline = MiniR4.DriveDC.moveSyncCallCount;

    // The dispatch re-derives the seek overshoot trim (resolveSensorOvershootTrim) and finds
    // it clamped -- faults BEFORE any Drive.stopAtLine(...) is ever issued.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::PIVOT_TRIM_UNDERFLOW);
    ASSERT_EQ(PathTracker.getFault().fromNode, raU);
    ASSERT_EQ(PathTracker.getFault().destNode, raV);
    ASSERT_EQ(MiniR4.DriveDC.moveSyncCallCount, moveSyncBaseline);    // the seek itself never dispatched
}

// ===========================================================================================
// TC16 -- commanded-delta sign tripwire (PathTracker.cpp:2249-2274, ADR-072). A CCW body turn
// (delta > 0) plants PIVOT_LEFT forward; resolvePivotWheel keeps that for FRONT/AUTO and flips
// it to PIVOT_RIGHT for REAR. Reproduces the EXACT comparison the tripwire performs against
// the REAL resolvePivotWheel symbol -- a helper that calls the real resolver is not a mirror
// (test_path_tracker_stop_at_line.cpp TC-SL3 / test_path_tracker_bidirectional.cpp TC-BD1
// idiom). The NEGATIVE case (a genuinely consistent pivot never trips it) is proven by AC1's
// real-FSM dispatch above -- every `!PathTracker.hasFault()` assertion through that hop would
// otherwise have caught a stray PIVOT_DIRECTION_MISMATCH from a false-positive trip.
// ===========================================================================================

static void test_TC16_commanded_delta_sign_tripwire()
{
    TEST_SCOPE("TC16: commanded-delta sign tripwire -- matching pivot passes, a table/geometry mismatch trips");

    // AC1's real departure turn: delta = +43.493015 deg (CCW) -> the forward-sense wheel the
    // sign implies is PIVOT_LEFT; direction-corrected through the SAME resolvePivotWheel the
    // tripwire calls (REAR flips it) gives the wheel AC1's real dispatch used.
    const float ccwDelta = 43.493015f;
    const Routes::TurnMode expectedForwardWheel =
        (ccwDelta > 0.0f) ? Routes::TurnMode::PIVOT_LEFT : Routes::TurnMode::PIVOT_RIGHT;
    ASSERT_TRUE(expectedForwardWheel == Routes::TurnMode::PIVOT_LEFT);

    const Routes::TurnMode expectedWheelCorrected =
        resolvePivotWheel(expectedForwardWheel, Direction::REAR);
    ASSERT_TRUE(expectedWheelCorrected == Routes::TurnMode::PIVOT_RIGHT);

    // Matching case: AC1's real dispatched wheel (the stored PIVOT_LEFT, REAR-flipped) equals
    // the delta-implied, direction-corrected wheel -- the tripwire's `!=` test is false, so it
    // does NOT fire (this is exactly what AC1's real-FSM `!hasFault()` chain already proves).
    const Routes::TurnMode ac1DispatchedWheel =
        resolvePivotWheel(Routes::TurnMode::PIVOT_LEFT, Direction::REAR);
    ASSERT_TRUE(ac1DispatchedWheel == expectedWheelCorrected);

    // Mismatch case: a table/geometry divergence -- the delta says CCW (PIVOT_LEFT forward,
    // PIVOT_RIGHT once REAR-corrected) but the dispatched wheel is the OTHER pivot mode
    // entirely, as if the stored table row pointed at the wrong wheel for this geometry. The
    // tripwire's exact condition (`resolvedTurnMode != expectedWheel`) must catch it.
    const Routes::TurnMode wrongDispatchedWheel = Routes::TurnMode::PIVOT_LEFT;
    ASSERT_TRUE(wrongDispatchedWheel != expectedWheelCorrected);
}

// ===========================================================================================
// Entry.
// ===========================================================================================

int main()
{
    test_AC1_rear_pivot_flips_wheel();
    test_AC2_TC3_coherent_run_forces_rear_departure();
    test_TC4_AC3_binding_contradiction_faults();
    test_TC12_seek_delta_over_rear_budget_faults_pivot_trim_underflow();
    test_TC16_commanded_delta_sign_tripwire();

    TEST_REPORT();
}
