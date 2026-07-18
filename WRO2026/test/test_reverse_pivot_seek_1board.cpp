/**
 * @file test_reverse_pivot_seek_1board.cpp
 * @brief Host-link test for the reverse-pivot REAR-seek serviceability guard on a 1-board
 *        chassis, driving the REAL `PathTracker` + `Drive` + `Localization` FSMs through the
 *        shared cooperative-tick harness (issue #54, ADR-072, T10).
 *
 * @details
 * Built with `-DLINE_BOARD_COUNT_DEFAULT=1` (a compile-time override hook in `ConfigLine.h`,
 * mirroring the existing `LINE_JUNCTION_T_MIN_ON_DEFAULT` idiom in the SAME file — zero
 * default-behavior change for the real sketch: the real Arduino build and the Python pipeline,
 * which hardcodes its OWN `LINE_BOARD_COUNT` in `sim/runtime.py`, are both untouched). Same
 * real-link recipe as the sibling `test_reverse_pivot_dispatch.cpp` (2-board target) — the T9
 * shadow `fixtures/routes_reverse_pivots.cpp` force-included FIRST.
 *
 * ===========================================================================================
 * AC8/TC11 covers TWO claims; this file proves BOTH against the REAL FSM.
 * ===========================================================================================
 *
 * 1. **"A non-pivot forced-REAR STOP_AT_LINE BLIND hop still degrades to dead-reckon reverse
 *    (unchanged)"** — proven by `test_AC8_TC11_non_pivot_forced_rear_degrades_...`: `goTo(CrV)`
 *    from `CrU` (the fixture's `CrU -> CrV` edge, pinned REAR) with `.setStopAtLine(ON)`. `CrV`
 *    IS the goal, so no pivot latches onto it (`_approachingPivot.active == false`) — the
 *    genuinely non-pivot half of AC8. The hop degrades to a plain dead-reckon reverse move with
 *    NO fault, exactly as before this feature shipped.
 *
 * 2. **"A REAR stop-at-line seek INTO a pivot on a 1-board chassis faults
 *    `Fault::REVERSE_UNAVAILABLE` (pivot-scoped)"** — proven by the two
 *    `test_AC8_TC11_..._faults_reverse_unavailable` tests below, one per chained pivot.
 *
 *    T10 originally documented this as a KNOWN GAP: the pivot-scoped guard lived INSIDE
 *    `case DispatchPrimitive::STOP_AT_LINE_SEEK`, but `resolveStopAtLine`'s own board gate
 *    (`StopAtLinePolicy.h`: `boardOK = (direction == FRONT) || (lineBoardCount >= 2)`) forces
 *    `seek = false` for EVERY REAR request on a `<2`-board chassis — REGARDLESS of the call-site
 *    `StopAtLine::ON` override — so `resolveEdgeDispatch` classified the hop `BLIND_DEAD_RECKON`,
 *    NEVER `STOP_AT_LINE_SEEK`, and the nested guard could never run. ADR-072's Fix 1 HOISTS the
 *    check into `_onPlanNextHop` (pre-turn, keyed off the RESOLVED direction and the "seek
 *    requested" rungs, asked of the SAME resolver with the board gate forced open), so it fires
 *    BEFORE `resolveStopAtLine`'s silent degrade. These tests now drive the REAL FSM over the
 *    fixture's chained `(CrU, CrV, CrW, CrX)` pivot approaches on the 1-board target and assert
 *    `PathTracker` hard-faults `Fault::REVERSE_UNAVAILABLE` — no longer a documented gap.
 *
 * ===========================================================================================
 * Harness gotchas (T10) preserved here
 * ===========================================================================================
 *   - `seatAt(...)` binds the world frame but does NOT bind heading toward the first edge, so a
 *     bare `goTo(...)` still resolves its own pre-turn. The two fault tests never reach the
 *     pre-turn (the guard faults at PLAN time), so heading is irrelevant to them; the non-pivot
 *     degrade test still drives its pre-turn to the REAR body target (90 + 180 = 270 deg).
 *   - `tickUntilTurnInFlight()` confirms a REAL pre-edge turn is genuinely claimed by `Drive`
 *     before parking the target yaw (the degrade test), and in the fault tests it returns
 *     `false` the moment the pivot-scoped guard latches — proving NO turn ever dispatches (the
 *     fault precedes the pre-turn).
 *   - Turns are retired via `isTurnDone()` (NOT `expectPreTurn`'s `isIdle()` exit): with
 *     `HOP_SETTLE_MS == 0` the degraded BLIND dead-reckon departure dispatches in the SAME tick
 *     the turn settles, so `Drive.isIdle()` can already read `false` again by the time a
 *     turn-retire loop re-checks it.
 */

#include "test_path_tracker_harness.h"

#include "src/nav/EdgeDispatchPolicy.h"    // resolveEdgeDispatch + resolveStopAtLine (real frozen resolvers).
#include "src/nav/NavTypes.h"              // Direction / StopAtLine enums.
#include "src/mission/PathTracker.h"       // PathTrackerClass::Fault + FaultInfo.
#include "src/nav/routes.h"                // Routes::EdgeType / DirectionMode / NodeType.

#include <cstdio>

namespace
{
/**
 * @brief Tick the real FSM until a pre-edge turn is GENUINELY running (`Drive.isTurnDone() ==
 *        false`), WITHOUT parking the target yaw first.
 *
 * @note `expectPreTurn` parks `mmL.mockYawDeg` at the target BEFORE its own tick loop starts.
 *       Calling it immediately after `.async()` (with no intervening tick) would let
 *       `_onPlanNextHop`'s FIRST read of `Localization.getHeadingDeg()` already see the
 *       parked target and skip the turn as "already aligned" — even when a REAL turn is
 *       expected. This helper confirms the turn has actually been claimed by `Drive` first.
 * @return `true` once the turn is genuinely in flight; `false` on a fault or a
 *         budget-exhausted, un-dispatched turn. In the pivot-seek fault tests below the guard
 *         faults at PLAN time (ahead of the pre-turn), so this returns `false` — the callers
 *         then assert the latched `REVERSE_UNAVAILABLE` fault to disambiguate.
 */
bool tickUntilTurnInFlight()
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (!Drive.isTurnDone())
        {
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
// AC8/TC11 (non-pivot half) — a non-pivot forced-REAR STOP_AT_LINE BLIND hop on a 1-board
// chassis degrades to a dead-reckon reverse move, unchanged by this feature.
// ===========================================================================================

static void test_AC8_TC11_non_pivot_forced_rear_degrades_to_dead_reckon_reverse()
{
    TEST_SCOPE("AC8/TC11: non-pivot forced-REAR STOP_AT_LINE BLIND hop on 1 board degrades to dead-reckon reverse (unchanged)");

    ASSERT_EQ(Config::LINE_BOARD_COUNT, (uint8_t) 1);

    const uint8_t crU = nodeByName("CrU");
    const uint8_t crV = nodeByName("CrV");
    ASSERT_TRUE(crU != FieldMap::NODE_NONE);
    ASSERT_TRUE(crV != FieldMap::NODE_NONE);

    const Routes::EdgeInfo& edge = FieldMap::getEdge(crU, crV);
    ASSERT_TRUE(edge.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(edge.direction_override == Routes::DirectionMode::FORCE_REVERSE);

    resetWorld();
    seatAt("CrU", 0);

    // goTo(CrV) directly: CrV IS the goal, so no pivot latches onto it -- the topological
    // pivot (CrU, CrV, CrW) only "approaches" when CrW is further down the route (see the
    // fault tests below, which drive goTo(CrW) instead). This is deliberately the NON-pivot
    // half of AC8/TC11.
    ASSERT_EQ(routedFirstHop(crU, crV), crV);
    PathTracker.goTo(crV).setStopAtLine(StopAtLine::ON).async();

    // Pre-edge turn to the REAR body target (90 + 180 = 270 deg). No departing pivot is
    // active at CrU (a fresh start), so the wheel is the pipeline default. Confirm the turn
    // is genuinely dispatched before parking the target (see tickUntilTurnInFlight's note).
    //
    // Retire via `isTurnDone()`, NOT `expectPreTurn`'s `isIdle()` exit: with `HOP_SETTLE_MS ==
    // 0` the degraded BLIND dead-reckon departure dispatches in the SAME tick the turn
    // settles, so `Drive.isIdle()` can already read `false` again (claimed by the NEW move)
    // by the time a turn-retire loop re-checks it.
    ASSERT_TRUE(tickUntilTurnInFlight());
    mmL.mockYawDeg = (double) hwYaw((int16_t) 270);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.isTurnDone(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(Drive.isTurnDone());
    ASSERT_FALSE(PathTracker.hasFault());

    // resolveStopAtLine's board gate degrades the requested seek (StopAtLine::ON) to a plain
    // dead-reckon (boardOK == false: REAR + 1 board) -- the hop must NOT fault; it dead-reckons
    // in reverse exactly as it did before this feature shipped (AC8's "unchanged" clause).
    expectBlindHop(edge.distance_cm);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_EQ(Localization.getCurrentNode(), crV);
}

// ===========================================================================================
// AC8/TC11 (pivot half) — a REAR stop-at-line seek INTO the first chained pivot (CrU, CrV, CrW)
// on a 1-board chassis hard-faults REVERSE_UNAVAILABLE (pivot-scoped, ADR-072 Fix 1).
//
// Also proves — at the resolver level — WHY the guard had to be HOISTED out of the
// STOP_AT_LINE_SEEK dispatch case: `resolveStopAtLine` STILL degrades a 1-board REAR seek to
// `seek == false` (so a dispatch-case guard keyed on the resolved primitive would be dead),
// while the "seek requested" probe (board gate forced open) is true -- the exact signal
// `_onPlanNextHop` now keys off before the degrade can hide it.
// ===========================================================================================

static void test_AC8_TC11_pivot_rear_seek_1board_faults_reverse_unavailable()
{
    TEST_SCOPE("AC8/TC11: REAR seek into approaching pivot (CrU,CrV,CrW) on 1 board faults REVERSE_UNAVAILABLE (ADR-072)");

    ASSERT_EQ(Config::LINE_BOARD_COUNT, (uint8_t) 1);

    const uint8_t crU = nodeByName("CrU");
    const uint8_t crV = nodeByName("CrV");
    const uint8_t crW = nodeByName("CrW");
    ASSERT_TRUE(crU != FieldMap::NODE_NONE);
    ASSERT_TRUE(crV != FieldMap::NODE_NONE);
    ASSERT_TRUE(crW != FieldMap::NODE_NONE);

    // The (CrU, CrV, CrW) pivot genuinely latches on this route, and CrU -> CrV (its APPROACH
    // leg) is a BLIND edge pinned REAR.
    ASSERT_TRUE(FieldMap::findPivotTurn(crU, crV, crW) != nullptr);
    const Routes::EdgeInfo& edge = FieldMap::getEdge(crU, crV);
    ASSERT_TRUE(edge.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(edge.direction_override == Routes::DirectionMode::FORCE_REVERSE);

    // ---- Resolver mechanism: WHY the guard is hoisted upstream. --------------------------
    // resolveStopAtLine still degrades a 1-board REAR seek to `seek == false` -- so a guard
    // keyed on the STOP_AT_LINE_SEEK primitive inside the dispatch case is dead (the primitive
    // never classifies as a seek). The hoisted guard instead asks the SAME resolver with the
    // board gate forced OPEN (lineBoardCount = 2 => boardOK always true => seek == requested).
    const StopAtLineQuery seekRealBoards{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ Config::LINE_BOARD_COUNT,    // == 1 here
        /*autoRuleActive*/ false};
    ASSERT_FALSE(resolveStopAtLine(seekRealBoards).seek);    // boardOK degrades EVERY REAR seek on <2 boards

    StopAtLineQuery seekBoardOpen = seekRealBoards;
    seekBoardOpen.lineBoardCount = (uint8_t) 2;               // force boardOK open
    ASSERT_TRUE(resolveStopAtLine(seekBoardOpen).seek);      // the "requested" signal the hoisted guard reads

    // And the composed dispatch classifies the degraded hop BLIND_DEAD_RECKON + reverse -- the
    // silent degrade the hoisted guard now pre-empts.
    EdgeDispatchQuery q{};
    q.edgeType          = Routes::EdgeType::BLIND;
    q.destNodeType      = Routes::NodeType::BLIND;
    q.direction         = Direction::REAR;
    q.rawDistanceCm     = 60.0f;
    q.speed             = Routes::SpeedTier::NORMAL;
    q.stopAtLine        = resolveStopAtLine(seekRealBoards);
    q.projPassThrough   = false;
    q.departingTrimCm   = 0.0f;
    q.approachingTrimCm = 9.0f;    // an ACTIVE approaching pivot's trim
    q.axleOffsetCm      = Config::LINE_SENSOR_AXLE_OFFSET_REAR_CM;
    q.minDriveCm        = Config::PIVOT_TRIM_MIN_DRIVE_CM;
    const EdgeDispatchDecision decision = resolveEdgeDispatch(q);
    ASSERT_TRUE(decision.primitive == DispatchPrimitive::BLIND_DEAD_RECKON);
    ASSERT_TRUE(decision.reverse);

    // ---- Real FSM: the hoisted guard hard-faults at PLAN time. ---------------------------
    resetWorld();
    seatAt("CrU", 0);
    ASSERT_EQ(routedFirstHop(crU, crW), crV);    // CrU -> CrV is the first hop toward CrW

    // goTo(CrW) makes the (CrU, CrV, CrW) pivot's approach ACTIVE on the CrU -> CrV hop, and
    // .setStopAtLine(ON) requests a stop-at-line seek on that BLIND approach leg. On a 1-board
    // chassis the pivot-scoped serviceability guard (ADR-072) hard-faults REVERSE_UNAVAILABLE
    // BEFORE the pre-edge turn is even dispatched -- rather than letting resolveStopAtLine
    // silently degrade the seek to a dead-reckon reverse and fire the pivot off-target.
    PathTracker.goTo(crW).setStopAtLine(StopAtLine::ON).async();

    // The guard fires during the plan tick (ahead of the pre-edge turn), so tickUntilTurnInFlight
    // never sees a turn in flight: it ticks the cooperative loop and returns false the moment the
    // fault latches (preserving the harness gotcha -- there is no turn to retire on this path).
    ASSERT_FALSE(tickUntilTurnInFlight());
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::REVERSE_UNAVAILABLE);

    // The fault fired at plan time, so the chassis never advanced off the start node -- no
    // silent dead-reckon-reverse degrade.
    ASSERT_EQ(Localization.getCurrentNode(), crU);
}

// ===========================================================================================
// AC8/TC11 (pivot half, second pivot) — the SAME serviceability fault on the SECOND chained
// pivot (CrV, CrW, CrX): a REAR seek into it on a 1-board chassis also hard-faults
// REVERSE_UNAVAILABLE, exercising a distinct approach edge (CrV -> CrW) so the guard is proven
// on more than one topological pivot, not a single edge.
// ===========================================================================================

static void test_AC8_TC11_pivot2_rear_seek_1board_faults_reverse_unavailable()
{
    TEST_SCOPE("AC8/TC11: REAR seek into approaching pivot (CrV,CrW,CrX) on 1 board faults REVERSE_UNAVAILABLE (ADR-072)");

    ASSERT_EQ(Config::LINE_BOARD_COUNT, (uint8_t) 1);

    const uint8_t crV = nodeByName("CrV");
    const uint8_t crW = nodeByName("CrW");
    const uint8_t crX = nodeByName("CrX");
    ASSERT_TRUE(crV != FieldMap::NODE_NONE);
    ASSERT_TRUE(crW != FieldMap::NODE_NONE);
    ASSERT_TRUE(crX != FieldMap::NODE_NONE);

    // The (CrV, CrW, CrX) pivot latches on this route, and CrV -> CrW (its APPROACH leg) is a
    // BLIND edge pinned REAR.
    ASSERT_TRUE(FieldMap::findPivotTurn(crV, crW, crX) != nullptr);
    const Routes::EdgeInfo& edge = FieldMap::getEdge(crV, crW);
    ASSERT_TRUE(edge.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(edge.direction_override == Routes::DirectionMode::FORCE_REVERSE);

    resetWorld();
    seatAt("CrV", 0);
    ASSERT_EQ(routedFirstHop(crV, crX), crW);    // CrV -> CrW is the first hop toward CrX

    // goTo(CrX) makes the (CrV, CrW, CrX) pivot's approach ACTIVE on the CrV -> CrW hop;
    // .setStopAtLine(ON) requests the seek. Same pivot-scoped fault as above, one pivot over.
    PathTracker.goTo(crX).setStopAtLine(StopAtLine::ON).async();

    ASSERT_FALSE(tickUntilTurnInFlight());
    ASSERT_TRUE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::REVERSE_UNAVAILABLE);
    ASSERT_EQ(Localization.getCurrentNode(), crV);
}

// ===========================================================================================
// Entry.
// ===========================================================================================

int main()
{
    test_AC8_TC11_non_pivot_forced_rear_degrades_to_dead_reckon_reverse();
    test_AC8_TC11_pivot_rear_seek_1board_faults_reverse_unavailable();
    test_AC8_TC11_pivot2_rear_seek_1board_faults_reverse_unavailable();

    TEST_REPORT();
}
