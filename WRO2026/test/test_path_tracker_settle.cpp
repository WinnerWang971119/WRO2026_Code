/**
 * @file test_path_tracker_settle.cpp
 * @brief Host-link test for the ADR-061 post-stop settle dwell (`State::SETTLE`),
 *        driving the REAL `PathTracker` + `Drive` + `Localization` + `YawSource` FSMs
 *        through the shared cooperative-tick harness (2026-07-12 hop-settle-wait plan, T4;
 *        TC1-TC7b; AC2-AC5, AC8 firmware half).
 *
 * @details
 * This is the FIRMWARE half of the ADR-061 eligibility tether (AC8). It asserts that the
 * real PathTracker FSM enters `State::SETTLE` EXACTLY on the boundaries the plan's
 * Settle-fire matrix says "yes", never on the "no" boundaries, and that in every SKIP case
 * the surrounding dispatch (LineFollower re-arm counts, Drive move dispatch, node advance,
 * brake flag) is BYTE-IDENTICAL to what the pre-feature sibling tests already assert for the
 * same transition. Compiled at the DEFAULT `HOP_SETTLE_MS = 100` (this target adds NO
 * `-DHOP_SETTLE_MS_DEFAULT` override — the legacy dispatch-assertion targets are pinned to 0,
 * this one runs at the shipped default so the settle actually fires).
 *
 * =====================================================================================
 * THE SETTLE-FIRE MATRIX (quoted from docs/plans/2026-07-12-hop-settle-wait.md, section
 * "Settle-fire matrix" — the single shared contract this ctest and the CTPS pytest both
 * assert against). Every "yes" additionally requires `Config::HOP_SETTLE_MS > 0`.
 * =====================================================================================
 *   | Arrival / boundary                                    | Braked? | Settle? |
 *   |-------------------------------------------------------|---------|---------|
 *   | LINE_OVERSHOOT, normal (`!_passThrough`)              | yes     | YES     |
 *   | LINE_OVERSHOOT, pass-through (`_passThrough`)         | no      | no      |
 *   | PROJECTION_FOLLOW, stop-at-B (`!_projPassThrough`)    | yes     | YES     |
 *   | PROJECTION_FOLLOW, pass-through (`_projPassThrough`)  | no      | no      |
 *   | STOP_AT_LINE (always braked)                          | yes     | YES     |
 *   | BLIND_FORWARD, non-chain (`!_chain.active`)           | yes     | YES     |
 *   | BLIND_FORWARD, chain intermediate (`_chain.active`)  | no      | no      |
 *   | Cold pre-edge turn done, normal hop (`!_returnActive`)| n/a     | YES (turn->move) |
 *   | Return-prologue pre-edge turn (`_returnActive`)       | n/a     | no      |
 *   | Goal arrival (final braked hop, `cur==dest`)          | yes     | YES     |
 *   | First-plan PRE-maneuver dwell (dwell-1, standing start)| n/a    | no (structurally absent) |
 *   | First-hop turn->move dwell (dwell-2, first cold pivot)| n/a     | YES (like any cold turn) |
 *   | First cold turn AFTER WALL_DEPART / RETURN_TO_NODE    | n/a     | YES (interlude is over) |
 *   | Arrival at a RESET node (`getNodeType==RESET`)        | yes     | no (wall interlude owns it) |
 *   | Any, `HOP_SETTLE_MS == 0`                             | —       | no      |
 *
 * TWO DWELLS, ONE KNOB. *dwell-1* = post-stop, before the next maneuver (fired inside the
 * braked arrival state before `PLAN_NEXT_HOP`; also covers goal arrival). *dwell-2* =
 * post-cold-turn, before the move (fired at `TURNING` idle). The "first plan -> no settle"
 * rule scopes to *dwell-1 only* (the FSM cannot reach dwell-1 on a standing start — no braked
 * arrival state precedes the first plan). *dwell-2* fires normally on the first hop's cold
 * pivot and on the first normal hop after the wall/return interludes.
 *
 * THE FIXTURE (`fixtures/routes_fixture.h`, 34-node feature-complete stress map; nodes
 * resolved BY NAME, never by index — issue #91):
 *   - `Start`(BLIND,24,24) -> `Inter1`(LINE,60,40): forced STOP_AT_LINE seek (`stop_at_line=on`).
 *   - `Inter2`(LINE,140,40) -> `Inter3`(LINE,200,40) -> `Inter4`(LINE,260,40): collinear
 *     LINE-LINE-LINE pass-through triple (heading 0).
 *   - `Inter3` -> `Inter4` -> `Corner`(LINE,260,100): the `(Inter3,Inter4,Corner)` pivot;
 *     `Inter4 -> Corner` is a genuine 90 deg cold turn (TC4 dwell-1 + dwell-2 on one hop).
 *   - `final`(BLIND,95,70) -> `ChainA`/`ChainB`/`ChainC` -> `ChainD`(168,70): ADR-059 blind
 *     chain, all collinear heading 0. `ChainA..C` are non-final intermediates (chain active,
 *     no settle); `ChainD` is the final braked chained hop (chain retired at its dispatch,
 *     so `_chain.active` reads false at ITS arrival -> settles).
 *   - `Inter1` -> `ProjL`(PROJECTION,100,40) -> `Inter2`: line-projection-line pass-through.
 *   - `position1`(BLIND,200,110) -> `Wall1`(RESET,165,139): BLIND dead-reckon into a RESET
 *     node; the wall interlude owns that boundary (TC6 / TC5b).
 *
 * The per-hop done-bit / sensor / turn handshakes are the shared harness helpers
 * (`expectLineHop` / `expectBlindHop` / `expectStopAtLine` / `expectPreTurn` /
 * `expectWallStop`); the settle dwell is retired by `driveSettle()` (jump the mock clock past
 * `_settleDeadlineMs`, then tick out of `SETTLE`). Time advances ONLY via `mockAdvanceMillis`
 * (inside those helpers / `driveSettle`), never `delay()`.
 */

#include "test_path_tracker_harness.h"

#include <cmath>
#include <cstdio>

namespace
{
/// @brief Convenience alias for the now-public per-edge FSM state enum (issue #108, AC12).
using State = PathTrackerClass::State;

// ===========================================================================================
// Shared drivers.
// ===========================================================================================

/// @brief Tick the real FSM (up to `kMaxHandshakeTicks`) until it reaches @p target, parking
///        the mock IMU yaw at @p hwHeading each tick so any in-flight pre-turn settles, and
///        asserting `State::SETTLE` is NEVER observed on the way (the "no settle here" proof).
///
/// @param target    The FSM state to stop on.
/// @param hwHeading The hardware-frame yaw to park each tick (so a cold pre-turn settles). Pass
///                  the seated facing when no turn is expected.
/// @note Asserts the target was reached AND `SETTLE` was never entered across the walk.
void tickToStateNoSettle(State target, int16_t hwHeading)
{
    bool sawSettle = false;
    bool reached   = (PathTracker.state() == target);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !reached && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
        if (PathTracker.state() == State::SETTLE)
        {
            sawSettle = true;
        }
        if (PathTracker.state() == target)
        {
            reached = true;
        }
    }
    ASSERT_TRUE(reached);
    ASSERT_FALSE(sawSettle);
}

/// @brief Drive an in-flight BLIND dead-reckon hop to completion WITHOUT ticking past the
///        arrival transition, then report whether the FSM landed in `SETTLE`.
///
/// @details
/// `expectBlindHop()` ticks until `Drive.isIdle()` and does not stop to inspect the arrival
/// transition, so it cannot distinguish a settle from a plan. This helper instead advances the
/// encoder + done-bit each tick until `Drive.isIdle()` (the move retired), then runs ONE more
/// tick — the tick on which `BLIND_FORWARD` observes `Drive.isIdle()`, stamps the node advance,
/// and takes its settle-or-plan branch — and returns the resulting state. The caller inspects it.
///
/// @param hwHeading The hardware-frame yaw to park each tick (so a GYRO hold / pre-turn settles).
/// @return The FSM state immediately AFTER the arrival-transition tick (`SETTLE` if dwell-1 fired,
///         `PLAN_NEXT_HOP` if it was skipped — a chain intermediate).
State retireBlindHopAndPeek(int16_t hwHeading)
{
    // Retire the move while the FSM is still in BLIND_FORWARD: over-advance the encoder + hold the
    // done-bit each tick. The loop exits on the very tick BLIND_FORWARD observes `Drive.isIdle()`,
    // stamps the node advance, and transitions out (to SETTLE or PLAN_NEXT_HOP) — so the returned
    // state IS the arrival-branch outcome, without ticking further past it.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() == State::BLIND_FORWARD;
         ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
    return PathTracker.state();
}

/// @brief Tick the FSM until a BLIND dead-reckon hop is dispatched (`State::BLIND_FORWARD`),
///        settling any cold pre-turn first by parking @p hwHeading each tick.
void tickUntilBlindForward(int16_t hwHeading)
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() != State::BLIND_FORWARD && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
    }
    ASSERT_EQ(PathTracker.state() == State::BLIND_FORWARD, true);
}

/// @brief Tick the FSM until the STOP_AT_LINE seek is genuinely IN FLIGHT (its first `MoveSync`
///        emission), parking @p hwHeading each tick so any residual turn/hold settles.
///
/// @details
/// A seek dispatched by `_dispatchEdgeAfterTurn` is ENQUEUED on Drive; it does not capture its
/// `_stopStartEnc` baseline until `Drive.update()` pops it and emits the first `MoveSync`. The
/// `expectStopAtLine` retire helper positions `MiniR4.M2` RELATIVE to that captured baseline, so
/// the seek must be in flight (baseline captured, `M2` still at it) BEFORE it is called -- exactly
/// the `driveToSeekInFlight` precondition the sibling `test_path_tracker_stop_at_line.cpp` relies
/// on. Polling `moveSyncCallCount` (not `state()`) is the clean "seek running" signal: the custom
/// TurnGyro pre-turn drives `Move`, NOT `MoveSync`, so a non-zero count means the seek (not a
/// lingering turn) is emitting.
void tickUntilSeekInFlight(int16_t hwHeading)
{
    const uint32_t before = MiniR4.DriveDC.moveSyncCallCount;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && MiniR4.DriveDC.moveSyncCallCount == before && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
    }
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, before);
}
}    // namespace

// ===========================================================================================
// TC1 — a braked STOPPED hop enters SETTLE, holds until the clock passes the deadline, then
//       dispatches the next maneuver. Uses the forced `Start -> Inter1` STOP_AT_LINE seek
//       (always braked). Matrix row: `STOP_AT_LINE` -> YES.
// ===========================================================================================
void test_TC1_braked_stop_enters_settle()
{
    TEST_SCOPE("TC1: braked STOP_AT_LINE hop (Start->Inter1) enters SETTLE, then proceeds");

    const uint8_t start  = nodeByName("Start");
    const uint8_t inter1 = nodeByName("Inter1");
    ASSERT_TRUE(start != FieldMap::NODE_NONE && inter1 != FieldMap::NODE_NONE);
    // Pin the forced-seek hop: Start->Inter1 is a BLIND edge with a `stop_at_line=on` override.
    ASSERT_EQ(routedFirstHop(start, inter1), inter1);
    ASSERT_TRUE(FieldMap::getEdge(start, inter1).type == Routes::EdgeType::BLIND);

    resetWorld();
    seatAt("Start", 0);    // Start->Inter1 departs heading ~24 deg; seated at 0 -> a cold pre-turn.
    PathTracker.goTo(inter1).async();

    const Routes::EdgeInfo& edge = FieldMap::getEdge(start, inter1);

    // The first hop dispatches a cold pre-turn (seated 0 vs edge ~24 deg). Tick to TURNING while
    // parking heading 0 (so the turn is in flight but not yet settled), asserting NO dwell-1 SETTLE
    // on the standing start (first-plan dwell-1 is structurally absent — no braked arrival precedes
    // the first plan). Then retire the turn: the first hop's cold turn->move dwell (dwell-2) DOES
    // fire (matrix: the first cold turn settles like any other).
    tickToStateNoSettle(State::TURNING, hwYaw(0));
    expectPreTurn((int16_t) lroundf(edge.heading_deg), (uint8_t) Routes::PIPELINE_CONFIG.turn);
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);    // dwell-2 on the first hop's cold turn
    driveSettle();
    ASSERT_FALSE(PathTracker.hasFault());

    // dwell-2's resume dispatched the seek; tick until it is genuinely in flight (its first
    // MoveSync) so `expectStopAtLine` positions the encoder relative to the captured baseline.
    tickUntilSeekInFlight(hwYaw((int16_t) lroundf(edge.heading_deg)));

    // Now the seek is in flight; drive it to a successful crossing.
    expectStopAtLine(edge.distance_cm, /*found*/ true);

    // THE TC1 CLAIM: after the braked STOP_AT_LINE arrival the FSM is in SETTLE (dwell-1), NOT
    // already in PLAN_NEXT_HOP — observed BEFORE retiring the dwell.
    ASSERT_EQ(Localization.getCurrentNode(), inter1);    // onIntersection stamped FIRST
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the dwell: the clock jumps past the deadline and the FSM leaves SETTLE. Inter1 is the
    // goal, so the resume plan sees cur==dest and settles to IDLE. Either way it LEFT SETTLE.
    driveSettle();
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);    // proceeded past the dwell
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC2 — a LINE pass-through hop NEVER enters SETTLE, and the dispatched arm cascade is
//       byte-identical to the no-settle path (the same observables `test_path_tracker_
//       passthrough.cpp` asserts). Matrix row: `LINE_OVERSHOOT, pass-through` -> no.
// ===========================================================================================
void test_TC2_line_pass_through_skips_settle()
{
    TEST_SCOPE("TC2: LINE-LINE-LINE pass-through (Inter2->Inter3->Inter4) never enters SETTLE");

    const uint8_t inter2 = nodeByName("Inter2");
    const uint8_t inter3 = nodeByName("Inter3");
    const uint8_t inter4 = nodeByName("Inter4");
    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE && inter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter4 != FieldMap::NODE_NONE);
    // Routing pin: goTo(Inter4) from Inter2 MUST traverse Inter3 (no bypass).
    ASSERT_EQ(routedFirstHop(inter2, inter4), inter3);
    assertNoBypass(inter2, inter4);
    ASSERT_TRUE(FieldMap::getEdge(inter2, inter3).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(inter3, inter4).type == Routes::EdgeType::LINE);

    // Drive to LINE_FOLLOW for the first hop Inter2->Inter3 (no pre-turn — heading 0 == seated 0).
    driveToLineFollowEntry("Inter2", "Inter4");
    ASSERT_EQ(Localization.getCurrentNode(), inter2);
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_FALSE(LineFollower.lastOvershootMode);

    // Cross the mid junction Inter3. `expectLineHop()` latches the junction + trips the overshoot
    // gate; the FSM runs the pass-through peek (Inter4 collinear + non-goal) and advances to Inter3
    // MID-FOLLOW without braking. The gate-trip tick is the one where a settle would have fired if
    // this were a braked arrival — it must NOT, because `_passThrough` sends it straight to
    // PLAN_NEXT_HOP.
    const int armsBefore = LineFollower.setEnabledCount;
    expectLineHop();

    // BYTE-IDENTICAL DISPATCH (the same observables test_path_tracker_passthrough.cpp asserts):
    // the overshoot re-arm happened, carried pass-through semantics (no brake-on-gate, no
    // ramp-down), and advanced the node — the SKIP produces the same dispatch as the no-settle
    // path, not just "no SETTLE".
    ASSERT_GT(LineFollower.setEnabledCount, armsBefore);
    ASSERT_TRUE(LineFollower.lastOvershootMode);
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);       // pass-through fired: no brake
    ASSERT_FALSE(LineFollower.lastTShapeRampDown);     // ... and no ramp-down
    ASSERT_EQ(Localization.getCurrentNode(), inter3);  // advanced mid-follow onto Inter3

    // THE TC2 CLAIM: the pass-through transition did NOT enter SETTLE. `_passThrough` routed the
    // LINE_OVERSHOOT arrival straight to PLAN_NEXT_HOP (or onward LINE_FOLLOW), never SETTLE.
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC3 — blind chain: the intermediate hops (`_chain.active`) NEVER settle; the FINAL chained
//       braked hop (`ChainD`, chain retired at its own dispatch) DOES settle. Matrix rows:
//       `BLIND_FORWARD, chain intermediate` -> no; `BLIND_FORWARD, non-chain` (the final
//       chained hop reads chain inactive) -> YES.
// ===========================================================================================
void test_TC3_blind_chain_intermediates_skip_final_settles()
{
    TEST_SCOPE("TC3: blind-chain intermediates skip SETTLE; final chained hop (ChainD) settles");

    const uint8_t nFinal = nodeByName("final");
    const uint8_t nA     = nodeByName("ChainA");
    const uint8_t nB     = nodeByName("ChainB");
    const uint8_t nC     = nodeByName("ChainC");
    const uint8_t nD     = nodeByName("ChainD");
    ASSERT_TRUE(nFinal != FieldMap::NODE_NONE && nA != FieldMap::NODE_NONE);
    ASSERT_TRUE(nB != FieldMap::NODE_NONE && nC != FieldMap::NODE_NONE && nD != FieldMap::NODE_NONE);
    // Pin the chain route so a re-weight fails loudly.
    ASSERT_EQ(routedFirstHop(nFinal, nD), nA);
    ASSERT_EQ(routedFirstHop(nA, nD), nB);
    ASSERT_EQ(routedFirstHop(nB, nD), nC);
    ASSERT_EQ(routedFirstHop(nC, nD), nD);
    ASSERT_TRUE(Config::PATH_BLIND_CHAIN_ENABLED);    // this target ships chaining ON (default)

    resetWorld();
    seatAt("final", 0);    // final->ChainA departs heading 0 == the seated facing (no pre-turn).
    PathTracker.goTo(nD).async();

    // ChainA/B/C are collinear intermediates at heading 0. Retire each hop and peek the arrival
    // state: an active chain routes straight to PLAN_NEXT_HOP (no settle), so the peek must NOT be
    // SETTLE and the node must have advanced.
    const int16_t hw0 = hwYaw(0);

    // --- Intermediate hop 0: final -> ChainA (chain active -> no settle). ---
    tickUntilBlindForward(hw0);
    State s0 = retireBlindHopAndPeek(hw0);
    ASSERT_EQ(Localization.getCurrentNode(), nA);
    ASSERT_FALSE(s0 == State::SETTLE);    // chain intermediate: no dwell-1

    // --- Intermediate hop 1: ChainA -> ChainB (chain active -> no settle). ---
    tickUntilBlindForward(hw0);
    State s1 = retireBlindHopAndPeek(hw0);
    ASSERT_EQ(Localization.getCurrentNode(), nB);
    ASSERT_FALSE(s1 == State::SETTLE);

    // --- Intermediate hop 2: ChainB -> ChainC (chain active -> no settle). ---
    tickUntilBlindForward(hw0);
    State s2 = retireBlindHopAndPeek(hw0);
    ASSERT_EQ(Localization.getCurrentNode(), nC);
    ASSERT_FALSE(s2 == State::SETTLE);

    // --- FINAL chained hop 3: ChainC -> ChainD. The chain retires at THIS hop's dispatch, so
    //     `_chain.active` reads false at its arrival -> the braked stop DOES settle (dwell-1). ---
    tickUntilBlindForward(hw0);
    State s3 = retireBlindHopAndPeek(hw0);
    ASSERT_EQ(Localization.getCurrentNode(), nD);
    ASSERT_EQ(s3 == State::SETTLE, true);    // final chained braked hop DOES settle
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the dwell: ChainD is the goal, so the FSM leaves SETTLE and settles to IDLE.
    driveSettle();
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC4 — a normal hop with a cold pre-edge turn followed by a move: TWO settles on ONE hop.
//       Route `goTo(Corner)` from `Inter3`: the `Inter3->Inter4` LINE hop brakes to rest at
//       Inter4 (dwell-1), then the `Inter4->Corner` cold 90 deg pivot turn settles turn->move
//       (dwell-2). Matrix rows: `LINE_OVERSHOOT normal` -> YES (dwell-1); `Cold pre-edge turn
//       done, normal hop` -> YES (dwell-2).
// ===========================================================================================
void test_TC4_cold_turn_then_move_two_dwells()
{
    TEST_SCOPE("TC4: Inter3->Inter4->Corner -- dwell-1 (braked arrival) AND dwell-2 (cold turn->move)");

    const uint8_t inter3 = nodeByName("Inter3");
    const uint8_t inter4 = nodeByName("Inter4");
    const uint8_t corner = nodeByName("Corner");
    ASSERT_TRUE(inter3 != FieldMap::NODE_NONE && inter4 != FieldMap::NODE_NONE);
    ASSERT_TRUE(corner != FieldMap::NODE_NONE);
    // Route pin: goTo(Corner) from Inter3 is Inter3->Inter4->Corner.
    ASSERT_EQ(routedFirstHop(inter3, corner), inter4);
    ASSERT_EQ(routedFirstHop(inter4, corner), corner);
    ASSERT_TRUE(FieldMap::getEdge(inter3, inter4).type == Routes::EdgeType::LINE);
    // Inter4->Corner is a genuine turn (heading 90 vs the inbound heading 0).
    const Routes::EdgeInfo& outEdge = FieldMap::getEdge(inter4, corner);
    ASSERT_TRUE(fabsf(wrapDeltaDeg(outEdge.heading_deg - 0.0f)) > Config::NAV_TURN_TOLERANCE_DEG);

    // Drive to LINE_FOLLOW for the first hop Inter3->Inter4 (heading 0 == seated 0, no pre-turn).
    driveToLineFollowEntry("Inter3", "Corner");
    ASSERT_EQ(Localization.getCurrentNode(), inter3);

    // Cross Inter4. This is a braked LINE arrival (NOT a pass-through: the onward Inter4->Corner is
    // a 90 deg bend, so the pass-through peek declines). dwell-1 must fire.
    expectLineHop();
    ASSERT_EQ(Localization.getCurrentNode(), inter4);
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);    // DWELL-1: braked arrival at Inter4
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire dwell-1: the FSM resumes to PLAN_NEXT_HOP, plans Inter4->Corner, and dispatches the
    // cold 90 deg pre-turn -> TURNING. Assert it reached TURNING without a second stray settle.
    driveSettle();
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() != State::TURNING && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_EQ(PathTracker.state() == State::TURNING, true);    // the cold turn dispatched
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the cold turn: park the app-frame Corner facing (90) each tick until the turn brakes.
    // At TURNING idle the FSM enters SETTLE with resume EDGE_AFTER_TURN (dwell-2).
    expectPreTurn((int16_t) lroundf(outEdge.heading_deg), (uint8_t) Routes::PIPELINE_CONFIG.turn);
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);    // DWELL-2: cold turn -> move
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire dwell-2: the FSM leaves SETTLE and dispatches the Inter4->Corner move (LINE follow).
    driveSettle();
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC5 (Part A) — goal arrival: the FINAL braked hop of a goal settles BEFORE the FSM reaches
//       IDLE. Route `goTo(Inter3)` from `Inter2` (the single LINE hop Inter2->Inter3): the
//       braked arrival at the goal Inter3 settles (dwell-1), then resumes to PLAN_NEXT_HOP,
//       sees cur==dest, and lands IDLE. Matrix row: `Goal arrival` -> YES.
// ===========================================================================================
void test_TC5a_goal_arrival_settles_before_idle()
{
    TEST_SCOPE("TC5a: goal arrival (Inter2->Inter3) settles on the final braked hop, then IDLE");

    const uint8_t inter2 = nodeByName("Inter2");
    const uint8_t inter3 = nodeByName("Inter3");
    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE && inter3 != FieldMap::NODE_NONE);
    ASSERT_EQ(routedFirstHop(inter2, inter3), inter3);    // a single direct LINE hop to the goal
    ASSERT_TRUE(FieldMap::getEdge(inter2, inter3).type == Routes::EdgeType::LINE);

    driveToLineFollowEntry("Inter2", "Inter3");
    ASSERT_EQ(Localization.getCurrentNode(), inter2);

    // Cross Inter3 — the goal. A braked LINE arrival at the destination: dwell-1 fires BEFORE the
    // goal-reached decision (PLAN_NEXT_HOP re-runs only AFTER the dwell, so the final hop's SETTLE
    // naturally precedes the IDLE transition).
    expectLineHop();
    ASSERT_EQ(Localization.getCurrentNode(), inter3);
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);    // goal-arrival settle
    ASSERT_FALSE(PathTracker.isIdle());                        // NOT idle yet -- still holding
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the dwell: PLAN_NEXT_HOP re-runs, cur==dest==Inter3 -> IDLE.
    driveSettle();
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC5 (Part B) — first-plan split: a FRESH goal whose first hop needs a COLD pre-edge turn.
//       dwell-1 (pre-maneuver) does NOT fire (no braked arrival precedes the first plan — the
//       FSM goes DEQUEUE -> PLAN_NEXT_HOP -> TURNING with no intervening SETTLE), BUT dwell-2
//       (turn->move) DOES fire. Route `goTo(Corner)` seated at `Inter4` facing 0: the first hop
//       Inter4->Corner is a 90 deg cold turn. Matrix rows: `First-plan PRE-maneuver dwell` ->
//       no (structurally absent); `First-hop turn->move dwell` -> YES.
// ===========================================================================================
void test_TC5b_first_plan_no_dwell1_but_dwell2_fires()
{
    TEST_SCOPE("TC5(B): fresh goal, first hop cold turn -- no dwell-1, but dwell-2 (turn->move) fires");

    const uint8_t inter4 = nodeByName("Inter4");
    const uint8_t corner = nodeByName("Corner");
    ASSERT_TRUE(inter4 != FieldMap::NODE_NONE && corner != FieldMap::NODE_NONE);
    ASSERT_EQ(routedFirstHop(inter4, corner), corner);
    const Routes::EdgeInfo& edge = FieldMap::getEdge(inter4, corner);
    // A genuine cold turn: Inter4->Corner departs heading 90, seated at app-yaw 0.
    ASSERT_TRUE(fabsf(wrapDeltaDeg(edge.heading_deg - 0.0f)) > Config::NAV_TURN_TOLERANCE_DEG);

    resetWorld();
    seatAt("Inter4", 0);    // seated facing 0; the first hop needs a 90 deg pre-turn.
    PathTracker.goTo(corner).async();

    // DWELL-1 STRUCTURALLY ABSENT: from DEQUEUE the FSM plans the first hop and dispatches the cold
    // turn straight into TURNING, with NO braked arrival state before it -- so SETTLE is never
    // observed between goTo() and the turn dispatch. `tickToStateNoSettle` walks to TURNING and
    // asserts SETTLE was never seen on the way (parking heading 0 -- the turn hasn't settled yet,
    // so the pre-turn stays in flight and the FSM parks in TURNING).
    tickToStateNoSettle(State::TURNING, hwYaw(0));

    // DWELL-2 FIRES: retire the cold turn (park the Corner app-facing 90 each tick). At TURNING idle
    // the FSM enters SETTLE (EDGE_AFTER_TURN) -- the first hop's cold turn settles like any other.
    expectPreTurn((int16_t) lroundf(edge.heading_deg), (uint8_t) Routes::PIPELINE_CONFIG.turn);
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);    // dwell-2 on the FIRST hop's cold turn
    ASSERT_FALSE(PathTracker.hasFault());

    driveSettle();
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC5b (matrix: first cold turn AFTER a wall/return interlude) — the interlude's OWN turns /
//       moves settle NEITHER; the first normal hop AFTER `WALL_DEPART` settles turn->move.
//       Drive the `position1 -> Wall1` RESET arrival, then a QUEUED goal beyond Wall1
//       (`goTo(Inter3)`, whose first hop Wall1->position1 needs a ~50 deg turn off the squared
//       270 facing). Matrix rows: interlude own turns/moves -> no; first normal hop after
//       WALL_DEPART -> YES.
// ===========================================================================================
void test_TC5b_first_cold_turn_after_wall_interlude()
{
    TEST_SCOPE("TC5b: wall interlude own turns/moves skip SETTLE; first hop after WALL_DEPART settles");

    const uint8_t position1 = nodeByName("position1");
    const uint8_t wall1      = nodeByName("Wall1");
    const uint8_t inter3     = nodeByName("Inter3");
    ASSERT_TRUE(position1 != FieldMap::NODE_NONE && wall1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNodeType(wall1) == Routes::NodeType::RESET);
    // The goal must be BEYOND Wall1 so the post-depart plan has a real next hop; from Wall1 the
    // only outbound edge is the reverse Wall1->position1 BLIND edge (heading ~320.36).
    ASSERT_EQ(routedFirstHop(wall1, inter3), position1);

    // Constants from the Wall1 reset record + depart geometry (see test_path_tracker_reset_arrival):
    // facing 270, standoff 15, WALL_CONTACT_OFFSET_CM 8 -> departCm 7.
    constexpr int16_t kWall1Facing    = 270;
    constexpr float   kWall1DepartCm   = 7.0f;

    // Drive the inbound BLIND dead-reckon hop into the RESET node Wall1 (firstPlan consumed, the
    // interlude pending on the next plan tick).
    driveInboundHop("position1", "Wall1");
    ASSERT_EQ(Localization.getCurrentNode(), wall1);

    // ARRIVAL AT A RESET NODE SKIPS dwell-1: the FSM enters the wall interlude (WALL_STOP), never
    // SETTLE. It is currently in PLAN_NEXT_HOP (the arrival's plan tick has not yet dispatched the
    // wallStop) or already WALL_STOP -- either way, NOT SETTLE.
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);

    // Queue the beyond-Wall1 goal so the post-depart plan has a real next hop.
    PathTracker.goTo(inter3).async();

    // Retire the wall interlude, asserting SETTLE is NEVER observed during its OWN SQUARE turn /
    // PUSH (WALL_STOP dispatches wallStop directly, not through the TURNING+settle path).
    // `expectWallStop` internally ticks the SQUARE turn + PUSH; wrap a state check by peeking after.
    expectWallStop(kWall1Facing, /*timedOut*/ false);
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);    // WALL_STOP's own SQUARE turn never settled

    // REANCHOR -> WALL_DEPART: tick until the depart drive is in flight (parking the squared 270
    // facing so the GYRO hold settles), asserting SETTLE is never entered (the interlude's own
    // depart move does not settle).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Drive.isIdle() && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
        cooperativeTick();
        ASSERT_FALSE(PathTracker.state() == State::SETTLE);
    }
    ASSERT_FALSE(Drive.isIdle());          // the depart drive dispatched (REANCHOR succeeded)
    ASSERT_FALSE(PathTracker.hasFault());

    // Retire the depart drive (a bounded GYRO driveDist) -- the interlude's own move, no settle.
    mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
    expectBlindHop(kWall1DepartCm);
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);

    // WALL_DEPART -> clearOffset -> PLAN_NEXT_HOP. The FSM now plans the FIRST NORMAL hop after the
    // interlude: Wall1->position1 (heading ~320.36), a genuine cold turn off the squared 270 facing.
    // That turn->move DOES settle (dwell-2). Tick to TURNING (parking 270 so the depart's residual
    // settle resolves), asserting no stray SETTLE before the turn dispatches.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() != State::TURNING && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(kWall1Facing);
        cooperativeTick();
    }
    ASSERT_EQ(PathTracker.state() == State::TURNING, true);    // first normal hop's cold turn dispatched

    // Retire the cold turn (park the Wall1->position1 app-heading each tick). At TURNING idle the
    // FSM enters SETTLE (EDGE_AFTER_TURN) -- the interlude is over, so this normal hop settles.
    const Routes::EdgeInfo& out = FieldMap::getEdge(wall1, position1);
    expectPreTurn((int16_t) lroundf(out.heading_deg), (uint8_t) Routes::PIPELINE_CONFIG.turn);
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);    // dwell-2 on the first hop after the interlude
    ASSERT_FALSE(PathTracker.hasFault());

    driveSettle();
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// TC6 — arrival at a RESET node does NOT settle: the wall interlude owns that boundary. Drive
//       `position1 -> Wall1` and assert the FSM transitions into WALL_STOP (never SETTLE), and
//       the interlude proceeds exactly as test_path_tracker_reset_arrival asserts. Matrix row:
//       `Arrival at a RESET node` -> no.
// ===========================================================================================
void test_TC6_reset_arrival_skips_settle()
{
    TEST_SCOPE("TC6: arrival at RESET node Wall1 skips SETTLE -> enters WALL_STOP interlude");

    const uint8_t position1 = nodeByName("position1");
    const uint8_t wall1      = nodeByName("Wall1");
    ASSERT_TRUE(position1 != FieldMap::NODE_NONE && wall1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getEdge(position1, wall1).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getNodeType(wall1) == Routes::NodeType::RESET);

    driveInboundHop("position1", "Wall1");
    ASSERT_EQ(Localization.getCurrentNode(), wall1);

    // The braked BLIND arrival at a RESET node routes to the wall interlude, NOT SETTLE
    // (`_settleOrPlanAfterArrival` gate: getNodeType==RESET -> plan straight, and the plan tick's
    // `_maybeEnterReset` enters WALL_STOP). Assert the FSM reaches WALL_STOP, never SETTLE.
    bool sawSettle = false;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() != State::WALL_STOP && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
        if (PathTracker.state() == State::SETTLE)
        {
            sawSettle = true;
        }
    }
    ASSERT_EQ(PathTracker.state() == State::WALL_STOP, true);    // the wall interlude was entered
    ASSERT_FALSE(sawSettle);                                     // ... and SETTLE never was
    ASSERT_FALSE(PathTracker.hasFault());

    // The interlude then proceeds exactly as test_path_tracker_reset_arrival asserts: retire the
    // wallStop push (SQUARE turn + PUSH) with no fault.
    expectWallStop(270, /*timedOut*/ false);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// TC7b — end-to-end at the shipped default (100): a full `braked hop -> SETTLE -> next
//       maneuver` sequence, EXPLICITLY asserting the dwell is genuinely non-blocking / clock-
//       gated -- SETTLE holds across a tick BEFORE the clock jump retires it (it is not a
//       same-tick passthrough). Reuses the STOP_AT_LINE scenario. Matrix: `STOP_AT_LINE` -> YES.
// ===========================================================================================
void test_TC7b_end_to_end_settle_is_clock_gated()
{
    TEST_SCOPE("TC7b: braked hop -> SETTLE holds across ticks until the clock passes the deadline -> next");

    ASSERT_GT(Config::HOP_SETTLE_MS, (uint32_t) 0);    // this target runs at the shipped default (100)

    const uint8_t start  = nodeByName("Start");
    const uint8_t inter1 = nodeByName("Inter1");
    ASSERT_EQ(routedFirstHop(start, inter1), inter1);

    resetWorld();
    seatAt("Start", 0);
    PathTracker.goTo(inter1).async();
    const Routes::EdgeInfo& edge = FieldMap::getEdge(start, inter1);

    // Cold pre-turn (first hop): tick to TURNING (parking heading 0), retire the turn, retire its
    // dwell-2, then drive the seek in flight and to a crossing.
    tickToStateNoSettle(State::TURNING, hwYaw(0));
    expectPreTurn((int16_t) lroundf(edge.heading_deg), (uint8_t) Routes::PIPELINE_CONFIG.turn);
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);
    driveSettle();
    tickUntilSeekInFlight(hwYaw((int16_t) lroundf(edge.heading_deg)));
    expectStopAtLine(edge.distance_cm, /*found*/ true);

    // The braked STOP_AT_LINE arrival entered SETTLE.
    ASSERT_EQ(PathTracker.state() == State::SETTLE, true);

    // NON-BLOCKING / CLOCK-GATED PROOF: tick the FSM WITHOUT advancing the clock past the deadline.
    // `cooperativeTick()` advances only `path_harness::kMillisPerTick` (5 ms) per tick, well below
    // the 100 ms dwell, so several bare ticks must NOT retire the dwell -- the FSM stays in SETTLE,
    // proving it genuinely waits on `millis()` and is not a same-tick passthrough.
    for (int i = 0; i < 5; ++i)
    {
        cooperativeTick();
        ASSERT_EQ(PathTracker.state() == State::SETTLE, true);    // still holding (clock < deadline)
    }
    ASSERT_FALSE(PathTracker.hasFault());

    // Now jump the clock past the deadline (driveSettle) and confirm the FSM proceeds -- the full
    // integrated braked-hop -> SETTLE -> next-maneuver path at the default 100.
    driveSettle();
    ASSERT_FALSE(PathTracker.state() == State::SETTLE);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================================
// Entry.
// ===========================================================================================
int main()
{
    std::printf("INFO: PathTracker ADR-061 post-stop settle dwell (T4, TC1-TC7b) -- real "
                "PathTracker + Drive + Localization over routes_fixture @ HOP_SETTLE_MS=%lu\n",
                (unsigned long) Config::HOP_SETTLE_MS);

    // Precondition: this target compiles at the shipped default so the settle actually fires.
    {
        TEST_SCOPE("precondition: HOP_SETTLE_MS > 0 (this target runs the settle at the default)");
        ASSERT_GT(Config::HOP_SETTLE_MS, (uint32_t) 0);
    }

    test_TC1_braked_stop_enters_settle();
    test_TC2_line_pass_through_skips_settle();
    test_TC3_blind_chain_intermediates_skip_final_settles();
    test_TC4_cold_turn_then_move_two_dwells();
    test_TC5a_goal_arrival_settles_before_idle();
    test_TC5b_first_plan_no_dwell1_but_dwell2_fires();
    test_TC5b_first_cold_turn_after_wall_interlude();
    test_TC6_reset_arrival_skips_settle();
    test_TC7b_end_to_end_settle_is_clock_gated();

    TEST_REPORT();
}
