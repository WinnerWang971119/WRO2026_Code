/**
 * @file test_path_tracker_lost_line.cpp
 * @brief Host-link integration test for the bounded lost-line fault ESCALATION
 *        (issue #146, T7), driving the REAL `PathTracker` + `Drive` FSMs through
 *        the shared cooperative-tick harness.
 *
 * @details
 * Exercises the #146 escalation: while line-following (`State::LINE_FOLLOW` ONLY),
 * PathTracker polls `LineFollower.isLineLost()` and, after
 * `Config::LINE_LOST_FAULT_MS` of CONTINUOUS loss outside a PathTracker-owned
 * pass-through dead-reckon window, latches `PathTrackerClass::Fault::LINE_LOST`.
 * This is bounded fault escalation, NOT the deferred ADR-009 sweep/reacquire/resume
 * recovery.
 *
 * THE HOP (`fixtures/routes_fixture.h`): the collinear `line-line-line` triple
 * `Inter2 -> Inter3 -> Inter4` (all LINE edges at heading 0, all LINE nodes) — the
 * same geometry the pass-through test uses. A `goTo(Inter3)` from `Inter2` is one
 * LINE hop (the poll site); a `goTo(Inter4)` from `Inter2` routes through the mid
 * junction `Inter3`, whose overshoot the pass-through gate collapses (the suppressed
 * dead-reckon window TC6 rides through).
 *
 * The mock LineFollower's `isLineLost()` is a test-set flag (`mockSetLineLost`), so a
 * test scripts a loss directly; the harness advances a mock `millis()` clock by
 * `path_harness::kMillisPerTick` per tick, so crossing `LINE_LOST_FAULT_MS` is a
 * matter of ticking a computed number of times with the loss held.
 *
 *   - TC5a: a sustained loss mid-`LINE_FOLLOW` past the window -> `Fault::LINE_LOST`.
 *   - TC5b: a transient loss that reacquires BEFORE the window resets the timer.
 *   - TC5c: the loss timer is cleared across `abort()` (the loss held continuous so
 *           only the abort-time clear can restart the clock).
 *   - TC5d: `Fault::LINE_LOST` maps to a distinct fault mnemonic.
 *   - TC6:  a "lost" line through a pass-through / overshoot hop does NOT fault —
 *           `LINE_OVERSHOOT` is a deliberate dead-reckon and is never polled.
 */

#include <cstring>

#include "test_path_tracker_harness.h"

namespace
{
using path_harness::kMaxHandshakeTicks;
using path_harness::kMillisPerTick;

/// @brief Ticks of continuous loss that keep the FSM SAFELY UNDER `LINE_LOST_FAULT_MS`
///        (no escalation). At 500 ms / 5 ms this is 90 ticks (~450 ms).
constexpr int kTicksUnderWindow =
    (int) (Config::LINE_LOST_FAULT_MS / kMillisPerTick) - 10;

/// @brief A generous cap on ticks to REACH the escalation once loss is continuous. At
///        500 ms / 5 ms this is 120 ticks (~600 ms) — comfortably past the 101-tick fault.
constexpr int kTicksToFaultCap =
    (int) (Config::LINE_LOST_FAULT_MS / kMillisPerTick) + 20;

/// @brief Tick @p n times holding the line reported LOST, from an already-`LINE_FOLLOW` FSM.
/// @note No intersection is latched, so the FSM stays in `LINE_FOLLOW` and the poll runs
///       every tick. The mock's `_lineLost` is sticky (only `mockSetLineLost` / `mockReset`
///       clear it), so one set suffices.
void tickLostFor(int n)
{
    for (int i = 0; i < n; ++i)
    {
        cooperativeTick();
    }
}
}    // namespace

// ===========================================================================
// Routing guard — pin the hop geometry so a future fixture re-weight fails LOUDLY.
// ===========================================================================

void test_routing_geometry_is_the_collinear_triple()
{
    TEST_SCOPE("routing: Inter2 -> Inter3 -> Inter4 is the collinear LINE triple");

    const uint8_t inter2 = nodeByName("Inter2");
    const uint8_t inter3 = nodeByName("Inter3");
    const uint8_t inter4 = nodeByName("Inter4");

    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter4 != FieldMap::NODE_NONE);

    // goTo(Inter3) is a single LINE hop; goTo(Inter4) routes THROUGH Inter3 (no bypass).
    ASSERT_EQ(routedFirstHop(inter2, inter3), inter3);
    ASSERT_EQ(routedFirstHop(inter2, inter4), inter3);
    assertNoBypass(inter2, inter4);

    ASSERT_TRUE(FieldMap::getEdge(inter2, inter3).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(inter3, inter4).type == Routes::EdgeType::LINE);
}

// ===========================================================================
// TC5a — a sustained lost line mid-LINE_FOLLOW escalates to Fault::LINE_LOST.
// ===========================================================================

void test_sustained_loss_escalates_to_line_lost()
{
    TEST_SCOPE("TC5a: sustained lost line > LINE_LOST_FAULT_MS -> Fault::LINE_LOST");

    const uint8_t inter2 = nodeByName("Inter2");
    const uint8_t inter3 = nodeByName("Inter3");

    driveToLineFollowEntry("Inter2", "Inter3");
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::LINE_FOLLOW);
    ASSERT_FALSE(PathTracker.hasFault());

    // Line drops and stays down. The FSM sits in LINE_FOLLOW (no intersection latched),
    // so the poll runs every tick; the escalation fires once the loss reaches the window.
    LineFollower.mockSetLineLost(true);
    bool faulted = false;
    for (int i = 0; i < kTicksToFaultCap; ++i)
    {
        cooperativeTick();
        if (PathTracker.hasFault())
        {
            faulted = true;
            break;
        }
    }

    ASSERT_TRUE(faulted);
    ASSERT_TRUE(PathTracker.getFault().kind == PathTrackerClass::Fault::LINE_LOST);
    // from = the departure node the chassis was still anchored to; to = the in-flight goal.
    ASSERT_EQ(PathTracker.getFault().fromNode, inter2);
    ASSERT_EQ(PathTracker.getFault().destNode, inter3);
    // The fault latched the FSM into FAULT (motion halted).
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::FAULT);
}

// ===========================================================================
// TC5b — a transient loss that reacquires BEFORE the window resets the timer.
// ===========================================================================

void test_transient_loss_reacquire_resets_timer()
{
    TEST_SCOPE("TC5b: transient loss that reacquires resets the timer (no escalation)");

    driveToLineFollowEntry("Inter2", "Inter3");
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::LINE_FOLLOW);

    // First loss burst — safely under the window.
    LineFollower.mockSetLineLost(true);
    tickLostFor(kTicksUnderWindow);
    ASSERT_FALSE(PathTracker.hasFault());

    // Reacquire for one tick — the poll's line-present branch clears the timer.
    LineFollower.mockSetLineLost(false);
    cooperativeTick();
    ASSERT_FALSE(PathTracker.hasFault());

    // Second loss burst — again under the window. Total elapsed loss is now
    // 2 * kTicksUnderWindow * kMillisPerTick (~900 ms), well past LINE_LOST_FAULT_MS; only
    // the reacquire-reset keeps this from faulting. A regression that failed to reset on
    // reacquire would escalate here.
    LineFollower.mockSetLineLost(true);
    tickLostFor(kTicksUnderWindow);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::LINE_FOLLOW);
}

// ===========================================================================
// TC5c — the loss timer is cleared across abort(), loss held CONTINUOUS.
// ===========================================================================

void test_loss_timer_cleared_across_abort()
{
    TEST_SCOPE("TC5c: lost-line timer cleared across abort() (loss held continuous)");

    driveToLineFollowEntry("Inter2", "Inter3");
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::LINE_FOLLOW);

    // Accrue loss time under the window, then abort while STILL lost.
    LineFollower.mockSetLineLost(true);
    tickLostFor(kTicksUnderWindow);
    ASSERT_FALSE(PathTracker.hasFault());

    PathTracker.abort();
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(PathTracker.isIdle());

    // Re-arm a fresh goal on the SAME seat (no resetWorld / begin() — either would ALSO reset
    // the timer and mask a missing abort-time clear) WITHOUT ever reporting the line present.
    // Because the FSM never sees the line back between abort and re-entry, the ONLY thing that
    // can restart the loss clock is abort()'s clear: were `_lineLostTiming` stale-true, the
    // first LINE_FOLLOW tick would see `millis() - _lineLostSinceMs` already past the window and
    // fault immediately (the loop would break on hasFault() with reachedFollow == false).
    PathTracker.goTo(nodeByName("Inter3")).async();
    bool reachedFollow = false;
    for (int i = 0; i < kMaxHandshakeTicks; ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(0);    // heading-0 departure — no pre-turn fires
        cooperativeTick();
        if (PathTracker.hasFault())
        {
            break;    // a stale timer would escalate here
        }
        if (PathTracker.state() == PathTrackerClass::State::LINE_FOLLOW)
        {
            reachedFollow = true;
            break;
        }
    }
    ASSERT_TRUE(reachedFollow);
    ASSERT_FALSE(PathTracker.hasFault());

    // Continue the (still continuous) loss for a fresh under-window burst — no escalation,
    // proving the clock restarted at the re-entry, not at the pre-abort loss start.
    tickLostFor(kTicksUnderWindow);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC5d — Fault::LINE_LOST has a distinct mnemonic (not the "??" fallthrough).
// ===========================================================================

void test_line_lost_has_distinct_mnemonic()
{
    TEST_SCOPE("TC5d: Fault::LINE_LOST maps to a distinct mnemonic");

    const char* mnem = PathTrackerClass::faultMnemonic(PathTrackerClass::Fault::LINE_LOST);
    ASSERT_TRUE(std::strcmp(mnem, "LLN") == 0);
    // Not the unknown-enum fallthrough, and distinct from the neighbouring fault tag.
    ASSERT_TRUE(std::strcmp(mnem, "??") != 0);
    ASSERT_TRUE(
        std::strcmp(mnem, PathTrackerClass::faultMnemonic(PathTrackerClass::Fault::PIVOT_TRIM_UNDERFLOW))
        != 0);
}

// ===========================================================================
// TC6 — a "lost" line through a pass-through / overshoot hop does NOT fault.
// ===========================================================================

void test_no_fault_through_passthrough_overshoot()
{
    TEST_SCOPE("TC6: lost line through a pass-through / overshoot hop does NOT fault (suppressed)");

    const uint8_t inter3 = nodeByName("Inter3");

    // goTo(Inter4): Inter3 is a NON-goal mid junction, so the pass-through peek fires there.
    driveToLineFollowEntry("Inter2", "Inter4");
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::LINE_FOLLOW);

    // Cross the mid junction Inter3. The pass-through peek (Inter4 collinear, non-goal) arms
    // LINE_OVERSHOOT with brakeOnGate == false and latches `_passThrough`; the FSM leaves the
    // LINE_FOLLOW poll site for the deliberate dead-reckon overshoot.
    Intersection.mockSetAtIntersection(true);
    cooperativeTick();
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::LINE_OVERSHOOT);
    ASSERT_TRUE(LineFollower.lastOvershootMode);
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);    // pass-through arm (no brake on the gate)

    // Hold the line "lost" through the overshoot WELL past LINE_LOST_FAULT_MS. LINE_OVERSHOOT
    // is never polled (a deliberate dead-reckon past the line), so no escalation fires — the
    // FSM stays in LINE_OVERSHOOT waiting on its distance gate the whole time.
    LineFollower.mockSetLineLost(true);
    for (int i = 0; i < kTicksToFaultCap; ++i)
    {
        cooperativeTick();
        ASSERT_FALSE(PathTracker.hasFault());
    }
    ASSERT_TRUE(PathTracker.state() == PathTrackerClass::State::LINE_OVERSHOOT);

    // Trip the overshoot distance gate: the pass-through completes and advances onto Inter3
    // (a dead-reckon re-anchor), still with no fault.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), inter3);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_routing_geometry_is_the_collinear_triple();
    test_sustained_loss_escalates_to_line_lost();
    test_transient_loss_reacquire_resets_timer();
    test_loss_timer_cleared_across_abort();
    test_line_lost_has_distinct_mnemonic();
    test_no_fault_through_passthrough_overshoot();
    TEST_REPORT();
}
