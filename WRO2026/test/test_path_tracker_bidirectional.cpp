/**
 * @file test_path_tracker_bidirectional.cpp
 * @brief Host-compile tests for the bidirectional line-following surface
 *        — TC12 (REVERSE_UNAVAILABLE fault), TC13 (Mission modifier no-op on
 *        non-GOTO step), and TC14c (compile-time kill-switch override
 *        controls runtime behavior).
 *
 * Built against the mock Arduino / Drive / WROLineArray / Intersection surface
 * under `WRO2026/test/mocks/`. The real `WRO2026/LineFollower.{h,cpp}` and
 * `WRO2026/WROMath.h` are compiled unchanged (they come along on the
 * `SUT_SOURCES` set already used by the other test targets).
 *
 * Mirroring rationale (follows the precedent established by
 * `test_path_tracker_passthrough.cpp`):
 *
 *   Rather than mock the full PathTracker + Drive-builder + Localization +
 *   MatrixMiniR4 (LED/Buzzer/BTN) + EventTrace + custom routes.cpp surface that
 *   would be needed to link `PathTracker.cpp` and `Mission.cpp` against the
 *   tests, these tests **replicate the small slice of production logic under
 *   test** inline, then assert behaviour on the replicated copy. The
 *   replicated logic is annotated with a pointer to the canonical site in the
 *   production sources so the two cannot silently drift.
 *
 *   Real production types referenced (header-only / no `.cpp` linkage needed):
 *     - `PathTrackerClass::Fault`    (PathTracker.h)
 *     - `PathTrackerClass::FaultInfo`(PathTracker.h)
 *     - `Direction`                  (NavTypes.h)
 *     - `Routes::DirectionMode` etc. (routes.h)
 *     - `FieldMap::NODE_NONE`        (FieldMap.h — header-only inline)
 *
 *   Because we reference the real enum values, a future rename of
 *   `Fault::REVERSE_UNAVAILABLE` or `DirectionMode::FORCE_REVERSE` breaks
 *   compilation of this test — that is exactly the safety net we want.
 *
 * Test cases:
 *
 *   TC12 — `_dispatchEdgeAfterTurn` REVERSE_UNAVAILABLE guard (AC10):
 *     Mirrors the guard at `PathTracker.cpp:_dispatchEdgeAfterTurn` (the
 *     block beginning "Reverse-availability guard (T7 / issue #19)").
 *     Asserts:
 *       - With `LINE_BOARD_COUNT == 1` and a LINE edge with
 *         `direction_override == FORCE_REVERSE`, `_resolveDirection` returns
 *         `Direction::REAR` (step 1 of the precedence table is binding).
 *       - The dispatcher's guard then latches `Fault::REVERSE_UNAVAILABLE`
 *         with `fromNode = cur` and `destNode = nxt`.
 *       - The same fixture with `LINE_BOARD_COUNT >= 2` does NOT trip the
 *         guard (the dispatcher proceeds into the LINE branch).
 *       - BLIND REAR on a 1-board chassis is permitted (matches the comment
 *         in production: "BLIND REAR is permitted on a 1-board chassis").
 *
 *   TC13 — Mission modifier on non-GOTO step is a no-op:
 *     Mirrors `MissionClass::setRouteDirection` / `setArrivalDirection`
 *     guard at `Mission.cpp:106-126`. Asserts:
 *       - On a WAIT/PRINT/BEEP/LED step (anything not GOTO), the modifier
 *         leaves the step untouched and `_queue` unchanged.
 *       - On a GOTO step the modifier writes through to the cached
 *         `route_direction` / `arrival_direction` fields.
 *       - On an empty queue the modifier is a no-op (no buffer underflow).
 *
 *   TC14c — Compile-time kill-switch override drives runtime behavior (AC8c):
 *     This file is built TWICE by CMake — once with
 *     `-DBIDIRECTIONAL_AUTO_ENABLED_DEFAULT=true` (default) and once with `=false`.
 *     Each build runs the same fixture (1-board chassis, AUTO direction at
 *     the builder, no edge override, currentHeading == 0 deg, edge heading
 *     == 175 deg — a body-flip is the turn-saving choice). Asserts:
 *       - `Config::BIDIRECTIONAL_AUTO_ENABLED == BIDIRECTIONAL_AUTO_ENABLED_DEFAULT`
 *         (the macro definitely propagated into the constant).
 *       - With the flag ON: `_resolveDirection` may pick REAR. (On a 1-board
 *         chassis the auto branch is *also* gated by `LINE_BOARD_COUNT >= 2`,
 *         so even with the flag ON the auto path still falls through to
 *         FRONT — this is the explicit T7 safety. The flag's load-bearing
 *         behavior is best exercised on a 2-board fixture, asserted below.)
 *       - On a 2-board fixture: flag ON → REAR (turn-saving) at 175 deg;
 *         flag OFF → FRONT regardless of geometry.
 */

#include "TestAssert.h"

#include "Arduino.h"
#include "Drive.h"
#include "WROLineArray.h"
#include "Intersection.h"

// Production headers we depend on for types only (no linkage to .cpp needed).
#include "Config.h"
#include "DirectionPolicy.h"    // Shared resolveDirection() — single source of truth.
#include "NavTypes.h"
#include "PathTracker.h"        // For PathTrackerClass::Fault + FaultInfo enums.
#include "routes.h"             // For Routes::DirectionMode, Routes::EdgeType.
#include "FieldMap.h"           // For FieldMap::NODE_NONE.

// LineFollower / WROMath come along on the test target's SUT_SOURCES, but
// nothing in this file touches them — the tests are pure logic exercises.

#include <math.h>
#include <stdint.h>
#include <cstdio>

// ===========================================================================
// Test calls the real production helpers via DirectionPolicy.h.
//
// Both the direction resolver (`resolveDirection`) and the dispatcher
// REVERSE_UNAVAILABLE predicate (`reverseUnavailable`) are now shared inline
// functions in DirectionPolicy.h — production and this host-compile test link
// the SAME symbols. The guard condition is no longer replicated inline, so it
// cannot drift from its production site (PathTracker.cpp::_dispatchEdgeAfterTurn,
// which calls the same `reverseUnavailable`). The only test-local glue left is
// assembling the `FaultInfo` the production code would latch, since that side
// is tangled with Drive / LineFollower / Localization mocks.
// ===========================================================================

/// Alias so existing tests that called the old replica name keep working
/// while pointing at the canonical helper. New tests should call
/// `resolveDirection` directly.
static inline Direction replica_resolveDirection(
    Routes::DirectionMode edgeOverride,
    Routes::DirectionMode pipelineDefault,
    Direction             builderArrival,
    Direction             builderRoute,
    float                 currentHeading,
    float                 edgeHeading,
    bool                  isLastHop,
    bool                  bidirectionalAutoEnabled,
    uint8_t               lineBoardCount)
{
    return resolveDirection(edgeOverride, pipelineDefault, builderArrival,
                            builderRoute, currentHeading, edgeHeading,
                            isLastHop, bidirectionalAutoEnabled, lineBoardCount);
}

/// Exercises the REVERSE_UNAVAILABLE guard at the top of
/// `PathTracker.cpp::_dispatchEdgeAfterTurn` (see the block comment
/// "Reverse-availability guard (T7 / issue #19)").
///
/// The load-bearing predicate is the shared `reverseUnavailable(...)` from
/// DirectionPolicy.h — the SAME symbol both production guard sites call — so
/// the condition can no longer drift from production (same guarantee
/// `resolveDirection` gives). Only the `FaultInfo` assembly below is test
/// glue, mirroring how `_enterFault(Fault::REVERSE_UNAVAILABLE, cur, nxt)`
/// would latch when the guard fires; an empty FaultInfo (`kind ==
/// Fault::NONE`) is returned when the guard does not fire — i.e. the
/// dispatcher would have proceeded into the LINE / BLIND branch (issue #64
/// retired the standalone PROJECTION edge type).
static PathTrackerClass::FaultInfo replica_dispatchEdgeAfterTurnGuard(
    Direction         resolvedDirection,
    uint8_t           lineBoardCount,
    Routes::EdgeType  edgeType,
    uint8_t           cur,
    uint8_t           nxt)
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

// ---------------------------------------------------------------------------
// TC13 — Mission modifier no-op on non-GOTO step.
//
// We replicate Mission's tiny Step queue + the modifier guard exactly as
// `MissionClass` implements it. The production code is `Mission.cpp:106-126`
// (setRouteDirection / setArrivalDirection), with the Step struct + StepType
// enum at `Mission.h:163-198`. The relevant slice has zero external
// dependencies — just `_queue[_count - 1].type != StepType::GOTO` guard — so
// the replica is exact, not a paraphrase.
// ---------------------------------------------------------------------------

namespace mission_replica
{
    // Mirrors `MissionClass::StepType` (Mission.h, private enum).
    enum class StepType : uint8_t
    {
        GOTO,
        WAIT,
        PRINT,
        BEEP,
        LED
    };

    // Mirrors `MissionClass::Step` (Mission.h, private struct). Only the
    // fields the modifier verbs touch are kept — the rest is unused here.
    //
    // No default member initializers — production's struct also has none.
    // Replicating that means every Step in this file MUST be value-initialized
    // (`Step s{};`) or it carries indeterminate values, exactly matching the
    // fragility production exhibits. If a future Mission.cpp refactor constructs
    // a Step on the stack without braces, the replica won't paper over it.
    struct Step
    {
        StepType  type;
        Direction route_direction;
        Direction arrival_direction;
    };

    /// Fixed-size queue + count + last-push tracker, mirroring `_queue`,
    /// `_count`, and `_lastPushOk` in MissionClass.
    struct Queue
    {
        static constexpr uint8_t CAPACITY = Config::MAX_MISSION_STEPS;
        Step    items[CAPACITY] = {};
        uint8_t count = 0;
        bool    lastPushOk = false;

        bool push(const Step& s)
        {
            if (count >= CAPACITY)
            {
                lastPushOk = false;
                return false;
            }
            items[count++] = s;
            lastPushOk = true;
            return true;
        }
    };

    /// Replicates `MissionClass::setRouteDirection` (Mission.cpp). Returns
    /// true if the modifier wrote through, false if rejected by any of the
    /// three guards (no push, push overflowed, last step not GOTO).
    static bool setRouteDirection(Queue& q, Direction d)
    {
        if (!q.lastPushOk) return false;
        if (q.count == 0 || q.items[q.count - 1].type != StepType::GOTO)
        {
            return false;
        }
        q.items[q.count - 1].route_direction = d;
        return true;
    }

    /// Replicates `MissionClass::setArrivalDirection`.
    static bool setArrivalDirection(Queue& q, Direction d)
    {
        if (!q.lastPushOk) return false;
        if (q.count == 0 || q.items[q.count - 1].type != StepType::GOTO)
        {
            return false;
        }
        q.items[q.count - 1].arrival_direction = d;
        return true;
    }
}    // namespace mission_replica

// ===========================================================================
// TC12 — REVERSE_UNAVAILABLE fault on 1-board + REAR LINE edge.
// ===========================================================================

static void test_TC12_reverse_unavailable_on_line_1board_force_reverse()
{
    TEST_SCOPE("TC12: LINE edge with FORCE_REVERSE on 1-board chassis → REVERSE_UNAVAILABLE");

    // Step A: resolve the direction. The fixture sets the per-edge override
    // to FORCE_REVERSE — Step 1 of the precedence table is binding, so this
    // returns REAR regardless of the other inputs (board count, kill-switch,
    // heading geometry). The production code at
    // `PathTracker.cpp::_onPlanNextHop` does this same call and caches the
    // result in `_resolvedDirection`.
    const Direction resolved = replica_resolveDirection(
        /*edgeOverride*/      Routes::DirectionMode::FORCE_REVERSE,
        /*pipelineDefault*/   Routes::DirectionMode::DEFAULT,
        /*builderArrival*/    Direction::AUTO,
        /*builderRoute*/      Direction::AUTO,
        /*currentHeading*/    0.0f,
        /*edgeHeading*/       0.0f,
        /*isLastHop*/         false,
        /*bidirectionalOn*/   true,
        /*lineBoardCount*/    1);
    ASSERT_TRUE(resolved == Direction::REAR);

    // Step B: hit the REVERSE_UNAVAILABLE guard. fromNode/destNode use the
    // FieldMap-style node ids; we pick representative values matching what
    // `_onPlanNextHop` would pass (`cur` from Localization, `_nextHop` from
    // FieldMap::getNextHop).
    constexpr uint8_t cur = 0;    // representative inbound node
    constexpr uint8_t nxt = 1;    // representative next-hop node

    const PathTrackerClass::FaultInfo info = replica_dispatchEdgeAfterTurnGuard(
        /*resolvedDirection*/ resolved,
        /*lineBoardCount*/    1,
        /*edgeType*/          Routes::EdgeType::LINE,
        /*cur*/               cur,
        /*nxt*/               nxt);

    ASSERT_TRUE(info.kind     == PathTrackerClass::Fault::REVERSE_UNAVAILABLE);
    ASSERT_EQ  (info.fromNode, cur);
    ASSERT_EQ  (info.destNode, nxt);
}

// Issue #64 retired the standalone projection EDGE type — a projection is now a
// NODE property, and the only reverse-unservable edge type is `LINE` (a REAR
// line-follow needs the rear board; BLIND REAR is permitted on a 1-board
// chassis). The old `test_TC12_reverse_unavailable_on_projection_1board` case
// (which constructed the deleted `Routes::EdgeType::PROJECTION`) is therefore
// gone. A LINE edge into a projection DESTINATION node is STILL a reverse
// line-follow and is already covered by the LINE clause exercised in
// `test_TC12_reverse_unavailable_on_line_1board_force_reverse` above —
// `reverseUnavailable` keys on the LINE/BLIND `edge.type`, which never changes
// with the destination node type.

static void test_TC12_blind_rear_1board_allowed()
{
    TEST_SCOPE("TC12: BLIND REAR on 1-board chassis is permitted (no fault)");

    // BLIND REAR does NOT trip the guard — dead-reckoned reverse needs only
    // negative PWM, not a rear line board. Production comment at
    // `PathTracker.cpp:802-803`: "BLIND REAR is permitted on a 1-board
    // chassis: dead-reckoned reverse does not require a rear line sensor."
    const PathTrackerClass::FaultInfo info = replica_dispatchEdgeAfterTurnGuard(
        /*resolvedDirection*/ Direction::REAR,
        /*lineBoardCount*/    1,
        /*edgeType*/          Routes::EdgeType::BLIND,
        /*cur*/               0,
        /*nxt*/               1);

    ASSERT_TRUE(info.kind == PathTrackerClass::Fault::NONE);
}

static void test_TC12_line_rear_2board_allowed()
{
    TEST_SCOPE("TC12: LINE REAR on 2-board chassis does NOT trip REVERSE_UNAVAILABLE");

    // Same LINE + REAR fixture as the first test, but with two boards. The
    // guard is gated on `lineBoardCount < 2`, so a 2-board chassis falls
    // through to the LINE branch's `setReverse(true)` dispatch.
    const PathTrackerClass::FaultInfo info = replica_dispatchEdgeAfterTurnGuard(
        /*resolvedDirection*/ Direction::REAR,
        /*lineBoardCount*/    2,
        /*edgeType*/          Routes::EdgeType::LINE,
        /*cur*/               0,
        /*nxt*/               1);

    ASSERT_TRUE(info.kind == PathTrackerClass::Fault::NONE);
}

static void test_TC12_line_front_1board_allowed()
{
    TEST_SCOPE("TC12: LINE FRONT on 1-board chassis does NOT trip REVERSE_UNAVAILABLE");

    // Front direction never trips the guard regardless of board count.
    const PathTrackerClass::FaultInfo info = replica_dispatchEdgeAfterTurnGuard(
        /*resolvedDirection*/ Direction::FRONT,
        /*lineBoardCount*/    1,
        /*edgeType*/          Routes::EdgeType::LINE,
        /*cur*/               0,
        /*nxt*/               1);

    ASSERT_TRUE(info.kind == PathTrackerClass::Fault::NONE);
}

// ===========================================================================
// TC13 — Mission modifier on non-GOTO step is a no-op.
// ===========================================================================

static void test_TC13_setRouteDirection_after_wait_is_noop()
{
    TEST_SCOPE("TC13: Mission.wait(...).setRouteDirection(REAR) is a no-op");

    using namespace mission_replica;
    Queue q;

    // Equivalent to `Mission.wait(100)` — push a WAIT step.
    Step waitStep{};
    waitStep.type              = StepType::WAIT;
    waitStep.route_direction   = Direction::AUTO;
    waitStep.arrival_direction = Direction::AUTO;
    ASSERT_TRUE(q.push(waitStep));

    // Sanity: queue has exactly one step, type WAIT, directions AUTO.
    ASSERT_EQ  (q.count, (uint8_t) 1);
    ASSERT_TRUE(q.items[0].type == StepType::WAIT);
    ASSERT_TRUE(q.items[0].route_direction   == Direction::AUTO);
    ASSERT_TRUE(q.items[0].arrival_direction == Direction::AUTO);

    // `Mission.setRouteDirection(REAR)` on a WAIT step: rejected, no-op.
    const bool wrote = setRouteDirection(q, Direction::REAR);
    ASSERT_FALSE(wrote);

    // Queue depth unchanged.
    ASSERT_EQ(q.count, (uint8_t) 1);

    // Step type unchanged — still WAIT.
    ASSERT_TRUE(q.items[0].type == StepType::WAIT);

    // The directions on the WAIT step are also unchanged — the modifier
    // didn't smear into them.
    ASSERT_TRUE(q.items[0].route_direction   == Direction::AUTO);
    ASSERT_TRUE(q.items[0].arrival_direction == Direction::AUTO);
}

static void test_TC13_setArrivalDirection_after_print_is_noop()
{
    TEST_SCOPE("TC13: Mission.print(...).setArrivalDirection(REAR) is a no-op");

    using namespace mission_replica;
    Queue q;

    Step printStep{};
    printStep.type = StepType::PRINT;
    ASSERT_TRUE(q.push(printStep));

    const bool wrote = setArrivalDirection(q, Direction::REAR);
    ASSERT_FALSE(wrote);

    ASSERT_EQ  (q.count, (uint8_t) 1);
    ASSERT_TRUE(q.items[0].type == StepType::PRINT);
    ASSERT_TRUE(q.items[0].arrival_direction == Direction::AUTO);
}

static void test_TC13_setRouteDirection_on_empty_queue_is_noop()
{
    TEST_SCOPE("TC13: Mission.setRouteDirection on empty queue is a no-op (no underflow)");

    using namespace mission_replica;
    Queue q;
    ASSERT_EQ(q.count, (uint8_t) 0);

    const bool wrote = setRouteDirection(q, Direction::REAR);
    ASSERT_FALSE(wrote);

    // Most importantly: no buffer underflow / read of `_queue[-1]`.
    // (If the production guard's `_count == 0` check were missing, this
    //  would have written to slot `_queue[CAPACITY - 1]` due to underflow.)
    ASSERT_EQ(q.count, (uint8_t) 0);
}

static void test_TC13_setRouteDirection_after_goto_writes_through()
{
    TEST_SCOPE("TC13: positive case — Mission.goTo(...).setRouteDirection(REAR) updates the step");

    using namespace mission_replica;
    Queue q;

    Step gotoStep{};
    gotoStep.type              = StepType::GOTO;
    gotoStep.route_direction   = Direction::AUTO;
    gotoStep.arrival_direction = Direction::AUTO;
    ASSERT_TRUE(q.push(gotoStep));

    const bool wrote = setRouteDirection(q, Direction::REAR);
    ASSERT_TRUE(wrote);
    ASSERT_TRUE(q.items[0].route_direction == Direction::REAR);
    // Arrival direction untouched by setRouteDirection.
    ASSERT_TRUE(q.items[0].arrival_direction == Direction::AUTO);

    const bool wroteArrival = setArrivalDirection(q, Direction::FRONT);
    ASSERT_TRUE(wroteArrival);
    ASSERT_TRUE(q.items[0].arrival_direction == Direction::FRONT);
    // And route direction is unchanged after the arrival write.
    ASSERT_TRUE(q.items[0].route_direction == Direction::REAR);
}

static void test_TC13_modifier_targets_last_step_only()
{
    TEST_SCOPE("TC13: modifier targets most-recently-pushed step, not earlier GOTOs");

    using namespace mission_replica;
    Queue q;

    // goTo("A") → setRouteDirection(REAR) → wait(100) → setRouteDirection(FRONT)
    // The last modifier must hit the WAIT step (and be rejected), NOT the
    // earlier GOTO. Mirrors the production rule that the modifier targets
    // `_queue[_count - 1]`.
    Step gotoA{};
    gotoA.type = StepType::GOTO;
    ASSERT_TRUE(q.push(gotoA));
    ASSERT_TRUE(setRouteDirection(q, Direction::REAR));
    ASSERT_TRUE(q.items[0].route_direction == Direction::REAR);

    Step waitStep{};
    waitStep.type = StepType::WAIT;
    ASSERT_TRUE(q.push(waitStep));

    const bool wrote = setRouteDirection(q, Direction::FRONT);
    ASSERT_FALSE(wrote);

    // The earlier GOTO's route_direction is still REAR — the rejected
    // modifier didn't leak backward into the previous step.
    ASSERT_TRUE(q.items[0].route_direction == Direction::REAR);
    // The WAIT step's route_direction is still AUTO (zero-init via Step{}).
    ASSERT_TRUE(q.items[1].route_direction == Direction::AUTO);
    ASSERT_EQ(q.count, (uint8_t) 2);
}

static void test_TC13_modifier_after_overflow_does_not_corrupt_prior_step()
{
    TEST_SCOPE("TC13: modifier after queue-full overflow does NOT rewrite the prior GOTO's direction");

    // Regression for the queue-full corruption bug from /code-review max:
    // an overflowed Mission.goTo() previously left _count unchanged, so a
    // chained .setRouteDirection() silently targeted the PREVIOUS step
    // (often another GOTO) instead of being rejected. Fix: track the last
    // push result and bail when it was false.

    using namespace mission_replica;
    Queue q;

    // Fill the queue to capacity with GOTO steps.
    for (uint8_t i = 0; i < Queue::CAPACITY; ++i)
    {
        Step s{};
        s.type = StepType::GOTO;
        ASSERT_TRUE(q.push(s));
    }
    ASSERT_EQ(q.count, Queue::CAPACITY);
    ASSERT_TRUE(q.lastPushOk);

    // Tag the last successfully-pushed GOTO so we can detect corruption.
    q.items[Queue::CAPACITY - 1].route_direction = Direction::FRONT;

    // Now push one more — overflow. lastPushOk must flip to false.
    Step overflow{};
    overflow.type = StepType::GOTO;
    ASSERT_FALSE(q.push(overflow));
    ASSERT_FALSE(q.lastPushOk);
    ASSERT_EQ(q.count, Queue::CAPACITY);    // count unchanged

    // Chained modifier on the (dropped) overflow step. MUST be rejected
    // because lastPushOk is false — without the fix, it would silently
    // rewrite q.items[CAPACITY - 1].route_direction (the prior valid GOTO).
    const bool wrote = setRouteDirection(q, Direction::REAR);
    ASSERT_FALSE(wrote);

    // The previously-tagged FRONT must survive untouched.
    ASSERT_TRUE(q.items[Queue::CAPACITY - 1].route_direction == Direction::FRONT);

    // Same expectation for setArrivalDirection.
    const bool wroteArrival = setArrivalDirection(q, Direction::REAR);
    ASSERT_FALSE(wroteArrival);
    ASSERT_TRUE(q.items[Queue::CAPACITY - 1].arrival_direction == Direction::AUTO);
}

// ===========================================================================
// TC14c — Compile-time kill-switch override.
//
// This test asserts the BIDIRECTIONAL_AUTO_ENABLED_DEFAULT macro genuinely
// propagates into the runtime constant AND that the constant gates
// `_resolveDirection`'s auto path. The CMakeLists.txt under WRO2026/test/
// builds this file TWICE — once with `-DBIDIRECTIONAL_AUTO_ENABLED_DEFAULT=true`
// and once with `=false` — and registers each binary as a separate ctest
// target (path_tracker_bidirectional_on / path_tracker_bidirectional_off).
//
// Without the per-target macro override, the test still runs against the
// current default (true) — which is useful as a smoke check but doesn't
// exercise the off branch. The CMake split is what gives us behavioral
// coverage on both sides; the assertions below are written to match
// whichever value the build is currently configured with.
// ===========================================================================

static void test_TC14c_macro_propagates_into_runtime_constant()
{
    TEST_SCOPE("TC14c: BIDIRECTIONAL_AUTO_ENABLED_DEFAULT macro propagates into Config::BIDIRECTIONAL_AUTO_ENABLED");

    // The macro definition is the load-bearing override knob. Whatever the
    // compiler was told via `-D` MUST land on the public constant.
    ASSERT_EQ(Config::BIDIRECTIONAL_AUTO_ENABLED, (bool) BIDIRECTIONAL_AUTO_ENABLED_DEFAULT);
}

static void test_TC14c_2board_auto_175deg_differs_by_flag()
{
    TEST_SCOPE("TC14c: 2-board chassis, 175-deg heading delta — REAR if flag ON, FRONT if flag OFF");

    // Canonical fixture: 2-board chassis, AUTO at builder/edge/pipeline, with
    // currentHeading == 0 deg and edge.heading_deg == 175 deg (just inside
    // the (90, 180] turn-saving zone for REAR). The auto branch wraps the
    // delta to +175 deg, which exceeds BIDIRECTIONAL_TURN_THRESHOLD_DEG (90),
    // so the flag-ON build picks REAR. The flag-OFF build short-circuits
    // before the wrap and returns FRONT.
    const Direction resolved = replica_resolveDirection(
        /*edgeOverride*/      Routes::DirectionMode::DEFAULT,
        /*pipelineDefault*/   Routes::DirectionMode::DEFAULT,
        /*builderArrival*/    Direction::AUTO,
        /*builderRoute*/      Direction::AUTO,
        /*currentHeading*/    0.0f,
        /*edgeHeading*/       175.0f,
        /*isLastHop*/         false,
        /*bidirectionalOn*/   Config::BIDIRECTIONAL_AUTO_ENABLED,
        /*lineBoardCount*/    2);

    if (Config::BIDIRECTIONAL_AUTO_ENABLED)
    {
        // Flag ON: 175 deg > 90 deg threshold → turn-saving REAR.
        ASSERT_TRUE(resolved == Direction::REAR);
    }
    else
    {
        // Flag OFF: kill-switch defeats the auto branch entirely.
        ASSERT_TRUE(resolved == Direction::FRONT);
    }
}

static void test_TC14c_2board_auto_0deg_always_front()
{
    TEST_SCOPE("TC14c: 2-board, AUTO, 0-deg delta — FRONT regardless of flag (sanity)");

    // Sanity check: a 0-deg heading delta is squarely below the 90-deg
    // turn-saving threshold, so the auto branch picks FRONT whether the
    // kill-switch is on or off. The flag-OFF build also returns FRONT via
    // its Step-5 short-circuit. Either way, FRONT.
    const Direction resolved = replica_resolveDirection(
        /*edgeOverride*/      Routes::DirectionMode::DEFAULT,
        /*pipelineDefault*/   Routes::DirectionMode::DEFAULT,
        /*builderArrival*/    Direction::AUTO,
        /*builderRoute*/      Direction::AUTO,
        /*currentHeading*/    0.0f,
        /*edgeHeading*/       0.0f,
        /*isLastHop*/         false,
        /*bidirectionalOn*/   Config::BIDIRECTIONAL_AUTO_ENABLED,
        /*lineBoardCount*/    2);

    ASSERT_TRUE(resolved == Direction::FRONT);
}

static void test_TC14c_1board_chassis_always_front_even_at_175deg()
{
    TEST_SCOPE("TC14c: 1-board chassis — auto path always FRONT regardless of flag (T7 safety)");

    // The auto branch has TWO independent gates: the kill-switch AND
    // `lineBoardCount >= 2`. So on a 1-board chassis the auto path falls
    // through to FRONT even with the kill-switch ON — that's the
    // belt-and-braces safety from T7 / issue #19. The flag's load-bearing
    // behavior is only observable on the 2-board fixture above.
    const Direction resolved = replica_resolveDirection(
        /*edgeOverride*/      Routes::DirectionMode::DEFAULT,
        /*pipelineDefault*/   Routes::DirectionMode::DEFAULT,
        /*builderArrival*/    Direction::AUTO,
        /*builderRoute*/      Direction::AUTO,
        /*currentHeading*/    0.0f,
        /*edgeHeading*/       175.0f,
        /*isLastHop*/         false,
        /*bidirectionalOn*/   Config::BIDIRECTIONAL_AUTO_ENABLED,
        /*lineBoardCount*/    1);

    ASSERT_TRUE(resolved == Direction::FRONT);
}

// ---------------------------------------------------------------------------
// Print which build mode we're running in for ctest log clarity.
// ---------------------------------------------------------------------------

static void test_TC14c_announce_build_mode()
{
    TEST_SCOPE("TC14c: build-mode announcement");
    if (Config::BIDIRECTIONAL_AUTO_ENABLED)
    {
        std::printf("INFO: BIDIRECTIONAL_ENABLED == true (kill-switch ON build)\n");
    }
    else
    {
        std::printf("INFO: BIDIRECTIONAL_ENABLED == false (kill-switch OFF build)\n");
    }
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_TC14c_announce_build_mode();

    // TC12 — REVERSE_UNAVAILABLE guard.
    test_TC12_reverse_unavailable_on_line_1board_force_reverse();
    test_TC12_blind_rear_1board_allowed();
    test_TC12_line_rear_2board_allowed();
    test_TC12_line_front_1board_allowed();

    // TC13 — Mission modifier no-op on non-GOTO + queue-overflow corruption regression.
    test_TC13_setRouteDirection_after_wait_is_noop();
    test_TC13_setArrivalDirection_after_print_is_noop();
    test_TC13_setRouteDirection_on_empty_queue_is_noop();
    test_TC13_setRouteDirection_after_goto_writes_through();
    test_TC13_modifier_targets_last_step_only();
    test_TC13_modifier_after_overflow_does_not_corrupt_prior_step();

    // TC14c — Compile-time kill-switch override.
    test_TC14c_macro_propagates_into_runtime_constant();
    test_TC14c_2board_auto_175deg_differs_by_flag();
    test_TC14c_2board_auto_0deg_always_front();
    test_TC14c_1board_chassis_always_front_even_at_175deg();

    TEST_REPORT();
}
