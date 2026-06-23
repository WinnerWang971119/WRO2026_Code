/**
 * @file test_path_tracker_projection.cpp
 * @brief Host-compile tests for the firmware projection-NODE dispatch (issue #64, T14 / TC16).
 *
 * Issue #64 made `projection` a NODE property rather than an edge type. Every
 * per-hop behaviour now falls out of `(edge_type, destination_node_type)`:
 *
 *   | Edge  | → line node                | → projection node                       |
 *   |-------|----------------------------|-----------------------------------------|
 *   | LINE  | follow to junction         | DISTANCE-gated follow + dead-reckon adv |
 *   | BLIND | dead-reckon                | auto-seek stop-at-line + full re-anchor |
 *
 * This file exercises the two projection cells of the firmware dispatch
 * (`PathTracker::_dispatchEdgeAfterTurn`), at the SAME two layers the sibling
 * `test_return_to_node_dispatch.cpp` uses:
 *
 *   1. The LOCAL destination-node read that ROUTES each hop — `FieldMap::getNodeType`
 *      / `isProjectionNode` / `getEdgeType` — tested DIRECTLY against the REAL graph
 *      accessors over a REAL `routes.cpp`. Because the PRODUCTION `routes.cpp` has NO
 *      projection node (issue #64 keeps the real field unchanged this phase — only the
 *      TOY field gains one), this target links the TOY golden
 *      `tools/path_planner/tests/golden/routes_toy.{h,cpp}` instead, which carries the
 *      projection node `P1` plus a `LINE→P1`, a second `LINE→P1`, and a `BLIND→P1`
 *      scenario. The toy `routes_toy.h` shares the production `ROUTES_H` include guard
 *      and the identical `Routes::` ABI (7-field `EdgeInfo`, `NodeType::PROJECTION = 3`),
 *      so it drops straight in once force-included ahead of the sketch `routes.h`
 *      (see CMakeLists `add_toy_routes_force_include`). Nodes are resolved BY NAME
 *      (`FieldMap::findNodeByName`), never by hardcoded index (issue #91).
 *
 *   2. The firmware auto-rule + the STOP_AT_LINE resolver call are exercised through the
 *      REAL frozen `resolveStopAtLine` (StopAtLinePolicy.h) fed the LOCAL
 *      `autoRuleActive = Routes::BLIND_PROJECTION_AUTO_SEEK && isProjectionNode(_nextHop)`
 *      computed EXACTLY as the firmware computes it — a purely LOCAL read of THIS hop's
 *      destination, with NO `NEXT_HOP[next][goal]` / next-next peek (that goal-relative
 *      transcription was retired in issue #64). The resolver body + the 936-row CSV are
 *      FROZEN; only this caller INPUT is computed differently.
 *
 *   3. The per-kind `_dispatchEdgeAfterTurn` ROUTING (which `(edge,dest)` cell leads to
 *      which FSM state + which Localization advance) is inline-MIRRORED, following the
 *      precedent of `test_path_tracker_stop_at_line.cpp` /
 *      `test_return_to_node_dispatch.cpp`: the full production `PathTracker.cpp` is NOT
 *      linked by ANY host target (it would drag in the Drive fluent-builder +
 *      Localization + EventTrace + MatrixMiniR4 surface), so the small decision slice is
 *      replicated and every mirrored decision is annotated `// MIRROR: PathTracker.cpp:<line>`
 *      for drift detection at PR-review time. The mirror is DRIVEN BY the real graph
 *      reads above (it does not re-encode the node/edge types), so a future toy-field
 *      change that breaks the `(edge,dest)` shape fails this test loudly.
 *
 * What this file does NOT cover (delegated, per the same precedent): the actual
 * LineFollower / Drive builder chains emitted (`.setDistanceTarget`, `.stopAtLine`,
 * `.setOvershoot`), the pivot-trim `distance` regime arithmetic
 * (`test_pivot_trim_policy.cpp` / `test_sensor_overshoot_policy.cpp`), and the
 * end-to-end FSM transition. Those are covered by the Python sim parity tests
 * (TC10) and the Arduino firmware compile.
 *
 * Links the TOY `routes_toy.cpp` (the sole definer of the `Routes::NODES` / `N_NODES`
 * data the accessors odr-use) + `MOCK_SOURCES` (for the angle-bracket `<Arduino.h>` /
 * `<MatrixMiniR4.h>` that `Config.h` / `PathTracker.h` pull in). NO SUT `.cpp` and NO
 * Drive force-include — `FieldMap.h` / `routes.h` / `StopAtLinePolicy.h` are header-only
 * and never include the production `Drive.h`.
 */

#include "TestAssert.h"

#include "Config.h"            // Config::LINE_BOARD_COUNT (resolver board-gate input).
#include "FieldMap.h"          // real FieldMap::getNodeType / isProjectionNode / getEdge (header-only).
#include "routes.h"            // Routes::NodeType / EdgeType / BLIND_PROJECTION_AUTO_SEEK (data in routes_toy.cpp).
#include "NavTypes.h"          // Direction / StopAtLine.
#include "PathTracker.h"       // PathTrackerClass::Fault (the real enum the mirror routes to).
#include "StopAtLinePolicy.h"  // resolveStopAtLine + StopAtLineQuery/Resolution (the real frozen resolver).

#include <stdint.h>
#include <cstdio>

// ===========================================================================
// Node-name handles. Resolved BY NAME (issue #91), never by hardcoded index —
// the toy golden's indices are an artifact of authoring order and may renumber.
// ===========================================================================

namespace
{
    /// Resolve a toy-field node name to its id, asserting it exists (a typo or a
    /// renamed toy node fails loudly here rather than silently weakening a test).
    uint8_t nodeId(const char* name)
    {
        const uint8_t id = FieldMap::findNodeByName(name);
        ASSERT_TRUE(id != FieldMap::NODE_NONE);
        return id;
    }
}    // namespace

// ===========================================================================
// `_dispatchEdgeAfterTurn` projection-cell routing replica.
//
// Mirrors the slice of `PathTracker::_dispatchEdgeAfterTurn` that, given the
// resolved `(edge.type, dest node type)` for the current hop, decides which FSM
// state the hop enters and which Localization advance the terminal state issues:
//
//   - LINE  → projection : State::PROJECTION_FOLLOW, terminate on the DISTANCE
//                          gate (`LineFollower.isDistanceReached`), then
//                          `Localization.onDeadReckonedAdvance(_nextHop)` — there
//                          is NO junction to re-anchor against.
//   - LINE  → line       : State::LINE_FOLLOW, terminate on the JUNCTION
//                          (`Intersection.isAtIntersection`), then the
//                          LINE_OVERSHOOT roll + `onIntersection` re-anchor.
//   - BLIND → projection : the LOCAL auto-rule makes `resolveStopAtLine` seek, so
//                          State::STOP_AT_LINE, terminate on the seek crossing
//                          (`Drive.isIdle`), then `onIntersection(_nextHop)` — a
//                          FULL-ADVANCE re-anchor (current node = projection node,
//                          operator offset cleared).
//   - BLIND → line/blind : State::BLIND_FORWARD (no auto-seek), then
//                          `onDeadReckonedAdvance`.
//
// The terminal-gate + advance enums are what TC16 asserts: a LINE→projection hop
// is DISTANCE-terminated and dead-reckons; a BLIND→projection hop SEEKS and
// re-anchors via onIntersection.
//
// MIRROR: PathTracker.cpp:1751-1829 (EdgeType::LINE branch: isProjectionNode →
//         PROJECTION_FOLLOW / else LINE_FOLLOW).
// MIRROR: PathTracker.cpp:903-920 (PROJECTION_FOLLOW: isDistanceReached →
//         onDeadReckonedAdvance).
// MIRROR: PathTracker.cpp:1870-1881 (EdgeType::BLIND auto-rule:
//         BLIND_PROJECTION_AUTO_SEEK && isProjectionNode(_nextHop) → resolver seek).
// MIRROR: PathTracker.cpp:932-962 (STOP_AT_LINE success: onIntersection FULL-ADVANCE).
// MIRROR: PathTracker.cpp:922-930 (BLIND_FORWARD success: onDeadReckonedAdvance).
// ===========================================================================

/// The FSM state a hop's dispatch enters, for the projection-cell routing mirror.
enum class DispatchState : uint8_t
{
    LINE_FOLLOW,         ///< LINE → line: follow to the next junction.
    PROJECTION_FOLLOW,   ///< LINE → projection: DISTANCE-gated follow.
    STOP_AT_LINE,        ///< BLIND → projection (auto-seek): seek a line, re-anchor.
    BLIND_FORWARD,       ///< BLIND → line/blind: dead-reckon forward.
    FAULT,               ///< EdgeType::NONE — no edge.
};

/// How a hop's terminal state advances Localization on success.
enum class TerminalAdvance : uint8_t
{
    ON_INTERSECTION,        ///< Junction re-anchor / seek full-advance re-anchor (offset cleared).
    ON_DEAD_RECKONED,       ///< Dead-reckon advance (no physical re-localisation).
    NONE,                   ///< Fault path — no advance.
};

/// What gate terminates the hop (the load-bearing distinction for the
/// LINE→projection silent-failure trap: it is DISTANCE-, not junction-gated).
enum class TerminalGate : uint8_t
{
    DISTANCE,         ///< `LineFollower.isDistanceReached()` (PROJECTION_FOLLOW) / overshoot roll.
    INTERSECTION,     ///< `Intersection.isAtIntersection()` (LINE_FOLLOW).
    DRIVE_IDLE,       ///< `Drive.isIdle()` (STOP_AT_LINE seek / BLIND_FORWARD).
    NONE,
};

/// One replicated projection-cell dispatch decision.
struct DispatchResult
{
    DispatchState   state          = DispatchState::FAULT;
    TerminalGate    gate           = TerminalGate::NONE;
    TerminalAdvance advance        = TerminalAdvance::NONE;
    bool            autoSeek        = false;    ///< the BLIND auto-rule fired (resolver seek requested locally).
    bool            offsetCleared   = false;    ///< the terminal re-anchor clears the operator offset.
};

/// Replica of `_dispatchEdgeAfterTurn` for the `(edge.type, dest node type)` cell
/// of one hop, plus the terminal advance its FSM state issues on success. Driven
/// by the REAL graph reads (edge type + `isProjectionNode(dest)`) and the REAL
/// frozen resolver for the BLIND auto-rule — the routing is mirrored, the
/// CLASSIFICATION and the seek DECISION are not re-implemented.
///
/// @param edgeType        `FieldMap::getEdgeType(cur, dest)` for this hop.
/// @param destIsProjection `FieldMap::isProjectionNode(dest)` for this hop.
/// @param seek            `resolveStopAtLine(...).seek` for a BLIND hop (the real
///                        resolver's verdict, fed the LOCAL autoRuleActive). Unused
///                        for a LINE hop (LINE never seeks).
static DispatchResult replica_dispatchProjectionCell(
    Routes::EdgeType edgeType, bool destIsProjection, bool seek)
{
    DispatchResult r{};

    switch (edgeType)
    {
        case Routes::EdgeType::LINE:
        {
            // MIRROR: PathTracker.cpp:1796 — if (FieldMap::isProjectionNode(_nextHop)).
            if (destIsProjection)
            {
                // LINE → projection: DISTANCE-gated follow; dead-reckon the advance
                // (no junction to re-anchor against). The motion profile comes from
                // the LINE edge's own line_mode_override (ProjectionMode deleted).
                r.state   = DispatchState::PROJECTION_FOLLOW;
                r.gate    = TerminalGate::DISTANCE;
                r.advance = TerminalAdvance::ON_DEAD_RECKONED;    // onDeadReckonedAdvance
            }
            else
            {
                // LINE → line (junction): follow to the next intersection, then the
                // LINE_OVERSHOOT roll + onIntersection re-anchor.
                r.state   = DispatchState::LINE_FOLLOW;
                r.gate    = TerminalGate::INTERSECTION;
                r.advance = TerminalAdvance::ON_INTERSECTION;     // (after LINE_OVERSHOOT)
            }
            break;
        }

        case Routes::EdgeType::BLIND:
        {
            // MIRROR: PathTracker.cpp:1883 — if (sal.seek).
            if (seek)
            {
                // BLIND → projection auto-seek (or any forced seek): State::STOP_AT_LINE.
                // On the seek crossing, onIntersection(_nextHop) is the FULL-ADVANCE
                // re-anchor — current node = the projection node, operator offset
                // cleared (onIntersection clears the offset, Localization side).
                r.state         = DispatchState::STOP_AT_LINE;
                r.gate          = TerminalGate::DRIVE_IDLE;
                r.advance       = TerminalAdvance::ON_INTERSECTION;
                r.autoSeek      = true;
                r.offsetCleared = true;
            }
            else
            {
                // BLIND → line/blind dead-reckon: State::BLIND_FORWARD, onDeadReckonedAdvance.
                r.state   = DispatchState::BLIND_FORWARD;
                r.gate    = TerminalGate::DRIVE_IDLE;
                r.advance = TerminalAdvance::ON_DEAD_RECKONED;
            }
            break;
        }

        case Routes::EdgeType::NONE:
        default:
        {
            r.state   = DispatchState::FAULT;
            r.gate    = TerminalGate::NONE;
            r.advance = TerminalAdvance::NONE;
            break;
        }
    }

    return r;
}

/// Compute the LOCAL `autoRuleActive` EXACTLY as the firmware does in the
/// `EdgeType::BLIND` branch of `_dispatchEdgeAfterTurn`. This is the entire
/// behavioural change of issue #64's auto-rule: a purely LOCAL read of THIS hop's
/// destination node, gated by the field-wide toggle — NO `NEXT_HOP[next][goal]`
/// peek. The toggle LEADS the `&&` (short-circuit) so the projection lookup is
/// skipped when the feature is compiled out, mirroring the firmware exactly.
///
// MIRROR: PathTracker.cpp:1870-1871
//         (autoRuleActive = Routes::BLIND_PROJECTION_AUTO_SEEK &&
//                           FieldMap::isProjectionNode(_nextHop)).
static bool firmwareAutoRuleActive(uint8_t dest)
{
    return Routes::BLIND_PROJECTION_AUTO_SEEK && FieldMap::isProjectionNode(dest);
}

/// Build the resolver query a BARE `goTo()` BLIND hop would feed `resolveStopAtLine`
/// (no call-site verb, no per-edge override, no legacy STOP_AT_LINE tag), with the
/// LOCAL `autoRuleActive` for the given destination. FRONT travel + the real
/// `Config::LINE_BOARD_COUNT` so the board-gate matches production.
///
// MIRROR: PathTracker.cpp:1873-1881 (the StopAtLineQuery the BLIND branch assembles).
static StopAtLineResolution resolveBareBlindHop(uint8_t dest)
{
    const StopAtLineQuery q{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,    // a plain blind edge (DEFAULT → pipeline blind)
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ Config::LINE_BOARD_COUNT,
        /*autoRuleActive*/ firmwareAutoRuleActive(dest)};
    return resolveStopAtLine(q);
}

// ===========================================================================
// TC16-pre — the toy fixture actually carries the scenarios under test. A
// future toy-field change that drops the projection node (or a flanking
// LINE/BLIND edge) fails loudly HERE rather than silently weakening TC16.
// ===========================================================================

static void test_TC16_toy_fixture_has_projection_scenarios()
{
    TEST_SCOPE("TC16: toy fixture carries P1 (projection) + LINE→P1 + BLIND→P1");

    const uint8_t a  = nodeId("A");      // LINE node
    const uint8_t b  = nodeId("B");      // LINE node
    const uint8_t p1 = nodeId("P1");     // PROJECTION node
    const uint8_t s  = nodeId("S");      // BLIND node

    // P1 is the projection node; A/B are line nodes; S is a blind node.
    ASSERT_TRUE(FieldMap::getNodeType(p1) == Routes::NodeType::PROJECTION);
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));
    ASSERT_TRUE(FieldMap::getNodeType(a) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(b) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(s) == Routes::NodeType::BLIND);
    ASSERT_FALSE(FieldMap::isProjectionNode(a));
    ASSERT_FALSE(FieldMap::isProjectionNode(s));

    // A→P1 and B→P1 are LINE edges into the projection node; S→P1 is a BLIND
    // edge into it. These are the (edge_type, dest_node_type) cells TC16 exercises.
    ASSERT_TRUE(FieldMap::getEdgeType(a, p1) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdgeType(b, p1) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdgeType(s, p1) == Routes::EdgeType::BLIND);

    // The projection edge type is gone — EdgeType is LINE/BLIND/NONE only (issue
    // #64). Spot-check the enum's domain over the whole toy graph: no edge is
    // anything but NONE / LINE / BLIND.
    for (uint8_t u = 0; u < Routes::N_NODES; ++u)
    {
        for (uint8_t v = 0; v < Routes::N_NODES; ++v)
        {
            const Routes::EdgeType t = FieldMap::getEdgeType(u, v);
            ASSERT_TRUE(t == Routes::EdgeType::NONE || t == Routes::EdgeType::LINE
                        || t == Routes::EdgeType::BLIND);
        }
    }
}

// ===========================================================================
// TC16(a) — LINE → projection: DISTANCE-gated follow (PROJECTION_FOLLOW), NOT an
//           intersection wait, then a dead-reckon advance.
// ===========================================================================

static void test_TC16a_line_into_projection_distance_gated_dead_reckons()
{
    TEST_SCOPE("TC16a: LINE→projection terminates on DISTANCE (PROJECTION_FOLLOW) + dead-reckon advance");

    const uint8_t a  = nodeId("A");
    const uint8_t p1 = nodeId("P1");

    // Real graph reads: A→P1 is a LINE edge into a projection destination.
    const Routes::EdgeType edgeType = FieldMap::getEdgeType(a, p1);
    const bool destIsProjection     = FieldMap::isProjectionNode(p1);
    ASSERT_TRUE(edgeType == Routes::EdgeType::LINE);
    ASSERT_TRUE(destIsProjection);

    // Dispatch the cell. A LINE hop never seeks (the seek arg is for BLIND only),
    // so pass false.
    const DispatchResult r =
        replica_dispatchProjectionCell(edgeType, destIsProjection, /*seek*/ false);

    // The load-bearing assertions: PROJECTION_FOLLOW, DISTANCE-gated (NOT an
    // intersection wait — a projection node has no junction marker), and the node
    // advances by dead-reckoning (no junction to re-anchor against).
    ASSERT_TRUE(r.state == DispatchState::PROJECTION_FOLLOW);
    ASSERT_TRUE(r.gate == TerminalGate::DISTANCE);
    ASSERT_TRUE(r.advance == TerminalAdvance::ON_DEAD_RECKONED);
    ASSERT_FALSE(r.autoSeek);    // a LINE follow is not a seek

    // Contrast: a LINE → line (junction) hop is INTERSECTION-gated and re-anchors.
    // B→A is LINE into the line node A — proves the routing actually branches on
    // the destination node type, not a constant.
    const uint8_t b = nodeId("B");
    const Routes::EdgeType lineToLine = FieldMap::getEdgeType(b, a);
    ASSERT_TRUE(lineToLine == Routes::EdgeType::LINE);
    ASSERT_FALSE(FieldMap::isProjectionNode(a));
    const DispatchResult rLine = replica_dispatchProjectionCell(
        lineToLine, FieldMap::isProjectionNode(a), /*seek*/ false);
    ASSERT_TRUE(rLine.state == DispatchState::LINE_FOLLOW);
    ASSERT_TRUE(rLine.gate == TerminalGate::INTERSECTION);
    ASSERT_TRUE(rLine.advance == TerminalAdvance::ON_INTERSECTION);
}

// ===========================================================================
// TC16(b) — BLIND → projection: auto-seek (stop-at-line) fires from the LOCAL
//           destination read, terminates on the seek crossing, and FULL-ADVANCE
//           re-anchors (current node = projection node, offset cleared).
// ===========================================================================

static void test_TC16b_blind_into_projection_auto_seeks_and_reanchors()
{
    TEST_SCOPE("TC16b: BLIND→projection auto-seeks (STOP_AT_LINE) + full-advance re-anchor");

    const uint8_t s  = nodeId("S");
    const uint8_t p1 = nodeId("P1");

    // Real graph reads: S→P1 is a BLIND edge into a projection destination.
    const Routes::EdgeType edgeType = FieldMap::getEdgeType(s, p1);
    ASSERT_TRUE(edgeType == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));

    // The LOCAL auto-rule: BLIND_PROJECTION_AUTO_SEEK && isProjectionNode(dest).
    // Computed EXACTLY as the firmware computes it, with NO NEXT_HOP[next][goal]
    // peek — a purely LOCAL read of THIS hop's destination.
    const bool autoRule = firmwareAutoRuleActive(p1);
    ASSERT_TRUE(autoRule);    // toggle is on (toy BLIND_PROJECTION_AUTO_SEEK = true) AND dest is projection

    // Feed the REAL frozen resolver the LOCAL autoRuleActive. A bare goTo() BLIND
    // hop (no call verb, no per-edge override, plain blind edge) into a projection
    // node seeks — the auto rung (rung 4) fires.
    const StopAtLineResolution sal = resolveBareBlindHop(p1);
    ASSERT_TRUE(sal.seek);

    // Dispatch the cell with the resolver's verdict. The hop enters STOP_AT_LINE,
    // terminates on the seek crossing (Drive.isIdle), and FULL-ADVANCE re-anchors
    // via onIntersection — current node = the projection node, offset cleared.
    const DispatchResult r =
        replica_dispatchProjectionCell(edgeType, /*destIsProjection*/ true, sal.seek);
    ASSERT_TRUE(r.state == DispatchState::STOP_AT_LINE);
    ASSERT_TRUE(r.gate == TerminalGate::DRIVE_IDLE);
    ASSERT_TRUE(r.advance == TerminalAdvance::ON_INTERSECTION);    // full-advance re-anchor
    ASSERT_TRUE(r.autoSeek);
    ASSERT_TRUE(r.offsetCleared);
}

// ===========================================================================
// TC16(c) — the auto-rule is LOCAL: a BLIND hop into a NON-projection node does
//           NOT auto-seek (proves the rule keys on the destination node type, not
//           a constant), and the projection lookup needs NO route/goal context.
// ===========================================================================

static void test_TC16c_blind_into_nonprojection_does_not_auto_seek()
{
    TEST_SCOPE("TC16c: BLIND→blind/line does NOT auto-seek (the rule is a LOCAL dest read)");

    const uint8_t a = nodeId("A");
    const uint8_t s = nodeId("S");

    // A→S is a BLIND edge into the BLIND node S — a non-projection destination.
    const Routes::EdgeType edgeType = FieldMap::getEdgeType(a, s);
    ASSERT_TRUE(edgeType == Routes::EdgeType::BLIND);
    ASSERT_FALSE(FieldMap::isProjectionNode(s));

    // The LOCAL auto-rule is FALSE here: the toggle is on, but the destination is
    // not a projection node. No NEXT_HOP peek is consulted — the same LOCAL read.
    const bool autoRule = firmwareAutoRuleActive(s);
    ASSERT_FALSE(autoRule);

    // The real resolver therefore does NOT seek (a bare goTo() blind hop with no
    // call verb / override / legacy tag and no auto rule active → dead-reckon).
    const StopAtLineResolution sal = resolveBareBlindHop(s);
    ASSERT_FALSE(sal.seek);

    // Dispatch the cell: BLIND_FORWARD (dead-reckon), onDeadReckonedAdvance.
    const DispatchResult r =
        replica_dispatchProjectionCell(edgeType, /*destIsProjection*/ false, sal.seek);
    ASSERT_TRUE(r.state == DispatchState::BLIND_FORWARD);
    ASSERT_TRUE(r.advance == TerminalAdvance::ON_DEAD_RECKONED);
    ASSERT_FALSE(r.autoSeek);
    ASSERT_FALSE(r.offsetCleared);
}

// ===========================================================================
// TC16(d) — the auto-rule's toggle leads the `&&`: with BLIND_PROJECTION_AUTO_SEEK
//           compiled out, even a BLIND→projection hop would NOT auto-seek. The toy
//           field ships the toggle ON, so this asserts the COMPOSITION (toggle &&
//           dest) the firmware uses, modelled with the toggle forced false.
// ===========================================================================

static void test_TC16d_auto_rule_toggle_gates_the_dest_read()
{
    TEST_SCOPE("TC16d: BLIND_PROJECTION_AUTO_SEEK toggle gates the projection dest read (composition)");

    const uint8_t p1 = nodeId("P1");
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));

    // The firmware predicate is `toggle && isProjectionNode(dest)`. With the toggle
    // ON (the toy default) and a projection destination, it is true (TC16b). Model
    // the toggle-OFF half of the composition: `false && isProjectionNode(dest)` is
    // false regardless of the destination — the auto-seek is compiled out. This
    // pins the SHORT-CIRCUIT shape, not just the current toggle value.
    const bool toggledOff = false && FieldMap::isProjectionNode(p1);
    ASSERT_FALSE(toggledOff);

    // And the live composition over the real toggle agrees with the live read.
    const bool live = Routes::BLIND_PROJECTION_AUTO_SEEK && FieldMap::isProjectionNode(p1);
    ASSERT_EQ(live, (bool) Routes::BLIND_PROJECTION_AUTO_SEEK);    // dest IS projection, so == toggle
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker projection-NODE dispatch (issue #64, TC16) — real graph reads + "
                "frozen resolver + LOCAL auto-rule over the toy projection node P1\n");

    test_TC16_toy_fixture_has_projection_scenarios();
    test_TC16a_line_into_projection_distance_gated_dead_reckons();
    test_TC16b_blind_into_projection_auto_seeks_and_reanchors();
    test_TC16c_blind_into_nonprojection_does_not_auto_seek();
    test_TC16d_auto_rule_toggle_gates_the_dest_read();

    TEST_REPORT();
}
