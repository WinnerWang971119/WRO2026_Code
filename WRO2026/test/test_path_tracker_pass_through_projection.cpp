/**
 * @file test_path_tracker_pass_through_projection.cpp
 * @brief Host-compile tests for the firmware `line(A)-projection(B)-line(C)`
 *        pass-through (issue #64, T19 / T21, TC11 / TC12 / TC13).
 *
 * Issue #64's pass-through (T19) collapses a `line(A)-projection(B)-line(C)`
 * triple routed PAST the marker-less projection node `B` into ONE continuous
 * A->C line-follow: the A->B LINE-into-projection hop keeps the follower live
 * across `B` (dead-reckon advance MID-FOLLOW, no stop) and re-anchors at the
 * real junction `C`. The TRIGGER is the A->B DISTANCE gate, NOT
 * `Intersection.isAtIntersection()` -- a projection node has no junction marker,
 * so an intersection NEVER fires at `B` (see the PROJECTION_FOLLOW state +
 * `_projPassThrough` branch in PathTracker.cpp).
 *
 * This file exercises the firmware's pass-through DECISION
 * (`PathTracker::_evaluateProjectionPassThrough`) at the SAME two layers the
 * sibling `test_path_tracker_projection.cpp` (TC16) uses:
 *
 *   1. The pass-through gate's per-condition reads -- the next-next-hop lookahead
 *      (`getNextHop`), the outbound flank edge type (`getEdge(...).type`), the
 *      next-next node type (`getNodeType`), and the collinearity / same-direction
 *      heading check -- are driven by the REAL graph accessors over a REAL
 *      `routes.cpp`. Because the PRODUCTION `routes.cpp` has NO projection node
 *      (issue #64 keeps the real field unchanged this phase), this target links
 *      the TOY golden `tools/path_planner/tests/golden/routes_toy.{h,cpp}`
 *      instead, which carries the projection node `P1` collinear between the LINE
 *      nodes `A` and `B` on `y = 10` (the `A-P1-B` pass-through triple) plus a
 *      `BLIND` `S-P1` flank. Nodes are resolved BY NAME (`findNodeByName`), never
 *      by hardcoded index (issue #91).
 *
 *   2. The collinearity / direction half of the gate is exercised through the
 *      REAL frozen `resolveDirection` (DirectionPolicy.h) + the REAL `wrapDeltaDeg`
 *      (NavTypes.h) -- the SAME helpers `_evaluateProjectionPassThrough` calls --
 *      so the heading math is not re-implemented, only the surrounding routing
 *      lookahead is mirrored. The mirror is annotated `// MIRROR: PathTracker.cpp:<line>`
 *      for drift detection at PR-review time (the full `PathTracker.cpp` is NOT
 *      host-linked by ANY target -- it would drag in the Drive fluent-builder +
 *      Localization + EventTrace + MatrixMiniR4 surface).
 *
 * CRITICAL fixture note (T21 / the plan's TOY-field warning): the toy field
 * deliberately keeps a DIRECT `A-B` LINE edge (190 cm) alongside the `A-P1-B`
 * triple (45 + 145 = 190 cm, equal cost), so the toy `NEXT_HOP` may pick the
 * direct edge and BYPASS `P1` for a plain `goTo(B)`. These tests therefore drive
 * the gate over the EXPLICIT `A -> P1 -> B` hop sequence (the same way TC16
 * exercises specific hops), so the pass-through path is genuinely reached. The
 * gate's own next-next lookahead (`getNextHop(P1, B)`) is then asserted to return
 * the real onward hop -- a route that never reached `P1` would not satisfy the
 * gate's reads and the test would fail loudly.
 *
 * Three cases (the plan's TC11 / TC12 / TC13):
 *   - TC11: `line(A)-projection(P1)-line(B)` routed PAST P1 -> the gate FIRES
 *           (`_projPassThrough == true`): non-goal projection node, both flanks
 *           LINE, B is a LINE junction, A->P1->B collinear + same direction.
 *   - TC12: `goTo(P1)` where P1 is the goal -> the gate does NOT fire (the
 *           not-final guard rejects a goal-is-P1 route) -> a normal DISTANCE stop.
 *   - TC13: a BLIND-flanked projection node -> the gate does NOT fire (the
 *           outbound flank is a BLIND edge) -> a normal stop.
 *
 * What this file does NOT cover (delegated, per the same precedent): the actual
 * LineFollower brake-on-gate arming, the `onDeadReckonedAdvance` Localization
 * call, and the end-to-end FSM transition across `B`. Those are covered by the
 * Python sim parity tests (`test_pass_through_projection.py`, TC11/12/13) and the
 * Arduino firmware compile.
 *
 * Links the TOY `routes_toy.cpp` (the sole definer of the toy `Routes::NODES` /
 * `NEXT_HOP` / `EDGES` data the accessors odr-use) + `MOCK_SOURCES` (for the
 * angle-bracket `<Arduino.h>` / `<MatrixMiniR4.h>` that `Config.h` / `PathTracker.h`
 * pull in). NO SUT `.cpp` and NO Drive force-include -- `FieldMap.h` / `routes.h`
 * / `DirectionPolicy.h` / `NavTypes.h` are header-only and never include the
 * production `Drive.h`.
 */

#include "TestAssert.h"

#include "Config.h"            // Config::NAV_TURN_TOLERANCE_DEG / BIDIRECTIONAL_AUTO_ENABLED / LINE_BOARD_COUNT / PATH_PASS_THROUGH_ENABLED.
#include "FieldMap.h"          // real FieldMap::getNextHop / getEdge / getNodeType / isProjectionNode (header-only).
#include "routes.h"            // Routes::NodeType / EdgeType / PIPELINE_CONFIG (data in routes_toy.cpp).
#include "NavTypes.h"          // Direction / wrapDeltaDeg.
#include "DirectionPolicy.h"   // resolveDirection (the real frozen direction resolver).

#include <math.h>
#include <stdint.h>
#include <cstdio>

// ===========================================================================
// Node-name handles. Resolved BY NAME (issue #91), never by hardcoded index.
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

    /// The body heading the chassis arrives facing on a hop, given the resolved
    /// travel direction + the edge heading. FRONT -> the edge heading; REAR ->
    /// the edge heading rotated 180 deg (the rear board leads).
    ///
    // MIRROR: PathTracker.cpp:1615-1620 (bodyTarget = edge.heading_deg; REAR +180).
    float bodyHeadingFor(Direction dir, float edgeHeading)
    {
        if (dir != Direction::REAR) return edgeHeading;
        float h = fmodf(edgeHeading + 180.0f, 360.0f);
        if (h < 0.0f) h += 360.0f;
        return h;
    }
}    // namespace

// ===========================================================================
// `_evaluateProjectionPassThrough` replica, driven by the REAL graph reads +
// the REAL `resolveDirection` / `wrapDeltaDeg`.
//
// Mirrors `PathTracker::_evaluateProjectionPassThrough` cell-for-cell. The five
// conditions (compiled-out / non-goal-with-next-hop / outbound-flank-LINE /
// next-node-is-LINE-junction / collinear-and-same-direction) are evaluated over
// the REAL toy graph: the routing reads come from `FieldMap`, the heading check
// from the REAL `resolveDirection` + `wrapDeltaDeg`. NOTHING in the decision is
// re-implemented -- the gate is replicated, the graph + heading helpers are the
// production ones.
//
// The inbound hop is the EXPLICIT `cur -> proj` LINE-into-projection hop the
// firmware is dispatching when it calls this gate; `goalNode` is the active
// `goTo` destination. `inboundBodyHeading` / `inboundDir` are the A->B hop's
// resolved body heading + direction (the firmware reads them off
// `_resolvedBodyHeading` / `_resolvedDirection`).
//
// MIRROR: PathTracker.cpp:1770-1847 (_evaluateProjectionPassThrough).
// MIRROR: PathTracker.cpp:1739-1768 (_peekNextNextHop: not-final / NODE_NONE / self-loop).
// ===========================================================================

static bool replica_evaluateProjectionPassThrough(
    uint8_t proj, uint8_t goalNode, float inboundBodyHeading, Direction inboundDir)
{
    // (1) Compiled out -> never pass through. Leads so the NEXT_HOP peek is
    //     skipped entirely on a build without pass-through.
    // MIRROR: PathTracker.cpp:1784.
    if (!Config::PATH_PASS_THROUGH_ENABLED)
    {
        return false;
    }

    // (2) `_peekNextNextHop`: the not-final guard (`proj == goal`) is exactly the
    //     `goTo(B)` case -> NODE_NONE -> stop at B. Then read the onward hop C and
    //     apply the NODE_NONE / self-loop guards.
    // MIRROR: PathTracker.cpp:1758-1767 + 1793-1797.
    if (proj == goalNode)
    {
        return false;
    }
    const uint8_t c = FieldMap::getNextHop(proj, goalNode);
    if (c == FieldMap::NODE_NONE || c == proj)
    {
        return false;
    }

    // (3) The outbound edge B->C must be a LINE edge (both flanks LINE). The
    //     inbound A->B is LINE by construction (this gate is only reached on the
    //     LINE->projection dispatch path). A BLIND flank -> a normal stop at B.
    // MIRROR: PathTracker.cpp:1802-1806.
    const Routes::EdgeInfo& outbound = FieldMap::getEdge(proj, c);
    if (outbound.type != Routes::EdgeType::LINE)
    {
        return false;
    }

    // (4) C must be a LINE (junction) node so the B->C hop re-anchors at a real
    //     intersection. A projection / blind C is out of scope for the collapse.
    // MIRROR: PathTracker.cpp:1812-1815.
    if (FieldMap::getNodeType(c) != Routes::NodeType::LINE)
    {
        return false;
    }

    // (5) A->B->C collinear AND same direction -- the SAME ADR-023 heading check the
    //     intersection-gated pass-through uses, via the REAL `resolveDirection` +
    //     `wrapDeltaDeg`. Re-resolve the B->C direction with the inbound body
    //     heading as the current heading, rotate the outbound edge heading 180 deg
    //     for REAR, then wrap-normalize the delta.
    // MIRROR: PathTracker.cpp:1826-1846.
    const Direction nextDir = resolveDirection(
        outbound.direction_override,
        Routes::PIPELINE_CONFIG.direction,
        Direction::AUTO,    // arrival override (no builder arrival in these tests)
        Direction::AUTO,    // route override   (no builder route in these tests)
        inboundBodyHeading,
        outbound.heading_deg,
        (c == goalNode),
        Config::BIDIRECTIONAL_AUTO_ENABLED,
        Config::LINE_BOARD_COUNT);

    const float outboundBody = bodyHeadingFor(nextDir, outbound.heading_deg);
    const float delta        = wrapDeltaDeg(outboundBody - inboundBodyHeading);
    return (fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG) && (nextDir == inboundDir);
}

/// Resolve the inbound `cur -> proj` LINE hop's direction + body heading EXACTLY
/// as the firmware does (the values the gate reads off `_resolvedDirection` /
/// `_resolvedBodyHeading`). A bare `goTo()` with no builder overrides, FRONT auto
/// on the collinear inbound edge.
///
// MIRROR: PathTracker.cpp:1588-1623 (direction resolution + bodyTarget).
static void resolveInboundLineHop(
    uint8_t cur, uint8_t proj, bool isLastHop, Direction& dirOut, float& bodyOut)
{
    const Routes::EdgeInfo& inbound = FieldMap::getEdge(cur, proj);
    dirOut = resolveDirection(
        inbound.direction_override,
        Routes::PIPELINE_CONFIG.direction,
        Direction::AUTO,
        Direction::AUTO,
        inbound.heading_deg,    // current heading == the inbound edge heading on a clean approach
        inbound.heading_deg,
        isLastHop,
        Config::BIDIRECTIONAL_AUTO_ENABLED,
        Config::LINE_BOARD_COUNT);
    bodyOut = bodyHeadingFor(dirOut, inbound.heading_deg);
}

// ===========================================================================
// Pre-flight -- the toy fixture actually carries the `line(A)-projection(P1)-
// line(B)` collinear triple under test. A future toy-field change that drops a
// flank edge or de-collinearizes P1 fails loudly HERE rather than silently
// weakening the pass-through cases.
// ===========================================================================

static void test_TC11pre_toy_fixture_has_collinear_line_proj_line_triple()
{
    TEST_SCOPE("TC11-pre: toy carries collinear line(A)-projection(P1)-line(B)");

    const uint8_t a  = nodeId("A");
    const uint8_t b  = nodeId("B");
    const uint8_t p1 = nodeId("P1");

    // P1 is the projection node; A and B are LINE junction nodes.
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));
    ASSERT_TRUE(FieldMap::getNodeType(a) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(b) == Routes::NodeType::LINE);

    // A->P1 and P1->B are LINE edges (both flanks LINE).
    const Routes::EdgeInfo& aToP1 = FieldMap::getEdge(a, p1);
    const Routes::EdgeInfo& p1ToB = FieldMap::getEdge(p1, b);
    ASSERT_TRUE(aToP1.type == Routes::EdgeType::LINE);
    ASSERT_TRUE(p1ToB.type == Routes::EdgeType::LINE);

    // Collinear: A->P1 and P1->B share a heading (the gate's |delta| <= tolerance).
    const float delta = wrapDeltaDeg(p1ToB.heading_deg - aToP1.heading_deg);
    ASSERT_TRUE(fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG);

    // The onward lookahead from P1 toward B is the direct hop B (so the gate's C is
    // the LINE junction B, exercising the real route, not a fabricated next hop).
    ASSERT_TRUE(FieldMap::getNextHop(p1, b) == b);
}

// ===========================================================================
// TC11 -- line(A)-projection(P1)-line(B) routed PAST P1: the pass-through gate
//         FIRES. Drives the EXPLICIT A->P1 hop with goal B (the toy NEXT_HOP may
//         pick the direct A-B edge, so the hop is exercised explicitly per T21).
// ===========================================================================

static void test_TC11_line_proj_line_routed_past_b_passes_through()
{
    TEST_SCOPE("TC11: line(A)-projection(P1)-line(B) routed PAST P1 -> gate fires");

    const uint8_t a  = nodeId("A");
    const uint8_t b  = nodeId("B");
    const uint8_t p1 = nodeId("P1");

    // The inbound A->P1 hop is a LINE edge into the projection node (the only
    // dispatch path that reaches the projection pass-through gate). P1 is NOT the
    // goal (goal is B), so this is the routed-PAST case.
    ASSERT_TRUE(FieldMap::getEdgeType(a, p1) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));

    // Resolve the inbound hop's direction + body heading the way the firmware does
    // before calling the gate (A->P1 is NOT the last hop -- goal is B beyond it).
    Direction inboundDir;
    float     inboundBody;
    resolveInboundLineHop(a, p1, /*isLastHop*/ false, inboundDir, inboundBody);

    // The gate's onward lookahead must reach the real junction B THROUGH P1 -- a
    // route that never reached P1 would not satisfy this read. This pins that the
    // pass-through is exercised over the genuine A->P1->B path, not a bypass.
    ASSERT_TRUE(FieldMap::getNextHop(p1, b) == b);
    ASSERT_TRUE(FieldMap::getEdge(p1, b).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(b) == Routes::NodeType::LINE);

    // The load-bearing assertion: the pass-through gate FIRES -- the A->P1->B
    // triple collapses to one continuous follow (no stop at P1).
    const bool pass =
        replica_evaluateProjectionPassThrough(p1, /*goal*/ b, inboundBody, inboundDir);
    ASSERT_TRUE(pass);
}

// ===========================================================================
// TC12 -- goTo(P1) where P1 IS the projection goal: the gate does NOT fire (the
//         not-final guard rejects a goal-is-P1 route) -> a normal DISTANCE stop.
// ===========================================================================

static void test_TC12_goto_projection_goal_does_not_pass_through()
{
    TEST_SCOPE("TC12: goTo(P1) where P1 is the goal -> gate does NOT fire (stop at P1)");

    const uint8_t a  = nodeId("A");
    const uint8_t p1 = nodeId("P1");
    const uint8_t b  = nodeId("B");

    // A->P1 is the LINE-into-projection hop, but here P1 IS the goal: this is the
    // last hop, so the not-final guard must reject the collapse even though a
    // collinear LINE edge P1->B exists beyond it.
    ASSERT_TRUE(FieldMap::getEdgeType(a, p1) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));

    // A collinear LINE flank P1->B genuinely exists -- so the ONLY reason the gate
    // declines is the goal-is-P1 (not-final) guard, not a missing flank.
    ASSERT_TRUE(FieldMap::getEdge(p1, b).type == Routes::EdgeType::LINE);
    const float delta =
        wrapDeltaDeg(FieldMap::getEdge(p1, b).heading_deg - FieldMap::getEdge(a, p1).heading_deg);
    ASSERT_TRUE(fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG);

    Direction inboundDir;
    float     inboundBody;
    resolveInboundLineHop(a, p1, /*isLastHop*/ true, inboundDir, inboundBody);

    // The gate does NOT fire: P1 is the goal, so `proj == goalNode` -> stop at P1.
    const bool pass =
        replica_evaluateProjectionPassThrough(p1, /*goal*/ p1, inboundBody, inboundDir);
    ASSERT_FALSE(pass);
}

// ===========================================================================
// TC13 -- a BLIND-flanked projection node is a NORMAL stop: with the outbound
//         flank a BLIND edge, the gate's both-flanks-LINE condition fails.
//
// The toy field's only BLIND out-edge from P1 is P1->S (S is a BLIND node). Drive
// the A->P1 LINE hop with goal S: the onward lookahead from P1 is the BLIND P1->S
// edge, so the gate must decline (condition 3, outbound flank not LINE).
// ===========================================================================

static void test_TC13_blind_flanked_projection_node_normal_stop()
{
    TEST_SCOPE("TC13: BLIND outbound flank -> gate does NOT fire (normal stop at P1)");

    const uint8_t a  = nodeId("A");
    const uint8_t p1 = nodeId("P1");
    const uint8_t s  = nodeId("S");

    // The inbound A->P1 is still a LINE-into-projection hop (the gate IS reached),
    // but the onward flank toward the goal S is a BLIND edge.
    ASSERT_TRUE(FieldMap::getEdgeType(a, p1) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));

    // The route P1 -> ... -> S exits P1 over the BLIND P1->S edge (the both-flanks-
    // LINE condition fails on this flank). Confirm the onward hop + its BLIND type.
    const uint8_t onward = FieldMap::getNextHop(p1, s);
    ASSERT_TRUE(onward == s);
    ASSERT_TRUE(FieldMap::getEdge(p1, s).type == Routes::EdgeType::BLIND);

    Direction inboundDir;
    float     inboundBody;
    resolveInboundLineHop(a, p1, /*isLastHop*/ false, inboundDir, inboundBody);

    // The gate does NOT fire: the outbound flank P1->S is BLIND, so condition 3
    // rejects -> a normal stop at P1.
    const bool pass =
        replica_evaluateProjectionPassThrough(p1, /*goal*/ s, inboundBody, inboundDir);
    ASSERT_FALSE(pass);
}

// ===========================================================================
// TC13(b) -- the heading half of the gate (condition 5) is live: a LINE outbound
//            flank that is NOT collinear with the inbound hop is ALSO a normal
//            stop. This proves the gate declines on the heading check, not only on
//            the flank-type (condition 3) / node-type (condition 4) reads.
//
// Both LINE flanks out of P1 (P1->A heading 180deg, P1->B heading 0deg) are LINE
// edges into LINE junctions, so conditions 3 + 4 PASS for either. Drive the A->P1
// inbound hop (heading 0deg) with goal A: the gate's onward lookahead is the
// P1->A flank (heading 180deg), a 180deg U-turn relative to the inbound. The
// decline is therefore purely the collinearity check (condition 5) over the REAL
// resolveDirection + wrapDeltaDeg. (This is a unit probe of condition 5, not a
// sensible mission route -- a real goTo never doubles back through P1 to its own
// origin; the point is to feed the gate a non-collinear LINE flank and confirm it
// declines on heading.)
// ===========================================================================

static void test_TC13b_non_collinear_outbound_flank_normal_stop()
{
    TEST_SCOPE("TC13b: non-collinear LINE outbound flank -> gate does NOT fire");

    const uint8_t a  = nodeId("A");
    const uint8_t p1 = nodeId("P1");

    // Inbound A->P1 is a LINE-into-projection hop (heading 0deg in the toy field).
    ASSERT_TRUE(FieldMap::getEdgeType(a, p1) == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::isProjectionNode(p1));

    // The onward flank P1->A is a LINE edge into the LINE junction A, so conditions
    // 3 (flank is LINE) and 4 (C is a LINE junction) BOTH pass -- isolating the
    // heading check (condition 5) as the sole reason the gate can decline here.
    ASSERT_TRUE(FieldMap::getEdge(p1, a).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getNodeType(a) == Routes::NodeType::LINE);
    ASSERT_TRUE(FieldMap::getNextHop(p1, a) == a);

    Direction inboundDir;
    float     inboundBody;
    resolveInboundLineHop(a, p1, /*isLastHop*/ false, inboundDir, inboundBody);

    // A->P1 is heading 0deg; the onward P1->A flank is heading 180deg -> a 180deg
    // delta, far outside NAV_TURN_TOLERANCE_DEG -> the gate's collinearity check
    // (condition 5) declines even though both flanks are LINE junctions. Proves the
    // heading half of the gate is live over the REAL resolveDirection/wrapDeltaDeg.
    const bool pass =
        replica_evaluateProjectionPassThrough(p1, /*goal*/ a, inboundBody, inboundDir);
    ASSERT_FALSE(pass);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker line(A)-projection(P1)-line(B) pass-through "
                "(issue #64, T19 / T21, TC11/12/13) -- real graph reads + frozen "
                "resolveDirection/wrapDeltaDeg over the toy projection node P1\n");

    test_TC11pre_toy_fixture_has_collinear_line_proj_line_triple();
    test_TC11_line_proj_line_routed_past_b_passes_through();
    test_TC12_goto_projection_goal_does_not_pass_through();
    test_TC13_blind_flanked_projection_node_normal_stop();
    test_TC13b_non_collinear_outbound_flank_normal_stop();

    TEST_REPORT();
}
