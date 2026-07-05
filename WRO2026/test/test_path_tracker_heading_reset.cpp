/**
 * @file test_path_tracker_heading_reset.cpp
 * @brief Host-link FSM gating tests for the line-arrival heading reset
 *        (`PathTrackerClass::_maybeHeadingReset`, ADR-052, T6). Covers TC8-TC14 (AC7-AC11).
 *
 * @details
 * Drives the REAL `PathTracker` + `Drive` + `Localization` + `YawSource` FSMs through the
 * shared cooperative-tick harness (`test_path_tracker_harness.h`), the same recipe as
 * `test_path_tracker_pass_through_projection.cpp` / `test_path_tracker_reset_arrival.cpp`. No
 * mirror of `_maybeHeadingReset`'s decision logic exists anywhere in this file — every
 * assertion reads an OBSERVABLE the real FSM produced (the live `Localization.getHeadingDeg()`
 * re-read, the mock `Serial` capture, the mock `EventTrace` push count, and — under
 * `PT_PARITY_TRACE` — the NDJSON `pt_parity::emit` stream).
 *
 * ===========================================================================================
 * FOUR EXECUTABLES FROM ONE SOURCE FILE (CMakeLists.txt)
 * ===========================================================================================
 *   1. `test_path_tracker_heading_reset` — shipped defaults (proj knob ON, junction knob OFF,
 *      strict ON, tolerance 10 deg), linked against the issue #91 `routes_fixture.cpp`. Runs
 *      TC8, TC9, TC10 (OFF half), TC12, TC13.
 *   2. `test_path_tracker_heading_reset_junction_on` — same recipe, `-D
 *      LINE_HEADING_RESET_ON_JUNCTION_DEFAULT=true` target-wide. Runs TC10 (ON half) only —
 *      `main()` branches at RUNTIME on the compiled-in `Config::LINE_HEADING_RESET_ON_JUNCTION`
 *      (the `test_reanchor_heading` idiom), so the SAME `.cpp` proves the compile-def actually
 *      reached the separately-compiled `Localization.cpp`/`PathTracker.cpp` TUs.
 *   3. `test_path_tracker_heading_reset_forced_reset` — links a DELIBERATELY invalid, hand-
 *      written 3-node routes table (`fixtures/routes_reset_passthrough_forced.{h,cpp}`, NOT
 *      `routes_fixture.cpp`) instead of the issue #91 fixture, compiled with
 *      `HEADING_RESET_FORCED_RESET_FIXTURE`. Runs TC11 only — see that table's header docblock
 *      for why AC9's forced-state case needs its own topology (in short: per ADR-048, no LINE
 *      edge may be incident to a RESET node on any validator-passed field, and
 *      `PathTrackerClass::_passThrough` / `_maybeHeadingReset` are private with no test-only
 *      accessor, so the ONLY way to drive the real FSM into that branch — without touching
 *      PathTracker.h/.cpp, out of scope for T6 — is a hand-rolled table that violates ADR-048
 *      on purpose, exactly the "hand-edited routes table that bypasses the validator" case the
 *      plan's Decisions section names as one of the branch's two real-world triggers).
 *   4. `test_path_tracker_heading_reset_parity` — `routes_fixture.cpp` + `PT_PARITY_TRACE`.
 *      Runs TC14 only: captures the real stdout NDJSON stream `pt_parity::emit` writes (the
 *      SAME facility `test_path_tracker_parity_dump.cpp` uses) around one scripted tick at a
 *      time via `freopen` + `ftell`/`fseek` marks, and asserts the `heading_reset` record count
 *      in that window — no production code is touched to make this observable.
 *
 * Every scenario resolves nodes BY NAME (`nodeByName`, `FieldMap::findNodeByName`), never by
 * index (issue #91). TC8-TC10/TC12-TC14 use the fixture's collinear projection triple
 * `Inter1 -> ProjL -> Inter2` (y=40) and collinear LINE row `Inter2 -> Inter3 -> Inter4` (both
 * already present — no fixture regeneration needed, verified against
 * `fixtures/routes_fixture.cpp`). Every arrival that is supposed to latch
 * `PathTrackerClass::_passThrough` is proven to have done so via the SAME observable
 * `test_path_tracker_pass_through_projection.cpp` (TC-PP1) uses:
 * `LineFollower.lastBrakeOnGate == false` at arm time, and the follower staying enabled (no
 * self-brake) after the gate trips — never a mirror of the FSM's own decision.
 *
 * `_maybeHeadingReset` only ever fires on the tick AFTER the node-advancing tick (PathTracker's
 * `_stepFsm()` handles exactly one state transition per `cooperativeTick()`, confirmed against
 * `test_path_tracker_pass_through_projection.cpp`'s TC-PP1 continuation loop) — every scenario
 * below drives one extra `cooperativeTick()` past the arrival before asserting the reset
 * outcome.
 *
 * @note `Serial` / `EventTrace` are the `path_mocks`/`hal_mocks` capturing mocks force-included
 *       target-wide by CMake (`add_path_force_includes`) — already visible here with no extra
 *       `#include`, same as every sibling PathTracker+Drive real-link test.
 */

#include "test_path_tracker_harness.h"

#include "src/nav/NavTypes.h"    // Direction enum (TC9's forced-REAR route override).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#if defined(PT_PARITY_TRACE)
#include "src/mission/PathTrackerParityTrace.h"    // pt_parity::emit (PT_PARITY_TRACE is defined for this target).
#endif

// ===========================================================================================
// Shared drivers — compiled for every target EXCEPT the forced-reset fixture (which has no
// `Inter1` / `ProjL` / `Inter2` / `Inter3` / `Inter4` nodes to drive).
// ===========================================================================================
#if !defined(HEADING_RESET_FORCED_RESET_FIXTURE)
namespace
{
// `driveToLineFollowEntry(start, goal, routeDirection=AUTO)` — the shared LINE-follow entry
// driver — now lives in `test_path_tracker_harness.h` (hoisted so the sibling
// `test_path_tracker_pass_through_projection.cpp` can reuse the SAME entry-drive handshake
// instead of maintaining a divergent copy). Called unqualified below.

/// @brief Drain whatever is left of the in-flight `goTo` to `IDLE` (or a fault), parking the
///        mock IMU yaw at the FSM's own `resolvedBodyHeadingDeg()` target every tick (settles
///        any incidental residual pre-turn generically,
///        `gotcha-pathtracker-host-test-preturn-park-yaw`) and unconditionally latching both
///        LINE gates each tick. Sufficient for this file's remaining tail hops (plain collinear
///        LINE hops, no seek/pivot) — used only to prove AC10's "no behavior change", never to
///        assert a specific intermediate state.
[[maybe_unused]] void driveRemainingHopsToIdle()
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks * 3
                    && !PathTracker.isIdle() && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg =
            (double) hwYaw((int16_t) lroundf(PathTracker.resolvedBodyHeadingDeg()));
        Intersection.mockSetAtIntersection(true);
        LineFollower.mockSetDistanceReached(true);
        cooperativeTick();
    }
}
}    // namespace
#endif    // !HEADING_RESET_FORCED_RESET_FIXTURE

// ===========================================================================================
// TC8 / TC9 / TC10 / TC12 / TC13 — default-knobs + junction-ON builds ONLY (not the forced-
// reset fixture, not the parity capture build).
// ===========================================================================================
#if !defined(HEADING_RESET_FORCED_RESET_FIXTURE) && !defined(PT_PARITY_TRACE)

namespace
{
/// @brief Drive the real FSM from a clean world through the `Inter2 -> Inter3 -> Inter4`
///        collinear LINE-LINE-LINE row's first hop to the arrival AT `Inter3` — the shared setup
///        for TC10 (both knob states) and TC13's knob-off silence case. Proves the pass-through
///        genuinely latched (`LineFollower.lastBrakeOnGate == false` at arm time, the follower
///        still enabled after the trip), the SAME observable TC-PP1 uses for the projection
///        triple.
void driveToLineJunctionPassThroughArrival(uint8_t& outInter2, uint8_t& outInter3,
                                            uint8_t& outInter4)
{
    outInter2 = nodeByName("Inter2");
    outInter3 = nodeByName("Inter3");
    outInter4 = nodeByName("Inter4");
    ASSERT_TRUE(outInter2 != FieldMap::NODE_NONE);
    ASSERT_TRUE(outInter3 != FieldMap::NODE_NONE);
    ASSERT_TRUE(outInter4 != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNodeType(outInter3) == Routes::NodeType::LINE);
    ASSERT_EQ(routedFirstHop(outInter2, outInter4), outInter3);
    assertNoBypass(outInter2, outInter4);

    driveToLineFollowEntry("Inter2", "Inter4");
    ASSERT_EQ(Localization.getCurrentNode(), outInter2);

    // Gate 1 — the intersection crossing. THE PASS-THROUGH ARM PROOF.
    Intersection.mockSetAtIntersection(true);
    cooperativeTick();
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);    // pass-through latched (collinear Inter2-3-4).

    // Gate 2 — axle-overshoot advance onto Inter3. Pass-through keeps the follower live.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), outInter3);
    ASSERT_TRUE(LineFollower.isEnabled());
}
}    // namespace

// -------------------------------------------------------------------------------------------
// TC8 (AC7) — forward collinear PROJECTION pass-through binds heading to the incoming edge's
// heading_deg.
// -------------------------------------------------------------------------------------------
void test_TC8_forward_projection_pass_through_binds_heading()
{
    TEST_SCOPE("TC8 (AC7): forward LINE-projection(ProjL)-LINE pass-through binds heading to "
               "the incoming edge heading");

    ASSERT_TRUE(Config::LINE_HEADING_RESET_ON_PROJECTION);    // precondition: default knob ON.

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");
    const uint8_t inter2 = nodeByName("Inter2");
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);
    ASSERT_TRUE(projL != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter2 != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNodeType(projL) == Routes::NodeType::PROJECTION);

    driveToLineFollowEntry("Inter1", "Inter2");
    ASSERT_EQ(Localization.getCurrentNode(), inter1);
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);    // pass-through armed (TC-PP1 precondition).

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter1, projL);    // heading 0, FRONT.

    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();    // mid-follow dead-reckon advance onto ProjL; _state = PLAN_NEXT_HOP (pending).
    ASSERT_EQ(Localization.getCurrentNode(), projL);
    ASSERT_TRUE(LineFollower.isEnabled());    // kept live -- _passThrough latched true.

    // Perturb the live IMU yaw a few degrees off the target BEFORE the PLAN_NEXT_HOP tick that
    // fires _maybeHeadingReset, so a snap back to EXACTLY the edge heading afterward is
    // unambiguous proof the bind fired (not a coincidence of an already-correct belief).
    const float perturbedApp = inbound.heading_deg + 3.0f;    // inside the default 10 deg tolerance.
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(perturbedApp));
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - perturbedApp) < 0.5f);    // genuinely off first.

    const int eventsBefore = EventTrace.mockCount();
    cooperativeTick();    // PLAN_NEXT_HOP -> _onPlanNextHop -> _maybeHeadingReset (proj pass-through).

    // THE BIND: heading snapped to EXACTLY the incoming edge heading (0), not the perturbed 3.
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - inbound.heading_deg) < 0.5f);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_GT(EventTrace.mockCount(), eventsBefore);    // "HDG reset" pushed on the fire.
}

// -------------------------------------------------------------------------------------------
// TC9 (AC7) — REAR/reverse projection pass-through binds heading_deg + 180 (body-corrected).
// -------------------------------------------------------------------------------------------
void test_TC9_rear_projection_pass_through_binds_heading_plus_180()
{
    TEST_SCOPE("TC9 (AC7): a REAR/reverse projection pass-through binds heading_deg + 180 "
               "(body-corrected)");

    ASSERT_TRUE(Config::LINE_HEADING_RESET_ON_PROJECTION);
    ASSERT_EQ(Config::LINE_BOARD_COUNT, (uint8_t) 2);    // REAR LINE hops need the rear board.

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");

    driveToLineFollowEntry("Inter1", "Inter2", Direction::REAR);
    ASSERT_EQ(Localization.getCurrentNode(), inter1);
    ASSERT_FALSE(PathTracker.hasFault());    // no REVERSE_UNAVAILABLE on the 2-board default.
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter1, projL);    // raw heading 0.
    float bodyCorrected = fmodf(inbound.heading_deg + 180.0f, 360.0f);
    if (bodyCorrected < 0.0f)
    {
        bodyCorrected += 360.0f;
    }

    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projL);
    ASSERT_TRUE(LineFollower.isEnabled());

    const float perturbedApp = bodyCorrected + 3.0f;
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(perturbedApp));
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - perturbedApp) < 0.5f);

    const int eventsBefore = EventTrace.mockCount();
    cooperativeTick();

    // THE BIND target is heading_deg + 180 (the body-corrected REAR heading), NOT the raw
    // edge.heading_deg (0) and NOT the perturbed value.
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - bodyCorrected) < 0.5f);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_GT(EventTrace.mockCount(), eventsBefore);
}

// -------------------------------------------------------------------------------------------
// TC10 (AC8) — junction pass-through: OFF -> no bind (default knobs), ON -> binds
// (junction-ON target). `main()` dispatches the ON half only when the compiled-in
// `Config::LINE_HEADING_RESET_ON_JUNCTION` is true.
// -------------------------------------------------------------------------------------------
void test_TC10_off_junction_pass_through_no_bind()
{
    TEST_SCOPE("TC10 (AC8): junction pass-through does NOT bind when "
               "LINE_HEADING_RESET_ON_JUNCTION is OFF (default)");
    ASSERT_FALSE(Config::LINE_HEADING_RESET_ON_JUNCTION);

    uint8_t inter2 = 0;
    uint8_t inter3 = 0;
    uint8_t inter4 = 0;
    driveToLineJunctionPassThroughArrival(inter2, inter3, inter4);

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter2, inter3);
    const float perturbedApp = inbound.heading_deg + 3.0f;
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(perturbedApp));

    const int eventsBefore = EventTrace.mockCount();
    Serial.clearCapture();
    cooperativeTick();    // PLAN_NEXT_HOP -> _maybeHeadingReset: knob OFF -> silent no-op.

    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - perturbedApp) < 0.5f);    // unchanged, no bind.
    ASSERT_EQ(EventTrace.mockCount(), eventsBefore);
    ASSERT_TRUE(Serial.captured().empty());

    // AC10: no behavior change -- the FSM proceeds exactly as before, reaching Inter4 / IDLE.
    driveRemainingHopsToIdle();
    ASSERT_EQ(Localization.getCurrentNode(), inter4);
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

void test_TC10_on_junction_pass_through_binds()
{
    TEST_SCOPE("TC10 (AC8): junction pass-through binds heading when "
               "LINE_HEADING_RESET_ON_JUNCTION is ON");
    ASSERT_TRUE(Config::LINE_HEADING_RESET_ON_JUNCTION);

    uint8_t inter2 = 0;
    uint8_t inter3 = 0;
    uint8_t inter4 = 0;
    driveToLineJunctionPassThroughArrival(inter2, inter3, inter4);
    (void) inter4;

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter2, inter3);
    const float perturbedApp = inbound.heading_deg + 3.0f;
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(perturbedApp));

    const int eventsBefore = EventTrace.mockCount();
    cooperativeTick();    // PLAN_NEXT_HOP -> _maybeHeadingReset: knob ON -> binds.

    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - inbound.heading_deg) < 0.5f);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_GT(EventTrace.mockCount(), eventsBefore);
}

// -------------------------------------------------------------------------------------------
// TC12 (AC10) — non-pass-through arrival and a first-plan each produce no reset and no FSM
// behavior change. (The knob-OFF no-op is TC10's OFF half above — AC8 and AC10 share that one
// drive; not duplicated here.)
// -------------------------------------------------------------------------------------------
void test_TC12a_non_pass_through_projection_stop_no_reset()
{
    TEST_SCOPE("TC12a (AC10): a non-pass-through PROJECTION stop (goTo(ProjL)) produces no reset");

    const uint8_t projL = nodeByName("ProjL");

    driveToLineFollowEntry("Inter1", "ProjL");    // ProjL IS the goal -> not-final guard rejects.
    ASSERT_TRUE(LineFollower.lastBrakeOnGate);    // a normal stop, NOT a pass-through (TC-PP2).

    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();    // stops + dead-reckons the advance onto ProjL.
    ASSERT_EQ(Localization.getCurrentNode(), projL);

    mmL.mockYawDeg = (double) hwYaw(9);    // a value a bind WOULD snap away from, if it fired.
    Serial.clearCapture();
    const int eventsBefore = EventTrace.mockCount();

    cooperativeTick();    // PLAN_NEXT_HOP -> _maybeHeadingReset: !_passThrough -> return, no-op.

    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - 9.0f) < 0.5f);    // unchanged -- no bind.
    ASSERT_EQ(EventTrace.mockCount(), eventsBefore);
    ASSERT_TRUE(Serial.captured().empty());

    // AC10: behavior unchanged -- the goal (ProjL) is still reached normally, no fault.
    driveRemainingHopsToIdle();
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_EQ(Localization.getCurrentNode(), projL);
    ASSERT_FALSE(PathTracker.hasFault());
}

void test_TC12b_first_plan_zero_hop_goal_no_reset()
{
    TEST_SCOPE("TC12b (AC10): a first-plan (zero-hop) goTo produces no reset; the FSM proceeds "
               "to IDLE unchanged");

    resetWorld();
    Serial.clearCapture();
    EventTrace.mockReset();
    seatAt("Inter1");

    const int eventsBefore = EventTrace.mockCount();
    PathTracker.goTo(nodeByName("Inter1")).async();    // already at the goal -- a genuine first plan.

    driveRemainingHopsToIdle();

    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_EQ(EventTrace.mockCount(), eventsBefore);
    ASSERT_TRUE(Serial.captured().find("heading reset") == std::string::npos);
}

// -------------------------------------------------------------------------------------------
// TC13 (AC11) — Serial logs one line on a fire and on a skip (read-failure / strict-reject);
// knob-OFF is SILENT.
// -------------------------------------------------------------------------------------------
void test_TC13_fire_logs_one_line()
{
    TEST_SCOPE("TC13 (AC11): Serial logs one line on a fire (node/kind/heading/delta)");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");

    driveToLineFollowEntry("Inter1", "Inter2");
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projL);

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter1, projL);
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(inbound.heading_deg + 3.0f));

    Serial.clearCapture();
    cooperativeTick();

    const std::string log = Serial.captured();
    ASSERT_TRUE(log.find("[PathTracker] heading reset: node ProjL (proj) bound heading=")
                != std::string::npos);
    ASSERT_TRUE(log.find("preBindDelta=") != std::string::npos);
    ASSERT_TRUE(log.find("SKIP") == std::string::npos);
}

void test_TC13_strict_reject_skip_logs_one_line()
{
    TEST_SCOPE("TC13 (AC11): Serial logs one line on a strict-reject skip (delta + reason)");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");

    driveToLineFollowEntry("Inter1", "Inter2");
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projL);

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter1, projL);
    const float rejectedApp = inbound.heading_deg + 45.0f;    // well past the default 10 deg tolerance.
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(rejectedApp));

    Serial.clearCapture();
    cooperativeTick();

    const std::string log = Serial.captured();
    ASSERT_TRUE(log.find("[PathTracker] heading reset SKIP: node ProjL (proj) strict-rejected")
                != std::string::npos);
    ASSERT_TRUE(log.find("delta=") != std::string::npos);
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - rejectedApp) < 0.5f);    // no bind -- unchanged.
}

void test_TC13_read_failure_skip_logs_one_line()
{
    TEST_SCOPE("TC13 (AC11): Serial logs one line on an IMU-read-failure skip (delta=n/a)");

    const uint8_t projL = nodeByName("ProjL");

    driveToLineFollowEntry("Inter1", "Inter2");
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projL);

    mmL.mockResult = MMLower::RESULT::ERROR;    // script a transient IMU timeout.
    Serial.clearCapture();
    cooperativeTick();
    mmL.mockResult = MMLower::RESULT::OK;    // restore so later reads in this process succeed.

    const std::string log = Serial.captured();
    ASSERT_TRUE(log.find("[PathTracker] heading reset SKIP: node ProjL (proj) IMU read failed; "
                          "delta=n/a")
                != std::string::npos);
}

void test_TC13_knob_off_is_silent()
{
    TEST_SCOPE("TC13 (AC11): Serial is SILENT on a junction pass-through when the junction knob "
               "is OFF (default)");
    ASSERT_FALSE(Config::LINE_HEADING_RESET_ON_JUNCTION);

    uint8_t inter2 = 0;
    uint8_t inter3 = 0;
    uint8_t inter4 = 0;
    driveToLineJunctionPassThroughArrival(inter2, inter3, inter4);
    (void) inter2;
    (void) inter3;
    (void) inter4;

    Serial.clearCapture();
    cooperativeTick();    // PLAN_NEXT_HOP -> _maybeHeadingReset: knob OFF -> return before any log.

    ASSERT_TRUE(Serial.captured().empty());
}

#endif    // !HEADING_RESET_FORCED_RESET_FIXTURE && !PT_PARITY_TRACE

// ===========================================================================================
// TC11 (AC9) — forced state, the `test_path_tracker_heading_reset_forced_reset` target ONLY.
// See `fixtures/routes_reset_passthrough_forced.h` for why this needs its own topology.
// ===========================================================================================
#if defined(HEADING_RESET_FORCED_RESET_FIXTURE)

namespace
{
/// @brief Drive the real FSM from a clean world through `PassA -> PassResetB -> PassC`'s
///        intersection-gated LINE pass-through, landing the discrete node on `PassResetB` with
///        `_passThrough` latched true (the SAME real gate TC-PP1 / TC8 exercise — it never
///        inspects the outbound node's `NodeType`), then return with the FSM parked at
///        `State::PLAN_NEXT_HOP`, `_maybeEnterReset` not yet consulted for this arrival.
void driveToForcedResetPassThroughArrival()
{
    resetWorld();
    Serial.clearCapture();
    EventTrace.mockReset();
    seatAt("PassA");

    const uint8_t passA      = nodeByName("PassA");
    const uint8_t passResetB = nodeByName("PassResetB");
    const uint8_t passC      = nodeByName("PassC");
    ASSERT_TRUE(passA != FieldMap::NODE_NONE);
    ASSERT_TRUE(passResetB != FieldMap::NODE_NONE);
    ASSERT_TRUE(passC != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNodeType(passResetB) == Routes::NodeType::RESET);
    ASSERT_TRUE(Config::PATH_PASS_THROUGH_ENABLED);

    PathTracker.goTo(passC).async();

    // Drive into LINE_FOLLOW for the PassA -> PassResetB leg (heading 0 == the seated facing,
    // so no pre-turn fires).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == passA)
        {
            break;
        }
    }
    ASSERT_TRUE(LineFollower.isEnabled());
    ASSERT_EQ(Localization.getCurrentNode(), passA);

    // Gate 1 -- the "intersection" crossing at PassResetB. THE PASS-THROUGH ARM PROOF: the real
    // LINE_FOLLOW peek (PathTracker.cpp) sees the collinear outbound PassResetB->PassC LINE edge
    // and latches _passThrough == true -- observed here via brake-on-gate=false, exactly like
    // TC-PP1's precondition assert. The gate itself never inspects C's NodeType (only
    // _evaluateProjectionPassThrough does), so it has no idea PassResetB is a RESET node --
    // which is precisely how a validator-bypassing table reaches PathTracker's defensive branch.
    Intersection.mockSetAtIntersection(true);
    cooperativeTick();
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);    // _passThrough latched true.

    // Gate 2 -- axle-overshoot: advances the node onto PassResetB. Pass-through keeps the
    // follower live (no self-brake) -- the second half of the proof.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), passResetB);
    ASSERT_TRUE(LineFollower.isEnabled());    // still live -- confirms no active-brake stop fired.
}
}    // namespace

void test_TC11_forced_pass_through_into_reset_node_is_a_no_op()
{
    TEST_SCOPE("TC11 (AC9): _passThrough latched true arriving at a RESET node -> BLIND/RESET "
               "no-op (forced, ADR-048-unreachable state)");

    driveToForcedResetPassThroughArrival();

    const uint8_t passResetB = nodeByName("PassResetB");

    // Perturb the live IMU yaw well within what WOULD bind if the reset fired, so a snap-back
    // would be unambiguous -- and assert it does NOT snap back.
    mmL.mockYawDeg = (double) hwYaw(7);    // 7 deg off 0, inside the default 10 deg tolerance.
    const float perturbedBelief = Localization.getHeadingDeg();
    ASSERT_TRUE(fabsf(perturbedBelief - 7.0f) < 0.5f);

    // Tick 1: PLAN_NEXT_HOP -> _onPlanNextHop() -> _maybeEnterReset(PassResetB) fires (RESET,
    // latch not yet set): _dispatchWallStop finds NO reset record (deliberately omitted from
    // this table) and short-circuits straight back to PLAN_NEXT_HOP, latching _resetNode.
    // _maybeHeadingReset is NOT reached this tick (the wall hook's early return in
    // _onPlanNextHop consumes it).
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), passResetB);    // the shortcut doesn't move the node.
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - perturbedBelief) < 0.5f);    // still untouched.

    const size_t serialLenAfterWallShortcut = Serial.captured().size();
    const int    eventsAfterWallShortcut    = EventTrace.mockCount();

    // Tick 2: PLAN_NEXT_HOP -> _onPlanNextHop() -> _maybeEnterReset(PassResetB) now returns
    // false (latch holds) -> _maybeHeadingReset(cur=PassResetB, firstPlan=false, ...) FINALLY
    // runs with _passThrough STILL true -- takes the RESET case of the node-type switch: a
    // silent no-op.
    cooperativeTick();

    // NO bind: the perturbed belief survives byte-for-byte.
    ASSERT_TRUE(fabsf(Localization.getHeadingDeg() - perturbedBelief) < 0.5f);
    // NO EventTrace push ("HDG reset" is the ONLY event this feature ever pushes).
    ASSERT_EQ(EventTrace.mockCount(), eventsAfterWallShortcut);
    // SILENT: no NEW Serial output at all -- the BLIND/RESET branch logs nothing (unlike a
    // knob-ON skip, which is loud). The only prior Serial output was _dispatchWallStop's
    // unrelated "no reset record" warning (tick 1), captured above as the baseline.
    ASSERT_EQ((int) Serial.captured().size(), (int) serialLenAfterWallShortcut);
    ASSERT_TRUE(Serial.captured().find("heading reset") == std::string::npos);

    ASSERT_FALSE(PathTracker.hasFault());
}

#endif    // HEADING_RESET_FORCED_RESET_FIXTURE

// ===========================================================================================
// TC14 (AC11) — parity: a bind emits exactly one `heading_reset` record; a strict-reject emits
// none. The `test_path_tracker_heading_reset_parity` target ONLY (`PT_PARITY_TRACE`).
// ===========================================================================================
#if defined(PT_PARITY_TRACE)

#ifndef HEADING_RESET_PARITY_CAPTURE_PATH
#define HEADING_RESET_PARITY_CAPTURE_PATH "heading_reset_parity_capture.ndjson"    // defensive default.
#endif

namespace
{
long g_parityMark = 0;

/// @brief Mark the current stdout write position (after an `fflush`) — the start of a capture
///        window. `pt_parity::emit` (PathTrackerParityTrace.h) is the SOLE stdout writer once
///        `stdout` has been redirected to a real file by `main()` below, so everything written
///        between a `parityCaptureBegin()` / `parityCaptureSince()` pair is exactly what the
///        bracketed `cooperativeTick()` call(s) emitted.
void parityCaptureBegin()
{
    std::fflush(stdout);
    g_parityMark = std::ftell(stdout);
}

/// @brief Return the bytes written to stdout since the matching `parityCaptureBegin()`, then
///        reposition the file cursor back to the end so subsequent writes keep appending.
std::string parityCaptureSince()
{
    std::fflush(stdout);
    const long endPos = std::ftell(stdout);
    std::string out;
    if (endPos > g_parityMark)
    {
        const size_t len = (size_t) (endPos - g_parityMark);
        out.resize(len);
        std::fseek(stdout, g_parityMark, SEEK_SET);
        const size_t got = std::fread(&out[0], 1, len, stdout);
        out.resize(got);
        std::fseek(stdout, 0, SEEK_END);
    }
    return out;
}

/// @brief Count non-overlapping occurrences of @p needle in @p hay.
int countOccurrences(const std::string& hay, const char* needle)
{
    int          count     = 0;
    size_t       pos       = 0;
    const size_t needleLen = std::strlen(needle);
    while ((pos = hay.find(needle, pos)) != std::string::npos)
    {
        ++count;
        pos += needleLen;
    }
    return count;
}
}    // namespace

void test_TC14_bind_emits_exactly_one_record()
{
    TEST_SCOPE("TC14 (AC11): a successful bind emits exactly one heading_reset parity record");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");

    driveToLineFollowEntry("Inter1", "Inter2");
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projL);

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter1, projL);
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(inbound.heading_deg + 3.0f));    // within tolerance.

    parityCaptureBegin();
    cooperativeTick();    // fires _maybeHeadingReset -> bind -> _ptEmitHeadingReset.
    const std::string captured = parityCaptureSince();

    ASSERT_EQ(countOccurrences(captured, "\"kind\":\"heading_reset\""), 1);
    ASSERT_TRUE(captured.find("\"node\":\"ProjL\"") != std::string::npos);
    ASSERT_TRUE(captured.find("\"pass_kind\":\"proj\"") != std::string::npos);
}

void test_TC14_strict_reject_emits_no_record()
{
    TEST_SCOPE("TC14 (AC11): a strict-reject emits NO heading_reset parity record");

    const uint8_t inter1 = nodeByName("Inter1");
    const uint8_t projL  = nodeByName("ProjL");

    driveToLineFollowEntry("Inter1", "Inter2");
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), projL);

    const Routes::EdgeInfo& inbound = FieldMap::getEdge(inter1, projL);
    // Far outside the default 10 deg tolerance -> strict reject, no bind, no parity record.
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(inbound.heading_deg + 45.0f));

    parityCaptureBegin();
    cooperativeTick();
    const std::string captured = parityCaptureSince();

    ASSERT_EQ(countOccurrences(captured, "\"kind\":\"heading_reset\""), 0);
}

#endif    // PT_PARITY_TRACE

// ===========================================================================================
// Entry.
// ===========================================================================================
int main()
{
#if defined(HEADING_RESET_FORCED_RESET_FIXTURE)
    std::fprintf(stderr,
                 "INFO: PathTracker heading-reset FSM gating -- forced-state RESET/BLIND "
                 "no-op build (TC11, AC9)\n");
    test_TC11_forced_pass_through_into_reset_node_is_a_no_op();

#elif defined(PT_PARITY_TRACE)
    std::fprintf(stderr,
                 "INFO: PathTracker heading-reset FSM gating -- PT_PARITY_TRACE build (TC14, "
                 "AC11)\n");
    // Redirect stdout to a real file for the whole run: pt_parity::emit is the sole stdout
    // writer once this succeeds (see the class docblock in PathTrackerParityTrace.h). ctest
    // scores this target on exit code only, so leaving stdout redirected through TEST_REPORT()'s
    // trailing "PASS" line is harmless.
    std::fflush(stdout);
    FILE* capFile = std::freopen(HEADING_RESET_PARITY_CAPTURE_PATH, "w+b", stdout);
    ASSERT_TRUE(capFile != nullptr);

    test_TC14_bind_emits_exactly_one_record();
    test_TC14_strict_reject_emits_no_record();

#else
    std::fprintf(stderr,
                 "INFO: PathTracker heading-reset FSM gating -- LINE_HEADING_RESET_ON_PROJECTION "
                 "= %s, LINE_HEADING_RESET_ON_JUNCTION = %s, LINE_HEADING_RESET_STRICT_ENABLED = "
                 "%s, tolerance = %.2f\n",
                 Config::LINE_HEADING_RESET_ON_PROJECTION ? "ON" : "OFF",
                 Config::LINE_HEADING_RESET_ON_JUNCTION ? "ON" : "OFF",
                 Config::LINE_HEADING_RESET_STRICT_ENABLED ? "ON" : "OFF",
                 Config::LINE_HEADING_RESET_TOLERANCE_DEG);

    if (Config::LINE_HEADING_RESET_ON_JUNCTION)
    {
        // Junction-ON target: focused TC10-ON only.
        test_TC10_on_junction_pass_through_binds();
    }
    else
    {
        // Default-knobs target: TC8/TC9/TC12/TC13 + TC10's OFF half.
        test_TC8_forward_projection_pass_through_binds_heading();
        test_TC9_rear_projection_pass_through_binds_heading_plus_180();
        test_TC10_off_junction_pass_through_no_bind();
        test_TC12a_non_pass_through_projection_stop_no_reset();
        test_TC12b_first_plan_zero_hop_goal_no_reset();
        test_TC13_fire_logs_one_line();
        test_TC13_strict_reject_skip_logs_one_line();
        test_TC13_read_failure_skip_logs_one_line();
        test_TC13_knob_off_is_silent();
    }
#endif

    if (wro_test::failCounter() == 0)
    {
        std::fprintf(stderr, "PATH TRACKER HEADING RESET OK\n");
    }
    TEST_REPORT();
}
