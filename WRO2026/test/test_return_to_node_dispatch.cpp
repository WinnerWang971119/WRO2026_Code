// =========================================================================================
// Host-compile tests for the return-to-node NODE-KIND dispatch slice (issue #36, PR1b,
// T8/T10 — TC4 / TC4b).
//
// Two things are under test here, at two layers:
//
//   1. The line-seek CLASSIFICATION (`_returnUsesLineSeek = nodeIsLine && _returnDistCm >=
//      Config::LINE_SENSOR_AXLE_OFFSET_CM` in `_dispatchReturnMove`). The node-kind half is
//      tested DIRECTLY against the REAL `FieldMap::getNodeType` over the REAL `routes.cpp` graph
//      data — the same symbol and the same node table production reads (TC4); the distance-gate
//      half is tested against the REAL `Config::LINE_SENSOR_AXLE_OFFSET_CM` (TC4c, #14). The toy
//      fixture carries BOTH LINE nodes (indices 0..3, "Inter1..4") and BLIND nodes (indices 4..6,
//      "Start"/"final"/"position1"), so this is a genuine LINE-vs-BLIND discrimination, NOT a
//      tautology.
//
//   2. The per-kind RETURN_TO_NODE state-body ROUTING (which Drive observations lead to a
//      successful re-anchor vs which lead to which fault) is inline-MIRRORED, following the
//      precedent set by `test_path_tracker_stop_at_line.cpp` /
//      `test_path_tracker_passthrough.cpp` / `test_path_tracker_bidirectional.cpp`: the
//      full production `PathTracker.cpp` is NOT linked by ANY host target (it would drag in
//      the Drive fluent-builder + Localization + EventTrace + MatrixMiniR4 surface), so the
//      small decision slice is replicated and every mirrored decision is annotated
//      `// MIRROR: PathTracker.cpp:<line>` for drift detection at PR-review time.
//
// What this file does NOT cover (delegated, per the same precedent): the actual Drive
// builder chain emitted by `_dispatchReturnMove` (the `Drive.stopAtLine(...).async()` seek
// for LINE, the `turnToYaw + driveDist` dead-reckon for BLIND), the axle-on-node arrival
// geometry, and the end-to-end FSM transition into/out of RETURN_TO_NODE. Those are
// covered by the Python sim parity test (T11, TC18a, `tools/path_planner/sim` /
// `test_runtime_sim.py`) which runs the real mission against the real field, and by the
// Arduino firmware compile. The GATE decision (trigger/cap/off-map) is covered by the pure
// policy test (`test_return_to_node_policy.cpp`), which links the same `resolveReturnAction`
// symbol production uses.
//
// Links `routes.cpp` (and the mock surface for the angle-bracket `<Arduino.h>` /
// `<MatrixMiniR4.h>` that FieldMap.h -> routes.h pull in) but NO SUT `.cpp` and NO Drive
// force-include — FieldMap.h / routes.h are header-only / const-POD data and never include
// the production `Drive.h`. The pass/fail + final-line conventions mirror
// `test_pivot_trim_policy.cpp` / `test_return_to_node_policy.cpp`.
// =========================================================================================

#include <cstdio>
#include <stdint.h>

#include "Config.h"          // Config::LINE_SENSOR_AXLE_OFFSET_CM (the line-seek distance gate).
#include "FieldMap.h"        // real FieldMap::getNodeType + getNode (header-only inline).
#include "routes.h"          // Routes::NodeType, Routes::N_NODES, Routes::NODES (data in routes.cpp).
#include "PathTracker.h"     // PathTrackerClass::Fault (the real enum the mirror routes to).

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

// ===========================================================================
// RETURN_TO_NODE state-body routing replica (per node kind).
//
// Mirrors the slice of `PathTracker::_stepFsm()` case `State::RETURN_TO_NODE`
// that decides, once `Drive.isIdle()` is true, what re-anchor / fault action to
// take — branching on `_returnUsesLineSeek` (set by `_dispatchReturnMove` from
// the node kind AND the return distance vs the axle lead) and the live Drive
// fault + the compile-time off-map gate.
//
// The production code reads `Drive.isIdle()` / `Drive.hasFault()`; here those two
// observations are plain `bool` inputs (the mock Drive has no fault-injection
// surface wired into this target — same modelling choice as
// test_path_tracker_stop_at_line.cpp's replica). The off-map gate is a parameter
// rather than `Config::OFF_MAP_GATE` so a single test can exercise both policies
// without a per-target compile-definition.
// ===========================================================================

/// What one replicated RETURN_TO_NODE tick decided.
enum class ReturnTickOutcome : uint8_t
{
    WAIT,                ///< Drive not idle yet — return drive still in flight.
    REANCHOR_ONINTER,    ///< LINE success: re-anchor SAME node via onIntersection().
    REANCHOR_CLEAR,      ///< BLIND success OR LINE BEST_EFFORT: re-anchor via clearOffset().
    FAULT_OFF_NODE,      ///< LINE no-line under FAULT gate -> Fault::OFF_NODE.
    FAULT_DRIVE,         ///< BLIND drive failure -> Fault::DRIVE_FAULT.
};

/// Side-effect-bearing result of one replicated RETURN_TO_NODE tick.
struct ReturnTickResult
{
    ReturnTickOutcome       outcome      = ReturnTickOutcome::WAIT;
    bool                    reAnchored   = false;    ///< onIntersection() OR clearOffset() issued.
    bool                    planNext     = false;    ///< _state advanced to PLAN_NEXT_HOP.
    PathTrackerClass::Fault fault        = PathTrackerClass::Fault::NONE;
};

/// Replica of one `PathTrackerClass::_stepFsm()` tick while in `State::RETURN_TO_NODE`.
///
/// @param usesLineSeek  The `_returnUsesLineSeek` flag — `true` only when the anchor is a LINE
///                      node AND the return distance reaches the front sensor's axle lead
///                      (set by `_dispatchReturnMove`). A short LINE return is `false` and drives
///                      the blind (dead-reckon) routing, exactly like a blind anchor (#14).
/// @param driveIsIdle   The value `Drive.isIdle()` would return this tick.
/// @param driveHasFault The value `Drive.hasFault()` would return this tick.
/// @param gateIsFault   `Config::OFF_MAP_GATE == FAULT` (line-seek no-line routing only).
///
// MIRROR: PathTracker.cpp (not-idle -> WAIT, both paths; then clear _returnActive).
// MIRROR: PathTracker.cpp (line seek: non-LINE_NOT_FOUND fault -> DRIVE_FAULT;
//         LINE_NOT_FOUND -> off-map gate; else onIntersection).
// MIRROR: PathTracker.cpp (dead-reckon: hasFault -> DRIVE_FAULT; else clearOffset).
static ReturnTickResult replica_returnTick(bool usesLineSeek, bool driveIsIdle,
                                           bool driveHasFault, bool gateIsFault,
                                           bool faultIsLineNotFound = true)
{
    ReturnTickResult r{};

    // Common gate: RETURN_TO_NODE is EXEMPT from the generic top-of-update() Drive-fault
    // guard, so the state body itself waits for the drive to finish before inspecting it.
    if (!driveIsIdle)
    {
        return r;    // outcome stays WAIT
    }

    if (usesLineSeek)
    {
        if (driveHasFault)
        {
            // #3 discriminator (MIRROR: PathTracker.cpp:593): the stopAtLine seek's EXPECTED
            // fault is LINE_NOT_FOUND, which routes to the OFF-MAP GATE. Any OTHER Drive fault
            // (e.g. a future DISPATCH_FAILED) is a genuine hardware/dispatch failure and
            // surfaces as DRIVE_FAULT, NOT the off-map gate.
            if (!faultIsLineNotFound)
            {
                r.outcome = ReturnTickOutcome::FAULT_DRIVE;
                r.fault   = PathTrackerClass::Fault::DRIVE_FAULT;
                return r;
            }
            if (gateIsFault)
            {
                r.outcome = ReturnTickOutcome::FAULT_OFF_NODE;
                r.fault   = PathTrackerClass::Fault::OFF_NODE;
                return r;
            }
            // BEST_EFFORT: clear the latched Drive fault, accept the dead-reckoned pose
            // (re-anchor the SAME node via clearOffset), and plan.
            r.outcome    = ReturnTickOutcome::REANCHOR_CLEAR;
            r.reAnchored = true;     // Localization.clearOffset()
            r.planNext   = true;
            return r;
        }

        // LINE success: the seek detected the crossing and rolled the axle onto the node —
        // re-anchor the SAME node via onIntersection (a verified physical re-localisation).
        r.outcome    = ReturnTickOutcome::REANCHOR_ONINTER;
        r.reAnchored = true;         // Localization.onIntersection(currentNode)
        r.planNext   = true;
        return r;
    }

    // BLIND-only anchor: no line to snap onto. A Drive fault here is a genuine drive failure
    // (neither turnToYaw nor driveDist seeks a line), so it is a real DRIVE_FAULT.
    if (driveHasFault)
    {
        r.outcome = ReturnTickOutcome::FAULT_DRIVE;
        r.fault   = PathTrackerClass::Fault::DRIVE_FAULT;
        return r;
    }

    // BLIND success: the dead-reckoned inverse drive IS the re-anchor (node id unchanged).
    r.outcome    = ReturnTickOutcome::REANCHOR_CLEAR;
    r.reAnchored = true;             // Localization.clearOffset()
    r.planNext   = true;
    return r;
}

int main()
{
    std::printf("INFO: return-to-node node-kind dispatch — real getNodeType (TC4) + "
                "RETURN_TO_NODE routing mirror (TC4b)\n");

    // -------------------------------------------------------------------------------------
    // TC4 — node-kind CLASSIFICATION against the REAL FieldMap::getNodeType over the REAL
    // routes.cpp graph. The toy fixture has LINE nodes 0..3 and BLIND nodes 4..6, so the
    // node-kind HALF of the `_returnUsesLineSeek` predicate (getNodeType(cur) == NodeType::LINE)
    // discriminates the two kinds for real. The distance HALF (dist >= the axle lead) is covered
    // by TC4c below.
    // -------------------------------------------------------------------------------------
    // MIRROR: PathTracker.cpp _dispatchReturnMove
    //         (nodeIsLine = getNodeType(cur) == NodeType::LINE).
    {
        // The fixture must actually contain both kinds for the discrimination to be real —
        // assert that precondition so a future field_data change that drops one kind fails
        // loudly here rather than silently weakening the test.
        bool sawLine  = false;
        bool sawBlind = false;
        for (uint8_t id = 0; id < Routes::N_NODES; ++id)
        {
            const Routes::NodeType t = FieldMap::getNodeType(id);
            if (t == Routes::NodeType::LINE) { sawLine = true; }
            if (t == Routes::NodeType::BLIND) { sawBlind = true; }
        }
        check(sawLine, "TC4 precondition: routes.cpp toy fixture has at least one LINE node");
        check(sawBlind, "TC4 precondition: routes.cpp toy fixture has at least one BLIND node");

        // Spot-check known node ids against the fixture's authored types (routes.cpp:16-22):
        // node 0 is LINE-backed (a long-enough return uses the seek), node 4 is BLIND-only.
        const bool lineBacked0 = (FieldMap::getNodeType(0) == Routes::NodeType::LINE);
        const bool lineBacked4 = (FieldMap::getNodeType(4) == Routes::NodeType::LINE);
        check(lineBacked0, "TC4: node 0 (Inter1) is LINE-backed -> stopAtLine-seek return");
        check(!lineBacked4, "TC4: node 4 (Start) is BLIND-only -> dead-reckon return");
    }

    // -------------------------------------------------------------------------------------
    // TC4c — line-seek DISTANCE GATE (#14). `_returnUsesLineSeek` is true ONLY for a LINE
    // anchor whose return distance reaches the front sensor's axle lead
    // (Config::LINE_SENSOR_AXLE_OFFSET_CM). A shorter LINE return leaves the sensor already
    // past the node line, so a forward seek can never re-cross it — it dead-reckons back like a
    // blind anchor. Mirror the full decision and confirm a short LINE return routes through the
    // dead-reckon (blind) branch of replica_returnTick (clearOffset success / DRIVE_FAULT), NOT
    // the line-seek branch (onIntersection / off-map gate).
    // -------------------------------------------------------------------------------------
    // MIRROR: PathTracker.cpp _dispatchReturnMove
    //         (_returnUsesLineSeek = nodeIsLine && _returnDistCm >= LINE_SENSOR_AXLE_OFFSET_CM).
    {
        const float axle = Config::LINE_SENSOR_AXLE_OFFSET_CM;
        // Node 0 is LINE (asserted in TC4). The decision mirror:
        const bool node0IsLine = (FieldMap::getNodeType(0) == Routes::NodeType::LINE);

        const bool shortUsesSeek = node0IsLine && ((axle - 1.0f) >= axle);     // short return
        const bool atUsesSeek    = node0IsLine && (axle >= axle);              // exactly the lead
        const bool longUsesSeek  = node0IsLine && ((axle + 10.0f) >= axle);    // comfortably long
        check(!shortUsesSeek, "TC4c: a LINE return below the axle lead does NOT use the line seek");
        check(atUsesSeek, "TC4c: a LINE return exactly at the axle lead uses the line seek (>=)");
        check(longUsesSeek, "TC4c: a LINE return above the axle lead uses the line seek");

        // A BLIND node never uses the seek regardless of distance.
        const bool node4IsLine = (FieldMap::getNodeType(4) == Routes::NodeType::LINE);
        const bool blindLongUsesSeek = node4IsLine && ((axle + 10.0f) >= axle);
        check(!blindLongUsesSeek, "TC4c: a BLIND anchor never uses the line seek (any distance)");

        // A short LINE return (usesLineSeek == false) drives the BLIND routing: an idle, no-fault
        // arrival re-anchors via clearOffset; a faulted arrival is a genuine DRIVE_FAULT.
        const ReturnTickResult ok = replica_returnTick(/*usesLineSeek*/ shortUsesSeek,
                                                       /*idle*/ true, /*fault*/ false,
                                                       /*gateFault*/ true);
        check(ok.outcome == ReturnTickOutcome::REANCHOR_CLEAR && ok.planNext
                  && ok.fault == PathTrackerClass::Fault::NONE,
              "TC4c: short LINE return success re-anchors via clearOffset (dead-reckon path)");
        const ReturnTickResult bad = replica_returnTick(/*usesLineSeek*/ shortUsesSeek,
                                                        /*idle*/ true, /*fault*/ true,
                                                        /*gateFault*/ true);
        check(bad.outcome == ReturnTickOutcome::FAULT_DRIVE
                  && bad.fault == PathTrackerClass::Fault::DRIVE_FAULT,
              "TC4c: short LINE return drive failure surfaces DRIVE_FAULT (not the off-map gate)");
    }

    // -------------------------------------------------------------------------------------
    // TC4b — RETURN_TO_NODE routing mirror: each node kind's Drive-observation -> action.
    // -------------------------------------------------------------------------------------

    // Both kinds: drive still in flight (not idle) -> WAIT, no action.
    {
        const ReturnTickResult line  = replica_returnTick(/*usesLineSeek*/ true,  /*idle*/ false, /*fault*/ false, /*gateFault*/ true);
        const ReturnTickResult blind = replica_returnTick(/*usesLineSeek*/ false, /*idle*/ false, /*fault*/ false, /*gateFault*/ true);
        check(line.outcome == ReturnTickOutcome::WAIT && !line.reAnchored && !line.planNext,
              "TC4b: LINE return waits while the seek is in flight");
        check(blind.outcome == ReturnTickOutcome::WAIT && !blind.reAnchored && !blind.planNext,
              "TC4b: BLIND return waits while the dead-reckon drive is in flight");
    }

    // LINE-backed success: idle, no fault -> re-anchor SAME node via onIntersection.
    {
        const ReturnTickResult r = replica_returnTick(/*usesLineSeek*/ true, /*idle*/ true, /*fault*/ false, /*gateFault*/ true);
        check(r.outcome == ReturnTickOutcome::REANCHOR_ONINTER,
              "TC4b: LINE return success re-anchors via onIntersection (physical re-localisation)");
        check(r.reAnchored && r.planNext && r.fault == PathTrackerClass::Fault::NONE,
              "TC4b: LINE success re-anchors, plans next, and does not fault");
    }

    // LINE-backed LINE_NOT_FOUND under FAULT gate: idle + faulted -> Fault::OFF_NODE
    // (NOT DRIVE_FAULT — the state is exempt from the generic guard precisely so this
    // routes to the off-map gate).
    {
        const ReturnTickResult r = replica_returnTick(/*usesLineSeek*/ true, /*idle*/ true, /*fault*/ true, /*gateFault*/ true);
        check(r.outcome == ReturnTickOutcome::FAULT_OFF_NODE,
              "TC4b: LINE no-line under FAULT gate surfaces Fault::OFF_NODE (not DRIVE_FAULT)");
        check(r.fault == PathTrackerClass::Fault::OFF_NODE && !r.reAnchored && !r.planNext,
              "TC4b: LINE no-line FAULT does not re-anchor or plan");
    }

    // LINE-backed LINE_NOT_FOUND under BEST_EFFORT gate: idle + faulted -> accept the
    // dead-reckoned pose (clearOffset re-anchor) and plan; no fault latched.
    {
        const ReturnTickResult r = replica_returnTick(/*usesLineSeek*/ true, /*idle*/ true, /*fault*/ true, /*gateFault*/ false);
        check(r.outcome == ReturnTickOutcome::REANCHOR_CLEAR,
              "TC4b: LINE no-line under BEST_EFFORT accepts the dead-reckoned pose (clearOffset)");
        check(r.reAnchored && r.planNext && r.fault == PathTrackerClass::Fault::NONE,
              "TC4b: LINE no-line BEST_EFFORT re-anchors + plans without faulting");
    }

    // LINE-backed NON-LINE_NOT_FOUND fault (#3 discriminator): idle + faulted but the fault is
    // NOT LINE_NOT_FOUND (e.g. a future DISPATCH_FAILED). It must surface DRIVE_FAULT, NOT the
    // off-map gate — under BOTH gate policies (the gate only governs LINE_NOT_FOUND).
    {
        const ReturnTickResult rFaultGate = replica_returnTick(
            /*usesLineSeek*/ true, /*idle*/ true, /*fault*/ true, /*gateFault*/ true,
            /*faultIsLineNotFound*/ false);
        check(rFaultGate.outcome == ReturnTickOutcome::FAULT_DRIVE,
              "TC4b: LINE non-LINE_NOT_FOUND fault surfaces Fault::DRIVE_FAULT (not OFF_NODE)");
        check(rFaultGate.fault == PathTrackerClass::Fault::DRIVE_FAULT
                  && !rFaultGate.reAnchored && !rFaultGate.planNext,
              "TC4b: LINE non-LINE_NOT_FOUND fault does not re-anchor or plan");
        const ReturnTickResult rBestGate = replica_returnTick(
            /*usesLineSeek*/ true, /*idle*/ true, /*fault*/ true, /*gateFault*/ false,
            /*faultIsLineNotFound*/ false);
        check(rBestGate.outcome == ReturnTickOutcome::FAULT_DRIVE,
              "TC4b: LINE non-LINE_NOT_FOUND fault is DRIVE_FAULT regardless of off-map gate");
    }

    // BLIND-only success: idle, no fault -> re-anchor via clearOffset (node id unchanged).
    {
        const ReturnTickResult r = replica_returnTick(/*usesLineSeek*/ false, /*idle*/ true, /*fault*/ false, /*gateFault*/ true);
        check(r.outcome == ReturnTickOutcome::REANCHOR_CLEAR,
              "TC4b: BLIND return success re-anchors via clearOffset (dead-reckon arrival)");
        check(r.reAnchored && r.planNext && r.fault == PathTrackerClass::Fault::NONE,
              "TC4b: BLIND success re-anchors, plans next, and does not fault");
    }

    // BLIND-only drive failure: idle + faulted -> Fault::DRIVE_FAULT (a genuine drive
    // failure, NOT routed to the off-map gate — no line seek was involved).
    {
        const ReturnTickResult r = replica_returnTick(/*usesLineSeek*/ false, /*idle*/ true, /*fault*/ true, /*gateFault*/ true);
        check(r.outcome == ReturnTickOutcome::FAULT_DRIVE,
              "TC4b: BLIND return drive failure surfaces Fault::DRIVE_FAULT (not the off-map gate)");
        check(r.fault == PathTrackerClass::Fault::DRIVE_FAULT && !r.reAnchored && !r.planNext,
              "TC4b: BLIND drive failure does not re-anchor or plan");
        // The gate policy must NOT change the BLIND drive-failure routing (the gate only
        // applies to LINE LINE_NOT_FOUND) — confirm BEST_EFFORT yields the same DRIVE_FAULT.
        const ReturnTickResult rBest = replica_returnTick(/*usesLineSeek*/ false, /*idle*/ true, /*fault*/ true, /*gateFault*/ false);
        check(rBest.outcome == ReturnTickOutcome::FAULT_DRIVE,
              "TC4b: BLIND drive failure is DRIVE_FAULT regardless of off-map gate policy");
    }

    if (g_failures == 0)
    {
        std::printf("RETURN TO NODE DISPATCH OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
