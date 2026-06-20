// =========================================================================================
// Host-compile tests for the PathTracker reset-node arrival interlude — the WALL_STOP -> REANCHOR
// slice (PR2 / T16, AC10 + AC11). TC10b.
//
// WHY AN INLINE MIRROR (not the real PathTracker.cpp)
//   `PathTracker.cpp` is NOT host-compiled by ANY target (it drags in the Drive fluent-builder +
//   EventTrace + MatrixMiniR4 + the full FSM). Following the precedent of
//   `test_return_to_node_dispatch.cpp` / `test_path_tracker_stop_at_line.cpp`, this file
//   inline-MIRRORS the small WALL_STOP -> REANCHOR decision slice and annotates each mirrored
//   decision `// MIRROR: PathTracker.cpp:<line>` so a reviewer diffs the replica against the cited
//   lines and the two cannot silently drift. The replica routes to the REAL
//   `PathTrackerClass::Fault` enum (so a rename of `Fault::WALL_NOT_SQUARE` breaks this build — the
//   safety net we want).
//
// REAL Localization IS LINKED (the "even better" path in the T18 brief)
//   The REANCHOR step calls the REAL `Localization.reanchorAtWall(...)` over the REAL `routes.cpp`
//   reset record (node 5 "Wall1", { axis=1, wall_coord=154, facing=270 }), so the success-path
//   re-anchor and its post-conditions are genuine, not mocked. This target compiles against the
//   SHIPPED default (strict wall-normal OFF), so the live IMU heading does not gate the re-anchor —
//   that lets TC10b drive the advisory-timeout + success cases through real Localization.
//
//   The interlude now runs WALL_STOP -> REANCHOR -> WALL_DEPART -> PLAN_NEXT_HOP: on a successful
//   reanchor the FSM drives `standoff - WALL_CONTACT_OFFSET_CM` forward off the wall (WALL_DEPART),
//   then on `Drive.isIdle()` calls `Localization.clearOffset()` so the FINAL believed pose lands on
//   the NODE coordinate (y=139), not the axle (146) and not the wall (154). The mirror models this
//   final clearOffset directly (the mock Drive carries no real motion); the believed-on-node
//   post-condition is therefore genuine against real Localization.
//
//   For the strict-REJECT routing (AC11: reanchorAtWall returns false -> WALL_NOT_SQUARE) the
//   replica is driven with `reanchorOk = false` — the exact value real Localization returns when
//   the strict check rejects (that ON-path rejection itself is proven against real Localization by
//   `test_reanchor_at_wall_strict.cpp`). Flipping the strict flag in THIS target would also flip the
//   success cases, so the reject ROUTING is modelled on the boolean outcome here and the boolean's
//   provenance is covered by the sibling strict target. This split keeps each target single-purpose.
//
// WHAT TC10b ASSERTS (AC10 + AC11):
//   - On arrival at a RESET node, the interlude dispatches wallStop and enters WALL_STOP.
//   - WALL_STOP waits for `Drive.isIdle()`, then enters REANCHOR.
//   - In REANCHOR, a wallStop ADVISORY TIMEOUT (`Drive.wallStopTimedOut()` true) does NOT halt:
//     the slice STILL calls reanchor (assume-at-wall), surfaces the advisory (EventTrace/Serial),
//     and CONTINUES (-> WALL_DEPART). It is NOT a fault.
//   - On a successful reanchor the slice dispatches the depart drive and enters WALL_DEPART; on
//     `Drive.isIdle()` it clears the offset (believed pose == NODE coordinate) and continues to
//     PLAN_NEXT_HOP. A strict-REJECT (reanchorAtWall returns false) routes to `Fault::WALL_NOT_SQUARE`
//     BEFORE any depart.
//   - The degenerate `departCm <= 0` skip is intentionally NOT a host case: the validator's
//     clearance floor (standoff >= WALL_PIVOT_CLEARANCE_CM) plus the
//     `WALL_PIVOT_CLEARANCE_CM >= WALL_CONTACT_OFFSET_CM` static_assert make
//     `standoff - WALL_CONTACT_OFFSET_CM > 0` on any valid field, so the guard is unreachable
//     here — it is covered by those compile-time / authoring guarantees, not a runtime test.
//   - A non-reset node is a true no-op (the interlude is not entered).
//
// Links the real `Localization.cpp` + `routes.cpp` + the mock surface (same shape as
// `test_localization_offset` / `test_return_to_node_dispatch`). Pass/fail + final-line conventions
// mirror `test_return_to_node_dispatch.cpp`.
// =========================================================================================

#include <cmath>
#include <cstdio>
#include <stdint.h>

#include "Drive.h"            // mock Drive (isIdle modelled; no wallStop surface — see mirror inputs).
#include "MatrixMiniR4.h"     // mock mmL for real Localization's IMU read.
#include "Localization.h"     // REAL Localization.cpp linked — reanchorAtWall is genuine.
#include "FieldMap.h"         // FieldMap::getNodeType / getNode / getResetInfo over real routes.cpp.
#include "routes.h"           // Routes::NodeType, RESET_NODES (data in routes.cpp).
#include "PathTracker.h"      // PathTrackerClass::Fault (the real enum the mirror routes to).

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

static bool approx(float a, float b, float tol)
{
    return std::fabs(a - b) <= tol;
}

// The toy field's sole RESET node + a non-reset control node (routes.cpp).
static constexpr uint8_t kResetNode    = 5;   // "Wall1" (NodeType::RESET) at (165, 139).
static constexpr uint8_t kNonResetNode = 0;   // "Inter1" (NodeType::LINE).

// ===========================================================================
// Replica of the reset-node arrival interlude (PathTracker FSM slice).
//
//   _maybeEnterReset(cur)  -> if RESET && not-yet-run: dispatch wallStop, enter WALL_STOP.
//   State::WALL_STOP       -> wait for Drive.isIdle(); then enter REANCHOR.
//   State::REANCHOR        -> surface the advisory (if any), call reanchorAtWall; on reject route
//                             Fault::WALL_NOT_SQUARE, else dispatch the depart drive + enter
//                             WALL_DEPART.
//   State::WALL_DEPART     -> wait for Drive.isIdle(); then clearOffset (believed pose == NODE
//                             coordinate) and continue to PLAN_NEXT_HOP.
//
// The production code reads `Drive.isIdle()` / `Drive.wallStopTimedOut()`; here those two
// observations are plain `bool` inputs to the replicated tick (same modelling choice as
// test_return_to_node_dispatch.cpp / test_path_tracker_stop_at_line.cpp — the mock Drive has no
// wallStop surface). The reanchor + clearOffset themselves call REAL Localization.
// ===========================================================================

enum class RState : uint8_t { ARRIVAL, WALL_STOP, REANCHOR, WALL_DEPART, PLAN_NEXT_HOP, FAULT };

struct ResetArrivalMirror
{
    RState                  state      = RState::ARRIVAL;
    uint8_t                 resetNode  = 0xFF;     ///< mirror of `_resetNode` latch (NODE_NONE-ish).
    bool                    interludeEntered = false;
    bool                    advisorySurfaced = false;
    bool                    reanchorCalled   = false;
    bool                    departDispatched = false;
    bool                    offsetCleared    = false;
    PathTrackerClass::Fault fault            = PathTrackerClass::Fault::NONE;

    /// MIRROR: PathTracker.cpp:1163-1184 (_maybeEnterReset) + 1232-1243 (_dispatchWallStop tail).
    /// Returns true if the interlude was entered (the node is a not-yet-run RESET node).
    bool maybeEnterReset(uint8_t cur)
    {
        if (FieldMap::getNodeType(cur) == Routes::NodeType::RESET && resetNode != cur)
        {
            // _dispatchWallStop latches `_resetNode = cur` BEFORE dispatch so a second tick cannot
            // re-fire, then dispatches the async wallStop and enters WALL_STOP.
            resetNode        = cur;
            interludeEntered = true;
            state            = RState::WALL_STOP;
            return true;
        }
        return false;
    }

    /// One WALL_STOP tick: wait for the push to settle, then enter REANCHOR.
    /// MIRROR: PathTracker.cpp:889-905 (case State::WALL_STOP).
    void tickWallStop(bool driveIsIdle)
    {
        if (driveIsIdle)
        {
            state = RState::REANCHOR;
        }
        // else: stay in WALL_STOP (the push is still in flight).
    }

    /// One REANCHOR tick. @p wallStopTimedOut = `Drive.wallStopTimedOut()`; @p reanchorOk = the
    /// value `Localization.reanchorAtWall(...)` returned this tick (the caller passes the real
    /// outcome). MIRROR: PathTracker.cpp:907-966 (case State::REANCHOR).
    void tickReanchor(uint8_t cur, bool wallStopTimedOut, bool reanchorOk)
    {
        // AC10 — surface the advisory but DO NOT halt. A timeout means the push never stalled
        // (never reached the wall); we STILL re-anchor and continue ("assume at wall").
        // MIRROR: PathTracker.cpp:918-924.
        if (wallStopTimedOut)
        {
            advisorySurfaced = true;    // EventTrace.pushEvent + Serial in production.
        }

        // The reanchor was attempted regardless of the advisory (assume-at-wall).
        reanchorCalled = true;

        // AC11 — strict wall-normal check rejected the squaring -> WALL_NOT_SQUARE, do not continue.
        // MIRROR: PathTracker.cpp:947-957.
        if (!reanchorOk)
        {
            state = RState::FAULT;
            fault = PathTrackerClass::Fault::WALL_NOT_SQUARE;
            // _enterFault clears the reset latch (re-arm) in production.
            resetNode = 0xFF;
            return;
        }

        // Success: re-anchored at the AXLE. Dispatch the depart drive (`standoff -
        // WALL_CONTACT_OFFSET_CM` forward, back onto the node) and enter WALL_DEPART. The
        // `_resetNode == cur` latch is left set so the immediate return does not re-enter the
        // interlude for this same node. MIRROR: PathTracker.cpp (REANCHOR success tail ->
        // WALL_DEPART dispatch).
        (void) cur;
        departDispatched = true;
        state            = RState::WALL_DEPART;
    }

    /// One WALL_DEPART tick: wait for the depart drive to finish, then clear the offset so the
    /// FINAL believed pose lands on the NODE coordinate (not the axle), and continue to
    /// PLAN_NEXT_HOP. The mock Drive carries no real motion, so the production `driveDist` is
    /// modelled by `departDispatched` above and the post-idle `clearOffset` is invoked here
    /// against REAL Localization. MIRROR: PathTracker.cpp (case State::WALL_DEPART).
    void tickWallDepart(bool driveIsIdle)
    {
        if (driveIsIdle)
        {
            Localization.clearOffset();    // believed pose -> node coordinate (offset zeroed).
            offsetCleared = true;
            state         = RState::PLAN_NEXT_HOP;
        }
        // else: stay in WALL_DEPART (the depart drive is still in flight).
    }
};

/// Reset both mock surfaces + real Localization, anchored on the reset node with the frame zeroed.
static void resetWorldOnResetNode()
{
    Drive.mockReset();
    mmL.mockReset();
    Localization.begin();
    mmL.mockYawDeg = 0.0;
    Localization.zeroHeadingFromCurrent();
    Localization.setNavActive(true);            // PathTracker owns the wheels during the interlude.
    Localization.declarePosition(kResetNode);   // arrival anchored the node to "Wall1".
}

int main()
{
    std::printf("INFO: reset-node arrival interlude (WALL_STOP -> REANCHOR) — real Localization; "
                "strict flag = %s\n",
                Config::WALL_NORMAL_STRICT_ENABLED ? "ON" : "OFF");

    // This target compiles against the shipped default (strict OFF) so the real reanchor proceeds
    // on the success/advisory cases without the live IMU heading gating it.
    check(Config::WALL_NORMAL_STRICT_ENABLED == false,
          "precondition: this target compiles with WALL_NORMAL_STRICT_ENABLED OFF (shipped default)");
    // The fixture must actually have node 5 tagged RESET for the interlude to be exercised.
    check(FieldMap::getNodeType(kResetNode) == Routes::NodeType::RESET,
          "precondition: routes.cpp node 5 (Wall1) is a RESET node");
    check(FieldMap::getResetInfo(kResetNode) != nullptr,
          "precondition: node 5 carries a reset record");

    // -------------------------------------------------------------------------------------
    // TC10b primary — advisory-timeout arrival: STILL re-anchors, surfaces the advisory, and
    // CONTINUES (does NOT halt). Re-anchor runs through REAL Localization, so the post-condition
    // (believed y == wall_coord) is genuine.
    // -------------------------------------------------------------------------------------
    {
        resetWorldOnResetNode();
        const Routes::ResetNode* info = FieldMap::getResetInfo(kResetNode);

        ResetArrivalMirror fsm;

        // Arrival at the reset node -> interlude entered, now in WALL_STOP.
        const bool entered = fsm.maybeEnterReset(kResetNode);
        check(entered && fsm.state == RState::WALL_STOP,
              "TC10b: arrival at a RESET node enters the wall-stop interlude");

        // WALL_STOP waits while the push is in flight (drive not idle), then advances on idle.
        fsm.tickWallStop(/*driveIsIdle*/ false);
        check(fsm.state == RState::WALL_STOP, "TC10b: WALL_STOP waits while the push is in flight");
        fsm.tickWallStop(/*driveIsIdle*/ true);
        check(fsm.state == RState::REANCHOR, "TC10b: WALL_STOP advances to REANCHOR once Drive is idle");

        // REANCHOR with the wallStop ADVISORY set: call the REAL reanchor (assume-at-wall) and feed
        // its genuine outcome into the slice. Live IMU yaw is off-normal (60), but strict is OFF so
        // the real reanchor accepts.
        mmL.mockYawDeg = -60.0;
        const bool reanchorOk =
            Localization.reanchorAtWall(info->axis, info->wall_coord, (float) info->facing);
        check(reanchorOk, "TC10b: real reanchorAtWall accepts under the advisory (strict OFF)");

        fsm.tickReanchor(kResetNode, /*wallStopTimedOut*/ true, reanchorOk);

        check(fsm.advisorySurfaced, "TC10b: the wallStop advisory is surfaced (AC10)");
        check(fsm.reanchorCalled, "TC10b: the interlude STILL calls reanchor on an advisory timeout");
        check(fsm.departDispatched && fsm.state == RState::WALL_DEPART,
              "TC10b: an advisory timeout STILL dispatches the depart drive (enters WALL_DEPART)");
        check(fsm.fault == PathTrackerClass::Fault::NONE,
              "TC10b: an advisory timeout is NOT a fault");

        // WALL_DEPART waits while the depart drive is in flight, then clears the offset on idle.
        fsm.tickWallDepart(/*driveIsIdle*/ false);
        check(fsm.state == RState::WALL_DEPART, "TC10b: WALL_DEPART waits while the depart drive runs");
        fsm.tickWallDepart(/*driveIsIdle*/ true);
        check(fsm.offsetCleared && fsm.state == RState::PLAN_NEXT_HOP,
              "TC10b: WALL_DEPART clears the offset and continues to PLAN_NEXT_HOP once Drive is idle");

        // Genuine post-condition from real Localization: after the depart + clearOffset the FINAL
        // believed pose lands on the NODE coordinate (139), NOT the wall (154) and NOT the axle (146).
        const Routes::NodeInfo& n = FieldMap::getNode(kResetNode);
        const Offset2D off        = Localization.getOffsetFromNode();
        check(approx(n.y_cm + off.y, n.y_cm, 0.01f),
              "TC10b: after depart + clearOffset, believed y == node coordinate (real Localization)");
        check(approx(Localization.getHeadingDeg(), (float) info->facing, 0.5f),
              "TC10b: after the assume-at-wall reanchor, heading == facing");
    }

    // -------------------------------------------------------------------------------------
    // TC10b success (no advisory) — a normal stall-success arrival: re-anchor + continue, no
    // advisory surfaced. Confirms the advisory path is gated on the flag, not always taken.
    // -------------------------------------------------------------------------------------
    {
        resetWorldOnResetNode();
        const Routes::ResetNode* info = FieldMap::getResetInfo(kResetNode);

        ResetArrivalMirror fsm;
        fsm.maybeEnterReset(kResetNode);
        fsm.tickWallStop(/*driveIsIdle*/ true);

        mmL.mockYawDeg = -90.0;    // squared to the normal.
        const bool reanchorOk =
            Localization.reanchorAtWall(info->axis, info->wall_coord, (float) info->facing);

        fsm.tickReanchor(kResetNode, /*wallStopTimedOut*/ false, reanchorOk);

        check(!fsm.advisorySurfaced, "TC10b: a clean stall-success arrival surfaces NO advisory");
        check(fsm.reanchorCalled && fsm.departDispatched && fsm.state == RState::WALL_DEPART,
              "TC10b: a clean success re-anchors and dispatches the depart drive (WALL_DEPART)");

        // Drive the depart to completion -> clearOffset -> PLAN_NEXT_HOP.
        fsm.tickWallDepart(/*driveIsIdle*/ true);
        check(fsm.offsetCleared && fsm.state == RState::PLAN_NEXT_HOP,
              "TC10b: a clean success departs the wall and continues to PLAN_NEXT_HOP");
        check(fsm.fault == PathTrackerClass::Fault::NONE, "TC10b: a clean success is not a fault");

        // FINAL believed pose lands on the node coordinate (139), not the axle (146).
        const Routes::NodeInfo& n = FieldMap::getNode(kResetNode);
        const Offset2D off        = Localization.getOffsetFromNode();
        check(approx(n.y_cm + off.y, n.y_cm, 0.01f),
              "TC10b: a clean success lands the believed pose on the node coordinate");
    }

    // -------------------------------------------------------------------------------------
    // TC10b strict-reject -> WALL_NOT_SQUARE (AC11). The replica is driven with reanchorOk = false
    // (the value real Localization returns under a strict rejection — proven against real
    // Localization in test_reanchor_at_wall_strict.cpp). Assert the slice routes WALL_NOT_SQUARE and
    // does NOT continue, and re-arms the reset latch.
    // -------------------------------------------------------------------------------------
    {
        resetWorldOnResetNode();

        ResetArrivalMirror fsm;
        fsm.maybeEnterReset(kResetNode);
        fsm.tickWallStop(/*driveIsIdle*/ true);

        // The strict check rejected the squaring -> reanchorAtWall returned false.
        fsm.tickReanchor(kResetNode, /*wallStopTimedOut*/ false, /*reanchorOk*/ false);

        check(fsm.reanchorCalled, "TC10b: a rejected reanchor was still attempted");
        check(fsm.state == RState::FAULT, "TC10b: a strict reject halts the interlude (enters FAULT)");
        check(fsm.fault == PathTrackerClass::Fault::WALL_NOT_SQUARE,
              "TC10b: a strict reject routes Fault::WALL_NOT_SQUARE (AC11)");
        check(!fsm.departDispatched && !fsm.offsetCleared,
              "TC10b: a strict reject faults BEFORE any depart (no depart drive, no clearOffset)");
        check(fsm.resetNode == 0xFF, "TC10b: a WALL_NOT_SQUARE fault re-arms the reset latch");
    }

    // -------------------------------------------------------------------------------------
    // TC10b non-reset node — a non-RESET arrival is a true no-op (the interlude is not entered),
    // so the FSM continues exactly as before (AC5-style "no behaviour change when not applicable").
    // -------------------------------------------------------------------------------------
    {
        check(FieldMap::getNodeType(kNonResetNode) != Routes::NodeType::RESET,
              "precondition: node 0 (Inter1) is NOT a RESET node");

        ResetArrivalMirror fsm;
        const bool entered = fsm.maybeEnterReset(kNonResetNode);
        check(!entered, "TC10b: a non-RESET arrival does NOT enter the interlude");
        check(fsm.state == RState::ARRIVAL && !fsm.interludeEntered && !fsm.reanchorCalled,
              "TC10b: a non-RESET arrival leaves the FSM untouched (no wall-stop, no reanchor)");
    }

    // -------------------------------------------------------------------------------------
    // TC10b one-shot latch — once the interlude has run for this arrival (`_resetNode == cur`), a
    // second _maybeEnterReset for the same node does NOT re-enter (the wallStop fired once).
    // MIRROR: PathTracker.cpp:1178 (_resetNode != cur guard).
    // -------------------------------------------------------------------------------------
    {
        ResetArrivalMirror fsm;
        check(fsm.maybeEnterReset(kResetNode), "TC10b latch: first arrival enters the interlude");
        check(!fsm.maybeEnterReset(kResetNode),
              "TC10b latch: a second tick for the SAME reset node does NOT re-enter (one-shot)");

        // Drive the interlude all the way through WALL_DEPART; the `_resetNode == cur` latch must
        // stay held across the extra depart step so the immediate post-depart return does not
        // re-fire the wallStop for this same node.
        resetWorldOnResetNode();
        const Routes::ResetNode* info = FieldMap::getResetInfo(kResetNode);
        fsm.tickWallStop(/*driveIsIdle*/ true);
        const bool reanchorOk =
            Localization.reanchorAtWall(info->axis, info->wall_coord, (float) info->facing);
        fsm.tickReanchor(kResetNode, /*wallStopTimedOut*/ false, reanchorOk);
        fsm.tickWallDepart(/*driveIsIdle*/ true);
        check(fsm.state == RState::PLAN_NEXT_HOP,
              "TC10b latch: a full success (through WALL_DEPART) reaches PLAN_NEXT_HOP");
        check(!fsm.maybeEnterReset(kResetNode),
              "TC10b latch: the latch holds through WALL_DEPART — no re-entry for the same node");
    }

    if (g_failures == 0)
    {
        std::printf("PATH TRACKER RESET ARRIVAL OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
