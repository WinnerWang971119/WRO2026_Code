/**
 * @file test_path_tracker_stop_at_line.cpp
 * @brief Host-compile tests for the PathTracker STOP_AT_LINE slice
 *        — TC7 (re-anchor on success), TC8 (DRIVE_FAULT on no-line),
 *        TC9 / TC10 (call-flag OFF/ON precedence over the per-edge BlindMode).
 *
 * Built against the mock Arduino / Drive / WROLineArray / Intersection surface
 * under `WRO2026/test/mocks/`. The real `WRO2026/LineFollower.{h,cpp}` and
 * `WRO2026/WROMath.h` come along on the test target's `SUT_SOURCES`, but nothing
 * in this file touches them — the tests are pure logic exercises over the
 * mirrored STOP_AT_LINE decision slice.
 *
 * Mirroring rationale (follows the precedent established by
 * `test_path_tracker_passthrough.cpp` and `test_path_tracker_bidirectional.cpp`):
 *
 *   Linking the production `PathTracker.cpp` would drag in the full Drive
 *   fluent-builder + Localization + FieldMap + routes.cpp + MatrixMiniR4 +
 *   EventTrace surface. Instead, this file **replicates the small slice of
 *   `PathTracker.cpp` under test** inline, then asserts behaviour on the
 *   replicated copy. Every mirrored decision is annotated
 *   `// MIRROR: PathTracker.cpp:<line>` with a back-pointer to the canonical
 *   site so the two cannot silently drift — a reviewer diffs the replica
 *   against the cited lines.
 *
 *   The harness deliberately does NOT extend the mock `Drive.h` with a fault
 *   surface (no `hasFault()` / `Fault::LINE_NOT_FOUND`). The production
 *   STOP_AT_LINE state reads `Drive.hasFault()` / `Drive.isIdle()`; here those
 *   two observations are modelled as plain `bool` inputs to the replicated
 *   slice, exactly the values the production guard branches on. This keeps the
 *   mock honest (it only models what `LineFollower.cpp` actually calls) while
 *   still covering the FSM decision behaviorally.
 *
 *   Real production types referenced (header-only / no `.cpp` linkage needed):
 *     - `PathTrackerClass::Fault`     (PathTracker.h)
 *     - `PathTrackerClass::FaultInfo` (PathTracker.h)
 *     - `StopAtLine`                  (NavTypes.h)
 *     - `Routes::BlindMode`           (routes.h)
 *
 *   Because the replica references the real enum values, a future rename of
 *   `Fault::DRIVE_FAULT`, `StopAtLine::ON/OFF/AUTO`, or
 *   `Routes::BlindMode::STOP_AT_LINE` breaks compilation of this test — that is
 *   exactly the safety net we want.
 *
 * @note Test-coverage limitation (stated in the 2026-05-30 stop-at-line plan):
 *       these assertions run against an inline-mirrored copy of the PathTracker
 *       slice that can diverge from production. The real
 *       `Drive._updateStopAtLine` stateful behaviour and the production
 *       `Localization.onIntersection(_nextHop)` call are covered only by code
 *       review plus the Phase-3 hardware bench (B3.7), NOT by this harness.
 */

#include "TestAssert.h"

#include "Arduino.h"
#include "Drive.h"
#include "WROLineArray.h"
#include "Intersection.h"

// Production headers we depend on for types only (no linkage to .cpp needed).
#include "Config.h"
#include "NavTypes.h"
#include "PathTracker.h"    // For PathTrackerClass::Fault + FaultInfo enums.
#include "routes.h"         // For Routes::BlindMode.
#include "FieldMap.h"       // For FieldMap::NODE_NONE.

#include <stdint.h>
#include <cstdio>

// ===========================================================================
// Replicated STOP_AT_LINE decision slice.
//
// Three independent decisions live in the production BLIND dispatch + the
// STOP_AT_LINE state. Each is replicated as a tiny pure function annotated with
// the canonical `PathTracker.cpp` site it mirrors.
// ===========================================================================

/// Replica of the effective stop-at-line resolution at the top of the BLIND
/// case in `PathTrackerClass::_dispatchEdgeAfterTurn`.
///
/// Precedence (mirrors `setArrivalDirection`): the call-site `StopAtLine` flag
/// — cached at DEQUEUE from the in-flight goal's NavEntry — wins over the
/// per-edge resolved `BlindMode`:
///   - ON   → stop-at-line, even on a plain (DEFAULT-resolved) blind edge.
///   - OFF  → never stop-at-line, even when the edge resolves to STOP_AT_LINE.
///   - AUTO → defer to the edge: stop-at-line iff the resolved BlindMode is
///            STOP_AT_LINE.
///
/// @param callFlag        The cached `_activeStopAtLine` value for the in-flight goal.
/// @param resolvedBlindMode The per-edge BlindMode after `_resolveBlindMode(edge)`.
/// @return `true` when the hop dispatches the `Drive.stopAtLine(...)` seek;
///         `false` when it falls through to the dead-reckoned blind branch.
///
// MIRROR: PathTracker.cpp:985-994 (BLIND case stop-at-line precedence switch).
static bool replica_resolveStopAtLine(StopAtLine callFlag, Routes::BlindMode resolvedBlindMode)
{
    bool stopAtLine;
    switch (callFlag)
    {
        case StopAtLine::ON:   stopAtLine = true;  break;
        case StopAtLine::OFF:  stopAtLine = false; break;
        case StopAtLine::AUTO:
        default:
            stopAtLine = (resolvedBlindMode == Routes::BlindMode::STOP_AT_LINE);
            break;
    }
    return stopAtLine;
}

/// Outcome of one `STOP_AT_LINE` FSM tick, for the replicated state body.
enum class StopAtLineTickOutcome : uint8_t
{
    WAIT,           ///< Seek still in flight — Drive neither idle nor faulted.
    RE_ANCHOR,      ///< Drive idle (line found) — advance via onIntersection().
    DRIVE_FAULT,    ///< Drive faulted (no line) — surface Fault::DRIVE_FAULT.
};

/// Records what a replicated `STOP_AT_LINE` FSM tick decided, mirroring the
/// side effects of the production state body.
struct StopAtLineTickResult
{
    StopAtLineTickOutcome    outcome      = StopAtLineTickOutcome::WAIT;
    bool                     reAnchored   = false;    ///< `Localization.onIntersection(_nextHop)` issued.
    bool                     chassisStopped = false;  ///< `Drive.stop(true)` issued (fault path).
    bool                     queueCleared = false;    ///< `_clearQueue()` issued (fault path).
    PathTrackerClass::Fault  fault        = PathTrackerClass::Fault::NONE;
};

/// Replica of one `PathTrackerClass::update()` tick while in `State::STOP_AT_LINE`.
///
/// Two production sites collapse into this one replicated tick:
///   1. The shared Drive-fault guard at the TOP of `update()` — it fires for
///      ANY non-IDLE/non-FAULT state when `Drive.hasFault()`, stops the
///      chassis, and enters `Fault::DRIVE_FAULT` (which also clears the queue
///      via `_enterFault`). STOP_AT_LINE relies on this guard for its no-line
///      fault path (no dead-reckon fallback, ADR-027 / AC7).
///   2. The `State::STOP_AT_LINE` success body — on `Drive.isIdle()` it
///      re-anchors via `Localization.onIntersection(_nextHop)` (a true physical
///      re-localisation, unlike BLIND_FORWARD's `onDeadReckonedAdvance`).
///
/// The production code reads `Drive.hasFault()` / `Drive.isIdle()`; we pass
/// those two observations as parameters (the mock Drive has no fault surface by
/// design — see the file docblock).
///
/// @param driveHasFault The value `Drive.hasFault()` would return this tick.
/// @param driveIsIdle   The value `Drive.isIdle()` would return this tick.
/// @return The decision the production tick would take, with the side-effect
///         flags it would issue.
///
// MIRROR: PathTracker.cpp:415-419 (shared Drive-fault guard → Fault::DRIVE_FAULT).
// MIRROR: PathTracker.cpp:659-663 (STOP_AT_LINE success → onIntersection(_nextHop)).
static StopAtLineTickResult replica_stopAtLineTick(bool driveHasFault, bool driveIsIdle)
{
    StopAtLineTickResult result{};

    // (1) Shared Drive-fault guard at the top of update(). When the seek runs
    //     the full max distance without a crossing, Drive latches
    //     Fault::LINE_NOT_FOUND and Drive.hasFault() goes true.
    if (driveHasFault)
    {
        result.outcome        = StopAtLineTickOutcome::DRIVE_FAULT;
        result.chassisStopped = true;    // Drive.stop(true)
        result.queueCleared   = true;    // _enterFault → _clearQueue()
        result.fault          = PathTrackerClass::Fault::DRIVE_FAULT;
        return result;
    }

    // (2) Success body — the seek detected the crossing, rolled the axle onto
    //     it, and active-braked. Advance the node with onIntersection(...) so
    //     this hop RE-ANCHORS the discrete position on the real line.
    if (driveIsIdle)
    {
        result.outcome    = StopAtLineTickOutcome::RE_ANCHOR;
        result.reAnchored = true;        // Localization.onIntersection(_nextHop)
        return result;
    }

    // Neither faulted nor idle — the seek is still in flight; keep waiting.
    return result;
}

// ===========================================================================
// TC7 — line-found path re-anchors via the mirrored onIntersection call.
// ===========================================================================

static void test_TC7_line_found_reanchors_via_on_intersection()
{
    TEST_SCOPE("TC7: STOP_AT_LINE success re-anchors via onIntersection (not dead-reckon)");

    // Drive finished the seek cleanly: not faulted, now idle (axle braked on
    // the line). The production tick advances via onIntersection(_nextHop) and
    // transitions back to PLAN_NEXT_HOP.
    const StopAtLineTickResult result = replica_stopAtLineTick(
        /*driveHasFault*/ false,
        /*driveIsIdle*/   true);

    ASSERT_TRUE(result.outcome == StopAtLineTickOutcome::RE_ANCHOR);
    ASSERT_TRUE(result.reAnchored);

    // The success path is a clean re-anchor — no fault, no chassis stop, no
    // queue clear (those belong to the no-line fault path only).
    ASSERT_TRUE(result.fault == PathTrackerClass::Fault::NONE);
    ASSERT_FALSE(result.chassisStopped);
    ASSERT_FALSE(result.queueCleared);
}

static void test_TC7_seek_in_flight_keeps_waiting()
{
    TEST_SCOPE("TC7: STOP_AT_LINE keeps waiting while the seek is in flight");

    // Mid-seek: Drive is neither faulted nor idle — the tick must take no
    // action and stay in STOP_AT_LINE (no premature re-anchor).
    const StopAtLineTickResult result = replica_stopAtLineTick(
        /*driveHasFault*/ false,
        /*driveIsIdle*/   false);

    ASSERT_TRUE(result.outcome == StopAtLineTickOutcome::WAIT);
    ASSERT_FALSE(result.reAnchored);
    ASSERT_TRUE(result.fault == PathTrackerClass::Fault::NONE);
}

// ===========================================================================
// TC8 — Drive reports LINE_NOT_FOUND → PathTracker surfaces DRIVE_FAULT.
// ===========================================================================

static void test_TC8_no_line_surfaces_drive_fault()
{
    TEST_SCOPE("TC8: STOP_AT_LINE no-line (Drive faulted) surfaces Fault::DRIVE_FAULT");

    // Drive searched the full max distance with no crossing and latched
    // Fault::LINE_NOT_FOUND, so Drive.hasFault() is true. The shared guard at
    // the top of update() stops the chassis and enters Fault::DRIVE_FAULT.
    const StopAtLineTickResult result = replica_stopAtLineTick(
        /*driveHasFault*/ true,
        /*driveIsIdle*/   false);

    ASSERT_TRUE(result.outcome == StopAtLineTickOutcome::DRIVE_FAULT);
    ASSERT_TRUE(result.fault == PathTrackerClass::Fault::DRIVE_FAULT);

    // The fault path records the stop + queue-clear the production
    // `_enterFault(DRIVE_FAULT, ...)` issues. No silent dead-reckon fallback.
    ASSERT_TRUE(result.chassisStopped);
    ASSERT_TRUE(result.queueCleared);
    ASSERT_FALSE(result.reAnchored);
}

static void test_TC8_fault_guard_wins_over_idle()
{
    TEST_SCOPE("TC8: the Drive-fault guard takes precedence over an idle observation");

    // Defensive: even if Drive were observed both faulted AND idle on the same
    // tick, the fault guard runs FIRST at the top of update() and short-circuits
    // before the success body — so the outcome is DRIVE_FAULT, never RE_ANCHOR.
    const StopAtLineTickResult result = replica_stopAtLineTick(
        /*driveHasFault*/ true,
        /*driveIsIdle*/   true);

    ASSERT_TRUE(result.outcome == StopAtLineTickOutcome::DRIVE_FAULT);
    ASSERT_FALSE(result.reAnchored);
}

// ===========================================================================
// TC9 — setStopAtLine(OFF) on a STOP_AT_LINE edge resolves to a normal blind move.
// ===========================================================================

static void test_TC9_off_flag_suppresses_stop_at_line_edge()
{
    TEST_SCOPE("TC9: setStopAtLine(OFF) on a STOP_AT_LINE edge → normal blind move");

    // The edge resolves to STOP_AT_LINE, but the call-site flag forces OFF.
    // OFF wins: the hop runs as a normal dead-reckoned blind move (no seek).
    const bool stopAtLine = replica_resolveStopAtLine(
        /*callFlag*/          StopAtLine::OFF,
        /*resolvedBlindMode*/ Routes::BlindMode::STOP_AT_LINE);

    ASSERT_FALSE(stopAtLine);
}

static void test_TC9_auto_flag_defers_to_stop_at_line_edge()
{
    TEST_SCOPE("TC9: setStopAtLine(AUTO) on a STOP_AT_LINE edge → stop-at-line (defers to edge)");

    // AUTO defers to the edge — a STOP_AT_LINE edge dispatches the seek.
    const bool stopAtLine = replica_resolveStopAtLine(
        /*callFlag*/          StopAtLine::AUTO,
        /*resolvedBlindMode*/ Routes::BlindMode::STOP_AT_LINE);

    ASSERT_TRUE(stopAtLine);
}

// ===========================================================================
// TC10 — setStopAtLine(ON) on a plain (DEFAULT-resolved) blind edge dispatches the seek.
// ===========================================================================

static void test_TC10_on_flag_forces_stop_at_line_on_plain_edge()
{
    TEST_SCOPE("TC10: setStopAtLine(ON) on a plain SYNC_TSHAPE blind edge → stop-at-line");

    // The edge resolves to a plain blind mode (SYNC_TSHAPE — the historical
    // dead-reckon profile), but the call-site flag forces ON. ON wins: the hop
    // dispatches the Drive.stopAtLine(...) seek even though the edge never asked
    // for it.
    const bool stopAtLine = replica_resolveStopAtLine(
        /*callFlag*/          StopAtLine::ON,
        /*resolvedBlindMode*/ Routes::BlindMode::SYNC_TSHAPE);

    ASSERT_TRUE(stopAtLine);
}

static void test_TC10_auto_flag_defers_to_plain_edge()
{
    TEST_SCOPE("TC10: setStopAtLine(AUTO) on a plain blind edge → normal blind move (defers to edge)");

    // AUTO on a non-STOP_AT_LINE edge defers to the edge → no seek. Covers the
    // common bare-goTo() path where the call flag was never set (DEQUEUE
    // defaults `_activeStopAtLine` to AUTO).
    const bool stopAtLine = replica_resolveStopAtLine(
        /*callFlag*/          StopAtLine::AUTO,
        /*resolvedBlindMode*/ Routes::BlindMode::SYNC_TSHAPE);

    ASSERT_FALSE(stopAtLine);
}

static void test_TC10_on_flag_overrides_every_plain_blind_mode()
{
    TEST_SCOPE("TC10: setStopAtLine(ON) wins over DEFAULT / GYRO_TSHAPE / RAW_FLAT edges too");

    // ON wins regardless of which concrete dead-reckon mode the edge resolved
    // to — the precedence is unconditional, not specific to SYNC_TSHAPE.
    ASSERT_TRUE(replica_resolveStopAtLine(StopAtLine::ON, Routes::BlindMode::DEFAULT));
    ASSERT_TRUE(replica_resolveStopAtLine(StopAtLine::ON, Routes::BlindMode::GYRO_TSHAPE));
    ASSERT_TRUE(replica_resolveStopAtLine(StopAtLine::ON, Routes::BlindMode::RAW_FLAT));

    // And OFF suppresses regardless of edge mode, symmetrically.
    ASSERT_FALSE(replica_resolveStopAtLine(StopAtLine::OFF, Routes::BlindMode::DEFAULT));
    ASSERT_FALSE(replica_resolveStopAtLine(StopAtLine::OFF, Routes::BlindMode::STOP_AT_LINE));
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker STOP_AT_LINE mirror-slice (TC7-TC10)\n");

    // TC7 — re-anchor on success.
    test_TC7_line_found_reanchors_via_on_intersection();
    test_TC7_seek_in_flight_keeps_waiting();

    // TC8 — DRIVE_FAULT on no-line.
    test_TC8_no_line_surfaces_drive_fault();
    test_TC8_fault_guard_wins_over_idle();

    // TC9 — OFF precedence.
    test_TC9_off_flag_suppresses_stop_at_line_edge();
    test_TC9_auto_flag_defers_to_stop_at_line_edge();

    // TC10 — ON precedence.
    test_TC10_on_flag_forces_stop_at_line_on_plain_edge();
    test_TC10_auto_flag_defers_to_plain_edge();
    test_TC10_on_flag_overrides_every_plain_blind_mode();

    TEST_REPORT();
}
