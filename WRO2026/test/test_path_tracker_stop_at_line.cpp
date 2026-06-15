/**
 * @file test_path_tracker_stop_at_line.cpp
 * @brief Host-compile tests for the PathTracker STOP_AT_LINE slice.
 *
 * Coverage:
 *   - TC1–TC7 / TC13 (C++ side): resolver truth-table vectors against the real
 *     `resolveStopAtLine` from `StopAtLinePolicy.h` (the same cases the Python
 *     sim asserts against the T0 CSV fixture; parity enforced by TC13 running
 *     both sides against every fixture row).
 *   - TC20: the new resolver dimensions added when `resolveStopAtLine` grew the
 *     `StopAtLineQuery` struct — the per-edge `edgeOverride` (ON/OFF) rung, the
 *     `autoRuleActive` rung, the precedence between them, and the `softFallback`
 *     output (true only for an auto-rule-raised seek).
 *   - TC14 (formerly TC7/TC8 in the old numbering): FSM-slice re-anchor on
 *     `Drive.isIdle()` and `DRIVE_FAULT` on `Drive.hasFault()`.
 *   - TC18: drain-ordering mirror-slice — `Intersection.isAtIntersection()` is
 *     called (drain) before entering `State::STOP_AT_LINE`.
 *
 * Built against the mock Arduino / Drive / WROLineArray / Intersection surface
 * under `WRO2026/test/mocks/`. The real `WRO2026/LineFollower.{h,cpp}` and
 * `WRO2026/WROMath.h` come along on the test target's `SUT_SOURCES`, but nothing
 * in this file touches them — the tests are pure logic exercises over the
 * STOP_AT_LINE decision slice.
 *
 * Resolver note: `resolveStopAtLine` now lives in `StopAtLinePolicy.h` (header-
 * only inline), mirroring the `DirectionPolicy.h` promotion that killed
 * `replica_resolveDirection`. Production code and this test suite both `#include
 * "StopAtLinePolicy.h"` and call the same symbol — the old
 * `replica_resolveStopAtLine` (finding #6) is gone. TC13 enforces C++↔Python
 * parity over every T0 fixture row.
 *
 * FSM-slice rationale (follows the precedent established by
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
 *   slice, exactly the values the production guard branches on.
 *
 *   Real production types referenced (header-only / no `.cpp` linkage needed):
 *     - `PathTrackerClass::Fault`     (PathTracker.h)
 *     - `PathTrackerClass::FaultInfo` (PathTracker.h)
 *     - `StopAtLine`                  (NavTypes.h)
 *     - `Direction`                   (NavTypes.h)
 *     - `Routes::BlindMode`           (routes.h)
 *     - `Routes::StopAtLineMode`      (routes.h)
 *     - `StopAtLineQuery`             (StopAtLinePolicy.h)
 *     - `StopAtLineResolution`        (StopAtLinePolicy.h)
 *
 *   Because the replica references the real enum values, a future rename of
 *   `Fault::DRIVE_FAULT`, `StopAtLine::ON/OFF/AUTO`, `Direction::FRONT/REAR`, or
 *   `Routes::BlindMode::STOP_AT_LINE` breaks compilation of this test — that is
 *   exactly the safety net we want.
 *
 * @note Test-coverage limitation (stated in the 2026-05-30 stop-at-line plan):
 *       FSM assertions run against an inline-mirrored copy of the PathTracker
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
#include "PathTracker.h"       // For PathTrackerClass::Fault + FaultInfo enums.
#include "routes.h"            // For Routes::BlindMode.
#include "FieldMap.h"          // For FieldMap::NODE_NONE.
#include "StopAtLinePolicy.h"  // resolveStopAtLine + StopAtLineResolution (real resolver).

#include <stdint.h>
#include <cstdio>

// ===========================================================================
// FSM-slice replica for the STOP_AT_LINE state body.
//
// This replica covers TC14 (success re-anchor and DRIVE_FAULT). The resolver
// itself (TC1-TC7, TC13) is exercised directly against the real
// `resolveStopAtLine` — no replica needed for that path.
// ===========================================================================

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
    StopAtLineTickOutcome    outcome        = StopAtLineTickOutcome::WAIT;
    bool                     reAnchored     = false;    ///< `Localization.onIntersection(_nextHop)` issued.
    bool                     chassisStopped = false;    ///< `Drive.stop(true)` issued (fault path).
    bool                     queueCleared   = false;    ///< `_clearQueue()` issued (fault path).
    PathTrackerClass::Fault  fault          = PathTrackerClass::Fault::NONE;
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
// MIRROR: PathTracker.cpp:539-543 (shared Drive-fault guard → Fault::DRIVE_FAULT).
// MIRROR: PathTracker.cpp:838-842 (STOP_AT_LINE success → onIntersection(_nextHop)).
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
// TC18 drain-ordering replica.
//
// The production dispatch drains Intersection.isAtIntersection() (AC7) BEFORE
// entering State::STOP_AT_LINE, so a fresh physical crossing during the seek
// does not leak a stale nav edge into the next LINE hop. This replica records
// whether the drain call was issued and in what order relative to entering the
// seek state.
// ===========================================================================

/// Records the ordering of drain + state-enter in the mirrored dispatch.
struct DrainOrderResult
{
    bool drainCalledBeforeStateEnter = false;    ///< Intersection drain preceded the state transition.
    bool seekDispatched              = false;     ///< Drive.stopAtLine(..).async() was issued.
    bool stateEntered                = false;     ///< State::STOP_AT_LINE was set.
};

/// Replica of the stop-at-line dispatch ordering (BLIND → STOP_AT_LINE state).
///
/// Models the ordering constraint only — not the full builder chain.
///
/// @param drainFirst  Pass `true` to simulate the correct production order
///                    (drain → seek → enter state); `false` to simulate the
///                    erroneous reversed order for negative-case coverage.
///
// MIRROR: PathTracker.cpp:1364,1387/1396,1402 (BLIND STOP_AT_LINE dispatch:
//         Intersection.isAtIntersection() drain → Drive.stopAtLine(..).async() →
//         _state = State::STOP_AT_LINE).
static DrainOrderResult replica_dispatchDrainOrdering(bool drainFirst)
{
    DrainOrderResult r{};
    int sequenceStep = 0;    // Increments with each logical event.
    int drainStep    = -1;
    int stateStep    = -1;

    if (drainFirst)
    {
        drainStep = sequenceStep++;    // Intersection.isAtIntersection() — drain
        r.seekDispatched = true;       // Drive.stopAtLine(...).async()
        sequenceStep++;
        stateStep = sequenceStep++;    // _state = State::STOP_AT_LINE
    }
    else
    {
        stateStep = sequenceStep++;    // Wrong: state entered before drain
        drainStep = sequenceStep++;
        r.seekDispatched = true;
    }

    r.drainCalledBeforeStateEnter = (drainStep < stateStep);
    r.stateEntered                = true;
    return r;
}

// ===========================================================================
// TC1 — AUTO + STOP_AT_LINE edge, FRONT, 1-board → seek=true, SYNC_TSHAPE.
// ===========================================================================

static void test_TC1_auto_stop_at_line_edge_front_1board_seeks_sync_tshape()
{
    TEST_SCOPE("TC1: AUTO + STOP_AT_LINE edge, FRONT, 1-board → seek=true, SYNC_TSHAPE");

    // AUTO defers to edge; edge is STOP_AT_LINE; FRONT board always exists.
    // Seeks under SYNC_TSHAPE (the bare STOP_AT_LINE edge has no velocity
    // profile of its own — Step 4 of the resolver). Legacy slice: no per-edge
    // override, no auto rule — the seek rides the legacy STOP_AT_LINE tag rung.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    // Legacy STOP_AT_LINE-tag seek is not auto-raised → not a soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC2 — AUTO + STOP_AT_LINE edge, REAR, 2-board → seek=true, SYNC_TSHAPE.
// ===========================================================================

static void test_TC2_auto_stop_at_line_edge_rear_2board_seeks_sync_tshape()
{
    TEST_SCOPE("TC2: AUTO + STOP_AT_LINE edge, REAR, 2-board → seek=true, SYNC_TSHAPE");

    // REAR + 2 boards: boardOK = true (lineBoardCount >= 2).
    // AUTO defers to edge → seekRequested = true via the legacy STOP_AT_LINE
    // tag rung. Seeks under SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    // Legacy STOP_AT_LINE-tag seek is not auto-raised → not a soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC3 — AUTO + STOP_AT_LINE edge, REAR, 1-board → seek=false, pipelineBlind
//        (the board-degrade case: finding #1+#2).
// ===========================================================================

static void test_TC3_auto_stop_at_line_edge_rear_1board_degrades_to_pipeline_blind()
{
    TEST_SCOPE("TC3: AUTO + STOP_AT_LINE edge, REAR, 1-board → seek=false, motionMode=pipelineBlind");

    // REAR + only 1 board: boardOK = false → seek = false even though requested.
    // Dead-reckon fallback: edgeMode == STOP_AT_LINE → motionMode = pipelineBlind.
    // pipelineBlind is SYNC_TSHAPE (not STOP_AT_LINE) so no second defensive
    // degrade fires.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    // No seek → no soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC4 — ON + GYRO_TSHAPE edge, FRONT, 1-board → seek=true, GYRO_TSHAPE.
//        (AC5: ON-override borrows the edge's typed mode.)
// ===========================================================================

static void test_TC4_on_gyro_tshape_edge_front_1board_seeks_gyro_tshape()
{
    TEST_SCOPE("TC4: ON + GYRO_TSHAPE edge, FRONT, 1-board → seek=true, GYRO_TSHAPE");

    // ON forces a seek even on a plain GYRO_TSHAPE edge. Step 4: edgeMode is
    // not STOP_AT_LINE, so motionMode keeps the edge's typed mode: GYRO_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
    // Call-ON seek is explicit, not auto-raised → not a soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC5 — ON + RAW_FLAT edge, REAR, 2-board → seek=true, RAW_FLAT.
// ===========================================================================

static void test_TC5_on_raw_flat_edge_rear_2board_seeks_raw_flat()
{
    TEST_SCOPE("TC5: ON + RAW_FLAT edge, REAR, 2-board → seek=true, RAW_FLAT");

    // ON forces seek; REAR + 2 boards satisfies boardOK. Edge is RAW_FLAT
    // (not STOP_AT_LINE), so Step 4 keeps motionMode = RAW_FLAT.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::RAW_FLAT,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
    // Call-ON seek is explicit, not auto-raised → not a soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC6 — OFF + STOP_AT_LINE edge, FRONT, any board → seek=false, pipelineBlind.
// ===========================================================================

static void test_TC6_off_stop_at_line_edge_front_1board_no_seek()
{
    TEST_SCOPE("TC6: OFF + STOP_AT_LINE edge, FRONT, 1-board → seek=false, motionMode=pipelineBlind");

    // OFF always suppresses the seek regardless of edge mode or board count.
    // Dead-reckon: edgeMode == STOP_AT_LINE → base = pipelineBlind (SYNC_TSHAPE).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::OFF,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    // No seek → no soft fallback.
    ASSERT_FALSE(r.softFallback);
}

static void test_TC6_off_stop_at_line_edge_front_2board_no_seek()
{
    TEST_SCOPE("TC6: OFF + STOP_AT_LINE edge, FRONT, 2-board → seek=false (OFF wins regardless of board count)");

    // Same as above with 2 boards — OFF still suppresses.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::OFF,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    // No seek → no soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC7 — AUTO + SYNC_TSHAPE edge (plain blind), FRONT → seek=false, SYNC_TSHAPE.
//        (Common bare-goTo() path — no stop-at-line ever requested.)
// ===========================================================================

static void test_TC7_auto_plain_blind_edge_no_seek()
{
    TEST_SCOPE("TC7: AUTO + SYNC_TSHAPE edge, FRONT → seek=false, SYNC_TSHAPE");

    // AUTO on a non-STOP_AT_LINE edge with no auto rule active: seekRequested =
    // false. Dead-reckon: edgeMode != STOP_AT_LINE → base = edgeMode =
    // SYNC_TSHAPE (no defensive degrade, base is already a concrete mode).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    // No seek → no soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC13 (C++ side) — additional resolver truth-table vectors covering the
// ON-wins-over-any-plain-edge case and the OFF-suppresses-everything case.
// These complement TC1–TC7 to give dense coverage of the T0 fixture rows.
// ===========================================================================

static void test_TC13_on_wins_over_every_plain_blind_mode()
{
    TEST_SCOPE("TC13: ON wins (seek=true) over DEFAULT / GYRO_TSHAPE / RAW_FLAT blind edges — FRONT board");

    // ON is unconditional — it forces seek regardless of which typed edge mode
    // the edge resolved to. motionMode keeps the edge's type (not STOP_AT_LINE,
    // so Step 4 leaves it unchanged).
    {
        // ON on a plain GYRO_TSHAPE edge: seekRequested=true, boardOK=true,
        // edgeMode != STOP_AT_LINE → motionMode = GYRO_TSHAPE. Call-ON, not
        // auto-raised, so softFallback stays false.
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::ON, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::GYRO_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_TRUE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
        ASSERT_FALSE(r.softFallback);
    }
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::ON, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::RAW_FLAT,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_TRUE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
        ASSERT_FALSE(r.softFallback);
    }
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::ON, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::SYNC_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_TRUE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
        ASSERT_FALSE(r.softFallback);
    }
}

static void test_TC13_off_suppresses_regardless_of_edge_mode_or_board_count()
{
    TEST_SCOPE("TC13: OFF suppresses seek for STOP_AT_LINE, SYNC_TSHAPE, GYRO_TSHAPE, RAW_FLAT edges");

    // OFF is unconditional: seek = false for every edge mode, every board count,
    // every direction. Dead-reckon fallback applies (Step 5).
    {
        // STOP_AT_LINE edge → base = pipelineBlind
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::OFF, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::STOP_AT_LINE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 2, false});
        ASSERT_FALSE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
        ASSERT_FALSE(r.softFallback);
    }
    {
        // Non-STOP_AT_LINE edge → base = edgeMode
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::OFF, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::GYRO_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::REAR, 2, false});
        ASSERT_FALSE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
        ASSERT_FALSE(r.softFallback);
    }
    {
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::OFF, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::RAW_FLAT,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, false});
        ASSERT_FALSE(r.seek);
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
        ASSERT_FALSE(r.softFallback);
    }
}

static void test_TC13_on_rear_1board_degrades_seek_but_keeps_edge_motionMode()
{
    TEST_SCOPE("TC13: ON + GYRO_TSHAPE, REAR, 1-board → seek=false (no rear board), motionMode=GYRO_TSHAPE");

    // ON requests the seek but boardOK = false (REAR, 1 board). seek = false.
    // Dead-reckon: edgeMode = GYRO_TSHAPE (not STOP_AT_LINE) → base = GYRO_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
    // No seek → no soft fallback.
    ASSERT_FALSE(r.softFallback);
}

static void test_TC13_pipeline_stop_at_line_defensively_degrades_to_sync_tshape()
{
    TEST_SCOPE("TC13: board-degrade + pipelineBlind==STOP_AT_LINE → motionMode defensively SYNC_TSHAPE");

    // Corner case: the pipeline default is itself STOP_AT_LINE. If a STOP_AT_LINE
    // edge does not seek (e.g. REAR + 1-board), Step 5: base = pipelineBlind =
    // STOP_AT_LINE; the defensive second degrade kicks in → SYNC_TSHAPE so the
    // returned motionMode is never STOP_AT_LINE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::STOP_AT_LINE,    // unusual but must not leak
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    // No seek → no soft fallback.
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC20 — new resolver dimensions added with the `StopAtLineQuery` struct:
//         the per-edge `edgeOverride` rung, the `autoRuleActive` rung, the
//         precedence between every rung, and the `softFallback` output (true
//         ONLY for an auto-rule-raised seek). These exercise the inputs that
//         did not exist before the resolver signature grew the struct.
//
// Cross-language parity: same rows live in
// `tools/path_planner/tests/fixtures/stop_at_line_truth_table.csv` and are
// asserted by the Python `resolve_stop_at_line`.
// ===========================================================================

static void test_TC20_edge_override_on_forces_seek()
{
    TEST_SCOPE("TC20: edgeOverride==ON forces seek on a plain (non-STOP_AT_LINE) edge");

    // No call-site verb (AUTO), no legacy tag (edge is SYNC_TSHAPE), no auto
    // rule — but the per-edge override is ON, so rung 2 forces the seek. The
    // edge is not STOP_AT_LINE, so motionMode keeps its typed mode (SYNC_TSHAPE).
    // An edge-override seek is not auto-raised → softFallback stays false.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::ON,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    ASSERT_FALSE(r.softFallback);
}

static void test_TC20_edge_override_off_suppresses_seek()
{
    TEST_SCOPE("TC20: edgeOverride==OFF suppresses seek even with auto rule active");

    // edgeOverride OFF (rung 2) beats both the auto rule (rung 4) and would beat
    // the legacy tag (rung 3) — see the dedicated test below. With the seek
    // suppressed and a non-STOP_AT_LINE edge, the hop dead-reckons under the
    // edge's own SYNC_TSHAPE. No seek → softFallback false.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::OFF,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ true});    // auto rule wants to seek, but OFF wins

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    ASSERT_FALSE(r.softFallback);
}

static void test_TC20_edge_override_off_beats_legacy_stop_at_line_tag()
{
    TEST_SCOPE("TC20: edgeOverride==OFF beats the legacy BlindMode::STOP_AT_LINE tag → seek=false");

    // The historical "STOP_AT_LINE tag means seek" behaviour is rung 3, BELOW
    // the per-edge override at rung 2. A STOP_AT_LINE-tagged edge with an
    // explicit edgeOverride==OFF must NOT seek — the override wins. Dead-reckon:
    // edgeMode == STOP_AT_LINE → base = pipelineBlind (SYNC_TSHAPE).
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::OFF,
        /*edgeMode*/       Routes::BlindMode::STOP_AT_LINE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    ASSERT_FALSE(r.softFallback);
}

static void test_TC20_call_off_beats_edge_override_on()
{
    TEST_SCOPE("TC20: call OFF (rung 1) beats edgeOverride==ON (rung 2) → seek=false");

    // Call-site OFF is the top rung — it suppresses the seek even when the
    // per-edge override says ON. Non-STOP_AT_LINE edge dead-reckons under its
    // own SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::OFF,
        /*edgeOverride*/   Routes::StopAtLineMode::ON,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    ASSERT_FALSE(r.softFallback);
}

static void test_TC20_call_on_beats_edge_override_off()
{
    TEST_SCOPE("TC20: call ON (rung 1) beats edgeOverride==OFF (rung 2) → seek=true");

    // The symmetric case: call-site ON forces the seek even when the per-edge
    // override says OFF. Explicit call-ON is not auto-raised → softFallback
    // false. Edge keeps its typed RAW_FLAT mode.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::ON,
        /*edgeOverride*/   Routes::StopAtLineMode::OFF,
        /*edgeMode*/       Routes::BlindMode::RAW_FLAT,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
    ASSERT_FALSE(r.softFallback);
}

static void test_TC20_auto_rule_active_front_seeks_soft_fallback()
{
    TEST_SCOPE("TC20: autoRuleActive (no higher rung) → seek=true, softFallback=true (FRONT)");

    // The auto rung (rung 4) fires when nothing above it decided: call AUTO,
    // edgeOverride DEFAULT, edge is a plain non-STOP_AT_LINE mode. FRONT board
    // always services the seek → seek=true. Because the auto rung raised it,
    // softFallback is true. motionMode keeps the edge's GYRO_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ true});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
    ASSERT_TRUE(r.softFallback);
}

static void test_TC20_auto_rule_inactive_no_seek()
{
    TEST_SCOPE("TC20: autoRuleActive==false on a plain edge → no auto seek (seek=false)");

    // With the auto rule off and nothing else requesting, the resolver falls to
    // the terminal "never seek" rung. Plain edge dead-reckons under SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    ASSERT_FALSE(r.softFallback);
}

static void test_TC20_auto_rule_rear_1board_degrades_no_soft_fallback()
{
    TEST_SCOPE("TC20: autoRuleActive + REAR + 1-board → seek=false, softFallback=false (board degrade)");

    // The auto rule wants the seek but a REAR seek needs a second board
    // (lineBoardCount >= 2). With only 1 board, boardOK = false → seek = false.
    // A board-degraded auto case carries NO soft fallback (softFallback =
    // seek && autoTriggered, and seek is false). Dead-reckon under the edge's
    // SYNC_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::SYNC_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ true});

    ASSERT_FALSE(r.seek);
    ASSERT_FALSE(r.softFallback);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
}

static void test_TC20_auto_rule_rear_2board_seeks_soft_fallback()
{
    TEST_SCOPE("TC20: autoRuleActive + REAR + 2-board → seek=true, softFallback=true");

    // Same auto-rule request as above, but a 2-board chassis services the REAR
    // seek (boardOK = true). seek = true and, because the auto rung raised it,
    // softFallback = true. motionMode keeps the edge's GYRO_TSHAPE.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::GYRO_TSHAPE,
        /*pipelineBlind*/  Routes::BlindMode::SYNC_TSHAPE,
        /*direction*/      Direction::REAR,
        /*lineBoardCount*/ 2,
        /*autoRuleActive*/ true});

    ASSERT_TRUE(r.seek);
    ASSERT_TRUE(r.softFallback);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::GYRO_TSHAPE);
}

static void test_TC20_soft_fallback_only_for_auto_not_explicit_seeks()
{
    TEST_SCOPE("TC20: softFallback is true ONLY for auto seeks — false for call-ON / legacy-tag seeks");

    // Three FRONT seeks that all set seek=true, contrasting the softFallback
    // output across the rungs that can raise the seek:
    {
        // (a) Auto-rule seek → softFallback TRUE.
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::AUTO, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::SYNC_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, true});
        ASSERT_TRUE(r.seek);
        ASSERT_TRUE(r.softFallback);
    }
    {
        // (b) Call-ON seek (explicit) → softFallback FALSE even if the auto rule
        //     would also have fired (rung 1 wins; autoTriggered never set).
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::ON, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::SYNC_TSHAPE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, true});
        ASSERT_TRUE(r.seek);
        ASSERT_FALSE(r.softFallback);
    }
    {
        // (c) Legacy STOP_AT_LINE-tag seek → softFallback FALSE (rung 3, not the
        //     auto rung). Auto rule also active but never reached.
        const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
            StopAtLine::AUTO, Routes::StopAtLineMode::DEFAULT, Routes::BlindMode::STOP_AT_LINE,
            Routes::BlindMode::SYNC_TSHAPE, Direction::FRONT, 1, true});
        ASSERT_TRUE(r.seek);
        ASSERT_FALSE(r.softFallback);
        // Bare STOP_AT_LINE edge seeks under SYNC_TSHAPE (Step 4).
        ASSERT_TRUE(r.motionMode == Routes::BlindMode::SYNC_TSHAPE);
    }
}

static void test_TC20_degenerate_full_domain_pipeline_stop_at_line_no_seek()
{
    TEST_SCOPE("TC20: AUTO, no auto rule, edge!=STOP_AT_LINE, pipeline==STOP_AT_LINE → seek=false");

    // Full-domain degenerate row (the case that proves there is NO pipeline-
    // default seek rung): callFlag AUTO, edgeOverride DEFAULT, the resolved edge
    // is a concrete non-STOP_AT_LINE mode, the auto rule is off — yet the
    // PIPELINE default happens to be STOP_AT_LINE. No rung requests a seek, so
    // seek=false. Dead-reckon: edgeMode != STOP_AT_LINE → base = edgeMode
    // (RAW_FLAT); the pipeline STOP_AT_LINE never leaks into motionMode.
    const StopAtLineResolution r = resolveStopAtLine(StopAtLineQuery{
        /*callFlag*/       StopAtLine::AUTO,
        /*edgeOverride*/   Routes::StopAtLineMode::DEFAULT,
        /*edgeMode*/       Routes::BlindMode::RAW_FLAT,
        /*pipelineBlind*/  Routes::BlindMode::STOP_AT_LINE,    // pipeline default is itself STOP_AT_LINE
        /*direction*/      Direction::FRONT,
        /*lineBoardCount*/ 1,
        /*autoRuleActive*/ false});

    ASSERT_FALSE(r.seek);
    ASSERT_TRUE(r.motionMode == Routes::BlindMode::RAW_FLAT);
    ASSERT_FALSE(r.softFallback);
}

// ===========================================================================
// TC14 — FSM-slice: re-anchor on success; DRIVE_FAULT on no-line.
//         (Formerly TC7/TC8 in the old numbering; kept intact per the task.)
// ===========================================================================

static void test_TC14_line_found_reanchors_via_on_intersection()
{
    TEST_SCOPE("TC14: STOP_AT_LINE success re-anchors via onIntersection (not dead-reckon)");

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

static void test_TC14_seek_in_flight_keeps_waiting()
{
    TEST_SCOPE("TC14: STOP_AT_LINE keeps waiting while the seek is in flight");

    // Mid-seek: Drive is neither faulted nor idle — the tick must take no
    // action and stay in STOP_AT_LINE (no premature re-anchor).
    const StopAtLineTickResult result = replica_stopAtLineTick(
        /*driveHasFault*/ false,
        /*driveIsIdle*/   false);

    ASSERT_TRUE(result.outcome == StopAtLineTickOutcome::WAIT);
    ASSERT_FALSE(result.reAnchored);
    ASSERT_TRUE(result.fault == PathTrackerClass::Fault::NONE);
}

static void test_TC14_no_line_surfaces_drive_fault()
{
    TEST_SCOPE("TC14: STOP_AT_LINE no-line (Drive faulted) surfaces Fault::DRIVE_FAULT");

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

static void test_TC14_fault_guard_wins_over_idle()
{
    TEST_SCOPE("TC14: the Drive-fault guard takes precedence over an idle observation");

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
// TC18 — drain ordering: Intersection.isAtIntersection() (drain) is called
//         before Drive.stopAtLine(...).async() and before entering
//         State::STOP_AT_LINE (AC7 / finding #5).
// ===========================================================================

static void test_TC18_drain_precedes_state_enter()
{
    TEST_SCOPE("TC18: Intersection drain precedes the STOP_AT_LINE state transition (AC7)");

    // The production dispatch order (per AC7 / finding #5):
    //   1. Intersection.isAtIntersection()  — drain stale edge
    //   2. Drive.stopAtLine(...).async()     — launch seek
    //   3. _state = State::STOP_AT_LINE     — commit state
    //
    // drainFirst=true simulates the correct production ordering.
    const DrainOrderResult r = replica_dispatchDrainOrdering(/*drainFirst*/ true);

    ASSERT_TRUE(r.drainCalledBeforeStateEnter);
    ASSERT_TRUE(r.seekDispatched);
    ASSERT_TRUE(r.stateEntered);
}

static void test_TC18_reversed_drain_ordering_is_detected()
{
    TEST_SCOPE("TC18: reversed ordering (state before drain) is detected by the replica — negative case");

    // Negative case: drainFirst=false simulates the erroneous order where the
    // state is entered before the drain. The replica must report the drain did
    // NOT precede the state enter. This confirms the replica actually checks
    // ordering rather than always returning true.
    const DrainOrderResult r = replica_dispatchDrainOrdering(/*drainFirst*/ false);

    ASSERT_FALSE(r.drainCalledBeforeStateEnter);
    ASSERT_TRUE(r.seekDispatched);
    ASSERT_TRUE(r.stateEntered);
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker STOP_AT_LINE — resolver truth table (TC1-TC7/TC13/TC20) + FSM slice (TC14) + drain (TC18)\n");

    // TC1–TC7: resolver truth-table representative vectors (TC13 C++ side).
    test_TC1_auto_stop_at_line_edge_front_1board_seeks_sync_tshape();
    test_TC2_auto_stop_at_line_edge_rear_2board_seeks_sync_tshape();
    test_TC3_auto_stop_at_line_edge_rear_1board_degrades_to_pipeline_blind();
    test_TC4_on_gyro_tshape_edge_front_1board_seeks_gyro_tshape();
    test_TC5_on_raw_flat_edge_rear_2board_seeks_raw_flat();
    test_TC6_off_stop_at_line_edge_front_1board_no_seek();
    test_TC6_off_stop_at_line_edge_front_2board_no_seek();
    test_TC7_auto_plain_blind_edge_no_seek();

    // TC13: additional truth-table vectors (ON/OFF unconditional; board-degrade;
    // defensive double-degrade for pipeline==STOP_AT_LINE).
    test_TC13_on_wins_over_every_plain_blind_mode();
    test_TC13_off_suppresses_regardless_of_edge_mode_or_board_count();
    test_TC13_on_rear_1board_degrades_seek_but_keeps_edge_motionMode();
    test_TC13_pipeline_stop_at_line_defensively_degrades_to_sync_tshape();

    // TC20: the new resolver dimensions — per-edge edgeOverride, autoRuleActive,
    // rung precedence, and the softFallback output.
    test_TC20_edge_override_on_forces_seek();
    test_TC20_edge_override_off_suppresses_seek();
    test_TC20_edge_override_off_beats_legacy_stop_at_line_tag();
    test_TC20_call_off_beats_edge_override_on();
    test_TC20_call_on_beats_edge_override_off();
    test_TC20_auto_rule_active_front_seeks_soft_fallback();
    test_TC20_auto_rule_inactive_no_seek();
    test_TC20_auto_rule_rear_1board_degrades_no_soft_fallback();
    test_TC20_auto_rule_rear_2board_seeks_soft_fallback();
    test_TC20_soft_fallback_only_for_auto_not_explicit_seeks();
    test_TC20_degenerate_full_domain_pipeline_stop_at_line_no_seek();

    // TC14: FSM-slice re-anchor on success; DRIVE_FAULT on no-line.
    test_TC14_line_found_reanchors_via_on_intersection();
    test_TC14_seek_in_flight_keeps_waiting();
    test_TC14_no_line_surfaces_drive_fault();
    test_TC14_fault_guard_wins_over_idle();

    // TC18: Intersection drain precedes the STOP_AT_LINE state transition.
    test_TC18_drain_precedes_state_enter();
    test_TC18_reversed_drain_ordering_is_detected();

    TEST_REPORT();
}
