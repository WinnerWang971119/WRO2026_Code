/**
 * @file test_path_tracker_parity_dump.cpp
 * @brief Sim<->firmware FSM parity DUMP target (issue #108, T6 / AC4).
 * @author WRO 2026
 *
 * @details
 * Drives the REAL `PathTracker` + `Drive` + `Localization` + `YawSource` singletons over the
 * feature-complete `fixtures/routes_fixture.cpp` and prints the normalized per-hop semantic
 * trace as VERSIONED NDJSON on stdout. The Python parity test (T8) invokes this binary, runs
 * the Python `PathTrackerSim` over the IDENTICAL case set, and diffs the two streams. This is a
 * DUMP binary, not a ctest — it is built + run by the dedicated `parity` CI job and the parity
 * pytest (`test_pathtracker_parity.py`); it carries NO `add_test(...)` (it still gets a compile
 * check from `cmake --build`).
 *
 * ===========================================================================================
 * THE GENERIC PER-TICK DRIVER — reads observed FSM state, never re-derives (the load-bearing
 * design / harness invariant "never re-implement an FSM decision")
 * ===========================================================================================
 * Each `goTo` is driven ONE STATE AT A TIME, dispatched off `PathTracker.state()` (the public
 * read-only accessor added by T1). For every state the driver feeds the SINGLE observable the
 * FSM is actually waiting on this tick and advances the cooperative tick loop until the FSM
 * LEAVES that state. It NEVER recomputes the hop's type, heading, distance, or mode in the test
 * — turn targets come from `PathTracker.resolvedBodyHeadingDeg()`, and the only graph DATA reads
 * are to script mocks (the reset node's `facing` for the SQUARE turn), never to predict an FSM
 * decision. Re-deriving a decision in the harness would make it score its own prediction and
 * silently pass while C++ and the sim disagree — the exact drift #108 exists to catch.
 *
 * Each per-state handler bounds its work by `state() == <thatState>` (NOT by `Drive.isIdle()`):
 * the moment the FSM transitions, the inner loop exits and the outer loop re-reads the new
 * state. This is what keeps a multi-hop sweep correct — the FSM dispatches the NEXT hop's move
 * inside the SAME tick the current one retires, so a naive "tick until Drive idle" loop would
 * over-drive into the next hop. Bounding on the observed state cannot over-drive.
 *
 * The PER-HOP trace records (`turn` / `line_follow` / `line_overshoot` / `projection_follow` /
 * `stop_at_line_seek` / `blind_dead_reckon` / `return_turn` / `return_move` / `wall_stop` /
 * `wall_reanchor` / `wall_depart`) are emitted by T1's `PT_PARITY_TRACE`-guarded hook from
 * INSIDE the FSM at its commit points — this file does NOT re-emit them. It emits ONLY the
 * `goal_start` / `goal_done` framing records (it alone knows the case_id / start / goal).
 *
 * ===========================================================================================
 * Enumeration (MUST match the Python side in T8 exactly — a count mismatch is a hard fail)
 * ===========================================================================================
 *  1. ALL-PAIRS: every ordered (start, goal) node pair over the fixture with `start != goal`
 *     and a reachable route, SORTED by (start_name, goal_name). Each is driven from a clean
 *     `seatAt(start)` (zero operator offset, so the return-to-node prologue never fires).
 *     `case_id = "<start>__<goal>__clean"`.
 *  2. CURATED_PARITY_CASES: a small pinned table guaranteeing the special features are hit
 *     (reset interlude, both pivot regimes, both pass-through kinds, a REAR hop) plus the
 *     pinned-offset return-to-node cases (a clean `seatAt` clears the offset, so all-pairs can
 *     never exercise RETURN_TO_NODE). `case_id = "<start>__<goal>__<tag>"`. Every curated tag is
 *     a distinct, non-"clean" suffix so a curated case_id never collides with an all-pairs one
 *     even when the (start, goal) pair coincides — the two sides stay count-identical.
 *
 * stdout is PURE NDJSON (one object per line). All human diagnostics + the harness ASSERT
 * machinery write to stderr (`TestAssert.h` already routes failures to stderr; the mock `Serial`
 * captures into an in-process buffer, never stdout), so T8's parser sees only NDJSON.
 *
 * ===========================================================================================
 * REALIZED-OFFSET EMISSION (finding 1, issue #116)
 * ===========================================================================================
 * The two offset cases manufacture a real operator offset through the harness
 * `pushOperatorOffset(...)`, which deposits via the real `Localization` encoder integrator in
 * `path_harness::kOffsetPushStepCm` (= 2 cm) increments. The achieved magnitude is therefore a
 * multiple of 2 cm along an axis-aligned heading and passes through a cm->encoder-degrees->cm
 * round-trip, so it differs slightly from the Python `operator_move` value. The firmware's
 * commanded `return_move.dist_cm` is a direct function of THIS realized offset, so to compare it
 * honestly `runCuratedCase` captures the magnitude `pushOperatorOffset` RETURNS and emits it as
 * the `realized_offset_cm` field on every `goal_start` record. The Python parity test seeds its
 * sim with that realized value (not the nominal table magnitude), which collapses the
 * `return_move.dist_cm` divergence to within `DIST_TOL` — so NO allowlist entry is needed and the
 * former `return-move-offset-encoder-quantization` entry has been removed.
 */

#include "test_path_tracker_harness.h"    // real singletons + per-hop expect* helpers + cooperativeTick

#include "src/mission/PathTrackerParityTrace.h"       // pt_parity::emit (PT_PARITY_TRACE is defined for this target)
#include "src/util/WROMath.h"                       // cmToEncDeg — sizes the generic seek/return encoder steps

#include <algorithm>    // std::sort — name-sorted all-pairs enumeration
#include <cmath>        // lroundf
#include <cstdio>       // std::snprintf
#include <cstring>      // std::strcmp
#include <stdint.h>

namespace
{
/// @brief Convenience alias for the now-public per-edge FSM state enum (issue #108, AC12).
using State = PathTrackerClass::State;

#ifndef PARITY_SCHEMA_VERSION
#define PARITY_SCHEMA_VERSION 2    // defensive default; the CMake target defines it to 2.
#endif

// ===========================================================================================
// Driver tuning.
// ===========================================================================================

/// @brief Outer-loop budget (state-handler iterations) per `goTo`. Each iteration retires one
///        FSM state (or one bounded sub-FSM). The longest fixture path is ~17 hops; with a
///        DEQUEUE + optional pre-turn + move per hop plus up to two 3-phase reset interludes
///        that is ~60 iterations. 2000 clears that with >30x margin and still fails a never-
///        settling FSM as a budget-exhausted assert rather than hanging the dump.
constexpr int kCaseStateBudget = 2000;

/// @brief Encoder advance per generic SEEK / RETURN retire tick (small, ~5 cm). NOT the big
///        `path_harness::kEncoderAdvanceStep` (20000 deg): a seek has a max-distance fault gate,
///        so a big over-advance would overrun `STOP_AT_LINE_MAX_MARGIN_CM` and mis-fault. 5 cm
///        is well below the 25 cm-wide arm window (ARM_WINDOW 10 + MAX_MARGIN 15), so the seek
///        always arms before the max gate, then the held `T_OR_CROSS` debounces and overshoots.
constexpr float kSeekStepCm = 5.0f;

// ===========================================================================================
// CURATED_PARITY_CASES — the pinned special-feature + offset cases (T8 mirrors this table and
// the EXACT offset application below; the C++ and Python case sets must be identical).
//
// Each entry is (start, goal, tag, hasOffset, offsetMagCm, offsetHeadingDeg). For an offset
// case the driver does, after resetWorld():
//     seatAt(start, offsetHeadingDeg);            // bind frame at the push heading
//     pushOperatorOffset(offsetMagCm, offsetHeadingDeg);   // manufacture a REAL operator offset
//     PathTracker.goTo(goal).async();             // DEQUEUE sees the offset -> RETURN_TO_NODE
// For a non-offset (special) case it is a clean seatAt(start) (app-yaw 0) + goTo(goal).
// `case_id = "<start>__<goal>__<tag>"`.
//
// Specials pinned (each verified against routes_fixture; routes shown for the reader):
//   reset_wall1  position1 -> Wall1   : position1->Wall1 (BLIND) arrives at a RESET node -> the
//                                       wall-zero interlude (wall_stop/reanchor/depart) fires.
//   reset_wall2  final     -> Wall2   : final->Wall2 (BLIND) arrives at the second RESET node.
//   pivot_line   Inter3    -> Corner  : Inter3->Inter4->Corner; pivot at Inter4 (LINE regime,
//                                       PivotTurn (5,6,1) PIVOT_LEFT) — overshoot-trimmed turn.
//   pivot_sensor Corner    -> Zed     : Corner->ProjB->Zed; seek into ProjB then pivot at ProjB
//                                       (sensor regime, PivotTurn (1,7,15) PIVOT_RIGHT) — the
//                                       seek overshoot is δ-trimmed and the depart pre-turn pivots.
//   pass_lll     Inter2    -> Inter4  : Inter2->Inter3->Inter4, collinear LINE-LINE-LINE
//                                       pass-through (line_overshoot pass_through=true at Inter3).
//   pass_lpl     Inter1    -> Inter2  : Inter1->ProjL->Inter2, collinear LINE-PROJECTION-LINE
//                                       pass-through (projection_follow pass_through=true at ProjL).
//   rear         Inter1    -> Spur    : the FORCE_REVERSE LINE edge — a REAR (reverse=true) hop.
//
// Offset return-to-node cases (pinned operator offset before the goTo so RETURN_TO_NODE fires):
//   off_n14_h0  Inter2 -> Inter3, +14 cm along app-heading 0 : Inter2 is a LINE anchor and
//               14 cm >= LINE_SENSOR_AXLE_OFFSET_CM (12.02), so the return is a LINE SEEK
//               (return_move mode="seek"); return heading resolves to 180 (atan2(-0,-14)).
//   off_n6_h0   Zed -> ProjB, +6 cm along app-heading 0      : Zed is a BLIND anchor, so the
//               return is a DEAD-RECKON drive (return_move mode="forward"); return heading 180.
//
// Both push along app-heading 0: `seatAt(...)` binds the world frame to `START_HEADING_DEG` (= 0)
// via `zeroHeadingFromCurrent`, so the believed heading is 0 and the integrator deposits the
// offset along world +x regardless of the parked raw yaw. A non-zero push heading would either
// land on the wrong axis (if it disagreed with the seat yaw) or introduce a yaw offset that
// breaks the generic park-at-resolved-heading retire — so heading 0 is the one clean choice here.
// Magnitudes are multiples of 2.0 cm (the integrator deposit step). The ACHIEVED (realized)
// magnitude `pushOperatorOffset` returns is emitted as `realized_offset_cm` on the goal_start
// record so the sim is seeded with it (not the nominal magnitude) — see the REALIZED-OFFSET
// EMISSION note in the file docblock.
// ===========================================================================================
struct CuratedCase
{
    const char* start;
    const char* goal;
    const char* tag;
    bool        hasOffset;
    float       offsetMagCm;
    int16_t     offsetHeadingDeg;
};

constexpr CuratedCase CURATED_PARITY_CASES[] = {
    // start        goal       tag             hasOffset  magCm  headingDeg
    {"position1", "Wall1", "reset_wall1", false, 0.0f, 0},
    {"final", "Wall2", "reset_wall2", false, 0.0f, 0},
    {"Inter3", "Corner", "pivot_line", false, 0.0f, 0},
    {"Corner", "Zed", "pivot_sensor", false, 0.0f, 0},
    {"Inter2", "Inter4", "pass_lll", false, 0.0f, 0},
    {"Inter1", "Inter2", "pass_lpl", false, 0.0f, 0},
    {"Inter1", "Spur", "rear", false, 0.0f, 0},
    {"Inter2", "Inter3", "off_n14_h0", true, 14.0f, 0},
    {"Zed", "ProjB", "off_n6_h0", true, 6.0f, 0},
};
constexpr size_t kNumCuratedCases = sizeof(CURATED_PARITY_CASES) / sizeof(CURATED_PARITY_CASES[0]);

// ===========================================================================================
// Framing-record emit (the only records THIS file emits; the per-hop records come from T1's
// in-FSM hook). The goal_start record carries one float (the realized operator-offset magnitude,
// %.6f); node names are emitted verbatim.
// ===========================================================================================

/// @brief Emit the per-case `goal_start` framing record (carries the schema version + the realized
///        operator-offset magnitude).
///
/// @param realizedOffsetCm The ACHIEVED (encoder-quantized) operator-offset magnitude for this
///        case (`pushOperatorOffset`'s return value), or `0.0f` for clean / non-offset cases. The
///        Python parity test seeds its sim with this so `return_move.dist_cm` is compared honestly
///        rather than masked by an allowlist entry (finding 1, issue #116).
void emitGoalStart(const char* caseId, const char* startName, const char* goalName,
                   float realizedOffsetCm)
{
    char line[320];
    std::snprintf(line, sizeof(line),
                  "{\"schema_version\":%d,\"kind\":\"goal_start\",\"case_id\":\"%s\","
                  "\"start\":\"%s\",\"goal\":\"%s\",\"route_dir\":\"AUTO\",\"arrival_dir\":\"AUTO\","
                  "\"realized_offset_cm\":%.6f}",
                  (int) (PARITY_SCHEMA_VERSION), caseId, startName, goalName,
                  (double) realizedOffsetCm);
    pt_parity::emit(line);
}

/// @brief Emit the per-case `goal_done` framing record (the node the FSM actually arrived on).
void emitGoalDone(const char* finalName)
{
    char line[256];
    std::snprintf(line, sizeof(line), "{\"kind\":\"goal_done\",\"final\":\"%s\"}", finalName);
    pt_parity::emit(line);
}

// ===========================================================================================
// Per-state observable feeders. Each parks the IMU yaw at the FSM's own resolved body heading
// (so a non-trivial pre-turn settles, gotcha-pathtracker-host-test-preturn-park-yaw) and feeds
// the single observable the FSM polls in that state, bounded by `state() == <thatState>`.
// ===========================================================================================

/// @brief Normalize a heading (deg) to the firmware's `[0, 360)` body-heading convention.
///
/// @note The return-to-node prologue sets `_resolvedBodyHeading` from `atan2(...)`, which yields
///       a value in `(-180, 180]` (e.g. -180 for a due-west return) that the FSM does NOT
///       re-wrap to `[0, 360)`, whereas a normal hop's `_resolvedBodyHeading` is the edge heading
///       already in `[0, 360)`. `Drive`'s turn target goes through `Localization::toImuYaw`, which
///       DOES `_normalize` to `[0, 360)`, so the chassis must be parked at the normalized heading
///       (a raw -180 vs +180 sign flip is the difference between settling and spinning forever).
///       Normalizing here makes the return turn settle EXACTLY like an all-pairs 180° edge turn
///       (which is already proven to settle) and is a no-op for every already-normalized hop.
inline int16_t normHeadingDeg(float deg)
{
    long d = lroundf(deg) % 360;
    if (d < 0)
    {
        d += 360;
    }
    return (int16_t) d;
}

/// @brief Park the mock IMU at the FSM's resolved body heading for the in-flight hop/return.
inline void parkYawAtResolvedHeading()
{
    mmL.mockYawDeg = (double) hwYaw(normHeadingDeg(PathTracker.resolvedBodyHeadingDeg()));
}

/// @brief Settle an in-flight pre-edge / return TURN: park yaw at the resolved body heading each
///        tick until the FSM leaves TURNING (the turn brakes, Drive idles, the edge/return move
///        is dispatched and the state advances within the SAME tick).
void driveTurn()
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && PathTracker.state() == State::TURNING
                    && !PathTracker.hasFault();
         ++i)
    {
        parkYawAtResolvedHeading();
        cooperativeTick();
    }
}

/// @brief Retire a BLIND dead-reckon move: over-advance the encoder + hold the done-bit (covers
///        both the profiled and flat regimes, CONTRACT 1) each tick until the FSM leaves
///        BLIND_FORWARD (onDeadReckonedAdvance -> PLAN_NEXT_HOP).
void driveBlind()
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() == State::BLIND_FORWARD && !PathTracker.hasFault();
         ++i)
    {
        parkYawAtResolvedHeading();
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
}

/// @brief Retire a STOP_AT_LINE seek the GENERIC way (cannot pass a per-hop `cm` without
///        re-deriving the edge): each tick park yaw, hold `T_OR_CROSS` (ignored during SEEK,
///        debounced once ARMED), advance the encoder a SMALL step so the arm window is hit
///        before the max gate, until the FSM leaves STOP_AT_LINE (success -> onIntersection ->
///        PLAN_NEXT_HOP). Always the SUCCESS path — host-unreachable fault branches are T7-listed.
void driveSeek()
{
    const int32_t step = (int32_t) cmToEncDeg(kSeekStepCm);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() == State::STOP_AT_LINE && !PathTracker.hasFault();
         ++i)
    {
        parkYawAtResolvedHeading();
        LineArray.mockSetJunctionType(JunctionType::T_OR_CROSS);
        MiniR4.M2.mockAdvanceDegrees(step);
        MiniR4.M3.mockAdvanceDegrees(step);
        cooperativeTick();
    }
}

/// @brief Retire the return-to-node prologue move — generic over BOTH a LINE line-seek return
///        and a BLIND dead-reckon return: hold `T_OR_CROSS` (for the seek) AND advance the
///        encoder a small step + hold the done-bit (for the dead-reckon `driveDist`), until the
///        FSM leaves RETURN_TO_NODE. Harmless for the dead-reckon case (`driveDist` ignores the
///        line array) and for the seek case (the done-bit is irrelevant while the seek owns the
///        move). Small step so the line-seek return arms before its own max gate.
void driveReturn()
{
    const int32_t step = (int32_t) cmToEncDeg(kSeekStepCm);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() == State::RETURN_TO_NODE && !PathTracker.hasFault();
         ++i)
    {
        parkYawAtResolvedHeading();
        LineArray.mockSetJunctionType(JunctionType::T_OR_CROSS);
        MiniR4.M2.mockAdvanceDegrees(step);
        MiniR4.M3.mockAdvanceDegrees(step);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
}

/// @brief Retire the wall-zero reset's WALL_DEPART move (a GYRO `driveDist` forward off the
///        wall, back onto the node): over-advance the encoder + hold the done-bit, parking yaw at
///        the reset node's `facing` (the squared heading the SQUARE turn left), until the FSM
///        leaves WALL_DEPART (clearOffset -> PLAN_NEXT_HOP).
void driveWallDepart()
{
    const Routes::ResetNode* info = FieldMap::getResetInfo(Localization.getCurrentNode());
    const int16_t            facing = (info != nullptr) ? info->facing : (int16_t) 0;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() == State::WALL_DEPART && !PathTracker.hasFault();
         ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(facing);
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
}

// ===========================================================================================
// The generic driver: run one queued goal to completion, dispatching off the observed state.
// ===========================================================================================

/// @brief Drive the in-flight `goTo` to IDLE (goal reached) or a latched fault, then return.
///        Bounded by `kCaseStateBudget` so a stuck FSM fails as an assert, never a hang.
void driveGoalToCompletion()
{
    int it = 0;
    for (; it < kCaseStateBudget && !PathTracker.isIdle() && !PathTracker.hasFault(); ++it)
    {
        switch (PathTracker.state())
        {
            case State::TURNING:
                driveTurn();
                break;

            case State::LINE_FOLLOW:
                // expectLineHop drives the junction-detect arm + the axle-overshoot gate in a
                // FIXED 2-tick sequence (no Drive-idle loop), so it never over-drives into the
                // next hop. Re-entered per LINE segment (incl. each pass-through leg).
                expectLineHop();
                break;

            case State::LINE_OVERSHOOT:
                // Defensive: normally retired inside expectLineHop's second gate, but handle a
                // standalone observation (trip the distance gate, advance to re-anchor).
                LineFollower.mockSetDistanceReached(true);
                cooperativeTick();
                break;

            case State::PROJECTION_FOLLOW:
                // The DISTANCE gate is the only trigger at a marker-less projection node (the
                // pass-through fires off it too). Trip it; one tick advances/continues.
                LineFollower.mockSetDistanceReached(true);
                cooperativeTick();
                break;

            case State::BLIND_FORWARD:
                driveBlind();
                break;

            case State::STOP_AT_LINE:
                driveSeek();
                break;

            case State::RETURN_TO_NODE:
                driveReturn();
                break;

            case State::WALL_STOP:
            {
                // Drive the SQUARE -> PUSH -> BACK_OFF -> DONE wall push to idle via the harness
                // helper, which exits at state==REANCHOR (the WALL_STOP handler transitions on
                // Drive idle, one-per-tick, so the helper's Drive-idle loop does not over-drive
                // into the REANCHOR-dispatched depart move). `facing` is graph DATA used ONLY to
                // script the SQUARE turn, not to predict an FSM decision (allowed).
                const Routes::ResetNode* info =
                    FieldMap::getResetInfo(Localization.getCurrentNode());
                const int16_t facing = (info != nullptr) ? info->facing : (int16_t) 0;
                expectWallStop(facing, /*timedOut*/ false);
                break;
            }

            case State::REANCHOR:
                // One tick: REANCHOR re-anchors, emits wall_reanchor + wall_depart, dispatches
                // the depart driveDist, and transitions to WALL_DEPART (or PLAN_NEXT_HOP if the
                // degenerate zero-standoff branch is taken).
                cooperativeTick();
                break;

            case State::WALL_DEPART:
                driveWallDepart();
                break;

            case State::IDLE:
            case State::DEQUEUE:
            case State::PLAN_NEXT_HOP:
            case State::FAULT:
            default:
                // Transitional / terminal: advance one tick (DEQUEUE evaluates the offset gate;
                // PLAN_NEXT_HOP resolves + dispatches the next hop or reaches the goal).
                cooperativeTick();
                break;
        }
    }

    ASSERT_TRUE(it < kCaseStateBudget);    // a never-settling FSM fails HERE, not as a hang.
}

// ===========================================================================================
// Case runners.
// ===========================================================================================

/// @brief Run one clean all-pairs case: seat at @p start (app-yaw 0, offset cleared), goTo
///        @p goal, drive to completion, frame the trace with goal_start / goal_done.
void runCleanCase(uint8_t start, uint8_t goal)
{
    const char* startName = FieldMap::getNode(start).name;
    const char* goalName  = FieldMap::getNode(goal).name;

    char caseId[96];
    std::snprintf(caseId, sizeof(caseId), "%s__%s__clean", startName, goalName);
    TEST_SCOPE(caseId);

    resetWorld();
    seatAt(startName);    // app-yaw 0 — declarePosition clears the offset, so no return prologue.

    emitGoalStart(caseId, startName, goalName, 0.0f);    // clean case: no operator offset.
    PathTracker.goTo(goal).async();
    driveGoalToCompletion();
    emitGoalDone(FieldMap::getNode(Localization.getCurrentNode()).name);
}

/// @brief Run one curated case (a pinned special, optionally with a manufactured operator
///        offset so the RETURN_TO_NODE prologue fires).
void runCuratedCase(const CuratedCase& c)
{
    const uint8_t start = nodeByName(c.start);
    const uint8_t goal  = nodeByName(c.goal);

    char caseId[96];
    std::snprintf(caseId, sizeof(caseId), "%s__%s__%s", c.start, c.goal, c.tag);
    TEST_SCOPE(caseId);

    // Loud pre-flight: a renamed/typo'd curated node or a fixture re-weight that drops the route
    // fails HERE rather than driving an un-armed FSM into a misleading divergence.
    ASSERT_TRUE(start != FieldMap::NODE_NONE);
    ASSERT_TRUE(goal != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNextHop(start, goal) != FieldMap::NODE_NONE);
    if (start == FieldMap::NODE_NONE || goal == FieldMap::NODE_NONE
        || FieldMap::getNextHop(start, goal) == FieldMap::NODE_NONE)
    {
        return;
    }

    resetWorld();
    float realized = 0.0f;    // realized (encoder-quantized) offset magnitude; 0 for a non-offset case.
    if (c.hasOffset)
    {
        // Bind the frame at the push heading, then manufacture a REAL operator offset through the
        // Localization integrator so DEQUEUE's return-gate sees a genuine offset (NOT a test hook).
        // `pushOperatorOffset` RETURNS the achieved magnitude — emitted as `realized_offset_cm` so
        // the sim seeds with it and `return_move.dist_cm` is compared honestly (finding 1, #116).
        seatAt(c.start, c.offsetHeadingDeg);
        realized = pushOperatorOffset(c.offsetMagCm, c.offsetHeadingDeg);
    }
    else
    {
        seatAt(c.start);    // app-yaw 0.
    }

    emitGoalStart(caseId, c.start, c.goal, realized);
    PathTracker.goTo(goal).async();
    driveGoalToCompletion();
    emitGoalDone(FieldMap::getNode(Localization.getCurrentNode()).name);
}

}    // namespace

// ===========================================================================================
// Entry — stdout is PURE NDJSON; the banner + summary go to stderr.
// ===========================================================================================

int main()
{
    std::fprintf(stderr,
                 "INFO: PathTracker sim<->firmware parity DUMP (issue #108, T6) — REAL "
                 "PathTracker + Drive + Localization + YawSource over routes_fixture; "
                 "schema_version=%d; NDJSON on stdout\n",
                 (int) (PARITY_SCHEMA_VERSION));

    // ---- All-pairs, sorted by (start_name, goal_name) so the two sides enumerate identically. ----
    uint8_t order[FieldMap::N_NODES];
    for (uint8_t i = 0; i < FieldMap::N_NODES; ++i)
    {
        order[i] = i;
    }
    std::sort(order, order + FieldMap::N_NODES, [](uint8_t a, uint8_t b) {
        return std::strcmp(FieldMap::getNode(a).name, FieldMap::getNode(b).name) < 0;
    });

    size_t nAllPairs = 0;
    for (uint8_t si = 0; si < FieldMap::N_NODES; ++si)
    {
        for (uint8_t gi = 0; gi < FieldMap::N_NODES; ++gi)
        {
            const uint8_t start = order[si];
            const uint8_t goal  = order[gi];
            if (start == goal)
            {
                continue;
            }
            if (FieldMap::getNextHop(start, goal) == FieldMap::NODE_NONE)
            {
                continue;    // unreachable pair — skip (deterministic; the sim skips it too).
            }
            runCleanCase(start, goal);
            ++nAllPairs;
        }
    }

    // ---- Curated pins (specials + pinned-offset return-to-node cases). ----
    for (size_t i = 0; i < kNumCuratedCases; ++i)
    {
        runCuratedCase(CURATED_PARITY_CASES[i]);
    }

    std::fprintf(stderr, "INFO: emitted %zu cases (%zu all-pairs + %zu curated)\n",
                 nAllPairs + kNumCuratedCases, nAllPairs, kNumCuratedCases);

    // DUMP binary: stdout already carries the NDJSON. Signal a non-zero exit ONLY if a harness
    // assertion failed (a stuck FSM / a bad curated pin) so the parity pytest sees a broken dump.
    if (wro_test::failCounter() != 0)
    {
        std::fprintf(stderr, "RESULT: %d/%d harness assertions failed during the dump\n",
                     wro_test::failCounter(), wro_test::assertCounter());
        return 1;
    }
    return 0;
}
