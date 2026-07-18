/**
 * @file test_path_tracker_harness.h (PATH HOST HARNESS)
 * @brief Shared header-only harness driving the REAL `PathTracker` + `Drive` +
 *        `Localization` + `YawSource` singletons for the five PathTracker host-link
 *        tests (issue #106, T3).
 * @author WRO 2026
 *
 * @details
 * The five rewritten PathTracker host tests (T5a-T5e) `#include` this header. It sits OVER
 * the real singletons and exposes SEMANTIC per-hop helpers so each test body reads as a
 * sequence of "drive this hop, assert the lower-MCU call" steps instead of hand-rolling the
 * per-tick done-bit / sensor handshake the real `Drive` + `PathTracker` FSMs require.
 *
 * THE INVERSION (vs `test_mission_dispatch`, #111): these targets link the REAL `Drive.cpp`
 * + `PathTracker.cpp`, so the PRODUCTION `Drive.h` / `PathTracker.h` are the definitions that
 * compile — there is NO `hal_mocks/Drive.h` and NO PathTracker mock. The trust boundary sits
 * one layer DOWN: the `MatrixMiniR4` facade + the line-sensor leaf (`WROLineArray`) are
 * mocked, and the real FSMs poll those mocks. The real headers are `#include`d FIRST below so
 * they win the include guards.
 *
 * ===========================================================================================
 * CONTRACT 1 — how a Drive move is retired (the single most important thing)
 * ===========================================================================================
 * Real `Drive::isIdle()` == `_isIdle() && _qCount == 0`, where `_isIdle()` ==
 * `_moveDone && _turnDone && _profileState == IDLE && _turnProfileState == IDLE &&
 * _stopState == IDLE && _wallState == IDLE` (Drive.cpp:1803, :465). A bounded move clears
 * `_moveDone`/`_turnDone` at dispatch and they are restored ONLY by `Drive.update()`'s stage-1
 * drain, which polls `MiniR4.DriveDC.isPrevTaskDone()` — but ONLY while no profile / turn-
 * profile / seek / wall-push state machine is in flight (Drive.cpp:109-123). So a move is
 * retired by the regime that owns it:
 *
 *   - **Non-profiled bounded move** (`driveDist().setMode(RAW)`, `RAW_FLAT` blind dead-reckon):
 *     `MoveDegs`/`MoveSyncDegs`/`MoveGyroDegs` set `_moveDone=false`, `_profileState=IDLE`.
 *     RETIRED by `MiniR4.DriveDC.mockSetPrevTaskDone(true)` + a `cooperativeTick()` (the drain
 *     flips `_moveDone`). There is NO `Drive.mockSetIdle` — that knob lives only on the mock
 *     `DriveClass` these targets do NOT link.
 *   - **Profiled (`.tShape()`) bounded move** (`SYNC_TSHAPE`/`GYRO_TSHAPE` blind dead-reckon):
 *     owns `_moveDone` via `_updateProfile`, which compares `abs(M2.getDegrees() -
 *     _profileStartEnc)` against `_profileTotalEnc` (Drive.cpp:714). RETIRED by ADVANCING
 *     `MiniR4.M2` past the profile total, then a tick (the profile flips `_moveDone`). The
 *     done-bit is irrelevant here (the drain is skipped while a profile is in flight).
 *   - **Turn PD controller** (`turnToYaw().setMode(...)` pre-edge turn, ADR-068): owns `_turnDone`
 *     via `_updateTurnProfile`, which runs ONE PD step per tick against `Move(L,R)` until the
 *     corrected IMU yaw read (`YawSource.readYawDeg` -> `mmL.mockYawDeg * IMU_YAW_SIGN`) has sat
 *     within `TURN_TOLERANCE_DEG` of the app-frame target for `Config::TURN_SETTLE_SAMPLES`
 *     CONSECUTIVE ticks — the settle debounce (Drive.cpp:1144-1153), not a single-shot check; one
 *     out-of-band sample resets the counter to 0. RETIRED by parking `mmL.mockYawDeg` at the
 *     hardware-frame value for the app target, then ticking until the debounce is satisfied
 *     (`expectPreTurn` parks ONCE before the loop — the mock field persists across ticks, so the
 *     required consecutive in-band samples accumulate without re-parking each iteration).
 *   - **stopAtLine seek** (BLIND-resolved-to-seek hop, STOP_AT_LINE): owns `_moveDone` via
 *     `_updateStopAtLine`, which reads `abs(M2.getDegrees() - _stopStartEnc)` for `traveled` and
 *     `LineArray.getJunctionType() == JUNCTION_T_OR_CROSS` for detection (Drive.cpp:1217,:1301).
 *     RETIRED (success) by advancing `M2` to the arm window then holding `T_OR_CROSS` for
 *     `STOP_AT_LINE_DETECT_TICKS` ticks + the overshoot roll; or FAULTED (no-line) by advancing
 *     `M2` past `expected + STOP_AT_LINE_MAX_MARGIN_CM` with `getJunctionType()` left NONE, which
 *     latches `DriveClass::Fault::LINE_NOT_FOUND`.
 *
 * To keep each helper robust against the exact profile/encoder math, the distance-terminated
 * helpers advance `MiniR4.M2` by a GENEROUS delta AND set the done-bit, then tick a bounded
 * number of times until `Drive.isIdle()`. Both regimes complete on the SAME observable.
 *
 * ===========================================================================================
 * CONTRACT 2 — the yaw-frame caveat (off-by-sign trap)
 * ===========================================================================================
 * The app world frame is CCW-positive; the STM32 raw frame is CW-positive. `YawSource` bridges
 * them with `Config::IMU_YAW_SIGN` (= -1) at exactly two points (ADR-045):
 *   - STM32 -> app: `readYawDeg` returns `rawYaw * IMU_YAW_SIGN`.
 *   - app -> STM32: `toHwHeading(app)` returns `app * IMU_YAW_SIGN`.
 * Consequences for scripting:
 *   - A GYRO straight-line / seek move records `MiniR4.DriveDC.lastMoveGyroDegsTarget` /
 *     `lastMoveGyroTarget` in the HARDWARE frame: `target == IMU_YAW_SIGN * appHeading`.
 *   - Scripted IMU yaw (`mmL.mockYawDeg`) is ALSO the raw/hw frame: the FSM reads it
 *     back through `YawSource` which multiplies by `IMU_YAW_SIGN` to recover the app value.
 * `expectPreTurn` takes the APP-frame target and converts via `Config::IMU_YAW_SIGN` internally
 * (parking `mmL.mockYawDeg` at the hw value), so the test reads in the app frame.
 *
 * @note A pre-edge turn dispatches `Drive.turnToYaw(...).setMode(byte).async()`, which runs the
 *       CUSTOM TurnGyro profile (`Move(L,R)`, NOT a lower-MCU `TurnGyro`/`MoveGyroDegs`). So the
 *       turn does NOT stamp `lastMoveGyroDegsTarget`; `expectPreTurn` asserts the turn COMPLETED
 *       (Drive idle, turn-profile back to IDLE) at the scripted app yaw, not a recorded gyro
 *       target. The representative lower-MCU gyro-target assert (CONTRACT 2) belongs to the GYRO
 *       MOVE helpers (`expectBlindHop` GYRO / `expectStopAtLine`), which the test pairs with a
 *       `MiniR4.DriveDC.lastMoveGyroDegsTarget` / `lastMoveGyroTarget` check using
 *       `hwYaw(appHeading)`.
 *
 * ===========================================================================================
 * Production cooperative-tick order (WRO2026.ino::cooperativeTick, line 136)
 * ===========================================================================================
 *   LineArray.update(); Drive.update(); LineFollower.update(); Intersection.update();
 *   Manipulator.update(); Tasks.update(); PathTracker.update(); Localization.update();
 *   Mission.update(); EventTrace.update();
 * `cooperativeTick()` below replays the SUBSET of that order over the modules these targets
 * link — `LineArray`, `Drive`, `LineFollower`, `Intersection`, `PathTracker`, `Localization` —
 * in the SAME relative order (Manipulator/Tasks/Mission/EventTrace are not linked, so they are
 * dropped, not reordered). `Localization.update()` is included because production runs it.
 *
 * ===========================================================================================
 * Fidelity — what this harness deliberately does NOT do
 * ===========================================================================================
 * It scripts observables and ticks the real FSMs; it never re-implements an FSM decision. Time
 * advances ONLY via `mockAdvanceMillis` (never `delay()` — the seek/turn watchdogs are
 * `millis()`-indexed, but the helpers never sleep; they advance the clock explicitly so a
 * watchdog cannot fire mid-handshake). Helpers that need many ticks bound their loop with a
 * fixed budget so a real-FSM regression hangs as a FAILED assert, not an infinite loop.
 */

#ifndef TEST_PATH_TRACKER_HARNESS_H
#define TEST_PATH_TRACKER_HARNESS_H

// -------------------------------------------------------------------------------------------
// Real production headers FIRST (the inversion): these win the DRIVE_H / PATHTRACKER_H /
// LOCALIZATION_H / YAW_SOURCE_H / FIELDMAP_H / CONFIG_H include guards over any mock. Do NOT
// include any hal_mocks/Drive.h or a PathTracker mock — real Drive.h / PathTracker.h must be
// the definitions that compile. Every facade a helper below dereferences (`Drive`,
// `PathTracker`, `Localization`, `YawSource`, `FieldMap`) is therefore a COMPLETE type at the
// point of use, avoiding the MSVC inline-incomplete-type trap (g++ is lenient; MSVC is not).
// -------------------------------------------------------------------------------------------
#include "Config.h"          // real — Config::IMU_YAW_SIGN / STOP_AT_LINE_* / START_NODE etc.
#include "src/drive/Drive.h"           // real — DriveClass + the global `Drive` (defined in real Drive.cpp)
#include "src/nav/FieldMap.h"        // real — getNextHop / getEdge / findNodeByName
#include "src/nav/Localization.h"    // real — declarePosition / onIntersection / zeroHeadingFromCurrent
#include "src/mission/PathTracker.h"     // real — PathTrackerClass + the global `PathTracker`
#include "src/nav/YawSource.h"       // real — YawSource.toHwHeading (static; call as YawSource.method())

// -------------------------------------------------------------------------------------------
// Mock surfaces (resolved by the hal_mocks/ + path_mocks/ -first include path). `<Arduino.h>`
// and `<MatrixMiniR4.h>` are angle-included by the real headers above and pick up the mocks via
// include-path order; pull the path_mocks/ peripheral facades + the test-assert helpers in
// explicitly so this harness is self-contained for the T5 authors.
// -------------------------------------------------------------------------------------------
#include <Arduino.h>           // mock — millis / mockAdvanceMillis / mockResetMillis / g_mockMillis
#include <MatrixMiniR4.h>      // mock — MiniR4 facade + mmL + mockResetMiniR4()
#include "WROLineArray.h"      // mock — LineArray (path_mocks): getJunctionType / setReverseActive
#include "Intersection.h"      // mock — Intersection (path_mocks): isAtIntersection read-and-clear
#include "LineFollower.h"      // mock — LineFollower (path_mocks): arm-cascade record + gates

#include "TestAssert.h"        // ASSERT_* + TEST_SCOPE

#include <math.h>              // sqrtf — operator-offset magnitude in pushOperatorOffset()

namespace path_harness
{
/// @brief Bound on the per-phase tick handshake. A real-FSM regression that never settles fails
///        as a budget-exhausted assert instead of hanging the test process. Generous enough for
///        any real per-hop handshake (turn settle, profile drain, seek SEEK->ARMED->OVERSHOOT).
constexpr int kMaxHandshakeTicks = 64;

/// @brief Encoder degrees advanced per distance-terminated retire step. Large enough to overrun
///        any fixture hop's `_profileTotalEnc` (the longest fixture edge is ~112 cm) in a few
///        steps, so a profiled move's encoder gate trips well within `kMaxHandshakeTicks`.
constexpr int32_t kEncoderAdvanceStep = 20000;

/// @brief millis advanced per tick. Kept well BELOW every seek/turn stall watchdog
///        (`STOP_AT_LINE_STALL_TIMEOUT_MS`, `TURN_IMU_STALL_TIMEOUT_MS`) so a multi-tick
///        handshake never trips a watchdog by accident — yet non-zero so a `millis()`-indexed
///        decision still sees the clock move.
constexpr uint32_t kMillisPerTick = 5;
}    // namespace path_harness

// ===========================================================================================
// Core tick + reset
// ===========================================================================================

/**
 * @brief Run ONE production-order cooperative tick over the linked module subset, advancing the
 *        mock clock by a small fixed step.
 *
 * @details
 * Replays `WRO2026.ino::cooperativeTick`'s order for the modules these targets link:
 *   `LineArray.update(); Drive.update(); LineFollower.update(); Intersection.update();
 *    PathTracker.update(); Localization.update();`
 * (Manipulator/Tasks/Mission/EventTrace are not linked into these targets, so they are omitted —
 * never reordered.) The clock advances by `path_harness::kMillisPerTick` AFTER the module
 * updates so any `millis()`-indexed FSM decision this tick reads a stable clock, while a
 * multi-tick handshake still sees time progress (time advances ONLY here — never via `delay()`).
 *
 * @note This is the same body as the public `tick()` escape hatch — call `tick()` from a test
 *       body that needs a single raw step to assert an off-nominal transition (e.g. a fault
 *       mid-turn); call this implicitly via the `expect*` helpers for the nominal per-hop flow.
 */
inline void cooperativeTick()
{
    LineArray.update();
    Drive.update();
    LineFollower.update();
    Intersection.update();
    PathTracker.update();
    Localization.update();

    mockAdvanceMillis(path_harness::kMillisPerTick);
}

/**
 * @brief Raw single-step escape hatch — one `cooperativeTick()` for off-nominal assertions.
 *
 * @details
 * Identical to `cooperativeTick()`. Exposed under a second name so a test body that drives an
 * off-nominal transition (a fault mid-turn, a partial seek, a per-tick state inspection) reads
 * as `tick();` rather than re-deriving the per-hop handshake from a semantic helper. Use the
 * `expect*` helpers for the nominal per-hop flow; use `tick()` when you need to observe the FSM
 * between scripted observable changes.
 */
inline void tick()
{
    cooperativeTick();
}

/**
 * @brief Reset every mock surface and re-begin the real singletons to a clean per-case world.
 *
 * @details
 * Mirrors the repo `resetWorld()` idiom (`test_localization_offset.cpp:83`) extended for this
 * target's surfaces. Order:
 *   1. `mockResetMillis()`        — clock to 0 (time advances only via `mockAdvanceMillis`).
 *   2. `mockResetMiniR4()`        — resets the `MiniR4` facade (DriveDC done-bit/encoder/PID
 *                                   captures, M2/M3, LED, buzzer, ...). Does NOT touch `mmL`.
 *   3. `mmL.mockReset()`          — yaw 0 / result OK. `mmL` is a FREE `inline` global at
 *                                   namespace scope (MatrixMiniR4.h:1147), NOT a member of the
 *                                   `MiniR4` facade, so `mockResetMiniR4()` cannot reach it — it
 *                                   must be reset on its own (same idiom as
 *                                   `test_localization_offset.cpp`).
 *   4. peripheral `mockReset()`s  — LineArray / Intersection / LineFollower path mocks.
 *   5. `Drive.begin()`            — real Drive: seeds done flags true, pushes PID. Does NOT clear a
 *                                   latched fault — `Drive::begin()` never touches `_fault` /
 *                                   `_stopState` (Drive.cpp:40); the fault-reset path is
 *                                   `clearFault()` (step 8), not `begin()`.
 *   6. `Localization.begin()`     — real Localization: seeds `_currentNode = START_NODE` (=Start,
 *                                   BLIND), clears offset + integrator baselines + `_zeroed`.
 *   7. `PathTracker.begin()`      — real PathTracker: FSM to IDLE, queue empty, fault cleared.
 *   8. `Drive.clearFault()` +     — clear BOTH fault surfaces so a Drive fault latched by a prior
 *      `PathTracker.clearFault()`   case (e.g. `LINE_NOT_FOUND` from a TC-SL2 no-line seek) cannot
 *                                   leak in and stall / mis-fault the next case's first `goTo()`.
 *                                   PathTracker already cleared its own in step 7; Drive's is the
 *                                   load-bearing one (its `begin()` does not), and clearing both
 *                                   keeps every rewritten file order-independent without a per-test
 *                                   workaround.
 *
 * After this, `Localization` is seated at `Routes::START_NODE` (`Start`) with `_zeroed == false`.
 * A test that drives a hop MUST (a) bind the world frame and (b) seat Localization at the hop's
 * start node before calling `goTo(...)` — use `seatAt(name)` (which does both). Driving a
 * `goTo(...)` before binding the frame faults `Fault::NOT_ZEROED` by design (PathTracker.cpp:418).
 *
 * @note `Drive.begin()` is called BEFORE `PathTracker.begin()` / `Localization.begin()` to match
 *       the production `setup()` ordering (Drive begins before Localization/PathTracker). The
 *       relative order does not affect correctness here (each `begin()` is self-contained), but it
 *       keeps the harness faithful to the sketch.
 */
inline void resetWorld()
{
    mockResetMillis();
    mockResetMiniR4();
    mmL.mockReset();

    LineArray.mockReset();
    Intersection.mockReset();
    LineFollower.mockReset();

    Drive.begin();
    Localization.begin();
    PathTracker.begin();

    // begin() re-seeds done/idle state but does NOT clear a latched fault (Drive::begin() never
    // touches _fault/_stopState). Clear both so a Drive fault latched by a prior case cannot leak
    // into this one and stall / mis-fault its first goTo() — every rewritten file is then
    // order-independent with no per-test workaround.
    Drive.clearFault();
    PathTracker.clearFault();
}

// ===========================================================================================
// Routing / node-resolution guards
// ===========================================================================================

/**
 * @brief Resolve a node id by NAME — never an index.
 *
 * @param n Node name as it appears in the fixture's `Routes::NODES[*].name`
 *          (`Start`, `Inter1`..`Inter4`, `ProjL`, `ProjB`, `Corner`, `Spur`, `Zed`,
 *          `Wall1`, `final`, ...).
 * @return The node id, or `FieldMap::NODE_NONE` if no node carries that name.
 * @note Node INDICES renumber across field edits; resolving by name keeps the five tests stable
 *       against a fixture re-order (the gotcha behind issue #91). Pure delegation to
 *       `FieldMap::findNodeByName`.
 */
inline uint8_t nodeByName(const char* n)
{
    return FieldMap::findNodeByName(n);
}

/**
 * @brief The first hop the frozen Dijkstra `NEXT_HOP` table routes for `start -> goal`.
 *
 * @param start Start node id.
 * @param goal  Goal node id.
 * @return `FieldMap::getNextHop(start, goal)` — the next node on the shortest path, or
 *         `FieldMap::NODE_NONE` if unreachable / out of range.
 * @note A test pins the hop it exercises with `ASSERT_EQ(routedFirstHop(start, goal),
 *       nodeByName("<expected>"))` so a future fixture re-weight that silently re-routes the hop
 *       under test fails LOUDLY instead of passing against a different path (Fable amendment 2).
 */
inline uint8_t routedFirstHop(uint8_t start, uint8_t goal)
{
    return FieldMap::getNextHop(start, goal);
}

/**
 * @brief Assert there is NO direct edge `a -> c` — Dijkstra MUST traverse the intermediate node.
 *
 * @param a Source node id of the pass-through triple A->B->C.
 * @param c Far node id of the pass-through triple.
 * @note Used on every pass-through test: `assertNoBypass(A, C)` proves the only route A->C goes
 *       THROUGH B, so the pass-through hop under test is genuinely traversed rather than bypassed
 *       by a shorter direct edge a fixture re-weight might introduce (Fable amendment 2). Asserts
 *       `FieldMap::getEdge(a, c).type == Routes::EdgeType::NONE`.
 */
inline void assertNoBypass(uint8_t a, uint8_t c)
{
    ASSERT_TRUE(FieldMap::getEdge(a, c).type == Routes::EdgeType::NONE);
}

// ===========================================================================================
// World-frame helpers
// ===========================================================================================

/**
 * @brief Convert an app-frame (CCW-positive) heading to the hardware/raw (CW-positive) frame.
 *
 * @param appHeadingDeg App-frame heading the FSM commands (degrees).
 * @return The hardware-frame yaw `Config::IMU_YAW_SIGN * appHeadingDeg`, rounded to `int16_t`.
 * @note This is the SAME conversion `YawSource.toHwHeading` applies (ADR-045). Use it (a) to
 *       PARK `mmL.mockYawDeg` so a real IMU read recovers the app value, and (b) to
 *       predict the recorded `MiniR4.DriveDC.lastMoveGyroDegsTarget` of a GYRO move. CONTRACT 2:
 *       the recorded gyro target is hw-frame, so a test asserts it equals `hwYaw(appHeading)`.
 */
inline int16_t hwYaw(int16_t appHeadingDeg)
{
    return YawSource.toHwHeading(appHeadingDeg);
}

/**
 * @brief Bind the world frame at a chosen IMU yaw and seat `Localization` at a named node.
 *
 * @details
 * After `resetWorld()` the chassis sits at `Routes::START_NODE` (`Start`) with `_zeroed == false`,
 * so a `goTo(...)` would fault `NOT_ZEROED` and a hop driven from any other node would plan from
 * `Start`. This helper makes a test ready to drive a hop FROM @p startNode:
 *   1. Parks `mmL.mockYawDeg` at the hw value of @p startAppYawDeg so the IMU read the
 *      bind consumes is deterministic, then `Localization.zeroHeadingFromCurrent()` binds the
 *      world frame (sets `_zeroed`, captures `_yawOffsetDeg`).
 *   2. `Localization.declarePosition(nodeByName(startNode))` seats the discrete node at the hop
 *      origin and clears the operator offset (so PathTracker's DEQUEUE return-gate sees offset
 *      ~0 and transitions straight to `PLAN_NEXT_HOP` — no spurious return-to-node prologue).
 *
 * @param startNode      Name of the node to seat `Localization` at (the hop origin).
 * @param startAppYawDeg App-frame IMU yaw to bind the frame at (default 0 — the fixture's
 *                       `START_HEADING_DEG`). Pass the start node's expected facing for a hop
 *                       whose pre-turn would otherwise be non-trivial off a 0 heading.
 * @note `declarePosition` leaves `_yawOffsetDeg` untouched (it declares graph position, not
 *       heading) — the frame stays bound to the yaw from step 1. The mock IMU keeps returning
 *       that yaw until the test re-parks `mmL.mockYawDeg`, so the pre-turn's heading math starts
 *       from a known facing.
 */
inline void seatAt(const char* startNode, int16_t startAppYawDeg = 0)
{
    mmL.mockYawDeg = (double) hwYaw(startAppYawDeg);
    Localization.zeroHeadingFromCurrent();
    Localization.declarePosition(nodeByName(startNode));
}

// ===========================================================================================
// FSM entry-state drivers
// ===========================================================================================

/**
 * @brief Drive the real FSM from a clean world, seated at @p startName (app-yaw 0), through the
 *        shared LINE-follow entry point for the FIRST hop of `goTo(goalName)`.
 *
 * @details
 * Leaves the FSM in the state every collinear pass-through case needs BEFORE tripping the
 * junction/distance gate: the LineFollower enabled, non-overshoot, still seated at @p startName.
 * Covers BOTH a LINE-junction first hop (LINE_FOLLOW) and a LINE->projection first hop
 * (PROJECTION_FOLLOW) — the entry condition is identical for both. An optional route-direction
 * override drives a forced-REAR first hop (the body target becomes edge heading + 180). Clears
 * the `Serial` / `EventTrace` mock capture surfaces so every caller starts clean.
 *
 * Sequence the real FSM runs to get here (PathTracker.cpp): IDLE -> DEQUEUE (offset~0 return-gate
 * -> PLAN_NEXT_HOP) -> PLAN_NEXT_HOP (LINE first hop; if the edge heading == the seated facing,
 * `_dispatchTurnIfNeeded` skips the pre-turn) -> `_dispatchEdgeAfterTurn` -> arm the LineFollower
 * -> LINE_FOLLOW / PROJECTION_FOLLOW. The mock IMU yaw is parked at the (body-corrected) first-hop
 * edge heading each tick so any non-zero pre-turn settles immediately (the driveToSeekInFlight
 * idiom).
 *
 * @param startName      Node to seat `Localization` at (the first hop's origin).
 * @param goalName       The active `goTo` destination.
 * @param routeDirection Forwarded to `.setRouteDirection(...)` when not `Direction::AUTO`; a bare
 *                       `goTo(...)` otherwise. `Direction::REAR` also +180's the parked body target.
 * @note Asserts the FSM reached the LINE-follow entry within `path_harness::kMaxHandshakeTicks`
 *       (a real-FSM regression that never arms fails as a budget-exhausted assert, not a hang).
 *       `Serial` / `EventTrace` are the capturing mocks force-included target-wide by CMake
 *       (`add_path_force_includes`), so they are complete types here with no extra `#include`.
 */
inline void driveToLineFollowEntry(const char* startName, const char* goalName,
                                   Direction routeDirection = Direction::AUTO)
{
    resetWorld();
    Serial.clearCapture();
    EventTrace.mockReset();
    seatAt(startName);

    const uint8_t          start     = nodeByName(startName);
    const uint8_t          goal      = nodeByName(goalName);
    const uint8_t          firstHop  = routedFirstHop(start, goal);
    const Routes::EdgeInfo& firstEdge = FieldMap::getEdge(start, firstHop);

    // Body-target heading for the first hop: edge heading, +180 for a forced REAR route.
    float bodyHeading = firstEdge.heading_deg;
    if (routeDirection == Direction::REAR)
    {
        bodyHeading = fmodf(bodyHeading + 180.0f, 360.0f);
        if (bodyHeading < 0.0f)
        {
            bodyHeading += 360.0f;
        }
    }
    const int16_t hwHeading = hwYaw((int16_t) lroundf(bodyHeading));

    if (routeDirection == Direction::AUTO)
    {
        PathTracker.goTo(goal).async();
    }
    else
    {
        PathTracker.goTo(goal).setRouteDirection(routeDirection).async();
    }

    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        mmL.mockYawDeg = (double) hwHeading;
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == start)
        {
            return;
        }
    }
    ASSERT_TRUE(false);    // budget exhausted: the FSM never reached the LINE-follow entry.
}

// ===========================================================================================
// Per-phase done-bit / sensor handshake helpers
// ===========================================================================================

/**
 * @brief Drive an in-flight pre-edge TURN to completion at a scripted app-frame yaw target.
 *
 * @details
 * A pre-edge turn dispatches `Drive.turnToYaw(Localization.toImuYaw(bodyHeading)).setMode(byte)
 * .async()` (PathTracker.cpp:1649), which runs the turn PD controller (`_updateTurnProfile`,
 * Drive.cpp:1090, ADR-068). That controller runs ONE PD step per tick against `Move(L,R)` and
 * exits only once the CORRECTED IMU yaw (`YawSource.readYawDeg` -> `mmL.mockYawDeg *
 * IMU_YAW_SIGN`) has sat within `Config::TURN_TOLERANCE_DEG` of the app-frame target
 * `_turnTargetYaw` for `Config::TURN_SETTLE_SAMPLES` CONSECUTIVE ticks — the settle debounce
 * (Drive.cpp:1144-1153); a single in-band sample is NOT enough, and one out-of-band sample resets
 * the counter to 0. Once the debounce is satisfied it BRAKEs and sets `_turnDone`/`_moveDone`.
 * PathTracker's `TURNING` state then sees `Drive.isIdle()` and dispatches the edge
 * (PathTracker.cpp:680).
 *
 * This helper PARKS `mmL.mockYawDeg` at the hardware-frame value of @p appYawTarget ONCE, before
 * the loop, then ticks until `Drive.isIdle()`. A single park is sufficient: `mmL.mockYawDeg` is a
 * persistent mock field that `cooperativeTick()` never touches, so the parked value stays in-band
 * across every loop iteration and the debounce counter climbs by one per tick with no further
 * scripting — `Config::TURN_SETTLE_SAMPLES` (currently 2) consecutive in-band ticks settle the
 * turn. `kMaxHandshakeTicks` (64) is generous headroom over that; do NOT assume a single tick
 * retires the turn.
 *
 * @param appYawTarget The APP-frame (CCW-positive) yaw the chassis turns to (deg). CONTRACT 2:
 *                     converted to the hw frame internally, so the test passes the app value.
 * @param wheelByte    The wheel-pattern byte the turn was dispatched with
 *                     (`Routes::TurnMode` cast to `uint8_t`: COUNTER_ROT=1 / PIVOT_LEFT=2 /
 *                     PIVOT_RIGHT=3, or the pipeline default). Recorded for the assert below.
 *
 * @note Asserts the turn actually COMPLETED: `Drive.isIdle()` and `Drive.isTurnDone()` true after
 *       the handshake (the representative observable for a turn — the turn PD controller never
 *       issues a lower-MCU `MoveGyroDegs`, so there is no `lastMoveGyroDegsTarget` to check; the
 *       gyro-target assert belongs to the GYRO MOVE helpers). @p wheelByte is accepted so the
 *       signature documents the dispatched pattern and a future TurnMode-recording assert has the
 *       expected value at the call site; it is not otherwise consulted (the PD controller drives
 *       `Move(L,R)`, whose L/R sign encodes the pattern, not a recorded mode byte).
 */
inline void expectPreTurn(int16_t appYawTarget, uint8_t wheelByte)
{
    (void) wheelByte;    // documents the dispatched pattern; see @note (no recorded mode byte).

    // Park the corrected IMU read AT the app-frame target (hw frame = app * IMU_YAW_SIGN) ONCE —
    // the mock field holds this value across every tick below (cooperativeTick() never touches
    // it), so the settle debounce's TURN_SETTLE_SAMPLES consecutive in-band ticks (Drive.cpp:
    // 1144-1153) accumulate without re-parking each iteration.
    mmL.mockYawDeg = (double) hwYaw(appYawTarget);

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.isIdle(); ++i)
    {
        cooperativeTick();
    }

    ASSERT_TRUE(Drive.isIdle());
    ASSERT_TRUE(Drive.isTurnDone());
}

/**
 * @brief Drive an in-flight LINE hop through intersection-detect + axle overshoot to re-anchor.
 *
 * @details
 * A LINE hop arms the LineFollower and the FSM sits in `LINE_FOLLOW` polling
 * `Intersection.isAtIntersection()` (PathTracker.cpp:712). On the edge it re-arms the follower
 * with the axle-offset distance gate and enters `LINE_OVERSHOOT`, which polls
 * `LineFollower.isDistanceReached()` (PathTracker.cpp:858) before calling
 * `Localization.onIntersection(_nextHop)` and returning to `PLAN_NEXT_HOP`.
 *
 * This helper scripts that two-gate sequence:
 *   1. Latch `Intersection.mockSetAtIntersection(true)` and tick — the FSM consumes the
 *      read-and-clear edge and arms the overshoot (asserts the LineFollower re-arm happened via
 *      `setEnabledCount`). The latch is auto-cleared by the read, so the FSM cannot re-detect.
 *   2. Latch `LineFollower.mockSetDistanceReached(true)` and tick — the overshoot gate trips and
 *      `Localization.onIntersection` advances the node.
 *
 * @note Asserts the FSM advanced the discrete node (`LineFollower.setEnabledCount > 0` after the
 *       intersection arm — the representative LineFollower-arm observable). The lower-MCU
 *       `MiniR4.DriveDC` call for a LINE hop is the LineFollower's per-tick `setSpeed`, which the
 *       follower MOCK absorbs (no real PID), so a LINE hop has no `DriveDC` move to fold in here —
 *       the test asserts the LineFollower ARM cascade (`lastBrakeOnGate`, `lastDistanceTarget`)
 *       it cares about directly on the path mock. Leaves `LineFollower.isDistanceReached()` set
 *       per the mock's non-auto-clear contract; the caller's next arm / `mockReset` clears it.
 */
inline void expectLineHop()
{
    const int armsBefore = LineFollower.setEnabledCount;

    // Gate 1 — the junction crossing. Read-and-clear: one latched tick is the edge.
    Intersection.mockSetAtIntersection(true);
    cooperativeTick();

    // The FSM should have re-armed the follower for the overshoot window (LINE_OVERSHOOT arm).
    ASSERT_GT(LineFollower.setEnabledCount, armsBefore);

    // Gate 2 — the axle overshoot distance gate. Trips `onIntersection(_nextHop)`.
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
}

/**
 * @brief Drive an in-flight BLIND dead-reckon hop (`driveDist`, profiled or flat) to completion.
 *
 * @details
 * A BLIND hop resolved to dead-reckon dispatches `Drive.driveDist(cm).setMode(...)[.tShape()]
 * .async()` and the FSM sits in `BLIND_FORWARD` polling `Drive.isIdle()` (PathTracker.cpp:928),
 * then calls `Localization.onDeadReckonedAdvance(_nextHop)`. The move is retired by whichever
 * regime owns it (CONTRACT 1):
 *   - profiled (`SYNC_TSHAPE` / `GYRO_TSHAPE`): `_updateProfile` completes when `M2`'s encoder
 *     delta reaches `_profileTotalEnc`.
 *   - non-profiled (`RAW_FLAT`): the stage-1 drain flips `_moveDone` when `isPrevTaskDone()` true.
 *
 * To cover BOTH regimes with one handshake, each tick this helper ADVANCES `MiniR4.M2` (and `M3`
 * for symmetry) by `kEncoderAdvanceStep` AND holds `mockSetPrevTaskDone(true)`, then ticks, until
 * `Drive.isIdle()`. The encoder overrun trips the profile gate; the done-bit retires the flat
 * move; the loop is budgeted so a stuck FSM fails as an assert, not a hang.
 *
 * @param cm The hop's commanded distance (cm) — accepted for call-site documentation /
 *           future distance asserts; the retire logic over-advances the encoder regardless, so
 *           the exact value is not consulted (a profiled move completes on ANY overrun past its
 *           total). Pass the fixture edge's `distance_cm` so the test reads self-documentingly.
 *
 * @note Asserts the move completed (`Drive.isIdle()` after the handshake) — the representative
 *       observable. For a GYRO dead-reckon the test additionally folds in the lower-MCU gyro
 *       target by asserting `MiniR4.DriveDC.lastMoveGyroDegsTarget == hwYaw(appBodyHeading)`
 *       (CONTRACT 2) on the recorded dispatch BEFORE calling this (the capture survives until the
 *       next gyro move). This helper does not re-assert it because the recorded value is the
 *       SAME field the seek helper also writes.
 */
inline void expectBlindHop(float cm)
{
    (void) cm;    // self-documenting at the call site; the retire over-advances regardless.

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.isIdle(); ++i)
    {
        // Profiled-move regime: overrun the encoder so `_updateProfile` trips its distance gate.
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        // Non-profiled regime: the stage-1 drain flips `_moveDone` off the done-bit.
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }

    ASSERT_TRUE(Drive.isIdle());
}

/**
 * @brief Drive an in-flight `Drive.stopAtLine` seek to a SUCCESSFUL crossing or a no-line FAULT.
 *
 * @details
 * A BLIND-resolved-to-seek hop dispatches `Drive.stopAtLine(edge.distance_cm).setReverse(...)
 * .setMode(...).setHeading(...).setOvershoot(...).async()` (PathTracker.cpp:2088) and the FSM sits
 * in `STOP_AT_LINE` polling `Drive.isIdle()` / (via the shared guard) `Drive.hasFault()`
 * (PathTracker.cpp:961, :511). The seek runs `_updateStopAtLine` (Drive.cpp:1195):
 *   - SEEK: cruise until `traveled >= expected - STOP_AT_LINE_ARM_WINDOW_CM` -> ARMED.
 *   - ARMED: debounce `LineArray.getJunctionType() == JUNCTION_T_OR_CROSS` for
 *     `STOP_AT_LINE_DETECT_TICKS` ticks -> OVERSHOOT; OR `traveled >= expected +
 *     STOP_AT_LINE_MAX_MARGIN_CM` (or a stall) -> latch `Fault::LINE_NOT_FOUND`.
 *   - OVERSHOOT: roll the overshoot, then DONE (sets `_moveDone`).
 * `traveled` is `abs(M2.getDegrees() - _stopStartEnc)` (Drive.cpp:1217). The seek captures
 * `_stopStartEnc` from `M2` at dispatch, so a test advances `M2` RELATIVE to dispatch.
 *
 * @param cm    The seek's expected distance (`edge.distance_cm`, cm) — used to size the encoder
 *              advance so the seek arms (success) or overruns the max margin (fault).
 * @param found `true`  -> SUCCESS: advance `M2` into the arm window, then hold `T_OR_CROSS` for
 *                         the debounce + roll the overshoot, so the seek confirms the crossing and
 *                         finishes DONE (Drive `isLineFound()` true, no fault).
 *              `false` -> FAULT: keep `getJunctionType()` NONE and over-advance `M2` PAST
 *                         `expected + STOP_AT_LINE_MAX_MARGIN_CM` so the seek latches
 *                         `DriveClass::Fault::LINE_NOT_FOUND`; the FSM's shared Drive-fault guard
 *                         then maps it to `PathTracker::Fault::DRIVE_FAULT` (TC-SL2).
 *
 * @note Folds in the representative lower-MCU seek assert: on the SUCCESS path it asserts the seek
 *       reached DONE and Drive latched the crossing (`Drive.isLineFound()`); on the FAULT path it
 *       asserts Drive latched `LINE_NOT_FOUND` (`Drive.hasFault()` with
 *       `getFault().kind == DriveClass::Fault::LINE_NOT_FOUND`). The encoder is positioned with
 *       `cmToEncDeg(...)` RELATIVE to the dispatch baseline (`M2` at entry == `_stopStartEnc`,
 *       since no tick has retired the seek yet): the SUCCESS path lands `traveled ≈ @p cm` INSIDE
 *       the arm window `[expected - ARM_WINDOW, expected + MAX_MARGIN)` so it arms WITHOUT
 *       overrunning the max gate; the FAULT path lands `traveled` PAST `expected + MAX_MARGIN` so
 *       the max gate latches LINE_NOT_FOUND. The big `kEncoderAdvanceStep` overrun is deliberately
 *       NOT used here — a seek HAS a max-distance fault, so an over-advance would mis-fault the
 *       success case. Drives the clock with `mockAdvanceMillis` per tick (via `cooperativeTick`),
 *       staying below `STOP_AT_LINE_STALL_TIMEOUT_MS` so the success path is not stolen by the
 *       stall watchdog.
 */
inline void expectStopAtLine(float cm, bool found)
{
    // The seek captured `_stopStartEnc = M2.getDegrees()` at dispatch and measures
    // `traveled = abs(M2.getDegrees() - _stopStartEnc)`. No tick has advanced `M2` since dispatch,
    // so the current reading IS that baseline — position `M2` ABSOLUTELY relative to it.
    const int32_t baseline = MiniR4.M2.getDegrees();

    if (found)
    {
        // Land `traveled ≈ cm`: inside the arm window (>= expected - ARM_WINDOW) and below the max
        // gate (< expected + MAX_MARGIN, since traveled == expected). Marker absent so the SEEK ->
        // ARMED transition does not also see a crossing on the arming tick.
        const int32_t armEnc = baseline + (int32_t) cmToEncDeg(cm);
        LineArray.mockSetJunctionType(JunctionType::NONE);
        MiniR4.M2.mockSetDegrees(armEnc);
        MiniR4.M3.mockSetDegrees(armEnc);
        cooperativeTick();    // SEEK -> ARMED (traveled now inside the arm window).

        // Present a debounced T_OR_CROSS: hold it for STOP_AT_LINE_DETECT_TICKS+1 ticks WITHOUT
        // advancing the encoder so the seek confirms detection and anchors the OVERSHOOT roll on
        // the current (detect) encoder. The +1 absorbs the tick the streak reaches the threshold.
        LineArray.mockSetJunctionType(JunctionType::T_OR_CROSS);
        for (int i = 0; i < Config::STOP_AT_LINE_DETECT_TICKS + 1 && !Drive.isIdle(); ++i)
        {
            cooperativeTick();
        }

        // Roll the OVERSHOOT to DONE: advance `M2` past the detect encoder by the axle offset so
        // `rolled >= _stopOvershoot` and the seek finalises. A generous extra margin guarantees the
        // roll completes regardless of the resolved overshoot trim.
        const int32_t overshootEnc =
            armEnc + (int32_t) cmToEncDeg(Config::LINE_SENSOR_AXLE_OFFSET_CM + 5.0f);
        for (int i = 0;
             i < path_harness::kMaxHandshakeTicks && !Drive.isIdle() && !Drive.hasFault();
             ++i)
        {
            MiniR4.M2.mockSetDegrees(overshootEnc);
            MiniR4.M3.mockSetDegrees(overshootEnc);
            cooperativeTick();
        }

        ASSERT_TRUE(Drive.isIdle());
        ASSERT_TRUE(Drive.isLineFound());
        ASSERT_FALSE(Drive.hasFault());
    }
    else
    {
        // No-line fault: marker stays NONE while `traveled` overruns `expected + MAX_MARGIN`, so
        // the seek's max-distance gate latches LINE_NOT_FOUND (clears the queue + stops). A small
        // slack past the margin guarantees the gate trips on the first tick.
        const int32_t faultEnc =
            baseline
            + (int32_t) cmToEncDeg(cm + Config::STOP_AT_LINE_MAX_MARGIN_CM + 5.0f);
        LineArray.mockSetJunctionType(JunctionType::NONE);
        for (int i = 0; i < path_harness::kMaxHandshakeTicks && !Drive.hasFault(); ++i)
        {
            MiniR4.M2.mockSetDegrees(faultEnc);
            MiniR4.M3.mockSetDegrees(faultEnc);
            cooperativeTick();
        }

        ASSERT_TRUE(Drive.hasFault());
        ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::LINE_NOT_FOUND);
    }
}

/**
 * @brief Retire an in-flight non-blocking `SETTLE` dwell: jump the mock clock past the
 *        settle deadline in ONE step, then tick until the FSM leaves `SETTLE` (ADR-061).
 *
 * @details
 * The ADR-061 post-stop settle is a `millis()`-gated hold, NOT a Drive command: on entry
 * `PathTracker::_enterSettle(...)` stamps `_settleDeadlineMs = millis() + Config::HOP_SETTLE_MS`
 * and parks the FSM in `State::SETTLE`; the `SETTLE` state body then returns each tick until the
 * rollover-safe compare `(int32_t)(millis() - _settleDeadlineMs) >= 0` goes true, at which point
 * it resumes to `PLAN_NEXT_HOP` (dwell-1 / goal arrival) or `_dispatchEdgeAfterTurn()` (dwell-2,
 * the cold-turn -> move dwell). Nothing in the harness scripts an encoder / done-bit for a settle
 * — the ONLY input the dwell consumes is the clock.
 *
 * `_settleDeadlineMs` is PRIVATE (`PathTracker.h`), so this helper cannot read the exact deadline.
 * It does NOT need to: the deadline is `millis() + Config::HOP_SETTLE_MS` and `Config::HOP_SETTLE_MS`
 * is a compile-time constant visible here (real `Config.h` is `#include`d at the top of this
 * header), so a SINGLE `mockAdvanceMillis(Config::HOP_SETTLE_MS + 1)` guarantees the clock is at
 * least one tick PAST the deadline regardless of what `millis()` read at entry — the known-magnitude
 * advance is exact without a getter. The `+ 1` covers the boundary (the compare is `>= 0`, so being
 * exactly AT the deadline already resolves; the extra ms is defensive against a same-tick stamp).
 * The follow-up tick loop then runs the resume transition; it is bounded by
 * `path_harness::kMaxHandshakeTicks` so a real-FSM regression that never leaves `SETTLE` fails as a
 * budget-exhausted assert instead of hanging, and it short-circuits on a fault so a mid-dwell fault
 * (which resets the settle members, `PathTracker::_enterFault`) exits cleanly.
 *
 * @note NO assert here on its own: unlike the per-hop retire helpers, `driveSettle()` is a pure
 *       clock-jump utility — the CALLER asserts the observed `State::SETTLE` BEFORE calling it and
 *       the resumed state (dispatch / IDLE / PLAN) AFTER. `mockAdvanceMillis` is declared in the
 *       mock `<Arduino.h>` already `#include`d by this harness; `Config::HOP_SETTLE_MS` and
 *       `PathTrackerClass::State::SETTLE` are visible via the real `Config.h` / `PathTracker.h`
 *       included at the top of this header. When `Config::HOP_SETTLE_MS == 0` the FSM never enters
 *       `SETTLE`, so the loop condition is false on entry and this is a harmless no-op.
 */
inline void driveSettle()
{
    mockAdvanceMillis(Config::HOP_SETTLE_MS + 1);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() == PathTrackerClass::State::SETTLE
                    && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
}

// ===========================================================================================
// Reset-arrival + return-to-node helpers (issue #113)
//
// Two ADDITIVE helpers retire the wall-zero reset interlude's `Drive.wallStop` push
// (`expectWallStop`) and manufacture a genuine operator offset through the real
// `Localization` integrator (`pushOperatorOffset`), so the reset-arrival / return-to-node
// test bodies drive the REAL FSM instead of an inline mirror. They use ONLY the public Drive
// / Localization surface and the existing scripting idioms above; they add no new mock state.
// ===========================================================================================

namespace path_harness
{
/// @brief Generous bound on the wall-stop retire loop. The wall-stop PUSH advisory / stall
///        windows are multi-second (`Config::WALL_STOP_TIMEOUT_MS` = 3000,
///        `STOP_AT_LINE_STALL_TIMEOUT_MS` = 1500). The STALL-success path advances the clock in a
///        SINGLE large `mockAdvanceMillis` jump (a frozen encoder IS a stall), so it idles in a
///        handful of ticks. The ADVISORY path CANNOT single-jump: a frozen encoder past the outer
///        timeout reads as a stall, and `_updateWallStop` checks the no-progress stall gate BEFORE
///        the outer advisory gate, so a single jump would retire as a (false) stall-success with
///        `_wallStopTimedOut` left false. Instead the advisory path steps the encoder forward
///        EVERY tick (re-baselining the stall watchdog so it never trips) while stepping the clock
///        by `kWallAdvisoryStepMs` (< the 1500 ms stall window) until the OUTER `_wallStartMs`
///        timeout (3000 ms) elapses — that is ~`WALL_STOP_TIMEOUT_MS / kWallAdvisoryStepMs`
///        ≈ 3-4 iterations, plus the SQUARE-turn settle (a few `kMillisPerTick` steps) and the
///        PUSH / BACK_OFF / DONE finalisation ticks — ~10-15 ticks all told. 64 clears that worst
///        case with wide margin for any advisory step >= ~100 ms (the chosen `kWallAdvisoryStepMs`
///        is 1000 ms) while failing a never-idling regression ~4x sooner than the former 256. A
///        real-FSM regression that never idles still fails as a budget-exhausted assert, not a hang.
constexpr int kMaxWallStopTicks = 64;

/// @brief Number of extra progress ticks the STALL-SUCCESS path runs (after the wall-stop is
///        dispatched out of IDLE) to guarantee the FSM has advanced SQUARE -> PUSH and run at
///        least one PUSH progress tick, re-baselining the no-progress watchdog (`_wallLastProgressMs`)
///        to the LIVE clock BEFORE the single stall-window jump. The wall-stop needs: one tick to
///        dispatch the queued push into SQUARE (the no-op turn short-circuits, since the facing is
///        already parked), one tick for `_updateWallStop` to seed SQUARE -> PUSH, and one PUSH tick
///        to register progress. 4 clears that 3-tick worst case with a margin, and a frozen-encoder
///        regression that idles early simply exits the trailing stall loop sooner — never a hang.
constexpr int kWallPushSettleTicks = 4;

/// @brief Clock step (ms) per ADVISORY-path PUSH tick in `expectWallStop`. Kept STRICTLY BELOW
///        `Config::STOP_AT_LINE_STALL_TIMEOUT_MS` (= 1500) so the inner stall watchdog can never
///        trip between two progress ticks — the advisory branch re-baselines the watchdog every
///        tick by stepping the encoder forward, but the clock must not jump a full stall window in
///        one go or the no-progress stall gate (which `_updateWallStop` checks first) would fire
///        ahead of the outer advisory gate. 1000 ms reaches the 3000 ms outer timeout in ~3-4 ticks,
///        well inside `kMaxWallStopTicks`. Do NOT shrink this toward `kMillisPerTick` — a tiny step
///        would need ~600 ticks to cross 3000 ms and blow the budget.
constexpr uint32_t kWallAdvisoryStepMs = 1000;

/// @brief Per-step push distance (cm) for `pushOperatorOffset`'s integrator drive. Comfortably
///        above `Config::OFFSET_TRANSLATION_DEADBAND_CM` (= 0.2) so EVERY step clears the deposit
///        gate (a sub-deadband step would carry across without depositing and could stall the
///        accumulation loop). 2 cm/step also keeps the loop short: a 20 cm target converges in
///        ~10 deposit ticks, a 6 cm target in ~3 — both well inside `kMaxWallStopTicks`.
constexpr float kOffsetPushStepCm = 2.0f;

/// @brief Bound on the operator-offset accumulation loop. At `kOffsetPushStepCm` per deposit tick
///        plus the one-tick reopen-edge re-baseline, even the largest sane target (the
///        `Config::RETURN_DISTANCE_CAP_CM` = 50 cap) converges in ~26 ticks — 64 clears that with
///        ~2.4x margin (the current 6 / 20 cm call sites converge in ~3 / ~10), and a
///        zero-accumulation regression (the two-tick gate broken) exhausts it and trips the
///        self-assert below instead of hanging.
constexpr int kMaxOffsetPushTicks = 64;
}    // namespace path_harness

/**
 * @brief Retire an in-flight `Drive.wallStop` push to a clean stall-success or an advisory timeout.
 *
 * @details
 * Drives the REAL `Drive` wall-stop state machine (`_updateWallStop`: SQUARE -> PUSH -> BACK_OFF ->
 * DONE) to idle, the way `PathTracker::_dispatchWallStop` issues it for a RESET-node arrival:
 * `.setReverse(true).setMode(RAW).setHeading(facing).setBackOff(0).async()`, with NO
 * `.setTimeout(...)` — so the dispatched `_wallTimeoutMs` inherits the builder default, which is
 * `Config::WALL_STOP_TIMEOUT_MS` (CONFIRMED read-only: the `WallStopBuilder` seeds its `_timeout`
 * member from `Config::WALL_STOP_TIMEOUT_MS`; the private `_wallTimeoutMs` has no public accessor).
 * That is why advancing the clock by `Config::WALL_STOP_TIMEOUT_MS` is exact — no magic literal, no
 * getter, no production change. The `ConfigChecks` static_assert that `WALL_STOP_TIMEOUT_MS` (3000)
 * exceeds `STOP_AT_LINE_STALL_TIMEOUT_MS` (1500) guarantees the two PUSH retire paths never race.
 *
 * Two phases, in order:
 *   1. **Settle the internal SQUARE turn FIRST.** `wallStop` turns in place to @p wallFacingAppDeg
 *      (its SQUARE phase, driven by `_updateTurnProfile`) BEFORE pressing. Like every turn in this
 *      harness (CONTRACT 2 / `expectPreTurn`), it settles only if `mmL.mockYawDeg` is parked at the
 *      hardware-frame facing EACH tick — so park `hwYaw(wallFacingAppDeg)` and advance the clock in
 *      SMALL `kMillisPerTick` steps until `Drive.isTurnDone()` (the SQUARE turn reached BRAKE, OR was
 *      a no-op turn that short-circuited at dispatch). NOTE: this loop can exit with the wall-stop
 *      still QUEUED (not yet dispatched) when `Drive.isTurnDone()` was ALREADY true from the inbound
 *      hop — the dispatch + SQUARE -> PUSH seeding then happen inside Phase 2's warm-up, which is why
 *      Phase 2 (stall-success) seeds the PUSH watchdogs from a LIVE clock before any big jump rather
 *      than assuming they were seeded here.
 *   2. **Retire the PUSH** per @p timedOut:
 *        - `timedOut == false` (stall-success): the wall-stop is typically STILL QUEUED after
 *          Phase 1 — the inbound hop already left `Drive.isTurnDone()` true, so the Phase-1 settle
 *          loop exited WITHOUT ticking the queued push through its dispatch + SQUARE -> PUSH
 *          transitions. So FIRST warm up the PUSH: advance `MiniR4.M2`/`M3` by a real progress step
 *          (> `Config::STOP_AT_LINE_PROGRESS_EPS_CM`) EVERY tick until the push is dispatched out of
 *          IDLE (`!Drive.isIdle()`), then a fixed `path_harness::kWallPushSettleTicks` more progress
 *          ticks so `_updateWallStop` has advanced SQUARE -> PUSH and run at least one PUSH progress
 *          tick — re-baselining the no-progress watchdog `_wallLastProgressMs` to the LIVE
 *          (still-small) clock. ONLY THEN HOLD the encoder (no further motion = wheels stalled
 *          against the wall) and jump the clock once past `Config::STOP_AT_LINE_STALL_TIMEOUT_MS`,
 *          so the next PUSH tick confirms the stall -> BACK_OFF. (Seeding the watchdog from a
 *          CURRENT clock is load-bearing: if the jump preceded the SQUARE -> PUSH seed, the seed
 *          would capture the already-jumped clock and the frozen encoder could never accrue a full
 *          stall window within budget.) Back-off is 0 here (`_dispatchWallStop` `.setBackOff(0)`),
 *          so BACK_OFF falls straight through to DONE. `wallStopTimedOut()` stays false.
 *        - `timedOut == true` (advisory): the GENUINE "wheels turned freely, never reached the wall"
 *          scenario. The stall gate (`now - _wallLastProgressMs >= 1500`) is checked BEFORE the
 *          advisory gate (`now - _wallStartMs >= 3000`), and BOTH watchdogs are seeded from
 *          `millis()` at the SQUARE -> PUSH transition. A frozen encoder is a
 *          STALL, so a single clock jump past the outer timeout would trip the stall gate FIRST and
 *          retire as a (false) stall-success with `_wallStopTimedOut` left false. Instead, LOOP: each
 *          iteration step `MiniR4.M2`/`M3` forward by > `Config::STOP_AT_LINE_PROGRESS_EPS_CM`
 *          (`stopAtLineProgressed` true -> the PUSH tick re-baselines `_wallLastProgressMs` to now, so
 *          the stall gate NEVER trips) and advance the clock by `path_harness::kWallAdvisoryStepMs`
 *          (STRICTLY < the 1500 ms stall window), then tick — until `Drive.isIdle()`. The OUTER
 *          `_wallStartMs` advisory timeout (3000 ms) elapses across these sub-stall-window steps,
 *          latching `_wallStopTimedOut = true` -> DONE. This is a NON-fault advisory (`hasFault()`
 *          stays false, the queue is not frozen).
 *
 * Bounded by its own generous `path_harness::kMaxWallStopTicks` budget (NOT `kMaxHandshakeTicks`,
 * which is sized for the sub-second per-hop handshakes) so a real-FSM regression that never idles
 * fails as a budget-exhausted assert, never an infinite loop.
 *
 * @param wallFacingAppDeg The RESET node's `facing` (app-frame, CCW-positive) — the wall normal the
 *                         SQUARE turn aligns to. For `Wall1` this is 270. CONTRACT 2: parked in the
 *                         hardware frame via `hwYaw(...)` internally, so the caller passes the app
 *                         value.
 * @param timedOut         `false` -> drive the clean stall-success path; `true` -> drive the advisory
 *                         no-stall timeout. Re-asserted below against `Drive.wallStopTimedOut()`.
 *
 * @note Asserts the push retired to the expected outcome: `Drive.isIdle()` (SQUARE/PUSH/BACK_OFF
 *       state machines all back to IDLE, queue empty), `Drive.wallStopTimedOut() == timedOut` (the
 *       advisory flag matches the path taken), and `!Drive.hasFault()` (the advisory timeout is NOT
 *       a hard fault, and a stall-success never faults either).
 */
inline void expectWallStop(int16_t wallFacingAppDeg, bool timedOut)
{
    // --- Phase 1: settle the internal SQUARE turn (park the wall facing every tick, small clock
    // steps). One tick lets the queued wallStop dispatch (pops it, enters SQUARE — a no-op turn
    // short-circuits at dispatch since the facing is already parked); the loop then settles any
    // real in-place turn. `isTurnDone()` reads true once the SQUARE turn BRAKEs (or was the no-op).
    // NOTE: when `isTurnDone()` was ALREADY true from the inbound hop, this loop exits with the
    // wall-stop still QUEUED (un-dispatched) — Phase 2 (stall-success) then drives the dispatch +
    // SQUARE -> PUSH seeding itself, from a live clock, before any big stall-window jump.
    mmL.mockYawDeg = (double) hwYaw(wallFacingAppDeg);
    cooperativeTick();    // let the queued wallStop dispatch -> SQUARE (or stay queued if idle already).

    for (int i = 0; i < path_harness::kMaxWallStopTicks && !Drive.isTurnDone(); ++i)
    {
        mmL.mockYawDeg = (double) hwYaw(wallFacingAppDeg);    // re-park EVERY tick (CONTRACT 2).
        cooperativeTick();
    }
    ASSERT_TRUE(Drive.isTurnDone());    // SQUARE settled (or the no-op turn short-circuited).

    // --- Phase 2: retire the PUSH. The seek/wall watchdogs compare `traveled` (M2 encoder delta from
    // the push baseline) and the clock; the mock has no physics, so the encoder is driven explicitly.
    if (!timedOut)
    {
        // Stall-success.
        //
        // After Phase 1 the wall-stop is NOT yet in its PUSH phase: the inbound hop already left
        // `Drive.isTurnDone()` true, so the Phase-1 settle loop exited without ticking the
        // wall-stop through its dispatch + SQUARE -> PUSH transitions. The queued wall-stop is
        // dispatched on the FIRST tick below (`_isIdle() && _qCount > 0` -> SQUARE, a no-op turn
        // since the facing is already parked), and `_updateWallStop` advances SQUARE -> PUSH a
        // tick later, seeding the no-progress watchdog (`_wallLastProgressMs`) at the clock AT THAT
        // TICK. If the big stall-window jump happened BEFORE the watchdog was seeded, the seed
        // would capture the already-jumped clock and the stall could never trip within budget.
        //
        // So WARM UP the PUSH first with `kWallPushSettleTicks` progress ticks, each advancing
        // `MiniR4.M2`/`M3` by a real progress step. The wall-stop is QUEUED at Phase 2 entry (the
        // inbound hop left `Drive.isTurnDone()` true, so Phase 1 exited without ticking it past
        // dispatch — `Drive.isIdle()` is already false here because the queue is non-empty). Tick 1
        // dispatches it (`_isIdle() && _qCount > 0` -> SQUARE, a no-op turn since the facing is
        // parked), tick 2 advances SQUARE -> PUSH (seeding the no-progress watchdog
        // `_wallLastProgressMs` at the live clock), and the remaining ticks run PUSH progress so the
        // mark sits at ~now. Only then freeze the encoder (wheels stalled against the wall) and jump
        // the clock once past the stall window — the next PUSH tick confirms the stall -> BACK_OFF ->
        // (back-off 0) -> DONE, all from a CURRENT baseline (a pre-seed jump could never trip it in
        // budget). A fixed count is correct and deterministic: tick 1 dispatch, tick 2 SQUARE->PUSH,
        // the rest PUSH progress — `kWallPushSettleTicks` covers it with margin for any entry state.
        const int32_t progressStepEnc =
            (int32_t) cmToEncDeg(Config::STOP_AT_LINE_PROGRESS_EPS_CM + 1.0f);

        for (int i = 0; i < path_harness::kWallPushSettleTicks; ++i)
        {
            mmL.mockYawDeg = (double) hwYaw(wallFacingAppDeg);
            MiniR4.M2.mockAdvanceDegrees(progressStepEnc);
            MiniR4.M3.mockAdvanceDegrees(progressStepEnc);
            cooperativeTick();
        }

        // (c) Encoder now held (no further advance) => stalled. Jump the clock past the stall
        //     timeout so the next PUSH tick confirms the stall -> BACK_OFF -> DONE.
        mockAdvanceMillis(Config::STOP_AT_LINE_STALL_TIMEOUT_MS + 1);

        for (int i = 0; i < path_harness::kMaxWallStopTicks && !Drive.isIdle(); ++i)
        {
            // Keep the facing parked (a stray off-target yaw cannot re-arm a turn here — the turn is
            // long done — but parking keeps the world frame stable for the trailing Localization tick).
            mmL.mockYawDeg = (double) hwYaw(wallFacingAppDeg);
            cooperativeTick();
        }
    }
    else
    {
        // Advisory: the "wheels turned freely, never reached the wall" path. A frozen
        // encoder reads as a STALL, and `_updateWallStop` checks the stall gate BEFORE the advisory
        // gate, with both watchdogs seeded together at SQUARE -> PUSH — so a single
        // big clock jump would retire as a (false) stall-success and leave `_wallStopTimedOut` false.
        // Instead, KEEP MAKING PROGRESS every PUSH tick (re-baselining the stall watchdog so it never
        // trips) while stepping the clock in sub-stall-window increments, so only the OUTER
        // `_wallStartMs` advisory timeout (`WALL_STOP_TIMEOUT_MS` = 3000) can elapse and send
        // PUSH -> DONE with `_wallStopTimedOut = true`.
        const int32_t advisoryStepEnc = (int32_t) cmToEncDeg(Config::STOP_AT_LINE_PROGRESS_EPS_CM + 1.0f);
        for (int i = 0; i < path_harness::kMaxWallStopTicks && !Drive.isIdle(); ++i)
        {
            // (a) Advance the encoder > STOP_AT_LINE_PROGRESS_EPS_CM so this tick's PUSH sees progress
            //     and re-baselines `_wallLastProgressMs` to `now` => the stall gate stays < 1500 ms.
            MiniR4.M2.mockAdvanceDegrees(advisoryStepEnc);
            MiniR4.M3.mockAdvanceDegrees(advisoryStepEnc);
            // (b) Step the clock by STRICTLY LESS than the stall window so the stall gate cannot trip
            //     between two progress ticks; the OUTER 3000 ms timeout accrues across these steps.
            mockAdvanceMillis(path_harness::kWallAdvisoryStepMs);
            // (c) Keep the facing parked (world frame stable for the trailing Localization tick), then
            //     run the PUSH tick. `cooperativeTick` adds `kMillisPerTick` on top, still < 1500 ms.
            mmL.mockYawDeg = (double) hwYaw(wallFacingAppDeg);
            cooperativeTick();
        }
    }

    ASSERT_TRUE(Drive.isIdle());
    ASSERT_TRUE(Drive.wallStopTimedOut() == timedOut);
    ASSERT_FALSE(Drive.hasFault());
}

/**
 * @brief Manufacture a REAL operator offset of magnitude >= @p targetCm along @p alongAppHeadingDeg
 *        by simulating an operator hand-push through the real `Localization` integrator.
 *
 * @details
 * Reproduces a runtime hand-push (nav inactive + the chassis rolled by hand) so the return-to-node
 * prologue sees a genuine offset, NOT a production test hook. `Localization::update()` integrates
 * only when the gate is open (`!_navActive && _zeroed`) AND it was open last tick (`_gateWasOpen`);
 * the FIRST open tick re-baselines and deposits NOTHING (the reopen edge), deposits start the SECOND
 * (Localization.cpp:282). Each deposit reads `Drive.getEncL()/getEncR()` (= `MiniR4.M2/M3`,
 * Drive.cpp:494), averages the wheel delta in cm, and — when it clears
 * `Config::OFFSET_TRANSLATION_DEADBAND_CM` — decomposes it along the LIVE world heading
 * (Localization.cpp:355): `_offset.x += dCm*cos(h)`, `_offset.y += dCm*sin(h)`. So the heading is
 * parked at @p alongAppHeadingDeg BEFORE advancing encoders, or the deposit lands on the wrong axis.
 *
 * Sequence (the two-phase reopen edge is load-bearing, gotcha
 * [[gotcha-localization-rezero-reopen-edge]]):
 *   1. `Localization.setNavActive(false)` — belt-and-suspenders: the gate's nav term is open. (The
 *      PRECONDITION is that the caller already `seatAt(anchor, alongAppHeadingDeg)` with the FSM
 *      IDLE so nav is inactive; this only re-asserts it.)
 *   2. Park `mmL.mockYawDeg = hwYaw(alongAppHeadingDeg)` so the integrator's world-heading read
 *      recovers @p alongAppHeadingDeg (CONTRACT 2).
 *   3. Tick ONCE — the reopen edge re-baselines off this single live read and deposits nothing.
 *   4. Loop: advance `MiniR4.M2` AND `MiniR4.M3` by `kOffsetPushStepCm` (well past the deadband) and
 *      tick (via `cooperativeTick`, which calls `Localization.update()`), so `_integrateOffset`
 *      deposits along the heading, until `getOffsetFromNode()` magnitude >= @p targetCm.
 *
 * @param targetCm           Minimum operator-offset magnitude to manufacture (cm). Must be > 0.
 * @param alongAppHeadingDeg App-frame heading the push runs along (REQUIRED — no default). Must
 *                           equal the `startAppYawDeg` the caller passed to `seatAt`, so the parked
 *                           frame and the deposit heading agree. The in-helper self-check is
 *                           magnitude-only, so a wrong/omitted heading would deposit the offset on
 *                           the wrong world axis yet still pass — the parameter is therefore
 *                           mandatory rather than defaulting silently to 0.
 * @return The ACHIEVED offset magnitude (`sqrtf(x*x + y*y)` of `getOffsetFromNode()`), always
 *         >= @p targetCm on success.
 *
 * @note ASSERTS the achieved magnitude >= @p targetCm. A zero-accumulation regression (the two-tick
 *       reopen gate broken, or the deadband gate mis-tuned) fails HERE, loudly, instead of silently
 *       degrading the return case to `ReturnAction::NONE` and passing green against the wrong path.
 *
 * @warning PRECONDITION (gate-open): call AFTER `seatAt(anchor, alongAppHeadingDeg)` — which sets
 *          `_zeroed` and clears the offset via `declarePosition` — with the FSM IDLE (so nav stays
 *          inactive; `resetWorld()` leaves it inactive and a tick with the FSM IDLE keeps it so), and
 *          BEFORE `goTo(...)`, which re-activates nav and freezes the offset for the DEQUEUE
 *          return-gate read.
 * @note    Callers must keep @p targetCm comfortably below `Config::RETURN_DISTANCE_CAP_CM` (50):
 *          the loop overshoots the target by at most one `kOffsetPushStepCm` step, so a target near
 *          the cap could nudge the achieved magnitude over it into the OVER_CAP gate path. The
 *          current call sites (6 and 20 cm) sit well under the cap.
 */
inline float pushOperatorOffset(float targetCm, int16_t alongAppHeadingDeg)
{
    // (1) Belt-and-suspenders: ensure the integrator gate's nav term is open (the caller's IDLE-FSM
    // precondition already guarantees this, but a stray tick could have re-activated nav).
    Localization.setNavActive(false);

    // (2) Park the world heading so the deposit decomposes along `alongAppHeadingDeg` (CONTRACT 2:
    // the integrator's IMU read recovers `mmL.mockYawDeg * IMU_YAW_SIGN`, minus the seated offset).
    mmL.mockYawDeg = (double) hwYaw(alongAppHeadingDeg);

    // (3) Reopen edge: the FIRST open tick re-baselines from this single live read and deposits
    // NOTHING. Deposits begin the next open tick.
    cooperativeTick();

    // (4) Drive the integrator: advance both wheels past the deadband each step and tick until the
    // accumulated magnitude reaches the target. `getEncL()/getEncR()` read `MiniR4.M2/M3`, so both
    // must advance (a one-wheel advance halves `dCm` but still deposits — advancing both is faithful
    // to a straight hand-push and keeps the per-step deposit == `kOffsetPushStepCm`).
    const int32_t stepEnc = (int32_t) cmToEncDeg(path_harness::kOffsetPushStepCm);
    float         mag      = 0.0f;
    for (int i = 0; i < path_harness::kMaxOffsetPushTicks; ++i)
    {
        const Offset2D off = Localization.getOffsetFromNode();
        mag                = sqrtf(off.x * off.x + off.y * off.y);
        if (mag >= targetCm)
        {
            break;
        }
        MiniR4.M2.mockAdvanceDegrees(stepEnc);
        MiniR4.M3.mockAdvanceDegrees(stepEnc);
        cooperativeTick();    // Localization.update() -> _integrateOffset deposits along the heading.
    }

    // Re-read after the final deposit tick so the assert + return reflect the latest accumulation.
    const Offset2D off = Localization.getOffsetFromNode();
    mag                = sqrtf(off.x * off.x + off.y * off.y);

    // Guardrail: a broken two-tick reopen gate (or a mis-tuned deadband) accumulates nothing and
    // would silently degrade the consuming return case to ReturnAction::NONE — fail loudly HERE.
    ASSERT_GE(mag, targetCm);

    return mag;
}

// ===========================================================================================
// Inbound-hop drivers (issue #113 cleanup) — shared so the reset-arrival (strict OFF + ON) and
// the negative non-reset case drive the SAME real BLIND dead-reckon walk instead of re-inlining it
// three times. `driveDeadReckonWalk` is the per-tick retire loop; `driveInboundHop` wraps it with
// the reset-world + frame-bind + goTo prologue.
// ===========================================================================================

/**
 * @brief Tick the real FSM until a BLIND dead-reckon advance lands the discrete node on @p toNode.
 *
 * @details
 * The per-tick retire walk shared by every dead-reckon hop in these tests: park the corrected IMU
 * read at @p hwHeadingDeg (so the pre-edge custom-TurnGyro turn settles, CONTRACT 2) and over-advance
 * `MiniR4.M2`/`M3` + hold the done-bit (so the move retires under EITHER regime — the profiled
 * `SYNC_TSHAPE` encoder gate or the flat done-bit drain, CONTRACT 1) every tick until `Localization`
 * reports @p toNode. Bounded by `kMaxHandshakeTicks` so a real-FSM regression that never arrives fails
 * as a budget-exhausted assert, not a hang.
 *
 * @param toNode       The node id the dead-reckon advance must land on.
 * @param hwHeadingDeg The inbound edge's forward heading in the HARDWARE frame (apply `hwYaw(...)` to
 *                     the app-frame `edge.heading_deg`) — parked every tick so the pre-turn settles.
 * @note Asserts arrival on @p toNode.
 */
inline void driveDeadReckonWalk(uint8_t toNode, int16_t hwHeadingDeg)
{
    for (int i = 0;
         i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != toNode;
         ++i)
    {
        mmL.mockYawDeg = (double) hwHeadingDeg;
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), toNode);    // the dead-reckon advance landed.
}

/**
 * @brief Reset the world, seat at @p fromName, and drive the real BLIND dead-reckon hop onto @p toName.
 *
 * @details
 * The shared inbound prologue for the reset-arrival (strict OFF + ON) and non-reset cases: from a
 * clean world, bind the frame at app-yaw 0 and seat `Localization` at @p fromName, pin that
 * `fromName -> toName` is a DIRECT routed hop (so a fixture re-weight fails loudly rather than silently
 * routing elsewhere), `goTo(toName).async()`, then `driveDeadReckonWalk` onto @p toName. Node-kind
 * specifics (RESET vs BLIND, edge type) are asserted by the CALLER — this helper is node-kind agnostic
 * so it serves both the reset arrivals and the non-reset case.
 *
 * @param fromName Origin node (resolved by name, issue #91).
 * @param toName   Destination node the single BLIND hop lands on (resolved by name).
 * @note FATAL preconditions (#5): if either name fails to resolve, or the hop is not a direct route,
 *       this RETURNS right after recording the assert — the rest of the body (and the caller's arrival
 *       asserts) would otherwise cascade against an un-armed FSM.
 */
inline void driveInboundHop(const char* fromName, const char* toName)
{
    resetWorld();    // clears both Drive + PathTracker faults (order-independent cases).

    const uint8_t from = nodeByName(fromName);
    const uint8_t to   = nodeByName(toName);

    // FATAL precondition: a hop whose endpoints do not resolve cannot be driven — return so the
    // caller's downstream asserts do not cascade against an un-seated FSM (#5).
    ASSERT_TRUE(from != FieldMap::NODE_NONE);
    ASSERT_TRUE(to != FieldMap::NODE_NONE);
    if (from == FieldMap::NODE_NONE || to == FieldMap::NODE_NONE)
    {
        return;
    }

    seatAt(fromName, 0);    // bind world frame @ app-yaw 0 + declare Localization at `fromName`.

    // FATAL precondition: pin the hop as a DIRECT routed step (a fixture re-weight that re-routes it
    // would make the walk never arrive) — return on miss so the failure is the routing assert here,
    // not a budget-exhausted arrival assert downstream (#5).
    ASSERT_EQ(routedFirstHop(from, to), to);
    if (routedFirstHop(from, to) != to)
    {
        return;
    }

    const Routes::EdgeInfo& edge      = FieldMap::getEdge(from, to);
    const int16_t           hwHeading = hwYaw((int16_t) lroundf(edge.heading_deg));

    // Enqueue the destination only; PathTracker walks NEXT_HOP. `.async()` so the harness drives the
    // cooperative tick loop instead of the builder's blocking drain.
    PathTracker.goTo(to).async();

    driveDeadReckonWalk(to, hwHeading);
    ASSERT_FALSE(PathTracker.hasFault());    // a clean arrival (any post-arrival interlude is pending).
}

#endif    // TEST_PATH_TRACKER_HARNESS_H
