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
 *   - **Custom TurnGyro profile** (`turnToYaw().setMode(...)` pre-edge turn): owns `_turnDone`
 *     via `_updateTurnProfile`, which drives `Move(L,R)` until the corrected IMU yaw read
 *     (`YawSource.readYawDeg` -> `mmL.mockYawDeg * IMU_YAW_SIGN`) is within `TURN_TOLERANCE_DEG`
 *     of the app-frame target (Drive.cpp:1124). RETIRED by parking `mmL.mockYawDeg` at the
 *     hardware-frame value for the app target, then a tick.
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
#include "Drive.h"           // real — DriveClass + the global `Drive` (defined in real Drive.cpp)
#include "FieldMap.h"        // real — getNextHop / getEdge / findNodeByName
#include "Localization.h"    // real — declarePosition / onIntersection / zeroHeadingFromCurrent
#include "PathTracker.h"     // real — PathTrackerClass + the global `PathTracker`
#include "YawSource.h"       // real — YawSource.toHwHeading (static; call as YawSource.method())

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
 *   5. `Drive.begin()`            — real Drive: clears fault, seeds done flags true, pushes PID.
 *   6. `Localization.begin()`     — real Localization: seeds `_currentNode = START_NODE` (=Start,
 *                                   BLIND), clears offset + integrator baselines + `_zeroed`.
 *   7. `PathTracker.begin()`      — real PathTracker: FSM to IDLE, queue empty, fault cleared.
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
// Per-phase done-bit / sensor handshake helpers
// ===========================================================================================

/**
 * @brief Drive an in-flight pre-edge TURN to completion at a scripted app-frame yaw target.
 *
 * @details
 * A pre-edge turn dispatches `Drive.turnToYaw(Localization.toImuYaw(bodyHeading)).setMode(byte)
 * .async()` (PathTracker.cpp:1649), which runs the CUSTOM TurnGyro profile (`_updateTurnProfile`,
 * Drive.cpp:1075). That profile drives `Move(L,R)` every tick and completes when the CORRECTED
 * IMU yaw (`YawSource.readYawDeg` -> `mmL.mockYawDeg * IMU_YAW_SIGN`) lands within
 * `Config::TURN_TOLERANCE_DEG` of the app-frame target `_turnTargetYaw` (Drive.cpp:1124), at
 * which point it BRAKEs and sets `_turnDone`/`_moveDone`. PathTracker's `TURNING` state then sees
 * `Drive.isIdle()` and dispatches the edge (PathTracker.cpp:680).
 *
 * This helper PARKS `mmL.mockYawDeg` at the hardware-frame value of @p appYawTarget so the
 * profile's next read sees the chassis already AT the target, then ticks until `Drive.isIdle()`.
 * One tick is normally enough (the FSM reads the parked yaw, transitions CRUISE/SLOW -> BRAKE ->
 * IDLE), but the loop is budgeted to absorb a SLOW-zone intermediate tick.
 *
 * @param appYawTarget The APP-frame (CCW-positive) yaw the chassis turns to (deg). CONTRACT 2:
 *                     converted to the hw frame internally, so the test passes the app value.
 * @param wheelByte    The wheel-pattern byte the turn was dispatched with
 *                     (`Routes::TurnMode` cast to `uint8_t`: COUNTER_ROT=1 / PIVOT_LEFT=2 /
 *                     PIVOT_RIGHT=3, or the pipeline default). Recorded for the assert below.
 *
 * @note Asserts the turn actually COMPLETED: `Drive.isIdle()` and `Drive.isTurnDone()` true after
 *       the handshake (the representative observable for a turn — the custom TurnGyro profile
 *       never issues a lower-MCU `MoveGyroDegs`, so there is no `lastMoveGyroDegsTarget` to check;
 *       the gyro-target assert belongs to the GYRO MOVE helpers). @p wheelByte is accepted so the
 *       signature documents the dispatched pattern and a future TurnMode-recording assert has the
 *       expected value at the call site; it is not otherwise consulted (the mock TurnGyro profile
 *       drives `Move(L,R)`, whose L/R sign encodes the pattern, not a recorded mode byte).
 */
inline void expectPreTurn(int16_t appYawTarget, uint8_t wheelByte)
{
    (void) wheelByte;    // documents the dispatched pattern; see @note (no recorded mode byte).

    // Park the corrected IMU read AT the app-frame target (hw frame = app * IMU_YAW_SIGN), so the
    // custom TurnGyro profile's next `readYawDeg` sees the chassis already aligned and BRAKEs.
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

#endif    // TEST_PATH_TRACKER_HARNESS_H
