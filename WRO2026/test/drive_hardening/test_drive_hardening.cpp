/**
 * @file test_drive_hardening.cpp
 * @brief Host-link tests for the Drive-lane hardening batch (issues #141-159 hardening plan,
 *        `docs/plans/2026-07-15-issues-141-159-hardening.md`).
 *
 * @details
 * This is the FIRST target in the `Drive.cpp` hardening lane (T1 #142 -> T2 #150 -> T3 #153 ->
 * T4 #154-16 -> T5 #141). T1 (#142, the in-flight turn passed-target guard) was dropped from
 * this PR and handed off separately, so this file starts at TC7 (#150). T5 (#141) APPENDS
 * further tests to this SAME file/target later in the chain — new tests must be added as
 * additional standalone `void tcN_...()` functions and dispatched from `main()`, never as a
 * second target.
 *
 * TC7 (#150, drain fault early-out) — `Drive::_drainBlocking()`'s spin loop now checks
 * `hasFault()` as the very first statement of every iteration. Without that check, a sync verb
 * issued while a fault is already latched enqueues successfully (`_enqueue` does not gate on
 * `hasFault()`) but can never dispatch (`_dispatchNext`'s own fault gate refuses to pop the
 * queue while `hasFault()`), so the blocking destructor's second `_drainBlocking()` call would
 * spin for the ENTIRE `(qCount+1) * DRIVE_DRAIN_TIMEOUT_MS_PER_CMD` budget and then clobber the
 * ORIGINAL fault kind with `DRAIN_TIMEOUT`. TC7 proves both halves of the fix: (a) the second
 * call returns on its very first loop iteration — asserted by the mock clock (`g_mockMillis`)
 * being byte-identical before and after, since the fixed path never reaches a single `delay()`
 * call — and (b) `_fault.kind` still reads the ORIGINAL fault (`IMU_READ_FAILED`), never
 * `DRAIN_TIMEOUT`.
 *
 * HOW THE SUT IS LINKED (mirrors the `test_drive_decel_floor` real-link recipe — see the block
 * comment above `test_path_tracker_passthrough_on` in CMakeLists.txt — MINUS PathTracker.cpp /
 * Localization.cpp / the fixture routes, since this target drives `Drive` directly and never
 * touches PathTracker):
 *   - Links the REAL `Drive.cpp` + `YawSource.cpp` (Drive.cpp's turn-profile state machine calls
 *     `YawSource::readYawDeg` / `readYawDegOr`, and its GYRO emit branch references
 *     `YawSource::toHwHeading` — the symbol must resolve at link time even though every
 *     scenario here uses the default SYNC mode and never executes that branch) + `path_mocks.cpp`
 *     (facade globals: `MiniR4` / `Serial` / `Serial1` / `EventTrace` / `g_mockMillis` — NOT
 *     `Drive`, so it never collides with the real `Drive.cpp`, the sole definer of `Drive`).
 *   - `${SKETCH_DIR}` leads the include path (the inversion) so the real `Drive.h` / `YawSource.h`
 *     win; `${HAL_MOCKS_DIR}` supplies the angle-bracket `<MatrixMiniR4.h>` / `<Arduino.h>` (the
 *     `MiniR4.M2`/`M3` mock encoders, the `mmL` mock lower-MCU IMU surface, and the
 *     `MiniR4.DriveDC` mock recorder this test reads); `${PATH_MOCKS_DIR}` supplies the
 *     force-included `WROLineArray.h` / `Intersection.h` / `LineFollower.h` peripherals the real
 *     `Drive.cpp` translation unit needs to link (its seek/dispatch machinery references them)
 *     even though this target never dispatches a `stopAtLine`/`wallStop`.
 *   - `${MOCKS_DIR}` (the OLDER `mocks/Drive.h` mock-`DriveClass` world) is OMITTED entirely —
 *     that mock's own `Fault` enum + `setSpeed`/queue model would false-green every assertion
 *     here (memory: gotcha-drive-not-host-compiled). This target links the REAL `Drive.cpp`;
 *     every fault/timing assertion is read off `Drive.hasFault()` / `Drive.getFault()` and the
 *     mock clock `g_mockMillis`, i.e. the production fault-state machine, not a mock stand-in.
 *
 * TICK IDIOM: a sustained IMU-read failure is scripted via `mmL.mockResult =
 * MMLower::RESULT::ERROR` (mirrors `test_yaw_source.cpp`), then `Drive.update()` is called
 * repeatedly. `YawSource::readYawDeg`'s internal retry-once backoff calls `delay(2)` on every
 * failed read, which is what actually advances the mock clock tick-by-tick here — no manual
 * `mockAdvanceMillis()` is needed to reach `Config::TURN_IMU_STALL_TIMEOUT_MS`.
 *
 * TC8 (#153, fabricated 0.0 yaw fallback) — `_execTurnDeg`, `_execTurnToYaw`, `getYaw()`, and
 * `holdYaw()` each read the current yaw via `YawSource.readYawDegOr(fallback)`; before the fix
 * every one of them passed a literal `0.0` fallback, so a double-failed IMU read silently
 * substituted a fabricated "due north" heading instead of the last known-good one. TC8 seeds a
 * known-good `_heldYaw` via a successful read, then forces every further read to fail
 * (`mmL.mockResult = MMLower::RESULT::ERROR`) and proves each of the four call sites now falls
 * back to `_heldYaw`: a relative `turnDeg` computes the correct absolute target (cur + delta,
 * not 0 + delta), an absolute `turnToYaw` short-circuits correctly instead of arming a spurious
 * CRUISE turn, and `holdYaw()` leaves `_heldYaw` at its prior value instead of collapsing it to
 * 0. Dispatch uses `.async()` + exactly one `Drive.update()` tick so the assertions land before
 * any further ticks could accumulate toward the persistent-IMU-stall watchdog TC7 exploits.
 *
 * TC9 (#154-16, wallStop SQUARE fault guard) — `_updateWallStop()`'s SQUARE branch now checks
 * `hasFault()` before its "still squaring?" check. `_updateTurnProfile()` runs BEFORE
 * `_updateWallStop()` every tick, so a fault that escalates from the square-up turn's own
 * persistent-IMU-read-failure watchdog (the same TC7/TC8 mechanism) collapses
 * `_turnProfileState` back to IDLE as part of ITS teardown, on the SAME tick — which, without the
 * guard, SQUARE misreads as "squared successfully" and wrongly hands off to PUSH right after the
 * chassis was hard-stopped for the fault. TC9 reuses TC7's persistent-IMU-dropout script during
 * an armed square-up turn and proves `_endWallStop(true)` fires instead: `Drive.isIdle()` reads
 * true on the fault tick (not the bug's `_wallState == PUSH`), and a further tick issues no
 * additional `MoveGyro` call (PUSH never actually drives the wheels).
 */

#include "TestAssert.h"

#include <cstdio>

#include "src/drive/Drive.h"    // real — DriveClass + the global `Drive` (defined in real Drive.cpp)
#include "MatrixMiniR4.h"       // mock MiniR4.M2/M3/DriveDC/mmL — hal_mocks/, resolved via the -I search order.

namespace
{
/// @brief Reset every mock actuator + Drive's own instance state to a clean slate, mirroring
///        `test_drive_decel_floor.cpp`'s `resetWorld()`.
///
/// @note `mockResetMiniR4()` does NOT reset `mmL` (the lower-MCU IMU mock is a separate global,
///       not a `MiniR4` child) or the mock clock — both are reset explicitly here, matching
///       `test_yaw_source.cpp`'s per-scenario reset idiom.
/// @note `Drive.clearFault()` guarantees a clean fault slate between test functions in this
///       file — TC7/TC8/TC9 each call `resetWorld()` first, and T5 will do the same for any
///       siblings it appends.
void resetWorld()
{
    mockResetMiniR4();
    mmL.mockReset();
    mockResetMillis();
    Drive.begin();
    Drive.stop();
    Drive.clearFault();
}

/// @brief Tick `Drive.update()` until `Drive.hasFault()` latches or @p maxTicks is exhausted
///        (a bug that stops the watchdog from tripping fails loudly instead of hanging ctest).
void tickUntilFault(int maxTicks)
{
    for (int i = 0; i < maxTicks && !Drive.hasFault(); ++i)
    {
        Drive.update();
    }
}
}    // namespace

// ===========================================================================================
// TC7 (#150) — a sync Drive verb issued while a fault is already latched must return
// immediately from `_drainBlocking()`, preserving the ORIGINAL `_fault.kind` (never clobbered
// to `DRAIN_TIMEOUT`).
//
// Setup: dispatch an async `Drive.turnDeg(90)` while the mock lower-MCU IMU read
// (`mmL.mockResult`) is scripted to fail every call. `Drive::_updateTurnProfile()`'s
// persistent-read-failure watchdog (`turnImuReadStalled`, `Config::TURN_IMU_STALL_TIMEOUT_MS`)
// escalates to a real, distinct-kind `Fault::IMU_READ_FAILED` — hard-stopping the motors and
// clearing the queue, exactly like a genuine sustained IMU dropout on hardware.
//
// Exercise: with that fault latched, issue a brand-new sync verb (`Drive.driveDist(10.0f)`,
// default blocking). Its destructor calls `_drainBlocking()` TWICE (drain-before, enqueue,
// drain-after — see `DriveDistBuilder::~DriveDistBuilder`). The first call is a no-op (the
// queue is already empty and the chassis is already idle from the fault handler). The second
// call is where the bug lived: the fresh enqueue makes `_qCount > 0`, and since `_dispatchNext`
// itself refuses to pop the queue while `hasFault()`, the old code spun for the FULL
// `(qCount+1) * DRIVE_DRAIN_TIMEOUT_MS_PER_CMD` budget before its timeout branch fired and
// stamped `_fault.kind = Fault::DRAIN_TIMEOUT` over the original `IMU_READ_FAILED`.
//
// Assert: the mock clock (`g_mockMillis`) is UNCHANGED across the sync verb call (the fixed
// loop returns on its very first iteration, before any `delay()`), and `_fault.kind` still
// reads `IMU_READ_FAILED` — proving both "no full-budget spin" and "kind preserved".
// ===========================================================================================
void tc7_drain_fault_early_out()
{
    TEST_SCOPE("TC7 (#150): sync verb enqueued while faulted — _drainBlocking early-outs, "
               "fault kind preserved");

    resetWorld();

    // Script a sustained lower-MCU IMU read failure (mirrors test_yaw_source.cpp's ERROR case).
    mmL.mockResult = MMLower::RESULT::ERROR;

    ASSERT_FALSE(Drive.hasFault());    // precondition: clean slate before the scripted fault.

    // Dispatch a 90 degree turn asynchronously so the test controls the tick pace directly
    // (a blocking `.turnDeg()` would immediately call `_drainBlocking()` itself, entangling
    // the fault-latching phase with the very drain this test wants to observe separately).
    // Single-temporary RAII enqueue — never store the builder in a named local (it is
    // move-only and a stored local corrupts the queue, per the Drive builder RAII gotcha).
    Drive.turnDeg(90).async();

    // Tick until the persistent-IMU-read-failure watchdog escalates to a hard fault. Each
    // failed read costs one internal `delay(IMU_READ_RETRY_DELAY_MS)` retry inside
    // `YawSource::readYawDeg`, so the mock clock advances ~2ms/tick; the watchdog trips once
    // that accumulated time reaches `Config::TURN_IMU_STALL_TIMEOUT_MS` (500ms by default) —
    // budget the tick cap generously so a future retune of either constant doesn't false-fail.
    tickUntilFault(2000);

    ASSERT_TRUE(Drive.hasFault());
    ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::IMU_READ_FAILED);

    // --- The actual T2/#150 exercise -------------------------------------------------------
    const uint32_t millisBeforeSyncVerb = g_mockMillis;

    // Default-blocking verb, issued while the IMU_READ_FAILED fault is still latched. Emitted
    // as a single temporary (RAII gotcha) — its destructor drives the two `_drainBlocking()`
    // calls under test.
    Drive.driveDist(10.0f);

    const uint32_t millisAfterSyncVerb = g_mockMillis;

    // No full-budget spin: the fixed `_drainBlocking()` bails out on its very first loop
    // iteration, before reaching a single `delay()` call, so the mock clock must not have
    // moved at all across the sync verb. Without the fix this would advance by roughly
    // `(qCount+1) * Config::DRIVE_DRAIN_TIMEOUT_MS_PER_CMD` (thousands of ms).
    ASSERT_EQ(millisAfterSyncVerb, millisBeforeSyncVerb);

    // Fault kind preserved: still the ORIGINAL IMU_READ_FAILED, never clobbered to
    // DRAIN_TIMEOUT.
    ASSERT_TRUE(Drive.hasFault());
    ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::IMU_READ_FAILED);
}

// ===========================================================================================
// TC8 (#153) — a turn/hold dispatched while the IMU read fails must fall back to `_heldYaw`
// (the last known-good heading), never to a fabricated `0.0`.
//
// Background: `_execTurnDeg`, `_execTurnToYaw`, `getYaw()`, and `holdYaw()` each read the
// current yaw via `YawSource.readYawDegOr(fallback)`. Before the fix, every call site passed
// a literal `0.0` fallback — a double-failed IMU read (both the primary attempt and the
// internal retry-after-`delay()` come back non-OK) silently substituted a fabricated "facing
// due north" heading. For `_execTurnDeg`, that fabricated `cur` is added to the caller's
// relative delta, aiming the turn at the WRONG absolute target. For `_execTurnToYaw`, the
// fabricated `cur` corrupts the tolerance/short-circuit decision (the delta-vs-target
// computation), possibly arming a full CRUISE turn where the true heading was already at
// target (or vice versa). `holdYaw()` would collapse `_heldYaw` itself to 0, corrupting every
// later yaw-locked move. The fix threads `_heldYaw` through as the fallback at all four sites
// instead.
//
// Since `_heldYaw` is a private member, the only public window onto it is `Drive.getYaw()`
// (which itself now falls back to `_heldYaw` on a failed read) — so each scenario below seeds
// a known-good `_heldYaw` via a successful read, then forces every subsequent IMU read to fail
// and observes the fallback value through `getYaw()` / `isTurnDone()`.
//
// Each sub-scenario dispatches via `.async()` + exactly ONE `Drive.update()` tick. Per
// `DriveClass::update()` (Drive.cpp ~:90-151), stage 2 (`_updateTurnProfile()`, an idempotent
// no-op while `_turnProfileState == IDLE`) runs BEFORE stage 3 (`_dispatchNext()`), so a single
// tick dispatches `_execTurnDeg`/`_execTurnToYaw` exactly once and returns without entering the
// CRUISE/BRAKE state machine on this same tick — no risk of accumulating enough failed reads to
// trip the persistent-IMU-stall watchdog (`Config::TURN_IMU_STALL_TIMEOUT_MS`) that TC7 exploits
// deliberately; TC8 must NOT trip it, or the fault would mask the fallback assertions below.
// ===========================================================================================
void tc8_fabricated_yaw_fallback()
{
    TEST_SCOPE("TC8 (#153): failed IMU read during turn/hold dispatch falls back to _heldYaw, "
               "never a fabricated 0.0");

    // --- Sub-scenario A: _execTurnDeg (Drive.cpp ~:354) + getYaw() (~:516) -----------------
    // Seed _heldYaw = 50 via a successful read, then force every further read to fail and
    // dispatch a RELATIVE turn of +45 degrees. `_execTurnDeg`'s target = cur + deg, so a
    // correctly-recovered cur (50, from _heldYaw) yields target 95; the old fabricated-0.0 bug
    // would instead compute target = 0 + 45 = 45. Either branch (short-circuit or CRUISE-arm)
    // unconditionally sets `_heldYaw = target`, so the post-dispatch `getYaw()` fallback read
    // (still failing) surfaces exactly that target: 95.0 with the fix, 45.0 with the bug —
    // a clean, non-tautological, numerically-distinct assertion.
    {
        resetWorld();

        // `mmL.mockYawDeg` is the RAW pre-sign-flip value: `YawSource::readYawDeg` applies
        // `appYaw = Config::IMU_YAW_SIGN * raw` (IMU_YAW_SIGN == -1.0f on this chassis), so a
        // raw mock of -50 reads back as the app-frame yaw +50.
        mmL.mockResult  = MMLower::RESULT::OK;
        mmL.mockYawDeg  = -50.0;
        Drive.holdYaw();    // successful read seeds _heldYaw = 50.

        mmL.mockResult = MMLower::RESULT::ERROR;    // every subsequent IMU read now fails.

        Drive.turnDeg(45).async();    // enqueue only — no drain, no execution yet.
        Drive.update();               // exactly one tick: dispatches _execTurnDeg.

        ASSERT_FALSE(Drive.hasFault());    // one failed read at dispatch must not trip the
                                            // persistent-stall watchdog (re-seeded at arm time).
        ASSERT_EQ(Drive.getYaw(), 95.0);    // 50 (held) + 45 (delta) — never 0 + 45 = 45.
    }

    // --- Sub-scenario B: _execTurnToYaw (Drive.cpp ~:416) -----------------------------------
    // Seed _heldYaw = 100, then dispatch an ABSOLUTE turnToYaw(100) — i.e. a true no-op turn.
    // `_execTurnToYaw`'s target is the caller's absolute yaw (100) regardless of cur, so both
    // the fix and the bug set `_heldYaw = 100` — but the SHORT-CIRCUIT decision (whether the
    // turn is already within `Config::TURN_TOLERANCE_DEG` of target) depends entirely on cur:
    // a correctly-recovered cur (100) makes |target-cur| = 0, short-circuiting immediately
    // (`isTurnDone()` true). The old fabricated-0.0 bug would compute |100-0| = 100, arming a
    // full (and entirely spurious) CRUISE turn instead (`isTurnDone()` false).
    {
        resetWorld();

        // Raw mock -100 -> app-frame yaw +100 (IMU_YAW_SIGN == -1.0f; see sub-scenario A).
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = -100.0;
        Drive.holdYaw();    // successful read seeds _heldYaw = 100.

        mmL.mockResult = MMLower::RESULT::ERROR;

        Drive.turnToYaw(100).async();
        Drive.update();    // exactly one tick: dispatches _execTurnToYaw.

        ASSERT_FALSE(Drive.hasFault());
        ASSERT_TRUE(Drive.isTurnDone());    // short-circuited — cur correctly recovered as 100,
                                             // not fabricated as 0 (which would arm a spurious
                                             // 100-degree CRUISE turn and leave this false).
        ASSERT_TRUE(Drive.isMoveDone());    // the short-circuit branch collapses both bits.
    }

    // --- Sub-scenario C: holdYaw() (Drive.cpp ~:523) ----------------------------------------
    // Seed _heldYaw = 77 via a successful read, then call holdYaw() again while every read
    // fails. The fix keeps _heldYaw at 77 (fallback = its own prior value); the old
    // fabricated-0.0 bug would collapse it to 0. Read the result back through getYaw()'s own
    // (already-proven, by sub-scenario A) fallback.
    {
        resetWorld();

        // Raw mock -77 -> app-frame yaw +77 (IMU_YAW_SIGN == -1.0f; see sub-scenario A).
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = -77.0;
        Drive.holdYaw();    // successful read seeds _heldYaw = 77.

        mmL.mockResult = MMLower::RESULT::ERROR;
        Drive.holdYaw();    // failed read — must NOT collapse _heldYaw to 0.

        ASSERT_FALSE(Drive.hasFault());
        ASSERT_EQ(Drive.getYaw(), 77.0);    // _heldYaw preserved at 77, never reset to 0.
    }
}

// ===========================================================================================
// TC9 (#154-16) — a fault latched during the wallStop SQUARE (square-up) phase must abort the
// push cleanly via `_endWallStop(true)`; SQUARE must NEVER hand off to PUSH once `hasFault()`.
//
// Background: `wallStop()` dispatch arms the square-up turn via `_execTurnToYaw` (Drive.cpp
// ~:1762) and enters `WallStopState::SQUARE`. Each tick, `_updateWallStop()`'s SQUARE branch
// waits for `_turnProfileState == IDLE` before seeding the push watchdogs and transitioning to
// PUSH. The bug: `_updateTurnProfile()` runs BEFORE `_updateWallStop()` on every tick (Drive.cpp
// `update()` stage 2 ordering), so if the square-up turn's own persistent-IMU-read-failure
// watchdog escalates to a hard `Fault::IMU_READ_FAILED` (exactly TC7/TC8's mechanism), that fault
// handler collapses `_turnProfileState` back to IDLE as PART OF its own teardown — on the SAME
// tick. Without a `hasFault()` guard, SQUARE's "still squaring?" check misreads that collapse as
// "squared successfully" and wrongly advances straight to PUSH, re-arming continuous motion
// (a fresh encoder baseline + a live push PWM) immediately after the chassis was just
// hard-stopped for a real fault. The fix checks `hasFault()` FIRST, at the very top of the SQUARE
// branch, and aborts via `_endWallStop(true)` (active-brake, mirrors every other fault-triggered
// wallStop abort — `stop()` and the `_drainBlocking` DRAIN_TIMEOUT path both use the same call).
//
// Setup: dispatch an async `Drive.wallStop(20.0f).setHeading(90)` while the IMU reads healthy, so
// dispatch arms a REAL square-up turn (current yaw 0 -> target 90, well past
// `Config::TURN_TOLERANCE_DEG`) rather than short-circuiting as a no-op. One tick with a healthy
// IMU is enough to dispatch the WALL_STOP command and enter SQUARE with `_turnProfileState ==
// CRUISE`. Then, exactly like TC7, the mock lower-MCU IMU read is scripted to fail every call
// (`mmL.mockResult = MMLower::RESULT::ERROR`) and ticked until the persistent-read-failure
// watchdog escalates (`Config::TURN_IMU_STALL_TIMEOUT_MS`).
//
// Assert: (a) the escalation really did latch `Fault::IMU_READ_FAILED` (the real mechanism, not a
// test-only hook); (b) `Drive.isIdle()` is TRUE immediately after that tick — proving
// `_endWallStop(true)` fired and collapsed `_wallState` back to IDLE (`_moveDone`/`_turnDone`)
// on the SAME tick the fault latched, rather than the bug's `_wallState == PUSH` /
// `_moveDone == false`; (c) one further tick issues NO additional `MoveGyro` call (wallStop's
// default push mode is GYRO) — proving PUSH never actually ran and drove the wheels. Without the
// fix, (b) fails immediately (SQUARE fell through to PUSH on the fault tick) and (c) would also
// fail the following tick (PUSH would command the flat push PWM via `MoveGyro`).
// ===========================================================================================
void tc9_wallstop_square_fault_guard()
{
    TEST_SCOPE("TC9 (#154-16): fault latched during wallStop SQUARE — _endWallStop(true) fires, "
               "PUSH never runs");

    resetWorld();

    // Seed a known-good _heldYaw = 0 via a successful read (mirrors TC8's seeding idiom). Raw
    // mock 0.0 -> app-frame yaw 0.0 (IMU_YAW_SIGN == -1.0f; see TC8 sub-scenario A).
    mmL.mockResult = MMLower::RESULT::OK;
    mmL.mockYawDeg = 0.0;
    Drive.holdYaw();

    // Dispatch an async wallStop whose square-up heading (90) differs from the current yaw (0) by
    // far more than Config::TURN_TOLERANCE_DEG, so `_execTurnToYaw` arms a real CRUISE turn
    // instead of short-circuiting as a no-op square-up. Single-temporary RAII enqueue (Drive
    // builder RAII gotcha) — never store the builder in a named local.
    Drive.wallStop(20.0f).setHeading(90).async();

    // One tick with a healthy IMU: dispatches WALL_STOP (_execTurnToYaw arms the CRUISE turn,
    // _turnProfileState -> CRUISE) and enters WallStopState::SQUARE.
    Drive.update();

    ASSERT_FALSE(Drive.hasFault());
    ASSERT_FALSE(Drive.isIdle());    // squaring up — neither idle nor faulted yet.

    // Force every subsequent IMU read to fail, exactly like TC7's persistent-dropout script.
    mmL.mockResult = MMLower::RESULT::ERROR;

    // Tick until the persistent-IMU-read-failure watchdog escalates to IMU_READ_FAILED. This is
    // the exact tick the bug lived on: the fault handler collapses _turnProfileState to IDLE
    // INSIDE this same Drive.update() call, before _updateWallStop() runs its SQUARE branch.
    tickUntilFault(2000);

    ASSERT_TRUE(Drive.hasFault());
    ASSERT_TRUE(Drive.getFault().kind == DriveClass::Fault::IMU_READ_FAILED);

    // --- The actual T4/#154-16 exercise -----------------------------------------------------
    // `_endWallStop(true)` must have fired on that very tick: _wallState back to IDLE, _moveDone /
    // _turnDone collapsed true, queue already cleared by the turn-profile fault handler — so
    // Drive.isIdle() (which requires _wallState == IDLE AND _moveDone/_turnDone AND _qCount == 0)
    // reads true. Without the fix, SQUARE fell through to PUSH on this same tick, leaving
    // _wallState == PUSH and _moveDone == false, so this would read false.
    ASSERT_TRUE(Drive.isIdle());

    const uint32_t moveGyroCallsAfterFault = MiniR4.DriveDC.moveGyroCallCount;

    // One further tick. If PUSH had wrongly started (the bug), _updateWallStop's PUSH branch
    // would command the flat push PWM via `_emitContinuous(DriveMode::GYRO, ...)` ->
    // `MiniR4.DriveDC.MoveGyro(...)` (wallStop's default mode is GYRO — see WallStopBuilder). With
    // the fix _wallState is already IDLE, so _updateWallStop() short-circuits at its very first
    // line and nothing fires.
    Drive.update();

    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, moveGyroCallsAfterFault);
    ASSERT_TRUE(Drive.isIdle());
}

// ===========================================================================================
// Entry.
// ===========================================================================================
int main()
{
    std::printf("INFO: Drive-lane hardening (#141-159 batch) — real Drive.cpp, real YawSource.cpp\n");

    tc7_drain_fault_early_out();
    tc8_fabricated_yaw_fallback();
    tc9_wallstop_square_fault_guard();

    TEST_REPORT();
}
