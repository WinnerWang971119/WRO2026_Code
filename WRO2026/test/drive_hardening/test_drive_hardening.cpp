/**
 * @file test_drive_hardening.cpp
 * @brief Host-link tests for the Drive-lane hardening batch (issues #141-159 hardening plan,
 *        `docs/plans/2026-07-15-issues-141-159-hardening.md`).
 *
 * @details
 * This is the FIRST target in the `Drive.cpp` hardening lane (T1 #142 -> T2 #150 -> T3 #153 ->
 * T4 #154-16 -> T5 #141). T1 (#142, the in-flight turn passed-target guard) was dropped from
 * this PR and handed off separately, so this file starts at TC7 (#150). T3 and T4 APPEND
 * TC8/TC9 to this SAME file/target later in the chain — new tests must be added as additional
 * standalone `void tcN_...()` functions and dispatched from `main()`, never as a second target.
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
///       file even though TC7 is currently the only one — T3/T4 append siblings that will also
///       call `resetWorld()` first.
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
// Entry. T3 (#153) / T4 (#154-16) append their `tc8_.../tc9_...` calls here.
// ===========================================================================================
int main()
{
    std::printf("INFO: Drive-lane hardening (#141-159 batch) — real Drive.cpp, real YawSource.cpp\n");

    tc7_drain_fault_early_out();

    TEST_REPORT();
}
