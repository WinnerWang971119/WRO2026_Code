// =========================================================================================
// Host unit test for the EncoderSource lower-MCU encoder / task-done adapter
// (EncoderSource.cpp, ADR-066, issue #141 — TC1/TC1b/TC1c/TC2).
//
// EncoderSource is the SOLE owner of lower-MCU per-motor encoder-degree reads and the drive
// task-done read. The vendor `MiniR4DC::getDegrees` / `MiniR4DriveDC::isPrevTaskDone` DISCARD
// the MMLower RESULT, so a comms timeout surfaces as a silent 0 / a fabricated `true`.
// EncoderSource reads the RESULT-surfacing `mmL.GetEncoderDegrees` / `mmL.Get_Drive_isTaskDone`
// directly and applies one of two policies. This test drives all four AC #141 cases:
//
//   TC1  (continuous)  — a read mocked to time out twice makes `readDegreesOr` serve the
//                        centralized last-good (the caller's prior value); a later success-0
//                        returns 0 (a success-0 is never treated as a failure).
//   TC1b (baseline)    — a baseline read mocked to fail makes `readDegrees` return false and
//                        leaves the caller's out-param UNTOUCHED (the caller does NOT arm on
//                        stale data). The retry-once convention is asserted via the call count.
//   TC1c (cache reset) — `resetCache` after a `resetCounter` prevents `readDegreesOr` from ever
//                        serving the pre-reset value on a subsequent failure.
//   TC2  (task-done)   — a non-`Drive_RESULT::OK` task-done read makes `isPrevTaskDone()` return
//                        false; an OK read honours the vendor done-bit convention.
//
// HOW THE SUT IS LINKED (see CMakeLists.txt `test_encoder_source`):
//   - `EncoderSource.cpp` is compiled UNCHANGED. It pulls `<MatrixMiniR4.h>` + quoted
//     `"Config.h"` via the `mocks/`-first `COMMON_INCLUDES`, so the mock lower-MCU surface
//     (`mmL.GetEncoderDegrees` / `mmL.Get_Drive_isTaskDone`, gated by `mmL.mockEncoderResult` /
//     `mmL.mockTaskDoneResult`) and the host `Arduino.h` (`delay()` on the retry path) resolve
//     without the real SDK. The mock delegates the degrees to the `MiniR4.M<num>` backing and
//     the done-bit to `MiniR4.DriveDC.mockPrevTaskDone`, so a fixture scripts them through the
//     existing per-motor / drive-base setters.
//   - `MOCK_SOURCES` supplies the `Serial` / `g_mockMillis` globals + the `MiniR4` singleton;
//     `mmL` is an `inline` variable in the mock header. `EncoderSource` is defined in its own .cpp.
// =========================================================================================

#include <cstdio>

#include "src/util/EncoderSource.h"
#include "MatrixMiniR4.h"    // mock: mmL.GetEncoderDegrees / Get_Drive_isTaskDone + the settable state.

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

int main()
{
    // -------------------------------------------------------------------------------------
    // TC1 — continuous read: a double timeout serves the centralized last-good; a later
    //       success-0 returns 0.
    // -------------------------------------------------------------------------------------
    {
        MiniR4.mockReset();
        mmL.mockReset();
        EncoderSource.resetCache(2);

        // Seed the last-good with a clean OK read of 500.
        MiniR4.M2.mockSetDegrees(500);
        mmL.mockEncoderResult = MMLower::RESULT::OK;
        check(EncoderSource.readDegreesOr(2, -1) == 500, "TC1: OK read returns the fresh value (500)");

        // Two consecutive timeouts — readDegreesOr must serve the cached last-good, NOT the
        // supplied fallback, and must have retried once (two GetEncoderDegrees calls).
        mmL.mockReset();
        mmL.mockEncoderResult = MMLower::RESULT::ERROR;
        check(EncoderSource.readDegreesOr(2, -777) == 500,
              "TC1: double timeout serves the cached last-good (500), not the fallback");
        check(mmL.getEncoderDegreesCallCount == 2,
              "TC1: a hard miss retries once (two GetEncoderDegrees calls)");

        // A later success returning a genuine 0 must read back as 0 (success-0 is not a failure).
        mmL.mockReset();
        MiniR4.M2.mockSetDegrees(0);
        mmL.mockEncoderResult = MMLower::RESULT::OK;
        check(EncoderSource.readDegreesOr(2, -1) == 0, "TC1: a later success-0 returns 0");
    }

    // -------------------------------------------------------------------------------------
    // TC1b — baseline read: a hard failure returns false and leaves `out` UNTOUCHED, so the
    //        caller keeps its own value and does NOT arm on stale data.
    // -------------------------------------------------------------------------------------
    {
        MiniR4.mockReset();
        mmL.mockReset();
        EncoderSource.resetCache(3);

        mmL.mockEncoderResult = MMLower::RESULT::ERROR;

        int32_t out = 12345;    // the caller's prior baseline — must survive a hard miss.
        bool    ok  = EncoderSource.readDegrees(3, out);
        check(!ok, "TC1b: readDegrees returns false on a hard failure");
        check(out == 12345, "TC1b: a failed baseline read leaves the out-param untouched (no arm on stale)");
        check(mmL.getEncoderDegreesCallCount == 2, "TC1b: a hard miss retries once (two calls)");

        // A clean read succeeds and DOES publish the value + refresh the cache.
        mmL.mockReset();
        MiniR4.M3.mockSetDegrees(77);
        mmL.mockEncoderResult = MMLower::RESULT::OK;
        int32_t out2 = -1;
        check(EncoderSource.readDegrees(3, out2), "TC1b: readDegrees returns true on an OK read");
        check(out2 == 77, "TC1b: an OK read writes the fresh value to out");
    }

    // -------------------------------------------------------------------------------------
    // TC1c — resetCache after a resetCounter: a subsequent failed read must NOT serve the
    //        pre-reset value; a subsequent OK read serves the post-reset physical 0.
    // -------------------------------------------------------------------------------------
    {
        MiniR4.mockReset();
        mmL.mockReset();
        EncoderSource.resetCache(4);

        // Populate the cache with 800.
        MiniR4.M4.mockSetDegrees(800);
        mmL.mockEncoderResult = MMLower::RESULT::OK;
        check(EncoderSource.readDegreesOr(4, -1) == 800, "TC1c: cache populated with 800");

        // Re-zero the encoder (resetCounter) and invalidate the cache (resetCache).
        MiniR4.M4.resetCounter();    // zeroes the M4 backing, as the real reset does
        EncoderSource.resetCache(4);

        // A failed read must fall back to the caller's fallback, never the pre-reset 800.
        mmL.mockEncoderResult = MMLower::RESULT::ERROR;
        int32_t v = EncoderSource.readDegreesOr(4, 42);
        check(v == 42, "TC1c: after resetCache, a failed read serves the fallback, not the stale value");
        check(v != 800, "TC1c: the pre-reset value (800) can never be served after resetCache");

        // A successful read after the reset serves the post-reset physical 0.
        mmL.mockEncoderResult = MMLower::RESULT::OK;
        check(EncoderSource.readDegreesOr(4, -1) == 0, "TC1c: post-reset OK read serves 0");
    }

    // -------------------------------------------------------------------------------------
    // TC2 — task-done: `isPrevTaskDone()` is true ONLY on an OK read whose done-bit reports the
    //       task finished; any non-OK read is "not done". Passes DRIVE_INSTANCE_ID to the vendor.
    // -------------------------------------------------------------------------------------
    {
        MiniR4.mockReset();
        mmL.mockReset();

        // OK read + done-bit set → true.
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        mmL.mockTaskDoneResult = MMLower::Drive_RESULT::OK;
        check(EncoderSource.isPrevTaskDone(), "TC2: OK read + done-bit → true");
        check(mmL.lastTaskDoneNum == EncoderSourceClass::DRIVE_INSTANCE_ID,
              "TC2: the read passes DRIVE_INSTANCE_ID (1) to the vendor task-done API");

        // OK read + still-running → false.
        MiniR4.DriveDC.mockSetPrevTaskDone(false);
        mmL.mockTaskDoneResult = MMLower::Drive_RESULT::OK;
        check(!EncoderSource.isPrevTaskDone(), "TC2: OK read + running → false");

        // Non-OK read → false, even though the (stale) done flag says done.
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        mmL.mockTaskDoneResult = MMLower::Drive_RESULT::ERROR;
        check(!EncoderSource.isPrevTaskDone(), "TC2: a non-OK task-done read is treated as NOT done");
    }

    if (g_failures == 0)
    {
        std::printf("ENCODER SOURCE OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
