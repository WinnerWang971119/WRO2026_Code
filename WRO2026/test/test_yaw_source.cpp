// =========================================================================================
// Host unit test for the YawSource CW->CCW adapter (YawSource.cpp, ADR-044, TC3/TC4).
//
// FIRST host target to compile the REAL production `YawSource.cpp` standalone. YawSource is
// the SOLE owner of lower-MCU IMU yaw I/O and the SOLE applier of `Config::IMU_YAW_SIGN`:
// it bridges the STM32 CW-positive raw frame and the app CCW-positive world frame at exactly
// two points. This test drives both:
//
//   TC3 (read-sign)    — `readYawDeg(out)` multiplies the raw `mmL` yaw by IMU_YAW_SIGN
//                        (-1.0f on the shipped chassis), so a raw -90 reads back as +90 and
//                        a raw +30 reads back as -30. Returns true and writes `out`.
//   TC3 (failure)      — a hard IMU failure (both attempts non-OK) returns false and leaves
//                        `out` UNTOUCHED (no 0.0-derived garbage heading). The retry-once
//                        MMLower-caller convention is asserted via the mock call counter
//                        (exactly 2 `GetIMUEuler` calls on a hard miss).
//   TC3 (readYawDegOr) — the fire-and-forget wrapper returns the corrected value on OK and
//                        the caller's fallback on failure.
//   TC4 (toHwHeading)  — the app->STM32 pure sign flip: app +90 -> raw -90, and the mirror.
//
// HOW THE SUT IS LINKED (see CMakeLists.txt `test_yaw_source`):
//   - `YawSource.cpp` is compiled UNCHANGED. It pulls `<MatrixMiniR4.h>` + quoted
//     `"Config.h"` via the `mocks/`-first `COMMON_INCLUDES`, so the mock IMU surface
//     (`mmL`, `MMLower::RESULT`) and the host `Arduino.h` (`delay()` on the retry path)
//     resolve without the real SDK. NO Drive.h force-include is needed — YawSource.cpp
//     includes no production header to shadow.
//   - `MOCK_SOURCES` supplies the `Serial` / `g_mockMillis` globals; `mmL` is an `inline`
//     variable in the mock header, so no extra source defines it.
//
// The shipped `Config::IMU_YAW_SIGN` default is `-1.0f`, so every expected value below is
// the raw input negated. All quantities are integer-valued, so the asserts compare exactly.
// The pass/fail + final-line conventions mirror `test_localization_offset.cpp`.
// =========================================================================================

#include <cstdio>

#include "YawSource.h"
#include "MatrixMiniR4.h"    // mock: mmL.GetIMUEuler + mmL.mockYawDeg / mockResult / getImuEulerCallCount.

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
    // TC3 (read-sign, negative raw) — a raw -90 deg yaw reads back as +90 (IMU_YAW_SIGN = -1).
    // -------------------------------------------------------------------------------------
    {
        mmL.mockReset();
        mmL.mockYawDeg = -90.0;
        mmL.mockResult = MMLower::RESULT::OK;

        double y  = 123.0;    // sentinel — must be overwritten on success.
        bool   ok = YawSource.readYawDeg(y);
        check(ok, "TC3: readYawDeg returns true on an OK read");
        check(y == 90.0, "TC3: raw -90 reads back as +90 (IMU_YAW_SIGN = -1)");
        check(mmL.getImuEulerCallCount == 1, "TC3: a clean OK read issues exactly one GetIMUEuler");
    }

    // -------------------------------------------------------------------------------------
    // TC3 (read-sign, positive raw) — a raw +30 deg yaw reads back as -30.
    // -------------------------------------------------------------------------------------
    {
        mmL.mockReset();
        mmL.mockYawDeg = 30.0;
        mmL.mockResult = MMLower::RESULT::OK;

        double y  = 0.0;
        bool   ok = YawSource.readYawDeg(y);
        check(ok, "TC3: readYawDeg returns true on an OK read (positive raw)");
        check(y == -30.0, "TC3: raw +30 reads back as -30 (IMU_YAW_SIGN = -1)");
    }

    // -------------------------------------------------------------------------------------
    // TC3 (failure) — a hard IMU failure returns false, leaves `out` UNTOUCHED, and proves
    // the retry-once convention: exactly two GetIMUEuler attempts before giving up.
    // -------------------------------------------------------------------------------------
    {
        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::ERROR;

        double y  = 555.0;    // prior value the caller is holding — must survive a hard miss.
        bool   ok = YawSource.readYawDeg(y);
        check(!ok, "TC3: readYawDeg returns false on a hard IMU failure");
        check(y == 555.0, "TC3: a failed read leaves the out-param untouched (no garbage heading)");
        check(mmL.getImuEulerCallCount == 2, "TC3: a hard miss retries once (two GetIMUEuler calls)");
    }

    // -------------------------------------------------------------------------------------
    // readYawDegOr — returns the corrected value on OK, the caller's fallback on failure.
    // -------------------------------------------------------------------------------------
    {
        mmL.mockReset();
        mmL.mockYawDeg = -45.0;
        mmL.mockResult = MMLower::RESULT::OK;
        check(YawSource.readYawDegOr(-1.0) == 45.0, "readYawDegOr: OK case returns the corrected value (+45)");

        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::ERROR;
        check(YawSource.readYawDegOr(7.0) == 7.0, "readYawDegOr: failure case returns the supplied fallback");
    }

    // -------------------------------------------------------------------------------------
    // TC4 (toHwHeading) — app CCW-positive heading -> STM32 raw CW-positive gyro-PID target,
    // a pure sign flip (IMU_YAW_SIGN = -1). int16_t compares are exact for these +/-1 signs.
    // -------------------------------------------------------------------------------------
    {
        check(YawSource.toHwHeading(90) == -90, "TC4: toHwHeading(+90) == -90");
        check(YawSource.toHwHeading(-90) == 90, "TC4: toHwHeading(-90) == +90");
        check(YawSource.toHwHeading(0) == 0, "TC4: toHwHeading(0) == 0");
        check(YawSource.toHwHeading(45) == -45, "TC4: toHwHeading(+45) == -45");
    }

    if (g_failures == 0)
    {
        std::printf("YAW SOURCE OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
