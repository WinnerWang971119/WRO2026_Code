// =========================================================================================
// Host unit test for the YawSource external-gyro read adapter (YawSource.cpp, ADR-071).
//
// YawSource is the SOLE owner of the external MXColor-protocol gyro board's yaw I/O and the
// SOLE applier of `Config::GYRO_YAW_SIGN` (read) / `Config::IMU_YAW_SIGN` (write, unchanged).
// The retired STM32-IMU read path (`mmL.GetIMUEuler`, the internal retry-once backoff) is GONE
// from `readYawDeg` — the gyro read has no retry delay. `mmL.mockYawDeg` / `mmL.mockResult` now
// drive a BRIDGE inside the host `MatrixColor` mock (`hal_mocks/MatrixMiniR4.h`) instead of the
// STM32 surface directly, so the 17 legacy FSM suites that scripted yaw through `mmL.mockYawDeg`
// keep passing untouched. This test drives that bridge, plus the raw-bytes injection path the
// bridge sits beside:
//
//   TC13 (bridged read)   — `readYawDeg(out)` decodes the bridged (G,R) bytes and returns
//                            `mmL.mockYawDeg * Config::IMU_YAW_SIGN` (the bridge's reverse-decode
//                            is built so the gyro sign/scale cancel out against it — see
//                            hal_mocks/MatrixMiniR4.h's `MatrixColor` class doc). Two signs of
//                            `mockYawDeg` prove the sign is applied exactly once, not doubled.
//   TC14 (dead raw bytes) — `mockUseRawBytes=true` with `mockG=mockR=0` (an all-zero pair) makes
//                            `readYawDeg` return false with `yawOut` UNTOUCHED (no `0.0`-derived
//                            garbage heading).
//   TC14b (legacy failure injection) — `mmL.mockResult = ERROR` (bridge mode, raw bytes NOT
//                            injected) synthesizes that same dead-byte pattern, so the retired
//                            STM32 failure-injection idiom the 17 FSM suites use still fails a
//                            read post-migration.
//   TC17 (tick cache)     — one `update()` + three `readYawDeg` reads in the same (frozen) mock
//                            tick cost exactly ONE device sample (`MatrixColor::mockDeviceReads
//                            == 1`, AC-5): `update()`'s read populates the cache; the three
//                            `readYawDeg` calls all serve it.
//   TC18 (inline refresh) — once the cache ages past `Config::GYRO_CACHE_MAX_AGE_MS`, the next
//                            `readYawDeg` refreshes INLINE and reflects a changed `mockYawDeg`
//                            rather than serving the now-stale cached value.
//   TC19 (begin liveness) — `begin()` returns false ONLY on a dead device (all-zero G/R read),
//                            and true on a live device even when the vendor `MatrixColor::begin()`
//                            itself reports false — that vendor result is inconclusive (see the
//                            `@note` on `YawSourceClass::begin()`); only the G/R liveness pair is
//                            authoritative.
//   TC20 (zeroHardware)   — a successful `begin()` issues exactly one `getColor(K)` zero read.
// HOW THE SUT IS LINKED (see CMakeLists.txt `test_yaw_source`):
//   - `YawSource.cpp` is compiled UNCHANGED. It pulls `<MatrixMiniR4.h>` + quoted `"Config.h"`
//     via the `mocks/`-first `COMMON_INCLUDES`, so the mock gyro surface (`MatrixColor`, `mmL`,
//     `MMLower::RESULT`) and the host `Arduino.h` (`millis()` / `mockAdvanceMillis()`) resolve
//     without the real SDK.
//   - This target pins `GYRO_SCALE_DEFAULT=1.0f` / `GYRO_YAW_SIGN_DEFAULT=-1.0f` via
//     `target_compile_definitions` (T3b) — REQUIRED for the bridge's whole-degree round-trip to
//     be exact; `Config::IMU_YAW_SIGN` stays at its shipped default (`-1.0f`), unpinned here.
//   - `MOCK_SOURCES` supplies the `Serial` / `g_mockMillis` globals; `mmL` and `MatrixColor`'s
//     `mock*` statics are `inline` / `static inline`, so no extra source defines them.
//
// TEST ORDERING NOTE: TC13/TC14/TC14b run BEFORE any `begin()`/`update()` call in this file, so
// `YawSource`'s tick-scoped cache is never written before them — every `readYawDeg` in those
// blocks exercises the inline-refresh path against freshly-scripted mock state, never a stale
// cache from an earlier block.
//
// The pass/fail + final-line conventions mirror `test_localization_offset.cpp`.
// =========================================================================================

#include <cstdio>

#include "src/nav/YawSource.h"
#include "Config.h"           // Config::GYRO_CACHE_MAX_AGE_MS + mock <Arduino.h>.
#include "MatrixMiniR4.h"     // mock: MatrixColor::mock* + mmL.mockYawDeg / mockResult.

// The mock yaw bridge (hal_mocks/MatrixMiniR4.h) round-trips mmL.mockYawDeg through readYawDeg
// with a fixed -1.0f sign (its kBridgeImuSign) — byte-identical to the retired production
// Config::IMU_YAW_SIGN but INDEPENDENT of it, so this test survives IMU_YAW_SIGN's removal (#169).
static constexpr double kBridgedYawSign = -1.0;

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
    // TC13 (bridged read, positive mockYawDeg) — readYawDeg decodes the bridged bytes back to
    // mockYawDeg * IMU_YAW_SIGN (IMU_YAW_SIGN == -1.0f, unpinned on this target).
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = 90.0;

        double y  = 123.0;    // sentinel — must be overwritten on success.
        bool   ok = YawSource.readYawDeg(y);
        check(ok, "TC13: readYawDeg returns true on a bridged OK read");
        check(y == 90.0 * kBridgedYawSign,
              "TC13: bridged read equals mockYawDeg * bridge sign (90 -> -90)");
    }

    // -------------------------------------------------------------------------------------
    // TC13 (bridged read, negative mockYawDeg) — a second value proves the sign is applied
    // exactly once (not doubled) in either direction.
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = -30.0;

        double y  = 0.0;
        bool   ok = YawSource.readYawDeg(y);
        check(ok, "TC13: readYawDeg returns true on a bridged OK read (negative mockYawDeg)");
        check(y == -30.0 * kBridgedYawSign,
              "TC13: bridged read equals mockYawDeg * bridge sign (-30 -> +30), proves the sign");
    }

    // -------------------------------------------------------------------------------------
    // TC14 (dead raw bytes) — mockUseRawBytes injects an all-zero (G,R) pair directly. readYawDeg
    // must return false and leave yawOut UNTOUCHED (no 0.0-derived garbage heading).
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        MatrixColor::mockUseRawBytes = true;
        MatrixColor::mockG           = 0;
        MatrixColor::mockR           = 0;

        double y  = 123.0;    // prior value the caller is holding — must survive a dead read.
        bool   ok = YawSource.readYawDeg(y);
        check(!ok, "TC14: readYawDeg returns false on an all-zero raw (G,R) pair (dead bus)");
        check(y == 123.0, "TC14: a dead-byte read leaves yawOut untouched");
    }

    // -------------------------------------------------------------------------------------
    // TC14b (legacy failure injection) — mmL.mockResult = ERROR in BRIDGE mode (raw bytes NOT
    // injected) synthesizes the same dead-byte pattern, proving the retired STM32
    // failure-injection idiom (still used by 17 FSM suites) still fails a read post-migration.
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::ERROR;    // mockUseRawBytes stays false (bridge mode).

        double y  = 123.0;
        bool   ok = YawSource.readYawDeg(y);
        check(!ok, "TC14b: readYawDeg returns false when mmL.mockResult == ERROR (bridged dead-byte)");
        check(y == 123.0, "TC14b: a bridged failure leaves yawOut untouched");
    }

    // -------------------------------------------------------------------------------------
    // TC17 (tick cache) — one update() + three readYawDeg reads in the same frozen mock tick
    // cost exactly ONE device sample. update()'s read populates the cache; the three readYawDeg
    // calls all serve it without touching the device again (AC-5).
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = 45.0;

        YawSource.update();    // the ONE device transaction this sequence should cost.

        double y1 = 0.0, y2 = 0.0, y3 = 0.0;
        const bool ok1 = YawSource.readYawDeg(y1);
        const bool ok2 = YawSource.readYawDeg(y2);
        const bool ok3 = YawSource.readYawDeg(y3);

        check(ok1 && ok2 && ok3, "TC17: all three cached reads succeed");
        check(y1 == 45.0 * kBridgedYawSign && y2 == y1 && y3 == y1,
              "TC17: all three cached reads return the same tick-scoped value");
        check(MatrixColor::mockDeviceReads == 1,
              "TC17: update() + 3 reads in one tick cost exactly one device sample (AC-5)");
    }

    // -------------------------------------------------------------------------------------
    // TC18 (inline refresh) — once the cache ages past GYRO_CACHE_MAX_AGE_MS, the next
    // readYawDeg refreshes INLINE and reflects a changed mockYawDeg, not the stale cached value.
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = 45.0;

        YawSource.update();    // seed the cache at the current mock clock.

        double cached = 0.0;
        check(YawSource.readYawDeg(cached), "TC18: precondition — the fresh cache serves a read");
        check(cached == 45.0 * kBridgedYawSign,
              "TC18: precondition — the cached value matches the seeded mockYawDeg");

        // Push the mock clock strictly past the staleness backstop, then change the underlying
        // heading — a still-served cache would return the OLD (cached) value here.
        mockAdvanceMillis((uint32_t) Config::GYRO_CACHE_MAX_AGE_MS + 1);
        mmL.mockYawDeg = 60.0;

        double fresh = 0.0;
        const bool ok = YawSource.readYawDeg(fresh);
        check(ok, "TC18: a stale-cache read still succeeds (inline refresh)");
        check(fresh == 60.0 * kBridgedYawSign,
              "TC18: a stale cache triggers an inline refresh reflecting the NEW mockYawDeg");
        check(fresh != cached, "TC18: the refreshed value differs from the stale cached value");
    }

    // -------------------------------------------------------------------------------------
    // TC19 (begin liveness, dead device) — begin() returns false when the G/R pair reads
    // all-zero (dead bus / no device on the selected mux channel).
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        MatrixColor::mockUseRawBytes = true;
        MatrixColor::mockG           = 0;
        MatrixColor::mockR           = 0;

        const bool ok = YawSource.begin();
        check(!ok, "TC19: begin() returns false when the device reads all-zero (dead bus)");
    }

    // -------------------------------------------------------------------------------------
    // TC19 (begin liveness, live device) — begin() returns true on a live device EVEN IF the
    // vendor MatrixColor::begin() itself reports false: that result is inconclusive (it
    // ID-checks 0x43 and writes a CONFIG reset the board may not implement), so only the G/R
    // liveness pair is authoritative.
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        MatrixColor::mockBeginOk = false;    // vendor begin() reports false — must be ignored.
        mmL.mockResult           = MMLower::RESULT::OK;
        mmL.mockYawDeg           = 15.0;     // live, nonzero bridged heading.

        const bool ok = YawSource.begin();
        check(ok, "TC19: begin() returns true on a live device even when the vendor begin() is "
                  "false (inconclusive vendor result)");
    }

    // -------------------------------------------------------------------------------------
    // TC20 (zeroHardware on begin) — a successful begin() issues exactly one getColor(K) zero
    // read (AC-7).
    // -------------------------------------------------------------------------------------
    {
        MatrixColor::resetMock();
        mmL.mockReset();
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = 20.0;    // live, nonzero bridged heading.

        const bool ok = YawSource.begin();
        check(ok, "TC20: begin() succeeds on a live device");
        check(MatrixColor::mockKReadCount == 1,
              "TC20: begin() zeroes the hardware exactly once (getColor(K))");
    }

    if (g_failures == 0)
    {
        std::printf("YAW SOURCE OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
