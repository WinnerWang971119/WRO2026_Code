// =========================================================================================
// Host unit test for the external gyro wire-decode + wrap + plausibility math
// (GyroDecodePolicy.h, ADR-071, TC1-TC12).
//
// `gyroRawBiased` / `gyroWrap180` / `decodeGyroYawDeg` / `gyroBytesDead` / `gyroShortestDelta`
// / `gyroReadPlausible` are header-only inline free functions with NO Arduino / Config
// surface (only `<stdint.h>` + `<math.h>`), so production and this host test link the SAME
// symbols — the established `TurnSignPolicy.h` / `PivotTrimPolicy.h` pattern (the precedent
// this file mirrors: see `test_turn_sign_policy.cpp`). This TU includes ONLY the header — no
// Arduino / Config mocks, no SUT source files.
//
//   TC1      — decodeGyroYawDeg: raw 32768 (mid-scale bias point) decodes to exactly 0.0 deg.
//   TC2/TC3  — decodeGyroYawDeg: a known raw value at scale=0.997, sign=+1 vs sign=-1; the
//              sign flip is an exact negation of the +1 result (both land away from the
//              +/-180 wrap boundary, where negate-before-wrap and negate-after-wrap agree).
//   TC4      — gyroWrap180: the (-180, +180] boundary is closed at the top, open at the
//              bottom, and every wrap result lands back in that same canonical range.
//   TC5      — gyroRawBiased: the two byte-pair extremes bias to the signed 16-bit extremes
//              with no overflow.
//   TC6      — large accumulation (biased at the +32767 / -32768 extremes) scaled then
//              wrapped lands at the correct wrapped value.
//   TC7      — gyroBytesDead: only the (0,0) byte pair is flagged dead.
//   TC8      — gyroShortestDelta: a legitimate +179/-179 wrap yields the SHORT step (+/-2),
//              never the long way around (358).
//   TC9-TC12 — gyroReadPlausible: the rate bound is time-normalized by elapsedMs (a bigger
//              gap widens the bound), a real wrap-around step is accepted, an oversized step
//              in a short window is rejected, and elapsedMs==0 is treated as unbounded (no
//              divide-by-zero, no false rejection).
//
// Integer-valued cases compare exactly; float cases use a tight tolerance to absorb float32
// rounding, and every wrap result is additionally checked against the canonical (-180, +180]
// range. The pass/fail + final-line conventions mirror `test_turn_sign_policy.cpp`.
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "src/nav/GyroDecodePolicy.h"

static const float kEps = 1e-3f;

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

static bool nearlyEqual(float got, float want, float eps)
{
    return std::fabs(got - want) < eps;
}

static bool inCanonicalRange(float deg)
{
    return deg > -180.0f && deg <= 180.0f;
}

int main()
{
    // -------------------------------------------------------------------------------------
    // TC1 — decodeGyroYawDeg: raw 32768 (hiG=0x80, loR=0x00) biases to 0 -> decodes to 0.0.
    // -------------------------------------------------------------------------------------
    check(nearlyEqual(decodeGyroYawDeg(0x80, 0x00, 1.0f, 1.0f), 0.0f, kEps),
          "TC1: raw 32768 (0x80,0x00) biases to 0 -> decodes to 0.0 deg");

    // -------------------------------------------------------------------------------------
    // TC2/TC3 — decodeGyroYawDeg at scale=0.997: hiG=0x84, loR=0x00 -> raw16=0x8400=33792
    // -> biased=1024 -> appDeg=0.997*1024=1020.928 -> wrap180 folds to -59.072 (sign=+1).
    // sign=-1 is the exact negation, +59.072 (both results sit well inside the canonical
    // range, away from the +/-180 boundary where negate-before/after-wrap could diverge).
    // -------------------------------------------------------------------------------------
    check(nearlyEqual(decodeGyroYawDeg(0x84, 0x00, 0.997f, 1.0f), -59.072f, kEps),
          "TC2: known raw at scale=0.997, sign=+1 decodes to -59.072 deg");
    check(nearlyEqual(decodeGyroYawDeg(0x84, 0x00, 0.997f, -1.0f), 59.072f, kEps),
          "TC3: sign=-1 mirrors TC2 as an exact negation (+59.072 deg)");

    // -------------------------------------------------------------------------------------
    // TC4 — gyroWrap180 boundaries: closed at +180, open at -180 (folds to +180); values
    // outside the range wrap the correct direction; every result stays in (-180, +180].
    // -------------------------------------------------------------------------------------
    check(gyroWrap180(180.0f) == 180.0f, "TC4: +180 stays +180 (range closed at the top)");
    check(gyroWrap180(-180.0f) == 180.0f, "TC4: -180 folds to +180 (range open at the bottom)");
    check(gyroWrap180(270.0f) == -90.0f, "TC4: a value >180 wraps down (270 -> -90)");
    check(gyroWrap180(-270.0f) == 90.0f, "TC4: a value <-180 wraps up (-270 -> +90)");
    check(inCanonicalRange(gyroWrap180(180.0f)), "TC4: wrap180(180) is in (-180, +180]");
    check(inCanonicalRange(gyroWrap180(-180.0f)), "TC4: wrap180(-180) is in (-180, +180]");
    check(inCanonicalRange(gyroWrap180(270.0f)), "TC4: wrap180(270) is in (-180, +180]");
    check(inCanonicalRange(gyroWrap180(-270.0f)), "TC4: wrap180(-270) is in (-180, +180]");
    check(inCanonicalRange(gyroWrap180(540.0f)), "TC4: wrap180(540) is in (-180, +180]");
    check(inCanonicalRange(gyroWrap180(-540.0f)), "TC4: wrap180(-540) is in (-180, +180]");

    // -------------------------------------------------------------------------------------
    // TC5 — gyroRawBiased: byte-pair extremes bias to the signed 16-bit extremes, exactly,
    // with no overflow.
    // -------------------------------------------------------------------------------------
    check(gyroRawBiased(0x00, 0x00) == -32768, "TC5: gyroRawBiased(0x00,0x00) == -32768");
    check(gyroRawBiased(0xFF, 0xFF) == 32767, "TC5: gyroRawBiased(0xFF,0xFF) == 32767");

    // -------------------------------------------------------------------------------------
    // TC6 — large accumulation near the +/-32768 extremes, scaled (scale=1.0) then wrapped:
    // +32767 mod 360 = 7 (360*91=32760); -32768 mod 360 = -8 (360*-91=-32760, remainder -8).
    // -------------------------------------------------------------------------------------
    check(nearlyEqual(decodeGyroYawDeg(0xFF, 0xFF, 1.0f, 1.0f), 7.0f, kEps),
          "TC6: biased +32767 scaled by 1.0 wraps to +7.0 deg (32767 mod 360)");
    check(nearlyEqual(decodeGyroYawDeg(0x00, 0x00, 1.0f, 1.0f), -8.0f, kEps),
          "TC6: biased -32768 scaled by 1.0 wraps to -8.0 deg (-32768 mod 360)");

    // -------------------------------------------------------------------------------------
    // TC7 — gyroBytesDead: only the all-zero byte pair is flagged dead.
    // -------------------------------------------------------------------------------------
    check(gyroBytesDead(0, 0) == true, "TC7: gyroBytesDead(0,0) is true");
    check(gyroBytesDead(0, 1) == false, "TC7: gyroBytesDead(0,1) is false");
    check(gyroBytesDead(1, 0) == false, "TC7: gyroBytesDead(1,0) is false");

    // -------------------------------------------------------------------------------------
    // TC8 — gyroShortestDelta: a legitimate 179/-179 wrap yields the SHORT step (+/-2), not
    // the long way around (358). Symmetric in both directions.
    // -------------------------------------------------------------------------------------
    check(gyroShortestDelta(179.0f, -179.0f) == -2.0f,
          "TC8: gyroShortestDelta(179,-179) == -2 (not 358)");
    check(gyroShortestDelta(-179.0f, 179.0f) == 2.0f,
          "TC8: gyroShortestDelta(-179,179) == +2 (symmetric)");

    // -------------------------------------------------------------------------------------
    // TC9-TC12 — gyroReadPlausible: time-normalized rate gate.
    // -------------------------------------------------------------------------------------
    check(gyroReadPlausible(-179.0f, 179.0f, 8, 1000.0f) == true,
          "TC9: the legitimate 179->-179 wrap (2 deg step) is accepted in 8 ms");
    check(gyroReadPlausible(100.0f, 0.0f, 8, 1000.0f) == false,
          "TC10: 100 deg step in 8 ms exceeds the 1000 deg/s rate bound (8 deg max)");
    check(gyroReadPlausible(100.0f, 0.0f, 500, 1000.0f) == true,
          "TC11: the SAME 100 deg step is accepted once more time has elapsed (500 ms -> 500 "
          "deg max) -- proves the bound scales with elapsed time, not a flat per-read constant");
    check(gyroReadPlausible(1000.0f, -1000.0f, 0, 1000.0f) == true,
          "TC12: elapsedMs==0 is treated as unbounded (accept), never a divide-by-zero");

    if (g_failures == 0)
    {
        std::printf("GYRO DECODE POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
