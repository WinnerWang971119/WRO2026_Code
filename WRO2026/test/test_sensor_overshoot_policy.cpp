// =========================================================================================
// Host unit test for resolveSensorOvershootTrim (PivotTrimPolicy.h, issue #65).
//
// This is the SENSOR-anchored twin of test_pivot_trim_policy (the DISTANCE path).
// `resolveSensorOvershootTrim` is the thin wrapper that feeds the sensor-hop post-detect
// axle roll through the shared subtract-and-clamp formula, fixing the approaching pivot
// trim only (departing trim is hard-coded 0 — ADR-044 D1: a sensor-terminated hop
// self-corrects on the line). Promoting the formula to PivotTrimPolicy.h guarantees
// production and this host test link the SAME symbol.
//
// TC1-TC3 from the issue #65 plan:
//   TC1 — normal case (delta < offset): overshoot == offset - delta, not clamped.
//   TC2 — over-trim (delta > offset):   overshoot == floor,           clamped.
//   TC3 — no trim (delta == 0):         overshoot == offset,          not clamped.
//   TC4 — strict boundary (delta == offset - floor): overshoot == floor, NOT clamped
//          (strict-< clamp boundary mirrors test_pivot_trim_policy TC6b).
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "src/nav/PivotTrimPolicy.h"

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

static bool approx(float a, float b)
{
    return std::fabs(a - b) < 1e-4f;
}

int main()
{
    const float floor = 0.5f;    // mirrors Config::PIVOT_TRIM_MIN_DRIVE_CM.

    // TC1: normal trim (delta < offset) — overshoot == offset - delta, not clamped.
    // offset=12.02, delta=9.0  =>  overshoot = 3.02, clamped = false.
    {
        const SensorOvershootResolution r = resolveSensorOvershootTrim(12.02f, 9.0f, floor);
        check(approx(r.overshootCm, 3.02f), "TC1: overshoot == offset - delta");
        check(!r.clamped, "TC1: within floor does not clamp");
    }

    // TC2: over-trim (delta > offset) — overshoot == floor, clamped.
    // offset=12.02, delta=15.588  =>  overshoot = 0.5, clamped = true.
    {
        const SensorOvershootResolution r = resolveSensorOvershootTrim(12.02f, 15.588f, floor);
        check(approx(r.overshootCm, floor), "TC2: over-trim clamps to floor");
        check(r.clamped, "TC2: over-trim is clamped");
    }

    // TC3: no pivot trim (delta == 0) — overshoot == offset, not clamped.
    // offset=12.02, delta=0  =>  overshoot = 12.02, clamped = false.
    {
        const SensorOvershootResolution r = resolveSensorOvershootTrim(12.02f, 0.0f, floor);
        check(approx(r.overshootCm, 12.02f), "TC3: no trim leaves overshoot unchanged");
        check(!r.clamped, "TC3: no trim does not clamp");
    }

    // TC4: strict-< boundary (delta == offset - floor => effective == floor, NOT clamped).
    // offset=12.02, delta=11.52  =>  effective = 0.5 (== floor), clamped = false.
    // The clamp fires only when effective STRICTLY < floor (mirrors TC6b in sibling test).
    {
        const float delta = 12.02f - floor;    // exactly offset - floor => effective == floor.
        const SensorOvershootResolution r = resolveSensorOvershootTrim(12.02f, delta, floor);
        check(approx(r.overshootCm, floor), "TC4: boundary effective==floor yields floor");
        check(!r.clamped, "TC4: effective==floor is NOT clamped (strict-< boundary)");
    }

    if (g_failures == 0)
    {
        std::printf("SENSOR OVERSHOOT POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
