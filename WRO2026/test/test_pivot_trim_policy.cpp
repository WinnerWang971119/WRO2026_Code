// =========================================================================================
// Host unit test for resolvePivotTrim (PivotTrimPolicy.h, ADR-036 / #11).
//
// The pivot trim subtract-and-clamp was promoted out of PathTracker.cpp into a
// header-only inline policy so production AND this host test link the SAME symbol
// (the DirectionPolicy.h / StopAtLinePolicy.h pattern, PR #34). The policy is pure
// float math with no Arduino surface, so this TU includes only the header.
//
// Parity: keep these cases consistent with
// tools/path_planner/sim/runtime.py::PathTrackerSim._pivot_trim_distance and the
// validator's gate boundary tests (tests/test_field_data.py TC6b).
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

    // No pivot latched: drives the raw edge length, no clamp.
    {
        const PivotTrimResolution r = resolvePivotTrim(100.0f, 0.0f, 0.0f, floor);
        check(approx(r.driveCm, 100.0f), "no-trim drives the raw length");
        check(approx(r.effectiveCm, 100.0f), "no-trim effective == raw");
        check(!r.clamped, "no-trim does not clamp");
    }

    // Departing trim only: subtract the outgoing over-travel.
    {
        const PivotTrimResolution r = resolvePivotTrim(100.0f, 9.0f, 0.0f, floor);
        check(approx(r.driveCm, 91.0f), "departing trim subtracted");
        check(!r.clamped, "departing within length does not clamp");
    }

    // Approaching trim only.
    {
        const PivotTrimResolution r = resolvePivotTrim(100.0f, 0.0f, 9.0f, floor);
        check(approx(r.driveCm, 91.0f), "approaching trim subtracted");
    }

    // Chained pivots: both trims subtract from the shared edge.
    {
        const PivotTrimResolution r = resolvePivotTrim(30.0f, 9.0f, 9.0f, floor);
        check(approx(r.driveCm, 12.0f), "chained trims both subtracted");
        check(!r.clamped, "chained within floor does not clamp");
    }

    // Underflow inside the floor band: clamp, report it, expose the pre-clamp eff
    // for the trace (matches validator TC6b: 9.3 - 9.0 = 0.3 < floor).
    {
        const PivotTrimResolution r = resolvePivotTrim(9.3f, 9.0f, 0.0f, floor);
        check(r.clamped, "underflow within floor clamps");
        check(approx(r.driveCm, floor), "clamp drives the floor");
        check(approx(r.effectiveCm, 0.3f), "pre-clamp eff exposed for the trace");
    }

    // Over-trim (negative pre-clamp): still degrades to the floor, never negative.
    {
        const PivotTrimResolution r = resolvePivotTrim(5.0f, 9.0f, 0.0f, floor);
        check(r.clamped, "over-trim clamps");
        check(approx(r.driveCm, floor), "over-trim drives the floor, never negative");
    }

    if (g_failures == 0)
    {
        std::printf("PIVOT TRIM POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
