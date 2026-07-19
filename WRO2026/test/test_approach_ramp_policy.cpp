// =========================================================================================
// Host unit test for ApproachRampPolicy.h (2026-07-17 anticipatory-line-approach-ramp plan, T3).
//
// Exercises `resolveApproachSchedule` + `approachSpeedAt` in isolation: pure float / int16_t
// math, zero Arduino / MatrixMiniR4 / Config.h surface (same convention as
// `test_pivot_trim_policy.cpp` / `test_sensor_overshoot_policy.cpp`). The real `Config::*`
// knobs are mirrored as local literals (see the comment on each constant below) rather than
// pulling in Config.h, so this target needs no mocks and no SUT `.cpp`.
//
// Covers (Testing Strategy table, docs/plans/2026-07-17-anticipatory-line-approach-ramp.md):
//   TC1  — swept `traveledCm` over a nominal hop: CRUISE -> RAMP (monotone non-increasing) ->
//          FLOOR_HOLD at `floorMag` -> OVERRUN (still returns `floorMag`), for two different
//          cruise/floor pairings so the coverage isn't an artifact of one specific ramp length.
//   TC7  — degrade to `active == false` on a short hop (`predictedCm < minMoveCm`) AND on a hop
//          that clears `minMoveCm` but leaves no room for the ramp (`rampStartCm <= 0`), plus the
//          strict `<` (not `<=`) boundary on `minMoveCm` in isolation.
//   TC17 — `predictedCm` plumbs straight through with NO internal offset math: shifting
//          `predictedCm` by a delta shifts every schedule boundary by the EXACT same delta, and
//          `approachSpeedAt` returns identical (pwmMag, phase) pairs at the same OFFSET into the
//          schedule regardless of where `predictedCm` sits.
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "src/drive/ApproachRampPolicy.h"

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

static const char* phaseName(ApproachPhase p)
{
    switch (p)
    {
        case ApproachPhase::CRUISE:
            return "CRUISE";
        case ApproachPhase::RAMP:
            return "RAMP";
        case ApproachPhase::FLOOR_HOLD:
            return "FLOOR_HOLD";
        case ApproachPhase::OVERRUN:
            return "OVERRUN";
    }
    return "?";
}

/**
 * @brief Sweep `traveledCm` from `startCm` to `endCm` (inclusive) in `stepCm` increments,
 *        asserting that: (1) the phase `approachSpeedAt` returns matches the schedule's own
 *        boundaries, (2) CRUISE samples pin to `cruiseMag` and FLOOR_HOLD/OVERRUN samples pin
 *        to `floorMag` exactly, and (3) `pwmMag` never INCREASES across the whole sweep (TC1).
 */
static void sweepAndCheck(const ApproachSchedule& s, float startCm, float endCm, float stepCm,
                          const char* label)
{
    bool    havePrev = false;
    int16_t prevMag  = 0;

    for (float t = startCm; t <= endCm + 1e-6f; t += stepCm)
    {
        const ApproachPhase expectedPhase = (t < s.rampStartCm)     ? ApproachPhase::CRUISE
                                            : (t < s.floorStartCm)   ? ApproachPhase::RAMP
                                            : (t < s.overrunLimitCm) ? ApproachPhase::FLOOR_HOLD
                                                                     : ApproachPhase::OVERRUN;
        const ApproachSpeed speed = approachSpeedAt(s, t);

        char msg[160];
        std::snprintf(msg, sizeof(msg), "%s: traveled=%.3f expected phase %s, got %s", label,
                      (double) t, phaseName(expectedPhase), phaseName(speed.phase));
        check(speed.phase == expectedPhase, msg);

        if (speed.phase == ApproachPhase::CRUISE)
        {
            check(speed.pwmMag == s.cruiseMag, "sweep: CRUISE pins to cruiseMag");
        }
        else if (speed.phase == ApproachPhase::FLOOR_HOLD
                 || speed.phase == ApproachPhase::OVERRUN)
        {
            check(speed.pwmMag == s.floorMag, "sweep: FLOOR_HOLD/OVERRUN pins to floorMag");
        }

        if (havePrev)
        {
            std::snprintf(msg, sizeof(msg),
                          "%s: traveled=%.3f pwmMag %d must be <= previous %d (non-increasing)",
                          label, (double) t, (int) speed.pwmMag, (int) prevMag);
            check(speed.pwmMag <= prevMag, msg);
        }
        prevMag  = speed.pwmMag;
        havePrev = true;
    }
}

int main()
{
    // Shared inputs mirroring the real firmware Config surface (ConfigDrive.h / ConfigGlobal.h) —
    // hardcoded literals rather than #include "Config.h", same convention as the sibling
    // pivot-trim / sensor-overshoot policy tests.
    const int16_t CRUISE_MAG    = 70;      // mirrors Config::SPEED_NORMAL
    const int16_t FLOOR_MAG     = 30;      // mirrors Config::PWM_MOVING_MIN
    const float   ACCEL         = 4.0f;    // mirrors Config::TSHAPE_ACCEL_PWM_PER_CM
    const float   RAMP_MIN      = 5.0f;    // mirrors Config::TSHAPE_RAMP_MIN_CM
    const float   RAMP_MAX      = 20.0f;   // mirrors Config::TSHAPE_RAMP_MAX_CM
    const int16_t NO_RAMP_EPS   = 5;       // mirrors Config::TSHAPE_NO_RAMP_EPSILON_PWM
    const float   MIN_MOVE_CM   = 5.0f;    // mirrors Config::TSHAPE_MIN_MOVE_CM
    const float   MARGIN_CM     = 4.0f;    // mirrors Config::LINE_APPROACH_MARGIN_CM
    const float   MAX_MARGIN_CM = 15.0f;   // mirrors Config::LINE_APPROACH_MAX_MARGIN_CM

    // ---- TC1: nominal hop, cruise=70 -> floor=30, predicted=50cm (sensor-frame, offset already
    //           applied by the caller). rampLenCm = |70-30|/4 = 10cm (within [5,20], no clamp).
    //           floorStartCm = 50-4 = 46; rampStartCm = 46-10 = 36; overrunLimitCm = 50+15 = 65.
    {
        const ApproachSchedule s = resolveApproachSchedule(50.0f, MARGIN_CM, MAX_MARGIN_CM,
                                                            CRUISE_MAG, FLOOR_MAG, ACCEL, RAMP_MIN,
                                                            RAMP_MAX, NO_RAMP_EPS, MIN_MOVE_CM);
        check(s.active, "TC1: nominal hop schedule is active");
        check(approx(s.floorStartCm, 46.0f), "TC1: floorStartCm == predictedCm - marginCm");
        check(approx(s.rampStartCm, 36.0f), "TC1: rampStartCm == floorStartCm - rampLenCm");
        check(approx(s.overrunLimitCm, 65.0f), "TC1: overrunLimitCm == predictedCm + maxMarginCm");
        check(s.cruiseMag == CRUISE_MAG, "TC1: cruiseMag carried through unchanged");
        check(s.floorMag == FLOOR_MAG, "TC1: floorMag carried through unchanged");

        sweepAndCheck(s, -10.0f, 90.0f, 0.5f, "TC1 nominal");
    }

    // ---- TC1b: a second cruise/floor pairing (SPEED_FAST-like) with a longer ramp, so the
    //            sweep+monotonicity coverage isn't an artifact of one specific delta.
    //            rampLenCm = |90-30|/4 = 15cm; floorStartCm = 80-4 = 76; rampStartCm = 76-15 =
    //            61; overrunLimitCm = 80+15 = 95.
    {
        const ApproachSchedule s = resolveApproachSchedule(80.0f, MARGIN_CM, MAX_MARGIN_CM, 90,
                                                            FLOOR_MAG, ACCEL, RAMP_MIN, RAMP_MAX,
                                                            NO_RAMP_EPS, MIN_MOVE_CM);
        check(s.active, "TC1b: fast-cruise hop schedule is active");
        check(approx(s.rampStartCm, 61.0f), "TC1b: rampStartCm");
        check(approx(s.floorStartCm, 76.0f), "TC1b: floorStartCm");
        check(approx(s.overrunLimitCm, 95.0f), "TC1b: overrunLimitCm");

        sweepAndCheck(s, -10.0f, 110.0f, 0.5f, "TC1b fast-cruise");
    }

    // ---- TC7a: degrade — predictedCm below the minimum meaningful move (`predictedCm <
    //            minMoveCm`), even though there would otherwise be plenty of ramp room.
    {
        const ApproachSchedule s = resolveApproachSchedule(3.0f, MARGIN_CM, MAX_MARGIN_CM,
                                                            CRUISE_MAG, FLOOR_MAG, ACCEL, RAMP_MIN,
                                                            RAMP_MAX, NO_RAMP_EPS, MIN_MOVE_CM);
        check(!s.active, "TC7a: predictedCm < minMoveCm degrades to inactive");
    }

    // ---- TC7b: degrade — predictedCm clears minMoveCm on its own, but the ramp has nowhere to
    //            start (`rampStartCm <= 0`). cruise=90/floor=30 -> rampLenCm=15; predicted=10
    //            (>= minMoveCm=5) -> floorStartCm=6, rampStartCm=6-15=-9 <= 0.
    {
        const ApproachSchedule s = resolveApproachSchedule(10.0f, MARGIN_CM, MAX_MARGIN_CM, 90,
                                                            FLOOR_MAG, ACCEL, RAMP_MIN, RAMP_MAX,
                                                            NO_RAMP_EPS, MIN_MOVE_CM);
        check(10.0f >= MIN_MOVE_CM, "TC7b: sanity — predictedCm clears minMoveCm on its own");
        check(!s.active, "TC7b: rampStartCm <= 0 degrades to inactive independent of minMoveCm");
    }

    // ---- TC7c: strict-< boundary on minMoveCm in isolation. cruiseMag == floorMag makes
    //            rampLenCm resolve to exactly 0.0f (deltaMag(0) < noRampEps), so at these small
    //            distances rampStartCm can't itself go non-positive — isolating the minMoveCm
    //            branch from the rampStartCm branch.
    {
        // predictedCm == minMoveCm exactly -> NOT degraded (strict `<`, not `<=`).
        const ApproachSchedule sAtBoundary = resolveApproachSchedule(
            MIN_MOVE_CM, MARGIN_CM, MAX_MARGIN_CM, FLOOR_MAG, FLOOR_MAG, ACCEL, RAMP_MIN,
            RAMP_MAX, NO_RAMP_EPS, MIN_MOVE_CM);
        check(approx(sAtBoundary.rampStartCm, MIN_MOVE_CM - MARGIN_CM),
              "TC7c: rampLenCm collapses to 0 when cruiseMag == floorMag");
        check(sAtBoundary.active, "TC7c: predictedCm == minMoveCm is NOT degraded (strict <)");

        // predictedCm just below minMoveCm -> degraded, same rampStartCm-clears-fine setup.
        const ApproachSchedule sBelowBoundary = resolveApproachSchedule(
            MIN_MOVE_CM - 0.001f, MARGIN_CM, MAX_MARGIN_CM, FLOOR_MAG, FLOOR_MAG, ACCEL, RAMP_MIN,
            RAMP_MAX, NO_RAMP_EPS, MIN_MOVE_CM);
        check(!sBelowBoundary.active, "TC7c: predictedCm just below minMoveCm degrades");
    }

    // ---- TC17: predictedCm plumbs straight through with no internal offset math — shifting
    //            predictedCm by a delta shifts every boundary by the EXACT same delta, and
    //            approachSpeedAt returns identical (pwmMag, phase) pairs at the same OFFSET
    //            into each schedule.
    {
        const float             deltaCm = 20.0f;
        const ApproachSchedule  sA      = resolveApproachSchedule(
            50.0f, MARGIN_CM, MAX_MARGIN_CM, CRUISE_MAG, FLOOR_MAG, ACCEL, RAMP_MIN, RAMP_MAX,
            NO_RAMP_EPS, MIN_MOVE_CM);
        const ApproachSchedule sB = resolveApproachSchedule(
            50.0f + deltaCm, MARGIN_CM, MAX_MARGIN_CM, CRUISE_MAG, FLOOR_MAG, ACCEL, RAMP_MIN,
            RAMP_MAX, NO_RAMP_EPS, MIN_MOVE_CM);

        check(approx(sB.rampStartCm - sA.rampStartCm, deltaCm),
              "TC17: rampStartCm shifts 1:1 with predictedCm");
        check(approx(sB.floorStartCm - sA.floorStartCm, deltaCm),
              "TC17: floorStartCm shifts 1:1 with predictedCm");
        check(approx(sB.overrunLimitCm - sA.overrunLimitCm, deltaCm),
              "TC17: overrunLimitCm shifts 1:1 with predictedCm");
        check(sA.cruiseMag == sB.cruiseMag && sA.floorMag == sB.floorMag,
              "TC17: cruise/floor magnitudes are unaffected by predictedCm");
        check(sA.active == sB.active, "TC17: active flag is unaffected by the shift");

        // Same OFFSET into each schedule -> identical (pwmMag, phase). Offsets below span
        // CRUISE (-20), RAMP (0, 5, 9.9), FLOOR_HOLD (15, 20), and OVERRUN (29, 40) relative to
        // sA.rampStartCm (rampLenCm=10, floorStartCm-rampStartCm=10, overrunLimitCm-rampStartCm=29).
        const float offsets[] = { -20.0f, 0.0f, 5.0f, 9.9f, 15.0f, 20.0f, 29.0f, 40.0f };
        for (float off : offsets)
        {
            const ApproachSpeed spA = approachSpeedAt(sA, sA.rampStartCm + off);
            const ApproachSpeed spB = approachSpeedAt(sB, sB.rampStartCm + off);
            char                 msg[128];
            std::snprintf(
                msg, sizeof(msg),
                "TC17: offset %.3f into the schedule must match across the predictedCm shift",
                (double) off);
            check(spA.pwmMag == spB.pwmMag && spA.phase == spB.phase, msg);
        }
    }

    if (g_failures == 0)
    {
        std::printf("APPROACH RAMP POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
