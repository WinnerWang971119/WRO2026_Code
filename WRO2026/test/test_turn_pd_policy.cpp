// =========================================================================================
// Host unit test for the turn PD control-law math (TurnPDPolicy.h, ADR-068, TC1-TC8).
//
// `turnWrap180` / `turnPdStep` / `turnSlewLimit` / `turnCreepFloor` / `turnSettleStep` /
// `turnSettled` are header-only, self-contained (`<math.h>` + `<stdint.h>` only) pure
// functions, so production AND this host test link the SAME symbols — the established
// `TurnSignPolicy.h` / `PivotTrimPolicy.h` pattern (see `test_turn_sign_policy.cpp`, which
// this file mirrors in structure, assertion style, and pass/fail reporting).
//
// Case list (see the T5 task spec for the full rationale on each):
//   TC1     turnWrap180    — folds a raw delta into (-180, +180]; +180 stays, -180 -> +180.
//   TC2     turnPdStep     — errNew == errOld: pure-P, D contributes exactly 0.
//   TC3     turnPdStep     — error shrinking fast: D is negative and opposes P (damping).
//   TC3b    turnPdStep     — TIME-NORMALIZED D is invariant to dt at a fixed angular rate
//                            (a per-iteration, non-normalized implementation would NOT be
//                            invariant here — that is the bug this case exists to catch).
//   TC3b'   turnPdStep     — fixed error numerator: D scales inversely with dt, by exactly
//                            the dt ratio (complement of TC3b's invariance argument).
//   TC3c    turnPdStep     — dtSec <= 0: D contributes 0, result stays finite (no NaN/Inf).
//   TC4     turnSlewLimit  — growth from rest is bounded to the slew rate (50 -> 10).
//   TC5     turnSlewLimit  — growth already within the rate passes through UNCHANGED (12).
//   --      turnSlewLimit  — reversal, deceleration, and negative-direction growth (the
//                            deadlock guard this header's docblock calls out by name).
//   TC6     turnCreepFloor — snap-up, pass-through-above-threshold, and cmd==0 stays 0.
//   TC7     turnSettleStep — 3x in-band increments the debounce, one out-of-band resets it.
//   --      turnSettleStep — absErr == toleranceDeg is OUT of band (strict-<) boundary.
//   TC8     turnSettled    — count/samplesRequired exit predicate.
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "src/drive/TurnPDPolicy.h"

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

/// Tolerance-based float comparison, for results that reach the same value along DIFFERENT
/// inexact paths — e.g. TC3b's two dt paths, which divide by different denominators and land
/// ~3 ULP apart (`-1.799995` vs `-1.800001`), so `==` would fail there against CORRECT code.
///
/// Use this ONLY for such a computed chain. Do NOT generalize it to "never `==` a float": the
/// exact comparisons elsewhere in this file are deliberate and must stay. They assert
/// exactly-representable values (`90.0f`, `45.0f`, `10.0f`, `-25.0f`, `0.0f`) reached by a
/// single operation, and TC6's `turnCreepFloor(0, ...) == 0.0f` is EXACT by spec — a commanded
/// stop is a stop. Loosening that one to a tolerance would silently stop testing the thing it
/// exists to test.
static bool approx(float a, float b, float tol = 1e-4f)
{
    return std::fabs(a - b) < tol;
}

// -------------------------------------------------------------------------------------------
// TC1 — turnWrap180: fold into (-180, +180], closed at the top, open at the bottom.
// -------------------------------------------------------------------------------------------
static void testTurnWrap180Boundaries()
{
    check(turnWrap180(180.0f) == 180.0f, "TC1: +180 stays +180 (range closed at the top)");
    check(turnWrap180(-180.0f) == 180.0f, "TC1: -180 folds to +180 (range open at the bottom)");
    check(turnWrap180(181.0f) == -179.0f, "TC1: +181 folds to -179");
    check(turnWrap180(-181.0f) == 179.0f, "TC1: -181 folds to +179");
    check(turnWrap180(359.0f) == -1.0f, "TC1: +359 folds to -1");
    check(turnWrap180(-359.0f) == 1.0f, "TC1: -359 folds to +1");
    check(turnWrap180(0.0f) == 0.0f, "TC1: 0 stays 0");
}

// -------------------------------------------------------------------------------------------
// TC2 — turnPdStep: errNew == errOld means zero rate, so the command is pure P.
// -------------------------------------------------------------------------------------------
static void testPdStepPureProportional()
{
    const float cmd = turnPdStep(90.0f, 90.0f, 0.029f, 1.0f, 0.018f);
    check(cmd == 90.0f, "TC2: errNew==errOld => cmd == Kp*errNew, D contributes 0");
}

// -------------------------------------------------------------------------------------------
// TC3 — error shrinking fast: the D term is negative and opposes (reduces) the P term.
// errNew=45, errOld=90 (error nearly halved in one tick) => rate is large and negative.
// -------------------------------------------------------------------------------------------
static void testPdStepDampingOpposesP()
{
    const float errNew = 45.0f;
    const float errOld = 90.0f;
    const float dtSec  = 0.01f;
    const float kp     = 1.0f;
    const float kd     = 0.018f;

    const float dOnly = turnPdStep(errNew, errOld, dtSec, 0.0f, kd);
    const float pOnly = turnPdStep(errNew, errOld, dtSec, kp, 0.0f);
    const float pdCmd = turnPdStep(errNew, errOld, dtSec, kp, kd);

    check(dOnly < 0.0f, "TC3: D-only term is negative when error is shrinking fast");
    check(pOnly == kp * errNew, "TC3: P-only term is Kp*errNew (sanity baseline)");
    check(pdCmd < pOnly, "TC3: full PD command is pulled below pure-P by the opposing D term");
    check(approx(pdCmd, pOnly + dOnly), "TC3: full PD command is exactly P + D (superposition)");
}

// -------------------------------------------------------------------------------------------
// TC3b — TIME-NORMALIZED D is invariant to dt at a FIXED angular rate omega=-100 deg/s.
// errOld = errNew - omega*dtSec, so the error delta scales with dt but rate = delta/dtSec
// stays constant. A non-normalized (per-iteration) implementation would NOT be invariant
// here (it would yield -0.0108 vs -0.0522) — that divergence is exactly what this pins.
// -------------------------------------------------------------------------------------------
static void testPdStepDtNormalizedInvariance()
{
    const float kp      = 0.0f;
    const float kd      = 0.018f;
    const float errNew  = 90.0f;
    const float expected = -1.8f;    // kd * omega == 0.018 * -100.

    const float shortDt = turnPdStep(errNew, 90.6f, 0.006f, kp, kd);
    const float longDt  = turnPdStep(errNew, 92.9f, 0.029f, kp, kd);

    check(approx(shortDt, expected), "TC3b: short-dt path lands on kd*omega (-1.8)");
    check(approx(longDt, expected), "TC3b: long-dt path lands on kd*omega (-1.8)");
    check(approx(shortDt, longDt), "TC3b: the two dt paths agree (time-normalized D)");
}

// -------------------------------------------------------------------------------------------
// TC3b' — fixed error numerator (errNew - errOld held constant) across two dt values: the D
// contribution scales inversely with dt, differing by exactly the dt ratio. Complements TC3b
// by asserting the division positively instead of only its invariance under rescaling.
// -------------------------------------------------------------------------------------------
static void testPdStepDtRatioScaling()
{
    const float kp     = 0.0f;
    const float kd     = 0.018f;
    const float errNew = 90.0f;
    const float errOld = 90.6f;    // fixed numerator: errNew - errOld == -0.6, NOT rescaled.
    const float dtA    = 0.006f;
    const float dtB    = 0.029f;

    const float dA = turnPdStep(errNew, errOld, dtA, kp, kd);
    const float dB = turnPdStep(errNew, errOld, dtB, kp, kd);

    const double expectedRatio = (double) dtB / (double) dtA;    // == 4.8333...
    const float actualRatio    = dA / dB;

    check(approx(actualRatio, (float) expectedRatio, 1e-3f),
          "TC3b': fixed-numerator D contributions differ by exactly the dt ratio (~4.8333x)");
}

// -------------------------------------------------------------------------------------------
// TC3c — dtSec <= 0 must never divide by zero: D contributes exactly 0 and the result stays
// finite. Covers both the zero and negative dt guards.
// -------------------------------------------------------------------------------------------
static void testPdStepZeroAndNegativeDt()
{
    const float kp     = 1.0f;
    const float kd     = 0.018f;
    const float errNew = 45.0f;
    const float errOld = 10.0f;

    const float zeroDtCmd = turnPdStep(errNew, errOld, 0.0f, kp, kd);
    check(std::isfinite(zeroDtCmd), "TC3c: dtSec==0 yields a finite result");
    check(zeroDtCmd == kp * errNew, "TC3c: dtSec==0 => D contributes exactly 0 (pure P)");

    const float negDtCmd = turnPdStep(errNew, errOld, -0.01f, kp, kd);
    check(std::isfinite(negDtCmd), "TC3c: negative dtSec yields a finite result");
    check(negDtCmd == kp * errNew, "TC3c: negative dtSec => D contributes exactly 0 (pure P)");
}

// -------------------------------------------------------------------------------------------
// TC4 — turnSlewLimit: growth from rest (prevCmd==0) is bounded by prevCmd + slewPerTick.
// -------------------------------------------------------------------------------------------
static void testSlewLimitGrowthFromRest()
{
    const float limited = turnSlewLimit(50.0f, 0.0f, 10.0f);
    check(limited == 10.0f, "TC4: growth from rest is bounded to slewPerTick");

    // Mirror case: a `prevCmd == 0` has no sign, so the "sign must match" half of the binding
    // rule must treat rest as compatible with growth in EITHER direction. Without this, a turn
    // dispatched CW from rest would ramp unlimited while a CCW one ramped — the asymmetry the
    // header's "previous command was exactly zero" rule exists to prevent.
    const float limitedNeg = turnSlewLimit(-50.0f, 0.0f, 10.0f);
    check(limitedNeg == -10.0f, "TC4: growth from rest is bounded in the negative direction too");
}

// -------------------------------------------------------------------------------------------
// TC5 — turnSlewLimit: growth within the slew budget passes through unchanged.
// -------------------------------------------------------------------------------------------
static void testSlewLimitGrowthWithinBudget()
{
    const float limited = turnSlewLimit(12.0f, 10.0f, 10.0f);
    check(limited == 12.0f, "TC5: growth within the slew budget passes through unchanged");
}

// -------------------------------------------------------------------------------------------
// The slew reversal case — the deadlock guard the header's docblock calls out by name. A
// naive symmetric constrain would clamp -20 to +5, which turnCreepFloor then snaps back to
// +15, latching the command in the wrong direction until the turn times out. A sign change
// must pass through completely unlimited.
// -------------------------------------------------------------------------------------------
static void testSlewLimitReversalUnlimited()
{
    const float limited = turnSlewLimit(-20.0f, 15.0f, 10.0f);
    check(limited == -20.0f, "slew reversal: sign change passes through UNLIMITED (deadlock guard)");
}

// -------------------------------------------------------------------------------------------
// The deceleration case — shrinking magnitude is never rate-limited; ramp-down needs no
// clamp because the P term already shrinks with the error.
// -------------------------------------------------------------------------------------------
static void testSlewLimitDecelerationUnlimited()
{
    const float limited = turnSlewLimit(20.0f, 50.0f, 10.0f);
    check(limited == 20.0f, "deceleration: shrinking magnitude passes through unlimited");
}

// -------------------------------------------------------------------------------------------
// Negative-direction growth — the limiter binds symmetrically when both cmd and prevCmd are
// negative and |cmd| is growing.
// -------------------------------------------------------------------------------------------
static void testSlewLimitNegativeGrowth()
{
    const float limited = turnSlewLimit(-50.0f, -15.0f, 10.0f);
    check(limited == -25.0f, "negative-direction growth: bounded symmetrically to -25");
}

// -------------------------------------------------------------------------------------------
// TC6 — turnCreepFloor: snap-up below threshold (both signs), pass-through at/above
// threshold, and the cmd==0 exact-zero case (a commanded stop must stay a stop).
// -------------------------------------------------------------------------------------------
static void testCreepFloorSnapAndPassthrough()
{
    check(turnCreepFloor(5.0f, 20, 15) == 15.0f, "TC6: (5,20,15) snaps up to +15");
    check(turnCreepFloor(-5.0f, 20, 15) == -15.0f, "TC6: (-5,20,15) snaps down to -15");
    check(turnCreepFloor(19.0f, 20, 15) == 15.0f, "TC6: (19,20,15) below threshold snaps to 15");
    check(turnCreepFloor(25.0f, 20, 15) == 25.0f, "TC6: (25,20,15) at/above threshold passes through");
    check(turnCreepFloor(0.0f, 20, 15) == 0.0f, "TC6: (0,20,15) stays EXACTLY 0 (a stop is a stop)");
}

// -------------------------------------------------------------------------------------------
// TC7 — turnSettleStep: three consecutive in-band samples increment 1, 2, 3; a single
// out-of-band sample resets the count to 0 (not decrements it).
// -------------------------------------------------------------------------------------------
static void testSettleStepDebounceAndReset()
{
    const float toleranceDeg = 1.0f;
    uint8_t count            = 0;

    count = turnSettleStep(0.5f, toleranceDeg, count);
    check(count == 1, "TC7: 1st in-band sample => count 1");

    count = turnSettleStep(0.3f, toleranceDeg, count);
    check(count == 2, "TC7: 2nd in-band sample => count 2");

    count = turnSettleStep(0.1f, toleranceDeg, count);
    check(count == 3, "TC7: 3rd in-band sample => count 3");

    count = turnSettleStep(2.0f, toleranceDeg, count);
    check(count == 0, "TC7: a single out-of-band sample resets the count to 0");
}

// -------------------------------------------------------------------------------------------
// The settle boundary — absErr exactly == toleranceDeg is OUT of band (strict-<), so the
// count stays 0 rather than incrementing. Pins the boundary against a later <=/< slip.
// -------------------------------------------------------------------------------------------
static void testSettleStepBoundaryExact()
{
    const float toleranceDeg = 1.0f;
    const uint8_t result     = turnSettleStep(1.0f, toleranceDeg, 2);
    check(result == 0, "settle boundary: absErr == toleranceDeg is OUT of band (strict-<)");
}

// -------------------------------------------------------------------------------------------
// TC8 — turnSettled: exit predicate over the debounce count.
// -------------------------------------------------------------------------------------------
static void testSettledPredicate()
{
    check(turnSettled(2, 3) == false, "TC8: count 2 of 3 required is not yet settled");
    check(turnSettled(3, 3) == true, "TC8: count 3 of 3 required is settled");
}

int main()
{
    testTurnWrap180Boundaries();
    testPdStepPureProportional();
    testPdStepDampingOpposesP();
    testPdStepDtNormalizedInvariance();
    testPdStepDtRatioScaling();
    testPdStepZeroAndNegativeDt();
    testSlewLimitGrowthFromRest();
    testSlewLimitGrowthWithinBudget();
    testSlewLimitReversalUnlimited();
    testSlewLimitDecelerationUnlimited();
    testSlewLimitNegativeGrowth();
    testCreepFloorSnapAndPassthrough();
    testSettleStepDebounceAndReset();
    testSettleStepBoundaryExact();
    testSettledPredicate();

    if (g_failures == 0)
    {
        std::printf("TURN PD POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
