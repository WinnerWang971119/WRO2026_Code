// =========================================================================================
// Host unit test for the turn PROFILE control-law math (TurnProfilePolicy.h, ADR-078, TC1-TC8).
//
// `turnPhaseFor` / `turnProfileSpeed` / `turnSettleNudge` are header-only, self-contained
// (`<math.h>` + `<stdint.h>` only) pure functions, so production AND this host test link the
// SAME symbols — the established `TurnSignPolicy.h` / `PivotTrimPolicy.h` pattern (see
// `test_turn_sign_policy.cpp`). This file replaces `test_turn_pd_policy.cpp`, which tested the
// now-deleted per-tick PD law (`turnPdStepRaw`, `turnSlewLimit`, `turnCreepFloor` — all
// zero-consumer since ADR-078 and removed from `TurnPDPolicy.h`).
//
// Case list (see `docs/plans/2026-07-21-turn-profile-controller.md` Testing Strategy for the
// full rationale on each):
//   TC1  turnPhaseFor      — absErr 30/25/15/3/1 (window 25, tol 3) -> CRUISE/CREEP/CREEP/SETTLE/SETTLE.
//   TC2  turnProfileSpeed  — CRUISE/CREEP/SETTLE(50,38) -> 50/38/0.
//   TC3  turnSettleNudge   — strict in-band (err 2, tol 3) -> exactly 0.
//   TC4  turnSettleNudge   — boundary (err 3.0 == tol) -> nudges (clamp(4*3,+/-38) == +12), NOT 0.
//   TC5  turnSettleNudge   — out-of-band clamp (err 20) -> +38.
//   TC6  turnSettleNudge   — overshoot (err -8) -> -32 (sign preserved, no clamp needed).
//   TC7  monotonic latch   — absErr 30 -> 20 -> 2 -> 35 through phase=max(phase,turnPhaseFor(...))
//                            -> CRUISE -> CREEP -> SETTLE -> SETTLE (never regresses).
//   TC8  Ported KEPT kernel — turnPdStep dt-edge cases + turnWrap180 fold cases (unchanged; the
//                            straight-line GYRO heading hold, GyroHoldPolicy.h, still reuses this
//                            kernel verbatim — see AC12 in the plan).
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "src/drive/TurnPDPolicy.h"
#include "src/drive/TurnProfilePolicy.h"

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
/// inexact paths — e.g. TC8's dt-invariance cases, which divide by different denominators and
/// land ~3 ULP apart. Use this ONLY for such a computed chain; the exact `==` comparisons
/// elsewhere in this file are deliberate (exactly-representable values reached by a single
/// operation, e.g. TC2/TC3's 0).
static bool approx(float a, float b, float tol = 1e-4f)
{
    return std::fabs(a - b) < tol;
}

// -------------------------------------------------------------------------------------------
// TC1 — turnPhaseFor: absErr 30/25/15/3/1 against window=25, tol=3. Boundaries round INTO the
// finer-grained phase: exactly ON the window edge is already CREEP; exactly ON the tolerance
// edge is already SETTLE.
// -------------------------------------------------------------------------------------------
static void testTurnPhaseForBoundaries()
{
    const float window = 25.0f;
    const float tol     = 3.0f;

    check(turnPhaseFor(30.0f, window, tol) == TurnPhase::CRUISE,
          "TC1: absErr 30 (> window) -> CRUISE");
    check(turnPhaseFor(25.0f, window, tol) == TurnPhase::CREEP,
          "TC1: absErr 25 (== window) -> CREEP (window edge rounds INTO CREEP)");
    check(turnPhaseFor(15.0f, window, tol) == TurnPhase::CREEP,
          "TC1: absErr 15 (inside the window, above tol) -> CREEP");
    check(turnPhaseFor(3.0f, window, tol) == TurnPhase::SETTLE,
          "TC1: absErr 3 (== tol) -> SETTLE (tolerance edge rounds INTO SETTLE)");
    check(turnPhaseFor(1.0f, window, tol) == TurnPhase::SETTLE,
          "TC1: absErr 1 (< tol) -> SETTLE");
}

// -------------------------------------------------------------------------------------------
// TC2 — turnProfileSpeed: CRUISE -> cruisePwm, CREEP -> creepPwm, SETTLE -> 0 (its command
// comes from turnSettleNudge instead).
// -------------------------------------------------------------------------------------------
static void testTurnProfileSpeedByPhase()
{
    check(turnProfileSpeed(TurnPhase::CRUISE, 50, 38) == 50, "TC2: CRUISE -> cruisePwm (50)");
    check(turnProfileSpeed(TurnPhase::CREEP, 50, 38) == 38, "TC2: CREEP -> creepPwm (38)");
    check(turnProfileSpeed(TurnPhase::SETTLE, 50, 38) == 0,
          "TC2: SETTLE -> 0 (defensive default; the nudge is the real command)");
}

// -------------------------------------------------------------------------------------------
// TC3 — turnSettleNudge: strict in-band (err 2, tol 3) commands exactly 0 so the settle
// debounce (turnSettleStep) can count the sample and the turn can release cleanly.
// -------------------------------------------------------------------------------------------
static void testSettleNudgeStrictInBandIsZero()
{
    const float nudge = turnSettleNudge(/*errProg*/ 2.0f, /*tol*/ 3.0f, /*kp*/ 4.0f, /*creep*/ 38);
    check(nudge == 0.0f, "TC3: strict in-band (err 2, tol 3) -> exactly 0");
}

// -------------------------------------------------------------------------------------------
// TC4 — turnSettleNudge: AT the tolerance boundary (err == tol) is OUT of band here (strict
// `<`, matching turnSettleStep), so it nudges rather than freezes: clamp(4*3, +/-38) == +12.
// -------------------------------------------------------------------------------------------
static void testSettleNudgeBoundaryNudgesNotFreezes()
{
    const float nudge = turnSettleNudge(/*errProg*/ 3.0f, /*tol*/ 3.0f, /*kp*/ 4.0f, /*creep*/ 38);
    check(nudge == 12.0f, "TC4: err == tol (3.0) -> nudges clamp(4*3, +/-38) == +12, NOT 0");
}

// -------------------------------------------------------------------------------------------
// TC5 — turnSettleNudge: out-of-band clamp. err=20, kp=4 -> raw 80, clamped down to the creep
// ceiling (+38) so SETTLE can never out-drive CREEP.
// -------------------------------------------------------------------------------------------
static void testSettleNudgeOutOfBandClampsToCreepCeiling()
{
    const float nudge = turnSettleNudge(/*errProg*/ 20.0f, /*tol*/ 3.0f, /*kp*/ 4.0f, /*creep*/ 38);
    check(nudge == 38.0f, "TC5: out-of-band clamp (err 20) -> +38 (creep ceiling)");
}

// -------------------------------------------------------------------------------------------
// TC6 — turnSettleNudge: overshoot (negative err) preserves sign. err=-8, kp=4 -> raw -32,
// within the +/-38 clamp, so it passes through unclamped but still negative (reversed).
// -------------------------------------------------------------------------------------------
static void testSettleNudgeOvershootPreservesSign()
{
    const float nudge = turnSettleNudge(/*errProg*/ -8.0f, /*tol*/ 3.0f, /*kp*/ 4.0f, /*creep*/ 38);
    check(nudge == -32.0f, "TC6: overshoot (err -8) -> -32 (sign preserved, no clamp needed)");
}

// -------------------------------------------------------------------------------------------
// TC7 — Monotonic latch: the FSM applies `phase = max(phase, turnPhaseFor(...))` every tick, so
// the phase only ever ADVANCES. Feeding absErr 30 -> 20 -> 2 -> 35 must latch
// CRUISE -> CREEP -> SETTLE -> SETTLE, i.e. the final swing back to 35 (> window) must NOT
// regress the already-latched SETTLE phase back to CRUISE.
// -------------------------------------------------------------------------------------------
static void testPhaseMonotonicLatchNeverRegresses()
{
    const float window = 25.0f;
    const float tol     = 3.0f;

    TurnPhase phase = TurnPhase::CRUISE;    // seeded wide, exactly like Drive.cpp's dispatch.

    TurnPhase natural = turnPhaseFor(30.0f, window, tol);
    if (natural > phase)
    {
        phase = natural;
    }
    check(phase == TurnPhase::CRUISE, "TC7: absErr 30 -> phase latches CRUISE");

    natural = turnPhaseFor(20.0f, window, tol);
    if (natural > phase)
    {
        phase = natural;
    }
    check(phase == TurnPhase::CREEP, "TC7: absErr 20 -> phase advances to CREEP");

    natural = turnPhaseFor(2.0f, window, tol);
    if (natural > phase)
    {
        phase = natural;
    }
    check(phase == TurnPhase::SETTLE, "TC7: absErr 2 -> phase advances to SETTLE");

    // The regression probe: absErr swings back ABOVE the window. `turnPhaseFor` alone would
    // answer CRUISE for this reading in isolation, but the monotonic latch must keep the
    // already-settled phase at SETTLE.
    natural = turnPhaseFor(35.0f, window, tol);
    check(natural == TurnPhase::CRUISE,
          "TC7: turnPhaseFor(35, ...) alone still answers CRUISE (it is un-latched by design)");
    if (natural > phase)
    {
        phase = natural;
    }
    check(phase == TurnPhase::SETTLE,
          "TC7: absErr swings back to 35 (> window) -> phase STAYS SETTLE (never regresses)");
}

// -------------------------------------------------------------------------------------------
// TC8a — turnWrap180: fold into (-180, +180], closed at the top, open at the bottom. Unchanged
// kernel — still the shared fold used by both the turn profile and the straight-line hold.
// -------------------------------------------------------------------------------------------
static void testTurnWrap180Boundaries()
{
    check(turnWrap180(180.0f) == 180.0f, "TC8a: +180 stays +180 (range closed at the top)");
    check(turnWrap180(-180.0f) == 180.0f, "TC8a: -180 folds to +180 (range open at the bottom)");
    check(turnWrap180(181.0f) == -179.0f, "TC8a: +181 folds to -179");
    check(turnWrap180(-181.0f) == 179.0f, "TC8a: -181 folds to +179");
    check(turnWrap180(359.0f) == -1.0f, "TC8a: +359 folds to -1");
    check(turnWrap180(-359.0f) == 1.0f, "TC8a: -359 folds to +1");
    check(turnWrap180(0.0f) == 0.0f, "TC8a: 0 stays 0");
}

// -------------------------------------------------------------------------------------------
// TC8b — turnPdStep: errNew == errOld means zero rate, so the command is pure P. Kept kernel —
// exercised here only because GyroHoldPolicy.h still calls this function; the in-place turn no
// longer does.
// -------------------------------------------------------------------------------------------
static void testPdStepPureProportional()
{
    const float cmd = turnPdStep(90.0f, 90.0f, 0.029f, 1.0f, 0.018f);
    check(cmd == 90.0f, "TC8b: errNew==errOld => cmd == Kp*errNew, D contributes 0");
}

// -------------------------------------------------------------------------------------------
// TC8c — turnPdStep: error shrinking fast: the D term is negative and opposes (reduces) the P
// term.
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

    check(dOnly < 0.0f, "TC8c: D-only term is negative when error is shrinking fast");
    check(pOnly == kp * errNew, "TC8c: P-only term is Kp*errNew (sanity baseline)");
    check(pdCmd < pOnly, "TC8c: full PD command is pulled below pure-P by the opposing D term");
    check(approx(pdCmd, pOnly + dOnly), "TC8c: full PD command is exactly P + D (superposition)");
}

// -------------------------------------------------------------------------------------------
// TC8d — turnPdStep: TIME-NORMALIZED D is invariant to dt at a FIXED angular rate omega=-100
// deg/s. A non-normalized (per-iteration) implementation would NOT be invariant here — this is
// exactly the edge case that distinguishes `turnPdStep` from the deleted `turnPdStepRaw`.
// -------------------------------------------------------------------------------------------
static void testPdStepDtNormalizedInvariance()
{
    const float kp        = 0.0f;
    const float kd        = 0.018f;
    const float errNew    = 90.0f;
    const float expected  = -1.8f;    // kd * omega == 0.018 * -100.

    const float shortDt = turnPdStep(errNew, 90.6f, 0.006f, kp, kd);
    const float longDt  = turnPdStep(errNew, 92.9f, 0.029f, kp, kd);

    check(approx(shortDt, expected), "TC8d: short-dt path lands on kd*omega (-1.8)");
    check(approx(longDt, expected), "TC8d: long-dt path lands on kd*omega (-1.8)");
    check(approx(shortDt, longDt), "TC8d: the two dt paths agree (time-normalized D)");
}

// -------------------------------------------------------------------------------------------
// TC8e — turnPdStep: fixed error numerator (errNew - errOld held constant) across two dt
// values — the D contribution scales inversely with dt, by exactly the dt ratio.
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
    const float  actualRatio   = dA / dB;

    check(approx(actualRatio, (float) expectedRatio, 1e-3f),
          "TC8e: fixed-numerator D contributions differ by exactly the dt ratio (~4.8333x)");
}

// -------------------------------------------------------------------------------------------
// TC8f — turnPdStep: dtSec <= 0 must never divide by zero — the D term contributes exactly 0
// and the result stays finite. Covers both the zero and negative dt guards.
// -------------------------------------------------------------------------------------------
static void testPdStepZeroAndNegativeDt()
{
    const float kp     = 1.0f;
    const float kd     = 0.018f;
    const float errNew = 45.0f;
    const float errOld = 10.0f;

    const float zeroDtCmd = turnPdStep(errNew, errOld, 0.0f, kp, kd);
    check(std::isfinite(zeroDtCmd), "TC8f: dtSec==0 yields a finite result");
    check(zeroDtCmd == kp * errNew, "TC8f: dtSec==0 => D contributes exactly 0 (pure P)");

    const float negDtCmd = turnPdStep(errNew, errOld, -0.01f, kp, kd);
    check(std::isfinite(negDtCmd), "TC8f: negative dtSec yields a finite result");
    check(negDtCmd == kp * errNew, "TC8f: negative dtSec => D contributes exactly 0 (pure P)");
}

int main()
{
    testTurnPhaseForBoundaries();
    testTurnProfileSpeedByPhase();
    testSettleNudgeStrictInBandIsZero();
    testSettleNudgeBoundaryNudgesNotFreezes();
    testSettleNudgeOutOfBandClampsToCreepCeiling();
    testSettleNudgeOvershootPreservesSign();
    testPhaseMonotonicLatchNeverRegresses();
    testTurnWrap180Boundaries();
    testPdStepPureProportional();
    testPdStepDampingOpposesP();
    testPdStepDtNormalizedInvariance();
    testPdStepDtRatioScaling();
    testPdStepZeroAndNegativeDt();

    if (g_failures == 0)
    {
        std::printf("TURN PROFILE POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
