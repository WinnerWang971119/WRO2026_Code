// =========================================================================================
// Host unit test for the GYRO straight-line heading-hold PD control-law math
// (GyroHoldPolicy.h, ADR-075, TC1-TC6).
//
// `gyroHoldCorrection` / `gyroHoldWheels` are header-only, self-contained functions (zero
// Arduino / Config.h surface — only `TurnPDPolicy.h` / `TurnSignPolicy.h`, which are
// themselves `<math.h>`/`<stdint.h>`-only), so production AND this host test link the SAME
// symbols — the established `TurnPDPolicy.h` / `TurnSignPolicy.h` pattern (see
// `test_turn_pd_policy.cpp`, which this file mirrors in structure, assertion style, and
// pass/fail reporting). Every tunable (kp, kd, corrLimitPwm, pwmMax) arrives as a function
// PARAMETER here — never `Config::GYRO_HOLD_*` — matching `test_turn_pd_policy.cpp`'s own
// discipline of exercising the pure math in isolation, decoupled from any particular gain
// tuning.
//
// Case list (see docs/plans/2026-07-19-169-gyro-move-r4-hold.md's Testing Strategy table for
// the full rationale on each):
//   TC1  gyroHoldCorrection — err>0, dt>0: kp*err + kd*(err-errOld)/dt, unclamped when the
//        raw command is within +/-corrLimitPwm (reuses turnPdStep verbatim).
//   TC2  gyroHoldCorrection — dtSec<=0 (zero and negative): D contributes exactly 0, pure-P.
//   TC3  gyroHoldCorrection — equal wrapped yaw-delta (a fixed angular rate) over two
//        different measured dt values yields the SAME rate-normalized D contribution.
//   TC4  gyroHoldCorrection — saturation at +/-corrLimitPwm, including the caller's dynamic
//        `corrLimitPwm == |base|` construction (simulated via a small corrLimitPwm), a
//        negative-direction saturation, and the corrLimitPwm==0 edge (a fully shrunk base).
//   TC5  gyroHoldWheels — L=base-corr, R=base+corr, capped +/-pwmMax, never reverses when
//        |corr|<=|base| (incl. NEGATIVE base), and cross-asserted against
//        `base + turnWheelPattern(+1, corr, COUNTER_ROT)` so the two policies cannot drift.
//   TC6  Fold-safe derivative-on-measurement — folding the yaw DELTA once (not differencing
//        two independently-wrapped errors) avoids a spurious +/-360 derivative jerk when the
//        true rotation crosses the +/-180 seam; a naive two-wrapped-errors construction is
//        shown landing on the WRONG SIGN, not merely the wrong magnitude.
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "src/drive/GyroHoldPolicy.h"

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

/// Tolerance-based float comparison for results reached along DIFFERENT inexact paths (e.g.
/// TC3's two-dt invariance, which divides by different denominators and lands a few ULP
/// apart). Mirrors `test_turn_pd_policy.cpp`'s own `approx()` verbatim — see that file's
/// docblock for why exact `==` stays the default everywhere else in this file.
static bool approx(float a, float b, float tol = 1e-4f)
{
    return std::fabs(a - b) < tol;
}

// -------------------------------------------------------------------------------------------
// TC1 — gyroHoldCorrection: err>0, dt>0 gives kp*err + kd*(err-errOld)/dt, unclamped when the
// raw command stays within +/-corrLimitPwm. Cross-checked against turnPdStep directly: when
// unclamped, gyroHoldCorrection returns turnPdStep's result VERBATIM (it mints no new control
// law — see the header's "reuses turnPdStep verbatim" claim).
// -------------------------------------------------------------------------------------------
static void testGyroHoldCorrectionBasicPD()
{
    const float errNew = 10.0f;
    const float errOld = 5.0f;
    const float dtSec   = 0.02f;
    const float kp       = 1.5f;
    const float kd       = 0.048f;
    const int16_t corrLimit = 100;    // generous — nowhere near clamping.

    // kp*errNew + kd*(errNew-errOld)/dtSec = 1.5*10 + 0.048*(10-5)/0.02 = 15 + 12 = 27.
    const float cmd = gyroHoldCorrection(errNew, errOld, dtSec, kp, kd, corrLimit);
    check(approx(cmd, 27.0f), "TC1: kp*err + kd*(err-errOld)/dt == 27.0 (unclamped)");
    check(cmd == turnPdStep(errNew, errOld, dtSec, kp, kd),
          "TC1: unclamped gyroHoldCorrection returns turnPdStep's result VERBATIM");
}

// -------------------------------------------------------------------------------------------
// TC2 — dtSec<=0 (zero AND negative): D contributes exactly 0, result is pure-P. Delegated to
// turnPdStep's own dtSec<=0 guard; this test pins that gyroHoldCorrection does not reintroduce
// a division anywhere on top of it.
// -------------------------------------------------------------------------------------------
static void testGyroHoldCorrectionZeroAndNegativeDt()
{
    const float errNew = 10.0f;
    const float errOld = 3.0f;    // deliberately different from errNew — if D leaked in
                                   // despite dtSec<=0, this would move the result off kp*errNew.
    const float kp = 1.0f;
    const float kd = 0.048f;
    const int16_t corrLimit = 100;

    const float zeroDt = gyroHoldCorrection(errNew, errOld, 0.0f, kp, kd, corrLimit);
    check(zeroDt == kp * errNew, "TC2: dtSec==0 => D contributes exactly 0 (pure P)");

    const float negDt = gyroHoldCorrection(errNew, errOld, -0.01f, kp, kd, corrLimit);
    check(negDt == kp * errNew, "TC2: dtSec<0 => D contributes exactly 0 (pure P)");
}

// -------------------------------------------------------------------------------------------
// TC3 — equal wrapped yaw-delta over two different dt gives the SAME rate-normalized D. Mirrors
// TurnPDPolicy.h's TC3b invariance argument, phrased the way `Drive::_emitGyroHold` actually
// constructs its inputs: a fixed measured angular rate `omega` produces `errOld = errNew -
// omega*dt` (the fold-safe caller-side construction), and the resulting D contribution must be
// identical regardless of how large or small the measured dt happens to be that tick.
// -------------------------------------------------------------------------------------------
static void testGyroHoldCorrectionDtInvariance()
{
    const float kp     = 0.0f;    // isolate D.
    const float kd     = 0.048f;
    const float errNew = 20.0f;
    const float omega  = -100.0f;    // deg/s — a fixed measured yaw rate.
    const int16_t corrLimit = 100;   // generous — |expected| == 4.8, nowhere near clamping.

    const float dtShort    = 0.006f;
    const float errOldShort = errNew - omega * dtShort;    // 20 - (-0.6) = 20.6.
    const float dtLong     = 0.029f;
    const float errOldLong  = errNew - omega * dtLong;     // 20 - (-2.9) = 22.9.

    const float expected = kd * omega;    // 0.048 * -100 = -4.8.

    const float cmdShort = gyroHoldCorrection(errNew, errOldShort, dtShort, kp, kd, corrLimit);
    const float cmdLong  = gyroHoldCorrection(errNew, errOldLong, dtLong, kp, kd, corrLimit);

    check(approx(cmdShort, expected), "TC3: short-dt path lands on kd*omega (-4.8)");
    check(approx(cmdLong, expected), "TC3: long-dt path lands on kd*omega (-4.8)");
    check(approx(cmdShort, cmdLong), "TC3: the two dt paths agree (time-normalized D)");
}

// -------------------------------------------------------------------------------------------
// TC4 — saturation at +/-corrLimitPwm. `corrLimitPwm` here plays the role of the CALLER's
// dynamic `min(GYRO_HOLD_MAX_CORR_PWM, |base|)` construction — a small corrLimitPwm simulates a
// profile's shrunk decel-tail base, so this proves the clamp binds correctly at whatever value
// the caller passes, not just a static ceiling.
// -------------------------------------------------------------------------------------------
static void testGyroHoldCorrectionSaturation()
{
    const float dtSec = 0.01f;
    const float kp    = 1.0f;
    const float kd    = 0.0f;    // isolate P so the raw command is trivially predictable.

    // corrLimitPwm=8 simulates a caller-computed min(GYRO_HOLD_MAX_CORR_PWM, |base|=8) deep in
    // a profile's decel tail. errNew=1000 makes the raw command (1000) wildly exceed the limit
    // in both directions.
    const int16_t shrunkBase = 8;
    const float   posSat = gyroHoldCorrection(1000.0f, 1000.0f, dtSec, kp, kd, shrunkBase);
    check(posSat == 8.0f, "TC4: large positive command clamps to +corrLimitPwm (simulated |base|=8)");
    const float negSat = gyroHoldCorrection(-1000.0f, -1000.0f, dtSec, kp, kd, shrunkBase);
    check(negSat == -8.0f, "TC4: large negative command clamps to -corrLimitPwm (simulated |base|=8)");

    // corrLimitPwm==0 — the fully-shrunk-base edge (a profile's base has decayed to exactly 0).
    // ANY nonzero raw command, either sign, must clamp to exactly 0.
    const float zeroLimitPos = gyroHoldCorrection(50.0f, 50.0f, dtSec, kp, kd, (int16_t) 0);
    check(zeroLimitPos == 0.0f, "TC4: corrLimitPwm==0 clamps a positive raw command to exactly 0");
    const float zeroLimitNeg = gyroHoldCorrection(-50.0f, -50.0f, dtSec, kp, kd, (int16_t) 0);
    check(zeroLimitNeg == 0.0f, "TC4: corrLimitPwm==0 clamps a negative raw command to exactly 0");
}

// -------------------------------------------------------------------------------------------
// TC5 — gyroHoldWheels: L=base-corr, R=base+corr, capped +/-pwmMax, never reverses when
// |corr|<=|base| (incl. NEGATIVE base), and cross-asserted against
// `base + turnWheelPattern(+1, corr, COUNTER_ROT)` for every case so the wheel-mix polarity can
// never silently drift from the turn's bench-verified chassis handedness.
// -------------------------------------------------------------------------------------------
static void crossAssertWheelMix(int16_t base, int16_t corr, const char* label)
{
    // pwmMax deliberately huge (1000) here so this cross-check is never itself capped — the
    // capping behaviour is asserted separately, below, with an explicit pwmMax.
    const GyroHoldWheels wheels  = gyroHoldWheels(base, corr, (int16_t) 1000);
    const TurnWheels     pattern = turnWheelPattern((int8_t) +1, corr, (uint8_t) 0 /* COUNTER_ROT */);
    const int16_t        expectedLeft  = (int16_t) (base + pattern.left);
    const int16_t        expectedRight = (int16_t) (base + pattern.right);

    char msg[160];
    std::snprintf(msg, sizeof(msg),
                  "TC5 cross-assert (%s): gyroHoldWheels == base + turnWheelPattern(+1,corr,0)",
                  label);
    check(wheels.left == expectedLeft && wheels.right == expectedRight, msg);
}

static void testGyroHoldWheelsMixAndNeverReverse()
{
    // (a) Positive base, uncapped: L=base-corr, R=base+corr.
    {
        const GyroHoldWheels wheels = gyroHoldWheels((int16_t) 50, (int16_t) 10, (int16_t) 100);
        check(wheels.left == 40, "TC5a: positive base -- left == base - corr (40)");
        check(wheels.right == 60, "TC5a: positive base -- right == base + corr (60)");
        crossAssertWheelMix(50, 10, "positive base");
    }

    // (b) Negative base (reverse move), |corr|<=|base|: BOTH wheels stay negative — the
    // correction never reverses either wheel's direction.
    {
        const GyroHoldWheels wheels = gyroHoldWheels((int16_t) -50, (int16_t) 10, (int16_t) 100);
        check(wheels.left == -60, "TC5b: negative base -- left == base - corr (-60)");
        check(wheels.right == -40, "TC5b: negative base -- right == base + corr (-40)");
        check(wheels.left <= 0 && wheels.right <= 0,
              "TC5b: negative base -- neither wheel crosses zero (never reverses)");
        crossAssertWheelMix(-50, 10, "negative base");
    }

    // (c) Positive-base capping: base+corr exceeds pwmMax, gets capped; the OTHER wheel (below
    // the cap) passes through unaffected.
    {
        const GyroHoldWheels wheels = gyroHoldWheels((int16_t) 90, (int16_t) 30, (int16_t) 100);
        check(wheels.left == 60, "TC5c: positive base -- uncapped side passes through (60)");
        check(wheels.right == 100, "TC5c: positive base -- over-cap side clamps to +pwmMax (100)");
    }

    // (d) Negative-base capping: the mirror of (c).
    {
        const GyroHoldWheels wheels = gyroHoldWheels((int16_t) -90, (int16_t) 30, (int16_t) 100);
        check(wheels.left == -100, "TC5d: negative base -- over-cap side clamps to -pwmMax (-100)");
        check(wheels.right == -60, "TC5d: negative base -- uncapped side passes through (-60)");
    }

    // (e) Never-reverse BOUNDARY: |corr| == |base| exactly (the caller's dynamic clamp
    // contract, `corrLimitPwm == |base|`, lands here) — the inner wheel touches EXACTLY zero,
    // never crosses into a reversed sign.
    {
        const GyroHoldWheels wheels = gyroHoldWheels((int16_t) 5, (int16_t) 5, (int16_t) 100);
        check(wheels.left == 0, "TC5e: |corr|==|base| boundary -- inner wheel lands at EXACTLY 0");
        check(wheels.right == 10, "TC5e: |corr|==|base| boundary -- outer wheel == 2*base (10)");
        crossAssertWheelMix(5, 5, "never-reverse boundary");
    }
}

// -------------------------------------------------------------------------------------------
// TC6 — fold-safe derivative-on-measurement avoids a spurious +/-360 jerk when the true
// rotation crosses the +/-180 seam. Both `turnWrap180(target - yaw)` (the error) AND
// `turnWrap180(yaw - prevYaw)` (the yaw delta) are exercised right at the boundary:
//
//   target=0, prevYaw=179, yaw=-179 -- a TINY +2 deg CCW rotation (179 -> 180/-180 -> -179)
//   that happens to straddle the wrap seam.
//
// The CORRECT fold-safe construction (`errOld = errNew + turnWrap180(yaw - prevYaw)`, single
// wrap on the MEASURED delta) yields a small, correctly-signed D contribution. A NAIVE
// construction that instead differences two INDEPENDENTLY wrapped errors
// (`errOld = turnWrap180(target - prevYaw)`) misreads the same tiny rotation as a huge ~358 deg
// swing -- not merely a magnitude error, but the WRONG SIGN entirely, which is exactly the class
// of bug the fold-safe design (ADR-075) exists to structurally rule out.
// -------------------------------------------------------------------------------------------
static void testGyroHoldFoldSafeDerivativeAvoidsStraddleJerk()
{
    const float target  = 0.0f;
    const float prevYaw = 179.0f;
    const float yaw      = -179.0f;

    // Prerequisite: both quantities individually fold correctly across the +/-180 seam.
    const float errNew   = turnWrap180(target - yaw);         // turnWrap180(179) == 179.
    const float yawDelta = turnWrap180(yaw - prevYaw);        // turnWrap180(-358) == 2.
    check(errNew == 179.0f, "TC6: prerequisite -- turnWrap180(target - yaw) folds to 179");
    check(yawDelta == 2.0f, "TC6: prerequisite -- turnWrap180(yaw - prevYaw) folds to the TRUE +2 deg motion");

    const float dtSec = 0.01f;
    const float kp    = 0.0f;    // isolate D.
    const float kd    = 0.048f;
    const int16_t corrLimit = 60;    // Config::GYRO_HOLD_MAX_CORR_PWM's shipped default.

    // Fold-safe (CORRECT, matches Drive::_emitGyroHold's construction): errOld = errNew +
    // turnWrap180(yaw - prevYaw) = 179 + 2 = 181. rate = (179-181)/0.01 = -200 deg/s.
    // D = 0.048 * -200 = -9.6 -- small, correctly signed, well inside the +/-60 clamp.
    const float errOldFoldSafe = errNew + yawDelta;
    const float cmdFoldSafe =
        gyroHoldCorrection(errNew, errOldFoldSafe, dtSec, kp, kd, corrLimit);
    check(approx(cmdFoldSafe, -9.6f, 1e-2f),
          "TC6: fold-safe errOld -> small, correctly-signed D (~-9.6), no straddle jerk");
    check(cmdFoldSafe < 0.0f, "TC6: fold-safe D is negative (yaw moved CCW, error correctly shrinking)");

    // Naive (WRONG -- differences two independently-wrapped errors): errOldNaive =
    // turnWrap180(target - prevYaw) = turnWrap180(-179) = -179 (already canonical, no wrap
    // needed). rate = (179 - (-179))/0.01 = 35800 deg/s -- a spurious ~358x-too-large rate from
    // a rotation that was actually only 2 degrees. D = 0.048*35800 = 1718.4, clamped to +60.
    const float errOldNaive = turnWrap180(target - prevYaw);
    check(errOldNaive == -179.0f, "TC6: naive errOld (two independently-wrapped errors) == -179");
    const float cmdNaive = gyroHoldCorrection(errNew, errOldNaive, dtSec, kp, kd, corrLimit);
    check(cmdNaive == 60.0f, "TC6: naive construction saturates at +corrLimitPwm (60) -- a spurious jerk");

    // The crux: the naive construction doesn't just get the MAGNITUDE wrong -- it flips the
    // SIGN entirely (a spurious positive command where the correct one is negative), which
    // would steer the correction the WRONG WAY on the very first tick after a segment boundary.
    check((cmdFoldSafe < 0.0f) != (cmdNaive < 0.0f),
          "TC6: naive vs fold-safe disagree on SIGN, not just magnitude -- the exact bug class "
          "the fold-safe design rules out structurally");
}

int main()
{
    testGyroHoldCorrectionBasicPD();
    testGyroHoldCorrectionZeroAndNegativeDt();
    testGyroHoldCorrectionDtInvariance();
    testGyroHoldCorrectionSaturation();
    testGyroHoldWheelsMixAndNeverReverse();
    testGyroHoldFoldSafeDerivativeAvoidsStraddleJerk();

    if (g_failures == 0)
    {
        std::printf("GYRO HOLD POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
