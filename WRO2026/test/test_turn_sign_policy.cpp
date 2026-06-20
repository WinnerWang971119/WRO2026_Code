// =========================================================================================
// Host unit test for the turn-sign + wheel-pattern math (TurnSignPolicy.h, ADR-044, TC5/TC6).
//
// `normalizeTurnDelta` / `turnSign` / `turnWheelPattern` were promoted out of Drive.cpp into
// header-only inline policies so production AND this host test link the SAME symbols — the
// established `PivotTrimPolicy.h` / `DirectionPolicy.h` pattern (the precedent this file
// mirrors: see `test_pivot_trim_policy.cpp`). Drive.cpp is NOT host-compiled (heavy
// MatrixMiniR4 deps), so before this promotion the turn-fold, the sign capture, and the
// wheel-pattern switch had no automated coverage. The header is deliberately self-contained
// (only `<stdint.h>`), so this TU includes ONLY the header — no Arduino / Config mocks.
//
//   TC5 (normalizeTurnDelta) — folds any signed delta into the half-open range (-180, +180]
//                              so the shortest rotation is chosen; the closed-top / open-bottom
//                              boundary (+180 stays, -180 -> +180) is pinned explicitly.
//   TC5 (turnWheelPattern)   — sign + magnitude + motor mode -> a left/right PWM pair, incl.
//                              the unknown-mode counter-rotation fallback.
//   TC6 (turnSign)           — rotation sign from a folded delta; the zero -> -1 contract is
//                              pinned DELIBERATELY (a refactor to `>=0 ? +1 : -1` must fail here).
//
// All quantities are integer-valued, so the asserts compare exactly. The pass/fail +
// final-line conventions mirror `test_pivot_trim_policy.cpp`.
// =========================================================================================

#include <cstdio>

#include "TurnSignPolicy.h"

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
    // TC5 — normalizeTurnDelta folds into (-180, +180]: shortest signed rotation.
    // -------------------------------------------------------------------------------------
    check(normalizeTurnDelta(270) == -90, "TC5: +270 folds to -90 (shortest rotation)");
    check(normalizeTurnDelta(-270) == 90, "TC5: -270 folds to +90 (shortest rotation)");
    check(normalizeTurnDelta(180) == 180, "TC5: +180 stays +180 (range closed at the top)");
    check(normalizeTurnDelta(-180) == 180, "TC5: -180 folds to +180 (range open at the bottom)");
    check(normalizeTurnDelta(0) == 0, "TC5: 0 stays 0");
    check(normalizeTurnDelta(90) == 90, "TC5: +90 within range is unchanged");

    // -------------------------------------------------------------------------------------
    // TC6 — turnSign: +1 means yaw must INCREASE (CCW), -1 means DECREASE (CW). The zero
    // delta resolves to -1 by design (Drive.cpp's `(delta > 0) ? +1 : -1`); a refactor to
    // `>= 0 ? +1 : -1` must FAIL the zero case below.
    // -------------------------------------------------------------------------------------
    check(turnSign(90) == 1, "TC6: +90 -> +1 (CCW)");
    check(turnSign(-90) == -1, "TC6: -90 -> -1 (CW)");
    check(turnSign(0) == -1, "TC6: 0 -> -1 (zero-delta contract pinned deliberately)");
    check(turnSign(normalizeTurnDelta(270)) == -1, "TC6: a +270 input folds to -90 then signs to -1");

    // -------------------------------------------------------------------------------------
    // TC5 — turnWheelPattern: sign + magnitude + motor mode -> left/right PWM pair.
    //   mode 0 COUNTER_ROT: CCW (+1) => L<0, R>0; CW (-1) => mirror.
    //   mode 1 PIVOT_LEFT  : left wheel stationary, right wheel drives.
    //   mode 2 PIVOT_RIGHT : right wheel stationary, left wheel drives.
    //   default (unknown)  : falls back to counter-rotation rather than refusing to turn.
    // -------------------------------------------------------------------------------------
    {
        const TurnWheels ccw = turnWheelPattern(+1, 60, 0);
        check(ccw.left == -60 && ccw.right == 60, "TC5: COUNTER_ROT CCW => L=-60, R=+60");

        const TurnWheels cw = turnWheelPattern(-1, 60, 0);
        check(cw.left == 60 && cw.right == -60, "TC5: COUNTER_ROT CW => L=+60, R=-60");

        const TurnWheels pivotLeft = turnWheelPattern(+1, 60, 1);
        check(pivotLeft.left == 0 && pivotLeft.right == 60, "TC5: PIVOT_LEFT (+1) => L=0, R=+60");

        const TurnWheels pivotRight = turnWheelPattern(+1, 60, 2);
        check(pivotRight.left == -60 && pivotRight.right == 0, "TC5: PIVOT_RIGHT (+1) => L=-60, R=0");

        const TurnWheels unknown = turnWheelPattern(+1, 60, 99);
        check(unknown.left == -60 && unknown.right == 60,
              "TC5: unknown mode falls back to COUNTER_ROT (L=-60, R=+60)");
    }

    if (g_failures == 0)
    {
        std::printf("TURN SIGN POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
