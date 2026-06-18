// =========================================================================================
// Host unit test for the operator-offset integrator (Localization.cpp, PR1a).
//
// FIRST host target to compile the REAL production `Localization.cpp`. It drives the
// per-tick offset integrator (`Localization::update()`) through the mock `Drive`
// encoders and the mock lower-MCU IMU (`mmL.GetIMUEuler`), asserting the
// acceptance cases:
//
//   TC1  (AC1)  — a forward drive of 30 cm grows the offset magnitude to ~30 cm.
//   TC2  (AC2)  — an idle, zeroed chassis accumulates ~0 offset over many ticks.
//   TC2b (AC2b) — a pure +45 deg world-heading change grows thetaDeg to ~+45 with
//                  ~0 translation (sign + channel-independence proof).
//   TC3  (AC3)  — a node arrival (`onIntersection`) re-anchors: the very next tick
//                  reads ~0 offset (no one-tick arrival phantom).
//   TC4  (AC3)  — navActive (PathTracker driving a hop) suppresses accumulation.
//   TC5  (AC1)  — a drive at a NON-ZERO world heading decomposes onto BOTH world
//                  axes with the correct signs (cosf/sinf split, not just magnitude).
//
// HOW THE SUT IS LINKED (see CMakeLists.txt `test_localization_offset`):
//   - `Localization.cpp` is compiled UNCHANGED. Its `#include "Drive.h"` is a quote
//     include source-relative to the sketch dir, so `add_mock_force_includes` force-
//     includes the mock `Drive.h` first (guard `DRIVE_H`) to shadow the real one.
//   - `<MatrixMiniR4.h>` / `<Arduino.h>` are angle-bracket includes resolved by the
//     `mocks/`-first include path — the mock IMU surface (`mmL`, `MMLower::RESULT`)
//     lives in `mocks/MatrixMiniR4.h`.
//   - `routes.cpp` is linked for the `Routes::START_NODE / START_HEADING_DEG /
//     NODES / N_NODES` symbols that `Localization.cpp` odr-uses.
//
// The float-tolerance checks use a local `approx`/`check` pair (the shared
// `TestAssert.h` ASSERT_EQ casts operands to `long long`, which would truncate the
// sub-cm / sub-degree quantities under test). The pass/fail + final-line conventions
// mirror `test_pivot_trim_policy.cpp`.
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "Localization.h"

// Mock surfaces — the same singletons `Localization.cpp` was compiled against.
#include "Drive.h"            // mock: settable encoders (mockSetEncL/R, mockAdvanceEncoders).
#include "MatrixMiniR4.h"     // mock: mmL.GetIMUEuler + mmL.mockYawDeg / mockResult.
#include "WROMath.h"          // cmToEncDeg — scripts "drive d cm" into encoder degrees.
#include "Config.h"           // Config::OFFSET_*_DEADBAND_* for the deadband-aware tolerances.

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

/// Absolute-tolerance float compare. `tol` is per-case (cm or deg) — see each call site.
static bool approx(float a, float b, float tol)
{
    return std::fabs(a - b) <= tol;
}

/// Euclidean magnitude of the offset translation vector (cm).
static float offsetMag(const Offset2D& o)
{
    return std::sqrt(o.x * o.x + o.y * o.y);
}

// -----------------------------------------------------------------------------------------
// Per-case world reset: clear both mock surfaces and re-seed Localization. Mirrors the
// sibling tests' `resetWorld()`. `Localization.begin()` seeds `_currentNode =
// Routes::START_NODE`, clears `_zeroed`, and re-snapshots the heading cache; the offset
// + integrator baselines start at zero. `mmL.mockReset()` puts yaw at 0 / result OK.
// -----------------------------------------------------------------------------------------
static void resetWorld()
{
    Drive.mockReset();
    mmL.mockReset();
    Localization.begin();
}

int main()
{
    // -------------------------------------------------------------------------------------
    // TC1 (AC1) — a forward drive of 30 cm grows the offset magnitude to ~30 cm.
    //
    // Setup: zero the frame at an arbitrary heading H, close the nav gate (operator may be
    // hand-driving), then advance BOTH encoders by cmToEncDeg(30) — a pure straight push.
    // One update() integrates it. World heading immediately after zeroing is
    // START_HEADING_DEG (== 0), so the 30 cm decomposes onto +x and the magnitude is ~30.
    //
    // Tolerance: cmToEncDeg truncates to uint16 encoder degrees, so the round-trip back
    // through encDegToCm loses < 1 cm. ±1.0 cm covers it.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        const uint8_t nodeBefore = Localization.getCurrentNode();

        mmL.mockYawDeg = 37.0;    // arbitrary boot heading H; world heading post-zero is 0.
        check(Localization.zeroHeadingFromCurrent(), "TC1: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(false);    // open the integrate gate (does NOT itself re-snapshot).

        // A "settling" tick at the post-zero pose re-anchors the integrator's baselines to the
        // current readings before the drive. Here it is a belt-and-suspenders no-op (begin()
        // already left _lastEnc*/_lastYaw at 0 and the mock encoders/heading are at 0), but it
        // keeps the first integrated delta exactly the scripted 30 cm under any future seeding.
        Localization.update();
        check(approx(offsetMag(Localization.getOffsetFromNode()), 0.0f, 0.001f),
              "TC1: offset is zero at the settled post-zero pose");

        // Drive 30 cm forward: advance BOTH wheels equally (pure translation, no rotation).
        const int32_t driveDeg = (int32_t) cmToEncDeg(30.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        Localization.update();

        const Offset2D off = Localization.getOffsetFromNode();
        check(approx(offsetMag(off), 30.0f, 1.0f), "TC1: offset magnitude ~= 30 cm after a 30 cm drive");
        check(Localization.getCurrentNode() == nodeBefore, "TC1: current node unchanged by a hand-drive");
    }

    // -------------------------------------------------------------------------------------
    // TC2 (AC2) — an idle, zeroed chassis accumulates ~0 offset (no phantom drift).
    //
    // Setup: zero + open the gate, then call update() ~15 times with NO encoder or yaw
    // change. The translation deadband (OFFSET_TRANSLATION_DEADBAND_CM) and rotation
    // deadband (OFFSET_ROTATION_DEADBAND_DEG) must filter every idle tick so neither the
    // magnitude nor thetaDeg grows.
    //
    // Tolerance: a stationary chassis nets EXACTLY zero delta each tick (dEnc == 0,
    // dYaw == 0), so the offset stays bit-exact zero; a tiny epsilon guards float noise.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        mmL.mockYawDeg = 0.0;
        check(Localization.zeroHeadingFromCurrent(), "TC2: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(false);

        for (int i = 0; i < 15; ++i)
        {
            Localization.update();    // encoders + yaw unchanged -> every tick is sub-deadband.
        }

        const Offset2D off = Localization.getOffsetFromNode();
        check(approx(offsetMag(off), 0.0f, 1e-4f), "TC2: idle offset magnitude stays ~0 (no phantom drift)");
        check(approx(off.thetaDeg, 0.0f, 1e-4f), "TC2: idle thetaDeg stays ~0");
    }

    // -------------------------------------------------------------------------------------
    // TC2b (AC2b) — a pure +45 deg world-heading change grows thetaDeg to ~+45 with ~0
    // translation (sign + channel-independence proof).
    //
    // Setup: zero (world heading -> 0) + open the gate + a settling tick that locks the
    // yaw baseline at 0. Then bump the IMU yaw by +45 (world heading -> +45) WITHOUT
    // touching the encoders, and integrate one tick. The rotation channel must add +45 to
    // thetaDeg (SIGN: a +45 deg world-heading change yields +45 thetaDeg) while the
    // translation channel — gated independently and seeing a zero encoder delta — leaves
    // x/y at ~0.
    //
    // Tolerance: the rotation channel records the world-heading delta exactly, so ±0.5 deg
    // (one rotation-deadband width) is generous; translation stays bit-exact zero.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        mmL.mockYawDeg = 0.0;    // zero at world heading 0.
        check(Localization.zeroHeadingFromCurrent(), "TC2b: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(false);

        // Settle the yaw baseline at world heading 0 (no encoder/yaw change yet).
        Localization.update();

        // Pure manual turn: +45 deg world heading, encoders untouched (no translation).
        mmL.mockYawDeg = 45.0;
        Localization.update();

        const Offset2D off = Localization.getOffsetFromNode();
        check(approx(off.thetaDeg, 45.0f, 0.5f), "TC2b: thetaDeg ~= +45 after a +45 deg turn (sign positive)");
        check(approx(offsetMag(off), 0.0f, 1e-4f), "TC2b: translation stays ~0 during a pure turn");
    }

    // -------------------------------------------------------------------------------------
    // TC3 (AC3 arrival-tick) — a node arrival re-anchors: the next tick reads ~0 offset.
    //
    // Setup: zero + open the gate + settle, then drive some offset and integrate it
    // (assert it is non-zero). Simulate a node arrival via onIntersection(node), which
    // clears the offset AND re-snapshots all three integrator baselines to the CURRENT
    // encoder/heading readings. Then integrate ONE more tick WITHOUT changing the encoders:
    // because the baselines were re-snapshotted, the per-tick delta is zero, so the offset
    // reads ~0 — the arrival-tick phantom (an offset equal to the final approach motion) is
    // killed.
    //
    // Tolerance: post-arrival the delta is exactly zero (baselines == current readings),
    // so the offset is bit-exact zero; a tiny epsilon guards float noise.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        mmL.mockYawDeg = 0.0;
        check(Localization.zeroHeadingFromCurrent(), "TC3: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(false);
        Localization.update();    // settle baselines at the post-zero pose.

        // Drive 20 cm forward and integrate -> the offset grows (the approach motion).
        const int32_t driveDeg = (int32_t) cmToEncDeg(20.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        Localization.update();
        check(offsetMag(Localization.getOffsetFromNode()) > 1.0f,
              "TC3: offset is non-zero after a hand-drive (pre-arrival)");

        // Arrive on a node: re-anchor + re-snapshot baselines to the current readings.
        const uint8_t arrivalNode = 0;
        Localization.onIntersection(arrivalNode);
        check(Localization.getCurrentNode() == arrivalNode, "TC3: arrival advanced the current node");

        // Next tick WITHOUT any encoder change: the re-snapshotted baselines win the race,
        // so the integrated delta is zero -> no one-tick arrival phantom.
        Localization.update();
        const Offset2D off = Localization.getOffsetFromNode();
        check(approx(offsetMag(off), 0.0f, 1e-4f), "TC3: offset reads ~0 on the arrival tick (no phantom)");
        check(approx(off.thetaDeg, 0.0f, 1e-4f), "TC3: thetaDeg reads ~0 on the arrival tick");
    }

    // -------------------------------------------------------------------------------------
    // TC4 (AC3 nav-gate) — while PathTracker is driving a hop (navActive == true), operator
    // offset accumulation is SUPPRESSED: encoder motion does not grow the offset. This is the
    // other half of AC3 ("the offset is NOT grown by PathTracker motion"); TC3 covers the
    // arrival-tick clear, TC4 covers the gate that keeps a goTo traversal from ever building
    // an offset. With the gate closed, update() only re-snapshots baselines, so the per-tick
    // delta stays zero no matter how far the wheels turned.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        mmL.mockYawDeg = 0.0;
        check(Localization.zeroHeadingFromCurrent(), "TC4: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(true);    // PathTracker is executing a hop.
        Localization.update();              // settle baselines under the closed gate.

        // Advance the encoders as if PathTracker drove a 25 cm hop, then tick.
        const int32_t hopDeg = (int32_t) cmToEncDeg(25.0f);
        Drive.mockAdvanceEncoders(hopDeg, hopDeg);
        Localization.update();

        // Gate was closed the whole time, so nothing accumulated.
        const Offset2D off = Localization.getOffsetFromNode();
        check(approx(offsetMag(off), 0.0f, 1e-4f),
              "TC4: navActive suppresses offset accumulation during a hop");
        check(approx(off.thetaDeg, 0.0f, 1e-4f), "TC4: thetaDeg stays ~0 while navActive");
    }

    // -------------------------------------------------------------------------------------
    // TC5 (AC1 off-axis) — a forward drive at a NON-ZERO world heading decomposes onto BOTH
    // world axes with the correct signs (proves the cosf/sinf split, not just the magnitude).
    //
    // Setup: zero at world heading 0, open the gate, then rotate the world heading to 150 deg
    // (a pure IMU-yaw change; encoders untouched) and settle so the yaw baseline tracks it.
    // Then drive 30 cm forward. At heading 150: x = 30*cos(150) = -25.98, y = 30*sin(150) =
    // +15.0. The magnitude-only check (TC1) would still pass on an axis-swapped or sign-flipped
    // decomposition; pinning the individual components is what catches that.
    //
    // Tolerance: encDegToCm truncation loses < 1 cm on the 30 cm drive, which scales onto each
    // component, so +/-1.0 cm per axis covers it.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        mmL.mockYawDeg = 0.0;    // zero at world heading 0.
        check(Localization.zeroHeadingFromCurrent(), "TC5: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(false);

        // Rotate the world heading to 150 deg and settle the baselines there (no translation yet).
        mmL.mockYawDeg = 150.0;
        Localization.update();

        // Drive 30 cm forward along the 150-deg heading.
        const int32_t driveDeg = (int32_t) cmToEncDeg(30.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        Localization.update();

        const Offset2D off = Localization.getOffsetFromNode();
        check(approx(off.x, -25.98f, 1.0f), "TC5: +x component ~= 30*cos(150) (negative)");
        check(approx(off.y, 15.0f, 1.0f), "TC5: +y component ~= 30*sin(150) (positive)");
        check(approx(offsetMag(off), 30.0f, 1.0f), "TC5: magnitude preserved off the 0-deg axis");
    }

    if (g_failures == 0)
    {
        std::printf("LOCALIZATION OFFSET OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
