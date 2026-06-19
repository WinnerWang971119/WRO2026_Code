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
//   TC6  (#5)   — a failed IMU read mid-drive deposits nothing (no stale-heading
//                  corruption); the motion carries to the next good-read tick.
//   TC7  (#3)   — no live IMU read while the nav gate is closed (mission hot path).
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
        Localization.setNavActive(true);    // PathTracker is executing a hop (gate closed).
        Localization.update();              // closed gate: no read, no accumulation.

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

    // -------------------------------------------------------------------------------------
    // TC6 (#5 stale-read) — a failed IMU read mid-drive does NOT deposit translation onto a
    // stale heading; the motion carries to the next good-read tick instead of corrupting x/y.
    //
    // Setup: zero + open the gate + a reopen-edge settle. Drive 30 cm, then force the IMU to
    // fail (mockResult = ERROR) and integrate: the tick is skipped, so the offset stays ~0 and
    // the baselines are left unadvanced. Recover the IMU at the SAME heading and integrate again:
    // the carried-across 30 cm is deposited once, at the correct heading.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        mmL.mockYawDeg = 0.0;
        check(Localization.zeroHeadingFromCurrent(), "TC6: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(false);
        Localization.update();    // reopen-edge settle (live read OK).

        // Drive 30 cm, then fail the IMU on the integrate tick.
        const int32_t driveDeg = (int32_t) cmToEncDeg(30.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        mmL.mockResult = MMLower::RESULT::ERROR;
        Localization.update();    // read fails -> tick skipped, baselines unadvanced.
        check(approx(offsetMag(Localization.getOffsetFromNode()), 0.0f, 1e-4f),
              "TC6: a failed IMU read deposits nothing (no stale-heading corruption)");

        // Recover at the same heading: the carried 30 cm is deposited now, at the right angle.
        mmL.mockResult = MMLower::RESULT::OK;
        mmL.mockYawDeg = 0.0;
        Localization.update();
        check(approx(offsetMag(Localization.getOffsetFromNode()), 30.0f, 1.0f),
              "TC6: the carried-across motion is deposited on the next good read");
    }

    // -------------------------------------------------------------------------------------
    // TC7 (#3 hot-path) — while PathTracker is driving a hop (gate closed), update() issues NO
    // live IMU read. This is the per-tick blocking read removed from the mission hot path; the
    // baseline is re-established once at the gate-reopen edge instead.
    //
    // Setup: zero (one read), then close the gate (navActive) and tick several times while the
    // wheels turn. The mock IMU's call counter must not advance across those closed ticks.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        mmL.mockYawDeg = 0.0;
        check(Localization.zeroHeadingFromCurrent(), "TC7: zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(true);    // PathTracker driving -> gate closed.

        const uint32_t readsBefore = mmL.getImuEulerCallCount;
        const int32_t  hopDeg      = (int32_t) cmToEncDeg(25.0f);
        for (int i = 0; i < 5; ++i)
        {
            Drive.mockAdvanceEncoders(hopDeg, hopDeg);
            Localization.update();    // closed gate: must not read the IMU.
        }
        check(mmL.getImuEulerCallCount == readsBefore,
              "TC7: no IMU read while the nav gate is closed (mission hot path)");
    }

    // -------------------------------------------------------------------------------------
    // TC8 (review P2 re-zero) — an idle re-zero while the integrate gate is ALREADY open must
    // reset the integrator: the offset reads ~0 afterward, and accumulation resumes in the NEW
    // frame. This is the bench re-run / MISSION_ALLOW_REARM path the review flagged.
    //
    // The state machine only re-baselines on the closed->open EDGE (gateOpen && !_gateWasOpen).
    // The normal start zeroes while the gate is CLOSED (!_zeroed), so the first open tick takes
    // the reopen edge. But a re-zero while idle leaves the gate OPEN and _gateWasOpen == true, so
    // without the fix the reopen edge is skipped: the next update() integrates against the stale
    // _offset / baselines under a freshly rebound _yawOffsetDeg, reporting the prior run's
    // displacement instead of zero. zeroHeadingFromCurrent() must clear the offset AND force the
    // reopen path so the fresh frame starts clean.
    // -------------------------------------------------------------------------------------
    {
        resetWorld();

        // First run: zero at world heading 0, open the gate, settle, then hand-drive 25 cm so the
        // integrator holds a real, non-zero offset (the "prior bench run" displacement).
        mmL.mockYawDeg = 0.0;
        check(Localization.zeroHeadingFromCurrent(), "TC8: first zeroHeadingFromCurrent succeeds");
        Localization.setNavActive(false);    // gate open
        Localization.update();               // reopen-edge settle (live read OK)

        const int32_t driveDeg = (int32_t) cmToEncDeg(25.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        Localization.update();
        check(offsetMag(Localization.getOffsetFromNode()) > 1.0f,
              "TC8: offset is non-zero after the first run's hand-drive (precondition)");

        // Bench re-zero: the operator re-orients the chassis (IMU now reads 30 deg) and re-zeroes
        // WITHOUT any begin() / node arrival. The gate stays open and _gateWasOpen stays true.
        mmL.mockYawDeg = 30.0;
        check(Localization.zeroHeadingFromCurrent(), "TC8: idle re-zero succeeds");

        // The very next tick must NOT report the prior run's offset. The reopen path re-baselines
        // from the fresh frame and the cleared offset reads ~0.
        Localization.update();
        const Offset2D afterRezero = Localization.getOffsetFromNode();
        check(approx(offsetMag(afterRezero), 0.0f, 1e-4f),
              "TC8: an idle re-zero clears the stale offset (starts from zero)");
        check(approx(afterRezero.thetaDeg, 0.0f, 1e-4f),
              "TC8: an idle re-zero clears the stale thetaDeg (no frame-discontinuity phantom)");

        // Accumulation resumes correctly in the NEW frame: a 10 cm push grows the offset to ~10 cm.
        const int32_t resumeDeg = (int32_t) cmToEncDeg(10.0f);
        Drive.mockAdvanceEncoders(resumeDeg, resumeDeg);
        Localization.update();
        check(approx(offsetMag(Localization.getOffsetFromNode()), 10.0f, 1.0f),
              "TC8: the integrator resumes in the fresh frame after a re-zero");
    }

    if (g_failures == 0)
    {
        std::printf("LOCALIZATION OFFSET OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
