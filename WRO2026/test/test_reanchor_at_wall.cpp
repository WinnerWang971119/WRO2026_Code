// =========================================================================================
// Host unit test for the wall-zero re-anchor (Localization::reanchorAtWall, PR2 / T15).
//
// Links the REAL production `Localization.cpp` (the same surface as
// `test_localization_offset.cpp`), driving `reanchorAtWall(axis, wallCoord, headingDeg)`
// through the mock lower-MCU IMU (`mmL.GetIMUEuler` + `mmL.mockYawDeg`) and the mock `Drive`
// encoders. Two acceptance cases:
//
//   TC11 (AC9)  — reanchorAtWall atomicity. On the success path the post-conditions hold all
//                 at once: getHeadingDeg() == headingDeg, the believed on-axis coordinate
//                 (nodeCoord[axis] + offset[axis]) == wallCoord, the off-axis offset + thetaDeg
//                 are 0, the node is unchanged, and the next update() tick integrates a zero
//                 delta. The strict-REJECT path (IMU read failure here, since the strict flag is
//                 OFF by default) mutates NOTHING — snapshot heading + offset + node, force a
//                 reject, assert every field is byte-for-byte unchanged.
//
//   TC12 (AC11, flag-OFF half) — with `Config::WALL_NORMAL_STRICT_ENABLED` OFF (the SHIPPED
//                 default this target compiles against), reanchorAtWall ALWAYS re-anchors and
//                 returns true regardless of how far the live IMU heading is from headingDeg —
//                 it trusts the preceding wallStop squared the chassis. The ON-path boundary
//                 (`diff > tolerance` rejects) is exercised by the sibling strict-ON target
//                 `test_reanchor_at_wall_strict` (see its docblock + CMakeLists.txt).
//
// HOW THE SUT IS LINKED (see CMakeLists.txt `test_reanchor_at_wall`): identical to
// `test_localization_offset` — `Localization.cpp` compiled UNCHANGED, the mock `Drive.h`
// force-included (guard `DRIVE_H`) to shadow the real one, `<MatrixMiniR4.h>` / `<Arduino.h>`
// resolved by the `mocks/`-first include path, and `routes.cpp` linked for the
// `Routes::START_NODE / NODES / N_NODES` symbols Localization odr-uses.
//
// Node fixture (routes.cpp): node 5 ("Wall1") is the toy field's sole RESET node at
// (x=165, y=139); its reset record is { axis=1 (y), wall_coord=154, facing=90 }. The test
// anchors the believed node to 5 via declarePosition(5) before each reanchor so the on-axis
// post-condition (nodeCoord.y + offset.y == 154) is checked against the real node coordinate.
//
// Float-tolerance checks use a local `approx`/`check` pair (the shared `TestAssert.h` ASSERT_EQ
// casts to `long long`, truncating the sub-cm / sub-degree quantities). Conventions mirror
// `test_localization_offset.cpp`.
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "Localization.h"

// Mock surfaces — the same singletons `Localization.cpp` was compiled against.
#include "Drive.h"            // mock: settable encoders (mockAdvanceEncoders / mockReset).
#include "MatrixMiniR4.h"     // mock: mmL.GetIMUEuler + mmL.mockYawDeg / mockResult.
#include "WROMath.h"          // cmToEncDeg — scripts "drive d cm" into encoder degrees.
#include "Config.h"           // Config::WALL_NORMAL_STRICT_ENABLED / WALL_NORMAL_TOLERANCE_DEG.
#include "FieldMap.h"         // FieldMap::getNode — the real node-coordinate source under test.
#include "routes.h"           // Routes::NODES / N_NODES (data in routes.cpp).

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

// The toy field's sole RESET node and its authored reset record (routes.cpp:21,129).
static constexpr uint8_t kResetNode   = 5;       // "Wall1"
static constexpr uint8_t kAxisY       = 1;       // axis 1 = y
static constexpr float   kWallCoordY  = 154.0f;  // wall_coord on y
static constexpr float   kFacingDeg   = 90.0f;   // wall normal heading

// -----------------------------------------------------------------------------------------
// Per-case world reset: clear both mock surfaces, re-seed Localization, then declare the
// believed node to be the reset node and zero the heading frame at world heading 0. After
// this helper the chassis is anchored on node 5 with a clean offset and a bound frame.
// -----------------------------------------------------------------------------------------
static void resetWorldOnResetNode()
{
    Drive.mockReset();
    mmL.mockReset();
    Localization.begin();

    mmL.mockYawDeg = 0.0;                       // zero at world heading 0.
    Localization.zeroHeadingFromCurrent();      // bind the frame (_yawOffsetDeg = 0).
    Localization.setNavActive(false);           // open the integrate gate (hand-drive recovery).
    Localization.declarePosition(kResetNode);   // anchor the believed node to "Wall1" (node 5).
}

/// Believed coordinate on @p axis = nodeCoord[axis] + offset[axis] (the pose model has no
/// stored absolute pose). Reads the REAL node coordinate via FieldMap::getNode.
static float believedCoordOnAxis(uint8_t node, uint8_t axis)
{
    const Routes::NodeInfo& n   = FieldMap::getNode(node);
    const Offset2D          off = Localization.getOffsetFromNode();
    const float             nodeCoord = (axis == 0) ? n.x_cm : n.y_cm;
    const float             offAxis   = (axis == 0) ? off.x : off.y;
    return nodeCoord + offAxis;
}

int main()
{
    std::printf("INFO: reanchorAtWall — real Localization.cpp; strict flag = %s\n",
                Config::WALL_NORMAL_STRICT_ENABLED ? "ON" : "OFF");

    // This target compiles against the SHIPPED default (strict OFF). Assert that precondition
    // loudly so a future flip of the production default does not silently change what TC12
    // (flag-OFF half) is proving here.
    check(Config::WALL_NORMAL_STRICT_ENABLED == false,
          "precondition: this target compiles with WALL_NORMAL_STRICT_ENABLED OFF (shipped default)");

    // -------------------------------------------------------------------------------------
    // TC11 (AC9) — success-path post-conditions all hold at once (atomic re-anchor).
    //
    // Setup: anchor on node 5, hand-drive a real offset so there is non-zero state to clear,
    // then reanchorAtWall(y, 154, 90). The believed y becomes 154, heading becomes 90, the
    // off-axis (x) offset and thetaDeg reset to 0, the node stays 5, and the next idle tick
    // integrates a zero delta (baselines re-snapshotted).
    // -------------------------------------------------------------------------------------
    {
        resetWorldOnResetNode();

        // Hand-drive 25 cm forward + bump the heading so BOTH channels carry non-zero state.
        Localization.update();    // settle baselines at the post-declare pose.
        const int32_t driveDeg = (int32_t) cmToEncDeg(25.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        mmL.mockYawDeg = 30.0;    // a manual turn so thetaDeg accumulates too.
        Localization.update();
        check(offsetMag(Localization.getOffsetFromNode()) > 1.0f,
              "TC11: offset is non-zero after a hand-drive (pre-reanchor)");

        const uint8_t nodeBefore = Localization.getCurrentNode();

        // Re-square against the wall. Live IMU yaw is 30 (well off the 90 normal), but with the
        // strict check OFF the re-anchor proceeds and binds heading to exactly 90.
        const bool ok = Localization.reanchorAtWall(kAxisY, kWallCoordY, kFacingDeg);
        check(ok, "TC11: reanchorAtWall returns true on the success path (strict OFF)");

        // (1) heading re-bound to the wall normal.
        check(approx(Localization.getHeadingDeg(), kFacingDeg, 0.5f),
              "TC11: getHeadingDeg() == facing (heading squared to the wall normal)");

        // (2) believed on-axis coordinate == wall coordinate (nodeCoord.y 139 + offset.y 15 == 154).
        check(approx(believedCoordOnAxis(kResetNode, kAxisY), kWallCoordY, 0.01f),
              "TC11: nodeCoord[y] + offset[y] == wall_coord (154)");

        // (3) off-axis offset (x) and thetaDeg are zeroed — the wall gives no off-axis info and
        //     the heading delta channel resets after the re-bind.
        const Offset2D off = Localization.getOffsetFromNode();
        check(approx(off.x, 0.0f, 1e-4f), "TC11: off-axis (x) offset is zeroed");
        check(approx(off.thetaDeg, 0.0f, 1e-4f), "TC11: thetaDeg is zeroed");

        // (4) the node is left as-is (the caller anchored it on arrival; reanchor never moves it).
        check(Localization.getCurrentNode() == nodeBefore,
              "TC11: reanchorAtWall leaves _currentNode unchanged");

        // (5) the next idle tick (encoders/yaw unchanged) integrates a zero delta — baselines
        //     were re-snapshotted, so no arrival-tick phantom equal to the pre-reanchor motion.
        Localization.update();
        const Offset2D afterTick = Localization.getOffsetFromNode();
        check(approx(believedCoordOnAxis(kResetNode, kAxisY), kWallCoordY, 0.01f),
              "TC11: believed y stays 154 on the tick after reanchor (baselines re-snapshotted)");
        check(approx(afterTick.x, 0.0f, 1e-4f) && approx(afterTick.thetaDeg, 0.0f, 1e-4f),
              "TC11: off-axis + thetaDeg stay 0 on the tick after reanchor");
    }

    // -------------------------------------------------------------------------------------
    // TC11 atomicity (AC9) — a REJECTED reanchor mutates NOTHING.
    //
    // The only reject path reachable with the strict flag OFF is a failed IMU read. Snapshot
    // heading + offset + node, script a transient lower-MCU timeout (mockResult = ERROR), call
    // reanchorAtWall, and assert it returns false having left every observable field byte-for-byte
    // unchanged (no half-anchored state). This is the single-threaded observation of atomicity:
    // the strict check / IMU read happen BEFORE any write, so a reject is a clean no-op.
    // -------------------------------------------------------------------------------------
    {
        resetWorldOnResetNode();

        // Build a known, non-trivial offset so "unchanged" is a meaningful assertion (not all-zero).
        Localization.update();    // settle baselines.
        const int32_t driveDeg = (int32_t) cmToEncDeg(12.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        mmL.mockYawDeg = 20.0;
        Localization.update();

        // Snapshot the full observable pose BEFORE the rejected call.
        const float    headingBefore = Localization.getHeadingDeg();
        const Offset2D offsetBefore  = Localization.getOffsetFromNode();
        const uint8_t  nodeBefore    = Localization.getCurrentNode();

        // Script a transient IMU timeout so the single up-front read fails -> reject, NO mutation.
        mmL.mockResult = MMLower::RESULT::ERROR;
        const bool ok  = Localization.reanchorAtWall(kAxisY, kWallCoordY, kFacingDeg);
        check(!ok, "TC11 atomicity: reanchorAtWall returns false when the IMU read fails");

        // Restore an OK read so the post-snapshot heading observation does not itself fail and
        // fall back to the cached value — we want to compare the LIVE heading, unchanged.
        mmL.mockResult = MMLower::RESULT::OK;

        const float    headingAfter = Localization.getHeadingDeg();
        const Offset2D offsetAfter  = Localization.getOffsetFromNode();
        const uint8_t  nodeAfter    = Localization.getCurrentNode();

        check(approx(headingAfter, headingBefore, 1e-4f),
              "TC11 atomicity: heading is byte-for-byte unchanged after a rejected reanchor");
        check(approx(offsetAfter.x, offsetBefore.x, 1e-4f)
                  && approx(offsetAfter.y, offsetBefore.y, 1e-4f)
                  && approx(offsetAfter.thetaDeg, offsetBefore.thetaDeg, 1e-4f),
              "TC11 atomicity: the whole offset is unchanged after a rejected reanchor");
        check(nodeAfter == nodeBefore,
              "TC11 atomicity: the current node is unchanged after a rejected reanchor");
    }

    // -------------------------------------------------------------------------------------
    // TC12 (AC11, flag-OFF half) — with the strict check OFF, reanchorAtWall ALWAYS re-anchors
    // regardless of how far the live IMU heading is from the expected normal. Sweep several
    // headings (including ones FAR beyond any plausible tolerance) and assert every one returns
    // true and binds heading to exactly the requested normal. The ON-path boundary is the
    // sibling strict-ON target's job.
    // -------------------------------------------------------------------------------------
    {
        // Headings spanning from on-normal to fully anti-normal (180 deg off). All must accept
        // because the flag is OFF — the method trusts the wallStop squared the chassis.
        const double sweepYaw[] = {90.0, 100.0, 70.0, 130.0, 0.0, 270.0};
        for (double yaw : sweepYaw)
        {
            resetWorldOnResetNode();
            Localization.update();    // settle baselines.

            mmL.mockYawDeg = yaw;     // live heading wildly off the 90-deg normal for most entries.
            const bool ok  = Localization.reanchorAtWall(kAxisY, kWallCoordY, kFacingDeg);
            check(ok, "TC12 (OFF): reanchorAtWall accepts any heading when the strict flag is OFF");
            check(approx(Localization.getHeadingDeg(), kFacingDeg, 0.5f),
                  "TC12 (OFF): heading is bound to the wall normal regardless of the live yaw");
            check(approx(believedCoordOnAxis(kResetNode, kAxisY), kWallCoordY, 0.01f),
                  "TC12 (OFF): believed on-axis coordinate is re-zeroed to wall_coord");
        }
    }

    if (g_failures == 0)
    {
        std::printf("REANCHOR AT WALL OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
