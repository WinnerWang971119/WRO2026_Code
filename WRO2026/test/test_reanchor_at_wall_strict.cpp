// =========================================================================================
// Host unit test for the STRICT wall-normal sanity check in Localization::reanchorAtWall
// (PR2 / T15, AC11) — the ON half of TC12.
//
// Links the REAL production `Localization.cpp`, compiled with `Config::WALL_NORMAL_STRICT_ENABLED`
// forced ON for THIS target only. The production default in `config/ConfigNav.h` is guarded by
// `#ifndef WALL_NORMAL_STRICT_ENABLED_DEFAULT` (shipped value `false`); the CMakeLists.txt
// `test_reanchor_at_wall_strict` target sets `-DWALL_NORMAL_STRICT_ENABLED_DEFAULT=true`, which
// reaches every TU in the target, so both `Localization.cpp` and this test main see the flag ON.
//
// What the strict check does (Localization.cpp:193-203): it reads the live IMU heading once,
// computes the wrap-around-correct absolute angular difference against the expected wall normal,
// and REJECTS the re-anchor (returns false, NO mutation) when `diff > WALL_NORMAL_TOLERANCE_DEG`.
// A difference EXACTLY equal to the tolerance still re-anchors (inclusive boundary; `>` rejects).
//
// TEST GEOMETRY
//   The frame is zeroed at world heading 0 (`_yawOffsetDeg == 0`), so the live world heading the
//   strict check sees equals `normalize(mockYawDeg)`. With the wall normal fixed at facing = 90 and
//   tolerance = 10, the difference is `|mockYawDeg - 90|` (for the headings used here, which stay
//   clear of the 0/360 seam), letting the test place yaw EXACTLY at / just inside / just outside the
//   boundary deterministically. The 0/360-wrap correctness of `_shortestAngleDelta` is already
//   covered by the offset-integrator tests; here the focus is the `> tol` boundary decision.
//
//   TC12 (ON) boundary cases:
//     - yaw = 90            -> diff 0     -> ACCEPT (on-normal).
//     - yaw = 100           -> diff 10    -> ACCEPT (exactly at tolerance; inclusive).
//     - yaw = 80            -> diff 10    -> ACCEPT (exactly at tolerance, other side).
//     - yaw = 99.5          -> diff 9.5   -> ACCEPT (just inside).
//     - yaw = 100.5         -> diff 10.5  -> REJECT, no mutation (just outside).
//     - yaw = 70 / 130 etc. -> diff 20    -> REJECT, no mutation (FAR off-normal, AC11 core).
//
// Float-tolerance checks use a local `approx`/`check` pair (same rationale as the sibling tests).
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "Localization.h"

#include "Drive.h"            // mock: settable encoders (mockReset / mockAdvanceEncoders).
#include "MatrixMiniR4.h"     // mock: mmL.GetIMUEuler + mmL.mockYawDeg.
#include "WROMath.h"          // cmToEncDeg.
#include "Config.h"           // Config::WALL_NORMAL_STRICT_ENABLED (forced ON via the shadow).
#include "FieldMap.h"         // FieldMap::getNode — believed-coordinate post-condition.
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

static bool approx(float a, float b, float tol)
{
    return std::fabs(a - b) <= tol;
}

static float offsetMag(const Offset2D& o)
{
    return std::sqrt(o.x * o.x + o.y * o.y);
}

// The toy field's RESET node + its authored reset record (routes.cpp:21,129).
static constexpr uint8_t kResetNode  = 5;       // "Wall1" at (165, 139)
static constexpr uint8_t kAxisY      = 1;       // axis 1 = y
static constexpr float   kWallCoordY = 154.0f;  // wall_coord on y
static constexpr float   kFacingDeg  = 90.0f;   // wall normal heading

/// Anchor the believed node on the reset node, frame zeroed at world heading 0, gate open.
static void resetWorldOnResetNode()
{
    Drive.mockReset();
    mmL.mockReset();
    Localization.begin();

    mmL.mockYawDeg = 0.0;
    Localization.zeroHeadingFromCurrent();      // _yawOffsetDeg = 0 -> live heading == normalize(yaw).
    Localization.setNavActive(false);
    Localization.declarePosition(kResetNode);
}

static float believedCoordOnAxis(uint8_t node, uint8_t axis)
{
    const Routes::NodeInfo& n   = FieldMap::getNode(node);
    const Offset2D          off = Localization.getOffsetFromNode();
    const float             nodeCoord = (axis == 0) ? n.x_cm : n.y_cm;
    const float             offAxis   = (axis == 0) ? off.x : off.y;
    return nodeCoord + offAxis;
}

/// Run one reanchor attempt at the given live IMU yaw and report whether it accepted.
/// Re-establishes a clean anchored pose first so each case is independent.
static bool attemptReanchorAtYaw(double yaw)
{
    resetWorldOnResetNode();
    Localization.update();        // settle baselines.
    mmL.mockYawDeg = yaw;         // live world heading == normalize(yaw) (offset is 0).
    return Localization.reanchorAtWall(kAxisY, kWallCoordY, kFacingDeg);
}

/// Attempt a reanchor at @p yaw expecting REJECTION, asserting NO mutation occurred.
static void expectRejectNoMutation(double yaw, const char* label)
{
    resetWorldOnResetNode();

    // Build a known non-zero offset so "unchanged" is meaningful (not trivially all-zero).
    Localization.update();
    const int32_t driveDeg = (int32_t) cmToEncDeg(8.0f);
    Drive.mockAdvanceEncoders(driveDeg, driveDeg);
    mmL.mockYawDeg = 0.0;         // accumulate the translation at heading 0 first.
    Localization.update();

    // Move the live IMU heading off-normal BEFORE snapshotting, so the before/after heading reads
    // are at the SAME yaw. `getHeadingDeg()` is a LIVE read = normalize(yaw - _yawOffsetDeg): with
    // yaw held fixed across both reads, any change in the observed heading is a `_yawOffsetDeg`
    // mutation. A rejected reanchor must leave `_yawOffsetDeg` untouched, so the two reads match.
    mmL.mockYawDeg = yaw;

    const float    headingBefore = Localization.getHeadingDeg();
    const Offset2D offsetBefore  = Localization.getOffsetFromNode();
    const uint8_t  nodeBefore    = Localization.getCurrentNode();

    // Attempt the (expected-rejected) reanchor at the off-normal heading.
    const bool ok  = Localization.reanchorAtWall(kAxisY, kWallCoordY, kFacingDeg);
    check(!ok, label);

    const float    headingAfter = Localization.getHeadingDeg();
    const Offset2D offsetAfter  = Localization.getOffsetFromNode();
    const uint8_t  nodeAfter    = Localization.getCurrentNode();

    // Same live yaw both reads -> an unchanged heading proves `_yawOffsetDeg` was not mutated.
    check(approx(headingAfter, headingBefore, 1e-4f),
          "TC12 (ON) reject: heading unchanged (no half-anchored mutation)");
    check(approx(offsetAfter.x, offsetBefore.x, 1e-4f)
              && approx(offsetAfter.y, offsetBefore.y, 1e-4f)
              && approx(offsetAfter.thetaDeg, offsetBefore.thetaDeg, 1e-4f),
          "TC12 (ON) reject: offset unchanged (no half-anchored mutation)");
    check(nodeAfter == nodeBefore, "TC12 (ON) reject: node unchanged");
}

int main()
{
    std::printf("INFO: reanchorAtWall STRICT boundary — real Localization.cpp; strict flag = %s\n",
                Config::WALL_NORMAL_STRICT_ENABLED ? "ON" : "OFF");

    // The whole point of this target is the strict flag being ON. Assert the shadow took effect —
    // if a build mis-wires the force-include, this fails loudly instead of silently testing OFF.
    check(Config::WALL_NORMAL_STRICT_ENABLED == true,
          "precondition: this target compiles with WALL_NORMAL_STRICT_ENABLED ON (scoped shadow)");
    check(approx(Config::WALL_NORMAL_TOLERANCE_DEG, 10.0f, 1e-4f),
          "precondition: tolerance is 10 deg (shadow mirrors the production default)");

    // -------------------------------------------------------------------------------------
    // ACCEPT cases: on-normal, exactly-at-tolerance (both sides), and just-inside.
    // -------------------------------------------------------------------------------------
    {
        const bool onNormal = attemptReanchorAtYaw(90.0);
        check(onNormal, "TC12 (ON): on-normal (diff 0) re-anchors");
        check(approx(Localization.getHeadingDeg(), kFacingDeg, 0.5f),
              "TC12 (ON): on-normal accept binds heading to the wall normal");
        check(approx(believedCoordOnAxis(kResetNode, kAxisY), kWallCoordY, 0.01f),
              "TC12 (ON): on-normal accept re-zeros the on-axis coordinate to wall_coord");

        // Exactly at the tolerance, high side: diff == 10 -> ACCEPT (inclusive `>` boundary).
        check(attemptReanchorAtYaw(100.0),
              "TC12 (ON): diff exactly == tolerance (high side) re-anchors (inclusive boundary)");
        // Exactly at the tolerance, low side: diff == 10 -> ACCEPT.
        check(attemptReanchorAtYaw(80.0),
              "TC12 (ON): diff exactly == tolerance (low side) re-anchors (inclusive boundary)");
        // Just inside: diff 9.5 -> ACCEPT.
        check(attemptReanchorAtYaw(99.5),
              "TC12 (ON): diff just inside the tolerance (9.5 < 10) re-anchors");
    }

    // -------------------------------------------------------------------------------------
    // REJECT cases: just outside the tolerance, and FAR off-normal (AC11 core). Each asserts
    // NO mutation (atomic reject).
    // -------------------------------------------------------------------------------------
    {
        // Just outside, high side: diff 10.5 > 10 -> REJECT.
        expectRejectNoMutation(100.5, "TC12 (ON): diff just outside (10.5 > 10) REJECTS the reanchor");
        // Just outside, low side: diff 10.5 -> REJECT.
        expectRejectNoMutation(79.5, "TC12 (ON): diff just outside (low side) REJECTS the reanchor");
        // Far off-normal (AC11 core): chassis stopped against a corner/obstacle, not the wall.
        expectRejectNoMutation(70.0, "TC12 (ON): far off-normal (diff 20) REJECTS the reanchor");
        expectRejectNoMutation(130.0, "TC12 (ON): far off-normal (diff 40) REJECTS the reanchor");
    }

    // -------------------------------------------------------------------------------------
    // Sanity: a rejected reanchor does NOT consume the offset (the offset integrator's state is
    // preserved across the rejected attempt — proven by offsetMag staying non-zero above), and a
    // subsequent accept at an in-tolerance heading still works (the reject did not latch a bad
    // state that blocks future re-anchors).
    // -------------------------------------------------------------------------------------
    {
        resetWorldOnResetNode();
        Localization.update();

        // Reject once (far off-normal)...
        mmL.mockYawDeg = 60.0;
        check(!Localization.reanchorAtWall(kAxisY, kWallCoordY, kFacingDeg),
              "TC12 (ON): off-normal reanchor rejects");

        // ...then succeed once the chassis is squared (yaw back on-normal).
        mmL.mockYawDeg = 92.0;    // diff 2 -> well inside tolerance.
        check(Localization.reanchorAtWall(kAxisY, kWallCoordY, kFacingDeg),
              "TC12 (ON): a later in-tolerance reanchor succeeds after an earlier reject");
        check(approx(believedCoordOnAxis(kResetNode, kAxisY), kWallCoordY, 0.01f),
              "TC12 (ON): the recovering accept re-zeros the on-axis coordinate");
        check(approx(offsetMag(Localization.getOffsetFromNode())
                         - std::fabs(kWallCoordY - FieldMap::getNode(kResetNode).y_cm),
                     0.0f, 0.01f),
              "TC12 (ON): post-accept offset magnitude equals the node->wall y gap (15 cm)");
    }

    if (g_failures == 0)
    {
        std::printf("REANCHOR AT WALL STRICT OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
