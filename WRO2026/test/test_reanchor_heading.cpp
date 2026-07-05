// =========================================================================================
// Host unit test for the heading-only line-arrival re-anchor (Localization::reanchorHeading,
// ADR-052 / T3). Covers TC1-TC6 from the plan's Testing Strategy (AC1-AC5); TC7 (AC6, the
// `reanchorAtWall` regression) is proven by simply not disturbing `test_reanchor_at_wall[_strict]`
// — this file adds NEW targets, it does not touch the wall siblings.
//
// Links the REAL production `Localization.cpp` (same surface as `test_reanchor_at_wall`), driving
// `reanchorHeading(headingDeg, outPreBindDeltaDeg)` through the mock lower-MCU IMU
// (`mmL.GetIMUEuler` + `mmL.mockYawDeg` / `mmL.mockResult`) and the mock `Drive` encoders
// (`Drive.mockAdvanceEncoders`, used only by the AC2 offset-untouched case).
//
// STRICT ON/OFF — TWO EXECUTABLES FROM ONE SOURCE FILE:
//   `Config::LINE_HEADING_RESET_STRICT_ENABLED` ships ON by default (`ConfigNav.h`), unlike the
//   wall-reset flag it is modeled on (which ships OFF). Rather than write two near-duplicate
//   `.cpp` files, this ONE file is compiled into TWO executables (`test_reanchor_heading`, the
//   shipped-default strict-ON build, and `test_reanchor_heading_strict_off`, which flips
//   `LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=false` target-wide via
//   `target_compile_definitions` — the `test_reanchor_at_wall_strict` `-D` idiom, CMakeLists.txt).
//   `main()` branches on the compiled-in `Config::LINE_HEADING_RESET_STRICT_ENABLED` value to run
//   the strict-boundary cases (TC3/TC4) in the ON build and a non-vacuity proof in the OFF build:
//   a delta (15 deg) that the ON build REJECTS must be ACCEPTED by the OFF build, proving the
//   compile-def flip actually reached the separately-compiled `Localization.cpp` TU (not just this
//   test main) — the exact cross-TU trap `gotcha-cross-tu-config-override-compile-def` warns about.
//
// Belief-zero setup: every case zeroes the frame at IMU yaw 0 with the fixture's
// `START_HEADING_DEG == 0.0`, so the pre-bind belief is exactly `0.0` and `|belief - h| == h` for
// every target heading `h` used below (h stays in [0, 90), so no 0/360-wrap correction is needed
// to predict the delta by hand) — deltas are exact float comparisons, no fixture-value coupling.
//
// Float-tolerance checks use a local `approx`/`check` pair (the shared `TestAssert.h` ASSERT_EQ
// casts to `long long`, truncating sub-degree quantities) — same convention as
// `test_reanchor_at_wall[_strict].cpp`.
// =========================================================================================

#include <cmath>
#include <cstdio>

#include "src/nav/Localization.h"

// Mock surfaces — the same singletons `Localization.cpp` was compiled against.
#include "Drive.h"            // mock: settable encoders (mockAdvanceEncoders / mockReset).
#include "MatrixMiniR4.h"     // mock: mmL.GetIMUEuler + mmL.mockYawDeg / mockResult.
#include "Arduino.h"          // mock: g_mockMillis / mockResetMillis / mockAdvanceMillis.
#include "src/util/WROMath.h"          // cmToEncDeg — scripts "drive d cm" into encoder degrees.
#include "Config.h"           // Config::LINE_HEADING_RESET_STRICT_ENABLED / _TOLERANCE_DEG.
#include "src/nav/routes.h"           // Routes::START_NODE / START_HEADING_DEG (data in routes_fixture.cpp).

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

/// Absolute-tolerance float compare. `tol` is per-case (deg) — see each call site.
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
// Per-case world reset: clear both mock surfaces, re-seed Localization, zero the heading
// frame at world heading 0 (Routes::START_HEADING_DEG == 0.0 in the fixture), and close the
// integrate gate (_navActive = true) — the precondition reanchorHeading's guard (1) enforces.
// After this helper: _zeroed == true, belief == 0.0, _navActive == true, offset == {0,0,0}.
// -----------------------------------------------------------------------------------------
static void resetWorldZeroedNavActive()
{
    Drive.mockReset();
    mmL.mockReset();
    Localization.begin();

    mmL.mockYawDeg = 0.0;                       // zero at world heading 0 (== START_HEADING_DEG).
    Localization.zeroHeadingFromCurrent();      // bind the frame (_yawOffsetDeg = 0 - START).
    Localization.setNavActive(true);            // close the gate — reanchorHeading precondition.
}

int main()
{
    std::printf("INFO: reanchorHeading — real Localization.cpp; LINE_HEADING_RESET_STRICT_ENABLED = %s, "
                "tolerance = %.2f\n",
                Config::LINE_HEADING_RESET_STRICT_ENABLED ? "ON" : "OFF",
                Config::LINE_HEADING_RESET_TOLERANCE_DEG);

    // -------------------------------------------------------------------------------------
    // TC1 (AC1) — accept path: strict satisfied (delta 5 <= default tolerance 10, or strict
    // OFF entirely), nav active. Post-conditions: the no-I/O cache (getLastHeadingDeg()) and the
    // update stamp are asserted FIRST, BEFORE getHeadingDeg() below — getHeadingDeg() itself does
    // a live read (_readWorldHeading) that refreshes both, so checking it first would let a broken
    // _bindHeadingFromYaw (one that skips the cache/stamp refresh, Localization.cpp:213-214) hide
    // behind that live re-read. Asserting the no-I/O reads first makes the stamp check isolating:
    // a stripped refresh leaves the stamp at 0 (from begin()), not 777. getHeadingDeg() == h and
    // the pre-bind delta are checked too.
    // -------------------------------------------------------------------------------------
    {
        resetWorldZeroedNavActive();
        mockResetMillis();
        mockAdvanceMillis(777);    // known "now" for the stamp-refresh assertion.

        const float kHeading = 5.0f;    // belief is 0.0, so delta == 5.0 (inside the default tolerance).
        float       delta    = -999.0f;
        const bool  ok       = Localization.reanchorHeading(kHeading, &delta);

        check(ok, "TC1: reanchorHeading accepts (strict satisfied / OFF), returns true");
        check(approx(delta, 5.0f, 0.01f), "TC1: outPreBindDeltaDeg reports the pre-bind delta (|0 - 5|)");
        // NO-I/O reads FIRST (isolating — see the block comment above): a regression that strips
        // the cache/stamp refresh out of _bindHeadingFromYaw would fail HERE, before the
        // getHeadingDeg() live read below gets a chance to silently repopulate both.
        check(approx(Localization.getLastHeadingDeg(), kHeading, 0.5f),
              "TC1: getLastHeadingDeg() cache reflects the bound heading (no I/O)");
        check(Localization.getLastHeadingUpdateMs() == 777,
              "TC1: getLastHeadingUpdateMs() stamp refreshed to the bind time");
        check(approx(Localization.getHeadingDeg(), kHeading, 0.5f),
              "TC1: getHeadingDeg() == h after the bind (live re-read)");
    }

    // -------------------------------------------------------------------------------------
    // TC2 (AC2) — offset untouched. Build a genuinely non-zero offset by hand-driving while
    // the gate is OPEN (_navActive == false), THEN close the gate (simulating a hop start) and
    // call reanchorHeading. getOffsetFromNode() must be byte-for-byte identical before/after —
    // reanchorHeading is heading-only and must never touch _offset / the integrator baselines.
    // -------------------------------------------------------------------------------------
    {
        Drive.mockReset();
        mmL.mockReset();
        Localization.begin();

        mmL.mockYawDeg = 0.0;
        Localization.zeroHeadingFromCurrent();
        Localization.setNavActive(false);          // open gate for hand-drive.
        Localization.update();                     // gate-reopen edge: baseline snapshot.

        const int32_t driveDeg = (int32_t) cmToEncDeg(18.0f);
        Drive.mockAdvanceEncoders(driveDeg, driveDeg);
        Localization.update();                     // integrate ~18 cm along heading 0.

        const Offset2D before = Localization.getOffsetFromNode();
        check(offsetMag(before) > 1.0f, "TC2 setup: offset is non-zero before reanchorHeading");

        Localization.setNavActive(true);            // close the gate — simulate a hop starting.

        float      delta = -999.0f;
        const bool ok    = Localization.reanchorHeading(5.0f, &delta);
        check(ok, "TC2: reanchorHeading accepts (setup for the offset-untouched assertion)");

        // Exact equality, not approx(): reanchorHeading provably never writes _offset (see the
        // Localization.cpp contract comment), so getOffsetFromNode() hands back the SAME struct
        // untouched — no float arithmetic happens in between to introduce rounding. AC2 literally
        // requires byte-for-byte, so assert that, not "close enough".
        const Offset2D after = Localization.getOffsetFromNode();
        check(after.x == before.x && after.y == before.y && after.thetaDeg == before.thetaDeg,
              "TC2: getOffsetFromNode() is byte-for-byte unchanged by reanchorHeading (AC2)");
    }

    // -------------------------------------------------------------------------------------
    // TC5 (AC4) — failed IMU read. Script a transient lower-MCU timeout so the single up-front
    // read fails: returns false, NOTHING mutated (cache/stamp/offset), and outPreBindDeltaDeg is
    // left UNTOUCHED (pre-seeded to a sentinel that must survive the call verbatim).
    // -------------------------------------------------------------------------------------
    {
        resetWorldZeroedNavActive();
        mockResetMillis();
        mockAdvanceMillis(300);

        const float    headingBefore = Localization.getHeadingDeg();       // live OK read -> stamp = 300.
        const Offset2D offsetBefore  = Localization.getOffsetFromNode();
        const uint32_t stampBefore   = Localization.getLastHeadingUpdateMs();
        check(stampBefore == 300, "TC5 setup: stamp captured at a known time");

        mmL.mockResult = MMLower::RESULT::ERROR;    // script a transient IMU timeout.
        float      delta = -12345.0f;               // sentinel — must remain UNTOUCHED (AC4).
        const bool ok    = Localization.reanchorHeading(20.0f, &delta);
        check(!ok, "TC5: reanchorHeading returns false when the IMU read fails");
        check(approx(delta, -12345.0f, 1e-6f),
              "TC5: outPreBindDeltaDeg is left UNTOUCHED on a failed read (AC4)");
        check(Localization.getLastHeadingUpdateMs() == stampBefore,
              "TC5: the heading-cache stamp is unchanged on a failed read");
        check(approx(Localization.getLastHeadingDeg(), headingBefore, 0.5f),
              "TC5: the cached heading is unchanged on a failed read");

        mmL.mockResult = MMLower::RESULT::OK;       // restore so later reads in this process succeed.
        const Offset2D offsetAfter = Localization.getOffsetFromNode();
        check(approx(offsetAfter.x, offsetBefore.x, 1e-4f) && approx(offsetAfter.y, offsetBefore.y, 1e-4f) &&
                  approx(offsetAfter.thetaDeg, offsetBefore.thetaDeg, 1e-4f),
              "TC5: offset is unchanged on a failed read");
    }

    // -------------------------------------------------------------------------------------
    // TC6a (AC5) — !_navActive guard. Reopen the gate (setNavActive(false)) so guard (1) fires
    // FIRST (before the !_zeroed / IMU-read / strict guards even run): returns false, nothing
    // mutated, outPreBindDeltaDeg untouched (the guard returns before any IMU read).
    // -------------------------------------------------------------------------------------
    {
        resetWorldZeroedNavActive();
        Localization.setNavActive(false);           // reopen the gate — guard (1) must fire.

        const Offset2D offsetBefore = Localization.getOffsetFromNode();
        const uint32_t stampBefore  = Localization.getLastHeadingUpdateMs();

        float      delta = -777.0f;
        const bool ok    = Localization.reanchorHeading(5.0f, &delta);
        check(!ok, "TC6a: reanchorHeading rejects while !_navActive");
        check(approx(delta, -777.0f, 1e-6f),
              "TC6a: outPreBindDeltaDeg untouched (guard fires before any IMU read)");
        check(Localization.getLastHeadingUpdateMs() == stampBefore,
              "TC6a: the heading-cache stamp is untouched by the !_navActive guard");
        const Offset2D offsetAfter = Localization.getOffsetFromNode();
        check(approx(offsetAfter.x, offsetBefore.x, 1e-4f) && approx(offsetAfter.y, offsetBefore.y, 1e-4f) &&
                  approx(offsetAfter.thetaDeg, offsetBefore.thetaDeg, 1e-4f),
              "TC6a: offset untouched by the !_navActive guard");
    }

    // -------------------------------------------------------------------------------------
    // TC6b (AC5) — !_zeroed guard, exercised SPECIFICALLY (not shadowed by guard (1)):
    // _navActive is set true so guard (1) passes and guard (2) (!_zeroed) is the one that
    // fires. Returns false, nothing mutated, outPreBindDeltaDeg untouched.
    // -------------------------------------------------------------------------------------
    {
        Drive.mockReset();
        mmL.mockReset();
        Localization.begin();                       // _zeroed == false (zeroHeadingFromCurrent not called).
        Localization.setNavActive(true);             // pass guard (1) so guard (2) is what fires.

        const Offset2D offsetBefore = Localization.getOffsetFromNode();
        const uint32_t stampBefore  = Localization.getLastHeadingUpdateMs();

        float      delta = -321.0f;
        const bool ok    = Localization.reanchorHeading(5.0f, &delta);
        check(!ok, "TC6b: reanchorHeading rejects while !_zeroed (even with _navActive true)");
        check(approx(delta, -321.0f, 1e-6f),
              "TC6b: outPreBindDeltaDeg untouched (guard fires before any IMU read)");
        check(Localization.getLastHeadingUpdateMs() == stampBefore,
              "TC6b: the heading-cache stamp is untouched by the !_zeroed guard");
        const Offset2D offsetAfter = Localization.getOffsetFromNode();
        check(approx(offsetAfter.x, offsetBefore.x, 1e-4f) && approx(offsetAfter.y, offsetBefore.y, 1e-4f) &&
                  approx(offsetAfter.thetaDeg, offsetBefore.thetaDeg, 1e-4f),
              "TC6b: offset untouched by the !_zeroed guard");
    }

    // -------------------------------------------------------------------------------------
    // Strict-ON-only (TC3/TC4, AC3) vs strict-OFF-only (non-vacuity) branch — see the file
    // docblock. Both halves use the SAME 15-deg delta so the OFF-accepts/ON-rejects contrast
    // is a direct A/B, not an incidental difference between two unrelated headings.
    // -------------------------------------------------------------------------------------
    if (Config::LINE_HEADING_RESET_STRICT_ENABLED)
    {
        // Assert the compile-def shadow actually took effect (this target's CMakeLists.txt
        // entry explicitly forces LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=true /
        // _TOLERANCE_DEG_DEFAULT=10.0f target-wide) — if a build mis-wires the `-D`, this fails
        // loudly instead of silently skipping straight to the non-vacuity branch below.
        check(Config::LINE_HEADING_RESET_STRICT_ENABLED == true,
              "precondition: this target compiles with LINE_HEADING_RESET_STRICT_ENABLED ON");
        check(approx(Config::LINE_HEADING_RESET_TOLERANCE_DEG, 10.0f, 1e-4f),
              "precondition: tolerance is 10 deg (this target's forced value)");

        // ---- TC3 (AC3) — strict reject: delta (15) exceeds the default tolerance (10). ----
        {
            resetWorldZeroedNavActive();
            mockResetMillis();
            mockAdvanceMillis(500);

            // Seed a DISTINCT non-zero cached heading via a real accept-bind (5 deg, inside
            // tolerance) before the reject call below. Without this, the pre-reject cache would
            // already sit at 0.0 — the SAME value a broken reject that wrongly re-caches to 0
            // would also produce — so "cache unchanged" could not be told apart from "cache wrongly
            // refreshed (to a coincidentally-matching 0)". Seeding to a captured non-zero value
            // makes the post-reject "still equals the pre-reject cache" check independently
            // meaningful. Belief is now 5 (not 0), so the reject target below shifts to 20 to keep
            // the SAME 15-deg delta this case is about.
            const bool seeded = Localization.reanchorHeading(5.0f, nullptr);
            check(seeded, "TC3 setup: seed a distinct non-zero cached heading via an accept-bind");

            const Offset2D offsetBefore = Localization.getOffsetFromNode();
            const uint32_t stampBefore  = Localization.getLastHeadingUpdateMs();
            const float    cacheBefore  = Localization.getLastHeadingDeg();
            check(approx(cacheBefore, 5.0f, 0.5f),
                  "TC3 setup: cache reflects the seeded heading (non-zero, distinct from a bad-reject 0)");

            mockAdvanceMillis(250);    // time passes before the (expected-rejected) call.
            float      delta = -999.0f;
            const bool ok    = Localization.reanchorHeading(20.0f, &delta);    // belief 5, so |5 - 20| = 15.
            check(!ok, "TC3: strict reject when |belief - h| (15) exceeds the tolerance (10)");
            check(approx(delta, 15.0f, 0.01f), "TC3: outPreBindDeltaDeg IS set on a strict reject");
            check(Localization.getLastHeadingUpdateMs() == stampBefore,
                  "TC3: the heading-cache stamp is unchanged on a strict reject");
            check(approx(Localization.getLastHeadingDeg(), cacheBefore, 0.5f),
                  "TC3: the cached heading is unchanged (still the seeded, non-zero pre-reject value) "
                  "on a strict reject");
            const Offset2D offsetAfter = Localization.getOffsetFromNode();
            check(approx(offsetAfter.x, offsetBefore.x, 1e-4f) && approx(offsetAfter.y, offsetBefore.y, 1e-4f) &&
                      approx(offsetAfter.thetaDeg, offsetBefore.thetaDeg, 1e-4f),
                  "TC3: offset unchanged on a strict reject");
        }

        // ---- TC4 (AC3) — strict boundary: delta == tolerance exactly still binds. ----
        {
            resetWorldZeroedNavActive();
            const float kBoundary = Config::LINE_HEADING_RESET_TOLERANCE_DEG;    // delta == tolerance exactly.
            float       delta     = -999.0f;
            const bool  ok        = Localization.reanchorHeading(kBoundary, &delta);
            check(ok, "TC4: delta exactly == tolerance still binds (inclusive boundary)");
            check(approx(delta, kBoundary, 0.01f), "TC4: outPreBindDeltaDeg reports the boundary delta");
            check(approx(Localization.getHeadingDeg(), kBoundary, 0.5f),
                  "TC4: heading is bound to h at the inclusive boundary");
        }
    }
    else
    {
        // Assert the compile-def shadow actually took effect (this target's CMakeLists.txt
        // entry forces LINE_HEADING_RESET_STRICT_ENABLED_DEFAULT=false target-wide) — if a
        // build mis-wires the `-D`, this fails loudly instead of silently re-running the
        // strict-ON branch above.
        check(Config::LINE_HEADING_RESET_STRICT_ENABLED == false,
              "precondition: this target compiles with LINE_HEADING_RESET_STRICT_ENABLED OFF");

        // ---- Non-vacuity proof (strict-OFF build): the SAME 15-deg delta that TC3 (strict-ON
        // build) rejects must be ACCEPTED here — proving this executable actually compiles
        // with LINE_HEADING_RESET_STRICT_ENABLED flipped OFF target-wide (both this test main
        // AND the separately-compiled Localization.cpp TU), not silently re-running the
        // strict-ON behavior (gotcha-cross-tu-config-override-compile-def). ----
        {
            resetWorldZeroedNavActive();
            float      delta = -999.0f;
            const bool ok    = Localization.reanchorHeading(15.0f, &delta);
            check(ok, "non-vacuity: strict OFF accepts a delta (15) that strict ON rejects (tolerance 10)");
            check(approx(delta, 15.0f, 0.01f), "non-vacuity: outPreBindDeltaDeg still reports the delta");
            check(approx(Localization.getHeadingDeg(), 15.0f, 0.5f),
                  "non-vacuity: heading is bound to h with strict OFF");
        }
    }

    if (g_failures == 0)
    {
        std::printf("REANCHOR HEADING OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
