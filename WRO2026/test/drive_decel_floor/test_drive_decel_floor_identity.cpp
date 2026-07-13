/**
 * @file test_drive_decel_floor_identity.cpp
 * @brief TC4 (AC5) — the `PWM_MOVING_MIN` off-switch / identity build.
 *
 * @details
 * Compiled into its OWN cmake target (`test_drive_decel_floor_identity`) with a TARGET-WIDE
 * compile definition `PWM_MOVING_MIN_DEFAULT=PWM_MIN_MAGNITUDE` (the SYMBOL, not a hardcoded
 * 35, so this stays correct across a future floor retune). Per the plan's off-switch decision:
 * "PWM_MOVING_MIN == PWM_MIN_MAGNITUDE -> identity". With the two floors equal,
 * `deadbandSnap(pwm, floorMag)` snaps every sub-floor setpoint to the SAME magnitude
 * regardless of whether `_decelTailFloorMag` resolved the static or kinetic floor for that
 * tick — so the SAME `.tShape()` stop scenario TC3 uses (cruise 70, startSpeed default 35,
 * endSpeed 0, distance 40cm) must dispatch `+-35` everywhere, including what would otherwise
 * be the decel tail.
 *
 * MUST be a target-wide `-D`, never an in-file `#define`: `Config::PWM_MOVING_MIN` is defined
 * in `ConfigGlobal.h` inside the SEPARATELY-COMPILED `Drive.cpp` translation unit. An in-file
 * `#define PWM_MOVING_MIN_DEFAULT ...` at the top of this .cpp would only affect THIS TU's view
 * of the constant (and this TU doesn't even define `Config::PWM_MOVING_MIN` itself — it is a
 * `constexpr` in a header, resolved wherever that header is included) — `Drive.cpp`, compiled
 * separately without the same in-file `#define`, would still read the default 30 and the
 * mock's recorded dispatch would disagree with this test's expectation. `target_compile_
 * definitions(test_drive_decel_floor_identity PRIVATE PWM_MOVING_MIN_DEFAULT=PWM_MIN_MAGNITUDE)`
 * in CMakeLists.txt reaches EVERY translation unit in the target (this test main AND the
 * separately-compiled `Drive.cpp`), so both sides agree. Under the VS multi-config generator,
 * ctest for this target is invoked with `-C Debug`.
 *
 * Otherwise an exact structural mirror of `test_drive_decel_floor.cpp`'s TC3 — same include
 * set, same real-Drive.cpp link recipe (see that file's header docblock for the full
 * rationale), same `resetWorld` / `tickProfileToEnc` helpers (duplicated here rather than
 * shared via a third header, matching the existing host-test precedent of self-contained
 * single-TU test mains — e.g. `test_path_tracker_passthrough.cpp` is its own complete file
 * per ON/OFF build, not split out).
 */

#include "TestAssert.h"

#include <cstdio>

#include "src/drive/Drive.h"    // real — DriveClass + the global `Drive` (defined in real Drive.cpp)
#include "MatrixMiniR4.h"    // mock MiniR4.M2/M3/DriveDC.
#include "src/util/WROMath.h"    // cmToEncDeg (ramp-distance math) — deadbandSnap itself is not
                                  // called directly by this file; it is exercised indirectly
                                  // through the real Drive.cpp under test.

namespace
{
void resetWorld()
{
    mockResetMiniR4();
    Drive.begin();
    Drive.stop();    // force-idle any in-flight profile/queue state — see the sibling
                      // test_drive_decel_floor.cpp resetWorld() docblock for why.
}

/// @brief Mirrors `test_drive_decel_floor.cpp`'s helper of the same name — advances `MiniR4.M2`/
///        `M3` one encoder-degree at a time (ticking `Drive.update()` each step) until the
///        advance since @p startEnc reaches @p targetEncDelta or @p maxTicks is exhausted.
void tickProfileToEnc(int32_t targetEncDelta, int32_t startEnc, int maxTicks)
{
    for (int i = 0; i < maxTicks; ++i)
    {
        const int32_t now = MiniR4.M2.getDegrees() - startEnc;
        if (now >= targetEncDelta)
        {
            break;
        }
        MiniR4.M2.mockAdvanceDegrees(1);
        MiniR4.M3.mockAdvanceDegrees(1);
        Drive.update();
    }
}
}    // namespace

// ===========================================================================================
// TC4 — identity build: PWM_MOVING_MIN == PWM_MIN_MAGNITUDE disables the feature. The TC3
// scenario's decel tail (which would dispatch +-PWM_MOVING_MIN=30 in the default build) must
// dispatch +-PWM_MIN_MAGNITUDE=35 here instead — byte-identical to the pre-feature build.
// ===========================================================================================
void test_TC4_identity_build_tail_dispatches_static_floor()
{
    TEST_SCOPE("TC4: PWM_MOVING_MIN_DEFAULT=PWM_MIN_MAGNITUDE identity — decel tail dispatches +-35");

    // Precondition proves the off-switch actually engaged for THIS build.
    ASSERT_EQ((int) Config::PWM_MOVING_MIN, (int) Config::PWM_MIN_MAGNITUDE);

    resetWorld();

    const int32_t startEnc = MiniR4.M2.getDegrees();

    Drive.driveDist(40.0f).tShape().async();
    Drive.update();    // dispatch tick.

    const int32_t profileStartEnc = MiniR4.M2.getDegrees();
    ASSERT_EQ(profileStartEnc, startEnc);

    const int32_t encPerCm = (int32_t) cmToEncDeg(1.0f);

    // Cruise plateau — same 15cm landing spot as TC3; still +-cruise regardless of the identity
    // override (the cruise phase never touches either floor constant).
    tickProfileToEnc(15L * encPerCm, profileStartEnc, 5000);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, (int16_t) Config::SPEED_NORMAL);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncR, (int16_t) Config::SPEED_NORMAL);

    // Decel tail — same 38cm landing spot as TC3. With the floors equal, this must now dispatch
    // the STATIC floor value (35), not 30 — the identity build's defining assertion.
    tickProfileToEnc(38L * encPerCm, profileStartEnc, 5000);
    const int16_t tailL = MiniR4.DriveDC.lastMoveSyncL;
    const int16_t tailR = MiniR4.DriveDC.lastMoveSyncR;
    ASSERT_EQ(tailL, (int16_t) Config::PWM_MIN_MAGNITUDE);
    ASSERT_EQ(tailR, (int16_t) Config::PWM_MIN_MAGNITUDE);
    ASSERT_FALSE(Drive.hasFault());
}

// ===========================================================================================
// Entry.
// ===========================================================================================
int main()
{
    std::printf("INFO: Drive.tShape() PWM_MOVING_MIN identity build (T6, TC4) -- "
                "PWM_MIN_MAGNITUDE=%d PWM_MOVING_MIN=%d (must be equal)\n",
                (int) Config::PWM_MIN_MAGNITUDE,
                (int) Config::PWM_MOVING_MIN);

    test_TC4_identity_build_tail_dispatches_static_floor();

    TEST_REPORT();
}
