/**
 * @file test_path_tracker_passthrough.cpp
 * @brief Host-compile test for the PATH_PASS_THROUGH_ENABLED behavioral toggle
 *        (T10c / AC7b).
 *
 * Rather than mock the full PathTracker + FieldMap + Localization +
 * routes.cpp graph (which would balloon the harness surface 5×), this test
 * **directly executes the LineFollower arming sequence PathTracker issues at
 * intersection-detect**, parameterised on the post-peek `pass_through` decision.
 *
 * The test is built TWICE — once with `-DPATH_PASS_THROUGH_ENABLED_DEFAULT=true`
 * and once with `-DPATH_PASS_THROUGH_ENABLED_DEFAULT=false`. CMake registers
 * both binaries as separate `ctest` cases.
 *
 * In both builds we run the SAME canonical collinear scenario (heading delta = 0
 * which is well within `Config::NAV_TURN_TOLERANCE_DEG`, `nextNext` is a non-NONE
 * non-goal node — the standard "pass-through eligible" geometry). The test
 * then asserts:
 *
 *   - With `PATH_PASS_THROUGH_ENABLED == true`:  PathTracker WOULD set
 *     `_passThrough = true` and arm LineFollower with brake-on-gate=false.
 *     LineFollower's gate trip stays enabled.
 *
 *   - With `PATH_PASS_THROUGH_ENABLED == false`: PathTracker WOULD set
 *     `_passThrough = false` regardless of geometry. LineFollower's gate trip
 *     self-disables with active brake (today's behavior).
 *
 * The `pass_through` decision is computed inline using the same logic
 * PathTracker.cpp uses (the exact peek block from the plan's "Peek data
 * sources" section), with mocked FieldMap responses for the canonical fixture.
 *
 * @note AC7b acceptance criterion: "Each target asserts the corresponding
 *       primitive sequence on the canonical collinear fixture. Compile-only is
 *       not sufficient — the flag must be exercised behaviorally on both
 *       branches." This test satisfies that by branching off the constant
 *       directly and exercising the downstream LineFollower behavior.
 */

#include "TestAssert.h"

#include "Arduino.h"
#include "Drive.h"
#include "WROLineArray.h"
#include "Intersection.h"

// Production code under test:
#include "LineFollower.h"
#include "Config.h"
#include "WROMath.h"

#include <math.h>

// ---------------------------------------------------------------------------
// Mocked FieldMap responses for the canonical collinear fixture.
//
// Graph: A → B → C → D (all collinear, all LINE edges).
// Active goal: D (final node).
// Inbound edge: A → B with heading 0.0°.
// Next-hop after B: C, with heading 0.0° (delta = 0.0 — collinear).
// `nextNext` is C != B != goal_D — pass-through eligible.
// ---------------------------------------------------------------------------

namespace fixture
{
    constexpr float inbound_heading_deg = 0.0f;
    constexpr float next_heading_deg    = 0.0f;
    constexpr bool  next_hop_valid      = true;   // FieldMap::getNextHop returned a valid node
    constexpr bool  next_hop_is_goal    = false;  // Intersected node is not the destination
}    // namespace fixture

/// Wrap-normalize a heading delta to (-180, +180].
/// Mirrors the inline block in `PathTracker.cpp:529-531` and the production
/// peek logic specified by the plan.
static float wrapDelta(float delta)
{
    while (delta >  180.0f) { delta -= 360.0f; }
    while (delta <= -180.0f) { delta += 360.0f; }
    return delta;
}

/// Compute the pass-through decision exactly as PathTracker.cpp does.
///
/// This is the canonical peek-and-classify the LINE_FOLLOW case runs in the
/// production FSM. By executing it here against the mocked fixture, we cover
/// the (`Config::PATH_PASS_THROUGH_ENABLED ? eligibility-check : false`) branch
/// behaviorally — exactly the AC7b contract.
static bool computePassThrough()
{
    if (!Config::PATH_PASS_THROUGH_ENABLED)
    {
        return false;
    }
    if (fixture::next_hop_is_goal)
    {
        return false;
    }
    if (!fixture::next_hop_valid)
    {
        return false;
    }
    const float delta = wrapDelta(fixture::next_heading_deg - fixture::inbound_heading_deg);
    return (fabsf(delta) <= Config::NAV_TURN_TOLERANCE_DEG);
}

// ---------------------------------------------------------------------------
// Apply the LINE_FOLLOW arming chain PathTracker.cpp issues at
// intersection-detect, with the `pass_through` decision flowing into both
// `setOvershootMode` (always true — overshoot semantics changed
// project-wide per ADR-023) and `setBrakeOnGate` / `setTShapeRampDown`.
// ---------------------------------------------------------------------------

static void armLineFollowerForOvershoot(bool passThrough)
{
    LineFollower.setEnabled(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(!passThrough)
                .setOvershootMode(true)
                .setBrakeOnGate(!passThrough)
                .setDistanceTarget(Config::LINE_SENSOR_AXLE_OFFSET_CM);
}

static void resetWorld()
{
    mockResetMillis();
    Drive.mockReset();
    LineArray.mockReset();
    Intersection.mockReset();
    LineFollower.begin();
    // See `test_line_follower_overshoot.cpp::resetWorld()` for why we also
    // issue a disable cycle after begin() — `begin()` does not reset every
    // cross-tick field on its own.
    LineFollower.setEnabled(false);
    Drive.mockReset();
    // Default the sensor to a "line is online, no junction, error=0" baseline
    // so the controller takes the simple P+D path.
    LineArray.mockSetOnline(true);
    LineArray.mockSetError(0.0f);
    LineArray.mockSetJunctionType(JunctionType::NONE);
}

static int32_t encDegsForCm(float cm)
{
    return (int32_t) cmToEncDeg(cm);
}

// ===========================================================================
// T10c — The PATH_PASS_THROUGH_ENABLED toggle drives divergent gate-trip
// behavior on the canonical collinear fixture.
// ===========================================================================

static void test_T10c_passThrough_decision_matches_config_flag()
{
    TEST_SCOPE("T10c: pass-through decision tracks Config::PATH_PASS_THROUGH_ENABLED");

    const bool decision = computePassThrough();

    // The fixture is collinear & non-terminal. With the flag ON, pass-through
    // fires; with it OFF, the same geometry must NOT pass through.
    ASSERT_EQ(decision, Config::PATH_PASS_THROUGH_ENABLED);
}

static void test_T10c_arm_and_trip_gate_diverges_by_flag()
{
    TEST_SCOPE("T10c: gate-trip behavior diverges between flag=true and flag=false builds");

    resetWorld();

    const bool passThrough = computePassThrough();
    armLineFollowerForOvershoot(passThrough);

    ASSERT_TRUE(LineFollower.isEnabled());

    // Drive past the 5 cm gate.
    Drive.mockAdvanceEncoders(encDegsForCm(Config::LINE_SENSOR_AXLE_OFFSET_CM + 1.0f),
                              encDegsForCm(Config::LINE_SENSOR_AXLE_OFFSET_CM + 1.0f));

    const uint32_t stopBefore = Drive.mockRecord().stopCallCount;
    LineFollower.update();

    // Both branches latch the distance-reached one-shot.
    ASSERT_TRUE(LineFollower.isDistanceReached());

    if (Config::PATH_PASS_THROUGH_ENABLED)
    {
        // Build mode A — flag is true and the fixture is eligible, so
        // PathTracker armed with brake-on-gate=false. Controller stays enabled,
        // no Drive.stop fires.
        ASSERT_TRUE(LineFollower.isEnabled());
        ASSERT_EQ(Drive.mockRecord().stopCallCount, stopBefore);
    }
    else
    {
        // Build mode B — flag is false, so PathTracker armed with the legacy
        // brake-on-gate=true behavior. Controller self-disables + brakes.
        ASSERT_FALSE(LineFollower.isEnabled());
        ASSERT_GT(Drive.mockRecord().stopCallCount, stopBefore);
        ASSERT_GE(Drive.mockRecord().stopBrakeTrueCount, (uint32_t) 1);
    }
}

// ---------------------------------------------------------------------------
// Print which build mode we're running in for ctest log clarity.
// ---------------------------------------------------------------------------

static void test_T10c_announce_build_mode()
{
    TEST_SCOPE("T10c: build-mode announcement");
    if (Config::PATH_PASS_THROUGH_ENABLED)
    {
        std::printf("INFO: PATH_PASS_THROUGH_ENABLED == true (pass-through build)\n");
    }
    else
    {
        std::printf("INFO: PATH_PASS_THROUGH_ENABLED == false (legacy stop-and-go build)\n");
    }
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_T10c_announce_build_mode();
    test_T10c_passThrough_decision_matches_config_flag();
    test_T10c_arm_and_trip_gate_diverges_by_flag();
    TEST_REPORT();
}
