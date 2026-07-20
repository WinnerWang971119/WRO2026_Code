/**
 * @file test_line_follower_offset.cpp
 * @brief Host-compile tests for the LineFollower lateral-offset setpoint bias
 *        (issue #156 — TC1..TC4 / AC1, AC2, AC3-composition, AC4).
 *
 * Built against the mock Arduino / Drive / WROLineArray / Intersection surface under
 * `WRO2026/test/mocks/`. The REAL `WRO2026/src/drive/LineFollower.{cpp,h}` and
 * `WRO2026/src/util/WROMath.h` are compiled UNCHANGED — the tests exercise the actual
 * production P+D controller, so the emitted differential PWM is the observable.
 *
 * The offset is a signed centroid-index bias applied to the PROPORTIONAL term only:
 *
 *     corr = LINE_KP*(err - offset) + LINE_KD*(err - _lastErr)      (_lastErr stays RAW)
 *
 * so `getRawError()` telemetry is intact and a mid-follow offset change injects a bounded
 * P-step (≈ LINE_KP*offset) with NO derivative kick.
 *
 * Every expectation is derived from `Config::LINE_FOLLOW_OFFSET_ERROR` / `Config::LINE_KP` /
 * `Config::LINE_KD` (never a literal PWM) so a bench retune of any of those constants does not
 * red the test for the wrong reason (gotcha-drive-constant-retune-fanout).
 *
 * Test cases:
 *   - TC1: a `getError()` sweep with the LEFT (+OFFSET), RIGHT (-OFFSET) and MIDDLE (0) bias.
 *          MIDDLE reproduces the pre-#156 baseline byte-for-byte; RIGHT mirrors LEFT.
 *   - TC2: a mid-follow MIDDLE->LEFT mode change — P steps by LINE_KP*offset, no D-kick,
 *          `getRawError()` still returns the raw centroid.
 *   - TC3: the reverse-hop COMPOSITION `sign * lineFollowModeToSetpoint(mode)` PathTracker
 *          passes — a rear-board LEFT hop negates the forward setpoint. Pins the composition,
 *          not physical travel-frame polarity (that is bench-confirmed).
 *   - TC4: the offset lifecycle — cold enable follows centred (offset 0); a re-arm on an
 *          already-enabled follower preserves the bias; `setEnabled(false)` RESETS it to 0 so a
 *          later `followDistance()`/`followUntilIntersection()` never inherits a stale bias.
 */

#include "TestAssert.h"

#include "Arduino.h"
#include "Drive.h"
#include "WROLineArray.h"
#include "Intersection.h"

// Production code under test:
#include "src/drive/LineFollower.h"
#include "Config.h"
#include "src/util/WROMath.h"    // presetToPwm / deadbandSnap / lineFollowModeToSetpoint

// ---------------------------------------------------------------------------
// Shared helpers.
// ---------------------------------------------------------------------------

/// @brief Reset every mock and re-init the LineFollower for a fresh scenario.
///
/// Mirrors `test_line_follower_overshoot.cpp::resetWorld()`: `begin()` does not scrub the
/// cross-tick fields (they are boot-once on hardware), so a `setEnabled(false)` after it drives
/// the disable branch of `_applyEnableBuilder`, which resets `_lastErr`, `_pidFrozen`, AND
/// `_lineFollowOffset` (issue #156) to their documented defaults. The trailing `Drive.mockReset()`
/// clears the stop/setSpeed records the disable path issued so each scenario starts from zero.
static void resetWorld()
{
    mockResetMillis();
    Drive.mockReset();
    LineArray.mockReset();
    Intersection.mockReset();
    LineFollower.begin();
    LineFollower.setEnabled(false);
    Drive.mockReset();
}

/// @brief Float near-equality without pulling in the `abs`/`fabs` macro clash from the mock
///        `Arduino.h` (which #defines `abs`).
static bool nearf(float a, float b, float eps = 1.0e-4f)
{
    const float d = a - b;
    return d < eps && d > -eps;
}

/// @brief Replicate the production steady-state (dErr == 0) differential mix EXACTLY, so the
///        expectation tracks the real `update()` math (same clamp, same int16 truncation, same
///        deadband snap, same NORMAL cruise) for ANY `Config::LINE_*` retune.
///
/// @param offset  Signed lateral-offset bias (centroid units) staged via `setLineFollowOffset`.
/// @param err     Held `getError()` centroid.
/// @param expL    OUT — expected left PWM commanded to `Drive.setSpeed`.
/// @param expR    OUT — expected right PWM commanded to `Drive.setSpeed`.
static void expectedSteadyLR(float offset, float err, int16_t& expL, int16_t& expR)
{
    const int16_t cruise = presetToPwm(Config::SpeedPreset::NORMAL);
    const float   dErr   = 0.0f;    // steady state: error held across the two settling ticks.
    float corr = (Config::LINE_KP * (err - offset)) + (Config::LINE_KD * dErr);
    if (corr > Config::LINE_MAX_CORR)
    {
        corr = Config::LINE_MAX_CORR;
    }
    else if (corr < -Config::LINE_MAX_CORR)
    {
        corr = -Config::LINE_MAX_CORR;
    }
    expL = deadbandSnap((int16_t) (cruise + corr), Config::PWM_MIN_MAGNITUDE);
    expR = deadbandSnap((int16_t) (cruise - corr), Config::PWM_MIN_MAGNITUDE);
}

/// @brief Arm a fresh centred/biased follow at NORMAL cruise (ramps off), hold @p err, and tick
///        twice so the derivative settles (`dErr == 0` on the second tick), then read back the
///        commanded PWM. Two ticks are required: tick 1 seeds `_lastErr = err` (with a one-tick
///        `dErr == err`), tick 2 differences `err - err == 0`.
static void driveSteady(float offset, float err, int16_t& outL, int16_t& outR)
{
    resetWorld();
    LineArray.mockSetOnline(true);
    LineArray.mockSetJunctionType(JunctionType::NONE);
    LineArray.mockSetError(err);

    LineFollower.setEnabled(true)
                .setPreset(Config::SpeedPreset::NORMAL)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setLineFollowOffset(offset);
    LineFollower.update();    // tick 1 — seeds _lastErr = err
    LineFollower.update();    // tick 2 — steady (dErr == 0)

    outL = Drive.mockRecord().lastSetSpeedL;
    outR = Drive.mockRecord().lastSetSpeedR;
}

// ===========================================================================
// TC1 — LEFT / RIGHT / MIDDLE bias over a getError() sweep.
// ===========================================================================

static void test_tc1_offset_biases_p_setpoint_over_error_sweep()
{
    TEST_SCOPE("TC1: LEFT(+)/RIGHT(-)/MIDDLE(0) bias shifts the P setpoint over a getError sweep");

    // Chosen so cruise +/- corr stays inside (PWM_MIN_MAGNITUDE, PWM_MAX) at every sample: no
    // clamp, no deadband snap, no saturation — the raw offset behaviour is what is asserted.
    const float errs[] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

    for (float err : errs)
    {
        // MIDDLE (offset 0) — byte-identical to the pre-#156 centred follow. The baseline is the
        // offset-free formula corr = LINE_KP*err (+ LINE_KD*0 at steady state); with offset 0 the
        // production `err - offset` collapses to `err`, so a MIDDLE follow steers exactly as before.
        int16_t midL, midR;
        driveSteady(0.0f, err, midL, midR);
        int16_t expMidL, expMidR;
        expectedSteadyLR(0.0f, err, expMidL, expMidR);
        ASSERT_EQ(midL, expMidL);
        ASSERT_EQ(midR, expMidR);

        // LEFT (offset = +LINE_FOLLOW_OFFSET_ERROR): the line settles right-of-centre under the
        // sensor (chassis rides left of the line).
        int16_t leftL, leftR;
        driveSteady(+Config::LINE_FOLLOW_OFFSET_ERROR, err, leftL, leftR);
        int16_t expLeftL, expLeftR;
        expectedSteadyLR(+Config::LINE_FOLLOW_OFFSET_ERROR, err, expLeftL, expLeftR);
        ASSERT_EQ(leftL, expLeftL);
        ASSERT_EQ(leftR, expLeftR);

        // RIGHT (offset = -LINE_FOLLOW_OFFSET_ERROR) mirrors LEFT about the MIDDLE baseline.
        int16_t rightL, rightR;
        driveSteady(-Config::LINE_FOLLOW_OFFSET_ERROR, err, rightL, rightR);
        int16_t expRightL, expRightR;
        expectedSteadyLR(-Config::LINE_FOLLOW_OFFSET_ERROR, err, expRightL, expRightR);
        ASSERT_EQ(rightL, expRightL);
        ASSERT_EQ(rightR, expRightR);
    }
}

// ===========================================================================
// TC2 — mid-follow MIDDLE->LEFT: bounded P-step, no derivative kick, raw telemetry intact.
// ===========================================================================

static void test_tc2_mid_follow_mode_change_bounded_p_step_no_d_kick()
{
    TEST_SCOPE("TC2: mid-follow MIDDLE->LEFT — P steps by LINE_KP*offset, no D-kick, raw getRawError");

    resetWorld();
    const float err = 1.0f;
    LineArray.mockSetOnline(true);
    LineArray.mockSetJunctionType(JunctionType::NONE);
    LineArray.mockSetError(err);

    // Follow centred (MIDDLE == offset 0) and settle to steady state (dErr -> 0).
    LineFollower.setEnabled(true)
                .setPreset(Config::SpeedPreset::NORMAL)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false);
    LineFollower.update();
    LineFollower.update();

    int16_t midL = Drive.mockRecord().lastSetSpeedL;
    int16_t midR = Drive.mockRecord().lastSetSpeedR;
    int16_t expMidL, expMidR;
    expectedSteadyLR(0.0f, err, expMidL, expMidR);
    ASSERT_EQ(midL, expMidL);
    ASSERT_EQ(midR, expMidR);
    // The proportional bias does NOT touch the derivative history — the telemetry stays raw.
    ASSERT_TRUE(nearf(LineFollower.getRawError(), err));

    // Mid-follow mode change MIDDLE->LEFT via a re-arm on the already-enabled follower. `_lastErr`
    // is preserved (== err) across the re-arm, so this tick's dErr is 0: the P setpoint steps by
    // exactly LINE_KP*offset with NO one-tick LINE_KD*offset derivative kick.
    LineFollower.setEnabled(true).setLineFollowOffset(+Config::LINE_FOLLOW_OFFSET_ERROR);
    LineFollower.update();    // mode-change tick — dErr == 0

    int16_t changeL = Drive.mockRecord().lastSetSpeedL;
    int16_t changeR = Drive.mockRecord().lastSetSpeedR;
    int16_t expLeftL, expLeftR;
    expectedSteadyLR(+Config::LINE_FOLLOW_OFFSET_ERROR, err, expLeftL, expLeftR);
    ASSERT_EQ(changeL, expLeftL);
    ASSERT_EQ(changeR, expLeftR);
    // `getRawError()` is still the RAW centroid — the offset never leaked into `_lastErr`.
    ASSERT_TRUE(nearf(LineFollower.getRawError(), err));

    // Tick once more with the error still held to reach the fully-settled LEFT output. The
    // mode-change tick above ALREADY matched it — a fold of the offset into the derivative would
    // have made the mode-change tick differ from the settled value by LINE_KD*offset.
    LineFollower.update();
    ASSERT_EQ(changeL, Drive.mockRecord().lastSetSpeedL);
    ASSERT_EQ(changeR, Drive.mockRecord().lastSetSpeedR);
}

// ===========================================================================
// TC3 — reverse-hop composition negates the forward setpoint (pins the composition only).
// ===========================================================================

static void test_tc3_reverse_hop_composition_negates_offset()
{
    TEST_SCOPE("TC3: LEFT + reverse — PathTracker composition passes the NEGATED offset");

    // The composition PathTracker performs at LINE_FOLLOW_INTERSECTION / PROJECTION_FOLLOW is
    // `.setLineFollowOffset(sign * lineFollowModeToSetpoint(mode))`, sign = decision.reverse ? -1 : +1.
    // Forward LEFT resolves to +LINE_FOLLOW_OFFSET_ERROR; a rear-board (reverse) hop negates it so
    // the bias stays left-of-TRAVEL. This pins the composition only — physical travel-frame
    // polarity is bench-confirmed (the negate is a one-line convention, ADR-069).
    const float forwardLeft = lineFollowModeToSetpoint(Routes::LineFollowMode::LEFT);
    ASSERT_TRUE(nearf(forwardLeft, Config::LINE_FOLLOW_OFFSET_ERROR));

    const float reverseSign = -1.0f;    // decision.reverse ? -1.0f : 1.0f
    const float composed    = reverseSign * forwardLeft;
    ASSERT_TRUE(nearf(composed, -Config::LINE_FOLLOW_OFFSET_ERROR));

    // Feed the composed (negated) setpoint into the REAL follower (forward follow, to isolate the
    // offset sign from the reverse cruise-negate) and confirm the steering honours exactly it.
    const float err = 1.0f;
    int16_t gotL, gotR;
    driveSteady(composed, err, gotL, gotR);
    int16_t expL, expR;
    expectedSteadyLR(composed, err, expL, expR);
    ASSERT_EQ(gotL, expL);
    ASSERT_EQ(gotR, expR);

    // A reverse LEFT composes to the same magnitude as a forward RIGHT (-OFFSET) — the mirror.
    int16_t rightL, rightR;
    expectedSteadyLR(-Config::LINE_FOLLOW_OFFSET_ERROR, err, rightL, rightR);
    ASSERT_EQ(gotL, rightL);
    ASSERT_EQ(gotR, rightR);
}

// ===========================================================================
// TC4 — offset lifecycle: cold enable -> 0, re-arm preserves, disable resets -> 0.
// ===========================================================================

static void test_tc4_offset_lifecycle_reset_on_disable()
{
    TEST_SCOPE("TC4: lifecycle — cold enable->0, re-arm preserves, disable resets->0");

    const float err = 1.0f;
    int16_t expCentredL, expCentredR;
    expectedSteadyLR(0.0f, err, expCentredL, expCentredR);
    int16_t expLeftL, expLeftR;
    expectedSteadyLR(+Config::LINE_FOLLOW_OFFSET_ERROR, err, expLeftL, expLeftR);

    resetWorld();
    LineArray.mockSetOnline(true);
    LineArray.mockSetJunctionType(JunctionType::NONE);
    LineArray.mockSetError(err);

    // (1) Cold enable with NO override — follows centred (offset 0).
    LineFollower.setEnabled(true)
                .setPreset(Config::SpeedPreset::NORMAL)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false);
    LineFollower.update();
    LineFollower.update();
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expCentredL);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expCentredR);

    // (2) Re-arm with the LEFT bias — the follower now steers biased...
    LineFollower.setEnabled(true).setLineFollowOffset(+Config::LINE_FOLLOW_OFFSET_ERROR);
    LineFollower.update();    // dErr == 0 (re-arm preserves _lastErr == err)
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expLeftL);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expLeftR);

    // ...and a further re-arm WITHOUT an override preserves the bias (mirrors setPreset/setReverse).
    LineFollower.setEnabled(true);
    LineFollower.update();
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expLeftL);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expLeftR);

    // (3) Disable RESETS the bias to 0 (unlike setPreset/setReverse, which persist). A fresh
    // enable with no override then follows CENTRED — not the stale LEFT bias.
    LineFollower.setEnabled(false);
    LineFollower.setEnabled(true)
                .setPreset(Config::SpeedPreset::NORMAL)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false);
    LineFollower.update();
    LineFollower.update();
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedL, expCentredL);
    ASSERT_EQ(Drive.mockRecord().lastSetSpeedR, expCentredR);
}

// ===========================================================================
// TC5 — setLineFollowMode(mode) sugar == setLineFollowOffset(lineFollowModeToSetpoint(mode)).
// ===========================================================================

/// @brief Same as driveSteady, but arms the follow through the ENUM sugar `setLineFollowMode(mode)`
///        instead of the raw `setLineFollowOffset(float)` knob, so the two entry points can be
///        compared for byte-identical steering.
static void driveSteadyMode(Routes::LineFollowMode mode, float err, int16_t& outL, int16_t& outR)
{
    resetWorld();
    LineArray.mockSetOnline(true);
    LineArray.mockSetJunctionType(JunctionType::NONE);
    LineArray.mockSetError(err);

    LineFollower.setEnabled(true)
                .setPreset(Config::SpeedPreset::NORMAL)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setLineFollowMode(mode);
    LineFollower.update();    // tick 1 — seeds _lastErr = err
    LineFollower.update();    // tick 2 — steady (dErr == 0)

    outL = Drive.mockRecord().lastSetSpeedL;
    outR = Drive.mockRecord().lastSetSpeedR;
}

static void test_tc5_mode_sugar_matches_offset_knob()
{
    TEST_SCOPE("TC5: setLineFollowMode(MIDDLE/LEFT/RIGHT/DEFAULT) steers identically to the mapped offset");

    const Routes::LineFollowMode modes[] = {
        Routes::LineFollowMode::DEFAULT,
        Routes::LineFollowMode::MIDDLE,
        Routes::LineFollowMode::LEFT,
        Routes::LineFollowMode::RIGHT,
    };
    const float errs[] = {-1.0f, 0.0f, 1.0f};

    for (Routes::LineFollowMode mode : modes)
    {
        // The sugar delegates through the SAME free function PathTracker uses; DEFAULT and MIDDLE
        // both resolve to 0 (centred). The enum path must produce byte-identical steering to arming
        // the float knob with that resolved offset.
        const float offset = lineFollowModeToSetpoint(mode);

        for (float err : errs)
        {
            int16_t modeL, modeR;
            driveSteadyMode(mode, err, modeL, modeR);

            int16_t offL, offR;
            driveSteady(offset, err, offL, offR);
            ASSERT_EQ(modeL, offL);
            ASSERT_EQ(modeR, offR);

            // ...and both match the closed-form expectation for that offset.
            int16_t expL, expR;
            expectedSteadyLR(offset, err, expL, expR);
            ASSERT_EQ(modeL, expL);
            ASSERT_EQ(modeR, expR);
        }
    }

    // DEFAULT and MIDDLE are both centred (offset 0) — pin that the sugar treats them identically.
    ASSERT_TRUE(nearf(lineFollowModeToSetpoint(Routes::LineFollowMode::DEFAULT), 0.0f));
    ASSERT_TRUE(nearf(lineFollowModeToSetpoint(Routes::LineFollowMode::MIDDLE), 0.0f));
}

// ===========================================================================
// TC6 — reverse follow negates the P+D correction sign (ADR-078).
// ===========================================================================

/// @brief Reverse counterpart of `expectedSteadyLR`: cruise is negated (backward travel), the
///        gains are the REAR set (ADR-070), and the whole correction is negated because the
///        reverse-active rear board trails the axle (ADR-078). Mirrors the production order
///        exactly — compute P+D with the rear gains, negate, THEN clamp.
static void expectedSteadyLRReverse(float offset, float err, int16_t& expL, int16_t& expR)
{
    const int16_t cruise = (int16_t) -presetToPwm(Config::SpeedPreset::NORMAL);
    const float   dErr   = 0.0f;    // steady state: error held across the two settling ticks.
    float corr = (Config::LINE_KP_REAR * (err - offset)) + (Config::LINE_KD_REAR * dErr);
    corr = -corr;                   // reverse: rear board is behind the axle (ADR-078).
    if (corr > Config::LINE_MAX_CORR)
    {
        corr = Config::LINE_MAX_CORR;
    }
    else if (corr < -Config::LINE_MAX_CORR)
    {
        corr = -Config::LINE_MAX_CORR;
    }
    expL = deadbandSnap((int16_t) (cruise + corr), Config::PWM_MIN_MAGNITUDE);
    expR = deadbandSnap((int16_t) (cruise - corr), Config::PWM_MIN_MAGNITUDE);
}

/// @brief Reverse counterpart of `driveSteady`: arms the follow with `.setReverse(true)` so the
///        controller negates cruise, selects the REAR gains, and negates the correction.
static void driveSteadyReverse(float offset, float err, int16_t& outL, int16_t& outR)
{
    resetWorld();
    LineArray.mockSetOnline(true);
    LineArray.mockSetJunctionType(JunctionType::NONE);
    LineArray.mockSetError(err);

    LineFollower.setEnabled(true)
                .setPreset(Config::SpeedPreset::NORMAL)
                .setReverse(true)
                .setTShapeRampUp(false)
                .setTShapeRampDown(false)
                .setLineFollowOffset(offset);
    LineFollower.update();    // tick 1 — seeds _lastErr = err
    LineFollower.update();    // tick 2 — steady (dErr == 0)

    outL = Drive.mockRecord().lastSetSpeedL;
    outR = Drive.mockRecord().lastSetSpeedR;
}

static void test_tc6_reverse_negates_correction_sign()
{
    TEST_SCOPE("TC6: reverse follow negates the P+D correction (rear board trails the axle, ADR-078)");

    const float err = 1.5f;    // line right-of-centre; non-zero so corr != 0.

    // Reverse steering must match the reverse closed-form EXACTLY (negated corr, rear gains,
    // negated cruise) for a centred and a biased follow. Offsets derived from Config so a bench
    // retune never reds the test for the wrong reason.
    const float offsets[] = {
        0.0f, +Config::LINE_FOLLOW_OFFSET_ERROR, -Config::LINE_FOLLOW_OFFSET_ERROR};
    for (float offset : offsets)
    {
        int16_t gotL, gotR;
        driveSteadyReverse(offset, err, gotL, gotR);
        int16_t expL, expR;
        expectedSteadyLRReverse(offset, err, expL, expR);
        ASSERT_EQ(gotL, expL);
        ASSERT_EQ(gotR, expR);
    }

    // The whole point of ADR-078: for the SAME held centroid, forward and reverse carry
    // OPPOSITE-sign corrections. The correction is `(L - R)/2`; compare its sign (the cruise sign
    // differs too, so raw PWM cannot be compared directly). A front board with err>0 steers L>R;
    // the rear board — behind the axle — must steer L<R for the same err, else reverse follow is
    // positive feedback and diverges.
    int16_t fL, fR, rL, rR;
    driveSteady(0.0f, err, fL, fR);
    driveSteadyReverse(0.0f, err, rL, rR);
    ASSERT_TRUE((int16_t) (fL - fR) > 0);    // forward: corr = +KP*err     -> L > R
    ASSERT_TRUE((int16_t) (rL - rR) < 0);    // reverse: corr = -KP_REAR*err -> L < R
}

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    test_tc1_offset_biases_p_setpoint_over_error_sweep();
    test_tc2_mid_follow_mode_change_bounded_p_step_no_d_kick();
    test_tc3_reverse_hop_composition_negates_offset();
    test_tc4_offset_lifecycle_reset_on_disable();
    test_tc5_mode_sugar_matches_offset_knob();
    test_tc6_reverse_negates_correction_sign();
    TEST_REPORT();
}
