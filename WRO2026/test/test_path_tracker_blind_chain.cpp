/**
 * @file test_path_tracker_blind_chain.cpp
 * @brief Host-link tests for ADR-080 blind-run coalescing.
 *
 * @details
 * These tests drive the real PathTracker, Drive, and Localization state machines against the
 * shared MiniR4 HAL mock. The main fixture route is `final -> ChainA -> ChainB -> ChainC ->
 * ChainD` with four collinear BLIND hops of 25, 20, 20, and 8 cm. With coalescing enabled it
 * must be dispatched as one 73 cm braked SYNC T-shape. Localization stays at `final` while the
 * combined move is in flight, then stamps through the buffered nodes and lands at `ChainD`.
 *
 * The same source is compiled for the default, disabled, hop-cap, and distance-cap targets.
 * Target-wide definitions are required because PathTracker.cpp is a separate translation unit.
 */

#include "test_path_tracker_harness.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace
{
using State = PathTrackerClass::State;

constexpr float   kMainHopDistCm[] = {25.0f, 20.0f, 20.0f, 8.0f};
constexpr uint8_t kMainHopCount    = 4;
constexpr float   kMainTotalCm     = 73.0f;
int32_t           gProfileStartEnc = 0;

/** @brief Tick until the first continuous SYNC setpoint of a BLIND profile is emitted. */
void tickUntilSyncProfileEmits()
{
    const uint32_t before = MiniR4.DriveDC.moveSyncCallCount;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && MiniR4.DriveDC.moveSyncCallCount == before && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, before);
    ASSERT_FALSE(PathTracker.hasFault());
    gProfileStartEnc = MiniR4.M2.getDegrees();
}

/** @brief Tick until either supported profiled BLIND mode emits its first setpoint. */
void tickUntilProfileEmits()
{
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() != State::BLIND_FORWARD && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_EQ((int) PathTracker.state(), (int) State::BLIND_FORWARD);
    ASSERT_FALSE(PathTracker.hasFault());
    // The first tick after PathTracker dispatch lets Drive dequeue/arm the profile; the second
    // runs the profile update and emits the first lower-MCU setpoint.
    cooperativeTick();
    cooperativeTick();
    gProfileStartEnc = MiniR4.M2.getDegrees();
}

/** @brief Advance the profiled move's encoder by an exact additional distance and tick once. */
void advanceProfileCm(float cm)
{
    const int32_t delta = (int32_t) cmToEncDeg(cm);
    MiniR4.M2.mockAdvanceDegrees(delta);
    MiniR4.M3.mockAdvanceDegrees(delta);
    MiniR4.DriveDC.mockSetPrevTaskDone(true);
    cooperativeTick();
}

/** @brief Advance the profile to its exact absolute encoder target and tick once. */
void finishProfileAt(float totalCm)
{
    const int32_t target = gProfileStartEnc + (int32_t) cmToEncDeg(totalCm);
    MiniR4.M2.mockSetDegrees(target);
    MiniR4.M3.mockSetDegrees(target);
    MiniR4.DriveDC.mockSetPrevTaskDone(true);
    cooperativeTick();
}

/** @brief Seat at `final`, queue the main goal, and wait for the first SYNC profile emission. */
void dispatchMainRoute()
{
    resetWorld();
    Serial.clearCapture();
    seatAt("final", 0);
    PathTracker.goTo(nodeByName("ChainD")).async();
    tickUntilProfileEmits();
}

/** @brief Assert the frozen route and geometry used by every coalescing case. */
void testRoutingGuards()
{
    TEST_SCOPE("routing: main blind run and real-turn split remain pinned");

    const uint8_t finalNode = nodeByName("final");
    const uint8_t a         = nodeByName("ChainA");
    const uint8_t b         = nodeByName("ChainB");
    const uint8_t c         = nodeByName("ChainC");
    const uint8_t d         = nodeByName("ChainD");

    ASSERT_TRUE(finalNode != FieldMap::NODE_NONE && a != FieldMap::NODE_NONE);
    ASSERT_TRUE(b != FieldMap::NODE_NONE && c != FieldMap::NODE_NONE);
    ASSERT_TRUE(d != FieldMap::NODE_NONE);
    ASSERT_EQ(routedFirstHop(finalNode, d), a);
    ASSERT_EQ(routedFirstHop(a, d), b);
    ASSERT_EQ(routedFirstHop(b, d), c);
    ASSERT_EQ(routedFirstHop(c, d), d);

    float total = 0.0f;
    const uint8_t nodes[] = {finalNode, a, b, c, d};
    for (uint8_t k = 0; k < kMainHopCount; ++k)
    {
        const Routes::EdgeInfo& edge = FieldMap::getEdge(nodes[k], nodes[k + 1]);
        ASSERT_TRUE(edge.type == Routes::EdgeType::BLIND);
        ASSERT_TRUE(std::fabs(edge.distance_cm - kMainHopDistCm[k]) < 1.0e-4f);
        total += edge.distance_cm;
    }
    ASSERT_TRUE(std::fabs(total - kMainTotalCm) < 1.0e-4f);

    const uint8_t bend = nodeByName("position1");
    const uint8_t wall = nodeByName("Wall1");
    ASSERT_EQ(routedFirstHop(finalNode, wall), bend);
    ASSERT_EQ(routedFirstHop(bend, wall), wall);
    const float h0 = FieldMap::getEdge(finalNode, bend).heading_deg;
    const float h1 = FieldMap::getEdge(bend, wall).heading_deg;
    ASSERT_GT(std::fabs(h1 - h0), Config::NAV_TURN_TOLERANCE_DEG);
}

/** @brief AC2-AC4: one combined move, exact summed target, one brake, final-node landing. */
void testEnabledRunCoalescesToOneMove()
{
    TEST_SCOPE("TC1-TC3: four-hop blind run is one 73 cm braked SYNC profile");
    ASSERT_TRUE(Config::PATH_BLIND_CHAIN_ENABLED);

    const uint8_t finalNode = nodeByName("final");
    const uint8_t d         = nodeByName("ChainD");
    dispatchMainRoute();

    // Correct run mode and cold-start seed. The fixture's resolved blind mode is SYNC_TSHAPE,
    // so the real Drive emits MoveSync, never a degree-bounded one-shot or the GYRO channel.
    ASSERT_EQ(MiniR4.DriveDC.moveSyncCallCount, 1u);
    ASSERT_EQ(MiniR4.DriveDC.moveSyncDegsCallCount, 0u);
    ASSERT_EQ(MiniR4.DriveDC.moveGyroCallCount, 0u);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncR, Config::TSHAPE_START_SPEED);
    ASSERT_EQ(Localization.getCurrentNode(), finalNode);

    // Crossing three old per-hop boundaries does not complete or redispatch anything. This is
    // the real-setpoint proof that the profile target is the run sum, not the first edge.
    MiniR4.M2.mockSetDegrees((int32_t) cmToEncDeg(65.0f));
    MiniR4.M3.mockSetDegrees((int32_t) cmToEncDeg(65.0f));
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), finalNode);
    ASSERT_FALSE(Drive.isIdle());
    ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, 0u);

    // The exact 73 cm target completes the one profile. BLIND_FORWARD then advances through
    // ChainA/B/C in order internally and leaves the externally visible belief at ChainD.
    finishProfileAt(kMainTotalCm);
    ASSERT_EQ(Localization.getCurrentNode(), d);
    ASSERT_TRUE(Drive.isIdle());
    ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, 1u);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    ASSERT_FALSE(PathTracker.hasFault());

    const std::string& log = Serial.captured();
    ASSERT_TRUE(log.find("ADR-080: blind run final -> ChainD: hops=4 dist=73")
                != std::string::npos);
}

/** @brief AC6: disabling coalescing restores four independent braked node arrivals. */
void testDisabledRunStopsAtEveryNode()
{
    TEST_SCOPE("TC5: coalescing disabled restores one braked move per node");
    ASSERT_FALSE(Config::PATH_BLIND_CHAIN_ENABLED);

    const uint8_t nodes[] = {
        nodeByName("ChainA"), nodeByName("ChainB"), nodeByName("ChainC"), nodeByName("ChainD")};
    dispatchMainRoute();

    for (uint8_t k = 0; k < kMainHopCount; ++k)
    {
        finishProfileAt(kMainHopDistCm[k]);
        ASSERT_EQ(Localization.getCurrentNode(), nodes[k]);
        ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
        ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, (uint32_t) k + 1u);
        ASSERT_FALSE(PathTracker.hasFault());
        if (k + 1 < kMainHopCount)
        {
            tickUntilSyncProfileEmits();
        }
    }
}

/** @brief Cap targets: the first two hops coalesce, then the remaining two re-form once. */
void testCapTruncatesAndReforms()
{
    TEST_SCOPE("TC cap: capped run dispatches 45 cm then re-forms for 28 cm");
    ASSERT_TRUE(Config::PATH_BLIND_CHAIN_ENABLED);

    const uint8_t finalNode = nodeByName("final");
    const uint8_t b         = nodeByName("ChainB");
    const uint8_t d         = nodeByName("ChainD");
    dispatchMainRoute();

    // A 25 cm first-hop target would have completed here in the retired per-hop design.
    advanceProfileCm(25.0f);
    ASSERT_EQ(Localization.getCurrentNode(), finalNode);
    ASSERT_FALSE(Drive.isIdle());

    finishProfileAt(45.0f);
    ASSERT_EQ(Localization.getCurrentNode(), b);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, 1u);

    tickUntilSyncProfileEmits();
    finishProfileAt(28.0f);
    ASSERT_EQ(Localization.getCurrentNode(), d);
    ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, 2u);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_TRUE(Serial.captured().find("chain capped") != std::string::npos);
}

/** @brief AC5: a real bend ends the run before the turn and forces a braked arrival. */
void testRealTurnSplitsRun()
{
    TEST_SCOPE("TC4: a turn larger than tolerance is not coalesced across");

    const uint8_t finalNode = nodeByName("final");
    const uint8_t bend      = nodeByName("position1");
    const uint8_t wall      = nodeByName("Wall1");
    const Routes::EdgeInfo& first = FieldMap::getEdge(finalNode, bend);

    resetWorld();
    seatAt("final", 0);
    // Bind the live yaw to the first edge so this case isolates the real bend at
    // position1 instead of exercising the unrelated initial-turn handshake.
    mmL.mockYawDeg = hwYaw((int16_t) lroundf(first.heading_deg));
    PathTracker.goTo(wall).async();
    tickUntilProfileEmits();

    finishProfileAt(first.distance_cm);
    ASSERT_EQ(Localization.getCurrentNode(), bend);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, 1u);

    // Planning the next edge dispatches a genuine turn before a second independent move.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && PathTracker.state() != State::TURNING && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_EQ(PathTracker.state() == State::TURNING, true);
    ASSERT_EQ(Localization.getCurrentNode(), bend);
    ASSERT_FALSE(PathTracker.hasFault());
}

/** @brief Run-wide corridor rejects cumulative bends hidden by pairwise angle tolerance. */
void testCumulativeBendSplitsRun()
{
    TEST_SCOPE("TC corridor: pairwise 9 degree bends cannot coalesce off the start ray");
    const uint8_t a = nodeByName("CumA");
    const uint8_t b = nodeByName("CumB");
    const uint8_t c = nodeByName("CumC");
    const uint8_t d = nodeByName("CumD");
    ASSERT_EQ(routedFirstHop(a, d), b);
    ASSERT_EQ(routedFirstHop(b, d), c);
    ASSERT_EQ(routedFirstHop(c, d), d);

    resetWorld();
    Serial.clearCapture();
    seatAt("CumA", 0);
    PathTracker.goTo(d).setRouteDirection(Direction::FRONT).async();
    tickUntilProfileEmits();
    ASSERT_EQ((int) PathTracker.state(), (int) State::BLIND_FORWARD);
    ASSERT_TRUE(Serial.captured().find("ADR-080: blind run CumA") == std::string::npos);
    ASSERT_EQ(Localization.getCurrentNode(), a);
    ASSERT_FALSE(Drive.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}
}    // namespace

int main()
{
    std::printf("INFO: PathTracker ADR-080 blind-run coalescing over routes_fixture\n");
    testRoutingGuards();

#if defined(BLIND_CHAIN_TEST_HOP_CAP) || defined(BLIND_CHAIN_TEST_DISTANCE_CAP)
    testCapTruncatesAndReforms();
#elif defined(BLIND_CHAIN_TEST_DISABLED)
    testDisabledRunStopsAtEveryNode();
#else
    testEnabledRunCoalescesToOneMove();
    testRealTurnSplitsRun();
    testCumulativeBendSplitsRun();
#endif

    TEST_REPORT();
}
