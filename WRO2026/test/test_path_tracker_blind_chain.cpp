/**
 * @file test_path_tracker_blind_chain.cpp
 * @brief Host-link test for the ADR-059 blind-chain FSM, driving the REAL
 *        `PathTracker` + `Drive` + `Localization` FSMs through the shared
 *        cooperative-tick harness (2026-07-10 collinear-run plan, T10;
 *        TC4-TC8; AC3/AC7/AC9 verification).
 *
 * @details
 * The blind-chain dispatch reduces to a small set of OBSERVABLE lower-MCU
 * facts, all read off the REAL `Drive.cpp` through the shared `MiniR4.DriveDC`
 * mock (`hal_mocks/MatrixMiniR4.h`) — there is NO public accessor on
 * `PathTrackerClass` for the private `_chain` run buffer, and this file does
 * NOT touch `PathTracker.h`/`.cpp` to add one:
 *
 *   - **Start PWM.** `Drive.cpp`'s `_updateProfile()` calls `_profileSpeedAt(0)`
 *     on the FIRST tick after a profiled move dispatches (`traveled == 0 <
 *     r.accelCm`, `TShapeProfilePolicy.h::tShapeSpeedAt` returns the start speed
 *     UNCHANGED). `SYNC_TSHAPE` (this fixture's pipeline default) emits it via
 *     `MiniR4.DriveDC.MoveSync(deadbandSnap(mag), deadbandSnap(mag))`; every
 *     chain-resolved start/end PWM lives in `[PWM_MIN_MAGNITUDE, |cruise|]` =
 *     `[50, 85]`, so `deadbandSnap` never perturbs it (verified against
 *     `WROMath.h::deadbandSnap`). One `cooperativeTick()` right after dispatch
 *     therefore stamps `MiniR4.DriveDC.lastMoveSyncL/R == startPwm` EXACTLY.
 *   - **Cruise-hold vs brake at a hop boundary (ADR-059, Drive finding #13 follow-up).** A
 *     non-braking chained mid-hop (`setBrake(false)`, endSpeed != 0) that completes while the
 *     FIFO is still empty NO LONGER calls `Move(0,0)+brake`: `_updateProfile` LEAVES the cruise
 *     setpoint latched and reports idle so the next chained hop overwrites it seamlessly (a
 *     bounded `Config::CHAIN_HOLD_TIMEOUT_MS` fail-safe issues the deferred stop only if no
 *     follow-up arrives -- the mock clock does not advance across a boundary here, so it never
 *     trips). So the precise TC4 observable for "no inter-hop STOP" is that `brakeCallCount` is
 *     UNCHANGED across every chained mid-hop boundary (no boundary brake at all), and a brake
 *     fires (`lastBrake == true`) ONLY at the run's braked FINAL hop. TC5 (chaining disabled)
 *     brakes for real at every node.
 *   - **Exact end PWM.** `tShapeSpeedAt(traveledCm, totalCm, ramps, start, end)`
 *     is a PURE, already-golden-tested (`test_pivot_trim_policy`-sibling
 *     `chain_speed_freshness` / `tshape_profile_freshness`) production policy —
 *     this file calls it DIRECTLY (the same "use the frozen resolver as an
 *     oracle" pattern as `resolveStopAtLine`/`resolvePivotTrim` in
 *     `test_path_tracker_stop_at_line.cpp`) to compute the EXACT PWM the real
 *     FSM must emit at a chosen `traveledCm`, then drives `MiniR4.M2`/`M3` to
 *     that exact encoder position and asserts equality — not re-deriving FSM
 *     logic, just reading its own frozen math at a controlled point.
 *   - **Chain-plan oracle.** `ChainSpeedPolicy.h::resolveChainSpeeds` (T4,
 *     already golden-pinned by `chain_speed_freshness`) is called directly to
 *     PREDICT each hop's `(startPwm, endPwm, brake)` for a given
 *     `(hopDistCm[], cruisePwm, entryPwm)` — the same oracle pattern.
 *
 * THE FIXTURES (`fixtures/field_fixture.py`, regenerated into
 * `fixtures/routes_fixture.h/.cpp`):
 *   - **TC4/TC8/TC9 (main chain):** `final -> ChainA -> ChainB -> ChainC ->
 *     ChainD`, 4 collinear BLIND hops (25/20/20/8 cm) at heading 0. Cold-start
 *     entry (`entryPwm = +TSHAPE_START_SPEED = 50`).
 *   - **TC5:** the SAME main chain, `PATH_BLIND_CHAIN_ENABLED_DEFAULT=false`
 *     target — every node stops (today's behavior, AC12 byte-identity).
 *   - **TC6 (hop cap):** the main chain, `MAX_CHAIN_HOPS_DEFAULT=2` target —
 *     the run truncates to `final->ChainA->ChainB` (2 hops), logs the
 *     truncation, and `ChainB->ChainC->ChainD` re-forms as a fresh 2-hop chain.
 *   - **TC6 (distance cap):** the main chain, `PATH_BLIND_CHAIN_MAX_CM_DEFAULT
 *     =50.0` target — the run truncates identically (25+20=45 <= 50 admits
 *     `ChainB`; +20=65 > 50 rejects `ChainC`).
 *   - **TC6 (ineligible reasons):** each probe REACHES its intended walk-ahead
 *     guard (a goal BEYOND the ineligible node, where needed, so the walk does not
 *     short-circuit on the goal guard first): `goTo(ChainA)` from `final` (goal IS
 *     the very next node — the degenerate goal-guard case, count stays 1);
 *     `ChainB->ChainC->ChainTiny` (the 3 cm `ChainC->ChainTiny` hop, below
 *     `TSHAPE_MIN_MOVE_CM`, hits the WALK-LEVEL short-hop break :2361);
 *     `ChainPivPre->Corner->ProjB->Zed` (the EXISTING `(Corner,ProjB,Zed)` pivot
 *     fires the pivot-onto-b guard :2312-2319 while evaluating `Corner->ProjB`);
 *     `ResetPre->Wall2->ResetBeyond` (Wall2 is an INTERIOR walk node, so the
 *     RESET-node guard :2287 ends the walk at Wall2); `SeekPre->Start->Inter1`
 *     (the forced-seek `Start->Inter1` hop fires the seek guard :2350);
 *     `DirFlipA->DirFlipB->DirFlipC` (an EDGES `direction="rear"` override on
 *     `DirFlipB->DirFlipC` flips direction mid-walk); `final -> position1 ->
 *     Wall1` (non-collinear bend, already in the base fixture).
 *   - **TC7 (LINE->BLIND spanning run):** the isolated `LpIn -> LpJoin ->
 *     LpBlindA -> LpBlindB` — a LINE pass-through junction (`LpJoin`) feeding
 *     directly into a 2-hop BLIND chain, all collinear at heading 0.
 *
 * Config overrides use `target_compile_definitions` (CMakeLists.txt), NEVER an
 * in-file `#define` (memory `gotcha-cross-tu-config-override-compile-def`):
 * `PathTracker.cpp` is a SEPARATELY-compiled TU from this test main, so an
 * in-file override would only affect this file's view of `Config::*`, not the
 * FSM's.
 */

#include "test_path_tracker_harness.h"

#include "src/drive/ChainSpeedPolicy.h"
#include "src/drive/TShapeProfilePolicy.h"

#include <cmath>
#include <cstdio>

namespace
{
// ===========================================================================
// Shared math helpers (frozen production policies used as oracles — see the
// file-level docblock).
// ===========================================================================

/// @brief Predict the exact PWM `tShapeSpeedAt` emits at @p traveledCm along a
///        SYNC_TSHAPE profile from @p startPwm to @p endPwm over @p totalCm at
///        the shipped `Config::TSHAPE_*` ramp knobs + cruise magnitude
///        @p cruiseMag. Mirrors `_execProfileMove`'s ramp resolution exactly
///        (same call signature Drive.cpp uses).
int16_t predictProfilePwmAt(float traveledCm, float totalCm, int16_t cruiseMag, int16_t startPwm,
                            int16_t endPwm)
{
    const TShapeRamps ramps = resolveTShapeRamps(totalCm, cruiseMag, startPwm, endPwm,
                                                 Config::TSHAPE_ACCEL_PWM_PER_CM,
                                                 Config::TSHAPE_RAMP_MIN_CM,
                                                 Config::TSHAPE_RAMP_MAX_CM,
                                                 Config::TSHAPE_NO_RAMP_EPSILON_PWM);
    return tShapeSpeedAt(traveledCm, totalCm, ramps, startPwm, endPwm);
}

/// @brief Resolve the ADR-059 chain speed plan for a run via the SAME frozen
///        `ChainSpeedPolicy.h::resolveChainSpeeds` production entry point
///        `PathTracker.cpp` calls, using the shipped `Config::*` knobs.
void predictChainPlan(const float* hopDistCm, uint8_t hopCount, int16_t cruisePwm,
                      int16_t entryPwm, ChainHopSpeed* out)
{
    const ChainSpeedConfig cfg{Config::PWM_MIN_MAGNITUDE, Config::PWM_MAX,
                               Config::TSHAPE_ACCEL_PWM_PER_CM, Config::TSHAPE_MIN_MOVE_CM};
    resolveChainSpeeds(hopDistCm, hopCount, cruisePwm, entryPwm, cfg, out);
}

/// @brief Predict the EXACT PWM the real FSM dispatches on the FIRST tick of chain
///        hop @p k (traveled == 0), given the already-resolved @p plan.
///
/// @details
/// Mirrors T13's dispatch, whose `endSpeed` differs by hop role:
///   - **Chained mid-hop** (`!plan[k].brake`): dispatches
///     `.setStartSpeed(plan[k].startPwm).setEndSpeed(plan[k].endPwm)`. When
///     `startSpd == cruise` (every post-cold-start mid-hop seeds its interior node
///     AT cruise), the forward ramp has ZERO accel room
///     (`resolveTShapeRampCm`'s no-ramp epsilon), so `traveled == 0` sits in the
///     ELSE-IF (decel) branch of `tShapeSpeedAt`, NOT the ramp-up branch — on a
///     SHORT hop this can even trigger the TRIANGLE fallback (reduced peak), so the
///     dispatched value at traveled==0 can differ from `plan[k].startPwm`. The
///     profile oracle below is the only correct predictor for a mid-hop.
///   - **Chain-FINAL braked hop** (`plan[k].brake`): T13 pins the braked end target
///     to the FLOOR — `.setStartSpeed(plan[k].startPwm).setEndSpeed(±PWM_MIN_MAGNITUDE)`
///     — NOT the braked-default `Config::TSHAPE_END_SPEED` (0), so the oracle runs the
///     profile with `endSpd == PWM_MIN_MAGNITUDE`. When the decel
///     `(cruise − PWM_MIN)/accel` shed FITS the hop, the accel ramp is zero and
///     `traveled == 0` sits BELOW the decel-zone entry, so `tShapeSpeedAt(0)` returns
///     `peakPwm` and the hop LAUNCHES at the plan start. When the shed OVERRUNS the hop
///     (a lower-floor / gentler-accel tune — e.g. `(70 − 35)/4 = 8.75 cm` in an 8 cm
///     hop), the profile triangle-collapses and `traveled == 0` launches BELOW the plan
///     start. Running the profile with the floor end target predicts BOTH regimes
///     exactly; a bare `plan[k].startPwm` short-circuit is only correct in the first.
///
/// This helper is the ONLY correct oracle for "what does the FSM actually emit on
/// hop k's first tick"; for a mid-hop it must run the profile, never hand-assert
/// `plan[k].startPwm`.
int16_t predictHopDispatchPwm(const float* hopDistCm, const ChainHopSpeed* plan, uint8_t k,
                              int16_t cruisePwm)
{
    if (plan[k].brake)
    {
        // Chain-final braked hop: T13 dispatches `.setEndSpeed(±PWM_MIN_MAGNITUDE)`, so the
        // dispatched decel targets the FLOOR, not plan[k].endPwm. Model that exactly by
        // running the profile with the floor end target: the launch equals plan[k].startPwm
        // ONLY when the (cruise−PWM_MIN)/accel shed fits the hop; when the shed overruns the
        // hop (a lower-floor / gentler-accel tune) the profile triangle-collapses and the
        // hop launches BELOW plan[k].startPwm.
        return predictProfilePwmAt(0.0f, hopDistCm[k], cruisePwm, plan[k].startPwm,
                                   Config::PWM_MIN_MAGNITUDE);
    }
    return predictProfilePwmAt(0.0f, hopDistCm[k], cruisePwm, plan[k].startPwm, plan[k].endPwm);
}

// ===========================================================================
// Main-chain driver (TC4/TC8/TC9): final -> ChainA -> ChainB -> ChainC -> ChainD.
// ===========================================================================

/// @brief The main chain's per-hop planned (uncarried) distances, in walk order.
constexpr float kMainHopDistCm[4] = {25.0f, 20.0f, 20.0f, 8.0f};
constexpr uint8_t kMainHopCount   = 4;

/// @brief Tick the real FSM until a BLIND_DEAD_RECKON SYNC-mode move has actually
///        emitted its first lower-MCU setpoint -- either the continuous profiled
///        `MiniR4.DriveDC.MoveSync(...)` (the common case) OR the ONE-SHOT
///        `MoveSyncDegs(...)` a hop below `Config::TSHAPE_MIN_MOVE_CM` degrades to
///        (Drive.h:1810 -- e.g. a chained hop whose carry-adjusted `dispatchCm`
///        shrinks below the profile floor dispatches FLAT via `.flat()`, which
///        routes through the non-profiled `DRIVE_DIST` path, never touching
///        `MoveSync`). Polling BOTH counters keeps this one helper correct for
///        every SYNC-mode BLIND dispatch in this file.
///
/// @details
/// Two ticks are load-bearing here, not one:
///   1. `IDLE -> DEQUEUE -> PLAN_NEXT_HOP -> dispatch` each advance ONE state per
///      tick (`DEQUEUE`'s body sets `_state = PLAN_NEXT_HOP` and returns WITHOUT
///      falling through to run that state's body the same tick), so a bare
///      `goTo(...).async(); tick();` is not enough to even reach dispatch.
///   2. The tick that DISPATCHES the profiled move (`Drive.update()`'s STAGE 3)
///      runs AFTER that same tick's STAGE 2 profile-advance, so `Drive.isIdle()`
///      flips `false` on the dispatch tick but `_updateProfile()` — and therefore
///      the first `MoveSync(...)` call — does not run until the FOLLOWING tick.
/// Polling the call counters (not `Drive.isIdle()`) covers both gaps in one bounded
/// loop. Bounded by `kMaxHandshakeTicks` so a real-FSM regression that never
/// dispatches/emits fails as a budget-exhausted assert, not a hang.
void tickUntilBlindDispatch()
{
    const uint32_t syncBefore     = MiniR4.DriveDC.moveSyncCallCount;
    const uint32_t syncDegsBefore = MiniR4.DriveDC.moveSyncDegsCallCount;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && MiniR4.DriveDC.moveSyncCallCount == syncBefore
                    && MiniR4.DriveDC.moveSyncDegsCallCount == syncDegsBefore;
        ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(MiniR4.DriveDC.moveSyncCallCount > syncBefore
               || MiniR4.DriveDC.moveSyncDegsCallCount > syncDegsBefore);
    ASSERT_FALSE(PathTracker.hasFault());
}

/// @brief Seat the world at `final` (app-yaw 0) and dispatch `goTo(ChainD)`, driving
///        the FSM to the point where the FIRST hop (`final -> ChainA`) is in flight
///        (dispatched, encoder still at baseline). Cold-start entry.
void seatAndDispatchMainChain()
{
    resetWorld();
    Serial.clearCapture();
    seatAt("final", 0);    // final -> ChainA departs heading 0 == the seated facing.
    PathTracker.goTo(nodeByName("ChainD")).async();
    tickUntilBlindDispatch();    // dispatch the first BLIND_DEAD_RECKON hop.
}

/// @brief Drive one profiled BLIND chain hop to completion via an EXACT (not
///        over-shot) encoder advance, so the boundary-carry math downstream
///        (TC8) sees a REALISTIC per-tick travel instead of the harness's
///        generic `kEncoderAdvanceStep` overshoot (T7's caveat).
///
/// @param commandedCm  The distance the FSM was told to drive THIS hop (after
///                      any boundary-carry subtraction).
/// @param carryCm       Extra travel to add on top of `commandedCm` before
///                      retiring (models the ~1.5-3 cm inter-hop boundary
///                      carry, AC9). Pass 0.0f to land exactly on target.
void driveChainHopExact(float commandedCm, float carryCm)
{
    const int32_t targetDelta = (int32_t) cmToEncDeg(commandedCm + carryCm);
    MiniR4.M2.mockAdvanceDegrees(targetDelta);
    MiniR4.M3.mockAdvanceDegrees(targetDelta);
    MiniR4.DriveDC.mockSetPrevTaskDone(true);
    cooperativeTick();
}

/// @brief The realistic per-tick encoder step this driver advances by (~3 cm/tick — a
///        plausible per-tick travel at cruise speed and a ~35 Hz loop, T7's caveat).
constexpr float kRealisticStepCm = 3.0f;

/// @brief Drive an in-flight profiled BLIND chain hop to completion via SMALL,
///        realistic per-tick encoder steps (@ref kRealisticStepCm) instead of a
///        single exact jump. Used by TC8, which does NOT hand-predict the
///        firmware's carry-adjusted `dispatchCm` (that value depends on the
///        PRECISE upstream measured carry, itself dependent on integer
///        encoder-degree truncation -- reproducing it bit-for-bit in the test
///        would just re-derive `_execProfileMove`'s own arithmetic). Instead
///        this polls `Localization.getCurrentNode()` until it advances,
///        bounded by `kMaxHandshakeTicks`, then continues stepping for
///        @p extraCarryTicks MORE ticks to model the inter-hop boundary carry
///        (the chassis coasting at the held PWM across the node boundary)
///        BEFORE the caller dispatches the next hop.
///
/// @param toNode          The node id this hop's dead-reckon advance must land on.
/// @param extraCarryTicks Additional small-step ticks to run AFTER arrival,
///                        modeling the boundary carry the next hop's dispatch
///                        will measure and subtract. 0 for the run's final hop
///                        (no next hop to carry into).
/// @return The number of ~3 cm advance-steps this driver took to REACH @p toNode
///         (the arrival-loop iteration count — the trailing carry ticks are NOT
///         counted). AC9 (TC8) asserts this is FEWER than the hop's uncarried
///         planned distance would require, making the firmware's carry SUBTRACTION
///         falsifiable: a hop whose commanded distance was reduced by the measured
///         carry completes in fewer steps than the raw edge length would. Removing
///         the subtraction (commanding the full edge length) inflates the count and
///         fails the assert.
int driveChainHopRealistic(uint8_t toNode, int extraCarryTicks)
{
    const int32_t stepEnc = (int32_t) cmToEncDeg(kRealisticStepCm);

    int steps = 0;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != toNode;
        ++i)
    {
        MiniR4.M2.mockAdvanceDegrees(stepEnc);
        MiniR4.M3.mockAdvanceDegrees(stepEnc);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
        ++steps;
    }
    ASSERT_EQ(Localization.getCurrentNode(), toNode);

    for (int i = 0; i < extraCarryTicks; ++i)
    {
        MiniR4.M2.mockAdvanceDegrees(stepEnc);
        MiniR4.M3.mockAdvanceDegrees(stepEnc);
        cooperativeTick();
    }
    return steps;
}
}    // namespace

// ===========================================================================
// Routing guards — pin the two chains + the ineligibility probes so a future
// fixture re-weight fails LOUDLY rather than silently re-routing the hop under
// test (Fable amendment 2).
// ===========================================================================

void test_routing_guards()
{
    TEST_SCOPE("routing: main chain + spanning run + TC6 probes resolve as expected");

    const uint8_t final_ = nodeByName("final");
    const uint8_t a      = nodeByName("ChainA");
    const uint8_t b      = nodeByName("ChainB");
    const uint8_t c      = nodeByName("ChainC");
    const uint8_t d      = nodeByName("ChainD");
    ASSERT_TRUE(final_ != FieldMap::NODE_NONE);
    ASSERT_TRUE(a != FieldMap::NODE_NONE);
    ASSERT_TRUE(b != FieldMap::NODE_NONE);
    ASSERT_TRUE(c != FieldMap::NODE_NONE);
    ASSERT_TRUE(d != FieldMap::NODE_NONE);

    ASSERT_EQ(routedFirstHop(final_, d), a);
    ASSERT_EQ(routedFirstHop(a, d), b);
    ASSERT_EQ(routedFirstHop(b, d), c);
    ASSERT_EQ(routedFirstHop(c, d), d);
    ASSERT_TRUE(FieldMap::getEdge(final_, a).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdge(a, b).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdge(b, c).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdge(c, d).type == Routes::EdgeType::BLIND);

    const uint8_t lpIn = nodeByName("LpIn");
    const uint8_t lpJ  = nodeByName("LpJoin");
    const uint8_t lpA  = nodeByName("LpBlindA");
    const uint8_t lpB  = nodeByName("LpBlindB");
    ASSERT_TRUE(lpIn != FieldMap::NODE_NONE && lpJ != FieldMap::NODE_NONE);
    ASSERT_TRUE(lpA != FieldMap::NODE_NONE && lpB != FieldMap::NODE_NONE);
    ASSERT_EQ(routedFirstHop(lpIn, lpB), lpJ);
    ASSERT_EQ(routedFirstHop(lpJ, lpB), lpA);
    ASSERT_TRUE(FieldMap::getEdge(lpIn, lpJ).type == Routes::EdgeType::LINE);
    ASSERT_TRUE(FieldMap::getEdge(lpJ, lpA).type == Routes::EdgeType::BLIND);

    // Ineligibility-probe routes: pin each probe's two-hop path so a fixture re-weight
    // that silently re-routes the hop under test fails LOUDLY here rather than binding a
    // probe on the wrong guard (Fable amendment 2). The RESET (ResetBeyond) and SEEK
    // (SeekPre) additive-leaf nodes MUST route through their intended interior node so
    // the walk reaches the RESET-node / seek guard (not the goal guard).
    const uint8_t chainB      = nodeByName("ChainB");
    const uint8_t chainC      = nodeByName("ChainC");
    const uint8_t chainTiny   = nodeByName("ChainTiny");
    const uint8_t chainPivPre = nodeByName("ChainPivPre");
    const uint8_t corner      = nodeByName("Corner");
    const uint8_t projB       = nodeByName("ProjB");
    const uint8_t zed         = nodeByName("Zed");
    const uint8_t resetPre    = nodeByName("ResetPre");
    const uint8_t wall2       = nodeByName("Wall2");
    const uint8_t beyond      = nodeByName("ResetBeyond");
    const uint8_t seekPre     = nodeByName("SeekPre");
    const uint8_t start       = nodeByName("Start");
    const uint8_t inter1      = nodeByName("Inter1");
    ASSERT_TRUE(chainTiny != FieldMap::NODE_NONE && chainPivPre != FieldMap::NODE_NONE);
    ASSERT_TRUE(beyond != FieldMap::NODE_NONE && seekPre != FieldMap::NODE_NONE);

    // Short-hop (walk-level): ChainB -> ChainC -> ChainTiny (the <5cm hop is mid-walk).
    ASSERT_EQ(routedFirstHop(chainB, chainTiny), chainC);
    ASSERT_EQ(routedFirstHop(chainC, chainTiny), chainTiny);
    ASSERT_TRUE(FieldMap::getEdge(chainC, chainTiny).distance_cm < Config::TSHAPE_MIN_MOVE_CM);
    // Pivot-onto-b: ChainPivPre -> Corner -> ProjB -> Zed (the existing pivot ends the walk).
    ASSERT_EQ(routedFirstHop(chainPivPre, zed), corner);
    ASSERT_EQ(routedFirstHop(corner, zed), projB);
    ASSERT_TRUE(FieldMap::findPivotTurn(corner, projB, zed) != nullptr);
    // RESET-node: ResetPre -> Wall2 -> ResetBeyond (Wall2 is interior, not the goal).
    ASSERT_EQ(routedFirstHop(resetPre, beyond), wall2);
    ASSERT_EQ(routedFirstHop(wall2, beyond), beyond);
    ASSERT_TRUE(FieldMap::getNodeType(wall2) == Routes::NodeType::RESET);
    // Seek: SeekPre -> Start -> Inter1 (the forced-seek Start->Inter1 ends the walk).
    ASSERT_EQ(routedFirstHop(seekPre, inter1), start);
    ASSERT_EQ(routedFirstHop(start, inter1), inter1);
    ASSERT_TRUE(FieldMap::getEdge(start, inter1).type == Routes::EdgeType::BLIND);
}

// ===========================================================================
// TC4 (enabled build) / TC5 (disabled build) — shared body, branches on the
// compiled `Config::PATH_BLIND_CHAIN_ENABLED`.
// ===========================================================================

void test_main_chain_dispatch()
{
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        TEST_SCOPE("TC4: blind chain ENABLED -- chained mid-hops, no inter-hop stop, braked final");
    }
    else
    {
        TEST_SCOPE("TC5: PATH_BLIND_CHAIN_ENABLED=false -- stops at every node (today's behavior)");
    }

    const uint8_t nA = nodeByName("ChainA");
    const uint8_t nB = nodeByName("ChainB");
    const uint8_t nC = nodeByName("ChainC");
    const uint8_t nD = nodeByName("ChainD");

    seatAndDispatchMainChain();

    // Predict the plan for the cold-start entry (+TSHAPE_START_SPEED) if chaining is
    // active; irrelevant to TC5, which never resolves a chain plan at all.
    ChainHopSpeed plan[4];
    predictChainPlan(kMainHopDistCm, kMainHopCount, Config::SPEED_NORMAL,
                     Config::TSHAPE_START_SPEED, plan);

    // ---- Hop 0: final -> ChainA. -------------------------------------------
    // seatAndDispatchMainChain() already ticked to the first MoveSync emission.
    // Predict the EXACT traveled==0 dispatch value via the frozen TShapeProfilePolicy
    // oracle (NOT the raw plan[k].startPwm -- see predictHopDispatchPwm's docblock).
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        const int16_t expected0 = predictHopDispatchPwm(kMainHopDistCm, plan, 0, Config::SPEED_NORMAL);
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, expected0);
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncR, expected0);
    }
    else
    {
        // TC5 (AC12): the ordinary per-hop path seeds TSHAPE_START_SPEED, unchanged
        // from pre-ADR-059 behavior.
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncR, Config::TSHAPE_START_SPEED);
    }

    const uint32_t brakeCallsBeforeHop0 = MiniR4.DriveDC.brakeCallCount;
    driveChainHopExact(kMainHopDistCm[0], 0.0f);    // retire hop 0 -> onDeadReckonedAdvance(ChainA).
    ASSERT_EQ(Localization.getCurrentNode(), nA);
    ASSERT_FALSE(PathTracker.hasFault());

    // THE NO-STOP ASSERT (TC4 core claim). ADR-059 cruise-hold (Drive finding #13 follow-up): a
    // non-braking chained mid-hop completing with an empty FIFO LEAVES the cruise setpoint latched
    // and issues NO `Move(0,0)+brake` at the boundary (the mock clock does not advance, so the
    // bounded fail-safe never trips) -- the next hop overwrites it seamlessly. So NO brake fires at
    // a chained mid-hop boundary (chaining ENABLED); TC5 (chaining disabled) brakes for real.
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop0);
    }
    else
    {
        ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop0);
        ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    }

    // Dispatch the next hop (final -> ChainA retired last tick; PLAN_NEXT_HOP ->
    // _dispatchEdgeAfterTurn -> BLIND_DEAD_RECKON for ChainA -> ChainB -- collinear,
    // no pre-turn).
    tickUntilBlindDispatch();

    // ---- Hop 1: ChainA -> ChainB. -------------------------------------------
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        const int16_t expected1 = predictHopDispatchPwm(kMainHopDistCm, plan, 1, Config::SPEED_NORMAL);
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, expected1);
    }
    else
    {
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);
    }

    const uint32_t brakeCallsBeforeHop1 = MiniR4.DriveDC.brakeCallCount;
    driveChainHopExact(kMainHopDistCm[1], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nB);
    // Chained mid-hop boundary: cruise-hold, no boundary brake (ENABLED); braked stop (DISABLED).
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop1);
    }
    else
    {
        ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop1);
        ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    }
    tickUntilBlindDispatch();

    // ---- Hop 2: ChainB -> ChainC. -------------------------------------------
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        const int16_t expected2 = predictHopDispatchPwm(kMainHopDistCm, plan, 2, Config::SPEED_NORMAL);
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, expected2);
    }
    else
    {
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);
    }

    const uint32_t brakeCallsBeforeHop2 = MiniR4.DriveDC.brakeCallCount;
    driveChainHopExact(kMainHopDistCm[2], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nC);
    // Chained mid-hop boundary: cruise-hold, no boundary brake (ENABLED); braked stop (DISABLED).
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop2);
    }
    else
    {
        ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop2);
        ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    }
    tickUntilBlindDispatch();

    // ---- Hop 3 (FINAL): ChainC -> ChainD. -----------------------------------
    // The final hop is the cruise->floor->brake stop: T13 dispatches
    // `.setEndSpeed(±PWM_MIN_MAGNITUDE)` (floor, not 0) and the brake completes floor->rest.
    // Under the retuned floor the (cruise−PWM_MIN)/accel = (70−45)/4 = 6.25 cm shed FITS within
    // the 8 cm hop, so the profile does NOT triangle-collapse and the hop LAUNCHES AT the plan
    // start. (The PWM_MIN_MAGNITUDE 35->45 retune, commit 55648f2, shortened the shed below the
    // hop length, reversing the earlier collapse-below-plan-start case.)
    // predictHopDispatchPwm models the now-uncollapsed dispatch via the profile math.
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        const int16_t expected3 = predictHopDispatchPwm(kMainHopDistCm, plan, 3, Config::SPEED_NORMAL);
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, expected3);
        ASSERT_TRUE(plan[3].brake);
        ASSERT_EQ(expected3, plan[3].startPwm);    // shed fits the hop -> launches at plan start
    }
    else
    {
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);
    }

    const uint32_t brakeCallsBeforeHop3 = MiniR4.DriveDC.brakeCallCount;
    driveChainHopExact(kMainHopDistCm[3], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nD);
    ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop3);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // THE FINAL HOP ALWAYS BRAKES FOR REAL.

    // Settle: cur == goal == ChainD -> IDLE.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());

    // Each intermediate node stamped ONCE: the discrete node walked A->B->C->D in
    // strict order above (each ASSERT_EQ checked the SPECIFIC node right after that
    // hop's retirement) -- a double-stamp or a skipped node would have failed one of
    // those checks already.

    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        std::printf("INFO: PATH_BLIND_CHAIN_ENABLED == true (TC4 chained dispatch)\n");
    }
    else
    {
        std::printf("INFO: PATH_BLIND_CHAIN_ENABLED == false (TC5 legacy per-hop stop)\n");
    }
}

// ===========================================================================
// TC6a — hop cap (MAX_CHAIN_HOPS override, dedicated target).
// ===========================================================================

void test_hop_cap_truncates_and_reforms()
{
    TEST_SCOPE("TC6a: MAX_CHAIN_HOPS cap truncates the run, logs it, and reforms after");

    const uint8_t nA = nodeByName("ChainA");
    const uint8_t nB = nodeByName("ChainB");
    const uint8_t nC = nodeByName("ChainC");
    const uint8_t nD = nodeByName("ChainD");

    seatAndDispatchMainChain();

    // Hop 0 (final -> ChainA) dispatched by seatAndDispatchMainChain(); retire it.
    driveChainHopExact(kMainHopDistCm[0], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nA);
    tickUntilBlindDispatch();    // dispatch hop 1 (ChainA -> ChainB), the run's capped FINAL hop.

    // With MAX_CHAIN_HOPS == 2, the run is final->ChainA->ChainB (2 hops): hop 1 is
    // this run's FINAL hop and brakes for real, even though ChainB is NOT the goal.
    driveChainHopExact(kMainHopDistCm[1], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nB);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // capped run's final hop -- a REAL brake.
    ASSERT_FALSE(PathTracker.hasFault());

    // The cap truncation was LOGGED (Error Handling: never silent).
    const std::string& log = Serial.captured();
    ASSERT_TRUE(log.find("chain capped") != std::string::npos);
    ASSERT_TRUE(log.find("hop cap") != std::string::npos);

    tickUntilBlindDispatch();    // dispatch the re-formed chain's hop 0 (ChainB -> ChainC).

    // A FRESH 2-hop chain re-forms from ChainB: ChainB->ChainC->ChainD reaches the
    // goal naturally (no cap trip -- the walk ends because ChainD IS the goal).
    const uint32_t brakeCallsBeforeMid = MiniR4.DriveDC.brakeCallCount;
    driveChainHopExact(kMainHopDistCm[2], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nC);
    // Mid-hop of the re-formed chain: ADR-059 cruise-hold -> no boundary brake (seamless handoff).
    ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeMid);
    tickUntilBlindDispatch();

    driveChainHopExact(kMainHopDistCm[3], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nD);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // the re-formed chain's own final hop.

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC6b — distance cap (PATH_BLIND_CHAIN_MAX_CM override, dedicated target).
// Structurally identical truncation point to TC6a (25+20=45 <= 50 admits
// ChainB; +20=65 > 50 rejects ChainC) -- same run shape, different cap.
// ===========================================================================

void test_distance_cap_truncates_and_reforms()
{
    TEST_SCOPE("TC6b: PATH_BLIND_CHAIN_MAX_CM cap truncates the run and logs it");

    const uint8_t nA = nodeByName("ChainA");
    const uint8_t nB = nodeByName("ChainB");

    seatAndDispatchMainChain();
    driveChainHopExact(kMainHopDistCm[0], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nA);
    tickUntilBlindDispatch();

    driveChainHopExact(kMainHopDistCm[1], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nB);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // capped run's final hop -- a REAL brake.
    ASSERT_FALSE(PathTracker.hasFault());

    const std::string& log = Serial.captured();
    ASSERT_TRUE(log.find("chain capped") != std::string::npos);
    ASSERT_TRUE(log.find("distance cap") != std::string::npos);
}

// ===========================================================================
// TC6c — ineligible-reason probes (default-build target: chaining ON, no
// override needed -- each probe is a SEPARATE, isolated goTo()).
// ===========================================================================

void test_ineligible_goal_ends_walk_before_it_starts()
{
    TEST_SCOPE("TC6c: goTo(ChainA) from final -- goal is the very next node, no chain forms");

    resetWorld();
    seatAt("final", 0);
    const uint8_t nA = nodeByName("ChainA");
    PathTracker.goTo(nA).async();
    tickUntilBlindDispatch();

    // A single hop to the goal is NOT a chain (count stays 1, AC12) -- it dispatches
    // on the ordinary per-hop path, seeded with the cold-start TSHAPE_START_SPEED
    // (no chain plan resolved at all).
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);

    driveChainHopExact(kMainHopDistCm[0], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nA);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // the only hop -- an ordinary braked stop.
    ASSERT_FALSE(PathTracker.hasFault());
}

void test_ineligible_short_hop_below_profile_floor()
{
    TEST_SCOPE("TC6c: ChainB -> ChainC -> ChainTiny -- the WALK-LEVEL <5cm hop ends the chain");

    resetWorld();
    Serial.clearCapture();
    seatAt("ChainB", 0);    // ChainB -> ChainC departs heading 0 == the seated facing.
    const uint8_t nC   = nodeByName("ChainC");
    const uint8_t tiny = nodeByName("ChainTiny");
    ASSERT_TRUE(nC != FieldMap::NODE_NONE && tiny != FieldMap::NODE_NONE);

    // Goal is ChainTiny, so hop 0 (ChainB->ChainC, 20 cm) passes hop-0 eligibility and
    // the walk ADVANCES to evaluate ChainC->ChainTiny (3 cm). That hop clears the
    // goal / reset / pivot / collinearity / seek guards, so the ONLY guard that ends the
    // walk is the WALK-LEVEL `e.distance_cm < TSHAPE_MIN_MOVE_CM` branch
    // (PathTracker.cpp:2361) -- distinct from the hop-0 pre-walk short-hop guard (:2255)
    // the old probe (seated at ChainC) exercised. Pin the two-hop route so a re-weight
    // fails loudly.
    ASSERT_EQ(routedFirstHop(nodeByName("ChainB"), tiny), nC);    // hop 0 is ChainB->ChainC (20 cm)
    ASSERT_EQ(routedFirstHop(nC, tiny), tiny);                    // then the sub-5cm ChainC->ChainTiny
    ASSERT_TRUE(FieldMap::getEdge(nC, tiny).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::getEdge(nC, tiny).distance_cm < Config::TSHAPE_MIN_MOVE_CM);

    PathTracker.goTo(tiny).async();

    // Hop 0 (ChainB->ChainC) is a PROFILED SYNC_TSHAPE hop -- the walk broke before
    // admitting the tiny hop, so no chain formed (count stayed 1) and hop 0 dispatches
    // on the ordinary per-hop path via the continuous `MoveSync(...)`, seeded with the
    // cold-start TSHAPE_START_SPEED.
    tickUntilBlindDispatch();
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);

    driveChainHopExact(kMainHopDistCm[2], 0.0f);    // ChainB->ChainC is 20 cm (kMainHopDistCm[2]).
    ASSERT_EQ(Localization.getCurrentNode(), nC);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // chain ended before the tiny hop -- braked stop at ChainC.
    ASSERT_FALSE(PathTracker.hasFault());

    // The onward ChainC->ChainTiny hop (3 cm < TSHAPE_MIN_MOVE_CM) then dispatches as a
    // NON-PROFILED `DRIVE_DIST` via `MiniR4.DriveDC.MoveSyncDegs(...)` (Drive.h's min-move
    // degrade), NOT the continuous `MoveSync(...)` -- a fresh braked hop, not a chained one.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && MiniR4.DriveDC.moveSyncDegsCallCount == 0;
        ++i)
    {
        cooperativeTick();
    }
    ASSERT_GT(MiniR4.DriveDC.moveSyncDegsCallCount, (uint32_t) 0);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncDegsL, Config::SPEED_NORMAL);    // flat power, no ramp pair.

    MiniR4.DriveDC.mockSetPrevTaskDone(true);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != tiny; ++i)
    {
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), tiny);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveSyncDegsBrake);
    ASSERT_FALSE(PathTracker.hasFault());
}

void test_ineligible_pivot_ends_walk()
{
    TEST_SCOPE("TC6c: ChainPivPre -> Corner -> ProjB -> Zed -- the pivot-onto-b guard ends the walk");

    resetWorld();
    seatAt("ChainPivPre", 0);    // ChainPivPre -> Corner departs heading 0.
    const uint8_t corner = nodeByName("Corner");
    const uint8_t projB  = nodeByName("ProjB");
    const uint8_t zed    = nodeByName("Zed");
    ASSERT_TRUE(corner != FieldMap::NODE_NONE);
    ASSERT_TRUE(projB != FieldMap::NODE_NONE && zed != FieldMap::NODE_NONE);

    // Goal is Zed (BEYOND Corner), so the walk does NOT stop at the goal guard on
    // hop 0 -- it admits ChainPivPre->Corner and REACHES the pivot check while
    // evaluating the next hop Corner->ProjB. The pivot-onto-b lookahead
    // `findPivotTurn(a=Corner, b=ProjB, c=Zed)` matches the EXISTING (Corner,ProjB,Zed)
    // pivot record, so the walk breaks at PathTracker.cpp:2312-2319 (the pivot-onto-b
    // guard -- a->b must stop delta short of b to pivot there) BEFORE the collinearity
    // check. This binds the probe on its intended guard, not the goal guard.
    ASSERT_EQ(routedFirstHop(nodeByName("ChainPivPre"), zed), corner);    // hop 0 is ChainPivPre->Corner
    ASSERT_EQ(routedFirstHop(corner, zed), projB);                        // then Corner->ProjB
    ASSERT_EQ(routedFirstHop(projB, zed), zed);                           // then ProjB->Zed (the c leg)
    ASSERT_TRUE(FieldMap::getEdge(corner, projB).type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(FieldMap::findPivotTurn(corner, projB, zed) != nullptr);

    PathTracker.goTo(zed).async();
    tickUntilBlindDispatch();

    // The walk broke at the pivot-onto-b guard, so NO chain formed (count stayed 1):
    // hop 0 (ChainPivPre->Corner) dispatches on the ordinary per-hop path, seeded with
    // the cold-start TSHAPE_START_SPEED, and brakes to rest at Corner.
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);

    driveChainHopExact(40.0f, 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), corner);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // chain ended before the pivot hop -- braked stop at Corner.
    ASSERT_FALSE(PathTracker.hasFault());
}

void test_ineligible_reset_node_ends_walk()
{
    TEST_SCOPE("TC6c: ResetPre -> Wall2 -> ResetBeyond -- the RESET-node guard ends the walk at Wall2");

    resetWorld();
    const uint8_t resetPre = nodeByName("ResetPre");
    const uint8_t wall2    = nodeByName("Wall2");
    const uint8_t beyond   = nodeByName("ResetBeyond");
    ASSERT_TRUE(resetPre != FieldMap::NODE_NONE && wall2 != FieldMap::NODE_NONE);
    ASSERT_TRUE(beyond != FieldMap::NODE_NONE);
    ASSERT_TRUE(FieldMap::getNodeType(wall2) == Routes::NodeType::RESET);

    // Goal is ResetBeyond (a leaf PAST Wall2 reached ONLY via Wall2->ResetBeyond), so
    // Wall2 is an INTERIOR walk node, not the goal: the walk admits ResetPre->Wall2 (hop
    // 0) then evaluates `a = Wall2` and breaks at the RESET-node guard
    // (PathTracker.cpp:2287) -- the wall-zero interlude needs a stopped chassis -- rather
    // than at the goal guard (:2283) the old probe (goTo(Wall2)) hit first. Pin the
    // two-hop route so a re-weight fails loudly.
    ASSERT_EQ(routedFirstHop(resetPre, beyond), wall2);    // hop 0 is ResetPre->Wall2
    ASSERT_EQ(routedFirstHop(wall2, beyond), beyond);      // Wall2 is interior (onward edge exists)

    // ResetPre -> Wall2 departs at 131 deg -- seated at app-yaw 0, so a real pre-edge
    // turn dispatches first (settled via the harness's `expectPreTurn` idiom: park the
    // corrected IMU read at the app-frame target every tick until the custom TurnGyro
    // profile completes).
    seatAt("ResetPre", 0);
    const Routes::EdgeInfo& edge = FieldMap::getEdge(resetPre, wall2);

    PathTracker.goTo(beyond).async();
    expectPreTurn((int16_t) lroundf(edge.heading_deg), (uint8_t) Routes::PIPELINE_CONFIG.turn);
    tickUntilBlindDispatch();

    // The walk broke at the RESET-node guard, so no chain formed (count stayed 1): hop 0
    // dispatches on the ordinary per-hop path, seeded with the cold-start TSHAPE_START_SPEED.
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);

    driveChainHopExact(20.0f, 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), wall2);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // chain ended at the RESET node -- braked stop at Wall2.
    // The wall-zero reset interlude fires on arrival (WALL_STOP -> REANCHOR ->
    // WALL_DEPART) -- out of scope here (covered by test_path_tracker_reset_arrival);
    // this test only asserts the BLIND-chain walk itself stopped braked at the RESET
    // node rather than trying to continue a chain through it toward ResetBeyond.
}

void test_ineligible_seek_node_ends_walk()
{
    TEST_SCOPE("TC6c: SeekPre -> Start -> Inter1 -- the stopAtLine SEEK guard ends the walk at Start");

    resetWorld();
    const uint8_t seekPre = nodeByName("SeekPre");
    const uint8_t start   = nodeByName("Start");
    const uint8_t inter1  = nodeByName("Inter1");
    ASSERT_TRUE(seekPre != FieldMap::NODE_NONE && start != FieldMap::NODE_NONE);
    ASSERT_TRUE(inter1 != FieldMap::NODE_NONE);

    // Goal is Inter1 (BEYOND Start), so hop 0 (SeekPre->Start) is admitted and the walk
    // advances to evaluate Start->Inter1. That edge carries the EDGES `stop_at_line="on"`
    // override, so `resolveStopAtLine` classifies it as a SEEK (stopAtLine) hop and the
    // walk breaks at the seek guard (PathTracker.cpp:2350) -- the seek is a fresh braked
    // hop whose arm window absorbs the chain's residual dead-reckon. SeekPre->Start is
    // authored EXACTLY collinear with Start->Inter1 (SeekPre lies on the backward
    // extension of that line, both heading 23.96 deg), so the collinearity check
    // (:2324) PASSES and the walk genuinely reaches the seek guard rather than ending on
    // a bend. Pin the two-hop route so a re-weight fails loudly.
    ASSERT_EQ(routedFirstHop(seekPre, inter1), start);    // hop 0 is SeekPre->Start
    ASSERT_EQ(routedFirstHop(start, inter1), inter1);     // then the forced-seek Start->Inter1
    ASSERT_TRUE(FieldMap::getEdge(start, inter1).type == Routes::EdgeType::BLIND);

    // SeekPre -> Start departs at ~24 deg (collinear with Start->Inter1) -- seated at
    // app-yaw 0, so a real pre-edge turn dispatches first (settled via `expectPreTurn`).
    seatAt("SeekPre", 0);
    const Routes::EdgeInfo& edge = FieldMap::getEdge(seekPre, start);

    PathTracker.goTo(inter1).async();
    expectPreTurn((int16_t) lroundf(edge.heading_deg), (uint8_t) Routes::PIPELINE_CONFIG.turn);
    tickUntilBlindDispatch();

    // The walk broke at the seek guard, so no chain formed (count stayed 1): hop 0
    // (SeekPre->Start, an ordinary dead-reckon -- Start is BLIND, no seek on this edge)
    // dispatches on the per-hop path, seeded with the cold-start TSHAPE_START_SPEED.
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);

    driveChainHopExact(edge.distance_cm, 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), start);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // chain ended before the seek hop -- braked stop at Start.
    ASSERT_FALSE(PathTracker.hasFault());
    // The onward Start->Inter1 seek (stopAtLine) then dispatches as a fresh braked seek
    // hop -- out of scope here (covered by test_path_tracker_stop_at_line); this probe
    // only asserts the BLIND-chain walk ended braked at Start rather than chaining
    // through the seek.
}

void test_ineligible_direction_flip_ends_walk()
{
    TEST_SCOPE("TC6c: DirFlipB -> DirFlipC (direction_override=rear) ends the chain at DirFlipB");

    resetWorld();
    Serial.clearCapture();
    seatAt("DirFlipA", 0);
    const uint8_t b = nodeByName("DirFlipB");
    const uint8_t c = nodeByName("DirFlipC");
    ASSERT_TRUE(b != FieldMap::NODE_NONE && c != FieldMap::NODE_NONE);

    // Pin the override: DirFlipB->DirFlipC is forced REAR while DirFlipA->DirFlipB
    // resolves FRONT (AUTO, 1-board default chassis never resolves REAR on its own).
    ASSERT_TRUE(FieldMap::getEdge(b, c).direction_override == Routes::DirectionMode::FORCE_REVERSE);

    PathTracker.goTo(c).async();
    tickUntilBlindDispatch();

    // Cold-start (DirFlipA->DirFlipB alone is not admitted into a 2-hop chain: the
    // walk's hop-1 candidate DirFlipB->DirFlipC fails the direction-consistency
    // check, so count never reaches 2 -- AC12 ordinary per-hop path).
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);

    driveChainHopExact(20.0f, 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), b);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // no chain -- ordinary braked stop at DirFlipB.
    ASSERT_FALSE(PathTracker.hasFault());

    // The onward DirFlipB -> DirFlipC hop dispatches REAR on its own (forced) -- REAR
    // rotates the BODY target 180 deg from the edge heading (0 -> 180), which differs
    // from the current world facing (0), so a REAL pre-edge turn dispatches FIRST
    // (`expectPreTurn`'s settle idiom: park the corrected IMU read at the app-frame
    // target every tick until the custom TurnGyro profile completes). A negative
    // signed cruise is NOT observable via MoveSync's unsigned deadbandSnap(mag) call
    // -- SYNC mode passes |mag| through _emitContinuous with no separate sign channel
    // exposed to this mock -- so this test scopes its claim to "the chain ended at
    // DirFlipB, the onward hop still completes"; the direction resolution ITSELF is
    // covered by test_path_tracker_bidirectional.cpp).
    expectPreTurn(180, (uint8_t) Routes::PIPELINE_CONFIG.turn);
    tickUntilBlindDispatch();
    driveChainHopExact(20.0f, 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), c);
    ASSERT_FALSE(PathTracker.hasFault());
}

void test_ineligible_non_collinear_bend_ends_walk()
{
    TEST_SCOPE("TC6c: final -> position1 -> Wall1 (non-collinear bend) ends the chain at position1");

    resetWorld();
    seatAt("final", 0);
    const uint8_t pos1  = nodeByName("position1");
    const uint8_t wall1 = nodeByName("Wall1");
    ASSERT_TRUE(pos1 != FieldMap::NODE_NONE && wall1 != FieldMap::NODE_NONE);

    // goTo(Wall1) from `final` MUST route through `position1` (NOT directly, and NOT
    // via any other node) -- pinned so a fixture re-weight fails LOUDLY rather than
    // silently re-routing the bend under test (Fable amendment 2; `final -> Inter3`
    // directly bypasses `position1` on this fixture's shortest path, so the goal here
    // is deliberately Wall1, not Inter3).
    ASSERT_EQ(routedFirstHop(nodeByName("final"), wall1), pos1);
    ASSERT_EQ(routedFirstHop(pos1, wall1), wall1);

    const Routes::EdgeInfo& inbound  = FieldMap::getEdge(nodeByName("final"), pos1);
    const Routes::EdgeInfo& outbound = FieldMap::getEdge(pos1, wall1);
    ASSERT_TRUE(inbound.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(outbound.type == Routes::EdgeType::BLIND);
    const float bend = wrapDeltaDeg(outbound.heading_deg - inbound.heading_deg);
    ASSERT_TRUE(fabsf(bend) > Config::NAV_TURN_TOLERANCE_DEG);

    PathTracker.goTo(wall1).async();

    // final -> position1 is GYRO_TSHAPE (a per-edge BlindMode override) and departs
    // heading 20.85 -- seated at yaw 0, so a real pre-edge turn dispatches once the FSM
    // works through IDLE -> DEQUEUE -> PLAN_NEXT_HOP -> TURNING. Park the corrected IMU
    // read (set ONCE before ticking is sufficient -- it is a static mock field, not a
    // per-tick decay; `expectPreTurn`'s own idiom relies on exactly this) so the turn
    // settles the instant it dispatches, then continue until the onward BLIND move
    // (or, if IMMEDIATELY re-turning, the NEXT turn) is in flight.
    const int16_t hwHeading = hwYaw((int16_t) lroundf(inbound.heading_deg));
    mmL.mockYawDeg = (double) hwHeading;
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Drive.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_FALSE(Drive.isIdle());

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != pos1;
        ++i)
    {
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), pos1);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // no chain -- ordinary braked stop at position1.
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC7 — LINE->BLIND spanning collinear run: entryPwm = captured cruise on the
// first blind hop (no floor dip).
// ===========================================================================

void test_line_to_blind_spanning_run_seeds_captured_cruise()
{
    TEST_SCOPE("TC7: LINE pass-through -> BLIND chain seeds entryPwm = captured cruise (no floor dip)");

    const uint8_t lpIn = nodeByName("LpIn");
    const uint8_t lpJ  = nodeByName("LpJoin");
    const uint8_t lpA  = nodeByName("LpBlindA");
    const uint8_t lpB  = nodeByName("LpBlindB");

    resetWorld();
    Serial.clearCapture();
    EventTrace.mockReset();
    seatAt("LpIn", 0);    // LpIn -> LpJoin departs heading 0 == the seated facing.
    PathTracker.goTo(lpB).async();

    // Drive to LINE_FOLLOW for the LpIn -> LpJoin hop (the harness's shared entry
    // helper -- identical precondition to every other LINE-follow-entry test).
    for (int i = 0; i < path_harness::kMaxHandshakeTicks; ++i)
    {
        cooperativeTick();
        if (LineFollower.isEnabled() && !LineFollower.lastOvershootMode
            && Localization.getCurrentNode() == lpIn)
        {
            break;
        }
    }
    ASSERT_EQ(Localization.getCurrentNode(), lpIn);
    ASSERT_TRUE(LineFollower.isEnabled());

    // Cross LpJoin: the pass-through peek fires (LpJoin's onward flank LpJoin ->
    // LpBlindA is collinear BLIND and LpJoin is not the goal), dead-reckons the
    // mid-follow advance onto LpJoin, and keeps the follower live into the BLIND hop
    // -- the LINE_OVERSHOOT re-arm is where `brakeOnGate` is actually SET (the
    // LINE_FOLLOW dispatch arm above never touches it), so the pass-through assert
    // belongs AFTER this trip, not before.
    const int armsBefore = LineFollower.setEnabledCount;
    Intersection.mockSetAtIntersection(true);
    cooperativeTick();
    ASSERT_GT(LineFollower.setEnabledCount, armsBefore);
    ASSERT_FALSE(LineFollower.lastBrakeOnGate);
    LineFollower.mockSetDistanceReached(true);
    cooperativeTick();
    ASSERT_EQ(Localization.getCurrentNode(), lpJ);
    ASSERT_FALSE(PathTracker.hasFault());

    // The BLIND_DEAD_RECKON dispatch for LpJoin -> LpBlindA consumes _passThrough:
    // entryPwm seeds the captured cruise (Config::SPEED_NORMAL via the LineMode
    // NORMAL preset), NOT TSHAPE_START_SPEED.
    tickUntilBlindDispatch();

    ChainHopSpeed plan[2];
    const float   hopDist[2] = {20.0f, 20.0f};
    predictChainPlan(hopDist, 2, Config::SPEED_NORMAL, Config::SPEED_NORMAL, plan);

    // THE TC7 ASSERT: the recorded start speed on the FIRST blind hop equals the
    // captured cruise (85), NOT TSHAPE_START_SPEED (50) -- no re-dip to the floor.
    const int16_t expectedLp0 = predictHopDispatchPwm(hopDist, plan, 0, Config::SPEED_NORMAL);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, expectedLp0);
    ASSERT_EQ(plan[0].startPwm, Config::SPEED_NORMAL);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveSyncL != Config::TSHAPE_START_SPEED);

    const uint32_t brakeCallsBeforeSpanMid = MiniR4.DriveDC.brakeCallCount;
    driveChainHopExact(hopDist[0], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), lpA);
    // Mid-hop of the spanning chain: ADR-059 cruise-hold -> no boundary brake (seamless handoff).
    ASSERT_EQ(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeSpanMid);
    tickUntilBlindDispatch();

    driveChainHopExact(hopDist[1], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), lpB);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake);    // the run's final hop -- a real brake.

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

// ===========================================================================
// TC8 — inter-hop boundary carry (AC9): the next hop's commanded distance is
// reduced by the measured carry; the final node lands on-node within
// tolerance, no drift with N.
// ===========================================================================

void test_boundary_carry_subtracted_no_drift_with_n()
{
    TEST_SCOPE("TC8: inter-hop boundary carry is measured + subtracted; final node lands on-node");

    const uint8_t nA = nodeByName("ChainA");
    const uint8_t nB = nodeByName("ChainB");
    const uint8_t nC = nodeByName("ChainC");
    const uint8_t nD = nodeByName("ChainD");

    // T7's caveat: the harness's generic `kEncoderAdvanceStep` (~1100 cm/tick) would
    // overshoot every hop so far that the measured carry always clamps to the
    // `PIVOT_TRIM_MIN_DRIVE_CM` floor, defeating this test. `driveChainHopRealistic`
    // instead steps the encoder by a REALISTIC ~3 cm/tick (a plausible per-tick
    // travel at cruise speed and a ~35 Hz loop) until the node advances, then keeps
    // stepping for a couple more ticks to model the inter-hop boundary carry (the
    // chassis coasting at the held PWM across the node boundary, AC9's ~1.5-3 cm
    // estimate) BEFORE the next hop's dispatch measures it. This does NOT
    // hand-predict the firmware's own carry-adjusted `dispatchCm` (which depends on
    // integer encoder-degree truncation internal to `_execProfileMove` -- reproducing
    // it bit-for-bit in the test would just re-derive that arithmetic); it instead
    // exercises the REAL carry-measure-and-subtract path end-to-end and asserts the
    // OUTCOME (every hop still lands exactly on its target node, no accumulated
    // drift, the run completes cleanly).
    seatAndDispatchMainChain();

    // NOTE: no separate `tickUntilBlindDispatch()` between hops here. A short
    // enough carry-adjusted `dispatchCm` degrades to the ONE-SHOT non-profiled
    // `MoveSyncDegs` path (Drive.h:1810's min-move threshold), which can dispatch
    // AND retire within the SAME tick if the done-bit is already latched true
    // (sticky from the prior hop's arrival loop) -- i.e. the next hop's arrival can
    // race ahead DURING the current hop's trailing carry ticks, before any explicit
    // dispatch-wait would run. `driveChainHopRealistic`'s own arrival loop is
    // dispatch-agnostic (it just polls the discrete node), so it is correct whether
    // the next hop is freshly dispatched or already retired by the time it is
    // called.
    //
    // Seed a genuine boundary carry (2 trailing ~3 cm ticks ~= 6 cm) after EACH hop
    // so the NEXT hop's dispatch measures + subtracts it. `driveChainHopRealistic`
    // returns the arrival-loop step count so the carry SUBTRACTION is falsifiable
    // below (a hop whose commanded distance was reduced by the carry completes in
    // fewer ~3 cm steps than its raw edge length would need).
    driveChainHopRealistic(nA, /*extraCarryTicks*/ 2);
    const int hop1Steps = driveChainHopRealistic(nB, /*extraCarryTicks*/ 2);
    driveChainHopRealistic(nC, /*extraCarryTicks*/ 2);

    // AC9 FALSIFIABILITY: hop 1 (ChainA->ChainB) is a 20 cm edge. Its dispatch measures
    // the ~6 cm carry rolled by hop 0's 2 trailing ticks and subtracts it, so it COMMANDS
    // ~14 cm and arrives in 6 advance-steps here (ceil(14/3)=5 travel + 1 dispatch-settle
    // tick). The UNCARRIED 20 cm edge measures 7 steps (ceil(20/3)=7) -- EMPIRICALLY
    // CONFIRMED by seeding hop 0 with 0 carry: hop1Steps went 6 -> 7. So the carry
    // subtraction is load-bearing and FALSIFIABLE: strip the
    // `dispatchCm = effectiveDistCm - carryCm` line in PathTracker.cpp and hop 1 commands
    // the full 20 cm -> 7 steps -> the strict ASSERT_GT below fails. This is the check
    // `onDeadReckonedAdvance` (which stamps the node at move completion regardless of the
    // commanded distance) alone could NOT provide -- the step count is what exposes a
    // vanished subtraction. The threshold 7 EQUALS the uncarried count, so the strict `GT`
    // is what carries the assertion: the carried path (6) passes and the uncarried path (7)
    // fails. In this deterministic host harness the carried count is stable at 6.
    constexpr int kUncarriedHop1Steps  = 7;     // == uncarried count; strict GT fails at 7, passes at carried 6.
    constexpr int kCarriedHop1MaxSteps = 6;     // the measured carry-adjusted budget.
    ASSERT_GT(kUncarriedHop1Steps, hop1Steps);  // strictly FEWER steps than the raw edge would need (6 < 7).
    ASSERT_GE(kCarriedHop1MaxSteps, hop1Steps); // and within the carry-adjusted budget.

    // Final hop: no further carry to seed (this is the run's last hop) -- land
    // EXACTLY on ChainD with no residual drift. Its carry-adjusted dispatchCm can
    // land EITHER regime (profiled `MoveSync`, brake via `MiniR4.DriveDC.brake()`,
    // OR a sub-TSHAPE_MIN_MOVE_CM one-shot `MoveSyncDegs(..., brake=true, ...)` --
    // see driveChainHopRealistic's docblock); assert whichever fired braked.
    driveChainHopRealistic(nD, /*extraCarryTicks*/ 0);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake || MiniR4.DriveDC.lastMoveSyncDegsBrake);
    ASSERT_FALSE(PathTracker.hasFault());

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());

    // NO-DRIFT-WITH-N: `Localization::onDeadReckonedAdvance` snaps the discrete node
    // to the GRAPH coordinate on every hop (blind nodes carry no positional
    // uncertainty state of their own -- they are always "at" their mapped
    // coordinate once the FSM stamps the advance). The load-bearing claim AC9
    // protects is that the FSM's own COMMANDED distance per hop tracks the
    // carry-adjusted plan rather than silently drifting the commanded distance high
    // (no error accumulation): every one of the 4 hops here landed EXACTLY on its
    // target node under a REALISTIC (not the harness's huge overshoot) per-hop
    // advance with a genuine inter-hop carry seeded before each next dispatch, and
    // hop 1's step count above PROVES the measured-carry subtraction actually
    // shortened the commanded distance -- a broken (assumed-zero) carry model would
    // command the full edge length, inflating the step count past the assert.
}

// ===========================================================================
// ADR-079 (2026-07-22 blind-column-overshoot plan, T6): host ctests asserting the
// REAL dispatched `MiniR4.DriveDC` setpoint for the two PARITY-INVISIBLE firmware
// changes -- the chained final-hop degraded-stub FLOOR pin (T4) and the generalized
// braked-blind measured-carry subtraction + its resets (T5). Per Decisions
// `gotcha-parity-ctps-record-plan-not-dispatch`: the parity/sim/CTPS traces record
// the PLAN endpoints, so neither change appears there -- these ctests are the SOLE
// guard. Every assertion below reads the actual lower-MCU call the real `Drive.cpp`
// made through the shared mock (`lastMoveSyncL/R`, `lastMoveSyncDegsL/R/Brake`,
// `moveSyncCallCount` vs `moveSyncDegsCallCount`, `lastBrake`), never a plan record.
//
// Which mock counter each dispatch stamps (load-bearing to read the asserts):
//   - A PROFILED braked blind hop dispatched at >= TSHAPE_MIN_MOVE_CM drives the
//     continuous `MiniR4.DriveDC.MoveSync(...)` (moveSyncCallCount++), first-tick
//     setpoint == the profile start PWM.
//   - A hop BELOW TSHAPE_MIN_MOVE_CM degrades to the one-shot FLAT
//     `MiniR4.DriveDC.MoveSyncDegs(power,power,deg,brake,async)` (moveSyncDegsCallCount++),
//     setpoint == the flat POWER, `brake` == the move's brake flag. The ADR-059
//     chained degraded stub takes THIS path; ADR-079's T4 pins its FINAL-hop power
//     to the motor floor (`±PWM_MIN_MAGNITUDE`) instead of the plan start PWM.
// So `MoveSync` fired == full-distance profiled dispatch; `MoveSyncDegs` fired ==
// a shrunk-below-floor dispatch. That binary is the robust observable the reset
// tests below key on: a braked blind hop whose carry was (wrongly) NOT reset shrinks
// under a large stale carry to the degraded `MoveSyncDegs` path, while a correctly
// reset carry leaves it a full-distance profiled `MoveSync`.
// ===========================================================================

// ---------------------------------------------------------------------------
// TC1 (AC3) — chained-final degraded-stub FLOOR pin (T4), asserted on the REAL
// dispatched flat setpoint. Two arms, both MUST-HAVE:
//   (a) a chain FINAL hop shrunk below TSHAPE_MIN_MOVE_CM by a seeded boundary
//       carry dispatches its degraded stub at `±PWM_MIN_MAGNITUDE` (the floor),
//       braked — NOT the plan start PWM (the pre-T4 behavior);
//   (b) a chain MID hop shrunk below the floor still dispatches its planned exit
//       PWM (`chainEndPwm`), un-braked (the mid stub is unchanged by T4).
// ---------------------------------------------------------------------------

void test_stub_guard_final_hop_floor()
{
    TEST_SCOPE("TC1a/AC3: chained FINAL hop below TSHAPE_MIN_MOVE_CM dispatches the floor, braked");

    const uint8_t nA = nodeByName("ChainA");
    const uint8_t nB = nodeByName("ChainB");
    const uint8_t nC = nodeByName("ChainC");

    seatAndDispatchMainChain();    // hop 0 (final -> ChainA) in flight.

    // Resolve the run's plan so the assertion can prove the dispatched floor is NOT
    // the (pre-T4) plan start PWM the degraded stub used to emit.
    ChainHopSpeed plan[4];
    predictChainPlan(kMainHopDistCm, kMainHopCount, Config::SPEED_NORMAL, Config::TSHAPE_START_SPEED,
                     plan);

    // Retire hops 0 and 1 with NO carry so their commanded distances stay exact and the
    // chain plan is undisturbed up to the final hop.
    driveChainHopExact(kMainHopDistCm[0], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nA);
    tickUntilBlindDispatch();
    driveChainHopExact(kMainHopDistCm[1], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nB);
    tickUntilBlindDispatch();

    // Retire hop 2 (ChainB -> ChainC, 20 cm) with a SEEDED boundary carry of 5 cm so the
    // FINAL hop's measured-carry subtraction shrinks it to 8 - 5 = 3 cm < TSHAPE_MIN_MOVE_CM
    // (the ADR-059 degraded-stub trigger for a chain-final hop).
    driveChainHopExact(kMainHopDistCm[2], 5.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nC);

    // Dispatch the FINAL hop (ChainC -> ChainD). It shrinks below the profile floor, so it
    // degrades to the one-shot FLAT `MoveSyncDegs` path — NOT the continuous profiled
    // `MoveSync`. Capture both counters so the assertion proves WHICH path fired.
    const uint32_t syncBefore = MiniR4.DriveDC.moveSyncCallCount;
    const uint32_t degsBefore = MiniR4.DriveDC.moveSyncDegsCallCount;
    tickUntilBlindDispatch();

    ASSERT_GT(MiniR4.DriveDC.moveSyncDegsCallCount, degsBefore);    // degraded flat dispatch fired
    ASSERT_EQ(MiniR4.DriveDC.moveSyncCallCount, syncBefore);        // NOT a profiled MoveSync

    // THE AC3 CORE CLAIM (T4): a FINAL degraded stub crawls at the motor FLOOR, braked —
    // NOT the plan start PWM (~67) that would brake from speed into the wall. Forward hop, so
    // the floor is +PWM_MIN_MAGNITUDE.
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncDegsL, Config::PWM_MIN_MAGNITUDE);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncDegsR, Config::PWM_MIN_MAGNITUDE);
    ASSERT_TRUE(MiniR4.DriveDC.lastMoveSyncDegsBrake);    // the FINAL stub still brakes (floor->rest)

    // Falsifiability against the pre-T4 dispatch: the old stub emitted `signedStartSpeed`
    // (== plan[3].startPwm) here. Whenever the plan start differs from the floor, assert the
    // dispatched value is the floor and NOT the plan start — so a revert to the old
    // `stubPwm = signedStartSpeed` reddens this test.
    ASSERT_TRUE(plan[3].brake);    // the run's final hop is the braked stop
    if (plan[3].startPwm != Config::PWM_MIN_MAGNITUDE)
    {
        ASSERT_TRUE(MiniR4.DriveDC.lastMoveSyncDegsL != plan[3].startPwm);
    }
    ASSERT_FALSE(PathTracker.hasFault());
}

void test_stub_guard_mid_hop_holds_exit()
{
    TEST_SCOPE("TC1b/AC3: chained MID hop below TSHAPE_MIN_MOVE_CM still dispatches chainEndPwm, unbraked");

    const uint8_t nA = nodeByName("ChainA");

    seatAndDispatchMainChain();    // hop 0 (final -> ChainA) in flight.

    ChainHopSpeed plan[4];
    predictChainPlan(kMainHopDistCm, kMainHopCount, Config::SPEED_NORMAL, Config::TSHAPE_START_SPEED,
                     plan);

    // Retire hop 0 (final -> ChainA, 25 cm) with a LARGE seeded boundary carry of 16 cm so the
    // MID hop 1 (ChainA -> ChainB, 20 cm) shrinks to 20 - 16 = 4 cm < TSHAPE_MIN_MOVE_CM. Hop 1
    // is a chained MID hop (brake == false), so its degraded stub holds `chainEndPwm`, NOT the
    // floor — T4 changed only the FINAL arm.
    driveChainHopExact(kMainHopDistCm[0], 16.0f);
    ASSERT_EQ(Localization.getCurrentNode(), nA);

    const uint32_t syncBefore = MiniR4.DriveDC.moveSyncCallCount;
    const uint32_t degsBefore = MiniR4.DriveDC.moveSyncDegsCallCount;
    tickUntilBlindDispatch();    // dispatch hop 1's degraded MID stub.

    ASSERT_GT(MiniR4.DriveDC.moveSyncDegsCallCount, degsBefore);    // degraded flat dispatch fired
    ASSERT_EQ(MiniR4.DriveDC.moveSyncCallCount, syncBefore);

    // A MID stub holds its planned exit PWM and does NOT brake (the run continues) — assert the
    // real flat setpoint equals the chain plan's endPwm for hop 1, un-braked.
    ASSERT_FALSE(plan[1].brake);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncDegsL, plan[1].endPwm);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncDegsR, plan[1].endPwm);
    ASSERT_FALSE(MiniR4.DriveDC.lastMoveSyncDegsBrake);    // mid stub never brakes
    ASSERT_FALSE(PathTracker.hasFault());
}

// ---------------------------------------------------------------------------
// TC5 (AC5) — a fresh goTo's origin bind resets the generalized carry, so the new
// goTo's first braked blind hop commands its FULL distance. Asserted via the robust
// MoveSync-vs-MoveSyncDegs observable: goTo #1's braked blind hop SNAPSHOTs a HUGE
// stale carry baseline (over-driven encoders), then goTo #2's first hop must dispatch
// a FULL-distance profiled MoveSync — a leaked stale carry would shrink the 8 cm hop
// below TSHAPE_MIN_MOVE_CM into the degraded MoveSyncDegs path. The within-goTo
// continuation (carry NOT reset between a single goTo's consecutive braked blind hops)
// is the POSITIVE path exercised by TC2 / the in-chain TC8.
// ---------------------------------------------------------------------------

void test_carry_reset_across_origin_bind()
{
    TEST_SCOPE("TC5/AC5: fresh-goTo origin bind resets the carry -> next hop commands FULL distance");

    const uint8_t nC = nodeByName("ChainC");
    const uint8_t nD = nodeByName("ChainD");

    resetWorld();
    Serial.clearCapture();
    seatAt("ChainB", 0);    // ChainB -> ChainC departs heading 0 == the seated facing (no pre-turn).

    // goTo #1: ChainB -> ChainC (single braked blind hop to the goal). Retire it with a HUGE overshoot
    // (`expectBlindHop` over-advances the encoder by `kEncoderAdvanceStep`), so the carry baseline it
    // SNAPSHOTs is far larger than any 8 cm hop — a stale carry that LEAKED across the goTo boundary
    // would clamp ChainC->ChainD below TSHAPE_MIN_MOVE_CM into the degraded path.
    PathTracker.goTo(nC).async();
    tickUntilBlindDispatch();
    expectBlindHop(kMainHopDistCm[2]);
    ASSERT_EQ(Localization.getCurrentNode(), nC);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());

    // goTo #2 (SEPARATE goTo): ChainC -> ChainD (8 cm). The IDLE->PLAN_NEXT_HOP origin bind
    // (`_firstHopOfGoTo`) must reset the carry, so this hop commands its FULL 8 cm -> a PROFILED
    // MoveSync (8 > TSHAPE_MIN_MOVE_CM), NOT the degraded MoveSyncDegs a leaked huge stale carry
    // would force.
    const uint32_t syncBefore = MiniR4.DriveDC.moveSyncCallCount;
    const uint32_t degsBefore = MiniR4.DriveDC.moveSyncDegsCallCount;
    PathTracker.goTo(nD).async();
    tickUntilBlindDispatch();
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, syncBefore);        // profiled -> FULL distance (reset OK)
    ASSERT_EQ(MiniR4.DriveDC.moveSyncDegsCallCount, degsBefore);    // NOT degraded (no stale carry)
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);    // cold-start braked seed
    ASSERT_FALSE(PathTracker.hasFault());
}

// ---------------------------------------------------------------------------
// TC3 (AC5) — a real pre-edge TURN between two SAME-DIRECTION braked blind hops resets
// the carry, so the post-turn hop commands its FULL distance. Same MoveSync-vs-
// MoveSyncDegs observable: hop 1 SNAPSHOTs a huge stale carry; the turn at `final` must
// INVALIDATE it (the ADR-079 turn reset in `_dispatchTurnIfNeeded`, PathTracker.cpp:2434)
// so hop 2 (final -> ChainA, 25 cm) dispatches a full-distance profiled MoveSync.
//
// ROUTE CHOICE (T6 review fix): both hops must resolve the SAME travel direction (FRONT),
// because the carry READ's DIRECTION-FLIP branch (PathTracker.cpp:3440-3452) zeroes the
// carry independently of the turn reset — a REAR hop 2 (the previous
// `final -> position1 -> Wall1` route, whose Wall1 wall-press approach resolves REAR)
// made this test VACUOUS: deleting the turn reset still passed. With both hops FRONT the
// READ takes the SUBTRACT branch, so ONLY the turn reset stands between the stale carry
// and hop 2's command — deleting it shrinks hop 2 below TSHAPE_MIN_MOVE_CM into the
// degraded MoveSyncDegs path and the asserts below fail.
// `Wall2 -> final -> ChainA` is that route: both edges BLIND SYNC-braked with no
// direction override, and the bend at `final` (~+48.8 deg) sits inside
// (NAV_TURN_TOLERANCE_DEG, BIDIRECTIONAL_TURN_THRESHOLD_DEG] — big enough to dispatch a
// REAL pre-edge turn, small enough that this 2-board host build's AUTO direction
// heuristic (DirectionPolicy.h step 5: REAR iff |delta| > 90) keeps hop 2 FRONT. Only
// hop 2's DISPATCH is observed — the goal arrival at ChainA is out of scope.
// ---------------------------------------------------------------------------

void test_carry_reset_across_turn()
{
    TEST_SCOPE("TC3/AC5: a pre-edge turn between two braked blind hops resets the carry -> hop 2 FULL");

    const uint8_t wall2  = nodeByName("Wall2");
    const uint8_t final_ = nodeByName("final");
    const uint8_t chainA = nodeByName("ChainA");
    ASSERT_TRUE(wall2 != FieldMap::NODE_NONE && final_ != FieldMap::NODE_NONE);
    ASSERT_TRUE(chainA != FieldMap::NODE_NONE);

    resetWorld();
    Serial.clearCapture();

    // goTo(ChainA) routes Wall2 -> final -> ChainA (the Chain spur hangs off `final`, and
    // Wall2's only edge into the main graph is Wall2 -> final). Pin the two-hop route so a
    // fixture re-weight fails loudly rather than binding this test on a different path.
    ASSERT_EQ(routedFirstHop(wall2, chainA), final_);
    ASSERT_EQ(routedFirstHop(final_, chainA), chainA);
    const Routes::EdgeInfo& e1 = FieldMap::getEdge(wall2, final_);
    const Routes::EdgeInfo& e2 = FieldMap::getEdge(final_, chainA);
    ASSERT_TRUE(e1.type == Routes::EdgeType::BLIND);
    ASSERT_TRUE(e2.type == Routes::EdgeType::BLIND);

    // SAME-DIRECTION premise (see the ROUTE CHOICE block above): no per-edge direction
    // override on either hop, and the bend at `final` is a REAL turn (> tolerance) that
    // stays at or below the AUTO rear threshold — so BOTH hops resolve FRONT and the carry
    // READ takes the SUBTRACT branch (the DIRECTION-FLIP branch cannot co-guard the reset).
    ASSERT_TRUE(e1.direction_override == Routes::DirectionMode::DEFAULT);
    ASSERT_TRUE(e2.direction_override == Routes::DirectionMode::DEFAULT);
    const float bend = wrapDeltaDeg(e2.heading_deg - e1.heading_deg);
    ASSERT_TRUE(fabsf(bend) > Config::NAV_TURN_TOLERANCE_DEG);
    ASSERT_TRUE(fabsf(bend) <= Config::BIDIRECTIONAL_TURN_THRESHOLD_DEG);
    // Hop 2's FULL command must be profiled (> TSHAPE_MIN_MOVE_CM), so a leaked stale carry
    // (which floors dispatchCm at PIVOT_TRIM_MIN_DRIVE_CM) is observable as the degraded
    // MoveSyncDegs path.
    ASSERT_TRUE(e2.distance_cm > Config::TSHAPE_MIN_MOVE_CM);

    // Seat at Wall2 (world frame binds the seated facing to heading 0). Seating ON the
    // Wall2 RESET node is safe: the wall-zero interlude is gated `!firstPlan`
    // (PathTracker.cpp:1997), so it never fires on a goTo's start node.
    seatAt("Wall2");

    PathTracker.goTo(chainA).async();
    // Park the corrected IMU read at hop 1's heading (~311) BEFORE the first plan tick, so
    // hop 1 (Wall2 -> final, SYNC braked blind) sees delta ~0.19 < NAV_TURN_TOLERANCE_DEG
    // and dispatches its move DIRECTLY — the ONLY turn in this test is the
    // carry-invalidating one at `final` (hop 2's plan).
    mmL.mockYawDeg = (double) hwYaw((int16_t) lroundf(e1.heading_deg));
    tickUntilBlindDispatch();    // dispatch hop 1 (profiled MoveSync; SNAPSHOTs the carry baseline)

    // Retire hop 1 with a HUGE overshoot (each tick over-advances by kEncoderAdvanceStep),
    // so the SNAPSHOTed carry baseline is far larger than hop 2's 25 cm — a failed turn
    // reset would shrink hop 2 below TSHAPE_MIN_MOVE_CM into the degraded path.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Localization.getCurrentNode() != final_;
         ++i)
    {
        MiniR4.M2.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.M3.mockAdvanceDegrees(path_harness::kEncoderAdvanceStep);
        MiniR4.DriveDC.mockSetPrevTaskDone(true);
        cooperativeTick();
    }
    ASSERT_EQ(Localization.getCurrentNode(), final_);

    // hop 2 = final -> ChainA. CRITICAL sequencing: the FSM plans hop 2 on the NEXT tick;
    // the belief heading is STILL hop 1's (~311 — the SYNC hop never touched the mock IMU),
    // so the plan sees the real ~49 deg delta to hop 2's FRONT body heading (0) and
    // DISPATCHES the carry-invalidating pre-edge turn. Tick until it is in flight.
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && Drive.isIdle() && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_FALSE(Drive.isIdle());    // a REAL pre-edge turn dispatched (its site invalidated the carry)

    // Settle the turn at hop 2's FRONT body heading (== e2.heading_deg, 0), then tick until
    // hop 2's MOVE dispatches. Poll BOTH move counters so the assertion reads WHICH dispatch
    // path hop 2 took (the turn runs Move(L,R), never a MoveSync/MoveSyncDegs, so neither
    // counter moves until hop 2's move itself dispatches).
    const uint32_t syncBefore = MiniR4.DriveDC.moveSyncCallCount;
    const uint32_t degsBefore = MiniR4.DriveDC.moveSyncDegsCallCount;
    mmL.mockYawDeg            = (double) hwYaw((int16_t) lroundf(e2.heading_deg));
    for (int i = 0; i < path_harness::kMaxHandshakeTicks
                    && MiniR4.DriveDC.moveSyncCallCount == syncBefore
                    && MiniR4.DriveDC.moveSyncDegsCallCount == degsBefore && !PathTracker.hasFault();
         ++i)
    {
        cooperativeTick();
    }
    ASSERT_FALSE(PathTracker.hasFault());
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, syncBefore);        // profiled -> FULL 25 cm (carry reset by the turn)
    ASSERT_EQ(MiniR4.DriveDC.moveSyncDegsCallCount, degsBefore);    // NOT degraded (no stale carry survived the turn)
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);    // FRONT cold-start braked seed
}

// ---------------------------------------------------------------------------
// TC4 (AC5) — a fault -> clear drops the carry, so the next goTo's braked blind hop
// commands its FULL distance. `goTo(RevA)` from the chain spur is UNREACHABLE (the
// RevA..RevD cluster is an isolated graph component), latching `Fault::UNREACHABLE`
// whose `_enterFault` drops the carry baseline (and clears the queue). Same
// MoveSync-vs-MoveSyncDegs observable on the post-clear hop.
//
// NOTE: the fault reset and the following fresh goTo's origin-bind reset CO-GUARD this
// hop, so a dispatch observable cannot ISOLATE the fault-specific reset (both drive the
// carry to inactive). What this test DOES guarantee — the end-to-end property AC5 asks
// for — is that no stale carry survives a fault -> clear -> new goTo.
// ---------------------------------------------------------------------------

void test_carry_reset_across_fault_then_clear()
{
    TEST_SCOPE("TC4/AC5: a fault->clear drops the carry -> the next goTo's braked blind hop is FULL");

    const uint8_t nC   = nodeByName("ChainC");
    const uint8_t nD   = nodeByName("ChainD");
    const uint8_t revA = nodeByName("RevA");    // isolated cluster -> UNREACHABLE from the chain spur
    ASSERT_TRUE(revA != FieldMap::NODE_NONE);

    resetWorld();
    Serial.clearCapture();
    seatAt("ChainB", 0);

    // goTo #1: ChainB -> ChainC (braked blind, huge overshoot) SNAPSHOTs a large carry baseline, then
    // IDLE at ChainC. The carry stays active across goal-arrival IDLE (its reset is deferred to the
    // next goTo's origin bind by design) — so the fault below is what must drop it here.
    PathTracker.goTo(nC).async();
    tickUntilBlindDispatch();
    expectBlindHop(kMainHopDistCm[2]);
    ASSERT_EQ(Localization.getCurrentNode(), nC);
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());

    // Induce a fault: goTo(RevA) is UNREACHABLE from ChainC, so PLAN_NEXT_HOP latches
    // Fault::UNREACHABLE. `_enterFault` drops the carry baseline and clears the queue.
    PathTracker.goTo(revA).async();
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.hasFault(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.hasFault());
    PathTracker.clearFault();
    ASSERT_FALSE(PathTracker.hasFault());

    // Post-clear goTo: ChainC -> ChainD (8 cm) commands its FULL distance -> a profiled MoveSync, NOT
    // the degraded MoveSyncDegs a leaked huge stale carry would force.
    const uint32_t syncBefore = MiniR4.DriveDC.moveSyncCallCount;
    const uint32_t degsBefore = MiniR4.DriveDC.moveSyncDegsCallCount;
    PathTracker.goTo(nD).async();
    tickUntilBlindDispatch();
    ASSERT_GT(MiniR4.DriveDC.moveSyncCallCount, syncBefore);
    ASSERT_EQ(MiniR4.DriveDC.moveSyncDegsCallCount, degsBefore);
    ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, Config::TSHAPE_START_SPEED);
    ASSERT_FALSE(PathTracker.hasFault());
}

// ---------------------------------------------------------------------------
// TC2 (AC4, chaining OFF) — a single goTo walking >=3 consecutive braked blind hops,
// each seeded a boundary carry, telescopes via the GENERALIZED (non-chain) measured-
// carry subtraction so no drift accumulates over N hops. Same realistic-step driver +
// falsifiable step-count proxy as the in-chain TC8 (`test_boundary_carry_subtracted_...`),
// but compiled with chaining OFF so the 4 collinear hops are SEPARATE braked blind
// dead-reckons — the exact path T5 generalizes ADR-059's in-chain carry onto. Runs ONLY
// on the disabled build.
// ---------------------------------------------------------------------------

void test_chaining_off_multi_hop_carry_no_drift()
{
    TEST_SCOPE("TC2/AC4: chaining OFF -- generalized carry subtracted across braked blind hops, no drift");

    const uint8_t nA = nodeByName("ChainA");
    const uint8_t nB = nodeByName("ChainB");
    const uint8_t nC = nodeByName("ChainC");
    const uint8_t nD = nodeByName("ChainD");

    seatAndDispatchMainChain();    // goTo(ChainD); chaining OFF -> hop 0 is a plain braked blind hop.

    // Seed a genuine ~6 cm boundary carry (2 trailing ~3 cm ticks) after each hop so the NEXT hop's
    // dispatch measures + subtracts it via the GENERALIZED path. Each hop lands EXACTLY on its target
    // node (`driveChainHopRealistic` asserts that internally), so no drift accumulates over the climb.
    driveChainHopRealistic(nA, /*extraCarryTicks*/ 2);
    const int hop1Steps = driveChainHopRealistic(nB, /*extraCarryTicks*/ 2);
    driveChainHopRealistic(nC, /*extraCarryTicks*/ 2);

    // FALSIFIABILITY (identical to the in-chain TC8): hop 1 (ChainA->ChainB, 20 cm) measures the ~6 cm
    // carry hop 0 rolled and subtracts it (generalized non-chain path), commanding ~14 cm -> 6 steps;
    // the UNCARRIED 20 cm edge needs 7. Strip the generalized subtraction and hop 1 commands the full
    // 20 cm -> 7 steps -> the strict GT below fails. This is the proxy AC4 names (arrival step-count
    // bound + exact final-node landing).
    constexpr int kUncarriedHop1Steps  = 7;    // == uncarried count; strict GT fails at 7, passes at carried 6.
    constexpr int kCarriedHop1MaxSteps = 6;    // the measured carry-adjusted budget.
    ASSERT_GT(kUncarriedHop1Steps, hop1Steps);
    ASSERT_GE(kCarriedHop1MaxSteps, hop1Steps);

    // Final hop: land exactly on ChainD, braked (profiled brake OR sub-TSHAPE_MIN_MOVE_CM one-shot).
    driveChainHopRealistic(nD, /*extraCarryTicks*/ 0);
    ASSERT_TRUE(MiniR4.DriveDC.lastBrake || MiniR4.DriveDC.lastMoveSyncDegsBrake);
    ASSERT_FALSE(PathTracker.hasFault());
    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

// ---------------------------------------------------------------------------
// TC8 (AC5 handoff) — the chain-retire ordering pin: a multi-hop chain retires, then a
// following NON-chain braked blind hop within the SAME goTo telescopes off the chain-
// final SNAPSHOT baseline (the SNAPSHOT survives `_chainReset`; the carry is read ONCE,
// no double-subtract). Runs on the MAX_CHAIN_HOPS=2 build — the only host build where a
// multi-hop chain retires at an INTERIOR node followed by a plain braked blind hop within
// one goTo (the uncapped default build always chains straight to the goal).
// ---------------------------------------------------------------------------

void test_post_chain_handoff_telescopes()
{
    TEST_SCOPE("TC8/AC5: chain retires (cap) -> post-chain braked blind hop telescopes off chain-final carry");

    const uint8_t nFinal = nodeByName("final");
    const uint8_t nA     = nodeByName("ChainA");
    const uint8_t nB     = nodeByName("ChainB");
    const uint8_t nC     = nodeByName("ChainC");

    resetWorld();
    Serial.clearCapture();
    seatAt("final", 0);

    // MAX_CHAIN_HOPS=2: goTo(ChainC) forms the chain final->ChainA->ChainB (2 hops), which RETIRES at
    // ChainB (cap trip); then ChainB->ChainC is the goal's single remaining hop -> a NON-chain braked
    // blind hop that telescopes off the chain-final (ChainA->ChainB) SNAPSHOT. Pin the route.
    ASSERT_EQ(routedFirstHop(nFinal, nC), nA);
    ASSERT_EQ(routedFirstHop(nA, nC), nB);
    ASSERT_EQ(routedFirstHop(nB, nC), nC);

    PathTracker.goTo(nC).async();
    tickUntilBlindDispatch();    // dispatch chain hop 0 (final -> ChainA).

    // Retire the chain: hop 0 (mid) with no carry, hop 1 (ChainA->ChainB, the chain-FINAL braked hop)
    // with a seeded ~6 cm boundary carry so the post-chain ChainB->ChainC hop measures it off the
    // chain-final baseline (which `_chainReset` must LEAVE intact — the chain-retire ordering pin).
    driveChainHopRealistic(nA, /*extraCarryTicks*/ 0);
    driveChainHopRealistic(nB, /*extraCarryTicks*/ 2);

    // The post-chain hop ChainB->ChainC (20 cm) telescopes: its command is reduced by the ~6 cm carry,
    // so it arrives in FEWER ~3 cm steps than the raw 20 cm edge (7). If the chain-retire ordering pin
    // were broken (the SNAPSHOT wiped by `_chainReset`), the hop would command the full 20 cm -> 7
    // steps -> the strict GT fails. Landing exactly on ChainC confirms no double-subtract drift.
    const int hopSteps = driveChainHopRealistic(nC, /*extraCarryTicks*/ 0);
    constexpr int kUncarriedSteps  = 7;    // ceil(20/3); strict GT fails at the uncarried count.
    constexpr int kCarriedMaxSteps = 6;    // the carry-adjusted budget (matches the in-chain TC8).
    ASSERT_GT(kUncarriedSteps, hopSteps);
    ASSERT_GE(kCarriedMaxSteps, hopSteps);
    ASSERT_EQ(Localization.getCurrentNode(), nC);

    for (int i = 0; i < path_harness::kMaxHandshakeTicks && !PathTracker.isIdle(); ++i)
    {
        cooperativeTick();
    }
    ASSERT_TRUE(PathTracker.isIdle());
    ASSERT_FALSE(PathTracker.hasFault());
}

// ---------------------------------------------------------------------------
// AC5 scenarios NOT covered by a dedicated dispatch-observable test above, and why —
// each needs fixture geometry the single-sourced `field_fixture.py` does not author.
// The PRODUCTION invalidation sites for all of these ARE present (PathTracker.cpp) and
// are exercised structurally by the tests above / the existing TC6c ineligibility probes;
// what is missing is a route that puts a SECOND braked blind hop AFTER the event within
// one goTo so its full-distance dispatch is observable.
//
// TODO(T6): non-blind LINE/PROJECTION hop BETWEEN two braked blind hops (AC5 "a non-blind
//   hop") — needs a blind -> LINE/proj -> blind route in one goTo. The fixture's LINE/proj
//   edges either start a run (LpIn->LpJoin->LpBlind*) or terminate at the goal, so no blind
//   hop 3 follows the non-blind hop to observe the reset. The invalidation itself fires at
//   PathTracker.cpp:3126 (LINE) / :3171 (PROJECTION) / :1922 (line-arrival re-anchor).
//
// TODO(T6): STOP_AT_LINE / seek hop BETWEEN two braked blind hops (AC5 "a seek arrival") —
//   `SeekPre->Start->Inter1` reaches the seek but Inter1 is the goal, so there is no braked
//   blind hop after the seek to observe. Invalidation fires at PathTracker.cpp:3241.
//
// TODO(T6): wall/line RE-ANCHOR between two braked blind hops (AC5 "a wall/line re-anchor")
//   — `ResetPre->Wall2->ResetBeyond` is the route, but driving PAST the Wall2 RESET node
//   requires retiring the multi-phase wall-zero interlude (`expectWallStop`) mid-goTo; the
//   invalidation fires at PathTracker.cpp:1415. (The turn-reset TC3 above only DEPARTS
//   from the Wall2 reset node — the interlude is `!firstPlan`-gated off a goTo's start
//   node, so TC3 never reaches this path.)
//
// TODO(T6): DIRECTION-FLIP with NO turn (AC5 "a REAR after a FRONT with no turn", TC6) —
//   the re-arm branch (PathTracker.cpp:3424-3452) fires ONLY when a REAR hop follows a FRONT
//   hop WITHOUT a pre-edge turn, i.e. when the two edges' headings differ by 180 (the body
//   faces the same way, driving backward). The fixture's only direction-flip pair
//   (DirFlipB->DirFlipC, forced REAR, collinear heading 0) turns the body 180 first, so the
//   pre-edge-turn invalidation (:2434) pre-empts the re-arm branch. Exercising the re-arm
//   needs an authored A->B (FRONT, heading H) then B->C (REAR, heading H+180) pair.
//
// TODO(T6): CHAIN-DIVERGENCE fallback (PathTracker.cpp:3366-3389, TC7) — the divergence
//   branch is "unreachable by construction on a static NEXT_HOP table" (its own comment); a
//   real FSM drive cannot reach it without corrupting the run buffer, so its ZERO-stale-carry
//   invalidation (:3384) has no host-reachable trigger. It is guarded by code review + the
//   explicit `_brakedBlindCarryActive = false` at the divergence site.
// ---------------------------------------------------------------------------

// ===========================================================================
// Entry.
// ===========================================================================

int main()
{
    std::printf("INFO: PathTracker ADR-059 blind-chain FSM (T10, TC4-TC8) -- "
                "real PathTracker + Drive + Localization over routes_fixture\n");

    test_routing_guards();

    // `test_main_chain_dispatch()` assumes the SHIPPED MAX_CHAIN_HOPS /
    // PATH_BLIND_CHAIN_MAX_CM defaults (an UNCAPPED 4-hop run) -- it must NOT run
    // under the capped builds (#26c/#26d), which drive their OWN dedicated body
    // asserting the truncation instead. It DOES run for both the default build
    // (TC4, chaining ON) and the disabled build (TC5, chaining OFF) since the
    // function itself branches on `Config::PATH_BLIND_CHAIN_ENABLED`.
    //
    // TC6c/TC7/TC8 all assume chaining is genuinely ACTIVE (they assert chain
    // formation / coast-not-brake mid-hop behavior), so they run ONLY on the
    // default build (#26a) -- never under #26b (disabled), #26c, or #26d.
#if defined(BLIND_CHAIN_TEST_HOP_CAP)
    test_hop_cap_truncates_and_reforms();
    // ADR-079 TC8 (AC5 handoff): the MAX_CHAIN_HOPS=2 build is the ONLY host build in which a
    // multi-hop chain retires at an INTERIOR node followed by a NON-chain braked blind hop within
    // ONE goTo (the uncapped default build always chains all the way to the goal, so no post-chain
    // hop exists). It reuses this cap build to exercise the chain-retire-ordering pin: the
    // chain-final SNAPSHOT survives `_chainReset` and the following non-chain hop telescopes off it.
    test_post_chain_handoff_telescopes();
#elif defined(BLIND_CHAIN_TEST_DISTANCE_CAP)
    test_distance_cap_truncates_and_reforms();
#else
    test_main_chain_dispatch();
#if defined(BLIND_CHAIN_TEST_DISABLED)
    // ADR-079 TC2 (AC4): chaining compiled OFF — a single goTo walking >=3 consecutive braked
    // blind hops, each seeded a boundary carry, telescopes via the GENERALIZED (non-chain)
    // measured-carry subtraction so no drift accumulates over N hops. Runs ONLY on the disabled
    // build (the generalized per-hop path is what chaining-off exercises; the enabled build would
    // instead form a chain and take the in-`_chain.active` path).
    test_chaining_off_multi_hop_carry_no_drift();
#else
    test_ineligible_goal_ends_walk_before_it_starts();
    test_ineligible_short_hop_below_profile_floor();
    test_ineligible_pivot_ends_walk();
    test_ineligible_reset_node_ends_walk();
    test_ineligible_seek_node_ends_walk();
    test_ineligible_direction_flip_ends_walk();
    test_ineligible_non_collinear_bend_ends_walk();
    test_line_to_blind_spanning_run_seeds_captured_cruise();
    test_boundary_carry_subtracted_no_drift_with_n();
    // ADR-079 T6 (AC3, AC5) — the parity-invisible T4 stub-guard + T5 generalized-carry guards,
    // asserted on the REAL dispatched setpoint. These run on the default (chaining-ON) build.
    test_stub_guard_final_hop_floor();          // TC1a (AC3)
    test_stub_guard_mid_hop_holds_exit();       // TC1b (AC3)
    test_carry_reset_across_turn();             // TC3  (AC5)
    test_carry_reset_across_fault_then_clear(); // TC4  (AC5)
    test_carry_reset_across_origin_bind();      // TC5  (AC5)
#endif
#endif

    TEST_REPORT();
}
