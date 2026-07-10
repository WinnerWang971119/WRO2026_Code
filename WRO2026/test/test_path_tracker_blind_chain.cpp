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
 *   - **Brake vs coast at a hop boundary.** `_updateProfile()`'s DONE branch
 *     always calls `MiniR4.DriveDC.Move(0,0)` then `MiniR4.DriveDC.brake(flag)`
 *     at profile completion — for BOTH a true braked stop (mid-hop `flag` would
 *     be `_profileBrake`, which the dispatch sets `false` for a chained mid-hop
 *     via `.setBrake(false)`) and the "chained end, queue empty" coast-release
 *     path (Drive.cpp finding #13, `MiniR4.DriveDC.brake(false)`). So the
 *     precise TC4 observable for "no inter-hop STOP" is `lastBrake == false`
 *     at every mid-hop boundary (the chained-end path emits `Move(0,0)` then
 *     `brake(false)` — a freewheel RELEASE, not a held setpoint; the chassis
 *     keeps rolling from inertia across the boundary until the next chained hop
 *     re-drives the wheels) and `lastBrake == true` ONLY at the run's FINAL hop.
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
///     — NOT the braked-default `Config::TSHAPE_END_SPEED` (0). With
///     `startSpd == plan[k].startPwm == cruise` (the backward pass leaves the final
///     hop's entry at cruise here) and `endSpd == PWM_MIN_MAGNITUDE`, the decel
///     `(cruise − PWM_MIN)/accel = 7 cm` fits the 8 cm hop, so the accel ramp is
///     zero and `traveled == 0` is BELOW the decel-zone entry
///     (`totalCm − decelCm`): `tShapeSpeedAt(0)` returns `peakPwm == cruise ==
///     plan[k].startPwm`. So the final hop LAUNCHES at the plan start (no
///     triangle-collapse re-dip — the T13 bug fix). Asserting `plan[k].startPwm`
///     directly is exact AND honest here (the launch IS the plan start), so this
///     helper short-circuits to it rather than re-dipping through the profile math
///     with a stale (endSpd=0) end target — that stale target is exactly the
///     pre-T13 model that predicted the wrong 56.
///
/// This helper is the ONLY correct oracle for "what does the FSM actually emit on
/// hop k's first tick"; for a mid-hop it must run the profile, never hand-assert
/// `plan[k].startPwm`.
int16_t predictHopDispatchPwm(const float* hopDistCm, const ChainHopSpeed* plan, uint8_t k,
                              int16_t cruisePwm)
{
    if (plan[k].brake)
    {
        // Chain-final braked hop: T13 dispatches setEndSpeed(±PWM_MIN_MAGNITUDE), so the
        // (cruise−PWM_MIN)/accel shed fits and the launch equals the plan start exactly.
        return plan[k].startPwm;
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

    // THE NO-STOP ASSERT (TC4 core claim): a brake call DID fire (Drive's
    // empty-queue chained-end finalize always calls Move(0,0)+brake(flag) --
    // Drive.cpp finding #13), but with `flag == false` (coast, not stop) for a
    // chained mid-hop; TC5 (chaining disabled) brakes for real (flag == true).
    ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop0);
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        ASSERT_FALSE(MiniR4.DriveDC.lastBrake);
    }
    else
    {
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
    ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop1);
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        ASSERT_FALSE(MiniR4.DriveDC.lastBrake);
    }
    else
    {
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
    ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeHop2);
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        ASSERT_FALSE(MiniR4.DriveDC.lastBrake);
    }
    else
    {
        ASSERT_TRUE(MiniR4.DriveDC.lastBrake);
    }
    tickUntilBlindDispatch();

    // ---- Hop 3 (FINAL): ChainC -> ChainD. -----------------------------------
    // The final hop is the cruise->floor->brake stop: it starts at the plan's start
    // speed (the backward pass leaves it at cruise 85 here) and completes with a true
    // brake. T13 dispatches `.setEndSpeed(±PWM_MIN_MAGNITUDE)` (floor, not 0) so the
    // (cruise−PWM_MIN)/accel = 7 cm shed fits the 8 cm hop -- the hop LAUNCHES at the
    // plan start (85), sheds cruise->floor, and the brake completes floor->rest. So the
    // recorded launch is `plan[3].startPwm` (85), NOT the pre-T13 triangle-collapse 56;
    // predictHopDispatchPwm short-circuits a braked final hop to `plan[k].startPwm`.
    if (Config::PATH_BLIND_CHAIN_ENABLED)
    {
        const int16_t expected3 = predictHopDispatchPwm(kMainHopDistCm, plan, 3, Config::SPEED_NORMAL);
        ASSERT_EQ(MiniR4.DriveDC.lastMoveSyncL, expected3);
        ASSERT_TRUE(plan[3].brake);
        ASSERT_EQ(expected3, plan[3].startPwm);    // T13: final hop launches at the plan start (85).
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
    ASSERT_GT(MiniR4.DriveDC.brakeCallCount, brakeCallsBeforeMid);
    ASSERT_FALSE(MiniR4.DriveDC.lastBrake);    // mid-hop of the re-formed chain -- coast.
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

    driveChainHopExact(hopDist[0], 0.0f);
    ASSERT_EQ(Localization.getCurrentNode(), lpA);
    ASSERT_FALSE(MiniR4.DriveDC.lastBrake);    // mid-hop of the spanning chain -- coast.
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
#elif defined(BLIND_CHAIN_TEST_DISTANCE_CAP)
    test_distance_cap_truncates_and_reforms();
#else
    test_main_chain_dispatch();
#if !defined(BLIND_CHAIN_TEST_DISABLED)
    test_ineligible_goal_ends_walk_before_it_starts();
    test_ineligible_short_hop_below_profile_floor();
    test_ineligible_pivot_ends_walk();
    test_ineligible_reset_node_ends_walk();
    test_ineligible_seek_node_ends_walk();
    test_ineligible_direction_flip_ends_walk();
    test_ineligible_non_collinear_bend_ends_walk();
    test_line_to_blind_spanning_run_seeds_captured_cruise();
    test_boundary_carry_subtracted_no_drift_with_n();
#endif
#endif

    TEST_REPORT();
}
