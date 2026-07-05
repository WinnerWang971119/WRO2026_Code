// =========================================================================================
// Host unit test for resolveReturnAction (ReturnToNodePolicy.h, issue #36 / PR1b, T8/T10).
//
// `resolveReturnAction` is the return-to-node GATE decision — the riskiest decision in
// PR1b (live off-map motion + fault gating under ADR-001). It was promoted to a header-
// only inline free function (pure float / enum, no Arduino / Drive / Localization
// dependency) exactly like `DirectionPolicy.h::resolveDirection`,
// `StopAtLinePolicy.h::resolveStopAtLine`, and `PivotTrimPolicy.h::resolvePivotTrim`, so
// production (`PathTracker::_evaluateReturnGate`) AND this host test link the SAME symbol
// via `#include` — the gate boundaries cannot drift from their test coverage.
//
// This file is the TC4/TC5/TC6/TC7 (+ AC4/AC5/AC6/AC7-gate) coverage layer, copying the
// `test_pivot_trim_policy.cpp` pure-policy template: it links no `mocks.cpp` and no SUT
// source. Unlike PivotTrimPolicy.h, `ReturnToNodePolicy.h` includes `Config.h` for the
// `Config::OffMapGate` enum, and `Config.h` pulls in `<Arduino.h>`, so the CMake target
// puts the `mocks/`-first include path on this TU to resolve that angle-bracket header to
// `mocks/Arduino.h`. Nothing here odr-uses a mock global (no `Serial` / `millis()` call), so
// no mock object is linked — the policy itself never touches an Arduino symbol.
//
// The four-case rule under test (mirrors the file docblock in ReturnToNodePolicy.h):
//   1. mag <= triggerCm                       -> NONE            (TC5: on-node; AC5 no-op)
//   2. triggerCm < mag <= capCm               -> RETURN          (TC4: drive inverse first)
//   3. mag > capCm AND gate == FAULT          -> OFF_MAP_FAULT   (TC6: refuse to plan)
//   4. mag > capCm AND gate == BEST_EFFORT    -> OFF_MAP_PROCEED (TC7: log + return anyway)
//
// Boundary semantics asserted explicitly: the trigger is CLOSED (mag == trigger -> NONE)
// and the cap is OPEN (mag == cap -> RETURN; only mag > cap trips the off-map gate). These
// are the exact `<=` / `>` boundaries the Config static_asserts (`0 < trigger <= cap`) and
// the production branch order depend on.
//
// The pass/fail + final-line conventions mirror `test_pivot_trim_policy.cpp` /
// `test_localization_offset.cpp`.
// =========================================================================================

#include <cstdio>

#include "src/nav/ReturnToNodePolicy.h"

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

int main()
{
    // Representative trigger / cap chosen to leave clear room below the trigger, inside the
    // band, and above the cap. The relationship 0 < trigger < cap mirrors the Config
    // invariant (Config::RETURN_TRIGGER_THRESHOLD_CM < RETURN_DISTANCE_CAP_CM), so these
    // values exercise the same branch structure as the production constants.
    const float trigger = 3.0f;    // mirrors Config::RETURN_TRIGGER_THRESHOLD_CM (suggested default).
    const float cap     = 50.0f;   // mirrors Config::RETURN_DISTANCE_CAP_CM (suggested default).

    // -------------------------------------------------------------------------------------
    // TC5 (AC5) — magnitude strictly below the trigger -> NONE (the true no-op path).
    //
    // A small operator nudge that never crosses the trigger leaves goTo() behaving exactly
    // as today: no return move, straight to planning. The off-map gate is irrelevant here
    // (offset is far below the cap), so it must be NONE under EITHER gate policy.
    // -------------------------------------------------------------------------------------
    check(resolveReturnAction(0.0f, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::NONE,
          "TC5: mag 0 (no offset) -> NONE");
    check(resolveReturnAction(1.5f, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::NONE,
          "TC5: mag below trigger -> NONE (FAULT gate)");
    check(resolveReturnAction(1.5f, trigger, cap, Config::OffMapGate::BEST_EFFORT) == ReturnAction::NONE,
          "TC5: mag below trigger -> NONE (gate policy does not matter below trigger)");

    // -------------------------------------------------------------------------------------
    // TC5 boundary — mag == trigger -> NONE (the trigger is CLOSED: only mag > trigger
    // leaves the on-node case). Production checks `offsetMagCm <= triggerCm` first.
    // -------------------------------------------------------------------------------------
    check(resolveReturnAction(trigger, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::NONE,
          "TC5 boundary: mag == trigger -> NONE (trigger is closed, <=)");

    // -------------------------------------------------------------------------------------
    // TC4 (AC4) — trigger < mag <= cap -> RETURN (drive the inverse return onto the node).
    //
    // An offset just past the trigger, and a mid-band offset, both dispatch a return move.
    // The gate policy is irrelevant inside the band (it only fires above the cap), so RETURN
    // must hold under EITHER gate.
    // -------------------------------------------------------------------------------------
    check(resolveReturnAction(3.01f, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::RETURN,
          "TC4: mag just past trigger -> RETURN");
    check(resolveReturnAction(25.0f, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::RETURN,
          "TC4: mid-band mag -> RETURN (FAULT gate, still within cap)");
    check(resolveReturnAction(25.0f, trigger, cap, Config::OffMapGate::BEST_EFFORT) == ReturnAction::RETURN,
          "TC4: mid-band mag -> RETURN (BEST_EFFORT gate, still within cap)");

    // -------------------------------------------------------------------------------------
    // TC4 boundary — mag == cap -> RETURN (the cap is OPEN: only mag > cap trips the
    // off-map gate). Production checks `offsetMagCm > capCm` for the gate, so the cap value
    // itself still dispatches a plain in-trust return.
    // -------------------------------------------------------------------------------------
    check(resolveReturnAction(cap, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::RETURN,
          "TC4 boundary: mag == cap -> RETURN (cap is open, only > cap trips the gate)");

    // -------------------------------------------------------------------------------------
    // TC6 (AC6) — mag > cap AND gate == FAULT -> OFF_MAP_FAULT (refuse to plan).
    //
    // Past the trust cap with the safer default gate, the policy refuses: PathTracker
    // latches Fault::OFF_NODE rather than driving further off-map on an untrustworthy
    // dead-reckon vector.
    // -------------------------------------------------------------------------------------
    check(resolveReturnAction(50.01f, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::OFF_MAP_FAULT,
          "TC6: mag just past cap, FAULT gate -> OFF_MAP_FAULT");
    check(resolveReturnAction(100.0f, trigger, cap, Config::OffMapGate::FAULT) == ReturnAction::OFF_MAP_FAULT,
          "TC6: mag far past cap, FAULT gate -> OFF_MAP_FAULT");

    // -------------------------------------------------------------------------------------
    // TC7 (AC7-gate) — mag > cap AND gate == BEST_EFFORT -> OFF_MAP_PROCEED (return anyway).
    //
    // Same over-cap offsets, but the BEST_EFFORT gate logs a warning and attempts the
    // (capped) return instead of faulting. The cap test runs BEFORE deciding RETURN, so an
    // over-cap offset never silently falls into a plain RETURN — it always routes through
    // the gate and surfaces the OFF_MAP_* outcome.
    // -------------------------------------------------------------------------------------
    check(resolveReturnAction(50.01f, trigger, cap, Config::OffMapGate::BEST_EFFORT) == ReturnAction::OFF_MAP_PROCEED,
          "TC7: mag just past cap, BEST_EFFORT gate -> OFF_MAP_PROCEED");
    check(resolveReturnAction(100.0f, trigger, cap, Config::OffMapGate::BEST_EFFORT) == ReturnAction::OFF_MAP_PROCEED,
          "TC7: mag far past cap, BEST_EFFORT gate -> OFF_MAP_PROCEED");

    // -------------------------------------------------------------------------------------
    // Degenerate trigger == cap (the Config invariant allows `trigger <= cap`, i.e. they may
    // be equal). With trigger == cap there is NO RETURN band: at-or-below is NONE, strictly
    // above immediately hits the gate. Confirms the two boundaries do not overlap or leave a
    // gap when they coincide.
    // -------------------------------------------------------------------------------------
    check(resolveReturnAction(10.0f, 10.0f, 10.0f, Config::OffMapGate::FAULT) == ReturnAction::NONE,
          "edge: trigger == cap, mag == both -> NONE (trigger closed wins)");
    check(resolveReturnAction(10.01f, 10.0f, 10.0f, Config::OffMapGate::FAULT) == ReturnAction::OFF_MAP_FAULT,
          "edge: trigger == cap, mag just above -> OFF_MAP_FAULT (no RETURN band)");
    check(resolveReturnAction(10.01f, 10.0f, 10.0f, Config::OffMapGate::BEST_EFFORT) == ReturnAction::OFF_MAP_PROCEED,
          "edge: trigger == cap, mag just above, BEST_EFFORT -> OFF_MAP_PROCEED");

    if (g_failures == 0)
    {
        std::printf("RETURN TO NODE POLICY OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
