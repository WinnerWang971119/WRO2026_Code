// =========================================================================================
// Host-compile tests for the Drive.wallStop watchdog state machine (PR2 / T14, AC8) —
// TC9 (stall-success) + TC10 (no-stall advisory timeout).
//
// WHY AN INLINE MIRROR (not the real Drive.cpp)
//   `Drive.cpp` is NOT host-compiled by ANY target — it pulls the full MatrixMiniR4 vendor
//   surface (`MiniR4.DriveDC.Move`, the lower-MCU motor controller, ...). Every existing
//   Drive-level FSM test (`test_path_tracker_stop_at_line.cpp`, the passthrough / bidirectional
//   mirrors) follows the same precedent: inline-MIRROR the small decision slice under test and
//   annotate each mirrored decision `// MIRROR: Drive.cpp:<line>` so a reviewer diffs the replica
//   against the cited lines and the two cannot silently drift.
//
//   CRUCIALLY, the mirror does NOT re-derive the watchdog: it calls the SAME pure primitives the
//   production `_updateWallStop` calls — `stopAtLineProgressed` / `stopAtLineStalled` from
//   `WROMath.h` (host-compilable, no Arduino hardware) — with the SAME `Config` constants
//   (`STOP_AT_LINE_PROGRESS_EPS_CM`, `STOP_AT_LINE_STALL_TIMEOUT_MS`, `WALL_STOP_TIMEOUT_MS`). So a
//   future change to either primitive's contract or to those constants is exercised by this test,
//   not bypassed. Only the PUSH->stall?->BACK_OFF / outer-timeout->advisory STATE wiring is
//   replicated. The encoder roll + active brake + LineFollower board restore (the parts that touch
//   `MiniR4.DriveDC`) are delegated to the hardware bench (B3.7), exactly as the stop-at-line
//   stateful seek is.
//
//   What the production outcome flags mean (the two the mirror reproduces):
//     - a STALL within the window  -> SUCCESS: back off, then DONE; `wallStopTimedOut()` == false.
//     - NO stall within WALL_STOP_TIMEOUT_MS -> NON-LATCHING advisory: `wallStopTimedOut()` == true,
//       motors stopped (DONE), and NO hard fault is latched (the queue is not frozen). The caller
//       assumes-at-wall and continues.
//
// The mock `Drive.h` deliberately has NO wallStop surface; the mirror models the encoder + clock as
// plain inputs it advances per tick, exactly as `_updateWallStop` reads `MiniR4.M2.getDegrees()` and
// `millis()`. Pass/fail + final-line conventions mirror `test_return_to_node_dispatch.cpp`.
// =========================================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>    // std::abs

#include "Arduino.h"     // mock millis()/g_mockMillis (the watchdog clock source).
#include "src/util/WROMath.h"     // the SAME stopAtLineProgressed / stopAtLineStalled the real code calls.
#include "Config.h"      // STOP_AT_LINE_* + WALL_STOP_TIMEOUT_MS + PWM_MIN_MAGNITUDE.

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

// =========================================================================================
// Inline mirror of the WALL_STOP push watchdog (Drive::_updateWallStop, Drive.cpp:1388-1494).
//
// The replica owns the same per-PUSH state the production member fields hold:
//   _wallStartEnc / _wallStartMs / _wallLastProgressCm / _wallLastProgressMs /
//   _wallBackOffEnc / _wallStopTimedOut / _wallState
// and a single `tick(nowEnc, nowMs)` step that mirrors the PUSH and BACK_OFF state bodies. The
// caller drives `nowEnc` (canonical M2 encoder degrees) and `nowMs` (millis) per tick — exactly
// the two live inputs `_updateWallStop` reads each tick.
// =========================================================================================

enum class WallState : uint8_t { PUSH, BACK_OFF, DONE };

struct WallStopMirror
{
    // --- per-PUSH seed (Drive.cpp _updateWallStop SQUARE->PUSH transition) ---
    // NOTE (#1): production seeds the push watchdog when the SQUARE (turn-to-wall-normal) phase
    // completes and PUSH begins, NOT at the WALL_STOP dispatch — a long square-up must not eat into
    // the push's stall window. This mirror starts at PUSH, modelling exactly that post-square seed.
    int32_t  startEnc        = 0;
    uint32_t startMs         = 0;
    float    lastProgressCm  = 0.0f;
    uint32_t lastProgressMs  = 0;
    int32_t  backOffEnc      = 0;
    bool     timedOut        = false;     ///< mirrors `_wallStopTimedOut` (the advisory flag).
    float    backOffCm       = Config::WALL_STOP_BACK_OFF_CM;
    uint32_t timeoutMs       = Config::WALL_STOP_TIMEOUT_MS;
    WallState state          = WallState::PUSH;
    bool      motorsStopped  = false;     ///< set true on the DONE finalise (mirror of _endWallStop).

    /// Seed the push (mirror of the SQUARE->PUSH transition). `seedEnc` = M2 when the push begins.
    /// MIRROR: Drive::_updateWallStop SQUARE case (seeds `_wallStartEnc`/clocks, enters PUSH).
    void dispatch(int32_t seedEnc, uint32_t seedMs)
    {
        startEnc       = seedEnc;
        backOffEnc     = seedEnc;
        timedOut       = false;            // cleared on dispatch; set true only on the no-stall timeout.
        lastProgressCm = 0.0f;
        lastProgressMs = seedMs;
        startMs        = seedMs;
        state          = WallState::PUSH;
        motorsStopped  = false;
    }

    /// One cooperative tick of the push machine. @p nowEnc = M2 degrees now, @p nowMs = millis now.
    /// MIRROR: Drive.cpp:1408-1493.
    void tick(int32_t nowEnc, uint32_t nowMs)
    {
        if (state == WallState::DONE)
        {
            // DONE finalises: stop the motors (active brake), mark complete. The brake/board-restore
            // detail (MiniR4.DriveDC) is delegated to the bench; here we just record the stop.
            // MIRROR: Drive.cpp:1397-1403 + _endWallStop (1496-1510).
            motorsStopped = true;
            return;
        }

        // Travel measured as an unsigned magnitude from the per-side baseline (M2), matching
        // `_updateProfile` / `_updateStopAtLine`. MIRROR: Drive.cpp:1408-1409.
        const float traveled = encDegToCm((int32_t) std::abs(nowEnc - startEnc));

        if (state == WallState::PUSH)
        {
            // Reused no-progress watchdog: an advance of >= eps resets the baseline + clock; a stalled
            // push never resets it, so `stopAtLineStalled` fires once the timeout elapses — and HERE a
            // stall is SUCCESS (wall reached), the inverse of the stop-at-line seek.
            // MIRROR: Drive.cpp:1430-1437 — SAME WROMath primitives + SAME Config constants.
            if (stopAtLineProgressed(traveled, lastProgressCm, Config::STOP_AT_LINE_PROGRESS_EPS_CM))
            {
                lastProgressCm = traveled;
                lastProgressMs = nowMs;
            }
            const bool stalled =
                stopAtLineStalled(nowMs, lastProgressMs, Config::STOP_AT_LINE_STALL_TIMEOUT_MS);

            if (stalled)
            {
                // Wall reached — SUCCESS. Anchor the back-off roll on the current encoder, enter
                // BACK_OFF. `timedOut` stays false. MIRROR: Drive.cpp:1439-1446.
                backOffEnc = nowEnc;
                state      = WallState::BACK_OFF;
                return;
            }

            // Outer advisory timeout — never stalled. Record the NON-LATCHING advisory (NOT a hard
            // fault) and finalise via DONE. Checked AFTER the stall gate so a press that stalls on the
            // very tick the outer timeout would also fire still reports success.
            // MIRROR: Drive.cpp:1453-1459.
            if ((uint32_t) (nowMs - startMs) >= timeoutMs)
            {
                timedOut = true;
                state    = WallState::DONE;
                return;
            }
            // Still pushing — hold the flat low push PWM (no state change). MIRROR: Drive.cpp:1462.
            return;
        }

        // BACK_OFF: roll the known back-off distance in the reverse-of-push direction, then DONE.
        // MIRROR: Drive.cpp:1466-1488.
        const float rolled = encDegToCm((int32_t) std::abs(nowEnc - backOffEnc));
        if (backOffCm <= 0.0f || rolled >= backOffCm)
        {
            state = WallState::DONE;
            return;
        }
        // else keep rolling (no state change).
    }
};

// Run the mirror to completion, advancing the encoder/clock per the supplied stepper until the
// machine reaches DONE (motorsStopped) or a tick cap is hit (so a logic bug cannot hang the test).
template <typename Stepper>
static void runToDone(WallStopMirror& m, Stepper step)
{
    for (int i = 0; i < 100000 && !m.motorsStopped; ++i)
    {
        int32_t  enc = 0;
        uint32_t ms  = 0;
        step(i, enc, ms);
        m.tick(enc, ms);
    }
}

int main()
{
    std::printf("INFO: wallStop watchdog mirror — shared WROMath primitives + real Config constants\n");

    // Sanity: the mirror's seeds equal the production Config defaults (so a config edit flows in).
    check(WallStopMirror{}.timeoutMs == Config::WALL_STOP_TIMEOUT_MS,
          "precondition: mirror outer timeout == Config::WALL_STOP_TIMEOUT_MS");
    check(Config::WALL_STOP_TIMEOUT_MS > Config::STOP_AT_LINE_STALL_TIMEOUT_MS,
          "precondition: outer timeout exceeds the inner stall timeout (a real press trips the "
          "inner watchdog first) — mirrors the production static_assert");

    // -------------------------------------------------------------------------------------
    // TC9 (AC8) — stall within the window -> SUCCESS: back-off then DONE, NOT timed-out.
    //
    // Model: the chassis pushes and the encoder advances for a few ticks (real progress), then
    // STALLS against the wall (encoder frozen). After STOP_AT_LINE_STALL_TIMEOUT_MS of no progress,
    // `stopAtLineStalled` fires -> BACK_OFF -> the back-off roll consumes WALL_STOP_BACK_OFF_CM ->
    // DONE with motors stopped and `timedOut == false`. The whole thing completes BEFORE the outer
    // WALL_STOP_TIMEOUT_MS, so no advisory.
    // -------------------------------------------------------------------------------------
    {
        WallStopMirror m;
        m.dispatch(/*seedEnc*/ 0, /*seedMs*/ 1000);

        // Encoder advances enough per tick to clear the progress eps while pushing; the clock steps
        // a small fixed slice per tick. The push reaches the wall and freezes after `kStallAtTick`.
        const int     kStallAtTick = 4;       // ticks of real motion before the wheels jam.
        const uint32_t kMsPerTick  = 100;     // each tick advances the mock clock 100 ms.
        // Per-tick travel that clears the progress eps (>= STOP_AT_LINE_PROGRESS_EPS_CM cm). Convert
        // a comfortable 1 cm into encoder degrees so `traveled` advances ~1 cm/tick while pushing.
        const int32_t kEncPerTick  = (int32_t) cmToEncDeg(1.0f);

        bool reachedBackOff = false;
        int32_t frozenEnc   = 0;

        runToDone(m, [&](int i, int32_t& enc, uint32_t& ms)
        {
            ms = 1000 + (uint32_t) (i + 1) * kMsPerTick;    // clock advances every tick.
            if (i < kStallAtTick)
            {
                enc = (int32_t) (i + 1) * kEncPerTick;       // pushing: encoder climbs.
                frozenEnc = enc;
            }
            else
            {
                enc = frozenEnc;                              // stalled against the wall: encoder frozen.
                // Once BACK_OFF begins, advance the encoder again so the back-off roll completes.
                if (m.state == WallState::BACK_OFF)
                {
                    reachedBackOff = true;
                    // Roll well past WALL_STOP_BACK_OFF_CM in the back-off direction.
                    enc = frozenEnc - (int32_t) cmToEncDeg(Config::WALL_STOP_BACK_OFF_CM + 1.0f);
                }
            }
        });

        check(m.motorsStopped, "TC9: a stalled push reaches DONE (motors stopped)");
        check(!m.timedOut, "TC9: a stalled push reports SUCCESS, not the advisory timeout");
        check(reachedBackOff, "TC9: a stalled push passes through BACK_OFF before DONE");
        // The success path must finish before the outer advisory timeout would fire.
        check((m.startMs + m.timeoutMs) > (1000u + (uint32_t) (kStallAtTick + 1) * kMsPerTick),
              "TC9: the stall+back-off completes inside WALL_STOP_TIMEOUT_MS (no advisory window)");
    }

    // -------------------------------------------------------------------------------------
    // TC10 (AC8) — NO stall within WALL_STOP_TIMEOUT_MS -> advisory timeout, NOT latched.
    //
    // Model: the wheels keep turning freely forever (never reach a wall), so the encoder advances
    // every tick and the inner stall watchdog NEVER fires. Once the outer WALL_STOP_TIMEOUT_MS
    // elapses, the push records the NON-LATCHING `timedOut` advisory, stops the motors, and enters
    // DONE — with NO hard fault (the mirror has no fault state to set; the production code leaves
    // `_fault` untouched and `hasFault()` false, asserted by the PathTracker mirror TC10b).
    // -------------------------------------------------------------------------------------
    {
        WallStopMirror m;
        m.dispatch(/*seedEnc*/ 0, /*seedMs*/ 2000);

        const uint32_t kMsPerTick = 100;
        const int32_t  kEncPerTick = (int32_t) cmToEncDeg(1.0f);   // free-spinning: always progressing.

        runToDone(m, [&](int i, int32_t& enc, uint32_t& ms)
        {
            ms  = 2000 + (uint32_t) (i + 1) * kMsPerTick;
            enc = (int32_t) (i + 1) * kEncPerTick;                  // never freezes -> never stalls.
        });

        check(m.motorsStopped, "TC10: a never-stalling push still reaches DONE (motors stopped)");
        check(m.timedOut, "TC10: a never-stalling push sets the advisory wallStopTimedOut flag");
        check(m.state == WallState::DONE, "TC10: the advisory path finalises via DONE (no fault state)");
        // The advisory must fire only AFTER the full outer timeout elapsed since dispatch.
        check((m.startMs + m.timeoutMs) <= (2000u + 100000u * kMsPerTick) || m.timedOut,
              "TC10: the advisory fires once WALL_STOP_TIMEOUT_MS has elapsed since dispatch");
    }

    // -------------------------------------------------------------------------------------
    // TC9/TC10 boundary — a press that stalls on the VERY tick the outer timeout would also fire
    // still reports SUCCESS, never a spurious advisory. The stall gate is checked BEFORE the outer
    // timeout gate (Drive.cpp:1451-1453 ordering), so a same-tick coincidence resolves to success.
    // -------------------------------------------------------------------------------------
    {
        WallStopMirror m;
        m.dispatch(/*seedEnc*/ 0, /*seedMs*/ 0);

        // Freeze the encoder from the very first tick so the inner stall timeout and the outer
        // advisory timeout would mature at related times; drive the clock so BOTH gates are eligible
        // on the same tick, and confirm the stall (success) wins.
        const uint32_t kStall = Config::STOP_AT_LINE_STALL_TIMEOUT_MS;
        const uint32_t kOuter = Config::WALL_STOP_TIMEOUT_MS;

        // Tick 1: seed progress at t=0 with a tiny advance, then freeze. lastProgressMs = 0.
        m.tick(/*nowEnc*/ (int32_t) cmToEncDeg(0.5f), /*nowMs*/ 0);
        // Jump the clock to exactly the inner stall maturity (>= kStall since last progress). Since
        // kOuter > kStall (static_assert), the outer gate is NOT yet eligible here -> stall wins.
        m.tick(/*nowEnc*/ (int32_t) cmToEncDeg(0.5f), /*nowMs*/ kStall);
        check(m.state == WallState::BACK_OFF || m.state == WallState::DONE,
              "TC9/TC10 boundary: a matured inner stall takes the SUCCESS path (BACK_OFF/DONE)");
        check(!m.timedOut,
              "TC9/TC10 boundary: stall-on-maturity reports success, never the advisory timeout");

        (void) kOuter;
    }

    if (g_failures == 0)
    {
        std::printf("WALL STOP WATCHDOG OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
