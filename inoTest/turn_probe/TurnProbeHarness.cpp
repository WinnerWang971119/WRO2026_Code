/**
 * @file TurnProbeHarness.cpp
 * @brief Implementation of the turn creep-floor / tick-period / IMU-noise bench probe.
 * @author WRO 2026
 *
 * @details
 * EXTRACTED from `WRO2026/Tasks.cpp::testOne` + its file-local `TurnProbe*` helpers
 * (docs/plans/2026-07-16-turn-pd-controller.md, task T10) into this standalone diagnostic
 * sketch. T10's original text called for reverting `testOne` to an empty stub and deleting
 * the harness outright; the operator amended that mid-implementation to EXTRACT it here
 * instead of deleting it, since the probe is a reusable bench tool (re-run it any time the
 * chassis, battery, or gearing changes) and is worth keeping buildable off the mission
 * sketch. `testOne` itself is now the trivial `return true;` stub T10 originally specified —
 * see `WRO2026/Tasks.cpp`.
 *
 * Kept in its own `.cpp` (not inlined into `turn_probe.ino`) because Arduino's primary
 * `.ino` file goes through a ctags-based auto-prototype pass that does not understand
 * anonymous namespaces: it would emit a global-scope forward declaration for every
 * function defined below, which have INTERNAL linkage inside `namespace { ... }`, and the
 * link step would then fail to resolve the mismatched global-scope declarations. A `.cpp`
 * file in the sketch folder is compiled as an ordinary translation unit with no such
 * rewriting, which is also why `WRO2026/Tasks.cpp` (this code's original home) never hit
 * this problem — it was never a `.ino`. Splitting into a `.h`/`.cpp` pair here mirrors that
 * same convention (CLAUDE.md: heavy logic lives in classes/free functions in a `.cpp`, not
 * inline in the `.ino`'s `setup()`/`loop()`).
 *
 * Measures four things needed before the PD turn executor (T2-T4) can be tuned:
 *   (a) creep sweep, PWM 10..48 step 2, Move(p,-p) in-place counter-rotation:
 *       (a1) from rest       -> static/breakaway floor (reference only)
 *       (a2) while rotating  -> KINETIC floor -> becomes `Config::TURN_CREEP_PWM`
 *   (b) time a 90 deg rotation at the kinetic floor to get omega (deg/s), and count the
 *       loop's own iterations to get the TURN-representative tick (see the (b) function
 *       below)
 *   (c) re-confirm the IDLE `cooperativeTick()` period (already measured 2026-07-16 as
 *       min=7995 avg=8056 max=11484 us / ~124 Hz via `inoTest/tick_bench`)
 *   (d) IMU yaw noise: 100 stationary reads, report min/max/spread
 *
 * EXECUTION ORDER IS NOT THE a/b/c/d LETTERING: the phases run (d), (c), (a1), (a2), (b).
 *   - (d) must run BEFORE (a1) because (a1)'s "did it move" degree threshold is derived from
 *     the measured noise spread -- a threshold set before the noise it discriminates against
 *     is measured is a guess.
 *   - (c) runs with (d) because both are stationary, so the operator sets the chassis down
 *     once for both and repositions once for the three motion phases.
 *
 * MEASUREMENT CONTRACTS THIS HARNESS MUST NOT BREAK (each cost a real bug once):
 *   - (a2) scores a SUSTAINED RATE, never a total-degree threshold. The chassis is already
 *     rotating when a kinetic trial begins, so a coast-down alone sweeps 5-15 deg and would
 *     score "moved" at EVERY swept PWM -- returning the sweep's own lower bound as a
 *     "measurement" independent of the plant.
 *   - (a2) drops to the test PWM and waits out `TURN_PROBE_DROP_SETTLE_MS` BEFORE opening its
 *     measurement window. Opening it AT the drop puts the whole kick->testPwm decay inside the
 *     first half by construction, so `omegaFirst` measures the transient instead of the rate the
 *     test PWM settles to -- and the sustain RATIO then scores a genuine sustainer against an
 *     inflated baseline and REJECTS it. Phase (b) has always done this; (a2) is the phase that
 *     actually produces `TURN_CREEP_PWM`.
 *   - (a2)'s kick must be STRICTLY above the top swept PWM, or the top of the sweep has no drop,
 *     passes on arithmetic rather than physics, and the "NOT FOUND" escape hatch goes dead.
 *   - (b) reports omega ONLY on reaching the target rotation. The degree accumulator sums
 *     ABSOLUTE per-poll deltas, so stationary IMU noise random-walks it upward forever; it
 *     can never be 0, and a timeout must therefore NEVER fall through into an omega.
 *   - (b) kicks before creeping. The kinetic floor lies BELOW the static breakaway floor by
 *     construction, so commanding it from rest never breaks away.
 *   - TURN_MIN_RESOLVABLE_DEG is derived from (b)'s TURN tick, not (c)'s IDLE tick: a real
 *     PD turn adds a blocking yaw read AND a blocking Move(L,R) write per tick.
 *   - EVERY measured quantity is a MAGNITUDE (fabsf), and every verdict compares magnitudes.
 *     That is what lets `TurnProbeUnwinder` alternate the rotation direction to keep the USB
 *     tether from winding up (it would otherwise reach 12-15 revolutions one way over a run,
 *     and a taut cable drags on the chassis -- loading the plant and biasing the creep floor UP,
 *     which is exactly the measurement this session exists to produce). A signed yaw leaking
 *     into any gate would silently invert it for half the trials.
 *   - A trial's kick and its test drive share ONE direction, threaded via `TurnProbeUnwinder`.
 *     Kicking one way and testing the other measures a spin-up through zero, not a decay.
 *
 * BYPASSES `Drive` ENTIRELY. `Drive`'s emit path runs `deadbandSnap` (WROMath.h), which
 * floors ANY non-zero PWM magnitude UP to `Config::PWM_MIN_MAGNITUDE` (35) -- so
 * commanding 10..30 through a `Drive` verb would silently emit 35 for every step and the
 * whole sweep would measure nothing. This harness drives `MiniR4.DriveDC.Move(L, R)`
 * directly -- the same raw vendor call the real turn executor itself issues
 * (`Drive.cpp::_updateTurnProfile`), just without Drive's snap/PID/queue.
 *
 * @note NOT in CI -- the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only
 *       local check that this links. Build/flash/monitor via `uv run inoTest/turn_probe/build.py`.
 */

#include "TurnProbeHarness.h"

#include <MatrixMiniR4.h>

#include <math.h>      // fabsf
#include <stdint.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "YawSource.h"

namespace
{
    constexpr int16_t  TURN_PROBE_SWEEP_MIN_PWM  = 10;
    // Deliberately ABOVE `Config::PWM_MIN_MAGNITUDE` (35). The sweep hunts two floors -- the
    // static breakaway floor at/near 35 and the kinetic floor below it -- so a ceiling under 35
    // structurally cannot bracket its own target and can only ever report "NOT FOUND", i.e. a
    // re-flash on a one-shot bench session. The plan's B-T1 row says 10..30; the plan is wrong
    // here and this harness is what runs.
    //
    // 48 and not 50, because the ceiling must stay STRICTLY below `TURN_PROBE_KICK_PWM` (= 50).
    // At testPwm == kickPwm the kinetic sweep's "kick then drop" has no drop at all: omegaFirst
    // and omegaSecond become two samples of one steady rate, the sustain ratio is ~1.0 by
    // arithmetic rather than by physics, and the top of the sweep passes unconditionally on ANY
    // chassis. That made `turnProbeRunSweep`'s -1 return -- and the "NOT FOUND" operator escape
    // hatch it is the only trigger for -- unreachable dead code. Both tripwires below pin this.
    constexpr int16_t  TURN_PROBE_SWEEP_MAX_PWM  = 48;
    constexpr int16_t  TURN_PROBE_SWEEP_STEP_PWM = 2;
    constexpr uint8_t  TURN_PROBE_TRIALS         = 3;      // per swept PWM value
    constexpr uint32_t TURN_PROBE_REST_WINDOW_MS = 400;    // (a1) drive window, from rest
    // (a2) MEASUREMENT window. Opened only AFTER the drop transient has died out (see
    // `TURN_PROBE_DROP_SETTLE_MS`) -- the coast-down is NOT inside it. Split in half, and both
    // halves now sample settled rotation: a true sustainer reads ratio ~1.0, so the ratio test
    // only fires on genuine residual decay.
    //
    // 500 ms per half is sized by the RATE FLOOR, not by the transient: certifying 15 deg/s over
    // a half needs 7.5 deg of apparent rotation, against a stationary IMU spread that (d)
    // measures at well under a degree. Do not shrink it without re-checking that margin.
    constexpr uint32_t TURN_PROBE_KINETIC_WINDOW_MS = 1000;
    constexpr uint32_t TURN_PROBE_SETTLE_MS         = 300;    // pause after each step; chassis fully stops
    // (a1) only. Lower bound on the "did it move" degree threshold; the effective threshold is
    // `max(this, TURN_PROBE_MOVED_THRESH_NOISE_K * measured (d) spread)` -- see
    // `turnProbeDeriveMovedThresh`. (a2) has NO degree threshold; it scores a sustained RATE.
    constexpr float TURN_PROBE_MOVED_THRESH_MIN_DEG = 2.0f;
    constexpr float TURN_PROBE_MOVED_THRESH_NOISE_K = 3.0f;
    // (a2) sustained-rotation criterion. BOTH must hold on the second half-window:
    //   1. `omegaSecond >= FRAC * omegaFirst`  -- both halves sample POST-settle rotation, so a
    //      chassis that genuinely sustains `testPwm` reads ratio ~1.0 and 0.6 is a barely-binding
    //      gate that fires only on real residual decay. A chassis still coasting to a stop is
    //      still decaying across both halves and reads well under it. (Before the settle existed
    //      this gate was scored against a decay-contaminated omegaFirst and rejected the very
    //      sustainers it exists to certify -- see `turnProbeMeasureStep`.)
    //   2. `omegaSecond >= RATE_FLOOR`         -- condition 1 alone is satisfiable by two
    //      near-zero noise rates on a MOTIONLESS chassis (pure noise also reads ratio ~1.0). This
    //      floor is what rejects that, and it is what rejects a coast that has already died
    //      inside the settle dead time.
    constexpr float TURN_PROBE_SUSTAIN_FRAC                = 0.6f;
    constexpr float TURN_PROBE_SUSTAIN_RATE_FLOOR_DEG_S    = 15.0f;
    constexpr int16_t  TURN_PROBE_KICK_PWM = Config::TURN_CRUISE_PWM;    // stiction-breaking spin-up
    constexpr uint32_t TURN_PROBE_KICK_MS  = 200;    // spin-up window before dropping to the test PWM
    // (a2) AND (b). Dead time between the drop to the test/creep PWM and the measurement opening,
    // so neither the kick's own rotation nor the decay off it is ever credited to the test PWM.
    // Both phases want the same thing -- the rate `testPwm` SUSTAINS -- and the chassis does not
    // reach that rate until the drop transient has died.
    //
    // Sized at >= ~2 decay time constants. This harness's own plant note puts the kick->creep
    // coast-down tau in the hundreds of ms; at tau = 250 ms a 500 ms dead time leaves ~13% of the
    // initial excess rate, which lands inside the ratio test's 0.6 slack rather than outside it.
    // Lofaix is the heavy chassis and sits at the long-tau end. If (a2) ever reports a floor well
    // above the one (b) then confirms, THIS is the constant to grow -- never the ratio.
    constexpr uint32_t TURN_PROBE_DROP_SETTLE_MS = 500;
    constexpr float    TURN_PROBE_OMEGA_TARGET_DEG = 90.0f;
    // Bounds a chassis that is NOT rotating -- never a slow-but-honest one. The slowest rate
    // (a2) can certify is `TURN_PROBE_SUSTAIN_RATE_FLOOR_DEG_S`, and at that rate the target
    // takes 6 s; a timeout below that would discard a real measurement and hand the operator
    // nothing. Kept at 2x that worst honest case. The tripwire below pins the coupling.
    constexpr uint32_t TURN_PROBE_OMEGA_TIMEOUT_MS  = 12000;
    constexpr uint16_t TURN_PROBE_TICK_SAMPLE_COUNT = 200;    // cooperativeTick() samples for (c)
    // (c) reference: the IDLE cooperativeTick() period already measured on this build 2026-07-16
    // via inoTest/tick_bench (min=7995 avg=8056 max=11484 us). Printed BESIDE (c)'s own number so
    // a materially inflated reading is visible on the spot rather than only to someone who goes
    // looking -- see `turnProbeMeasureTick` for what inflates it.
    constexpr float TURN_PROBE_TICK_REF_AVG_US = 8056.0f;
    constexpr uint16_t TURN_PROBE_IMU_SAMPLE_COUNT  = 100;    // stationary yaw reads for (d)
    constexpr uint32_t TURN_PROBE_IMU_SAMPLE_GAP_MS = 20;
    // Bench data alone cannot set this -- it is a design headroom choice made in T3/T11 (the
    // mentor's verified pair is creep=15 / thresh=20, a +5 headroom). Printed as a SUGGESTION
    // only; see `turnProbePrintSummary`.
    constexpr int16_t TURN_PROBE_CREEP_THRESH_HEADROOM_PWM = 5;

    static_assert(TURN_PROBE_SWEEP_MAX_PWM > Config::PWM_MIN_MAGNITUDE,
                  "TURN_PROBE_SWEEP_MAX_PWM must exceed Config::PWM_MIN_MAGNITUDE -- the sweep "
                  "hunts the breakaway floor, which sits AT that value. A ceiling below it can "
                  "only ever report 'NOT FOUND' and force a re-flash.");
    static_assert(TURN_PROBE_KICK_PWM > TURN_PROBE_SWEEP_MAX_PWM,
                  "TURN_PROBE_KICK_PWM must be STRICTLY greater than TURN_PROBE_SWEEP_MAX_PWM so "
                  "that EVERY kinetic-sweep trial is a real spin-up followed by a real drop. A "
                  "kick BELOW the test PWM is a spin-UP at the test PWM and measures the opposite "
                  "transient. A kick EQUAL to it is no drop at all: omegaFirst and omegaSecond "
                  "then sample one steady rate, the sustain ratio is ~1.0 by arithmetic rather "
                  "than by physics, the top of the sweep passes on any chassis, and "
                  "turnProbeRunSweep can never return -1 -- which makes its 'NOT FOUND' operator "
                  "escape hatch, the only thing that reports mistuned sustain thresholds, "
                  "unreachable dead code.");
    static_assert(TURN_PROBE_OMEGA_TIMEOUT_MS > (uint32_t) (1000.0f * TURN_PROBE_OMEGA_TARGET_DEG
                                                            / TURN_PROBE_SUSTAIN_RATE_FLOOR_DEG_S),
                  "TURN_PROBE_OMEGA_TIMEOUT_MS must exceed the time the SLOWEST rate (a2) can "
                  "certify needs to cover TURN_PROBE_OMEGA_TARGET_DEG, else phase (b) discards a "
                  "valid slow measurement as a timeout and the session yields no omega.");

    /// @brief Per-phase measurement results, threaded through the harness and printed by
    /// `turnProbePrintSummary` at the end. `-1` / `*Valid == false` mean "not measured".
    /// @note "not measured" is a first-class outcome here and MUST stay distinguishable from a
    ///       value. Every consumer of this bench (`TURN_CREEP_PWM`, `TURN_CREEP_THRESH_PWM`,
    ///       `TURN_MIN_RESOLVABLE_DEG` -> an AC8 `static_assert`) takes its number on trust, so a
    ///       plausible-but-fabricated result is strictly worse than a printed `NOT MEASURED`.
    struct TurnProbeResults
    {
        int16_t breakawayPwm   = -1;
        int16_t sustainPwm     = -1;
        float   omegaDegPerSec = 0.0f;
        bool    omegaValid     = false;
        /// (c) IDLE tick: cooperativeTick() alone. Reference only -- NOT the turn's tick.
        uint32_t tickMinUs = 0;
        uint32_t tickMaxUs = 0;
        float    tickAvgUs = 0.0f;
        bool     tickValid = false;
        /// (b) TURN-representative tick: pump + Move(L,R) write + yaw read, i.e. the per-tick
        /// work a real PD turn actually does. This is the tick `TURN_MIN_RESOLVABLE_DEG` uses.
        float    turnTickAvgUs     = 0.0f;
        uint32_t turnTickIterations = 0;
        bool     turnTickValid     = false;
        double   imuMinDeg    = 0.0;
        double   imuMaxDeg    = 0.0;
        double   imuSpreadDeg = 0.0;
        bool     imuValid     = false;
    };

    /// @brief Minimal cooperative pump for the harness: drive the caller's tick callback
    /// (falls back to `Drive.update()` alone if none is registered -- defensive only; the
    /// caller, `turn_probe.ino`, always passes its real `cooperativeTick`) and yield.
    void turnProbePump(void (*tickCb)())
    {
        if (tickCb != nullptr)
        {
            tickCb();
        }
        else
        {
            Drive.update();
        }
        yield();
    }

    /// @brief Fold a raw heading delta into (-180, +180]. Local, throwaway twin of
    /// `TurnPDPolicy.h::turnWrap180` -- kept file-local so this harness stays a single,
    /// cleanly-revertible block with no dependency beyond `YawSource`.
    float turnProbeWrap180(float deltaDeg)
    {
        while (deltaDeg > 180.0f)
        {
            deltaDeg -= 360.0f;
        }
        while (deltaDeg <= -180.0f)
        {
            deltaDeg += 360.0f;
        }
        return deltaDeg;
    }

    /// @brief Stop the motors and hold for `TURN_PROBE_SETTLE_MS` so the chassis is fully at
    /// rest before the next probe step. Called after EVERY windowed drive in this harness --
    /// never leave the chassis driving between phases.
    void turnProbeStopMotors(void (*tickCb)())
    {
        MiniR4.DriveDC.Move(0, 0);
        MiniR4.DriveDC.brake(true);
        const uint32_t start = millis();
        while ((uint32_t) (millis() - start) < TURN_PROBE_SETTLE_MS)
        {
            turnProbePump(tickCb);
        }
    }

    /// @brief Blocking wait for a fresh `BTN_UP` rising edge, pumping cooperative ticks.
    /// Waits out any already-held press first so a leftover press cannot immediately
    /// satisfy this wait without a real press-after-prompt.
    void turnProbeWaitForBtnUp(void (*tickCb)(), const __FlashStringHelper* prompt)
    {
        Serial.println();
        Serial.println(prompt);
        Serial.println(F("[T1] press BTN_UP to continue..."));

        while (MiniR4.BTN_UP.getState())
        {
            turnProbePump(tickCb);
        }
        while (!MiniR4.BTN_UP.getState())
        {
            turnProbePump(tickCb);
        }
        Serial.println(F("[T1] ...go."));
    }

    /// @brief Read yaw via `YawSource`, printing a warning and leaving `out` untouched on a
    /// hard transport failure (see `YawSource::readYawDeg` -- already retried once inside).
    bool turnProbeReadYaw(double& out, const __FlashStringHelper* site)
    {
        if (YawSource.readYawDeg(out))
        {
            return true;
        }
        Serial.print(F("[T1] yaw read FAILED at "));
        Serial.println(site);
        return false;
    }

    /// @brief Greedy tether unwinder for ONE trial. Cable management ONLY -- nothing this class
    /// produces reaches a measurement or a verdict (see `commit`).
    ///
    /// EVERY drive in this harness is an in-place counter-rotation, and before this class existed
    /// every one of them spun the SAME way: ~2 rev of (a1) + ~10 rev of (a2) + ~0.3 rev of (b),
    /// call it 12-15 revolutions in one direction over a full run. The operator reads the results
    /// off Serial over the USB cable, so the cable winds onto the chassis; once taut it drags,
    /// and a drag torque loads the very plant whose friction floor this session exists to measure
    /// -- biasing `TURN_CREEP_PWM` UPWARD into a plausible-but-wrong number. It also walks the
    /// chassis out of its clear circle.
    ///
    /// The rule is NOT a per-trial sign flip: `TURN_PROBE_TRIALS` is odd (3), so a flip leaves a
    /// net trial per PWM step, and rotation per trial grows with PWM, so the leftovers do not
    /// cancel across steps either. Instead each trial drives whichever way REDUCES the running
    /// signed net. That bounds |net| by the largest single trial's rotation for any ordering (see
    /// `commit`), and self-corrects when a trial rotates more than expected.
    ///
    /// @note The net is carried in "dir = +1" units -- commanded direction x MEASURED rotation
    ///       MAGNITUDE -- never in signed IMU yaw. The magnitude is real data (that is what makes
    ///       the loop self-correcting), while the sign comes from the command, which IS the
    ///       physical direction by definition. That keeps this class independent of whether
    ///       `Move(+p, -p)` happens to read as +yaw or -yaw under `Config::IMU_YAW_SIGN`: a signed
    ///       -yaw accumulator paired with this greedy rule would drive |net| UP, not down, and
    ///       run away in one direction -- the exact failure it exists to prevent.
    class TurnProbeUnwinder
    {
    public:
        /// @brief Bind to the run-wide net and pick this trial's direction against it.
        /// @param netDeg Run-wide signed net rotation, in "dir = +1" units. Mutated by `commit`.
        explicit TurnProbeUnwinder(float& netDeg)
            : _netDeg(netDeg), _dir(netDeg > 0.0f ? (int16_t) -1 : (int16_t) 1)
        {
        }

        /// @return This trial's commanded direction: +1 or -1.
        int16_t dir() const { return _dir; }

        /// @brief Apply this trial's direction to a magnitude PWM, for `Move(pwm(p), -pwm(p))`.
        /// @note The kick and the test drive of a single trial MUST both go through this, i.e.
        ///       share one direction. A kick one way and a test drive the other measures a
        ///       spin-up THROUGH zero, not the decay off a kick, and (a2)'s whole sustain
        ///       criterion is scored against that decay.
        int16_t pwm(int16_t magnitude) const { return (int16_t) (_dir * magnitude); }

        /// @brief Fold one more yaw checkpoint into this trial's rotation MAGNITUDE.
        ///
        /// Sums |wrapped delta| between CONSECUTIVE checkpoints, never an end-to-end diff: a
        /// top-of-sweep (a2) trial sweeps ~200 deg all told, which a single diff aliases.
        ///
        /// @note Call ONLY on a successful read. A failed read must not advance `_prev` onto a
        ///       value that was never read -- the same dropout rule phase (b)'s poll loop follows.
        ///       The next successful fold then spans two intervals. The widest span here is
        ///       pre-kick -> window-open (kick 200 ms + drop-settle 500 ms), which aliases only
        ///       above ~257 deg/s, against the ~115 deg/s this harness's own plant notes assume.
        ///       That is ~2x margin, not a proof. If it ever does alias, `_trialDeg` under-counts
        ///       and the cable margin shrinks -- no measurement moves, since the in-window spans
        ///       are 500 ms and would need >360 deg/s.
        void fold(double yawNow)
        {
            if (_havePrev)
            {
                _trialDeg += fabsf(turnProbeWrap180((float) (yawNow - _prev)));
            }
            _prev     = yawNow;
            _havePrev = true;
        }

        /// @brief Commit this trial's rotation to the run-wide net, signed by the command.
        ///
        /// Bound: let R be the largest single-trial rotation. If |net| <= R before a trial then
        /// after it |net| <= R too -- net > 0 picks dir = -1 giving net' = net - r in [-R, net),
        /// and net <= 0 picks dir = +1 giving net' = net + r in [net, R]; both stay inside +-R
        /// for any r in [0, R]. Starting from net = 0 the invariant holds for the whole run, in
        /// ANY trial order. R here is the top-of-sweep (a2) trial -- kick + drop-settle + window
        /// + brake, ~200 deg -- so a full run drifts at most ~0.55 rev instead of 12-15.
        ///
        /// @note That bound is over the RECORDED net. A failed unwind checkpoint drops its span
        ///       from `_trialDeg` (loudly -- `turnProbeReadYaw` prints the site), and the greedy
        ///       rule counters what it recorded, so the physical drift exceeds the bound by
        ///       whatever went unrecorded. It costs cable margin, never a measurement.
        /// @note Zeroes the trial accumulator, so this is committed exactly once even though
        ///       every one of `turnProbeMeasureStep`'s abort paths routes through
        ///       `turnProbeStopAndUnwind`.
        void commit()
        {
            _netDeg  += (float) _dir * _trialDeg;
            _trialDeg = 0.0f;
        }

    private:
        float&  _netDeg;
        int16_t _dir;
        float   _trialDeg = 0.0f;
        double  _prev     = 0.0;
        bool    _havePrev = false;
    };

    /// @brief Stop the motors, fold the brake/settle rotation in, and commit the trial to the
    /// run-wide net. Every path that has commanded a `Move` must exit through this.
    ///
    /// @note The post-stop checkpoint is what makes the committed net the trial's REAL rotation
    ///       rather than its measured window alone. It closes the same gap the pre-kick
    ///       checkpoint opens -- see `turnProbeMeasureStep`.
    void turnProbeStopAndUnwind(void (*tickCb)(), TurnProbeUnwinder& unwind)
    {
        turnProbeStopMotors(tickCb);

        double yawStop = 0.0;
        if (turnProbeReadYaw(yawStop, F("unwind post-stop")))
        {
            unwind.fold(yawStop);
        }
        unwind.commit();
    }

    /// @brief Report the run-wide tether net after a phase, so the operator can see the unwinder
    /// working rather than take it on trust -- and so a runaway (a net that climbs monotonically
    /// past a revolution) is visible on the spot instead of only as a taut cable.
    void turnProbePrintTetherNet(float netDeg)
    {
        Serial.print(F("[T1] tether: net rotation so far = "));
        Serial.print(netDeg, 1);
        Serial.print(F(" deg ("));
        Serial.print(netDeg / 360.0f, 2);
        Serial.println(F(" rev). Bounded by one trial's rotation, NOT driven to zero."));
    }

    /// @brief One probe step's raw measurement. `valid == false` means a yaw read failed and
    /// EVERY field is meaningless -- there is no "partial" sample.
    struct TurnProbeStepSample
    {
        bool  valid = false;
        /// Total rotation swept over the whole window, as the SUM of the two half-window
        /// magnitudes -- never a wrapped end-to-end diff, which aliases past 180 deg at the top
        /// of the sweep. Scored by (a1); diagnostic for (a2).
        float totalDeg = 0.0f;
        /// Mean |rate| over each half-window, deg/s, each divided by its OWN measured duration.
        float omegaFirstDegS  = 0.0f;
        float omegaSecondDegS = 0.0f;
    };

    /// @brief Drive `testPwm` in `unwind`'s direction (`Move(+-testPwm, -+testPwm)`) and sample
    /// yaw at three points -- the
    /// measurement window's open, midpoint and close -- to recover a total rotation AND the mean
    /// rate over each half separately. Always stops the motors before returning.
    ///
    /// Two shapes, selected by `kickPwm`:
    ///   - `kickPwm == 0` -- "from rest" / breakaway (a1). The window opens AT the `Move`. There
    ///     is no transient to wait out and the question is exactly "does it move at all", so the
    ///     moment the command lands is the moment the measurement must start.
    ///   - `kickPwm != 0` -- "while rotating" / kinetic (a2). Spin up at `kickPwm` for `kickMs`,
    ///     DROP to `testPwm`, wait `TURN_PROBE_DROP_SETTLE_MS`, and only THEN open the window.
    ///
    /// @note The settle dead time on the kicked path is load-bearing, not politeness. Opening the
    ///       window at the drop puts the entire kick->testPwm decay inside the first half BY
    ///       CONSTRUCTION, making `omegaFirst` a decay-contaminated mean rather than the rate
    ///       `testPwm` settles to -- so the caller's `omegaSecond >= FRAC * omegaFirst` test would
    ///       score a genuine sustainer against an inflated baseline and REJECT it. For a chassis
    ///       entering at ~115 deg/s and settling to ~25 deg/s, that mis-scoring begins at a decay
    ///       tau of only ~80-100 ms, and this harness's own plant note puts tau in the HUNDREDS of
    ///       ms. Phase (b) has always waited out the drop; this is the same idiom for the same
    ///       reason, in the phase that actually produces `TURN_CREEP_PWM`.
    /// @note With the settle in place both half-rates measure SETTLED rotation, so the two verdict
    ///       gates stay independent: a sustainer reads ratio ~1.0 and clears the rate floor, while
    ///       a chassis still coasting to a stop keeps decaying (fails the ratio) AND is closer to
    ///       zero than it was under the old window (fails the floor). It fails both, not one.
    /// @param unwind Supplies the trial's direction (both the kick and the test drive take it
    ///               from here, so they cannot disagree) and collects the rotation the chassis
    ///               actually performs. Cable management only -- it feeds no field of the sample.
    /// @return The sample; `valid == false` if any of the three yaw reads failed, or if either
    ///         measured half came back implausibly short (see `minHalfSec`).
    /// @note Every measured field is a MAGNITUDE, so the sample -- and every gate the caller
    ///       scores it against -- reads identically whichever way `unwind` sent this trial. The
    ///       direction reaches the chassis and the CSV, never the arithmetic.
    TurnProbeStepSample turnProbeMeasureStep(
        int16_t             testPwm,
        int16_t             kickPwm,
        uint32_t            kickMs,
        uint32_t            windowMs,
        TurnProbeUnwinder&  unwind,
        void (*tickCb)())
    {
        TurnProbeStepSample sample;

        const int16_t testDrive = unwind.pwm(testPwm);

        if (kickPwm != 0)
        {
            // Unwind checkpoint BEFORE the kick. The kick and the drop-settle rotate the chassis
            // too -- ~78 deg at the top of the sweep -- and at the BOTTOM they are all of it: a
            // pwm-10 trial rotates only during the kick. A window-only accumulator therefore reads
            // ~0 for every non-sustaining trial, the greedy rule never counters them, and the low
            // half of the sweep winds the tether up in ONE direction exactly as before. This read
            // is what makes the net track the trial's real rotation rather than its measured part.
            double yawPreKick = 0.0;
            if (turnProbeReadYaw(yawPreKick, F("unwind pre-kick")))
            {
                unwind.fold(yawPreKick);
            }

            // Kick -> drop -> settle, and only then (below) open the window. The settle must
            // happen BEFORE `t0`/`yaw0` are sampled: it is what keeps the kick's decay out of the
            // measurement entirely, rather than merely out of its second half. Mirrors phase (b).
            const int16_t kickDrive = unwind.pwm(kickPwm);
            MiniR4.DriveDC.Move(kickDrive, -kickDrive);
            uint32_t phaseStart = millis();
            while ((uint32_t) (millis() - phaseStart) < kickMs)
            {
                turnProbePump(tickCb);
            }

            MiniR4.DriveDC.Move(testDrive, -testDrive);
            phaseStart = millis();
            while ((uint32_t) (millis() - phaseStart) < TURN_PROBE_DROP_SETTLE_MS)
            {
                turnProbePump(tickCb);
            }
        }

        // The window opens here. Every timestamp is taken immediately AFTER its yaw read, so each
        // sample and its clock carry the same transport latency and the latency cancels in the
        // deltas.
        double yaw0 = 0.0;
        if (!turnProbeReadYaw(yaw0, F("probe window open")))
        {
            turnProbeStopAndUnwind(tickCb, unwind);
            return sample;
        }
        const uint32_t t0 = millis();
        // On the kicked path this closes the pre-kick span; on the from-rest path the chassis has
        // not moved yet, so this is simply the trial's first checkpoint.
        unwind.fold(yaw0);

        if (kickPwm == 0)
        {
            // From-rest probe only. The kicked path is already running `testPwm` and has already
            // settled on it, so commanding it again here would only add a blocking round-trip.
            MiniR4.DriveDC.Move(testDrive, -testDrive);
        }

        while ((uint32_t) (millis() - t0) < (windowMs / 2u))
        {
            turnProbePump(tickCb);
        }
        double yaw1 = 0.0;
        if (!turnProbeReadYaw(yaw1, F("probe t=W/2")))
        {
            turnProbeStopAndUnwind(tickCb, unwind);
            return sample;
        }
        const uint32_t t1 = millis();
        unwind.fold(yaw1);

        while ((uint32_t) (millis() - t0) < windowMs)
        {
            turnProbePump(tickCb);
        }
        double         yaw2   = 0.0;
        const bool     readOk = turnProbeReadYaw(yaw2, F("probe t=W"));
        const uint32_t t2     = millis();
        if (readOk)
        {
            unwind.fold(yaw2);
        }

        turnProbeStopAndUnwind(tickCb, unwind);

        if (!readOk)
        {
            return sample;
        }

        // MEASURED half durations, never the nominal `windowMs / 2`: a blocking yaw read can
        // stretch either half, and dividing a real delta by an assumed time fabricates a rate.
        const float firstSec  = (float) (uint32_t) (t1 - t0) / 1000.0f;
        const float secondSec = (float) (uint32_t) (t2 - t1) / 1000.0f;
        // Reject a half that is not merely zero-length but implausibly SHORT of its nominal
        // `windowMs / 2`. A pathological stall inside a yaw read (the lower-MCU transport
        // busy-spins up to ~100 ms per read and `readYawDeg` retries once) collapses the
        // following half toward nothing, and a few tenths of a degree of IMU noise divided by a
        // millisecond fabricates a triple-digit rate from a motionless chassis. A quarter of the
        // FULL window -- half of each nominal half -- sits far above any honest scheduling jitter
        // and far below the fabrication regime. An offender is an INVALID sample (printed `NA`),
        // never a pass: `valid` stays false.
        const float minHalfSec = (float) windowMs / 4000.0f;
        if (firstSec < minHalfSec || secondSec < minHalfSec)
        {
            return sample;
        }

        const float firstDeg  = fabsf(turnProbeWrap180((float) (yaw1 - yaw0)));
        const float secondDeg = fabsf(turnProbeWrap180((float) (yaw2 - yaw1)));

        sample.valid           = true;
        sample.totalDeg        = firstDeg + secondDeg;
        sample.omegaFirstDegS  = firstDeg / firstSec;
        sample.omegaSecondDegS = secondDeg / secondSec;
        return sample;
    }

    /// @brief Which floor a sweep is hunting -- this selects the SCORING CRITERION, not just a
    /// label. The two floors are physically different quantities and cannot share one.
    enum class TurnProbeSweepMode : uint8_t
    {
        FROM_REST,       ///< (a1) static/breakaway floor: scored on total degrees vs a threshold.
        WHILE_ROTATING   ///< (a2) kinetic floor: scored on a SUSTAINED RATE. No degree threshold.
    };

    /// @brief Sweep `TURN_PROBE_SWEEP_MIN_PWM..MAX_PWM` step `STEP_PWM`, `TURN_PROBE_TRIALS`
    /// trials per value, printing a copy-pasteable CSV row per trial plus a per-PWM verdict.
    /// "Reliable" is ALL trials passing the mode's criterion -- unanimity, not a majority. The
    /// trials of one step are split across both rotation directions by the greedy unwinder, and a
    /// majority over a split sample is biased toward min(floor+, floor-) when the executor needs
    /// the max. See the verdict line's comment for the simulated failure rate and why the
    /// low-floor error is the silent one.
    ///
    /// @param mode           Selects the criterion; see `TurnProbeSweepMode`.
    /// @param movedThreshDeg (a1) only: the degree threshold, derived from the (d) noise spread
    ///                       by `turnProbeDeriveMovedThresh`. Ignored by (a2).
    /// @param netDeg         Run-wide tether net, carried ACROSS phases -- (a1), (a2) and (b) all
    ///                       unwind against one accumulator, because the tether does not reset
    ///                       between phases. Read and updated once per trial.
    /// @return The lowest reliable PWM, or -1 if none in the swept range qualified.
    /// @note Every trial prints `omegaFirst`/`omegaSecond` even in (a1), where they are only
    ///       diagnostics. They are what lets the operator re-derive the floor offline against
    ///       different thresholds if these prove mistuned -- without re-flashing the robot.
    /// @note `dir` is a CSV column and not merely an internal detail, because the per-PWM verdict
    ///       POOLS whatever directions its trials happened to take (which they take depends on the
    ///       running net, so one step can land 2:1 one way and its neighbour 1:2 the other).
    ///       Pooling is the right default -- the real turn executor turns BOTH ways, so a floor
    ///       measured in one direction only was never the floor it needs; what it needs is
    ///       `max(floor+, floor-)`. But pooling under a MAJORITY rule estimated a split-dependent
    ///       quantity biased toward the MIN instead, which is why the verdict is unanimous; see
    ///       the `reliable` line below.
    ///       The column, and the per-step `(dir+1: p/t  dir-1: p/t)` split now printed on each
    ///       verdict line, are what let the operator see a direction asymmetry AS motor bias
    ///       rather than as the floor. The split print is the one that actually reaches them: a
    ///       warning that lives only in this comment reaches nobody at the bench.
    int16_t turnProbeRunSweep(
        TurnProbeSweepMode mode, float movedThreshDeg, float& netDeg, void (*tickCb)())
    {
        const bool     fromRest = (mode == TurnProbeSweepMode::FROM_REST);
        const int16_t  kickPwm  = fromRest ? (int16_t) 0 : TURN_PROBE_KICK_PWM;
        const uint32_t kickMs   = fromRest ? 0u : TURN_PROBE_KICK_MS;
        const uint32_t windowMs = fromRest ? TURN_PROBE_REST_WINDOW_MS : TURN_PROBE_KINETIC_WINDOW_MS;

        Serial.println(fromRest
                           ? F("\n[T1] --- creep sweep: FROM REST (static/breakaway floor) ---")
                           : F("\n[T1] --- creep sweep: WHILE ROTATING (KINETIC floor) ---"));

        if (fromRest)
        {
            Serial.print(F("[T1] criterion: totalDeg >= "));
            Serial.print(movedThreshDeg, 3);
            Serial.println(F(" deg over the window"));
        }
        else
        {
            Serial.print(F("[T1] criterion: omegaSecond >= "));
            Serial.print(TURN_PROBE_SUSTAIN_FRAC, 2);
            Serial.print(F(" * omegaFirst AND omegaSecond >= "));
            Serial.print(TURN_PROBE_SUSTAIN_RATE_FLOOR_DEG_S, 1);
            Serial.println(F(" deg/s (a coast-down fails both)"));
        }
        // The operator must see this: it is what makes the `dir` column usable rather than
        // decorative, and the reported floor is only as trustworthy as the split behind it.
        Serial.print(F("[T1] a step is RELIABLE only if ALL "));
        Serial.print(TURN_PROBE_TRIALS);
        Serial.println(F(" trials pass. Trials alternate direction to unwind the tether, so"));
        Serial.println(F("[T1]   a step's trials are SPLIT across dir +1/-1 and unanimity means it "
                          "held BOTH ways."));
        Serial.println(F("[T1]   Watch the per-step (dir+1 / dir-1) split below: if one direction "
                          "systematically fails"));
        Serial.println(F("[T1]   while the other passes, that is MOTOR BIAS, not the floor -- "
                          "report it, do not average it away."));
        Serial.println(F("[T1] pwm,trial,dir,totalDeg,omegaFirstDegS,omegaSecondDegS,verdict"));

        int16_t lowestReliablePwm = -1;

        for (int16_t pwm = TURN_PROBE_SWEEP_MIN_PWM; pwm <= TURN_PROBE_SWEEP_MAX_PWM;
             pwm += TURN_PROBE_SWEEP_STEP_PWM)
        {
            uint8_t passTrials  = 0;
            uint8_t trialsPlus  = 0;    // per-direction tallies: the verdict pools directions,
            uint8_t passPlus    = 0;    // so the operator must be able to SEE the split. Without
            uint8_t trialsMinus = 0;    // it, a motor-bias artifact is invisible on the bench and
            uint8_t passMinus   = 0;    // the CSV's `dir` column is an escape hatch nobody knows to use.
            for (uint8_t trial = 0; trial < TURN_PROBE_TRIALS; ++trial)
            {
                // Direction is chosen HERE, per trial, against the run-wide net -- not per PWM
                // step and not on a fixed schedule. `TURN_PROBE_TRIALS` is odd, so a fixed
                // schedule leaves a whole trial uncancelled at every step.
                TurnProbeUnwinder unwind(netDeg);

                const TurnProbeStepSample s =
                    turnProbeMeasureStep(pwm, kickPwm, kickMs, windowMs, unwind, tickCb);

                bool pass = false;
                if (s.valid)
                {
                    if (fromRest)
                    {
                        pass = (s.totalDeg >= movedThreshDeg);
                    }
                    else
                    {
                        // BOTH conditions, always. See TURN_PROBE_SUSTAIN_* for why either one
                        // alone is satisfiable by something that is not sustained rotation.
                        pass = (s.omegaSecondDegS >= TURN_PROBE_SUSTAIN_FRAC * s.omegaFirstDegS)
                               && (s.omegaSecondDegS >= TURN_PROBE_SUSTAIN_RATE_FLOOR_DEG_S);
                    }
                }
                if (pass)
                {
                    ++passTrials;
                }
                if (unwind.dir() > 0)
                {
                    ++trialsPlus;
                    passPlus = (uint8_t) (passPlus + (pass ? 1 : 0));
                }
                else
                {
                    ++trialsMinus;
                    passMinus = (uint8_t) (passMinus + (pass ? 1 : 0));
                }

                Serial.print(F("[T1] "));
                Serial.print(pwm);
                Serial.print(',');
                Serial.print(trial);
                Serial.print(',');
                // Printed even for an invalid sample: the direction was still COMMANDED, so the
                // row stays a fixed 7 columns and an aborted trial is still attributable.
                Serial.print(unwind.dir() > 0 ? F("+1") : F("-1"));
                Serial.print(',');
                if (!s.valid)
                {
                    Serial.println(F("NA,NA,NA,NA"));
                    continue;
                }
                Serial.print(s.totalDeg, 2);
                Serial.print(',');
                Serial.print(s.omegaFirstDegS, 2);
                Serial.print(',');
                Serial.print(s.omegaSecondDegS, 2);
                Serial.print(',');
                Serial.println(pass ? F("YES") : F("no"));
            }

            // UNANIMOUS, not a strict majority. The greedy unwinder picks each trial's direction
            // against the run-wide net, so a step's trials are SPLIT across both directions. A
            // 2-of-3 majority over a split sample does not estimate the max(floor+, floor-) the
            // turn executor actually needs -- it estimates a split-dependent quantity biased
            // toward the MIN. Simulated against a 4-PWM motor bias (this chassis has a Drive bias
            // calibration precisely because the motors are not symmetric), a majority latched the
            // floor LOW ~49% of the time, avg 1.6 steps; unanimity latched low 0%.
            //
            // The asymmetry decides it. A floor set LOW is the SILENT failure: the executor creeps
            // at a PWM that cannot sustain rotation, so every turn stalls short of tolerance and
            // hunts to its 3 s timeout -- which is non-faulting BY DESIGN, so nothing reports it.
            // A floor set HIGH is loud (AC8's static_assert) and is absorbed by the +5 headroom in
            // TURN_CREEP_THRESH_PWM. Unanimity trades a ~49% chance of a silently-wrong number for
            // a ~15% chance of a conservative one.
            //
            // Unanimity also IMPLIES both directions passed, for free -- and structurally, not
            // just as a matter of plant numbers. `dir=-1` requires net > 0, so a -1 trial leaves
            // net' = net - r > -r, and +1 trials only raise net; hence net > -R_prior always.
            // Three same-direction trials therefore need R_prior > r1 + r2. For a step that is
            // UNANIMOUS, r1/r2 are passing rotations at this pwm, and rotation is non-decreasing
            // in pwm, so R_prior <= r_pass(pwm) < r1 + r2. Unreachable, with ~2x margin, in both
            // the all-+1 and (symmetrically) the all--1 case. Note the qualifier: three same-
            // direction FAILING trials are perfectly reachable (a dead-low step with r == 0 never
            // moves net, so it takes dir=+1 every time) -- such a step just can never be unanimous.
            // So this costs no extra trials and does not touch direction selection: the
            // |net| <= R drift bound that keeps the tether unwound stands untouched.
            const bool reliable = (passTrials == TURN_PROBE_TRIALS);

            Serial.print(F("[T1]   pwm="));
            Serial.print(pwm);
            Serial.print(F(" passTrials="));
            Serial.print(passTrials);
            Serial.print('/');
            Serial.print(TURN_PROBE_TRIALS);
            Serial.print(F("  (dir+1: "));
            Serial.print(passPlus);
            Serial.print('/');
            Serial.print(trialsPlus);
            Serial.print(F("  dir-1: "));
            Serial.print(passMinus);
            Serial.print('/');
            Serial.print(trialsMinus);
            Serial.print(')');
            Serial.println(reliable ? F("  -> RELIABLE") : F("  -> not reliable"));

            if (reliable && lowestReliablePwm < 0)
            {
                lowestReliablePwm = pwm;
            }
        }

        Serial.print(fromRest ? F("[T1] lowest pwm that BROKE AWAY from rest = ")
                              : F("[T1] lowest pwm that SUSTAINED rotation = "));
        if (lowestReliablePwm >= 0)
        {
            Serial.println(lowestReliablePwm);
        }
        else if (fromRest)
        {
            Serial.println(F("NOT FOUND"));
            Serial.println(F("[T1]   The chassis did not break away at ANY swept pwm, including "
                              "values above Config::PWM_MIN_MAGNITUDE. FIRST rule out dropped "
                              "samples: a step needs ALL trials to pass, so one 'NA' row sinks it. "
                              "Check the passTrials= counts above -- if steps read 2/3 with an NA, "
                              "that is a yaw-read dropout, NOT the plant. Otherwise this is a "
                              "HARDWARE symptom, not a range problem: check battery, wiring and "
                              "motor mapping before trusting anything else this run printed."));
        }
        else
        {
            Serial.println(F("NOT FOUND"));
            Serial.println(F("[T1]   No swept pwm sustained rotation. FIRST rule out dropped "
                              "samples: a step needs ALL trials to pass, so one 'NA' row sinks it "
                              "-- check the passTrials= counts above. Otherwise read the "
                              "omegaSecondDegS column: if it is non-trivial but the verdict is "
                              "'no', the sustain thresholds are mistuned and the floor can be "
                              "re-derived from this CSV offline -- no re-flash needed."));
        }

        turnProbePrintTetherNet(netDeg);
        return lowestReliablePwm;
    }

    /// @brief (W4) Derive (a1)'s "did it move" degree threshold from the phase-(d) noise
    /// measurement: `max(TURN_PROBE_MOVED_THRESH_MIN_DEG, k * measuredSpread)`.
    ///
    /// @note This is why (d) runs first. A threshold fixed before the noise it must clear is
    ///       measured is a guess; deriving it makes (a1) self-validating at zero bench cost.
    /// @note (d)'s spread comes from a ~2 s sampling window while (a1)'s window is 400 ms, so
    ///       more drift accumulates into the spread than (a1) can actually see. That biases the
    ///       threshold UP and hence the breakaway floor UP -- conservative, and (a1) is the
    ///       reference-only phase, so a conservative bound is the right error direction.
    float turnProbeDeriveMovedThresh(const TurnProbeResults& results)
    {
        Serial.println(F("\n[T1] --- (a1) 'moved' threshold, derived from the (d) noise spread ---"));

        float thresh = TURN_PROBE_MOVED_THRESH_MIN_DEG;
        if (!results.imuValid)
        {
            Serial.print(F("[T1] WARNING: (d) measured nothing -- falling back to the bare floor "));
            Serial.println(thresh, 2);
            return thresh;
        }

        const float noiseThresh = TURN_PROBE_MOVED_THRESH_NOISE_K * (float) results.imuSpreadDeg;
        if (noiseThresh > thresh)
        {
            thresh = noiseThresh;
        }

        Serial.print(F("[T1] max(floor "));
        Serial.print(TURN_PROBE_MOVED_THRESH_MIN_DEG, 2);
        Serial.print(F(", "));
        Serial.print(TURN_PROBE_MOVED_THRESH_NOISE_K, 1);
        Serial.print(F(" x spread "));
        Serial.print((float) results.imuSpreadDeg, 3);
        Serial.print(F(" = "));
        Serial.print(noiseThresh, 3);
        Serial.print(F(") -> "));
        Serial.print(thresh, 3);
        Serial.println(F(" deg"));
        return thresh;
    }

    /// @brief (b) Time a `TURN_PROBE_OMEGA_TARGET_DEG` in-place rotation at the measured kinetic
    /// creep PWM (falls back to the breakaway PWM if the kinetic sweep found none) to compute
    /// omega in deg/s, AND measure the turn-representative tick from the loop's own iterations.
    ///
    /// Accumulates |wrapped per-poll delta| rather than a single before/after diff, since polling
    /// cannot land exactly on the target degree.
    ///
    /// @note KICKS BEFORE CREEPING. The creep PWM is the KINETIC floor and therefore sits BELOW
    ///       the static breakaway floor by construction -- commanding it from rest would never
    ///       break away, and the phase would spend its whole timeout measuring a motionless
    ///       chassis. The clock starts only after the drop has settled, so the kick's own
    ///       rotation is never credited to the creep.
    /// @note A TIMEOUT IS NOT A SLOW MEASUREMENT -- it is the absence of one, and this function
    ///       must never let one become an omega. `accumulatedDeg` sums ABSOLUTE deltas, so a
    ///       motionless chassis random-walks it upward on IMU noise alone; it can never reach 0
    ///       and is not evidence of rotation at any value. Reaching the target is the only
    ///       honest criterion.
    /// @note Takes its direction from the same greedy rule as the sweeps, against the same
    ///       run-wide net -- this phase is worth ~0.3 rev and the tether does not care which
    ///       phase produced the wind-up. `accumulatedDeg` already sums |per-poll delta|, so the
    ///       target test, the timeout test and omega itself are magnitudes and read identically
    ///       in either direction; omega is reported as a positive rate whichever way it spun.
    void turnProbeMeasureOmega(TurnProbeResults& results, float& netDeg, void (*tickCb)())
    {
        Serial.println(F("\n[T1] === (b) omega + turn-tick at the creep PWM ==="));

        const int16_t pwm = (results.sustainPwm > 0) ? results.sustainPwm : results.breakawayPwm;
        if (pwm <= 0)
        {
            Serial.println(F("[T1] SKIPPED: no valid creep pwm from the sweeps above"));
            return;    // nothing was commanded, so the net is untouched -- no commit to make
        }

        // Direction chosen once, here, and shared by this phase's kick AND its creep.
        TurnProbeUnwinder unwind(netDeg);
        const int16_t     kickDrive  = unwind.pwm(TURN_PROBE_KICK_PWM);
        const int16_t     creepDrive = unwind.pwm(pwm);

        Serial.print(F("[T1] direction="));
        Serial.print(unwind.dir() > 0 ? F("+1") : F("-1"));
        Serial.println(F(" (greedy tether unwind)"));

        Serial.print(F("[T1] kick pwm="));
        Serial.print(TURN_PROBE_KICK_PWM);
        Serial.print(F(" for "));
        Serial.print(TURN_PROBE_KICK_MS);
        Serial.print(F(" ms -> drop to pwm="));
        Serial.print(pwm);
        Serial.print(F(" -> settle "));
        Serial.print(TURN_PROBE_DROP_SETTLE_MS);
        Serial.println(F(" ms -> THEN start the clock"));

        // Unwind checkpoint before the kick -- the kick and the drop-settle are ~78 deg of
        // rotation that the measured window never sees. Same reason as `turnProbeMeasureStep`.
        double yawPreKick = 0.0;
        if (turnProbeReadYaw(yawPreKick, F("unwind pre-kick")))
        {
            unwind.fold(yawPreKick);
        }

        MiniR4.DriveDC.Move(kickDrive, -kickDrive);
        uint32_t phaseStart = millis();
        while ((uint32_t) (millis() - phaseStart) < TURN_PROBE_KICK_MS)
        {
            turnProbePump(tickCb);
        }

        // Drop, then let the coast-down transient finish BEFORE the clock starts. Same idiom as
        // the (a2) sweep's kicked path -- see `turnProbeMeasureStep`.
        MiniR4.DriveDC.Move(creepDrive, -creepDrive);
        phaseStart = millis();
        while ((uint32_t) (millis() - phaseStart) < TURN_PROBE_DROP_SETTLE_MS)
        {
            turnProbePump(tickCb);
        }

        double yawStart = 0.0;
        if (!turnProbeReadYaw(yawStart, F("omega start")))
        {
            Serial.println(F("[T1] ABORTED: could not read starting yaw"));
            turnProbeStopAndUnwind(tickCb, unwind);
            return;
        }

        const uint32_t start          = millis();
        float          accumulatedDeg = 0.0f;
        double         yawPrev        = yawStart;
        uint32_t       iterations     = 0;
        bool           timedOut       = false;

        // Folded only AFTER the clock has started, so the read -> `millis()` adjacency the omega
        // measurement depends on stays exactly as it was. `yawStart` is already captured; the
        // unwinder is reading it, not re-reading the IMU.
        unwind.fold(yawStart);

        while (accumulatedDeg < TURN_PROBE_OMEGA_TARGET_DEG)
        {
            if ((uint32_t) (millis() - start) >= TURN_PROBE_OMEGA_TIMEOUT_MS)
            {
                timedOut = true;
                break;    // before ++iterations: this pass did no work and must not be counted
            }

            turnProbePump(tickCb);
            // Re-issue the command EVERY iteration. A real PD turn writes Move(L,R) per tick and
            // that write is a blocking SoftwareSerial round-trip; a loop that merely HOLDS a
            // setpoint omits it and under-measures the tick that actually applies during a turn.
            // Safe to re-send: `Move` is idempotent (unlike `MoveDegs`, whose vendor retry loop
            // re-sends and doubles the dose). `creepDrive` is the SAME value dropped to above --
            // re-sending it holds the setpoint rather than changing it.
            MiniR4.DriveDC.Move(creepDrive, -creepDrive);

            double yawNow = 0.0;
            if (turnProbeReadYaw(yawNow, F("omega poll")))
            {
                // |delta|: omega is a RATE and must come out positive whichever way this phase
                // was sent. The unwinder takes the same successful read and applies the sign
                // itself, from the command.
                accumulatedDeg += fabsf(turnProbeWrap180((float) (yawNow - yawPrev)));
                yawPrev = yawNow;
                unwind.fold(yawNow);
            }
            ++iterations;
        }

        // Re-read the clock AFTER the loop. Sampling it at the TOP of an iteration closes the
        // time interval one pump + one yaw read BEFORE the degree interval closes, so elapsed
        // under-measures and omega comes out high.
        const uint32_t elapsedMs = (uint32_t) (millis() - start);

        turnProbeStopAndUnwind(tickCb, unwind);
        turnProbePrintTetherNet(netDeg);

        // The turn-representative tick is honest whether or not the rotation reached the target:
        // every counted iteration did the same pump + Move + yaw-read work either way.
        if (iterations > 0 && elapsedMs > 0)
        {
            results.turnTickAvgUs      = ((float) elapsedMs * 1000.0f) / (float) iterations;
            results.turnTickIterations = iterations;
            results.turnTickValid      = true;

            Serial.print(F("[T1] turn tick: "));
            Serial.print(iterations);
            Serial.print(F(" iterations in "));
            Serial.print(elapsedMs);
            Serial.print(F(" ms -> "));
            Serial.print(results.turnTickAvgUs, 1);
            Serial.println(F(" us/tick (pump + Move write + yaw read)"));
        }

        const bool reachedTarget = (accumulatedDeg >= TURN_PROBE_OMEGA_TARGET_DEG);
        if (timedOut || !reachedTarget || elapsedMs == 0)
        {
            Serial.print(F("[T1] omega NOT MEASURED -- did not reach "));
            Serial.print(TURN_PROBE_OMEGA_TARGET_DEG, 0);
            Serial.print(F(" deg within "));
            Serial.print(elapsedMs);
            Serial.println(F(" ms"));
            Serial.print(F("[T1]   DIAGNOSTIC ONLY, NOT A RESULT -- do NOT derive any Config "
                            "value from this number: accumulated "));
            Serial.print(accumulatedDeg, 2);
            Serial.println(F(" deg of |per-poll delta|"));
            Serial.println(F("[T1]   That accumulator sums ABSOLUTE deltas, so a MOTIONLESS "
                              "chassis still climbs several deg on IMU noise alone. It is not "
                              "evidence of rotation."));
            Serial.println(F("[T1]   Likely cause: the creep pwm above does not actually sustain "
                              "rotation. Re-check the (a2) omegaSecondDegS column."));
            return;    // omegaValid stays false -> the summary prints NOT MEASURED
        }

        const float elapsedSec = (float) elapsedMs / 1000.0f;
        const float omega      = accumulatedDeg / elapsedSec;

        Serial.print(F("[T1] rotated "));
        Serial.print(accumulatedDeg, 2);
        Serial.print(F(" deg in "));
        Serial.print(elapsedMs);
        Serial.print(F(" ms -> omega = "));
        Serial.print(omega, 3);
        Serial.println(F(" deg/s"));

        results.omegaDegPerSec = omega;
        results.omegaValid     = true;
    }

    /// @brief (c) Re-confirm the IDLE `cooperativeTick()` period on this build: no chassis
    /// motion, just min/avg/max microsecond deltas between consecutive tick-callback
    /// invocations, mirroring `inoTest/tick_bench`'s `TICK us min=... avg=... max=...` shape.
    ///
    /// @note REFERENCE ONLY -- this is NOT the tick a turn runs at, and `TURN_MIN_RESOLVABLE_DEG`
    ///       must NOT be derived from it. `Drive` is idle here, whereas a real PD turn adds a
    ///       blocking yaw read AND a blocking `Move(L,R)` write per tick. This figure therefore
    ///       under-measures the turn's tick, which would bias `TURN_MIN_RESOLVABLE_DEG` low --
    ///       the same optimistic direction that makes the AC8 asserts pass when they should not.
    ///       Phase (b) measures the tick that actually applies; the summary uses that one.
    void turnProbeMeasureTick(TurnProbeResults& results, void (*tickCb)())
    {
        Serial.println(F("\n[T1] === (c) IDLE tick-period re-confirm ==="));

        if (tickCb == nullptr)
        {
            Serial.println(F("[T1] SKIPPED: no tick callback registered "
                              "(turn_probe.ino should always pass cooperativeTick here)"));
            return;
        }

        uint32_t minUs = 0xFFFFFFFFu;
        uint32_t maxUs = 0;
        uint64_t sumUs = 0;

        uint32_t prevUs = micros();
        for (uint16_t i = 0; i < TURN_PROBE_TICK_SAMPLE_COUNT; ++i)
        {
            tickCb();
            yield();
            const uint32_t nowUs   = micros();
            const uint32_t deltaUs = nowUs - prevUs;    // unsigned wrap-safe
            prevUs                 = nowUs;

            if (i > 0)    // first sample has no valid predecessor delta
            {
                if (deltaUs < minUs)
                {
                    minUs = deltaUs;
                }
                if (deltaUs > maxUs)
                {
                    maxUs = deltaUs;
                }
                sumUs += deltaUs;
            }
        }

        const float avgUs = (float) sumUs / (float) (TURN_PROBE_TICK_SAMPLE_COUNT - 1);

        Serial.print(F("[T1] TICK us min="));
        Serial.print(minUs);
        Serial.print(F(" avg="));
        Serial.print(avgUs, 1);
        Serial.print(F(" max="));
        Serial.println(maxUs);

        // Print the known-good figure NEXT TO the fresh one. Without it, an inflated tick is
        // invisible unless someone independently remembers what this build used to measure.
        Serial.print(F("[T1]   reference avg="));
        Serial.print(TURN_PROBE_TICK_REF_AVG_US, 0);
        Serial.println(F(" us (measured 2026-07-16 via inoTest/tick_bench)"));
        Serial.println(F("[T1]   If avg reads materially HIGHER (~9.6 ms), an extra blocking yaw "
                          "read is landing in every tick: Localization::update() self-polls when "
                          "it is zeroed AND nav is idle, which is exactly this harness's state, "
                          "so zeroing Localization before running adds ~1.6 ms/tick that a real "
                          "nav turn does not pay. Not a fault and nothing here needs re-running "
                          "-- but (c) is then not comparable to the reference, and (b)'s TURN "
                          "tick carries the same inflation, which biases TURN_MIN_RESOLVABLE_DEG "
                          "HIGH (the safe direction). Power-cycle without zeroing to compare."));

        results.tickMinUs = minUs;
        results.tickMaxUs = maxUs;
        results.tickAvgUs = avgUs;
        results.tickValid = true;
    }

    /// @brief (d) Read yaw `TURN_PROBE_IMU_SAMPLE_COUNT` times while stationary, reporting
    /// min/max/spread. Assumes the true heading stays well clear of the +-180 wrap boundary
    /// over the sampling window (a stationary bench probe never crosses it in practice).
    void turnProbeMeasureImuNoise(TurnProbeResults& results, void (*tickCb)())
    {
        Serial.println(F("\n[T1] === (d) IMU noise (100 stationary reads) ==="));

        double   minDeg    = 0.0;
        double   maxDeg    = 0.0;
        bool     haveFirst = false;
        uint16_t okCount    = 0;
        uint16_t failCount  = 0;

        for (uint16_t i = 0; i < TURN_PROBE_IMU_SAMPLE_COUNT; ++i)
        {
            double yaw = 0.0;
            if (turnProbeReadYaw(yaw, F("imu noise")))
            {
                if (!haveFirst)
                {
                    minDeg    = maxDeg = yaw;
                    haveFirst = true;
                }
                else
                {
                    if (yaw < minDeg)
                    {
                        minDeg = yaw;
                    }
                    if (yaw > maxDeg)
                    {
                        maxDeg = yaw;
                    }
                }
                ++okCount;
            }
            else
            {
                ++failCount;
            }

            const uint32_t start = millis();
            while ((uint32_t) (millis() - start) < TURN_PROBE_IMU_SAMPLE_GAP_MS)
            {
                turnProbePump(tickCb);
            }
        }

        Serial.print(F("[T1] IMU samples ok="));
        Serial.print(okCount);
        Serial.print(F(" failed="));
        Serial.println(failCount);

        if (!haveFirst)
        {
            Serial.println(F("[T1] IMU noise measurement invalid -- every read failed"));
            return;
        }

        const double spread = maxDeg - minDeg;
        Serial.print(F("[T1] IMU yaw min="));
        Serial.print(minDeg, 3);
        Serial.print(F(" max="));
        Serial.print(maxDeg, 3);
        Serial.print(F(" spread="));
        Serial.println(spread, 3);

        results.imuMinDeg    = minDeg;
        results.imuMaxDeg    = maxDeg;
        results.imuSpreadDeg = spread;
        results.imuValid     = true;
    }

    /// @brief Final copy-pasteable block naming the three Config outputs this bench feeds
    /// (T3's `TURN_CREEP_PWM` / `TURN_CREEP_THRESH_PWM` / `TURN_MIN_RESOLVABLE_DEG`), plus the
    /// raw measurements they are derived from.
    ///
    /// @note Every line is labelled MEASURED, SUGGESTED or NOT MEASURED, and that labelling is
    ///       load-bearing: nothing downstream re-questions a number taken from this block.
    void turnProbePrintSummary(const TurnProbeResults& results, float movedThreshDeg)
    {
        Serial.println(F("\n[T1] ================= SUMMARY ================="));

        Serial.print(F("[T1] [MEASURED] static/breakaway floor (from-rest sweep, thresh "));
        Serial.print(movedThreshDeg, 2);
        Serial.print(F(" deg; reference only) = "));
        if (results.breakawayPwm > 0)
        {
            Serial.println(results.breakawayPwm);
        }
        else
        {
            Serial.println(F("NOT MEASURED"));
        }

        Serial.print(F("[T1] [MEASURED] TURN_CREEP_PWM (kinetic floor, sustained-rate sweep) = "));
        if (results.sustainPwm > 0)
        {
            Serial.println(results.sustainPwm);
        }
        else
        {
            Serial.println(F("NOT MEASURED -- see the (a2) verdict above"));
        }

        Serial.print(F("[T1] [SUGGESTED] TURN_CREEP_THRESH_PWM (bench does not measure this "
                        "directly -- pick headroom above TURN_CREEP_PWM so the PD has room to "
                        "modulate before snapping; mentor used +5) = "));
        if (results.sustainPwm > 0)
        {
            Serial.println(results.sustainPwm + TURN_PROBE_CREEP_THRESH_HEADROOM_PWM);
        }
        else
        {
            Serial.println(F("NOT COMPUTABLE -- TURN_CREEP_PWM not measured"));
        }

        Serial.print(F("[T1] [MEASURED] omega at the creep pwm = "));
        if (results.omegaValid)
        {
            Serial.print(results.omegaDegPerSec, 3);
            Serial.println(F(" deg/s"));
        }
        else
        {
            Serial.println(F("NOT MEASURED"));
        }

        Serial.print(F("[T1] [MEASURED] tick period, TURN / phase (b) -- pump + Move + yaw read, "
                        "the tick a real PD turn runs at = "));
        if (results.turnTickValid)
        {
            Serial.print(results.turnTickAvgUs, 1);
            Serial.print(F(" us  (over "));
            Serial.print(results.turnTickIterations);
            Serial.println(F(" iterations)"));
        }
        else
        {
            Serial.println(F("NOT MEASURED"));
        }

        Serial.print(F("[T1] [MEASURED] tick period, IDLE / phase (c) -- Drive idle, REFERENCE "
                        "ONLY, not used below = "));
        if (results.tickValid)
        {
            Serial.print(results.tickAvgUs, 1);
            Serial.print(F(" us  (reference "));
            Serial.print(TURN_PROBE_TICK_REF_AVG_US, 0);
            Serial.println(F(" us -- see the (c) note above if these differ materially)"));
        }
        else
        {
            Serial.println(F("NOT MEASURED"));
        }

        // Derived from the TURN tick, never the idle one: the idle tick omits the per-tick yaw
        // read and Move write, so it under-measures and would bias this value LOW -- the exact
        // direction that makes the AC8 static_asserts pass a tolerance the plant cannot hold.
        Serial.print(F("[T1] [MEASURED] TURN_MIN_RESOLVABLE_DEG (= omega * TURN tick from (b)) = "));
        if (results.omegaValid && results.turnTickValid)
        {
            const float minResolvableDeg = results.omegaDegPerSec * (results.turnTickAvgUs / 1.0e6f);
            Serial.println(minResolvableDeg, 4);
        }
        else
        {
            Serial.println(F("NOT MEASURED -- omega and/or the turn tick is missing. Do NOT "
                              "substitute the idle tick from (c), and do NOT ship T3's "
                              "placeholder as if it were bench data: re-run this probe."));
        }

        Serial.print(F("[T1] [MEASURED] IMU yaw noise spread (100 stationary reads) = "));
        if (results.imuValid)
        {
            Serial.print(results.imuSpreadDeg, 3);
            Serial.println(F(" deg"));
        }
        else
        {
            Serial.println(F("NOT MEASURED"));
        }

        Serial.println(F("[T1] =============================================="));
    }
}    // namespace

void runTurnProbe(void (*tickCb)())
{
    Serial.println(F("\n[T1] ############################################################"));
    Serial.println(F("[T1] #  Turn creep-floor / tick-period / IMU-noise bench probe    #"));
    Serial.println(F("[T1] ############################################################"));

    Drive.stop(true);

    // Mute the 500 ms heartbeat for the whole probe: this bench pumps `cooperativeTick()` every
    // iteration (phase (b)'s turn tick MUST include that work to stay representative), so the
    // heartbeat would otherwise interleave `HB ...` lines through the important [T1] output. A
    // blocking heartbeat print would also stretch a phase (b) iteration and inflate the measured
    // tick, so muting it makes that measurement more honest, not less. Restored before return.
    EventTrace.setHeartbeatEnabled(false);

    if (tickCb == nullptr)
    {
        Serial.println(F("[T1] WARNING: no tick callback passed -- only Drive.update() "
                          "will tick during waits. turn_probe.ino should always pass "
                          "cooperativeTick to runTurnProbe()."));
    }

    TurnProbeResults results;

    // Run-wide tether net, in "dir = +1" units. Deliberately spans ALL THREE motion phases: the
    // USB cable does not unwind between phases, so (a1), (a2) and (b) unwind against one
    // accumulator. See `TurnProbeUnwinder`.
    float netDeg = 0.0f;

    // Phases run (d), (c), (a1), (a2), (b) -- NOT in a/b/c/d lettering order. (d) must precede
    // (a1) because (a1)'s degree threshold is derived from (d)'s measured noise; (c) joins it
    // because both are stationary, so the chassis is set down once and repositioned once.
    turnProbeWaitForBtnUp(
        tickCb,
        F("[T1] PHASE 1/5 -- (d) IMU noise, 100 stationary reads.\n"
          "[T1]   Set the chassis down INSIDE a clear circle (later phases spin it in place).\n"
          "[T1]   Leave the USB cable some slack: the spins alternate direction and keep the\n"
          "[T1]   net wind-up BOUNDED (under ~2/3 of a turn), but they do not cancel it.\n"
          "[T1]   Then KEEP IT PERFECTLY STILL -- do not touch the robot or knock the bench\n"
          "[T1]   until this phase has printed its spread. This number sets the (a1) threshold."));
    turnProbeMeasureImuNoise(results, tickCb);

    turnProbeWaitForBtnUp(
        tickCb,
        F("[T1] PHASE 2/5 -- (c) IDLE tick period.\n"
          "[T1]   Leave the chassis exactly where it is. No motion, no repositioning."));
    turnProbeMeasureTick(results, tickCb);

    const float movedThreshDeg = turnProbeDeriveMovedThresh(results);

    turnProbeWaitForBtnUp(
        tickCb,
        F("[T1] PHASE 3/5 -- (a1) creep sweep FROM REST (static/breakaway floor).\n"
          "[T1]   The chassis now spins in place, 20 pwm values x 3 trials (~45 s).\n"
          "[T1]   Each trial picks whichever direction unwinds the USB tether, so the spins\n"
          "[T1]   swap back and forth and the chassis stays put -- that is expected.\n"
          "[T1]   Clear a circle around it. ALL WHEELS ON THE GROUND -- a lifted wheel\n"
          "[T1]   measures the wrong floor."));
    results.breakawayPwm =
        turnProbeRunSweep(TurnProbeSweepMode::FROM_REST, movedThreshDeg, netDeg, tickCb);

    turnProbeWaitForBtnUp(
        tickCb,
        F("[T1] PHASE 4/5 -- (a2) creep sweep WHILE ROTATING (KINETIC floor -> TURN_CREEP_PWM).\n"
          "[T1]   Confirm the chassis is still roughly centred in its clear circle -- the\n"
          "[T1]   alternating direction bounds the drift, it does not zero it. Nudge it back\n"
          "[T1]   only if it has actually crept out; leave the cable slack either way.\n"
          "[T1]   It kicks, drops, settles then creeps, 20 x 3 times (~2 min). This is the\n"
          "[T1]   phase that produces TURN_CREEP_PWM."));
    results.sustainPwm =
        turnProbeRunSweep(TurnProbeSweepMode::WHILE_ROTATING, 0.0f, netDeg, tickCb);

    turnProbeWaitForBtnUp(
        tickCb,
        F("[T1] PHASE 5/5 -- (b) omega + turn tick at the creep PWM.\n"
          "[T1]   Confirm the chassis is still roughly centred. It kicks, then creeps 90 deg\n"
          "[T1]   in place, in whichever direction unwinds the tether."));
    turnProbeMeasureOmega(results, netDeg, tickCb);

    turnProbePrintSummary(results, movedThreshDeg);

    EventTrace.setHeartbeatEnabled(true);

    Serial.println(F("\n[T1] probe complete -- press RESET to run again."));
}
