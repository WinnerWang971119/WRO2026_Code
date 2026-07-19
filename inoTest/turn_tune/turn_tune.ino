/**
 * @file turn_tune.ino
 * @brief Standalone bench harness for tuning the turn PD gains against real closed-loop
 *        `Drive.turnDeg()` accuracy (docs/plans/2026-07-16-turn-pd-controller.md, task T11).
 * @author WRO 2026
 *
 * @details
 * The companion to `inoTest/turn_probe`. The probe measures the OPEN-LOOP plant constants
 * (creep floors, turn tick, IMU noise); this harness runs the assembled PD executor over and
 * over and reports how accurately it lands, so the `Config::TURN_*` gains can be bench-tuned.
 * The tuning loop is: flash -> BTN_UP -> read the residual summary -> edit a `TURN_*` gain in
 * `WRO2026/config/ConfigDrive.h` -> reflash. The measurement logic lives in
 * `TurnTuneHarness.cpp`, not here — see that file's header for the full contract.
 *
 * CRITICAL: the sync-blocking `Drive.turnDeg()` spins re-enter `cooperativeTick()` every
 * iteration, so `cooperativeTick()` below MUST tick the same modules, in the same order, as
 * `WRO2026.ino::cooperativeTick()` — otherwise the PD turn dynamics measured here are not what
 * a real nav turn pays. If a module is added to or removed from `WRO2026.ino::cooperativeTick()`,
 * mirror the change here too (the same rule `inoTest/turn_probe` and `inoTest/tick_bench` follow).
 *
 * BUILD MECHANIC — see `inoTest/tick_bench/tick_bench.ino`'s header for why each
 * `WRO2026/src/<domain>` folder needs its own `--library` flag; this sketch uses the identical
 * six domains for the identical reason.
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/turn_tune/build.py` (add `--build-only` to compile
 *       without touching a connected board). Log STRAIGHT to a file, not `| tee`, or the R4 USB
 *       CDC backpressures and freezes mid-run — see the turn-PD branch's bench notes.
 * @note Serial diagnostics run at 115200 baud, matching `WRO2026.ino`; `setup()` waits up to
 *       3 s for a USB CDC listener but does not hang a headless boot.
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only
 *       local check that this links.
 */

#include <MatrixMiniR4.h>

#include "Config.h"
#include "Drive.h"
#include "EventTrace.h"
#include "Intersection.h"
#include "LineFollower.h"
#include "Localization.h"
#include "Manipulator.h"
#include "Mission.h"
#include "PathTracker.h"
#include "Tasks.h"
#include "TurnTuneHarness.h"
#include "Vision.h"
#include "WROLineArray.h"
#include "YawSource.h"

/**
 * @brief Cooperative tick — every module's `update()` exactly once, in the SAME order as
 *        `WRO2026.ino::cooperativeTick()`.
 *
 * Deliberate, harmless simplification vs. the production sketch: `WRO2026.ino` guards its final
 * `EventTrace.update()` behind the calibration-menu state so the on-OLED menu is not clobbered.
 * This sketch has no calibration menu, so the guard is always true here and is omitted;
 * `EventTrace.update()` runs unconditionally, doing exactly the same per-tick work.
 *
 * Order: YawSource -> LineArray -> Drive -> LineFollower -> Intersection -> Manipulator -> Tasks ->
 *     PathTracker -> Localization -> Mission -> EventTrace.
 */
void cooperativeTick()
{
    // NOTE: mirror any change to WRO2026.ino::cooperativeTick() here too (and in
    // inoTest/turn_probe + inoTest/tick_bench) or this harness's measured turn dynamics silently
    // drift from what a real nav turn pays.

    // YawSource.update() runs FIRST, as in production: one gated gyro read refreshes the
    // tick-scoped yaw cache so Drive's in-turn readYawDeg (and this harness's own reads) serve
    // that single device transaction instead of each triggering an inline re-read (#163, AC-5).
    YawSource.update();
    LineArray.update();
    Drive.update();
    LineFollower.update();
    Intersection.update();
    Manipulator.update();
    Tasks.update();
    PathTracker.update();
    Localization.update();
    Mission.update();
    EventTrace.update();
}

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }    // give USB CDC a moment, but don't hang headless

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // All 12 begin()s, in the same order as WRO2026.ino::setup(). On a false return, print and
    // CONTINUE (do not halt) — a degraded module still lets the rest of the harness run, and the
    // failure is visible on Serial either way.
    // YawSource.begin() is NOT optional here: it is the only place the gyro's I2C transport
    // (Wire1 + mux channel) is wired, and Drive.turnDeg() closes its loop on that yaw read.
    if (!LineArray.begin())    { Serial.println(F("[turn_tune] LineArray begin() FAILED")); }
    if (!YawSource.begin())    { Serial.println(F("[turn_tune] YawSource begin() FAILED")); }
    if (!Drive.begin())        { Serial.println(F("[turn_tune] Drive begin() FAILED")); }
    if (!LineFollower.begin()) { Serial.println(F("[turn_tune] LineFollower begin() FAILED")); }
    if (!Intersection.begin()) { Serial.println(F("[turn_tune] Intersection begin() FAILED")); }
    if (!Manipulator.begin())  { Serial.println(F("[turn_tune] Manipulator begin() FAILED")); }
    if (!Localization.begin()) { Serial.println(F("[turn_tune] Localization begin() FAILED")); }
    if (!PathTracker.begin())  { Serial.println(F("[turn_tune] PathTracker begin() FAILED")); }
    if (!Mission.begin())      { Serial.println(F("[turn_tune] Mission begin() FAILED")); }
    if (!Tasks.begin())        { Serial.println(F("[turn_tune] Tasks begin() FAILED")); }
    if (!Vision.begin())       { Serial.println(F("[turn_tune] Vision begin() FAILED")); }
    if (!EventTrace.begin())   { Serial.println(F("[turn_tune] EventTrace begin() FAILED")); }

    // Full tick-callback wiring, matching WRO2026.ino::setup() exactly — every sync-blocking
    // drain (including turnDeg's) re-enters cooperativeTick per spin iteration.
    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);
    LineFollower.setTickCallback(cooperativeTick);
    Manipulator.setTickCallback(cooperativeTick);
    Tasks.setTickCallback(cooperativeTick);
    Vision.setTickCallback(cooperativeTick);

    // The harness gates its first turn behind a BTN_UP prompt (runTurnTune ->
    // turnTuneWaitForBtnUp), so calling it once here already matches the "arm on BTN_UP" UX:
    // nothing drives until the operator has positioned the chassis and pressed BTN_UP. This call
    // blocks for the whole run, pumping cooperativeTick() throughout via the tickCb parameter.
    runTurnTune(cooperativeTick);
}

void loop()
{
    // The harness is one-shot per power cycle (its closing line says "press RESET to run again")
    // — once setup() returns, keep every module ticking so Serial/OLED stay alive for the
    // operator to read the summary.
    cooperativeTick();
}
