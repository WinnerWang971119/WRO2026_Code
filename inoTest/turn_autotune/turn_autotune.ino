/**
 * @file turn_autotune.ino
 * @brief Standalone on-robot Twiddle auto-tuner for the turn PD gains
 *        (docs/plans/2026-07-16-turn-pd-controller.md, task T13).
 * @author WRO 2026
 *
 * @details
 * The third turn-PD bench sketch. `turn_probe` measures the open-loop plant; `turn_tune` reports
 * the REAL `Drive.turnDeg()` accuracy at a fixed gain set; this one SEARCHES the gains
 * automatically on the robot and prints the winner to paste into `WRO2026/config/ConfigDrive.h`.
 * `Drive.cpp` is never modified — the search runs a runtime-gain replica of the executor built
 * from the same shared `TurnPDPolicy.h` / `TurnSignPolicy.h` math. The measurement + search logic
 * lives in `TurnAutotuneHarness.cpp` — see that file's header for the full contract and the
 * fidelity argument for why the tuned gains transfer to production.
 *
 * CRITICAL: the replica pumps `cooperativeTick()` once per turn-loop iteration so its tick period
 * matches a real nav turn's, so `cooperativeTick()` below MUST tick the same modules, in the same
 * order, as `WRO2026.ino::cooperativeTick()`. If a module is added to or removed from
 * `WRO2026.ino::cooperativeTick()`, mirror the change here too (the same rule `inoTest/turn_probe`,
 * `inoTest/turn_tune`, and `inoTest/tick_bench` follow).
 *
 * BUILD MECHANIC — see `inoTest/tick_bench/tick_bench.ino`'s header for why each
 * `WRO2026/src/<domain>` folder needs its own `--library` flag; identical six domains here.
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/turn_autotune/build.py` (add `--build-only` to compile
 *       without touching a connected board). Log STRAIGHT to a file, not `| tee`, or the R4 USB CDC
 *       backpressures and freezes mid-run. BTN_UP starts the search; BTN_DOWN aborts it.
 * @note Serial diagnostics run at 115200 baud, matching `WRO2026.ino`; `setup()` waits up to 3 s
 *       for a USB CDC listener but does not hang a headless boot.
 * @note NOT in CI — the `inoTest/` diagnostic sketches never are. `arduino-cli` is the only local
 *       check that this links.
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
#include "TurnAutotuneHarness.h"
#include "Vision.h"
#include "WROLineArray.h"
#include "YawSource.h"

/**
 * @brief Cooperative tick — every module's `update()` exactly once, in the SAME order as
 *        `WRO2026.ino::cooperativeTick()`.
 *
 * Deliberate, harmless simplification vs. the production sketch: `WRO2026.ino` guards its final
 * `EventTrace.update()` behind the calibration-menu state; this sketch has no menu, so the guard
 * is always true here and is omitted.
 *
 * Order: YawSource -> LineArray -> Drive -> LineFollower -> Intersection -> Manipulator -> Tasks ->
 *     PathTracker -> Localization -> Mission -> EventTrace.
 */
void cooperativeTick()
{
    // NOTE: mirror any change to WRO2026.ino::cooperativeTick() here too (and in the other inoTest
    // sketches) or the replica's measured tick silently drifts from what a real nav turn pays.

    // YawSource.update() runs FIRST, as in production: one gated gyro read refreshes the
    // tick-scoped yaw cache, so the replica's per-tick readYawDeg serves that single device
    // transaction instead of triggering its own inline re-read (issue #163, AC-5).
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
    // CONTINUE (do not halt) — a degraded module still lets the rest of the search run.
    // YawSource.begin() is NOT optional here: it is the only place the gyro's I2C transport
    // (Wire1 + mux channel) is wired, and every turn in this search reads yaw through it.
    if (!LineArray.begin())    { Serial.println(F("[turn_autotune] LineArray begin() FAILED")); }
    if (!YawSource.begin())    { Serial.println(F("[turn_autotune] YawSource begin() FAILED")); }
    if (!Drive.begin())        { Serial.println(F("[turn_autotune] Drive begin() FAILED")); }
    if (!LineFollower.begin()) { Serial.println(F("[turn_autotune] LineFollower begin() FAILED")); }
    if (!Intersection.begin()) { Serial.println(F("[turn_autotune] Intersection begin() FAILED")); }
    if (!Manipulator.begin())  { Serial.println(F("[turn_autotune] Manipulator begin() FAILED")); }
    if (!Localization.begin()) { Serial.println(F("[turn_autotune] Localization begin() FAILED")); }
    if (!PathTracker.begin())  { Serial.println(F("[turn_autotune] PathTracker begin() FAILED")); }
    if (!Mission.begin())      { Serial.println(F("[turn_autotune] Mission begin() FAILED")); }
    if (!Tasks.begin())        { Serial.println(F("[turn_autotune] Tasks begin() FAILED")); }
    if (!Vision.begin())       { Serial.println(F("[turn_autotune] Vision begin() FAILED")); }
    if (!EventTrace.begin())   { Serial.println(F("[turn_autotune] EventTrace begin() FAILED")); }

    // Full tick-callback wiring, matching WRO2026.ino::setup() exactly — every blocking wait
    // re-enters cooperativeTick per spin iteration.
    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);
    LineFollower.setTickCallback(cooperativeTick);
    Manipulator.setTickCallback(cooperativeTick);
    Tasks.setTickCallback(cooperativeTick);
    Vision.setTickCallback(cooperativeTick);

    // The search gates its first turn behind a BTN_UP prompt (runTurnAutotune ->
    // autotuneWaitForBtnUp), so calling it once here matches the "arm on BTN_UP" UX. This call
    // blocks for the whole search, pumping cooperativeTick() throughout via the tickCb parameter.
    runTurnAutotune(cooperativeTick);
}

void loop()
{
    // One-shot per power cycle (the search prints "press RESET to run again"). Keep every module
    // ticking so Serial/OLED stay alive for the operator to read the result.
    cooperativeTick();
}
