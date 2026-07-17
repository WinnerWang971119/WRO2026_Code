/**
 * @file turn_probe.ino
 * @brief Standalone bench probe for the turn PD executor's creep floor, tick period, and IMU
 *        noise (docs/plans/2026-07-16-turn-pd-controller.md, task T10 AS AMENDED).
 * @author WRO 2026
 *
 * @details
 * EXTRACTED from `WRO2026/Tasks.cpp::testOne`. T10 originally called for reverting `testOne`
 * to an empty stub and deleting the harness; the operator amended that mid-implementation to
 * extract the probe here instead, since it is a reusable bench tool worth keeping buildable
 * independently of the mission sketch. `testOne` is now the trivial `return true;` stub — see
 * `WRO2026/Tasks.cpp`.
 *
 * The five-phase probe itself (IMU noise, idle tick, breakaway sweep, kinetic sweep, omega +
 * turn tick) lives in `TurnProbeHarness.cpp`, not here — see that file's header for the full
 * phase-by-phase contract and for why it is not inlined into this `.ino`. This file only
 * wires up the real firmware modules exactly as `WRO2026.ino::setup()` does and calls
 * `runTurnProbe(cooperativeTick)` once from `setup()`.
 *
 * CRITICAL: phase (b) of the probe measures the TURN-representative tick by counting
 * `cooperativeTick()` iterations, so `cooperativeTick()` below MUST tick the same modules, in
 * the same order, as `WRO2026.ino::cooperativeTick()` — otherwise the measured tick (and the
 * `TURN_MIN_RESOLVABLE_DEG` it feeds) is not representative of what a real nav turn pays. If a
 * module is ever added to or removed from `WRO2026.ino::cooperativeTick()`, mirror the change
 * here too (the same rule `inoTest/tick_bench` already follows for issue #123).
 *
 * BUILD MECHANIC — see `inoTest/tick_bench/tick_bench.ino`'s header for the full explanation
 * of why each `WRO2026/src/<domain>` folder needs its own `--library` flag; this sketch uses
 * the identical six domains for the identical reason (`cooperativeTick()` below ticks the same
 * module set as `tick_bench`'s `benchTick()`, and `Tasks.cpp` — compiled in via `--library
 * WRO2026` — pulls in the same six domains via its own includes).
 *
 * @note BUILD/FLASH/MONITOR: `uv run inoTest/turn_probe/build.py` (add `--build-only` to
 *       compile without touching a connected board).
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
#include "TurnProbeHarness.h"
#include "Vision.h"
#include "WROLineArray.h"

/**
 * @brief Cooperative tick — every module's `update()` exactly once, in the SAME order as
 *        `WRO2026.ino::cooperativeTick()`.
 *
 * Deliberate, harmless simplification vs. the production sketch: `WRO2026.ino` guards its
 * final `EventTrace.update()` behind `_calMenuState != CalMenuState::OPEN` so the on-OLED
 * calibration menu is not clobbered by the event-ring repaint. This sketch has no calibration
 * menu — that state is always the equivalent of IDLE — so the guard is always true here and is
 * omitted; `EventTrace.update()` runs unconditionally, doing exactly the same per-tick work.
 *
 * Order: LineArray -> Drive -> LineFollower -> Intersection -> Manipulator -> Tasks ->
 *     PathTracker -> Localization -> Mission -> EventTrace.
 */
void cooperativeTick()
{
    // NOTE: mirror any change to WRO2026.ino::cooperativeTick() here too (and in
    // inoTest/tick_bench/tick_bench.ino) or this probe's measured tick silently drifts from
    // what a real nav turn pays.
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

    // All 11 begin()s, in the same order as WRO2026.ino::setup(). On a false return, print and
    // CONTINUE (do not halt) — mirrors inoTest/tick_bench's posture: a degraded module still
    // lets the rest of the probe run, and the failure is visible on Serial either way.
    if (!LineArray.begin())    { Serial.println(F("[turn_probe] LineArray begin() FAILED")); }
    if (!Drive.begin())        { Serial.println(F("[turn_probe] Drive begin() FAILED")); }
    if (!LineFollower.begin()) { Serial.println(F("[turn_probe] LineFollower begin() FAILED")); }
    if (!Intersection.begin()) { Serial.println(F("[turn_probe] Intersection begin() FAILED")); }
    if (!Manipulator.begin())  { Serial.println(F("[turn_probe] Manipulator begin() FAILED")); }
    if (!Localization.begin()) { Serial.println(F("[turn_probe] Localization begin() FAILED")); }
    if (!PathTracker.begin())  { Serial.println(F("[turn_probe] PathTracker begin() FAILED")); }
    if (!Mission.begin())      { Serial.println(F("[turn_probe] Mission begin() FAILED")); }
    if (!Tasks.begin())        { Serial.println(F("[turn_probe] Tasks begin() FAILED")); }
    if (!Vision.begin())       { Serial.println(F("[turn_probe] Vision begin() FAILED")); }
    if (!EventTrace.begin())   { Serial.println(F("[turn_probe] EventTrace begin() FAILED")); }

    // Full tick-callback wiring, matching WRO2026.ino::setup() exactly — every sync-blocking
    // drain the probe's own blocking waits might otherwise starve stays serviced.
    Drive.setTickCallback(cooperativeTick);
    PathTracker.setTickCallback(cooperativeTick);
    LineFollower.setTickCallback(cooperativeTick);
    Manipulator.setTickCallback(cooperativeTick);
    Tasks.setTickCallback(cooperativeTick);
    Vision.setTickCallback(cooperativeTick);

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 turn probe (inoTest/turn_probe)                  #"));
    Serial.println(F("############################################################"));

    // The probe itself gates its first phase behind a BTN_UP prompt (see
    // TurnProbeHarness.cpp::runTurnProbe -> turnProbeWaitForBtnUp), so calling it once here
    // from setup() already matches the "run once on a BTN_UP arm" operator UX: nothing drives
    // until the operator has positioned the chassis and pressed BTN_UP. This call blocks for
    // the probe's whole multi-minute run, pumping cooperativeTick() throughout via the
    // tickCb parameter — the same pattern every other blocking wait in this codebase uses.
    runTurnProbe(cooperativeTick);
}

void loop()
{
    // The probe is one-shot per power cycle (see runTurnProbe()'s closing "press RESET to run
    // again" line) — once setup() returns, just keep every module ticking so Serial/OLED stay
    // alive for the operator to read the summary at their leisure.
    cooperativeTick();
}
