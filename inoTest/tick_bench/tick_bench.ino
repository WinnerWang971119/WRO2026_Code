/**
 * @file tick_bench.ino
 * @brief On-device `cooperativeTick()` loop-time benchmark (issue #123).
 * @author WRO 2026
 *
 * @details
 * Standalone diagnostic sketch that wires up the REAL `WRO2026/` module sources (not host mocks,
 * not a mirror) exactly as `WRO2026.ino::setup()` does, then loops a `benchTick()` that calls each
 * ticked module's `update()` once, in `cooperativeTick()` order, timing each call with a
 * `TickProfiler<SEGMENT_COUNT>`. Every `WINDOW_TICKS` passes it prints a Serial summary — whole-tick
 * `min/avg/max` µs, effective Hz, a `GOOD`/`MARGINAL`/`TOO_LONG` verdict, and a per-module breakdown
 * — then resets the window. No mission is authored and no blocking Drive move is ever issued: the
 * bench stays idle so `loop()` never blocks, and `benchTick()` alone drives every module's `update()`.
 *
 * BUILD MECHANIC — `WRO2026/` and each `WRO2026/src/<domain>/` as legacy (1.0) Arduino libraries
 * -------------------------------------------------------------------------------------------------
 * Since the sketch-src reorg (issue #66) `WRO2026/` DOES have a `src/` subdir, but it still has no
 * `library.properties`, so arduino-cli does not treat it as a recursive 1.5-format library — a plain
 * `--library WRO2026` still resolves as a legacy 1.0 library rooted at exactly `WRO2026/`, adding ONLY
 * that folder (non-recursively) to the include path. That reaches the operator-editable root files
 * (`Config.h`/`config/ConfigGlobal.h`, `Tasks.h`, `TasksTypes.h`) and compiles `Tasks.cpp` /
 * `MissionScript.cpp`, but NOT the framework modules now nested under `src/<domain>/` (Drive.cpp,
 * WROLineArray.cpp, ...). Each `src/<domain>/` folder is itself a flat legacy-1.0 library root (same
 * rule: no `library.properties`, so passing it via `--library` adds exactly that folder, non-
 * recursively), so this bench needs one additional `--library WRO2026/src/<domain>` flag per domain it
 * touches. This bench uses all six: `drive` (Drive, LineFollower, Intersection), `perception`
 * (WROLineArray, Vision), `nav` (Localization), `mission` (Mission, PathTracker), `manip`
 * (Manipulator), `util` (EventTrace, TickProfiler). Every `#include` below stays a bare filename —
 * each domain folder is added directly to the include path, so no path-qualified include is needed
 * (contrast `tools/vision_bench/vision_bench.ino`, which needs a root-relative subpath instead because
 * it reaches both an unmoved `config/` header and a moved `src/perception/` header from outside
 * `WRO2026/` via a raw `-I` flag, not `--library`). The `test/` host-mock dirs are NOT on the include
 * path (no header shadowing). The `.ino` files in `WRO2026/` are ignored by the library compiler, so
 * there is no `setup()`/`loop()` collision with `WRO2026.ino`.
 *
 * @note BUILD: run from the repo root `C:/users/wangd/documents/wro2026/wro2026_code` —
 *       arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi inoTest/tick_bench \
 *         --library WRO2026 --library WRO2026/src/drive --library WRO2026/src/perception \
 *         --library WRO2026/src/nav --library WRO2026/src/mission --library WRO2026/src/manip \
 *         --library WRO2026/src/util
 *       (verified clean with arduino-cli 1.5.1, arduino:renesas_uno core 1.6.0, MatrixMiniR4 1.2.2.)
 *
 * @note Serial diagnostics run at 115200 baud (matches `WRO2026.ino`); `setup()` waits up to 3 s for
 *       a USB CDC listener but does not hang a headless boot.
 *
 * @note IDLE-VS-ACTIVE CAVEAT: this bench never runs a mission, so `Localization.update()`'s
 *       operator-offset gate is open every tick (it is only CLOSED while nav is active) and it
 *       issues an IMU read every pass. In a real mission that gate is closed for most of the run, so
 *       this idle bench slightly OVER-weights the `Localization` segment and under-weights the extra
 *       active-move work `Drive`/`LineFollower` do mid-mission. This does not move the headline
 *       finding — `LineArray`'s per-tick I2C read dominates regardless of mission state.
 *
 * @note KEEP IN SYNC WITH cooperativeTick(): the 10 segments below (`LineArray, Drive, LineFollower,
 *       Intersection, Manipulator, Tasks, PathTracker, Localization, Mission, EventTrace`) and their
 *       order mirror `WRO2026.ino::cooperativeTick()` by hand. If a module is added to or removed
 *       from that function, mirror the change here in THREE places — the `addSegment()` registration
 *       list, the `benchTick()` call list, AND the `SEGMENT_COUNT` constant — or this benchmark
 *       silently drifts from what actually ships. `SEGMENT_COUNT` is the one edit with no compile
 *       error to catch a miss: if it is left too small, the extra `addSegment()` returns
 *       `TICK_INVALID_SEGMENT` and that module vanishes from every report with no warning — so
 *       `setup()` below hard-checks every returned index against the sentinel and prints a FATAL
 *       line if any comes back invalid. `Vision` is NOT a segment — it is capture-once via a task,
 *       not ticked in `cooperativeTick()` — but its `begin()` still runs below so the wiring order
 *       matches `setup()` exactly.
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
#include "TickProfiler.h"
#include "Vision.h"
#include "WROLineArray.h"

namespace
{
    /// @brief Ticks accumulated per report window before printing + resetting (Decisions Q3).
    constexpr uint32_t WINDOW_TICKS = 200;

    /// @brief Below this effective Hz the verdict is TOO_LONG (Decisions Q2).
    constexpr uint32_t TICK_FLOOR_HZ = 30;

    /// @brief At or above this effective Hz the verdict is GOOD (Decisions Q2).
    constexpr uint32_t TICK_GOOD_HZ = 45;

    /// @brief Number of ticked `cooperativeTick()` modules registered as profiler segments.
    constexpr uint8_t SEGMENT_COUNT = 10;

    /// @brief Segment indices, assigned in `setup()` in `cooperativeTick()` order.
    uint8_t segLineArray, segDrive, segLineFollower, segIntersection, segManipulator;
    uint8_t segTasks, segPathTracker, segLocalization, segMission, segEventTrace;
}    // namespace

/// @brief The fixed-size timing accumulator driving this bench. Registered once in `setup()`.
static TickProfiler<SEGMENT_COUNT> prof;

/**
 * @brief Print the `?` help block + the current window's live state.
 * @note Does NOT reset the window — it is a read-only status query (AC-serialcmd).
 */
void printHelp()
{
    Serial.println(F("\n-- tick_bench commands --"));
    Serial.println(F("  ?  help + current window state (does not reset)"));
    Serial.println(F("  r  reset the report window now"));
    Serial.print(F("[state] ticks so far="));
    Serial.print(prof.tickCount());
    Serial.print(F(" / "));
    Serial.print(WINDOW_TICKS);
    Serial.print(F("  floorHz="));
    Serial.print(TICK_FLOOR_HZ);
    Serial.print(F("  goodHz="));
    Serial.println(TICK_GOOD_HZ);
}

/// @brief Poll Serial for the single-char command set (gyro-move.ino style). Ignores whitespace.
void pollSerial()
{
    if (!Serial.available())
    {
        return;
    }
    const int c = Serial.read();
    switch (c)
    {
    case '?':
        printHelp();
        break;
    case 'r':
        prof.reset();
        Serial.println(F("[bench] window reset."));
        break;
    case '\n':
    case '\r':
    case ' ':
        break;    // ignore whitespace
    default:
        Serial.print(F("[?] unknown cmd '"));
        Serial.print((char) c);
        Serial.println(F("' -- press ? for help"));
        break;
    }
}

/**
 * @brief One instrumented cooperative tick: call each of the 10 ticked modules' `update()` once, in
 *        `cooperativeTick()` order, timing each call into its profiler segment, then the whole pass.
 * @note `Vision.update()` is deliberately NOT called here — `Vision` is capture-once via a task, not
 *       ticked in the real `cooperativeTick()`.
 */
void benchTick()
{
    const uint32_t t0 = micros();

    uint32_t s = micros();
    LineArray.update();
    prof.sampleSegment(segLineArray, micros() - s);

    s = micros();
    Drive.update();
    prof.sampleSegment(segDrive, micros() - s);

    s = micros();
    LineFollower.update();
    prof.sampleSegment(segLineFollower, micros() - s);

    s = micros();
    Intersection.update();
    prof.sampleSegment(segIntersection, micros() - s);

    s = micros();
    Manipulator.update();
    prof.sampleSegment(segManipulator, micros() - s);

    s = micros();
    Tasks.update();
    prof.sampleSegment(segTasks, micros() - s);

    s = micros();
    PathTracker.update();
    prof.sampleSegment(segPathTracker, micros() - s);

    s = micros();
    Localization.update();
    prof.sampleSegment(segLocalization, micros() - s);

    s = micros();
    Mission.update();
    prof.sampleSegment(segMission, micros() - s);

    s = micros();
    EventTrace.update();
    prof.sampleSegment(segEventTrace, micros() - s);

    prof.sampleTick(micros() - t0);
}

void setup()
{
    MiniR4.begin();

    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }    // give USB CDC a moment, but don't hang headless

    MiniR4.PWR.setBattCell(Config::BATT_CELLS);

    // All 11 begin()s, in the same order as WRO2026.ino::setup() — INCLUDING Vision.begin(), even
    // though Vision is not a ticked segment below. On a false return, print and CONTINUE (do not
    // halt) — a degraded module just reports near-zero/degraded segment time, which is still useful.
    if (!LineArray.begin())     { Serial.println(F("[bench] LineArray begin() FAILED")); }
    if (!Drive.begin())         { Serial.println(F("[bench] Drive begin() FAILED")); }
    if (!LineFollower.begin())  { Serial.println(F("[bench] LineFollower begin() FAILED")); }
    if (!Intersection.begin())  { Serial.println(F("[bench] Intersection begin() FAILED")); }
    if (!Manipulator.begin())   { Serial.println(F("[bench] Manipulator begin() FAILED")); }
    if (!Localization.begin()) { Serial.println(F("[bench] Localization begin() FAILED")); }
    if (!PathTracker.begin())  { Serial.println(F("[bench] PathTracker begin() FAILED")); }
    if (!Mission.begin())      { Serial.println(F("[bench] Mission begin() FAILED")); }
    if (!Tasks.begin())        { Serial.println(F("[bench] Tasks begin() FAILED")); }
    if (!Vision.begin())       { Serial.println(F("[bench] Vision begin() FAILED")); }
    if (!EventTrace.begin())   { Serial.println(F("[bench] EventTrace begin() FAILED")); }

    // Deliberate divergence from WRO2026.ino::setup(): register NO tick callbacks. The bench issues
    // no blocking Drive move (no mission, no goTo), so no sync-blocking drain ever fires, and wiring
    // benchTick() as a tick callback would risk re-entering the profiler mid-segment.

    // Register the 10 ticked segments, in cooperativeTick() order.
    segLineArray    = prof.addSegment("LineArray");
    segDrive        = prof.addSegment("Drive");
    segLineFollower = prof.addSegment("LineFollower");
    segIntersection = prof.addSegment("Intersection");
    segManipulator  = prof.addSegment("Manipulator");
    segTasks        = prof.addSegment("Tasks");
    segPathTracker  = prof.addSegment("PathTracker");
    segLocalization = prof.addSegment("Localization");
    segMission      = prof.addSegment("Mission");
    segEventTrace   = prof.addSegment("EventTrace");

    // Fail LOUD on SEGMENT_COUNT drift: if a module was added above but SEGMENT_COUNT was left too
    // small, its addSegment() overflowed and returned TICK_INVALID_SEGMENT, silently dropping that
    // module from every report window. Catch it at boot instead (see the KEEP-IN-SYNC note above).
    const uint8_t regd[] = { segLineArray, segDrive, segLineFollower, segIntersection, segManipulator,
                             segTasks, segPathTracker, segLocalization, segMission, segEventTrace };
    for (uint8_t i = 0; i < sizeof(regd) / sizeof(regd[0]); ++i)
    {
        if (regd[i] == TICK_INVALID_SEGMENT)
        {
            Serial.println(
                F("[bench] FATAL: SEGMENT_COUNT too small -- a module was dropped from the benchmark!"));
            Serial.println(F("[bench]        bump SEGMENT_COUNT to match the addSegment() list above."));
        }
    }

    Serial.println(F("\n############################################################"));
    Serial.println(F("#  WRO2026 cooperativeTick() bench (inoTest/tick_bench)     #"));
    Serial.println(F("############################################################"));
    printHelp();
}

void loop()
{
    benchTick();

    if (prof.tickCount() >= WINDOW_TICKS)
    {
        prof.report(Serial, TICK_FLOOR_HZ, TICK_GOOD_HZ);
        prof.reset();
    }

    pollSerial();
}
