/**
 * @file ctps_dump.cpp
 * @brief CTPS command-stream dump driver (Mechanism #4, task T4).
 * @author WRO 2026
 *
 * @details
 * CTPS (Clear Trajectory Prediction Simulator) links the REAL `Mission.cpp` +
 * `Tasks.cpp` + `MissionScript.cpp` + `routes.cpp` on a host x86 toolchain
 * against the recording mocks under `ctps_mocks/`. This driver runs a mission
 * to completion against those mocks and serializes the captured, ordered
 * command stream (`ctps::g_ctpsLog`) to the Data-Model-A JSON via
 * `ctps::commandStreamToJson(...)`.
 *
 * Three modes (selected by argv[1]):
 *   - `--self-test` : runs `buildMissionFixture()` — a synthetic mission that
 *                     exercises the ENTIRE op vocabulary on valid production
 *                     nodes — then asserts the expected ops appear in order in
 *                     `g_ctpsLog` and that the run terminated within
 *                     `CTPS_MAX_TICKS`. Prints `CTPS DUMP SELFTEST OK` and
 *                     returns 0 on success; a diagnostic + return 1 on failure.
 *                     This is the CI-enforced ctest (TC8 / TC9 / AC4 / AC6).
 *   - `--fixture`   : runs `buildMissionFixture()` and prints its JSON.
 *   - (default)     : runs the REAL `buildMission()` and prints its JSON,
 *                     resolving its gotos against the linked production
 *                     `routes.cpp` (the CTPS player then renders the trace).
 *
 * @note HOST-ONLY translation unit — compiled into `ctps_dump` only, never the
 *       Arduino sketch. It freely uses `<iostream>` / `<vector>` / `<string>`.
 */

#include "ctps_record.h"

// Recording mocks + real production headers. The `ctps_mocks/`-first include
// order (and the T4 force-include wiring) resolves each of these to the mock
// where one exists, and to the real sketch header otherwise.
#include "Manipulator.h"     // ctps_mocks — records manip ops
#include "Drive.h"           // ctps_mocks — records raw drive ops
#include "src/mission/Mission.h"         // real — operator DSL (recording hooks under CTPS_RECORD)
#include "MissionScript.h"   // real — buildMission()
#include "Tasks.h"           // real — Tasks.begin()
#include "src/nav/FieldMap.h"        // real — node-name resolution
#include "src/nav/routes.h"          // real — Routes::NODES / START_NODE / START_HEADING_DEG

#include <MatrixMiniR4.h>    // ctps_mocks — MiniR4 facade (angle-include)
#include <Arduino.h>         // ctps_mocks — mock clock millis() / mockAdvanceMillis()

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

namespace
{
    /// @brief Hard ceiling on drain-loop iterations — guards against a mock that
    ///        never reports idle so the dump can never hang (TC9).
    constexpr unsigned CTPS_MAX_TICKS = 100000;

    /// @brief Per-tick mock-clock advance. Large enough that every Mission
    ///        WAIT / BEEP / LED duration gate (`now - start >= duration`) trips
    ///        on the next tick, so the dump completes instantly.
    constexpr uint32_t CTPS_TICK_ADVANCE_MS = 100000u;

    /**
     * @brief Bring every CTPS-linked facade up and clear the recording sink.
     *
     * Mirrors the sketch's `setup()` ordering for the singletons CTPS links,
     * installs no-op tick callbacks where the mocks expose `setTickCallback`,
     * and finally resets the capture buffer + mock clock so the run starts from
     * `seq == 0` / `millis() == 0`.
     */
    void ctpsSetup()
    {
        MiniR4.begin();
        Drive.begin();
        LineFollower.begin();
        Manipulator.begin();
        PathTracker.begin();
        Tasks.begin();
        Mission.begin();
        EventTrace.begin();

        // No-op tick callbacks: the mocks complete instantly, so the
        // cooperative re-entry the real firmware needs is unnecessary here.
        Drive.setTickCallback(nullptr);
        PathTracker.setTickCallback(nullptr);
        LineFollower.setTickCallback(nullptr);

        ctps::reset();
        mockResetMillis();
    }

    /**
     * @brief Build the synthetic self-test fixture mission.
     *
     * Exercises the FULL op vocabulary against the toy `routes.*` graph using
     * two valid node names (`"Start"` and `"final"`) so the real `Mission.cpp`
     * GOTO hook fires and the recording `PathTracker` mock captures a `goto`.
     * The Mission verbs cover `goto` / `wait` / `beep` / `led` / `print` /
     * `task` (the real `demoEcho` registry task → `task_start` + `task_end`).
     *
     * @note `manip` + `drive` ops (AC4) are NOT produced by any registered task
     *       body, so they are appended AFTER the mission drains via a synthetic
     *       bracketed `recordTaskStart` / `recordTaskEnd` segment in `main`.
     */
    void buildMissionFixture()
    {
        // Two valid PRODUCTION nodes (see routes.cpp NODES[]): "Start" and "MYU".
        // Both resolve via FieldMap::findNodeByName so the GOTO hook fires and the
        // PathTracker mock records a `goto`.
        Mission.goTo("Start")
            .wait(500)
            .beep(1000, 150)
            .led(0, 255, 0, 300)
            .print("AT")
            .task("demoEcho", uint8_t(7), uint8_t(42))
            .goTo("MYU")
            .print("DONE");
        Mission.start();
    }

    /**
     * @brief Drain the armed mission to completion (or fault) under the mock clock.
     *
     * Advances the mock clock each tick so every duration gate trips on the next
     * tick. Returns `false` (and prints a diagnostic to stderr) if the loop hit
     * `CTPS_MAX_TICKS` without the mission going idle or faulting — the TC9
     * hang-guard.
     *
     * @return `true` if the mission reached IDLE or FAULT within the tick budget.
     */
    bool drainMission()
    {
        unsigned ticks = 0;
        while (!Mission.isIdle() && !Mission.hasFault() && ticks < CTPS_MAX_TICKS)
        {
            Mission.update();
            mockAdvanceMillis(CTPS_TICK_ADVANCE_MS);
            ++ticks;
        }

        if (ticks >= CTPS_MAX_TICKS)
        {
            std::cerr << "ctps_dump: drain exceeded CTPS_MAX_TICKS (" << CTPS_MAX_TICKS
                      << ") without idle/fault\n";
            return false;
        }
        return true;
    }

    /**
     * @brief Append the synthetic AC4 manip + drive segment, bracketed by a task.
     *
     * No registered task body issues `manip` / `drive` ops, so the self-test
     * synthesises a bracketed segment directly against the recording sink +
     * mocks: a `task_start` (args begin `072a`), `setGripper` / `setLift` manip
     * ops, and a family of `drive` ops — a plain `driveDist` (records neither
     * `mode` nor `heading`), a `driveDist` under GYRO with an explicit `heading`,
     * both `lineFollowDist` producers (`followDistance` and
     * `setEnabled(true).setDistanceTarget()`) plus the two follows that record
     * NOTHING (`setEnabled(false)` and a target-less `setEnabled(true)`), a
     * reversed `lineFollowDist`, a `turnDeg`, and a reversed `wallStop` — then
     * `task_end`.
     *
     * The Drive / LineFollower builders record once from their `_executed`-guarded
     * DESTRUCTOR (at the statement's `;`, so the one-verb-per-statement authored
     * order is preserved), which is what lets the chained `.setMode(GYRO)` /
     * `.setHeading(...)` / `.setReverse(true)` setters run before the single emit.
     */
    void recordManipDriveSegment()
    {
        uint8_t fxArgs[Config::MAX_TASK_ARG_BYTES] = {0x07, 0x2a, 0};
        ctps::recordTaskStart("fixtureManip", fxArgs, sizeof(fxArgs));
        Manipulator.setGripper(180);
        Drive.driveDist(15);    // plain driveDist — records neither mode nor heading (NC3).
        Drive.turnDeg(-90);
        Manipulator.setLift(120);

        // GYRO / lineFollowDist / reverse capture (NC1-NC3; AC2 / AC5 / AC6). Each is a
        // one-verb-per-statement discarded temporary: the builder destructs at the `;`,
        // emitting one record with the fully-resolved mode / heading / reverse. Do NOT
        // author two Drive verbs in one full-expression (temporaries destruct in reverse
        // construction order, which would reorder the emitted records).
        Drive.driveDist(35).setMode(DriveMode::GYRO).setHeading(0);    // gyro + heading (AC2)
        LineFollower.followDistance(90);                       // lineFollowDist #1 (forward)
        LineFollower.setEnabled(true).setDistanceTarget(60);   // lineFollowDist #2 (forward)
        LineFollower.setEnabled(false);                        // silence: records nothing
        LineFollower.setEnabled(true);            // silence: target-less, records nothing
        LineFollower.setEnabled(true).setDistanceTarget(45).setReverse(true);    // reversed (AC6)
        Drive.wallStop(10).setReverse(true);                  // reversed wallStop, cm=10 (NC3)

        ctps::recordTaskEnd("fixtureManip", true);
    }

    /**
     * @brief Serialize the captured log to the Data-Model-A JSON on stdout.
     *
     * Uses the start-node name + heading from the auto-generated `routes.*`.
     */
    void emitJson()
    {
        std::cout << ctps::commandStreamToJson(Routes::NODES[Routes::START_NODE].name,
                                               Routes::START_HEADING_DEG);
        std::cout.flush();
    }

    // -------------------------------------------------------------------------
    // Self-test assertion helpers.
    // -------------------------------------------------------------------------

    /**
     * @brief Index of the first record (at or after @p from) matching @p pred.
     * @return The matching index, or `g_ctpsLog.size()` if none matched.
     */
    template <typename Pred>
    size_t findFrom(size_t from, Pred pred)
    {
        for (size_t i = from; i < ctps::g_ctpsLog.size(); ++i)
        {
            if (pred(ctps::g_ctpsLog[i]))
            {
                return i;
            }
        }
        return ctps::g_ctpsLog.size();
    }

    /**
     * @brief Lowercase-hex prefix test on a record's raw `args` bytes.
     * @return `true` if @p rec is a TASK_START whose hex args begin with @p prefix.
     */
    bool argsHexBeginsWith(const ctps::Record& rec, const char* prefix)
    {
        static const char* kHex = "0123456789abcdef";
        std::string hex;
        for (uint8_t byte : rec.args)
        {
            hex += kHex[(byte >> 4) & 0x0F];
            hex += kHex[byte & 0x0F];
        }
        return hex.rfind(prefix, 0) == 0;
    }

    /**
     * @brief Run the full self-test: assert the expected op stream is present and
     *        ordered, and that the mission drained within the tick budget.
     *
     * Also asserts the T2/T3 drive-detail capture appended by
     * `recordManipDriveSegment`: the GYRO `mode` + held `heading`, exactly three
     * `lineFollowDist` records (both producers plus the reversed one; the two
     * silence cases record nothing), a `reverse` flag, and the `wallStop` `cm` —
     * verified both at the in-memory `Record` level (NC1, NC2) and in the
     * serialized command-stream JSON (NC3, AC12).
     *
     * @return 0 on success (after printing `CTPS DUMP SELFTEST OK`); 1 on the
     *         first failed assertion (after printing a diagnostic to stderr).
     */
    int runSelfTest()
    {
        ctpsSetup();
        buildMissionFixture();

        // TC9 — the mission must drain (idle/fault) within the tick budget.
        if (!drainMission())
        {
            return 1;
        }

        // AC4 — manip + drive ops live in a synthetic bracketed task segment
        // appended after the Mission drains.
        recordManipDriveSegment();

        // Direction-override capture: a goto carrying setArrivalDirection /
        // setRouteDirection must serialize the "arrival"/"route" tags. A
        // one-statement discarded temporary, so the recording mock builder
        // destructs at the `;` after BOTH setters run. ("Start" is a live
        // production node; this asserts the recorded direction tags, not the hop.)
        PathTracker.goTo("Start")
            .setArrivalDirection(Direction::REAR)
            .setRouteDirection(Direction::FRONT);

        const std::vector<ctps::Record>& log = ctps::g_ctpsLog;
        size_t cursor = 0;

        // The Mission verbs execute in authored order; assert each op appears at
        // or after the previous one's position so order is verified, not just
        // presence.
        struct OpStep
        {
            ctps::Op    op;
            const char* label;
        };
        const OpStep kOrderedOps[] = {
            {ctps::Op::GOTO,       "goto"},
            {ctps::Op::WAIT,       "wait"},
            {ctps::Op::BEEP,       "beep"},
            {ctps::Op::LED,        "led"},
            {ctps::Op::PRINT,      "print"},
            {ctps::Op::TASK_START, "task_start"},
            {ctps::Op::TASK_END,   "task_end"},
        };

        for (const OpStep& step : kOrderedOps)
        {
            const ctps::Op want = step.op;
            const size_t found = findFrom(cursor, [want](const ctps::Record& r)
                                          { return r.op == want; });
            if (found >= log.size())
            {
                std::cerr << "ctps_dump self-test: missing op '" << step.label
                          << "' at/after index " << cursor << "\n";
                return 1;
            }
            cursor = found + 1;
        }

        // The ordered walk above only proves the FIRST goTo fired. Assert the
        // second goTo("MYU") recorded too, so a bad second node faults the
        // self-test instead of slipping past coverage.
        const size_t gotoSecond =
            findFrom(0, [](const ctps::Record& r)
                     { return r.op == ctps::Op::GOTO && r.target == "MYU"; });
        if (gotoSecond >= log.size())
        {
            std::cerr << "ctps_dump self-test: missing goto with target 'MYU'\n";
            return 1;
        }

        // TC8 — at least one TASK_START whose args_hex begins "072a". Both the
        // real `demoEcho` step (uint8_t 7, 42) and the synthetic fixtureManip
        // segment satisfy this; either match is acceptable.
        const size_t taskStart072a =
            findFrom(0, [](const ctps::Record& r)
                     { return r.op == ctps::Op::TASK_START && argsHexBeginsWith(r, "072a"); });
        if (taskStart072a >= log.size())
        {
            std::cerr << "ctps_dump self-test: no task_start with args_hex beginning '072a'\n";
            return 1;
        }

        // AC4 — manip ops: a `setGripper` and a `setLift`.
        const size_t setGripper =
            findFrom(0, [](const ctps::Record& r)
                     { return r.op == ctps::Op::MANIP && r.method == "setGripper"; });
        const size_t setLift =
            findFrom(0, [](const ctps::Record& r)
                     { return r.op == ctps::Op::MANIP && r.method == "setLift"; });
        if (setGripper >= log.size() || setLift >= log.size())
        {
            std::cerr << "ctps_dump self-test: missing manip op (setGripper/setLift)\n";
            return 1;
        }

        // AC4 — drive ops: a `driveDist` and a `turnDeg`.
        const size_t driveDist =
            findFrom(0, [](const ctps::Record& r)
                     { return r.op == ctps::Op::DRIVE && r.method == "driveDist"; });
        const size_t turnDeg =
            findFrom(0, [](const ctps::Record& r)
                     { return r.op == ctps::Op::DRIVE && r.method == "turnDeg"; });
        if (driveDist >= log.size() || turnDeg >= log.size())
        {
            std::cerr << "ctps_dump self-test: missing drive op (driveDist/turnDeg)\n";
            return 1;
        }

        // NC1 — the GYRO+heading driveDist captures mode + held heading (AC2).
        const size_t gyroDrive =
            findFrom(0, [](const ctps::Record& r) {
                return r.op == ctps::Op::DRIVE && r.method == "driveDist" && r.mode == "gyro"
                       && r.hasHeading && r.heading == 0.0f;
            });
        if (gyroDrive >= log.size())
        {
            std::cerr << "ctps_dump self-test: missing GYRO driveDist "
                         "(method==driveDist, mode==gyro, hasHeading, heading==0)\n";
            return 1;
        }

        // NC3 (record level) — the plain driveDist(15) records NEITHER mode nor
        // heading. This half of NC3 is asserted here, not in the serialized-JSON
        // substring below: the gyro driveDist is also present and carries
        // mode/heading, so a whole-document substring can't isolate the plain one.
        const size_t plainDrive =
            findFrom(0, [](const ctps::Record& r) {
                return r.op == ctps::Op::DRIVE && r.method == "driveDist" && r.cm == 15.0f;
            });
        if (plainDrive >= log.size() || !log[plainDrive].mode.empty()
            || log[plainDrive].hasHeading)
        {
            std::cerr << "ctps_dump self-test: plain driveDist(15) must record "
                         "neither mode nor heading\n";
            return 1;
        }

        // NC2 — exactly three lineFollowDist records: followDistance(90) + the
        // setEnabled(true).setDistanceTarget(60) builder + the reversed target
        // builder. setEnabled(false) and the target-less setEnabled(true) record
        // nothing, so any count != 3 means a silence case leaked or a producer
        // dropped its emit. (buildMissionFixture's demoEcho produces no follows.)
        size_t lineFollowCount = 0;
        for (const ctps::Record& r : log)
        {
            if (r.op == ctps::Op::DRIVE && r.method == "lineFollowDist")
            {
                ++lineFollowCount;
            }
        }
        if (lineFollowCount != 3)
        {
            std::cerr << "ctps_dump self-test: expected exactly 3 lineFollowDist records, got "
                      << lineFollowCount << "\n";
            return 1;
        }

        // NC2 / AC6 — at least one reversed drive move (the reversed lineFollowDist
        // or the reversed wallStop).
        const size_t reversed =
            findFrom(0, [](const ctps::Record& r)
                     { return r.op == ctps::Op::DRIVE && r.reverse; });
        if (reversed >= log.size())
        {
            std::cerr << "ctps_dump self-test: missing a reverse==true drive record\n";
            return 1;
        }

        // NC3 — a wallStop record carrying its cm (the wallStop cm-serialization fix).
        const size_t wallStop =
            findFrom(0, [](const ctps::Record& r) {
                return r.op == ctps::Op::DRIVE && r.method == "wallStop" && r.cm == 10.0f;
            });
        if (wallStop >= log.size())
        {
            std::cerr << "ctps_dump self-test: missing wallStop record with cm==10\n";
            return 1;
        }

        // NC3 / AC12 — the SERIALIZED command-stream JSON carries the new
        // drive-detail keys. Substring assertions over the whole document: the
        // gyro driveDist's `mode`/`heading`, a reversed move's `reverse`, and the
        // `cm` emitted for both `wallStop` and `lineFollowDist`.
        const std::string json =
            ctps::commandStreamToJson(Routes::NODES[Routes::START_NODE].name,
                                      Routes::START_HEADING_DEG);
        struct JsonProbe
        {
            const char* needle;
            const char* label;
        };
        const JsonProbe kJsonProbes[] = {
            {"\"mode\": \"gyro\"", "mode: gyro"},
            {"\"heading\"", "heading key"},
            {"\"reverse\": true", "reverse: true"},
            {"\"method\": \"wallStop\", \"cm\": 10", "wallStop cm"},
            {"\"method\": \"lineFollowDist\", \"cm\":", "lineFollowDist cm"},
            {"\"arrival\": \"rear\"", "goto arrival: rear"},
            {"\"route\": \"front\"", "goto route: front"},
        };
        for (const JsonProbe& probe : kJsonProbes)
        {
            if (json.find(probe.needle) == std::string::npos)
            {
                std::cerr << "ctps_dump self-test: serialized JSON missing " << probe.label
                          << " (needle: " << probe.needle << ")\n";
                return 1;
            }
        }

        std::cout << "CTPS DUMP SELFTEST OK\n";
        std::cout.flush();
        return 0;
    }

    /**
     * @brief Run a fixture/real mission, drain it, and print its JSON.
     *
     * @param fixture `true` to run `buildMissionFixture()`, `false` to run the
     *                authored `buildMission()`.
     * @return 0 on success, 1 if the drain exceeded the tick budget (TC9).
     */
    int runDump(bool fixture)
    {
        ctpsSetup();

        if (fixture)
        {
            buildMissionFixture();
        }
        else
        {
            // buildMission() self-arms via Mission.start() (see MissionScript.cpp),
            // so the authored and fixture paths each start exactly once.
            buildMission();
        }

        if (!drainMission())
        {
            return 1;
        }

        if (fixture)
        {
            // Mirror the self-test op coverage so `--fixture` JSON also contains
            // the AC4 manip + drive segment.
            recordManipDriveSegment();
        }

        emitJson();
        return 0;
    }
}    // namespace

/**
 * @brief CTPS dump entry point. Dispatches on argv[1] (see file docstring).
 *
 * @param argc Argument count.
 * @param argv Argument vector. `argv[1]` selects the mode:
 *             `--self-test` / `--fixture` / (default = authored mission dump).
 * @return 0 on success, 1 on a self-test mismatch or a drain-budget overrun.
 */
int main(int argc, char** argv)
{
    const char* mode = (argc > 1) ? argv[1] : "";

    if (std::strcmp(mode, "--self-test") == 0)
    {
        return runSelfTest();
    }
    if (std::strcmp(mode, "--fixture") == 0)
    {
        return runDump(/*fixture=*/true);
    }
    return runDump(/*fixture=*/false);
}
