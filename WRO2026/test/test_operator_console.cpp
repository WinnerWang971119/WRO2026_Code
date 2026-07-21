/**
 * @file test_operator_console.cpp
 * @brief Host unit tests for the headless operator-diagnostics console core (T8, TC1-TC28).
 *
 * Links the REAL `OperatorConsole.cpp` + `ConsoleTypes.h` directly -- NO mocks (neither
 * `hal_mocks/` nor `path_mocks/`). The core is `<stdint.h>`-only (see OperatorConsole.h), so
 * this is the ONE host test target with zero Arduino/MatrixMiniR4 surface; that absence is
 * itself the assertion (a future Arduino dependency in the core would break this compile).
 *
 * Build (mirrors what CI's ctest target -- added separately by T9 -- will run):
 *   c++ -std=c++17 -Wall -Wextra -I WRO2026 WRO2026/test/test_operator_console.cpp \
 *       WRO2026/src/operator/OperatorConsole.cpp -o /tmp/t8 && /tmp/t8
 *
 * `ConsoleInput` fixtures are plain structs built in-test; `Harness` bundles one
 * `OperatorConsoleClass` instance with its `ConsoleInput` and drives it tick-by-tick. Every
 * helper mirrors the pinned adapter tick order (docs/plans/2026-07-20-operator-diagnostics-
 * console.md, "Adapter tick order"): PULLED fields are written and `seq` bumped BEFORE
 * `notifyReadDone()`.
 */

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "config/ConfigGlobal.h"
#include "config/ConfigUi.h"
#include "src/operator/ConsoleTypes.h"
#include "src/operator/OperatorConsole.h"

static int g_failures = 0;

static void check(bool cond, const char* what)
{
    if (!cond)
    {
        std::printf("FAIL: %s\n", what);
        ++g_failures;
    }
}

/**
 * @brief Format a per-iteration assertion message, so a failure inside a loop names the
 *        offending item / label / fault code instead of repeating one generic string.
 * @note Single shared buffer: the result must be consumed by the very next `check()` call.
 */
static const char* msgf(const char* fmt, ...)
{
    static char buf[160];
    va_list     args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return buf;
}

/**
 * @brief The label text under the cursor (row body, past the 2-char prefix).
 *
 * `ConsoleFrame::cursorRow` documents `0xFF` as the "no cursor" sentinel, so indexing
 * `rows[cursorRow]` unguarded is out-of-bounds whenever a renderer legitimately (or
 * buggily) leaves no cursor. Asserts the row index first and returns an empty string on the
 * sentinel, turning a would-be UB read into a clean, named failure.
 */
static const char* cursorLabel(const Console::ConsoleFrame& f, const char* what)
{
    check(f.cursorRow < Console::FRAME_ROWS, what);
    if (f.cursorRow >= Console::FRAME_ROWS)
    {
        return "";
    }
    return f.rows[f.cursorRow] + 2;
}

// ---------------------------------------------------------------------------------------
// VisionClass::Fault code mirror -- OperatorConsole.cpp cannot include Vision.h (Arduino
// surface), so it hand-mirrors these four codes; the test mirrors them the same way.
// ---------------------------------------------------------------------------------------
namespace VisionFault
{
    constexpr uint8_t NONE         = 0;
    constexpr uint8_t READ_TIMEOUT = 1;
    constexpr uint8_t BAD_CHECKSUM = 2;
    constexpr uint8_t BAD_CELL     = 3;
}    // namespace VisionFault

/** @brief The item labels, in documented order -- shared by TC1/TC2/TC4/TC7. */
static const char* const kLabels[Console::ITEM_COUNT] = {
    "SELF TEST", "BATT",       "LINE VIEW", "VISION VIEW", "VISION TEST",
    "GYRO VIEW", "GYRO ZERO",  "CAL LINE",  "CAL MOTOR",   "EXIT",
};

// ---------------------------------------------------------------------------------------
// Fixture builders
// ---------------------------------------------------------------------------------------

static Console::BootReport allGoodBoot()
{
    Console::BootReport b{};
    b.batt = b.lineArray = b.yaw = b.drive = b.lineFollower = b.intersection = true;
    b.manipulator = b.localization = b.pathTracker = b.mission = b.tasks = b.vision = b.trace =
        true;
    b.battVoltageAtBoot = 12.0f;
    return b;
}

static Console::DiagSample healthySample()
{
    Console::DiagSample s{};
    s.seq              = 1;
    s.battVolts         = 12.0f;
    s.battPercent       = 80;
    s.battReadOk        = true;
    s.battRetryPending  = false;
    s.gyroYawDeg        = 0.0;
    s.gyroHealthy       = true;
    s.gyroReadOk        = true;
    s.junctionType      = 0;
    s.lineOnline        = true;
    s.lineError         = 0.0f;
    for (uint8_t b = 0; b < 2; ++b)
    {
        s.lineBoardPresent[b] = true;
        s.lineBoardOk[b]      = true;
        for (uint8_t c = 0; c < 8; ++c)
        {
            s.lineRaw[b][c] = 5;
        }
    }
    s.lineBoardCount = 2;
    for (uint8_t i = 0; i < 12; ++i)
    {
        s.visionCell[i] = 0;
    }
    s.visionFaultKind = VisionFault::NONE;
    return s;
}

static Console::ConsoleInput makeInput()
{
    Console::ConsoleInput in{};
    in.btnUp       = false;
    in.btnDown     = false;
    in.nowMs       = 0;
    in.missionArmed = false;
    in.sample      = healthySample();
    in.boot        = allGoodBoot();
    return in;
}

// ---------------------------------------------------------------------------------------
// Harness: one OperatorConsoleClass instance + its ConsoleInput, ticked explicitly.
// ---------------------------------------------------------------------------------------

struct Harness
{
    OperatorConsoleClass  core;
    Console::ConsoleInput in;

    Harness()
    {
        core.begin();
        in = makeInput();
    }

    Console::ConsoleFrame tick()
    {
        Console::ConsoleFrame f{};
        core.update(in, f);
        return f;
    }

    /** @brief Latch + consume the one-shot open() request. */
    void open()
    {
        core.open();
        tick();
    }
};

// ---------------------------------------------------------------------------------------
// Tick-sequencing helpers -- every one mirrors the pinned adapter tick order.
// ---------------------------------------------------------------------------------------

/** @brief BTN_DOWN edge (press tick) then release (release tick); returns the release frame. */
static Console::ConsoleFrame tapDown(Harness& h)
{
    h.in.btnDown = true;
    h.tick();
    h.in.btnDown = false;
    return h.tick();
}

/** @brief BTN_UP edge (press tick, returned) then release (release tick, for cleanliness). */
static Console::ConsoleFrame confirmCursor(Harness& h)
{
    h.in.btnUp              = true;
    Console::ConsoleFrame f = h.tick();
    h.in.btnUp              = false;
    h.tick();
    return f;
}

/** @brief Advance time only (no seq bump) -- for dwell / timeout waits. */
static Console::ConsoleFrame stepMs(Harness& h, uint32_t deltaMs)
{
    h.in.nowMs += deltaMs;
    return h.tick();
}

/** @brief PUSHED-style refresh: bump `seq` (simulating any adapter cadence write) and tick. */
static Console::ConsoleFrame bump(Harness& h, uint32_t deltaMs = 1)
{
    h.in.nowMs += deltaMs;
    h.in.sample.seq += 1;
    return h.tick();
}

/** @brief Begin/latch tick for whichever check is next (no seq bump needed to latch). */
static Console::ConsoleFrame beginCheck(Harness& h)
{
    return stepMs(h, 1);
}

/**
 * @brief Complete a dispatched read per the pinned adapter tick order: the caller has already
 *        written the PULLED fields into `h.in.sample`; this bumps `seq`, calls
 *        `notifyReadDone()`, THEN ticks -- fields + seq strictly BEFORE the notify.
 */
static Console::ConsoleFrame completeRead(Harness& h, Console::ActionRequest which, uint32_t deltaMs = 1)
{
    h.in.nowMs += deltaMs;
    h.in.sample.seq += 1;
    h.core.notifyReadDone(which);
    return h.tick();
}

/** @brief Passes a read-free check (BATT/GYRO) that is already latched: judge, then dwell. */
static Console::ConsoleFrame passSimpleCheck(Harness& h)
{
    bump(h);
    return stepMs(h, Config::UI_SELFTEST_STEP_MS);
}

/** @brief Enter SELF TEST from a fresh, unopened Harness (cursor already at item 0). */
static Console::ConsoleFrame enterSelfTest(Harness& h)
{
    h.open();
    return confirmCursor(h);
}

/** @brief Passes VISION with a healthy (NONE-fault) read, then clears its dwell. */
static void passVisionOk(Harness& h)
{
    beginCheck(h);
    h.in.sample.visionFaultKind = VisionFault::NONE;
    completeRead(h, Console::ActionRequest::READ_MOSAIC);
    stepMs(h, Config::UI_SELFTEST_STEP_MS);
}

/** @brief Drives BATT -> GYRO -> VISION to PASS and latches Check::LINE (read dispatched). */
static void reachLineCheck(Harness& h)
{
    passSimpleCheck(h);           // BATT (already latched by enterSelfTest's confirm)
    beginCheck(h);
    passSimpleCheck(h);           // GYRO
    passVisionOk(h);              // VISION
    beginCheck(h);                // latches LINE, emits READ_LINE_BOARDS
}

// =========================================================================================
// Test cases
// =========================================================================================

int main()
{
    // TC1 -- item table/labels: order + count.
    {
        Harness h;
        h.open();
        check(Console::ITEM_COUNT == 10, "TC1: ITEM_COUNT == 10");
        Console::ConsoleFrame f = h.tick();
        for (uint8_t i = 0; i < Console::ITEM_COUNT; ++i)
        {
            const char* label = cursorLabel(f, msgf("TC1: item %u has a visible cursor row", i));
            check(std::strcmp(label, kLabels[i]) == 0,
                  msgf("TC1: item %u label order/spelling -- want \"%s\", got \"%s\"", i, kLabels[i],
                       label));
            if (i + 1 < Console::ITEM_COUNT)
            {
                f = tapDown(h);
            }
        }
    }

    // TC2 -- BTN_DOWN cycles the cursor through all entries and wraps.
    {
        Harness h;
        h.open();
        Console::ConsoleFrame f = h.tick();
        check(std::strcmp(cursorLabel(f, "TC2: start frame has a visible cursor row"), kLabels[0]) ==
                  0,
              "TC2: cursor starts at 0");
        for (int i = 1; i <= Console::ITEM_COUNT; ++i)
        {
            f                   = tapDown(h);
            const int expected = i % Console::ITEM_COUNT;
            const char* label =
                cursorLabel(f, msgf("TC2: tap %d leaves a visible cursor row", i));
            check(std::strcmp(label, kLabels[expected]) == 0,
                  msgf("TC2: tap %d -> item %d, want \"%s\", got \"%s\"", i, expected,
                       kLabels[expected], label));
        }
    }

    // TC3 -- scroll window at both ends; cursor always visible.
    {
        Harness h;
        h.open();
        Console::ConsoleFrame f = h.tick();
        check(std::strcmp(f.rows[1] + 2, "SELF TEST") == 0, "TC3: cursor0 window row1");
        check(std::strcmp(f.rows[2] + 2, "BATT") == 0, "TC3: cursor0 window row2");
        check(std::strcmp(f.rows[3] + 2, "LINE VIEW") == 0, "TC3: cursor0 window row3");
        for (int i = 0; i < Console::ITEM_COUNT - 1; ++i)
        {
            f = tapDown(h);
        }
        check(std::strcmp(f.rows[1] + 2, "CAL LINE") == 0, "TC3: bottom window row1");
        check(std::strcmp(f.rows[2] + 2, "CAL MOTOR") == 0, "TC3: bottom window row2");
        check(std::strcmp(f.rows[3] + 2, "EXIT") == 0, "TC3: bottom window row3");
        check(f.cursorRow == 3, "TC3: cursor stays visible at the bottom end");
    }

    // TC4 -- rendered rows are exact at cursor 0.
    {
        Harness h;
        h.open();
        Console::ConsoleFrame f = h.tick();
        check(std::strcmp(f.rows[0], "DIAGNOSTICS") == 0, "TC4: title row exact");
        check(std::strcmp(f.rows[1], "> SELF TEST") == 0, "TC4: cursor row exact");
        check(std::strcmp(f.rows[2], "  BATT") == 0, "TC4: plain row 2 exact");
        check(std::strcmp(f.rows[3], "  LINE VIEW") == 0, "TC4: plain row 3 exact");
    }

    // TC5 -- a level held 3 ticks produces exactly one cursor advance.
    {
        Harness h;
        h.open();
        h.in.btnDown             = true;
        Console::ConsoleFrame f1 = h.tick();
        Console::ConsoleFrame f2 = h.tick();
        Console::ConsoleFrame f3 = h.tick();
        check(std::strcmp(cursorLabel(f1, "TC5: tick 1 has a visible cursor row"), "BATT") == 0,
              "TC5: held button advances once");
        check(std::strcmp(cursorLabel(f2, "TC5: tick 2 has a visible cursor row"), "BATT") == 0,
              "TC5: no further advance while held (tick 2)");
        check(std::strcmp(cursorLabel(f3, "TC5: tick 3 has a visible cursor row"), "BATT") == 0,
              "TC5: no further advance while held (tick 3)");
        h.in.btnDown = false;
        h.tick();
    }

    // TC6 -- WAIT_RELEASE blocks re-trigger: confirm with both held -> no action until release.
    {
        Harness h;
        h.open();
        check(h.core.state() == Console::State::LIST, "TC6: precondition LIST");
        h.in.btnUp               = true;
        h.in.btnDown             = true;
        Console::ConsoleFrame f = h.tick();
        check(h.core.state() == Console::State::WAIT_RELEASE, "TC6: chord enters WAIT_RELEASE");
        check(f.action == Console::ActionRequest::NONE, "TC6: chord entry emits no action");
        check(std::strcmp(f.rows[1], "RELEASE BUTTONS") == 0, "TC6: chord entry shows the wait screen");

        // Every tick of the wait is asserted, not a sampled one. A release gate deleted outright
        // ("release whatever the buttons read") does NOT park the FSM: it oscillates
        // WAIT_RELEASE -> LIST -> WAIT_RELEASE, re-entering the wait each tick off the still-held
        // chord, so a test that samples only the Nth tick can land back on WAIT_RELEASE and pass
        // against a core with no gate at all. `departures` is the oscillation witness -- the
        // rendered row is authoritative about which state actually produced each frame, so a
        // WAIT_RELEASE observed on both sides of a LIST tick cannot masquerade as a held wait.
        int departures = 0;
        for (int i = 0; i < 6; ++i)
        {
            f = h.tick();
            check(h.core.state() == Console::State::WAIT_RELEASE,
                  msgf("TC6: still waiting on hold tick %d while both held", i));
            check(f.action == Console::ActionRequest::NONE,
                  msgf("TC6: no action on hold tick %d while both held", i));
            if (std::strcmp(f.rows[1], "RELEASE BUTTONS") != 0)
            {
                ++departures;
            }
        }
        check(departures == 0,
              "TC6: the console never leaves the wait screen while the chord is held (no "
              "WAIT_RELEASE -> LIST -> WAIT_RELEASE oscillation)");

        h.in.btnUp   = false;
        h.in.btnDown = false;
        f            = h.tick();
        check(h.core.state() == Console::State::LIST, "TC6: resumes to LIST once both release");
        check(f.action == Console::ActionRequest::NONE, "TC6: no confirm action fired by the chord");
    }

    // TC7 -- every item dispatches to its documented state/action.
    {
        struct Case
        {
            Console::Item         item;
            Console::State         expectState;
            Console::ActionRequest expectAction;
        };
        static const Case cases[] = {
            {Console::Item::SELF_TEST, Console::State::SELF_TEST, Console::ActionRequest::NONE},
            {Console::Item::BATT, Console::State::VIEW_BATT, Console::ActionRequest::NONE},
            {Console::Item::LINE_VIEW, Console::State::VIEW_LINE, Console::ActionRequest::READ_LINE_BOARDS},
            {Console::Item::VISION_VIEW, Console::State::VIEW_VISION, Console::ActionRequest::READ_MOSAIC},
            {Console::Item::VISION_TEST, Console::State::VISION_TEST, Console::ActionRequest::READ_MOSAIC},
            {Console::Item::GYRO_VIEW, Console::State::VIEW_GYRO, Console::ActionRequest::NONE},
            {Console::Item::CAL_LINE, Console::State::WAIT_RELEASE, Console::ActionRequest::CAL_LINE},
            {Console::Item::CAL_MOTOR, Console::State::WAIT_RELEASE, Console::ActionRequest::CAL_MOTOR},
            {Console::Item::EXIT, Console::State::CLOSED, Console::ActionRequest::NONE},
        };
        for (const auto& c : cases)
        {
            Harness h;
            h.open();
            for (uint8_t i = 0; i < static_cast<uint8_t>(c.item); ++i)
            {
                tapDown(h);
            }
            const uint8_t         idx = static_cast<uint8_t>(c.item);
            Console::ConsoleFrame f   = confirmCursor(h);
            check(h.core.state() == c.expectState,
                  msgf("TC7: item %u (%s) confirm yields its documented state -- want %u, got %u",
                       idx, kLabels[idx], static_cast<unsigned>(c.expectState),
                       static_cast<unsigned>(h.core.state())));
            check(f.action == c.expectAction,
                  msgf("TC7: item %u (%s) confirm yields its documented action -- want %u, got %u",
                       idx, kLabels[idx], static_cast<unsigned>(c.expectAction),
                       static_cast<unsigned>(f.action)));
        }

        // GYRO ZERO round trip: emits the action, parks, then returns to LIST on notifyActionDone.
        Harness h;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::GYRO_ZERO); ++i)
        {
            tapDown(h);
        }
        Console::ConsoleFrame f = confirmCursor(h);
        check(f.action == Console::ActionRequest::GYRO_ZERO, "TC7: GYRO ZERO emits ActionRequest::GYRO_ZERO");
        check(h.core.state() == Console::State::WAIT_RELEASE, "TC7: GYRO ZERO parks in WAIT_RELEASE");
        h.core.notifyActionDone(Console::ActionRequest::GYRO_ZERO, true);
        h.tick();
        check(h.core.state() == Console::State::LIST, "TC7: GYRO ZERO returns to LIST after notifyActionDone");
    }

    // TC8 -- no view timeout across 10000 idle ticks.
    {
        Harness h;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::LINE_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_LINE, "TC8: precondition VIEW_LINE");
        for (int i = 0; i < 10000; ++i)
        {
            h.in.nowMs += 1;
            h.tick();
        }
        check(h.core.state() == Console::State::VIEW_LINE, "TC8: no timeout after 10000 ticks");
    }

    // TC9 -- wantsLineData() truth table.
    {
        Harness hClosed;
        check(hClosed.core.wantsLineData() == false, "TC9: false when CLOSED");

        Harness h;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::LINE_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_LINE, "TC9: precondition VIEW_LINE");
        check(h.core.wantsLineData() == true, "TC9: true in VIEW_LINE");

        h.in.boot.lineArray = false;
        h.tick();
        check(h.core.wantsLineData() == false,
              "TC9: false whenever boot.lineArray==false, even inside VIEW_LINE");

        Harness h2;
        h2.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::GYRO_VIEW); ++i)
        {
            tapDown(h2);
        }
        confirmCursor(h2);
        check(h2.core.state() == Console::State::VIEW_GYRO, "TC9: precondition VIEW_GYRO");
        check(h2.core.wantsLineData() == false, "TC9: false in VIEW_GYRO");

        Harness h3;
        enterSelfTest(h3);
        reachLineCheck(h3);
        check(h3.core.wantsLineData() == true,
              "TC9: true inside SELF TEST while Check::LINE's read is in flight");
        completeRead(h3, Console::ActionRequest::READ_LINE_BOARDS);
        check(h3.core.wantsLineData() == false, "TC9: false once the line verdict is recorded");
    }

    // TC10 -- SELF TEST order: all healthy -> BATT->GYRO->VISION->LINE then DONE.
    {
        Harness h;
        Console::ConsoleFrame f = enterSelfTest(h);
        check(std::strstr(f.rows[1], "BATT") != nullptr, "TC10: sequence begins at BATT");
        f = passSimpleCheck(h);
        check(std::strstr(f.rows[1], "GYRO") != nullptr, "TC10: advances to GYRO after BATT passes");
        beginCheck(h);
        f = passSimpleCheck(h);
        check(std::strstr(f.rows[1], "VISION") != nullptr, "TC10: advances to VISION after GYRO passes");
        beginCheck(h);
        h.in.sample.visionFaultKind = VisionFault::NONE;
        completeRead(h, Console::ActionRequest::READ_MOSAIC);
        f = stepMs(h, Config::UI_SELFTEST_STEP_MS);
        check(std::strstr(f.rows[1], "LINE") != nullptr, "TC10: advances to LINE after VISION passes");
        beginCheck(h);
        completeRead(h, Console::ActionRequest::READ_LINE_BOARDS);
        f = stepMs(h, Config::UI_SELFTEST_STEP_MS);
        check(std::strstr(f.rows[2], "ALL PASS") != nullptr, "TC10: all four checks pass in order -> DONE");
    }

    // TC11 -- halt on first failure: GYRO unhealthy stops the sequence; VISION/LINE never run.
    {
        Harness h;
        h.in.sample.gyroHealthy = false;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        Console::ConsoleFrame f = bump(h);
        check(std::strstr(f.rows[2], "GYRO FAIL") != nullptr, "TC11: GYRO fails");
        check(f.signal == Console::Signal::FAIL, "TC11: FAIL publishes Signal::FAIL immediately");
        for (int i = 0; i < 10; ++i)
        {
            f = stepMs(h, Config::UI_SELFTEST_STEP_MS);
            check(std::strstr(f.rows[1], "GYRO") != nullptr, "TC11: stays parked on GYRO");
            check(f.action == Console::ActionRequest::NONE, "TC11: no further read dispatched after halting");
        }
    }

    // TC12 -- verdict signal: all-pass -> Signal::PASS; any failure -> Signal::FAIL.
    {
        Harness h;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        passSimpleCheck(h);
        beginCheck(h);
        h.in.sample.visionFaultKind = VisionFault::NONE;
        completeRead(h, Console::ActionRequest::READ_MOSAIC);
        stepMs(h, Config::UI_SELFTEST_STEP_MS);
        beginCheck(h);
        completeRead(h, Console::ActionRequest::READ_LINE_BOARDS);
        Console::ConsoleFrame f = stepMs(h, Config::UI_SELFTEST_STEP_MS);
        check(f.signal == Console::Signal::PASS, "TC12: all-pass drives Signal::PASS");
    }
    {
        Harness h;
        h.in.sample.gyroReadOk = false;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        Console::ConsoleFrame f = bump(h);
        check(f.signal == Console::Signal::FAIL, "TC12: any failure drives Signal::FAIL");
    }

    // TC13 -- line check uses presence, not values.
    {
        // PASS despite all-zero raw + lineBoardOk=false,false, because both boards are present.
        Harness h;
        enterSelfTest(h);
        reachLineCheck(h);
        h.in.sample.lineBoardPresent[0] = true;
        h.in.sample.lineBoardPresent[1] = true;
        h.in.sample.lineBoardOk[0]      = false;
        h.in.sample.lineBoardOk[1]      = false;
        for (uint8_t b = 0; b < 2; ++b)
        {
            for (uint8_t c = 0; c < 8; ++c)
            {
                h.in.sample.lineRaw[b][c] = 0;
            }
        }
        Console::ConsoleFrame f = completeRead(h, Console::ActionRequest::READ_LINE_BOARDS);
        check(std::strstr(f.rows[2], "LINE PASS") != nullptr,
              "TC13: PASS on all-present despite all-zero raw + lineBoardOk=false,false");
    }
    {
        // FAIL despite healthy raw + lineBoardOk=true,true, because board 1 is absent.
        Harness h;
        enterSelfTest(h);
        reachLineCheck(h);
        h.in.sample.lineBoardPresent[0] = true;
        h.in.sample.lineBoardPresent[1] = false;
        h.in.sample.lineBoardOk[0]      = true;
        h.in.sample.lineBoardOk[1]      = true;
        for (uint8_t b = 0; b < 2; ++b)
        {
            for (uint8_t c = 0; c < 8; ++c)
            {
                h.in.sample.lineRaw[b][c] = 5;
            }
        }
        Console::ConsoleFrame f = completeRead(h, Console::ActionRequest::READ_LINE_BOARDS);
        check(std::strstr(f.rows[2], "LINE FAIL") != nullptr,
              "TC13: FAIL on board1 absent despite healthy raw + lineBoardOk=true,true");
    }

    // TC13b -- an unconfigured board's absence is ignored.
    {
        Harness h;
        enterSelfTest(h);
        reachLineCheck(h);
        h.in.sample.lineBoardCount      = 1;
        h.in.sample.lineBoardPresent[0] = true;
        h.in.sample.lineBoardPresent[1] = false;
        Console::ConsoleFrame f = completeRead(h, Console::ActionRequest::READ_LINE_BOARDS);
        check(std::strstr(f.rows[2], "LINE PASS") != nullptr,
              "TC13b: unconfigured board 1 ignored -> PASS");
    }

    // TC13c -- LINE VIEW raw-row validity: lineBoardOk==false renders NO DATA, never zeros.
    {
        Harness h;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::LINE_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        h.in.sample.lineBoardOk[0] = true;
        h.in.sample.lineBoardOk[1] = false;
        for (uint8_t c = 0; c < 8; ++c)
        {
            h.in.sample.lineRaw[0][c] = 3;
            h.in.sample.lineRaw[1][c] = 0;
        }
        Console::ConsoleFrame f = h.tick();
        // Absence of "NO DATA" alone would also hold for a blank row, so assert the eight raw
        // digits are actually there.
        check(std::strstr(f.rows[1], "33333333") != nullptr,
              "TC13c: board0 (ok) renders all eight raw digits");
        check(std::strstr(f.rows[1], "NO DATA") == nullptr,
              "TC13c: board0 (ok) is not reported as NO DATA");
        check(std::strstr(f.rows[2], "NO DATA") != nullptr,
              "TC13c: board1 (lineBoardOk=false) renders NO DATA, not zeros");
    }

    // TC14 -- vision fault classification: NONE/BAD_CELL pass; READ_TIMEOUT/BAD_CHECKSUM fail;
    // and any code OUTSIDE the four-code mirror fails closed. judgeVision() is an ALLOW-list
    // precisely so an unrecognised code -- a `VisionClass::Fault` the console's hand-mirrored
    // copy has not learned about -- reads as "not fit to run"; a `default: PASS` would silently
    // green-light every future fault kind in the advisory that gates a competition run.
    {
        struct VC
        {
            uint8_t     code;
            bool        expectPass;
            const char* name;
        };
        static const VC cases[] = {
            {VisionFault::BAD_CELL, true, "BAD_CELL"},
            {VisionFault::READ_TIMEOUT, false, "READ_TIMEOUT"},
            {VisionFault::BAD_CHECKSUM, false, "BAD_CHECKSUM"},
            {9, false, "unknown code 9"},
            {255, false, "unknown code 255"},
        };
        for (const auto& c : cases)
        {
            Harness h;
            enterSelfTest(h);
            passSimpleCheck(h);
            beginCheck(h);
            passSimpleCheck(h);
            Console::ConsoleFrame f = beginCheck(h);
            check(f.action == Console::ActionRequest::READ_MOSAIC,
                  msgf("TC14: VISION dispatches READ_MOSAIC (fault %s)", c.name));
            h.in.sample.visionFaultKind = c.code;
            f                           = completeRead(h, Console::ActionRequest::READ_MOSAIC);
            const char* want            = c.expectPass ? "VISION PASS" : "VISION FAIL";
            check(std::strstr(f.rows[2], want) != nullptr,
                  msgf("TC14: fault %s (code %u) must render \"%s\", got \"%s\"", c.name, c.code,
                       want, f.rows[2]));

            if (c.expectPass)
            {
                continue;
            }
            // A FAIL must also halt the sequence on VISION and publish Signal::FAIL -- a judge
            // that fails open would instead sail on to the LINE check.
            check(f.signal == Console::Signal::FAIL,
                  msgf("TC14: fault %s publishes Signal::FAIL", c.name));
            f = stepMs(h, Config::UI_SELFTEST_STEP_MS * 2);
            check(std::strstr(f.rows[1], "VISION") != nullptr,
                  msgf("TC14: fault %s halts the sequence parked on VISION", c.name));
            check(h.core.state() == Console::State::SELF_TEST,
                  msgf("TC14: fault %s leaves the console halted in SELF_TEST", c.name));
        }
    }

    // TC15 -- battery retry then fail: pending withholds the verdict; exhausted retry FAILs+halts.
    {
        Harness h;
        h.in.sample.battVolts        = 0.0f;
        h.in.sample.battRetryPending = true;
        enterSelfTest(h);
        Console::ConsoleFrame f = bump(h);
        check(std::strstr(f.rows[2], "RUNNING") != nullptr, "TC15: retry pending withholds the verdict");
        h.in.sample.battRetryPending = false;
        f                            = bump(h);
        check(std::strstr(f.rows[2], "BATT FAIL") != nullptr, "TC15: retry exhausted at 0V -> FAIL");
        check(h.core.state() == Console::State::SELF_TEST, "TC15: halts on the failed BATT check");
        f = stepMs(h, Config::UI_SELFTEST_STEP_MS * 2);
        check(std::strstr(f.rows[2], "BATT FAIL") != nullptr, "TC15: stays halted, never advances past BATT");
    }

    // TC16 -- vision fault is always cleared, on both exit routes.
    {
        // (a) leaving VIEW_VISION via BTN_DOWN.
        Harness h;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::VISION_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_VISION, "TC16: precondition VIEW_VISION");
        h.in.btnDown             = true;
        Console::ConsoleFrame f = h.tick();
        check(h.core.state() == Console::State::LIST, "TC16: BTN_DOWN exits to LIST");
        check(f.action == Console::ActionRequest::CLEAR_VISION_FAULT,
              "TC16: leaving VIEW_VISION emits CLEAR_VISION_FAULT");
        h.in.btnDown = false;
        h.tick();
    }
    {
        // (b) aborting SELF TEST while parked mid-VISION (read already dispatched).
        Harness h;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        passSimpleCheck(h);
        Console::ConsoleFrame f = beginCheck(h);
        check(f.action == Console::ActionRequest::READ_MOSAIC, "TC16: precondition VISION read dispatched");
        h.in.btnDown = true;
        f            = h.tick();
        check(h.core.state() == Console::State::LIST, "TC16: BTN_DOWN aborts SELF TEST to LIST");
        check(f.action == Console::ActionRequest::CLEAR_VISION_FAULT,
              "TC16: aborting mid-VISION check emits CLEAR_VISION_FAULT");
        h.in.btnDown = false;
        h.tick();
    }

    // TC17 -- gyro drift is wrap-correct: baseline 179, sample -179 -> drift +2, not -358.
    {
        Harness h;
        h.in.sample.gyroYawDeg = 179.0;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::GYRO_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_GYRO, "TC17: precondition VIEW_GYRO");
        h.in.sample.gyroYawDeg  = -179.0;
        Console::ConsoleFrame f = h.tick();
        check(std::strstr(f.rows[3], "+2.0") != nullptr, "TC17: wrap-correct drift reads +2.0");
        check(std::strstr(f.rows[3], "-358") == nullptr, "TC17: never the long way around (-358)");
    }

    // TC18 -- drift baseline resets on re-entry.
    {
        Harness h;
        h.in.sample.gyroYawDeg = 50.0;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::GYRO_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        h.in.sample.gyroYawDeg  = 80.0;
        Console::ConsoleFrame f = h.tick();
        check(std::strstr(f.rows[3], "+30.0") != nullptr, "TC18: drift accumulates before re-entry");

        h.in.btnDown = true;
        h.tick();
        h.in.btnDown = false;
        h.tick();
        check(h.core.state() == Console::State::LIST, "TC18: exited to LIST");

        h.in.sample.gyroYawDeg = 80.0;    // re-entry sample -- baseline must reset HERE
        f                      = confirmCursor(h);
        check(std::strcmp(f.rows[3], "DRIFT: 0.0") == 0, "TC18: drift baseline resets to 0 at re-entry");
    }

    // TC18b -- the counterpart to TC18: a both-button chord pressed INSIDE VIEW_GYRO is a
    // *resume*, not a transition, so releasing it must NOT re-baseline the drift watch. AC8
    // measures drift since the view OPENED; re-running VIEW_GYRO's entry effect on a thumb-brush
    // would silently zero the number the operator is reading and hand them a wrong value they
    // trust. Classifying the chord as a transition (_enterState instead of _resumeState) is the
    // exact defect this pins.
    {
        Harness h;
        h.in.sample.gyroYawDeg = 50.0;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::GYRO_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_GYRO, "TC18b: precondition VIEW_GYRO");

        h.in.sample.gyroYawDeg  = 80.0;    // 30 deg of drift accrued against the entry baseline
        Console::ConsoleFrame f = h.tick();
        check(std::strstr(f.rows[3], "+30.0") != nullptr, "TC18b: drift accrues to +30.0 in view");

        h.in.btnUp   = true;    // ambiguous chord -- parks, but never leaves the view
        h.in.btnDown = true;
        f            = h.tick();
        check(h.core.state() == Console::State::WAIT_RELEASE, "TC18b: chord parks in WAIT_RELEASE");
        f = h.tick();
        check(h.core.state() == Console::State::WAIT_RELEASE, "TC18b: stays parked while held");

        h.in.btnUp   = false;
        h.in.btnDown = false;
        f            = h.tick();
        check(h.core.state() == Console::State::VIEW_GYRO, "TC18b: releasing the chord resumes VIEW_GYRO");
        check(std::strstr(f.rows[3], "+30.0") != nullptr,
              "TC18b: the ORIGINAL drift baseline survives the chord -- drift still reads +30.0");
        check(std::strcmp(f.rows[3], "DRIFT: 0.0") != 0,
              "TC18b: the chord resume must not re-zero the drift baseline");

        // And it keeps accumulating from that same baseline, not from the chord release.
        h.in.sample.gyroYawDeg = 90.0;
        f                      = h.tick();
        check(std::strstr(f.rows[3], "+40.0") != nullptr,
              "TC18b: drift continues from the entry baseline (50 deg), not the release sample");
    }

    // TC19 -- armed force-closes from any state.
    {
        Harness h;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::LINE_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_LINE, "TC19: precondition VIEW_LINE");
        h.in.missionArmed = true;
        h.tick();
        check(h.core.state() == Console::State::CLOSED, "TC19: missionArmed force-closes from VIEW_LINE");
    }
    {
        Harness h;
        h.open();
        h.in.missionArmed = true;
        h.tick();
        check(h.core.state() == Console::State::CLOSED, "TC19: missionArmed force-closes from LIST");
    }
    {
        Harness h;
        enterSelfTest(h);
        h.in.missionArmed = true;
        h.tick();
        check(h.core.state() == Console::State::CLOSED, "TC19: missionArmed force-closes from SELF_TEST");
    }

    // TC20 -- row truncation: an overflowing boot-failure list truncates with a '+' marker,
    // and every row stays within TEXT_COLS + NUL. (All 13 boot flags false is the one real
    // input path that overflows the 2-row failure list.)
    {
        Harness h;
        h.in.boot = Console::BootReport{};
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::BATT); ++i)
        {
            tapDown(h);
        }
        Console::ConsoleFrame f = confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_BATT, "TC20: precondition VIEW_BATT");
        for (uint8_t r = 0; r < Console::FRAME_ROWS; ++r)
        {
            check(std::strlen(f.rows[r]) <= Console::TEXT_COLS, "TC20: every row stays within TEXT_COLS");
        }
        check(std::strchr(f.rows[3], '+') != nullptr,
              "TC20: an overflowing boot-failure list truncates with a '+' marker");
    }

    // TC21 -- open latch is one-shot.
    {
        Harness h;
        h.core.open();
        Console::ConsoleFrame f = h.tick();
        check(h.core.state() == Console::State::LIST, "TC21: open()+tick -> LIST");
        check(std::strcmp(cursorLabel(f, "TC21: opening frame has a visible cursor row"),
                          "SELF TEST") == 0,
              "TC21: cursor reset to SELF_TEST");

        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::LINE_VIEW); ++i)
        {
            tapDown(h);
        }
        confirmCursor(h);
        check(h.core.state() == Console::State::VIEW_LINE, "TC21: precondition VIEW_LINE");
        h.core.open();    // repeated call while already open
        h.tick();
        check(h.core.state() == Console::State::VIEW_LINE, "TC21: repeated open() while open is a no-op");
    }

    // TC22 -- REGRESSION GUARD: vision cannot pass on the boot state. Sample stays fixed at
    // visionFaultKind==NONE (Vision::begin()'s post-boot value) and notifyReadDone(READ_MOSAIC)
    // is NEVER called, but `seq` keeps advancing on the (realistic) adapter PUSHED cadence.
    // An implementation that judges on `seq` alone -- ignoring whether the dispatched read
    // actually completed -- would PASS an unplugged camera here.
    {
        Harness h;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        passSimpleCheck(h);

        Console::ConsoleFrame f = beginCheck(h);    // latches VISION, dispatches READ_MOSAIC
        check(f.action == Console::ActionRequest::READ_MOSAIC, "TC22: VISION dispatches READ_MOSAIC on entry");
        check(h.in.sample.visionFaultKind == VisionFault::NONE,
              "TC22: precondition -- sample is stuck at the boot-state NONE fault");

        int mosaicRequests = 0;
        for (int i = 0; i < 20; ++i)
        {
            // Adapter PUSHED cadence: seq keeps advancing, notifyReadDone(READ_MOSAIC) never
            // called, visionFaultKind never rewritten.
            f = bump(h, 100);
            if (f.action == Console::ActionRequest::READ_MOSAIC)
            {
                ++mosaicRequests;
            }
            check(h.core.state() == Console::State::SELF_TEST, "TC22: never leaves SELF_TEST early");
        }
        check(mosaicRequests == 0,
              "TC22: READ_MOSAIC is emitted exactly once -- never re-issued while in flight");
        check(std::strstr(f.rows[2], "VISION") != nullptr && std::strstr(f.rows[2], "NO DATA") != nullptr,
              "TC22: times out to VISION NO DATA, never PASS, on the un-refreshed boot sample");
        check(f.signal == Console::Signal::FAIL, "TC22: timeout publishes Signal::FAIL");
    }

    // TC23 -- a stale (seq-frozen) sample is never judged; timeout still fails closed.
    {
        Harness h;
        Console::ConsoleFrame f = enterSelfTest(h);
        for (int i = 0; i < 5; ++i)
        {
            f = stepMs(h, 100);
            check(h.core.state() == Console::State::SELF_TEST, "TC23: still running while seq is frozen");
            check(std::strstr(f.rows[2], "RUNNING") != nullptr, "TC23: no verdict while seq stays stale");
        }
        f = bump(h);    // bump seq before the deadline -> judges and passes.
        check(std::strstr(f.rows[2], "BATT PASS") != nullptr, "TC23: bumping seq before the deadline yields PASS");
        f = stepMs(h, Config::UI_SELFTEST_STEP_MS);
        check(std::strstr(f.rows[1], "GYRO") != nullptr, "TC23: passes and advances to the next check");
    }
    {
        Harness h;
        enterSelfTest(h);
        Console::ConsoleFrame f = stepMs(h, Config::UI_SELFTEST_CHECK_TIMEOUT_MS + 50);
        check(std::strstr(f.rows[2], "BATT NO DATA") != nullptr,
              "TC23: seq never bumped -> times out to BATT NO DATA, never a PASS");
        check(f.signal == Console::Signal::FAIL, "TC23: timeout publishes Signal::FAIL");
    }

    // TC24 -- read/judge handshake, happy path: explicit notifyReadDone for both dispatched
    // reads, ending DONE + Signal::PASS.
    {
        Harness h;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        passSimpleCheck(h);

        Console::ConsoleFrame f = beginCheck(h);
        check(f.action == Console::ActionRequest::READ_MOSAIC, "TC24: VISION dispatches READ_MOSAIC");
        h.in.sample.visionFaultKind = VisionFault::BAD_CELL;
        f                           = completeRead(h, Console::ActionRequest::READ_MOSAIC);
        check(std::strstr(f.rows[2], "VISION PASS") != nullptr, "TC24: notifyReadDone(seq+1)+BAD_CELL -> PASS");
        stepMs(h, Config::UI_SELFTEST_STEP_MS);

        f = beginCheck(h);
        check(f.action == Console::ActionRequest::READ_LINE_BOARDS, "TC24: LINE dispatches READ_LINE_BOARDS");
        h.in.sample.lineBoardPresent[0] = true;
        h.in.sample.lineBoardPresent[1] = true;
        f                               = completeRead(h, Console::ActionRequest::READ_LINE_BOARDS);
        check(std::strstr(f.rows[2], "LINE PASS") != nullptr,
              "TC24: notifyReadDone(seq+1)+present={true,true} -> PASS");
        f = stepMs(h, Config::UI_SELFTEST_STEP_MS);
        check(std::strstr(f.rows[2], "ALL PASS") != nullptr, "TC24: sequence completes to DONE");
        check(f.signal == Console::Signal::PASS, "TC24: DONE publishes Signal::PASS");
    }

    // TC25 -- at most one request in flight across a full run; each read emitted exactly once.
    {
        Harness h;
        enterSelfTest(h);
        int mosaicCount = 0, lineCount = 0, otherCount = 0;

        Console::ConsoleFrame f = bump(h);    // BATT judge
        if (f.action != Console::ActionRequest::NONE) ++otherCount;
        stepMs(h, Config::UI_SELFTEST_STEP_MS);

        f = beginCheck(h);    // GYRO begin
        if (f.action != Console::ActionRequest::NONE) ++otherCount;
        f = bump(h);    // GYRO judge
        if (f.action != Console::ActionRequest::NONE) ++otherCount;
        stepMs(h, Config::UI_SELFTEST_STEP_MS);

        f = beginCheck(h);    // VISION begin
        if (f.action == Console::ActionRequest::READ_MOSAIC) ++mosaicCount;
        h.in.sample.visionFaultKind = VisionFault::NONE;
        f                           = completeRead(h, Console::ActionRequest::READ_MOSAIC);
        if (f.action != Console::ActionRequest::NONE) ++otherCount;
        stepMs(h, Config::UI_SELFTEST_STEP_MS);

        f = beginCheck(h);    // LINE begin
        if (f.action == Console::ActionRequest::READ_LINE_BOARDS) ++lineCount;
        f = completeRead(h, Console::ActionRequest::READ_LINE_BOARDS);
        if (f.action != Console::ActionRequest::NONE) ++otherCount;
        f = stepMs(h, Config::UI_SELFTEST_STEP_MS);

        check(mosaicCount == 1, "TC25: READ_MOSAIC emitted exactly once across the run");
        check(lineCount == 1, "TC25: READ_LINE_BOARDS emitted exactly once across the run");
        check(otherCount == 0, "TC25: no unexpected extra action at any other tick");
        check(f.signal == Console::Signal::PASS, "TC25: run completes cleanly");
    }

    // TC26 -- dirty/signal are level-triggered until acknowledged.
    {
        Harness h;
        h.open();
        Console::ConsoleFrame f = h.tick();
        check(f.dirty == true, "TC26: dirty raised on first render");
        f = h.tick();
        check(f.dirty == true, "TC26: dirty stays true until notifyFrameRendered (no repaint lost)");
        h.core.notifyFrameRendered();
        f = h.tick();
        check(f.dirty == false, "TC26: dirty clears once notifyFrameRendered acknowledges it");
    }
    {
        Harness h;
        h.in.sample.gyroReadOk = false;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        Console::ConsoleFrame g = bump(h);
        check(g.signal == Console::Signal::FAIL, "TC26: signal raised on SELF TEST FAIL");
        g = h.tick();
        check(g.signal == Console::Signal::FAIL, "TC26: signal stays FAIL until notifySignalDone");
        h.core.notifySignalDone();
        g = h.tick();
        check(g.signal == Console::Signal::NONE, "TC26: signal clears once notifySignalDone acknowledges it");
    }

    // TC27 -- degraded availability truth table (Console::isDispatchable is pure).
    {
        Console::BootReport boot = allGoodBoot();
        boot.drive                = false;
        check(Console::isDispatchable(Console::Item::CAL_MOTOR, boot) == false,
              "TC27: CAL_MOTOR non-dispatchable when boot.drive==false");
        check(Console::isDispatchable(Console::Item::CAL_LINE, boot) == false,
              "TC27: CAL_LINE non-dispatchable on an unclean boot report");
        check(Console::isDispatchable(Console::Item::GYRO_ZERO, boot) == false,
              "TC27: GYRO_ZERO non-dispatchable on an unclean boot report");
        check(Console::isDispatchable(Console::Item::BATT, boot) == true, "TC27: BATT stays dispatchable");
        check(Console::isDispatchable(Console::Item::SELF_TEST, boot) == true,
              "TC27: SELF_TEST stays dispatchable");
        check(Console::isDispatchable(Console::Item::EXIT, boot) == true, "TC27: EXIT stays dispatchable");
    }
    {
        // boot.batt==false alone is a VOLTAGE verdict, never an init verdict -- it must not
        // disable any read-only entry, including BATT itself.
        Console::BootReport boot = allGoodBoot();
        boot.batt                 = false;
        check(Console::isDispatchable(Console::Item::BATT, boot) == true,
              "TC27: BATT screen unaffected by boot.batt==false");
        check(Console::isDispatchable(Console::Item::LINE_VIEW, boot) == true,
              "TC27: LINE_VIEW unaffected by boot.batt==false");
        check(Console::isDispatchable(Console::Item::VISION_VIEW, boot) == true,
              "TC27: VISION_VIEW unaffected by boot.batt==false");
        check(Console::isDispatchable(Console::Item::GYRO_VIEW, boot) == true,
              "TC27: GYRO_VIEW unaffected by boot.batt==false");
        check(Console::isDispatchable(Console::Item::SELF_TEST, boot) == true,
              "TC27: SELF_TEST unaffected by boot.batt==false");
        check(Console::isDispatchable(Console::Item::EXIT, boot) == true,
              "TC27: EXIT unaffected by boot.batt==false");
        // Hardware-writing entries still require a fully clean boot report.
        check(Console::isDispatchable(Console::Item::GYRO_ZERO, boot) == false,
              "TC27: writing entries still require boot.clean(), which boot.batt==false breaks");
    }

    // TC27b -- isDispatchable is WIRED, not merely pure. TC27 exercises the predicate in
    // isolation; this drives it through the console, because the two things that make it matter
    // are the "! " row prefix (the operator's only feedback for why nothing happens) and
    // _confirmItem's guard. Without the guard a blocked CAL_MOTOR would dispatch a blocking
    // motor-bias calibration on a boot where Drive::begin() failed.
    {
        Harness h;
        h.in.boot        = allGoodBoot();
        h.in.boot.drive  = false;    // CAL_MOTOR's own module down => also breaks boot.clean()
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::CAL_MOTOR); ++i)
        {
            tapDown(h);
        }
        Console::ConsoleFrame f = h.tick();
        check(h.core.state() == Console::State::LIST, "TC27b: precondition LIST at CAL MOTOR");
        check(std::strcmp(cursorLabel(f, "TC27b: CAL MOTOR row is the cursor row"), "CAL MOTOR") == 0,
              "TC27b: cursor sits on CAL MOTOR");
        // Cursor 7 of 9 => the window shows items 5-7 on rows 1-3, all three blocked by the
        // unclean boot report.
        check(f.cursorRow == 3, "TC27b: CAL MOTOR is the bottom window row");
        check(std::strcmp(f.rows[3], "! CAL MOTOR") == 0,
              "TC27b: a blocked entry under the cursor renders \"! \", not the \"> \" cursor prefix");
        check(std::strcmp(f.rows[2], "! CAL LINE") == 0,
              "TC27b: a blocked entry off the cursor renders \"! \" too");
        check(std::strcmp(f.rows[1], "! GYRO ZERO") == 0,
              "TC27b: every entry the unclean boot report blocks carries the \"! \" prefix");

        f = confirmCursor(h);
        check(h.core.state() == Console::State::LIST,
              "TC27b: confirming a blocked entry is a no-op -- the FSM stays in LIST");
        check(f.action == Console::ActionRequest::NONE,
              "TC27b: confirming a blocked entry emits ActionRequest::NONE, never CAL_MOTOR");
        f = h.tick();
        check(f.action == Console::ActionRequest::NONE,
              "TC27b: and no action leaks out on the following tick either");
        check(std::strcmp(cursorLabel(f, "TC27b: cursor survives the blocked confirm"),
                          "CAL MOTOR") == 0,
              "TC27b: the cursor does not move on a blocked confirm");

        // Control: a still-dispatchable entry on the SAME degraded boot report keeps the normal
        // cursor prefix, so "! " is a real per-entry verdict and not a blanket degraded-mode mark.
        f = tapDown(h);    // -> EXIT (always dispatchable)
        check(std::strcmp(f.rows[3], "> EXIT") == 0,
              "TC27b: a dispatchable entry under the cursor still renders \"> \"");
        check(std::strcmp(f.rows[2], "! CAL MOTOR") == 0,
              "TC27b: the blocked entry keeps its \"! \" prefix off the cursor");
    }

    // TC28 -- degraded SELF TEST self-fails straight from the boot report.
    {
        Harness h;
        h.in.boot.yaw = false;
        enterSelfTest(h);
        passSimpleCheck(h);    // BATT passes normally
        Console::ConsoleFrame f = beginCheck(h);
        check(f.action == Console::ActionRequest::NONE, "TC28: no gyro read requested when boot.yaw==false");
        check(std::strstr(f.rows[2], "GYRO FAIL") != nullptr,
              "TC28: GYRO check self-fails straight from the boot report");
        check(h.core.state() == Console::State::SELF_TEST, "TC28: halts at GYRO");
        f = stepMs(h, Config::UI_SELFTEST_STEP_MS * 2);
        check(std::strstr(f.rows[1], "GYRO") != nullptr, "TC28: stays parked on GYRO, never advances");
    }
    {
        Harness h;
        h.in.boot.lineArray = false;
        enterSelfTest(h);
        passSimpleCheck(h);
        beginCheck(h);
        passSimpleCheck(h);
        passVisionOk(h);
        Console::ConsoleFrame f = beginCheck(h);
        check(f.action == Console::ActionRequest::NONE,
              "TC28: no READ_LINE_BOARDS requested when boot.lineArray==false");
        check(h.core.wantsLineData() == false, "TC28: wantsLineData() stays false when boot.lineArray==false");
        check(std::strstr(f.rows[2], "LINE FAIL") != nullptr,
              "TC28: LINE check self-fails straight from the boot report");
    }

    // Pin BootReport's 13 boot-health flags against _renderBootFailures's hand-maintained
    // name table (OperatorConsole.cpp has no compile-time tie between the two): each flag,
    // failed alone, must surface under its own short name, and an all-clean report must
    // read "ALL MODULES OK".
    //
    // WHAT THIS COVERS: all 13 name<->flag bindings, individually. A renderer that drops a
    // name, mislabels one, or reads the wrong struct member fails here.
    //
    // WHAT IT DOES NOT COVER: a 14th flag. The FlagCase table below is hand-maintained, so a
    // flag added to BootReport without teaching _renderBootFailures about it compiles and
    // passes -- neither table learns of it. A `sizeof(BootReport)` tripwire does not help:
    // 13-, 14-, 15- and 16-bool + float structs all measure 20 bytes, so it cannot catch the
    // one thing it would be claimed to guard, while it would false-fail on a harmless member
    // re-order or type change.
    //
    // THE REAL FIX (follow-up, deliberately not done here -- it touches ConsoleTypes.h and
    // OperatorConsole.cpp, which belong to other tasks): a `Console::BOOT_FLAG_COUNT`
    // constant in ConsoleTypes.h that the renderer uses to size its name/value arrays and
    // that this test asserts its own case count against. Then a 14th flag breaks the build
    // in exactly one place, on purpose.
    {
        struct FlagCase
        {
            const char* shortName;
            bool Console::BootReport::*field;
        };
        static const FlagCase flags[] = {
            {"BATT", &Console::BootReport::batt},         {"LINE", &Console::BootReport::lineArray},
            {"YAW", &Console::BootReport::yaw},           {"DRV", &Console::BootReport::drive},
            {"LF", &Console::BootReport::lineFollower},   {"INT", &Console::BootReport::intersection},
            {"MAN", &Console::BootReport::manipulator},   {"LOC", &Console::BootReport::localization},
            {"PT", &Console::BootReport::pathTracker},    {"MIS", &Console::BootReport::mission},
            {"TSK", &Console::BootReport::tasks},         {"VIS", &Console::BootReport::vision},
            {"TRC", &Console::BootReport::trace},
        };
        check(sizeof(flags) / sizeof(flags[0]) == 13, "BootReport flag-count pin: 13 named flags enumerated");

        for (const auto& fc : flags)
        {
            Harness h;
            h.in.boot           = allGoodBoot();
            h.in.boot.*fc.field = false;
            h.open();
            for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::BATT); ++i)
            {
                tapDown(h);
            }
            Console::ConsoleFrame f = confirmCursor(h);
            check(std::strstr(f.rows[2], fc.shortName) != nullptr ||
                      std::strstr(f.rows[3], fc.shortName) != nullptr,
                  msgf("BootReport flag pin: failing flag \"%s\" must surface on the BATT screen "
                       "-- got rows[2]=\"%s\" rows[3]=\"%s\"",
                       fc.shortName, f.rows[2], f.rows[3]));
        }

        Harness h;
        h.open();
        for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::BATT); ++i)
        {
            tapDown(h);
        }
        Console::ConsoleFrame f = confirmCursor(h);
        check(std::strstr(f.rows[2], "ALL MODULES OK") != nullptr,
              "BootReport flag-count pin: an all-clean report reads ALL MODULES OK");
    }

    // TC29 -- VISION TEST grades the mosaic against the expected colour code.
    // Expected code (kExpectedMosaic): {4,3,1,2,3,4,2,2,1,1,4,3} -> letters WGYBGWBBYYWG.
    {
        static const uint8_t kExpect[12] = {4, 3, 1, 2, 3, 4, 2, 2, 1, 1, 4, 3};

        // Navigate to VISION TEST and confirm -> enters VISION_TEST, one-shot READ_MOSAIC in flight.
        auto enterVisionTest = [](Harness& h) {
            h.open();
            for (uint8_t i = 0; i < static_cast<uint8_t>(Console::Item::VISION_TEST); ++i)
            {
                tapDown(h);
            }
            Console::ConsoleFrame f = confirmCursor(h);
            check(h.core.state() == Console::State::VISION_TEST, "TC29: entered VISION_TEST");
            check(f.action == Console::ActionRequest::READ_MOSAIC,
                  "TC29: VISION_TEST emits a one-shot READ_MOSAIC on entry");
        };

        // PASS: an exact match -> Signal::PASS, "all 12 match", PASS tag on the title row.
        {
            Harness h;
            enterVisionTest(h);
            for (uint8_t c = 0; c < 12; ++c)
            {
                h.in.sample.visionCell[c] = kExpect[c];
            }
            h.in.sample.visionFaultKind = VisionFault::NONE;
            Console::ConsoleFrame f = completeRead(h, Console::ActionRequest::READ_MOSAIC);
            check(f.signal == Console::Signal::PASS, "TC29: exact match raises Signal::PASS");
            check(std::strstr(f.rows[0], "PASS") != nullptr, "TC29: title row shows PASS");
            check(std::strcmp(f.rows[1], "GOT WGYBGWBBYYWG") == 0, "TC29: GOT row (match)");
            check(std::strcmp(f.rows[2], "EXP WGYBGWBBYYWG") == 0, "TC29: EXP row");
            check(std::strcmp(f.rows[3], "all 12 match") == 0, "TC29: verdict all match");
        }

        // FAIL: cell 4 reads UNKNOWN instead of GREEN -> Signal::FAIL + first-mismatch detail.
        {
            Harness h;
            enterVisionTest(h);
            for (uint8_t c = 0; c < 12; ++c)
            {
                h.in.sample.visionCell[c] = kExpect[c];
            }
            h.in.sample.visionCell[4]   = 0;    // UNKNOWN where GREEN (3) was expected
            h.in.sample.visionFaultKind = VisionFault::NONE;
            Console::ConsoleFrame f = completeRead(h, Console::ActionRequest::READ_MOSAIC);
            check(f.signal == Console::Signal::FAIL, "TC29: a mismatch raises Signal::FAIL");
            check(std::strstr(f.rows[0], "FAIL") != nullptr, "TC29: title row shows FAIL");
            check(std::strcmp(f.rows[1], "GOT WGYB?WBBYYWG") == 0, "TC29: GOT row (mismatch)");
            check(std::strcmp(f.rows[3], "1 bad c4 G->?") == 0,
                  "TC29: verdict names the first bad cell (expected G, got ?)");
        }

        // Transport fault -> graded FAIL with a READ FAULT line, never a spurious pass.
        {
            Harness h;
            enterVisionTest(h);
            h.in.sample.visionFaultKind = VisionFault::READ_TIMEOUT;
            Console::ConsoleFrame f = completeRead(h, Console::ActionRequest::READ_MOSAIC);
            check(f.signal == Console::Signal::FAIL, "TC29: a read fault grades FAIL");
            check(std::strcmp(f.rows[3], "READ FAULT") == 0, "TC29: read fault shows READ FAULT");
        }
    }

    if (g_failures == 0)
    {
        std::printf("OPERATOR CONSOLE OK\n");
        return 0;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return 1;
}
