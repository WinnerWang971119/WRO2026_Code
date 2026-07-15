/**
 * @file Tasks.cpp
 * @brief Implementation of the Tasks framework registry + dispatch.
 * @author WRO 2026
 */

#include "Tasks.h"
#include "src/manip/Manipulator.h"
#include "src/mission/PathTracker.h"
#include "src/drive/Drive.h"
#include "src/drive/LineFollower.h"
#include "src/perception/Vision.h"

#include <Arduino.h>
#ifdef CTPS_RECORD
#include <time.h>
#endif
#include <string.h>    // strcmp

#include "Config.h"

/**
 * @brief Single global instance, declared `extern` in Tasks.h.
 */
TasksClass Tasks;

// =============================================================================
// Static registry — single source of truth for valid task names.
//
// Add new tasks here. The array must stay sorted by neither name nor address;
// `findTask` is an O(N) strcmp scan and registry size is hard-capped at
// `Config::MAX_TASKS` by the static_assert below.
// =============================================================================

const TaskRegistryEntry TasksClass::_registry[] =
{
    { "demoEcho",   &TasksClass::demoEcho   },
    { "servoTest", &TasksClass::servoTest },
    { "debugYield", &TasksClass::debugYield },
    { "idleWait", &TasksClass::idleWait },
    { "testOne", &TasksClass::testOne },
    { "ctpsExample1", &TasksClass::ctpsExample1 },
    { "emptyTest", &TasksClass::emptyTest },
    { "manipDirectTest", &TasksClass::manipDirectTest },
    { "beltTest", &TasksClass::beltTest },
    { "readMosaic", &TasksClass::readMosaic },
    //Add new tasks here
};

const size_t TasksClass::_registry_count =
    sizeof(_registry) / sizeof(_registry[0]);

static_assert(TasksClass::_registry_count <= Config::MAX_TASKS,
              "Registry exceeds MAX_TASKS");

// =============================================================================
// RAII scope guard for the `_inTask` flag.
//
// Exceptions are off on this target so a try/finally is not available; the
// idiomatic replacement is a stack-local object whose destructor clears the
// flag. This keeps `_inTask` correct even if `runTask`'s body acquires more
// early-return paths in the future.
// =============================================================================

namespace
{
    struct InTaskGuard
    {
        bool* flag;
        explicit InTaskGuard(bool* f) : flag(f) { *flag = true; }
        ~InTaskGuard() { *flag = false; }
    };
}    // namespace

bool TasksClass::begin()
{
    _inTask  = false;
    _aborted = false;
    return true;
}

void TasksClass::update()
{
    // Intentional no-op. Tasks has no persistent active-task FSM in this
    // plan; bodies run to completion inside `runTask`. Kept non-trivial as
    // a stable hook for the cooperative tick list — see Tasks.h class
    // docstring.
}

void TasksClass::setTickCallback(void (*cb)())
{
    _tickCallback = cb;
}

const TaskRegistryEntry* TasksClass::findTask(const char* name) const
{
    if (name == nullptr)
    {
        return nullptr;
    }
    for (size_t i = 0; i < _registry_count; i++)
    {
        // Skip rows with a null name pointer (defensive — a hand-edit typo
        // like `{ nullptr, &TasksClass::foo }` would otherwise feed nullptr
        // into strcmp and yield UB).
        if (_registry[i].name == nullptr)
        {
            continue;
        }
        if (strcmp(_registry[i].name, name) == 0)
        {
            return &_registry[i];
        }
    }
    return nullptr;
}

bool TasksClass::runTask(const char* name, const TaskArgs& args)
{
    // Reentrancy guard. Tasks are leaf operations: a task body must NOT
    // dispatch another task. Allowing it would (a) clobber `_inTask` when the
    // inner InTaskGuard's destructor fires, breaking the `Mission.update()`
    // re-entrancy guard that relies on `inTask()` staying true for the whole
    // outer body, and (b) recurse with no depth bound (a task calling itself
    // would blow the stack). Refuse here, before resolving the name or arming
    // the guard. Share logic between tasks via private helpers, not runTask.
    // See ADR-026.
    if (_inTask)
    {
        Serial.print(F("[Tasks] runTask: refusing reentrant call to '"));
        Serial.print(name == nullptr ? "(null)" : name);
        Serial.println(F("'"));
        return false;
    }

    const TaskRegistryEntry* entry = findTask(name);
    if (entry == nullptr)
    {
        Serial.print(F("[Tasks] runTask: unknown name '"));
        Serial.print(name == nullptr ? "(null)" : name);
        Serial.println(F("'"));
        return false;
    }

    // Defensive null-check on the member-function pointer: a registry edit
    // that lands a placeholder like `{ "name", nullptr }` would otherwise
    // hard-fault on the pmf call below. The static_assert on count alone
    // does not catch this.
    if (entry->fn == nullptr)
    {
        Serial.print(F("[Tasks] runTask: null fn for '"));
        Serial.print(name);
        Serial.println(F("'"));
        return false;
    }

    // Scope guard sets `_inTask = true` now and clears it on return through
    // any path (early return, normal return, future-added returns alike).
    InTaskGuard guard(&_inTask);

    const bool ok = (this->*(entry->fn))(args);
    return ok;
}

void TasksClass::abort()
{
    // Sticky latch. Intentionally does NOT touch `_inTask` — that flag is
    // owned exclusively by `runTask`'s InTaskGuard so the in-flight body
    // (if any) still unwinds cleanly with `_inTask` correctly cleared by
    // the guard's destructor.
    _aborted = true;
}

void TasksClass::clearAbort()
{
    // Per-run reset of the abort latch, called from Mission.start() so an
    // abort latched during run N does not bleed into run N+1. Intentionally
    // does NOT touch `_inTask` — runTask's guard owns that.
    _aborted = false;
}

bool TasksClass::isAborted() const
{
    // Staged hook — the latch is set by abort() and cleared by begin()/clearAbort(),
    // but NO production code polls this yet. The actual stop signal is delivered
    // by PathTracker.abort() + Drive.stop(true); those fire synchronously while
    // this flag merely records that an abort was requested.
    //
    // Intended first consumer: a future cooperative task body that should call
    // Tasks.isAborted() at the top of its work loop and bail (return false) so
    // long-running tasks honour mission aborts without busy-waiting. Until then
    // this reader is exercised only by the host-test mirror (sim/).
    return _aborted;
}

bool TasksClass::inTask() const
{
    return _inTask;
}

bool TasksClass::_waitStationaryMs(uint32_t ms, const __FlashStringHelper* site)
{
    if (!Drive.isIdle() || !PathTracker.isIdle() || LineFollower.isEnabled())
    {
        Serial.print(F("[Tasks] "));
        Serial.print(site);
        Serial.println(F(": refusing idle wait while motion is active"));
        return false;
    }

    const uint32_t startMs = millis();
    static bool    noCallbackWarned = false;

    while ((millis() - startMs) < ms)
    {
        if (_aborted)
        {
            Serial.print(F("[Tasks] "));
            Serial.print(site);
            Serial.println(F(": aborted during idle wait"));
            return false;
        }

        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!noCallbackWarned)
            {
                Serial.print(F("[Tasks] "));
                Serial.print(site);
                Serial.println(F(": no tick callback set — only wall-clock time will "
                                 "advance during the wait. Call Tasks.setTickCallback(...) "
                                 "in setup()."));
                noCallbackWarned = true;
            }
            delay(1);
        }
        yield();
    }

    return true;
}

// =============================================================================
// Task bodies
// =============================================================================

bool TasksClass::demoEcho(const TaskArgs& args)
{
    Serial.print(F("[Tasks] demoEcho a="));
    Serial.print(args.u8[0]);
    Serial.print(F(" b="));
    Serial.println(args.u8[1]);
    return true;
}

bool TasksClass::servoTest(const TaskArgs& pos)
{
    Manipulator.sorter.setAngle(static_cast<uint16_t>(pos.u8[0]));

    // Wait until done. Drive the registered cooperative-tick callback each iteration so every
    // module keeps ticking during the block (the same pattern as Manipulator::_drain); fall
    // back to ticking only Manipulator.update() with a one-shot warning if no callback is set.
    // yield() services the R4 background; we never block with a fixed delay().
    static bool noCallbackWarned = false;
    while (!Manipulator.sorter.isDone())
    {
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!noCallbackWarned)
            {
                Serial.println(F("[Tasks] servoTest: no tick callback set — only "
                                 "Manipulator.update() will tick during the wait. Call "
                                 "Tasks.setTickCallback(...) in setup()."));
                noCallbackWarned = true;
            }
            Manipulator.update();
        }
        yield();
    }
    return true;
}

bool TasksClass::testOne(const TaskArgs& /*args*/)
{
    //PathTracker.goTo("i1");
    LineFollower.setEnabled(true).setReverse(false).setDistanceTarget(50);
    LineFollower.waitUntilDistanceReached();
    LineFollower.setEnabled(false);
    return true;
}

bool TasksClass::debugYield(const TaskArgs& /*args*/)
{
    // Production-side stub for the YIELD path so Mission's TASK case has
    // an end-to-end test target. The host-mirror's `alwaysYield` stub used
    // to be the only path that exercised `_advanceStep` + `TASK YIELD %s`
    // emission; this body lets a real Mission/Tasks integration check the
    // same branch.
    Serial.println(F("[Tasks] debugYield"));
    return false;
}

bool TasksClass::idleWait(const TaskArgs& args)
{
    // Cooperative idle-hold for `args.u32[0]` ms (chassis must be stationary):
    // pumps the tick callback while the mock/real clock advances. Also the
    // end-to-end target for the "refusing idle wait while motion is active" guard.
    return _waitStationaryMs(args.u32[0], F("idleWait"));
}

// =============================================================================
// Cooperative-wait helpers.
//
// Shared by the bring-up tasks below (emptyTest / actuatorSweep / actuatorStep /
// manipDirectTest). Each keeps every module ticking while a task blocks on an
// actuator, driving the registered cooperative-tick callback per iteration and
// falling back to Drive+Manipulator update() (with a one-shot warning) when no
// callback is set. yield() services the R4 background; we never block on a fixed
// delay(). See ADR-025/026 and Manipulator::_drain.
// =============================================================================

void TasksClass::_pumpTick()
{
    if (_tickCallback != nullptr)
    {
        _tickCallback();
    }
    else
    {
        if (!_noCallbackWarned)
        {
            Serial.println(F("[Tasks] no tick callback set — only Drive/Manipulator "
                             "will tick during waits. Call Tasks.setTickCallback(...) "
                             "in setup()."));
            _noCallbackWarned = true;
        }
        Drive.update();
        Manipulator.update();
    }
    yield();
}

void TasksClass::_waitMs(uint32_t durationMs)
{
    const uint32_t start = millis();
    while ((uint32_t)(millis() - start) < durationMs)
    {
        _pumpTick();
    }
}

bool TasksClass::_waitForServo(ManipServo& servo, uint32_t timeoutMs, const char* label)
{
    const uint32_t start = millis();
    while (!servo.isDone())
    {
        if ((uint32_t)(millis() - start) >= timeoutMs)
        {
            Serial.print(F("[Tasks] servo timeout at "));
            Serial.println(label);
            return false;
        }
        _pumpTick();
    }
    return true;
}

bool TasksClass::_waitForBelt(uint32_t timeoutMs, const char* label)
{
    const uint32_t start = millis();
    while (!Manipulator.belt.isDone())
    {
        if ((uint32_t)(millis() - start) >= timeoutMs)
        {
            Serial.print(F("[Tasks] belt timeout at "));
            Serial.println(label);
            return false;
        }
        _pumpTick();
    }
    return true;
}

bool TasksClass::_grabToBin(uint8_t bin)
{
    Manipulator.sortTo(bin).async();
    Manipulator.release();
    Manipulator.grab();
    return true;
}

bool TasksClass::_visionRead()
{
    if (Vision.readMosaic())
    {
        return true;
    }

    // First attempt faulted. readMosaic() left the latch untouched (ADR-037); clear the
    // fault and give it one more try before reporting failure up to the task.
    Serial.print(F("[Tasks] vision read fault ("));
    Serial.print((uint8_t) Vision.getFault().kind);
    Serial.println(F("), retrying"));
    Vision.clearFault();

    if (Vision.readMosaic())
    {
        return true;
    }

    Serial.println(F("[Tasks] vision read failed"));
    return false;
}

bool TasksClass::emptyTest(const TaskArgs& /*args*/)
{
    Serial.println(F("[Tasks] emptyTest"));

    Manipulator.dumper.open();
    (void)_waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper open");
    _waitMs(200);

    for (uint8_t bin = 0; bin < 4; ++bin)
    {
        Manipulator.sortTo(bin);
        _waitMs(200);

        Manipulator.grabber.open();
        (void)_waitForServo(Manipulator.grabber,
                            Config::MANIP_SERVO_DRAIN_TIMEOUT_MS,
                            "grabber open");
        _waitMs(3000);

        Manipulator.grabber.home();
        (void)_waitForServo(Manipulator.grabber,
                            Config::MANIP_SERVO_DRAIN_TIMEOUT_MS,
                            "grabber home");
        _waitMs(1000);

        Manipulator.grabber.close();
        (void)_waitForServo(Manipulator.grabber,
                            Config::MANIP_SERVO_DRAIN_TIMEOUT_MS,
                            "grabber close");
        _waitMs(2000);
    }

    Manipulator.dumper.close();
    (void)_waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper close");
    _waitMs(500);

    Manipulator.dumper.home();
    (void)_waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper home");

    return true;
}

bool TasksClass::beltTest(const TaskArgs& /*args*/)
{
    Manipulator.belt.home();

    return true;
}

bool TasksClass::manipDirectTest(const TaskArgs& /*args*/)
{
    static constexpr uint32_t TEST_SETTLE_MS = 250;

    Serial.println(F("[Tasks] manipDirectTest: start"));

    Drive.stop(true);

    Serial.println(F("[Tasks] manipDirectTest: grabber home/open/close/home"));
    Manipulator.grabber.home();
    if (!_waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber home 1"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.open();
    if (!_waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber open"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.close();
    if (!_waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber close"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.home();
    if (!_waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber home 2"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] manipDirectTest: sorter bins"));
    Manipulator.sorter.home();
    if (!_waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter home 1"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    for (uint8_t bin = 0; bin < Config::SORTER_BIN_COUNT; ++bin)
    {
        Manipulator.sorter.setAngle(Config::SORTER_BIN_ANGLE[bin]);
        if (!_waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter bin"))
        {
            Drive.stop(true);
            return false;
        }
        _waitMs(TEST_SETTLE_MS);
    }

    Manipulator.sorter.home();
    if (!_waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter home 2"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] manipDirectTest: dumper home/open/close/home"));
    Manipulator.dumper.home();
    if (!_waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper home 1"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.open();
    if (!_waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper open"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.close();
    if (!_waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper close"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.home();
    if (!_waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper home 2"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] manipDirectTest: belt left/mid/far_end/mid"));
    Manipulator.belt.toStation(Config::BeltStation::LEFT);
    if (!_waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt left"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::MID);
    if (!_waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt mid 1"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::FAR_END);
    if (!_waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt far_end"))
    {
        Drive.stop(true);
        return false;
    }
    _waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::MID);
    if (!_waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt mid 2"))
    {
        Drive.stop(true);
        return false;
    }

    Drive.stop(true);
    Serial.println(F("[Tasks] manipDirectTest: done"));
    return true;
}

bool TasksClass::ctpsExample1(const TaskArgs& /*args*/)
{
    constexpr uint8_t kMosaicPickCount = 12;
    constexpr uint8_t kMosaicVisitCount = 6;
    constexpr uint8_t kMosaicPoolCount  = 8;
    static_assert((kMosaicPickCount % Config::SORTER_BIN_COUNT) == 0,
                  "ctpsExample1 expects the mosaic pick count to divide evenly across sorter bins");
    constexpr uint8_t kLaneDepth = kMosaicPickCount / Config::SORTER_BIN_COUNT;
    static_assert((kMosaicPickCount % kMosaicVisitCount) == 0,
                  "ctpsExample1 expects an even number of picks per mosaic stop");
    constexpr uint8_t kPicksPerStop = kMosaicPickCount / kMosaicVisitCount;

    uint8_t laneOrder[kMosaicPickCount];
    uint8_t laneWrite = 0;
    for (uint8_t lane = 0; lane < Config::SORTER_BIN_COUNT; ++lane)
    {
        for (uint8_t depth = 0; depth < kLaneDepth; ++depth)
        {
            laneOrder[laneWrite++] = lane;
        }
    }

    uint32_t rngState = ((uint32_t) millis() << 16) ^ 0x9E3779B9u;
#ifdef CTPS_RECORD
    rngState ^= (uint32_t)time(nullptr);
    rngState ^= (uint32_t)clock();
    rngState ^= (uint32_t)(uintptr_t)&rngState;
#endif
    auto nextRand = [&rngState]() -> uint32_t
    {
        // Small xorshift32 shuffle source: cheap on-device and dependency-free in host CTPS.
        rngState ^= (rngState << 13);
        rngState ^= (rngState >> 17);
        rngState ^= (rngState << 5);
        return rngState;
    };

    for (int8_t i = (int8_t)kMosaicPickCount - 1; i > 0; --i)
    {
        const uint8_t j = (uint8_t)(nextRand() % (uint32_t)(i + 1));
        const uint8_t t = laneOrder[(uint8_t)i];
        laneOrder[(uint8_t)i] = laneOrder[j];
        laneOrder[j]          = t;
    }

    struct MosaicStop
    {
        const char* name;
        Direction   arrivalDirection;
        bool        forceArrivalDirection;
    };

    const MosaicStop mosaicStops[kMosaicPoolCount] = {
        { "MYU", Direction::FRONT, false },
        { "MYD", Direction::FRONT, false },
        { "MBU", Direction::FRONT, false },
        { "MBD", Direction::FRONT, false },
        { "MGU", Direction::FRONT, false },
        { "MGD", Direction::FRONT, false },
        { "MWU", Direction::FRONT, false },
        { "MWD", Direction::FRONT, false },
    };

    uint8_t mosaicOrder[kMosaicPoolCount] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    for (int8_t i = (int8_t)kMosaicPoolCount - 1; i > 0; --i)
    {
        const uint8_t j = (uint8_t)(nextRand() % (uint32_t)(i + 1));
        const uint8_t t = mosaicOrder[(uint8_t)i];
        mosaicOrder[(uint8_t)i] = mosaicOrder[j];
        mosaicOrder[j]          = t;
    }

    uint8_t laneCursor = 0;
    auto grabToNextLane = [&laneOrder, &laneCursor]()
    {
        Manipulator.sortTo(laneOrder[laneCursor++]);
        Manipulator.grab();
    };

    PathTracker.goTo("MYD").setArrivalDirection(Direction::FRONT);
    Drive.turnToYaw(90);

    for (uint8_t stopIndex = 0; stopIndex < kMosaicVisitCount; ++stopIndex)
    {
        const MosaicStop& stop = mosaicStops[mosaicOrder[stopIndex]];

        Serial.print(F("[Tasks] ctpsExample1 mosaic -> "));
        Serial.println(stop.name);

        if (stop.forceArrivalDirection)
        {
            PathTracker.goTo(stop.name).setArrivalDirection(stop.arrivalDirection);
        }
        else
        {
            PathTracker.goTo(stop.name);
        }

        Drive.turnToYaw(90);
        for (uint8_t pick = 0; pick < kPicksPerStop; ++pick)
        {
            grabToNextLane();
        }
    }

    //Grab shovel
    PathTracker.goTo("shovel1O");
    Manipulator.boxClose();

    //Go to shovel drop
    PathTracker.goTo("shovel1F");
    Manipulator.boxOpen();

    //Go to dump
    PathTracker.goTo("dump1").setArrivalDirection(Direction::FRONT);
    Manipulator.dump();
    PathTracker.goTo("dump2").setArrivalDirection(Direction::REAR);
    Manipulator.dump();
    PathTracker.goTo("dump3").setArrivalDirection(Direction::REAR);

    Manipulator.dump();

    PathTracker.goTo("CG1").setArrivalDirection(Direction::REAR);
    Manipulator.boxClose();
    PathTracker.goTo("CG2");
    Drive.turnToYaw(0);
    PathTracker.goTo("CG3");
    Manipulator.boxOpen();
    PathTracker.goTo("CY1").setArrivalDirection(Direction::REAR);
    Manipulator.boxClose();
    PathTracker.goTo("CY2");
    Drive.turnToYaw(90);
    PathTracker.goTo("CY3").setArrivalDirection(Direction::REAR);
    Manipulator.boxOpen();

    return true;
}

bool TasksClass::readMosaic(const TaskArgs& /*args*/)
{
    PathTracker.goTo("CY3").setArrivalDirection(Direction::FRONT);
    return _visionRead();
}
