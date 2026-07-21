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
#include "src/nav/Localization.h"

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
    { "rectangleShovel", &TasksClass::rectangleShovel },
    { "triangleShovel", &TasksClass::triangleShovel },
    { "pot", &TasksClass::pot },
    { "mosaic", &TasksClass::mosaic },
    { "debries", &TasksClass::debries },
    { "dumpMosaic", &TasksClass::dumpMosaic },
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
    //Manipulator.beltTo(Config::BeltStation::REAR_BOX_CLOSE);
    //Manipulator.beltTo(Config::BeltStation::GRAB_RIGHT);
    //_grabToBin(0);


    /*PathTracker.goTo("MR").setArrivalDirection(Direction::FRONT);
    Drive.turnToYaw(90);
    PathTracker.goTo("MYD").setArrivalDirection(Direction::FRONT);
    Manipulator.dumper.open();
    Manipulator.beltTo(Config::BeltStation::GRAB_MID);
    _grabToBin(0);
    Manipulator.beltTo(Config::BeltStation::GRAB_RIGHT);
    _grabToBin(1);
    PathTracker.goTo("MYU");
    Manipulator.beltTo(Config::BeltStation::GRAB_MID);
    _grabToBin(2);
    Manipulator.beltTo(Config::BeltStation::GRAB_RIGHT);
    _grabToBin(3);

    PathTracker.goTo("MBD");
    Manipulator.beltTo(Config::BeltStation::GRAB_MID);
    _grabToBin(0);
    Manipulator.beltTo(Config::BeltStation::GRAB_RIGHT);
    _grabToBin(1);
    PathTracker.goTo("MBU");
    Manipulator.beltTo(Config::BeltStation::GRAB_MID);
    _grabToBin(2);
    Manipulator.beltTo(Config::BeltStation::GRAB_RIGHT);
    _grabToBin(3);
    Manipulator.beltTo(Config::BeltStation::MID);

    PathTracker.goTo("i3");
    Drive.turnToYaw(90);

    PathTracker.goTo("dump1").setArrivalDirection(Direction::FRONT);
    Manipulator.dump();
    _waitMs(500);
    Manipulator.dumpReset();
    PathTracker.goTo("dump2");*/

    Drive.turnToYaw(Localization.toImuYaw(180));

    

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
    _waitMs(1000);
    Manipulator.belt.toStation(Config::BeltStation::LEFT);
    _waitMs(1000);
    Manipulator.belt.toStation(Config::BeltStation::MID);
    _waitMs(1000);
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


// =============================================================================
// Mosaic model — shared by `mosaic` and `_mosaicAlgo`.
//
// The pattern read at CY3 is a 4-column x 3-row grid; cell index = row * kMosaicCols
// + col, matching `Vision.getColor()`'s cell numbering.
//
// The carrier mirrors that grid: one sorter bin per COLUMN, each bin `kMosaicRows`
// tiles deep. Bin `c` must be loaded row 0 first, then row 1, then row 2, so the
// dumped mosaic comes out identical to the pattern that was read. That means bin `c`
// can only accept ONE colour at any moment — the colour of cell `depth[c] * 4 + c`.
//
// The four fill depths are the entire planning state, so they pack into a single byte
// (2 bits per bin). Stopping at a colour station and loading every tile it can is
// always optimal — the remaining requirement is the same whether a tile is taken now
// or later, so taking it while already parked can only remove a future revisit. The
// only real decision left is the ORDER of station visits, which `_mosaicAlgo` solves
// exactly; see its docblock.
// =============================================================================

// `WHITE` is a macro (`SSD1306_WHITE`) in every TU that includes <MatrixMiniR4.h>,
// which Manipulator.h does. Hide it while naming `Config::Color::WHITE`, then restore
// it so OLED code elsewhere still compiles — same idiom as ConfigVision.h.
#pragma push_macro("WHITE")
#undef WHITE

namespace
{
    constexpr uint8_t kMosaicCols       = 4;
    constexpr uint8_t kMosaicRows       = 3;
    constexpr uint8_t kMosaicColorCount = 4;   // YELLOW / BLUE / GREEN / WHITE

    static_assert(kMosaicCols * kMosaicRows == Config::VISION_CELL_COUNT,
                  "Mosaic grid must cover exactly the vision cells");
    static_assert(kMosaicCols == Config::SORTER_BIN_COUNT,
                  "One sorter bin per mosaic column — column index IS the bin index");
    static_assert(kMosaicRows == 3 && kMosaicCols == 4,
                  "Fill depths are packed 2 bits per bin into one byte");

    /** @brief Colours the wall stations supply, in station index order. */
    constexpr Config::Color kMosaicColors[kMosaicColorCount] = {
        Config::Color::YELLOW,
        Config::Color::BLUE,
        Config::Color::GREEN,
        Config::Color::WHITE,
    };

    /**
     * @brief Single-letter tag per station index, for logging a route.
     * @note Strings, not `char` — `Serial.print(char)` has no overload on the host mock and
     *       silently binds to `print(uint8_t)`, printing the character CODE instead.
     */
    constexpr const char* kMosaicColorTag[kMosaicColorCount] = { "Y", "B", "G", "W" };

    // -------------------------------------------------------------------------
    // Wall supply model
    //
    // Each colour occupies two rows of the M wall — a lower node (`MxD`) and an
    // upper node (`MxU`) — and each row holds `kMosaicTilesPerRow` tiles that the
    // belt reaches without the chassis moving. So one colour supplies
    // 2 * kMosaicTilesPerRow tiles in total.
    // -------------------------------------------------------------------------

    constexpr uint8_t kMosaicTilesPerRow = 3;
    constexpr uint8_t kMosaicRowLower    = 0;
    constexpr uint8_t kMosaicRowUpper    = 1;

    /**
     * @brief One row node of the M wall.
     *
     * `chainPos` is the node's place along the wall from the top (`MWU` = 0) down to the
     * bottom (`MYD` = 7), matching the blind-edge chain in `field_data.json`. It lets a
     * stop enter a colour at whichever of its two rows is nearer to where the robot
     * currently sits, instead of always taking the same one.
     */
    struct MosaicRowNode
    {
        const char* node;
        uint8_t     chainPos;
    };

    /** @brief Wall row nodes, indexed `[colour index][kMosaicRowLower | kMosaicRowUpper]`. */
    constexpr MosaicRowNode kMosaicRowNodes[kMosaicColorCount][2] = {
        { { "MYD", 7 }, { "MYU", 6 } },   // YELLOW
        { { "MBD", 5 }, { "MBU", 4 } },   // BLUE
        { { "MGD", 3 }, { "MGU", 2 } },   // GREEN
        { { "MWD", 1 }, { "MWU", 0 } },   // WHITE
    };

    /**
     * @brief Belt positions reaching the three tiles of one wall row, in take order.
     *
     * `GRAB_MID` comes first because the belt is parked there after every drive (it is the
     * travel pose — see `Config::BeltStation`), so the first tile of a row costs no belt move.
     */
    constexpr Config::BeltStation kMosaicGrabOrder[kMosaicTilesPerRow] = {
        Config::BeltStation::GRAB_MID,
        Config::BeltStation::GRAB_LEFT,
        Config::BeltStation::GRAB_RIGHT,
    };

    /**
     * @brief Sorter bin that receives the tile for mosaic column @p column.
     *
     * The carrier faces the mosaic as it dumps, so its bins run opposite to the pattern's
     * columns — pattern column 0 lands in the LAST bin. Mirroring here keeps the planner
     * working in pattern coordinates and confines the flip to one place.
     */
    uint8_t mosaicBinForColumn(uint8_t column)
    {
        return (uint8_t)(kMosaicCols - 1 - column);
    }

    /**
     * @brief Station index of @p color.
     * @return Index in `[0, kMosaicColorCount)`, or `0xFF` for a colour with no station
     *         (`UNKNOWN` / `RED`).
     */
    uint8_t mosaicColorIndex(Config::Color color)
    {
        for (uint8_t i = 0; i < kMosaicColorCount; ++i)
        {
            if (kMosaicColors[i] == color)
            {
                return i;
            }
        }
        return 0xFF;
    }

    /** @brief Absolute distance between two wall chain positions. */
    uint8_t mosaicChainGap(uint8_t a, uint8_t b)
    {
        return (uint8_t)((a > b) ? (a - b) : (b - a));
    }

    // -------------------------------------------------------------------------
    // Planning state — four bin fill depths packed 2 bits each into one byte.
    // -------------------------------------------------------------------------

    /** @brief Every bin filled to `kMosaicRows` — the mosaic is complete. */
    constexpr uint8_t  kMosaicStateDone  = 0xFF;
    /** @brief Number of distinct packed states (one byte). */
    constexpr uint16_t kMosaicStateCount = 256;
    /** @brief Cost sentinel for "the mosaic cannot be completed from here". */
    constexpr uint8_t  kMosaicCostInf    = 254;

    /**
     * @brief Wall-chain position of each station, in HALF-node units.
     *
     * A station spans two row nodes, so its centre falls between them; doubling the chain
     * position keeps that centre an integer. `MYD` = 7 and `MYU` = 6 give YELLOW 13, and so
     * on up the wall. Used only as the planner's travel cost — the executor picks the actual
     * row node.
     */
    constexpr uint8_t kMosaicStationHalf[kMosaicColorCount] = { 13, 9, 5, 1 };   // Y B G W

    /**
     * @brief Reset node the robot enters the wall through.
     *
     * A direct drive from the field to a wall row node cuts across the mosaic. `MR` sits
     * clear of it at y = 20 and is a `NodeType::RESET` node, so simply arriving there makes
     * PathTracker square up, press the rear into the wall and re-anchor `Localization`
     * (`_maybeEnterReset`) — no explicit reset call needed.
     */
    constexpr const char* kMosaicResetNode = "MR";

    /**
     * @brief Wall-chain position the robot starts from, having come in through `MR`.
     *
     * `MR` edges to `MYD` (the bottom of the chain), so the FIRST station's route cost is
     * measured from there rather than from an arbitrary end.
     */
    constexpr uint8_t kMosaicEntryChainPos = 7;   // MYD — the node MR opens onto

    /** @brief Fill depth of bin @p bin in packed @p state. */
    uint8_t mosaicDepth(uint8_t state, uint8_t bin)
    {
        return (uint8_t)((state >> (2 * bin)) & 0x03);
    }

    /** @brief Copy of @p state with bin @p bin set to fill depth @p depth. */
    uint8_t mosaicWithDepth(uint8_t state, uint8_t bin, uint8_t depth)
    {
        const uint8_t shift = (uint8_t)(2 * bin);
        return (uint8_t)((state & (uint8_t) ~(0x03u << shift)) | (uint8_t)(depth << shift));
    }

    /** @brief Total tiles loaded in @p state, i.e. the sum of the four fill depths. */
    uint8_t mosaicLoaded(uint8_t state)
    {
        uint8_t loaded = 0;
        for (uint8_t bin = 0; bin < kMosaicCols; ++bin)
        {
            loaded = (uint8_t)(loaded + mosaicDepth(state, bin));
        }
        return loaded;
    }

    /**
     * @brief State after stopping at station @p station and loading every tile it can.
     *
     * Each bin takes tiles while its next required colour is this station's. Loading one
     * exposes the row beneath, which may need the same colour again, so each bin advances in
     * a loop. Returns @p state unchanged when the station has nothing this carrier can accept.
     *
     * @param state     Packed fill depths before the stop.
     * @param station   Station index in `[0, kMosaicColorCount)`.
     * @param cellIndex Per-cell station index of the read pattern (`kMosaicCellCount` entries).
     */
    uint8_t mosaicAdvance(uint8_t state, uint8_t station, const uint8_t* cellIndex)
    {
        for (uint8_t bin = 0; bin < kMosaicCols; ++bin)
        {
            uint8_t depth = mosaicDepth(state, bin);
            while (depth < kMosaicRows && cellIndex[depth * kMosaicCols + bin] == station)
            {
                ++depth;
            }
            state = mosaicWithDepth(state, bin, depth);
        }
        return state;
    }
}    // namespace

#pragma pop_macro("WHITE")

// =============================================================================
// Competition tasks
// Very important: Keep task and task task specific.
// Do not place every task in a single task. This will make the code hard to read and maintain.
// =============================================================================

bool TasksClass::readMosaic(const TaskArgs& /*args*/)
{
    PathTracker.goTo("i3");
    Drive.turnDeg(-90);
    return _visionRead();
}

bool TasksClass::rectangleShovel(const TaskArgs& /*args*/)
{
    PathTracker.goTo("shovel1Get").setArrivalDirection(Direction::REAR);
    Drive.turnToYaw(90);
    Manipulator.boxClose();
    PathTracker.goTo("shovel1Put").setArrivalDirection(Direction::REAR);
    Manipulator.boxOpen();
    return true;
}

bool TasksClass::triangleShovel(const TaskArgs& /*args*/)
{
    PathTracker.goTo("shovel2Get").setArrivalDirection(Direction::REAR);
    Drive.turnToYaw(90);
    Manipulator.boxClose();
    PathTracker.goTo("shovel2Put").setArrivalDirection(Direction::REAR);
    Drive.turnToYaw(25);
    Manipulator.boxOpen();
    return true;
}

bool TasksClass::pot(const TaskArgs& /*args*/)
{
    PathTracker.goTo("PotGet").setArrivalDirection(Direction::REAR);
    Manipulator.boxClose();
    PathTracker.goTo("shovel2Get").setArrivalDirection(Direction::REAR);
    PathTracker.goTo("PotPut") ;
    Manipulator.boxOpen();
    return true;
}

bool TasksClass::mosaic(const TaskArgs& /*args*/)
{
    // Consumes the latch captured by the `readMosaic` task — this body does not read the
    // camera itself. `_mosaicAlgo` reports one entry per tile, already in optimal collection
    // order, and a run of consecutive entries sharing a colour is one stop at that station.
    MosaicPick    picks[Config::VISION_CELL_COUNT];
    const uint8_t tiles = _mosaicAlgo(picks, Config::VISION_CELL_COUNT);

    if (tiles == 0)
    {
        Serial.println(F("[Tasks] mosaic: no route"));
        return false;
    }

    Serial.print(F("[Tasks] mosaic plan:"));
    for (uint8_t i = 0; i < tiles; ++i)
    {
        Serial.print(F(" "));
        Serial.print(kMosaicColorTag[mosaicColorIndex(picks[i].color)]);
        Serial.print(F("->bin"));
        Serial.print(mosaicBinForColumn((uint8_t)(picks[i].cell % kMosaicCols)));
    }
    Serial.println();

    // Tiles still on the wall, indexed [station][row]. Both rows start full.
    uint8_t stock[kMosaicColorCount][2];
    for (uint8_t c = 0; c < kMosaicColorCount; ++c)
    {
        stock[c][kMosaicRowLower] = kMosaicTilesPerRow;
        stock[c][kMosaicRowUpper] = kMosaicTilesPerRow;
    }

    // Row node the chassis is parked at; 0xFF until the first drive. `chainPos` starts at the
    // node the robot reaches the wall on, so the first row choice is measured from there.
    uint8_t parkedColor = 0xFF;
    uint8_t parkedRow   = 0xFF;
    uint8_t chainPos    = kMosaicEntryChainPos;

    // Enter the wall through the reset node — a direct run at a row node crosses the mosaic.
    // `MR` is a NodeType::RESET node, so arriving squares up, presses the rear into the wall
    // and re-anchors Localization on its own.
    Manipulator.beltTo(Config::BeltStation::MID);
    PathTracker.goTo(kMosaicResetNode);

    for (uint8_t i = 0; i < tiles; ++i)
    {
        const uint8_t station = mosaicColorIndex(picks[i].color);
        const uint8_t bin     = mosaicBinForColumn((uint8_t)(picks[i].cell % kMosaicCols));

        const uint8_t row =
            _mosaicPickRow(stock[station], station, parkedColor, parkedRow, chainPos);
        if (row >= 2)
        {
            Serial.print(F("[Tasks] mosaic: wall out of "));
            Serial.println(kMosaicColorTag[station]);
            return false;
        }

        if (station != parkedColor || row != parkedRow)
        {
            // Retract before every chassis move — an extended belt throws the centre of mass
            // too far off to drive on.
            Manipulator.beltTo(Config::BeltStation::GRAB_MID);
            PathTracker.goTo(kMosaicRowNodes[station][row].node);
            Drive.turnToYaw(90);

            parkedColor = station;
            parkedRow   = row;
            chainPos    = kMosaicRowNodes[station][row].chainPos;
        }

        // Tiles come off a row in take order, so the slot index is how many are gone.
        const uint8_t slot = (uint8_t)(kMosaicTilesPerRow - stock[station][row]);
        Manipulator.beltTo(kMosaicGrabOrder[slot]);
        _grabToBin(bin);
        --stock[station][row];
    }

    Manipulator.beltTo(Config::BeltStation::GRAB_MID);
    return true;
}

bool TasksClass::debries(const TaskArgs& /*args*/)
{
    PathTracker.goTo("CG1").setArrivalDirection(Direction::REAR);
    Manipulator.boxClose();
    PathTracker.goTo("CG2").setArrivalDirection(Direction::FRONT);
    Drive.turnToYaw(0);
    PathTracker.goTo("CG3").setArrivalDirection(Direction::REAR);
    Manipulator.boxOpen();

    PathTracker.goTo("CY1").setArrivalDirection(Direction::REAR);
    Manipulator.boxClose();
    PathTracker.goTo("CY2").setArrivalDirection(Direction::FRONT);
    Drive.turnDeg(-90);
    PathTracker.goTo("CY3").setArrivalDirection(Direction::REAR);
    Manipulator.boxOpen();
    return true;
}

bool TasksClass::dumpMosaic(const TaskArgs& /*args*/)
{
    PathTracker.goTo("dump1").setArrivalDirection(Direction::FRONT);
    Manipulator.dump();
    Manipulator.dumpReset();
    PathTracker.goTo("dump2").setArrivalDirection(Direction::REAR);
    Manipulator.dump();
    Manipulator.dumpReset();
    PathTracker.goTo("dump3").setArrivalDirection(Direction::REAR);
    Manipulator.dump();
    Manipulator.dumpReset();
    return true;
}

// =============================================================================
// Private helpers for competition tasks
// =============================================================================

uint8_t TasksClass::_mosaicPickRow(
    const uint8_t stock[2],
    uint8_t       colorIdx,
    uint8_t       parkedColor,
    uint8_t       parkedRow,
    uint8_t       chainPos) const
{
    // Already parked on a row of this colour that still has tiles: stay put and let the
    // belt reach the next one — a belt move is far cheaper than a drive.
    if (colorIdx == parkedColor && parkedRow < 2 && stock[parkedRow] > 0)
    {
        return parkedRow;
    }

    // Otherwise enter the colour at whichever of its two rows still has tiles, preferring
    // the one nearer along the wall chain to where the robot currently sits.
    const bool lowerOk = stock[kMosaicRowLower] > 0;
    const bool upperOk = stock[kMosaicRowUpper] > 0;

    if (lowerOk && upperOk)
    {
        const uint8_t lowerGap =
            mosaicChainGap(chainPos, kMosaicRowNodes[colorIdx][kMosaicRowLower].chainPos);
        const uint8_t upperGap =
            mosaicChainGap(chainPos, kMosaicRowNodes[colorIdx][kMosaicRowUpper].chainPos);
        return (lowerGap <= upperGap) ? kMosaicRowLower : kMosaicRowUpper;
    }

    if (lowerOk) return kMosaicRowLower;
    if (upperOk) return kMosaicRowUpper;
    return 0xFF;    // this colour is exhausted on the wall
}

uint8_t TasksClass::_mosaicAlgo(MosaicPick picks[], uint8_t maxPicks) const
{
    if (picks == nullptr || maxPicks == 0)
    {
        return 0;
    }

    // ---- Read the pattern as station indices -------------------------------------------
    uint8_t cellIndex[Config::VISION_CELL_COUNT];
    uint8_t needed[kMosaicColorCount] = { 0, 0, 0, 0 };
    uint8_t maskedCells               = 0;   // cells substituted with the fallback (masking ON)

    for (uint8_t cell = 0; cell < Config::VISION_CELL_COUNT; ++cell)
    {
        uint8_t station = mosaicColorIndex(Vision.getColor(cell));
        if (station >= kMosaicColorCount)
        {
            // UNKNOWN (uncalibrated / surprise hue, or no successful read) or RED — no wall station
            // supplies it. With masking ON (Config::VISION_MASK_UNKNOWN_ENABLED) substitute the
            // fallback station so the tile is still placed (scores 5 pts) rather than aborting the
            // whole build. With masking OFF it is a genuine failure — name the cell rather than
            // silently planning a partial route.
            if (!Config::VISION_MASK_UNKNOWN_ENABLED)
            {
                Serial.print(F("[Tasks] mosaic: cell "));
                Serial.print(cell);
                Serial.println(F(" has no station colour"));
                return 0;
            }
            // Count the substitution and enforce the cap HERE, at the point of use — do NOT trust
            // that the read already bounded it. The diagnostic `Vision::readMosaicRaw()` (VISION
            // VIEW / VISION TEST) writes the same latch with NO cap, and a failed mission
            // `readMosaic()` leaves whatever the latch already held in place, so the frame reaching
            // this line is not guaranteed to have passed Config::VISION_MASK_MAX_UNKNOWN_CELLS. The
            // over-cap check below bounds the number of fabricated tiles regardless of provenance.
            station = mosaicColorIndex(Config::VISION_MASK_FALLBACK_COLOR);
            ++maskedCells;
            Serial.print(F("[Tasks] mosaic: cell "));
            Serial.print(cell);
            Serial.print(F(" UNKNOWN -> masked to "));
            Serial.println(kMosaicColorTag[station]);
        }
        cellIndex[cell] = station;
        ++needed[station];
    }

    if (maskedCells > Config::VISION_MASK_MAX_UNKNOWN_CELLS)
    {
        // Over the mask budget: the latch is not a clean capped read (a stale diagnostic read, or a
        // failed mission read that left an old frame behind). Abort rather than build a mostly
        // invented mosaic — this is the point-of-use half of the cap the read enforces on its side.
        Serial.print(F("[Tasks] mosaic: "));
        Serial.print(maskedCells);
        Serial.print(F(" masked cells over cap "));
        Serial.println(Config::VISION_MASK_MAX_UNKNOWN_CELLS);
        return 0;
    }

    for (uint8_t station = 0; station < kMosaicColorCount; ++station)
    {
        if (needed[station] > (uint8_t)(2 * kMosaicTilesPerRow))
        {
            Serial.print(F("[Tasks] mosaic: pattern needs "));
            Serial.print(needed[station]);
            Serial.print(kMosaicColorTag[station]);
            Serial.print(F(", wall holds "));
            Serial.println((uint8_t)(2 * kMosaicTilesPerRow));
            return 0;
        }
    }

    // ---- Solve the visit order exactly -------------------------------------------------
    // Every stop loads at least one tile, so a stop strictly increases `mosaicLoaded()` and
    // the state graph is a DAG layered by tile count. Filling the layers from the finished
    // mosaic downwards therefore needs no priority queue: by the time a layer is processed,
    // every state reachable from it already holds its final cost.
    //
    // `bestCost[state][station]` = minimum remaining travel to finish, given the carrier is
    // in `state` with the robot parked at `station`. 1 KB, static so it stays off the stack.
    static uint8_t bestCost[kMosaicStateCount][kMosaicColorCount];

    for (int8_t loaded = (int8_t) Config::VISION_CELL_COUNT; loaded >= 0; --loaded)
    {
        for (uint16_t s = 0; s < kMosaicStateCount; ++s)
        {
            const uint8_t state = (uint8_t) s;
            if (mosaicLoaded(state) != (uint8_t) loaded)
            {
                continue;
            }

            for (uint8_t from = 0; from < kMosaicColorCount; ++from)
            {
                if (state == kMosaicStateDone)
                {
                    bestCost[s][from] = 0;
                    continue;
                }

                uint8_t best = kMosaicCostInf;
                for (uint8_t to = 0; to < kMosaicColorCount; ++to)
                {
                    const uint8_t next = mosaicAdvance(state, to, cellIndex);
                    if (next == state || bestCost[next][to] >= kMosaicCostInf)
                    {
                        continue;    // loads nothing, or leads to a dead end
                    }

                    const uint8_t total = (uint8_t)(
                        mosaicChainGap(kMosaicStationHalf[from], kMosaicStationHalf[to]) +
                        bestCost[next][to]);
                    if (total < best)
                    {
                        best = total;
                    }
                }
                bestCost[s][from] = best;
            }
        }
    }

    // ---- Walk the optimal order, emitting one entry per tile ----------------------------
    uint8_t state = 0;
    uint8_t count = 0;
    // The first hop is measured from the wall entry node, not from another station.
    uint8_t fromHalf = (uint8_t)(2 * kMosaicEntryChainPos);

    while (state != kMosaicStateDone && count < maxPicks)
    {
        uint8_t chosen = 0xFF;
        uint8_t best   = kMosaicCostInf;

        for (uint8_t to = 0; to < kMosaicColorCount; ++to)
        {
            const uint8_t next = mosaicAdvance(state, to, cellIndex);
            if (next == state || bestCost[next][to] >= kMosaicCostInf)
            {
                continue;
            }

            const uint8_t total = (uint8_t)(
                mosaicChainGap(fromHalf, kMosaicStationHalf[to]) + bestCost[next][to]);
            if (total < best)
            {
                best   = total;
                chosen = to;
            }
        }

        if (chosen >= kMosaicColorCount)
        {
            Serial.println(F("[Tasks] mosaic: pattern cannot be completed"));
            return 0;
        }

        // Load this stop bin by bin. A tile taken here fills cell `depth * kMosaicCols + bin`
        // and the bin index IS the sorter bin, so the dump reproduces the pattern row for row.
        for (uint8_t bin = 0; bin < kMosaicCols && count < maxPicks; ++bin)
        {
            uint8_t depth = mosaicDepth(state, bin);
            while (depth < kMosaicRows && cellIndex[depth * kMosaicCols + bin] == chosen &&
                   count < maxPicks)
            {
                picks[count].color = kMosaicColors[chosen];
                picks[count].cell  = (uint8_t)(depth * kMosaicCols + bin);
                ++count;
                ++depth;
            }
            state = mosaicWithDepth(state, bin, depth);
        }

        fromHalf = kMosaicStationHalf[chosen];
    }

    return count;
}
