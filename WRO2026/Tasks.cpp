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

#include <Arduino.h>
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
    { "debugYield", &TasksClass::debugYield },
    { "servoTest", &TasksClass::servoTest },
    { "home", &TasksClass::home },
    { "testChassis", &TasksClass::testChassis },
    { "testOne", &TasksClass::testOne },
    { "test1", &TasksClass::test1 },
    { "test0621", &TasksClass::test0621 },
    { "ctpsExample1", &TasksClass::ctpsExample1 },
    //Add new tasks here
};

const size_t TasksClass::_registry_count =
    sizeof(_registry) / sizeof(_registry[0]);

static_assert(sizeof(TasksClass::_registry) / sizeof(TaskRegistryEntry)
              <= Config::MAX_TASKS,
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

bool TasksClass::home(const TaskArgs& /*args*/)
{
    // belt.home() is the whole routine: reset() (stall to the LEFT/HOME end-stop + re-zero
    // the encoder there), settle, then drive on to the MID rest position. It's non-blocking,
    // so poll until the full sequence lands (the belt's own watchdogs bound each stage).
    //
    // Drive the registered cooperative-tick callback each iteration so every module keeps
    // ticking during the block (the same pattern as Manipulator::_drain); fall back to ticking
    // only Manipulator.update() with a one-shot warning if no callback is set. yield() services
    // the R4 background; we never block with a fixed delay().
    Manipulator.belt.home();
    static bool noCallbackWarned = false;
    while (!Manipulator.belt.isDone())
    {
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!noCallbackWarned)
            {
                Serial.println(F("[Tasks] home: no tick callback set — only "
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

bool TasksClass::testChassis(const TaskArgs& /*args*/)
{
    LineFollower.setEnabled(true).setDistanceTarget(50);
    return LineFollower.waitUntilDistanceReached();
}

bool TasksClass::testOne(const TaskArgs& /*args*/)
{
    PathTracker.goTo("Inter1");
    PathTracker.goTo("Inter2");
    PathTracker.goTo("Inter3");
    PathTracker.goTo("position1");
    return true;
}

bool TasksClass::test1(const TaskArgs& /*args*/)
{
    PathTracker.goTo("Inter1");
    (void)Manipulator.grab();
    Manipulator.belt.toStation(Config::BeltStation::ABC);
    Drive.setSpeed(30, 60);
    PathTracker.goTo("Inter2");
    PathTracker.goTo("Inter3");
    PathTracker.goTo("position1");
    Drive.wallStop(20.0f);
    Drive.driveDist(20).setPower(80).setMode(DriveMode::GYRO);
    LineFollower.setEnabled(true).setDistanceTarget(90);
    LineFollower.waitUntilDistanceReached();
    return true;
}

bool TasksClass::test0621(const TaskArgs& /*args*/)
{
    Drive.driveDist(35).setStartSpeed(40).setPower(100).setEndSpeed(10).setMode(DriveMode::GYRO).setHeading(0);
    return true;
}

bool TasksClass::ctpsExample1(const TaskArgs& /*args*/)
{
    PathTracker.goTo("MY1"); // Get mosaic
    Drive.turnToYaw(90);
    Manipulator.sortTo(1);
    Manipulator.grab();
    Manipulator.sortTo(1);
    Manipulator.grab();
    Manipulator.sortTo(1);
    Manipulator.grab();
    Manipulator.sortTo(1);
    Manipulator.grab();
    Manipulator.sortTo(1);
    Manipulator.grab();

    

    //Grab shovel
    PathTracker.goTo("shovel").setArrivalDirection(Direction::REAR);
    Manipulator.boxClose();
    
    //Go to shovel drop
    PathTracker.goTo("shovel_drop");
    Drive.turnToYaw(270);
    Manipulator.boxOpen();

    //Go to dump
    PathTracker.goTo("dump1").setArrivalDirection(Direction::FRONT);
    Manipulator.dump();

    return true;
}
