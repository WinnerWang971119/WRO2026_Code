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
    { "emptyTest", &TasksClass::emptyTest },
    { "testOne", &TasksClass::testOne },
    { "test1", &TasksClass::test1 },
    { "test0621", &TasksClass::test0621 },
    { "actuatorSweep", &TasksClass::actuatorSweep },
    { "actuatorStep", &TasksClass::actuatorStep },
    { "manipDirectTest", &TasksClass::manipDirectTest },
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

bool TasksClass::emptyTest(const TaskArgs& /*args*/)
{
    Serial.println(F("[Tasks] emptyTest"));

    auto pumpTick = [this]()
    {
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            Drive.update();
            Manipulator.update();
        }
        yield();
    };

    auto waitMs = [&pumpTick](uint32_t durationMs)
    {
        const uint32_t start = millis();
        while ((uint32_t)(millis() - start) < durationMs)
        {
            pumpTick();
        }
    };

    auto waitForServo = [&pumpTick](ManipServo& servo)
    {
        while (!servo.isDone())
        {
            pumpTick();
        }
    };

    Manipulator.dumper.open();
    waitForServo(Manipulator.dumper);
    waitMs(200);

    Manipulator.sortTo(2);
    waitMs(200);

    Manipulator.grabber.open();
    waitForServo(Manipulator.grabber);
    waitMs(3000);

    Manipulator.grabber.close();
    waitForServo(Manipulator.grabber);
    waitMs(1000);

    Manipulator.grabber.open();
    waitForServo(Manipulator.grabber);
    waitMs(1000);

    Manipulator.dumper.close();
    waitForServo(Manipulator.dumper);
    waitMs(500);

    Manipulator.dumper.home();
    waitForServo(Manipulator.dumper);

    return true;
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

bool TasksClass::actuatorSweep(const TaskArgs& /*args*/)
{
    static constexpr int16_t  TEST_DRIVE_PWM      = 35;
    static constexpr uint32_t TEST_DRIVE_MS       = 700;
    static constexpr uint32_t TEST_SETTLE_MS      = 250;
    static constexpr uint32_t TEST_DRIVE_STOP_MS  = 150;

    static bool noCallbackWarned = false;

    auto pumpTick = [this]()
    {
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!noCallbackWarned)
            {
                Serial.println(F("[Tasks] actuatorSweep: no tick callback set — only "
                                 "Drive/Manipulator will tick during waits. Call "
                                 "Tasks.setTickCallback(...) in setup()."));
                noCallbackWarned = true;
            }
            Drive.update();
            Manipulator.update();
        }
        yield();
    };

    auto waitMs = [&pumpTick](uint32_t durationMs)
    {
        const uint32_t start = millis();
        while ((uint32_t)(millis() - start) < durationMs)
        {
            pumpTick();
        }
    };

    auto waitForServo = [&pumpTick](ManipServo& servo, uint32_t timeoutMs, const char* label) -> bool
    {
        const uint32_t start = millis();
        while (!servo.isDone())
        {
            if ((uint32_t)(millis() - start) >= timeoutMs)
            {
                Serial.print(F("[Tasks] actuatorSweep: servo timeout at "));
                Serial.println(label);
                return false;
            }
            pumpTick();
        }
        return true;
    };

    auto waitForBelt = [&pumpTick](uint32_t timeoutMs, const char* label) -> bool
    {
        const uint32_t start = millis();
        while (!Manipulator.belt.isDone())
        {
            if ((uint32_t)(millis() - start) >= timeoutMs)
            {
                Serial.print(F("[Tasks] actuatorSweep: belt timeout at "));
                Serial.println(label);
                return false;
            }
            pumpTick();
        }
        return true;
    };

    auto runDriveLeg = [&waitMs](const char* label, int16_t left, int16_t right)
    {
        Serial.print(F("[Tasks] actuatorSweep: "));
        Serial.println(label);
        Drive.setSpeed(left, right);
        waitMs(TEST_DRIVE_MS);
        Drive.stop(true);
        waitMs(TEST_DRIVE_STOP_MS);
    };

    Serial.println(F("[Tasks] actuatorSweep: start"));

    Drive.stop(true);
    (void)Manipulator.reset();

    runDriveLeg("left forward", TEST_DRIVE_PWM, 0);
    runDriveLeg("left reverse", -TEST_DRIVE_PWM, 0);
    runDriveLeg("right forward", 0, TEST_DRIVE_PWM);
    runDriveLeg("right reverse", 0, -TEST_DRIVE_PWM);
    runDriveLeg("drive forward", TEST_DRIVE_PWM, TEST_DRIVE_PWM);
    runDriveLeg("drive reverse", -TEST_DRIVE_PWM, -TEST_DRIVE_PWM);

    Serial.println(F("[Tasks] actuatorSweep: belt stations"));
    Manipulator.belt.toStation(Config::BeltStation::LEFT);
    if (!waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt left"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::FAR_END);
    if (!waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt far end"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::MID);
    if (!waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt mid"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] actuatorSweep: grabber"));
    Manipulator.grabber.open();
    if (!waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber open"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.close();
    if (!waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber close"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.home();
    if (!waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber home"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] actuatorSweep: sorter bins"));
    for (uint8_t bin = 0; bin < Config::SORTER_BIN_COUNT; ++bin)
    {
        Manipulator.sorter.setAngle(Config::SORTER_BIN_ANGLE[bin]);
        if (!waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter bin"))
        {
            Drive.stop(true);
            return false;
        }
        waitMs(TEST_SETTLE_MS);
    }
    for (int8_t bin = (int8_t)Config::SORTER_BIN_COUNT - 1; bin >= 0; --bin)
    {
        Manipulator.sorter.setAngle(Config::SORTER_BIN_ANGLE[(uint8_t)bin]);
        if (!waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter return"))
        {
            Drive.stop(true);
            return false;
        }
        waitMs(TEST_SETTLE_MS);
    }
    Manipulator.sorter.home();
    if (!waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter home"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] actuatorSweep: dumper"));
    Manipulator.dumper.open();
    if (!waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper open"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.close();
    if (!waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper close"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.home();
    if (!waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper home"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    (void)Manipulator.reset();
    Drive.stop(true);
    Serial.println(F("[Tasks] actuatorSweep: done"));
    return true;
}

bool TasksClass::actuatorStep(const TaskArgs& args)
{
    static constexpr uint8_t TARGET_GRABBER_POSE  = 0;
    static constexpr uint8_t TARGET_GRABBER_ABS   = 1;
    static constexpr uint8_t TARGET_SORTER_BIN    = 2;
    static constexpr uint8_t TARGET_SORTER_ABS    = 3;
    static constexpr uint8_t TARGET_DUMPER_POSE   = 4;
    static constexpr uint8_t TARGET_DUMPER_ABS    = 5;
    static constexpr uint8_t TARGET_BELT_STATION  = 6;
    static constexpr uint8_t TARGET_DRIVE_LEG     = 7;

    static constexpr uint8_t POSE_HOME  = 0;
    static constexpr uint8_t POSE_OPEN  = 1;
    static constexpr uint8_t POSE_CLOSE = 2;

    static constexpr uint8_t DRIVE_LEFT_FORWARD   = 0;
    static constexpr uint8_t DRIVE_LEFT_REVERSE   = 1;
    static constexpr uint8_t DRIVE_RIGHT_FORWARD  = 2;
    static constexpr uint8_t DRIVE_RIGHT_REVERSE  = 3;
    static constexpr uint8_t DRIVE_BOTH_FORWARD   = 4;
    static constexpr uint8_t DRIVE_BOTH_REVERSE   = 5;

    static bool noCallbackWarned = false;

    auto pumpTick = [this]()
    {
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!noCallbackWarned)
            {
                Serial.println(F("[Tasks] actuatorStep: no tick callback set — only "
                                 "Drive/Manipulator will tick during waits. Call "
                                 "Tasks.setTickCallback(...) in setup()."));
                noCallbackWarned = true;
            }
            Drive.update();
            Manipulator.update();
        }
        yield();
    };

    auto waitMs = [&pumpTick](uint32_t durationMs)
    {
        const uint32_t start = millis();
        while ((uint32_t)(millis() - start) < durationMs)
        {
            pumpTick();
        }
    };

    auto waitForServo = [&pumpTick](ManipServo& servo, uint32_t timeoutMs, const char* label) -> bool
    {
        const uint32_t start = millis();
        while (!servo.isDone())
        {
            if ((uint32_t)(millis() - start) >= timeoutMs)
            {
                Serial.print(F("[Tasks] actuatorStep: servo timeout at "));
                Serial.println(label);
                return false;
            }
            pumpTick();
        }
        return true;
    };

    auto waitForBelt = [&pumpTick](uint32_t timeoutMs, const char* label) -> bool
    {
        const uint32_t start = millis();
        while (!Manipulator.belt.isDone())
        {
            if ((uint32_t)(millis() - start) >= timeoutMs)
            {
                Serial.print(F("[Tasks] actuatorStep: belt timeout at "));
                Serial.println(label);
                return false;
            }
            pumpTick();
        }
        return true;
    };

    const uint8_t target = args.u8[0];
    const uint8_t value  = args.u8[1];
    const uint8_t aux    = args.u8[2];
    const uint8_t dwell  = args.u8[3];

    switch (target)
    {
    case TARGET_GRABBER_POSE:
        if (value == POSE_HOME) { Manipulator.grabber.home(); }
        else if (value == POSE_OPEN) { Manipulator.grabber.open(); }
        else if (value == POSE_CLOSE) { Manipulator.grabber.close(); }
        else
        {
            Serial.print(F("[Tasks] actuatorStep: bad grabber pose "));
            Serial.println((int)value);
            return false;
        }
        return waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber pose");

    case TARGET_GRABBER_ABS:
        Manipulator.grabber.setAngle(value);
        return waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber abs");

    case TARGET_SORTER_BIN:
        if (value >= Config::SORTER_BIN_COUNT)
        {
            Serial.print(F("[Tasks] actuatorStep: bad sorter bin "));
            Serial.println((int)value);
            return false;
        }
        (void)Manipulator.sortTo(value);
        return true;

    case TARGET_SORTER_ABS:
        Manipulator.sorter.setAngle(value);
        return waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter abs");

    case TARGET_DUMPER_POSE:
        if (value == POSE_HOME) { Manipulator.dumper.home(); }
        else if (value == POSE_OPEN) { Manipulator.dumper.open(); }
        else if (value == POSE_CLOSE) { Manipulator.dumper.close(); }
        else
        {
            Serial.print(F("[Tasks] actuatorStep: bad dumper pose "));
            Serial.println((int)value);
            return false;
        }
        return waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper pose");

    case TARGET_DUMPER_ABS:
        Manipulator.dumper.setAngle(value);
        return waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper abs");

    case TARGET_BELT_STATION:
        if (value >= Config::BELT_STATION_COUNT)
        {
            Serial.print(F("[Tasks] actuatorStep: bad belt station "));
            Serial.println((int)value);
            return false;
        }
        Manipulator.belt.toStation(static_cast<Config::BeltStation>(value));
        return waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt station");

    case TARGET_DRIVE_LEG:
    {
        const int16_t pwm = (aux > 100) ? 100 : aux;
        const uint32_t dwellMs = (dwell == 0 ? 5u : (uint32_t)dwell) * 100u;
        int16_t left = 0;
        int16_t right = 0;

        if (value == DRIVE_LEFT_FORWARD) { left = pwm; }
        else if (value == DRIVE_LEFT_REVERSE) { left = -pwm; }
        else if (value == DRIVE_RIGHT_FORWARD) { right = pwm; }
        else if (value == DRIVE_RIGHT_REVERSE) { right = -pwm; }
        else if (value == DRIVE_BOTH_FORWARD) { left = pwm; right = pwm; }
        else if (value == DRIVE_BOTH_REVERSE) { left = -pwm; right = -pwm; }
        else
        {
            Serial.print(F("[Tasks] actuatorStep: bad drive leg "));
            Serial.println((int)value);
            return false;
        }

        Drive.setSpeed(left, right);
        waitMs(dwellMs);
        Drive.stop(true);
        waitMs(150);
        return true;
    }

    default:
        Serial.print(F("[Tasks] actuatorStep: bad target "));
        Serial.println((int)target);
        return false;
    }
}

bool TasksClass::manipDirectTest(const TaskArgs& /*args*/)
{
    static constexpr uint32_t TEST_SETTLE_MS = 250;

    static bool noCallbackWarned = false;

    auto pumpTick = [this]()
    {
        if (_tickCallback != nullptr)
        {
            _tickCallback();
        }
        else
        {
            if (!noCallbackWarned)
            {
                Serial.println(F("[Tasks] manipDirectTest: no tick callback set — only "
                                 "Drive/Manipulator will tick during waits. Call "
                                 "Tasks.setTickCallback(...) in setup()."));
                noCallbackWarned = true;
            }
            Drive.update();
            Manipulator.update();
        }
        yield();
    };

    auto waitMs = [&pumpTick](uint32_t durationMs)
    {
        const uint32_t start = millis();
        while ((uint32_t)(millis() - start) < durationMs)
        {
            pumpTick();
        }
    };

    auto waitForServo = [&pumpTick](ManipServo& servo, uint32_t timeoutMs, const char* label) -> bool
    {
        const uint32_t start = millis();
        while (!servo.isDone())
        {
            if ((uint32_t)(millis() - start) >= timeoutMs)
            {
                Serial.print(F("[Tasks] manipDirectTest: servo timeout at "));
                Serial.println(label);
                return false;
            }
            pumpTick();
        }
        return true;
    };

    auto waitForBelt = [&pumpTick](uint32_t timeoutMs, const char* label) -> bool
    {
        const uint32_t start = millis();
        while (!Manipulator.belt.isDone())
        {
            if ((uint32_t)(millis() - start) >= timeoutMs)
            {
                Serial.print(F("[Tasks] manipDirectTest: belt timeout at "));
                Serial.println(label);
                return false;
            }
            pumpTick();
        }
        return true;
    };

    Serial.println(F("[Tasks] manipDirectTest: start"));

    Drive.stop(true);

    Serial.println(F("[Tasks] manipDirectTest: grabber home/open/close/home"));
    Manipulator.grabber.home();
    if (!waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber home 1"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.open();
    if (!waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber open"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.close();
    if (!waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber close"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.grabber.home();
    if (!waitForServo(Manipulator.grabber, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "grabber home 2"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] manipDirectTest: sorter bins"));
    Manipulator.sorter.home();
    if (!waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter home 1"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    for (uint8_t bin = 0; bin < Config::SORTER_BIN_COUNT; ++bin)
    {
        Manipulator.sorter.setAngle(Config::SORTER_BIN_ANGLE[bin]);
        if (!waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter bin"))
        {
            Drive.stop(true);
            return false;
        }
        waitMs(TEST_SETTLE_MS);
    }

    Manipulator.sorter.home();
    if (!waitForServo(Manipulator.sorter, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "sorter home 2"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] manipDirectTest: dumper home/open/close/home"));
    Manipulator.dumper.home();
    if (!waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper home 1"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.open();
    if (!waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper open"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.close();
    if (!waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper close"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.dumper.home();
    if (!waitForServo(Manipulator.dumper, Config::MANIP_SERVO_DRAIN_TIMEOUT_MS, "dumper home 2"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Serial.println(F("[Tasks] manipDirectTest: belt left/mid/far_end/mid"));
    Manipulator.belt.toStation(Config::BeltStation::LEFT);
    if (!waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt left"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::MID);
    if (!waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt mid 1"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::FAR_END);
    if (!waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt far_end"))
    {
        Drive.stop(true);
        return false;
    }
    waitMs(TEST_SETTLE_MS);

    Manipulator.belt.toStation(Config::BeltStation::MID);
    if (!waitForBelt(Config::BELT_HOME_TIMEOUT_MS, "belt mid 2"))
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
    PathTracker.goTo("MYU"); // Get mosaic
    Drive.turnToYaw(90);
    Manipulator.sortTo(0);
    Manipulator.grab();
    Manipulator.sortTo(1);
    Manipulator.grab();
    PathTracker.goTo("MYD"); // Get mosaic
    Drive.turnToYaw(90);
    Manipulator.sortTo(0);
    Manipulator.grab();
    Manipulator.sortTo(1);
    Manipulator.grab();
    PathTracker.goTo("MBU"); // Get mosaic
    Drive.turnToYaw(90);
    Manipulator.sortTo(2);
    Manipulator.grab();
    Manipulator.sortTo(3);
    Manipulator.grab();
    PathTracker.goTo("MBD"); // Get mosaic
    Drive.turnToYaw(90);
    Manipulator.sortTo(2);
    Manipulator.grab();
    Manipulator.sortTo(3);
    Manipulator.grab();
    PathTracker.goTo("MGU"); // Get mosaic
    Drive.turnToYaw(90);
    Manipulator.sortTo(0);
    Manipulator.grab();
    Manipulator.sortTo(1);
    Manipulator.grab();
    PathTracker.goTo("MGD"); // Get mosaic
    Drive.turnToYaw(90);
    Manipulator.sortTo(2);
    Manipulator.grab();
    Manipulator.sortTo(3);
    Manipulator.grab();

    //Grab shovel
    PathTracker.goTo("shovel1O");
    Drive.turnToYaw(90);
    Manipulator.boxClose();

    //Go to shovel drop
    PathTracker.goTo("shovel1F");
    Drive.turnToYaw(0);
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
    Drive.turnToYaw(0);
    PathTracker.goTo("CG3");
    Manipulator.boxOpen();
    PathTracker.goTo("CY1");
    Drive.turnToYaw(180);
    Manipulator.boxClose();
    PathTracker.goTo("CY3").setArrivalDirection(Direction::REAR);
    Manipulator.boxOpen();



    return true;
}
