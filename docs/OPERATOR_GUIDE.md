# WRO 2026 — Operator & Programming Guide

A practical, hands-on guide to writing and running missions on the robot. It covers
how the firmware is wired together, the Mission DSL, the Tasks framework, how to fire
the PathTracker, how to drive the chassis directly (Drive + LineFollower), the
Manipulator, the configs you'll actually touch, and the rules that keep you out of
trouble.

This is a companion to the deeper docs — see [`ARCHITECTURE.md`](../ARCHITECTURE.md),
[`DECISIONS.md`](../DECISIONS.md), and the per-module Doxygen headers in `WRO2026/`.
When in doubt, the header comment for a class is the source of truth; this guide is the
map.

---

## Table of contents

1. [Mental model — how the firmware runs](#1-mental-model)
2. [The operator flow at the field](#2-the-operator-flow-at-the-field)
3. [Writing a mission — the Mission DSL](#3-writing-a-mission--the-mission-dsl)
4. [Tasks — named, parameterised actions](#4-tasks)
5. [Adding a new task (and helpers inside a task)](#5-adding-a-new-task)
6. [Firing the PathTracker](#6-firing-the-pathtracker)
7. [Driving the chassis — Drive](#7-driving-the-chassis--drive)
8. [Following lines — LineFollower](#8-following-lines--linefollower)
9. [The Manipulator](#9-the-manipulator)
10. [Configs you'll actually touch](#10-configs-you'll-actually-touch)
11. [Golden rules & common gotchas](#11-golden-rules--common-gotchas)
12. [Build, flash, test](#12-build-flash-test)

---

## 1. Mental model

Everything you call is a **global singleton** with a fixed name: `Mission`, `Tasks`,
`PathTracker`, `Drive`, `LineFollower`, `Manipulator`, `Localization`, `Intersection`,
`Vision`, `EventTrace`, `LineArray`. You never construct these — you just call methods
on them.

### The cooperative tick

There is **no business logic in `loop()`**. The whole robot runs off one function,
`cooperativeTick()` (in `WRO2026.ino`), which calls every module's `update()` exactly
once, in this order:

```
LineArray → Drive → LineFollower → Intersection → Manipulator → Tasks
          → PathTracker → Localization → Mission → EventTrace
```

Each `update()` is **non-blocking**: it does a tiny slice of work and returns. State
machines advance one step per tick. The main `loop()` just calls `cooperativeTick()`
over and over.

`cooperativeTick` is also registered as the **tick callback** on Drive, PathTracker,
LineFollower, Manipulator, Tasks and Vision via `setTickCallback(cooperativeTick)`.
That means even when a "blocking" (sync) call spins waiting for the chassis, it keeps
re-entering the whole tick chain so nothing else freezes (the OLED keeps drawing, the
belt keeps moving, the line sensor keeps refreshing, etc.).

### Fluent builders, sync-by-default

Most "verbs" return a **fluent builder** you chain setters onto. The builder executes
when the statement ends (its destructor fires — RAII). By default the chassis-level
verbs **block until done**, so a mission reads top-to-bottom like a script:

```cpp
Drive.driveDist(30);            // drives 30 cm, blocks until finished
Drive.turnToYaw(90);            // turns to 90°, blocks until finished
LineFollower.followDistance(50);// follows the line 50 cm, blocks until finished
```

Chain `.async()` to opt out of the wait (used by the path planner, which ticks `loop()`
itself and polls `isIdle()`):

```cpp
Drive.driveDist(30).async();    // returns immediately; poll Drive.isIdle()
```

> ⚠️ **The single most important rule** is in §11: a builder chain must be **one
> statement**. Never store it in a variable.

---

## 2. The operator flow at the field

This is what physically happens, end to end (from `WRO2026.ino`):

1. **Power on.** Every module's `begin()` runs. On success the OLED shows `BOOT OK`,
   the buzzer chirps 3×, and both RGB LEDs go **green**. On any `begin()` failure the
   OLED shows which module failed, one long low tone plays, and the LEDs go **red** —
   the robot will not run.
2. **Place the chassis** on `Routes::START_NODE` (node `A`) facing
   `Routes::START_HEADING_DEG`.
3. **(Optional) Calibration menu** — hold **BTN_DOWN ≥ 2 s** to open the OLED
   `CALIBRATION` menu. BTN_DOWN short-presses cycle the cursor (`LINE → MOTOR → EXIT`),
   BTN_UP confirms:
   - **LINE** → runs `LineArray.calibrate()` (per-board reflectance sweep; BTN_UP gates
     between boards).
   - **MOTOR** → runs `Drive.calibrateMotorBias()` — **lift the chassis first**, the
     wheels spin ~2 s.
   - **EXIT** → back to idle.
   The menu is only available *before* the mission is armed.
4. **Press BTN_UP once.** This zeroes the IMU yaw at the current heading, then calls
   `buildMission()` → `Mission.start()`. The mission now runs asynchronously off
   `loop()`. In production it arms exactly once per power cycle.
5. **During the run**, Serial prints state-change events plus a `HB t=… yaw=… node=…`
   heartbeat every 500 ms; the OLED scrolls the same events.
6. **On a fault** (PathTracker fault, or a `goTo` to an unknown node), the chassis
   brakes, the queue is dropped, the buzzer beeps 5× and the LEDs flash red, then hold
   solid red. **Press BTN_DOWN** to acknowledge, clear the fault, and return to idle.

Your job as a programmer is almost entirely **step 4's `buildMission()`** — it lives in
`MissionScript.cpp`.

---

## 3. Writing a mission — the Mission DSL

The mission is a hand-typed chain of `Mission.<verb>(...)` calls in
**`MissionScript.cpp`** → `buildMission()`. Each verb **enqueues a step**; nothing
moves until `Mission.start()`.

```cpp
// MissionScript.cpp
#include "MissionScript.h"
#include "Mission.h"

void buildMission()
{
    Mission
        .goTo("A")
        .beep(1000, 200)
        .print("at A")
        .wait(500)
        .goTo("B")
        .led(0, 255, 0, 300)
        .goTo("C");
    Mission.start();          // arm; returns immediately, loop() drains the queue
}
```

### The verbs

| Verb | What it does |
|------|--------------|
| `goTo(const char* node)` | Navigate to a node. Resolves the name **now**; walks the precomputed route there. |
| `wait(uint32_t ms)` | Non-blocking pause. |
| `print(const char* msg)` | Log a line to Serial + OLED. |
| `beep(uint16_t freq_hz, uint32_t ms)` | Buzzer tone (duration clamped to ~65 s). |
| `led(uint8_t r, g, b, uint32_t ms = 0)` | RGB LEDs. `ms = 0` holds the colour until the next `led`. |
| `task(const char* name, args...)` | Run a registered Task (see §4). |

### Per-`goTo` route modifiers

These chain **immediately after** a `goTo(...)` and tune *that* hop's route. They only
apply to the most-recently-enqueued `goTo`; calling them after a non-`goTo` verb is a
no-op + Serial warning.

```cpp
Mission
    .goTo("D").setRouteDirection(Direction::REAR)   // every hop on the path drives in reverse
    .goTo("E").setArrivalDirection(Direction::FRONT)// only the final hop into E faces forward
    .goTo("F").setStopAtLine(StopAtLine::ON);       // force blind hops to seek a line to re-anchor
```

- `setRouteDirection(Direction)` — force `FRONT`/`REAR`/`AUTO` for **every** hop.
- `setArrivalDirection(Direction)` — force direction for the **final** hop only (wins
  over route direction for that hop).
- `setStopAtLine(StopAtLine)` — `ON`/`OFF`/`AUTO` for the path's **blind** hops.
  `ON`/`OFF` overrides each edge's own setting; `AUTO` defers to the edge.

Defaults (`AUTO`) just let the planner and the per-edge field-data settings decide —
that's what you want unless you have a specific reason.

### Lifetime rule for string arguments

`goTo`, `print`, and `task` store the `const char*` **by pointer** for later logging.
**Pass string literals** (`"A"`, `"at A"`) — they live forever. Never pass a stack
buffer or a temporary `String.c_str()`; it will dangle by the time the step executes.

### Mission control methods (act immediately, not queued)

- `Mission.start()` — arm and run the queue. **Calling it while already running rewinds
  to step 0 and re-runs everything.** The `.ino` gates this with `_missionArmed`, so in
  practice you call it exactly once.
- `Mission.abort()` — hard safety stop: cancels PathTracker, disables LineFollower,
  brakes, empties the queue, clears faults, returns to `IDLE`.
- `Mission.declarePosition(const char* node)` — operator recovery: declare the chassis
  is on `node` with zero offset and clear a latched PathTracker fault. Usable only when
  PathTracker is idle or faulted (never mid-hop). Re-zero heading separately with
  `Localization.zeroHeadingFromCurrent()` if you re-oriented the robot.
- `Mission.isIdle()` / `Mission.hasFault()` — state queries.

### Fault behaviour

If a `goTo` targets an unknown node, or PathTracker faults mid-route, the remaining
queue is dropped, the chassis brakes, and Mission runs the beep/flash/await-reset
sequence (5 × 1 kHz beeps, then solid red, waiting for BTN_DOWN). A queue-time **typo
in a `task` name refuses to arm** at `start()` — you get a Serial warning instead of a
mid-run fault.

### ⚠️ Mission verbs are queue-only — do **not** call them from inside a Task body

This trips everyone up. Calling `Mission.goTo(...)` / `wait` / `beep` / `led` from
inside a task body does **not** run synchronously — it *appends* a step to the queue
that runs *after* the current task step finishes. Task bodies that need to move the
chassis or beep **must use the singletons directly** (`PathTracker`, `Drive`,
`LineFollower`, `MiniR4.Buzzer`, `MiniR4.LED`). The `task` verb specifically guards
against nesting (warns + ignores); the other verbs queue silently. See ADR-025.

---

## 4. Tasks

A **Task** is a named, parameterised action you can call from a mission with
`Mission.task("name", args...)`. It's the bridge from the operator-authored mission to
real C++ logic (grip a block, run a sub-routine, drive a custom sequence) without
Mission needing to know what any task does.

A task body is a private member function on `TasksClass` with this signature:

```cpp
bool TasksClass::myTask(const TaskArgs& args);
```

Return `true` to advance to the next mission step; return `false` to **yield** (Mission
advances without aborting — used for "soft" no-ops).

### Calling a task from a mission

```cpp
Mission.task("servoTest", uint8_t(120));   // run the "servoTest" task with angle 120
Mission.task("home");                       // no-arg task
```

Tasks ship today (registered in `Tasks.cpp`): `demoEcho`, `debugYield`, `servoTest`,
`home`, `testChassis`, `testOne`. Most are bring-up/smoke-test helpers — read them as
worked examples.

### Passing arguments

Args are packed by `memcpy` into a fixed 16-byte POD (`TaskArgs`,
`Config::MAX_TASK_ARG_BYTES = 16`). Two compile-time rules are enforced:

- total arg bytes ≤ 16, and
- every arg type is trivially-copyable (no `String`, no classes with constructors).

Inside the body you read them back through the union views (`args.u8[0]`, `args.u16[0]`,
`args.u32[0]`, `args.i32[0]`, `args.f32[0]`), or `reinterpret_cast` to your own struct.

**Recommended pattern for anything beyond one or two scalars: define a per-task struct**
so the byte layout is unambiguous, and **zero-init it** at the call site to avoid
padding garbage:

```cpp
// somewhere shared
struct MosaicPlaceArgs { uint8_t cell; uint8_t color; uint16_t timeout_ms; };

// in the mission
MosaicPlaceArgs a{};      // zero-init kills padding bytes
a.cell = 3; a.color = 1; a.timeout_ms = 800;
Mission.task("mosaicPlace", a);

// in the task body
bool TasksClass::mosaicPlace(const TaskArgs& args)
{
    const auto* a = reinterpret_cast<const MosaicPlaceArgs*>(args.bytes);
    // ... use a->cell, a->color, a->timeout_ms
    return true;
}
```

### Lifecycle facts

- Tasks are **non-reentrant leaf operations**: a task body may not call `runTask`
  (directly or via `Mission.task`). Share code between tasks with **private helpers**,
  not by calling one task from another (ADR-026).
- `Tasks.inTask()` is true while a body runs; `Mission.update()` short-circuits during
  that window, so it's safe to drive the cooperative tick from inside a blocking task
  wait.
- `update()` is intentionally a no-op today (no multi-tick task FSM yet) — task bodies
  run to completion inside `runTask`.

---

## 5. Adding a new task

Three steps. (1) declare the body, (2) define it, (3) register the name.

**1 — Declare** it as a private method in `Tasks.h`:

```cpp
// Tasks.h, in the private: section alongside demoEcho/home/...
/**
 * @brief Grab the block in front of the chassis, then back off 10 cm.
 * @param args u8[0] = grip strength preset.
 * @return true on success.
 */
bool grabBlock(const TaskArgs& args);
```

**2 — Define** the body in `Tasks.cpp`. Use the singletons directly; if you block
waiting on an actuator, **drive the registered tick callback** each spin so the rest of
the robot keeps ticking (copy the pattern from `servoTest`/`home`):

```cpp
// Tasks.cpp
bool TasksClass::grabBlock(const TaskArgs& args)
{
    _approachLine();                 // a private helper (see below)
    Manipulator.grab();              // blocking verb — settles the grabber
    Drive.driveDist(10).async();     // back off; async so we can tick while it runs

    while (!Drive.isIdle())
    {
        if (_tickCallback != nullptr) _tickCallback();  // keep everyone alive
        else                          Drive.update();   // degraded fallback
        yield();
    }
    return true;
}
```

**3 — Register** the name in the `_registry[]` table in `Tasks.cpp` (one line). This is
the single source of truth for valid task names:

```cpp
const TaskRegistryEntry TasksClass::_registry[] =
{
    { "demoEcho",   &TasksClass::demoEcho   },
    { "home",       &TasksClass::home       },
    // ...
    { "grabBlock",  &TasksClass::grabBlock  },   // ← add here
};
```

The registry is capped at `Config::MAX_TASKS = 16` (a `static_assert` fails the build if
you exceed it). A typo'd name in `Mission.task(...)` is caught at `start()` time, before
the robot moves.

### Adding a helper *inside* a task

Tasks must not call each other, so **factor shared logic into private helper methods**
(this is also Diego's preference — decompose into named helpers rather than one long
body).

**1 — Declare** the helper in `Tasks.h` (private):

```cpp
private:
    /** @brief Line-follow up to the next intersection, then brake. */
    void _approachLine();
```

**2 — Define** it in `Tasks.cpp`:

```cpp
void TasksClass::_approachLine()
{
    LineFollower.followUntilIntersection();   // blocking helper, ticks the loop for us
}
```

Now both `grabBlock` and any future task can call `_approachLine()`. A helper is plain
C++ — it does **not** go in the registry (only callable tasks do) and it can have any
signature you like.

---

## 6. Firing the PathTracker

`PathTracker` turns a destination node into chassis motion. It consumes the precomputed
`Routes::NEXT_HOP` table (offline Dijkstra) and dispatches **one edge at a time** onto
Drive + LineFollower. You almost always reach it through `Mission.goTo(...)`, but you
can call it directly inside a task body.

### Always pass the FINAL node — never single hops

`goTo("F")` walks the *whole* route to `F` automatically (multi-hop). **Do not** issue a
chain of single-hop `goTo`s to "step" the robot — that's not how it's designed. Give it
the destination and let `NEXT_HOP` do the routing. (The `testOne` task lists several
`goTo`s only because each names a *destination*, run back-to-back.)

```cpp
// inside a task body — direct PathTracker use
PathTracker.goTo("Inter1");   // blocking by default: drives the full route, then returns
PathTracker.goTo("position1");
```

### The goTo builder

`goTo` returns a fluent builder. **Blocking (sync) by default**; chain `.async()` for
the poll-it-yourself style (what Mission's internal stepping uses):

```cpp
PathTracker.goTo("C");                                    // block until arrival
PathTracker.goTo("C").async();                            // return now; poll PathTracker.isIdle()
PathTracker.goTo("C").setRouteDirection(Direction::REAR);
PathTracker.goTo("C").setArrivalDirection(Direction::FRONT);
PathTracker.goTo("C").setStopAtLine(StopAtLine::ON);
```

You can target by name (`goTo("C")`) or by resolved id (`goTo(uint8_t)`). The same three
modifiers as the Mission DSL apply (they're the same mechanism underneath).

### Faults & recovery

PathTracker latches a typed fault and stops:

| Fault | Meaning |
|-------|---------|
| `NOT_ZEROED` | Heading was never zeroed (press BTN_UP first). |
| `UNREACHABLE` | No route from current node to the goal. |
| `DRIVE_FAULT` | The underlying Drive move faulted. |
| `BAD_NODE` | Goal name/id didn't resolve. |
| `DRAIN_TIMEOUT` | A goal exceeded its time budget. |
| `REVERSE_UNAVAILABLE` | A reverse hop was requested but no reverse edge exists. |
| `OFF_NODE` | Operator pushed the chassis past the trust cap; off-map gate refused. |
| `WALL_NOT_SQUARE` | Strict wall-normal check rejected a wall re-square. |
| `PIVOT_TRIM_UNDERFLOW` | A stop-at-line seek into a sharp pivot can't land short. |

Query with `hasFault()` / `getFault()`; `faultMnemonic(kind)` gives a short tag for
logs. `clearFault()` re-arms the queue **but does not restart motion**. For a true
cancel use `abort()` (stop + empty queue + reset to idle + clear fault) — that's what
`Mission.abort()` calls.

At the mission level you generally don't poll faults yourself — Mission does it and runs
the beep/await-BTN_DOWN recovery for you.

---

## 7. Driving the chassis — Drive

`Drive` is the single entry point to the wheels and to encoder/yaw state. Three families
of methods.

### A) Continuous primitives (direct calls, no builder)

Run until the next command or `stop()`. These do **not** touch the bounded-task "done"
flags.

```cpp
Drive.setSpeed(L, R);       // raw PWM each wheel, [-100,100]. Bias-compensated. (LineFollower uses this.)
Drive.setSpeedSync(L, R);   // STM32 wheel-sync PID closes the loop. No bias.
Drive.forwardGyro(power);   // drive straight holding the last yaw target. No bias.
Drive.stop(true);           // stop + active brake (false = coast)
```

### B) Bounded moves (fluent builders, sync-by-default)

```cpp
Drive.driveDist(30);                         // 30 cm, SYNC mode + trapezoid (tShape) on, blocks
Drive.driveTime(2.0f);                        // drive for 2.0 s
Drive.driveDist(30).setPower(50);             // override cruise PWM
Drive.driveDist(30).setMode(DriveMode::GYRO); // straight under yaw lock
Drive.driveDist(30).setMode(DriveMode::RAW);  // open-loop fallback (bias-compensated)
Drive.driveDist(30).flat();                   // no accel/decel ramps, constant cruise
Drive.driveDist(30).async();                  // return now, poll Drive.isIdle()
```

`DriveMode`: `SYNC` (default — encoder-sync PID), `GYRO` (heading-locked), `RAW`
(open-loop + motor bias). `SYNC + tShape` is the project-wide default — don't
re-specify it. Distance is always non-negative; direction comes from mode/power, not a
negative cm.

Common builder setters (Dist/Time): `.setPower(int16)`, `.setPreset(SpeedPreset)`,
`.setMode(DriveMode)`, `.setBrake(bool)`, `.setHeading(int16)` (GYRO target),
`.tShape()` / `.flat()`, `.setAccel(pwmPerCm)`, `.setStartSpeed(pwm)`,
`.setEndSpeed(pwm)`, `.async()`.

### C) Turns (fluent builders, sync-by-default)

```cpp
Drive.turnDeg(90);              // relative +90° (CCW)
Drive.turnToYaw(180);          // absolute heading 180°
Drive.turnToYaw(180).setPower(60).async();
```

Turn setters: `.setPower(int16)`, `.setMode(uint8)` (wheel pattern: 0 = counter-rotate,
1 = pivot-left, 2 = pivot-right), `.setBrake(bool)`, `.async()`. Yaw is in the **app
frame (CCW-positive)** — never re-apply the IMU sign yourself (see §10/§11).

### D) Sensor-terminated moves

```cpp
Drive.stopAtLine(40);                        // drive ~40 cm, arm a line detector near the end, stop on the line
Drive.stopAtLine(40).setMode(DriveMode::GYRO).async();
Drive.stopAtLine(40).setReverse(true);       // back into the line via the rear board (needs 2 boards)
Drive.wallStop(20);                          // push to a wall; encoder STALL = success; back off + idle
Drive.wallStop(20).setReverse(true).async(); // push rearward (only the rear may take wall load)
Drive.wallStop(20).setBackOff(3.0f).setTimeout(4000);
```

After a `stopAtLine`, check `Drive.isLineFound()` (a no-line outcome latches
`Fault::LINE_NOT_FOUND`). `wallStop` is stall-terminated, **not** a hard fault on
timeout — query the advisory `Drive.wallStopTimedOut()`.

### State, encoders, yaw, faults

```cpp
Drive.isIdle();        // true when chassis AND queue are fully done — use this to gate "next step"
Drive.isMoveDone();    // queue-blind; prefer isIdle()
Drive.waitUntilIdle(); // block until idle (top-level orchestration only)
Drive.getEncL(); Drive.getEncR(); Drive.resetEnc();
Drive.getYaw();        // app-frame (CCW+) yaw in degrees
Drive.holdYaw();       // snapshot current yaw as the forwardGyro/GYRO target
Drive.hasFault(); Drive.getFault(); Drive.clearFault();
```

Drive faults: `DRAIN_TIMEOUT`, `DISPATCH_FAILED`, `LINE_NOT_FOUND`, `IMU_READ_FAILED`.

---

## 8. Following lines — LineFollower

An outer P+D controller that rides on top of `Drive.setSpeed`. It reads the 8-channel
reflectance array and steers. It shares one `LineArray` singleton with `Intersection`.

### Enable / disable (fluent builder)

```cpp
LineFollower.setEnabled(true);                                       // start, current preset
LineFollower.setEnabled(true).setPreset(Config::SpeedPreset::FAST);  // start at FAST cruise
LineFollower.setEnabled(false);                                      // stop + active brake
LineFollower.setEnabled(false).setBrake(false);                     // stop, coast (planner issues next move)
```

Builder setters: `.setBrake(bool)` (disable path), `.setPreset(SpeedPreset)`,
`.setReverse(bool)` (follow backwards — flips to the rear board on 2-board configs),
`.setDistanceTarget(cm)` (stop after N cm), `.tShape()` / `.flat()` /
`.setTShapeRampUp(bool)` / `.setTShapeRampDown(bool)` / `.setAccel(pwmPerCm)`,
`.setOvershootMode(bool)` and `.setBrakeOnGate(bool)` (used by PathTracker for
pass-through traversals — you rarely touch these).

`tShape` (ramp up from 0, ramp down to floor before a distance gate) is **on by
default**. Re-enabling an already-running follower does **not** reset cruise or reseed
the ramp.

### Blocking helpers (top-level / task bodies — never from an `update()`)

```cpp
LineFollower.followDistance(50);          // follow 50 cm, then return (true = gate tripped)
LineFollower.followUntilIntersection();   // follow until the next fresh intersection, then stop
```

Need extra knobs first? Stage them, then wait:

```cpp
LineFollower.setEnabled(true).setPreset(Config::SpeedPreset::FAST).setDistanceTarget(50);
bool ok = LineFollower.waitUntilDistanceReached();
// or LineFollower.waitUntilIntersection();
```

### State

```cpp
LineFollower.isEnabled(); LineFollower.isLineLost(); LineFollower.isPidFrozen();
LineFollower.getPreset(); LineFollower.setPreset(p);
LineFollower.getReverse(); LineFollower.setReverse(b);
LineFollower.isDistanceReached();  // read-and-clear one-shot gate latch
LineFollower.getRawError();        // last weighted error, [-3.5, +3.5]
```

> When a bounded `Drive` move is dispatched, Drive auto-disables LineFollower so the two
> don't fight for the motors. You don't need to disable it manually before a
> `driveDist`.

---

## 9. The Manipulator

`Manipulator` owns four actuators: three servos (`grabber`, `sorter`, `dumper`) and one
encoder conveyor `belt`. Two API layers.

### A) Semantic verbs (everyday use, blocking-by-default)

Each verb fires its setpoint immediately and returns a `ManipAction` that **waits** for
the actuator to settle at end-of-statement (chain `.async()` to skip the wait — the
hardware still moves).

```cpp
Manipulator.grab();              // open the grabber to home, then close — block until settled
Manipulator.release();           // open the grabber
Manipulator.dump();              // move the dumper to its dump pose
Manipulator.sortTo(2);           // swing the sorter to bin index 2 (bounds-checked)
Manipulator.reset();             // home all three servos + stall-home the belt (safe pose)
Manipulator.grab().async();      // fire and don't wait (update() advances the home->close stroke)
```

> Today the verbs are thin scaffolding (`grab()` runs the grabber's home->close stroke, etc.).
> The real multi-actuator choreography gets authored once the mechanism is built — that's a TODO.

### B) Primitive handles (calibration / bring-up)

The raw joints are public so you can poke one actuator directly:

```cpp
Manipulator.grabber.setAngle(120);   // ManipServo: setAngle / open / close / home / isDone / commandedAngle
Manipulator.sorter.home();
Manipulator.belt.toStation(Config::BeltStation::MID);  // ManipBelt: toStation / beltTo / runFor / reset / home
Manipulator.belt.beltTo(400);        // absolute encoder target
Manipulator.belt.runFor(1500, 50);   // timed run (ms, signed speed: + toward FAR_END, − toward HOME)
```

### Completion model & ticking

- Servos have **no feedback** — `isDone()` is a slew+settle timer estimate.
- The belt has **real encoder feedback**.
- `Manipulator.isDone()` = AND of all four.
- `Manipulator.update()` advances only the belt's state machine (servos are pure
  timers); it's already in `cooperativeTick`.

If you write a task that blocks on a manipulator move, drive the tick callback while you
wait (same pattern as `servoTest`/`home` in §5), or just use the blocking verb and let
its drain tick the loop for you.

> ⚠️ A function registered via `setTickCallback()` must never issue a *blocking*
> manipulator verb — that re-enters the drain spin and recurses. Use `.async()` if a
> tick path ever needs manipulator motion.

---

## 10. Configs you'll actually touch

`Config.h` is a thin umbrella over per-domain headers in `WRO2026/config/`. Everything
is `static constexpr` in `namespace Config` — **no magic numbers in code**. The headers:
`ConfigGlobal`, `ConfigDrive`, `ConfigLine`, `ConfigNav`, `ConfigMission`,
`ConfigManipulator`, `ConfigVision`, and `ConfigChecks` (static_asserts, included last).

The ones you'll change most:

### Power & boot (`ConfigGlobal.h`)
- `BATT_CELLS = 2` — **MUST match the physical pack** (2S = 2). Wrong value risks the
  battery.
- `PWM_MAX = 95`, `PWM_MIN_MAGNITUDE = 50` — saturation cap and deadband floor. Non-zero
  PWM below the floor snaps up to 50 (so corrections always produce motion).
- Boot feedback tones (`BOOT_OK_BEEPS`, `BOOT_FAIL_BEEP_HZ`, …).

### Drive (`ConfigDrive.h`)
- `MOTOR_LEFT_ID = 2`, `MOTOR_RIGHT_ID = 3` — **don't edit in isolation** (hardcoded as
  M2/M3 across Drive; a `static_assert` enforces it).
- `enum class SpeedPreset { SLOW, NORMAL, FAST }`, with `SPEED_SLOW = 70`,
  `SPEED_NORMAL = 85`, `SPEED_FAST = 90`.
- `TURN_MODE = 0`, `TURN_BRAKE = true`, `TURN_CRUISE_PWM = 50`.
- `MOTOR_BIAS_L / MOTOR_BIAS_R` (default `1.0f`) — paste the values
  `Drive.calibrateMotorBias()` prints to Serial, then re-flash.

### Line (`ConfigLine.h`)
- `LINE_KP = 4.0f`, `LINE_KD = 0.0f` (no integral term in Phase 1).
- `LINE_BOARD_COUNT = 2` (front + rear), the I2C ports, and the
  `LINE_CALIB_LOW/HIGH[boards][8]` baselines.
- `LOST_LINE_ACTION = STOP` (or `COAST` / `CRAWL`).

### Nav / PathTracker (`ConfigNav.h`)
- `PATH_QUEUE_CAPACITY = 32`, `PATH_DRAIN_TIMEOUT_MS_PER_GOAL = 60000`.
- `NAV_TURN_TOLERANCE_DEG = 0.5f` (skip sub-degree turns).
- `PATH_PASS_THROUGH_ENABLED = true` (cruise straight through collinear junctions).
- `BIDIRECTIONAL_AUTO_ENABLED`, `BIDIRECTIONAL_TURN_THRESHOLD_DEG = 90`.
- `IMU_YAW_SIGN = -1.0f` — converts raw STM32 yaw (CW+) to app frame (CCW+). Applied at
  exactly one place (`YawSource`); **never apply it anywhere else**. `+1.0f` for a
  CCW-positive chassis (override via `-DIMU_YAW_SIGN_DEFAULT=1.0f`).
- Operator-offset / return-to-node tuning (`RETURN_TRIGGER_THRESHOLD_CM`,
  `RETURN_DISTANCE_CAP_CM`, `OFF_MAP_GATE`) and wall-reset constants — all bench-tunable.

### Mission & Tasks (`ConfigMission.h`)
- `MAX_MISSION_STEPS = 64`, `MAX_TASKS = 16`, `MAX_TASK_ARG_BYTES = 16`.
- `MISSION_ALLOW_REARM = true` — ships `true` for bench convenience (a fresh BTN_UP
  re-fires the mission once it goes idle, no power cycle needed between test runs).
  **MUST be flipped to `false` before the competition run** (edit the literal in
  `ConfigMission.h` or build with `-DMISSION_ALLOW_REARM_DEFAULT=false`) so a stray
  BTN_UP cannot restart the route mid-field. A self-silencing boot `[WARN]` banner
  prints once at startup whenever this is `true`, as a reminder.
- Fault beep sequence (`MISSION_FAULT_BEEPS = 5`, `MISSION_FAULT_BEEP_HZ = 1000`, …).

### Manipulator (`ConfigManipulator.h`)
- **Every value here is a bring-up placeholder — measure before trusting.**
- Servo channels (`GRABBER_RC_ID`, `SORTER_RC_ID`, `DUMPER_RC_ID`) and `ServoCfg` poses
  (`homeDeg`/`openDeg`/`closeDeg`/`msPerDeg`/`settleMs`/`reverse`).
- `SORTER_BIN_COUNT = 4`, `SORTER_BIN_ANGLE[]` (the angles `sortTo(bin)` uses).
- Belt: `BELT_M_ID` (must differ from the drive motors), `BELT_STATIONS[]`, speeds and
  timeouts.

> Several constants have a compile-time override hook (`#ifndef X_DEFAULT`) so host tests
> can flip them with `-DX_DEFAULT=…` without editing the shipped source. Leave the
> shipped defaults competition-safe.

---

## 11. Golden rules & common gotchas

**1. A builder chain is ONE statement. Never store it in a variable.**
The builders are move-only RAII objects that enqueue and execute when the temporary
destructs at end-of-statement. Splitting the chain corrupts the command queue.

```cpp
Drive.driveDist(30).setPower(50);          // ✅ correct
auto b = Drive.driveDist(30); b.setPower(50); // ❌ WRONG — corrupts the queue
```

**2. Mission verbs are queue-only — don't call them from a task body.** Use the
singletons directly (`PathTracker`, `Drive`, `LineFollower`, `MiniR4.Buzzer`,
`MiniR4.LED`). See §3 and §4.

**3. Never call a blocking verb from inside any module's `update()`.** `update()` must
stay non-blocking. Blocking calls (`driveDist` without `.async()`, `followDistance`,
`waitUntil*`, blocking manipulator verbs) are for top-level orchestration / task bodies
only. A blocking call from a registered tick callback recurses infinitely.

**4. `goTo` takes the FINAL destination.** PathTracker multi-hops via `NEXT_HOP`. Don't
hand-issue single hops.

**5. Pass string literals to `goTo` / `print` / `task`.** They're stored by pointer.
Stack buffers and `String.c_str()` will dangle.

**6. Yaw is app-frame (CCW-positive) everywhere except inside `YawSource`.** Don't
re-apply `IMU_YAW_SIGN`. Use `Drive.getYaw()` / `turnToYaw(...)` in app degrees.

**7. Use `isIdle()` to gate "is the next step safe?"** — `isMoveDone()` /
`isTurnDone()` are queue-blind.

**8. Lift the chassis before `Drive.calibrateMotorBias()`** — the wheels spin freely
for ~2 s.

**9. Two-layer line calibration.** MATRIX sensors have a volatile firmware illumination
layer (reset on power-cycle, re-applied in `begin()`) **and** software per-channel
baselines. Both are required — a power-cycle without re-running `begin()` reads wrong.

**10. Task args: zero-init your struct.** `memcpy` copies padding bytes verbatim; an
uninitialised struct leaks stack garbage into the unused offsets.

**11. Regenerate routes after editing the field.** If you change
`tools/path_planner/field_data.py`, re-run the planner (next section) and commit the
generated `routes.h/.cpp`, or CI fails.

---

## 12. Build, flash, test

### Arduino firmware
- Open `WRO2026/WRO2026.ino` in the **Arduino IDE** (board: *Arduino UNO R4 WiFi*),
  Verify / Upload. The MatrixMiniR4 library must be installed; deps: `ArduinoBLE`,
  `PubSubClient`.
- Headless compile:
  ```
  arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/
  ```
  Required: library ≥ v1.1.8, STM32 firmware ≥ v6.0.

### Python path planner (offline)
Regenerate `WRO2026/src/nav/routes.h` + `routes.cpp` whenever the field changes:
```
uv run python tools/path_planner/build_routes.py
```
Run the test suite:
```
uv run --extra dev pytest tools/path_planner/tests/
```

### CI
GitHub Actions runs the pytest suite and an Arduino compile check on every push/PR, and
checks that the generated `routes.*` are fresh.

---

## Quick reference — where things live

| You want to… | File |
|---|---|
| Write the competition mission | `WRO2026/MissionScript.cpp` → `buildMission()` |
| See the Mission verbs | `WRO2026/src/mission/Mission.h` |
| Add / edit a Task | `WRO2026/Tasks.h` + `WRO2026/Tasks.cpp` (`_registry[]`) |
| Navigate to a node | `WRO2026/src/mission/PathTracker.h` (`goTo`) |
| Drive / turn the chassis | `WRO2026/src/drive/Drive.h` |
| Follow a line | `WRO2026/src/drive/LineFollower.h` |
| Actuate the mechanism | `WRO2026/src/manip/Manipulator.h` (+ `ManipServo.h`, `ManipBelt.h`) |
| Tune a constant | `WRO2026/config/Config*.h` |
| Boot / arm flow | `WRO2026/WRO2026.ino` |
