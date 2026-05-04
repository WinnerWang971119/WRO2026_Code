# WRO 2026 — Architecture & Criteria

## Platform constraints (the rules everything else follows)

- **Hardware**: Matrix Mini R4 — Arduino UNO R4 WiFi + STM32 co-processor
- **Language**: C++ via Arduino IDE
- **Memory**: limited. No `new` / `malloc` in steady state. No `std::vector`, `std::map`. Use fixed-size arrays.
- **Execution**: single-threaded `setup()` + `loop()`. No `delay()` blocking. All long-running work uses `millis()` comparisons + state machines.
- **Style**: module-based (`.h` / `.cpp` pairs). Global singletons exposed via `extern`. No class hierarchies for system structure.

## File tree

```
WRO2026/
├── WRO2026.ino              # setup() + loop(), nothing else
│
├── Config.h                 # pins, constants, PID gains, MAX_NODES, ...
│
├── FieldMap.h / .cpp        # nodes + edges + directions (const arrays)
├── Pathfinder.h / .cpp      # Dijkstra over fixed array
├── Localization.h / .cpp    # currentNode + heading (single source of truth)
├── PathTracker.h / .cpp     # consume node sequence -> emit drive/turn cmds
│
├── Drive.h / .cpp           # wraps Matrix motor lib: setSpeed, turnDeg, stop
├── LineFollower.h / .cpp    # sensor read + PID -> wheel speed diff into Drive
├── Intersection.h / .cpp    # intersection detection from sensor array
├── ColorReader.h / .cpp     # I2C color sensor -> enum {Y, B, G, W, NONE}
├── CameraMosaic.h / .cpp    # camera capture + 3x3 pixel sampling + classify
├── Manipulator.h / .cpp     # setLift, setGripper, setArm (control only)
│
├── Tasks.h / .cpp           # per-task state machines
├── Mission.h / .cpp         # task queue + scheduler + time guard
│
└── StateMachine.h           # template helper, header-only
```

## Dependency graph

Calls go top-down only. Lower layers never call upper layers.

```
WRO2026.ino
   |
   v
Mission ────────► Tasks
                    |
                    v
                PathTracker ──► Pathfinder ──► FieldMap
                    |               |
                    |               └──► Localization
                    v
                  Drive ◄──► LineFollower ◄── Intersection
                    |
                    v
              Manipulator   ColorReader   CameraMosaic
                    |             |             |
                    v             v             v
              ╔═══════════════════════════════════╗
              ║   MatrixMiniR4 library (vendor)   ║
              ║   Motor / Encoder / IMU / Servo   ║
              ║   I2C / OLED / Button / WiFi      ║
              ╚═══════════════════════════════════╝
```

## Module responsibilities

| Module | Owns | Exposes |
|---|---|---|
| `Config` | constants, pin numbers | `MOTOR_L_PIN`, `KP_LINE`, `MAX_NODES`, ... |
| `FieldMap` | static graph data | `getNeighbors(n)`, `getEdgeDir(a,b)`, `getEdgeCost(a,b)` |
| `Pathfinder` | shortest-path search | `findPath(start, goal, outArr, &len) -> bool` |
| `Localization` | current node + heading | `currentNode`, `heading`, `onIntersection()`, `onTurnDone(deg)` |
| `PathTracker` | path execution | `start(path, len)`, `update()`, `isDone()` |
| `Drive` | motion primitives | `setSpeed(L,R)`, `turnDeg(deg)`, `driveDist(cm)`, `stop()` |
| `LineFollower` | line PID | `update()` (writes to Drive internally) |
| `Intersection` | node arrival event | `isAtIntersection() -> bool` |
| `ColorReader` | single-cell color | `read() -> ColorEnum` |
| `CameraMosaic` | 3×3 pattern read | `capturePattern(out[9]) -> bool` |
| `Manipulator` | actuator control | `setLift(h)`, `setGripper(a)`, `setArm(a)`, `update()` |
| `Tasks` | one mission goal | `start()`, `update()`, `isDone()`, `abort()` |
| `Mission` | full-run scheduler | `start()`, `update()`, time-guard |
| `StateMachine` | helper | template |

## The two iron rules

### Rule 1 — Everything that takes time has `update()`. Nothing blocks.

Every module that performs ongoing work exposes a non-blocking `update()` method. The main loop calls each one in turn:

```
loop():
   Mission.update()
   PathTracker.update()
   LineFollower.update()
   Manipulator.update()
```

If you need to "wait 100 ms," store a `millis()` timestamp and check elapsed time on each `update()` call. Never write `while (something) { delay(...); }` inside a module that runs during a task.

### Rule 2 — Global singletons via `extern`

```cpp
// Drive.h
class DriveClass { ... };
extern DriveClass Drive;

// Drive.cpp
DriveClass Drive;
```

No `new` for system objects. No passing `Drive&` into every constructor. One instance per module, exposed via `extern`, used directly.

## Where PID lives (and where it doesn't)

| PID loop | Implementation |
|---|---|
| Motor speed → PWM | **STM32 firmware (vendor).** Not ours. |
| Motor position → speed | **STM32 firmware (vendor).** Not ours. |
| IMU yaw → motor commands (turning) | App code, in `Drive.turnDeg` |
| Line offset → wheel speed difference | App code, in `LineFollower.update` |

When `Drive.setSpeed(L, R)` is called, the STM32 ensures the wheels actually hit those speeds. The application's job is to compute *what* those speeds should be.

## Open items to confirm before coding

1. **Matrix library motor API** — exact class name and `setSpeed` signature. Read `examples/` in the GitHub repo.
2. **Line-tracker helper in Matrix lib?** — if yes, decide whether to use it or write our own. If no, the line-follower PID is fully ours to write.
3. **Camera choice** — HuskyLens (officially supported, has built-in color recognition) vs. raw camera + on-device pixel classification. HuskyLens is faster but adds a dependency.
4. **Manipulator mechanism** — undecided. Locks down the parameters of `setLift`, `setGripper`, `setArm`.

## Development priority ladder

| Phase | Goal | Modules in scope |
|---|---|---|
| 1 | Robot follows a single line and stops at intersections | `Drive`, `LineFollower`, `Intersection` |
| 2 | Robot navigates from any node to any node | + `FieldMap`, `Pathfinder`, `Localization`, `PathTracker` |
| 3 | Robot reads colors and the mosaic pattern | + `ColorReader`, `CameraMosaic` |
| 4 | Robot executes one full task end-to-end | + `Tasks` (one task), `Manipulator` (control only) |
| 5 | Full mission with scheduling and time guard | + `Mission`, all `Tasks` |
| 6 | Failure recovery (lost line, missed intersection) | retrofitted into existing modules |

A working Phase 4 with one task scores more than a polished Phase 1.

## Out of scope (this iteration)

- Logging / data recording
- SD card storage
- Replay tooling
- Bluetooth / WiFi telemetry
- Sensor fusion beyond IMU sanity checks against expected heading

## Coding criteria checklist

When writing or reviewing any module, confirm:

- [ ] No `delay()` calls inside `update()` paths
- [ ] No dynamic allocation after `setup()` finishes
- [ ] Module compiles standalone (only depends on layers strictly below it)
- [ ] State stored in module-private static or in the module's singleton instance
- [ ] No reads/writes to `Localization.currentNode` from anywhere except `Localization` itself
- [ ] All public methods are either pure queries (`isDone()`, `read()`) or one-shot commands (`start()`, `setSpeed()`); no method does both
- [ ] Loop iteration count or time bound is provable for any internal `for`/`while`
