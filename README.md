# WRO 2026 — Mosaic Masters

Autonomous robot codebase for **WRO RoboMission Senior 2026 — Mosaic Masters**.
One 2-minute run on a fixed field. Up to 233 points across tool delivery,
mosaic-tile placement, cement-block delivery, and brick-trowel return.

Hardware: **Matrix Mini R4** — Arduino UNO R4 WiFi + STM32 motor/IMU
co-processor + 6-axis IMU. Language: **C++** via the Arduino IDE, on top of
the **MatrixMiniR4** vendor library.

For deeper reading, in this order:
- [`ARCHITECTURE.md`](./ARCHITECTURE.md) — module map and the two iron rules.
- [`DECISIONS.md`](./DECISIONS.md) — every load-bearing design decision +
  why it is the way it is.
- [`Roadmap.md`](./Roadmap.md) — phase plan, exit criteria, benchmarks.

## Goals

**Destroy every other team in every WRO competition we enter, by
overwhelming coding.** Whatever the field, whatever the rules, the bot
should out-think and out-execute the room.

What that looks like in engineering terms:

1. **Reliable navigation first.** Phase 1 (line follow + 90° turn) has to
   be rock-solid before any task logic gets written. Every higher layer
   assumes intersections are ground truth and turns land where they should.
2. **The line graph is the ground truth.** No dead-reckoning. The robot
   re-anchors at every intersection so a missed turn never silently
   compounds across the 2-minute run.
3. **Score-per-second wins.** The `Mission` scheduler picks tasks by
   expected score ÷ expected time, with a hard 2-minute time guard that
   triggers return-to-start around `t = 100 s`.
4. **Programmable, not scripted.** New tasks compose existing navigation
   and manipulator primitives. They never invent fresh motion sequences.
5. **A working Phase 4 with one task beats a polished Phase 1.** The
   roadmap is a strict ladder — resist polishing low layers when the next
   phase is still empty.

Deadlines: Regional `2026-07-21`, National `2026-08-24`, World final
`2026-11-30`.

## Codebase concepts

### Three layers, top-down only

```
planning   (Mission, Tasks, PathTracker, Pathfinder)
   |
   v
map        (FieldMap, Localization, routes.h)
   |
   v
perception + actuation
           (LineFollower, Intersection, Camera (mosaic + color),
            Drive, Manipulator)
   |
   v
MatrixMiniR4 vendor library (motors, IMU, OLED, I2C, servos)
```

Calls only flow downward. Lower layers never call upper layers. Modules
talk to each other through global singletons exposed via `extern` — no
dependency injection, no class hierarchies for system structure.

### The two iron rules

**Rule 1 — Nothing blocks.** Every long-running module exposes a
non-blocking `update()` that the main loop ticks once per iteration. Time
is `millis()`-based. The only `delay()` sites allowed in the codebase are
command-time orchestration (`_drainBlocking`, `calibrateMotorBias`,
blocking + tShape spins).

**Rule 2 — Global singletons via `extern`.** One instance per module:

```cpp
// Drive.h
class DriveClass { ... };
extern DriveClass Drive;

// Drive.cpp
DriveClass Drive;
```

### Drive — sync-default fluent builders + a command queue

Distance-, time-, and turn-bounded `Drive` verbs return a fluent builder
that runs in its destructor. The defaults are **SYNC + tShape +
blocking**, which means hand-written missions read top-to-bottom with no
`waitUntilIdle()` boilerplate:

```cpp
Drive.driveDist(30);                                       // SYNC + tShape, blocks until done
Drive.driveDist(20).setMode(DriveMode::GYRO).tShape();     // gyro-locked travel
Drive.turnDeg(-90);                                        // blocks until done
Drive.driveDist(30).async();                               // enqueues, returns immediately
```

- `.setMode(RAW|SYNC|GYRO)` and `.setStartSpeed`/`.setEndSpeed` opt out
  of the defaults per call.
- `.async()` opts out of blocking. The auto path planner ticks `loop()`
  and polls `Drive.isIdle()`.
- A small FIFO queue (`Config::DRIVE_QUEUE_CAPACITY = 8`) serialises
  queued moves so chained `.async()` calls dispatch in source order.

The rule for the whole project: **any verb with 3+ knobs is a fluent
builder; 0–2 knobs stay direct.** See [`DECISIONS.md`](./DECISIONS.md) —
ADR-012 (fluent builder), ADR-015 (drive queue), ADR-016 (sync-default),
ADR-017 (SYNC + tShape default) for the full story.

### Localization is the single source of truth

`Localization.getCurrentNode()` and `getHeadingDeg()` are the single
source of truth for "where am I?". Writes happen only through
`Localization.onIntersection(advanceTo)` and `onDeadReckonedAdvance(advanceTo)`,
both called from `PathTracker`. Heading is computed live from IMU yaw +
offset — no cached copy means no staleness. Every other module reads
from `Localization` and never writes to it. If position is ever wrong,
there is exactly one place to look.

### Path planning is offline

`tools/path_planner/` is a Python pipeline that turns
hand-authored field data (`field_data.py`) into `WRO2026/routes.h` +
`WRO2026/routes.cpp`. The robot never runs Dijkstra at runtime — it walks
a pre-computed next-hop table that the planner emits. The `routes.h`
banner embeds `sha256(field_data.py)` so reviewers can spot a stale
header at a glance.

### Mission DSL

Missions are hand-authored in `buildMission()` inside `WRO2026.ino`,
using a chainable fluent DSL:

```cpp
Mission
    .print("MISSION GO")
    .led(0, 0, 255, 200)
    .goTo("B")
    .beep(1000, 150)
    .wait(500)
    .goTo("G")
    .goTo("A")
    .print("DONE");
Mission.start();
```

Verbs available today: `goTo`, `wait`, `print`, `beep`, `led`. See
`Mission.h` for exact signatures.

## Coding rules

The strict set lives in [`CLAUDE.md`](./CLAUDE.md) and the criteria
checklist at the end of [`ARCHITECTURE.md`](./ARCHITECTURE.md). The
highlights:

- **One class per concern, one header per class.** Include guards
  `#ifndef FOO_H / #define FOO_H / #endif // FOO_H`.
- **Header-only where reasonable.** Templates for port- or ID-bound
  classes (`template<uint8_t ID> class MiniR4DC`).
- **Doxygen on every public class and method** — `@brief`, `@param`,
  `@return`, `@note`. Match the tone of MatrixMiniR4 headers.
- **Return convention:** `bool` for OK/fail; `uint8_t` (`0x00` = OK) for
  multi-state failures; `MMLower::RESULT` / `MMLower::Drive_RESULT` for
  lower-MCU calls. On transient failure, retry once.
- **No magic numbers.** Constants in `Config.h` or `static constexpr` at
  the top of the header.
- **No `delay()` inside `update()` paths.** Time = `millis()` deltas.
- **No dynamic allocation after `setup()` finishes.** No `new`, no
  `std::vector`, no `std::map`. Fixed-size arrays.
- **`Localization.currentNode` writes only from `Localization` itself.**
- **Fluent builders for any verb with 3+ knobs; direct calls for 0–2.**
- **Names:** classes `WRO<Component>` / `MiniR4<Component>`; public
  methods `lowerCamelCase`; private members `_camelOrSnake` with leading
  underscore.
- **Formatting:** LLVM base, 4-space indent, 100-col, braces on a new
  line after class/struct/enum/function, `PointerAlignment: Left`.
- **Workflow:** feature branches only — never commit to `main`. For
  non-trivial work run the `plan` skill first, then `execute`. When
  debugging, present 2–3 options with tradeoffs **before** picking a fix.

## Repo layout

```
WRO2026_code/
├── WRO2026/                  Arduino sketch — firmware that runs on the robot.
│   ├── WRO2026.ino           setup() + loop() + the hand-typed mission DSL.
│   ├── Config.h              pins, PID gains, MAX_NODES, default speeds.
│   ├── Drive.{h,cpp}         fluent builders, command queue, motor bias.
│   ├── LineFollower.{h,cpp}  PID + distance-gated stop with tShape.
│   ├── Intersection.{h,cpp}  intersection detection from the sensor array.
│   ├── Localization.{h,cpp}  currentNode + heading, single source of truth.
│   ├── PathTracker.{h,cpp}   walks the routes table node-by-node.
│   ├── FieldMap.h            field graph helpers over routes.h.
│   ├── routes.{h,cpp}        GENERATED — do not hand-edit.
│   ├── Mission.{h,cpp}       fluent DSL: goTo / wait / print / beep / led.
│   ├── Manipulator.{h,cpp}   setLift / setGripper / setArm (control only).
│   ├── EventTrace.{h,cpp}    OLED + serial event log.
│   ├── StateMachine.h        header-only state-machine helper.
│   └── WROMath.h             angle wrap, clamp, etc.
│
├── tools/path_planner/       offline Python pipeline.
│   ├── field_data.py         hand-authored nodes, edges, projections.
│   ├── build_routes.py       validator → graph → planner → header writer.
│   ├── sim/                  PathTracker simulator (chassis + runtime).
│   ├── tests/                pytest suite (run from repo root).
│   └── README.md             full pipeline docs + CLI flags + exit codes.
│
├── docs/
│   ├── decisions/            one ADR per load-bearing design decision.
│   ├── archive/              shipped plans + audits + superseded specs.
│   └── PHASE2_DEFERRED.md    Phase 2.5 work that's still deferred.
├── ARCHITECTURE.md           module map + dependency graph + iron rules.
├── DECISIONS.md              index into docs/decisions/.
├── Roadmap.md                phase plan + exit criteria + benchmarks.
└── CLAUDE.md / AGENTS.md     coding rules for AI assistants on this repo.
```

## How to use

### Arduino — build and flash

Open `WRO2026/WRO2026.ino` in **Arduino IDE**, select board *Arduino UNO
R4 WiFi*, then **Verify** / **Upload**. The MatrixMiniR4 library must be
installed in the Arduino `libraries/` folder so `#include
<MatrixMiniR4.h>` resolves. Required Arduino library deps: `ArduinoBLE`,
`PubSubClient`.

VS Code tasks (PowerShell-driven, defined in `.vscode/tasks.json`) make
this one keystroke:

| Task                         | What it does                                                       |
| ---------------------------- | ------------------------------------------------------------------ |
| `Arduino: Verify WRO2026`    | Auto-runs `Build Routes` then `arduino-cli compile`. Default build (Ctrl+Shift+B). |
| `Arduino: Upload WRO2026`    | Verify → `arduino-cli upload` to the configured port.              |
| `Arduino: List Boards`       | `arduino-cli board list` — find the COM port for your bot.         |

Tool paths are read from `wro2026.*` keys in `.vscode/settings.json`.

### Path planner — regenerate `routes.h` / `routes.cpp`

```powershell
uv run python tools/path_planner/build_routes.py
```

That regenerates `WRO2026/routes.{h,cpp}` and a sanity-check
`tools/path_planner/output/map.png`. Workflow when you edit the field:

1. Edit `tools/path_planner/field_data.py`.
2. Re-run `build_routes.py`.
3. Open `output/map.png` and eyeball the geometry — nodes in the right
   places, blind edges hopping the right gaps, no edge clipping an
   obstacle.
4. Commit `field_data.py`, `routes.h`, and `routes.cpp` **in the same
   commit**. If the SHA256 in the `routes.h` banner ≠ `sha256(field_data.py)`,
   the commit is incomplete.

Full CLI flags, exit codes, and pipeline architecture:
[`tools/path_planner/README.md`](./tools/path_planner/README.md).

### Simulator — bench-test PathTracker without hardware

`tools/path_planner/sim/` runs the same node-by-node PathTracker
semantics as the firmware, in Python. Great for catching bad
`field_data.py` edits before flashing.

```powershell
uv run python tools/path_planner/sim/demo.py --goals B,G,D,A
uv run python tools/path_planner/sim/demo.py --goals B,G,D,A --plot
```

`Sim: Run PathTracker Demo` and `Sim: Plot PathTracker Demo` in VS Code
are the same calls with `wro2026.sim.goals` pinned as the default goal
list.

### Tests

All Python tests live under `tools/path_planner/tests/` and **must be run
from the repo root** — the imports use the `tools.path_planner.*` form:

```powershell
uv run --extra dev pytest tools/path_planner/tests/ -v
```

(Or `python -m pytest tools/path_planner/tests/ -v` with a plain venv.)

There is no host-side test runner for the Arduino code. Firmware "tests"
are hardware benchmarks per phase — see `Roadmap.md` section 13 ("Test
Fixture & Measurement Plan") for the exact setups and pass criteria.

### Author a mission + run it on the bench

1. Edit `buildMission()` in `WRO2026/WRO2026.ino`. Chain `Mission.<verb>(...)`
   calls, end with `Mission.start()`.
2. Verify + upload (Arduino IDE or VS Code tasks).
3. Power on the robot. `setup()` runs every module's `begin()`. OLED
   shows `BOOT OK` (or `BOOT FAIL` — read serial to find out why).
4. Place the chassis on `Routes::START_NODE` (`A`) facing
   `Config::START_HEADING_DEG`.
5. Press **`BTN_UP`** once. The IMU yaw offset zeros, `Mission.start()`
   arms the queue, the run begins.
6. Serial (`115200`) and OLED log every state change plus a periodic
   `HB t=... yaw=... node=...` heartbeat.
7. On a fault (`UNREACHABLE`, `DRIVE_FAULT`, unknown node name) the
   chassis stops, the queue drops, the buzzer beeps 5× at 1 kHz, and the
   RGB LEDs flash red. Press **`BTN_DOWN`** to acknowledge and reset.

## Quick links

- Hardware quick map and required init sequence: [`CLAUDE.md`](./CLAUDE.md).
- Path planner pipeline: [`tools/path_planner/README.md`](./tools/path_planner/README.md).
- Design decisions (ADRs): [`docs/decisions/`](./docs/decisions/).
- Phase plans + archived audits: [`docs/archive/`](./docs/archive/).
- Vendor library reference (out-of-repo):
  `C:\Users\wangd\documents\WRO2026\MatrixMiniR4\`.
