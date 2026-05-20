# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

This is the **WRO 2026** competition robot codebase. All R4-side code is built on top of the **MatrixMiniR4** Arduino library and runs on a MATRIX Mini R4 controller (Arduino Uno R4 WiFi + on-board STM32 motor/IMU coprocessor + 6-axis IMU). Path planning runs offline in Python; the firmware consumes a precomputed `NEXT_HOP` table.

### What's already built (as of 2026-05-20)

R4-side modules in `WRO2026/`:
- `Drive` — fluent builders, FIFO command queue, T-shape velocity profile, motor-bias calibration, fault state.
- `LineFollower` — outer P+D over `WROLineArray`, fluent enable builder, distance-gated stops, sync helpers, junction-PID freeze.
- `Intersection` — rising-edge detector with debounce, `JunctionType` classification, edge counter for multi-observer reads.
- `Localization` — single-writer current node + IMU-yaw offset; world↔IMU frame conversion.
- `FieldMap` — header-only namespace over `Routes::*`.
- `routes.h` / `routes.cpp` — **auto-generated** by the Python pipeline; holds `NODES`, `NEXT_HOP[N][N]`, `EDGES[N][N]`, `PipelineConfig`.
- `PathTracker` — FSM consuming `NEXT_HOP`, fluent `goTo()` builder + queue, fault state.
- `Mission` — operator DSL (`goTo`/`wait`/`print`/`beep`/`led`) with fault beep-and-flash.
- `EventTrace` — OLED ring buffer (4 events) + edge-triggered event push + 500 ms Serial heartbeat.
- `WROLineArray` — custom 8-channel reflectance line array over 1–2 MXColor I2C boards (drop-in for vendor `MatrixLineTracer`).
- `WROMath` / `StateMachine` — header-only helpers (StateMachine defined but not yet instantiated).

Stubs awaiting Phase 3+ hardware decisions: `Manipulator.h/.cpp` (every method no-op).

Offline tooling in `tools/path_planner/`:
- Pure-Python build pipeline (`validator`, `graph_builder`, `planner`, `compiler`, `header_writer`, `build_routes` CLI).
- `sim/` — Python mirror of the PathTracker FSM for hardware-free verification.
- `pick_xy.py` — Tk GUI for measuring field coordinates off `maps/map.png`.
- `pytest` suite (~9 test files, 70+ tests) wired into GitHub Actions CI.

### Reference library — always consult before writing code

The MatrixMiniR4 library source lives **outside this repo** at:

```
C:\Users\wangd\documents\WRO2026\MatrixMiniR4\
```

- `src/MatrixMiniR4.h` / `MatrixMiniR4.cpp` — top-level facade exposing the `MiniR4` global.
- `src/Modules/` — header-only hardware module classes (`MiniR4DC`, `MiniR4DriveDC`, `MiniR4Motion`, `MiniR4LED`, `MiniR4OLED`, `MiniR4Power`, `MiniR4PWM`, `MiniR4RC`, `MiniR4BTN`, `MiniR4Buzzer`, `MiniR4I2C`, `MiniR4Analog`, `MiniR4DAC`, `MiniR4Tone`, `MMLower`).
- `src/Modules/Sensors/` — sensor drivers (`MiniR4SmartCamReader`, `MiniR4PS2X_lib`, `MiniR4VernierLib`, `MiniR4_MXColorV3`, `MiniR4_MXLaserV2`, `MiniR4_MXGesture`, `MiniR4_MXLineTracer`, `MiniR4_DHT11`, `MiniR4_DS18B20`, `MiniR4_Grove_US`, etc.).
- `src/Util/` — `BaseUtil`, `BitConverter`.
- `examples/` — canonical usage. `MiniR4_LibraryTest` is the most thorough; `4-DriveDC Functions/MiniR4_DriveDC_Test` shows the differential-drive flow used for WRO line/path missions.

**Before introducing any abstraction, search the library first.** A class very likely already exists. Re-read the header to confirm method signatures, `bool`/error-code return conventions, and whether a method is blocking or non-blocking (e.g. `MoveDegs(..., async=false)` blocks until done; `rotateFor` is non-blocking — pair with `ChkRotateEnd`).

## Hardware quick map

- 4× DC motors with encoders: `MiniR4.M1`–`MiniR4.M4` (template `MiniR4DC<ID>`).
- Differential-drive helper: `MiniR4.DriveDC` (`MiniR4DriveDC<1>`). The WRO chassis runs **Left = M2, Right = M3** (`Config::MOTOR_LEFT_ID = 2`, `MOTOR_RIGHT_ID = 3`).
- 4× RC servos: `MiniR4.RC1`–`MiniR4.RC4`.
- IMU: `MiniR4.Motion` (gyro / accel / Euler via `MiniR4Motion::AxisType::{X,Y,Z,Roll,Pitch,Yaw}`).
- OLED: `MiniR4.OLED` (Adafruit_SSD1306, 128×32 @ 0x3D on Wire1).
- I2C ports: `I2C0`–`I2C4` (port 0 is the on-board `Wire`; 1–4 are external on `Wire1`).
- **Line sensor**: `WROLineArray` — 8 reflectance channels per board, 1 or 2 MXColor-protocol boards on `Wire1` (front + optional rear for bidirectional follow). Configured via `Config::LINE_BOARD_COUNT`, `LINE_FRONT_I2C_PORT`, `LINE_REAR_I2C_PORT`, `LINE_LED_PWM`, `LINE_CALIB_LOW[2][8]`, `LINE_CALIB_HIGH[2][8]`. Two-layer calibration (firmware illumination + software per-channel baselines) — see `WROLineArray.h` header docstring.
- Digital ports `D1`–`D4`, Analog ports `A1`–`A3`, RGB LEDs (×2) on pin 7, Buzzer on pin 6, two buttons (`BTN_UP`, `BTN_DOWN`).
- Power: `MiniR4.PWR.setBattCell(N)` — must match the actual battery (`2` for 18650×2 / 2S, up to `6`).

## Required initialization sequence

Every sketch's `setup()` follows this order (see `WRO2026/WRO2026.ino`):

```cpp
#include <MatrixMiniR4.h>
#include "Config.h"
// ... all module headers

void setup() {
    MiniR4.begin();                              // STM32, IMU, motors, servos, OLED
    Serial.begin(115200);
    MiniR4.PWR.setBattCell(Config::BATT_CELLS);  // MUST match physical battery

    LineArray.begin();      // BEFORE LineFollower — LineFollower binds onto it
    Drive.begin();
    LineFollower.begin();
    Intersection.begin();
    Manipulator.begin();
    Localization.begin();
    PathTracker.begin();
    Mission.begin();
    EventTrace.begin();

    Drive.setTickCallback(cooperativeTick);          // sync-blocking spins
    PathTracker.setTickCallback(cooperativeTick);    //   re-enter every module
    LineFollower.setTickCallback(cooperativeTick);   //   per spin iteration
}
```

`MiniR4.begin()` itself blocks on motor controller readiness and on a firmware-version check — if STM32 firmware is < v6.0 it will halt with an OLED prompt until `BTN_DOWN` is pressed. **Required: library ≥ v1.1.8, STM32 firmware ≥ v6.0.**

## Build, flash, test

### Arduino firmware

- Open `WRO2026/WRO2026.ino` in **Arduino IDE** (board: *Arduino UNO R4 WiFi*) and use **Verify** / **Upload**.
- The MatrixMiniR4 library must be installed (Library Manager) or symlinked into the Arduino `libraries/` folder so `#include <MatrixMiniR4.h>` resolves.
- Required Arduino library deps (from `library.properties`): `ArduinoBLE`, `PubSubClient`.
- Headless compile via `arduino-cli` (if installed):
  ```
  arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/
  ```

### Python pipeline (offline path planner)

Regenerate `WRO2026/routes.h` + `WRO2026/routes.cpp` whenever `tools/path_planner/field_data.py` changes:

```
uv run python tools/path_planner/build_routes.py
```

Run the test suite (validator, graph builder, Dijkstra, compiler, header writer, simulator):

```
uv run pytest tools/path_planner/tests/
```

### CI

GitHub Actions runs the Python pytest suite and an Arduino compile check on every push and pull request (see `.github/workflows/`).

## Code conventions (strict)

The user requires **strict OOP** and clean, debuggable code. Match the MatrixMiniR4 style exactly.

1. **One class per concern, one header per class.** File `Foo.h` declares `class Foo`. Use include guards `#ifndef FOO_H / #define FOO_H / #endif // FOO_H`.
2. **Header-only where reasonable.** Most MatrixMiniR4 modules are header-only template classes; follow that pattern for thin wrappers. Only split into `.cpp` when implementation is heavy (see `Drive`, `LineFollower`, `PathTracker` in this repo).
3. **Templates for port/ID-bound classes.** When a class is parameterized by a hardware port or motor ID, use a non-type template parameter (`template<uint8_t ID> class MiniR4DC { ... };`) — do **not** pass the ID at runtime if it's known at compile time.
4. **Doxygen comments on every public class and method**, using `@brief`, `@param`, `@return`, `@note`. Match the tone in existing module headers.
5. **Return convention:** `bool` for simple OK/fail; `uint8_t` error code (`0x00` = OK) for multi-state failures; or the strongly-typed `MMLower::RESULT` / `MMLower::Drive_RESULT` enums when calling lower-MCU APIs. Always check the result; on transient failure, retry once with a small `delay()` — that's the pattern used throughout `MMLower` callers.
6. **Fluent builder for any verb with 3+ knobs.** `Drive.driveDist(30).setMode(...).tShape().setBrake(false)` — see [DECISIONS.md](./DECISIONS.md). Direct call for 0–2 knobs.
7. **Naming:** classes `MiniR4<Component>` (or project-specific `WRO<Component>`); public methods `lowerCamelCase`; private members `_snake_or_camel` with leading underscore (consistent with `_id`, `_pin`, `_leds` in the library). Prefer camelCase for most things.
8. **No magic numbers.** Use `#define` or `static constexpr` constants at the top of the header or in `Config.h` (see `LINE_KP`, `TSHAPE_ACCEL_PWM_PER_CM`, etc.).
9. **No inline styles in code, no half-finished implementations, no speculative abstractions.** If the library already exposes `MiniR4.M1.setSpeed(...)`, do not wrap it in a one-line method "for symmetry" — call it directly.
10. **Formatting:** the reference repo ships a `.clang-format` (LLVM base, 4-space indent, 100-col limit, `AlignAfterOpenBracket: AlwaysBreak`, custom brace wrapping with braces on a new line after class/struct/enum/function, `PointerAlignment: Left`).

## Project layout

```
WRO2026_Code/
├── WRO2026/                  # Arduino sketch — flashes to the R4
│   ├── WRO2026.ino           # setup() + cooperativeTick() + mission DSL
│   ├── Config.h              # all constants, pin maps, PID gains, calibration tables
│   ├── Drive.h / .cpp        # drivebase facade + fluent builders + queue + T-shape
│   ├── LineFollower.h / .cpp # P+D over WROLineArray + distance gate + sync helpers
│   ├── Intersection.h / .cpp # rising-edge detector + JunctionType + edge counter
│   ├── Localization.h / .cpp # single-writer node + IMU-yaw offset
│   ├── FieldMap.h            # header-only namespace over Routes::*
│   ├── routes.h / .cpp       # AUTO-GENERATED graph data — do not hand-edit
│   ├── PathTracker.h / .cpp  # FSM consuming NEXT_HOP + fluent goTo() builder
│   ├── Mission.h / .cpp      # operator DSL (goTo/wait/print/beep/led)
│   ├── Manipulator.h / .cpp  # stub — Phase 3 mechanism TBD
│   ├── EventTrace.h / .cpp   # OLED ring + Serial heartbeat
│   ├── WROLineArray.h / .cpp # 8-ch reflectance array over MXColor boards
│   ├── WROMath.h             # free helpers (cmToEncDeg, presetToPwm, ...)
│   └── StateMachine.h        # generic FSM template (header-only, currently unused)
│
├── tools/path_planner/       # offline Python pipeline (host machine, not on R4)
│   ├── field_data.py         # hand-authored field input
│   ├── validator.py          # strict schema validation
│   ├── graph_builder.py      # NetworkX DiGraph with C-space inflation
│   ├── planner.py            # all-pairs Dijkstra → NEXT_HOP table
│   ├── compiler.py           # path → primitive tuples
│   ├── header_writer.py      # emits routes.h + routes.cpp (SHA256-stamped)
│   ├── build_routes.py       # CLI orchestrator
│   ├── visualizer.py         # matplotlib map.png renderer
│   ├── pick_xy.py            # Tk GUI to measure field coordinates
│   ├── sim/                  # pure-Python PathTracker mirror for verification
│   ├── tests/                # pytest suite + goldens
│   └── maps/                 # map.png + field-mat photos
│
├── docs/                     # design history + deferred features
│   ├── decisions/            # ADRs (one file per major decision)
│   ├── archive/              # shipped plans + audits + superseded specs
│   └── PHASE2_DEFERRED.md    # what's still deferred from Phase 2
│
├── README.md                 # entrypoint — points into the other docs
├── ARCHITECTURE.md           # module map + dependency graph + iron rules
├── DECISIONS.md              # index into docs/decisions/
├── Roadmap.md                # phase plan + exit criteria + benchmarks
├── CLAUDE.md                 # this file (guidance for Claude Code)
└── AGENTS.md                 # guidance for Codex (kept in sync with CLAUDE.md)
```

Keep `setup()` and `loop()` thin — they construct/configure objects and dispatch to class methods via `cooperativeTick()`. Heavy logic lives in classes, not in free functions in the `.ino`.

## Documenting design decisions (ADRs)

Load-bearing design decisions live as one file per decision under [`docs/decisions/`](./docs/decisions/), indexed by [`DECISIONS.md`](./DECISIONS.md). When you make a decision that crosses module boundaries, changes an existing rule, or sets a project-wide convention, write a new ADR — don't bury the rationale in a commit message that future readers won't find.

**When to write one**

- New design rule that crosses module boundaries (e.g., "all builders block by default").
- Change to an existing rule — write a **new** ADR with the next number, mark the old one **Superseded by ADR-NNN**, update cross-links in both files.
- A "rule" that lasted long enough to be worth citing. One-PR conventions don't need an ADR; project-wide constraints do.

**Required shape** (stay ~30–60 lines per file):

```markdown
# ADR-NNN: <one-line decision title>

**Status:** Implemented | Partially implemented | Pending implementation | Deferred | Superseded by ADR-XXX
**Date decided:** YYYY-MM (phase)

## Rule
<1-3 sentences — the rule as it is in force today>

## Why
<1-2 paragraphs — the motivation. Skip the implementation play-by-play; that lives in commits.>

## How to apply
<Where/when this rule kicks in. Cite specific module / API surfaces.>

## Trigger to revisit
<The specific condition that would cause us to re-open this decision.>

## See also
<Links to related ADRs, code modules, or other docs.>
```

**Rules**

- ADR numbers are assigned **in order of first appearance**, not by topic. Never renumber.
- Strip implementation history (reviewer-found bugs, struct sizes, "first cut was X" narratives). That belongs in commit messages / PR descriptions.
- After adding an ADR, update the index in `DECISIONS.md` (one line in the appropriate section + bump the status-summary table).
- If you find yourself making a one-off implementation decision while writing code, ask whether it's project-wide enough to deserve an ADR. If yes, write the ADR in the same PR that lands the code.

## Workflow rules (from user's global CLAUDE.md, repeated for emphasis)

- For non-trivial work, run the `plan` skill first, then `execute`.
- Create a feature branch — never commit straight to `main`.
- When debugging, present 2–3 options with tradeoffs **before** picking a fix.
- Use the `code-writer` agent for implementation once a plan is agreed.
- When planning something, always read `ARCHITECTURE.md` and `DECISIONS.md` and `Roadmap.md` first to understand the big picture.
