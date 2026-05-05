# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Context

This is the **WRO 2026** competition robot codebase. At time of writing the working tree is empty — all new code is built on top of the **MatrixMiniR4** Arduino library and runs on a MATRIX Mini R4 controller (Arduino Uno R4 WiFi + on-board STM32 motor/IMU coprocessor + 6-axis IMU).

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
- Differential-drive helper over two of those: `MiniR4.DriveDC` (`MiniR4DriveDC<1>`); the WRO convention in the example tests is **Left = M2, Right = M3**.
- 4× RC servos: `MiniR4.RC1`–`MiniR4.RC4`.
- IMU: `MiniR4.Motion` (gyro / accel / Euler via `MiniR4Motion::AxisType::{X,Y,Z,Roll,Pitch,Yaw}`).
- OLED: `MiniR4.OLED` (Adafruit_SSD1306, 128×32 @ 0x3D on Wire1).
- I2C ports: `I2C0`–`I2C4` (port 0 is the on-board `Wire`; 1–4 are external on `Wire1`).
- Digital ports `D1`–`D4`, Analog ports `A1`–`A3`, RGB LEDs (×2) on pin 7, Buzzer on pin 6, two buttons (`BTN_UP`, `BTN_DOWN`).
- Power: `MiniR4.PWR.setBattCell(N)` — must match the actual battery (`2` for 18650×2 / 2S, up to `6`).

## Required initialization sequence

Every sketch's `setup()` must follow this order (see `examples/MiniR4_GettingStartedTpl` and `MiniR4_LibraryTest`):

```cpp
#include <MatrixMiniR4.h>

void setup() {
    MiniR4.begin();                 // brings up STM32, IMU, motors, servos, OLED
    Serial.begin(115200);
    MiniR4.PWR.setBattCell(2);      // MUST match physical battery
    // motor reverse, brake, servo direction, DriveDC.begin(L,R,...) , PIDs ...
}
```

`MiniR4.begin()` itself blocks on motor controller readiness and on a firmware-version check — if STM32 firmware is < v6.0 it will halt with an OLED prompt until `BTN_DOWN` is pressed. **Required: library ≥ v1.1.8, STM32 firmware ≥ v6.0.**

## Build & flash

This is an Arduino sketch project, not a CMake/Make project — there are no shell-runnable build/test commands.

- Open the `.ino` in **Arduino IDE** (board: *Arduino UNO R4 WiFi*) and use **Verify** / **Upload**.
- The MatrixMiniR4 library must be installed (Library Manager) or symlinked into the Arduino `libraries/` folder so `#include <MatrixMiniR4.h>` resolves.
- Required Arduino library deps (from `library.properties`): `ArduinoBLE`, `PubSubClient`.
- "Tests" are run on hardware by uncommenting `Task*()` calls in `loop()` — mirror the structure used in `MiniR4_LibraryTest.ino`.

If `arduino-cli` is available, a headless compile looks like:
`arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi <sketch_dir>` and upload with `--upload -p <port>`. Don't assume it's installed — ask first.

## Code conventions (strict)

The user requires **strict OOP** and clean, debuggable code. Match the MatrixMiniR4 style exactly.

1. **One class per concern, one header per class.** File `Foo.h` declares `class Foo`. Use include guards `#ifndef FOO_H / #define FOO_H / #endif // FOO_H`.
2. **Header-only where reasonable.** Most MatrixMiniR4 modules are header-only template classes; follow that pattern for thin wrappers. Only split into `.cpp` when implementation is heavy (see `MMLower`, `MiniR4OLED`, `MiniR4VernierLib`).
3. **Templates for port/ID-bound classes.** When a class is parameterized by a hardware port or motor ID, use a non-type template parameter (`template<uint8_t ID> class MiniR4DC { ... };`) — do **not** pass the ID at runtime if it's known at compile time.
4. **Doxygen comments on every public class and method**, using `@brief`, `@param`, `@return`, `@note`. Match the tone in existing module headers.
5. **Return convention:** `bool` for simple OK/fail; `uint8_t` error code (`0x00` = OK) for multi-state failures; or the strongly-typed `MMLower::RESULT` / `MMLower::Drive_RESULT` enums when calling lower-MCU APIs. Always check the result; on transient failure, retry once with a small `delay()` — that's the pattern used throughout `MMLower` callers.
6. **Naming:** classes `MiniR4<Component>` (or project-specific `WRO<Component>`); public methods `lowerCamelCase`; private members `_snake_or_camel` with leading underscore (consistent with `_id`, `_pin`, `_leds` in the library). Prefer camelCase for most things.
7. **No magic numbers.** Use `#define` or `static constexpr` constants at the top of the header (see `IMU_idle_Period`, `Drive_retry_Period` in `MiniR4DriveDC.h`).
8. **No inline styles in code, no half-finished implementations, no speculative abstractions.** If the library already exposes `MiniR4.M1.setSpeed(...)`, do not wrap it in a one-line method "for symmetry" — call it directly.
9. **Formatting:** the reference repo ships a `.clang-format` (LLVM base, 4-space indent, 100-col limit, `AlignAfterOpenBracket: AlwaysBreak`, custom brace wrapping with braces on a new line after class/struct/enum/function, `PointerAlignment: Left`). Copy that file into this repo if/when adding C++ tooling.

## Project layout convention (when code is added)

There is no committed source yet. When introducing files, follow MatrixMiniR4's shape:

```
WRO2026_code/
├── <SketchName>/
│   └── <SketchName>.ino     // entry, only setup()/loop() and high-level orchestration
├── lib/                      // (optional) reusable WRO-specific classes
│   ├── WROChassis.h          // wraps MiniR4.DriveDC + tuning for our robot
│   ├── WROMission.h          // mission state machine
│   └── WROSensors.h
└── AGENTS.md
```

Keep `setup()` and `loop()` thin — they construct/configure objects and dispatch to class methods. Heavy logic lives in classes, not in free functions in the `.ino`.

## Workflow rules (from user's global AGENTS.md, repeated for emphasis)

- For non-trivial work, run the `plan` skill first, then `execute`.
- Create a feature branch — never commit straight to `main`.
- When debugging, present 2–3 options with tradeoffs **before** picking a fix.
- Use the `code-writer` agent for implementation once a plan is agreed.
- When planning something, always read `ARCHITECTURE.md` and `DECISIONS.md` and `Roadmap.md` first to understand the big picture.
