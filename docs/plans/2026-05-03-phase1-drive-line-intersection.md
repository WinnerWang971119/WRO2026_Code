# Phase 1 — Drive, Line Following & Intersections (Code-Only) Plan

**Status:** Approved 2026-05-03. Awaiting execute.

**Goal:** Author the complete Phase 1 software stack — `Config`, `Drive` (rich movement primitives), `LineFollower`, `Intersection`, `Manipulator` placeholder, OLED event trace, and the `WRO2026.ino` entry — wrapping `MatrixMiniR4` for an unbuilt robot, so the moment the chassis is ready every primitive is callable and tunable from `Config.h`.

**Approach:** Follow `ARCHITECTURE.md`'s flat module tree at the repo root. Wrap `MiniR4.DriveDC` (Move/MoveSync/MoveGyro/TurnGyro families) into a `Drive` singleton that exposes both raw, encoder-bounded, time-bounded, gyro-locked, and turn primitives in async-with-`update()` and blocking variants. Wrap `MatrixLineTracer.getError()` and `getJunctionType()` for `LineFollower` and `Intersection`. Every constant — pin, motor ID, reverse flag, PID gains, wheel geometry, cruise speed, I2C port — lives in `Config.h` so the same code targets either LEGO or TT motors and either MXLineTracer-port choice without source edits. Verification: `arduino-cli compile` (if available) + manual review against MatrixMiniR4 headers.

## Scope

- **In scope:**
  - `Config.h` — single source of truth for pins, IDs, reverse flags, motor profile, PID gains (MoveSync/MoveGyro/TurnGyro/Line), wheel geometry (diameter, track, encoder PPR), cruise speed, MXLineTracer I2C port + threshold + per-channel weights, OLED line capacity, lost-line behavior flag.
  - `Drive.h`/`.cpp` (singleton `Drive`) — rich movement API:
    - Continuous: `setSpeed(L,R)` (raw PWM via `Move`), `setSpeedSync(L,R)` (`MoveSync`), `forwardGyro(power)` (`MoveGyro` holding current yaw), `stop(brakeMode)`.
    - Distance-bounded: `driveDist(cm)`, `driveDistSync(cm)`, `driveDistGyro(cm, power)` — convert cm to encoder degrees via `Config::WHEEL_DIAM_MM`, dispatch to `MoveDegs` / `MoveSyncDegs` / `MoveGyroDegs` with `async=true`.
    - Time-bounded: `driveTime(power, sec)`, `driveTimeSync(L,R,sec)`, `driveTimeGyro(power, sec)`.
    - Turns (relative + absolute, async + blocking, both naming clear):
      - `turnDeg(deg)` — relative, async (default).
      - `turnDegBlocking(deg)` — relative, blocking.
      - `turnToYaw(yaw)` — absolute, async.
      - `turnToYawBlocking(yaw)` — absolute, blocking.
    - Lifecycle: `begin()`, `update()` (polls `isPrevTaskDone()` and updates `_moveDone`/`_turnDone` flags), `isMoveDone()`, `isTurnDone()`, `getEncL()`, `getEncR()`, `resetEnc()`, `getYaw()`, `holdYaw()` (snapshots current yaw as the gyro reference for `forwardGyro`).
    - Yaw wrap-around helper: `_normalizeYaw(deg)` keeps targets in `[-360, +360]` per `TurnGyro` spec.
  - `LineFollower.h`/`.cpp` (singleton `LineFollower`) — `begin()`, `update()` (no `delay()`), `setEnabled(bool)`, `isLineLost()`, `getRawError()`. Uses `MatrixLineTracer.getError()` → P/D loop → `Drive.setSpeed`. On line lost: `Drive.stop(true)` + raise `_lostLine` flag.
  - `Intersection.h`/`.cpp` (singleton `Intersection`) — `begin()`, `update()`, `isAtIntersection()` (edge-triggered, true once per detection), `getLastType()` (`NONE`/`LEFT`/`RIGHT`/`T_OR_CROSS`/`UNKNOWN` enum mirroring `getJunctionType()`).
  - `Manipulator.h`/`.cpp` (singleton `Manipulator`) — empty stubs: `begin()`, `update()`, `setLift(uint8_t)`, `setGripper(uint8_t)`, `setArm(uint8_t)`, `isDone()`. All no-ops, doxygen `@todo Phase 3`.
  - `StateMachine.h` — header-only templated helper (per architecture tree) usable by Phase 4 `Tasks`. Minimal: enum-templated state, `enter`/`update`/`exit` hooks.
  - `EventTrace.h`/`.cpp` (singleton `EventTrace`, owns OLED trace) — small ring buffer of last 4 short event strings, `pushEvent(const char*)`, `update()` re-renders if dirty. Polls `LineFollower.isLineLost()`/`Intersection.isAtIntersection()`/`Drive.isTurnDone()` edges and pushes them. (P1.8 deliverable.)
  - `WRO2026.ino` — `setup()` calls `MiniR4.begin()`, `PWR.setBattCell(Config::BATT_CELLS)`, `Drive.begin()`, `LineFollower.begin()`, `Intersection.begin()`, `Manipulator.begin()`, `EventTrace.begin()`. `loop()` calls each module's `update()`. A `#ifdef DEMO_MODE` block runs a small line-follow + turn sequence on `BTN_UP`.
  - Top-level `.clang-format` copied from `MatrixMiniR4` repo (LLVM, 4-space, 100-col).
  - `arduino-cli` compile check if installed; otherwise documented manual verification.
- **Out of scope:**
  - Hardware bring-up (P0), PID tuning on a physical strip (P1.5 hardware step), manipulator mechanism design (P1.7) — all gated on hardware.
  - Phase 2 modules: `FieldMap`, `Pathfinder`, `Localization`, `PathTracker`.
  - Phase 3 modules: `ColorReader`, `CameraMosaic`.
  - Phase 4 modules: `Tasks`, `Mission`, recovery branches.
  - Logging / SD / WiFi telemetry (per `DECISIONS.md`).
  - Any unit-test harness on host PC (deferred — code-only verification this round).

## Decisions

- **File layout:** flat tree at repo root, sketch dir `WRO2026/`, sketch file `WRO2026/WRO2026.ino` — matches `ARCHITECTURE.md` exactly.
- **Naming:** `class DriveClass; extern DriveClass Drive;` (and equivalents) — matches architecture's iron rule 2 example. No `WRO` prefix.
- **Drive API:** ships both async and blocking turn variants with explicit names (`turnDeg` / `turnDegBlocking`, `turnToYaw` / `turnToYawBlocking`). Blocking versions are documented as "command-time only — do not call from inside any `update()` chain".
- **Line PID:** uses `MatrixLineTracer.getError()` → P + D loop into `Drive.setSpeed(cruise + corr, cruise − corr)`. Gains live in `Config::LINE_KP/KD`. Default `KI = 0` (per library docs we follow same convention). Switchable from raw 10-channel array later by re-implementing `LineFollower.update()` only — no API change.
- **Intersection events:** edge-triggered. Internally tracks "was at junction last call"; returns true only on `false → true` transition.
- **Lost line:** `Drive.stop(brake=true)` + `_lostLine = true`. Phase 4 owns recovery; Phase 1 just halts safely.
- **Yaw convention:** `Drive` reads `MiniR4.Motion.getEuler(Yaw)`; `turnDeg(rel)` snapshots current yaw and calls `TurnGyro(power, normalize(yaw + rel), mode, brake, async)`. `_normalizeYaw` keeps targets in `[-360, +360]`.
- **Wheel/encoder geometry:** `Config::WHEEL_DIAM_MM`, `Config::TRACK_MM`, `Config::ENCODER_PPR_DEG_PER_REV` — initial placeholder values clearly tagged `TODO: measure`. Conversion `cmToEncDeg(cm) = (cm * 10 / (PI * WHEEL_DIAM_MM)) * 360`.
- **Motor profile:** Default to LEGO motor PIDs from the example (`MoveSyncPID(0.002, 0, 0.004)`, `MoveGyroPID(0.52, 0, 0.15)`, `TurnGyroPID(22.75, 0.02, 1.54)`). Switchable to TT via `#define MOTOR_PROFILE_TT` in `Config.h`.
- **Battery:** `Config::BATT_CELLS = 2`.
- **Manipulator:** stub-only this phase. Files exist so the module slot is reserved.
- **Verification:** attempt `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`. If `arduino-cli` is absent, fall back to manual cross-check of every `MiniR4.*` call against the library headers in `C:\Users\wangd\documents\WRO2026\MatrixMiniR4\`.
- **Branch:** new feature branch `phase1/drive-line-intersection` per global CLAUDE.md.

## Acceptance Criteria

- [ ] AC1 — All eight files (`Config.h`, `Drive.h/.cpp`, `LineFollower.h/.cpp`, `Intersection.h/.cpp`, `Manipulator.h/.cpp`, `EventTrace.h/.cpp`, `StateMachine.h`, `WRO2026.ino`) exist at the layout above.
- [ ] AC2 — `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` succeeds (or manual review note explaining why arduino-cli is unavailable, plus a per-symbol audit log).
- [ ] AC3 — `Drive` exposes the full method set listed in Scope; every method has doxygen with `@brief @param @return @note`.
- [ ] AC4 — No `delay()` call appears inside any `update()` method body in any module file (grep-verified).
- [ ] AC5 — Every tunable constant (pin, ID, gain, geometry, cruise speed, MXLineTracer threshold/weights, lost-line flag) lives in `Config.h`. No magic numbers in module bodies (`#define`/`constexpr` only).
- [ ] AC6 — `Drive.turnDeg(deg)` is async-by-default; `Drive.turnDegBlocking(deg)` exists with identical signature except blocking; `Drive.turnToYaw` and `Drive.turnToYawBlocking` both exist; yaw wrap-around handled by a private `_normalizeYaw` helper called from all four.
- [ ] AC7 — `LineFollower.update()` is non-blocking; on line lost it calls `Drive.stop(true)` and sets `_lostLine = true`; `isLineLost()` returns the flag.
- [ ] AC8 — `Intersection.isAtIntersection()` is edge-triggered (returns true only on transition from no-junction to junction); `getLastType()` reflects the type captured at that transition.
- [ ] AC9 — `Manipulator` files compile as no-op stubs with TODO doxygen and reserved method signatures.
- [ ] AC10 — `EventTrace` pushes events for: line lost, line reacquired, intersection detected (with type), turn start, turn done; OLED ring buffer holds last 4 events; redraws only when dirty.
- [ ] AC11 — `WRO2026.ino` `setup()` follows the required init sequence (`MiniR4.begin` → `Serial.begin(115200)` → `PWR.setBattCell` → motor reverse + `DriveDC.begin` + PIDs → module begins). `loop()` is a clean `update()` chain with no business logic.
- [ ] AC12 — `.clang-format` copied to repo root from MatrixMiniR4.
- [ ] AC13 — Branch `phase1/drive-line-intersection` is open with one commit per logical task; nothing pushed to `main`.

## Data Model

**Config.h shape (constexpr / #define, single header, no `.cpp`):**

```cpp
namespace Config {
    // Battery
    constexpr uint8_t BATT_CELLS = 2;

    // Motor IDs and direction
    constexpr uint8_t MOTOR_LEFT_ID  = 2;     // M2
    constexpr uint8_t MOTOR_RIGHT_ID = 3;     // M3
    constexpr bool    MOTOR_LEFT_REVERSE  = true;
    constexpr bool    MOTOR_RIGHT_REVERSE = false;

    // Motor profile (LEGO default; #define MOTOR_PROFILE_TT to switch)
    constexpr uint16_t MOTOR_PPR = 360;
    constexpr uint16_t MOTOR_RPM = 300;

    // PID gains
    constexpr float MOVESYNC_KP = 0.002f, MOVESYNC_KI = 0.0f, MOVESYNC_KD = 0.004f;
    constexpr float MOVEGYRO_KP = 0.52f,  MOVEGYRO_KI = 0.0f, MOVEGYRO_KD = 0.15f;
    constexpr float TURNGYRO_KP = 22.75f, TURNGYRO_KI = 0.02f, TURNGYRO_KD = 1.54f;
    constexpr float LINE_KP = 8.0f, LINE_KI = 0.0f, LINE_KD = 4.0f;  // TODO: tune on strip

    // Geometry (TODO: measure on real chassis)
    constexpr float WHEEL_DIAM_MM = 60.0f;
    constexpr float TRACK_MM      = 150.0f;

    // Cruise / line following
    constexpr int16_t CRUISE_SPEED      = 50;     // -100..100
    constexpr int16_t LINE_MAX_CORR     = 40;
    constexpr uint8_t LINE_THRESHOLD    = 60;     // MXLineTracer threshold

    // MXLineTracer wiring (I2C1 = first external port on Wire1)
    constexpr uint8_t LINE_I2C_PORT     = 1;

    // Turn defaults
    constexpr int16_t TURN_POWER        = 50;
    constexpr uint8_t TURN_MODE         = 0;     // 0:single motor, 1:two motor
    constexpr bool    TURN_BRAKE        = true;

    // Lost-line behavior
    enum class LostLineAction : uint8_t { STOP, COAST, CRAWL };
    constexpr LostLineAction LOST_LINE_ACTION = LostLineAction::STOP;

    // OLED event trace
    constexpr uint8_t EVENT_TRACE_CAPACITY = 4;
}
```

**Drive class (header-only sketch):**

```cpp
class DriveClass {
public:
    bool    begin();
    void    update();

    // Continuous
    uint8_t setSpeed(int16_t L, int16_t R);
    uint8_t setSpeedSync(int16_t L, int16_t R);
    uint8_t forwardGyro(int16_t power);
    uint8_t stop(bool brake = true);

    // Distance-bounded (cm)
    uint8_t driveDist(int16_t power, float cm);
    uint8_t driveDistSync(int16_t power, float cm);
    uint8_t driveDistGyro(int16_t power, float cm);

    // Time-bounded (seconds)
    uint8_t driveTime(int16_t power, float sec);
    uint8_t driveTimeSync(int16_t L, int16_t R, float sec);
    uint8_t driveTimeGyro(int16_t power, float sec);

    // Turns
    uint8_t turnDeg(int16_t deg);              // relative, async
    uint8_t turnDegBlocking(int16_t deg);      // relative, blocking
    uint8_t turnToYaw(int16_t yaw);            // absolute, async
    uint8_t turnToYawBlocking(int16_t yaw);    // absolute, blocking

    // State
    bool    isMoveDone() const;
    bool    isTurnDone() const;
    int32_t getEncL() const;
    int32_t getEncR() const;
    void    resetEnc();
    double  getYaw() const;
    void    holdYaw();

private:
    int16_t  _heldYaw   = 0;
    bool     _moveDone  = true;
    bool     _turnDone  = true;
    static int16_t _normalizeYaw(int16_t deg);
    static uint16_t _cmToEncDeg(float cm);
};
extern DriveClass Drive;
```

**Intersection enum:**

```cpp
enum class JunctionType : uint8_t { NONE = 0, LEFT = 1, RIGHT = 2, T_OR_CROSS = 3, UNKNOWN = 4 };
```

## Error Handling

- **MMLower retry results:** every wrapped library call returns `uint8_t` (0x00 OK). Drive returns the underlying code unchanged. Caller may ignore for fire-and-forget; `WRO2026.ino` does not check (Phase 4 will).
- **Line lost:** see Decisions. `Drive.stop(true)` + `_lostLine` flag.
- **Turn fails to start (returns 0x07/0x01):** `Drive` records `_turnDone = true` immediately so caller is unblocked; logs a one-line event `TURN ERR`.
- **MXLineTracer offline (no I2C ack):** `LineFollower._lostLine = true` indefinitely; `EventTrace` shows `LINE OFFLINE`.
- **Yaw wrap:** `_normalizeYaw` clamps target to `[-360, +360]` per library spec; if `current + rel` overflows, wraps via `((target % 360) + 540) % 360 - 180` style normalization.
- **Zero/negative wheel diameter:** compile-time `static_assert(Config::WHEEL_DIAM_MM > 0)` in Drive.cpp.

## Testing Strategy

**Levels:** static (compile + grep) only this round — code-only, no hardware.

| ID  | Test Case | Type | Expected Behavior | AC |
|-----|-----------|------|-------------------|----|
| TC1 | `arduino-cli compile` against `WRO2026/` (or manual review log) | Compile | Exit 0, no warnings about undefined symbols | AC2 |
| TC2 | `grep -RnE "delay\s*\(" WRO2026/` filtered to `update()` method bodies | Static | Zero hits inside any `update()` method | AC4 |
| TC3 | `grep -nE "[0-9]" WRO2026/Drive.cpp WRO2026/LineFollower.cpp WRO2026/Intersection.cpp` for non-Config numerics | Static | All numerics are array indices, error codes, or sourced from `Config::` | AC5 |
| TC4 | Inspect Drive.h public method list against Scope checklist | Review | All 17 methods present with matching signatures | AC3, AC6 |
| TC5 | Inspect Drive.cpp `_normalizeYaw` is called from all four turn methods | Review | Every turn entry-point passes target through `_normalizeYaw` | AC6 |
| TC6 | Inspect LineFollower.cpp lost-line branch | Review | Calls `Drive.stop(true)` and sets `_lostLine` per `Config::LOST_LINE_ACTION` | AC7 |
| TC7 | Inspect Intersection.cpp edge logic | Review | `_wasAtJunction` flag flips correctly on transition | AC8 |
| TC8 | Inspect Manipulator stubs | Review | All five methods exist, all return immediately, doxygen `@todo Phase 3` | AC9 |
| TC9 | Inspect EventTrace ring buffer + dirty-flag rendering | Review | Capacity = `Config::EVENT_TRACE_CAPACITY`; `update()` only redraws when `_dirty` | AC10 |
| TC10 | Inspect `WRO2026.ino` setup ordering | Review | Matches the documented init sequence in CLAUDE.md exactly | AC11 |
| TC11 | Verify `.clang-format` is at repo root and matches MatrixMiniR4's | Static | `diff` against source returns no differences | AC12 |
| TC12 | `git branch --show-current` + `git log --oneline main..HEAD` | Static | On `phase1/drive-line-intersection`; multiple commits, no merges to main | AC13 |

**Run command:** `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` (verify availability first; if not available, the executor logs the absence and runs review-only TC2–TC12).

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | Create branch + scaffold dirs + `.clang-format` | — | low | `.clang-format`, `WRO2026/` (dir) | `git checkout -b phase1/drive-line-intersection`. Copy `.clang-format` from `C:\Users\wangd\documents\WRO2026\MatrixMiniR4\.clang-format` to repo root. Create empty `WRO2026/` directory. Satisfies AC13 prerequisite, AC12. |
| T2 | Author `Config.h` | T1 | med | `WRO2026/Config.h` | Implement the full `Config` namespace from the Data Model section above, exactly. All values constexpr. Top-of-file include guard `CONFIG_H`. Doxygen `@brief` on the namespace. Satisfies AC5 prerequisite. |
| T3 | Author `StateMachine.h` (header-only template helper) | T1 | low | `WRO2026/StateMachine.h` | Minimal templated state machine: `template <typename State> class StateMachine { State _s; void enter(State); State current(); };` with doxygen. No `.cpp`. Satisfies AC1. |
| T4 | Author `Drive.h` + `Drive.cpp` | T2 | high | `WRO2026/Drive.h`, `WRO2026/Drive.cpp` | Implement `DriveClass` per Data Model. `begin()` configures motor reverse flags, calls `MiniR4.DriveDC.begin(Config::MOTOR_LEFT_ID, Config::MOTOR_RIGHT_ID, Config::MOTOR_LEFT_REVERSE, Config::MOTOR_RIGHT_REVERSE)`, sets all three PIDs from `Config`. Each movement method maps 1:1 to a `MiniR4.DriveDC.*` call with `async=true` for non-blocking variants. `update()` polls `MiniR4.DriveDC.isPrevTaskDone()` and updates `_moveDone`/`_turnDone`. `_normalizeYaw` and `_cmToEncDeg` are private statics. `extern DriveClass Drive;` in header, instance in cpp. No `delay()` in `update()`. Satisfies AC3, AC4, AC6. |
| T5 | Author `Manipulator.h` + `Manipulator.cpp` (empty stubs) | T2 | low | `WRO2026/Manipulator.h`, `WRO2026/Manipulator.cpp` | `class ManipulatorClass { bool begin(); void update(); void setLift(uint8_t); void setGripper(uint8_t); void setArm(uint8_t); bool isDone(); };` All bodies `return true;` / `return;` / no-op. Doxygen each method with `@todo Phase 3 — implement after mechanism finalized`. `extern ManipulatorClass Manipulator;` in header. Satisfies AC9. |
| T6 | Author `LineFollower.h` + `LineFollower.cpp` | T2, T4 | high | `WRO2026/LineFollower.h`, `WRO2026/LineFollower.cpp` | Class `LineFollowerClass` owns a `MatrixLineTracer _tracer`. `begin()` sets `_tracer._pWire = &Wire1; _tracer._ch = Config::LINE_I2C_PORT; _tracer.begin(); _tracer.setThreshold(Config::LINE_THRESHOLD);`. `update()` (non-blocking): if `!_tracer.isOnline()` → handle per `Config::LOST_LINE_ACTION` (default STOP: `Drive.stop(true)`, `_lostLine=true`); else clears `_lostLine`, reads `getError()`, computes P+D correction (`_lastErr` for D term), calls `Drive.setSpeed(cruise + corr, cruise - corr)` clamped to `±100`. `setEnabled(bool)`, `isLineLost()`, `getRawError()` accessors. `extern LineFollowerClass LineFollower;`. Satisfies AC4, AC7. |
| T7 | Author `Intersection.h` + `Intersection.cpp` | T2, T6 | med | `WRO2026/Intersection.h`, `WRO2026/Intersection.cpp` | Class `IntersectionClass` shares the `MatrixLineTracer` reference (use a pointer set in `begin()` from `LineFollower.getTracer()`, OR own its own instance — pick the shared-pointer route to avoid double I2C reads). `update()` polls `_tracer->getJunctionType()`, edge-detects against `_wasAtJunction`. `isAtIntersection()` returns the edge event (clears the flag on read). `getLastType()` returns captured `JunctionType` enum (mirroring 0..4 from the library). `extern IntersectionClass Intersection;`. Satisfies AC4, AC8. |
| T8 | Author `EventTrace.h` + `EventTrace.cpp` | T2, T4, T6, T7 | low | `WRO2026/EventTrace.h`, `WRO2026/EventTrace.cpp` | Class `EventTraceClass` owns `char _buf[Config::EVENT_TRACE_CAPACITY][20]`, `uint8_t _head`, `bool _dirty`. `pushEvent(const char*)` writes to ring buffer + sets dirty. `update()` polls: `LineFollower.isLineLost()` edge → push `LINE LOST` / `LINE OK`; `Intersection.isAtIntersection()` → push `INT <type>`; `Drive.isTurnDone()` rising edge → push `TURN END`. If `_dirty`, render via `MiniR4.OLED` (clearDisplay, print each line, display), clear dirty. No `delay()`. Satisfies AC4, AC10. |
| T9 | Author `WRO2026/WRO2026.ino` | T2, T4, T5, T6, T7, T8 | med | `WRO2026/WRO2026.ino` | `#include <MatrixMiniR4.h>` + all module headers + `Config.h`. `setup()`: `MiniR4.begin(); Serial.begin(115200); MiniR4.PWR.setBattCell(Config::BATT_CELLS); Drive.begin(); LineFollower.begin(); Intersection.begin(); Manipulator.begin(); EventTrace.begin();`. `loop()`: each module's `update()` in order Drive → LineFollower → Intersection → Manipulator → EventTrace. `#ifdef DEMO_MODE` block: on `BTN_UP`, call `LineFollower.setEnabled(true); Drive.driveDistGyro(50, 30); Drive.turnDeg(90);` etc. — minimal smoke routine. No business logic. Satisfies AC11. |
| T10 | Verification — compile + grep audit | T9 | med | (no file changes; produces a review log) | Try `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/`. If unavailable, log absence and run TC2 (grep delay), TC3 (grep magic numbers), TC4–TC12 (review checklist) and produce a `docs/phase1-verification.md` with results. Satisfies AC2 (or its documented fallback). |
| T11 | Per-task commits + final commit-graph review | T1–T10 | low | (git only) | After each completed task, commit on the branch with `feat(phaseN): <module>`. Final `git log --oneline main..HEAD` should show 10–11 commits. No merges to main. Satisfies AC13. |

Parallelization plan: After T1 lands, T2 + T3 + T5 run in parallel. Once T2 + T4 land, T5 already done independently, T6 + T7 fan out (T7 only after T6 since they share the tracer pointer). T8 waits for T4/T6/T7. T9 last among code tasks. T10 + T11 finalize.

## Notes for Implementer

- **Yaw wrap is the #1 trap.** `MiniR4.Motion.getEuler(Yaw)` is unbounded; the example sketch shows turn targets like 270 and -270 working fine with `TurnGyro`. Do not pre-modulo to `[-180, 180]` — `TurnGyro` handles `±360`. Just normalize to that band.
- **`MatrixLineTracer` is shared between `LineFollower` and `Intersection`.** Two simultaneous I2C reads of the same sensor wastes bus time and risks contention. `Intersection` should hold a pointer set during `begin()` from `LineFollower.getTracer()`. If you choose to give each its own instance, document why.
- **`Drive.setSpeed` calls `MiniR4.DriveDC.Move(L, R)` (raw PWM, no encoder regulation).** That is what the line-follower wants — it's a tight feedback loop and PID-on-PID would oscillate. Don't accidentally route it through `MoveSync`.
- **`MiniR4.DriveDC` async vs blocking is per-call** (`async = true/false` parameter in MoveDegs/MoveTime/MoveGyroDegs/MoveGyroTime/MoveSyncDegs/MoveSyncTime/TurnGyro). The blocking form internally `delay(50)` polls — that's fine because it's invoked from `Drive.turnDegBlocking`/etc. which are command-time entry points, NOT inside `update()`. Document this in `Drive.h` as the rule.
- **Compile target:** Arduino UNO R4 WiFi FQBN is `arduino:renesas_uno:unor4wifi`. The MatrixMiniR4 library must be installed in the IDE's library path or symlinked.
- **No hardware to validate against.** The reviewer for T10 should cross-check every `MiniR4.*.method(...)` call against `C:\Users\wangd\documents\WRO2026\MatrixMiniR4\src\Modules\*.h` for exact signatures and return-type expectations.
- **Rollback plan:** all work on a single feature branch `phase1/drive-line-intersection`. If anything proves badly wrong, `git branch -D` it from main and start over — `main` is untouched.
- **Future Phase 2 hand-off:** `Drive.turnDeg` (relative, async) is the call shape `PathTracker` will drive. Keep the API stable; Phase 2 should not have to change `Drive.h`.
