# Manipulator API + Config Split Plan

**Goal:** Replace the stale `Manipulator` stub with a real, mistake-proof actuator API for Robot1 (3 RC servos + 1 LEGO belt motor), and carve the new manipulator constants — plus a separate validation file — out of the monolithic `Config.h`. Build the *structure* now; leave numeric calibration as bring-up `TODO`s (mechanism is CAD-done, not yet assembled).

**Approach:** Two actuator classes instantiated four times — `ManipServo` (grabber/sorter/dumper; fire-and-forget, timer-based `isDone()`) and `ManipBelt` (encoder DC motor; position + timed move modes, stall-home) — under a `Manipulator` facade that exposes coordinated semantic verbs (`grab`/`release`/`sortTo`/`dump`/`reset`) as `[[nodiscard]]` blocking-by-default builders with `.async()` opt-out, mirroring the Drive builder/`_drainBlocking`/`setTickCallback` machinery. Config is split *minimally*: two new headers (`ConfigManipulator.h`, `ConfigChecks.h`) included from `Config.h` (checks last), leaving the existing body and its inline `static_assert`s untouched.

## Scope

- **In scope:**
  - `ManipServo.h` — RC-servo joint wrapper: `begin()` (one-time `setHWDir`), `setAngle`/`open`/`close`/`home` (fire-and-forget), timer-based `isDone()`, software-shadow commanded angle, clamp.
  - `ManipBelt.h` — belt DC-motor wrapper: `toStation(BeltStation)`, raw `beltTo(absDeg)` (absolute POSITION), raw `runFor(ms,speed)` (TIMED), `home()` (stall-home), mode-dependent `isDone()`, move watchdog, `update()`.
  - `Manipulator.h`/`.cpp` — facade owning `grabber`/`sorter`/`dumper`/`belt`; coordinated verbs `grab`/`release`/`sortTo(uint8_t bin)`/`dump`/`reset` as blocking-default builders with `.async()`; `begin()` (boot-time bounded blocking home), `update()`, `isDone()`, `setTickCallback()`. Replaces `setLift`/`setGripper`/`setArm` (zero existing callers).
  - `ConfigManipulator.h` — per-actuator `constexpr` config (channel IDs w/ `*_DEFAULT` override hooks, home/open/close/detent angles, ms-per-deg, settle, sorter bin table, belt stations + PPR/RPM/home/timeout params, sign convention). Placeholder values + `TODO: measure at bring-up`.
  - `ConfigChecks.h` — NEW manipulator `static_assert`s only (ID ranges + collisions, angle ranges, table sizes); a `#ifndef CONFIG_H / #error` guard so it can't be misincluded directly. Included last by `Config.h`.
  - `Config.h` — add two trailing `#include`s (manipulator, then checks). No other change.
  - Test harness: extend `test/mocks/MatrixMiniR4.h` to expose `MiniR4.RC1..4` / `MiniR4.M1..4`; new `test_manipulator.cpp` (logic) + config compile-fail tests; `CMakeLists.txt` targets.
  - Docs: supersede ADR-007 via ADR-032 (status-line + cross-link only); add ADR-031 (config split) + ADR-032 (manipulator API pattern); update `DECISIONS.md`, `ARCHITECTURE.md` (Manipulator row), `Roadmap.md` (Phase-5 pulled-forward note).
- **Out of scope:**
  - Real calibrated angles/positions/timings — placeholders only (no built mechanism to measure against).
  - Mission DSL / Tasks wiring of the verbs (mission code calls `Manipulator` directly; Tasks composition is later) — chosen "class + config only".
  - Full Config split of drivetrain/line buckets (chosen "minimal"); the existing inline `static_assert`s are NOT relocated.
  - Camera, Phase-5 task FSMs, any `ActiveManipulator` Robot2 profile (note as future extension point; do not build).

## Decisions

- **Two classes, four instances** (not four bespoke classes) — servos are behaviorally identical, differ only in data; belt is genuinely different (feedback + position). Mirrors `MiniR4RC<ID>`/`MiniR4DC<ID>` and CLAUDE.md rule 3.
- **Servo `isDone()` is timer-based, not feedback** — `MiniR4RC` exposes only `setAngle` (verified: no position getter anywhere in the vendor lib). Estimate = `|clamped - prevShadow| * msPerDeg + settleMs` from a recorded `millis()` start, recomputed on every `setAngle`.
- **Servo reverse is hardware, not software** — `ManipServo` calls the vendor `MiniR4RC::setHWDir(cfg.reverse)` once at `begin()`; there is NO software `180-deg` transform. The shadow stores the clamped commanded angle; the hardware owns direction.
- **Belt has two move modes** — mid stations use POSITION (`rotateFor` + `ChkRotateEnd`/`getDegrees`, real `isDone()`); end stations use TIMED (`setSpeed` + `millis` timer, timer `isDone()`) because commanding an encoder degree into a hard end-stop stalls. Each `BeltStation` carries its mode in config.
- **Belt POSITION moves are absolute against the homed zero** — `rotateFor` is a *relative* "rotate N degrees" command, so `toStation`/`beltTo` read `getDegrees()`, compute `delta = absTarget - current`, and issue `rotateFor(sign(delta)*speed, |delta|)`. `home()` + `resetCounter()` re-establish the zero that makes absolute targets repeatable.
- **Stall-home** — `home()` runs the belt toward the HOME end-stop (TIMED, low speed) then `resetCounter()` to zero. No limit switch exists.
- **Home at `begin()`, wall-clock bounded** — servos snap to home angle, belt stall-homes, during `setup()`. Blocking via plain `millis`/`delay` waits (the tick callback is registered AFTER `begin()` in the sketch, so `begin()` must not rely on it), each loop bounded by a hard `BELT_HOME_TIMEOUT_MS` ceiling so it can't hang. The manipulator physically moves on power-up.
- **Semantic verbs are blocking-default `[[nodiscard]]` builders with `.async()`** — the blocking/async mode is the "knob" (same justification as `Drive.turnDeg`), so ADR-012 is respected. Verbs fire their actuators concurrently then drain until all involved `isDone()` (unless `.async()`). Primitives (`grabber.setAngle`, `belt.beltTo`, `belt.toStation`, `belt.runFor`) are direct calls (0–2 knobs) and stay public for calibration.
- **Own completion state, not Drive's queue** — Drive's done-flag is single-slot, wheel-exclusive. Manipulator gets a 4th `setTickCallback(cooperativeTick)` + its own bounded `_drain`. No FIFO queue (verbs are sequential when blocking; concurrent async on the same actuator is last-wins — `_estMs` recomputes on each `setAngle`, documented).
- **Sorter bins by index** — `sortTo(uint8_t bin)` indexes `Config::SORTER_BIN_ANGLE[bin]`; out-of-range rejected before any array read.
- **Minimal config split** — only `ConfigManipulator.h` + `ConfigChecks.h` are added; `Config.h` includes them (checks last); existing asserts stay put. Satisfies #39's "datacheck in another file" for the new surface without the risky relocation of the load-bearing motor-ID asserts.

## Acceptance Criteria

- [ ] **AC1** — `Config.h` compiles and all 16 production includers + 5 test includers build unchanged; `ConfigManipulator.h` then `ConfigChecks.h` are included from `Config.h`, checks LAST. (Host ctest green locally/CI; the Arduino-compile half is CI-only — no host C++ compiler locally.)
- [ ] **AC2** — A build forcing an ID collision (`-DBELT_M_ID_DEFAULT=2` == MOTOR_LEFT_ID, or `-DSORTER_RC_ID_DEFAULT=1` == GRABBER_RC_ID) fails to compile with the specific manipulator `static_assert` message; asserted via a compile-fail ctest `PASS_REGULAR_EXPRESSION` on the full message.
- [ ] **AC3** — `ManipServo.setAngle(d)` clamps to `[0,180]`, records the clamped commanded angle + start; `isDone()` is false until `|clamped-prevShadow|*msPerDeg + settleMs` elapses, true after. Direction is set once via `setHWDir` at `begin()` (no software transform).
- [ ] **AC4** — `Manipulator.sortTo(bin)` drives the sorter to `SORTER_BIN_ANGLE[bin]`; `bin >= SORTER_BIN_COUNT` is rejected BEFORE indexing (Serial warning, no actuator write, action already-done). Verified with the test TU built under `-fsanitize=address,undefined`.
- [ ] **AC5** — `belt.toStation(s)` with a POSITION station issues `rotateFor(sign*speed, |absTarget - getDegrees()|)` and `isDone()` follows `ChkRotateEnd(bool&)`; with a TIMED station it issues `setSpeed` and `isDone()` follows the timer; mode is read from config.
- [ ] **AC6** — `belt.home()` runs a TIMED move toward the HOME end-stop then calls `resetCounter()`; the mock's `resetCounter()` zeroes its backing `getDegrees()`, and the test asserts `getDegrees()==0` after `home()` completes (order: timed run, then reset).
- [ ] **AC7** — A blocking verb (`grab()`) returns only after its actuators report `isDone()`; `grab().async()` returns immediately while `isDone()` is still false (host test advances mock `millis`).
- [ ] **AC8** — `Manipulator.begin()` drives servos to home + stall-homes the belt during `setup()` (no tick callback yet), returns `true` on vendor success / `false` on failure or home-timeout (boot-fail path unchanged). `Manipulator.update()` is non-blocking and advances the belt's in-flight POSITION move + watchdog.
- [ ] **AC9** — `setLift`/`setGripper`/`setArm` are removed; no caller breaks (verified zero callers); `WRO2026.ino` compiles with `begin()`/`update()` signatures unchanged.
- [ ] **AC10** — A blocking POSITION belt move that never completes trips a bounded watchdog (`isDone()` stays false at `t = BELT_MOVE_TIMEOUT_MS - 1`, forced true + Serial warning at `t >= BELT_MOVE_TIMEOUT_MS`) rather than hanging the drain.
- [ ] **AC11** — Only the facade semantic verbs are `[[nodiscard]]` builders; the actuator primitives are direct calls (ADR-012). The tick callback / `update()` never issues a blocking verb (ADR-016 reentrancy `@warning` present). (The `[[nodiscard]]` warning itself is by-inspection — the host harness suppresses unused-result warnings to allow bare-expression builders, matching Drive.)
- [ ] **AC12** — ADR-007 superseded by ADR-032 (status-line + cross-link only, body intact); ADR-031 + ADR-032 added; `DECISIONS.md`, `ARCHITECTURE.md`, `Roadmap.md` updated.
- [ ] **AC13** — `begin()`'s blocking home loops are wall-clock bounded (`while(!isDone() && millis()-start < BELT_HOME_TIMEOUT_MS)`); `begin()` returns `false` if a home never completes within the ceiling (asserted on host by holding the mock done-condition false).

## Data Model

All new constants live in `namespace Config` (reopened in `ConfigManipulator.h`). Values below are PLACEHOLDERS (`TODO: measure at bring-up`).

```cpp
// ----- ConfigManipulator.h (self-contained: <stdint.h> only; included via Config.h) -----
namespace Config
{
    // Channel IDs. Servos are on the RC bus, the belt on the M bus — the RC and M id
    // spaces are INDEPENDENT, so a servo on RC1 and the belt on M1 do NOT collide. Only
    // servo-vs-servo (RC) and belt-vs-drive-pair (M2/M3) collisions matter; that is exactly
    // what ConfigChecks.h enforces. The *_DEFAULT override hooks (mirroring Config.h's
    // PATH_PASS_THROUGH_ENABLED_DEFAULT pattern at Config.h:926/942) let the compile-fail
    // ctest force a collision via -D without macro-corrupting a constexpr.
    constexpr uint8_t GRABBER_RC_ID = 1;   // TODO confirm wiring
#ifndef SORTER_RC_ID_DEFAULT
#define SORTER_RC_ID_DEFAULT 2
#endif
    constexpr uint8_t SORTER_RC_ID  = SORTER_RC_ID_DEFAULT;
    constexpr uint8_t DUMPER_RC_ID  = 3;
#ifndef BELT_M_ID_DEFAULT
#define BELT_M_ID_DEFAULT 1
#endif
    constexpr uint8_t BELT_M_ID     = BELT_M_ID_DEFAULT;   // must differ from MOTOR_LEFT_ID(2)/MOTOR_RIGHT_ID(3)

    // Per-servo calibration.
    struct ServoCfg {
        uint8_t  rcId;
        uint16_t homeDeg;
        uint16_t openDeg;     // for grabber/dumper; sorter uses the bin table instead
        uint16_t closeDeg;
        float    msPerDeg;    // travel-time estimate basis (timer isDone)
        uint16_t settleMs;    // extra dwell after travel
        bool     reverse;     // HARDWARE direction flag — passed once to MiniR4RC::setHWDir(reverse)
                              // at ManipServo::begin(); NOT applied in software.
    };
    constexpr ServoCfg GRABBER = { GRABBER_RC_ID, 90, 30, 150, 2.0f, 80, false };
    constexpr ServoCfg DUMPER  = { DUMPER_RC_ID,  90,  90, 150, 2.0f, 80, false };
    constexpr ServoCfg SORTER  = { SORTER_RC_ID,  90,   0,   0, 2.0f, 80, false }; // open/close unused

    // Sorter discrete bins (indexed by sortTo(bin)).
    constexpr uint8_t  SORTER_BIN_COUNT     = 3;          // TODO finalize
    constexpr uint16_t SORTER_BIN_ANGLE[SORTER_BIN_COUNT] = { 30, 90, 150 };

    // Belt motor. Sign convention: +speed drives toward FAR_END, -speed drives toward HOME.
    constexpr uint16_t BELT_PPR             = 360;     // LEGO EV3
    constexpr uint16_t BELT_RPM             = 300;
    constexpr int16_t  BELT_MOVE_SPEED      = 40;      // POSITION-move magnitude
    constexpr int16_t  BELT_HOME_SPEED      = 25;      // stall-home magnitude (toward HOME)
    constexpr uint32_t BELT_HOME_MS         = 1500;    // stall-home timed run
    constexpr uint32_t BELT_HOME_TIMEOUT_MS = BELT_HOME_MS + 1000; // wall-clock ceiling for begin() home
    constexpr uint32_t BELT_MOVE_TIMEOUT_MS = 4000;    // POSITION-move watchdog

    enum class BeltMode : uint8_t { POSITION, TIMED };
    struct BeltStationCfg {
        BeltMode mode;
        int32_t  deg;     // POSITION: ABSOLUTE encoder target vs the home zero. toStation/beltTo
                          //   read getDegrees() and issue rotateFor(sign(delta)*speed, |delta|).
        uint32_t ms;      // TIMED: run duration
        int16_t  speed;   // signed: +toward FAR_END, -toward HOME (magnitude = PWM)
    };
    enum class BeltStation : uint8_t { HOME = 0, MID = 1, FAR_END = 2 };  // TODO finalize
    constexpr uint8_t BELT_STATION_COUNT = 3;
    constexpr BeltStationCfg BELT_STATIONS[BELT_STATION_COUNT] = {
        { BeltMode::TIMED,    0, BELT_HOME_MS, -BELT_HOME_SPEED }, // HOME (timed into near end)
        { BeltMode::POSITION, 400, 0,           BELT_MOVE_SPEED }, // MID (absolute 400)
        { BeltMode::TIMED,    0, 1500,          BELT_MOVE_SPEED },  // FAR_END (stalls -> timed)
    };
}
```

```cpp
// ----- ConfigChecks.h (included LAST by Config.h) -----
#ifndef CONFIG_H
#error "ConfigChecks.h must be included via Config.h (it references Config drivetrain symbols)."
#endif
namespace Config
{
    static_assert(GRABBER_RC_ID >= 1 && GRABBER_RC_ID <= 4 &&
                  SORTER_RC_ID  >= 1 && SORTER_RC_ID  <= 4 &&
                  DUMPER_RC_ID  >= 1 && DUMPER_RC_ID  <= 4,
                  "Manipulator servo RC IDs must be in [1,4].");
    static_assert(GRABBER_RC_ID != SORTER_RC_ID && GRABBER_RC_ID != DUMPER_RC_ID &&
                  SORTER_RC_ID  != DUMPER_RC_ID,
                  "Manipulator servo RC IDs must be pairwise distinct.");
    static_assert(BELT_M_ID >= 1 && BELT_M_ID <= 4, "BELT_M_ID must be in [1,4].");
    static_assert(BELT_M_ID != MOTOR_LEFT_ID && BELT_M_ID != MOTOR_RIGHT_ID,
                  "BELT_M_ID collides with a drive motor (MOTOR_LEFT_ID/MOTOR_RIGHT_ID).");
    static_assert(SORTER_BIN_COUNT >= 1, "SORTER_BIN_COUNT must be >= 1.");
    static_assert(BELT_STATION_COUNT >= 1, "BELT_STATION_COUNT must be >= 1.");
    static_assert(GRABBER.openDeg <= 180 && GRABBER.closeDeg <= 180 && GRABBER.homeDeg <= 180 &&
                  DUMPER.openDeg  <= 180 && DUMPER.closeDeg  <= 180 && DUMPER.homeDeg  <= 180 &&
                  SORTER.homeDeg  <= 180,
                  "Manipulator servo angles must be <= 180.");
}
```

```cpp
// ----- ManipServo.h (header-only; #include <MatrixMiniR4.h> with ANGLE BRACKETS) -----
class ManipServo {
public:
    explicit ManipServo(const Config::ServoCfg& cfg);
    void begin();                   // calls MiniR4.RC<id>.setHWDir(cfg.reverse) ONCE; sets _shadow=homeDeg
    void setAngle(uint16_t deg);    // fire-and-forget; clamp [0,180]; NO software reverse; record shadow+start+est
    void open();                    // -> cfg.openDeg
    void close();                   // -> cfg.closeDeg
    void home();                    // -> cfg.homeDeg
    bool isDone() const;            // millis()-_startMs >= _estMs
    uint16_t commandedAngle() const; // returns _shadow (the clamped commanded angle)
private:
    const Config::ServoCfg& _cfg;
    uint16_t _shadow;   // last commanded (clamped) angle (init = homeDeg after begin)
    uint32_t _startMs;
    uint32_t _estMs;    // = |clamped - prevShadow| * msPerDeg + settleMs, recomputed each setAngle
};
```

```cpp
// ----- ManipBelt.h (#include <MatrixMiniR4.h> with ANGLE BRACKETS) -----
class ManipBelt {
public:
    explicit ManipBelt(const Config::BeltStationCfg* table, uint8_t count, uint8_t motorId);
    bool begin();                          // setPPR_RPM, setReverse, then bounded home() (blocking, setup-time)
    void toStation(Config::BeltStation s); // POSITION vs TIMED by table[s].mode (POSITION uses getDegrees delta)
    void beltTo(int32_t absDeg);           // raw absolute POSITION: delta=absDeg-getDegrees(); rotateFor(sign*speed,|delta|)
    void runFor(uint32_t ms, int16_t speed); // raw TIMED
    void home();                           // TIMED stall-home toward HOME + resetCounter() (zeroes getDegrees)
    void update();                         // advance in-flight POSITION move + watchdog
    bool isDone() const;                   // POSITION: ChkRotateEnd(bool& out) -> read out; TIMED: timer
private:
    enum class State { IDLE, POS_MOVE, TIMED_MOVE };
    // motorId, table, in-flight target, _startMs, _durMs, _timeoutMs, _state ...
};
```

```cpp
// ----- Manipulator.h (facade + verb builder; #include <MatrixMiniR4.h> with ANGLE BRACKETS) -----
class [[nodiscard]] ManipAction {            // returned by every semantic verb
public:
    ManipAction& async();                    // flips _blocking=false
    ManipAction(ManipAction&&) noexcept;     // move transfers _executed=true
    ManipAction(const ManipAction&) = delete;
    ~ManipAction();                          // fire involved actuators concurrently; if blocking, _drain until all isDone()
private:
    ManipulatorClass& _m;
    /* which actuators + targets this action drives */
    bool _blocking = true;
    bool _executed = false;
};

class ManipulatorClass {
public:
    bool begin();                            // setHWDir each servo + home servos + stall-home belt (bounded, pre-tick)
    void update();                           // belt.update(); servos are pure timers
    bool isDone() const;                     // all four actuators done
    void setTickCallback(void (*cb)());      // @warning callback MUST NOT issue a blocking verb (ADR-016)

    ManipAction grab();                      // close grabber (thin scaffold)
    ManipAction release();                   // open grabber
    ManipAction sortTo(uint8_t bin);         // sorter -> SORTER_BIN_ANGLE[bin]
    ManipAction dump();                      // dumper -> dump pose
    ManipAction reset();                     // all servos home + belt.home()

    ManipServo grabber, sorter, dumper;      // public primitives
    ManipBelt  belt;
private:
    void _drain();                           // bounded spin: tickCallback()+delay(2) until target actuators done
    void (*_tickCallback)() = nullptr;
};
extern ManipulatorClass Manipulator;
```

## API Contracts

Public surface (mission code uses the verbs; primitives are for calibration/edge cases):

```
Manipulator.grab() / .release() / .dump() / .reset()      -> ManipAction (blocking; .async() opts out)
Manipulator.sortTo(uint8_t bin)                            -> ManipAction  (bin in [0, SORTER_BIN_COUNT))
Manipulator.isDone() -> bool                               (all actuators settled)

grabber/sorter/dumper.setAngle(uint16_t 0..180)            (fire-and-forget; no software reverse)
grabber/dumper.open()/close()/home()                       (presets)
belt.toStation(BeltStation) / .beltTo(absDeg) / .runFor(ms,speed) / .home()   (direct)
belt.isDone() -> bool ; belt.update()
```

Error responses:
- Out-of-range servo angle -> clamped to `[0,180]` (no error).
- `sortTo(bin >= SORTER_BIN_COUNT)` -> Serial warning, no array read, no actuator write; action completes immediately (`isDone()` true).
- POSITION belt move exceeding `BELT_MOVE_TIMEOUT_MS` -> watchdog forces `isDone()` true + Serial warning.
- `begin()` home exceeding `BELT_HOME_TIMEOUT_MS` -> `begin()` returns `false`.
- Vendor `bool` returns are checked; on `false`, retry once after a small `delay()` (CLAUDE.md rule 5). Persistent failure in `begin()` -> `begin()` returns `false`; in a primitive -> Serial warning.
- Blocking-verb `_drain` exceeding a per-action ceiling -> stop the belt, warn, return (mirrors `DRIVE_DRAIN_TIMEOUT_MS_PER_CMD`).

## Error Handling

- **Servo angle out of range:** clamp into `[0,180]` before write; never reject.
- **Sort bin / belt station index out of range:** reject with a Serial warning and a no-op BEFORE any array read; the returned action is already-done so a blocking caller does not hang.
- **Belt POSITION move stalls / never completes:** `update()` watchdog (`BELT_MOVE_TIMEOUT_MS`) forces completion + warns. End stations avoid this entirely by being TIMED.
- **`begin()` home never completes:** each home loop is bounded by `BELT_HOME_TIMEOUT_MS`; on expiry `begin()` returns `false`.
- **Vendor call returns false:** retry once after a small `delay()` (rule 5); fold persistent `begin()` failures into its `bool` return (existing `FAIL MANIP` boot path).
- **Blocking verb drain timeout:** bounded spin with a per-action ceiling; on expiry stop the belt, warn, return (chassis-safe — manipulator does not own the wheels).
- **Reentrancy:** blocking verbs MUST NOT be called from inside `cooperativeTick`/any `update()`; documented as an `@warning` on `setTickCallback` (same contract as Drive, ADR-016).

## Testing Strategy

**Levels:** Unit (host, g++/ctest via the existing `host-tests` job, manipulator TU built with `-fsanitize=address,undefined`) + compile-fail ctest + the existing Arduino-compile job for the real sketch/vendor build.

| ID | Test Case | Type | Expected Behavior |
|----|-----------|------|-------------------|
| TC1 | `setAngle(200)` then read commanded | Unit | Clamped to 180 (no software reverse; direction is via setHWDir at begin) |
| TC2 | `setAngle`, advance mock millis < estimate | Unit | `isDone()` false |
| TC3 | `setAngle`, advance mock millis >= estimate | Unit | `isDone()` true |
| TC4 | `sortTo(1)` | Unit | Sorter servo commanded to `SORTER_BIN_ANGLE[1]` |
| TC5 | `sortTo(SORTER_BIN_COUNT)` (under UBSan) | Unit | No array read, no servo write; warning; action already-done |
| TC6 | `belt.toStation(MID)` (POSITION), `getDegrees()==100` | Unit | `rotateFor(+speed, 300)` issued (`|400-100|`, + sign); `isDone()` tracks mocked `ChkRotateEnd(bool&)` |
| TC7 | `belt.toStation(FAR_END)` (TIMED) | Unit | `setSpeed` issued; `isDone()` flips after mocked timer |
| TC8 | `belt.home()` | Unit | TIMED run toward HOME then `resetCounter()`; `getDegrees()==0` after (order asserted) |
| TC9 | POSITION move, mock never ends | Unit | `isDone()` false at `t=timeout-1`; at `t>=BELT_MOVE_TIMEOUT_MS` watchdog forces `isDone()` true + warning |
| TC10 | `grab()` blocking, mock millis advanced in tick cb | Unit | Returns only after grabber `isDone()` |
| TC11 | `grab().async()` | Unit | Returns immediately; `isDone()` still false |
| TC12 | `reset()` | Unit | All servos commanded to home; `belt.home()` invoked |
| TC13 | Build with `-DBELT_M_ID_DEFAULT=2` (== MOTOR_LEFT_ID) | Compile-fail | Build fails; ctest matches the collision `static_assert` message |
| TC14 | Build with `-DSORTER_RC_ID_DEFAULT=1` (== GRABBER_RC_ID) | Compile-fail | Build fails; matches the "pairwise distinct" message |
| TC15 | `begin()`; mock `delay()` advances `g_mockMillis` | Unit | All actuators `isDone()` true; `resetCounter()` called once; `begin()` returns true |
| TC16 | `begin()` home never completes (mock done-condition held false) | Unit | `begin()` returns false once `millis()-start >= BELT_HOME_TIMEOUT_MS` (loop is wall-clock bounded) |

**Test data / mock changes:** extend `test/mocks/MatrixMiniR4.h` from the empty shim into a `MiniR4` mock exposing `RC1..RC4` (record last `setAngle`; `setHWDir`) and `M1..M4` (`rotateFor`/`ChkRotateEnd(bool&)`/`getDegrees`/`setSpeed`/`setBrake`/`resetCounter`/`setPPR_RPM`/`setReverse`; the mock `resetCounter()` zeroes its backing `getDegrees()`; test-settable `mockRotateEnded`, call counters, last-args); define the `MiniR4` global + reset helpers in `mocks.cpp`. Extend `MockSerial` (in `mocks/Arduino.h`) with the numeric `print(...)`/`println(...)` overloads the manipulator warning paths use. **Mock `delay(ms)` now advances `g_mockMillis` by `ms` (monotonic)** so `begin()`'s plain-delay home loops terminate on host — AUDIT TC1..TC14 and any existing test that asserted on a `millis()`-derived value while assuming `delay()` was a no-op: re-pin those to set `g_mockMillis` explicitly rather than relying on it staying frozen across `delay()` calls. New manipulator headers MUST use `#include <MatrixMiniR4.h>` (ANGLE brackets) so the host build shadows it with the mock (a quoted include would pick the real vendor header). Compile-fail TUs use the `-D*_DEFAULT` override hooks (NOT `#define` over a `constexpr`) and follow the `test_tasks_dispatch_overflow` precedent (no mock force-includes; `PASS_REGULAR_EXPRESSION` on the full assert message).
**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|----|------|------------|------|-------|-------------|
| T1 | ConfigManipulator.h | — | med | `WRO2026/ConfigManipulator.h` | Create per the Data Model: channel IDs WITH `SORTER_RC_ID_DEFAULT`/`BELT_M_ID_DEFAULT` override hooks (`#ifndef`/`#define`/`#endif` then `constexpr ... = *_DEFAULT;`), `ServoCfg` (with the hardware-`reverse` doc) + GRABBER/SORTER/DUMPER, `SORTER_BIN_COUNT`/`SORTER_BIN_ANGLE`, belt PPR/RPM/speeds/`BELT_HOME_TIMEOUT_MS`/`BELT_MOVE_TIMEOUT_MS`, the belt sign convention docstring (+FAR_END / -HOME), `BeltMode`/`BeltStationCfg` (deg = ABSOLUTE) / `BeltStation` / `BELT_STATIONS`. Placeholders + `TODO: measure`. Self-contained (`<stdint.h>`). Doxygen per constant. |
| T2 | ConfigChecks.h | T1 | med | `WRO2026/ConfigChecks.h` | Create per the Data Model. First line guard: `#ifndef CONFIG_H / #error "include via Config.h" / #endif`. Then the NEW manipulator `static_assert`s only (servo IDs in [1,4] + distinct; `BELT_M_ID` in [1,4] + != MOTOR_LEFT/RIGHT_ID; counts >= 1; angles <= 180), each with a comment pointer to the constants + code it guards. References `Config::MOTOR_LEFT_ID`/`MOTOR_RIGHT_ID` (visible because Config.h includes this LAST). Do NOT relocate existing asserts. |
| T3 | Wire Config.h | T1, T2 | low | `WRO2026/Config.h` | After `}  // namespace Config` and before `#endif`, add `#include "ConfigManipulator.h"` then `#include "ConfigChecks.h"` (checks LAST). No other change. |
| T4 | ManipServo | T1 | med | `WRO2026/ManipServo.h` | Header-only per Data Model; `#include <MatrixMiniR4.h>` (angle brackets). `begin()` calls `setHWDir(cfg.reverse)` once on the right RC channel + sets `_shadow=homeDeg`. `setAngle` clamps `[0,180]`, writes the clamped angle directly via a small `_writeServo(rcId,angle)` `switch` over `MiniR4.RC1..4` (NO software reverse), records `_shadow=clamped`, `_startMs=millis()`, `_estMs=|clamped-prevShadow|*msPerDeg+settleMs`. `open/close/home` call `setAngle`. `isDone()` = `millis()-_startMs >= _estMs`. Check vendor `bool` returns (retry-once per rule 5). Doxygen + reentrancy notes. |
| T5 | ManipBelt | T1 | high | `WRO2026/ManipBelt.h` (+ `.cpp` if heavy) | Per Data Model; angle-bracket vendor include. `begin()`: `setPPR_RPM`, `setReverse`, then bounded blocking `home()` (`while(!isDone() && millis()-start < BELT_HOME_TIMEOUT_MS)`), return false on timeout/vendor-fail. `toStation`: read `BELT_STATIONS[s].mode`; POSITION -> compute `delta = deg - getDegrees()`, `rotateFor(sign(delta)*|speed|, |delta|)` + state POS_MOVE; TIMED -> `setSpeed(speed)` + `_startMs`/`_durMs` + TIMED_MOVE. `beltTo(absDeg)` raw absolute POSITION; `runFor` raw TIMED. `home()` = TIMED toward HOME (`-BELT_HOME_SPEED`,`BELT_HOME_MS`) then `setBrake(true)` + `resetCounter()`. `update()`: TIMED -> stop+IDLE when timer elapses; POSITION -> read `ChkRotateEnd(bool&)` -> IDLE, OR watchdog `BELT_MOVE_TIMEOUT_MS` forces stop+IDLE+warn. `isDone()` = state==IDLE. Check vendor `bool` returns. |
| T6 | Manipulator.h facade | T4, T5 | high | `WRO2026/Manipulator.h` | Rewrite; angle-bracket vendor include. Members `grabber/sorter/dumper`/`belt`. `ManipAction` `[[nodiscard]]` builder: deleted copy, move ctor transfers `_executed=true`, `.async()`, destructor fires involved actuators concurrently then `_drain` unless async. Verbs `grab/release/sortTo/dump/reset` (THIN scaffolds: grab=grabber.close, release=grabber.open, dump=dumper->dump pose, sortTo=bounds-check then sorter->bin, reset=all home + belt.home; each `TODO: real choreography at bring-up`). `begin()/update()/isDone()/setTickCallback()` declared; `@warning` on `setTickCallback` re reentrancy. Keep `bool begin()` / `void update()` signatures. `extern ManipulatorClass Manipulator;`. |
| T7 | Manipulator.cpp | T6 | high | `WRO2026/Manipulator.cpp` | Singleton + member init from `Config::GRABBER/SORTER/DUMPER` + belt table. `begin()`: `grabber/sorter/dumper.begin()` (setHWDir) + drive each to home with a bounded plain-`delay`/`millis` settle, `belt.begin()` (bounded stall-home), return AND of successes (false on any home-timeout/vendor-fail). `update()`: `belt.update()`. `isDone()`: AND of all four. `_drain()`: bounded spin calling `_tickCallback` (fallback `update()`+one-time warn) + `delay(2)` until the action's actuators `isDone()` or per-action ceiling -> stop belt + warn. `ManipAction` dtor logic. |
| T8 | Extend vendor + serial mock | — | med | `WRO2026/test/mocks/MatrixMiniR4.h`, `WRO2026/test/mocks/mocks.cpp`, `WRO2026/test/mocks/Arduino.h` | Replace the empty `MatrixMiniR4.h` shim with a `MiniR4` mock: `RC1..RC4` (`setAngle` record + `setHWDir`); `M1..M4` (`rotateFor`/`ChkRotateEnd(bool&)`/`getDegrees`/`setSpeed`/`setBrake`/`resetCounter` [zeroes backing `getDegrees`]/`setPPR_RPM`/`setReverse` + test-settable `mockRotateEnded`, counters, last-args). Define `MiniR4` global + reset helpers in `mocks.cpp`. Make `Arduino.h` `delay(ms)` advance `g_mockMillis` by `ms`; extend `MockSerial` with the numeric `print`/`println` overloads the manipulator uses. Keep additive — do not break existing LineFollower/Intersection/Tasks tests (re-pin any that assumed `delay()` froze the clock). |
| T9 | test_manipulator.cpp | T7, T8 | med | `WRO2026/test/test_manipulator.cpp` | Link real `Manipulator.cpp` + `ManipServo.h`/`ManipBelt.h` against mocks (angle-bracket vendor include resolves to the mock). Implement TC1–TC12, TC15–TC16 with `TestAssert.h`; `resetWorld()` resets mock + `g_mockMillis`; a tick-callback stub that bumps millis so blocking `_drain` terminates; drive timers via `g_mockMillis`. |
| T10 | config compile-fail tests | T2 | low | `WRO2026/test/test_manipulator_config_fail.cpp` | A TU that `#include`s `Config.h` and relies on the build passing `-DBELT_M_ID_DEFAULT=2` (and a second target/translation with `-DSORTER_RC_ID_DEFAULT=1`) to fire the collision asserts (TC13/TC14). Mirror `test_tasks_dispatch_overflow` (no mock force-includes). The override is via compile definition, NOT `#define` over a `constexpr`. |
| T11 | CMakeLists targets | T9, T10 | med | `WRO2026/test/CMakeLists.txt` | Add `add_executable(test_manipulator ... ${MOCK_SOURCES})` + `add_test`; apply the mock force-include set as the other linked-SUT targets do; add `-fsanitize=address,undefined` (compile+link) to the `test_manipulator` target (gcc/clang). Add two compile-fail targets `EXCLUDE_FROM_ALL`/`EXCLUDE_FROM_DEFAULT_BUILD` built with `-DBELT_M_ID_DEFAULT=2` and `-DSORTER_RC_ID_DEFAULT=1` respectively (NO force-includes), each with an `add_test` running `cmake --build --target ...` + `PASS_REGULAR_EXPRESSION` on the full assert message. |
| T12 | ADRs + index | — | low | `docs/decisions/007-manipulator-control-primitives-only.md`, `docs/decisions/031-config-split-per-domain.md`, `docs/decisions/032-manipulator-api-pattern.md`, `DECISIONS.md` | Set ADR-007's **status line** to "Superseded by ADR-032" and add a cross-link to ADR-032 — leave the Rule/Why body intact (no in-place rewrite, per the ADR convention). Write ADR-032 (two-class + coordinated-verb facade + servo-timer `isDone()` + belt dual-mode) carrying the new rule that the Manipulator now exposes coordinated semantic verbs; its Rule notes it supersedes ADR-007's no-action-verbs-until-built stance; its Why records that the API shape is locked now though verbs ship as thin scaffolds until the mechanism is assembled. Write ADR-031 (minimal config split: umbrella includes ConfigManipulator + ConfigChecks-last; existing asserts not relocated; supersedes the single-file convention). Update `DECISIONS.md`: add ADR-031/032 index lines, bump the status table (ADR-007 -> Superseded by ADR-032), verify both cross-links resolve. |
| T13 | Final integration + doc sync | T3, T7, T11, T12 | low | `WRO2026/WRO2026.ino`, `ARCHITECTURE.md`, `Roadmap.md` | Run Arduino-compile + full ctest; fix any breakage in the owning task's files (note conflicts back). Add the one-line `Manipulator.setTickCallback(cooperativeTick);` to `WRO2026.ino` setup (after the other registrations) — intentional future-proofing so blocking verbs called later have a live cooperative-tick drain. Update `ARCHITECTURE.md` Manipulator row (new surface) + the `cooperativeTick`/`setTickCallback` note. Add a `Roadmap.md` note that Manipulator software was pulled forward from Phase 5. |

## Notes for Implementer

- **Angle-bracket vendor include is load-bearing.** `ManipServo.h`/`ManipBelt.h`/`Manipulator.h`/`.cpp` must `#include <MatrixMiniR4.h>` (angle brackets, like LineFollower/Intersection) so the host harness shadows it with the mock. A quoted `"MatrixMiniR4.h"` would pick the real vendor header and the host build fails.
- **`begin()` runs before any `setTickCallback`** (sketch registers callbacks after all `begin()`s). So `begin()`/`belt.begin()` homing must use plain `delay`/`millis` waits bounded by `BELT_HOME_TIMEOUT_MS`, NOT `_drain` (no callback yet). Permitted setup-time blocking site (like `Drive.calibrateMotorBias`). It also relies on `MiniR4.begin()` (WRO2026.ino:187) having already auto-initialized `RC1..4`/`M1..4`.
- **The manipulator physically moves on power-up** (servos snap to home, belt drives into its end-stop for up to `BELT_HOME_MS`). Intended — call it out in ADR-032/Doxygen so an operator handling the robot at boot expects it.
- **RC vs M id spaces are independent** — a servo on `RC1` and the belt on `M1` do NOT collide (both being "1" is fine). Only servo-vs-servo (RC) and belt-vs-drive-pair (`M2`/`M3`) collisions are checked.
- **Belt POSITION is absolute against the homed zero** — `rotateFor` is relative, so compute `delta = absTarget - getDegrees()` and put the sign in the speed. `home()`+`resetCounter()` is what makes absolute targets repeatable.
- **`ChkRotateEnd` is an out-param** — `bool e=false; ChkRotateEnd(e);` then read `e`; it is not a predicate return.
- **Don't reuse Drive's queue/done-flag** — it's single-slot and wheel-exclusive. The manipulator's `_drain` waits on its OWN actuators' `isDone()`.
- **`ManipAction` fires in the destructor**, not the verb body — matches Drive's RAII so future setters (e.g. a belt-verb `.setSpeed()`) compose; `.async()` only flips whether the destructor waits.
- **Thin scaffolds:** the verb->actuator mapping is placeholder. `grab()` only closes the grabber today; real multi-actuator choreography (and whether grab+sortTo truly overlap) lands once the mechanism is assembled. Keep the fire-all-then-wait concurrency mechanism in place so adding actuators to a verb is a scaffold edit, not a redesign.
- **`_writeServo(id,angle)` dispatch:** `MiniR4.RC1..RC4` are distinct template instances, not an array — use a small `switch(id)`, not a template on the ID (keeps three identical instances from becoming three types). Confirm member names against `MatrixMiniR4.h`.
- **Mock `delay()` now advances `g_mockMillis`** — re-pin any pre-existing test that asserted a `millis()`-derived value while assuming `delay()` froze the clock (T8 must audit, not just add).
- **Rollback:** additive + isolated. Reverting = drop the two new Config headers + their two `#include`s, restore the old `Manipulator.h/.cpp` stub, drop the new test targets + the one `.ino` line. No other module depends on the new surface (zero callers today).
- **CI freshness:** no `routes.*` regeneration; keep `arduino-compile` and `host-tests` green.