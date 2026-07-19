# Operator Diagnostics Console Plan

**Issue:** [#166](https://github.com/WinnerWang971119/WRO2026_Code/issues/166) — *operator menu: add port view, vision mode, and a mode-selectable (SYNC/GYRO/RAW) move test*

**Goal:** Replace the 3-item pre-arm calibration menu with a read-only on-OLED diagnostics console so a competition-day operator can verify the robot's sensors and power are fit to run — at the venue, in minutes, with no laptop and no reflashing.

**Approach:** Extract the console out of `WRO2026.ino` into a new upper-layer `src/operator/` domain, split into an **OLED-free core** (`OperatorConsole`) and a thin **hardware adapter** (`OperatorPanel`). The core owns the item table, the scroll/confirm FSM, button edge detection, the live-view states and the SELF TEST sequencer; it consumes a pushed-in sample POD and emits a 4-row text frame plus LED/buzzer intents and an action request. The adapter samples hardware, renders the frame, and executes requested actions. The core links into the host ctest harness; the adapter does not. Every screen is per-tick and non-blocking — no view ever holds the cooperative tick.

## Scope

- **In scope:**
  - New `src/operator/` domain: `ConsoleTypes.h` (PODs), `OperatorConsole.h/.cpp` (headless core), `OperatorPanel.h/.cpp` (Arduino adapter).
  - Nine menu entries on a flat scrolling list: `SELF TEST`, `BATT`, `LINE VIEW`, `VISION VIEW`, `GYRO VIEW`, `GYRO ZERO`, `CAL LINE`, `CAL MOTOR`, `EXIT`.
  - Three live views (LINE / VISION / GYRO), per-tick, exit on BTN_DOWN, no timeout.
  - `SELF TEST`: battery → gyro → vision → line, **halting on the first failure**, verdict signalled on LEDs + buzzer, advisory only.
  - `BATT` screen: pack voltage + percentage + the per-module `begin()` OK/FAIL results captured at boot.
  - `GYRO ZERO`: dispatches `YawSource.zeroHardware()`.
  - Replacing both blocking release-spins (`WRO2026.ino:491`, `:530`) with a non-blocking `WAIT_RELEASE` state.
  - Capturing the 13 boot `begin()` booleans into a persistent `BootReport` POD (today they are `setup()` locals that die immediately).
  - `.ino` integration: line-array demand OR, EventTrace suppression, boot-report capture, removal of the old `CalMenuItem`/`renderCalMenu()`/menu FSM.
  - New `config/ConfigUi.h` + `ConfigChecks.h` static asserts.
  - Host ctest coverage for the core; `ADR-076`; `DECISIONS.md` + `ARCHITECTURE.md` updates.

- **Out of scope:**
  - **Any entry that moves the robot.** The mode-selectable SYNC/GYRO/RAW move test named in #166 is explicitly dropped — user decision, this console is read-only. `CAL MOTOR` is the sole exception and is pre-existing behaviour, unchanged.
  - **A raw I2C / mux port-scan entry.** Rejected by the user ("the vendor I2C sucks"). Its diagnostic value is partly recovered by the per-subsystem SELF TEST checks.
  - Live constant tuning, EEPROM persistence, post-run fault forensics, tick-rate/encoder screens — all considered and dropped.
  - Retiring or modifying any `inoTest/` sketch. The large tuning harnesses (`turn_autotune`, `line_pid_tune`, `motor_match`, `turn_probe`) stay sketches.
  - Changing the mission-arm path, `Mission`, `PathTracker`, or any drive behaviour.

## Decisions

- **Domain is `src/operator/`, not `src/util/`** — `src/util/` is the *bottom* layer (`WROMath`, `TickProfiler`, `EncoderSource` all sit below `Drive`). A console that reads `Drive`, `WROLineArray`, `Vision`, `YawSource` and `Localization` is a top-layer peer of `Mission`; putting it in `util/` would invert the documented top-down dependency arrow. Both approach consultants raised this independently. Fable proposed `src/ui/`, Codex proposed `src/operator/`; `operator/` chosen for consistency with the functional domain names (`drive`, `perception`, `nav`, `mission`, `manip`).
- **Core + adapter split** — user decision, after both consultants confirmed a single OLED-owning `.cpp` cannot enter the host harness. `WRO2026/test/CMakeLists.txt:363` documents that `EventTrace.cpp` is deliberately never host-linked to dodge the `MiniR4OLED.h` `WHITE`-macro collision; verified: 81 test executables, zero link `src/util/EventTrace.cpp`. A monolithic `OperatorMenu.cpp` would inherit that exclusion and buy zero CI coverage.
- **Per-tick FSM, no blocking view loops** — a no-timeout view implemented as a blocking spin is an *unbounded* operator hold. Every existing sanctioned blocking site (`WROLineArray::calibrate()`, `Vision::readMosaic()`, `Drive::calibrateMotorBias()`) is bounded; an unbounded one erodes iron rule 1's escape hatch rather than using it. Dispatched **actions** stay sanctioned-blocking; **views** are FSM states.
- **Release-spins replaced with `WAIT_RELEASE`** — user decision. The current `while (pressed) delay(20)` spins do **not** call `cooperativeTick()`, so every module freezes while a button is held. Benign pre-arm today, unacceptable with resident views.
- **SELF TEST halts on the first failure** — user decision, upheld against Fable's veto. Fable argued for a 4-row all-checks verdict "exactly the panel's capacity"; that miscounts — the 128×32 panel is 4 rows *total* and row 0 is the title, leaving 3. Codex caught the error and defended halt-on-first: first actionable fault fastest, no pagination, and no exercising later hardware after a known failure.
- **Two data paths for line, each where it is correct** — `readBoard()` (direct I2C, returns `false` on transaction failure) is **authoritative for the board-answered verdict**; it is the only way to distinguish "answered with all-zero channels" from "transaction failed", which is exactly the BOARD-ANSWERED-ONLY rule. The forced-open tick gate feeds the **derived telemetry** (`getJunctionType()`, `isOnline()`, `getError()`), which are pure cache readers when `LINE_ARRAY_TICK_CACHE_ENABLED` is on and would otherwise render a frozen cache.
- **The line gate is a demand signal, not a hack** — `lineArrayNeededThisTick()` already means "does any consumer want line data this tick". The console becomes a third demander via `|| OperatorPanel.wantsLineData()`. This keeps the reopen-drain working unchanged and means LINE VIEW exercises the *production* data path.
- **SELF TEST is advisory only** — user decision. It never gates or warns on the BTN_UP arm path. A diagnostic false-fail must never strand the operator at the start line.
- **Line values are never judged** — user decision, grounded in the vendor gotcha that reads return no error code and a dead bus returns 0, indistinguishable from a legitimately dark channel.
- **Vision check judges link health only** — `READ_TIMEOUT` / `BAD_CHECKSUM` = FAIL; `BAD_CELL` = **PASS**. `BAD_CELL` fires when a cell classifies `UNKNOWN`, which is the *expected* pre-run state with the camera not aimed at a valid mosaic. Judging cell content would transplant the exact mistake already vetoed for line channels.
- **Menu entry, gating and title** — entry stays BTN_DOWN long-press ≥ 2 s, pre-arm only. Title renames `CALIBRATION` → `DIAGNOSTICS`.
- **`CAL MOTOR` is retained but cautioned** — its output is untrustworthy while the M2/M3 encoder cables are crossed (it compares wheels differentially, so it assigns each wheel's bias to the other). It stays in the list with an on-screen caution row; it is currently inert (bias 1.0/1.0).
- **Nothing is persisted** — no EEPROM. Calibration output still goes to Serial for paste-back into `Config.h`.

*Assumptions carried unconfirmed (the Phase 3 batch went unanswered — correct any at approval):* title rename to `DIAGNOSTICS`; `CAL MOTOR` retained; no persistence; SELF TEST order batt→gyro→vision→line; GYRO VIEW includes a drift watch; VISION VIEW re-reads on a cadence; BATT carries the boot module list.

## Acceptance Criteria

- [ ] **AC1** — A BTN_DOWN long-press ≥ 2 s while un-armed opens a `DIAGNOSTICS` list showing the title plus 3 item rows, cursor on `SELF TEST`.
- [ ] **AC2** — BTN_DOWN cycles the cursor through all 9 entries in order and wraps; the visible 3-row window scrolls to keep the cursor on screen at both ends.
- [ ] **AC3** — BTN_UP confirms the highlighted entry and dispatches it.
- [ ] **AC4** — No code path in the console calls `delay()` or spins while holding the cooperative tick. `cooperativeTick()` continues to run at its normal cadence with the menu open and with any view open, including while a button is held.
- [ ] **AC5** — While a live view is open, `lineArrayNeededThisTick()` returns `true`, and LINE VIEW's derived fields (`junction type`, `isOnline`, centroid error) change in response to physically moving the robot over a line.
- [ ] **AC6** — LINE VIEW additionally shows 8 raw channel values per configured board, sourced from `readBoard()`, and indicates per-board whether the read succeeded.
- [ ] **AC7** — VISION VIEW shows 12 cells as colour letters (`? Y B G W R`) refreshing on a cadence, plus the current `Vision::Fault`. After leaving the view, `Vision::hasFault()` is `false` (the view clears any fault it latched).
- [ ] **AC8** — GYRO VIEW shows current yaw, `isHealthy()`, and wrap-correct drift since the view was opened; leaving and re-entering resets the drift baseline to zero.
- [ ] **AC9** — GYRO ZERO dispatches `YawSource.zeroHardware()` and returns to the list.
- [ ] **AC10** — BATT shows pack voltage, percentage, and a per-module boot OK/FAIL summary reflecting the `begin()` results captured in `setup()`.
- [ ] **AC11** — SELF TEST runs battery → gyro → vision → line in that order and **stops at the first FAIL**, naming the failed subsystem on screen.
- [ ] **AC12** — SELF TEST all-pass drives green LEDs + the boot-OK chirp pattern; any failure drives red LEDs + the boot-FAIL long tone.
- [ ] **AC13** — The SELF TEST line check passes iff every configured board's `readBoard()` returns `true`. It never inspects channel values.
- [ ] **AC14** — The SELF TEST vision check FAILs on `READ_TIMEOUT` or `BAD_CHECKSUM` and PASSes on `BAD_CELL` or `NONE`.
- [ ] **AC15** — A SELF TEST failure never blocks, delays, or warns on the BTN_UP arm path; arming behaves identically before and after a failed self test.
- [ ] **AC16** — BTN_DOWN exits any live view back to the list. No view has a timeout.
- [ ] **AC17** — `EventTrace.update()` is suppressed for the whole time the console is open (list or view) and repaints its accumulated events on the first tick after it closes.
- [ ] **AC18** — The console is unavailable once the mission is armed; BTN_DOWN retains its fault-acknowledge behaviour.
- [ ] **AC19** — `OperatorConsole` compiles and links in the host ctest harness with no `<MatrixMiniR4.h>` / `<Arduino.h>` dependency, and the new tests pass in CI.
- [ ] **AC20** — `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` succeeds.

## Contracts & Interfaces

Single source of truth for every seam between tasks. A task that touches a seam conforms to the exact name, signature and shape here.

### Shared types — `WRO2026/src/operator/ConsoleTypes.h` (owner: T2)

```cpp
namespace Console
{
    /** @brief Console entries, contiguous so the scroll window can index by integer. */
    enum class Item : uint8_t
    {
        SELF_TEST = 0, BATT = 1, LINE_VIEW = 2, VISION_VIEW = 3,
        GYRO_VIEW = 4, GYRO_ZERO = 5, CAL_LINE = 6, CAL_MOTOR = 7, EXIT = 8
    };
    constexpr uint8_t ITEM_COUNT = 9;

    /** @brief Top-level console state. */
    enum class State : uint8_t
    {
        CLOSED = 0, LIST = 1, VIEW_LINE = 2, VIEW_VISION = 3,
        VIEW_GYRO = 4, VIEW_BATT = 5, SELF_TEST = 6, WAIT_RELEASE = 7
    };

    /** @brief Work the core asks the adapter to perform. One request per tick, at most. */
    enum class ActionRequest : uint8_t
    {
        NONE = 0, CAL_LINE = 1, CAL_MOTOR = 2, GYRO_ZERO = 3,
        READ_MOSAIC = 4, READ_LINE_BOARDS = 5, CLEAR_VISION_FAULT = 6
    };

    /** @brief Verdict signal the adapter drives on LEDs + buzzer. */
    enum class Signal : uint8_t { NONE = 0, PASS = 1, FAIL = 2 };

    /** @brief Which SELF TEST check is running / failed. */
    enum class Check : uint8_t { BATT = 0, GYRO = 1, VISION = 2, LINE = 3, DONE = 4 };

    /** @brief Per-module begin() results captured once in setup(). */
    struct BootReport
    {
        bool  batt, lineArray, yaw, drive, lineFollower, intersection,
              manipulator, localization, pathTracker, mission, tasks, vision, trace;
        float battVoltageAtBoot;
    };

    /** @brief Everything the core may know about the hardware this tick. Pushed in by the adapter. */
    struct DiagSample
    {
        float    battVolts;          // 0.0f => read failed
        uint8_t  battPercent;
        double   gyroYawDeg;
        bool     gyroHealthy;
        bool     gyroReadOk;
        uint8_t  lineRaw[2][8];      // per board, 0..9
        bool     lineBoardOk[2];     // readBoard() return value; index >= LINE_BOARD_COUNT is false
        uint8_t  lineBoardCount;
        uint8_t  junctionType;
        bool     lineOnline;
        float    lineError;
        uint8_t  visionCell[12];     // static_cast<uint8_t>(Config::Color)
        uint8_t  visionFaultKind;    // static_cast<uint8_t>(Vision::Fault)
        bool     visionReadOk;
    };

    /** @brief One tick of input to the core. */
    struct ConsoleInput
    {
        bool       btnUp;
        bool       btnDown;
        uint32_t   nowMs;
        bool       missionArmed;
        DiagSample sample;
        BootReport boot;
    };

    constexpr uint8_t FRAME_ROWS = 4;
    constexpr uint8_t FRAME_COLS = 22;   // 21 chars + NUL

    /** @brief What the adapter must draw and signal this tick. */
    struct ConsoleFrame
    {
        char          rows[FRAME_ROWS][FRAME_COLS];
        uint8_t       cursorRow;       // 0xFF = no cursor
        bool          dirty;           // false => adapter may skip the OLED write
        ActionRequest action;
        Signal        signal;
    };
}
```

### Signatures

- `void OperatorConsoleClass::begin()` — owner: T3; consumers: T5.
- `void OperatorConsoleClass::update(const Console::ConsoleInput& in, Console::ConsoleFrame& out)` — owner: T3; consumers: T5, T7. Pure: no I/O, no `millis()`, no statics outside the instance. `in.nowMs` is the only time source.
- `void OperatorConsoleClass::notifyActionDone(Console::ActionRequest which, bool ok)` — owner: T3; consumers: T5. Adapter reports completion of the previous tick's request.
- `bool OperatorConsoleClass::isOpen() const` — owner: T3; consumers: T5, T6.
- `bool OperatorConsoleClass::wantsLineData() const` — owner: T3; consumers: T5, T6. `true` while `LIST` is on a line-consuming entry, `VIEW_LINE` is active, or SELF TEST is at `Check::LINE`.
- `Console::State OperatorConsoleClass::state() const` — owner: T3; consumers: T7.
- `bool OperatorPanelClass::begin()` — owner: T5; consumers: T6.
- `void OperatorPanelClass::update()` — owner: T5; consumers: T6. Samples hardware, calls the core, renders, executes the action request.
- `void OperatorPanelClass::open()` — owner: T5; consumers: T6. Called by the `.ino` long-press trigger.
- `bool OperatorPanelClass::isOpen() const` — owner: T5; consumers: T6. Delegates to the core.
- `bool OperatorPanelClass::wantsLineData() const` — owner: T5; consumers: T6. Delegates to the core.
- `void OperatorPanelClass::setBootReport(const Console::BootReport& r)` — owner: T5; consumers: T6. Called once from `setup()`.
- `extern OperatorConsoleClass OperatorConsole;` / `extern OperatorPanelClass OperatorPanel;` — iron rule 2.

### File ownership

| File | Owner task | Consumer tasks |
|---|---|---|
| `WRO2026/config/ConfigUi.h` | T1 | T2, T3, T5 |
| `WRO2026/config/ConfigChecks.h` | T1 | — |
| `WRO2026/src/operator/ConsoleTypes.h` | T2 | T3, T5, T7 |
| `WRO2026/src/operator/OperatorConsole.h` | T3 | T4, T5, T7 |
| `WRO2026/src/operator/OperatorConsole.cpp` | T3 | T4 |
| `WRO2026/src/operator/OperatorPanel.h` | T5 | T6 |
| `WRO2026/src/operator/OperatorPanel.cpp` | T5 | T6 |
| `WRO2026/WRO2026.ino` | T6 | — |
| `WRO2026/test/test_operator_console.cpp` | T7 | — |
| `WRO2026/test/CMakeLists.txt` | T8 | — |
| `docs/decisions/076-*.md`, `DECISIONS.md`, `ARCHITECTURE.md` | T9 | — |

**T3 and T4 both edit `OperatorConsole.cpp`** — T4 is therefore blocked by T3, not parallel with it.
**`WRO2026/test/CMakeLists.txt` is owned solely by T8.** No other task appends to it (a new Drive-linked `.cpp` fans out to ~27 targets; serialise the edit).

### Naming (exact strings that must match across tasks)

- OLED title: `DIAGNOSTICS`
- Item labels, in order: `SELF TEST`, `BATT`, `LINE VIEW`, `VISION VIEW`, `GYRO VIEW`, `GYRO ZERO`, `CAL LINE`, `CAL MOTOR`, `EXIT`
- Cursor prefix `"> "`, non-cursor prefix `"  "` (matching today's `renderCalMenu()`)
- Vision colour letters, indexed by `Config::Color`: `?` `Y` `B` `G` `W` `R`
- `EventTrace.pushEvent` strings: `"DIAG MENU"`, `"SELF TEST"`, `"ST PASS"`, `"ST FAIL"`, `"GYRO ZERO"`, `"CAL LINE"`, `"CAL MOTOR"`, `"DIAG EXIT"` (≤ 19 chars, `EventTrace::MAX_LINE_LEN` is 20 incl. NUL)

## Data Model

New constants — `WRO2026/config/ConfigUi.h` (owner: T1), included from `Config.h` **before** `ConfigChecks.h`:

```cpp
namespace Config
{
    constexpr uint8_t  UI_FRAME_ROWS          = 4;     // 128x32 @ textSize 1
    constexpr uint8_t  UI_FRAME_COLS          = 21;    // printable chars per row
    constexpr uint8_t  UI_LIST_VISIBLE_ROWS   = 3;     // rows left after the title
    constexpr uint32_t UI_REPAINT_INTERVAL_MS = 200;   // OLED shares Wire1 with line + gyro
    constexpr uint32_t UI_VIEW_SAMPLE_MS      = 250;   // live-view hardware sample cadence
    constexpr uint32_t UI_MENU_LONGPRESS_MS   = 2000;  // unchanged from today
    constexpr float    UI_GYRO_DRIFT_WARN_DEG = 2.0f;  // drift flagged in GYRO VIEW
    constexpr uint32_t UI_SELFTEST_STEP_MS    = 300;   // dwell per check so the row is readable
}
```

`ConfigChecks.h` gains static asserts: `UI_LIST_VISIBLE_ROWS == UI_FRAME_ROWS - 1`, `UI_FRAME_COLS < Console::FRAME_COLS`, `UI_REPAINT_INTERVAL_MS >= 100`, `Console::ITEM_COUNT >= UI_LIST_VISIBLE_ROWS`.

## Error Handling

- **Battery read fails (`getBattVoltage()` returns 0):** retry once after `delay(50)` — the existing `setup()` idiom. Still 0 → SELF TEST batt check **FAILs** (never assume a pack is good when it cannot be confirmed); BATT screen shows `--.- V`.
- **Gyro read fails (`readYawDeg()` returns false) or `isHealthy()` false:** SELF TEST gyro check FAILs and halts. GYRO VIEW shows `YAW --` and holds the last drift baseline rather than re-baselining on a bad sample.
- **`readBoard()` returns false for any configured board:** SELF TEST line check FAILs. LINE VIEW shows `BOARDn: NO DATA` for that board and leaves its raw row blank. Never substitute zeros — a zero row is indistinguishable from a real dark reading.
- **`Vision::readMosaic()` returns false / latches a fault:** `READ_TIMEOUT` or `BAD_CHECKSUM` → SELF TEST vision FAIL. `BAD_CELL` → PASS (expected pre-run). **In every case the console issues `CLEAR_VISION_FAULT` before returning to the list**, so an advisory diagnostic can never leave a latched fault for the mission to inherit. Note `readMosaic()` is a latching *command*: it overwrites the capture-once latch, which is acceptable pre-arm because the mission re-captures.
- **Both buttons held on entry or on confirm:** the FSM enters `WAIT_RELEASE` and takes no action until both read released for one tick. Replaces the blocking spins; the tick keeps running throughout.
- **A dispatched blocking action (`CAL LINE` / `CAL MOTOR`) is in flight:** the core sits in `WAIT_RELEASE` and ignores button edges until `notifyActionDone()` arrives. Those routines spin the registered tick callback themselves, as today.
- **Mission armed while the console is somehow open:** `update()` force-closes to `CLOSED` on `in.missionArmed == true`. The console is pre-arm only; this is a belt-and-braces guard, not a reachable path.
- **Frame row overflow:** every row is written with a bounded formatter that truncates at `UI_FRAME_COLS` and always NUL-terminates. No `sprintf` without a width bound.

## Testing Strategy

**Levels:** Host C++ unit (ctest, CI-enforced) for the core; `arduino-cli compile` for the adapter and sketch; manual on-hardware for anything I/O-bound.

| ID | Test Case | Type | Expected Behavior |
|---|---|---|---|
| TC1 | Item table order and labels | Unit | The 9 labels appear in the exact documented order; `ITEM_COUNT == 9` |
| TC2 | BTN_DOWN cycles cursor and wraps | Unit | Cursor advances 0→8 then wraps to 0 |
| TC3 | Scroll window at both ends | Unit | Cursor 0 → rows show items 0–2; cursor 8 → rows show items 6–8; cursor always visible |
| TC4 | Rendered rows are exact | Unit | For cursor 0, `rows[0]=="DIAGNOSTICS"`, `rows[1]=="> SELF TEST"`, `rows[2]=="  BATT"`, `rows[3]=="  LINE VIEW"` |
| TC5 | Button edge detection | Unit | A level held across 3 ticks produces exactly one cursor advance |
| TC6 | `WAIT_RELEASE` blocks re-trigger | Unit | Confirm with both buttons held → no second action until both release for one tick |
| TC7 | View enter/exit | Unit | BTN_UP on `LINE VIEW` → `State::VIEW_LINE`; BTN_DOWN → `State::LIST`, cursor preserved |
| TC8 | No view timeout | Unit | 10 000 ticks with no button input leaves the state in `VIEW_LINE` |
| TC9 | `wantsLineData()` truth table | Unit | `true` in `VIEW_LINE` and at `Check::LINE`; `false` in `CLOSED`, `VIEW_GYRO` |
| TC10 | SELF TEST order | Unit | With all samples healthy, checks run BATT→GYRO→VISION→LINE then `Check::DONE` |
| TC11 | Halt on first failure | Unit | Gyro unhealthy → stops at `Check::GYRO`; vision and line checks never run |
| TC12 | Verdict signal | Unit | All-pass → `Signal::PASS`; any fail → `Signal::FAIL` |
| TC13 | Line check ignores values | Unit | `lineBoardOk={true,true}` with all-zero `lineRaw` → PASS; `lineBoardOk={true,false}` with healthy values → FAIL |
| TC14 | Vision `BAD_CELL` passes | Unit | `visionFaultKind == BAD_CELL` → PASS; `READ_TIMEOUT` → FAIL; `BAD_CHECKSUM` → FAIL |
| TC15 | Battery unreadable fails | Unit | `battVolts == 0.0f` → batt check FAIL, halts immediately |
| TC16 | Vision fault always cleared | Unit | Leaving `VIEW_VISION` or a vision-failing SELF TEST emits `ActionRequest::CLEAR_VISION_FAULT` |
| TC17 | Gyro drift is wrap-correct | Unit | Baseline 179°, sample −179° → drift +2°, not −358° |
| TC18 | Drift baseline resets on re-entry | Unit | Exit and re-enter `VIEW_GYRO` → drift 0 at the entry sample |
| TC19 | Armed force-closes the console | Unit | `missionArmed == true` → `State::CLOSED` regardless of prior state |
| TC20 | Row truncation | Unit | A 40-char label yields a NUL-terminated row of ≤ `UI_FRAME_COLS` chars |
| TC21 | Sketch compiles | Integration | `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` exits 0 |
| TC22 | Tick never stalls (hardware) | Manual | With a view open and a button held, the `[gate] la=n/m` heartbeat keeps advancing |
| TC23 | LINE VIEW is live (hardware) | Manual | Moving the chassis over a line changes junction type and centroid on screen |
| TC24 | Arm path unaffected (hardware) | Manual | After a failed SELF TEST, one BTN_UP press arms normally |

**AC → test map:** AC1→TC1/TC4; AC2→TC2/TC3; AC3→TC7; AC4→TC6/TC8/TC22; AC5→TC9/TC23; AC6→TC13; AC7→TC14/TC16; AC8→TC17/TC18; AC9→TC7; AC10→TC15; AC11→TC10/TC11; AC12→TC12; AC13→TC13; AC14→TC14; AC15→TC19/TC24; AC16→TC7/TC8; AC17→manual (TC22 window); AC18→TC19; AC19→TC1–TC20; AC20→TC21.

**Test data:** `ConsoleInput` fixtures built in-test as plain structs — no mocks needed, the core is pure. A `tick(core, in)` helper advances one frame and returns the `ConsoleFrame`.
**Run command:** `cmake -S WRO2026/test -B build/test && cmake --build build/test && ctest --test-dir build/test --output-on-failure`

## Tasks

| ID | Task | Blocked By | Risk | Files | Description |
|---|---|---|---|---|---|
| T1 | Config constants | — | low | `WRO2026/config/ConfigUi.h`, `WRO2026/config/ConfigChecks.h`, `WRO2026/Config.h` | Create `ConfigUi.h` with the constants in the Data Model section, Doxygen on each. Include it from `Config.h` **before** `ConfigChecks.h`. Add the four static asserts to `ConfigChecks.h`. Keep any `// comment` off the `#define` line (the Python config parser's regex breaks on it). Satisfies AC1 geometry. |
| T2 | Console PODs | T1 | low | `WRO2026/src/operator/ConsoleTypes.h` | Create the header exactly as pinned in Contracts → Shared types. Guard `CONSOLETYPES_H`. `<stdint.h>` only — **no `<Arduino.h>`, no `<MatrixMiniR4.h>`**; this header must compile standalone on the host. Doxygen every enum and struct. |
| T3 | Console core: FSM, list, views | T2 | high | `WRO2026/src/operator/OperatorConsole.h`, `WRO2026/src/operator/OperatorConsole.cpp` | Implement `OperatorConsoleClass` + `extern OperatorConsole` per the pinned signatures. Owns: item table, scroll-window math, button edge detection, `WAIT_RELEASE`, `CLOSED/LIST/VIEW_*` transitions, per-view row formatting, wrap-correct gyro drift against a view-entry baseline, `wantsLineData()`. Emits `ConsoleFrame` including `action`. **Pure** — no `<Arduino.h>`, no `millis()`, no I/O; time comes only from `in.nowMs`. Bounded row formatter, always NUL-terminated. Satisfies AC1–AC9, AC16, AC19. |
| T4 | SELF TEST sequencer | T3 | high | `WRO2026/src/operator/OperatorConsole.cpp` | Add the `SELF_TEST` state to the core: checks in order BATT→GYRO→VISION→LINE, halting on the first FAIL, dwelling `UI_SELFTEST_STEP_MS` per check. Battery FAIL when `battVolts < Config::BATT_MIN_VOLTAGE_V` or `== 0.0f`. Gyro FAIL when `!gyroHealthy || !gyroReadOk`. Vision FAIL **only** on `READ_TIMEOUT`/`BAD_CHECKSUM`; PASS on `BAD_CELL`/`NONE`; always emit `CLEAR_VISION_FAULT` on exit. Line PASS iff every board `< lineBoardCount` has `lineBoardOk == true` — **never inspect `lineRaw`**. Set `Signal::PASS`/`FAIL`. Same file as T3 — do not run in parallel with it. Satisfies AC11–AC14. |
| T5 | Hardware adapter | T2 | med | `WRO2026/src/operator/OperatorPanel.h`, `WRO2026/src/operator/OperatorPanel.cpp` | Implement `OperatorPanelClass` + `extern OperatorPanel`. Each `update()`: poll `MiniR4.BTN_UP/BTN_DOWN`, sample hardware into `DiagSample` on the `UI_VIEW_SAMPLE_MS` cadence (battery via `MiniR4.PWR`, gyro via `YawSource`, line derived via `LineArray` getters + raw via `readBoard()`, vision via `Vision`), call `OperatorConsole.update()`, render `ConsoleFrame` to `MiniR4.OLED` when `dirty` and `UI_REPAINT_INTERVAL_MS` has elapsed, drive `Signal` on `MiniR4.LED` + `MiniR4.Buzzer` reusing the boot-OK/boot-FAIL patterns, execute `ActionRequest` then call `notifyActionDone()`. Push `EventTrace.pushEvent` strings per Contracts → Naming. This file is **not** host-linked. Satisfies AC6, AC10, AC12. |
| T6 | Sketch integration | T3, T5 | high | `WRO2026/WRO2026.ino` | Delete `CalMenuState`, `CalMenuItem`, `CAL_MENU_ITEM_COUNT`, `renderCalMenu()`, `_calMenuState`, `_calMenuSel` and the entire menu block in `loop()` (lines ~452–545), **including both blocking release-spins**. Add `#include "src/operator/OperatorPanel.h"`. In `setup()`: capture the 13 `begin()` booleans plus `battVoltage` into a `Console::BootReport` and call `OperatorPanel.setBootReport(...)`; add `OperatorPanel.begin()`. In `cooperativeTick()`: change the gate to `LineFollower.isEnabled() || Drive.isSeekingLine() || OperatorPanel.wantsLineData()`, and the suppression to `if (!OperatorPanel.isOpen()) EventTrace.update();`. In `loop()`: keep the ≥ 2 s BTN_DOWN long-press trigger (calling `OperatorPanel.open()`), call `OperatorPanel.update()`, and early-return from the arm path while `OperatorPanel.isOpen()`. Do **not** add the panel to `cooperativeTick()` — it must not re-enter during a blocking calibration dispatch. Satisfies AC4, AC5, AC15, AC17, AC18. |
| T7 | Host tests for the core | T3, T4 | low | `WRO2026/test/test_operator_console.cpp` | Implement TC1–TC20 against the real `OperatorConsole.cpp` + `ConsoleTypes.h`, driving `ConsoleInput` structs directly. No mocks required — assert this by including neither `hal_mocks/` nor `path_mocks/`. Include a compile-level assertion that the core needs no Arduino surface. |
| T8 | CMake target | T7 | low | `WRO2026/test/CMakeLists.txt` | Add one `add_executable(test_operator_console ...)` linking `${SKETCH_DIR}/src/operator/OperatorConsole.cpp` + the test file, and its `add_test`. **No force-includes and no `MOCK_SOURCES`** — the core is Arduino-free, so it needs neither. Sole owner of this file; serialise behind every other task. |
| T9 | ADR + docs | T6 | low | `docs/decisions/076-operator-console-core-adapter.md`, `DECISIONS.md`, `ARCHITECTURE.md` | Write ADR-076 (~30–60 lines, the standard shape) recording: the `src/operator/` upper-layer domain and its downward-only dependency; the core/adapter split and why (`CMakeLists.txt:363` OLED exclusion); views-are-FSM-states vs actions-are-sanctioned-blocking. Add one line to `DECISIONS.md` in the right section and bump the status-summary table. Update `ARCHITECTURE.md`: file tree, dependency graph, and the module-responsibilities table. |
| T10 | Compile + full-suite verification | T6, T8, T9 | low | — | Run `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi WRO2026/` (compile only — **never** flash; check `arduino-cli board list` first and force `--build-only` if any tooling is involved). Run the full `ctest` suite, not just the new target. Report both outputs verbatim. Satisfies AC20. |

Parallel: T3 and T5 both unblock on T2 and touch disjoint files. T4 must wait for T3 (same `.cpp`). T7 waits for T3+T4; T8 waits for T7.

## Notes for Implementer

- **The `WHITE` macro trap.** `MiniR4OLED.h` `#define`s bare `WHITE`/`BLACK`/`INVERSE`. `Config::Color` already guards its enum with `#pragma push_macro("WHITE") / #undef / #pragma pop_macro`. `OperatorPanel.cpp` includes `<MatrixMiniR4.h>` and calls `setTextColor(WHITE)` — fine. `OperatorConsole.cpp` must never include `<MatrixMiniR4.h>`; if it needs a colour value it has the wrong responsibility. Host ctest will **not** catch a violation here — only a real `arduino-cli` compile will.
- **`readBoard()` is a live I2C read, not a cache read.** Do not call it every tick; gate it to `UI_VIEW_SAMPLE_MS`. Each board read costs several ms on Wire1 and the OLED shares that bus.
- **The cached line getters do not self-refresh** when `LINE_ARRAY_TICK_CACHE_ENABLED` is on — that is exactly why the gate must be forced open for the derived fields.
- **`Vision::readMosaic()` is blocking and latching.** It is a sanctioned blocking site; it must be dispatched as an `ActionRequest` from the adapter (which can spin the tick callback), never called from inside the core.
- **Do not re-introduce a blocking release-spin.** If you find yourself writing `while (MiniR4.BTN_*.getState())`, the answer is a `WAIT_RELEASE` tick, not a spin.
- **`OperatorPanel.update()` goes in `loop()`, not `cooperativeTick()`.** `cooperativeTick()` is re-entered from inside blocking calibration dispatches; putting the panel there would re-enter the console mid-dispatch.
- **`EncoderSource`, `Drive` and `Localization` are not sampled** by any current screen. Do not add them "for symmetry" — no speculative abstractions.
- **Config-constant blast radius:** retuning a load-bearing constant reddens up to four surfaces (host goldens, config-parse pins, behavioural tests, the Python sim mirror). The `UI_*` constants are new and consumed only by this feature, so they should be inert — verify with a full `pytest` run if `build_routes.py` output shifts at all.
- **Rollback:** the feature is additive apart from T6. Reverting T6 restores the old menu verbatim; the `src/operator/` files and the test target are inert without it.
- **Known correction to carry forward:** an earlier review claimed `WROLineArray` has no raw per-channel accessor. It does — `readBoard(uint8_t, uint8_t[8])` at `WROLineArray.h:380`, documented "Reserved for diagnostics". No new accessor is needed.
